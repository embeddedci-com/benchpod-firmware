// EMC test build (CONFIG_BRIDGE_EMC_TEST, `./build.sh emc`): the bridge's worst case for the EMC
// lab, from boot, without an access point (a shielded chamber has none, and a station that has
// not joined does not transmit). Development only: refused with the production overlay and with
// RELEASE=1. It takes signed updates like a dev image (the pod's release keys and the dev key),
// and after an update confirms itself once Ethernet has started and the first frame went out,
// so it never rolls back on its own. Its version is "<version>-emc", which the pod does not
// auto-update; a forced `bridge update` on the pod puts a newer EMC build or the normal firmware on.
//
//   Ethernet as usual: EMAC + LAN8742A up, 50 MHz RMII clock running, control channel (HELLO).
//   The pod's other frames (its EMI build streams EtherType 0x88B6 at about 20 Mbit/s) go on the
//   air instead of being bridged: eth.c hands them to emc_test_eth_rx().
//   Radio: raw 802.11 data frames (esp_wifi_80211_tx), back to back, broadcast, on one channel at
//   one rate. Each frame carries the newest Ethernet frame (LLC/SNAP, its EtherType and payload,
//   cut to fit), or a generated 0x88B6 pattern of the largest size when none is waiting, so the
//   transmitter never idles.
//
//   Ethernet stream (CONFIG_BRIDGE_EMC_ETH_MBIT, `emc eth <mbit>|off`): full 1514-byte frames
//   of EtherType 0x88B6 out of the Ethernet port, like the pod's EMI build, to the same reserved
//   link-local group address (01:80:C2:00:00:0F, which switches do not forward), so the bridge's
//   own Ethernet transmitter and RMII run loaded without a pod. Off by default; the radio can be
//   off at boot (CONFIG_BRIDGE_EMC_RADIO=n) to measure the Ethernet side alone.
//
// The radio runs as in production: station mode, the same country code (CONFIG_BRIDGE_COUNTRY,
// so only its channels), the PHY init data's power limits. The power cap can only go down from
// what the radio has after start, never above.
//
// Espressif's certification test mode (esp_phy_cert_test.h: esp_phy_wifi_tx and friends, which
// ESP-IDF v5.5 supports on the ESP32) was not used: it replaces the Wi-Fi driver with a PHY test
// mode, so it cannot send the pod's frames, and it sets the channel and power outside the
// driver's country checks.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bridge.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#if CONFIG_SECURE_BOOT || CONFIG_SECURE_FLASH_ENC_ENABLED
#error "the EMC test build is development only: not with secure boot or flash encryption"
#endif

static const char *TAG = "emc";

#define EMC_REPORT_MS 5000
// Recovery, as the pod's EMI build has for its own functions: a radio that sent nothing for
// EMC_STUCK_MS (it stopped, or every send fails) is restarted; after EMC_RESTARTS_MAX restarts in
// a row without a frame going out, the bridge reboots (the pod sees the reset reason, "software").
#define EMC_STUCK_MS      10000
#define EMC_RESTARTS_MAX  3
#define FRAME_MAX     1500  // esp_wifi_80211_tx limit, 802.11 header included
#define HDR_LEN       24
#define SNAP_LEN      8
#define PATTERN_TYPE  0x88B6
#define ETH_FRAME     1514  // Ethernet frame without FCS
#define ETH_WIRE_BITS ((ETH_FRAME + 4 + 8 + 12) * 8)  // + FCS, preamble/SFD, inter-frame gap
#define ETH_MBIT_MAX  90

typedef enum { K_DSSS_LONG, K_DSSS_SHORT, K_OFDM, K_HT } kind_t;

typedef struct {
    const char *name;
    wifi_phy_rate_t rate;
    uint32_t kbps;
    kind_t kind;
} rate_t;

static const rate_t k_rates[] = {
    {"1m", WIFI_PHY_RATE_1M_L, 1000, K_DSSS_LONG},      {"2m", WIFI_PHY_RATE_2M_L, 2000, K_DSSS_LONG},
    {"5.5m", WIFI_PHY_RATE_5M_L, 5500, K_DSSS_LONG},    {"11m", WIFI_PHY_RATE_11M_L, 11000, K_DSSS_LONG},
    {"2ms", WIFI_PHY_RATE_2M_S, 2000, K_DSSS_SHORT},    {"5.5ms", WIFI_PHY_RATE_5M_S, 5500, K_DSSS_SHORT},
    {"11ms", WIFI_PHY_RATE_11M_S, 11000, K_DSSS_SHORT}, {"6m", WIFI_PHY_RATE_6M, 6000, K_OFDM},
    {"9m", WIFI_PHY_RATE_9M, 9000, K_OFDM},             {"12m", WIFI_PHY_RATE_12M, 12000, K_OFDM},
    {"18m", WIFI_PHY_RATE_18M, 18000, K_OFDM},          {"24m", WIFI_PHY_RATE_24M, 24000, K_OFDM},
    {"36m", WIFI_PHY_RATE_36M, 36000, K_OFDM},          {"48m", WIFI_PHY_RATE_48M, 48000, K_OFDM},
    {"54m", WIFI_PHY_RATE_54M, 54000, K_OFDM},          {"mcs0", WIFI_PHY_RATE_MCS0_LGI, 6500, K_HT},
    {"mcs1", WIFI_PHY_RATE_MCS1_LGI, 13000, K_HT},      {"mcs2", WIFI_PHY_RATE_MCS2_LGI, 19500, K_HT},
    {"mcs3", WIFI_PHY_RATE_MCS3_LGI, 26000, K_HT},      {"mcs4", WIFI_PHY_RATE_MCS4_LGI, 39000, K_HT},
    {"mcs5", WIFI_PHY_RATE_MCS5_LGI, 52000, K_HT},      {"mcs6", WIFI_PHY_RATE_MCS6_LGI, 58500, K_HT},
    {"mcs7", WIFI_PHY_RATE_MCS7_LGI, 65000, K_HT},
};
#define N_RATES (sizeof(k_rates) / sizeof(k_rates[0]))

static struct {
    SemaphoreHandle_t lock;  // the radio: the TX task's sends against reconfiguration
    portMUX_TYPE mux;        // pending
    uint8_t *pending;        // newest Ethernet frame (malloc'd by the EMAC driver), NULL: none
    uint32_t pending_len;
    volatile bool running;
    volatile uint32_t eth_mbit;                   // Ethernet stream, 0 = off
    volatile uint32_t eth_sent, eth_no_link, eth_errors;
    volatile bool eth_started;  // a new image on trial confirms itself once a frame went out too
    bool radio_up;
    uint32_t radio_restarts;     // since boot
    uint32_t restarts_in_row;    // since the last frame went out
    uint8_t channel, ch_first, ch_last;
    const rate_t *rate;
    int8_t power, power_max;  // 0.25 dBm
    uint8_t mac[6];
    // Counters (emc_test_counts): Ethernet side written by the EMAC rx task, the rest by the TX task.
    volatile uint32_t eth_rx, eth_skipped;
    volatile uint32_t sent, sent_eth, sent_pattern, busy, errors;
    volatile uint64_t airtime_us;
    volatile int64_t last_ok_us, last_eth_us;
    esp_err_t last_err;
    int64_t t_start;
} s = {.mux = portMUX_INITIALIZER_UNLOCKED,
#if CONFIG_BRIDGE_EMC_RADIO
       .running = true,
#else
       .running = false,
#endif
       .eth_mbit = CONFIG_BRIDGE_EMC_ETH_MBIT};

// Time on the air for one frame of `len` bytes (FCS added), preamble included; no gaps.
static uint32_t airtime_us(const rate_t *r, size_t len)
{
    uint32_t bits = (uint32_t)(len + 4) * 8;
    switch (r->kind) {
    case K_DSSS_LONG: return 192 + bits * 1000 / r->kbps;
    case K_DSSS_SHORT: return 96 + bits * 1000 / r->kbps;
    default: {
        uint32_t per_sym = r->kbps * 4 / 1000;  // data bits per 4 us symbol
        uint32_t syms = (16 + bits + 6 + per_sym - 1) / per_sym;
        return (r->kind == K_HT ? 36 : 20) + syms * 4;
    }
    }
}

static const rate_t *find_rate(const char *name)
{
    for (size_t i = 0; i < N_RATES; i++) {
        if (strcasecmp(k_rates[i].name, name) == 0) {
            return &k_rates[i];
        }
    }
    return NULL;
}

// Power, channel and settings that only hold while the radio runs. Caller holds s.lock.
static esp_err_t radio_settings(void)
{
    esp_err_t err = esp_wifi_set_channel(s.channel, WIFI_SECOND_CHAN_NONE);
    if (err == ESP_OK && s.power < s.power_max) {
        err = esp_wifi_set_max_tx_power(s.power);
        esp_wifi_get_max_tx_power(&s.power);  // the driver rounds to its steps
    }
    return err;
}

// (Re)starts the radio with s.rate (the 802.11 TX rate is only set while it is stopped).
// Caller holds s.lock.
static esp_err_t radio_start(void)
{
    if (s.radio_up) {
        esp_wifi_stop();
        s.radio_up = false;
    }
    esp_err_t err = esp_wifi_config_80211_tx_rate(WIFI_IF_STA, s.rate->rate);
    if (err == ESP_OK) {
        err = esp_wifi_start();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "radio start: %s", esp_err_to_name(err));
        return err;
    }
    s.radio_up = true;
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
    if (s.power_max == 0) {
        // First start: the maximum the PHY init data allows, as in normal operation. The cap
        // never goes above it.
        esp_wifi_get_max_tx_power(&s.power_max);
        s.power = CONFIG_BRIDGE_EMC_TX_POWER && CONFIG_BRIDGE_EMC_TX_POWER < s.power_max
                      ? CONFIG_BRIDGE_EMC_TX_POWER : s.power_max;
    }
    return radio_settings();
}

void emc_test_eth_started(void)
{
    s.eth_started = true;
}

void emc_test_eth_rx(uint8_t *frame, size_t len)
{
    s.eth_rx++;
    s.last_eth_us = esp_timer_get_time();
    portENTER_CRITICAL(&s.mux);
    uint8_t *old = s.pending;
    s.pending = frame;
    s.pending_len = len;
    portEXIT_CRITICAL(&s.mux);
    if (old) {
        s.eth_skipped++;  // the radio was slower than the wire: only the newest frame waits
        free(old);
    }
}

// The 802.11 header and LLC/SNAP for `type`. Data frame, no ToDS/FromDS (no BSS joined),
// address 1 broadcast (no ACK, no retries), addresses 2 and 3 our station MAC, as ESP-IDF
// recommends for esp_wifi_80211_tx without a connection. The driver sets the sequence number.
static void put_header(uint8_t *f, uint16_t type)
{
    memset(f, 0, HDR_LEN);
    f[0] = 0x08;
    memset(f + 4, 0xff, 6);
    memcpy(f + 10, s.mac, 6);
    memcpy(f + 16, s.mac, 6);
    static const uint8_t snap[6] = {0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00};
    memcpy(f + HDR_LEN, snap, 6);
    f[HDR_LEN + 6] = type >> 8;
    f[HDR_LEN + 7] = type & 0xff;
}

// Next frame to send into f: the newest Ethernet frame, else the pattern. Returns its length;
// *from_eth says which.
static size_t next_frame(uint8_t *f, bool *from_eth, uint32_t *seq)
{
    portENTER_CRITICAL(&s.mux);
    uint8_t *eth = s.pending;
    uint32_t len = s.pending_len;
    s.pending = NULL;
    portEXIT_CRITICAL(&s.mux);

    const size_t room = FRAME_MAX - HDR_LEN - SNAP_LEN;
    if (eth && len >= 14) {
        size_t n = len - 14 < room ? len - 14 : room;
        put_header(f, (uint16_t)(eth[12] << 8 | eth[13]));
        memcpy(f + HDR_LEN + SNAP_LEN, eth + 14, n);
        free(eth);
        *from_eth = true;
        return HDR_LEN + SNAP_LEN + n;
    }
    free(eth);
    // Pattern: a frame counter, then xorshift32 bytes (802.11 scrambles the bits anyway).
    put_header(f, PATTERN_TYPE);
    uint8_t *p = f + HDR_LEN + SNAP_LEN;
    uint32_t x = ++*seq;
    memcpy(p, &x, 4);
    x = x * 2654435761u | 1;
    for (size_t i = 4; i < room; i++) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        p[i] = (uint8_t)x;
    }
    *from_eth = false;
    return FRAME_MAX;
}

static void tx_task(void *arg)
{
    static uint8_t f[FRAME_MAX];
    size_t len = 0;  // 0: no frame held (a refused frame is kept and sent again)
    bool from_eth = false;
    uint32_t seq = 0;
    for (;;) {
        if (!s.running) {
            len = 0;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (!len) {
            len = next_frame(f, &from_eth, &seq);
        }
        xSemaphoreTake(s.lock, portMAX_DELAY);
        esp_err_t err = s.radio_up ? esp_wifi_80211_tx(WIFI_IF_STA, f, (int)len, true) : ESP_ERR_WIFI_NOT_STARTED;
        const rate_t *r = s.rate;
        xSemaphoreGive(s.lock);
        if (err == ESP_OK) {
            s.sent++;
            from_eth ? s.sent_eth++ : s.sent_pattern++;
            s.airtime_us += airtime_us(r, len);
            s.last_ok_us = esp_timer_get_time();
            s.restarts_in_row = 0;
            len = 0;
            if (s.eth_started) {
                // Radio and Ethernet work: keep this image (OTA trial boot) without waiting for
                // the pod, so an EMC build never rolls back on its own. No-op when not on trial.
                ota_confirm_self();
                s.eth_started = false;
            }
        } else if (err == ESP_ERR_NO_MEM) {
            s.busy++;  // the driver's queue is full: the radio is sending, try again next tick
            vTaskDelay(1);
        } else {
            s.errors++;
            s.last_err = err;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// The Ethernet stream: frames at s.eth_mbit (wire rate, preamble and gap included), paced by the
// microsecond timer in 10 ms batches (one FreeRTOS tick). Each frame starts with "BPEMI", a sequence
// number and the uptime in ms, as the pod's, so `tcpdump ether proto 0x88b6` can count them.
static void eth_stream_task(void *arg)
{
    static const uint8_t dst[6] = {0x01, 0x80, 0xC2, 0x00, 0x00, 0x0F};
    static uint8_t f[ETH_FRAME];
    uint32_t seq = 0, x = 0x9E3779B9u;
    for (size_t i = 14; i < ETH_FRAME; i++) {  // payload pattern, filled once
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        f[i] = (uint8_t)x;
    }
    memcpy(f, dst, 6);
    f[12] = PATTERN_TYPE >> 8;
    f[13] = PATTERN_TYPE & 0xff;
    int64_t t_last = esp_timer_get_time();
    double owed = 0;  // frames due and not yet sent
    for (;;) {
        vTaskDelay(1);
        const uint32_t mbit = s.eth_mbit;
        const int64_t now = esp_timer_get_time();
        if (!mbit) {
            owed = 0;
            t_last = now;
            continue;
        }
        owed += (double)(now - t_last) * mbit / ETH_WIRE_BITS;  // us x Mbit/s = bits
        t_last = now;
        if (owed > 64) {
            owed = 64;  // after a stall (no link): no burst to catch up
        }
        while (owed >= 1) {
            owed -= 1;
            eth_get_mac(f + 6);
            seq++;
            uint32_t ms = (uint32_t)(now / 1000);
            memcpy(f + 14, "BPEMI", 5);
            memcpy(f + 19, &seq, 4);
            memcpy(f + 23, &ms, 4);
            esp_err_t err = eth_send(f, ETH_FRAME);
            if (err == ESP_OK) {
                s.eth_sent++;
                if (s.eth_started && !s.running) {
                    // Radio off at boot: Ethernet alone confirms a new image (see tx_task).
                    ota_confirm_self();
                    s.eth_started = false;
                }
            } else if (err == ESP_ERR_INVALID_STATE) {
                s.eth_no_link++;
                owed = 0;
                break;
            } else {
                s.eth_errors++;
                break;
            }
        }
    }
}

const char *emc_test_set_eth(uint32_t mbit)
{
    if (mbit > ETH_MBIT_MAX) {
        static char why[48];
        snprintf(why, sizeof(why), "eth stream: 0 (off) to %u Mbit/s", ETH_MBIT_MAX);
        return why;
    }
    s.eth_mbit = mbit;
    if (mbit) {
        ESP_LOGI(TAG, "Ethernet stream %lu Mbit/s", (unsigned long)mbit);
    } else {
        ESP_LOGI(TAG, "Ethernet stream off");
    }
    return NULL;
}

// Called every EMC_REPORT_MS: restart a radio that has sent nothing for EMC_STUCK_MS, reboot after
// EMC_RESTARTS_MAX restarts that did not help.
static void recovery_check(void)
{
    if (!s.running) {
        return;
    }
    const int64_t now = esp_timer_get_time();
    const int64_t since = s.last_ok_us ? s.last_ok_us : s.t_start;
    if (now - since < (int64_t)EMC_STUCK_MS * 1000) {
        return;
    }
    if (s.restarts_in_row >= EMC_RESTARTS_MAX) {
        ESP_LOGE(TAG, "recovery: nothing sent after %u radio restarts, rebooting", (unsigned)s.restarts_in_row);
        vTaskDelay(pdMS_TO_TICKS(100));  // the console line out
        esp_restart();
    }
    s.restarts_in_row++;
    s.radio_restarts++;
    ESP_LOGW(TAG, "recovery: nothing sent for %lld s, restarting the radio (%u of %u)",
             (long long)((now - since) / 1000000), (unsigned)s.restarts_in_row, (unsigned)EMC_RESTARTS_MAX);
    xSemaphoreTake(s.lock, portMAX_DELAY);
    esp_err_t err = radio_start();
    xSemaphoreGive(s.lock);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "recovery: radio start: %s", esp_err_to_name(err));
    }
    s.last_ok_us = now;  // the next check is EMC_STUCK_MS from now
}

static void report_task(void *arg)
{
    emc_counts_t prev = {0};
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(EMC_REPORT_MS));
        recovery_check();
        emc_counts_t c;
        emc_test_counts(&c);
        uint64_t air = c.airtime_us - prev.airtime_us;
        ESP_LOGI(TAG, "%s ch %u %s %.2f dBm | %lu frames/s, air %u%% | eth rx %lu, on air %lu, pattern %lu, "
                      "busy %lu, err %lu | eth stream %lu Mbit/s: %lu frames/s, no link %lu, err %lu",
                 c.running ? "TX" : "stopped", c.channel, c.rate, c.power_qdbm / 4.0,
                 (unsigned long)((c.sent - prev.sent) * 1000 / EMC_REPORT_MS),
                 (unsigned)(air / (EMC_REPORT_MS * 10)), (unsigned long)c.eth_rx, (unsigned long)c.sent_eth,
                 (unsigned long)c.sent_pattern, (unsigned long)c.busy, (unsigned long)c.errors,
                 (unsigned long)c.eth_mbit, (unsigned long)((c.eth_sent - prev.eth_sent) * 1000 / EMC_REPORT_MS),
                 (unsigned long)c.eth_no_link, (unsigned long)c.eth_errors);
        prev = c;
    }
}

void emc_test_counts(emc_counts_t *c)
{
    memset(c, 0, sizeof(*c));
    c->running = s.running;
    c->channel = s.channel;
    c->rate = s.rate ? s.rate->name : "?";
    c->power_qdbm = s.power;
    c->power_max_qdbm = s.power_max;
    c->eth_rx = s.eth_rx;
    c->eth_skipped = s.eth_skipped;
    c->sent = s.sent;
    c->sent_eth = s.sent_eth;
    c->sent_pattern = s.sent_pattern;
    c->busy = s.busy;
    c->errors = s.errors;
    c->last_err = s.last_err;
    c->airtime_us = s.airtime_us;
    c->radio_restarts = s.radio_restarts;
    c->eth_mbit = s.eth_mbit;
    c->eth_sent = s.eth_sent;
    c->eth_no_link = s.eth_no_link;
    c->eth_errors = s.eth_errors;
    c->uptime_us = esp_timer_get_time() - s.t_start;
}

emc_state_t emc_test_state(void)
{
    int64_t now = esp_timer_get_time();
    if (!s.running) {
        return EMC_STOPPED;
    }
    if (!s.last_ok_us || now - s.last_ok_us > 1000000) {
        return EMC_FAILING;
    }
    return s.last_eth_us && now - s.last_eth_us < 1000000 ? EMC_TX_ETH : EMC_TX_PATTERN;
}

void emc_test_run(bool on)
{
    s.running = on;
    ESP_LOGI(TAG, "transmitting %s", on ? "on" : "off");
}

const char *emc_test_set_channel(int ch)
{
    if (ch < s.ch_first || ch > s.ch_last) {
        static char why[48];
        snprintf(why, sizeof(why), "channel must be %u-%u (country %s)", s.ch_first, s.ch_last, CONFIG_BRIDGE_COUNTRY);
        return why;
    }
    xSemaphoreTake(s.lock, portMAX_DELAY);
    s.channel = (uint8_t)ch;
    esp_err_t err = s.radio_up ? esp_wifi_set_channel(s.channel, WIFI_SECOND_CHAN_NONE) : ESP_OK;
    xSemaphoreGive(s.lock);
    return err == ESP_OK ? NULL : esp_err_to_name(err);
}

const char *emc_test_set_rate(const char *name)
{
    const rate_t *r = find_rate(name);
    if (!r) {
        return "unknown rate (1m 2m 5.5m 11m 2ms 5.5ms 11ms 6m 9m 12m 18m 24m 36m 48m 54m mcs0-mcs7)";
    }
    xSemaphoreTake(s.lock, portMAX_DELAY);
    s.rate = r;
    esp_err_t err = radio_start();
    xSemaphoreGive(s.lock);
    return err == ESP_OK ? NULL : esp_err_to_name(err);
}

const char *emc_test_set_power(int qdbm)
{
    static char why[64];
    if (qdbm <= 0 || qdbm >= s.power_max) {
        qdbm = s.power_max;
    } else if (qdbm < 8) {
        return "power must be 2 dBm or more";
    }
    xSemaphoreTake(s.lock, portMAX_DELAY);
    esp_err_t err = ESP_OK;
    if (s.radio_up) {
        err = esp_wifi_set_max_tx_power((int8_t)qdbm);
        esp_wifi_get_max_tx_power(&s.power);
    } else {
        s.power = (int8_t)qdbm;
    }
    xSemaphoreGive(s.lock);
    if (err != ESP_OK) {
        snprintf(why, sizeof(why), "%s", esp_err_to_name(err));
        return why;
    }
    return NULL;
}

esp_err_t emc_test_start(void)
{
    s.lock = xSemaphoreCreateMutex();
    s.t_start = esp_timer_get_time();
    s.channel = CONFIG_BRIDGE_EMC_CHANNEL;
    s.rate = find_rate(CONFIG_BRIDGE_EMC_RATE);
    if (!s.rate) {
        ESP_LOGW(TAG, "unknown CONFIG_BRIDGE_EMC_RATE \"%s\": using 1m", CONFIG_BRIDGE_EMC_RATE);
        s.rate = &k_rates[0];
    }
    esp_read_mac(s.mac, ESP_MAC_WIFI_STA);

    // As wifi_start() in production: station mode, credentials not used, the same country code.
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_country_code(CONFIG_BRIDGE_COUNTRY, false));
    wifi_country_t country;
    ESP_ERROR_CHECK(esp_wifi_get_country(&country));
    s.ch_first = country.schan;
    s.ch_last = country.schan + country.nchan - 1;
    if (s.channel < s.ch_first || s.channel > s.ch_last) {
        ESP_LOGW(TAG, "channel %u not allowed in %s: using %u", s.channel, CONFIG_BRIDGE_COUNTRY, s.ch_first);
        s.channel = s.ch_first;
    }

    xSemaphoreTake(s.lock, portMAX_DELAY);
    esp_err_t err = radio_start();
    xSemaphoreGive(s.lock);

    ESP_LOGW(TAG, "EMC TEST BUILD: radio %s from boot, no access point; channel %u, rate %s, "
                  "power cap %.2f dBm (maximum %.2f dBm), country %s; Ethernet stream %lu Mbit/s",
             s.running ? "transmitting" : "stopped", s.channel, s.rate->name, s.power / 4.0, s.power_max / 4.0,
             CONFIG_BRIDGE_COUNTRY, (unsigned long)s.eth_mbit);
    xTaskCreate(tx_task, "emc_tx", 4096, NULL, 4, NULL);
    xTaskCreate(eth_stream_task, "emc_eth", 3072, NULL, 4, NULL);
    xTaskCreate(report_task, "emc_report", 3072, NULL, 2, NULL);
    return err;
}
