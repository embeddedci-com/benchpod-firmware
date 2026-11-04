/*
 * emi_test — the EMC pre-compliance activity loop (emi_test.h). Built only with EMI_TEST=1.
 */
#include "emi_test.h"
#include "command_handler.h"
#include "command_handler_internal.h"   /* capture_busy */
#include "cal_data.h"                   /* dac_code16_for_volts */
#include "dac_quad.h"
#include "signal_engine.h"              /* dac_quad_write */
#include "status_led.h"                 /* status_led_emi_t */
#include "board_variant.h"              /* board_has_analog */
#include "stm32h5xx_hal.h"              /* HAL_GetTick */
#include "lwip/netif.h"
#include "lwip/pbuf.h"

#include <stdio.h>
#include <string.h>

/* What a bulk read-back may send to CH_EMI_CONN per worker pass (emi_test_avail). */
#define EMI_SINK_BUDGET      16384u
/* Pause between two captures, so the console and the network task get the worker too. */
#define EMI_CAPTURE_GAP_MS   200u

/* The two captures the loop takes in turn: ADC + LA (the ADC at its full rate, the LA beside it),
   then a deep LA-only capture at the LA's top rate. Both read back over the PSRAM bus. */
static const char *const k_captures[] = {
    "{\"cmd\":\"capture_dual\",\"adc_samples\":200000,\"la_samples\":200000,"
    "\"adc_rate_mhz\":0.4,\"la_rate_mhz\":2.0}",
    "{\"cmd\":\"la_capture\",\"samples\":1000000}",
};

static const char *const k_dac_names[] = { "sine", "dc", "current", "off", "all" };

/* A failure turns the LED pattern red for this long (status_led.h). */
#define EMI_LED_ERROR_MS  30000u

/* The DC level on the 0-5 V output (J11 to J8: 3 V over 249 ohm is 12 mA in the loop input). */
#define EMI_DC_VOLTS  3.0f

static struct {
    bool           booted;        /* the boot start has happened (or `emi stop` came first) */
    bool           booted_mode;   /* the default DAC mode for this board is chosen */
    bool           running;
    emi_dac_mode_t dac;
    int            uart_la;       /* NMEA out of this LA pin, 0 = off */
    uint32_t       t_start;       /* HAL_GetTick() at start */
    uint32_t       t_report;
    uint32_t       t_next_cap;
    unsigned       next_kind;     /* index into k_captures */
    bool           cap_issued;    /* a capture went out and has not finished yet */
    uint32_t       cap_errors;    /* s.errors when it went out */
    uint32_t       captures;      /* finished without an error */
    uint32_t       errors;
    uint32_t       last_err_ms;   /* HAL_GetTick() of the last failure */
    const char    *cur;           /* the command in flight, for the error log */
    char           last_err[112];
    size_t         budget;
} s = { .dac = EMI_DAC_SINE };   /* EMI_DAC_ALL on a quad-DAC board, set at the start */

/* The Ethernet stream. The rate is set by the console on the hw worker and read by the net task
   (emi_test_net_poll), which alone touches lwIP; aligned 32-bit words, so no lock. */
static struct {
    volatile uint32_t mbit;       /* 0 = off */
    volatile uint32_t sent;
    volatile uint32_t drops;      /* no pbuf, or the MAC refused the frame */
    volatile uint32_t link_losses; /* the link went down while the stream ran */
    volatile uint32_t t_err;      /* HAL_GetTick() of the last Ethernet problem */
    volatile bool     err;        /* t_err is set (since the last start) */
    volatile bool     link;       /* the link is up (as the net task last saw it) */
    bool              had_link;   /* net task: the link was up since the start */
    /* net task only */
    uint32_t          t_last;
    uint32_t          credit;     /* bytes allowed to go now */
    uint32_t          seq;
} ns = { .mbit = EMI_NET_DEFAULT_MBIT };

/* At most this many frames per net_poll pass (HAL_ETH_Transmit waits for each one), so the
   stream never holds the net task for long. */
#define EMI_NET_BURST  4u

static void run(const char *label, const char *json) {
    s.cur = label;
    command_handler_process(CH_EMI_CONN, (const uint8_t *)json, strlen(json));
}

static bool drives_current(emi_dac_mode_t m) { return m == EMI_DAC_CURRENT || m == EMI_DAC_ALL; }

/* The quad DAC holds a written level per channel, but routing an output parks the voltage
   channels it leaves: after the sine routes to channel A, write the DC back to channel B. */
static void hold_dc_on_b(void) {
    s.cur = "hold 0-5 V";
    if (dac_quad_write(DAC_QUAD_B, dac_code16_for_volts(1, EMI_DC_VOLTS)) != 0) {
        s.errors++;
        s.last_err_ms = HAL_GetTick();
        snprintf(s.last_err, sizeof(s.last_err), "hold 0-5 V: DAC write failed");
        printf("[emi] holding 0-5 V at the DC level failed\n");
    }
}

/* Stop whatever the DAC plays (s.dac), then start `mode`. */
static void apply_dac(emi_dac_mode_t mode) {
    run("dac_stop", "{\"cmd\":\"dac_stop\"}");
    if (s.dac == EMI_DAC_ALL)   /* parks the 0-5 V output it held */
        run("analog_path", "{\"cmd\":\"analog_path\",\"path\":\"off\"}");
    if (drives_current(s.dac) && !drives_current(mode))   /* dac_stop leaves the loop current */
        run("current_out", "{\"cmd\":\"current_out\",\"ua\":4000}");
    switch (mode) {
    case EMI_DAC_ALL:
        run("current_out", "{\"cmd\":\"current_out\",\"ua\":12000}");
        run("generate", "{\"cmd\":\"generate\",\"waveform\":\"sine\",\"freq\":1000,"
                        "\"path\":\"3v3\",\"amplitude_v\":1.5,\"offset_v\":1.65}");
        hold_dc_on_b();
        break;
    case EMI_DAC_SINE:
        run("generate", "{\"cmd\":\"generate\",\"waveform\":\"sine\",\"freq\":1000,"
                        "\"path\":\"3v3\",\"amplitude_v\":1.5,\"offset_v\":1.65}");
        break;
    case EMI_DAC_DC:
        run("dac_out", "{\"cmd\":\"dac_out\",\"path\":\"5v\",\"volts\":3.0}");
        break;
    case EMI_DAC_CURRENT:
        run("current_out", "{\"cmd\":\"current_out\",\"ua\":12000}");
        break;
    case EMI_DAC_OFF:
        run("analog_path", "{\"cmd\":\"analog_path\",\"path\":\"off\"}");
        break;
    }
}

static void apply_uart(int la) {
    run("gps_stop", "{\"cmd\":\"gps_stop\"}");
    if (la <= 0) return;
    char cmd[96];
    snprintf(cmd, sizeof(cmd), "{\"cmd\":\"gps_start\",\"tx\":%d,\"baud\":115200,\"rate_hz\":10}", la);
    run("gps_start", cmd);
}

void emi_test_start(void) {
    s.booted = true;
    if (s.running) return;
    s.running    = true;
    s.t_start    = HAL_GetTick();
    s.t_report   = s.t_start;
    s.t_next_cap = s.t_start;
    s.captures   = 0;
    s.errors     = 0;
    s.last_err[0] = '\0';
    ns.err         = false;   /* the net task re-reads the link from here on */
    ns.had_link    = false;
    ns.link_losses = 0;
    if (!s.booted_mode) {   /* first start: the quad DAC runs every output by default */
        s.booted_mode = true;
        if (dac_quad_present()) s.dac = EMI_DAC_ALL;
        if (!board_has_analog()) s.dac = EMI_DAC_OFF;   /* digital-only board: no DAC, no ADC */
    }
    printf("[emi] starting the EMI test activity\n");
    run("la_voltage", "{\"cmd\":\"la_voltage\",\"mv\":3300}");
    run("target_power 1", "{\"cmd\":\"target_power\",\"efuse\":1,\"state\":1}");
    run("target_power 2", "{\"cmd\":\"target_power\",\"efuse\":2,\"state\":1}");
    if (board_has_analog()) apply_dac(s.dac);
    apply_uart(s.uart_la);
}

void emi_test_stop(void) {
    s.booted = true;   /* a stop before the boot delay ran out also cancels the boot start */
    if (!s.running) return;
    s.running = false;
    if (board_has_analog()) {
        run("dac_stop", "{\"cmd\":\"dac_stop\"}");
        if (drives_current(s.dac)) run("current_out", "{\"cmd\":\"current_out\",\"ua\":4000}");
        run("analog_path", "{\"cmd\":\"analog_path\",\"path\":\"off\"}");
    }
    run("gps_stop", "{\"cmd\":\"gps_stop\"}");
    run("target_power 1", "{\"cmd\":\"target_power\",\"efuse\":1,\"state\":0}");
    run("target_power 2", "{\"cmd\":\"target_power\",\"efuse\":2,\"state\":0}");
    /* A capture still in flight finishes on its own and frees the heavy gate. */
    printf("[emi] stopped: DAC off, target power off\n");
}

bool emi_test_running(void) { return s.running; }

bool emi_test_set_dac(emi_dac_mode_t mode) {
    if (mode == EMI_DAC_ALL && !dac_quad_present()) return false;
    if (mode != EMI_DAC_OFF && !board_has_analog()) return false;
    s.booted_mode = true;   /* a choice before the boot start sticks */
    if (s.running) apply_dac(mode);
    s.dac = mode;
    return true;
}

void emi_test_set_uart(int la) {
    if (la < 0 || la > 14) return;
    s.uart_la = la;
    if (s.running) apply_uart(la);
}

bool emi_test_set_net_rate(unsigned mbit) {
    if (mbit > EMI_NET_MAX_MBIT) return false;
    ns.mbit = mbit;
    return true;
}

void emi_test_net_poll(struct netif *eth) {
    static const uint8_t k_dst[6] = EMI_NET_DST_MAC;
    const uint32_t now = HAL_GetTick();
    const uint32_t dt  = now - ns.t_last;
    ns.t_last = now;
    const uint32_t mbit = ns.mbit;
    if (!s.running || !mbit) {
        ns.credit = 0;
        return;
    }
    /* Layer 2 only: no IP address, DHCP or peer needed, just a link. */
    const bool link = netif_is_up(eth) && netif_is_link_up(eth);
    if (!link) {
        if (ns.link) ns.link_losses++;
        ns.link   = false;
        ns.credit = 0;
        /* No link counts as a problem once it had come up, or after the grace period. */
        if (ns.had_link || now - s.t_start >= EMI_NET_LINK_GRACE_MS) {
            ns.t_err = now;
            ns.err   = true;
        }
        return;
    }
    ns.link     = true;
    ns.had_link = true;
    /* mbit Mbit/s = mbit * 125 bytes per ms; cap the credit at one burst, so a stalled pass
       does not turn into a flood afterwards. */
    const uint32_t cap = EMI_NET_BURST * EMI_NET_FRAME;
    ns.credit += (dt > 100u ? 100u : dt) * mbit * 125u;
    if (ns.credit > cap) ns.credit = cap;
    while (ns.credit >= EMI_NET_FRAME) {
        ns.credit -= EMI_NET_FRAME;
        struct pbuf *p = pbuf_alloc(PBUF_RAW, EMI_NET_FRAME, PBUF_RAM);
        if (!p) { ns.drops++; ns.t_err = now; ns.err = true; break; }
        uint8_t *d = (uint8_t *)p->payload;
        const uint32_t seq = ns.seq++;
        memcpy(d, k_dst, 6);
        memcpy(d + 6, eth->hwaddr, 6);
        d[12] = (uint8_t)(EMI_NET_ETHERTYPE >> 8);
        d[13] = (uint8_t)EMI_NET_ETHERTYPE;
        memcpy(d + 14, "BPEMI", 5);
        memcpy(d + 19, &seq, 4);   /* little-endian, like the pod's other binary formats */
        memcpy(d + 23, &now, 4);
        for (uint32_t i = 27; i < EMI_NET_FRAME; i++) d[i] = (uint8_t)(seq + i);
        if (eth->linkoutput(eth, p) == ERR_OK) {
            ns.sent++;
        } else {
            ns.drops++;
            ns.t_err = now;
            ns.err   = true;
        }
        pbuf_free(p);
    }
}

int emi_test_dac_from_name(const char *name) {
    for (int i = 0; i < (int)(sizeof(k_dac_names) / sizeof(k_dac_names[0])); i++)
        if (!strcmp(name, k_dac_names[i])) return i;
    return -1;
}

void emi_test_reply(const uint8_t *buf, size_t len) {
    if (len > s.budget) s.budget = 0;
    else                s.budget -= len;
    /* Replies are one JSON line each; bulk read-back chunks never start like an error. */
    static const char k_err[] = "{\"status\":\"error\"";
    if (len < sizeof(k_err) - 1 || memcmp(buf, k_err, sizeof(k_err) - 1) != 0) return;
    s.errors++;
    s.last_err_ms = HAL_GetTick();
    const char *msg = strstr((const char *)buf, "\"message\":\"");
    char text[80] = {0};
    if (msg) {
        msg += 11;
        size_t n = 0;
        while (n < sizeof(text) - 1 && msg + n < (const char *)buf + len && msg[n] != '"') n++;
        memcpy(text, msg, n);
    }
    snprintf(s.last_err, sizeof(s.last_err), "%s: %s", s.cur ? s.cur : "?", text);
    printf("[emi] %s refused: %s\n", s.cur ? s.cur : "?", text);
}

size_t emi_test_avail(void) { return s.budget; }

uint8_t emi_test_led_state(void) {
    if (!s.booted) return STATUS_LED_EMI_WAITING;
    if (!s.running) return STATUS_LED_EMI_NONE;
    const uint32_t now = HAL_GetTick();
    if (s.errors && now - s.last_err_ms < EMI_LED_ERROR_MS) return STATUS_LED_EMI_ERRORS;
    if (ns.err && now - ns.t_err < EMI_LED_ERROR_MS) return STATUS_LED_EMI_ERRORS;   /* Ethernet */
    return STATUS_LED_EMI_RUNNING;
}

/* "off", or "20 Mbit/s, sent 1200, drops 0, link up, link losses 0". */
static void net_describe(char *buf, size_t cap) {
    if (!ns.mbit) { snprintf(buf, cap, "off"); return; }
    snprintf(buf, cap, "%lu Mbit/s, sent %lu, drops %lu, link %s, link losses %lu",
             (unsigned long)ns.mbit, (unsigned long)ns.sent, (unsigned long)ns.drops,
             ns.link ? "up" : "down", (unsigned long)ns.link_losses);
}

size_t emi_test_describe(char *buf, size_t cap) {
    const uint32_t up = s.running ? (HAL_GetTick() - s.t_start) / 1000u : 0u;
    char uart[8] = "off";
    if (s.uart_la) snprintf(uart, sizeof(uart), "LA%d", s.uart_la);
    char net[112];
    net_describe(net, sizeof(net));
    int len = snprintf(buf, cap,
                       "emi: %s, dac %s, uart %s, net %s, core %lu MHz, %lus, captures %lu, "
                       "errors %lu%s%s",
                       s.running ? "running" : (s.booted ? "stopped" : "waiting for boot start"),
                       k_dac_names[s.dac], uart, net, (unsigned long)(SystemCoreClock / 1000000u),
                       (unsigned long)up, (unsigned long)s.captures, (unsigned long)s.errors,
                       s.last_err[0] ? ", last: " : "", s.last_err);
    if (len < 0) return 0;
    return (size_t)len < cap ? (size_t)len : cap - 1;
}

size_t emi_test_json(char *buf, size_t cap) {
    const uint32_t up = s.running ? (HAL_GetTick() - s.t_start) / 1000u : 0u;
    int len = snprintf(buf, cap,
                       "{\"running\":%s,\"dac\":\"%s\",\"uart_la\":%d,\"core_mhz\":%lu,\"uptime_s\":%lu,"
                       "\"captures\":%lu,\"errors\":%lu,\"net_mbit\":%lu,"
                       "\"net_sent\":%lu,\"net_drops\":%lu,\"net_link\":%s,\"net_link_losses\":%lu}",
                       s.running ? "true" : "false", k_dac_names[s.dac], s.uart_la,
                       (unsigned long)(SystemCoreClock / 1000000u), (unsigned long)up,
                       (unsigned long)s.captures, (unsigned long)s.errors, (unsigned long)ns.mbit,
                       (unsigned long)ns.sent, (unsigned long)ns.drops,
                       ns.link ? "true" : "false", (unsigned long)ns.link_losses);
    if (len < 0 || (size_t)len >= cap) { if (cap) buf[0] = '\0'; return 0; }
    return (size_t)len;
}

void emi_test_poll(void) {
    const uint32_t now = HAL_GetTick();
    s.budget = EMI_SINK_BUDGET;   /* the read-back's share of this worker pass */
    if (!s.booted) {
        if (now >= EMI_TEST_START_DELAY_MS) emi_test_start();
        return;
    }
    if (s.cap_issued && !capture_busy() && heavy_owner_conn() != CH_EMI_CONN) {
        s.cap_issued = false;
        if (s.errors == s.cap_errors) s.captures++;   /* else its read-back failed */
        s.t_next_cap = now + EMI_CAPTURE_GAP_MS;
    }
    if (!s.running) return;
    if (!s.cap_issued && (int32_t)(now - s.t_next_cap) >= 0) {
        s.cap_errors = s.errors;
        if (!board_has_analog()) s.next_kind = 1;   /* LA only: capture_dual takes the ADC too */
        run(s.next_kind ? "la_capture" : "capture_dual", k_captures[s.next_kind]);
        s.next_kind = (s.next_kind + 1u) % (sizeof(k_captures) / sizeof(k_captures[0]));
        if (s.errors == s.cap_errors) s.cap_issued = true;
        else                          s.t_next_cap = now + 1000u;   /* refused: the other kind next */
    }
    if (now - s.t_report >= EMI_TEST_REPORT_MS) {
        s.t_report = now;
        char line[256];
        emi_test_describe(line, sizeof(line));
        printf("[emi] %s\n", line);
    }
}
