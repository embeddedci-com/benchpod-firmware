/*
 * emi_test — the EMC pre-compliance activity loop (emi_test.h). Built only with EMI_TEST=1.
 */
#include "emi_test.h"
#include "command_handler.h"
#include "command_handler_internal.h"   /* capture_busy */
#include "stm32h5xx_hal.h"              /* HAL_GetTick */

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

static const char *const k_dac_names[] = { "sine", "dc", "current", "off" };

static struct {
    bool           booted;        /* the boot start has happened (or `emi stop` came first) */
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
    const char    *cur;           /* the command in flight, for the error log */
    char           last_err[112];
    size_t         budget;
} s = { .dac = EMI_DAC_SINE };

static void run(const char *label, const char *json) {
    s.cur = label;
    command_handler_process(CH_EMI_CONN, (const uint8_t *)json, strlen(json));
}

/* Stop whatever the DAC plays (s.dac), then start `mode`. */
static void apply_dac(emi_dac_mode_t mode) {
    run("dac_stop", "{\"cmd\":\"dac_stop\"}");
    if (s.dac == EMI_DAC_CURRENT && mode != EMI_DAC_CURRENT)   /* dac_stop leaves the loop current */
        run("current_out", "{\"cmd\":\"current_out\",\"ua\":4000}");
    switch (mode) {
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
    printf("[emi] starting the EMI test activity\n");
    run("la_voltage", "{\"cmd\":\"la_voltage\",\"mv\":3300}");
    run("target_power 1", "{\"cmd\":\"target_power\",\"efuse\":1,\"state\":1}");
    run("target_power 2", "{\"cmd\":\"target_power\",\"efuse\":2,\"state\":1}");
    apply_dac(s.dac);
    apply_uart(s.uart_la);
}

void emi_test_stop(void) {
    s.booted = true;   /* a stop before the boot delay ran out also cancels the boot start */
    if (!s.running) return;
    s.running = false;
    run("dac_stop", "{\"cmd\":\"dac_stop\"}");
    if (s.dac == EMI_DAC_CURRENT) run("current_out", "{\"cmd\":\"current_out\",\"ua\":4000}");
    run("analog_path", "{\"cmd\":\"analog_path\",\"path\":\"off\"}");
    run("gps_stop", "{\"cmd\":\"gps_stop\"}");
    run("target_power 1", "{\"cmd\":\"target_power\",\"efuse\":1,\"state\":0}");
    run("target_power 2", "{\"cmd\":\"target_power\",\"efuse\":2,\"state\":0}");
    /* A capture still in flight finishes on its own and frees the heavy gate. */
    printf("[emi] stopped: DAC off, target power off\n");
}

bool emi_test_running(void) { return s.running; }

void emi_test_set_dac(emi_dac_mode_t mode) {
    if (s.running) apply_dac(mode);
    s.dac = mode;
}

void emi_test_set_uart(int la) {
    if (la < 0 || la > 14) return;
    s.uart_la = la;
    if (s.running) apply_uart(la);
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

size_t emi_test_describe(char *buf, size_t cap) {
    const uint32_t up = s.running ? (HAL_GetTick() - s.t_start) / 1000u : 0u;
    int n = snprintf(buf, cap,
                     "emi: %s, dac %s, uart %s%d, %lus, captures %lu, errors %lu%s%s",
                     s.running ? "running" : (s.booted ? "stopped" : "waiting for boot start"),
                     k_dac_names[s.dac], s.uart_la ? "LA" : "off", s.uart_la,
                     (unsigned long)up, (unsigned long)s.captures, (unsigned long)s.errors,
                     s.last_err[0] ? ", last: " : "", s.last_err);
    if (n < 0) return 0;
    return (size_t)n < cap ? (size_t)n : cap - 1;
}

size_t emi_test_json(char *buf, size_t cap) {
    const uint32_t up = s.running ? (HAL_GetTick() - s.t_start) / 1000u : 0u;
    int n = snprintf(buf, cap,
                     "{\"running\":%s,\"dac\":\"%s\",\"uart_la\":%d,\"uptime_s\":%lu,"
                     "\"captures\":%lu,\"errors\":%lu}",
                     s.running ? "true" : "false", k_dac_names[s.dac], s.uart_la, (unsigned long)up,
                     (unsigned long)s.captures, (unsigned long)s.errors);
    if (n < 0 || (size_t)n >= cap) { if (cap) buf[0] = '\0'; return 0; }
    return (size_t)n;
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
        run(s.next_kind ? "la_capture" : "capture_dual", k_captures[s.next_kind]);
        s.next_kind = (s.next_kind + 1u) % (sizeof(k_captures) / sizeof(k_captures[0]));
        if (s.errors == s.cap_errors) s.cap_issued = true;
        else                          s.t_next_cap = now + 1000u;   /* refused: the other kind next */
    }
    if (now - s.t_report >= EMI_TEST_REPORT_MS) {
        s.t_report = now;
        char line[200];
        emi_test_describe(line, sizeof(line));
        printf("[emi] %s\n", line);
    }
}
