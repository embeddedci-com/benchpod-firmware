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
#include "fault.h"                      /* fault_last_reset_str */
#include "emi_log.h"
#include "w25q.h"                       /* w25q_open: the run log's flash */
#include "version.h"                    /* FIRMWARE_VERSION */
#include "bp_json.h"
#include "at_driver.h"                  /* at_send_data */
#include "b64url.h"
#include "pico/time.h"                  /* time_us_32: the DAC glitch of a log write */
#include "FreeRTOS.h"                   /* pvPortMalloc: the log lives on the heap */
#include "task.h"                       /* vTaskDelay before a recovery reset */
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "recovery.h"
#include "ice40_flash.h"                /* ice40_reload: the recovery's iCE40 step (no write) */
#include "target_power.h"               /* power_poll_emi: an eFuse that tripped; reverse blocks */
#include "net_server.h"                 /* net_eth_restart */
#include "boot_guard.h"                 /* boot_guard_planned_reset */
#include "esp_wifi_ctrl.h"              /* C3 restarts (c3_restarts.h) */
#include "cloud_client.h"               /* health: cloud drops */
#include "i2c_bus.h"                    /* health: I2C bus clears */
#include "can_bus.h"                    /* health: CAN bus-off recoveries */
#include "ice40_flash.h"                /* health: CDONE */
#include "eth_diag.h"                   /* summary: Ethernet CRC and missed frames */
#if defined(BENCHPOD_WIFI_BRIDGE)
#include "wifi_bridge_net.h"            /* summary: the bridge's failed transmits */
#endif

#include <stdio.h>
#include <stdlib.h>
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

/* A one-off failure (a refused command or capture, a frame not sent) shows red this long; after
   that the LED stays blue until power is removed (status_led.h). A missing Ethernet link stays red
   for as long as it lasts. */
#define EMI_LED_FAIL_MS  5000u
/* Failing this long without a break is a fail under the lab's criterion B (it must recover within
   60 s): the LED turns from blue (recovering) to red. The recovery ladder (recovery.h) is back by
   about 45 s after a failure starts. */
#define EMI_LED_VERDICT_MS  60000u
/* A capture still in flight after this long has hung (the loop's take well under 2 s): it counts
   as failing from when it went out, for the LED and the recovery ladder alike. */
#define EMI_CAP_HUNG_MS  10000u
/* An eFuse that trips on the DUT supply (ESD or a surge on the target power) is a failure the
   lab must see (red, then blue), and the loop turns that supply off and on again after
   EMI_EFUSE_RETRY_MS so it can recover by itself, at most EMI_EFUSE_RETRIES_PER_HOUR times. */
#define EMI_EFUSE_RETRY_MS          2000u
#define EMI_EFUSE_RETRIES_PER_HOUR  6u

/* Any failure, or a restart that was not a power-on, since power-on: the LED shows blue from then
   on. Not cleared by `emi stop` / `emi start`, only by removing power. */
static bool s_failed_before;
static bool s_reset_checked;

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
    uint32_t       hold_until;    /* no new capture before this: a log read wants the bus */
    uint8_t        last_led;      /* for the run log's LED events */
    bool           last_link;
    bool           link_seen;     /* the link was up once since the start */
    uint32_t       t_checkpoint;
    emi_cp_sig_t   cp_last;       /* the values in the last checkpoint */
    /* the recovery ladder (recovery.h) */
    uint32_t       fail_since;    /* captures failing since (0 = working) */
    bool           power_fault;   /* an eFuse tripped and has not come back yet (power_poll_emi) */
    bool           efuse_fault[3];/* per eFuse (1, 2), the last seen FLT state */
    bool           efuse_off[3];  /* per eFuse: switched off at the retry limit */
    uint32_t       efuse_retry_ms[EMI_EFUSE_RETRIES_PER_HOUR];   /* ring: when power was retried */
    uint32_t       efuse_retry_n;
    uint32_t       cap_issued_at; /* the capture in flight went out then */
    uint32_t       link_down_since;
    bool           rcv_init;
    /* ESP32-C3 restarts (the built-in Wi-Fi, v3 boards): counted from the pod's own counts */
    bool           c3_init;
    uint32_t       c3_seen, c3_seen_assoc;   /* the pod's counts at the last look */
    uint32_t       c3_restarts;   /* restarts while the activity ran, association failures left out */
    uint32_t       c3_last_ms;    /* HAL_GetTick() of the last one, 0 = none */
    /* the Wi-Fi bridge (lab configuration 2): watched once one was seen this run */
    bool           br_seen;
    uint32_t       br_reboots_seen;   /* the pod's count at the last look */
    uint32_t       br_restarts;       /* restarts this run */
    uint32_t       br_restart_ms;     /* the last one (0 = none), for the LED */
    uint32_t       br_restart_pending;/* a restart waiting for its reason (the next STATUS), since */
    uint32_t       br_silent_from;    /* no HELLO since (0 = present) */
    bool           br_silent_logged;
    uint32_t       br_silences;       /* this run */
    bool           br_radio_failing;  /* the EMC build's radio reports it is not transmitting */
    /* the LED's verdict (emi_test_led_state) */
    uint32_t       failing_from;  /* failing without a break since (0 = not failing) */
    bool           gave_up;       /* the recovery is out of steps for this failure */
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
    uint32_t          mon_sent;   /* the monitor line generation last sent */
    uint32_t          mon_frames;
} ns = { .mbit = EMI_NET_DEFAULT_MBIT };

/* The monitor line, every EMI_TEST_REPORT_MS (publish_monitor, hw worker): on the USB console while
   the loop runs, and as a BPMON frame on the Ethernet port from boot on (emi_test_net_poll, net
   task). Two buffers: the worker writes the one the net task is not reading, then flips `cur`. */
typedef struct {
    char              buf[2][384];
    volatile uint8_t  cur;
    volatile uint32_t gen;
} emi_mon_t;
/* On the heap (allocated by the first publish_monitor): the pod's RAM has no 1 KB to spare. */
static emi_mon_t *volatile s_mon;

/* At most this many frames per net_poll pass (HAL_ETH_Transmit waits for each one), so the
   stream never holds the net task for long. */
#define EMI_NET_BURST  4u

static void send_monitor(struct netif *eth);

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

/* Start `mode` on outputs that are stopped or already play it. Also what puts the DAC back after
   a W25Q session (a run log write or read), whose quiesce stopped it. */
static void start_dac(emi_dac_mode_t mode) {
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

/* After a W25Q session whose quiesce stopped the DAC (the deep-replay fallback, or a DAC_RUN
   found clear): the stop left the quad DAC's held channels (the DC on B, the 12 mA) and the
   sine's table in the gateware's BRAM, so the sine only needs its START_DAC again. The DC is written again (a single-DAC board has no held
   channel); the 4-20 mA output keeps its current (dac_stop leaves it). Falls back to start_dac
   when the table is gone. */
static void resume_dac(emi_dac_mode_t mode) {
    switch (mode) {
    case EMI_DAC_ALL:
    case EMI_DAC_SINE:
        if (dac_restart_generated() != 0) start_dac(mode);
        break;
    case EMI_DAC_DC:
        start_dac(mode);
        break;
    default:
        break;
    }
}

/* Stop whatever the DAC plays (s.dac), then start `mode`. */
static void apply_dac(emi_dac_mode_t mode) {
    run("dac_stop", "{\"cmd\":\"dac_stop\"}");
    if (s.dac == EMI_DAC_ALL)   /* parks the 0-5 V output it held */
        run("analog_path", "{\"cmd\":\"analog_path\",\"path\":\"off\"}");
    if (drives_current(s.dac) && !drives_current(mode))   /* dac_stop leaves the loop current */
        run("current_out", "{\"cmd\":\"current_out\",\"ua\":4000}");
    start_dac(mode);
}

/* UART activity: an emulated UART sensor sending two NMEA sentences every 100 ms at 115200 baud
   (the definition below, 302 bytes of JSON, base64 in one chunk). Starting it loads the emulation
   image, which keeps the sine and DC outputs this loop plays. */
#define EMI_UART_DEF_LEN 302
static const char k_emi_uart_def_b64[] =
    "eyJmb3JtYXQiOjEsImtpbmQiOiJ1YXJ0IiwibmFtZSI6ImVtaV91YXJ0IiwidWFydCI6eyJiYXVkIjoxMTUyMDAsImZvcm1hdCI6IjhOMSJ9"
    "LCJtZXNzYWdlcyI6W3siZXZlcnlfbXMiOjEwMCwidGV4dCI6IiRHUFJNQywxMjM1MTksQSw0ODA3LjAzOCxOLDAxMTMxLjAwMCxFLDAyMi40"
    "LDA4NC40LDIzMDM5NCwwMDMuMSxXIiwiZW5kIjoibm1lYSJ9LHsiZXZlcnlfbXMiOjEwMCwidGV4dCI6IiRHUEdHQSwxMjM1MTksNDgwNy4w"
    "MzgsTiwwMTEzMS4wMDAsRSwxLDA4LDAuOSw1NDUuNCxNLDQ2LjksTSwsIiwiZW5kIjoibm1lYSJ9XX0=";

static void apply_uart(int la) {
    run("sensor_stop", "{\"cmd\":\"sensor_stop\",\"instance\":\"emi_uart\"}");
    if (la <= 0) return;
    static char cmd[600];
    snprintf(cmd, sizeof(cmd), "{\"cmd\":\"sensor_define\",\"instance\":\"emi_uart\",\"offset\":0,\"total\":%d,"
             "\"data_b64\":\"%s\"}", EMI_UART_DEF_LEN, k_emi_uart_def_b64);
    run("sensor_define", cmd);
    snprintf(cmd, sizeof(cmd), "{\"cmd\":\"sensor_start\",\"instance\":\"emi_uart\",\"pins\":{\"tx\":%d}}", la);
    run("sensor_start", cmd);
}


/* ---- the run log (emi_log.h) ---------------------------------------------------------------
   The W25Q shares its bus with the PSRAM, so every log access is a W25Q session, and it may not
   start while a capture or upload owns the bus (heavy_or_claimed). The DAC keeps playing through
   it: the sine plays from the gateware's BRAM table and the DC and 4-20 mA outputs are held by the
   quad DAC, none of which touch the bus (signal_engine_quiesce_psram_keep_dac). Only a deep
   replay, which reads the PSRAM, would be stopped and started again; the EMI loop never plays
   one. So the log is scanned and the sectors it needs erased at boot, before the activity starts;
   records wait in a RAM queue and are written between two captures, all in one session, which
   delays the next capture by the session's time (session_us). After a session the DAC_RUN bit
   is checked: any time the DAC was not playing goes into glitch_us (0 in the normal path), and
   the longest into the next checkpoint. */

/* Erased at boot, ahead of the write position: 16 sectors, 1,024 records, about 2.8 hours of
   checkpoints (measured 58 ms a sector, so about 1 s when all 16 need it, at most 6.4 s at the
   W25Q's 400 ms worst case, inside the start delay; sectors already blank are skipped). A run
   that uses them up erases the next sector between two captures (that capture waits about 60 ms,
   up to 400 ms; the outputs keep running). */
#define EMI_LOG_BOOT_RESERVE   (16u * EMI_LOG_PER_SECTOR)
#define EMI_LOG_QUEUE          16u
/* Write queued records at most this often (except when the queue fills), so a flapping link
   cannot turn into a stream of W25Q sessions. */
#define EMI_LOG_FLUSH_MS       1000u
/* emi_log: records per read. The reply must fit a LAN connection's 2 KB TX ring whole (conn_tx):
   16 records are 1,366 characters of base64 beside ~420 of state. 12 on the cloud link
   (BP_CLOUD_REPLY_MAX, 1,792). */
#define EMI_LOG_READ_MAX       16u
#define EMI_LOG_READ_MAX_CLOUD 12u
/* After a log read, the loop waits this long before its next capture, so a client reading the log
   chunk after chunk finds the bus free. */
#define EMI_LOG_READ_HOLD_MS   1500u

/* The log's state and queue: on the heap, allocated at boot (log_boot), since the pod's RAM is
   nearly all taken; buffers for reads are allocated while they are needed. */
typedef struct {
    emi_log_t log;
    emi_rec_t q[EMI_LOG_QUEUE];
} emi_logmem_t;
static emi_logmem_t *s_lm;
#define s_log  (s_lm->log)
static bool log_ok(void) { return s_lm && s_lm->log.ok; }
static struct {
    unsigned  n;
    uint32_t  dropped;       /* queue full */
    uint32_t  t_flush;
    uint32_t  writes;        /* records written since boot */
    uint32_t  sessions;      /* W25Q sessions while the DAC ran */
    uint32_t  dac_stops;     /* of those, how many found the DAC stopped (deep-replay fallback) */
    uint32_t  glitch_us_last, glitch_us_max;    /* the DAC not playing, 0 when it kept running */
    uint32_t  session_us_last, session_us_max;  /* the W25Q session (wait for the bus + flash) */
    bool      dac_stopped;   /* this session's quiesce stopped the DAC */
    bool      tried;         /* the boot scan ran */
} lg;

/* The `emi log` print job: walks the slots, a sector at a time read in one session, and prints
   a few lines per worker pass on the device log. */
static struct {
    bool     active;
    uint32_t slot, left, printed;
    int      cached;         /* the sector in buf, -1 = none */
    uint32_t t;
    uint8_t *buf;            /* EMI_LOG_SECTOR bytes while the job runs */
} pj = { .cached = -1 };

static int log_read(void *ctx, uint32_t addr, uint8_t *buf, uint32_t len) {
    (void)ctx;
    return w25q_read(addr, buf, len);
}
static int log_program(void *ctx, uint32_t addr, const uint8_t *buf, uint32_t len) {
    (void)ctx;
    return w25q_program(addr, buf, len);
}
static int log_erase(void *ctx, uint32_t addr) {
    (void)ctx;
    return w25q_erase_sector(addr);
}
static const emi_log_io_t k_log_io = { log_read, log_program, log_erase, NULL };

/* The bus is free for a W25Q session: no capture of the loop's or a client's, no upload. */
static bool log_bus_free(void) { return !s.cap_issued && !heavy_or_claimed(); }

/* Open a W25Q session that leaves the DAC playing (unless a deep replay holds the bus);
   *t0 = when. Not w25q_session_open: its quiesce stops the DAC, for the OTA and CA writers. */
static int log_session_open(uint32_t *t0) {
    *t0 = time_us_32();
    lg.dac_stopped = signal_engine_quiesce_psram_keep_dac();
    return w25q_open();
}
/* Close it. The DAC is started again only when the quiesce stopped it, or when the sine's
   DAC_RUN bit is found clear; that time is the glitch, 0 when the DAC kept running. */
static void log_session_close(uint32_t t0) {
    w25q_close();
    const uint32_t session = time_us_32() - t0;
    lg.session_us_last = session;
    if (session > lg.session_us_max) lg.session_us_max = session;
    if (s.running && board_has_analog() && s.dac != EMI_DAC_OFF) {
        const bool sine = s.dac == EMI_DAC_SINE || s.dac == EMI_DAC_ALL;
        uint32_t us = 0;
        if (lg.dac_stopped || (sine && !signal_engine_dac_run_bit())) {
            resume_dac(s.dac);
            us = time_us_32() - t0;
            lg.dac_stops++;
        }
        lg.sessions++;
        lg.glitch_us_last = us;
        if (us > lg.glitch_us_max) lg.glitch_us_max = us;
    }
    lg.dac_stopped = false;
}

static void log_queue(const emi_rec_t *r) {
    if (!log_ok()) return;
    if (lg.n >= EMI_LOG_QUEUE) { lg.dropped++; return; }
    s_lm->q[lg.n] = *r;
    s_lm->q[lg.n].uptime_ms = HAL_GetTick();
    lg.n++;
}

/* Copy src into a fixed-size record field, cut to fit. */
static void cut(char *dst, size_t n, const char *src) {
    size_t i = 0;
    if (src) for (; i + 1 < n && src[i]; i++) dst[i] = src[i];
    dst[i] = '\0';
}

static void log_event(uint8_t kind, uint8_t from, uint8_t to) {
    emi_rec_t r;
    memset(&r, 0, sizeof(r));
    r.type = EMI_REC_EVENT;
    r.led = emi_test_led_state();
    r.u.ev.kind = kind;
    r.u.ev.from = from;
    r.u.ev.to = to;
    r.u.ev.captures = s.captures;
    r.u.ev.errors = s.errors;
    r.u.ev.net_sent = ns.sent;
    cut(r.u.ev.text, sizeof(r.u.ev.text), s.last_err);
    log_queue(&r);
}

/* A recovery step's record: the event, with its text. */
static void log_event_text(uint8_t kind, const char *text) {
    emi_rec_t r;
    memset(&r, 0, sizeof(r));
    r.type = EMI_REC_EVENT;
    r.led = emi_test_led_state();
    r.u.ev.kind = kind;
    r.u.ev.captures = s.captures;
    r.u.ev.errors = s.errors;
    r.u.ev.net_sent = ns.sent;
    cut(r.u.ev.text, sizeof(r.u.ev.text), text);
    log_queue(&r);
}

static emi_cp_sig_t cp_sig(void) {
    emi_cp_sig_t g;
    memset(&g, 0, sizeof(g));
    g.errors = s.errors;
    g.net_drops = ns.drops;
    g.link_losses = ns.link_losses;
    g.rec_dropped = lg.dropped;
    g.glitch_us_max = lg.glitch_us_max;
    g.led = emi_test_led_state();
    g.dac = (uint8_t)s.dac;
    g.running = s.running;
    g.link = ns.link;
    return g;
}

static void log_checkpoint(void) {
    s.cp_last = cp_sig();
    emi_rec_t r;
    memset(&r, 0, sizeof(r));
    r.type = EMI_REC_CHECKPOINT;
    r.led = emi_test_led_state();
    r.u.cp.captures = s.captures;
    r.u.cp.errors = s.errors;
    r.u.cp.net_sent = ns.sent;
    r.u.cp.net_drops = ns.drops;
    r.u.cp.link_losses = ns.link_losses;
    r.u.cp.rec_dropped = lg.dropped;
    r.u.cp.glitch_us_max = lg.glitch_us_max;
    r.u.cp.dac = (uint8_t)s.dac;
    r.u.cp.net_mbit = (uint8_t)ns.mbit;
    r.u.cp.running = s.running;
    r.u.cp.link = ns.link;
    cut(r.u.cp.text, sizeof(r.u.cp.text), s.last_err);
    log_queue(&r);
}

/* The first time: the quad DAC runs every output by default; a digital-only board has no DAC. */
static void choose_default_dac(void) {
    if (s.booted_mode) return;
    s.booted_mode = true;
    if (dac_quad_present()) s.dac = EMI_DAC_ALL;
    if (!board_has_analog()) s.dac = EMI_DAC_OFF;   /* digital-only board: no DAC, no ADC */
}

/* ---- recovery (recovery.h) -----------------------------------------------------------------
   The recovery ladder's resets survive in .noinit (random at power-on, so a power-on clears them
   through the magic): how many STM32 resets the ladder made, and whether the last reset was one,
   for the next run's RUN_START. */
#define EMI_RCV_MAGIC 0x31564352u   /* "RCV1" */
static struct {
    uint32_t magic;
    uint32_t resets;
    uint32_t reset_pending;
    uint32_t fault_stuck;          /* `emi fault ice40-stuck`: survives the ladder's resets */
} s_keep __attribute__((section(".noinit")));
static recovery_t s_rcv;

static void keep_init(void) {
    if (s_keep.magic != EMI_RCV_MAGIC) {
        s_keep.magic = EMI_RCV_MAGIC;
        s_keep.resets = 0;
        s_keep.reset_pending = 0;
        s_keep.fault_stuck = 0;
    }
}

bool emi_test_fault(const char *name) {
    keep_init();
    if (!strcmp(name, "off")) { s_keep.fault_stuck = 0; printf("[emi] fault test off\n"); return true; }
    if (strcmp(name, "ice40") && strcmp(name, "ice40-stuck")) return false;
    s_keep.fault_stuck = !strcmp(name, "ice40-stuck");
    printf("[emi] fault test: holding the iCE40 in reset%s\n", s_keep.fault_stuck ? " (stuck: again after every reload)" : "");
    ice40_hold_off_bus();
    return true;
}
static bool s_quick_start;   /* this boot is the recovery ladder's reset: start the loop sooner */
static bool emi_recovery_reset_pending(void) {
    keep_init();
    return s_keep.reset_pending != 0;
}

/* At boot, before the activity: scan, erase the reserve, write RUN_START. */
static void log_boot(void) {
    lg.tried = true;
    const uint32_t t0 = HAL_GetTick();
    s_lm = pvPortMalloc(sizeof(*s_lm));
    if (!s_lm) { printf("[emi] run log: out of memory, no log this run\n"); return; }
    memset(s_lm, 0, sizeof(*s_lm));
    if (w25q_session_open() != 0) {
        printf("[emi] run log: the W25Q does not answer, no log this run\n");
        return;
    }
    int rc = emi_log_scan(&s_log, &k_log_io, EMI_LOG_BASE, EMI_LOG_SECTORS);
    const uint32_t t_scan = HAL_GetTick();
    int erased = rc == 0 ? emi_log_prepare(&s_log, EMI_LOG_BOOT_RESERVE) : -1;
    if (rc == 0 && erased >= 0) {
        emi_rec_t r;
        memset(&r, 0, sizeof(r));
        r.type = EMI_REC_RUN_START;
        r.led = emi_test_led_state();
        r.uptime_ms = HAL_GetTick();
        cut(r.u.start.fw, sizeof(r.u.start.fw), FIRMWARE_VERSION);
        cut(r.u.start.reset, sizeof(r.u.start.reset),
            emi_recovery_reset_pending() ? "recovery reset" : fault_last_reset_str());
        r.u.start.core_mhz = (uint16_t)(SystemCoreClock / 1000000u);
        /* The mode the run will play: the boot start picks the default (choose_default_dac) only
           later, once the DAC is known, so name that default here without fixing it. */
        r.u.start.dac = (uint8_t)(s.booted_mode ? s.dac
                                 : !board_has_analog() ? EMI_DAC_OFF
                                 : dac_quad_present() ? EMI_DAC_ALL : s.dac);
        r.u.start.net_mbit = (uint8_t)ns.mbit;
        r.u.start.gw_version = signal_engine_fpga_version();
        if (emi_log_append(&s_log, &r) == 0) lg.writes++;
    }
    s_keep.reset_pending = 0;
    w25q_session_close();
    if (rc != 0 || erased < 0) {
        s_lm->log.ok = false;
        printf("[emi] run log: scan or erase failed (%d/%d), no log this run\n", rc, erased);
        return;
    }
    printf("[emi] run log: run %lu, %lu records in %lu slots, scan %lu ms, erased %d sectors in %lu ms\n",
           (unsigned long)s_log.run, (unsigned long)s_log.valid, (unsigned long)s_log.slots,
           (unsigned long)(t_scan - t0), erased, (unsigned long)(HAL_GetTick() - t_scan));
}

/* Write the queued records, in one session. */
static void log_flush(void) {
    uint32_t t0;
    if (log_session_open(&t0) != 0) { printf("[emi] run log: W25Q session failed\n"); return; }
    unsigned done = 0;
    int erased = 0;
    for (; done < lg.n; done++) {
        if (s_log.ready == 0) {
            int e = emi_log_prepare(&s_log, EMI_LOG_PER_SECTOR);
            if (e < 0) break;
            erased += e;
        }
        if (emi_log_append(&s_log, &s_lm->q[done]) != 0) break;
        lg.writes++;
    }
    log_session_close(t0);
    if (done < lg.n) {
        printf("[emi] run log: write failed, %u records lost\n", lg.n - done);
        lg.dropped += lg.n - done;
    }
    lg.n = 0;
    lg.t_flush = HAL_GetTick();
    if (s.running && board_has_analog() && s.dac != EMI_DAC_OFF)
        printf("[emi] run log: wrote %u, W25Q session %lu us%s, DAC gap %lu us\n", done,
               (unsigned long)lg.session_us_last, erased ? " (with a sector erase)" : "",
               (unsigned long)lg.glitch_us_last);
}

static void log_fill_poll(void);

/* Every worker pass once booted: events, checkpoints, the write, the print job. */
static void log_poll(uint32_t now) {
    if (!log_ok()) return;
    const uint8_t led = emi_test_led_state();
    if (led != s.last_led) {
        log_event(EMI_EV_LED, s.last_led, led);
        s.last_led = led;
    }
    const bool link = ns.link;
    /* Only once the link was up since the start: its first coming up is no event. */
    if (s.running && ns.had_link && s.link_seen && link != s.last_link)
        log_event(link ? EMI_EV_LINK_BACK : EMI_EV_LINK_LOST, 0, 0);
    if (s.running && ns.had_link) s.link_seen = true;
    s.last_link = link;
    const emi_cp_sig_t sig = cp_sig();
    if (emi_log_checkpoint_due(&s.cp_last, &sig, now - s.t_checkpoint)) {
        s.t_checkpoint = now;
        log_checkpoint();
    }
    log_fill_poll();
    if (lg.n && log_bus_free() && (now - lg.t_flush >= EMI_LOG_FLUSH_MS || lg.n >= EMI_LOG_QUEUE - 2u))
        log_flush();

    /* The print job: up to 4 lines per pass, a pass at most every 5 ms (the USB console's pace). */
    if (!pj.active || now - pj.t < 5u) return;
    pj.t = now;
    for (unsigned lines = 0, steps = 0; lines < 4 && steps < 256; steps++) {
        if (!pj.left) {
            printf("[emi] log: %lu records printed\n", (unsigned long)pj.printed);
            pj.active = false;
            pj.cached = -1;
            vPortFree(pj.buf);
            pj.buf = NULL;
            return;
        }
        const uint32_t slot = pj.slot;
        if (!emi_log_slot_good(&s_log, slot)) { pj.slot = (slot + 1u) % s_log.slots; pj.left--; continue; }
        const int sector = (int)(slot / EMI_LOG_PER_SECTOR);
        if (sector != pj.cached) {
            if (!log_bus_free()) { s.hold_until = now + EMI_LOG_READ_HOLD_MS; return; }
            uint32_t t0;
            if (log_session_open(&t0) != 0) return;
            int rc = w25q_read(EMI_LOG_BASE + (uint32_t)sector * EMI_LOG_SECTOR, pj.buf, EMI_LOG_SECTOR);
            log_session_close(t0);
            if (rc) {
                printf("[emi] log: read failed\n");
                pj.active = false;
                vPortFree(pj.buf);
                pj.buf = NULL;
                return;
            }
            pj.cached = sector;
            s.hold_until = now + EMI_LOG_READ_HOLD_MS;
            return;   /* print from the next pass on */
        }
        emi_rec_t r;
        char line[320];   /* a SUMMARY line runs to ~260 characters */
        if (emi_log_decode(pj.buf + (slot % EMI_LOG_PER_SECTOR) * EMI_LOG_REC, &r)) {
            emi_log_format(&r, line, sizeof(line));
            printf("[emi] log %s\n", line);
            pj.printed++;
            lines++;
        }
        pj.slot = (slot + 1u) % s_log.slots;
        pj.left--;
    }
}

#if defined(EMI_LOG_TEST_SECTORS)
/* Test builds with a small ring (make EMI_LOG_SECTORS=<n>): `emi log fill <n>` queues n
   checkpoints as fast as the queue drains, to see the ring wrap and erase between captures. */
static uint32_t s_fill;
void emi_test_log_fill(uint32_t n) { s_fill = n; }
static void log_fill_poll(void) {
    while (s_fill && lg.n < EMI_LOG_QUEUE - 2u) { log_checkpoint(); s_fill--; }
}
#else
static void log_fill_poll(void) {}
#endif

bool emi_test_log_print(uint32_t n) {
    if (!log_ok()) return false;
    const uint32_t span = s_log.slots - s_log.ready;
    uint32_t from = n ? emi_log_slot_back(&s_log, n) : s_log.slots;
    if (from == s_log.slots) from = emi_log_oldest_slot(&s_log);
    pj.slot = from;
    pj.left = (s_log.write_slot + s_log.slots - from) % s_log.slots;
    if (!pj.left && span) pj.left = s_log.slots;   /* full ring, from the oldest */
    if (!pj.buf) pj.buf = pvPortMalloc(EMI_LOG_SECTOR);
    if (!pj.buf) return false;
    pj.printed = 0;
    pj.cached = -1;
    pj.t = 0;
    pj.active = true;
    return true;
}

size_t emi_test_log_describe(char *buf, size_t cap) {
    int n;
    if (!log_ok())
        n = snprintf(buf, cap, "run log: %s", lg.tried ? "not available (W25Q)" : "not read yet");
    else
        n = snprintf(buf, cap,
                     "run log: run %lu, %lu records in %lu slots (0x%06lx-0x%06lx), %lu queued, %lu lost, "
                     "%lu written, %lu erases, %lu write errors, W25Q session last %lu us, max %lu us, "
                     "DAC gap last %lu us, max %lu us (%lu of %lu sessions)",
                     (unsigned long)s_log.run, (unsigned long)s_log.valid, (unsigned long)s_log.slots,
                     (unsigned long)EMI_LOG_BASE,
                     (unsigned long)(EMI_LOG_BASE + s_log.slots * EMI_LOG_REC),
                     (unsigned long)lg.n, (unsigned long)lg.dropped, (unsigned long)lg.writes,
                     (unsigned long)s_log.erases, (unsigned long)s_log.write_errors,
                     (unsigned long)lg.session_us_last, (unsigned long)lg.session_us_max,
                     (unsigned long)lg.glitch_us_last, (unsigned long)lg.glitch_us_max,
                     (unsigned long)lg.dac_stops, (unsigned long)lg.sessions);
    if (n < 0) return 0;
    return (size_t)n < cap ? (size_t)n : cap - 1;
}

static void handle_emi_log_with(int conn_id, uint32_t offset, uint32_t count, uint32_t max,
                                char *resp, size_t resp_cap, uint8_t *raw);

/* {"cmd":"emi_log"} the log's state; with "count" (and "offset", slots from 0) also that many raw
   64-byte slots as base64url in "data", in slot order (a host sorts them by seq). With "after"
   (a sequence number) in place of "offset": the slots from the first record newer than it, so a
   viewer that keeps a copy fetches only what it does not have; "count" is then cut to the records
   there are, and 0 when it has them all. An "after" older than the oldest record starts at the
   oldest. The reply's "offset" says where the read started. */
void handle_emi_log(int conn_id, const char *json) {
    if (!log_ok()) { send_error(conn_id, lg.tried ? "emi_log: no run log (W25Q)" : "emi_log: not read yet, retry after boot"); return; }
    char v[16];
    uint32_t offset = bp_json_get(json, "offset", v, sizeof(v)) ? (uint32_t)strtoul(v, NULL, 10) : 0u;
    uint32_t count  = bp_json_get(json, "count", v, sizeof(v)) ? (uint32_t)strtoul(v, NULL, 10) : 0u;
    const uint32_t max = conn_id == CH_CLOUD_CONN ? EMI_LOG_READ_MAX_CLOUD : EMI_LOG_READ_MAX;
    if (bp_json_get(json, "after", v, sizeof(v))) {
        uint32_t newer;
        offset = emi_log_slot_after(&s_log, (uint32_t)strtoul(v, NULL, 10), &newer);
        if (count > newer) count = newer;
    }
    if (offset >= s_log.slots) count = 0;
    if (count > max) count = max;
    if (count > s_log.slots - offset) count = s_log.slots - offset;
    /* The buffers come from the heap for the length of the call (the pod's RAM is nearly full). */
    enum { RESP_CAP = 1024 + B64URL_ENCODED_LEN(EMI_LOG_READ_MAX * EMI_LOG_REC) };
    char *resp = pvPortMalloc(RESP_CAP + EMI_LOG_READ_MAX * EMI_LOG_REC);
    if (!resp) { send_error(conn_id, "emi_log: out of memory"); return; }
    uint8_t *raw = (uint8_t *)resp + RESP_CAP;
    handle_emi_log_with(conn_id, offset, count, max, resp, RESP_CAP, raw);
    vPortFree(resp);
}

static void handle_emi_log_with(int conn_id, uint32_t offset, uint32_t count, uint32_t max,
                                char *resp, size_t resp_cap, uint8_t *raw) {
    if (count) {
        if (!log_bus_free()) {
            s.hold_until = HAL_GetTick() + EMI_LOG_READ_HOLD_MS;   /* the next try finds it free */
            send_error(conn_id, "busy: an EMI capture is using the bus, retry in a moment");
            return;
        }
        uint32_t t0;
        if (log_session_open(&t0) != 0) { send_error(conn_id, "emi_log: W25Q session failed"); return; }
        int rc = w25q_read(emi_log_slot_addr(&s_log, offset), raw, count * EMI_LOG_REC);
        log_session_close(t0);
        s.hold_until = HAL_GetTick() + EMI_LOG_READ_HOLD_MS;
        if (rc) { send_error(conn_id, "emi_log: read failed"); return; }
    }
    bp_emit_t e;
    bp_emit_init(&e, resp, resp_cap);
    bp_emit(&e, "{\"status\":\"ok\",\"data\":{\"base\":%lu,\"record\":%u,\"slots\":%lu,\"valid\":%lu,"
                "\"run\":%lu,\"next_seq\":%lu,\"write_slot\":%lu,\"oldest_slot\":%lu,\"queued\":%u,"
                "\"lost\":%lu,\"written\":%lu,\"erases\":%lu,\"write_errors\":%lu,"
                "\"session_us_last\":%lu,\"session_us_max\":%lu,\"glitch_us_last\":%lu,\"glitch_us_max\":%lu,"
                "\"dac_stops\":%lu,\"max_count\":%lu,\"uptime_ms\":%lu,\"offset\":%lu,\"count\":%lu",
            (unsigned long)EMI_LOG_BASE, (unsigned)EMI_LOG_REC, (unsigned long)s_log.slots,
            (unsigned long)s_log.valid, (unsigned long)s_log.run, (unsigned long)s_log.next_seq,
            (unsigned long)s_log.write_slot, (unsigned long)emi_log_oldest_slot(&s_log), lg.n,
            (unsigned long)lg.dropped, (unsigned long)lg.writes, (unsigned long)s_log.erases,
            (unsigned long)s_log.write_errors, (unsigned long)lg.session_us_last,
            (unsigned long)lg.session_us_max, (unsigned long)lg.glitch_us_last,
            (unsigned long)lg.glitch_us_max, (unsigned long)lg.dac_stops, (unsigned long)max,
            (unsigned long)HAL_GetTick(),
            (unsigned long)offset,
            (unsigned long)count);
    if (!count) {   /* the loop's whole state, with the log's (no room beside records) */
        char st[352];
        if (emi_test_json(st, sizeof(st))) bp_emit(&e, ",\"emi_test\":%s", st);
        bp_emit_raw(&e, ",\"last_error\":");
        bp_emit_jstr(&e, s.last_err);
        bp_emit(&e, ",\"fw\":\"%s\"", FIRMWARE_VERSION);
    }
    if (count) {
        bp_emit_raw(&e, ",\"data\":\"");
        char *dst = resp + bp_emit_len(&e);
        const size_t room = resp_cap - bp_emit_len(&e);
        const size_t n = b64url_encode(raw, count * EMI_LOG_REC, dst, room);
        if (!n) { send_error(conn_id, "emi_log: reply too large"); return; }
        e.len += n;
        bp_emit_raw(&e, "\"");
    }
    bp_emit_raw(&e, "}}\n");
    if (!bp_emit_ok(&e)) { send_error(conn_id, "emi_log: reply too large"); return; }
    if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) at_close_connection(conn_id);
}

/* Take one recovery step (recovery.h). Each one goes into the run log with what it did. */
static void recovery_take(recovery_action_t a, uint32_t now) {
    char text[28];
    switch (a) {
    case RECOVERY_ICE40_RELOAD: {
        printf("[emi] recovery: captures failing for %lu s, reloading the iCE40 from its flash\n",
               (unsigned long)((now - s.fail_since) / 1000u));
        const int rc = ice40_reload();   /* reset + configure from the flash: no erase, no write */
        if (s_keep.fault_stuck) ice40_hold_off_bus();   /* `emi fault ice40-stuck`: it stays down */
        const bool alive = rc == 0 && signal_engine_refresh_version();
        snprintf(text, sizeof(text), "iCE40 reload %s v%u", alive ? "ok" : "FAILED",
                 (unsigned)signal_engine_fpga_version());
        printf("[emi] recovery: %s\n", text);
        log_event_text(EMI_EV_RECOVERY, text);
        s.cap_issued = false;
        s.t_next_cap = now + 1000u;
        if (board_has_analog()) apply_dac(s.dac);   /* the reload stopped the outputs */
        break;
    }
    case RECOVERY_MCU_RESET:
        snprintf(text, sizeof(text), "STM32 reset (%lu of %u)", (unsigned long)s_keep.resets,   /* counted already */
                 (unsigned)RECOVERY_RESETS_MAX);
        printf("[emi] recovery: still failing after the iCE40 reloads: %s\n", text);
        log_event_text(EMI_EV_RECOVERY, text);
        log_flush();                     /* the record first: the reset drops the RAM queue */
        s_keep.reset_pending = 1;
        boot_guard_planned_reset(true);  /* not a crash: no safe mode for it */
        vTaskDelay(pdMS_TO_TICKS(50));   /* the console line out */
        NVIC_SystemReset();
        break;
    case RECOVERY_ETH_RESTART:
        snprintf(text, sizeof(text), "Eth restart, down %lus",  /* fits the 27 chars a record keeps */
                 (unsigned long)((now - s.link_down_since) / 1000u));
        printf("[emi] recovery: %s\n", text);
        log_event_text(EMI_EV_RECOVERY, text);
        net_eth_restart();               /* PHY reset + DHCP; harmless with the cable out */
        break;
    default:
        break;
    }
}

/* Every worker pass while the loop runs: how long captures and the link have been failing, and
   the next recovery step if one is due. */
static void recovery_poll_emi(uint32_t now) {
    if (!s.rcv_init) {
        keep_init();
        recovery_init(&s_rcv, now);
        s.rcv_init = true;
    }
    if (!s.running) return;
    static bool stuck_applied;
    if (s_keep.fault_stuck && !stuck_applied) { stuck_applied = true; ice40_hold_off_bus(); }
    if (s.cap_issued && now - s.cap_issued_at > EMI_CAP_HUNG_MS && !s.fail_since)
        s.fail_since = s.cap_issued_at ? s.cap_issued_at : 1u;   /* a capture that never ends */
    const uint32_t fail_ms = s.fail_since ? now - s.fail_since : 0u;
    /* The Ethernet step's clock: the link down, or up while frames keep failing to go out (a
       wedged MAC or PHY with the cable in: no link loss, so it needs its own trigger). */
    const bool eth_failing = (s.link_seen && !ns.link) ||
                             (ns.link && ns.err && now - ns.t_err < EMI_LED_FAIL_MS);
    if (eth_failing) { if (!s.link_down_since) s.link_down_since = now ? now : 1u; }
    else s.link_down_since = 0;
    const uint32_t down_ms = s.link_down_since ? now - s.link_down_since : 0u;
    const recovery_action_t a = recovery_poll(&s_rcv, now, fail_ms, down_ms, &s_keep.resets);
    if (a != RECOVERY_NONE) {
        recovery_done(&s_rcv, a, now, &s_keep.resets);
        recovery_take(a, now);
    } else if (recovery_exhausted(&s_rcv, now, fail_ms, s_keep.resets)) {
        s.gave_up = true;   /* red now, until the failure ends by itself */
        printf("[emi] recovery: limits reached, the LED is red while the failure lasts\n");
        log_event_text(EMI_EV_RECOVERY, "recovery limit reached");
    }
}

/* Every worker pass while the loop runs: an eFuse that tripped counts as a failure and gets its
   supply turned off and on again (EMI_EFUSE_RETRY_MS later), within EMI_EFUSE_RETRIES_PER_HOUR. */
static void power_poll_emi(uint32_t now) {
    if (!s.running) { s.power_fault = false; s.efuse_off[1] = s.efuse_off[2] = false; return; }
    bool any = false;
    for (int e = 1; e <= 2; e++) {
        bool en = false, fault = false;
        if (target_power_get_cached(e, &en, &fault) != 0) continue;
        if (fault) any = true;
        if (fault && !s.efuse_fault[e]) {
            s.errors++;
            s.last_err_ms = now;
            snprintf(s.last_err, sizeof(s.last_err), "eFuse %d tripped", e);
            uint32_t recent = 0;
            const uint32_t have = s.efuse_retry_n < EMI_EFUSE_RETRIES_PER_HOUR ? s.efuse_retry_n
                                                                               : EMI_EFUSE_RETRIES_PER_HOUR;
            for (uint32_t i = 0; i < have; i++)
                if (now - s.efuse_retry_ms[i] < 3600000u) recent++;
            char text[40];
            if (recent < EMI_EFUSE_RETRIES_PER_HOUR) {
                s.efuse_retry_ms[s.efuse_retry_n++ % EMI_EFUSE_RETRIES_PER_HOUR] = now;
                (void)target_power_enable(e, false);
                (void)target_power_schedule(e, true, EMI_EFUSE_RETRY_MS);
                snprintf(text, sizeof(text), "eFuse %d tripped: power retry", e);
            } else {
                /* Out of retries: leave that output off (it would keep tripping into the fault,
                   once per retry of the eFuse itself) and count it as failing until `emi stop`
                   or a power cycle: recovering (blue) at first, red after 60 s. */
                (void)target_power_enable(e, false);
                s.efuse_off[e] = true;
                snprintf(text, sizeof(text), "eFuse %d tripped: off (retry limit)", e);
            }
            printf("[emi] recovery: %s\n", text);
            log_event_text(EMI_EV_RECOVERY, text);
        }
        s.efuse_fault[e] = fault;
        if (s.efuse_off[e]) any = true;
    }
    s.power_fault = any;
}

uint32_t emi_test_run(void) { return log_ok() ? s_log.run : 0u; }

static void health_run_reset(void);   /* below, with the health state */

void emi_test_start(void) {
    s.booted = true;
    if (s.running) return;
    s.running    = true;
    health_run_reset();
    s.t_start    = HAL_GetTick();
    s.t_report   = s.t_start;
    s.t_next_cap = s.t_start;
    s.captures   = 0;
    s.errors     = 0;
    s.fail_since = 0;
    s.link_down_since = 0;
    s.last_err[0] = '\0';
    ns.err         = false;   /* the net task re-reads the link from here on */
    s.link_seen    = false;
    ns.had_link    = false;
    ns.link_losses = 0;
    choose_default_dac();
    printf("[emi] starting the EMI test activity\n");
    run("la_voltage", "{\"cmd\":\"la_voltage\",\"mv\":3300}");
    run("target_power 1", "{\"cmd\":\"target_power\",\"efuse\":1,\"state\":1}");
    run("target_power 2", "{\"cmd\":\"target_power\",\"efuse\":2,\"state\":1}");
    if (board_has_analog()) apply_dac(s.dac);
    apply_uart(s.uart_la);
    log_event(EMI_EV_STARTED, 0, 0);
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
    log_event(EMI_EV_STOPPED, 0, 0);
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
    emi_mon_t *const m = s_mon;
    const uint32_t gen = m ? m->gen : 0u;
    if (m && gen != ns.mon_sent && netif_is_up(eth) && netif_is_link_up(eth)) {
        ns.mon_sent = gen;
        send_monitor(eth);
    }
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

/* One BPMON frame: broadcast, so a laptop sees it without promiscuous mode; padded to the 60-byte
   minimum. */
static void send_monitor(struct netif *eth) {
    static const uint8_t k_bcast[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    emi_mon_t *const m = s_mon;
    const char *text = m->buf[m->cur];
    size_t len = strnlen(text, sizeof(m->buf[0]));
    size_t frame = 14u + len;
    if (frame < 60u) frame = 60u;
    struct pbuf *p = pbuf_alloc(PBUF_RAW, (u16_t)frame, PBUF_RAM);
    if (!p) return;
    uint8_t *d = (uint8_t *)p->payload;
    memset(d, 0, frame);
    memcpy(d, k_bcast, 6);
    memcpy(d + 6, eth->hwaddr, 6);
    d[12] = (uint8_t)(EMI_NET_ETHERTYPE >> 8);
    d[13] = (uint8_t)EMI_NET_ETHERTYPE;
    memcpy(d + 14, text, len);
    if (eth->linkoutput(eth, p) == ERR_OK) ns.mon_frames++;
    pbuf_free(p);
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

/* A reset other than power-on (watchdog, reset pin, software, low-power) is a restart the lab
   should see. A brown-out sets the same flag as power-on, so a supply dip that resets the pod
   comes back green; the LED going dark and the uptime show that one. */
static bool restarted(void) {
    const char *c = fault_last_reset_str();
    return c && strcmp(c, "power-on") != 0 && strcmp(c, "unknown") != 0;
}

static bool failing_now(uint32_t now) {
    if (s.fail_since) return true;   /* captures failing, or one hung: until one succeeds */
    if (s.br_silent_logged || s.br_radio_failing) return true;   /* the Wi-Fi bridge */
    if (s.br_restart_ms && now - s.br_restart_ms < EMI_LED_FAIL_MS) return true;
    if (s.c3_last_ms && now - s.c3_last_ms < EMI_LED_FAIL_MS) return true;   /* the C3 restarted */
    if (s.power_fault) return true;  /* an eFuse tripped (power_poll_emi) */
    if (s.errors && now - s.last_err_ms < EMI_LED_FAIL_MS) return true;
    if (ns.err && now - ns.t_err < EMI_LED_FAIL_MS) return true;   /* Ethernet */
    return false;
}

uint8_t emi_test_led_state(void) {
    if (!s_reset_checked) {
        s_reset_checked = true;
        if (restarted()) s_failed_before = true;
    }
    const uint32_t now = HAL_GetTick();
    if (s.running && (s.errors || ns.err || s.fail_since || s.power_fault || s.c3_restarts ||
                      s.br_restarts || s.br_silences || s.br_radio_failing))
        s_failed_before = true;
    /* Red only for a failure that fails criterion B: still failing EMI_LED_VERDICT_MS after it
       began, or the recovery is out of steps. Until then the pod is recovering (blue, fast). Not
       latched: a pod that comes back after a long burst test goes blue (the lab reads the LED
       60 s after each test). */
    const bool failing = s.running && failing_now(now);
    if (!failing) { s.failing_from = 0; s.gave_up = false; }
    else if (!s.failing_from) s.failing_from = now ? now : 1u;
    const bool too_long = failing && (s.gave_up || now - s.failing_from >= EMI_LED_VERDICT_MS);
    return status_led_emi_state(s.booted, s.running, failing, too_long, s_failed_before);
}

static const char *led_name(uint8_t st) {
    switch (st) {
    case STATUS_LED_EMI_WAITING:   return "green (starting)";
    case STATUS_LED_EMI_RUNNING:   return "green";
    case STATUS_LED_EMI_FAILING:   return "red";
    case STATUS_LED_EMI_RECOVERING: return "blue (recovering)";
    case STATUS_LED_EMI_RECOVERED: return "blue";
    default:                       return "off";
    }
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
    char c3[24] = "";
    if (esp_wifi_ctrl_module_present())
        snprintf(c3, sizeof(c3), ", c3 restarts %lu", (unsigned long)s.c3_restarts);
    int len = snprintf(buf, cap,
                       "emi: %s, led %s, dac %s, uart %s, net %s, core %lu MHz, %lus, captures %lu, "
                       "errors %lu%s%s%s",
                       s.running ? "running" : (s.booted ? "stopped" : "waiting for boot start"),
                       led_name(emi_test_led_state()), k_dac_names[s.dac], uart, net, (unsigned long)(SystemCoreClock / 1000000u),
                       (unsigned long)up, (unsigned long)s.captures, (unsigned long)s.errors, c3,
                       s.last_err[0] ? ", last: " : "", s.last_err);
    if (len < 0) return 0;
    return (size_t)len < cap ? (size_t)len : cap - 1;
}

size_t emi_test_json_short(char *buf, size_t cap) {
    int len = snprintf(buf, cap,
                       "{\"running\":%s,\"led\":\"%s\",\"failed_since_power_on\":%s,"
                       "\"captures\":%lu,\"errors\":%lu,\"net_link\":%s,\"run\":%lu}",
                       s.running ? "true" : "false", led_name(emi_test_led_state()),
                       s_failed_before ? "true" : "false", (unsigned long)s.captures,
                       (unsigned long)s.errors, ns.link ? "true" : "false",
                       (unsigned long)emi_test_run());
    if (len < 0 || (size_t)len >= cap) { if (cap) buf[0] = '\0'; return 0; }
    return (size_t)len;
}

size_t emi_test_json(char *buf, size_t cap) {
    const uint32_t up = s.running ? (HAL_GetTick() - s.t_start) / 1000u : 0u;
    int len = snprintf(buf, cap,
                       "{\"running\":%s,\"dac\":\"%s\",\"uart_la\":%d,\"core_mhz\":%lu,\"uptime_s\":%lu,"
                       "\"captures\":%lu,\"errors\":%lu,\"net_mbit\":%lu,"
                       "\"net_sent\":%lu,\"net_drops\":%lu,\"net_link\":%s,\"net_link_losses\":%lu,"
                       "\"led\":\"%s\",\"failed_since_power_on\":%s,\"reset_cause\":\"%s\","
                       "\"c3_restarts\":%lu}",
                       s.running ? "true" : "false", k_dac_names[s.dac], s.uart_la,
                       (unsigned long)(SystemCoreClock / 1000000u), (unsigned long)up,
                       (unsigned long)s.captures, (unsigned long)s.errors, (unsigned long)ns.mbit,
                       (unsigned long)ns.sent, (unsigned long)ns.drops,
                       ns.link ? "true" : "false", (unsigned long)ns.link_losses,
                       led_name(emi_test_led_state()), s_failed_before ? "true" : "false",
                       fault_last_reset_str() ? fault_last_reset_str() : "unknown",
                       (unsigned long)s.c3_restarts);
    if (len < 0 || (size_t)len >= cap) { if (cap) buf[0] = '\0'; return 0; }
    return (size_t)len;
}

static void publish_monitor(uint32_t now) {
    if (now - s.t_report < EMI_TEST_REPORT_MS) return;
    s.t_report = now;
    char line[320];
    emi_test_describe(line, sizeof(line));
    if (s.running) printf("[emi] %s\n", line);
    if (!s_mon) {
        emi_mon_t *m = pvPortMalloc(sizeof(*m));
        if (!m) return;
        memset(m, 0, sizeof(*m));
        s_mon = m;
    }
    emi_mon_t *const mon = s_mon;
    const uint8_t next = mon->cur ^ 1u;
    const char *reset = fault_last_reset_str();
    snprintf(mon->buf[next], sizeof(mon->buf[next]), "BPMON run %lu, reset %s, uptime %lu ms, seq %lu, %s",
             (unsigned long)emi_test_run(), reset ? reset : "unknown", (unsigned long)now,
             (unsigned long)ns.seq, line);
    __asm volatile ("dmb" ::: "memory");
    mon->cur = next;
    mon->gen++;
}

/* The pod restarts its ESP32-C3 when the C3 stops answering, reboots by itself or fails a setup
   step (esp_wifi_ctrl.c, c3_restarts.h). While the activity runs, each one is a failure the lab
   should see: red for EMI_LED_FAIL_MS, then blue, and an event in the run log. An access point
   that refuses the association is the network's doing, not the pod's, so those restarts are
   left out (a pod set up for a Wi-Fi network that is not there would be red all day). */
static void c3_poll_emi(uint32_t now) {
    if (!esp_wifi_ctrl_module_present()) return;
    const c3_restarts_t *r = esp_wifi_ctrl_restarts();
    const uint32_t total = r->total, assoc = r->count[C3_RESTART_ASSOC];
    if (!s.c3_init || !s.running) {   /* restarts before the start (or after a stop): baseline */
        s.c3_init = true;
        s.c3_seen = total;
        s.c3_seen_assoc = assoc;
        return;
    }
    if (total == s.c3_seen) return;
    const uint32_t real = (total - s.c3_seen) - (assoc - s.c3_seen_assoc);
    s.c3_seen = total;
    s.c3_seen_assoc = assoc;
    if (!real) return;
    s.c3_restarts += real;
    s.c3_last_ms = now ? now : 1u;
    char reason[C3_RESTART_REASON_LEN];
    memcpy(reason, r->last_reason, sizeof(reason));   /* the net task writes it: take a copy */
    reason[sizeof(reason) - 1] = '\0';
    snprintf(s.last_err, sizeof(s.last_err), "ESP32-C3 restart: %s", reason);
    printf("[emi] ESP32-C3 restarted (%lu this run): %s\n", (unsigned long)s.c3_restarts, reason);
    char text[28];
    snprintf(text, sizeof(text), "C3 restart: %s",
             c3_restart_cause_name((c3_restart_cause_t)r->last_cause));
    log_event_text(EMI_EV_C3_RESTART, text);
}

/* Health: things the pod counts that the LED does not show, each an EMI_EV_HEALTH event in the
   run log when it changes, at most one per item every EMI_HEALTH_GAP_MS (the count in between is
   summed into the next). Read once a second. */
#define EMI_HEALTH_GAP_MS 10000u
/* The cloud link is not under test (a lab network may have no internet at all), so it never
   touches the LED, and a drop that reconnects is only counted (SUMMARY). Down for
   EMI_CLOUD_DOWN_MS is an event, and so is its coming back: the shared lab view stops updating. */
#define EMI_CLOUD_DOWN_MS (2u * 60u * 1000u)
enum { H_I2C, H_PS_RETRY, H_PS_FAIL, H_CAN, H_REV1, H_REV2, H_CDONE, H_VCCIO, H_N };
static struct {
    bool     init;
    uint32_t t_read;
    uint32_t last[H_N];       /* the value at the last event (or the start) */
    uint32_t t_event[H_N];
    uint32_t cdone_lost;      /* CDONE went low this run */
    uint32_t cloud_down_from; /* set up but not connected since (0 = connected or not set up) */
    bool     cloud_down_logged;
    uint32_t cloud_down_ms;   /* this run, in total */
    uint32_t t_cloud;         /* the last cloud_poll */
    uint32_t t_summary;       /* the last SUMMARY (or the start) */
} hs;

static uint32_t health_value(int i) {
    switch (i) {
    case H_I2C:      return i2c_bus_clear_count();
    case H_PS_RETRY: { uint32_t r = 0, f = 0; int rc = 0; signal_engine_psram_read_stats(&r, &f, &rc); return r; }
    case H_PS_FAIL:  { uint32_t r = 0, f = 0; int rc = 0; signal_engine_psram_read_stats(&r, &f, &rc); return f; }
    case H_CAN:      { can_status_t st; return can_get_status(&st) == 0 ? st.bus_off_recoveries : 0u; }
    case H_REV1:     return target_power_reverse_blocks(1);
    case H_REV2:     return target_power_reverse_blocks(2);
    case H_CDONE:    return ice40_is_configured() ? 1u : 0u;
    case H_VCCIO:    return (uint32_t)la_vccio_status_pin();
    default:         return 0u;
    }
}

static void health_poll(uint32_t now) {
    if (now - hs.t_read < 1000u && hs.init) return;
    hs.t_read = now;
    if (!hs.init) {
        for (int i = 0; i < H_N; i++) hs.last[i] = health_value(i);
        hs.init = true;
        return;
    }
    for (int i = 0; i < H_N; i++) {
        const uint32_t v = health_value(i);
        /* Reverse blocking is harmless and frequent with a Wi-Fi bridge on the pod's 5 V: a minute
           between its events, so it does not crowd the run log. */
        const uint32_t gap = (i == H_REV1 || i == H_REV2) ? 6u * EMI_HEALTH_GAP_MS : EMI_HEALTH_GAP_MS;
        if (v == hs.last[i] || now - hs.t_event[i] < gap) continue;
        char text[64];   /* the console line gets it all; the run log keeps the first 27 */
        const uint32_t d = v - hs.last[i];
        switch (i) {
        case H_I2C:      snprintf(text, sizeof(text), "I2C bus clear +%lu", (unsigned long)d); break;
        case H_PS_RETRY: snprintf(text, sizeof(text), "PSRAM read retry +%lu", (unsigned long)d); break;
        case H_PS_FAIL:  snprintf(text, sizeof(text), "PSRAM read fail +%lu", (unsigned long)d); break;
        case H_CAN:      snprintf(text, sizeof(text), "CAN bus-off +%lu", (unsigned long)d); break;
        case H_REV1:     snprintf(text, sizeof(text), "eFuse1 rev block +%lu", (unsigned long)d); break;
        case H_REV2:     snprintf(text, sizeof(text), "eFuse2 rev block +%lu", (unsigned long)d); break;
        case H_CDONE:
            if (!v) hs.cdone_lost++;
            snprintf(text, sizeof(text), v ? "iCE40 configured" : "iCE40 config lost");
            break;
        case H_VCCIO:    snprintf(text, sizeof(text), "LA supply ST -> %lu", (unsigned long)v); break;
        default:         text[0] = '\0'; break;
        }
        printf("[emi] health: %s\n", text);
        log_event_text(EMI_EV_HEALTH, text);
        hs.last[i] = v;
        hs.t_event[i] = now;
    }
}

/* The Wi-Fi bridge in the lab's configuration 2 (the pod's frames on the air from the bridge's EMC
   test build): part of the equipment under test, so once a bridge was seen this run, its failures
   are the pod's: a restart (with the reason from its next STATUS), silence for EMI_BRIDGE_SILENT_MS,
   and its radio reporting it is not transmitting. Each is a health event and counts for the LED.
   Without a bridge this does nothing. */
#define EMI_BRIDGE_SILENT_MS 10000u
static void bridge_poll_emi(uint32_t now) {
#if defined(BENCHPOD_WIFI_BRIDGE)
    static wb_info_t bi;   /* static: the hw worker's stack is tight */
    wifi_bridge_get_info(&bi);
    char text[64];
    if (!s.br_seen) {
        if (!bi.present) return;
        s.br_seen = true;
        s.br_reboots_seen = bi.reboots;
        printf("[emi] Wi-Fi bridge %02x:%02x:%02x:%02x:%02x:%02x fw %s: watched (configuration 2)\n",
               bi.mac[0], bi.mac[1], bi.mac[2], bi.mac[3], bi.mac[4], bi.mac[5], bi.fw_version);
        return;
    }
    if (!bi.present) {
        if (!s.br_silent_from) s.br_silent_from = now ? now : 1u;
        if (!s.br_silent_logged && now - s.br_silent_from >= EMI_BRIDGE_SILENT_MS) {
            s.br_silent_logged = true;
            s.br_silences++;
            s.last_err_ms = now;
            snprintf(s.last_err, sizeof(s.last_err), "Wi-Fi bridge silent");
            printf("[emi] health: Wi-Fi bridge silent for %lu s\n", (unsigned long)(EMI_BRIDGE_SILENT_MS / 1000u));
            log_event_text(EMI_EV_HEALTH, "bridge silent 10 s");
        }
        return;
    }
    if (s.br_silent_logged) {
        snprintf(text, sizeof(text), "bridge back after %lu s", (unsigned long)((now - s.br_silent_from) / 1000u));
        printf("[emi] health: %s\n", text);
        log_event_text(EMI_EV_HEALTH, text);
    }
    s.br_silent_from = 0;
    s.br_silent_logged = false;
    if (bi.reboots != s.br_reboots_seen) {
        s.br_restarts += bi.reboots - s.br_reboots_seen;
        s.br_reboots_seen = bi.reboots;
        s.br_restart_ms = now ? now : 1u;
        s.br_restart_pending = now ? now : 1u;
        s.last_err_ms = now;
        snprintf(s.last_err, sizeof(s.last_err), "Wi-Fi bridge restarted");
    }
    /* Its reason comes with the next STATUS after the new boot (the pod asks at once); give up on
       it after 10 s. */
    if (s.br_restart_pending && ((bi.have_status && bi.status_age_ms < now - s.br_restart_pending) ||
                                 now - s.br_restart_pending >= 10000u)) {
        const bool fresh = bi.have_status && bi.status_age_ms < now - s.br_restart_pending;
        snprintf(text, sizeof(text), "bridge restart: %s", fresh ? wb_reset_str(bi.reset) : "unknown");
        printf("[emi] health: Wi-Fi %s\n", text);
        log_event_text(EMI_EV_HEALTH, text);
        s.br_restart_pending = 0;
    }
    const bool radio_failing = bi.have_status && bi.emc == BWP_EMC_FAILING;
    if (radio_failing != s.br_radio_failing) {
        s.br_radio_failing = radio_failing;
        snprintf(text, sizeof(text), radio_failing ? "bridge radio not transmitting" : "bridge radio back");
        printf("[emi] health: Wi-Fi %s\n", text);
        log_event_text(EMI_EV_HEALTH, text);
        if (radio_failing) {
            s.last_err_ms = now;
            snprintf(s.last_err, sizeof(s.last_err), "Wi-Fi bridge radio not transmitting");
        }
    }
#else
    (void)now;
#endif
}

static void health_run_reset(void) {
    hs.cdone_lost = 0;
    hs.cloud_down_from = 0;
    hs.cloud_down_logged = false;
    hs.cloud_down_ms = 0;
    hs.t_cloud = 0;
    hs.t_summary = 0;
    s.br_seen = false;
    s.br_restarts = 0;
    s.br_restart_ms = 0;
    s.br_restart_pending = 0;
    s.br_silent_from = 0;
    s.br_silent_logged = false;
    s.br_silences = 0;
    s.br_radio_failing = false;
}

static void cloud_poll(uint32_t now) {
    const char *st = cloud_client_state_str();
    const bool set_up = strcmp(st, "disabled") != 0;
    const bool up = strcmp(st, "connected") == 0;
    const uint32_t dt = hs.t_cloud ? now - hs.t_cloud : 0u;
    hs.t_cloud = now;
    if (up || !set_up) {
        if (hs.cloud_down_logged) {
            char text[48];
            snprintf(text, sizeof(text), "cloud back after %lu min",
                     (unsigned long)((now - hs.cloud_down_from + 30000u) / 60000u));
            printf("[emi] health: %s\n", text);
            log_event_text(EMI_EV_HEALTH, text);
        }
        hs.cloud_down_from = 0;
        hs.cloud_down_logged = false;
        return;
    }
    hs.cloud_down_ms += dt;
    if (!hs.cloud_down_from) hs.cloud_down_from = now ? now : 1u;
    if (!hs.cloud_down_logged && now - hs.cloud_down_from >= EMI_CLOUD_DOWN_MS) {
        char err[48], text[80];
        cloud_client_last_error(err, sizeof(err));
        const char *why = err;
        if (!err[0]) (void)cloud_client_drops(&why);
        snprintf(text, sizeof(text), "cloud down 2 min%s%s", why && why[0] ? ": " : "", why ? why : "");
        printf("[emi] health: %s\n", text);
        log_event_text(EMI_EV_HEALTH, text);
        hs.cloud_down_logged = true;
    }
}

/* Every EMI_LOG_SUMMARY_MS while the loop runs: every total, changed or not. */
static void summary_poll(uint32_t now) {
    if (!hs.t_summary) { hs.t_summary = now ? now : 1u; return; }
    if (now - hs.t_summary < EMI_LOG_SUMMARY_MS) return;
    hs.t_summary = now;
    emi_rec_t r;
    memset(&r, 0, sizeof(r));
    r.type = EMI_REC_SUMMARY;
    r.led = emi_test_led_state();
    uint32_t retry = 0, fail = 0; int rc = 0;
    signal_engine_psram_read_stats(&retry, &fail, &rc);
    can_status_t cs;
    const uint32_t busoff = can_get_status(&cs) == 0 ? cs.bus_off_recoveries : 0u;
    static eth_diag_t d;   /* static: about 100 bytes, the hw worker's stack is tight */
    net_eth_diag(&d);
    uint32_t bridge_err = 0;
#if defined(BENCHPOD_WIFI_BRIDGE)
    if (wifi_bridge_in_use()) { static wb_info_t bi; wifi_bridge_get_info(&bi); bridge_err = bi.counters.wifi_tx_err; }
#endif
#define SAT16(x) ((uint16_t)((x) > 0xFFFFu ? 0xFFFFu : (x)))
    r.u.sum.cloud_drops = SAT16(cloud_client_drops(NULL));
    r.u.sum.c3_restarts = SAT16(esp_wifi_ctrl_restarts()->total);
    r.u.sum.i2c_clears  = SAT16(i2c_bus_clear_count());
    r.u.sum.can_busoff  = SAT16(busoff);
    r.u.sum.psram_fail  = SAT16(fail);
    r.u.sum.cdone_lost  = SAT16(hs.cdone_lost);
#undef SAT16
    r.u.sum.psram_retry = retry;
    r.u.sum.rev_block[0] = target_power_reverse_blocks(1);
    r.u.sum.rev_block[1] = target_power_reverse_blocks(2);
    r.u.sum.eth_crc = d.rx_crc + d.rx_align;
    r.u.sum.eth_missed = d.rx_missed;
    r.u.sum.bridge_tx_err = bridge_err;
    r.u.sum.cloud_down_s = hs.cloud_down_ms / 1000u;
    r.u.sum.bridge_restarts = (uint16_t)(s.br_restarts > 0xFFFFu ? 0xFFFFu : s.br_restarts);
    r.u.sum.bridge_silences = (uint16_t)(s.br_silences > 0xFFFFu ? 0xFFFFu : s.br_silences);
    log_queue(&r);
    char line[320];
    emi_log_format(&r, line, sizeof(line));
    printf("[emi] %s\n", line);
}

void emi_test_poll(void) {
    const uint32_t now = HAL_GetTick();
    s.budget = EMI_SINK_BUDGET;   /* the read-back's share of this worker pass */
    if (!lg.tried && !s.running) {   /* the run log, before the activity starts */
        s_quick_start = emi_recovery_reset_pending();   /* log_boot clears the flag */
        log_boot();
        s.last_led = emi_test_led_state();
        s.t_checkpoint = now;
        s.cp_last = cp_sig();
    }
    if (!s.booted) {
        if (now >= (s_quick_start ? EMI_TEST_RECOVERY_START_MS : EMI_TEST_START_DELAY_MS)) emi_test_start();
        publish_monitor(now);
        return;
    }
    if (s.cap_issued && !capture_busy() && heavy_owner_conn() != CH_EMI_CONN) {
        s.cap_issued = false;
        if (s.errors == s.cap_errors) { s.captures++; s.fail_since = 0; }   /* else its read-back failed */
        else if (!s.fail_since) s.fail_since = now ? now : 1u;
        s.t_next_cap = now + EMI_CAPTURE_GAP_MS;
    }
    c3_poll_emi(now);
    if (s.running) {
        health_poll(now);
        if (now - hs.t_cloud >= 1000u || !hs.t_cloud) { cloud_poll(now); bridge_poll_emi(now); }
        summary_poll(now);
    }
    log_poll(now);
    recovery_poll_emi(now);
    power_poll_emi(now);
    publish_monitor(now);
    if (!s.running) return;
    if (!s.cap_issued && (int32_t)(now - s.t_next_cap) >= 0 && (int32_t)(now - s.hold_until) >= 0) {
        s.cap_errors = s.errors;
        if (!board_has_analog()) s.next_kind = 1;   /* LA only: capture_dual takes the ADC too */
        run(s.next_kind ? "la_capture" : "capture_dual", k_captures[s.next_kind]);
        s.next_kind = (s.next_kind + 1u) % (sizeof(k_captures) / sizeof(k_captures[0]));
        if (s.errors == s.cap_errors) { s.cap_issued = true; s.cap_issued_at = now; }
        else {
            s.t_next_cap = now + 1000u;   /* refused: the other kind next */
            if (!s.fail_since) s.fail_since = now ? now : 1u;
        }
    }
}
