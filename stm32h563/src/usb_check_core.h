#ifndef USB_CHECK_CORE_H
#define USB_CHECK_CORE_H

/* usb_check_core — the logic of the usb_check command (usb_check.h), pure: no hardware, no RTOS.
 *
 * The pod hosts the board under test on its USB port and powers it through target-power eFuse 1,
 * so it can measure what a computer cannot: how long the board takes to enumerate after power-on,
 * the current it draws against its declared bMaxPower and in suspend, whether it survives bus
 * resets and short VBUS dropouts. This file holds the argument parser, the statistics, the
 * pass/fail decisions, the job state machine (one step per call, driven through uc_ops_t) and the
 * reply. Host tests (test_usb_check.c) run it against a simulated board with a fake clock. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    UC_ENUMERATE = 0, UC_POWER, UC_SUSPEND, UC_RESET, UC_REPLUG,
    UC_TESTS
} uc_test_t;

const char *uc_test_name(uc_test_t t);
/* "enumerate" ... -> the test; -1 otherwise. */
int uc_test_from_name(const char *name);

/* ---- limits and defaults (the documented ranges) ---- */
#define UC_COUNT_MAX            200u
#define UC_ENUM_COUNT_DEFAULT   10u
#define UC_RESET_COUNT_DEFAULT  20u
#define UC_REPLUG_COUNT_DEFAULT 10u
#define UC_REPLUG_COUNT_MAX     100u
#define UC_OFF_MS_DEFAULT       500u
#define UC_OFF_MS_MIN           50u      /* enumerate */
#define UC_OFF_MS_MAX           10000u
#define UC_DROP_MS_MIN          1u       /* replug lengths */
#define UC_DROP_MS_MAX          5000u
#define UC_REPLUG_LENGTHS_MAX   6u
#define UC_TIMEOUT_DEFAULT_MS   5000u
#define UC_TIMEOUT_MIN_MS       500u
#define UC_TIMEOUT_MAX_MS       30000u
#define UC_POWER_MS_DEFAULT     3000u
#define UC_SUSPEND_MS_DEFAULT   1000u
#define UC_DURATION_MIN_MS      200u
#define UC_DURATION_MAX_MS      60000u
#define UC_SETTLE_DEFAULT_MS    200u
#define UC_SETTLE_MIN_MS        20u
#define UC_SETTLE_MAX_MS        5000u

/* A suspended bus-powered device may draw at most 2.5 mA (USB 2.0, 7.2.3). */
#define UC_SUSPEND_LIMIT_UA     2500
/* Spikes up to 25 % over bMaxPower are accepted (the rail's monitor reads about once per
   millisecond, and the spec allows a brief inrush); the average must stay within it. */
#define UC_PEAK_ALLOW_NUM       5
#define UC_PEAK_ALLOW_DEN       4
/* After power returns, a device that never dropped off the bus within this time stayed. */
#define UC_STAY_MS              400u
/* A disconnect that has not shown after the rail was off this much longer than asked: the board
   is not powered from eFuse 1. */
#define UC_DROP_WAIT_MS         2500u
#define UC_GAP_MS               100u
#define UC_BUS_WAIT_MS          2000u
#define UC_RECOVER_MS           5000u

typedef struct {
    uc_test_t test;
    uint32_t  count;
    uint32_t  off_ms;                       /* enumerate */
    uint32_t  timeout_ms;
    uint32_t  duration_ms;                  /* power, suspend */
    uint32_t  settle_ms;                    /* suspend */
    uint32_t  drops[UC_REPLUG_LENGTHS_MAX]; /* replug */
    uint8_t   n_drops;
} uc_args_t;

/* The command's JSON into `a`. 0 = ok; -1 = refused, `why` says what (a sentence for BP0087). */
int uc_parse_args(const char *json, uc_args_t *a, char *why, size_t cap);

/* ---- statistics ---- */
typedef struct { uint32_t n, min, max; uint64_t sum; } uc_stat_t;
void     uc_stat_add(uc_stat_t *s, uint32_t v);
uint32_t uc_stat_avg(const uc_stat_t *s);   /* rounded; 0 when empty */

/* ---- decisions ---- */
typedef enum {
    UC_V_OK = 0, UC_V_AVG_OVER, UC_V_PEAK_OVER, UC_V_NO_LIMIT, UC_V_SKIPPED, UC_V_NOT_MEASURED
} uc_verdict_t;
const char *uc_verdict_name(uc_verdict_t v);
/* The current in the configured state against the configuration's bMaxPower (mA; 0 = none). */
uc_verdict_t uc_power_verdict(int32_t avg_ua, int32_t peak_ua, uint16_t max_power_ma);
/* The suspended current against 2.5 mA; a self-powered device is not held to it (SKIPPED). */
uc_verdict_t uc_suspend_verdict(int32_t avg_ua, bool self_powered);

/* ---- what the job needs from the pod ---- */
typedef struct {
    bool     hosting;          /* USB host mode is on */
    bool     connected;        /* a pull-up is on the port */
    bool     attached;         /* a device is mounted */
    uint8_t  enumeration;      /* usb_host_enum_t: 0 none, 1 running, 2 done, 3 stuck */
    uint16_t enumerations;     /* mounts since host mode started */
    uint16_t descriptors;      /* device descriptors read since host mode started */
    uint16_t vid, pid;
    bool     cfg_valid;
    uint16_t max_power_ma;
    uint8_t  cfg_attrs;        /* bmAttributes: bit 6 = self-powered */
} uc_host_t;

enum { UC_ENUM_STUCK = 3 };
enum { UC_BUS_SUSPEND = 1, UC_BUS_RESUME, UC_BUS_RESET };
enum { UC_BUS_IDLE = 0, UC_BUS_BUSY, UC_BUS_OK, UC_BUS_FAIL };

typedef struct {
    bool susprdy;       /* the peripheral reported the suspend state */
    bool sof_stopped;   /* the frame counter stood still after it */
    bool sof_resumed;   /* it ran again after the resume */
} uc_bus_info_t;

typedef struct {
    void *ctx;
    uint32_t (*now_ms)(void *ctx);
    void (*host)(void *ctx, uc_host_t *out);
    bool (*efuse_on)(void *ctx);
    int  (*efuse_set)(void *ctx, bool on);                    /* 0 = ok */
    int  (*current_ua)(void *ctx, int32_t *ua);               /* eFuse 1's monitor; 0 = ok */
    int  (*bus_request)(void *ctx, int op);                   /* UC_BUS_*; 0 = accepted */
    int  (*bus_state)(void *ctx, uc_bus_info_t *info);        /* UC_BUS_IDLE..FAIL */
    int  (*get_status)(void *ctx);                            /* GET_STATUS to the device; 0 = answered */
    /* The role switch usb_host stop and start do, to bring a dead port back (NULL: none). */
    void (*host_stop)(void *ctx);
    int  (*host_start)(void *ctx);                            /* 0 = asked for */
} uc_ops_t;

/* ---- the result ---- */
typedef struct {
    bool        pass;
    const char *error;          /* "" or a short code: aborted, host_stopped, not_powered_by_efuse1,
                                   no_device, monitor_error, efuse_error, bus_busy, bus_timeout */
    uint32_t    ms;
    uint16_t    vid, pid;
    /* enumerate, reset */
    uint32_t    cycles, ok, no_connect, stuck, timeout, identity_changed;
    uc_stat_t   connect, descriptor, mount;   /* reset: mount = bus reset to mounted again */
    /* power, suspend */
    uint32_t    samples;
    int64_t     sum_ua;
    int32_t     min_ua, peak_ua;
    uint16_t    limit_ma;
    bool        self_powered;
    uint8_t     verdict;                      /* uc_verdict_t */
    /* suspend */
    bool        suspend_ok, resume_ok, answers, recovered;
    bool        port_reinit, reinit_ok;       /* the port was re-initialized (host mode stopped and started); it enumerated again */
    bool        susprdy, sof_stopped, sof_resumed;
    /* replug */
    uint8_t     nlen;
    struct { uint32_t off_ms, stayed, reenumerated, failed; } len[UC_REPLUG_LENGTHS_MAX];
} uc_result_t;

/* The short codes of uc_result_t.error. The step (in the W25Q image) names an error by its code and
   gets the string from uc_error_str, internal flash: the result outlives the session, so it must not
   point into the image. */
typedef enum {
    UC_E_NONE = 0, UC_E_INTERNAL, UC_E_EFUSE, UC_E_NO_DEVICE, UC_E_MONITOR, UC_E_BUS_BUSY,
    UC_E_BUS_TIMEOUT, UC_E_HOST_STOPPED, UC_E_ABORTED, UC_E_NOT_POWERED
} uc_err_t;
const char *uc_error_str(uc_err_t e);

/* The job's phases (j->phase): uc_job_phase names them, the step in the W25Q image (xip_mod_usb_check.c) runs them. */
enum {
    PH_START = 0, PH_OFF, PH_OFF_WAIT, PH_ON_WAIT, PH_GAP, PH_RESET_WAIT, PH_SAMPLE, PH_SUSPEND,
    PH_SETTLE, PH_MEASURE, PH_RESUME, PH_CHECK, PH_RECOVER,
    PH_CLEAN, PH_REINIT_STOP, PH_REINIT_OFF, PH_REINIT_ON,   /* every exit of a test that touched the bus */
    PH_DONE
};

typedef struct {
    uc_args_t   a;
    uc_ops_t    ops;
    uc_result_t r;
    uint8_t     phase;
    bool        running, abort_asked, suspended, efuse_was_on;
    uint32_t    t0, tp;                       /* job start, phase start */
    uint32_t    i, k, done, total;
    uint16_t    enum0, desc0;
    uint16_t    vid0, pid0;
    uint32_t    t_conn, t_desc;
    bool        got_conn, got_desc, dropped;
    uint8_t     bus_op;
    uint8_t     clean_stage;
    uint8_t     read_errors;
} uc_job_t;

/* Set up a job; the first uc_job_step starts it. `ops->efuse_on` is read now: the state to
   put back at the end. */
void uc_job_init(uc_job_t *j, const uc_args_t *a, const uc_ops_t *ops);
/* One step: 1 = still running, 0 = finished (the result is in j->r). Code of the usb image in the
   W25Q (xip_mod_usb_check.c): usb_check.c calls it through the XIP session's table, never directly. */
int  uc_job_step(uc_job_t *j);
/* Stop at the next step (the rail and the bus are put back). */
void uc_job_abort(uc_job_t *j);
const char *uc_job_phase(const uc_job_t *j);

/* {"running":..,"have_result":true,"test":..,"phase":..,"done":N,"total":N,"ms":N[,"pass":..,
   "error":..,"result":{..}]} into `out`. Returns the length, 0 when it does not fit. */
size_t uc_reply_json(const uc_job_t *j, char *out, size_t cap);

#endif /* USB_CHECK_CORE_H */
