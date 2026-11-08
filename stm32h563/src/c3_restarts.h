/*
 * c3_restarts.h — why and how often the pod restarts its ESP32-C3 (the built-in Wi-Fi).
 *
 * esp_wifi_ctrl.c restarts the C3 from reset whenever its link fails: the C3 stops answering, it
 * rebooted by itself, it never announced a boot, a setup step failed or timed out, the access
 * point refused the association, or its automatic flash failed. Each restart is counted here by
 * cause, with the reason and the uptime of the last one.
 *
 * The counts cover everything since power-on: they live in .noinit RAM, so a software reset, a
 * watchdog reset or an OTA update keeps them, and a power-on or the reset pin starts them at 0.
 * That makes a pod that restarts its C3 a few times an hour (or after every ESD hit, then
 * crashes) visible in its status, the console, the cloud capabilities it sends on every connect,
 * and the EMI build's monitor and run log.
 *
 * The record and its rules are here, over a caller-owned struct, so they run in the host tests.
 */
#ifndef C3_RESTARTS_H
#define C3_RESTARTS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum {
    C3_RESTART_NO_RESPONSE = 0,   /* it stopped answering while the link was up (RSSI requests) */
    C3_RESTART_REBOOTED,          /* it announced a new boot in the middle of the session */
    C3_RESTART_NO_BOOT,           /* it never announced a boot after its reset was released */
    C3_RESTART_SETUP,             /* a setup step (init, storage, mode, config, start, connect, MAC)
                                     failed or timed out, or a request could not be sent */
    C3_RESTART_ASSOC,             /* the access point did not take the association */
    C3_RESTART_FLASH,             /* its automatic flash failed or could not be queued */
    C3_RESTART_CAUSES
} c3_restart_cause_t;

#define C3_RESTART_REASON_LEN 40u

typedef struct {
    uint32_t magic;
    uint32_t count[C3_RESTART_CAUSES];
    uint32_t total;
    uint32_t boots;                   /* boots since power-on that kept these counts (1 = this one) */
    uint32_t last_uptime_s;           /* uptime of the boot that did the last restart */
    uint32_t last_boot;               /* the value of `boots` at the last restart */
    uint8_t  last_cause;              /* c3_restart_cause_t; meaningless while total == 0 */
    char     last_reason[C3_RESTART_REASON_LEN];
    uint32_t check;                   /* c3_restarts_check() over everything above */
} c3_restarts_t;

/* At boot, before the first restart: a power-on (or a record that does not check out) starts
   the counts over; any other reset keeps them and counts one more boot. */
void c3_restarts_boot(c3_restarts_t *r, bool power_on);

/* One restart: cause and the controller's reason text (cut to fit), at `uptime_s`. */
void c3_restarts_note(c3_restarts_t *r, c3_restart_cause_t cause, const char *why, uint32_t uptime_s);

/* "no_response", "rebooted", ...: the JSON and console names. */
const char *c3_restart_cause_name(c3_restart_cause_t cause);

/* {"total":N,"no_response":N,...,"last":"<reason>","last_cause":"<name>","last_uptime_s":N,
    "last_this_boot":true,"boots":N} into buf; returns the length, 0 if it did not fit. */
size_t c3_restarts_json(const c3_restarts_t *r, char *buf, size_t cap);

/* "C3 restarts since power-on: 3 (no response 1, rebooted 2, ...), last: <reason> ..." */
size_t c3_restarts_describe(const c3_restarts_t *r, char *buf, size_t cap);

uint32_t c3_restarts_check(const c3_restarts_t *r);

#endif
