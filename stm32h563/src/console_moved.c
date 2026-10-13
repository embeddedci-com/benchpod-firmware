/*
 * console_moved.c — the console commands firmware 3.9.0 moved to the JSON API, and the line the
 * console prints for them. See console_moved.h.
 */
#include "console_moved.h"

#include <stdio.h>
#include <string.h>

static const struct { const char *verb, *json; } k_moved[] = {
    { "la-voltage",  "la_voltage" },
    { "power",       "target_power" },
    { "pstat",       "target_status" },
    { "nrst",        "nrst" },
    { "usb-cc",      "usb_cc" },
    { "dac",         "dac_out" },
    { "path",        "analog_path" },
    { "adc",         "adc_read" },
    { "measure",     "adc_read" },
    { "current-out", "current_out" },
    { "calibrate",   "calibrate" },
    { "dac-limits",  "dac_limits" },
    { "dacraw",      "dac_set" },
    { "dacmux",      "dac_mux" },
    { "calsw",       "cal_switch" },
    { "can",         "can_*" },
    { "upload-begin",  "ota_begin" },
    { "upload-data",   "ota_data" },
    { "upload-end",    "ota_end" },
    { "upload-commit", "ota_commit" },
    { "upload-status", "ota_status" },
    { "upload-abort",  "ota_abort" },
    { "upload-sig",    "ota_begin sig" },
};

const char *console_moved_to(const char *verb)
{
    if (!verb) return NULL;
    for (size_t i = 0; i < sizeof(k_moved) / sizeof(k_moved[0]); i++)
        if (!strcmp(verb, k_moved[i].verb)) return k_moved[i].json;
    return NULL;
}

size_t console_moved_line(const char *verb, char *out, size_t cap)
{
    const char *json = console_moved_to(verb);
    if (!out || cap == 0) return 0;
    out[0] = '\0';
    if (!json) return 0;
    int n = snprintf(out, cap, "%s: moved to the network API (JSON %s) in firmware 3.9.0; update the "
                     "benchpod CLI to 0.6.0 or later (or embeddedci to 2.12.0), which uses it over USB",
                     verb, json);
    if (n < 0) { out[0] = '\0'; return 0; }
    return (size_t)n < cap ? (size_t)n : cap - 1;
}
