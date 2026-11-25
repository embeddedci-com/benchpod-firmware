/*
 * command_handler_xip.c — the xip command: code that runs from the W25Q (xip_image.h,
 * xip_session.h).
 *
 *   {"cmd":"xip"}                  -> {"supported":bool,"mode":"mapped|ram|none","image":{"size":N,
 *                                      "slot":"ok|outdated|missing|unknown"},"prog_image":{...}}
 *   {"cmd":"xip","selftest":true}  -> the same plus "selftest":{"ok":bool,"result":N,"expected":N,
 *                                      "begin_us":N,"call_us":N}
 *   {"cmd":"xip","selftest":true,"image":"prog"}   the self-test of the prog image (default "usb")
 *
 * "image" is the USB flash image (slot xip), "prog_image" the pin programmers' (slot xip_prog).
 * The self-test opens a session, runs the image's self-test entry (it reads constants from the
 * image and calls back into internal flash) and closes it. Refused with BP0083 when the board or
 * the slot cannot run it, BP0048 while the bus is busy.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "bp_json.h"
#include "bp_errors.h"
#include "blob_store.h"
#include "xip_image.h"
#include "xip_session.h"

#include "stm32h5xx_hal.h"

#include <stdio.h>
#include <string.h>

/* The same probe text and checksum as xip_mod_core.c's self-test, computed here in internal flash
   (a copy: internal code may not read the image's constants). */
static const char k_probe[] = "BenchPod XIP self-test: code and constants from the W25Q";
extern uint32_t xip_core_fnv_step(uint32_t h, uint8_t b);

static uint32_t expected_selftest(uint32_t seed)
{
    uint32_t h = 2166136261u ^ seed;
    for (unsigned i = 0; i < sizeof(k_probe) - 1u; i++) h = xip_core_fnv_step(h, (uint8_t)k_probe[i]);
    return h;
}

void handle_xip(int conn_id, const char *json)
{
    const xip_hash_t *want = xip_image_expected();
    const xip_hash_t *want_p = xip_image_expected_of(XIP_IMG_PROG);
    const uint32_t len = want->magic == XIP_HASH_MAGIC ? want->m.len : 0u;
    const uint32_t len_p = want_p->magic == XIP_HASH_MAGIC ? want_p->m.len : 0u;
    const xip_mode_t mode = xip_mode();
    char st[256] = "";
    if (bp_json_flag(json, "selftest")) {
        const char *busy = bus_busy_reason();   /* BP0048 with what holds the bus */
        if (busy) { send_error(conn_id, busy); return; }
        char which[8] = "usb";
        if (!bp_json_get(json, "image", which, sizeof(which))) strcpy(which, "usb");
        const xip_img_t img = strcmp(which, "prog") == 0 ? XIP_IMG_PROG : XIP_IMG_USB;
        xip_err_t rc = xip_session_begin(img);
        if (rc != XIP_OK) { bp_err_xip_unavailable(conn_id, xip_img_err_text(img, rc)); return; }
        const uint32_t seed = HAL_GetTick();
        const uint32_t t0 = DWT->CYCCNT;
        const uint32_t got = xip_session_call_u32(img == XIP_IMG_PROG ? XIP_PROG_FN_SELFTEST : XIP_FN_SELFTEST, seed);
        const uint32_t call_us = (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000u);
        xip_session_end();
        const uint32_t exp = expected_selftest(seed);
        snprintf(st, sizeof(st), ",\"selftest\":{\"ok\":%s,\"result\":%lu,\"expected\":%lu,\"begin_us\":%lu,\"call_us\":%lu}",
                 got == exp ? "true" : "false", (unsigned long)got, (unsigned long)exp,
                 (unsigned long)xip_session_last_begin_us(), (unsigned long)call_us);
    }
    char data[480];
    snprintf(data, sizeof(data), "{\"supported\":%s,\"mode\":\"%s\",\"image\":{\"size\":%lu,\"slot\":\"%s\"},"
             "\"prog_image\":{\"size\":%lu,\"slot\":\"%s\"}%s}",
             (mode != XIP_MODE_NONE && blob_listed(BLOB_XIP)) ? "true" : "false", xip_mode_name(mode),
             (unsigned long)len, blob_state_str(blob_state(BLOB_XIP)),
             (unsigned long)len_p, blob_state_str(blob_state(BLOB_XIP_PROG)), st);
    send_ok_str(conn_id, data);
}
