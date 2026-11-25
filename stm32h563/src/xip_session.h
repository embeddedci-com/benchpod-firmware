#ifndef XIP_SESSION_H
#define XIP_SESSION_H

/* xip_session — running code from the W25Q image (xip_image.h).
 *
 * A session owns the quad bus from begin to end: it quiesces the gateware's PSRAM masters and takes
 * the bus (w25q_session_open), so no capture, DAC replay, upload or update runs meanwhile
 * (heavy_in_flight() reports it, BP0048). It runs on the hw worker task only.
 *
 * Modes:
 *  - mapped: the W25Q's chip-select is on an OCTOSPI1 NCS pin (digital v3r4 PB6, or a dev build with
 *    XIP_BODGE_PB6=1 on a board with a wire to PB6). The controller maps the W25Q at 0x90000000 and
 *    the CPU runs the image in place. Indirect commands meanwhile go through xspi_guard, which
 *    unmaps and maps again around them.
 *  - ram: a dev build with XIP_RAM=1 links the image at a reserved RAM address instead; begin copies
 *    the slot there. Runs on any board: everything but the memory-mapped execution is the same.
 *
 * begin(img) checks the image's slot length and SHA-256 against the image this firmware was linked
 * with (xip_image_expected_of()) over the bytes it is about to run, and the table's magic, ABI and
 * count. One session at a time: the two images share the RAM region (ram mode) and the bus.
 */

#include <stdbool.h>
#include <stdint.h>
#include "xip_image.h"

typedef enum { XIP_MODE_NONE = 0, XIP_MODE_MAPPED, XIP_MODE_RAM } xip_mode_t;

typedef enum {
    XIP_OK = 0,
    XIP_ERR_UNSUPPORTED,   /* this board or build cannot run W25Q code */
    XIP_ERR_NO_IMAGE,      /* the firmware was linked without an image */
    XIP_ERR_SLOT,          /* the slot is empty or holds another build's image */
    XIP_ERR_BUS,           /* the bus could not be taken */
    XIP_ERR_MAP,           /* memory-mapped mode could not be entered */
    XIP_ERR_HASH,          /* the bytes read back do not match the expected SHA-256 */
    XIP_ERR_TABLE,         /* bad table magic, ABI or count */
    XIP_ERR_ACTIVE,        /* a session is already running */
    XIP_ERR_POOL_BUSY,     /* RAM mode: a capture or upload is using the adc_pool the image overlays */
} xip_err_t;

xip_mode_t  xip_mode(void);
const char *xip_mode_name(xip_mode_t m);
const char *xip_err_text(xip_err_t e);

/* Start a session for image `img` (xip_image.h): XIP_ERR_ACTIVE while another session, of either
   image, runs (they share the RAM region and the bus). Release with xip_session_end on every path. */
xip_err_t xip_session_begin(xip_img_t img);
xip_img_t xip_session_image(void);   /* the image of the running session (meaningful while active) */
/* xip_err_text with the image's own slot named. */
const char *xip_img_err_text(xip_img_t img, xip_err_t e);
/* Call the running image's table entry `fn` (XIP_FN_* or XIP_PROG_FN_*) with one argument. Inside a session only. */
uint32_t  xip_session_call_u32(unsigned fn, uint32_t arg);
/* Call entry `fn` with a context pointer (heap memory the caller owns). -1 outside a session. */
int       xip_session_call_ctx(unsigned fn, void *ctx);
/* The same with two more arguments (the prog image's XIP_PROG_FN_CALL). */
int       xip_session_call_ctx2(unsigned fn, void *ctx, unsigned a, unsigned b);
void      xip_session_end(void);
bool      xip_session_active(void);

/* Timing of the last begin (us), for the command's reply. */
uint32_t  xip_session_last_begin_us(void);

#endif /* XIP_SESSION_H */
