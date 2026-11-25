#ifndef USB_FLASH_WAIT_H
#define USB_FLASH_WAIT_H

/* usb_flash_wait — when to read the drive back after a flash (usb_flash.c uses it; host-tested).
 *
 * A DAPLink loader re-enumerates after a copy (good or failed): the drive goes away and comes back,
 * and FAIL.TXT is there after a failure. A UF2 bootloader that stays in its loader keeps the drive;
 * one that starts the new program drops it, and it may come back as some other drive.
 * The caller polls with the clock, the drive's mount count and whether a drive answers now.
 *  - FILE (check FAIL.TXT): ready once a drive that mounted after the flash has been there for the
 *    settle time; gives up after 10 s.
 *  - UF2 (verify): ready once a drive has been there for the settle time and at least 1.5 s have
 *    passed since the last block (the same drive, or one that came back); gives up after 5 s. */

#include <stdbool.h>
#include <stdint.h>

typedef enum { USB_FLASH_WAIT_PENDING = 0, USB_FLASH_WAIT_READY, USB_FLASH_WAIT_TIMEOUT } usb_flash_wait_rc_t;

typedef struct {
    uint32_t t0, ready_since, mounts0, settle_ms, min_ms, timeout_ms;
    bool     need_remount, was_ready;
} usb_flash_wait_t;

static inline void usb_flash_wait_start(usb_flash_wait_t *w, bool file_mode, uint32_t now, uint32_t mounts0)
{
    w->t0 = now;
    w->mounts0 = mounts0;
    w->ready_since = 0;
    w->was_ready = false;
    w->settle_ms = 500u;
    w->need_remount = file_mode;
    w->min_ms = file_mode ? 0u : 1500u;
    w->timeout_ms = file_mode ? 10000u : 5000u;
}

static inline usb_flash_wait_rc_t usb_flash_wait_poll(usb_flash_wait_t *w, uint32_t now, uint32_t mounts, bool ready)
{
    if (!ready) w->was_ready = false;
    else if (!w->was_ready || mounts != w->mounts0) {
        /* a drive that just appeared: the settle time runs from here (and from the flash for the
           drive that never left) */
        if (!w->was_ready) w->ready_since = now;
        w->was_ready = true;
    }
    if (w->was_ready && (!w->need_remount || mounts != w->mounts0) &&
        now - w->ready_since >= w->settle_ms && now - w->t0 >= w->min_ms)
        return USB_FLASH_WAIT_READY;
    return now - w->t0 >= w->timeout_ms ? USB_FLASH_WAIT_TIMEOUT : USB_FLASH_WAIT_PENDING;
}

#endif /* USB_FLASH_WAIT_H */
