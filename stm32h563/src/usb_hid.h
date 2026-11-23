#ifndef USB_HID_H
#define USB_HID_H

/* usb_hid — the HID interfaces of the board the pod hosts (USB_HOST=1 builds): TinyUSB's HID host
 * class, the report descriptors and the input reports.
 *
 * Reports are caught by a capture that runs from {"capture":true} until it is stopped, the board
 * detaches or host mode ends: one heap ring (USB_HID_CAPTURE_SLOTS reports) takes the reports of
 * every HID instance, each tagged with its instance and the ms since the capture started, and
 * reads drain it. The usb task arms the interrupt endpoints (usb_hid_pump) and the callbacks push
 * into the ring; the hw worker reads it. TinyUSB is only called on the usb task. The class keeps
 * the device's default (report) protocol. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "usb_hid_core.h"

#define USB_HID_MAX_ITF  2u    /* HID interfaces the pod keeps (CFG_TUH_HID) */

typedef struct { uint8_t inst, itf, protocol; } usb_hid_info_t;

/* The mounted HID interfaces of the attached device; returns how many (out has USB_HID_MAX_ITF). */
unsigned usb_hid_interfaces(usb_hid_info_t *out);
/* True while a capture runs. */
bool usb_hid_capture_active(void);
/* Start the capture on every mounted instance (and on the ones that mount later while it runs).
   0 = started, 1 = already running, -1 = out of memory, -2 = no HID interface is mounted. */
int usb_hid_capture_start(void);
/* Stop and free the ring (a no-op when none runs). Also done when the last interface unmounts. */
void usb_hid_capture_stop(void);
/* Buffered reports of the instances in `mask` (bit i = instance i); 0 without a capture. */
unsigned usb_hid_capture_ready(uint8_t mask);
/* Drain up to `max` reports of the instances in `mask` as the reply JSON (usb_hid_reports_json).
   0 when there is no capture or the frame does not fit `cap`. */
size_t usb_hid_capture_read(uint8_t mask, unsigned max, char *out, size_t cap);
/* {"capturing","buffered","dropped"} as JSON; 0 when it does not fit. */
size_t usb_hid_capture_state(char *out, size_t cap);
/* hw worker loop: stop the capture when host mode has ended. */
void usb_hid_poll(void);
/* usb task, every pass of its host loop. */
void usb_hid_pump(void);

#endif /* USB_HID_H */
