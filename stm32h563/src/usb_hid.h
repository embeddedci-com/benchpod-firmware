#ifndef USB_HID_H
#define USB_HID_H

/* usb_hid — the HID interfaces of the board the pod hosts (USB_HOST=1 builds): TinyUSB's HID host
 * class, the report descriptors and the input reports.
 *
 * Reports are caught only while a read runs: the hw worker starts a capture into a ring (heap
 * memory it owns), the usb task arms the interrupt endpoints and the callbacks push into the
 * ring, and the worker stops the capture and frees the ring. TinyUSB is only called on the usb
 * task (usb_hid_pump and the callbacks). The class keeps the device's default (report) protocol. */

#include <stdbool.h>
#include <stdint.h>

#include "usb_hid_core.h"

#define USB_HID_MAX_ITF  2u    /* HID interfaces the pod keeps (CFG_TUH_HID) */

typedef struct { uint8_t inst, itf, protocol; } usb_hid_info_t;

/* The mounted HID interfaces of the attached device; returns how many (out has USB_HID_MAX_ITF). */
unsigned usb_hid_interfaces(usb_hid_info_t *out);
/* Start catching input reports of the instances in `mask` (bit i = instance i) into `ring`.
   False when one is not mounted or a capture is running. */
bool usb_hid_capture_start(uint8_t mask, usb_hid_ring_t *ring);
unsigned usb_hid_capture_count(void);
/* Stop; the ring is not touched afterwards, so the caller may free it. */
void usb_hid_capture_stop(void);
/* usb task, every pass of its host loop. */
void usb_hid_pump(void);

#endif /* USB_HID_H */
