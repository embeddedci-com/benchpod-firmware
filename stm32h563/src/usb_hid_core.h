#ifndef USB_HID_CORE_H
#define USB_HID_CORE_H

/* usb_hid_core — the pure parts of usb_hid (usb_hid.h): the ring input reports go into, and the
 * JSON of the usb_hid replies. No TinyUSB, no hardware: host tests run them.
 * The DUT is under test, not trusted: a report longer than USB_HID_REPORT_MAX is cut, a full ring
 * drops (and counts) instead of overwriting. The ring is the capture's: it lives from
 * {"capture":true} to {"capture":false} and holds the reports of every instance, in arrival order. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define USB_HID_RING_MAX    32u    /* reports a read can return */
#define USB_HID_CAPTURE_SLOTS 64u  /* reports the capture ring buffers between reads */
#define USB_HID_REPORT_MAX  64u    /* a full-speed interrupt endpoint's largest packet */

typedef struct {
    uint32_t ms;                   /* since the capture started */
    uint8_t  inst, len;
    uint8_t  data[USB_HID_REPORT_MAX];
} usb_hid_rep_t;

typedef struct {
    usb_hid_rep_t *slot;           /* `cap` entries, owned by the caller */
    uint8_t  cap, head, count;
    uint16_t dropped;              /* reports that arrived with the ring full */
} usb_hid_ring_t;

void usb_hid_ring_init(usb_hid_ring_t *r, usb_hid_rep_t *storage, uint8_t cap);
/* false (and dropped++) when full. */
bool usb_hid_ring_push(usb_hid_ring_t *r, uint32_t ms, uint8_t inst, const uint8_t *data, size_t len);
bool usb_hid_ring_pop(usb_hid_ring_t *r, usb_hid_rep_t *out);
/* The oldest report whose instance is in `mask` (bit i = instance i); the others stay, in order. */
bool usb_hid_ring_pop_match(usb_hid_ring_t *r, uint8_t mask, usb_hid_rep_t *out);
/* How many buffered reports match `mask`. */
unsigned usb_hid_ring_count_match(const usb_hid_ring_t *r, uint8_t mask);

/* Lower-case hex of n bytes, NUL-terminated. Returns the characters written, 0 when it does not fit. */
size_t usb_hid_hex(const uint8_t *p, size_t n, char *out, size_t cap);

typedef struct {
    uint8_t  inst, itf, protocol;  /* protocol: 0 none, 1 keyboard, 2 mouse */
    uint16_t desc_len;             /* the report descriptor's length, 0 when it could not be read */
    const uint8_t *desc;           /* its bytes, desc_len of them (NULL when none) */
} usb_hid_itf_t;

const char *usb_hid_protocol_name(uint8_t protocol);

/* {"attached":bool,"interfaces":[{"instance","interface","protocol","descriptor_len",
   "descriptor_offset","descriptor":"hex","truncated"}]} into out. The descriptors share what
   fits in `cap`; `offset` starts each at that byte. Returns the length, 0 when it does not fit. */
size_t usb_hid_list_json(const usb_hid_itf_t *itf, unsigned n, bool attached, size_t offset, char *out, size_t cap);

/* {"reports":[{"ms","instance","data":"hex"}],"count":N,"dropped":D,"capturing":bool}: pops up to
   `max` reports of the instances in `mask` from the ring that fit in `cap`; the rest stay.
   `dropped` is the ring's count since the capture started. Returns the length, 0 when not even
   the frame fits. */
size_t usb_hid_reports_json(usb_hid_ring_t *r, unsigned max, uint8_t mask, bool capturing, char *out, size_t cap);

/* {"capturing":bool,"buffered":N,"dropped":D}: the reply of capture true / false. */
size_t usb_hid_capture_json(bool capturing, unsigned buffered, unsigned dropped, char *out, size_t cap);

#endif /* USB_HID_CORE_H */
