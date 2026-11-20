#ifndef USB_HOST_DESC_H
#define USB_HOST_DESC_H

/* usb_host_desc — what the pod reads out of a DUT's descriptors in USB host mode (usb_host.h).
 * Pure functions, no TinyUSB, no hardware: host tests run them on hostile input. The DUT is
 * under test, not trusted: every length is checked against the buffer before it is used, and a
 * malformed descriptor stops the walk instead of reading past the end. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define USB_HOST_ITF_MAX  8    /* interfaces reported; more are counted, not listed */
#define USB_HOST_STR_MAX  48   /* string descriptors are cut here (ASCII, NUL included) */

typedef struct {
    uint8_t number, alt, cls, subclass, protocol, endpoints;
} usb_host_itf_t;

typedef struct {
    bool     valid;            /* a configuration descriptor header was read */
    bool     truncated;        /* wTotalLength is longer than the bytes the pod has */
    bool     malformed;        /* a descriptor had a bad length: the walk stopped there */
    uint8_t  value;            /* bConfigurationValue */
    uint8_t  attributes;       /* bmAttributes */
    uint16_t max_power_ma;     /* bMaxPower x 2 (full speed) */
    uint16_t total_len;        /* wTotalLength as the DUT declared it */
    uint8_t  num_itf;          /* bNumInterfaces as declared */
    uint8_t  itf_count;        /* interface descriptors seen (alternates included) */
    usb_host_itf_t itf[USB_HOST_ITF_MAX];
} usb_host_config_t;

/* Parse a configuration descriptor (with its interfaces) of `len` bytes. */
void usb_host_parse_config(const uint8_t *buf, size_t len, usb_host_config_t *out);

/* A string descriptor (bLength, bDescriptorType 3, UTF-16LE) to printable ASCII in `out`
   (`cap` bytes, NUL-terminated): anything outside 0x20..0x7E, and the JSON specials " and \,
   become '?'. Returns false (and an empty string) when it is not a string descriptor. */
bool usb_host_string_ascii(const uint8_t *buf, size_t len, char *out, size_t cap);

/* The class name the reply shows ("cdc", "hid", "msc", ...); "vendor" for 0xFF, NULL for a
   class without a name (the number is still reported). */
const char *usb_host_class_name(uint8_t cls);

#endif /* USB_HOST_DESC_H */
