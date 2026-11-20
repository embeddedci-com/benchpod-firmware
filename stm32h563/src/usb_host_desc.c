/* usb_host_desc.c — descriptor parsing for USB host mode (usb_host_desc.h). */
#include "usb_host_desc.h"

#include <string.h>

enum { DESC_CONFIG = 2, DESC_STRING = 3, DESC_INTERFACE = 4 };

void usb_host_parse_config(const uint8_t *buf, size_t len, usb_host_config_t *out)
{
    memset(out, 0, sizeof(*out));
    if (!buf || len < 9 || buf[0] < 9 || buf[1] != DESC_CONFIG) return;
    out->valid        = true;
    out->total_len    = (uint16_t)(buf[2] | (buf[3] << 8));
    out->num_itf      = buf[4];
    out->value        = buf[5];
    out->attributes   = buf[7];
    out->max_power_ma = (uint16_t)(buf[8] * 2u);

    size_t end = out->total_len < len ? out->total_len : len;
    if (out->total_len > len) out->truncated = true;

    size_t off = buf[0];
    while (off + 2 <= end) {
        uint8_t dlen = buf[off], type = buf[off + 1];
        if (dlen < 2 || off + dlen > end) {
            /* A zero length would loop forever; a length past the end is cut off by the
               buffer when the descriptor was truncated, a lie otherwise. */
            if (!(out->truncated && dlen >= 2)) out->malformed = true;
            break;
        }
        if (type == DESC_INTERFACE && dlen >= 9) {
            if (out->itf_count < USB_HOST_ITF_MAX) {
                usb_host_itf_t *i = &out->itf[out->itf_count];
                i->number = buf[off + 2]; i->alt = buf[off + 3]; i->endpoints = buf[off + 4];
                i->cls = buf[off + 5]; i->subclass = buf[off + 6]; i->protocol = buf[off + 7];
            }
            if (out->itf_count < 255) out->itf_count++;
        }
        off += dlen;
    }
}

bool usb_host_string_ascii(const uint8_t *buf, size_t len, char *out, size_t cap)
{
    if (!out || cap == 0) return false;
    out[0] = '\0';
    if (!buf || len < 2 || buf[1] != DESC_STRING || buf[0] < 2) return false;
    size_t n = buf[0] < len ? buf[0] : len;
    size_t o = 0;
    for (size_t i = 2; i + 1 < n && o + 1 < cap; i += 2) {
        uint16_t c = (uint16_t)(buf[i] | (buf[i + 1] << 8));
        out[o++] = (c >= 0x20 && c <= 0x7E && c != '"' && c != '\\') ? (char)c : '?';
    }
    out[o] = '\0';
    return true;
}

const char *usb_host_class_name(uint8_t cls)
{
    switch (cls) {
    case 0x01: return "audio";
    case 0x02: return "cdc";
    case 0x03: return "hid";
    case 0x06: return "image";
    case 0x07: return "printer";
    case 0x08: return "msc";
    case 0x09: return "hub";
    case 0x0A: return "cdc_data";
    case 0x0B: return "smart_card";
    case 0x0E: return "video";
    case 0xDC: return "diagnostic";
    case 0xE0: return "wireless";
    case 0xEF: return "misc";
    case 0xFE: return "app_specific";
    case 0xFF: return "vendor";
    default:   return NULL;
    }
}
