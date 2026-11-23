/* usb_serial_con.c — see usb_serial_con.h. Pure C, host-tested. */
#include "usb_serial_con.h"

#include "bp_json.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void usb_ring_init(usb_ring_t *r, uint8_t *buf, uint32_t size)
{
    r->buf = buf;
    r->size = size;
    r->head = r->tail = r->dropped = 0;
}

uint32_t usb_ring_avail(const usb_ring_t *r) { return r->head - r->tail; }

uint32_t usb_ring_push(usb_ring_t *r, const uint8_t *data, uint32_t n)
{
    uint32_t room = r->size - (r->head - r->tail);
    uint32_t k = n < room ? n : room;
    for (uint32_t i = 0; i < k; i++) r->buf[(r->head + i) & (r->size - 1u)] = data[i];
    r->head += k;
    r->dropped += n - k;
    return k;
}

uint32_t usb_ring_pop(usb_ring_t *r, uint8_t *out, uint32_t n)
{
    uint32_t have = r->head - r->tail;
    uint32_t k = n < have ? n : have;
    for (uint32_t i = 0; i < k; i++) out[i] = r->buf[(r->tail + i) & (r->size - 1u)];
    r->tail += k;
    return k;
}

/* ---- arguments --------------------------------------------------------------------------- */

static bool parse_bool(const char *s, bool *out)
{
    if (!strcmp(s, "true"))  { *out = true;  return true; }
    if (!strcmp(s, "false")) { *out = false; return true; }
    return false;
}

usb_serial_arg_err_t usb_serial_args_parse(const char *json, usb_serial_args_t *a)
{
    memset(a, 0, sizeof(*a));
    a->data_bits = 8;
    a->parity = USB_SER_PARITY_NONE;
    a->stop_bits = USB_SER_STOP_1;
    char v[16];

    if (bp_json_get(json, "open", v, sizeof(v))) {
        if (!parse_bool(v, &a->open)) return USB_SER_ARG_OPEN;
        a->has_open = true;
    }
    if (bp_json_get(json, "baud", v, sizeof(v))) {
        char *end = NULL;
        unsigned long n = strtoul(v, &end, 10);
        if (!end || *end || v[0] == '-' || n < USB_SER_BAUD_MIN || n > USB_SER_BAUD_MAX) return USB_SER_ARG_BAUD;
        a->has_baud = true;
        a->baud = (uint32_t)n;
    }
    if (bp_json_get(json, "dtr", v, sizeof(v))) {
        if (!parse_bool(v, &a->dtr)) return USB_SER_ARG_DTR;
        a->has_dtr = true;
    }
    if (bp_json_get(json, "rts", v, sizeof(v))) {
        if (!parse_bool(v, &a->rts)) return USB_SER_ARG_RTS;
        a->has_rts = true;
    }
    if (bp_json_get(json, "data_bits", v, sizeof(v))) {
        if (strlen(v) != 1 || v[0] < '5' || v[0] > '8') return USB_SER_ARG_DATA_BITS;
        a->data_bits = (uint8_t)(v[0] - '0');
        a->has_frame = true;
    }
    if (bp_json_get(json, "parity", v, sizeof(v))) {
        if (!strcmp(v, "none"))      a->parity = USB_SER_PARITY_NONE;
        else if (!strcmp(v, "even")) a->parity = USB_SER_PARITY_EVEN;
        else if (!strcmp(v, "odd"))  a->parity = USB_SER_PARITY_ODD;
        else return USB_SER_ARG_PARITY;
        a->has_frame = true;
    }
    if (bp_json_get(json, "stop_bits", v, sizeof(v))) {
        if (!strcmp(v, "1"))      a->stop_bits = USB_SER_STOP_1;
        else if (!strcmp(v, "2")) a->stop_bits = USB_SER_STOP_2;
        else return USB_SER_ARG_STOP_BITS;
        a->has_frame = true;
    }
    return USB_SER_ARG_OK;
}

const char *usb_serial_arg_err_text(usb_serial_arg_err_t e)
{
    switch (e) {
    case USB_SER_ARG_OPEN:      return "usb_serial: open must be true or false";
    case USB_SER_ARG_BAUD:      return "usb_serial: baud must be 300 to 3000000";
    case USB_SER_ARG_DTR:       return "usb_serial: dtr must be true or false";
    case USB_SER_ARG_RTS:       return "usb_serial: rts must be true or false";
    case USB_SER_ARG_DATA_BITS: return "usb_serial: data_bits must be 5, 6, 7 or 8";
    case USB_SER_ARG_PARITY:    return "usb_serial: parity must be none, even or odd";
    case USB_SER_ARG_STOP_BITS: return "usb_serial: stop_bits must be 1 or 2";
    default:                    return "usb_serial: bad argument";
    }
}

bool usb_serial_args_is_read(const usb_serial_args_t *a)
{
    return !a->has_open && !a->has_baud && !a->has_dtr && !a->has_rts && !a->has_frame;
}

/* ---- refusals ---------------------------------------------------------------------------- */

usb_serial_refusal_t usb_serial_plan(const usb_serial_args_t *a, bool is_open, bool host_on,
                                     bool port_present, bool flash_busy)
{
    if (usb_serial_args_is_read(a)) return USB_SER_GO;
    if (a->has_open && !a->open) return USB_SER_GO;          /* a close always works */
    if (a->has_open) {                                       /* open (or open again with new settings) */
        if (flash_busy)    return USB_SER_FLASHING;
        if (!host_on)      return USB_SER_NO_HOST;
        if (!port_present) return USB_SER_NO_PORT;
        return USB_SER_GO;
    }
    /* settings of a console that is open: a port that is away is applied when it is back */
    return is_open ? USB_SER_GO : USB_SER_NOT_OPEN;
}

const char *usb_serial_refusal_why(usb_serial_refusal_t r)
{
    switch (r) {
    case USB_SER_NO_HOST:  return "USB host mode is off";
    case USB_SER_NO_PORT:  return "no serial port of the board is mounted";
    case USB_SER_FLASHING: return "a usb_flash is running";
    default:               return "";
    }
}

/* ---- state ------------------------------------------------------------------------------- */

void usb_serial_frame_str(uint8_t data_bits, uint8_t parity, uint8_t stop_bits, char out[4])
{
    out[0] = (char)('0' + (data_bits > 9 ? 8 : data_bits));
    out[1] = parity == USB_SER_PARITY_EVEN ? 'E' : parity == USB_SER_PARITY_ODD ? 'O' : 'N';
    out[2] = stop_bits == USB_SER_STOP_2 ? '2' : '1';
    out[3] = 0;
}

const char *usb_serial_chip_name(bool bridge, uint16_t vid)
{
    if (!bridge) return "cdc";
    switch (vid) {
    case 0x0403: return "ftdi";
    case 0x10C4: return "cp210x";
    case 0x1A86: return "ch34x";
    case 0x067B: return "pl2303";
    default:     return "bridge";
    }
}

size_t usb_serial_state_json(const usb_serial_state_t *s, char *out, size_t cap)
{
    char frame[4];
    usb_serial_frame_str(s->data_bits, s->parity, s->stop_bits, frame);
    int n = snprintf(out, cap,
                     "{\"open\":%s,\"baud\":%lu,\"frame\":\"%s\",\"dtr\":%s,\"rts\":%s,\"bridge\":\"%s\","
                     "\"rx_buffered\":%lu,\"rx_dropped\":%lu,\"tx_total\":%lu,\"rx_total\":%lu,"
                     "\"port_present\":%s}",
                     s->open ? "true" : "false", (unsigned long)s->baud, frame, s->dtr ? "true" : "false",
                     s->rts ? "true" : "false", s->bridge ? s->bridge : "cdc", (unsigned long)s->rx_buffered,
                     (unsigned long)s->rx_dropped, (unsigned long)s->tx_total, (unsigned long)s->rx_total,
                     s->port_present ? "true" : "false");
    return (n > 0 && (size_t)n < cap) ? (size_t)n : 0;
}

/* ---- data -------------------------------------------------------------------------------- */

static int b64_val(char c)
{
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+' || c == '-') return 62;
    if (c == '/' || c == '_') return 63;
    return -1;
}

int usb_serial_b64_decode(const char *in, uint8_t *out, size_t cap, size_t *len)
{
    size_t n = strlen(in), o = 0;
    while (n && in[n - 1] == '=') n--;
    if (n % 4 == 1) return -1;
    uint32_t acc = 0;
    int bits = 0;
    for (size_t i = 0; i < n; i++) {
        int v = b64_val(in[i]);
        if (v < 0) return -1;
        acc = (acc << 6) | (uint32_t)v;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            if (o >= cap) return -1;
            out[o++] = (uint8_t)(acc >> bits);
            acc &= (1u << bits) - 1u;
        }
    }
    *len = o;
    return 0;
}

bool usb_serial_ends_with(const uint8_t *data, size_t n, const uint8_t *u, size_t un)
{
    return un > 0 && n >= un && memcmp(data + n - un, u, un) == 0;
}
