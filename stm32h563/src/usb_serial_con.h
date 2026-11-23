#ifndef USB_SERIAL_CON_H
#define USB_SERIAL_CON_H

/* usb_serial_con — the pure parts of the USB serial console (the usb_serial, usb_serial_write and
 * usb_serial_read commands): the receive ring, the argument parser, the refusal rules, the state
 * reply, the base64 of the data and the `until` match. No TinyUSB and no FreeRTOS, so the host
 * tests build it (test_usb_serial_con.c). The port itself is usb_serial.h. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---- the receive ring ---------------------------------------------------------------------
 * `size` is a power of two. A push that does not fit keeps what is buffered and counts the bytes
 * it dropped. Not thread safe: the caller holds a critical section. */
typedef struct {
    uint8_t *buf;
    uint32_t size, head, tail;   /* head and tail run free */
    uint32_t dropped;
} usb_ring_t;

void     usb_ring_init(usb_ring_t *r, uint8_t *buf, uint32_t size);
uint32_t usb_ring_avail(const usb_ring_t *r);
/* Returns the bytes stored; the rest is counted in r->dropped. */
uint32_t usb_ring_push(usb_ring_t *r, const uint8_t *data, uint32_t n);
uint32_t usb_ring_pop(usb_ring_t *r, uint8_t *out, uint32_t n);

/* ---- the usb_serial arguments ------------------------------------------------------------ */
#define USB_SER_BAUD_MIN  300u
#define USB_SER_BAUD_MAX  3000000u
#define USB_SER_PARITY_NONE 0u   /* the CDC line coding's values */
#define USB_SER_PARITY_ODD  1u
#define USB_SER_PARITY_EVEN 2u
#define USB_SER_STOP_1      0u
#define USB_SER_STOP_2      2u

typedef struct {
    bool     has_open, open;
    bool     has_baud;  uint32_t baud;
    bool     has_dtr,  dtr;
    bool     has_rts,  rts;
    bool     has_frame;                       /* any of data_bits, parity, stop_bits given */
    uint8_t  data_bits, parity, stop_bits;    /* 5..8, USB_SER_PARITY_*, USB_SER_STOP_* */
} usb_serial_args_t;

typedef enum {
    USB_SER_ARG_OK = 0,
    USB_SER_ARG_OPEN, USB_SER_ARG_BAUD, USB_SER_ARG_DTR, USB_SER_ARG_RTS,
    USB_SER_ARG_DATA_BITS, USB_SER_ARG_PARITY, USB_SER_ARG_STOP_BITS,
} usb_serial_arg_err_t;

/* Parse {"cmd":"usb_serial",...}. Defaults for the frame when only part of it is given: 8N1. */
usb_serial_arg_err_t usb_serial_args_parse(const char *json, usb_serial_args_t *a);
const char *usb_serial_arg_err_text(usb_serial_arg_err_t e);   /* a sentence for send_error */
/* True when the request changes nothing and only asks for the state. */
bool usb_serial_args_is_read(const usb_serial_args_t *a);

/* ---- the refusal rules ------------------------------------------------------------------- */
typedef enum {
    USB_SER_GO = 0,
    USB_SER_NO_HOST,      /* BP0085: usb_host is off */
    USB_SER_NO_PORT,      /* BP0085: hosting, but no serial port mounted */
    USB_SER_FLASHING,     /* BP0086: a usb_flash owns the port */
    USB_SER_NOT_OPEN,     /* BP0087 */
} usb_serial_refusal_t;

/* Whether a usb_serial request may go ahead. A close and a state read always do. */
usb_serial_refusal_t usb_serial_plan(const usb_serial_args_t *a, bool is_open, bool host_on,
                                     bool port_present, bool flash_busy);
/* The words for the `why` of BP0085 / BP0086. */
const char *usb_serial_refusal_why(usb_serial_refusal_t r);

/* ---- the state reply --------------------------------------------------------------------- */
typedef struct {
    bool     open;
    uint32_t baud;
    bool     dtr, rts;
    uint8_t  data_bits, parity, stop_bits;
    const char *bridge;       /* "cdc" or the chip name */
    uint32_t rx_buffered, rx_dropped, tx_total, rx_total;
    bool     port_present;
} usb_serial_state_t;

/* The reply data object. Returns its length, 0 when it does not fit. */
size_t usb_serial_state_json(const usb_serial_state_t *s, char *out, size_t cap);
/* "8N1", "7E2", ... (into a 4 byte buffer at least). */
void   usb_serial_frame_str(uint8_t data_bits, uint8_t parity, uint8_t stop_bits, char out[4]);
/* "cdc" for the board's own USB (CDC ACM), else the chip by its USB vendor ID. */
const char *usb_serial_chip_name(bool bridge, uint16_t vid);

/* ---- data -------------------------------------------------------------------------------- */
/* Base64 of the "data_b64" argument: standard or URL alphabet, padding optional, white space
   not allowed. 0 = ok. */
int    usb_serial_b64_decode(const char *in, uint8_t *out, size_t cap, size_t *len);
/* usb_serial_read's `until`: the data ends with the `un` bytes of `u` (un > 0). */
bool   usb_serial_ends_with(const uint8_t *data, size_t n, const uint8_t *u, size_t un);

#endif /* USB_SERIAL_CON_H */
