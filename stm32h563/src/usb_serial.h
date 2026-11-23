#ifndef USB_SERIAL_H
#define USB_SERIAL_H

/* usb_serial — the USB serial port of the board the pod hosts (USB_HOST=1 builds): TinyUSB's CDC
 * host class (CDC ACM, and the FTDI, CP210x and CH34x USB-serial chips), as a byte stream for the
 * hw worker.
 *
 * TinyUSB's host is not called from two tasks: the worker writes into a TX ring and reads from an
 * RX ring, the usb task (usb_serial_pump, every pass of its host loop) moves the bytes between the
 * rings and TinyUSB. Line changes (DTR/RTS, the baud rate) go through a one-request mailbox, as
 * in usb_msc.h. */

#include <stdbool.h>
#include <stdint.h>

#include "usb_serial_con.h"

/* A serial interface is mounted. */
bool     usb_serial_ready(void);
/* Serial interfaces mounted since boot: a change means the board went away and came back (a
   1200 baud touch starts a bootloader that enumerates as another device). */
uint32_t usb_serial_mounts(void);
/* The mounted interface is a USB-serial chip (FTDI, CP210x, CH34x) rather than the board's own
   USB: the baud rate is real. */
bool     usb_serial_is_bridge(void);

/* Queue `n` bytes, waiting up to timeout_ms for room. 0 = all queued, -1 = no port or timeout.
   hw worker only. */
int      usb_serial_write(const void *buf, uint32_t n, uint32_t timeout_ms);
/* The same, the bytes queued: n, or fewer when the port went away or the room did not come. */
uint32_t usb_serial_write_some(const void *buf, uint32_t n, uint32_t timeout_ms);
/* The next received byte, -1 when none is waiting. hw worker only. */
int      usb_serial_getc(void);
/* Read up to `n` bytes, waiting up to timeout_ms in all: the bytes read. hw worker only. */
uint32_t usb_serial_read(uint8_t *buf, uint32_t n, uint32_t timeout_ms);
/* Drop what was received so far. hw worker only. */
void     usb_serial_rx_clear(void);

/* Set DTR and RTS / the baud rate (8N1), waiting up to timeout_ms. 0 = done, -1 = failed.
   hw worker only. */
int      usb_serial_lines(bool dtr, bool rts, uint32_t timeout_ms);
int      usb_serial_baud(uint32_t baud, uint32_t timeout_ms);

/* Baud rate and data format (data_bits 5..8, parity and stop_bits as USB_SER_*), waiting up to
   timeout_ms. 0 = done, -1 = failed. hw worker only. */
int      usb_serial_coding(uint32_t baud, uint8_t data_bits, uint8_t parity, uint8_t stop_bits,
                           uint32_t timeout_ms);

/* ---- the console: the usb_serial, usb_serial_write and usb_serial_read commands ---------------
 * While it is open the received bytes go into a 4 KB ring from the heap (what does not fit is
 * counted), and the flashers cannot use the port. It follows the board through a reset: when a
 * port of the same kind mounts again it gets the baud rate, format and lines again. All of these
 * are for the hw worker. */
bool     usb_serial_con_is_open(void);
/* Open with the settings of `a` (defaults 115200 8N1, DTR and RTS on), or change them when already
   open. 0 = open, -1 = no port mounted, -2 = out of memory. */
int      usb_serial_con_open(const usb_serial_args_t *a);
void     usb_serial_con_close(void);
void     usb_serial_con_set(const usb_serial_args_t *a);
/* Open and bound to a mounted port. */
bool     usb_serial_con_present(void);
/* Up to n received bytes, without waiting. */
uint32_t usb_serial_con_read(uint8_t *buf, uint32_t n);
/* Queue bytes for the board (waits up to timeout_ms for room): the bytes queued. */
uint32_t usb_serial_con_write(const void *buf, uint32_t n, uint32_t timeout_ms);
void     usb_serial_con_state(usb_serial_state_t *st);
/* Every pass of the worker loop: bind the console to a port that came back. */
void     usb_serial_con_poll(void);

/* usb task, every pass of its host loop. */
void     usb_serial_pump(void);

#endif /* USB_SERIAL_H */
