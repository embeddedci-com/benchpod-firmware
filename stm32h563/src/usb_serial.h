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

/* usb task, every pass of its host loop. */
void     usb_serial_pump(void);

#endif /* USB_SERIAL_H */
