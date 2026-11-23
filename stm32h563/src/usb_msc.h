#ifndef USB_MSC_H
#define USB_MSC_H

/* usb_msc — the USB drive the pod hosts (USB_HOST=1 builds): TinyUSB's mass-storage host class,
 * and sector reads and writes for the hw worker.
 *
 * TinyUSB's host is not called from two tasks: the worker posts one request at a time into a
 * mailbox, the usb task (usb_msc_pump, called from its host loop) issues it, the completion
 * callback marks it done on the usb task, and the worker waits for that. */

#include <stdbool.h>
#include <stdint.h>

/* A drive is mounted and answered READ CAPACITY: its size. */
bool usb_msc_ready(uint32_t *blocks, uint32_t *block_size);
/* How many drives have mounted since boot: a drive that goes away and comes back counts again. */
uint32_t usb_msc_mounts(void);
/* count sectors at lba, read into or written from buf (which must stay valid until it returns).
   Waits up to timeout_ms. 0 = ok. hw worker only. */
int  usb_msc_io(bool write, uint32_t lba, void *buf, uint32_t count, uint32_t timeout_ms);
/* One control transfer to the mounted device (usb_host_daddr): setup is the 8-byte setup packet,
   data its data stage, *actual the bytes moved. 0 = ok, -1 = stalled or failed, -2 = no device.
   hw worker only. */
int  usb_ctrl_io(const uint8_t setup[8], uint8_t *data, uint32_t *actual, uint32_t timeout_ms);
/* No request is in flight (a buffer may be freed). */
bool usb_msc_idle(void);
/* usb task, every pass of its host loop. */
void usb_msc_pump(void);

#endif /* USB_MSC_H */
