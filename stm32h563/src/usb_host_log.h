/* usb_host_log.h — TinyUSB's host log in a RAM ring (USB_HOST_LOG=1 development builds). */
#ifndef USB_HOST_LOG_H
#define USB_HOST_LOG_H
#include <stddef.h>

#define USB_HOST_LOG_SIZE 3072u

int    usb_host_log_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
size_t usb_host_log_read(char *out, size_t cap);   /* the newest bytes, NUL-terminated */
void   usb_host_log_clear(void);

#endif
