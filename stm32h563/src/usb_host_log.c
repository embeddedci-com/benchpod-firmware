/* usb_host_log.c — TinyUSB's host log in a RAM ring, read over the network with
 * {"cmd":"usb_host","log":true}: development builds made with USB_HOST_LOG=1 only (the pod's console
 * is on the USB-C connector, which hosting takes over). */
#include "usb_host_log.h"

#if defined(BENCHPOD_USB_HOST_LOG)
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "stm32h5xx.h"

static char s_ring[USB_HOST_LOG_SIZE];
static volatile uint32_t s_head, s_count;   /* next write position, bytes held (<= size) */

int usb_host_log_printf(const char *fmt, ...)
{
    char line[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n <= 0) return n;
    if (n > (int)sizeof(line) - 1) n = (int)sizeof(line) - 1;
    uint32_t primask = __get_PRIMASK();   /* TinyUSB logs from the USB interrupt too */
    __disable_irq();
    for (int i = 0; i < n; i++) {
        if (line[i] == '\r') continue;
        s_ring[s_head] = line[i];
        s_head = (s_head + 1u) % USB_HOST_LOG_SIZE;
        if (s_count < USB_HOST_LOG_SIZE) s_count++;
    }
    __set_PRIMASK(primask);
    return n;
}

size_t usb_host_log_read(char *out, size_t cap)
{
    if (cap == 0) return 0;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t count = s_count, head = s_head;
    size_t n = count < cap - 1 ? count : cap - 1;   /* the newest n bytes */
    uint32_t start = (head + USB_HOST_LOG_SIZE - (uint32_t)n) % USB_HOST_LOG_SIZE;
    for (size_t i = 0; i < n; i++) out[i] = s_ring[(start + i) % USB_HOST_LOG_SIZE];
    __set_PRIMASK(primask);
    out[n] = '\0';
    return n;
}

void usb_host_log_clear(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_head = s_count = 0;
    __set_PRIMASK(primask);
}
#endif
