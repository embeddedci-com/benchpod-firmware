#ifndef USB_FLASH_H
#define USB_FLASH_H

/* usb_flash — flash a board that shows up as a USB drive (USB_HOST=1 builds): UF2 bootloaders and
 * drag-and-drop loaders such as DAPLink (msc_flash.h).
 *
 * The file is the first `len` bytes a `load_bin` with "psram" staged. The pod must already host the
 * board (usb_host on, a drive mounted). The job runs in an XIP session (xip_session.h: the logic
 * runs from the W25Q) in steps on the hw worker (usb_flash_poll), so the worker keeps answering
 * commands; the session holds off captures, DAC playback, uploads and updates meanwhile. It keeps
 * host mode alive while it runs. */

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t  mode;          /* msc_flash_mode_t */
    char     name[16];      /* file mode: the 8.3 file name */
    uint32_t len;
} usb_flash_params_t;

/* 0 = started; < 0 = refused, `why` set (BP0084 or BP0083 text). hw worker. */
int  usb_flash_start(const usb_flash_params_t *p, const char **why);
void usb_flash_poll(void);
void usb_flash_abort(void);
bool usb_flash_busy(void);

typedef struct {
    bool        running, have_result;
    const char *mode, *phase, *result;
    int         rc;
    uint32_t    done, total, len, ms;
    char        name[16];
} usb_flash_status_t;
void usb_flash_status(usb_flash_status_t *st);

#endif /* USB_FLASH_H */
