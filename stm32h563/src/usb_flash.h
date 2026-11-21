#ifndef USB_FLASH_H
#define USB_FLASH_H

/* usb_flash — flash a board over the pod's USB host port (USB_HOST=1 builds).
 *
 *  - uf2, file: the board shows up as a USB drive (msc_flash.h): UF2 bootloaders, drag-and-drop
 *    loaders such as DAPLink.
 *  - dfu, teensy: the board's loader takes the image over control transfers (ctrl_flash.h): USB
 *    DFU and ST's DfuSe (STM32 ROM), the Teensy HalfKay loader.
 *
 * The file is the first `len` bytes a `load_bin` with "psram" staged. The pod must already host the
 * board (usb_host on, the device enumerated; a drive mounted for uf2/file). The job runs in an XIP
 * session (xip_session.h: the protocol logic runs from the W25Q) in steps on the hw worker
 * (usb_flash_poll), so the worker keeps answering commands; the session holds off captures, DAC
 * playback, uploads and updates meanwhile. It keeps host mode alive while it runs. */

#include <stdbool.h>
#include <stdint.h>

typedef enum { USB_FLASH_UF2 = 0, USB_FLASH_FILE, USB_FLASH_DFU, USB_FLASH_TEENSY } usb_flash_mode_t;

typedef struct {
    uint8_t  mode;          /* usb_flash_mode_t */
    char     name[16];      /* file: the 8.3 file name */
    uint32_t len;
    uint32_t address;       /* dfu (DfuSe): flash address; teensy: loader address of the image */
    uint16_t block;         /* teensy: 1024 (default) or 512 (Teensy LC) */
    bool     leave;         /* dfu, teensy: start the new firmware at the end */
} usb_flash_params_t;

/* 0 = started; -1 = refused (BP0084 `why`); -2 = the W25Q code cannot run (BP0083 `why`). */
int  usb_flash_start(const usb_flash_params_t *p, const char **why);
void usb_flash_poll(void);
void usb_flash_abort(void);
bool usb_flash_busy(void);

typedef struct {
    bool        running, have_result;
    const char *mode, *phase, *result;
    int         rc;
    uint32_t    done, total, len, ms, address;
    char        name[16];
} usb_flash_status_t;
void usb_flash_status(usb_flash_status_t *st);

const char *usb_flash_mode_name(uint8_t mode);
/* "uf2", "file", "dfu", "teensy" -> the mode; -1 otherwise. */
int usb_flash_mode_from_name(const char *name);

#endif /* USB_FLASH_H */
