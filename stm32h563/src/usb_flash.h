#ifndef USB_FLASH_H
#define USB_FLASH_H

/* usb_flash — flash a board over the pod's USB host port (USB_HOST=1 builds).
 *
 *  - uf2, file: the board shows up as a USB drive (msc_flash.h): UF2 bootloaders, drag-and-drop
 *    loaders such as DAPLink.
 *  - dfu, teensy: the board's loader takes the image over control transfers (ctrl_flash.h): USB
 *    DFU and ST's DfuSe (STM32 ROM), the Teensy HalfKay loader.
 *  - picoboot: an RP2040 or RP2350 in BOOTSEL, through its PICOBOOT interface (ctrl_flash.h): erase,
 *    write, read every page back and compare, reboot into the app.
 *  - avr, samd: the board's bootloader talks over its USB serial port (ser_flash.h): the Arduino
 *    AVR bootloaders (AVR109, STK500v1, STK500v2) and the Arduino SAMD bootloader (SAM-BA).
 *  - esp: an ESP32 chip's ROM loader over the board's USB serial port (its native USB, or the
 *    USB-serial chip of a dev kit), with the same flasher as over UART (esp_rom_job.h). The pod
 *    resets the chip into its loader with DTR/RTS, as esptool does.
 *
 * The file is the first `len` bytes a `load_bin` with "psram" staged. The pod must already host the
 * board (usb_host on, the device enumerated; a drive mounted for uf2/file, a serial port for
 * avr/samd/esp; a device for dfu/teensy/picoboot). The job runs in steps on the hw worker (usb_flash_poll), so the worker keeps
 * answering commands; it holds off captures, DAC playback, uploads and updates meanwhile, and keeps
 * host mode alive. Every mode but esp runs its protocol code from the W25Q in an XIP session
 * (xip_session.h); esp uses the internal flasher. */

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    USB_FLASH_UF2 = 0, USB_FLASH_FILE, USB_FLASH_DFU, USB_FLASH_TEENSY, USB_FLASH_AVR, USB_FLASH_SAMD,
    USB_FLASH_ESP, USB_FLASH_PICOBOOT,
    USB_FLASH_MODES
} usb_flash_mode_t;

typedef struct {
    uint8_t  mode;          /* usb_flash_mode_t */
    char     name[16];      /* file: the 8.3 file name */
    uint32_t len;
    uint32_t address;       /* dfu (DfuSe): flash address; teensy: loader address of the image;
                               avr: byte address (0); samd: flash address (0x2000 on a SAMD21);
                               esp: flash offset (0 for a merged image);
                               picoboot: flash address (0: 0x10000000), 4 KB aligned */
    uint16_t block;         /* teensy: 1024 (default) or 512 (Teensy LC) */
    bool     leave;         /* dfu, teensy, picoboot, avr, samd, esp: start the new firmware at the end */
} usb_flash_params_t;

/* 0 = started; -1 = refused (BP0084 `why`); -2 = the W25Q code cannot run (BP0083 `why`). */
int  usb_flash_start(const usb_flash_params_t *p, const char **why);
void usb_flash_poll(void);
void usb_flash_abort(void);
bool usb_flash_busy(void);

typedef struct {
    bool        running, have_result;
    const char *mode, *phase, *result;
    const char *protocol;   /* avr, samd: the bootloader found; esp: the chip; else "" */
    int         rc;
    uint32_t    done, total, len, ms, address;
    char        name[16];
} usb_flash_status_t;
void usb_flash_status(usb_flash_status_t *st);

const char *usb_flash_mode_name(uint8_t mode);
/* "uf2", "file", "dfu", "teensy", "avr", "samd", "esp", "picoboot" -> the mode; -1 otherwise. */
int usb_flash_mode_from_name(const char *name);

#endif /* USB_FLASH_H */
