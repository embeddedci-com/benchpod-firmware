#ifndef CTRL_FLASH_H
#define CTRL_FLASH_H

/* ctrl_flash — flashing a board whose bootloader takes the image over USB control transfers (the
 * logic; xip_mod_ctrl_flash.c runs from the W25Q, usb_flash.c drives it).
 *
 *  - DFU: USB DFU 1.1 (UNO R4, GD32, many others) and ST's DfuSe extension (every STM32 in its ROM
 *    bootloader, Black Pill). The pod reads the DFU interface and its transfer size from the
 *    configuration descriptor; for DfuSe it reads the flash layout from the interface's string
 *    ("@Internal Flash /0x08000000/04*016Kg,01*064Kg,07*128Kg"), erases the sectors the image
 *    covers, sets the address pointer and writes the blocks. It then leaves the loader, so the
 *    board starts the new firmware.
 *  - HalfKay: the Teensy loader (3.x, 4.0, 4.1, MicroMod). Blocks of 1024 bytes (512 on the Teensy
 *    LC) behind a 64-byte header with the address, as HID SET_REPORT requests; blocks that are all
 *    0xFF are skipped (except the first), and a last request reboots the board.
 *  - PICOBOOT: an RP2040 or RP2350 in BOOTSEL (2E8A:0003, 2E8A:000F). Its vendor interface takes
 *    commands over bulk endpoints: exclusive access, exit XIP, erase the 4 KB sectors the image
 *    covers, write 256-byte pages, read every page back and compare, then reboot into the app.
 *    The address is where the image goes (0x10000000 is the start of flash).
 *
 * The image is a binary staged by the client, with its start address (the SDK converts .hex and
 * .dfu files). As for msc_flash, the context holds all the state and one step does a bounded
 * amount of work. Every descriptor and status the device sends is checked: the board is under
 * test.
 */

#include <stdbool.h>
#include <stdint.h>

#define CTRL_FLASH_BUF  2048u    /* a DFU block (up to 2048, the STM32 ROM's size) or a HalfKay report */

typedef struct {
    void *io;
    /* One control transfer: setup is the 8-byte setup packet, data the data stage (IN: filled,
       OUT: sent), *actual the bytes moved. 0 = ok, -1 = stalled or failed, -2 = no device. */
    int  (*ctrl)(void *io, const uint8_t setup[8], uint8_t *data, uint32_t *actual);
    int  (*src)(void *io, uint32_t off, uint8_t *buf, uint32_t n);
    void (*delay_ms)(void *io, uint32_t ms);
    uint16_t vid, pid;   /* the attached device */
    /* One bulk transfer (PICOBOOT): ep is the endpoint address; OUT sends len bytes (0: a
       zero-length packet), IN reads up to len. 0 = ok, -1 = stalled or failed, -2 = no device. */
    int  (*bulk)(void *io, uint8_t ep, uint8_t *data, uint32_t len, uint32_t *actual);
} ctrl_flash_ops_t;

typedef enum { CTRL_FLASH_DFU = 0, CTRL_FLASH_HALFKAY = 1, CTRL_FLASH_PICOBOOT = 2 } ctrl_flash_mode_t;

typedef enum {
    CTRL_FLASH_RUNNING = 1,
    CTRL_FLASH_DONE = 0,
    CTRL_FLASH_E_IO = -1,         /* a control transfer failed */
    CTRL_FLASH_E_SRC = -2,        /* reading the staged image failed */
    CTRL_FLASH_E_NO_DFU = -3,     /* no DFU interface in DFU mode on the device */
    CTRL_FLASH_E_STATUS = -4,     /* the loader reported an error (DFU bStatus) */
    CTRL_FLASH_E_LAYOUT = -5,     /* DfuSe: no flash layout, or the image does not fit it */
    CTRL_FLASH_E_NOT_TEENSY = -6, /* not a Teensy in its loader (16C0:0478) */
    CTRL_FLASH_E_TIMEOUT = -7,    /* the loader stayed busy */
    CTRL_FLASH_E_ARGS = -8,       /* bad length, address or block size */
    CTRL_FLASH_E_NOT_RP2 = -9,    /* not an RP2040 or RP2350 in BOOTSEL */
    CTRL_FLASH_E_NO_PICOBOOT = -10, /* no PICOBOOT interface with two bulk endpoints */
    CTRL_FLASH_E_VERIFY = -11,    /* a page read back different from what was written */
} ctrl_flash_rc_t;

typedef enum {
    CTRL_PH_START = 0, CTRL_PH_DESC, CTRL_PH_LAYOUT, CTRL_PH_ERASE, CTRL_PH_ADDR, CTRL_PH_WRITE,
    CTRL_PH_VERIFY, CTRL_PH_LEAVE, CTRL_PH_DONE
} ctrl_flash_phase_t;

#define CTRL_FLASH_MAX_REGIONS 8

typedef struct {
    /* in */
    ctrl_flash_ops_t ops;
    uint8_t  mode;               /* ctrl_flash_mode_t */
    uint32_t len;                /* bytes of the staged image */
    uint32_t address;            /* where it starts (DfuSe: a flash address; HalfKay: loader address) */
    uint16_t block;              /* HalfKay block size: 1024 (default) or 512 (Teensy LC) */
    bool     leave;              /* DFU: leave the loader at the end (start the new firmware) */
    /* out */
    uint8_t  phase;              /* ctrl_flash_phase_t */
    int      result;             /* ctrl_flash_rc_t once done */
    uint32_t done, total;        /* blocks written / to write */
    bool     dfuse;              /* DFU: ST's DfuSe extension */
    uint8_t  itf;                /* DFU interface */
    uint16_t xfer;               /* DFU transfer size used (the device's, at most 2048) */
    bool     addr_each;          /* DfuSe with a device size over 2048: the address before each block */
    bool     addr_sent;          /* addr_each: this block's address is set */
    uint8_t  dfu_status;         /* DFU: the last bStatus; PICOBOOT: the last command status code */
    uint8_t  pb_step, ep_in, ep_out, mps;   /* PICOBOOT: setup step, bulk endpoints, packet size */
    uint32_t pb_token, fail_addr; /* PICOBOOT: command token, address of a page that failed to verify */
    /* DfuSe flash layout */
    uint8_t  nregions;
    struct { uint32_t start, count, size; } region[CTRL_FLASH_MAX_REGIONS];
    /* work */
    uint32_t pos, erase_addr, block_no;
    uint8_t  i_string;
    bool     waiting;            /* DFU: a DNLOAD's status is awaited */
    uint32_t waited;             /* ms waited for the loader (DFU status, HalfKay retries) */
    uint8_t  buf[CTRL_FLASH_BUF];
} ctrl_flash_ctx_t;

int  ctrl_flash_step(ctrl_flash_ctx_t *c);
const char *ctrl_flash_rc_str(int rc);
const char *ctrl_flash_phase_str(int phase);

#endif /* CTRL_FLASH_H */
