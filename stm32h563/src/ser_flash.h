#ifndef SER_FLASH_H
#define SER_FLASH_H

/* ser_flash — flashing a board whose bootloader talks over its USB serial port (the logic;
 * xip_mod_ser_flash.c runs from the W25Q, usb_flash.c drives it).
 *
 *  - AVR: the Arduino bootloaders. The pod finds the one the board runs:
 *      - AVR109 (Caterina: Leonardo, Micro, Pro Micro, Feather 32u4). A board running its sketch
 *        gets the 1200 baud touch and comes back as its bootloader.
 *      - STK500v1 (optiboot and the older ATmegaBOOT: Uno, Nano, Pro Mini, Duemilanove), after a
 *        DTR reset, at 115200 then 57600 baud.
 *      - STK500v2 (the Mega 2560 and 1280 bootloader), after a DTR reset at 115200.
 *    Every page is written, then read back and compared.
 *  - SAMD: the Arduino SAMD bootloader (Zero, MKR, Nano 33 IoT, and the UF2 bootloaders of
 *    Adafruit and Seeed SAMD boards, which answer the same way): SAM-BA with Arduino's fast write
 *    commands, as bossac uses them. 1200 baud touch, flash geometry read from the chip, erase from
 *    the image's address to the end, 4 KB at a time through the board's RAM, read back and
 *    compared, then a reset into the new sketch.
 *
 * The image is a binary staged by the client with its start address (the SDK converts .hex).
 * The context holds all the state and one step does one exchange with the bootloader (a few at
 * most), so the hw worker keeps answering between steps. Every reply is checked: the board is
 * under test.
 */

#include <stdbool.h>
#include <stdint.h>

#define SER_FLASH_BUF  1024u

typedef struct {
    void *io;
    /* Queue n bytes. 0 = ok. */
    int      (*write)(void *io, const uint8_t *buf, uint32_t n);
    /* Read up to n bytes, waiting up to timeout_ms in all: the bytes read. */
    uint32_t (*read)(void *io, uint8_t *buf, uint32_t n, uint32_t timeout_ms);
    void     (*rx_clear)(void *io);
    /* DTR and RTS, the baud rate. 0 = ok. */
    int      (*lines)(void *io, bool dtr, bool rts);
    int      (*baud)(void *io, uint32_t baud);
    /* The serial port: mounted (true) and how many have mounted so far; its VID and PID. */
    bool     (*port)(void *io, uint32_t *mounts, uint16_t *vid, uint16_t *pid);
    int      (*src)(void *io, uint32_t off, uint8_t *buf, uint32_t n);
    void     (*delay_ms)(void *io, uint32_t ms);
} ser_flash_ops_t;

typedef enum { SER_FLASH_AVR = 0, SER_FLASH_SAMD = 1 } ser_flash_mode_t;
typedef enum { SER_PROTO_NONE = 0, SER_PROTO_AVR109, SER_PROTO_STK500V1, SER_PROTO_STK500V2, SER_PROTO_SAMBA } ser_flash_proto_t;

typedef enum {
    SER_FLASH_RUNNING = 1,
    SER_FLASH_DONE = 0,
    SER_FLASH_E_IO = -1,         /* the serial port failed or went away */
    SER_FLASH_E_SRC = -2,        /* reading the staged image failed */
    SER_FLASH_E_NO_LOADER = -3,  /* no bootloader answered */
    SER_FLASH_E_REPLY = -4,      /* the bootloader answered something unexpected */
    SER_FLASH_E_VERIFY = -5,     /* what the board holds is not the image */
    SER_FLASH_E_FIT = -6,        /* the image does not fit the chip, or would overwrite the bootloader */
    SER_FLASH_E_REATTACH = -7,   /* the board did not come back as its bootloader */
    SER_FLASH_E_ARGS = -8,       /* bad length or address */
    SER_FLASH_E_UNSUPPORTED = -9,/* SAMD: the bootloader lacks the fast write commands */
} ser_flash_rc_t;

typedef enum {
    SER_PH_START = 0, SER_PH_TOUCH, SER_PH_REATTACH, SER_PH_RESET, SER_PH_SYNC, SER_PH_INFO,
    SER_PH_ERASE, SER_PH_WRITE, SER_PH_VERIFY, SER_PH_LEAVE, SER_PH_DONE
} ser_flash_phase_t;

typedef struct {
    /* in */
    ser_flash_ops_t ops;
    uint8_t  mode;               /* ser_flash_mode_t */
    uint32_t len;                /* bytes of the staged image */
    uint32_t address;            /* AVR: byte address (0 for a sketch); SAMD: flash address */
    bool     leave;              /* start the new firmware at the end */
    /* out */
    uint8_t  phase;              /* ser_flash_phase_t */
    int      result;             /* ser_flash_rc_t once done */
    uint8_t  proto;              /* ser_flash_proto_t found */
    uint32_t baud;               /* the rate the bootloader answered at (bridge chips) */
    uint8_t  sig[3];             /* AVR signature */
    uint32_t chip_id;            /* SAMD: DSU DID */
    uint32_t flash_size, page;   /* the chip's flash and page size (0 when unknown) */
    uint32_t done, total;        /* bytes written / to write (then verified / to verify) */
    /* work */
    uint8_t  probe;              /* AVR: which bootloader and rate is being tried */
    uint8_t  attempt;
    uint32_t mounts;             /* serial ports mounted before the 1200 baud touch */
    bool     touch_fallback;     /* the touch came after nothing answered (not from the board's IDs) */
    uint32_t waited;
    uint32_t pos;
    uint8_t  seq;                /* STK500v2 message number */
    uint32_t boot_end;           /* SAMD: first address after the bootloader */
    uint8_t  buf[SER_FLASH_BUF + 32];
} ser_flash_ctx_t;

int  ser_flash_step(ser_flash_ctx_t *c);
const char *ser_flash_rc_str(int rc);
const char *ser_flash_phase_str(int phase);
const char *ser_flash_proto_str(int proto);

#endif /* SER_FLASH_H */
