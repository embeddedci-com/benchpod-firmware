#ifndef SPI_FLASH_H
#define SPI_FLASH_H

/*
 * spi_flash — program a standard SPI NOR flash (W25Q, MX25, GD25, IS25, AT25SF...) through
 * any byte-level SPI transport.  On the pod the transport is the iCE40's SPI master on four
 * LA channels (fpga_spi_*); the host tests plug in a flash model.
 *
 * Only the JEDEC basics every 25-series part shares: 9F ID, 05 RDSR, 06 WREN, 03 READ,
 * 02 PAGE PROGRAM (256-byte pages), 20 4 KB SECTOR ERASE, D8 64 KB BLOCK ERASE, C7 CHIP ERASE.
 * 3-byte addresses, so the first 16 MB.  No status-register writes: a part whose block-protect
 * bits are set fails its first program with SPI_FLASH_E_WEL or SPI_FLASH_E_VERIFY.
 *
 * Pure C, no hardware calls.  Host-tested in test/test_spi_flash.c.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SPI_FLASH_PAGE        256u
#define SPI_FLASH_SECTOR      4096u
#define SPI_FLASH_BLOCK       65536u
#define SPI_FLASH_ADDR_LIMIT  0x1000000u    /* 3-byte addressing */

typedef struct {
    void    *ctx;
    void     (*cs)(void *ctx, bool asserted);
    /* Full duplex: shift tx[0..n) out, what came back into rx (rx may be NULL).  n <= max_xfer.
       0 ok, nonzero = transport failure. */
    int      (*xfer)(void *ctx, const uint8_t *tx, uint8_t *rx, size_t n);
    /* Wait between busy polls (the pod also feeds its watchdog here). */
    void     (*sleep_us)(void *ctx, uint32_t us);
    uint64_t (*now_us)(void *ctx);
    size_t   max_xfer;                     /* >= 8 */
} spi_flash_io_t;

enum {
    SPI_FLASH_OK        = 0,
    SPI_FLASH_E_IO      = -1,   /* the transport failed */
    SPI_FLASH_E_TIMEOUT = -2,   /* WIP never cleared */
    SPI_FLASH_E_ARGS    = -3,   /* range outside 16 MB, zero length, bad alignment */
    SPI_FLASH_E_VERIFY  = -4,   /* read-back differs (protected, not erased, or a bad wire) */
    SPI_FLASH_E_WEL     = -5,   /* WREN did not set WEL: no flash answering, or write-protected */
};

const char *spi_flash_strerror(int rc);

int spi_flash_read_id(const spi_flash_io_t *io, uint8_t id[3]);
/* True when an ID looks like a real part: not all 00 / all FF (an empty bus). */
bool spi_flash_id_valid(const uint8_t id[3]);
/* Capacity in bytes from the ID's third byte (2^n for the usual 0x10..0x19), or 0 unknown. */
uint32_t spi_flash_capacity(const uint8_t id[3]);

int spi_flash_read_status(const spi_flash_io_t *io, uint8_t *sr);
int spi_flash_read(const spi_flash_io_t *io, uint32_t addr, uint8_t *buf, size_t len);

/* Erase every 4 KB sector that [addr, addr+len) touches, with 64 KB block erases where a whole
   aligned block is covered.  *done_start / *done_len (may be NULL) get the range actually
   erased, which is sector-aligned and may be larger than asked. */
int spi_flash_erase(const spi_flash_io_t *io, uint32_t addr, uint32_t len,
                    uint32_t *done_start, uint32_t *done_len);
int spi_flash_chip_erase(const spi_flash_io_t *io, uint32_t timeout_ms);

/* Page-program data at addr (split on 256-byte pages).  The range must already be erased. */
int spi_flash_program(const spi_flash_io_t *io, uint32_t addr, const uint8_t *data, size_t len);
/* Read [addr, addr+len) back and compare with data.  *bad_at (may be NULL) = first mismatch. */
int spi_flash_verify(const spi_flash_io_t *io, uint32_t addr, const uint8_t *data, size_t len,
                     uint32_t *bad_at);

#endif /* SPI_FLASH_H */
