#ifndef SPI_STREAM_H
#define SPI_STREAM_H

/*
 * spi_stream — shift a long byte source out of an SPI master inside one chip-select frame:
 * an optional short header, then `len` bytes read from the source in transport-sized chunks.
 * On the pod the source is the PSRAM region a `load_bin` staged and the transport is the iCE40's
 * SPI master on the LA pins (fpga_spi_*), which pauses SCK between chunks with CS held; that is
 * what an FPGA's slave-SPI configuration port takes (e.g. an ECP5 bitstream after
 * LSC_BITSTREAM_BURST).  The host tests plug in a recording transport.
 *
 * Pure C, no hardware calls.  Host-tested in test/test_spi_stream.c.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SPI_STREAM_HEAD_MAX   64u

typedef struct {
    void   *ctx;
    void    (*cs)(void *ctx, bool asserted);
    /* Shift tx[0..n) out (nothing read back).  n <= max_xfer.  0 ok, nonzero = failure. */
    int     (*xfer)(void *ctx, const uint8_t *tx, size_t n);
    /* Copy source bytes [off, off + n) into buf.  0 ok, nonzero = failure. */
    int     (*read)(void *ctx, uint32_t off, uint8_t *buf, size_t n);
    /* Called after every chunk with the source bytes sent so far (the pod feeds its watchdog). */
    void    (*progress)(void *ctx, uint32_t done);
    size_t  max_xfer;                      /* >= SPI_STREAM_HEAD_MAX + 1 */
} spi_stream_io_t;

enum {
    SPI_STREAM_OK      = 0,
    SPI_STREAM_E_IO    = -1,   /* the transport failed (CS released) */
    SPI_STREAM_E_READ  = -2,   /* the source failed (CS released) */
    SPI_STREAM_E_ARGS  = -3,   /* header too long, nothing to send, bad io */
};

const char *spi_stream_strerror(int rc);

/* Assert CS, send head[0..head_len) and source[0..len), then release CS unless hold_cs.  On any
   error CS is released.  *sent (may be NULL) = source bytes the transport accepted. */
int spi_stream_run(const spi_stream_io_t *io, const uint8_t *head, size_t head_len,
                   uint32_t len, bool hold_cs, uint32_t *sent);

#endif
