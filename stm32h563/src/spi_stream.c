/*
 * spi_stream.c — see spi_stream.h.
 */
#include "spi_stream.h"

#include <string.h>

const char *spi_stream_strerror(int rc) {
    switch (rc) {
    case SPI_STREAM_OK:     return "ok";
    case SPI_STREAM_E_IO:   return "SPI transfer failed";
    case SPI_STREAM_E_READ: return "source read failed";
    case SPI_STREAM_E_ARGS: return "invalid arguments";
    default:                return "unknown error";
    }
}

int spi_stream_run(const spi_stream_io_t *io, const uint8_t *head, size_t head_len,
                   uint32_t len, bool hold_cs, uint32_t *sent) {
    if (sent) *sent = 0;
    if (!io || !io->cs || !io->xfer || (len && !io->read) ||
        io->max_xfer < SPI_STREAM_HEAD_MAX + 1u || io->max_xfer > 4096u ||
        head_len > SPI_STREAM_HEAD_MAX || (head_len && !head) || (head_len == 0 && len == 0))
        return SPI_STREAM_E_ARGS;

    /* static, not a 4 KB local: the only caller runs on the 12 KB hw worker stack, and
       the worker is the only task that streams, so this is never entered twice at once. */
    static uint8_t buf[4096];
    uint32_t done = 0;
    int rc = SPI_STREAM_OK;
    io->cs(io->ctx, true);
    /* the header rides in the first chunk with the first source bytes */
    size_t fill = head_len;
    if (head_len) memcpy(buf, head, head_len);
    while (fill || done < len) {
        size_t room = io->max_xfer - fill;
        size_t n = (len - done < room) ? (size_t)(len - done) : room;
        if (n && io->read(io->ctx, done, buf + fill, n) != 0) { rc = SPI_STREAM_E_READ; break; }
        if (io->xfer(io->ctx, buf, fill + n) != 0) { rc = SPI_STREAM_E_IO; break; }
        done += (uint32_t)n;
        fill = 0;
        if (io->progress) io->progress(io->ctx, done);
    }
    if (sent) *sent = done;
    if (rc != SPI_STREAM_OK || !hold_cs) io->cs(io->ctx, false);
    return rc;
}
