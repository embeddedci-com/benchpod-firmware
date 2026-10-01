/* test_spi_stream.c — spi_stream_run against a recording transport. */
#include "spi_stream.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails;
#define CHECK(c) do { if (!(c)) { fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

typedef struct {
    uint8_t *src; uint32_t src_len;
    uint8_t out[300000]; size_t out_len;
    int cs, cs_asserts, cs_releases, bytes_outside_cs;
    size_t max_chunk; int xfers;
    int fail_xfer_at, fail_read_at;        /* 1-based call index, 0 = never */
    int reads; uint32_t last_progress;
} rec_t;

static void t_cs(void *c, bool a) { rec_t *r = c; r->cs = a; if (a) r->cs_asserts++; else r->cs_releases++; }
static int t_xfer(void *c, const uint8_t *tx, size_t n) {
    rec_t *r = c;
    r->xfers++;
    if (r->fail_xfer_at && r->xfers == r->fail_xfer_at) return 1;
    if (!r->cs) r->bytes_outside_cs += (int)n;
    if (n > r->max_chunk) r->max_chunk = n;
    memcpy(r->out + r->out_len, tx, n); r->out_len += n;
    return 0;
}
static int t_read(void *c, uint32_t off, uint8_t *buf, size_t n) {
    rec_t *r = c;
    r->reads++;
    if (r->fail_read_at && r->reads == r->fail_read_at) return 1;
    if (off + n > r->src_len) return 1;
    memcpy(buf, r->src + off, n);
    return 0;
}
static void t_prog(void *c, uint32_t done) { ((rec_t *)c)->last_progress = done; }

static rec_t *mk(uint32_t len) {
    rec_t *r = calloc(1, sizeof(rec_t));
    r->src = malloc(len ? len : 1); r->src_len = len;
    for (uint32_t i = 0; i < len; i++) r->src[i] = (uint8_t)(i * 131u + 7u);
    return r;
}
static spi_stream_io_t io_of(rec_t *r, size_t max) {
    spi_stream_io_t io = { r, t_cs, t_xfer, t_read, t_prog, max };
    return io;
}

int main(void) {
    /* 1: an ECP5-sized stream with a burst-command header: one CS frame, exact bytes, chunks <= 512 */
    {
        uint32_t len = 262123;
        rec_t *r = mk(len);
        spi_stream_io_t io = io_of(r, 512);
        const uint8_t head[4] = { 0x7A, 0, 0, 0 };
        uint32_t sent = 0;
        int rc = spi_stream_run(&io, head, 4, len, false, &sent);
        CHECK(rc == SPI_STREAM_OK);
        CHECK(sent == len);
        CHECK(r->out_len == len + 4);
        CHECK(memcmp(r->out, head, 4) == 0);
        CHECK(memcmp(r->out + 4, r->src, len) == 0);
        CHECK(r->cs_asserts == 1 && r->cs_releases == 1 && r->cs == 0);
        CHECK(r->bytes_outside_cs == 0);
        CHECK(r->max_chunk == 512);
        CHECK(r->xfers == (int)((len + 4 + 511) / 512));
        CHECK(r->last_progress == len);
        free(r->src); free(r);
    }
    /* 2: header only (a short command frame), and hold CS */
    {
        rec_t *r = mk(0);
        spi_stream_io_t io = io_of(r, 512);
        const uint8_t head[4] = { 0xC6, 0, 0, 0 };
        int rc = spi_stream_run(&io, head, 4, 0, true, NULL);
        CHECK(rc == SPI_STREAM_OK);
        CHECK(r->out_len == 4 && r->xfers == 1);
        CHECK(r->cs == 1 && r->cs_releases == 0);
        free(r->src); free(r);
    }
    /* 3: the transport fails half-way: error, CS released, `sent` counts what went */
    {
        rec_t *r = mk(10000);
        r->fail_xfer_at = 5;
        spi_stream_io_t io = io_of(r, 512);
        uint32_t sent = 0;
        int rc = spi_stream_run(&io, NULL, 0, 10000, true, &sent);
        CHECK(rc == SPI_STREAM_E_IO);
        CHECK(sent == 4 * 512);
        CHECK(r->cs == 0);
        free(r->src); free(r);
    }
    /* 4: the source fails: error, CS released, nothing more sent */
    {
        rec_t *r = mk(10000);
        r->fail_read_at = 3;
        spi_stream_io_t io = io_of(r, 512);
        uint32_t sent = 0;
        int rc = spi_stream_run(&io, NULL, 0, 10000, false, &sent);
        CHECK(rc == SPI_STREAM_E_READ);
        CHECK(sent == 2 * 512 && r->out_len == 2 * 512);
        CHECK(r->cs == 0);
        free(r->src); free(r);
    }
    /* 5: bad arguments: nothing touched */
    {
        rec_t *r = mk(100);
        spi_stream_io_t io = io_of(r, 512);
        uint8_t big[SPI_STREAM_HEAD_MAX + 1] = {0};
        CHECK(spi_stream_run(&io, big, sizeof(big), 100, false, NULL) == SPI_STREAM_E_ARGS);
        CHECK(spi_stream_run(&io, NULL, 0, 0, false, NULL) == SPI_STREAM_E_ARGS);
        spi_stream_io_t small = io_of(r, 16);
        CHECK(spi_stream_run(&small, NULL, 0, 100, false, NULL) == SPI_STREAM_E_ARGS);
        CHECK(r->cs_asserts == 0 && r->xfers == 0);
        free(r->src); free(r);
    }
    /* 6: second run with another chunk size and length: exact again */
    {
        uint32_t len = 1000;
        rec_t *r = mk(len);
        spi_stream_io_t io = io_of(r, 100);
        const uint8_t head[3] = { 1, 2, 3 };
        int rc = spi_stream_run(&io, head, 3, len, false, NULL);
        CHECK(rc == SPI_STREAM_OK && r->out_len == len + 3 && r->max_chunk == 100);
        CHECK(memcmp(r->out + 3, r->src, len) == 0);
        free(r->src); free(r);
    }
    if (fails) { printf("FAIL test_spi_stream: %d checks\n", fails); return 1; }
    printf("PASS test_spi_stream\n");
    return 0;
}
