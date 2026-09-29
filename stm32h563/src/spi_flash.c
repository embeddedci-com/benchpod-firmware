/* spi_flash.c — see spi_flash.h. */
#include "spi_flash.h"

#include <string.h>

#define OP_WREN   0x06u
#define OP_RDSR        0x05u
#define OP_READ        0x03u
#define OP_PP          0x02u
#define OP_SE_4K       0x20u
#define OP_BE_64K      0xD8u
#define OP_CE          0xC7u
#define OP_JEDEC_ID    0x9Fu

#define SR_WIP         0x01u
#define SR_WEL         0x02u

/* Worst cases from the W25Q64JV / MX25L6433F / GD25Q64C tables, with margin. */
#define TIMEOUT_PP_US      20000u
#define TIMEOUT_SE_US    2000000u
#define TIMEOUT_BE_US    5000000u
#define POLL_SLEEP_ERASE_US  1000u

#define CHUNK_MAX 512u   /* staging buffer: the pod's gateware queue depth */

static uint8_t s_tx[CHUNK_MAX];
static uint8_t s_rx[CHUNK_MAX];

const char *spi_flash_strerror(int rc) {
    switch (rc) {
    case SPI_FLASH_OK:        return "ok";
    case SPI_FLASH_E_IO:      return "spi transfer failed";
    case SPI_FLASH_E_TIMEOUT: return "flash stayed busy (WIP never cleared)";
    case SPI_FLASH_E_ARGS:    return "invalid address or length (3-byte addressing: first 16 MB)";
    case SPI_FLASH_E_VERIFY:  return "verify failed: read-back differs (not erased, write-protected, or a bad wire)";
    case SPI_FLASH_E_WEL:     return "write enable did not stick: no flash answering, or it is write-protected";
    default:                  return "spi flash error";
    }
}

static size_t chunk_max(const spi_flash_io_t *io) {
    return io->max_xfer < CHUNK_MAX ? io->max_xfer : CHUNK_MAX;
}

/* One command inside a single CS assertion: hdr[0..hdr_n) then n data bytes, where the data
   is `out` (NULL = zeros) and what comes back during the data phase lands in `in` (may be
   NULL).  Split into transport-sized pieces with CS held. */
static int command(const spi_flash_io_t *io, const uint8_t *hdr, size_t hdr_n,
                   const uint8_t *out, uint8_t *in, size_t n) {
    const size_t cmax = chunk_max(io);
    size_t hdr_done = 0, done = 0;
    int rc = 0;
    io->cs(io->ctx, true);
    while (rc == 0 && (hdr_done < hdr_n || done < n)) {
        size_t k = 0;
        while (hdr_done < hdr_n && k < cmax) s_tx[k++] = hdr[hdr_done++];
        size_t data_at = k;
        size_t take = n - done;
        if (take > cmax - k) take = cmax - k;
        if (out) memcpy(s_tx + k, out + done, take);
        else     memset(s_tx + k, 0, take);
        k += take;
        if (io->xfer(io->ctx, s_tx, in ? s_rx : NULL, k) != 0) { rc = SPI_FLASH_E_IO; break; }
        if (in) memcpy(in + done, s_rx + data_at, take);
        done += take;
    }
    io->cs(io->ctx, false);
    return rc;
}

static int simple(const spi_flash_io_t *io, uint8_t op) {
    return command(io, &op, 1, NULL, NULL, 0);
}

static void addr_hdr(uint8_t *h, uint8_t op, uint32_t addr) {
    h[0] = op;
    h[1] = (uint8_t)(addr >> 16);
    h[2] = (uint8_t)(addr >> 8);
    h[3] = (uint8_t)addr;
}

static bool range_ok(uint32_t addr, size_t len) {
    return len > 0 && addr < SPI_FLASH_ADDR_LIMIT && len <= SPI_FLASH_ADDR_LIMIT - addr;
}

int spi_flash_read_id(const spi_flash_io_t *io, uint8_t id[3]) {
    uint8_t op = OP_JEDEC_ID;
    return command(io, &op, 1, NULL, id, 3);
}

bool spi_flash_id_valid(const uint8_t id[3]) {
    bool all0 = id[0] == 0x00 && id[1] == 0x00 && id[2] == 0x00;
    bool all1 = id[0] == 0xFF && id[1] == 0xFF && id[2] == 0xFF;
    return !all0 && !all1;
}

uint32_t spi_flash_capacity(const uint8_t id[3]) {
    return (id[2] >= 0x10 && id[2] <= 0x19) ? (1u << id[2]) : 0u;
}

int spi_flash_read_status(const spi_flash_io_t *io, uint8_t *sr) {
    uint8_t op = OP_RDSR;
    return command(io, &op, 1, NULL, sr, 1);
}

static int wait_idle(const spi_flash_io_t *io, uint32_t timeout_us, uint32_t sleep_us) {
    uint64_t deadline = io->now_us(io->ctx) + timeout_us;
    for (;;) {
        uint8_t sr = 0xFF;
        int rc = spi_flash_read_status(io, &sr);
        if (rc) return rc;
        if (!(sr & SR_WIP)) return SPI_FLASH_OK;
        if (io->now_us(io->ctx) > deadline) return SPI_FLASH_E_TIMEOUT;
        if (sleep_us) io->sleep_us(io->ctx, sleep_us);
    }
}

/* WREN, and check it took: an empty bus or a protected part never sets WEL, and a program or
   erase sent then would do nothing and "succeed". */
static int write_enable(const spi_flash_io_t *io) {
    int rc = simple(io, OP_WREN);
    if (rc) return rc;
    uint8_t sr = 0;
    rc = spi_flash_read_status(io, &sr);
    if (rc) return rc;
    return ((sr & (SR_WEL | SR_WIP)) == SR_WEL) ? SPI_FLASH_OK : SPI_FLASH_E_WEL;
}

int spi_flash_read(const spi_flash_io_t *io, uint32_t addr, uint8_t *buf, size_t len) {
    if (!buf || !range_ok(addr, len)) return SPI_FLASH_E_ARGS;
    uint8_t h[4];
    addr_hdr(h, OP_READ, addr);
    return command(io, h, 4, NULL, buf, len);
}

static int erase_one(const spi_flash_io_t *io, uint8_t op, uint32_t addr, uint32_t timeout_us) {
    int rc = write_enable(io);
    if (rc) return rc;
    uint8_t h[4];
    addr_hdr(h, op, addr);
    rc = command(io, h, 4, NULL, NULL, 0);
    if (rc) return rc;
    return wait_idle(io, timeout_us, POLL_SLEEP_ERASE_US);
}

int spi_flash_erase(const spi_flash_io_t *io, uint32_t addr, uint32_t len,
                    uint32_t *done_start, uint32_t *done_len) {
    if (!range_ok(addr, len)) return SPI_FLASH_E_ARGS;
    uint32_t a   = addr & ~(SPI_FLASH_SECTOR - 1u);
    uint32_t end = addr + len;
    end = (end + SPI_FLASH_SECTOR - 1u) & ~(SPI_FLASH_SECTOR - 1u);
    if (done_start) *done_start = a;
    if (done_len)   *done_len   = 0;
    while (a < end) {
        int rc;
        uint32_t step;
        if ((a & (SPI_FLASH_BLOCK - 1u)) == 0 && end - a >= SPI_FLASH_BLOCK) {
            rc = erase_one(io, OP_BE_64K, a, TIMEOUT_BE_US);
            step = SPI_FLASH_BLOCK;
        } else {
            rc = erase_one(io, OP_SE_4K, a, TIMEOUT_SE_US);
            step = SPI_FLASH_SECTOR;
        }
        if (rc) return rc;
        a += step;
        if (done_len) *done_len = a - (done_start ? *done_start : 0);
    }
    return SPI_FLASH_OK;
}

int spi_flash_chip_erase(const spi_flash_io_t *io, uint32_t timeout_ms) {
    int rc = write_enable(io);
    if (rc) return rc;
    rc = simple(io, OP_CE);
    if (rc) return rc;
    return wait_idle(io, timeout_ms * 1000u, 10u * POLL_SLEEP_ERASE_US);
}

int spi_flash_program(const spi_flash_io_t *io, uint32_t addr, const uint8_t *data, size_t len) {
    if (!data || !range_ok(addr, len)) return SPI_FLASH_E_ARGS;
    size_t done = 0;
    while (done < len) {
        uint32_t a = addr + (uint32_t)done;
        size_t n = SPI_FLASH_PAGE - (a & (SPI_FLASH_PAGE - 1u));   /* to the page end */
        if (n > len - done) n = len - done;
        int rc = write_enable(io);
        if (rc) return rc;
        uint8_t h[4];
        addr_hdr(h, OP_PP, a);
        rc = command(io, h, 4, data + done, NULL, n);
        if (rc) return rc;
        rc = wait_idle(io, TIMEOUT_PP_US, 0);
        if (rc) return rc;
        done += n;
    }
    return SPI_FLASH_OK;
}

int spi_flash_verify(const spi_flash_io_t *io, uint32_t addr, const uint8_t *data, size_t len,
                     uint32_t *bad_at) {
    if (!data || !range_ok(addr, len)) return SPI_FLASH_E_ARGS;
    static uint8_t rb[CHUNK_MAX];
    size_t done = 0;
    while (done < len) {
        size_t n = len - done;
        if (n > sizeof(rb)) n = sizeof(rb);
        int rc = spi_flash_read(io, addr + (uint32_t)done, rb, n);
        if (rc) return rc;
        for (size_t i = 0; i < n; i++) {
            if (rb[i] != data[done + i]) {
                if (bad_at) *bad_at = addr + (uint32_t)(done + i);
                return SPI_FLASH_E_VERIFY;
            }
        }
        done += n;
    }
    return SPI_FLASH_OK;
}
