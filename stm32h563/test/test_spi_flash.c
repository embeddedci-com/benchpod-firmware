/* test_spi_flash.c — host tests for spi_flash.c against a byte-level SPI NOR model.
 *
 * The model decodes the byte stream per CS assertion the way a W25Q does: page program wraps
 * inside its 256-byte page and only clears bits, erases set to FF, WREN sets WEL and every
 * program/erase clears it, WIP stays set for a programmable number of RDSR polls.  Every
 * test also runs with a tiny transport (8 bytes per xfer) so commands split across xfers
 * with CS held are covered, not only the 512-byte pod transport.
 */
#include "spi_flash.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

/* ---- flash model --------------------------------------------------------- */
#define MEM_SIZE (1u << 20)                 /* 1 MB part: ID EF 40 14 */
static uint8_t  mem[MEM_SIZE];
static bool     cs_low, wel, protect, stuck;
static int      busy_polls;                 /* RDSR polls left reading WIP=1 */
static int      busy_after_write = 3;
static uint8_t  op;
static size_t   pos;                        /* bytes seen in this CS assertion */
static uint32_t addr;
static int      n_se, n_be, n_ce, n_pp, n_cs_cycles;
static size_t   max_xfer_seen;
static uint64_t now;

static void model_reset(void) {
    memset(mem, 0xFF, sizeof(mem));
    wel = protect = stuck = false;
    busy_polls = 0;
    n_se = n_be = n_ce = n_pp = n_cs_cycles = 0;
    max_xfer_seen = 0;
}

static void finish_command(void) {
    if (pos == 0) return;
    bool can_write = wel && !protect && busy_polls == 0;
    if (op == 0x02 && pos > 4) {
        if (can_write) n_pp++;
        wel = false; busy_polls = busy_after_write;
    } else if (op == 0x20 && pos == 4) {
        if (can_write) { memset(mem + (addr & ~0xFFFu), 0xFF, 4096); n_se++; }
        wel = false; busy_polls = busy_after_write;
    } else if (op == 0xD8 && pos == 4) {
        if (can_write) { memset(mem + (addr & ~0xFFFFu), 0xFF, 65536); n_be++; }
        wel = false; busy_polls = busy_after_write;
    } else if (op == 0xC7 && pos == 1) {
        if (can_write) { memset(mem, 0xFF, sizeof(mem)); n_ce++; }
        wel = false; busy_polls = busy_after_write;
    }
}

static uint8_t model_byte(uint8_t in) {
    uint8_t out = 0xFF;
    if (pos == 0) {
        op = in;
        if (op == 0x06 && busy_polls == 0 && !protect) wel = true;
    } else {
        switch (op) {
        case 0x9F: { static const uint8_t id[3] = { 0xEF, 0x40, 0x14 }; out = pos <= 3 ? id[pos - 1] : 0; break; }
        case 0x05: {
            bool wip = stuck || busy_polls > 0;
            out = (uint8_t)((wip ? 1u : 0u) | (wel ? 2u : 0u));
            if (busy_polls > 0) busy_polls--;
            break;
        }
        case 0x03:
            if (pos <= 3) addr = (addr << 8) | in;
            else out = mem[(addr + (uint32_t)(pos - 4)) % MEM_SIZE];
            break;
        case 0x02:
            if (pos <= 3) addr = (addr << 8) | in;
            else if (wel && !protect && busy_polls == 0) {
                uint32_t a = (addr & ~0xFFu) | ((addr + (uint32_t)(pos - 4)) & 0xFFu);   /* page wrap */
                mem[a % MEM_SIZE] &= in;
            }
            break;
        case 0x20: case 0xD8:
            if (pos <= 3) addr = (addr << 8) | in;
            break;
        default: break;
        }
    }
    pos++;
    return out;
}

static void io_cs(void *ctx, bool asserted) {
    (void)ctx;
    if (asserted) { cs_low = true; pos = 0; addr = 0; n_cs_cycles++; }
    else          { finish_command(); cs_low = false; }
}
static int io_xfer(void *ctx, const uint8_t *tx, uint8_t *rx, size_t n) {
    const spi_flash_io_t *io = ctx;
    if (!cs_low || n == 0 || n > io->max_xfer) return -1;
    if (n > max_xfer_seen) max_xfer_seen = n;
    for (size_t i = 0; i < n; i++) {
        uint8_t o = model_byte(tx[i]);
        if (rx) rx[i] = o;
    }
    now += 10 * n;                          /* ~10 us per byte */
    return 0;
}
static void     io_sleep(void *ctx, uint32_t us) { (void)ctx; now += us; }
static uint64_t io_now(void *ctx)                { (void)ctx; return now; }

static spi_flash_io_t make_io(size_t max_xfer) {
    spi_flash_io_t io = { NULL, io_cs, io_xfer, io_sleep, io_now, max_xfer };
    return io;
}

/* ---- tests ---------------------------------------------------------------- */

static uint8_t data[70000], back[70000];

static void test_suite(size_t max_xfer) {
    spi_flash_io_t io = make_io(max_xfer);
    io.ctx = &io;
    printf("  transport %zu bytes per xfer\n", max_xfer);

    model_reset();
    uint8_t id[3] = { 0 };
    CHECK(spi_flash_read_id(&io, id) == 0);
    CHECK(id[0] == 0xEF && id[1] == 0x40 && id[2] == 0x14);
    CHECK(spi_flash_id_valid(id));
    CHECK(spi_flash_capacity(id) == (1u << 20));
    CHECK(max_xfer_seen <= max_xfer);

    /* program across page boundaries: 0xF0 .. 0xF0+700 spans 4 pages */
    for (size_t i = 0; i < sizeof(data); i++) data[i] = (uint8_t)(i * 13 + 5);
    CHECK(spi_flash_program(&io, 0xF0, data, 700) == 0);
    CHECK(n_pp == 4);                      /* 16 + 256 + 256 + 172 */
    CHECK(memcmp(mem + 0xF0, data, 700) == 0);
    CHECK(mem[0xEF] == 0xFF && mem[0xF0 + 700] == 0xFF);
    CHECK(spi_flash_read(&io, 0xF0, back, 700) == 0);
    CHECK(memcmp(back, data, 700) == 0);
    uint32_t bad = 0;
    CHECK(spi_flash_verify(&io, 0xF0, data, 700, &bad) == 0);

    /* programming over non-erased data clears bits only: verify must catch it */
    uint8_t ones[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
    CHECK(spi_flash_program(&io, 0x100, ones, 4) == 0);
    CHECK(spi_flash_verify(&io, 0x100, ones, 4, &bad) == SPI_FLASH_E_VERIFY);
    CHECK(bad == 0x100 + (data[0x10] == 0xFF ? 1u : 0u));

    /* erase 0x0FFF..+0x11001: sector 0 (partial), sectors 1..15, block 1, then sector 0x20 */
    n_se = n_be = 0;
    uint32_t ds = 0, dl = 0;
    CHECK(spi_flash_erase(&io, 0x0FFF, 0x20002, &ds, &dl) == 0);
    CHECK(ds == 0x0000 && dl == 0x22000);
    CHECK(n_be == 2);                      /* blocks 0x00000 and 0x10000 (whole blocks in range) */
    CHECK(n_se == 2);                      /* sectors 0x20000 and 0x21000 */
    CHECK(mem[0xF0] == 0xFF && mem[0x21FFF] == 0xFF);

    /* a 64 KB-aligned 128 KB range is two block erases, nothing else */
    n_se = n_be = 0;
    memset(mem + 0x40000, 0x00, 0x20000);
    CHECK(spi_flash_erase(&io, 0x40000, 0x20000, NULL, NULL) == 0);
    CHECK(n_be == 2 && n_se == 0);
    CHECK(mem[0x40000] == 0xFF && mem[0x5FFFF] == 0xFF);

    /* a large program + verify: 64 KB + 4 KB, each page's WIP polled */
    CHECK(spi_flash_program(&io, 0x40000, data, 0x11000) == 0);
    CHECK(spi_flash_verify(&io, 0x40000, data, 0x11000, &bad) == 0);

    /* chip erase */
    CHECK(spi_flash_chip_erase(&io, 1000) == 0);
    CHECK(n_ce == 1 && mem[0x40000] == 0xFF);

    /* write-protected (or empty bus): WEL never sets, nothing is sent after it */
    protect = true;
    n_pp = 0;
    CHECK(spi_flash_program(&io, 0, data, 16) == SPI_FLASH_E_WEL);
    CHECK(spi_flash_erase(&io, 0, 4096, NULL, NULL) == SPI_FLASH_E_WEL);
    CHECK(n_pp == 0);
    protect = false;

    /* a flash that never finishes: time out instead of hanging */
    CHECK(spi_flash_program(&io, 0x80000, data, 16) == 0);
    stuck = true;
    CHECK(spi_flash_erase(&io, 0x80000, 4096, NULL, NULL) == SPI_FLASH_E_WEL);   /* WIP set: WEL check refuses */
    stuck = false;
    busy_after_write = 1000000;
    uint64_t t0 = now;
    CHECK(spi_flash_erase(&io, 0x80000, 4096, NULL, NULL) == SPI_FLASH_E_TIMEOUT);
    CHECK(now - t0 >= 2000000);            /* waited the sector timeout, not forever */
    busy_after_write = 3;
    busy_polls = 0;

    /* range checks: beyond 16 MB, zero length */
    CHECK(spi_flash_read(&io, 0xFFFFFF, back, 2) == SPI_FLASH_E_ARGS);
    CHECK(spi_flash_program(&io, 0, data, 0) == SPI_FLASH_E_ARGS);
    CHECK(spi_flash_erase(&io, 0x1000000, 1, NULL, NULL) == SPI_FLASH_E_ARGS);
}

int main(void) {
    test_suite(512);
    test_suite(8);
    spi_flash_id_valid((const uint8_t[3]){ 0, 0, 0 }) ? failures++ : 0;
    spi_flash_id_valid((const uint8_t[3]){ 0xFF, 0xFF, 0xFF }) ? failures++ : 0;
    if (failures) { printf("test_spi_flash: %d FAILURE(S)\n", failures); return 1; }
    printf("test_spi_flash: all passed\n");
    return 0;
}
