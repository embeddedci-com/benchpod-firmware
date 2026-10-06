/* Host unit test for board_variant: an analog board is recognised by its I2C expanders, a
   missed ACK does not turn it into a digital board, and a board without them is digital. */
#include "board_variant.h"
#include "i2c_bus.h"
#include <stdio.h>

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* The fake bus: which expanders exist, and how many probes each loses first. */
static bool g_u55, g_u58;
static int  g_miss55, g_miss58, g_calls;

static bool fake_probe(uint8_t addr)
{
    g_calls++;
    if (addr == I2C_ADDR_TCA9554_DACMUX) {
        if (g_miss55 > 0) { g_miss55--; return false; }
        return g_u55;
    }
    if (addr == I2C_ADDR_TCA9554_ANASW) {
        if (g_miss58 > 0) { g_miss58--; return false; }
        return g_u58;
    }
    return false;
}

static void bus(bool u55, bool u58, int miss55, int miss58)
{
    g_u55 = u55; g_u58 = u58; g_miss55 = miss55; g_miss58 = miss58; g_calls = 0;
}

/* i2c_bus_probe is only used by board_variant_init, which these tests do not call. */
bool i2c_bus_probe(uint8_t addr) { (void)addr; return false; }

int main(void)
{
    bool a, b;

    bus(true, true, 0, 0);
    CHECK(board_variant_detect(fake_probe, &a, &b) && a && b, "analog board: both expanders answer");
    CHECK(g_calls == 2, "analog board: one probe each, %d calls", g_calls);

    bus(false, false, 0, 0);
    CHECK(!board_variant_detect(fake_probe, &a, &b) && !a && !b, "digital board: no expanders");
    CHECK(g_calls == 2 * (int)BOARD_VARIANT_PROBE_TRIES, "digital board: every try made, %d calls", g_calls);

    /* A glitch on the first tries must not hide the analog front end. */
    bus(true, true, 2, 2);
    CHECK(board_variant_detect(fake_probe, &a, &b) && a && b, "analog board with two missed ACKs each");

    /* One expander dead: still an analog board (the other proves the section is there). */
    bus(true, false, 0, 0);
    CHECK(board_variant_detect(fake_probe, &a, &b) && a && !b, "only U55 answers");
    bus(false, true, 0, 0);
    CHECK(board_variant_detect(fake_probe, &a, &b) && !a && b, "only U58 answers");

    /* NULL out-pointers are allowed. */
    bus(true, true, 0, 0);
    CHECK(board_variant_detect(fake_probe, NULL, NULL), "NULL out-pointers");

    /* Before board_variant_init the board counts as analog: nothing is hidden early. */
    CHECK(board_has_analog(), "default is analog");

    if (fails) { printf("test_board_variant: %d failure(s)\n", fails); return 1; }
    printf("test_board_variant: all passed\n");
    return 0;
}
