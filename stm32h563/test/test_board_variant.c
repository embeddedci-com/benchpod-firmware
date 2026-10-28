/* Host unit test for board_variant: an analog board is recognised by its I2C expanders, a
   missed ACK does not turn it into a digital board, and a board without them is digital.
   Digital v3r3: the analog module is recognised by MOD_PRESENT# and U58 on the module bus, and
   never by anything on the main bus. */
#include "board_variant.h"
#include "i2c_bus.h"
#include "module_link.h"
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

/* The module bus: U58 there, and how many probes it loses first.  Counts every probe, so a test
   can see which bus was looked at. */
static bool g_mod_u58;
static int  g_mod_miss, g_mod_calls;

static bool fake_probe_module(uint8_t addr)
{
    g_mod_calls++;
    if (addr != I2C_ADDR_TCA9554_ANASW) return false;
    if (g_mod_miss > 0) { g_mod_miss--; return false; }
    return g_mod_u58;
}

static void modbus(bool u58, int miss) { g_mod_u58 = u58; g_mod_miss = miss; g_mod_calls = 0; }

/* Used only by board_variant_init and board_has_wifi_module. */
static bool g_link;
bool i2c_bus_probe(uint8_t addr) { (void)addr; return false; }
bool i2c_bus_probe_on(i2c_bus_id_t bus, uint8_t addr) { (void)bus; (void)addr; return false; }
void analog_switch_set_bus(i2c_bus_id_t bus) { (void)bus; }
bool module_link_board(void) { return g_link; }
bool module_present(void) { return false; }
bool module_power_on(void) { return true; }

static void test_module(void)
{
    board_variant_seen_t seen;

    /* One-piece boards: the main bus decides, the module bus is never touched. */
    bus(true, true, 0, 0); modbus(true, 0);
    CHECK(board_variant_decide(false, false, fake_probe, fake_probe_module, &seen) == BOARD_ANALOG_ONBOARD,
          "one-piece analog board");
    CHECK(seen.u55 && seen.u58 && !seen.module_u58 && g_mod_calls == 0, "one-piece: module bus untouched");
    bus(false, true, 0, 0); modbus(false, 0);
    CHECK(board_variant_decide(false, false, fake_probe, fake_probe_module, &seen) == BOARD_ANALOG_ONBOARD,
          "analog v3r2 (U58 only)");
    bus(false, false, 0, 0);
    CHECK(board_variant_decide(false, false, fake_probe, fake_probe_module, NULL) == BOARD_ANALOG_NONE,
          "digital-v3/v3r2");
    /* A low header pin on an old board (present) without the link pins changes nothing. */
    bus(false, false, 0, 0); modbus(true, 0);
    CHECK(board_variant_decide(false, true, fake_probe, fake_probe_module, NULL) == BOARD_ANALOG_NONE &&
          g_mod_calls == 0, "old board with PD9 low: still digital, module bus untouched");

    /* Digital v3r3 + module: the module bus decides, the main bus is never probed. */
    bus(true, true, 0, 0); modbus(true, 0);
    CHECK(board_variant_decide(true, true, fake_probe, fake_probe_module, &seen) == BOARD_ANALOG_MODULE,
          "digital v3r3 + module");
    CHECK(seen.module_u58 && !seen.u55 && !seen.u58 && g_calls == 0, "v3r3: main bus not probed");
    CHECK(g_mod_calls == 1, "v3r3 + module: one probe, %d", g_mod_calls);

    /* Missed ACKs on the module bus do not hide the module. */
    modbus(true, (int)BOARD_VARIANT_PROBE_TRIES - 1);
    CHECK(board_variant_decide(true, true, fake_probe, fake_probe_module, NULL) == BOARD_ANALOG_MODULE,
          "module with missed ACKs");
    modbus(true, (int)BOARD_VARIANT_PROBE_TRIES);
    CHECK(board_variant_decide(true, true, fake_probe, fake_probe_module, NULL) == BOARD_ANALOG_NONE,
          "module that never ACKs within the tries");

    /* Plugged in but U58 silent (unpowered, broken FFC): no analog front end. */
    bus(true, true, 0, 0); modbus(false, 0);
    CHECK(board_variant_decide(true, true, fake_probe, fake_probe_module, &seen) == BOARD_ANALOG_NONE,
          "module present, U58 silent");
    CHECK(g_mod_calls == (int)BOARD_VARIANT_PROBE_TRIES && g_calls == 0, "U58 silent: every try, main untouched");

    /* No module: nothing probed at all. */
    modbus(true, 0); bus(true, true, 0, 0);
    CHECK(board_variant_decide(true, false, fake_probe, fake_probe_module, NULL) == BOARD_ANALOG_NONE &&
          g_mod_calls == 0 && g_calls == 0, "digital v3r3 without a module");

    /* The ESP32-C3 is gone from digital v3r3 only. */
    g_link = false;
    CHECK(board_has_wifi_module(), "earlier boards have the C3");
    g_link = true;
    CHECK(!board_has_wifi_module(), "digital v3r3 has no C3");
    g_link = false;
}

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

    CHECK(!board_analog_is_module(), "default is not the module");

    test_module();

    if (fails) { printf("test_board_variant: %d failure(s)\n", fails); return 1; }
    printf("test_board_variant: all passed\n");
    return 0;
}
