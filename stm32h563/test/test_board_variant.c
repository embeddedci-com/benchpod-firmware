/* Host unit test for board_variant: an analog board is recognised by its I2C expanders, a
   missed ACK does not turn it into a digital board, and a board without them is digital.
   Digital v3r3: the analog module is recognised by MOD_PRESENT# and U58 on the module bus, and
   never by anything on the main bus. */
#include "board_variant.h"
#include "i2c_bus.h"
#include "module_link.h"
#include <stdio.h>
#include <string.h>

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

/* Used only by board_variant_init, board_variant_module_reread and board_has_wifi_module. */
static bool g_link, g_present, g_main_ana, g_mod_ana, g_ee_low, g_ee_100r;
static i2c_bus_id_t g_ana_bus;
bool i2c_bus_probe(uint8_t addr) { (void)addr; return g_main_ana; }
bool i2c_bus_probe_on(i2c_bus_id_t bus, uint8_t addr) { (void)bus; (void)addr; return g_mod_ana; }
void analog_switch_set_bus(i2c_bus_id_t bus) { g_ana_bus = bus; }
bool module_link_board(void) { return g_link; }
bool module_present(void) { return g_present; }
bool module_power_on(void) { return true; }
/* The EEPROM getters as module_link.c has them: only with the link board and a module. */
bool module_adc_low_range(void) { return g_link && g_present && g_ee_low; }
bool module_current_in_100r(void) { return g_link && g_present && g_ee_100r; }

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

static bool analog_is(bool present, bool module, bool low, bool r100)
{
    const board_analog_caps_t *a = &board_caps()->analog;
    return a->present == present && a->module == module && a->adc_low_range == low
        && a->current_in_100r == r100;
}

/* board_caps()->analog after board_variant_init: the decision plus the module EEPROM facts, which
   count only with the module; module_reread picks up a header read again. */
static void test_analog_caps(void)
{
    /* The pure map. */
    for (int k = BOARD_ANALOG_NONE; k <= BOARD_ANALOG_MODULE; k++)
        for (int e = 0; e < 4; e++) {
            const board_analog_caps_t a = board_analog_caps_for((board_analog_t)k, e & 1, e & 2);
            const bool mod = k == BOARD_ANALOG_MODULE;
            CHECK(a.present == (k != BOARD_ANALOG_NONE) && a.module == mod
                  && a.adc_low_range == (mod && (e & 1)) && a.current_in_100r == (mod && (e & 2)),
                  "caps_for kind %d ee %d", k, e);
        }

    /* One-piece analog board: on the board, no module facts even if a stray EEPROM said so. */
    g_link = false; g_present = false; g_main_ana = true; g_mod_ana = false; g_ee_low = g_ee_100r = true;
    board_variant_init();
    CHECK(analog_is(true, false, false, false), "one-piece analog board");
    CHECK(g_ana_bus == I2C_BUS_MAIN, "one-piece: expanders on the main bus");
    CHECK(strcmp(board_variant_str(), "analog") == 0, "one-piece: analog");

    /* Digital-only board. */
    g_main_ana = false;
    board_variant_init();
    CHECK(analog_is(false, false, false, false), "digital-only board");
    CHECK(strcmp(board_variant_str(), "digital") == 0, "digital-only: digital");

    /* Digital v3r3 with the module: its EEPROM facts. */
    g_link = true; g_present = true; g_mod_ana = true; g_ee_low = true; g_ee_100r = false;
    board_variant_init();
    CHECK(analog_is(true, true, true, false), "module v3r4 (low range, 249 ohm)");
    CHECK(g_ana_bus == I2C_BUS_MODULE, "module: expanders on the module bus");

    /* The module's power came back with a newer header. */
    g_ee_100r = true;
    CHECK(analog_is(true, true, true, false), "a header change alone latches nothing");
    board_variant_module_reread();
    CHECK(analog_is(true, true, true, true), "module_reread latches the new header");

    /* A module that is present but whose U58 does not answer: no analog, no module facts. */
    g_mod_ana = false;
    board_variant_init();
    CHECK(analog_is(false, false, false, false), "module without U58");

    /* No module on digital v3r3. */
    g_present = false; g_mod_ana = true;
    board_variant_init();
    CHECK(analog_is(false, false, false, false), "digital v3r3 without a module");
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
    CHECK(board_caps()->analog.present, "default is analog");
    CHECK(!board_caps()->analog.module, "default is not the module");

    test_module();
    test_analog_caps();

    if (fails) { printf("test_board_variant: %d failure(s)\n", fails); return 1; }
    printf("test_board_variant: all passed\n");
    return 0;
}
