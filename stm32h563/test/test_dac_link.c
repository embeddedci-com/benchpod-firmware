/* Host unit test for dac_link.c: which digital board / analog module pairs may drive the DAC,
 * which commands are refused on a mismatched pair, and which iCE40 commands reach the DAC. */
#include "dac_link.h"
#include "board_caps.h"
#include "board_rev.h"
#include "cmd_opcodes.h"
#include "module_eeprom.h"

#include <stdio.h>
#include <string.h>

static int fails = 0, checks = 0;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* The module side as the firmware reads it: an EEPROM result and, when it decoded, a revision. */
typedef struct { const char *name; int rc; uint8_t major, minor; bool swap; } module_case_t;

static const module_case_t k_modules[] = {
    { "2.9",       MOD_EE_OK,        2, 9, false },
    { "3.3",       MOD_EE_OK,        3, 3, false },
    { "3.4",       MOD_EE_OK,        3, 4, false },
    { "3.5",       MOD_EE_OK,        3, 5, false },
    { "3.6",       MOD_EE_OK,        3, 6, false },
    { "4.0",       MOD_EE_OK,        4, 0, true  },
    { "4.1",       MOD_EE_OK,        4, 1, true  },
    { "blank",     MOD_EE_BLANK,     4, 0, false },   /* rev fields ignored: never written */
    { "bad crc",   MOD_EE_BAD_CRC,   4, 0, false },
    { "bad magic", MOD_EE_BAD_MAGIC, 4, 0, false },
    { "no chip",   -10,              4, 0, false },   /* MOD_EE_NO_CHIP (module_link.h) */
};

static void test_module_revision(void)
{
    for (size_t i = 0; i < sizeof(k_modules) / sizeof(k_modules[0]); i++) {
        module_id_t id = { 0 };
        id.type = MOD_TYPE_ANALOG;
        id.rev_major = k_modules[i].major; id.rev_minor = k_modules[i].minor;
        CHECK(module_ee_has_cn1_dac_swap(k_modules[i].rc, &id) == k_modules[i].swap,
              "module %s: clock on pin %d", k_modules[i].name, k_modules[i].swap ? 7 : 8);
    }
    module_id_t other = { 0 };
    other.type = 7; other.rev_major = 4; other.rev_minor = 0;
    CHECK(!module_ee_has_cn1_dac_swap(MOD_EE_OK, &other), "not an analog module: older pinout");
    CHECK(!module_ee_has_cn1_dac_swap(MOD_EE_OK, NULL), "no header: older pinout");
}

static void test_digital_side(void)
{
    CHECK(board_caps_for_rev(BOARD_REV_V4).cn1_dac_swap, "digital v4: clock on pin 7");
    const int older[] = { BOARD_REV_V2, BOARD_REV_V3, BOARD_REV_V3R2, BOARD_REV_UNKNOWN };
    for (size_t i = 0; i < sizeof(older) / sizeof(older[0]); i++)
        CHECK(!board_caps_for_rev(older[i]).cn1_dac_swap, "rev %d: clock on pin 8", older[i]);
}

/* Every digital board x every module x module present or not x two-board pod or not. */
static void test_pairs(void)
{
    const struct { const char *name; int rev; } digital[] = {
        { "v3r3", BOARD_REV_V3 }, { "v4", BOARD_REV_V4 },
    };
    for (size_t d = 0; d < 2; d++) {
        bool dswap = board_caps_for_rev(digital[d].rev).cn1_dac_swap;
        for (size_t m = 0; m < sizeof(k_modules) / sizeof(k_modules[0]); m++) {
            module_id_t id = { 0 };
            id.type = MOD_TYPE_ANALOG;
            id.rev_major = k_modules[m].major; id.rev_minor = k_modules[m].minor;
            bool mswap = module_ee_has_cn1_dac_swap(k_modules[m].rc, &id);
            bool want = (d == 1) != k_modules[m].swap;   /* new digital needs a v4+ module */
            CHECK(dac_link_mismatch(true, true, dswap, mswap) == want,
                  "digital %s + module %s: %s", digital[d].name, k_modules[m].name,
                  want ? "refused" : "allowed");
            CHECK(!dac_link_mismatch(true, false, dswap, mswap),
                  "digital %s, no module at boot: nothing to refuse", digital[d].name);
            CHECK(!dac_link_mismatch(false, true, dswap, mswap),
                  "not a two-board pod (%s, module %s): never refused", digital[d].name,
                  k_modules[m].name);
        }
    }
}

static void test_commands(void)
{
    const char *always[] = { "generate", "measure", "replay", "dac_set", "dac_out", "dac_control_loop" };
    for (size_t i = 0; i < sizeof(always) / sizeof(always[0]); i++)
        CHECK(dac_link_command_drives_dac(always[i], "{}"), "%s drives the DAC", always[i]);

    CHECK(dac_link_command_drives_dac("current_out", "{\"cmd\":\"current_out\",\"ua\":4000}"),
          "current_out with ua");
    CHECK(!dac_link_command_drives_dac("current_out", "{\"cmd\":\"current_out\"}"), "bare current_out reads");
    CHECK(dac_link_command_drives_dac("calibrate", "{\"cmd\":\"calibrate\",\"dac\":\"5v\"}"), "calibrate dac");
    CHECK(dac_link_command_drives_dac("calibrate", "{\"cmd\":\"calibrate\",\"adc\":\"cal1\"}"), "calibrate adc");
    CHECK(!dac_link_command_drives_dac("calibrate", "{\"cmd\":\"calibrate\",\"source\":\"current_in\"}"),
          "calibrate current_in offset does not drive the DAC");
    CHECK(!dac_link_command_drives_dac("calibrate", "{\"cmd\":\"calibrate\",\"clear\":\"dac\"}"),
          "clearing the DAC calibration does not drive it");
    CHECK(!dac_link_command_drives_dac("calibrate", "{\"cmd\":\"calibrate\"}"), "calibrate read");
    CHECK(dac_link_command_drives_dac("adc_range", "{\"cmd\":\"adc_range\",\"test\":true}"), "adc_range test");
    CHECK(!dac_link_command_drives_dac("adc_range", "{\"cmd\":\"adc_range\",\"test\":false}"), "adc_range test false");
    CHECK(!dac_link_command_drives_dac("adc_range", "{\"cmd\":\"adc_range\",\"range\":\"low\"}"), "adc_range set");

    const char *never[] = { "dac_stop", "status", "capture", "adc_read", "analog_path", "dac_mux",
                            "cal_switch", "load", "load_bin", "dac_limits", "module", "la_voltage" };
    for (size_t i = 0; i < sizeof(never) / sizeof(never[0]); i++)
        CHECK(!dac_link_command_drives_dac(never[i], "{}"), "%s does not drive the DAC", never[i]);
    CHECK(!dac_link_command_drives_dac(NULL, "{}"), "no command");
}

static void test_opcodes(void)
{
    const uint8_t dac[] = { CMD_START_DAC, CMD_START_DAC_PSRAM, CMD_START_DAC_LOOP, CMD_DAC_CTRL };
    for (size_t i = 0; i < sizeof(dac); i++)
        CHECK(dac_link_opcode_drives_dac(dac[i]), "opcode 0x%02x sends DAC frames", dac[i]);
    /* Stopping, loading the waveform memory and reads stay allowed: no frame goes out. */
    const uint8_t other[] = { CMD_STOP_DAC, CMD_LOAD_WAVE, CMD_PING, CMD_VERSION, CMD_STATUS };
    for (size_t i = 0; i < sizeof(other); i++)
        CHECK(!dac_link_opcode_drives_dac(other[i]), "opcode 0x%02x sends no DAC frame", other[i]);
}

int main(void)
{
    test_module_revision();
    test_digital_side();
    test_pairs();
    test_commands();
    test_opcodes();
    printf("test_dac_link: %d checks, %s\n", checks, fails ? "FAILED" : "all passed");
    return fails ? 1 : 0;
}
