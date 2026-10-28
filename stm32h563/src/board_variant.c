/*
 * board_variant.c — analog or digital-only BenchPod, from its I2C expanders or the analog module.
 * See board_variant.h.
 */
#include "board_variant.h"
#include "i2c_bus.h"
#include "module_link.h"
#include "bp_log.h"
#include <stdio.h>

static board_analog_t s_analog = BOARD_ANALOG_ONBOARD;

bool board_variant_detect(bool (*probe)(uint8_t addr), bool *u55, bool *u58)
{
    bool a = false, b = false;
    for (unsigned i = 0; i < BOARD_VARIANT_PROBE_TRIES && !(a && b); i++) {
        if (!a) a = probe(I2C_ADDR_TCA9554_DACMUX);
        if (!b) b = probe(I2C_ADDR_TCA9554_ANASW);
    }
    if (u55) *u55 = a;
    if (u58) *u58 = b;
    return a || b;
}

board_analog_t board_variant_decide(bool link_board, bool present,
                                    bool (*probe_main)(uint8_t addr),
                                    bool (*probe_module)(uint8_t addr),
                                    board_variant_seen_t *seen)
{
    board_variant_seen_t s = { false, false, false };
    board_analog_t r = BOARD_ANALOG_NONE;
    if (link_board) {
        if (present) {
            for (unsigned i = 0; i < BOARD_VARIANT_PROBE_TRIES && !s.module_u58; i++)
                s.module_u58 = probe_module(I2C_ADDR_TCA9554_ANASW);
            if (s.module_u58) r = BOARD_ANALOG_MODULE;
        }
    } else if (board_variant_detect(probe_main, &s.u55, &s.u58)) {
        r = BOARD_ANALOG_ONBOARD;
    }
    if (seen) *seen = s;
    return r;
}

static bool probe_module_bus(uint8_t addr) { return i2c_bus_probe_on(I2C_BUS_MODULE, addr); }

void board_variant_init(void)
{
    board_variant_seen_t seen;
    bool link = module_link_board(), present = module_present();
    s_analog = board_variant_decide(link, present, i2c_bus_probe, probe_module_bus, &seen);
    analog_switch_set_bus(s_analog == BOARD_ANALOG_MODULE ? I2C_BUS_MODULE : I2C_BUS_MAIN);

    if (link) {
        if (s_analog == BOARD_ANALOG_MODULE) {
            log_printf("[board] digital v3r3 with the analog module (U58 at 0x%02x on I2C2)\r\n",
                   I2C_ADDR_TCA9554_ANASW);
        } else if (present) {
            log_printf("[board] WARNING: digital v3r3: a module is plugged in (MOD_PRESENT# low) but "
                   "U58 does not answer at 0x%02x on I2C2; running without the analog front end%s\r\n",
                   I2C_ADDR_TCA9554_ANASW, module_power_on() ? "" : " (module power is off)");
        } else {
            log_printf("[board] digital v3r3, no module: DAC, ADC and the analog outputs are off "
                   "(restart after plugging in the analog module)\r\n");
        }
    } else if (s_analog == BOARD_ANALOG_NONE) {
        log_printf("[board] digital-only BenchPod (no analog expanders at 0x%02x/0x%02x): "
               "DAC, ADC and the analog outputs are off (restart after fitting an analog add-on)\r\n",
               I2C_ADDR_TCA9554_DACMUX, I2C_ADDR_TCA9554_ANASW);
    } else if (!(seen.u55 && seen.u58)) {
        log_printf("[board] analog board, but only the %s expander answers (0x%02x)%s\r\n",
               seen.u55 ? "DAC mux" : "analog switching",
               seen.u55 ? I2C_ADDR_TCA9554_DACMUX : I2C_ADDR_TCA9554_ANASW,
               seen.u55 ? "; WARNING: check U58" : " (no U55: expected on analog v3r2)");
    } else {
        log_printf("[board] analog front end present\r\n");
    }
}

bool board_has_analog(void) { return s_analog != BOARD_ANALOG_NONE; }

bool board_analog_is_module(void) { return s_analog == BOARD_ANALOG_MODULE; }

const char *board_variant_str(void) { return board_has_analog() ? "analog" : "digital"; }

bool board_has_wifi_module(void) { return !module_link_board(); }
