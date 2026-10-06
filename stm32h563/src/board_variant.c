/*
 * board_variant.c — analog or digital-only BenchPod, from its I2C expanders. See board_variant.h.
 */
#include "board_variant.h"
#include "i2c_bus.h"
#include <stdio.h>

static bool s_has_analog = true;

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

void board_variant_init(void)
{
    bool u55 = false, u58 = false;
    s_has_analog = board_variant_detect(i2c_bus_probe, &u55, &u58);
    if (!s_has_analog) {
        printf("[board] digital-only BenchPod (no analog expanders at 0x%02x/0x%02x): "
               "DAC, ADC and the analog outputs are off (restart after fitting an analog add-on)\r\n",
               I2C_ADDR_TCA9554_DACMUX, I2C_ADDR_TCA9554_ANASW);
    } else if (!(u55 && u58)) {
        printf("[board] WARNING: analog board, but only the %s expander answers (0x%02x); "
               "check U55/U58\r\n", u55 ? "DAC mux" : "analog switching",
               u55 ? I2C_ADDR_TCA9554_ANASW : I2C_ADDR_TCA9554_DACMUX);
    } else {
        printf("[board] analog front end present\r\n");
    }
}

bool board_has_analog(void) { return s_has_analog; }

const char *board_variant_str(void) { return s_has_analog ? "analog" : "digital"; }
