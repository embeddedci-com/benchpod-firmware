/* analog_path.c — the analog path table behind analog_path_set (i2c_bus.h).  Pure: no HAL, so the
   switch state of every path is pinned by the host tests (test_analog_path). */
#include "i2c_bus.h"

#include <string.h>

#define MUX(e1, s1, e2, s2)       .mux = true, .ctrl1_en = (e1), .ctrl1_sel = (s1), \
                                  .ctrl2_en = (e2), .ctrl2_sel = (s2)
#define RELAYS(c1, c2, cur, cp)   .relays = true, .cal1 = (c1), .cal2 = (c2), \
                                  .current_in = (cur), .cal_path = (cp)

int analog_path_switches(analog_path_t path, bool quad, analog_switches_t *out)
{
    static const analog_switches_t tbl[ANALOG_PATH__COUNT] = {
        [ANALOG_PATH_OFF]         = { MUX(false, 0, false, 0), RELAYS(false, false, false, false) },
        [ANALOG_PATH_DAC_3V3]     = { MUX(true, DACMUX_CTRL1_3V3, false, 0), RELAYS(false, false, false, false) },
        [ANALOG_PATH_DAC_5V]      = { MUX(true, DACMUX_CTRL1_5V,  false, 0), RELAYS(false, false, false, false) },
        [ANALOG_PATH_DAC_12V]     = { MUX(true, DACMUX_CTRL1_12V, true, DACMUX_CTRL2_12V_VMID),
                                      RELAYS(false, false, false, false) },
        [ANALOG_PATH_ADC_EXT]     = { RELAYS(false, false, false, false) },        /* mux as-is */
        [ANALOG_PATH_CAL1]        = { MUX(true, DACMUX_CTRL1_5V, false, 0), RELAYS(true, false, false, true) },
        [ANALOG_PATH_CAL2]        = { MUX(true, DACMUX_CTRL1_12V_ADC, true, DACMUX_CTRL2_ADC_VMID),
                                      RELAYS(false, true, false, true) },
        [ANALOG_PATH_CURRENT_IN]  = { RELAYS(false, false, true, false) },         /* mux as-is */
        /* The XTR116 follows the DAC buffer whatever the mux does: this only switches the voltage
           outputs OFF so they do not follow the current.  The relays stay as they are. */
        [ANALOG_PATH_CURRENT_OUT] = { MUX(false, 0, false, 0) },
    };
    if (!out || (unsigned)path >= ANALOG_PATH__COUNT) return -1;
    *out = tbl[path];
    if (quad) out->mux = false;   /* analog v3r2: no U55; the DAC8554 channel replaces it */
    return 0;
}

uint8_t analog_calsw_reg(bool cal1, bool cal2, bool current_in, bool cal_path, bool ana14)
{
    return (uint8_t)((cal1 ? 0x01u : 0u) | (cal2 ? 0x02u : 0u) | (current_in ? 0x04u : 0u)
                     | (cal_path ? 0x08u : 0u) | (ana14 ? 0x10u : 0u));
}

static const struct { const char *name; analog_path_t path; } s_path_tbl[] = {
    { "off",     ANALOG_PATH_OFF     },
    { "dac_3v3", ANALOG_PATH_DAC_3V3 }, { "3v3", ANALOG_PATH_DAC_3V3 },
    { "dac_5v",  ANALOG_PATH_DAC_5V  }, { "5v",  ANALOG_PATH_DAC_5V  },
    { "dac_12v", ANALOG_PATH_DAC_12V }, { "12v", ANALOG_PATH_DAC_12V },
    { "adc_ext", ANALOG_PATH_ADC_EXT }, { "ext", ANALOG_PATH_ADC_EXT },
                                        { "sma", ANALOG_PATH_ADC_EXT },
    { "cal1",    ANALOG_PATH_CAL1    },
    { "cal2",    ANALOG_PATH_CAL2    },
    { "current_in",     ANALOG_PATH_CURRENT_IN     },
    { "current_out",    ANALOG_PATH_CURRENT_OUT    },
};

const char *analog_path_name(analog_path_t path)
{
    /* canonical names only (first hit per enum in the table) */
    static const char *canon[ANALOG_PATH__COUNT] = {
        "off", "dac_3v3", "dac_5v", "dac_12v", "adc_ext", "cal1", "cal2", "current_in",
        "current_out",
    };
    return (path < ANALOG_PATH__COUNT) ? canon[path] : "";
}

int analog_path_from_name(const char *name, analog_path_t *out)
{
    if (!name || !out) return -1;
    for (unsigned i = 0; i < sizeof(s_path_tbl) / sizeof(s_path_tbl[0]); i++) {
        if (strcmp(name, s_path_tbl[i].name) == 0) { *out = s_path_tbl[i].path; return 0; }
    }
    return -1;
}
