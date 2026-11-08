/* efuse_flt.c: tells a real eFuse fault from reverse-current blocking (target_power.h,
   target_power_set_flt_check), with the output's current monitor. */
#include "efuse_flt.h"
#include "target_power.h"
#include "ina238.h"
#include "i2c_bus.h"

bool efuse_flt_is_fault(int efuse, int out_mv, int out_ua)
{
    (void)efuse;
    /* A short circuit, an overtemperature shutdown or an ILM fault leaves the output collapsed;
       a persistent overload sits at the current limit (about 2 A on eFuse 1, 3 A on eFuse 2 with
       the board's ILM resistors). Reverse-current blocking leaves the output up with no current. */
    if (out_mv < EFUSE_FLT_COLLAPSED_MV) return true;
    if (out_ua > EFUSE_FLT_LOAD_UA) return true;
    return false;
}

static bool check(int efuse)
{
    int bus_mv = 0, shunt_uv = 0, cur_ua = 0;
    const uint8_t addr = efuse == 1 ? I2C_ADDR_INA238_INTERNAL : I2C_ADDR_INA238_EXTERNAL;
    if (ina238_read(addr, &bus_mv, &shunt_uv, &cur_ua) != 0) return true;   /* cannot tell: a fault */
    return efuse_flt_is_fault(efuse, bus_mv, cur_ua);
}

void efuse_flt_init(void) { target_power_set_flt_check(check); }
