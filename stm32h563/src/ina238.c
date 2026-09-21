/*
 * ina238.c — TI INA238 current/power monitor driver.
 *
 * Registers (16-bit, big-endian):
 *   0x00 CONFIG     bit4 ADCRANGE (0 = ±163.84 mV, 5 µV/LSB)
 *   0x04 VSHUNT     signed, 5 µV/LSB   (ADCRANGE=0)
 *   0x05 VBUS       unsigned, 3.125 mV/LSB
 *   0x3F DEVICE_ID  0x2381
 *
 * Current is computed from the shunt voltage and the per-connector shunt
 * resistance, matching the RP2350B INA219 approach (which self-calibrated each
 * read), so this never depends on SHUNT_CAL having been programmed.
 *
 * NOTE: scaling constants below are from the INA238 datasheet; verify the
 * absolute current reading against a bench reference during calibration.
 */
#include "ina238.h"
#include "i2c_bus.h"

#define INA238_REG_CONFIG     0x00
#define INA238_REG_ADC_CONFIG 0x01
#define INA238_REG_VSHUNT     0x04
#define INA238_REG_VBUS       0x05
#define INA238_REG_DEVICE_ID  0x3F

#define VSHUNT_LSB_NV   5000    /* 5 µV  = 5000 nV per LSB (ADCRANGE=0) */
#define VBUS_LSB_UV     3125    /* 3.125 mV = 3125 µV per LSB */

/* Shunt resistance per connector (milliohms) — unchanged from the INA219 board. */
static int shunt_mohm_for(uint8_t addr)
{
    return (addr == I2C_ADDR_INA238_EXTERNAL) ? 30 : 50;
}

static int read_reg16(uint8_t addr, uint8_t reg, uint16_t *out)
{
    uint8_t rx[2];
    if (i2c_bus_write_read(addr, &reg, 1, rx, 2) != 0) return -1;
    *out = (uint16_t)((rx[0] << 8) | rx[1]);
    return 0;
}

static int write_reg16(uint8_t addr, uint8_t reg, uint16_t val)
{
    uint8_t tx[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return i2c_bus_write(addr, tx, 3);
}

int ina238_read(uint8_t addr, int *bus_mv, int *shunt_uv, int *current_ua)
{
    /* Force ADCRANGE=0 and clear RST; continuous mode is the power-up default. */
    (void)write_reg16(addr, INA238_REG_CONFIG, 0x0000);

    uint16_t vsh_raw = 0, vbus_raw = 0;
    if (read_reg16(addr, INA238_REG_VSHUNT, &vsh_raw) != 0) return -1;
    if (read_reg16(addr, INA238_REG_VBUS, &vbus_raw) != 0) return -1;

    int32_t vsh = (int16_t)vsh_raw;                       /* signed LSBs */
    int32_t shunt_uv_v = (vsh * VSHUNT_LSB_NV) / 1000;    /* nV -> µV */
    int32_t bus_mv_v   = ((int32_t)vbus_raw * VBUS_LSB_UV) / 1000; /* µV -> mV */

    /* I[µA] = Vshunt[µV] / R[Ω] = Vshunt_uv * 1000 / R_mohm. */
    int32_t cur_ua = (shunt_uv_v * 1000) / shunt_mohm_for(addr);

    if (shunt_uv)   *shunt_uv = (int)shunt_uv_v;
    if (bus_mv)     *bus_mv = (int)bus_mv_v;
    if (current_ua) *current_ua = (int)cur_ua;
    return 0;
}

int ina238_read_id(uint8_t addr, uint16_t *id)
{
    return read_reg16(addr, INA238_REG_DEVICE_ID, id);
}

/* ---- single-register access (power_profile.c) ---- */

int ina238_set_adcrange_fine(uint8_t addr)
{
    return write_reg16(addr, INA238_REG_CONFIG, 0x0000);
}

int ina238_write_adc_config(uint8_t addr, uint16_t adc_config)
{
    return write_reg16(addr, INA238_REG_ADC_CONFIG, adc_config);
}

int ina238_read_shunt_ua(uint8_t addr, int32_t *current_ua)
{
    uint16_t raw = 0;
    if (read_reg16(addr, INA238_REG_VSHUNT, &raw) != 0) return -1;
    int32_t shunt_uv = ((int32_t)(int16_t)raw * VSHUNT_LSB_NV) / 1000;
    if (current_ua) *current_ua = (shunt_uv * 1000) / shunt_mohm_for(addr);
    return 0;
}

int ina238_read_bus_mv(uint8_t addr, uint16_t *bus_mv)
{
    uint16_t raw = 0;
    if (read_reg16(addr, INA238_REG_VBUS, &raw) != 0) return -1;
    if (bus_mv) *bus_mv = (uint16_t)(((uint32_t)raw * VBUS_LSB_UV) / 1000u);
    return 0;
}
