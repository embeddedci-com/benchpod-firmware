/*
 * ina238.c — TI INA238 current/power monitor driver, which also drives the pin-compatible INA226
 * (boards fitted while the INA238 is unobtainable). The chip is detected per I2C address on first
 * use; every reading comes out in the same units either way.
 *
 * INA238 registers (16-bit, big-endian):
 *   0x00 CONFIG     bit4 ADCRANGE (0 = ±163.84 mV, 5 µV/LSB)
 *   0x01 ADC_CONFIG MODE[15:12] VBUSCT[11:9] VSHCT[8:6] VTCT[5:3] AVG[2:0]
 *   0x04 VSHUNT     signed, 5 µV/LSB   (ADCRANGE=0)
 *   0x05 VBUS       unsigned, 3.125 mV/LSB
 *   0x3F DEVICE_ID  0x2381
 * INA226 registers:
 *   0x00 CONFIG     AVG[11:9] VBUSCT[8:6] VSHCT[5:3] MODE[2:0]   (±81.92 mV shunt range, fixed)
 *   0x01 VSHUNT     signed, 2.5 µV/LSB
 *   0x02 VBUS       unsigned, 1.25 mV/LSB
 *   0xFF DIE_ID     0x2260
 *
 * Current is computed from the shunt voltage and the per-connector shunt
 * resistance, matching the RP2350B INA219 approach (which self-calibrated each
 * read), so this never depends on SHUNT_CAL / CALIBRATION having been programmed.
 *
 * NOTE: scaling constants below are from the datasheets; verify the
 * absolute current reading against a bench reference during calibration.
 */
#include "ina238.h"
#include "i2c_bus.h"

#define INA238_REG_CONFIG     0x00
#define INA238_REG_ADC_CONFIG 0x01
#define INA238_REG_VSHUNT     0x04
#define INA238_REG_VBUS       0x05
#define INA238_REG_DEVICE_ID  0x3F

#define INA226_REG_CONFIG     0x00
#define INA226_REG_VSHUNT     0x01
#define INA226_REG_VBUS       0x02
#define INA226_REG_DIE_ID     0xFF

#define INA238_VSHUNT_LSB_NV  5000    /* 5 µV  = 5000 nV per LSB (ADCRANGE=0) */
#define INA238_VBUS_LSB_UV    3125    /* 3.125 mV = 3125 µV per LSB */
#define INA226_VSHUNT_LSB_NV  2500    /* 2.5 µV */
#define INA226_VBUS_LSB_UV    1250    /* 1.25 mV */

/* Shunt resistance per connector (milliohms). INA238 boards keep the INA219 board's shunts
   (internal 50, external 30). INA226 boards have smaller ones (internal 30, external 20) so the
   INA226's ±81.92 mV range still covers the eFuse limits: 2.73 A over the 2.03 A internal
   limit, 4.10 A over the 3.03 A external limit. */
static int shunt_mohm_for(uint8_t addr)
{
    if (addr == I2C_ADDR_INA_POD) return 30;   /* R138, whichever chip is fitted */
    int ext = (addr == I2C_ADDR_INA238_EXTERNAL);
    if (ina238_chip(addr) == INA_CHIP_INA226) return ext ? 20 : 30;
    return ext ? 30 : 50;
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

/* ---- chip detection ---- */

#define INA_CHIP_SLOTS 3u   /* internal, external, pod */
static struct { uint8_t addr; ina_chip_t chip; } s_chip[INA_CHIP_SLOTS];
static int8_t s_pod_present = -1;   /* -1 = not probed yet */

static ina_chip_t detect(uint8_t addr)
{
    uint16_t v = 0;
    if (read_reg16(addr, INA238_REG_DEVICE_ID, &v) == 0 && (v >> 4) == 0x238u) return INA_CHIP_INA238;
    if (read_reg16(addr, INA226_REG_DIE_ID, &v) == 0 && (v >> 4) == 0x226u) return INA_CHIP_INA226;
    /* Answers but is neither: treat it as the INA238 the board was designed for, as this driver
       always did. Not answering at all (an external rail with no supply) is retried next time. */
    return (read_reg16(addr, INA238_REG_DEVICE_ID, &v) == 0) ? INA_CHIP_INA238 : INA_CHIP_NONE;
}

ina_chip_t ina238_chip(uint8_t addr)
{
    for (unsigned i = 0; i < INA_CHIP_SLOTS; i++)
        if (s_chip[i].addr == addr && s_chip[i].chip != INA_CHIP_NONE) return s_chip[i].chip;
    ina_chip_t c = detect(addr);
    if (c == INA_CHIP_NONE) return c;
    for (unsigned i = 0; i < INA_CHIP_SLOTS; i++)
        if (s_chip[i].addr == addr || s_chip[i].chip == INA_CHIP_NONE) {
            s_chip[i].addr = addr;
            s_chip[i].chip = c;
            break;
        }
    return c;
}

void ina238_forget_chips(void)
{
    for (unsigned i = 0; i < INA_CHIP_SLOTS; i++) { s_chip[i].addr = 0; s_chip[i].chip = INA_CHIP_NONE; }
    s_pod_present = -1;
}

bool ina_pod_present(void)
{
    /* The chip is soldered on or not: probe once (at boot, before the tasks run) and keep the
       answer, so status and the cloud announce never touch the bus for it. */
    if (s_pod_present < 0) s_pod_present = (ina238_chip(I2C_ADDR_INA_POD) != INA_CHIP_NONE);
    return s_pod_present > 0;
}

const char *ina_chip_name(ina_chip_t c)
{
    switch (c) {
        case INA_CHIP_INA238: return "INA238";
        case INA_CHIP_INA226: return "INA226";
        default:              return "none";
    }
}

/* ---- per-chip register layout and scale ---- */

static uint8_t reg_vshunt(ina_chip_t c) { return c == INA_CHIP_INA226 ? INA226_REG_VSHUNT : INA238_REG_VSHUNT; }
static uint8_t reg_vbus(ina_chip_t c)   { return c == INA_CHIP_INA226 ? INA226_REG_VBUS : INA238_REG_VBUS; }
static int32_t vshunt_lsb_nv(ina_chip_t c) { return c == INA_CHIP_INA226 ? INA226_VSHUNT_LSB_NV : INA238_VSHUNT_LSB_NV; }
static uint32_t vbus_lsb_uv(ina_chip_t c)  { return c == INA_CHIP_INA226 ? INA226_VBUS_LSB_UV : INA238_VBUS_LSB_UV; }

int ina238_read(uint8_t addr, int *bus_mv, int *shunt_uv, int *current_ua)
{
    ina_chip_t c = ina238_chip(addr);
    if (c == INA_CHIP_NONE) return -1;
    /* INA238: force ADCRANGE=0 and clear RST; continuous mode is the power-up default. The
       INA226 has one shunt range and powers up converting continuously: nothing to set. */
    if (c == INA_CHIP_INA238) (void)write_reg16(addr, INA238_REG_CONFIG, 0x0000);

    uint16_t vsh_raw = 0, vbus_raw = 0;
    if (read_reg16(addr, reg_vshunt(c), &vsh_raw) != 0) return -1;
    if (read_reg16(addr, reg_vbus(c), &vbus_raw) != 0) return -1;

    int32_t vsh = (int16_t)vsh_raw;                               /* signed LSBs */
    int32_t shunt_uv_v = (vsh * vshunt_lsb_nv(c)) / 1000;         /* nV -> µV */
    int32_t bus_mv_v   = (int32_t)(((uint32_t)vbus_raw * vbus_lsb_uv(c)) / 1000u); /* µV -> mV */

    /* I[µA] = Vshunt[µV] / R[Ω] = Vshunt_uv * 1000 / R_mohm. */
    int32_t cur_ua = (shunt_uv_v * 1000) / shunt_mohm_for(addr);

    if (shunt_uv)   *shunt_uv = (int)shunt_uv_v;
    if (bus_mv)     *bus_mv = (int)bus_mv_v;
    if (current_ua) *current_ua = (int)cur_ua;
    return 0;
}

int ina238_read_id(uint8_t addr, uint16_t *id)
{
    ina_chip_t c = ina238_chip(addr);
    if (c == INA_CHIP_NONE) return -1;
    return read_reg16(addr, c == INA_CHIP_INA226 ? INA226_REG_DIE_ID : INA238_REG_DEVICE_ID, id);
}

/* ---- single-register access (power_profile.c) ---- */

int ina238_set_adcrange_fine(uint8_t addr)
{
    ina_chip_t c = ina238_chip(addr);
    if (c == INA_CHIP_NONE) return -1;
    if (c == INA_CHIP_INA226) return 0;              /* one fixed range */
    return write_reg16(addr, INA238_REG_CONFIG, 0x0000);
}

int ina238_write_adc_config(uint8_t addr, uint16_t adc_config)
{
    ina_chip_t c = ina238_chip(addr);
    if (c == INA_CHIP_NONE) return -1;
    return write_reg16(addr, c == INA_CHIP_INA226 ? INA226_REG_CONFIG : INA238_REG_ADC_CONFIG, adc_config);
}

int ina238_restore_adc_config(uint8_t addr)
{
    ina_chip_t c = ina238_chip(addr);
    if (c == INA_CHIP_NONE) return -1;
    return ina238_write_adc_config(addr, c == INA_CHIP_INA226 ? INA226_CONFIG_RESET : INA238_ADC_CONFIG_RESET);
}

int ina238_read_shunt_ua(uint8_t addr, int32_t *current_ua)
{
    ina_chip_t c = ina238_chip(addr);
    if (c == INA_CHIP_NONE) return -1;
    uint16_t raw = 0;
    if (read_reg16(addr, reg_vshunt(c), &raw) != 0) return -1;
    int32_t shunt_uv = ((int32_t)(int16_t)raw * vshunt_lsb_nv(c)) / 1000;
    if (current_ua) *current_ua = (shunt_uv * 1000) / shunt_mohm_for(addr);
    return 0;
}

int ina238_read_bus_mv(uint8_t addr, uint16_t *bus_mv)
{
    ina_chip_t c = ina238_chip(addr);
    if (c == INA_CHIP_NONE) return -1;
    uint16_t raw = 0;
    if (read_reg16(addr, reg_vbus(c), &raw) != 0) return -1;
    if (bus_mv) *bus_mv = (uint16_t)(((uint32_t)raw * vbus_lsb_uv(c)) / 1000u);
    return 0;
}
