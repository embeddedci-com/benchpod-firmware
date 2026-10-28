/*
 * module_link.c — digital v3r3's analog-module link and power switches. See module_link.h.
 */
#include "module_link.h"
#include "board_variant.h"
#include "i2c_bus.h"
#include "signal_engine.h"   /* dac_stop, dac_quad_boot */
#include "stm32h5xx_hal.h"
#include "board_pins.h"
#include "pico_compat.h"
#include "bp_log.h"

#include <string.h>

#define EE_READ_CHUNK   32u    /* one transfer: ~3 ms at 100 kHz, inside the 20 ms HAL timeout */
#define EE_WRITE_POLLS  10u    /* 1 ms apart; the 24C32 write cycle is 5 ms max */
#define MOD_POWER_SETTLE_MS 20u

static bool s_link;            /* digital v3r3 */
static bool s_present;         /* MOD_PRESENT# low at boot */
static int  s_id_rc = MOD_EE_NO_CHIP;
static module_id_t s_id;
static bool s_cal_valid;

static void pin_mode(uint16_t pin, uint32_t mode, uint32_t pull)
{
    GPIO_InitTypeDef gp = {0};
    gp.Pin = pin;
    gp.Mode = mode;
    gp.Pull = pull;
    gp.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOD_GPIO_PORT, &gp);
}

static bool pin_high(uint16_t pin) { return (MOD_GPIO_PORT->IDR & pin) != 0u; }

/* Driven low when off; on is the released pin, its 10k pull-up enables the switch, the same
   state as at power-up. */
static void rail_drive(uint16_t pin, bool on)
{
    if (on) {
        pin_mode(pin, GPIO_MODE_INPUT, GPIO_NOPULL);
    } else {
        MOD_GPIO_PORT->BSRR = (uint32_t)pin << 16;   /* ODR low before the switch to output */
        pin_mode(pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    }
}

static int ee_read_hdr(void)
{
    uint8_t hdr[MOD_EE_HDR_LEN];
    s_cal_valid = false;
    if (module_eeprom_read(0, hdr, sizeof(hdr)) != 0) return s_id_rc = MOD_EE_NO_CHIP;
    s_id_rc = module_ee_decode(hdr, sizeof(hdr), &s_id);
    if (s_id_rc == MOD_EE_OK && s_id.cal_len > 0 && s_id.cal_len <= MOD_EE_CAL_MAX) {
        uint16_t crc = 0xFFFFu;
        uint8_t buf[EE_READ_CHUNK];
        bool ok = true;
        for (uint16_t off = 0; off < s_id.cal_len && ok; off += EE_READ_CHUNK) {
            size_t left = (size_t)s_id.cal_len - off, n = left < EE_READ_CHUNK ? left : EE_READ_CHUNK;
            ok = module_eeprom_read((uint16_t)(MOD_EE_CAL_ADDR + off), buf, n) == 0;
            crc = module_ee_crc16(buf, n, crc);
        }
        s_cal_valid = ok && crc == s_id.cal_crc;
    }
    return s_id_rc;
}

void module_link_init(void)
{
    MOD_GPIO_CLK_EN();
    /* The enables with the internal pull-down on: the board's 10k pull-ups hold them high on
       digital v3r3 (the rails stay on), a header pin on any earlier board reads low. */
    pin_mode(MOD_PWR_EN_PIN | BRIDGE_PWR_EN_PIN, GPIO_MODE_INPUT, GPIO_PULLDOWN);
    busy_wait_us(50);
    s_link = pin_high(MOD_PWR_EN_PIN) && pin_high(BRIDGE_PWR_EN_PIN);
    if (!s_link) {
        pin_mode(MOD_PWR_EN_PIN | BRIDGE_PWR_EN_PIN, GPIO_MODE_ANALOG, GPIO_NOPULL);   /* as at reset */
        return;
    }
    pin_mode(MOD_PWR_EN_PIN | BRIDGE_PWR_EN_PIN, GPIO_MODE_INPUT, GPIO_NOPULL);
    pin_mode(MOD_PRESENT_PIN, GPIO_MODE_INPUT, GPIO_NOPULL);   /* 10k pull-up on the board */
    busy_wait_us(10);
    s_present = !pin_high(MOD_PRESENT_PIN);
    log_printf("[module] digital v3r3: module %s, module power %s, bridge power %s\r\n",
           s_present ? "present" : "not plugged in",
           module_power_on() ? "on" : "off", bridge_power_on() ? "on" : "off");
    (void)i2c_bus_init_on(I2C_BUS_MODULE);
    if (s_present) {
        int rc = ee_read_hdr();
        if (rc == MOD_EE_OK) {
            log_printf("[module] EEPROM: %s rev %u.%u serial \"%s\", calibration %s (%u bytes)\r\n",
                   module_ee_type_str(s_id.type), s_id.rev_major, s_id.rev_minor, s_id.serial,
                   s_id.cal_len == 0 ? "none" : s_cal_valid ? "ok" : "CRC mismatch",
                   (unsigned)s_id.cal_len);
        } else {
            log_printf("[module] EEPROM: %s\r\n",
                   rc == MOD_EE_NO_CHIP ? "no answer at 0x50" : module_ee_result_str(rc));
        }
    }
}

bool module_link_board(void) { return s_link; }
bool module_present(void)    { return s_present; }
bool module_present_now(void) { return s_link && !pin_high(MOD_PRESENT_PIN); }
bool module_power_on(void)   { return s_link && pin_high(MOD_PWR_EN_PIN); }
bool bridge_power_on(void)   { return s_link && pin_high(BRIDGE_PWR_EN_PIN); }

int bridge_power_set(bool on)
{
    if (!s_link) return -1;
    rail_drive(BRIDGE_PWR_EN_PIN, on);
    log_printf("[module] bridge port power %s\r\n", on ? "on" : "off");
    return 0;
}

int module_power_set(bool on)
{
    if (!s_link) return -1;
    if (on == module_power_on()) return 0;
    bool analog = board_analog_is_module();
    if (!on) {
        if (analog) {
            dac_stop();               /* nothing streams into an unpowered DAC */
            (void)ana14_set(false);   /* +-14 V off before its input goes */
        }
        rail_drive(MOD_PWR_EN_PIN, false);
        log_printf("[module] module power off\r\n");
        return 0;
    }
    rail_drive(MOD_PWR_EN_PIN, true);
    sleep_ms(MOD_POWER_SETTLE_MS);
    log_printf("[module] module power on\r\n");
    (void)ee_read_hdr();
    if (!analog) return 0;
    /* U58 powered up with its outputs as inputs (relays off, +-14 V off): set it up as at boot,
       then park the DAC and switch the +-14 V supply on again. */
    bool u58 = false;
    for (unsigned i = 0; i < BOARD_VARIANT_PROBE_TRIES && !u58; i++)
        u58 = i2c_bus_probe_on(I2C_BUS_MODULE, I2C_ADDR_TCA9554_ANASW);
    if (!u58) {
        log_printf("[module] WARNING: U58 does not answer after power-on\r\n");
        return -2;
    }
    analog_switch_init();
    dac_quad_boot();
    return 0;
}

int module_eeprom_read(uint16_t addr, uint8_t *buf, size_t len)
{
    if (!s_link || (size_t)addr + len > MOD_EE_SIZE) return -1;
    while (len) {
        size_t n = len < EE_READ_CHUNK ? len : EE_READ_CHUNK;
        uint8_t a[2] = { (uint8_t)(addr >> 8), (uint8_t)addr };
        if (i2c_bus_write_read_on(I2C_BUS_MODULE, I2C_ADDR_MOD_EEPROM, a, 2, buf, n) != 0) return -1;
        addr = (uint16_t)(addr + n); buf += n; len -= n;
    }
    return 0;
}

int module_eeprom_write(uint16_t addr, const uint8_t *buf, size_t len)
{
    if (!s_link || (size_t)addr + len > MOD_EE_SIZE) return -1;
    while (len) {
        size_t n = module_ee_chunk(addr, len);
        uint8_t tx[2 + MOD_EE_PAGE];
        tx[0] = (uint8_t)(addr >> 8);
        tx[1] = (uint8_t)addr;
        memcpy(tx + 2, buf, n);
        if (i2c_bus_write_on(I2C_BUS_MODULE, I2C_ADDR_MOD_EEPROM, tx, 2 + n) != 0) return -1;
        /* The chip NACKs its address until the write cycle is done. */
        bool done = false;
        for (unsigned i = 0; i < EE_WRITE_POLLS && !done; i++) {
            sleep_ms(1);
            done = i2c_bus_probe_on(I2C_BUS_MODULE, I2C_ADDR_MOD_EEPROM);
        }
        if (!done) return -1;
        addr = (uint16_t)(addr + n); buf += n; len -= n;
    }
    return 0;
}

int module_id(module_id_t *id)
{
    if (id) *id = s_id;
    return s_id_rc;
}

int  module_id_reload(void) { return s_link ? ee_read_hdr() : MOD_EE_NO_CHIP; }
bool module_cal_valid(void) { return s_cal_valid; }

static int write_hdr(const module_id_t *id)
{
    uint8_t hdr[MOD_EE_HDR_LEN];
    module_ee_encode(id, hdr);
    int rc = module_eeprom_write(0, hdr, sizeof(hdr));
    (void)ee_read_hdr();
    return rc;
}

int module_id_write(uint8_t type, uint8_t rev_major, uint8_t rev_minor, const char *serial)
{
    if (!s_link) return -1;
    module_id_t id = {0};
    if (ee_read_hdr() == MOD_EE_OK) id = s_id;   /* keep the calibration fields */
    else { id.cal_len = 0; id.cal_crc = 0; }
    id.type = type;
    id.rev_major = rev_major;
    id.rev_minor = rev_minor;
    memset(id.serial, 0, sizeof(id.serial));
    strncpy(id.serial, serial ? serial : "", MOD_EE_SERIAL_LEN);
    return write_hdr(&id);
}

int module_cal_commit(uint16_t len)
{
    if (!s_link || len > MOD_EE_CAL_MAX) return -1;
    module_id_t id = {0};
    if (ee_read_hdr() == MOD_EE_OK) id = s_id;
    uint16_t crc = 0xFFFFu;
    uint8_t buf[EE_READ_CHUNK];
    for (uint16_t off = 0; off < len; off += EE_READ_CHUNK) {
        size_t left = (size_t)len - off, n = left < EE_READ_CHUNK ? left : EE_READ_CHUNK;
        if (module_eeprom_read((uint16_t)(MOD_EE_CAL_ADDR + off), buf, n) != 0) return -1;
        crc = module_ee_crc16(buf, n, crc);
    }
    id.cal_len = len;
    id.cal_crc = len ? crc : 0;
    return write_hdr(&id);
}
