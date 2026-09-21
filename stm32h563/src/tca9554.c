/*
 * tca9554.c — TCA9554 8-bit I2C GPIO expander driver (stateless RMW).
 */
#include "tca9554.h"
#include "i2c_bus.h"

#define TCA_REG_INPUT   0x00
#define TCA_REG_OUTPUT  0x01
#define TCA_REG_POLARITY 0x02
#define TCA_REG_CONFIG  0x03   /* 1 = input, 0 = output */

int tca9554_read_reg(uint8_t addr, uint8_t reg, uint8_t *val)
{
    return i2c_bus_write_read(addr, &reg, 1, val, 1);
}

int tca9554_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg, val };
    return i2c_bus_write(addr, tx, 2);
}

int tca9554_set_dir(uint8_t addr, uint8_t pin, bool is_output)
{
    if (pin > 7) return -1;
    uint8_t cfg;
    if (tca9554_read_reg(addr, TCA_REG_CONFIG, &cfg) != 0) return -1;
    if (is_output) cfg &= ~(1u << pin);   /* 0 = output */
    else           cfg |=  (1u << pin);   /* 1 = input  */
    return tca9554_write_reg(addr, TCA_REG_CONFIG, cfg);
}

int tca9554_write_pin(uint8_t addr, uint8_t pin, bool value)
{
    if (pin > 7) return -1;
    uint8_t out;
    if (tca9554_read_reg(addr, TCA_REG_OUTPUT, &out) != 0) return -1;
    if (value) out |=  (1u << pin);
    else       out &= ~(1u << pin);
    return tca9554_write_reg(addr, TCA_REG_OUTPUT, out);
}

int tca9554_read_pin(uint8_t addr, uint8_t pin, bool *value)
{
    if (pin > 7) return -1;
    uint8_t in;
    if (tca9554_read_reg(addr, TCA_REG_INPUT, &in) != 0) return -1;
    if (value) *value = (in >> pin) & 1u;
    return 0;
}

int tca9554_read_inputs(uint8_t addr, uint8_t *port)
{
    return tca9554_read_reg(addr, TCA_REG_INPUT, port);
}
