/* ============================================================================
 * sensor_sim.h — generic emulated-I2C-sensor manager (the orchestrator side).
 *
 * The iCE40 gateware is a sensor-agnostic I2C target: it serves a 256-byte
 * register image we load and runs a configurable busy-bit handshake.  All
 * sensor-specific knowledge lives in a `sensor_model_t` implemented per device
 * (sensor_bmp280.c is the first).  Adding a new I2C sensor = a new model file
 * (its register map + handshake config) and one registry entry here — NO
 * gateware change.
 *
 * The manager owns the single active sensor: it builds the register image,
 * loads it over SPI (signal_engine.c), arms the target, and exposes set/stop/
 * status to the JSON command layer.
 * ========================================================================== */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "signal_engine.h"   /* i2c_sensor_status_t */

#define SENSOR_REGIMAGE_LEN 256

typedef struct sensor_model sensor_model_t;

/* A sensor model.  `reset`, `set_param` and `build_regimage` operate on the
   model's own (file-static) parameter state.  The handshake fields describe
   how the FPGA should fake a conversion: a DUT write to `trig_reg` sets
   `busy_mask` in `busy_reg` for `conv_us` microseconds (0/0 = no handshake). */
struct sensor_model {
    const char *name;            /* e.g. "bmp280" */
    uint8_t     default_addr7;   /* 7-bit I2C address */
    uint8_t     trig_reg;        /* register whose write triggers a conversion */
    uint8_t     busy_reg;        /* register that carries the busy bit */
    uint8_t     busy_mask;       /* busy bit(s) OR-ed in while converting */
    uint16_t    conv_us;         /* conversion time in microseconds */

    void (*reset)(sensor_model_t *m);
    int  (*set_param)(sensor_model_t *m, const char *key, float value); /* 0 ok, -1 unknown */
    void (*build_regimage)(sensor_model_t *m, uint8_t img[SENSOR_REGIMAGE_LEN]);
};

/* Start an emulated sensor.  `type` selects the model ("bmp280"); addr7=0 uses
   the model default; sda_ch/scl_ch are 1-based LA channels.
   Returns 0 on success, -1 unknown type, -2 bad channel/SPI config. */
int sensor_sim_start(const char *type, uint8_t addr7,
                     unsigned sda_ch, unsigned scl_ch);

/* Update a model parameter (e.g. "temperature_c", "pressure_pa"), rebuild the
   register image and reload it.  Returns 0 ok, -1 no active sensor,
   -2 unknown key for this model. */
int sensor_sim_set(const char *key, float value);

/* Disarm and forget the active sensor. */
void sensor_sim_stop(void);

bool        sensor_sim_active(void);
const char *sensor_sim_type(void);     /* active model name, or "" */
uint8_t     sensor_sim_addr7(void);
unsigned    sensor_sim_sda(void);      /* active SDA LA channel (1-based; 0 = none) */
unsigned    sensor_sim_scl(void);      /* active SCL LA channel (1-based; 0 = none) */

/* Read the FPGA-side target status; -1 if no sensor is active. */
int sensor_sim_get_status(i2c_sensor_status_t *out);

/* Read `len` register bytes back from the FPGA image (also reflects DUT
   writes); -1 if inactive or out of range. */
int sensor_sim_read_regs(uint8_t start_addr, uint8_t *buf, size_t len);
