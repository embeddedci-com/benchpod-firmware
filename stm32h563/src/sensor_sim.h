/* ============================================================================
 * sensor_sim.h — generic emulated-I2C-sensor manager (the orchestrator side).
 *
 * The iCE40 gateware is a sensor-agnostic I2C target: it serves a 256-byte
 * register image we load and runs a configurable busy-bit handshake.  Its
 * register pointer is the first byte the DUT writes, auto-increments on every
 * byte and wraps at 0xFF, and is kept across transactions.  All sensor-specific
 * knowledge lives in a `sensor_model_t` implemented per device.  Adding a new
 * I2C sensor = a new model (its register map, parameter table and handshake
 * config) and one registry entry in sensor_sim.c — NO gateware change.
 *
 * The manager owns the single active sensor: it keeps the parameter values,
 * builds the register image, loads it over SPI (signal_engine.c), arms the
 * target, and exposes set/stop/status to the JSON command layer.
 *
 * Registers the DUT writes (configuration: a measurement range, a power mode)
 * land in the FPGA's copy.  A parameter change therefore reloads only the
 * model's data window [data_lo, data_hi], and builds it from the live image so
 * the data follows the DUT's configuration.  A model with `watch_writes` is
 * also rebuilt whenever the DUT writes a register (sensor_sim_poll).
 * ========================================================================== */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "signal_engine.h"   /* i2c_sensor_status_t */

#define SENSOR_REGIMAGE_LEN 256
#define SENSOR_MAX_PARAMS   8

/* One settable value of a model: the `sensor_set` JSON key, its unit, the
   accepted range (inclusive) and the value at sensor_start. */
typedef struct {
    const char *key;      /* e.g. "temperature_c" */
    const char *unit;     /* e.g. "C", "Pa", "%", "g", "dps" */
    float       min, max;
    float       def;
} sensor_param_t;

typedef struct sensor_model sensor_model_t;

/* A sensor model.  `build_regimage` is a pure function of the parameter values
   (in `params` order) and, when not NULL, `live`: the FPGA's current image, which
   carries what the DUT wrote.  live is NULL at sensor_start (power-on state).
   It writes the whole image; only [data_lo, data_hi] is loaded after the start,
   unless it returns true: the DUT asked for something that rewrites the whole part
   (a soft reset), and the full image is loaded.
   The handshake fields describe how the FPGA fakes a conversion: a DUT write to
   `trig_reg` sets `busy_mask` in `busy_reg` for `conv_us` microseconds (0/0 =
   no handshake). */
struct sensor_model {
    const char *name;            /* e.g. "bmp280" */
    const char *label;           /* human-readable, e.g. "Bosch BMP280 (temperature, pressure)" */
    uint8_t     default_addr7;   /* 7-bit I2C address */
    uint8_t     alt_addr7;       /* the other strap address (0 = none) */
    uint8_t     trig_reg;        /* register whose write triggers a conversion */
    uint8_t     busy_reg;        /* register that carries the busy bit */
    uint8_t     busy_mask;       /* busy bit(s) OR-ed in while converting */
    uint16_t    conv_us;         /* conversion time in microseconds */
    uint8_t     data_lo, data_hi;/* the registers a parameter change reloads */
    bool        watch_writes;    /* rebuild the data window after every DUT register write */

    const sensor_param_t *params;
    uint8_t               n_params;

    bool (*build_regimage)(const float *values, const uint8_t *live,
                           uint8_t img[SENSOR_REGIMAGE_LEN]);
};

/* The registry, for listings (`sensor_types`).  *count gets the number of models. */
const sensor_model_t *const *sensor_sim_models(size_t *count);
const sensor_model_t *sensor_sim_find(const char *type);   /* NULL for an unknown type */

/* Start an emulated sensor.  `type` selects the model ("bmp280"); addr7=0 uses
   the model default; sda_ch/scl_ch are 1-based LA channels.  Every parameter
   starts at its default.
   Returns 0 on success, -1 unknown type, -2 bad channel/SPI config. */
int sensor_sim_start(const char *type, uint8_t addr7,
                     unsigned sda_ch, unsigned scl_ch);

/* Set a parameter of the active model (no reload yet).  Returns 0 ok, -1 no
   active sensor, -2 unknown key for this model, -3 value out of range. */
int sensor_sim_set_value(const char *key, float value);

/* Rebuild and reload the data window after sensor_sim_set_value calls.
   Returns 0 ok, -1 no active sensor, -2 SPI failure. */
int sensor_sim_apply(void);

/* sensor_sim_set_value + sensor_sim_apply for one key (same return codes). */
int sensor_sim_set(const char *key, float value);

/* Disarm and forget the active sensor. */
void sensor_sim_stop(void);

/* Main-loop service: for a `watch_writes` model, rebuild the data window when the
   DUT's write count moved.  Cheap (one status read every few ms) while a sensor is
   active, nothing otherwise. */
int  sensor_sim_poll(void);   /* 0, or -1 when a re-arm failed and the sensor stopped */

/* The fabric was reconfigured (image swap, reflash): the I2C target and its image
   are gone.  Re-arm the active sensor with its values on the next poll. */
void sensor_sim_on_gateware_reconfigured(void);

bool                  sensor_sim_active(void);
const sensor_model_t *sensor_sim_model(void);  /* active model, or NULL */
const char *sensor_sim_type(void);             /* active model name, or "" */
uint8_t     sensor_sim_addr7(void);
unsigned    sensor_sim_sda(void);      /* active SDA LA channel (1-based; 0 = none) */
unsigned    sensor_sim_scl(void);      /* active SCL LA channel (1-based; 0 = none) */
float       sensor_sim_value(unsigned i);  /* parameter i of the active model */

/* Read the FPGA-side target status; -1 if no sensor is active. */
int sensor_sim_get_status(i2c_sensor_status_t *out);

/* Read `len` register bytes back from the FPGA image (also reflects DUT
   writes); -1 if inactive or out of range. */
int sensor_sim_read_regs(uint8_t start_addr, uint8_t *buf, size_t len);
