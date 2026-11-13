/* sensor_sim.h (test reference only) — the model type the C sensor models in test/ref were written
 * against. Those models were the firmware's built-in sensors until the sensor definition engine
 * (sensor_def.h) replaced them; test_sensor_def.c keeps comparing the engine with them. */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>


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

