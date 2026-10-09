/* ============================================================================
 * sensor_sim.c — generic emulated-I2C-sensor manager.  See sensor_sim.h.
 * ========================================================================== */

#include "sensor_sim.h"
#include "sensor_bmp280.h"
#include "sensor_sht4x.h"
#include "sensor_mpu6050.h"
#include "bp_log.h"
#include "pico/time.h"

#include <string.h>
#include <stdio.h>
#include <math.h>           /* lroundf — newlib-nano printf has no %f */

/* ---- model registry — add new sensors here ---- */
static const sensor_model_t *const k_models[] = {
    &sensor_bmp280_model,
    &sensor_bme280_model,
    &sensor_sht4x_model,
    &sensor_mpu6050_model,
};
#define N_MODELS (sizeof(k_models) / sizeof(k_models[0]))

const sensor_model_t *const *sensor_sim_models(size_t *count) {
    if (count) *count = N_MODELS;
    return k_models;
}

const sensor_model_t *sensor_sim_find(const char *type) {
    if (!type) return NULL;
    for (size_t i = 0; i < N_MODELS; i++)
        if (strcmp(type, k_models[i]->name) == 0) return k_models[i];
    return NULL;
}

/* ---- active-sensor state ---- */
static const sensor_model_t *active = NULL;
static uint8_t  active_addr7  = 0;
static unsigned active_sda    = 0;
static unsigned active_scl    = 0;
static float    values[SENSOR_MAX_PARAMS];
static uint8_t  regimg[SENSOR_REGIMAGE_LEN];
static uint8_t  live[SENSOR_REGIMAGE_LEN];
static uint16_t seen_wr_count;
static uint64_t next_watch_us;
static bool     rearm_pending;

/* Poll the write counter of a watch_writes model this often (one 7-byte status read). */
#define WATCH_PERIOD_US 2000u

static int arm(void) {
    (void)active->build_regimage(values, NULL, regimg);
    if (fpga_i2c_load_regs(0, regimg, sizeof(regimg)) != 0) return -2;
    if (fpga_i2c_sensor_config(active_addr7, active_sda, active_scl, true,
                               active->trig_reg, active->busy_reg, active->busy_mask,
                               active->conv_us) != 0) return -2;
    i2c_sensor_status_t st = {0};
    seen_wr_count = (fpga_i2c_sensor_status(&st) == 0) ? st.wr_count : 0;
    return 0;
}

/* Rebuild from the live image (what the DUT configured) and load the data window only, so
   the DUT's own register writes survive. */
static int reload_data(void) {
    if (fpga_i2c_read_regs(0, live, sizeof(live)) != 0) return -2;
    bool full = active->build_regimage(values, live, regimg);
    uint8_t lo = full ? 0x00 : active->data_lo, hi = full ? 0xFF : active->data_hi;
    return fpga_i2c_load_regs(lo, &regimg[lo], (size_t)hi - lo + 1u) == 0 ? 0 : -2;
}

int sensor_sim_start(const char *type, uint8_t addr7,
                     unsigned sda_ch, unsigned scl_ch) {
    const sensor_model_t *m = sensor_sim_find(type);
    if (!m) return -1;

    if (active) sensor_sim_stop();

    for (unsigned i = 0; i < m->n_params; i++) values[i] = m->params[i].def;
    active        = m;
    active_addr7  = addr7 ? addr7 : m->default_addr7;
    active_sda    = sda_ch;
    active_scl    = scl_ch;
    rearm_pending = false;
    next_watch_us = 0;

    if (arm() != 0) { active = NULL; return -2; }

    log_printf("[i2c-sim] started \"%s\" addr=0x%02x SDA=LA%u SCL=LA%u\n",
           m->name, active_addr7, active_sda, active_scl);
    return 0;
}

int sensor_sim_set_value(const char *key, float value) {
    if (!active) return -1;
    for (unsigned i = 0; i < active->n_params; i++) {
        const sensor_param_t *p = &active->params[i];
        if (strcmp(key, p->key) != 0) continue;
        if (!isfinite(value) || value < p->min || value > p->max) return -3;
        values[i] = value;
        /* Integer int.frac split — newlib-nano printf has no %f. */
        long milli = lroundf(value * 1000.0f);
        log_printf("[i2c-sim] set %s=%s%ld.%03ld\n", key, milli < 0 ? "-" : "",
                   (milli < 0 ? -milli : milli) / 1000, (milli < 0 ? -milli : milli) % 1000);
        return 0;
    }
    return -2;
}

int sensor_sim_apply(void) {
    if (!active) return -1;
    if (rearm_pending) return 0;   /* the re-arm on the next poll loads every value */
    return reload_data();
}

int sensor_sim_set(const char *key, float value) {
    int rc = sensor_sim_set_value(key, value);
    return rc != 0 ? rc : sensor_sim_apply();
}

void sensor_sim_stop(void) {
    if (!active) return;
    fpga_i2c_sensor_disable();
    log_printf("[i2c-sim] stopped \"%s\"\n", active->name);
    active = NULL;
    rearm_pending = false;
}

int sensor_sim_poll(void) {
    if (!active) return 0;
    if (rearm_pending) {
        rearm_pending = false;
        if (arm() != 0) {
            log_printf("[i2c-sim] gateware reconfigured: could not re-arm \"%s\", stopped\n",
                       active->name);
            active = NULL;
            return -1;
        }
        log_printf("[i2c-sim] gateware reconfigured: \"%s\" re-armed\n", active->name);
        return 0;
    }
    if (!active->watch_writes) return 0;
    uint64_t now = time_us_64();
    if (now < next_watch_us) return 0;
    next_watch_us = now + WATCH_PERIOD_US;
    i2c_sensor_status_t st = {0};
    if (fpga_i2c_sensor_status(&st) != 0 || st.wr_count == seen_wr_count) return 0;
    seen_wr_count = st.wr_count;
    (void)reload_data();
    return 0;
}

void sensor_sim_on_gateware_reconfigured(void) {
    if (active) rearm_pending = true;
}

bool                  sensor_sim_active(void) { return active != NULL; }
const sensor_model_t *sensor_sim_model(void)  { return active; }
const char *sensor_sim_type(void)   { return active ? active->name : ""; }
uint8_t     sensor_sim_addr7(void)  { return active_addr7; }
unsigned    sensor_sim_sda(void)    { return active ? active_sda : 0u; }
unsigned    sensor_sim_scl(void)    { return active ? active_scl : 0u; }
float       sensor_sim_value(unsigned i) {
    return (active && i < active->n_params) ? values[i] : 0.0f;
}

int sensor_sim_get_status(i2c_sensor_status_t *out) {
    if (!active || !out) return -1;
    return fpga_i2c_sensor_status(out);
}

int sensor_sim_read_regs(uint8_t start_addr, uint8_t *buf, size_t len) {
    if (!active) return -1;
    return fpga_i2c_read_regs(start_addr, buf, len);
}
