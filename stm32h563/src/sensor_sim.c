/* ============================================================================
 * sensor_sim.c — generic emulated-I2C-sensor manager.  See sensor_sim.h.
 * ========================================================================== */

#include "sensor_sim.h"
#include "sensor_bmp280.h"

#include <string.h>
#include <stdio.h>
#include <math.h>           /* lroundf — newlib-nano printf has no %f */

/* ---- model registry — add new sensors here ---- */
static sensor_model_t *const k_models[] = {
    /* resolved lazily via getters so each model owns its static state */
};

static sensor_model_t *find_model(const char *type) {
    if (strcmp(type, "bmp280") == 0) return sensor_bmp280_model();
    /* add other sensors: if (strcmp(type,"bme280")==0) return sensor_bme280_model(); */
    (void)k_models;
    return NULL;
}

/* ---- active-sensor state ---- */
static sensor_model_t *active = NULL;
static uint8_t  active_addr7  = 0;
static unsigned active_sda    = 0;
static unsigned active_scl    = 0;
static uint8_t  regimg[SENSOR_REGIMAGE_LEN];

static int reload_image(void) {
    active->build_regimage(active, regimg);
    return fpga_i2c_load_regs(0, regimg, sizeof(regimg));
}

int sensor_sim_start(const char *type, uint8_t addr7,
                     unsigned sda_ch, unsigned scl_ch) {
    sensor_model_t *m = find_model(type);
    if (!m) return -1;

    if (active) sensor_sim_stop();

    m->reset(m);
    active       = m;
    active_addr7 = addr7 ? addr7 : m->default_addr7;
    active_sda   = sda_ch;
    active_scl   = scl_ch;

    if (reload_image() != 0) { active = NULL; return -2; }

    if (fpga_i2c_sensor_config(active_addr7, active_sda, active_scl, true,
                               m->trig_reg, m->busy_reg, m->busy_mask,
                               m->conv_us) != 0) {
        active = NULL;
        return -2;
    }

    printf("[i2c-sim] started \"%s\" addr=0x%02x SDA=LA%u SCL=LA%u\n",
           m->name, active_addr7, active_sda, active_scl);
    return 0;
}

int sensor_sim_set(const char *key, float value) {
    if (!active) return -1;
    int rc = active->set_param(active, key, value);
    if (rc != 0) return -2;
    if (reload_image() != 0) return -2;
    /* Integer int.frac split — newlib-nano printf has no %f (the old %.3f printed
       nothing). Scale to milli-units and print the whole/fraction parts as %ld. */
    long milli = lroundf(value * 1000.0f);
    printf("[i2c-sim] set %s=%ld.%03ld -> image reloaded\n",
           key, milli / 1000, (milli < 0 ? -milli : milli) % 1000);
    return 0;
}

void sensor_sim_stop(void) {
    if (!active) return;
    fpga_i2c_sensor_disable();
    printf("[i2c-sim] stopped \"%s\"\n", active->name);
    active = NULL;
}

bool        sensor_sim_active(void) { return active != NULL; }
const char *sensor_sim_type(void)   { return active ? active->name : ""; }
uint8_t     sensor_sim_addr7(void)  { return active_addr7; }
unsigned    sensor_sim_sda(void)    { return active ? active_sda : 0u; }
unsigned    sensor_sim_scl(void)    { return active ? active_scl : 0u; }

int sensor_sim_get_status(i2c_sensor_status_t *out) {
    if (!active || !out) return -1;
    return fpga_i2c_sensor_status(out);
}

int sensor_sim_read_regs(uint8_t start_addr, uint8_t *buf, size_t len) {
    if (!active) return -1;
    return fpga_i2c_read_regs(start_addr, buf, len);
}
