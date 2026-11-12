/*
 * sensor_port.c — the pod side of sensor_port.h: FreeRTOS heap, HAL clock, the image swap and
 * the analog outputs the sensor manager drives.
 */
#include "sensor_port.h"
#include "signal_engine.h"
#include "ice40_flash.h"
#include "blob_store.h"
#include "i2c_bus.h"
#include "cal_data.h"
#include "current_out.h"
#include "dac_quad.h"
#include "dac_limits.h"
#include "cloud_client.h"
#include "sys_health.h"
#include "command_handler_internal.h"   /* la_pull_mask_now */
#include "pico/time.h"

#include <math.h>
#include <stdio.h>

void *pvPortMalloc(size_t size);
void  vPortFree(void *p);

uint64_t sensor_port_now_us(void) { return time_us_64(); }
void    *sensor_port_alloc(size_t n) { return pvPortMalloc(n); }
void     sensor_port_free(void *p)   { if (p) vPortFree(p); }
size_t   sensor_port_heap_free(void) { return sys_health_heap_free(); }

int sensor_port_image(void) { return signal_engine_image_kind(); }

int sensor_port_load_emulation(char *err, size_t cap) {
    if (signal_engine_fpga_version() < EMULATION_MIN_GW) {
        snprintf(err, cap, "I2C, SPI and UART sensors need gateware v54 or later (the pod runs v%u)",
                 (unsigned)signal_engine_fpga_version());
        return -1;
    }
    if (!blob_store_present(BLOB_GW2)) {
        snprintf(err, cap, "the emulation image is not installed (W25Q slot gw2 is empty): install the gateware blobs");
        return -1;
    }
    if (bus_busy_reason()) { snprintf(err, cap, "%s", bus_busy_reason()); return -1; }
    printf("[sensor] loading the emulation image\n");
    int rc = fpga_warmboot(2);
    if (rc != 0) {
        snprintf(err, cap, "loading the emulation image failed (%d); the pod is back on the loop image", rc);
        return -1;
    }
    cloud_client_request_caps_resend();
    if (dac_limits_get()->enabled) (void)dac_limits_park_now();
    return 0;
}

bool sensor_port_dac_running(void) {
    uint8_t st = 0;
    return fpga_status_read(&st) == 0 && (st & 0x01u) && !(st & 0x80u);
}

bool sensor_port_current_shares_dac(void) { return !dac_quad_present(); }

/* The path routed for the DAC sensor: routed once, not on every value (the relays would click). */
static int s_routed = -1;

int sensor_port_analog_set(sm_out_t out, double value, double max_v, char *err, size_t cap) {
    if (out == SM_OUT_CURRENT) {
        uint16_t code = 0;
        const char *why = current_out_code(lround(value * 1000.0), &code);
        if (why) { snprintf(err, cap, "%s", why); return -1; }
        if (dac_quad_present()) {
            if (dac_quad_current_out(code) != 0) { snprintf(err, cap, "dac set failed"); return -1; }
        } else {
            if (s_routed != (int)ANALOG_PATH_CURRENT_OUT) {
                if (analog_path_set(ANALOG_PATH_CURRENT_OUT) != 0) { snprintf(err, cap, "route failed"); return -1; }
                s_routed = (int)ANALOG_PATH_CURRENT_OUT;
            }
            if (dac_set_constant16(code, 240) != 0) { snprintf(err, cap, "dac set failed"); return -1; }
        }
        return 0;
    }
    /* the smallest voltage path that covers the definition's range */
    int idx = max_v <= 3.3 ? 0 : max_v <= 5.0 ? 1 : 2;
    analog_path_t p = idx == 0 ? ANALOG_PATH_DAC_3V3 : idx == 1 ? ANALOG_PATH_DAC_5V : ANALOG_PATH_DAC_12V;
    if (s_routed != (int)p) {
        if (analog_path_set(p) != 0) { snprintf(err, cap, "route failed"); return -1; }
        s_routed = (int)p;
    }
    if (dac_set_constant16(dac_code16_for_volts(idx, (float)value), 240) != 0) { snprintf(err, cap, "dac set failed"); return -1; }
    return 0;
}

void sensor_port_analog_release(sm_out_t out) {
    if (out == SM_OUT_DAC) s_routed = -1;
    if (out == SM_OUT_CURRENT && dac_quad_present()) {
        uint16_t code = 0;
        if (!current_out_code(4000, &code)) (void)dac_quad_current_out(code);
        return;
    }
    dac_stop();
}

bool     sensor_port_la_voltage_set(void) { return la_vccio_get_mv() != LA_VCCIO_UNSET; }
uint16_t sensor_port_pull_mask(void) { return la_pull_mask_now(); }
void     sensor_port_caps_changed(void) { cloud_client_request_caps_resend(); }
