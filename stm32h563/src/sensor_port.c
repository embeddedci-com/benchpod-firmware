/*
 * sensor_port.c — the pod side of sensor_port.h: FreeRTOS heap, HAL clock, the image swap and
 * the analog outputs the sensor manager drives.
 */
#include "sensor_port.h"
#include "bp_log.h"
#include "command_handler.h"   /* la_voltage_from_profile */
#include "signal_engine.h"
#include "gw_caps.h"
#include "ice40_flash.h"
#include "blob_store.h"
#include "i2c_bus.h"
#include "cal_data.h"
#include "current_out.h"
#include "dac_quad.h"
#include "dac_limits.h"
#include "cloud_client.h"
#include "sys_health.h"
#include "command_handler_internal.h"
#include "bp_errors.h"
#include "la_pull.h"                    /* la_pull_mask_now */
#include "xip_session.h"
#include "hw_worker.h"
#include "pico/time.h"

#include <math.h>
#include <stdio.h>

void *pvPortMalloc(size_t size);
void  vPortFree(void *p);

uint64_t sensor_port_now_us(void) { return time_us_64(); }
void    *sensor_port_alloc(size_t n) { return pvPortMalloc(n); }
void     sensor_port_free(void *p)   { if (p) vPortFree(p); }
size_t   sensor_port_heap_free(void) { return sys_health_heap_free(); }

/* ---- the definition compiler, from the xip_prog image ---- */
#if defined(BENCHPOD_EMI_TEST)
/* The EMC test build links the compiler into internal flash (the linker script's
   XIP_PROG_SENSOR_COMPILE lines are dropped, Makefile) and defines its emi_uart sensor itself
   with no XIP session: it runs on boards that cannot map the W25Q. */
int sensor_port_compile_ready(char *err, size_t cap) { (void)err; (void)cap; return 0; }
int sensor_port_compile(const char *json, size_t json_len, void *prog, size_t prog_cap, void *work,
                        size_t work_cap, size_t *prog_len, sd_err_t *se, char *err, size_t cap)
{
    (void)err; (void)cap;
    return sd_compile(json, json_len, prog, prog_cap, work, work_cap, prog_len, se);
}
#else
/* Why no session: the busy answers are BP0048 (what holds the bus, bus_busy_reason's words), a
   missing or foreign xip_prog blob BP0083 with the install-the-blobs fix. */
static void compile_refused(xip_err_t xe, char *err, size_t cap)
{
    const char *busy = bus_busy_reason();
    if (xe == XIP_ERR_POOL_BUSY || xe == XIP_ERR_ACTIVE) {
        snprintf(err, cap, "%s", busy ? busy : bp_tag_storage_bus_busy(
                 "busy: a capture, upload or flash is using the sample buffer; try again when it ends",
                 "a capture or upload is running"));
    } else if (xe == XIP_ERR_SLOT || xe == XIP_ERR_NO_IMAGE) {
        bp_note_xip_unavailable(err, cap, "sensor_define needs the xip_prog blob, which is missing or "
                                "from another build: install the blobs (benchpod install-blobs)");
    } else {
        bp_note_xip_unavailable(err, cap, xip_img_err_text(XIP_IMG_PROG, xe));
    }
}

int sensor_port_compile_ready(char *err, size_t cap)
{
    const char *busy = bus_busy_reason();   /* the test every user of the W25Q bus makes first */
    if (busy) { snprintf(err, cap, "%s", busy); return -1; }
    xip_err_t xe = xip_image_check_slot(XIP_IMG_PROG);
    if (xe != XIP_OK) { compile_refused(xe, err, cap); return -1; }
    return 0;
}

int sensor_port_compile(const char *json, size_t json_len, void *prog, size_t prog_cap, void *work,
                        size_t work_cap, size_t *prog_len, sd_err_t *se, char *err, size_t cap)
{
    /* One XIP session at a time, on the hw worker only (xip_session.h): every sensor command runs
       there (command_handler_sensors.c). */
    if (!hw_worker_is_current()) { snprintf(err, cap, "internal error: sensor_define off the hw worker"); return SM_COMPILE_REFUSED; }
    if (sensor_port_compile_ready(err, cap) != 0) return SM_COMPILE_REFUSED;
    xip_err_t xe = xip_session_begin(XIP_IMG_PROG);
    if (xe != XIP_OK) { compile_refused(xe, err, cap); return SM_COMPILE_REFUSED; }
    sd_compile_args_t a = { json, json_len, prog, prog_cap, work, work_cap, prog_len, se };
    int rc = xip_session_call_ctx(XIP_PROG_FN_SD_COMPILE, &a);
    xip_session_end();
    return rc;
}
#endif

int sensor_port_image(void) { return signal_engine_image_kind(); }

int sensor_port_load_emulation(char *err, size_t cap) {
    if (!gw_caps()->emulation) {
        bp_note_gateware_too_old(err, cap, "I2C, SPI and UART sensors", EMULATION_MIN_GW);
        return -1;
    }
    if (!blob_store_present(BLOB_GW2)) {
        snprintf(err, cap, "the emulation image is not installed (W25Q slot gw2 is empty): install the gateware blobs");
        return -1;
    }
    if (bus_busy_reason()) { snprintf(err, cap, "%s", bus_busy_reason()); return -1; }
    LOG_D("[sensor] loading the emulation image\n");
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

bool     sensor_port_la_voltage_set(void) { return la_voltage_from_profile(); }   /* from the profile when unset */
uint16_t sensor_port_pull_mask(void) { return la_pull_mask_now(); }
void     sensor_port_caps_changed(void) { cloud_client_request_caps_resend(); }
