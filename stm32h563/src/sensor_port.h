/*
 * sensor_port.h — what the sensor manager (sensor_mgr.c) needs from the pod besides the
 * emulation gateware (fpga_emu.h): the clock, the heap, the gateware image, the analog outputs
 * and the LA pulls. sensor_port.c implements it on the pod; the host tests fake it.
 */
#ifndef SENSOR_PORT_H
#define SENSOR_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sensor_mgr.h"

uint64_t sensor_port_now_us(void);
void    *sensor_port_alloc(size_t n);    /* NULL when the heap is short */
void     sensor_port_free(void *p);
size_t   sensor_port_heap_free(void);    /* for the out-of-memory message */

/* The definition compiler (sd_compile, sensor_def.h). On the pod it runs from the W25Q: the prog
   image (xip_prog_mod.c), inside one XIP session on the hw worker, so a capture or an upload
   running (BP0048) or a missing xip_prog blob (BP0083) refuses it. The host tests call it directly.
   sensor_port_compile_ready: 0 when a compile would start now, else <0 with the reason in err (the
   refusal sensor_define would give after the upload: asked before the first chunk). */
#define SM_COMPILE_REFUSED (-1000)   /* sensor_port_compile: no compile ran, err holds the reason */
int  sensor_port_compile_ready(char *err, size_t cap);
/* sd_compile's own result (SD_OK, or SD_E_* with *se filled), or SM_COMPILE_REFUSED with err set. */
int  sensor_port_compile(const char *json, size_t json_len, void *prog, size_t prog_cap, void *work,
                         size_t work_cap, size_t *prog_len, sd_err_t *se, char *err, size_t cap);

/* The running gateware image: 0 loop, 1 deep, 2 emulation, -1 unknown. */
int  sensor_port_image(void);
/* Load the emulation image (about 2 s). 0 ok; <0 with the reason in err. The reconfiguration
   calls sm_on_gateware_reconfigured like every other one. */
int  sensor_port_load_emulation(char *err, size_t cap);
/* A DAC stream (generate, replay, the control loop) is running: STATUS bit 0. */
bool sensor_port_dac_running(void);

/* Drive an analog output: volts on the DAC (the path picked from max_v: 3v3, 5v or 12v), or
   milliamps on the 4-20 mA output. 0 ok, <0 with err. */
int  sensor_port_analog_set(sm_out_t out, double value, double max_v, char *err, size_t cap);
/* The instance on that output stopped: the DAC stops, the current loop goes to 4 mA. */
void sensor_port_analog_release(sm_out_t out);
/* The 4-20 mA output shares the DAC with the voltage outputs (no quad DAC): true. */
bool sensor_port_current_shares_dac(void);
/* The LA I/O voltage was chosen (la_voltage): digital sensors need it. */
bool sensor_port_la_voltage_set(void);
/* Engaged LA pulls, bit la-1 (la_pins pull compatibility). */
uint16_t sensor_port_pull_mask(void);
/* The capabilities changed (an image swap): re-announce them to the cloud. */
void sensor_port_caps_changed(void);

#endif /* SENSOR_PORT_H */
