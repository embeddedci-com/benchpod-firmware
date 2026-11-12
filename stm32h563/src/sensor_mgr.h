/*
 * sensor_mgr.h — the emulated-sensor manager: plays sensor definitions (sensor_def.h) on the
 * emulation gateware (fpga_emu.h) and on the analog outputs.
 *
 * Up to 4 I2C instances (gateware targets 0..3, on one bus or several), 1 SPI instance (the SPI
 * target), 1 UART instance (UART2) and SM_MAX_ANALOG analog instances (the DAC voltage output and
 * the 4-20 mA output, one instance per output). An instance is defined (a definition uploaded in
 * chunks and compiled), started (pins, the gateware configured, its power-on image loaded) and
 * polled: the DUT's register writes come back through the gateware's write log and run through
 * the engine's on_write rules, data-ready and FIFO run on their timers, UART epochs and answers
 * go out of UART2, a stimulus steps the params, faults switch on and off.
 *
 * The JSON commands are in sensor_cmd.c; the hardware hooks the manager needs besides the
 * gateware are in sensor_port.h. Everything runs on the hw worker task. Instances, programs and
 * the upload buffer live on the heap (sensor_port_alloc) only while they are in use: an idle pod
 * holds a few hundred bytes for the manager.
 */
#ifndef SENSOR_MGR_H
#define SENSOR_MGR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bp_json.h"
#include "sensor_def.h"

#define SM_MAX_ANALOG   2u                    /* one per analog output: dac and current */
#define SM_MAX_INST     (SD_MAX_I2C_INST + SD_MAX_SPI_INST + SD_MAX_UART_INST + SM_MAX_ANALOG)
#define SM_NAME_MAX     16u                   /* instance names: [a-z0-9_]{1,16} */
#define SM_CHUNK_IDLE_MS 60000u               /* an upload idle this long is dropped */
#define SM_BUS_STUCK_MAX_MS 1000u             /* bus_stuck lasts at most this long */
#define SM_STIM_MAX_HZ  100.0
#define SM_UART_STAGE   1024u                 /* one UART epoch (bytes) */
#define SM_FAULTS       4u                    /* faults applied at once per instance */

typedef enum { SM_OUT_NONE = 0, SM_OUT_DAC = 1, SM_OUT_CURRENT = 2 } sm_out_t;

/* Pins of a start request: LA channels 1..14, 0 = not given. */
typedef struct {
    uint8_t sda, scl, drdy;          /* i2c (drdy also spi: refused, the SPI target has none) */
    uint8_t sclk, mosi, miso, cs;    /* spi */
    uint8_t tx, rx;                  /* uart: tx = pod to DUT, rx = DUT to pod */
    sm_out_t output;                 /* analog */
} sm_pins_t;

typedef struct {
    sm_pins_t pins;
    int       address;               /* i2c: -1 = the definition's address */
    uint32_t  baud;                  /* uart: 0 = the definition's */
    unsigned  n_values;              /* initial param values */
    const char *keys[SD_MAX_PARAMS];
    double    values[SD_MAX_PARAMS];
} sm_start_req_t;

/* Errors carry a client-facing message in err (at least SM_ERR_MAX bytes). */
#define SM_ERR_MAX 200u

/* ---- uploads (sensor_define / sensor_stimulus) ----
   One upload is in flight at a time; a chunk at offset 0 (re)starts it. Returns 0 when the chunk
   was taken and more are expected, 1 when the last chunk compiled (the reply is in e), <0 with
   err set. data/n are the decoded bytes of this chunk. */
int  sm_define_chunk(const char *inst, uint32_t offset, uint32_t total, const uint8_t *data, size_t n,
                     bool dry_run, bp_emit_t *e, char *err, size_t cap);
int  sm_stim_chunk(uint32_t offset, uint32_t total, const uint8_t *data, size_t n, bool dry_run,
                   bp_emit_t *e, char *err, size_t cap);
/* Bytes received so far of the upload in flight (for the chunk reply). */
uint32_t sm_upload_received(void);

/* ---- instances ---- */
/* 0 ok (reply fields emitted into e), <0 err. */
int  sm_start(const char *inst, const sm_start_req_t *req, bp_emit_t *e, char *err, size_t cap);
/* All values are checked before any is applied. */
int  sm_set(const char *inst, unsigned n, const char *const keys[], const double vals[],
            char *err, size_t cap);
int  sm_fault(const char *inst, const sd_fault_t *f, char *err, size_t cap);
int  sm_fault_clear(const char *inst, char *err, size_t cap);
/* inst NULL = every instance. Releases pins, forgets the definition. 0 ok, <0 unknown. */
int  sm_stop(const char *inst, char *err, size_t cap);
int  sm_stim_start(double at_s, char *err, size_t cap);
void sm_stim_stop(void);
/* The bytes the DUT reads now (its writes included) of an I2C/SPI instance. */
int  sm_regs(const char *inst, unsigned start, unsigned len, uint8_t *out, char *err, size_t cap);
/* An I2C instance's two pins (for sensor_la). */
int  sm_i2c_pins(const char *inst, unsigned *sda, unsigned *scl, char *err, size_t cap);

/* ---- status (sensor_status) ----
   Instances are reported from index `first`; the reply carries "next" when more follow (a reply
   always fits BP_CLOUD_REPLY_MAX). inst non-NULL: only that instance. */
int  sm_status(const char *inst, unsigned first, bp_emit_t *e, char *err, size_t cap);

/* ---- hooks ---- */
void sm_poll(void);
/* The gateware was reconfigured: re-arm the running instances on the next poll (or stop them
   when the emulation image is gone). */
void sm_on_gateware_reconfigured(void);
/* dac_stop: the analog instances on the DAC stop driving it (they stay defined). */
void sm_on_dac_stop(void);
/* The names of the running I2C/SPI/UART instances ("env, imu"), false when none. */
bool sensor_mgr_digital_names(char *out, size_t cap);
/* A command that would fight a running analog sensor for its output: the error to send, or NULL. */
const char *sensor_mgr_command_conflict(const char *verb);
/* Every instance gone, the pins released, the heap given back (tests, a reboot path). */
void sm_reset(void);
/* The instance name of a slot (la_pins owner names), NULL when the slot is free. */
const char *sm_slot_name(unsigned slot);

#endif /* SENSOR_MGR_H */
