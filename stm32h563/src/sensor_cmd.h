/*
 * sensor_cmd.h — the JSON side of the emulated-sensor commands (docs/API.md "Emulated sensors"):
 * sensor_define, sensor_start, sensor_set, sensor_stimulus, sensor_stimulus_start,
 * sensor_stimulus_stop, sensor_fault, sensor_fault_clear, sensor_stop, sensor_status and
 * sensor_regs. sensor_la streams and lives in command_handler_capture.c.
 *
 * Pure: parses the command, calls the manager (sensor_mgr.h) and builds the whole reply line, so
 * the host tests drive the commands end to end. command_handler_sensors.c sends what this builds.
 */
#ifndef SENSOR_CMD_H
#define SENSOR_CMD_H

#include <stdbool.h>
#include <stddef.h>

#include "bp_json.h"

/* Is `verb` one of the commands above? */
bool sensor_cmd_known(const char *verb);
/* Run one command. 0: e holds the {"status":"ok","data":{...}}\n line. <0: err holds the
   message for {"status":"error"}. e should have BP_CLOUD_REPLY_MAX bytes: every reply fits. */
int  sensor_cmd(const char *verb, const char *json, bp_emit_t *e, char *err, size_t cap);

/* Standard or URL base64, padding optional. 0 ok, -1 malformed or out too small. */
int  sensor_b64_decode(const char *in, size_t n, unsigned char *out, size_t cap, size_t *out_len);

#endif /* SENSOR_CMD_H */
