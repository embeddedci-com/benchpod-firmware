/*
 * command_handler_sensors.c — the emulated-sensor verbs (docs/API.md "Emulated sensors"): the
 * JSON is in sensor_cmd.c, the manager in sensor_mgr.c; this sends what they build.
 * sensor_la (a stream) is in command_handler_capture.c.
 */
#include "command_handler_internal.h"
#include "at_driver.h"
#include "bp_json.h"
#include "bp_limits.h"
#include "sensor_cmd.h"
#include "sensor_mgr.h"
#include "sensor_port.h"

#include <stdio.h>

void handle_sensor(int conn_id, const char *json) {
    char verb[32] = { 0 };
    bp_json_get(json, "cmd", verb, sizeof(verb));
    /* the reply buffer only for this call (RAM is tight; every reply fits BP_CLOUD_REPLY_MAX) */
    char *resp = sensor_port_alloc(BP_CLOUD_REPLY_MAX);
    if (!resp) { send_error(conn_id, "out of memory"); return; }
    char err[SM_ERR_MAX];
    bp_emit_t e;
    bp_emit_init(&e, resp, BP_CLOUD_REPLY_MAX);
    if (sensor_cmd(verb, json, &e, err, sizeof(err)) != 0) send_error(conn_id, err);
    else if (!bp_emit_ok(&e)) send_error(conn_id, "reply too large");
    else {
        printf("[cmd] -> ok  %s (%u bytes)\n", verb, (unsigned)bp_emit_len(&e));
        if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) at_close_connection(conn_id);
    }
    sensor_port_free(resp);
}
