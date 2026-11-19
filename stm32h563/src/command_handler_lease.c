/*
 * command_handler_lease.c: lease_acquire, lease_renew and lease_release, the pod's own lease
 * (lease_local.h), on every transport. dispatch_line runs the lease gate (command_handler_lease_gate)
 * for every command and command_handler_conn_closed drops a closed connection's owner flag.
 *
 * Runs on the hw worker task, like the rest of command_handler.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "at_driver.h"
#include "bp_errors.h"
#include "bp_json.h"
#include "cmd_table.h"
#include "lease_gate.h"
#include "lease_local.h"
#include "stm32h5xx_hal.h"   /* HAL_GetTick: the lease deadline clock */

#include <stdio.h>
#include <stdlib.h>

static void send_busy(int conn_id, const char *holder, const char *purpose, uint32_t left_s) {
    char text[160];
    if (purpose && purpose[0])
        snprintf(text, sizeof(text), "busy: %s holds this pod (%s, %lu s left)", holder, purpose,
                 (unsigned long)left_s);
    else
        snprintf(text, sizeof(text), "busy: %s holds this pod (%lu s left)", holder,
                 (unsigned long)left_s);
    bp_err_lease_busy(conn_id, BP_ERR_TEXT(text), holder, purpose ? purpose : "", (int32_t)left_s);
}

static void send_busy_view(int conn_id, const lease_local_view_t *v) {
    send_busy(conn_id, v->holder, v->purpose, v->left_s);
}

/* {"status":"ok","data":{"lease":{...}}}. Labels are printable ASCII without quotes or
   backslashes (lease_local.c), so they go out as they are. The id is the caller's own. */
static void send_lease(int conn_id, const lease_local_view_t *v) {
    char resp[320];
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    bp_emit(&e, "{\"status\":\"ok\",\"data\":{\"lease\":{\"id\":\"%s\",\"holder\":\"%s\","
                "\"purpose\":\"%s\",\"left_s\":%lu,\"source\":\"local\"}}}\n",
            v->id, v->holder, v->purpose, (unsigned long)v->left_s);
    if (!bp_emit_ok(&e)) { send_error(conn_id, "lease reply too large"); return; }
    printf("[cmd] -> ok  lease %s (%s, %lu s)\n", v->holder, v->purpose, (unsigned long)v->left_s);
    if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) at_close_connection(conn_id);
}

static uint32_t get_ttl(const char *json) {
    char t[16] = {0};
    if (!bp_json_get(json, "ttl_s", t, sizeof(t))) return 0;
    long v = strtol(t, NULL, 10);
    return v > 0 ? (uint32_t)v : 1u;   /* given but 0 or negative: the minimum */
}

/* The id, or a refusal sent. A buffer longer than the limit so a long id is refused, not cut. */
static bool get_id(int conn_id, const char *json, char *id, size_t cap) {
    if (!bp_json_get(json, "id", id, cap) || !id[0]) {
        bp_err_missing_argument(conn_id, BP_ERR_TEXT("lease: id missing"), "id");
        return false;
    }
    return true;
}

static void bad_id(int conn_id) {
    bp_err_missing_argument(conn_id,
        BP_ERR_TEXT("lease: id must be 1-64 printable characters, no spaces, quotes or backslashes"),
        "id");
}

void handle_lease_acquire(int conn_id, const char *json) {
    char id[LEASE_LOCAL_ID_MAX + 16], holder[LEASE_LOCAL_HOLDER_MAX + 8] = {0},
         purpose[LEASE_LOCAL_PURPOSE_MAX + 8] = {0};
    if (!get_id(conn_id, json, id, sizeof(id))) return;
    bp_json_get(json, "holder", holder, sizeof(holder));
    bp_json_get(json, "purpose", purpose, sizeof(purpose));
    const uint32_t now = HAL_GetTick();

    /* A cloud job holds the pod (lease_gate.h): nobody takes the local lease over it. */
    uint32_t cloud_left = 0;
    if (lease_gate_active(now, &cloud_left)) {
        const char *h = lease_gate_holder();
        send_busy(conn_id, h && h[0] ? h : "a cloud job", "cloud", cloud_left);
        return;
    }
    lease_local_view_t v;
    switch (lease_local_acquire(conn_id, id, holder, purpose, get_ttl(json), now, &v)) {
        case LEASE_LOCAL_OK:     send_lease(conn_id, &v); break;
        case LEASE_LOCAL_BUSY:   send_busy_view(conn_id, &v); break;
        default:                 bad_id(conn_id); break;
    }
}

void handle_lease_renew(int conn_id, const char *json) {
    char id[LEASE_LOCAL_ID_MAX + 16];
    if (!get_id(conn_id, json, id, sizeof(id))) return;
    lease_local_view_t v;
    switch (lease_local_renew(conn_id, id, get_ttl(json), HAL_GetTick(), &v)) {
        case LEASE_LOCAL_OK:       send_lease(conn_id, &v); break;
        case LEASE_LOCAL_BUSY:     send_busy_view(conn_id, &v); break;
        case LEASE_LOCAL_NOT_HELD:
            bp_err_lease_not_held(conn_id, BP_ERR_TEXT("lease: no lease with this id is held (it ran out, was released or stopped, or the pod restarted)"), id);
            break;
        default:                   bad_id(conn_id); break;
    }
}

void handle_lease_release(int conn_id, const char *json) {
    char id[LEASE_LOCAL_ID_MAX + 16];
    if (!get_id(conn_id, json, id, sizeof(id))) return;
    lease_local_view_t before, v;
    const uint32_t now = HAL_GetTick();
    bool was_held = lease_local_get(conn_id, now, &before);
    if (lease_local_release(conn_id, id, now, &v) == LEASE_LOCAL_BUSY) { send_busy_view(conn_id, &v); return; }
    send_ok_str(conn_id, was_held ? "{\"released\":true}" : "{\"released\":false}");
}

bool command_handler_lease_gate(int conn_id, const char *cmd, const char *json) {
    const cmd_desc_t *d = cmd_find(cmd);
    lease_local_view_t v;
    char by[LEASE_LOCAL_HOLDER_MAX];
    switch (lease_local_check(conn_id, cmd_has(d, CMD_F_LEASE) != 0, cmd_tier_light(cmd, json) != 0,
                              HAL_GetTick(), &v, by, sizeof(by))) {
        case LEASE_LOCAL_REFUSE_LOST: {
            char text[80];
            snprintf(text, sizeof(text), "lease lost: stopped by %s", by);
            bp_err_lease_lost(conn_id, BP_ERR_TEXT(text), by);
            return false;
        }
        case LEASE_LOCAL_REFUSE_BUSY:
            send_busy_view(conn_id, &v);
            return false;
        default:
            return true;
    }
}
