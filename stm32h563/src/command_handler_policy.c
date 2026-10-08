/*
 * command_handler_policy.c — the sig_policy and lan_policy JSON commands
 * (docs/design/policy-commands.md, pod_policy.h).
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "at_driver.h"
#include "pod_policy.h"
#include "fw_sign.h"
#include "bp_json.h"
#include "cloud_extras.h"

#include <string.h>

#include <stdio.h>

/* Where a command came from, for who-may-change-what (pod_policy.h). */
policy_src_t command_handler_policy_src(int conn_id) {
    if (conn_id == CH_CONSOLE_CONN) return POLICY_SRC_USB;
    if (conn_id == CH_CLOUD_CONN ||
        (conn_id >= CH_CLOUD_TUNNEL_CONN && conn_id <= CH_CLOUD_TUNNEL_CONN_LAST))
        return POLICY_SRC_CLOUD;
    return POLICY_SRC_LAN;
}

static void sig_policy_reply(int conn_id) {
    char data[96];
    snprintf(data, sizeof(data), "{\"policy\":\"%s\",\"enforces\":true,\"keys\":%u}",
             fw_sign_policy_name(pod_policy_sig()), (unsigned)fw_sign_key_count());
    send_ok_str(conn_id, data);
}

/* {"cmd":"sig_policy"[,"set":"audit|permissive|required"]} */
void handle_sig_policy(int conn_id, const char *json) {
    char v[16] = {0};
    if (bp_json_get(json, "set", v, sizeof(v))) {
        int p = pod_policy_sig_from_name(v);
        if (p < 0) { send_error(conn_id, "sig_policy: set must be audit, permissive or required"); return; }
        const char *why = pod_policy_set_sig((fw_sig_policy_t)p, command_handler_policy_src(conn_id));
        if (why) { send_error(conn_id, why); return; }
    }
    sig_policy_reply(conn_id);
}

/* {"cmd":"lan_policy"[,"set":"open|locked|off"]} */
void handle_lan_policy(int conn_id, const char *json) {
    char v[16] = {0};
    if (bp_json_get(json, "set", v, sizeof(v))) {
        int p = pod_policy_lan_from_name(v);
        if (p < 0) { send_error(conn_id, "lan_policy: set must be open, locked or off"); return; }
        const char *why = pod_policy_set_lan((pod_lan_policy_t)p, command_handler_policy_src(conn_id));
        if (why) { send_error(conn_id, why); return; }
    }
    char data[48];
    snprintf(data, sizeof(data), "{\"policy\":\"%s\"}", pod_policy_lan_name(pod_policy_lan()));
    send_ok_str(conn_id, data);
}

/* ---- cloud link: company CA and HTTP proxy (cloud_extras.h) ----------------- */

/* One certificate of the cloud_ca reply. The list is capped at CA_CERTS_MAX bytes (the reply's
   one buffer has to hold the rest too); the walk stops at the first one that does not fit. */
#define CA_CERTS_MAX 640u
typedef struct {
    bp_emit_t e;
    size_t    used;   /* bytes of certificate items so far */
} ca_json_t;

static bool ca_json_cert(void *ctx, const char *subject, const char *sha256_hex) {
    ca_json_t *cj = ctx;
    char item[200];
    int w = snprintf(item, sizeof(item), "%s{\"subject\":\"%s\",\"sha256\":\"%s\"}",
                     cj->used ? "," : "", subject, sha256_hex);
    if (w < 0 || (size_t)w >= sizeof(item) || cj->used + (size_t)w >= CA_CERTS_MAX) return false;
    bp_emit_raw(&cj->e, item);
    cj->used += (size_t)w;
    return true;
}

/* {"cmd":"cloud_ca"[,"clear":true]} -> {"present":bool,"certs":[{"subject":..,"sha256":..}][,"error":..]}
   Installing one goes through the upload path (OTA target "ca"). Reading works from anywhere;
   clearing (like installing and the proxy) only from the cloud or USB (pod_policy_cloud_link_gate). */
void handle_cloud_ca(int conn_id, const char *json) {
    char v[8] = {0};
    if (bp_json_get(json, "clear", v, sizeof(v)) && strcmp(v, "true") == 0) {
        const char *why = pod_policy_cloud_link_gate("cloud_ca", command_handler_policy_src(conn_id));
        if (!why) why = bus_busy_reason();   /* the W25Q write takes the shared bus */
        if (!why) why = cloud_extras_ca_clear();
        if (why) { send_error(conn_id, why); return; }
    }
    /* Built here, not with send_ok_str: its 256-byte frame cannot hold a certificate list (a
       long subject, or a chain of several, is several hundred bytes). Worker task only. The
       certificates go in behind the "false" header, which becomes "true" once there is one. */
    static const char hdr_false[] = "{\"status\":\"ok\",\"data\":{\"present\":false,\"certs\":[";
    static const char hdr_true[]  = "{\"status\":\"ok\",\"data\":{\"present\":true,\"certs\":[";
    static char resp[720];
    ca_json_t cj = { .used = 0 };
    bp_emit_init(&cj.e, resp, sizeof(resp));
    bp_emit_raw(&cj.e, hdr_false);
    int n = cloud_extras_ca_each(ca_json_cert, &cj);
    bp_emit_t e = cj.e;
    if (n > 0 && bp_emit_ok(&e)) {
        memmove(resp + sizeof(hdr_true) - 1, resp + sizeof(hdr_false) - 1, e.len - (sizeof(hdr_false) - 1) + 1);
        memcpy(resp, hdr_true, sizeof(hdr_true) - 1);
        e.len -= 1;
    }
    bp_emit_raw(&e, "]");
    /* An installed CA that is corrupt or does not parse is not used (built-in roots only). */
    const char *err = cloud_extras_ca_error();
    if (err) { bp_emit_raw(&e, ",\"error\":"); bp_emit_jstr(&e, err); }
    bp_emit_raw(&e, "}}\n");
    if (!bp_emit_ok(&e)) { send_error(conn_id, "cloud_ca: reply too large"); return; }
    if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) at_close_connection(conn_id);
}

/* {"cmd":"cloud_proxy"[,"set":"host:port","user":..,"password":..|"clear":true]}
   -> {"host":..,"port":N,"auth":bool}, or {} when none (never the password).
   Reading works from anywhere; set and clear only from the cloud or USB (pod_policy_cloud_link_gate). */
void handle_cloud_proxy(int conn_id, const char *json) {
    char spec[80] = {0}, user[CLOUD_PROXY_USER_MAX] = {0}, pass[CLOUD_PROXY_PASS_MAX] = {0}, v[8] = {0};
    const char *why = NULL;
    const policy_src_t src = command_handler_policy_src(conn_id);
    if (bp_json_get(json, "set", spec, sizeof(spec))) {
        bp_json_get(json, "user", user, sizeof(user));
        bp_json_get(json, "password", pass, sizeof(pass));
        why = pod_policy_cloud_link_gate("cloud_proxy", src);
        if (!why) why = bus_busy_reason();   /* the W25Q write takes the shared bus */
        if (!why) why = cloud_extras_proxy_set(spec, user, pass);
        memset(pass, 0, sizeof(pass));
    } else if (bp_json_get(json, "clear", v, sizeof(v)) && strcmp(v, "true") == 0) {
        why = pod_policy_cloud_link_gate("cloud_proxy", src);
        if (!why) why = bus_busy_reason();
        if (!why) why = cloud_extras_proxy_set(NULL, NULL, NULL);
    }
    if (why) { send_error(conn_id, why); return; }
    cloud_proxy_t p;
    cloud_extras_proxy_get(&p);
    char data[160];
    if (p.host[0])
        snprintf(data, sizeof(data), "{\"host\":\"%s\",\"port\":%u,\"auth\":%s}", p.host,
                 (unsigned)p.port, p.user[0] ? "true" : "false");
    else
        snprintf(data, sizeof(data), "{}");
    memset(&p, 0, sizeof(p));
    send_ok_str(conn_id, data);
}
