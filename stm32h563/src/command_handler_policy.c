/*
 * command_handler_policy.c — the sig_policy and lan_policy JSON commands
 * (docs/design/policy-commands.md, pod_policy.h).
 */
#include "command_handler.h"
#include "command_handler_internal.h"
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

/* {"cmd":"cloud_ca"[,"clear":true]} -> {"present":bool,"certs":[{"subject":..,"sha256":..}]}
   Installing one goes through the upload path (OTA target "ca"). */
void handle_cloud_ca(int conn_id, const char *json) {
    char v[8] = {0};
    if (bp_json_get(json, "clear", v, sizeof(v)) && strcmp(v, "true") == 0) {
        const char *why = cloud_extras_ca_clear();
        if (why) { send_error(conn_id, why); return; }
    }
    static char certs[640];
    int n = cloud_extras_ca_describe(certs, sizeof(certs));
    static char data[700];
    snprintf(data, sizeof(data), "{\"present\":%s,\"certs\":[%s]}", n > 0 ? "true" : "false", certs);
    send_ok_str(conn_id, data);
}

/* {"cmd":"cloud_proxy"[,"set":"host:port","user":..,"password":..|"clear":true]}
   -> {"host":..,"port":N,"auth":bool}, or {} when none (never the password). */
void handle_cloud_proxy(int conn_id, const char *json) {
    char spec[80] = {0}, user[CLOUD_PROXY_USER_MAX] = {0}, pass[CLOUD_PROXY_PASS_MAX] = {0}, v[8] = {0};
    const char *why = NULL;
    if (bp_json_get(json, "set", spec, sizeof(spec))) {
        bp_json_get(json, "user", user, sizeof(user));
        bp_json_get(json, "password", pass, sizeof(pass));
        why = cloud_extras_proxy_set(spec, user, pass);
        memset(pass, 0, sizeof(pass));
    } else if (bp_json_get(json, "clear", v, sizeof(v)) && strcmp(v, "true") == 0) {
        why = cloud_extras_proxy_set(NULL, NULL, NULL);
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
