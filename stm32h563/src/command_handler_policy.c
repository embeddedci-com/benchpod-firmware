/*
 * command_handler_policy.c — the sig_policy and lan_policy JSON commands
 * (docs/design/policy-commands.md, pod_policy.h).
 */
#include "net_server.h"   /* net_conn_on_usb: the USB link may set the CA and proxy */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "bp_errors.h"
#include "at_driver.h"
#include "pod_policy.h"
#include "fw_sign.h"
#include "bp_json.h"
#include "cloud_extras.h"
#include "lan_tls.h"
#include "lan_clients.h"
#include "FreeRTOS.h"

#include <stdlib.h>
#include <string.h>

#include <stdio.h>

/* Where a command came from, for who-may-change-what (pod_policy.h). */
policy_src_t command_handler_policy_src(int conn_id) {
    if (conn_id == CH_CLOUD_CONN ||
        (conn_id >= CH_CLOUD_TUNNEL_CONN && conn_id <= CH_CLOUD_TUNNEL_CONN_LAST))
        return POLICY_SRC_CLOUD;
#if defined(BENCHPOD_EMI_TEST)
    if (conn_id == CH_EMI_CONN) return POLICY_SRC_USB;   /* the pod's own EMI test loop */
#endif
    return POLICY_SRC_LAN;
}

static void sig_policy_reply(int conn_id) {
    char data[160], fl[16];
    uint32_t floor = pod_policy_rollback_floor();
    snprintf(data, sizeof(data),
             "{\"policy\":\"%s\",\"enforces\":true,\"keys\":%u,\"rollback_floor\":%s%s%s}",
             fw_sign_policy_name(pod_policy_sig()), (unsigned)fw_sign_key_count(), floor ? "\"" : "",
             floor ? fw_sign_release_str(floor, fl, sizeof(fl)) : "null", floor ? "\"" : "");
    send_ok_str(conn_id, data);
}

/* {"cmd":"sig_policy"[,"set":"required"][,"clear_floor":true]}: firmware 4.0.0 always requires
   signatures. "set":"required" (a server pushing it) is fine from any source; audit and permissive,
   and the 3.10.x "lan_unsigned":true, are refused with BP0074. "clear_floor":true drops the rollback
   floor, over the USB link only (the console's rollback-floor clear up to 3.10.13). */
void handle_sig_policy(int conn_id, const char *json) {
    char v[16] = {0};
    if (bp_json_get(json, "set", v, sizeof(v))) {
        int p = pod_policy_sig_from_name(v);
        if (p < 0) { send_error(conn_id, "sig_policy: set must be required"); return; }
        const char *why = pod_policy_set_sig((fw_sig_policy_t)p, command_handler_policy_src(conn_id));
        if (why) { send_error(conn_id, why); return; }
    }
    if (bp_json_get(json, "lan_unsigned", v, sizeof(v)) && strcmp(v, "false") != 0) {
        bp_err_signatures_always_required(conn_id, "lan_unsigned", v);
        return;
    }
    if (bp_json_flag(json, "clear_floor")) {
        const char *why = pod_policy_clear_floor(command_handler_policy_src(conn_id), net_conn_on_usb(conn_id));
        if (why) { send_error(conn_id, why); return; }
    }
    sig_policy_reply(conn_id);
}

/* {"cmd":"lan_policy"[,"set":"open|locked|off"][,"pairing":true|false]} */
void handle_lan_policy(int conn_id, const char *json) {
    char v[16] = {0};
    if (bp_json_get(json, "pairing", v, sizeof(v))) {
        if (strcmp(v, "true") != 0 && strcmp(v, "false") != 0) {
            send_error(conn_id, "lan_policy: pairing must be true or false");
            return;
        }
        const char *why = pod_policy_set_lan_pairing(!strcmp(v, "true"), command_handler_policy_src(conn_id),
                                                     net_conn_on_usb(conn_id));
        if (why) { send_error(conn_id, why); return; }
    }
    if (bp_json_get(json, "set", v, sizeof(v))) {
        int p = pod_policy_lan_from_name(v);
        if (p < 0) { send_error(conn_id, "lan_policy: set must be open, locked or off"); return; }
        const char *why = pod_policy_set_lan((pod_lan_policy_t)p, command_handler_policy_src(conn_id));
        if (why) { send_error(conn_id, why); return; }
    }
    char data[64];
    snprintf(data, sizeof(data), "{\"policy\":\"%s\",\"pairing\":%s}", pod_policy_lan_name(pod_policy_lan()),
             pod_policy_lan_pairing() ? "true" : "false");
    send_ok_str(conn_id, data);
}

/* {"cmd":"lan_clients"[,"offset":n]} lists the paired computers (docs/design/lan-pairing.md), 12 a
   reply (the cloud reply limit);
   {"cmd":"lan_clients","add":"<name>","spki_sha256":"<64 hex>"} pairs one (a new name for a key
   already paired renames it);
   {"cmd":"lan_clients","remove":"<name, or 8+ hex digits of a key>"}; {"cmd":"lan_clients","clear":true}.
   The changes come over USB (the USB link; the console has lan-clients) or the cloud, never from
   the external LAN, paired or not. */
#define LAN_CLIENTS_PAGE 12
void handle_lan_clients(int conn_id, const char *json) {
    char name[LAN_CLIENT_NAME_MAX + 8] = {0};
    char hex[2 * LAN_CLIENT_FP_LEN + 8] = {0};
    const bool add = bp_json_get(json, "add", name, sizeof(name));
    const bool rm = !add && bp_json_get(json, "remove", hex, sizeof(hex));
    const bool clr = !add && !rm && bp_json_flag(json, "clear");
    if ((add || rm || clr) && command_handler_policy_src(conn_id) == POLICY_SRC_LAN && !net_conn_on_usb(conn_id)) {
        char why[112];
        send_error(conn_id, bp_note_policy_not_from_lan(why, sizeof(why), "lan_clients"));
        return;
    }
    const char *why = NULL;
    if (add) {
        uint8_t fp[LAN_CLIENT_FP_LEN];
        if (!lan_clients_name_ok(name)) {
            send_error(conn_id, "lan_clients: add needs a name of 1 to 23 printable characters, no quotes");
            return;
        }
        if (!bp_json_get(json, "spki_sha256", hex, sizeof(hex)) || !lan_clients_fp_from_hex(hex, fp)) {
            send_error(conn_id, "lan_clients: add needs spki_sha256, 64 hex digits");
            return;
        }
        lan_clients_add_t res = LAN_CLIENTS_ADDED;
        why = lan_tls_clients_add(fp, name, &res);
        if (!why && res == LAN_CLIENTS_FULL) {
            lan_clients_t *t = pvPortMalloc(sizeof(*t));
            unsigned cap = LAN_CLIENTS_MAX;
            if (t) { lan_tls_clients_get(t); cap = t->cap; vPortFree(t); }
            bp_err_lan_clients_full(conn_id, (int32_t)cap);
            return;
        }
    } else if (rm) {
        int removed = 0;
        why = lan_tls_clients_remove(hex, &removed);
        if (!why && removed == -2) why = "lan_clients: that matches more than one key; give more digits";
        else if (!why && removed == 0) why = "lan_clients: no paired computer has that name or key";
    } else if (clr) {
        why = lan_tls_clients_clear();
    }
    if (why) { send_error(conn_id, why); return; }

    long offset = 0;
    char ov[12] = {0};
    if (bp_json_get(json, "offset", ov, sizeof(ov))) offset = strtol(ov, NULL, 10);
    lan_clients_t *l = pvPortMalloc(sizeof(*l));
    const size_t cap = 1700;   /* 12 x ~115 B + the frame: within BP_CLOUD_REPLY_MAX */
    char *resp = pvPortMalloc(cap);
    if (!l || !resp) {
        if (l) vPortFree(l);
        if (resp) vPortFree(resp);
        send_error(conn_id, "lan_clients: out of memory");
        return;
    }
    lan_tls_clients_get(l);
    if (offset < 0 || offset > l->n) offset = l->n;
    bp_emit_t e;
    bp_emit_init(&e, resp, cap);
    bp_emit(&e, "{\"status\":\"ok\",\"data\":{\"pairing\":%s,\"count\":%u,\"max\":%u,\"offset\":%ld,\"clients\":[",
            pod_policy_lan_pairing() ? "true" : "false", (unsigned)l->n, (unsigned)l->cap, offset);
    for (long i = offset; i < l->n && i < offset + LAN_CLIENTS_PAGE; i++) {
        char fh[2 * LAN_CLIENT_FP_LEN + 1];
        lan_clients_fp_to_hex(l->c[i].fp, fh);
        bp_emit(&e, "%s{\"name\":\"%s\",\"spki_sha256\":\"%s\"}", i > offset ? "," : "", l->c[i].name, fh);
    }
    bp_emit_raw(&e, "]}}\n");
    if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) at_close_connection(conn_id);
    vPortFree(resp);
    memset(l, 0, sizeof(*l));
    vPortFree(l);
}

/* {"cmd":"log_level"[,"set":"error|warn|info|debug"]} -> {"level":"info"}: the console log level
   (log_level.h), stored with the pod policy. Any connection may read or change it. */
void handle_log_level(int conn_id, const char *json) {
    char v[12] = {0};
    if (bp_json_get(json, "set", v, sizeof(v))) {
        int lv = log_level_from_name(v);
        if (lv < 0) {
            send_error(conn_id, "log_level: set must be error, warn, info or debug");
            return;
        }
        const char *why = pod_policy_set_log_level((bp_log_level_t)lv);
        if (why) { send_error(conn_id, why); return; }
    }
    char data[40];
    snprintf(data, sizeof(data), "{\"level\":\"%s\"}", log_level_name(log_level()));
    send_ok_str(conn_id, data);
}

/* {"cmd":"usb_network"[,"set":"on|off"]}: the USB network adapter setting (pod_policy.h). Only
   the device may change it: over the USB link itself here (the console has usb-network). */
void handle_usb_network(int conn_id, const char *json) {
    char v[8] = {0};
    if (bp_json_get(json, "enabled", v, sizeof(v))) {   /* firmware up to 3.10.13: "set":"on"|"off" */
        if (strcmp(v, "true") != 0 && strcmp(v, "false") != 0) {
            send_error(conn_id, "usb_network: enabled must be true or false");
            return;
        }
        const char *why = pod_policy_set_usb_net(!strcmp(v, "true"), command_handler_policy_src(conn_id),
                                                 net_conn_on_usb(conn_id));
        if (why) { send_error(conn_id, why); return; }
    }
    char data[48];
    snprintf(data, sizeof(data), "{\"enabled\":%s}", pod_policy_usb_net() ? "true" : "false");
    send_ok_str(conn_id, data);
}

/* ---- LAN TLS (lan_tls.h, docs/design/lan-tls.md) ----------------------------- */

/* {"cmd":"lan_tls"[,"https_only":true|false][,"scpi":true|false][,"regenerate":true]}
   -> {"port":8443,"https_only":bool,"scpi":bool,"sessions":N,"origin":..,"key":..,"subject":..,
       "san":[..],"not_after":..,"sha256":..[,"error":..]}
   Installing a certificate goes through the upload path (OTA target "tls"). Reading works from
   anywhere. Changing HTTPS-only (on by default from 4.0.0; false = plaintext JSON on :8080 from
   the LAN too) and SCPI (off by default), and regenerating, need the USB console, the USB link or
   the cloud (pod_policy_lan_tls_gate): not the external LAN, TLS or not. */
void handle_lan_tls(int conn_id, const char *json) {
    const policy_src_t src = command_handler_policy_src(conn_id);
    const bool usb = net_conn_on_usb(conn_id);
    char v[8] = {0};
    if (bp_json_get(json, "https_only", v, sizeof(v))) {
        if (strcmp(v, "true") != 0 && strcmp(v, "false") != 0) {
            send_error(conn_id, "lan_tls: https_only must be true or false");
            return;
        }
        const char *why = pod_policy_set_https_only(!strcmp(v, "true"), src, usb);
        if (why) { send_error(conn_id, why); return; }
    }
    if (bp_json_get(json, "scpi", v, sizeof(v))) {
        if (strcmp(v, "true") != 0 && strcmp(v, "false") != 0) {
            send_error(conn_id, "lan_tls: scpi must be true or false");
            return;
        }
        const char *why = pod_policy_set_scpi(!strcmp(v, "true"), src, usb);
        if (why) { send_error(conn_id, why); return; }
    }
    if (bp_json_get(json, "regenerate", v, sizeof(v)) && strcmp(v, "true") == 0) {
        const char *why = pod_policy_lan_tls_gate(src, usb);
        if (!why) why = lan_tls_regenerate();
        if (why) { send_error(conn_id, why); return; }
    }
    lan_tls_info_t in;
    lan_tls_get_info(&in);
    char resp[640];   /* worker stack (12 KB) */
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    bp_emit(&e, "{\"status\":\"ok\",\"data\":{\"port\":%d,\"https_only\":%s,\"scpi\":%s,\"sessions\":%d,",
            LAN_TLS_PORT, pod_policy_https_only() ? "true" : "false", pod_policy_scpi() ? "true" : "false",
            net_tls_sessions());
    if (in.ready) {
        bp_emit_raw(&e, "\"origin\":");    bp_emit_jstr(&e, in.origin);
        bp_emit_raw(&e, ",\"key\":");      bp_emit_jstr(&e, in.key);
        bp_emit_raw(&e, ",\"subject\":");  bp_emit_jstr(&e, in.subject);
        bp_emit_raw(&e, ",\"san\":[");
        for (int i = 0; i < in.nsan; i++) {
            if (i) bp_emit_raw(&e, ",");
            bp_emit_jstr(&e, in.san[i]);
        }
        bp_emit_raw(&e, "],\"not_after\":"); bp_emit_jstr(&e, in.not_after);
        bp_emit_raw(&e, ",\"sha256\":");     bp_emit_jstr(&e, in.sha256);
    } else {
        bp_emit_raw(&e, "\"error\":");
        bp_emit_jstr(&e, in.error[0] ? in.error : "no certificate");
    }
    bp_emit_raw(&e, "}}\n");
    if (!bp_emit_ok(&e)) { bp_err_reply_too_large(conn_id); return; }
    if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) at_close_connection(conn_id);
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
        const char *why = pod_policy_cloud_link_gate("cloud_ca", command_handler_policy_src(conn_id),
                                                     net_conn_on_usb(conn_id));
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
    if (!bp_emit_ok(&e)) { bp_err_reply_too_large(conn_id); return; }
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
        why = pod_policy_cloud_link_gate("cloud_proxy", src, net_conn_on_usb(conn_id));
        if (!why) why = bus_busy_reason();   /* the W25Q write takes the shared bus */
        if (!why) why = cloud_extras_proxy_set(spec, user, pass);
        memset(pass, 0, sizeof(pass));
    } else if (bp_json_get(json, "clear", v, sizeof(v)) && strcmp(v, "true") == 0) {
        why = pod_policy_cloud_link_gate("cloud_proxy", src, net_conn_on_usb(conn_id));
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
