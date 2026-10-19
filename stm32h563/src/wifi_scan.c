/*
 * wifi_scan.c — the Wi-Fi network list shared by the built-in Wi-Fi and the Wi-Fi bridge (see
 * wifi_scan.h).
 */
#include "wifi_scan.h"

#include <string.h>

void wifi_scan_list_clear(wifi_scan_list_t *l) { memset(l, 0, sizeof(*l)); }

static void remove_at(wifi_scan_list_t *l, unsigned i) {
    memmove(&l->ap[i], &l->ap[i + 1], (size_t)(l->n - i - 1u) * sizeof(l->ap[0]));
    l->n--;
}

void wifi_scan_list_add(wifi_scan_list_t *l, const char *ssid, size_t ssid_len, int rssi,
                        unsigned channel, unsigned authmode) {
    wifi_scan_ap_t a;
    memset(&a, 0, sizeof(a));
    if (ssid_len > sizeof(a.ssid) - 1u) ssid_len = sizeof(a.ssid) - 1u;
    memcpy(a.ssid, ssid, ssid_len);
    a.rssi     = (int8_t)(rssi < -128 ? -128 : rssi > 0 ? 0 : rssi);
    a.channel  = (uint8_t)(channel > 255u ? 0u : channel);
    a.authmode = (uint8_t)(authmode > 255u ? 255u : authmode);
    if (l->seen < UINT16_MAX) l->seen++;

    if (a.ssid[0]) {                  /* one entry per network: the strongest access point */
        for (unsigned i = 0; i < l->n; i++) {
            if (strcmp(l->ap[i].ssid, a.ssid) != 0) continue;
            if (l->ap[i].rssi >= a.rssi) return;
            remove_at(l, i);
            break;
        }
    }
    unsigned pos = 0;                 /* strongest first; equal RSSI keeps arrival order */
    while (pos < l->n && l->ap[pos].rssi >= a.rssi) pos++;
    if (pos >= WIFI_SCAN_MAX) return; /* weaker than everything in a full list */
    if (l->n == WIFI_SCAN_MAX) l->n--;
    memmove(&l->ap[pos + 1], &l->ap[pos], (size_t)(l->n - pos) * sizeof(l->ap[0]));
    l->ap[pos] = a;
    l->n++;
}

const char *wifi_scan_auth_str(unsigned authmode) {
    static const char *const names[] = {"open", "wep", "wpa", "wpa2", "wpa/wpa2", "wpa2-ent",
                                        "wpa3", "wpa2/wpa3", "wapi", "owe"};
    return authmode < sizeof(names) / sizeof(names[0]) ? names[authmode] : "other";
}

unsigned wifi_scan_emit_networks(bp_emit_t *e, const wifi_scan_list_t *l, size_t limit) {
    unsigned n = 0;
    bp_emit_raw(e, "\"networks\":[");
    for (unsigned i = 0; i < l->n; i++) {
        char one[256];   /* the longest entry: a 32-character SSID of escaped control bytes */
        bp_emit_t t;
        bp_emit_init(&t, one, sizeof(one));
        bp_emit_raw(&t, n ? ",{\"ssid\":" : "{\"ssid\":");
        bp_emit_jstr(&t, l->ap[i].ssid);
        bp_emit(&t, ",\"rssi\":%d,\"channel\":%u,\"auth\":\"%s\"}", l->ap[i].rssi,
                l->ap[i].channel, wifi_scan_auth_str(l->ap[i].authmode));
        /* +1 for this list's closing bracket (the caller's limit leaves room for its own tail) */
        if (!bp_emit_ok(&t) || bp_emit_len(e) + bp_emit_len(&t) + 1u > limit) break;
        bp_emit_raw(e, one);
        n++;
    }
    bp_emit_raw(e, "]");
    return n;
}
