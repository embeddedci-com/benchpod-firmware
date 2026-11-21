/*
 * lan_clients.c — the list of paired computers (see lan_clients.h). Pure and host-tested.
 */
#include "lan_clients.h"
#include "bp_json.h"   /* bp_hex_nibble */

#include <string.h>

void lan_clients_init(lan_clients_t *l, uint8_t cap) {
    memset(l, 0, sizeof(*l));
    l->cap = cap > LAN_CLIENTS_MAX ? LAN_CLIENTS_MAX : cap;
}

int lan_clients_find(const lan_clients_t *l, const uint8_t fp[LAN_CLIENT_FP_LEN]) {
    for (int i = 0; i < l->n; i++)
        if (!memcmp(l->c[i].fp, fp, LAN_CLIENT_FP_LEN)) return i;
    return -1;
}

bool lan_clients_name_ok(const char *name) {
    if (!name) return false;
    size_t n = strnlen(name, LAN_CLIENT_NAME_MAX);
    if (n == 0 || n >= LAN_CLIENT_NAME_MAX) return false;
    for (size_t i = 0; i < n; i++) {
        unsigned char ch = (unsigned char)name[i];
        if (ch < 0x20 || ch > 0x7e || ch == '"' || ch == '\\') return false;
    }
    return true;
}

static bool is_hex(const char *s, size_t n) {
    for (size_t i = 0; i < n; i++) if (bp_hex_nibble(s[i]) < 0) return false;
    return true;
}

bool lan_clients_fp_from_hex(const char *hex, uint8_t fp[LAN_CLIENT_FP_LEN]) {
    if (!hex || strlen(hex) != 2 * LAN_CLIENT_FP_LEN || !is_hex(hex, 2 * LAN_CLIENT_FP_LEN)) return false;
    for (int i = 0; i < LAN_CLIENT_FP_LEN; i++)
        fp[i] = (uint8_t)(bp_hex_nibble(hex[2 * i]) << 4 | bp_hex_nibble(hex[2 * i + 1]));
    return true;
}

void lan_clients_fp_to_hex(const uint8_t fp[LAN_CLIENT_FP_LEN], char out[2 * LAN_CLIENT_FP_LEN + 1]) {
    static const char d[] = "0123456789abcdef";
    for (int i = 0; i < LAN_CLIENT_FP_LEN; i++) {
        out[2 * i] = d[fp[i] >> 4];
        out[2 * i + 1] = d[fp[i] & 15];
    }
    out[2 * LAN_CLIENT_FP_LEN] = '\0';
}

lan_clients_add_t lan_clients_add(lan_clients_t *l, const uint8_t fp[LAN_CLIENT_FP_LEN], const char *name) {
    if (!lan_clients_name_ok(name)) return LAN_CLIENTS_BAD_NAME;
    int i = lan_clients_find(l, fp);
    if (i >= 0) {
        if (!strcmp(l->c[i].name, name)) return LAN_CLIENTS_SAME;
        memset(l->c[i].name, 0, LAN_CLIENT_NAME_MAX);
        strcpy(l->c[i].name, name);
        return LAN_CLIENTS_RENAMED;
    }
    if (l->n >= l->cap) return LAN_CLIENTS_FULL;
    lan_client_t *c = &l->c[l->n++];
    memset(c, 0, sizeof(*c));
    memcpy(c->fp, fp, LAN_CLIENT_FP_LEN);
    strcpy(c->name, name);
    return LAN_CLIENTS_ADDED;
}

static void drop(lan_clients_t *l, int i) {
    memmove(&l->c[i], &l->c[i + 1], (size_t)(l->n - i - 1) * sizeof(l->c[0]));
    l->n--;
    memset(&l->c[l->n], 0, sizeof(l->c[0]));
}

int lan_clients_remove(lan_clients_t *l, const char *what) {
    if (!what || !*what) return 0;
    int removed = 0;
    for (int i = 0; i < l->n;) {
        if (!strcmp(l->c[i].name, what)) { drop(l, i); removed++; } else i++;
    }
    if (removed) return removed;

    size_t n = strlen(what);
    if (n < 8 || n > 2 * LAN_CLIENT_FP_LEN || !is_hex(what, n)) return 0;
    int hit = -1;
    for (int i = 0; i < l->n; i++) {
        char hex[2 * LAN_CLIENT_FP_LEN + 1];
        lan_clients_fp_to_hex(l->c[i].fp, hex);
        bool match = true;
        for (size_t k = 0; k < n && match; k++) {
            char a = what[k];
            if (a >= 'A' && a <= 'F') a = (char)(a - 'A' + 'a');
            match = a == hex[k];
        }
        if (!match) continue;
        if (hit >= 0) return -2;
        hit = i;
    }
    if (hit < 0) return 0;
    drop(l, hit);
    return 1;
}

void lan_clients_clear(lan_clients_t *l) {
    uint8_t cap = l->cap;
    lan_clients_init(l, cap);
}

size_t lan_clients_packed_size(uint8_t n) { return 1u + (size_t)n * sizeof(lan_client_t); }

size_t lan_clients_pack(const lan_clients_t *l, uint8_t *out, size_t cap) {
    size_t need = lan_clients_packed_size(l->n);
    if (need > cap) return 0;
    out[0] = l->n;
    memcpy(out + 1, l->c, (size_t)l->n * sizeof(lan_client_t));
    return need;
}

bool lan_clients_unpack(lan_clients_t *l, const uint8_t *in, size_t len) {
    if (len < 1) return false;
    uint8_t n = in[0];
    if (n > l->cap || len != lan_clients_packed_size(n)) return false;
    lan_clients_t t;
    lan_clients_init(&t, l->cap);
    memcpy(t.c, in + 1, (size_t)n * sizeof(lan_client_t));
    t.n = n;
    for (int i = 0; i < n; i++) {
        if (t.c[i].name[LAN_CLIENT_NAME_MAX - 1] != '\0' || !lan_clients_name_ok(t.c[i].name)) return false;
        for (int j = 0; j < i; j++)
            if (!memcmp(t.c[i].fp, t.c[j].fp, LAN_CLIENT_FP_LEN)) return false;
    }
    *l = t;
    return true;
}
