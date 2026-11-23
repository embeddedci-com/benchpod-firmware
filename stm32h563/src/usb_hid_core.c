/* usb_hid_core.c — see usb_hid_core.h. */
#include "usb_hid_core.h"

#include <stdio.h>
#include <string.h>

void usb_hid_ring_init(usb_hid_ring_t *r, usb_hid_rep_t *storage, uint8_t cap)
{
    memset(r, 0, sizeof(*r));
    r->slot = storage;
    r->cap = cap;
}

bool usb_hid_ring_push(usb_hid_ring_t *r, uint32_t ms, uint8_t inst, const uint8_t *data, size_t len)
{
    if (r->count >= r->cap) {
        if (r->dropped < UINT16_MAX) r->dropped++;
        return false;
    }
    usb_hid_rep_t *e = &r->slot[(uint8_t)((r->head + r->count) % r->cap)];
    if (len > USB_HID_REPORT_MAX) len = USB_HID_REPORT_MAX;
    e->ms = ms;
    e->inst = inst;
    e->len = (uint8_t)len;
    if (len) memcpy(e->data, data, len);
    r->count++;
    return true;
}

bool usb_hid_ring_pop(usb_hid_ring_t *r, usb_hid_rep_t *out)
{
    if (r->count == 0) return false;
    *out = r->slot[r->head];
    r->head = (uint8_t)((r->head + 1u) % r->cap);
    r->count--;
    return true;
}

size_t usb_hid_hex(const uint8_t *p, size_t n, char *out, size_t cap)
{
    static const char d[] = "0123456789abcdef";
    if (cap < 2 * n + 1) { if (cap) out[0] = '\0'; return 0; }
    for (size_t i = 0; i < n; i++) { out[2 * i] = d[p[i] >> 4]; out[2 * i + 1] = d[p[i] & 15u]; }
    out[2 * n] = '\0';
    return 2 * n;
}

const char *usb_hid_protocol_name(uint8_t protocol)
{
    return protocol == 1 ? "keyboard" : protocol == 2 ? "mouse" : "none";
}

#define ITF_OVERHEAD 150u   /* one interface's JSON without its descriptor bytes */

size_t usb_hid_list_json(const usb_hid_itf_t *itf, unsigned n, bool attached, size_t offset, char *out, size_t cap)
{
    size_t o = (size_t)snprintf(out, cap, "{\"attached\":%s,\"interfaces\":[", attached ? "true" : "false");
    if (o >= cap) return 0;
    size_t share = n ? (cap - o - 4u) / n : 0;
    size_t bytes = share > ITF_OVERHEAD ? (share - ITF_OVERHEAD) / 2u : 0;
    for (unsigned i = 0; i < n; i++) {
        const usb_hid_itf_t *t = &itf[i];
        size_t off = offset < t->desc_len ? offset : t->desc_len;
        size_t take = t->desc ? t->desc_len - off : 0;
        if (take > bytes) take = bytes;
        int w = snprintf(out + o, cap - o,
                         "%s{\"instance\":%u,\"interface\":%u,\"protocol\":\"%s\",\"descriptor_len\":%u,"
                         "\"descriptor_offset\":%u,\"truncated\":%s,\"descriptor\":\"",
                         i ? "," : "", t->inst, t->itf, usb_hid_protocol_name(t->protocol), t->desc_len,
                         (unsigned)off, (off + take < t->desc_len) ? "true" : "false");
        if (w < 0 || o + (size_t)w >= cap) return 0;
        o += (size_t)w;
        if (take) {
            if (usb_hid_hex(t->desc + off, take, out + o, cap - o) == 0) return 0;
            o += 2 * take;
        }
        if (o + 4 >= cap) return 0;
        out[o++] = '"'; out[o++] = '}';
    }
    if (o + 3 > cap) return 0;
    out[o++] = ']'; out[o++] = '}'; out[o] = '\0';
    return o;
}

size_t usb_hid_reports_json(usb_hid_ring_t *r, unsigned max, char *out, size_t cap)
{
    static const char tail[] = "],\"count\":%u,\"dropped\":%u}";
    const size_t reserve = 48;   /* the tail */
    size_t o = (size_t)snprintf(out, cap, "{\"reports\":[");
    if (cap < o + reserve) return 0;
    unsigned n = 0;
    usb_hid_rep_t e;
    while (n < max && r->count) {
        if (o + reserve + 64u + 2u * USB_HID_REPORT_MAX > cap) break;   /* the next one may not fit */
        (void)usb_hid_ring_pop(r, &e);
        int w = snprintf(out + o, cap - o, "%s{\"ms\":%lu,\"instance\":%u,\"data\":\"", n ? "," : "",
                         (unsigned long)e.ms, e.inst);
        o += (size_t)w;
        o += usb_hid_hex(e.data, e.len, out + o, cap - o);
        out[o++] = '"'; out[o++] = '}';
        n++;
    }
    int w = snprintf(out + o, cap - o, tail, n, r->dropped);
    return (w < 0 || o + (size_t)w >= cap) ? 0 : o + (size_t)w;
}
