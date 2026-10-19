/*
 * wifi_bridge_ota.c — relay the bridge firmware from the W25Q to the Wi-Fi bridge
 * (see wifi_bridge_ota.h).
 */
#include "wifi_bridge_ota.h"

#include <stdio.h>
#include <string.h>

#define BUF_LEN (3 + 4 + 3 + WBO_CHUNK)   /* OFFSET + DATA; OTA_BEGIN (7 + 131) fits too */

static bool elapsed(uint32_t now, uint32_t at)
{
    return (int32_t)(now - at) >= 0;
}

uint32_t wbo_pack_version(const char *s)
{
    unsigned a, b, c;
    char tail;
    if (!s || sscanf(s, "%u.%u.%u%c", &a, &b, &c, &tail) != 3 || a > 255 || b > 255 || c > 255) {
        return 0;
    }
    return (uint32_t)a << 16 | (uint32_t)b << 8 | c;
}

void wbo_version_str(uint32_t v, char out[16])
{
    snprintf(out, 16, "%u.%u.%u", (unsigned)(v >> 16) & 0xff, (unsigned)(v >> 8) & 0xff, (unsigned)v & 0xff);
}

void wbo_init(wbo_t *o, const wbo_io_t *io, wb_reply_t *reply)
{
    memset(o, 0, sizeof(*o));
    o->io = *io;
    o->reply = reply;
    strcpy(o->status, "idle");
}

bool wbo_busy(const wbo_t *o)
{
    return o->state >= WBO_BEGIN && o->state <= WBO_REBOOT;
}

void wbo_force(wbo_t *o)
{
    o->forced = true;
    o->next_check = 0;
}

const char *wbo_status(const wbo_t *o)
{
    return o->status;
}

static void stop(wbo_t *o)
{
    if (o->buf) {
        o->io.release(o->buf);
        o->buf = NULL;
    }
    o->state = WBO_IDLE;
    o->forced = false;
}

static void fail(wbo_t *o, const char *why, const char *detail)
{
    char v[16];
    wbo_version_str(o->want, v);
    snprintf(o->status, sizeof(o->status), "update to %s failed: %s%s%s", v, why, detail && detail[0] ? ": " : "",
             detail ? detail : "");
    o->failed = o->want;  /* not again until the pod reboots or `bridge update` */
    stop(o);
}

static void put_tlv(uint8_t *p, uint8_t tag, size_t len)
{
    p[0] = tag;
    p[1] = (uint8_t)(len >> 8);
    p[2] = (uint8_t)len;
}

static void put_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24), p[1] = (uint8_t)(v >> 16), p[2] = (uint8_t)(v >> 8), p[3] = (uint8_t)v;
}

/* Send the chunk at o->off (o->resend set if the slot was busy). */
static void send_chunk(wbo_t *o)
{
    uint32_t n = o->size - o->off < WBO_CHUNK ? o->size - o->off : WBO_CHUNK;
    put_tlv(o->buf, BWP_T_OFFSET, 4);
    put_be32(o->buf + 3, o->off);
    put_tlv(o->buf + 7, BWP_T_DATA, n);
    if (o->io.read(o->io.ctx, o->off, o->buf + 10, n) != 0) {
        fail(o, "the pod could not read the image from its W25Q", NULL);
        return;
    }
    int rc = o->io.start(o->io.ctx, BWP_OTA_DATA, o->buf, 10 + n, 2000);
    o->resend = rc == -1;
    if (rc != 0 && rc != -1) {
        fail(o, "the bridge cannot take a request", NULL);
        return;
    }
    char v[16];
    wbo_version_str(o->want, v);
    snprintf(o->status, sizeof(o->status), "updating to %s: %u%%", v, (unsigned)((uint64_t)o->off * 100 / o->size));
}

static void send_simple(wbo_t *o, uint8_t type, uint32_t timeout)
{
    int rc = o->io.start(o->io.ctx, type, NULL, 0, timeout);
    o->resend = rc == -1;
    if (rc != 0 && rc != -1) {
        fail(o, "the bridge cannot take a request", NULL);
    }
}

static void try_start(wbo_t *o, const wb_info_t *br, const wbo_image_t *img, bool bus_free, uint32_t now)
{
    bool forced = o->forced;
    o->next_check = now + WBO_CHECK_EVERY_MS;
    uint32_t have = br->present ? wbo_pack_version(br->fw_version) : 0;
    const char *why = NULL;
    if (!img->present) {
        why = "the pod holds no bridge image (slot \"bridge\")";
    } else if (!br->present) {
        why = "no Wi-Fi bridge on the Ethernet port";
    } else if (!br->paired) {
        why = "the bridge is not paired with this pod";
    } else if (br->flags & (BWP_F_PENDING_VERIFY | BWP_F_OTA_ACTIVE)) {
        why = "the bridge is busy with another update";
    } else if (!forced && (have == 0 || img->version <= have || img->version == o->failed)) {
        return;  /* up to date (or an unknown version, or one that failed): nothing to do */
    } else if (!bus_free) {
        snprintf(o->status, sizeof(o->status), "update waiting: the bus is busy");
        return;
    }
    if (why) {
        if (forced) {
            snprintf(o->status, sizeof(o->status), "cannot update: %s", why);
            o->forced = false;
        }
        return;
    }
    o->buf = o->io.alloc(BUF_LEN);
    if (!o->buf) {
        snprintf(o->status, sizeof(o->status), "update waiting: out of memory");
        return;
    }
    o->want = img->version;
    o->size = img->size;
    o->off = 0;
    put_tlv(o->buf, BWP_T_IMAGE_SIZE, 4);
    put_be32(o->buf + 3, o->size);
    put_tlv(o->buf + 7, BWP_T_SIGNATURE, WBO_MANIFEST_LEN);
    memcpy(o->buf + 10, img->manifest, WBO_MANIFEST_LEN);
    int rc = o->io.start(o->io.ctx, BWP_OTA_BEGIN, o->buf, 10 + WBO_MANIFEST_LEN, 30000);  /* it erases */
    if (rc != 0) {
        o->io.release(o->buf);
        o->buf = NULL;
        return;  /* busy: try on the next check */
    }
    o->state = WBO_BEGIN;
    char v[16], h[16];
    wbo_version_str(o->want, v);
    wbo_version_str(have, h);
    snprintf(o->status, sizeof(o->status), "updating %s to %s: starting", have ? h : "?", v);
}

void wbo_poll(wbo_t *o, const wb_info_t *br, const wbo_image_t *img, bool bus_free, uint32_t now)
{
    if (o->state == WBO_IDLE) {
        if (o->forced || elapsed(now, o->next_check)) {
            try_start(o, br, img, bus_free, now);
        }
        return;
    }
    if (o->state == WBO_WAIT_BACK) {
        if (br->present && wbo_pack_version(br->fw_version) == o->want) {
            char v[16];
            wbo_version_str(o->want, v);
            snprintf(o->status, sizeof(o->status), "updated to %s", v);
            o->failed = 0;
            stop(o);
        } else if (elapsed(now, o->deadline)) {
            fail(o, br->present ? "the bridge came back with its old firmware (rolled back?)"
                                : "the bridge did not come back", NULL);
        }
        return;
    }
    if (o->resend) {  /* the request slot was busy with the client's own STATUS */
        if (o->state == WBO_DATA) {
            send_chunk(o);
        } else if (o->state == WBO_END) {
            send_simple(o, BWP_OTA_END, 30000);
        } else if (o->state == WBO_REBOOT) {
            send_simple(o, BWP_REBOOT, 1000);
        }
        return;
    }
    wb_req_state_t st = o->io.take(o->io.ctx, o->reply);
    if (st == WB_REQ_PENDING || st == WB_REQ_IDLE) {
        return;
    }
    const char *step = o->state == WBO_BEGIN ? "OTA_BEGIN" : o->state == WBO_DATA ? "OTA_DATA"
                     : o->state == WBO_END ? "OTA_END" : "REBOOT";
    if (o->state == WBO_REBOOT) {  /* answered or not: the bridge restarts */
        stop(o);
        o->state = WBO_WAIT_BACK;
        o->deadline = now + WBO_COME_BACK_MS;
        char v[16];
        wbo_version_str(o->want, v);
        snprintf(o->status, sizeof(o->status), "updating to %s: waiting for the bridge to restart", v);
        return;
    }
    if (st == WB_REQ_TIMEOUT) {
        char why[48];
        snprintf(why, sizeof(why), "no answer to %s", step);
        fail(o, why, o->reply->message);
        return;
    }
    uint8_t r = o->reply->result;
    switch (o->state) {
    case WBO_BEGIN:
        if (r != BWP_OK) {
            fail(o, wb_result_str(r), o->reply->message);
            return;
        }
        o->state = WBO_DATA;
        send_chunk(o);
        return;
    case WBO_DATA:
        if (r != BWP_OK && r != BWP_E_OTA_BAD_OFFSET) {
            fail(o, wb_result_str(r), o->reply->message);
            return;
        }
        o->off = o->reply->offset;  /* the next offset the bridge wants (also after a loss) */
        if (o->off > o->size) {
            fail(o, "the bridge asked for an offset past the image", NULL);
        } else if (o->off == o->size) {
            o->state = WBO_END;
            send_simple(o, BWP_OTA_END, 30000);
        } else {
            send_chunk(o);
        }
        return;
    case WBO_END:
        if (r != BWP_OK) {
            fail(o, wb_result_str(r), o->reply->message);
            return;
        }
        o->state = WBO_REBOOT;
        send_simple(o, BWP_REBOOT, 1000);
        return;
    default:
        return;
    }
}
