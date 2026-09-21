/*
 * command_handler_power.c — the `power_profile` command and its chunked reply.
 *
 *   {"cmd":"power_profile","action":"start","efuse":1,"rate_hz":1000,"max_duration_ms":60000,"keep_samples":0}
 *   {"cmd":"power_profile","action":"stop"}     -> chunked result (same framing as la_capture)
 *   {"cmd":"power_profile","action":"status"}
 *   {"cmd":"power_profile","efuse":1,"duration_ms":N,...}   one-shot: start, wait, stop reply
 *
 * The sampler lives in power_profile.c; this file parses, validates and delivers.  The reply is
 * paced against the connection's send ring like bulk_pump.  Runs on the hw worker task.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "power_profile.h"
#include "at_driver.h"
#include "bp_json.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Samples per frame.  25 B/sample worst case ("600000000," "-5460000," "21800,") + the frame
   header and the final frame's stats (~330 B) stay under the 1600 B frame buffer. */
#define PP_FRAME_SAMPLES   48u
#define PP_FRAME_OVERHEAD  400u
#define PP_SAMPLE_BYTES    25u

static struct {
    bool     active;
    int      conn_id;
    uint32_t next;     /* next kept sample to send */
    bool     first;
} s_reply;

static struct {
    bool pending;      /* one-shot: reply to conn_id when the sampler stops by itself */
    int  conn_id;
} s_oneshot;

/* Optional unsigned field; false (and the error sent) when present but not a number in range. */
static bool get_u32(int conn_id, const char *json, const char *key, uint32_t dflt, uint32_t max,
                    uint32_t *out) {
    char s[16] = {0};
    *out = dflt;
    if (!bp_json_get(json, key, s, sizeof(s)) || strcmp(s, "null") == 0) return true;
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s || *end != '\0' || v < 0 || (unsigned long)v > max) {
        char msg[64];
        snprintf(msg, sizeof(msg), "%s must be a whole number 0..%lu", key, (unsigned long)max);
        send_error(conn_id, msg);
        return false;
    }
    *out = (uint32_t)v;
    return true;
}

static void reply_begin(int conn_id) {
    s_reply.active  = true;
    s_reply.conn_id = conn_id;
    s_reply.next    = 0;
    s_reply.first   = true;
    power_profile_service();   /* the console / cloud capture take it all at once */
}

/* Deliver the result frames as far as the send ring allows; resume on the next worker pass. */
static void reply_pump(void) {
    int conn = s_reply.conn_id;
    uint32_t total = power_profile_kept();
    for (;;) {
        size_t avail = at_send_avail(conn);
        if (avail < PP_FRAME_OVERHEAD + PP_SAMPLE_BYTES) return;
        uint32_t k = total - s_reply.next;
        if (k > PP_FRAME_SAMPLES) k = PP_FRAME_SAMPLES;
        uint32_t room = (uint32_t)((avail - PP_FRAME_OVERHEAD) / PP_SAMPLE_BYTES);
        if (k > room) k = room;
        bool last = (s_reply.next + k >= total);

        char buf[1600];
        bp_emit_t e;
        bp_emit_init(&e, buf, sizeof(buf));
        bp_emit(&e, "{\"status\":\"%s\",\"t_us\":[", s_reply.first ? "ok" : "chunk");
        uint32_t t; int32_t ua; uint16_t mv;
        for (uint32_t i = 0; i < k; i++) {
            power_profile_sample(s_reply.next + i, &t, &ua, &mv);
            bp_emit(&e, "%s%lu", i ? "," : "", (unsigned long)t);
        }
        bp_emit_raw(&e, "],\"current_ua\":[");
        for (uint32_t i = 0; i < k; i++) {
            power_profile_sample(s_reply.next + i, &t, &ua, &mv);
            bp_emit(&e, "%s%ld", i ? "," : "", (long)ua);
        }
        bp_emit_raw(&e, "],\"bus_mv\":[");
        for (uint32_t i = 0; i < k; i++) {
            power_profile_sample(s_reply.next + i, &t, &ua, &mv);
            bp_emit(&e, "%s%u", i ? "," : "", (unsigned)mv);
        }
        bp_emit_raw(&e, "]");
        if (last) {
            bp_emit_raw(&e, ",");
            power_profile_emit_stats(&e);
        }
        bp_emit(&e, ",\"more\":%s}\n", last ? "false" : "true");
        if (!bp_emit_ok(&e)) {   /* cannot happen with the sizes above; never send a torn frame */
            s_reply.active = false;
            send_error(conn, "power profile reply frame overflow");
            return;
        }
        if (at_send_data(conn, (const uint8_t *)buf, bp_emit_len(&e)) != 0) {
            /* The client left: keep the result so a stop from a new connection still gets it. */
            s_reply.active = false;
            at_close_connection(conn);
            return;
        }
        s_reply.next += k;
        s_reply.first = false;
        if (last) {
            s_reply.active = false;
            power_profile_discard();   /* read: results are kept until read or the next start */
            return;
        }
    }
}

void power_profile_service(void) {
    if (s_oneshot.pending && !power_profile_running() && power_profile_has_result() && !s_reply.active) {
        s_oneshot.pending = false;
        reply_begin(s_oneshot.conn_id);
        return;
    }
    if (s_reply.active) reply_pump();
}

void power_profile_conn_closed(int conn_id) {
    if (s_reply.active && s_reply.conn_id == conn_id) s_reply.active = false;   /* result kept */
    if (s_oneshot.pending && s_oneshot.conn_id == conn_id) {
        s_oneshot.pending = false;
        power_profile_stop();
    }
}

void handle_power_profile(int conn_id, const char *json) {
    char action[12] = {0};
    bp_json_get(json, "action", action, sizeof(action));

    if (strcmp(action, "status") == 0) {
        pp_status_t st;
        power_profile_status(&st);
        char payload[128];
        if (st.any)
            snprintf(payload, sizeof(payload), "{\"running\":%s,\"efuse\":%d,\"elapsed_ms\":%lu,\"n\":%lu}",
                     st.running ? "true" : "false", st.efuse, (unsigned long)st.elapsed_ms, (unsigned long)st.n);
        else
            snprintf(payload, sizeof(payload), "{\"running\":false,\"efuse\":null,\"elapsed_ms\":0,\"n\":0}");
        send_ok_str(conn_id, payload);
        return;
    }

    if (strcmp(action, "stop") == 0) {
        if (s_reply.active) { send_error(conn_id, "the power profile result is already being sent"); return; }
        if (!power_profile_running() && !power_profile_has_result()) {
            send_error(conn_id, "no power profile to stop");
            return;
        }
        power_profile_stop();
        if (conn_id == CH_CLOUD_CONN && power_profile_kept() > 0) {
            /* The cloud command channel carries exactly one reply line. */
            char msg[160];
            snprintf(msg, sizeof(msg), "the power profile kept %lu samples; read them over a stream "
                                       "connection (LAN or tunnel), or start with keep_samples 0",
                     (unsigned long)power_profile_kept());
            send_error(conn_id, msg);
            return;
        }
        if (s_oneshot.pending) {
            if (s_oneshot.conn_id != conn_id)
                send_error(s_oneshot.conn_id, "power profile stopped by another client");
            s_oneshot.pending = false;
        }
        reply_begin(conn_id);
        return;
    }

    bool oneshot = (action[0] == '\0');
    if (!oneshot && strcmp(action, "start") != 0) {
        send_error(conn_id, "action must be start, stop or status");
        return;
    }
    uint32_t efuse, rate, dur, keep, oneshot_ms = 0;
    if (!get_u32(conn_id, json, "efuse", 1u, 2u, &efuse)) return;
    if (!get_u32(conn_id, json, "rate_hz", PP_RATE_DEFAULT_HZ, 1000000u, &rate)) return;
    if (!get_u32(conn_id, json, "max_duration_ms", PP_DURATION_DEFAULT_MS, 0x7FFFFFFFu, &dur)) return;
    if (!get_u32(conn_id, json, "keep_samples", 0u, 0x7FFFFFFFu, &keep)) return;
    if (oneshot) {
        char probe[16];
        if (!bp_json_get(json, "duration_ms", probe, sizeof(probe))) {
            send_error(conn_id, "power_profile needs an action (start, stop, status) or duration_ms for a one-shot");
            return;
        }
        if (!get_u32(conn_id, json, "duration_ms", 0u, 0x7FFFFFFFu, &oneshot_ms)) return;
        if (oneshot_ms < 1u || oneshot_ms > PP_DURATION_MAX_MS) {
            send_error(conn_id, "duration_ms must be 1..600000");
            return;
        }
        if (conn_id == CH_CLOUD_CONN) {
            send_error(conn_id, "a one-shot power_profile waits for its result, which needs a stream "
                                "connection; over the cloud command channel use action start, then stop");
            return;
        }
        dur = oneshot_ms;
    }
    if (s_reply.active) {
        send_error(conn_id, "the previous power profile result is still being sent; try again when it finishes");
        return;
    }

    char err[160];
    if (power_profile_start((int)efuse, rate, dur, keep, err, sizeof(err)) != 0) {
        send_error(conn_id, err);
        return;
    }
    if (oneshot) {
        s_oneshot.pending = true;
        s_oneshot.conn_id = conn_id;
        return;   /* the stop-style reply follows from power_profile_service() */
    }
    pp_status_t st;
    power_profile_status(&st);
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"started\":true,\"efuse\":%d,\"rate_hz\":%lu}",
             st.efuse, (unsigned long)st.rate_hz);
    send_ok_str(conn_id, payload);
}
