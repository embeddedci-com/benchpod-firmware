/*
 * lease_local.c: see lease_local.h.
 */
#include "lease_local.h"

#include <string.h>

/* Single core: a compiler barrier orders the writer's stores against a reader that preempts it. */
#define LL_FENCE() __atomic_signal_fence(__ATOMIC_SEQ_CST)

static bool     s_held;
static uint32_t s_deadline_ms;
static char     s_id[LEASE_LOCAL_ID_MAX];
static char     s_holder[LEASE_LOCAL_HOLDER_MAX];
static char     s_purpose[LEASE_LOCAL_PURPOSE_MAX];
static uint32_t s_owners;   /* bit per connection id */
static uint32_t s_lost;     /* owners of a broken lease that have not been told yet */
/* The last broken lease: its id stays refused until s_broken_until_ms. */
static char     s_broken_id[LEASE_LOCAL_ID_MAX];
static char     s_lost_by[LEASE_LOCAL_HOLDER_MAX];
static uint32_t s_broken_until_ms;

/* A seqlock: odd while the writer (the hw worker) changes the fields above; a reader (the net
   task, lease_local_snapshot) retries when it changed under it. It relies on a single core: the
   reader can only run while the writer is preempted, so LL_FENCE (a compiler fence, no memory
   barrier) is enough to order the stores. A second core would need real barriers. */
static volatile uint32_t s_seq;
static volatile uint32_t s_gen;

/* lease.break from the net task, applied by the writer. */
static volatile bool s_break_pending;
static char          s_break_by[LEASE_LOCAL_HOLDER_MAX];

static void wr_begin(void) { s_seq = s_seq + 1u; LL_FENCE(); }
static void wr_end(void)   { LL_FENCE(); s_seq = s_seq + 1u; }

static uint32_t conn_bit(int conn) {
    return (conn >= 0 && conn < LEASE_LOCAL_MAX_CONN) ? (1u << (unsigned)conn) : 0u;
}

/* A label is for people: printable ASCII only, no quote or backslash, so it is safe unescaped in
   any reply. */
static void copy_label(char *dst, size_t cap, const char *src) {
    size_t n = 0;
    for (const char *p = src ? src : ""; *p && n < cap - 1; p++)
        dst[n++] = (*p >= 0x20 && *p < 0x7f && *p != '"' && *p != '\\') ? *p : '?';
    dst[n] = '\0';
}

/* The id is a token compared byte for byte: refuse what a label would rewrite. */
static bool id_ok(const char *id) {
    if (!id || !id[0]) return false;
    size_t n = 0;
    for (const char *p = id; *p; p++, n++) {
        if (n >= LEASE_LOCAL_ID_MAX - 1) return false;
        if (*p <= 0x20 || *p >= 0x7f || *p == '"' || *p == '\\') return false;
    }
    return true;
}

uint32_t lease_local_clamp_ttl(uint32_t ttl_s) {
    if (ttl_s == 0) return LEASE_LOCAL_TTL_DEFAULT_S;
    if (ttl_s < LEASE_LOCAL_TTL_MIN_S) return LEASE_LOCAL_TTL_MIN_S;
    if (ttl_s > LEASE_LOCAL_TTL_MAX_S) return LEASE_LOCAL_TTL_MAX_S;
    return ttl_s;
}

static bool live(uint32_t now_ms) {
    return s_held && (int32_t)(s_deadline_ms - now_ms) > 0;   /* wrap-safe */
}

static void clear_locked(void) {
    s_held = false;
    s_id[0] = s_holder[0] = s_purpose[0] = '\0';
    s_owners = 0;
    s_gen = s_gen + 1u;
}

static void fill_view(int conn, uint32_t now_ms, lease_local_view_t *v) {
    if (!v) return;
    memset(v, 0, sizeof(*v));
    if (!live(now_ms)) return;
    v->held = true;
    v->mine = (s_owners & conn_bit(conn)) != 0;
    v->left_s = ((uint32_t)(s_deadline_ms - now_ms) + 999u) / 1000u;
    memcpy(v->id, s_id, sizeof(v->id));
    memcpy(v->holder, s_holder, sizeof(v->holder));
    memcpy(v->purpose, s_purpose, sizeof(v->purpose));
}

static bool broken(const char *id, uint32_t now_ms) {
    return id && id[0] && s_broken_id[0] && (int32_t)(s_broken_until_ms - now_ms) > 0 &&
           strcmp(s_broken_id, id) == 0;
}

/* The writer's housekeeping before each call: a posted break, then the deadlines. */
static void settle(uint32_t now_ms) {
    if (s_break_pending) {
        char by[LEASE_LOCAL_HOLDER_MAX];
        memcpy(by, s_break_by, sizeof(by));
        LL_FENCE();
        s_break_pending = false;
        if (live(now_ms)) {
            wr_begin();
            s_lost |= s_owners;
            copy_label(s_lost_by, sizeof(s_lost_by), by[0] ? by : "the server");
            memcpy(s_broken_id, s_id, sizeof(s_broken_id));
            s_broken_until_ms = now_ms + LEASE_LOCAL_BROKEN_S * 1000u;
            clear_locked();
            wr_end();
        }
    }
    if (s_held && !live(now_ms)) { wr_begin(); clear_locked(); wr_end(); }
    /* Forget a broken id after its time, so a stale deadline cannot read as live 2^31 ms on.
       Inside the write section like every other store to the fields the seqlock covers. */
    if (s_broken_id[0] && (int32_t)(s_broken_until_ms - now_ms) <= 0) {
        wr_begin();
        s_broken_id[0] = '\0';
        wr_end();
    }
}

const char *lease_local_broken_by(void) { return s_lost_by; }

lease_local_rc_t lease_local_acquire(int conn, const char *id, const char *holder,
                                     const char *purpose, uint32_t ttl_s, uint32_t now_ms,
                                     lease_local_view_t *view) {
    settle(now_ms);
    if (!id_ok(id)) { fill_view(conn, now_ms, view); return LEASE_LOCAL_BAD_ID; }
    if (broken(id, now_ms)) { fill_view(conn, now_ms, view); return LEASE_LOCAL_LOST; }
    if (s_held && strcmp(s_id, id) != 0) { fill_view(conn, now_ms, view); return LEASE_LOCAL_BUSY; }
    wr_begin();
    if (!s_held) {
        memcpy(s_id, id, strlen(id) + 1);
        copy_label(s_holder, sizeof(s_holder), holder && holder[0] ? holder : "unnamed");
        copy_label(s_purpose, sizeof(s_purpose), purpose);
        s_owners = 0;
        s_held = true;
        s_gen = s_gen + 1u;
    }
    s_deadline_ms = now_ms + lease_local_clamp_ttl(ttl_s) * 1000u;
    s_owners |= conn_bit(conn);
    s_lost &= ~conn_bit(conn);
    wr_end();
    fill_view(conn, now_ms, view);
    return LEASE_LOCAL_OK;
}

lease_local_rc_t lease_local_renew(int conn, const char *id, uint32_t ttl_s, uint32_t now_ms,
                                   lease_local_view_t *view) {
    settle(now_ms);
    if (!id_ok(id)) { fill_view(conn, now_ms, view); return LEASE_LOCAL_BAD_ID; }
    if (broken(id, now_ms)) { fill_view(conn, now_ms, view); return LEASE_LOCAL_LOST; }
    if (!s_held) { fill_view(conn, now_ms, view); return LEASE_LOCAL_NOT_HELD; }
    if (strcmp(s_id, id) != 0) { fill_view(conn, now_ms, view); return LEASE_LOCAL_BUSY; }
    wr_begin();
    s_deadline_ms = now_ms + lease_local_clamp_ttl(ttl_s) * 1000u;
    wr_end();
    fill_view(conn, now_ms, view);
    return LEASE_LOCAL_OK;
}

lease_local_rc_t lease_local_release(int conn, const char *id, uint32_t now_ms,
                                     lease_local_view_t *view) {
    settle(now_ms);
    if (!s_held) { fill_view(conn, now_ms, view); return LEASE_LOCAL_OK; }
    if (!id || strcmp(s_id, id) != 0) { fill_view(conn, now_ms, view); return LEASE_LOCAL_BUSY; }
    wr_begin();
    clear_locked();
    wr_end();
    fill_view(conn, now_ms, view);
    return LEASE_LOCAL_OK;
}

void lease_local_conn_closed(int conn) {
    uint32_t b = conn_bit(conn);
    if (!((s_owners | s_lost) & b)) return;
    wr_begin();
    s_owners &= ~b;
    s_lost &= ~b;
    wr_end();
}

void lease_local_expire(uint32_t now_ms) { settle(now_ms); }

bool lease_local_get(int conn, uint32_t now_ms, lease_local_view_t *view) {
    settle(now_ms);
    fill_view(conn, now_ms, view);
    return view->held;
}

lease_local_gate_t lease_local_check(int conn, bool lease_verb, bool light, const char *cmd_id,
                                     uint32_t now_ms, lease_local_view_t *view, char *by,
                                     size_t by_cap) {
    settle(now_ms);
    uint32_t b = conn_bit(conn);
    const bool lost_once = (s_lost & b) != 0;
    if (lost_once || broken(cmd_id, now_ms)) {
        s_lost &= ~b;
        if (by && by_cap) { strncpy(by, s_lost_by, by_cap - 1); by[by_cap - 1] = '\0'; }
        fill_view(conn, now_ms, view);
        return LEASE_LOCAL_REFUSE_LOST;
    }
    /* The held id on the command: this command, and its connection, are the owner's. */
    if (cmd_id && cmd_id[0] && live(now_ms) && !(s_owners & b) && strcmp(s_id, cmd_id) == 0) {
        wr_begin();
        s_owners |= b;
        wr_end();
    }
    fill_view(conn, now_ms, view);
    if (!live(now_ms) || lease_verb || light || (s_owners & b)) return LEASE_LOCAL_PASS;
    return LEASE_LOCAL_REFUSE_BUSY;
}

void lease_local_break(const char *by) {
    copy_label(s_break_by, sizeof(s_break_by), by);
    LL_FENCE();
    s_break_pending = true;
}

uint32_t lease_local_generation(void) { return s_gen; }

bool lease_local_snapshot(uint32_t now_ms, lease_local_view_t *view, uint32_t *gen) {
    for (int tries = 0; tries < 4; tries++) {
        uint32_t s0 = s_seq;
        if (s0 & 1u) continue;
        LL_FENCE();
        uint32_t g = s_gen;
        fill_view(-1, now_ms, view);
        LL_FENCE();
        if (s_seq == s0) { if (gen) *gen = g; return true; }
    }
    return false;
}

void lease_local_reset(void) {
    wr_begin();
    clear_locked();
    s_lost = 0;
    s_lost_by[0] = '\0';
    s_broken_id[0] = '\0';
    s_break_pending = false;
    wr_end();
}
