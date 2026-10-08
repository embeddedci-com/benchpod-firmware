/*
 * net_reload.c — see net_reload.h.
 *
 * Lock-free between the two tasks: the worker writes the tick and the bits, then bumps the
 * sequence; the net task snapshots the sequence and only reads what the worker wrote before it.
 * Bits requested while a reload is being taken are OR-ed in and taken on the next poll. The
 * net task outranks the worker, so the worst a preemption inside the worker's s_bits |= can do
 * is leave a bit set that was just taken: one extra reload, never a lost one.
 */
#include "net_reload.h"

static volatile uint32_t s_bits;          /* requested, not yet taken                     */
static volatile uint32_t s_req_seq;       /* bumped by every request                       */
static volatile uint32_t s_req_ms;        /* tick of the first request since the last take */
static volatile uint32_t s_commit_seq;    /* the request sequence the worker committed     */
static volatile uint32_t s_commit_ms;
static volatile uint32_t s_taken_seq;     /* the request sequence the net task last took   */

void net_reload_request(uint32_t what, uint32_t now_ms) {
    if (!what) return;
    if (s_req_seq == s_taken_seq) s_req_ms = now_ms;   /* the first one starts the clock */
    s_bits |= what;
    s_req_seq = s_req_seq + 1u;
}

void net_reload_commit(uint32_t now_ms) {
    uint32_t seq = s_req_seq;
    if (seq == s_taken_seq || seq == s_commit_seq) return;
    s_commit_ms  = now_ms;
    s_commit_seq = seq;
}

uint32_t net_reload_take(uint32_t now_ms, bool tx_busy) {
    uint32_t seq = s_req_seq;
    if (seq == s_taken_seq) return 0;
    bool due = (uint32_t)(now_ms - s_req_ms) >= NET_RELOAD_MAX_WAIT_MS;
    if (!due && s_commit_seq == seq)
        due = (uint32_t)(now_ms - s_commit_ms) >= NET_RELOAD_SETTLE_MS && !tx_busy;
    if (!due) return 0;
    uint32_t bits = s_bits;
    s_bits &= ~bits;
    s_taken_seq = seq;
    return bits;
}

bool net_reload_pending(void) {
    return s_req_seq != s_taken_seq;
}
