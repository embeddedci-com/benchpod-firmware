/*
 * test_net_reload.c — host tests for the reload-after-reply latch (src/net_reload.c).
 *
 * The bug it guards: a settings command that came in over the cloud (cloud_proxy set/clear,
 * cloud_ca clear, a company-CA install, cloud_set, wifi_set) reloaded the cloud link inside its
 * handler, before the reply went out on that link, so the caller got "offline" although the
 * change was applied. The reload must wait for the reply, and still happen if the link is stuck.
 */
#include "net_reload.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); failures++; } } while (0)

/* Leave the latch empty between tests. */
static void drain(uint32_t now) {
    (void)net_reload_take(now + NET_RELOAD_MAX_WAIT_MS, false);
    CHECK(!net_reload_pending(), "still pending after a drain");
}

static void test_nothing_pending(void) {
    drain(0);
    CHECK(net_reload_take(5000, false) == 0, "took a reload nobody asked for");
    net_reload_commit(5000);   /* a commit without a request is a no-op */
    CHECK(!net_reload_pending() && net_reload_take(6000, false) == 0, "commit alone armed a reload");
}

/* Committed, link idle: due once the settle time has passed, and taken once. */
static void test_settle_then_once(void) {
    drain(0);
    net_reload_request(NET_RELOAD_CLOUD, 1000);
    net_reload_commit(1001);
    CHECK(net_reload_take(1001, false) == 0, "reloaded before the settle time");
    CHECK(net_reload_take(1001 + NET_RELOAD_SETTLE_MS - 1, false) == 0, "reloaded 1 ms early");
    CHECK(net_reload_take(1001 + NET_RELOAD_SETTLE_MS, false) == NET_RELOAD_CLOUD, "not reloaded when due");
    CHECK(net_reload_take(1100, false) == 0 && !net_reload_pending(), "reloaded twice");
}

/* Not committed (the handler that asked is still running, its reply not queued yet): wait,
   even with an idle link, until the commit or the bound. */
static void test_waits_for_commit(void) {
    drain(0);
    net_reload_request(NET_RELOAD_CLOUD, 2000);
    CHECK(net_reload_take(2000 + 500, false) == 0, "reloaded before the handler finished");
    net_reload_commit(2600);
    CHECK(net_reload_take(2600 + NET_RELOAD_SETTLE_MS, false) == NET_RELOAD_CLOUD, "not reloaded after commit");
}

/* A request nobody commits (asked from outside the worker) still reloads at the bound. */
static void test_uncommitted_bound(void) {
    drain(0);
    net_reload_request(NET_RELOAD_WIFI, 3000);
    CHECK(net_reload_take(3000 + NET_RELOAD_MAX_WAIT_MS - 1, false) == 0, "reloaded before the bound");
    CHECK(net_reload_take(3000 + NET_RELOAD_MAX_WAIT_MS, false) == NET_RELOAD_WIFI, "bound did not reload");
}

/* A stuck link (output never drains) reloads at the bound, counted from the request. */
static void test_stuck_link_bound(void) {
    drain(0);
    net_reload_request(NET_RELOAD_CLOUD, 4000);
    net_reload_commit(4002);
    for (uint32_t t = 4002; t < 4000 + NET_RELOAD_MAX_WAIT_MS; t += 7)
        CHECK(net_reload_take(t, true) == 0, "reloaded at %u with output queued", (unsigned)t);
    CHECK(net_reload_take(4000 + NET_RELOAD_MAX_WAIT_MS, true) == NET_RELOAD_CLOUD, "stuck link never reloaded");
}

/* Two settings in one go (wifi_set then cloud_set) reload both, once, on the first clock. */
static void test_bits_merge(void) {
    drain(0);
    net_reload_request(NET_RELOAD_WIFI, 5000);
    net_reload_commit(5001);
    net_reload_request(NET_RELOAD_CLOUD, 5400);   /* the next command, not committed yet */
    CHECK(net_reload_take(5500, false) == 0, "reloaded with the newer request uncommitted");
    net_reload_commit(5501);
    CHECK(net_reload_take(5501 + NET_RELOAD_SETTLE_MS, false) == (NET_RELOAD_WIFI | NET_RELOAD_CLOUD),
          "both bits not taken together");
    /* the bound ran from the first request (5000), not the second */
    net_reload_request(NET_RELOAD_CLOUD, 6000);
    net_reload_request(NET_RELOAD_CLOUD, 6900);
    CHECK(net_reload_take(6000 + NET_RELOAD_MAX_WAIT_MS, true) == NET_RELOAD_CLOUD, "bound restarted by a later request");
}

/* The millisecond clock wraps every 49.7 days; the latch must not care. */
static void test_clock_wrap(void) {
    drain(0);
    uint32_t t0 = 0xFFFFFFF0u;
    drain(t0 - NET_RELOAD_MAX_WAIT_MS);
    net_reload_request(NET_RELOAD_CLOUD, t0);
    net_reload_commit(t0);
    CHECK(net_reload_take(t0 + 5, false) == 0, "reloaded early across the wrap");
    CHECK(net_reload_take(t0 + NET_RELOAD_SETTLE_MS, false) == NET_RELOAD_CLOUD, "not reloaded across the wrap");
    net_reload_request(NET_RELOAD_CLOUD, t0);
    CHECK(net_reload_take(t0 + NET_RELOAD_MAX_WAIT_MS, true) == NET_RELOAD_CLOUD, "bound broken by the wrap");
}

/* ---- the ordering, end to end, on a model of the two tasks -------------------------------------
   A cloud command.request for cloud_proxy set: the worker runs the handler (persist, request the
   reload), queues the reply and commits; the net task frames the reply, TCP holds it until the
   server ACKs it some milliseconds later. The reload must come after that ACK, never before the
   reply is queued. tx_busy is what cloud_client_tx_busy reports: a command in flight, or its
   response written but not acknowledged. */
enum { EV_PERSIST = 1, EV_REPLY_QUEUED, EV_REPLY_SENT, EV_REPLY_ACKED, EV_RELOAD };
static int  s_log[16], s_nlog;
static void ev(int e) { if (s_nlog < 16) s_log[s_nlog++] = e; }

static void run_cloud_command(uint32_t start, uint32_t handler_ms, uint32_t rtt_ms, bool ack_ever) {
    s_nlog = 0;
    bool cloud_pending = true;          /* hw_worker_cloud_pending() */
    bool reply_ready = false, in_tcp = false;
    uint32_t sent_at = 0, done_at = start + handler_ms;
    bool handler_done = false;
    for (uint32_t now = start; now < start + 3000; now++) {
        /* worker: the handler persists and asks for the reload first thing, finishes later */
        if (now == start) { ev(EV_PERSIST); net_reload_request(NET_RELOAD_CLOUD, now); }
        if (!handler_done && now >= done_at) {
            handler_done = true; reply_ready = true; ev(EV_REPLY_QUEUED);
            net_reload_commit(now);
        }
        /* net task, one poll: frame a ready reply, see the ACK, then maybe reload */
        if (reply_ready) { reply_ready = false; cloud_pending = false; in_tcp = true; sent_at = now; ev(EV_REPLY_SENT); }
        if (in_tcp && ack_ever && now - sent_at >= rtt_ms) { in_tcp = false; ev(EV_REPLY_ACKED); }
        if (net_reload_take(now, cloud_pending || in_tcp) & NET_RELOAD_CLOUD) { ev(EV_RELOAD); return; }
    }
}

static int pos(int e) { for (int i = 0; i < s_nlog; i++) if (s_log[i] == e) return i; return -1; }

static void test_reply_leaves_before_reload(void) {
    drain(0);
    run_cloud_command(10000, 3, 40, true);     /* a W25Q write, a 40 ms round trip */
    CHECK(pos(EV_RELOAD) > pos(EV_REPLY_ACKED) && pos(EV_REPLY_ACKED) > pos(EV_REPLY_SENT) &&
          pos(EV_REPLY_SENT) > pos(EV_PERSIST), "order: reload before the reply left (%d events)", s_nlog);

    drain(20000);
    run_cloud_command(20000, 0, 0, true);      /* instant handler and ACK: still the settle time */
    CHECK(pos(EV_RELOAD) > pos(EV_REPLY_SENT), "instant: reload before the reply was sent");

    drain(30000);
    run_cloud_command(30000, 200, 5000, false);  /* the ACK never comes: the bound reloads */
    CHECK(pos(EV_REPLY_SENT) >= 0 && pos(EV_RELOAD) > pos(EV_REPLY_SENT) && pos(EV_REPLY_ACKED) < 0,
          "stuck: no reload after the reply went out");

    drain(40000);
    run_cloud_command(40000, 1500, 10, true);   /* a handler slower than the bound */
    CHECK(pos(EV_RELOAD) > 0, "slow handler: never reloaded");
}

int main(void) {
    test_nothing_pending();
    test_settle_then_once();
    test_waits_for_commit();
    test_uncommitted_bound();
    test_stuck_link_bound();
    test_bits_merge();
    test_clock_wrap();
    test_reply_leaves_before_reload();
    if (failures) { printf("test_net_reload: %d FAILED\n", failures); return 1; }
    printf("test_net_reload: all passed\n");
    return 0;
}
