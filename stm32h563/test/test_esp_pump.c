/*
 * test_esp_pump.c — host unit tests for the esp-hosted transaction pump
 * (src/esp_hosted_pump.c).  A fake slave drives HANDSHAKE / DATA_READY / the
 * outbound queue and records every transaction, so the batching policy can be
 * pinned down without any HAL, SPI or ESP32:
 *
 *   - gating: nothing is clocked unless HANDSHAKE is active AND there is work
 *     (queued TX or DATA_READY);
 *   - THE FIX: a backlog drains in ONE pump call instead of one frame per call;
 *   - the batch is bounded — a slave that never deasserts cannot starve the net
 *     task of lwIP/Ethernet servicing;
 *   - a batch stops as soon as the work drains, and does not clock idle frames;
 *   - a transfer error stops the batch immediately (the frame stays queued);
 *   - the settle hook is only consulted mid-batch, never on an idle link, and a
 *     slave that stays deasserted through the settle window ends the batch;
 *   - a malformed backend is a no-op rather than a crash.
 */
#include "esp_hosted_pump.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

/* ---- fake slave ---------------------------------------------------------- */

typedef struct {
    /* knobs */
    bool hs;                /* HANDSHAKE level                                */
    bool hs_drops_after_xact;  /* slave deasserts HS after each transaction    */
    bool settle_recovers;   /* ...and re-asserts within the settle window      */
    int  rx_frames;         /* frames the slave still has queued for us        */
    int  tx_frames;         /* frames we still have queued outbound            */
    int  fail_on_xact;      /* 1-based index of a transaction that errors, 0=none */

    /* observations */
    int  xacts;             /* transactions actually clocked                   */
    int  settles;           /* hs_settle() invocations                         */
    int  hs_reads, dr_reads, tx_reads;
} fake_t;

static fake_t F;
static esp_hosted_pump_stats_t S;

static void fake_reset(void)
{
    memset(&F, 0, sizeof(F));
    memset(&S, 0, sizeof(S));
    F.hs = true;
}

static bool fk_hs(void *ctx)  { (void)ctx; F.hs_reads++; return F.hs; }
static bool fk_dr(void *ctx)  { (void)ctx; F.dr_reads++; return F.rx_frames > 0; }
static bool fk_tx(void *ctx)  { (void)ctx; F.tx_reads++; return F.tx_frames > 0; }

static int fk_xact(void *ctx)
{
    (void)ctx;
    F.xacts++;
    if (F.fail_on_xact && F.xacts == F.fail_on_xact) {
        /* Transfer error: the caller must leave the queued TX frame alone, so
           the fake does not consume anything here either. */
        return -1;
    }
    /* A full-duplex transaction moves one frame each way (when present). */
    if (F.tx_frames > 0) F.tx_frames--;
    if (F.rx_frames > 0) F.rx_frames--;
    if (F.hs_drops_after_xact) F.hs = false;
    return 0;
}

static bool fk_settle(void *ctx)
{
    (void)ctx;
    F.settles++;
    if (F.settle_recovers) F.hs = true;
    return F.hs;
}

static esp_hosted_pump_backend_t mk_backend(uint8_t max_batch, bool with_settle)
{
    esp_hosted_pump_backend_t b = {
        .hs_active  = fk_hs,
        .dr_active  = fk_dr,
        .tx_pending = fk_tx,
        .xact       = fk_xact,
        .hs_settle  = with_settle ? fk_settle : NULL,
        .ctx        = NULL,
        .max_batch  = max_batch,
        .stats      = &S,
    };
    return b;
}

/* ---- gating: when nothing may be clocked --------------------------------- */

static void test_gating(void)
{
    esp_hosted_pump_backend_t b = mk_backend(4, true);

    /* Idle link: HS asserted but no work either way. */
    fake_reset();
    CHECK(esp_hosted_pump(&b) == 0);
    CHECK(F.xacts == 0);
    /* And an idle link must never reach the settle path — that is what keeps
       net_poll() cheap on every tick where the ESP has nothing to say. */
    CHECK(F.settles == 0);

    /* Slave not ready: work pending both ways, but HANDSHAKE is low. */
    fake_reset();
    F.hs = false;
    F.rx_frames = 3;
    F.tx_frames = 3;
    CHECK(esp_hosted_pump(&b) == 0);
    CHECK(F.xacts == 0);
    /* No settle on the FIRST transaction either: a slave that is simply not
       ready is left to the next net_poll() pass rather than spun on. */
    CHECK(F.settles == 0);

    /* HS active + only DATA_READY -> a transaction runs. */
    fake_reset();
    F.rx_frames = 1;
    CHECK(esp_hosted_pump(&b) == 1);
    CHECK(F.xacts == 1);

    /* HS active + only queued TX -> a transaction runs. */
    fake_reset();
    F.tx_frames = 1;
    CHECK(esp_hosted_pump(&b) == 1);
    CHECK(F.xacts == 1);
}

/* ---- the actual fix: a backlog drains in one pass ------------------------ */

static void test_batches_a_backlog(void)
{
    /* 4 frames waiting on the slave.  Pre-fix this took 4 net_poll() passes =
       ~4 ms; now one pass clears it. */
    esp_hosted_pump_backend_t b = mk_backend(4, true);
    fake_reset();
    F.rx_frames = 4;

    CHECK(esp_hosted_pump(&b) == 4);
    CHECK(F.xacts == 4);
    CHECK(F.rx_frames == 0);

    /* Outbound backlog batches the same way. */
    fake_reset();
    F.tx_frames = 4;
    CHECK(esp_hosted_pump(&b) == 4);
    CHECK(F.tx_frames == 0);

    /* Mixed traffic: full duplex moves both directions per transaction, so the
       batch length is set by the longer queue. */
    fake_reset();
    F.rx_frames = 3;
    F.tx_frames = 2;
    CHECK(esp_hosted_pump(&b) == 3);
    CHECK(F.rx_frames == 0 && F.tx_frames == 0);
}

/* ---- the bound: a chatty slave cannot starve the net task ---------------- */

static void test_batch_is_bounded(void)
{
    /* A slave with an unbounded backlog (and HS wedged high) must still yield
       after max_batch so net_poll() gets back to lwIP / Ethernet. */
    esp_hosted_pump_backend_t b = mk_backend(4, true);
    fake_reset();
    F.rx_frames = 1000;

    CHECK(esp_hosted_pump(&b) == 4);
    CHECK(F.xacts == 4);

    /* The rest is picked up by subsequent passes, not dropped. */
    CHECK(esp_hosted_pump(&b) == 4);
    CHECK(F.xacts == 8);

    /* max_batch == 0 means "use the default", not "run nothing" / "run forever". */
    fake_reset();
    F.rx_frames = 1000;
    esp_hosted_pump_backend_t d = mk_backend(0, true);
    CHECK(esp_hosted_pump(&d) == (int)ESP_HOSTED_PUMP_MAX_BATCH);

    /* A cap of 1 reproduces exactly the old one-transaction-per-pass behaviour,
       which is what a bisect would want if batching ever needs backing out. */
    fake_reset();
    F.rx_frames = 1000;
    esp_hosted_pump_backend_t one = mk_backend(1, true);
    CHECK(esp_hosted_pump(&one) == 1);
    CHECK(F.xacts == 1);

    /* An over-large max_batch is clamped to the compile-time maximum: the stats
       histogram is sized by that constant, so an unclamped cap would index off
       the end of batch_len[]. */
    fake_reset();
    F.rx_frames = 1000;
    esp_hosted_pump_backend_t big = mk_backend(200, true);
    CHECK(esp_hosted_pump(&big) == (int)ESP_HOSTED_PUMP_MAX_BATCH);
    CHECK(S.batch_len[ESP_HOSTED_PUMP_MAX_BATCH] == 1);
}

/* ---- instrumentation: the counters the bench measurement reads ----------- */

static void test_stats(void)
{
    esp_hosted_pump_backend_t b = mk_backend(4, true);

    /* An idle pass is counted as a call with a batch length of zero — that is
       how the histogram distinguishes "link quiet" from "batching not working". */
    fake_reset();
    CHECK(esp_hosted_pump(&b) == 0);
    CHECK(S.calls == 1 && S.xacts == 0 && S.batch_len[0] == 1);

    /* A batched pass lands in the histogram at its length.  On hardware, a
       non-zero bucket at index >= 2 is the proof that the fix is live: the old
       transport could only ever produce buckets 0 and 1. */
    fake_reset();
    F.rx_frames = 3;
    CHECK(esp_hosted_pump(&b) == 3);
    CHECK(S.calls == 1 && S.xacts == 3);
    CHECK(S.batch_len[3] == 1);
    CHECK(S.batch_len[0] == 0 && S.batch_len[1] == 0 && S.batch_len[2] == 0);

    /* Counters accumulate across passes rather than resetting per call. */
    F.rx_frames = 2;
    CHECK(esp_hosted_pump(&b) == 2);
    CHECK(S.calls == 2 && S.xacts == 5);
    CHECK(S.batch_len[3] == 1 && S.batch_len[2] == 1);

    /* Settle hits/misses are separated so the bench can tell a well-sized
       settle window from one that is too short. */
    fake_reset();
    F.rx_frames = 4;
    F.hs_drops_after_xact = true;
    F.settle_recovers     = true;
    CHECK(esp_hosted_pump(&b) == 4);
    CHECK(S.settle_hit == 3 && S.settle_miss == 0);

    fake_reset();
    F.rx_frames = 4;
    F.hs_drops_after_xact = true;
    F.settle_recovers     = false;
    CHECK(esp_hosted_pump(&b) == 1);
    CHECK(S.settle_hit == 0 && S.settle_miss == 1);
    CHECK(S.batch_len[1] == 1);

    /* Transfer errors are counted and do not inflate the transaction count. */
    fake_reset();
    F.rx_frames = 4;
    F.fail_on_xact = 2;
    CHECK(esp_hosted_pump(&b) == 1);
    CHECK(S.xact_err == 1 && S.xacts == 1);

    /* A stats-less backend must still work (the pointer is optional). */
    fake_reset();
    F.rx_frames = 2;
    esp_hosted_pump_backend_t nostats = mk_backend(4, true);
    nostats.stats = NULL;
    CHECK(esp_hosted_pump(&nostats) == 2);
}

/* ---- a batch stops as soon as there is nothing left to move -------------- */

static void test_stops_when_work_drains(void)
{
    esp_hosted_pump_backend_t b = mk_backend(4, true);
    fake_reset();
    F.rx_frames = 2;

    /* Two frames, cap of four: exactly two transactions, no idle clocking of
       1600-byte dummy buffers at the slave. */
    CHECK(esp_hosted_pump(&b) == 2);
    CHECK(F.xacts == 2);

    /* Nothing left: the next pass is a no-op. */
    CHECK(esp_hosted_pump(&b) == 0);
    CHECK(F.xacts == 2);
}

/* ---- handshake settling between batched transactions -------------------- */

static void test_settle_between_transactions(void)
{
    /* Real slave behaviour: HS drops after each transaction while the slave
       re-arms its next buffer, then comes back.  Without the settle hook a
       level re-check right after CS-high sees it low and batching degrades to
       one transaction per pass. */
    fake_reset();
    F.rx_frames = 4;
    F.hs_drops_after_xact = true;
    F.settle_recovers     = true;
    esp_hosted_pump_backend_t b = mk_backend(4, true);
    CHECK(esp_hosted_pump(&b) == 4);
    CHECK(F.settles == 3);        /* settled before each transaction after the first */

    /* Same slave, no settle hook wired: correct but degenerate — this is the
       measurement that tells us the hook is load-bearing, not decoration. */
    fake_reset();
    F.rx_frames = 4;
    F.hs_drops_after_xact = true;
    F.settle_recovers     = true;
    esp_hosted_pump_backend_t ns = mk_backend(4, false);
    CHECK(esp_hosted_pump(&ns) == 1);

    /* Slave stays deasserted through the settle window: the batch ends there,
       and we do not spin on it more than once per remaining transaction. */
    fake_reset();
    F.rx_frames = 4;
    F.hs_drops_after_xact = true;
    F.settle_recovers     = false;
    CHECK(esp_hosted_pump(&b) == 1);
    CHECK(F.settles == 1);
    CHECK(F.rx_frames == 3);      /* the remainder waits for the next pass */
}

/* ---- transfer errors ----------------------------------------------------- */

static void test_error_stops_batch(void)
{
    esp_hosted_pump_backend_t b = mk_backend(4, true);

    /* Error on the very first transaction: nothing counted, nothing consumed. */
    fake_reset();
    F.rx_frames  = 4;
    F.tx_frames  = 4;
    F.fail_on_xact = 1;
    CHECK(esp_hosted_pump(&b) == 0);
    CHECK(F.xacts == 1);          /* attempted once... */
    CHECK(F.tx_frames == 4);      /* ...and the frame is still queued for retry */
    CHECK(F.rx_frames == 4);

    /* Error mid-batch: the successful transactions still count, the batch stops
       immediately rather than hammering a failing link. */
    fake_reset();
    F.rx_frames  = 4;
    F.fail_on_xact = 3;
    CHECK(esp_hosted_pump(&b) == 2);
    CHECK(F.xacts == 3);
    CHECK(F.rx_frames == 2);
}

/* ---- defensive ----------------------------------------------------------- */

static void test_malformed_backend(void)
{
    CHECK(esp_hosted_pump(NULL) == 0);

    fake_reset();
    F.rx_frames = 4;

    esp_hosted_pump_backend_t b = mk_backend(4, true);
    esp_hosted_pump_backend_t bad;

    bad = b; bad.hs_active  = NULL; CHECK(esp_hosted_pump(&bad) == 0);
    bad = b; bad.dr_active  = NULL; CHECK(esp_hosted_pump(&bad) == 0);
    bad = b; bad.tx_pending = NULL; CHECK(esp_hosted_pump(&bad) == 0);
    bad = b; bad.xact       = NULL; CHECK(esp_hosted_pump(&bad) == 0);

    CHECK(F.xacts == 0);
}

int main(void)
{
    test_gating();
    test_batches_a_backlog();
    test_batch_is_bounded();
    test_stops_when_work_drains();
    test_settle_between_transactions();
    test_stats();
    test_error_stops_batch();
    test_malformed_backend();

    if (failures) { printf("test_esp_pump: %d FAILURES\n", failures); return 1; }
    printf("test_esp_pump: all tests passed\n");
    return 0;
}
