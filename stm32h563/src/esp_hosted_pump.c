/*
 * esp_hosted_pump.c — bounded batching loop for the esp-hosted SPI transport.
 * Pure logic (no HAL): see esp_hosted_pump.h for the rationale, and
 * test/test_esp_pump.c for the behaviour this pins down.
 */
#include "esp_hosted_pump.h"

int esp_hosted_pump(const esp_hosted_pump_backend_t *b)
{
    /* A backend missing any of the four mandatory hooks is a programming
       error, not a runtime condition — refuse to clock anything. */
    if (!b || !b->hs_active || !b->dr_active || !b->tx_pending || !b->xact)
        return 0;

    unsigned cap = b->max_batch ? b->max_batch : ESP_HOSTED_PUMP_MAX_BATCH;
    if (cap > ESP_HOSTED_PUMP_MAX_BATCH) cap = ESP_HOSTED_PUMP_MAX_BATCH;

    esp_hosted_pump_stats_t *st = b->stats;
    int done = 0;

    if (st) st->calls++;

    while ((unsigned)done < cap) {
        /* Is there anything to move?  Either we have a frame queued, or the
           slave is holding DATA_READY.  Checked first so an idle link never
           reaches the settle path below. */
        if (!b->tx_pending(b->ctx) && !b->dr_active(b->ctx))
            break;

        if (!b->hs_active(b->ctx)) {
            /* First pass: plain poll semantics — the slave is simply not ready,
               come back next net_poll().  Mid-batch: it is most likely just
               re-arming after the transaction we just ran, so give it the
               bounded settle window rather than losing the batch. */
            if (done == 0 || !b->hs_settle)
                break;
            if (!b->hs_settle(b->ctx)) {
                if (st) st->settle_miss++;
                break;
            }
            if (st) st->settle_hit++;
        }

        if (b->xact(b->ctx) < 0) {
            if (st) st->xact_err++;
            break;   /* transfer error — retry on the next pass */
        }

        done++;
    }

    if (st) {
        st->xacts += (uint32_t)done;
        st->batch_len[done]++;
    }
    return done;
}
