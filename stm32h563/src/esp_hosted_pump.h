#ifndef ESP_HOSTED_PUMP_H
#define ESP_HOSTED_PUMP_H

#include <stdint.h>
#include <stdbool.h>

/* ---- ESP-Hosted transaction pump (transport-independent) -------------------
 *
 * The batching half of the esp-hosted SPI transport, split out from
 * esp_hosted_spi.c so it can be unit-tested on the host without any HAL.
 *
 * Background: the transport used to run at most ONE 1600-byte transaction per
 * net_poll() pass, and net_poll() runs on a 1 ms tick (main.c net_task).  That
 * capped RX/TX at one frame per millisecond regardless of how much the slave
 * had queued — a backlog drained at 1 frame/ms even though a transaction only
 * takes ~1.6 ms of SPI time and the link is otherwise idle.  The pump keeps
 * clocking transactions while the slave says it is ready and there is still
 * something to move, bounded so the net task cannot be starved of lwIP /
 * Ethernet servicing by a chatty (or stuck-asserted) slave.
 *
 * The bound matters: everything below runs inside net_poll(), so a batch of N
 * transactions delays ethernetif_input() and sys_check_timeouts() by N
 * transaction times.  Keep max_batch small.
 *
 * Handshake subtlety: the slave drops HANDSHAKE for a short moment after each
 * transaction while it re-arms its next buffer.  A naive level re-check right
 * after CS goes high therefore almost always sees it low, which would make
 * batching a no-op.  The optional hs_settle() hook gives the slave a bounded
 * moment to re-assert — but only mid-batch, once we already know work remains.
 * The first transaction of a pass never settles, so an idle link costs exactly
 * what it did before: two GPIO reads.
 * ---------------------------------------------------------------------------*/

/* Default cap on transactions per esp_hosted_pump() call. At ~1.6 ms each
   (1600 B, SPI4 /32) four transactions bound the net-task stall at ~7 ms, well
   inside the 8 s WD_TASK_NET grace window and short enough that the MAC's RX
   descriptor ring absorbs the Ethernet traffic arriving meanwhile. */
#ifndef ESP_HOSTED_PUMP_MAX_BATCH
#define ESP_HOSTED_PUMP_MAX_BATCH   4u
#endif

/* Optional instrumentation.  batch_len[] is the evidence that batching is
   actually happening on hardware: index = transactions completed in one pump
   call, so any non-zero count at index >= 2 is something the pre-batching
   transport could not produce.  settle_hit/settle_miss say whether the
   HANDSHAKE settle window is the right length. */
typedef struct {
    uint32_t calls;                              /* esp_hosted_pump() calls    */
    uint32_t xacts;                              /* transactions completed     */
    uint32_t batch_len[ESP_HOSTED_PUMP_MAX_BATCH + 1];
    uint32_t settle_hit;                         /* HS came back in the window */
    uint32_t settle_miss;                        /* ...did not; batch ended    */
    uint32_t xact_err;                           /* transfer errors            */
} esp_hosted_pump_stats_t;

typedef struct {
    /* Slave says it is ready for a transaction (HANDSHAKE). */
    bool (*hs_active)(void *ctx);
    /* Slave says it has a frame for us (DATA_READY). */
    bool (*dr_active)(void *ctx);
    /* We have at least one frame queued outbound. */
    bool (*tx_pending)(void *ctx);
    /* Run one full-duplex transaction and demux what came back.
       Returns 0 on success, <0 on transfer error (the pump stops; any queued
       TX frame must be left queued by the callee so it retries next pass). */
    int  (*xact)(void *ctx);
    /* Optional: wait a bounded moment for hs_active() to come back mid-batch.
       Returns true if HANDSHAKE became active.  NULL = never wait. */
    bool (*hs_settle)(void *ctx);

    void   *ctx;
    uint8_t max_batch;   /* 0 -> ESP_HOSTED_PUMP_MAX_BATCH */

    /* Optional counters, updated in place.  NULL = no instrumentation. */
    esp_hosted_pump_stats_t *stats;
} esp_hosted_pump_backend_t;

/* Drive the transport for one net_poll() pass.
   Returns the number of transactions completed successfully (0 if the link had
   nothing to do, or if the slave was not ready). */
int esp_hosted_pump(const esp_hosted_pump_backend_t *b);

#endif /* ESP_HOSTED_PUMP_H */
