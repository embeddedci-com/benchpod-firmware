/*
 * xfer.c — the DMA-or-blocking transfer policy (see xfer.h).  No HAL here; this
 * compiles both into the firmware and into the host unit tests.
 */
#include "xfer.h"

/* Would DMA be used for a transfer of n bytes right now?  Requires the start +
   wait hooks, a big-enough transfer, and the backend to report DMA usable. */
static bool use_dma(const xfer_backend_t *be, void *start_hook, size_t n)
{
    return start_hook != NULL &&
           be->wait   != NULL &&
           n >= be->dma_min &&
           be->dma_ready != NULL && be->dma_ready(be->ctx);
}

/* Shared tail: a DMA transfer was started; wait for it, and on timeout/error
   abort so the peripheral is idle for the next call.  Returns the xfer code. */
static int finish_dma(const xfer_backend_t *be)
{
    int r = be->wait(be->ctx, be->timeout_ms);
    if (r != XFER_OK && be->abort) be->abort(be->ctx);
    return r;
}

int xfer_txrx(const xfer_backend_t *be, const uint8_t *tx, uint8_t *rx, size_t n)
{
    if (!be || n == 0) return XFER_ERR;

    if (use_dma(be, (void *)be->start_txrx, n)) {
        if (be->start_txrx(be->ctx, tx, rx, n) == 0)
            return finish_dma(be);
        /* could not start (DMA busy / not inited) — fall back to blocking */
    }
    if (!be->block_txrx) return XFER_ERR;
    return be->block_txrx(be->ctx, tx, rx, n, be->timeout_ms);
}

int xfer_tx(const xfer_backend_t *be, const uint8_t *tx, size_t n)
{
    if (!be || n == 0) return XFER_ERR;

    if (use_dma(be, (void *)be->start_tx, n)) {
        if (be->start_tx(be->ctx, tx, n) == 0)
            return finish_dma(be);
    }
    if (!be->block_tx) return XFER_ERR;
    return be->block_tx(be->ctx, tx, n, be->timeout_ms);
}

int xfer_rx(const xfer_backend_t *be, uint8_t *rx, size_t n)
{
    if (!be || n == 0) return XFER_ERR;

    if (use_dma(be, (void *)be->start_rx, n)) {
        if (be->start_rx(be->ctx, rx, n) == 0)
            return finish_dma(be);
    }
    if (!be->block_rx) return XFER_ERR;
    return be->block_rx(be->ctx, rx, n, be->timeout_ms);
}
