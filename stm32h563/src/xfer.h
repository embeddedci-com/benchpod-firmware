/*
 * xfer.h — hardware-independent transfer policy for the SPI/XSPI read/write
 * paths.  One place decides "DMA or blocking?" and drives the completion
 * handshake, so every peripheral (SPI1/iCE40, SPI4/esp-hosted, OCTOSPI/PSRAM)
 * shares the same tested state machine instead of open-coding it.
 *
 * The policy is:
 *   - transfers smaller than backend->dma_min always go blocking (DMA setup
 *     overhead beats the transfer; also covers the pre-scheduler init path);
 *   - otherwise, if the backend says DMA is usable right now, start the DMA and
 *     block the calling task on its completion (the task yields the CPU while
 *     the controller moves the bytes) — on a *start* failure fall back to
 *     blocking, but on a completion timeout/error report it (never silently
 *     spin a blocking retry that could wedge).
 *
 * The peripheral primitives (start DMA / wait / abort / blocking equivalents /
 * "is DMA usable") are injected via xfer_backend_t.  Firmware wires them to the
 * STM32 HAL DMA calls plus a FreeRTOS completion waiter (dma_wait.*); the host
 * unit tests wire them to a fake, so the decision logic below is fully testable
 * off-target.  This file pulls in NO HAL headers on purpose.
 */
#ifndef XFER_H
#define XFER_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

enum {
    XFER_OK      =  0,
    XFER_ERR     = -1,   /* start/blocking failed, or bad arguments        */
    XFER_TIMEOUT = -2,   /* DMA started but never signalled completion     */
};

/* Per-peripheral primitives the policy engine drives.  Any of the start / wait /
 * abort hooks may be NULL — a NULL start hook (or NULL wait) simply means "this
 * peripheral has no DMA path", so the policy always uses the blocking
 * equivalent.  The block hooks for the directions actually used must be set. */
typedef struct xfer_backend {
    /* Kick off a non-blocking (DMA) transfer.  Return 0 if it started, <0 if it
       could not be started right now (policy then falls back to blocking). */
    int  (*start_txrx)(void *ctx, const uint8_t *tx, uint8_t *rx, size_t n);
    int  (*start_tx)  (void *ctx, const uint8_t *tx, size_t n);
    int  (*start_rx)  (void *ctx, uint8_t *rx, size_t n);

    /* Block until the in-flight transfer completes.  Return XFER_OK on
       completion, XFER_TIMEOUT if it did not finish in time, XFER_ERR on a
       transfer error reported by the controller. */
    int  (*wait)      (void *ctx, uint32_t timeout_ms);

    /* Abort the in-flight transfer.  Called before returning a timeout/error so
       the peripheral+DMA are left idle for the next call. */
    void (*abort)     (void *ctx);

    /* Blocking equivalents, used when DMA is not chosen/available. */
    int  (*block_txrx)(void *ctx, const uint8_t *tx, uint8_t *rx, size_t n, uint32_t timeout_ms);
    int  (*block_tx)  (void *ctx, const uint8_t *tx, size_t n, uint32_t timeout_ms);
    int  (*block_rx)  (void *ctx, uint8_t *rx, size_t n, uint32_t timeout_ms);

    /* True when DMA may be used right now: DMA inited, the RTOS scheduler is
       running, and we are not in an ISR.  When false the blocking path is used
       regardless of size (this is what keeps init-time transfers safe). */
    bool (*dma_ready) (void *ctx);

    void    *ctx;         /* opaque backend context (handle + waiter)          */
    size_t   dma_min;     /* transfers < this many bytes always go blocking    */
    uint32_t timeout_ms;  /* completion timeout for the DMA path               */
} xfer_backend_t;

/* Full-duplex: clock out tx[0..n) while clocking in rx[0..n).  tx or rx may be
   the same buffer only if the backend's blocking/DMA primitives allow it. */
int xfer_txrx(const xfer_backend_t *be, const uint8_t *tx, uint8_t *rx, size_t n);

/* Transmit-only (rx discarded). */
int xfer_tx(const xfer_backend_t *be, const uint8_t *tx, size_t n);

/* Receive-only: the backend supplies whatever is clocked out on the TX line. */
int xfer_rx(const xfer_backend_t *be, uint8_t *rx, size_t n);

#endif /* XFER_H */
