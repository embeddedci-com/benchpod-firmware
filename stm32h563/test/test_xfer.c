/*
 * test_xfer.c — host unit tests for the DMA-or-blocking transfer policy
 * (src/xfer.c).  A fake backend records which path each call took and simulates
 * DMA completion (filling rx from a canned pattern), so we can pin down every
 * branch of the decision logic without any HAL/DMA hardware:
 *   - small transfers stay blocking (DMA setup not worth it / pre-scheduler);
 *   - large transfers use DMA when the backend reports it usable;
 *   - dma_ready()==false forces blocking regardless of size;
 *   - a DMA *start* failure falls back to blocking (and still delivers bytes);
 *   - a DMA completion timeout/error is reported (never a silent blocking retry)
 *     and aborts the in-flight transfer;
 *   - tx-only / rx-only variants; zero-length and mis-configured backends error.
 */
#include "xfer.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

/* ---- fake backend -------------------------------------------------------- */

#define PATTERN_MAX 8192

typedef struct {
    /* knobs */
    bool     dma_usable;      /* what dma_ready() returns                     */
    int      start_rc;        /* 0 = DMA starts; <0 = start fails (fallback)  */
    int      wait_rc;         /* XFER_OK / XFER_TIMEOUT / XFER_ERR            */
    uint8_t  pattern[PATTERN_MAX];  /* bytes "received" on any read           */

    /* observations */
    int      dma_starts;      /* start_txrx/tx/rx invocations                 */
    int      blocks;          /* block_txrx/tx/rx invocations                 */
    int      waits;
    int      aborts;
    int      dma_ready_calls;

    /* pending DMA transfer (captured at start, applied at wait) */
    uint8_t *pend_rx;
    size_t   pend_n;
} fake_t;

static fake_t F;

static void fake_reset(void)
{
    memset(&F, 0, sizeof(F));
    F.dma_usable = true;
    F.start_rc   = 0;
    F.wait_rc    = XFER_OK;
    for (size_t i = 0; i < PATTERN_MAX; i++) F.pattern[i] = (uint8_t)(i * 7 + 1);
}

static bool fk_dma_ready(void *ctx) { (void)ctx; F.dma_ready_calls++; return F.dma_usable; }

static int fk_start_txrx(void *ctx, const uint8_t *tx, uint8_t *rx, size_t n)
{
    (void)ctx; (void)tx;
    if (F.start_rc != 0) return F.start_rc;
    F.dma_starts++; F.pend_rx = rx; F.pend_n = n;
    return 0;
}
static int fk_start_rx(void *ctx, uint8_t *rx, size_t n)
{
    (void)ctx;
    if (F.start_rc != 0) return F.start_rc;
    F.dma_starts++; F.pend_rx = rx; F.pend_n = n;
    return 0;
}
static int fk_start_tx(void *ctx, const uint8_t *tx, size_t n)
{
    (void)ctx; (void)tx;
    if (F.start_rc != 0) return F.start_rc;
    F.dma_starts++; F.pend_rx = NULL; F.pend_n = n;
    return 0;
}
static int fk_wait(void *ctx, uint32_t to)
{
    (void)ctx; (void)to;
    F.waits++;
    if (F.wait_rc == XFER_OK && F.pend_rx) memcpy(F.pend_rx, F.pattern, F.pend_n);
    return F.wait_rc;
}
static void fk_abort(void *ctx) { (void)ctx; F.aborts++; }

static int fk_block_txrx(void *ctx, const uint8_t *tx, uint8_t *rx, size_t n, uint32_t to)
{
    (void)ctx; (void)tx; (void)to;
    F.blocks++;
    if (rx) memcpy(rx, F.pattern, n);
    return XFER_OK;
}
static int fk_block_rx(void *ctx, uint8_t *rx, size_t n, uint32_t to)
{
    (void)ctx; (void)to;
    F.blocks++;
    if (rx) memcpy(rx, F.pattern, n);
    return XFER_OK;
}
static int fk_block_tx(void *ctx, const uint8_t *tx, size_t n, uint32_t to)
{
    (void)ctx; (void)tx; (void)n; (void)to;
    F.blocks++;
    return XFER_OK;
}

static xfer_backend_t make_be(size_t dma_min)
{
    xfer_backend_t be = {0};
    be.start_txrx = fk_start_txrx;
    be.start_tx   = fk_start_tx;
    be.start_rx   = fk_start_rx;
    be.wait       = fk_wait;
    be.abort      = fk_abort;
    be.block_txrx = fk_block_txrx;
    be.block_tx   = fk_block_tx;
    be.block_rx   = fk_block_rx;
    be.dma_ready  = fk_dma_ready;
    be.ctx        = NULL;
    be.dma_min    = dma_min;
    be.timeout_ms = 100;
    return be;
}

static bool rx_matches_pattern(const uint8_t *rx, size_t n)
{
    for (size_t i = 0; i < n; i++) if (rx[i] != F.pattern[i]) return false;
    return true;
}

/* ---- tests --------------------------------------------------------------- */

static void test_small_stays_blocking(void)
{
    fake_reset();
    xfer_backend_t be = make_be(/*dma_min=*/16);
    uint8_t rx[8] = {0}, tx[8] = {0};
    CHECK(xfer_txrx(&be, tx, rx, 8) == XFER_OK);
    CHECK(F.dma_starts == 0);          /* below dma_min -> no DMA */
    CHECK(F.blocks == 1);
    CHECK(rx_matches_pattern(rx, 8));
}

static void test_large_uses_dma(void)
{
    fake_reset();
    xfer_backend_t be = make_be(16);
    uint8_t rx[64] = {0}, tx[64] = {0};
    CHECK(xfer_txrx(&be, tx, rx, 64) == XFER_OK);
    CHECK(F.dma_starts == 1);
    CHECK(F.waits == 1);
    CHECK(F.blocks == 0);
    CHECK(F.aborts == 0);
    CHECK(rx_matches_pattern(rx, 64));  /* DMA path delivered the bytes */
}

static void test_dma_not_ready_forces_blocking(void)
{
    fake_reset();
    F.dma_usable = false;               /* e.g. scheduler not running yet */
    xfer_backend_t be = make_be(16);
    uint8_t rx[64] = {0}, tx[64] = {0};
    CHECK(xfer_txrx(&be, tx, rx, 64) == XFER_OK);
    CHECK(F.dma_starts == 0);
    CHECK(F.blocks == 1);
    CHECK(rx_matches_pattern(rx, 64));
}

static void test_start_failure_falls_back(void)
{
    fake_reset();
    F.start_rc = -1;                    /* DMA channel busy / not inited */
    xfer_backend_t be = make_be(16);
    uint8_t rx[64] = {0}, tx[64] = {0};
    CHECK(xfer_txrx(&be, tx, rx, 64) == XFER_OK);
    CHECK(F.dma_starts == 0);           /* start rejected */
    CHECK(F.waits == 0);
    CHECK(F.blocks == 1);               /* fell back */
    CHECK(rx_matches_pattern(rx, 64));
}

static void test_timeout_reported_and_aborts(void)
{
    fake_reset();
    F.wait_rc = XFER_TIMEOUT;
    xfer_backend_t be = make_be(16);
    uint8_t rx[64] = {0}, tx[64] = {0};
    CHECK(xfer_txrx(&be, tx, rx, 64) == XFER_TIMEOUT);
    CHECK(F.dma_starts == 1);
    CHECK(F.aborts == 1);               /* aborted the stuck transfer */
    CHECK(F.blocks == 0);               /* must NOT silently retry blocking */
}

static void test_error_reported_and_aborts(void)
{
    fake_reset();
    F.wait_rc = XFER_ERR;
    xfer_backend_t be = make_be(16);
    uint8_t rx[64] = {0}, tx[64] = {0};
    CHECK(xfer_txrx(&be, tx, rx, 64) == XFER_ERR);
    CHECK(F.aborts == 1);
    CHECK(F.blocks == 0);
}

static void test_rx_only_variants(void)
{
    fake_reset();
    xfer_backend_t be = make_be(16);
    uint8_t rx[64] = {0};
    CHECK(xfer_rx(&be, rx, 64) == XFER_OK);
    CHECK(F.dma_starts == 1);
    CHECK(rx_matches_pattern(rx, 64));

    fake_reset();
    uint8_t rx2[8] = {0};
    CHECK(xfer_rx(&be, rx2, 8) == XFER_OK);   /* small -> blocking */
    CHECK(F.blocks == 1);
    CHECK(rx_matches_pattern(rx2, 8));
}

static void test_tx_only_variant(void)
{
    fake_reset();
    xfer_backend_t be = make_be(16);
    uint8_t tx[64] = {0};
    CHECK(xfer_tx(&be, tx, 64) == XFER_OK);
    CHECK(F.dma_starts == 1);
    CHECK(F.waits == 1);
}

static void test_zero_length_errors(void)
{
    fake_reset();
    xfer_backend_t be = make_be(16);
    uint8_t rx[4] = {0}, tx[4] = {0};
    CHECK(xfer_txrx(&be, tx, rx, 0) == XFER_ERR);
    CHECK(xfer_txrx(NULL, tx, rx, 4) == XFER_ERR);
    CHECK(F.dma_starts == 0 && F.blocks == 0);
}

static void test_no_dma_hooks_uses_blocking(void)
{
    /* A peripheral with no DMA path at all: start_* / wait NULL. */
    fake_reset();
    xfer_backend_t be = make_be(1);     /* dma_min tiny, but no start hook */
    be.start_txrx = NULL;
    be.start_rx   = NULL;
    be.start_tx   = NULL;
    be.wait       = NULL;
    uint8_t rx[64] = {0}, tx[64] = {0};
    CHECK(xfer_txrx(&be, tx, rx, 64) == XFER_OK);
    CHECK(F.dma_starts == 0);
    CHECK(F.dma_ready_calls == 0);      /* never even asked — no start hook */
    CHECK(F.blocks == 1);
}

int main(void)
{
    test_small_stays_blocking();
    test_large_uses_dma();
    test_dma_not_ready_forces_blocking();
    test_start_failure_falls_back();
    test_timeout_reported_and_aborts();
    test_error_reported_and_aborts();
    test_rx_only_variants();
    test_tx_only_variant();
    test_zero_length_errors();
    test_no_dma_hooks_uses_blocking();

    if (failures) { printf("FAIL — %d xfer checks failed\n", failures); return 1; }
    printf("PASS — all xfer tests\n");
    return 0;
}
