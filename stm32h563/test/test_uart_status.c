/*
 * test_uart_status.c — host unit tests for fpga_uart_avail_ok (signal_engine.h),
 * the guard that stops a UART proxy from flooding 0xFF when the iCE40 isn't
 * responding over SPI.
 *
 * Regression guard: a floating MISO / unconfigured FPGA reads 0xFF for every SPI
 * byte, so CMD_UART_STATUS reports rx_avail = 0xFFFF.  Treating that as "65535 bytes
 * available" made fpga_uart_read spew 0xFF at the client (overrunning the tunnel and
 * tearing the proxy down with "[uart] disabled").  A real 9-bit FIFO count is at most
 * FPGA_UART_RX_FIFO (256).
 */
#include "signal_engine.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

int main(void) {
    uint16_t out;

    /* Empty FIFO — the common idle case. */
    out = 0xDEAD;
    CHECK(fpga_uart_avail_ok(0, &out));
    CHECK(out == 0);

    /* Mid-range and exactly-full (256) are valid counts. */
    CHECK(fpga_uart_avail_ok(1, NULL));
    CHECK(fpga_uart_avail_ok(200, NULL));
    out = 0xDEAD;
    CHECK(fpga_uart_avail_ok(FPGA_UART_RX_FIFO, &out));
    CHECK(out == FPGA_UART_RX_FIFO);

    /* One past the FIFO is already impossible -> reject. */
    CHECK(!fpga_uart_avail_ok(FPGA_UART_RX_FIFO + 1, NULL));

    /* The floating-MISO signature (all-ones) is rejected — the whole point. */
    CHECK(!fpga_uart_avail_ok(0xFFFF, NULL));
    CHECK(!fpga_uart_avail_ok(0xFF00, NULL));
    CHECK(!fpga_uart_avail_ok(0x0100 + 1, NULL));   /* 257 */

    /* A rejected read must NOT write *out (caller keeps its "no data" default). */
    out = 0xDEAD;
    CHECK(!fpga_uart_avail_ok(0xFFFF, &out));
    CHECK(out == 0xDEAD);

    if (failures) { printf("test_uart_status: %d FAILURES\n", failures); return 1; }
    printf("test_uart_status: all passed\n");
    return 0;
}
