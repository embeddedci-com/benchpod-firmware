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

/* UART_CONFIG (gateware < v60) and UART_CONFIG2 (v60+) payloads: the same fields, DE in rx_ch's
   high nibble for the first and in its own byte (after a reserved ext byte) for the second. */
static void test_config_args(void) {
    uint8_t a[8];
    size_t n = fpga_uart_config_args(a, false, 2, 5, 9, true, 0x032010u, UART_FRAME_PARITY, true);
    CHECK(n == 6);
    CHECK(a[0] == 0x92 && a[1] == 0x05 && a[2] == 0x10 && a[3] == 0x20 && a[4] == 0x03);
    CHECK(a[5] == (0x01 | UART_FRAME_PARITY | UART_FRAME_DE | UART_FRAME_DE_INV));
    n = fpga_uart_config_args(a, true, 3, 3, 12, false, 0xD0u, UART_FRAME_HALFDUPLEX, true);
    CHECK(n == 8);
    CHECK(a[0] == 0x00 && a[1] == 12 && a[2] == 3 && a[3] == 3 && a[4] == 0xD0 && a[5] == 0 && a[6] == 0);
    CHECK(a[7] == (0x01 | UART_FRAME_HALFDUPLEX | UART_FRAME_DE));
    /* no DE: channel bytes 0, no DE flags; disarm: bit 0 clear; stray frame bits are masked */
    n = fpga_uart_config_args(a, true, 7, 8, -1, true, 208u, 0x80u, false);
    CHECK(n == 8 && a[1] == 0 && a[2] == 7 && a[7] == 0x00);
    n = fpga_uart_config_args(a, false, 7, 8, -1, true, 208u, 0, true);
    CHECK(n == 6 && a[0] == 0x07 && a[5] == 0x01);
}

int main(void) {
    uint16_t out;

    test_config_args();

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
