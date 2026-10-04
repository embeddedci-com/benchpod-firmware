/* test_dap.c — host unit tests for the CMSIS-DAP command processor (dap.c),
 * driven against a modelled SWD target (mocks/mock_swd_ll.c).  Validates
 * info/connect/config parsing, single + block transfers, the posted-AP-read
 * pipeline, WAIT retries, and ACK propagation — i.e. that the pod behaves like a
 * real CMSIS-DAP probe.  Ported from rp2350/test/test_dap.c to the lightweight
 * CHECK() harness used by the rest of the STM32 host tests (no Unity).
 */
#include "dap.h"
#include "swd_ll.h"
#include "mocks/mock_swd_ll.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

static uint8_t resp[DAP_PACKET_SIZE];

static bool g_batch;   /* run the suite one-by-one, then batched: the answers must not change */
static void setup(void) { mock_swd_ll_reset(); dap_reset(); mock_swd_ll_batch = g_batch; }

static size_t run(const uint8_t *req, size_t n) {
    memset(resp, 0xEE, sizeof(resp));
    return dap_process(req, n, resp, sizeof(resp));
}

static uint32_t le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ---- DAP_Info ----------------------------------------------------------- */

static void test_info_capabilities(void) {
    setup();
    uint8_t req[] = { 0x00, 0xF0 };
    size_t n = run(req, sizeof(req));
    CHECK(n == 3);
    CHECK(resp[0] == 0x00);
    CHECK(resp[1] == 0x01);   /* 1 info byte */
    CHECK(resp[2] == 0x01);   /* SWD capable  */
}

static void test_info_packet_size(void) {
    setup();
    uint8_t req[] = { 0x00, 0xFF };
    size_t n = run(req, sizeof(req));
    CHECK(n == 4);
    CHECK(resp[1] == 0x02);
    CHECK((uint16_t)(resp[2] | (resp[3] << 8)) == DAP_PACKET_DEFAULT);   /* old clients */
    dap_configure(1024, 4);
    run(req, sizeof(req));
    CHECK((uint16_t)(resp[2] | (resp[3] << 8)) == 1024);
    uint8_t cnt[] = { 0x00, 0xFE };
    run(cnt, sizeof(cnt));
    CHECK(resp[1] == 1 && resp[2] == 4);
    dap_configure(99999, 99);
    run(req, sizeof(req));
    CHECK((uint16_t)(resp[2] | (resp[3] << 8)) == DAP_PACKET_SIZE);
    dap_reset();
    run(cnt, sizeof(cnt));
    CHECK(resp[2] == 1);
}

/* ---- connect / config --------------------------------------------------- */

static void test_connect_swd(void) {
    setup();
    uint8_t req[] = { 0x02, 0x01 };          /* DAP_Connect SWD */
    run(req, sizeof(req));
    CHECK(resp[0] == 0x02);
    CHECK(resp[1] == 0x01);   /* connected as SWD */
}

static void test_connect_jtag_rejected(void) {
    setup();
    uint8_t req[] = { 0x02, 0x02 };          /* DAP_Connect JTAG */
    run(req, sizeof(req));
    CHECK(resp[1] == 0x00);   /* failed */
}

static void test_transfer_configure(void) {
    setup();
    uint8_t req[] = { 0x04, 0x00, 0x64, 0x00, 0x00, 0x00 }; /* idle0 wait100 match0 */
    size_t n = run(req, sizeof(req));
    CHECK(n == 2);
    CHECK(resp[1] == 0x00);   /* DAP_OK */
}

/* ---- DAP_Transfer ------------------------------------------------------- */

static void test_transfer_dp_read(void) {
    setup();
    mock_swd_ll_dp_value = 0x2BA01477;       /* a plausible DPIDR */
    /* count=1, request = DP read addr0 (RnW) */
    uint8_t req[] = { 0x05, 0x00, 0x01, SWD_REQ_RnW };
    size_t n = run(req, sizeof(req));
    CHECK(resp[0] == 0x05);
    CHECK(resp[1] == 0x01);   /* 1 transfer done */
    CHECK(resp[2] == SWD_ACK_OK);
    CHECK(le32(&resp[3]) == 0x2BA01477);
    CHECK(n == 7);
}

static void test_transfer_dp_write(void) {
    setup();
    /* count=1, request = DP write SELECT (A3 set), data 0x12345678 */
    uint8_t req[] = { 0x05, 0x00, 0x01, SWD_REQ_A3, 0x78, 0x56, 0x34, 0x12 };
    run(req, sizeof(req));
    CHECK(resp[1] == 0x01);
    CHECK(resp[2] == SWD_ACK_OK);
    CHECK(mock_swd_ll_writes_n == 1);
    CHECK(mock_swd_ll_writes[0] == 0x12345678);
}

static void test_transfer_ap_read_posted(void) {
    setup();
    /* A single AP read must post then flush via RDBUFF, returning the value. */
    mock_swd_ll_ap_seq[0] = 0xDEADBEEF;
    mock_swd_ll_ap_seq_len = 1;
    uint8_t req[] = { 0x05, 0x00, 0x01, (uint8_t)(SWD_REQ_APnDP | SWD_REQ_RnW) };
    run(req, sizeof(req));
    CHECK(resp[1] == 0x01);
    CHECK(resp[2] == SWD_ACK_OK);
    CHECK(le32(&resp[3]) == 0xDEADBEEF);
}

static void test_transfer_fault_stops(void) {
    setup();
    mock_swd_ll_ack = SWD_ACK_FAULT;
    uint8_t req[] = { 0x05, 0x00, 0x01, SWD_REQ_RnW };
    run(req, sizeof(req));
    CHECK(resp[1] == 0x00);   /* 0 transfers completed */
    CHECK(resp[2] == SWD_ACK_FAULT);
}

static void test_transfer_wait_retries(void) {
    setup();
    mock_swd_ll_wait_before_ok = 3;          /* 3 WAITs, then OK */
    mock_swd_ll_dp_value = 0xCAFEBABE;
    uint8_t req[] = { 0x05, 0x00, 0x01, SWD_REQ_RnW };
    run(req, sizeof(req));
    CHECK(resp[1] == 0x01);
    CHECK(resp[2] == SWD_ACK_OK);
    CHECK(le32(&resp[3]) == 0xCAFEBABE);
    CHECK(mock_swd_ll_xfer_calls == 4);      /* 3 retries + 1 OK */
}

/* ---- DAP_TransferBlock -------------------------------------------------- */

static void test_block_read_ap(void) {
    setup();
    mock_swd_ll_ap_seq[0] = 0x11111111;
    mock_swd_ll_ap_seq[1] = 0x22222222;
    mock_swd_ll_ap_seq[2] = 0x33333333;
    mock_swd_ll_ap_seq[3] = 0x44444444;
    mock_swd_ll_ap_seq_len = 4;
    /* cmd, index, count=4 (LE), request = AP read DRW (APnDP|RnW|A2|A3) */
    uint8_t req[] = { 0x06, 0x00, 0x04, 0x00,
                      (uint8_t)(SWD_REQ_APnDP | SWD_REQ_RnW | SWD_REQ_A2 | SWD_REQ_A3) };
    size_t n = run(req, sizeof(req));
    CHECK(resp[0] == 0x06);
    CHECK((uint16_t)(resp[1] | (resp[2] << 8)) == 4);
    CHECK(resp[3] == SWD_ACK_OK);
    CHECK(le32(&resp[4])  == 0x11111111);
    CHECK(le32(&resp[8])  == 0x22222222);
    CHECK(le32(&resp[12]) == 0x33333333);
    CHECK(le32(&resp[16]) == 0x44444444);
    CHECK(n == 4 + 4 * 4);
}

static void test_block_write_ap(void) {
    setup();
    /* count=3, request = AP write (APnDP), values 1,2,3 */
    uint8_t req[] = { 0x06, 0x00, 0x03, 0x00, SWD_REQ_APnDP,
                      0x01, 0x00, 0x00, 0x00,
                      0x02, 0x00, 0x00, 0x00,
                      0x03, 0x00, 0x00, 0x00 };
    run(req, sizeof(req));
    CHECK((uint16_t)(resp[1] | (resp[2] << 8)) == 3);
    CHECK(resp[3] == SWD_ACK_OK);
    CHECK(mock_swd_ll_writes_n == 3);
    CHECK(mock_swd_ll_writes[0] == 1);
    CHECK(mock_swd_ll_writes[1] == 2);
    CHECK(mock_swd_ll_writes[2] == 3);
}

/* ---- misc --------------------------------------------------------------- */

static void test_swj_pins_nreset(void) {
    setup();
    /* assert nRESET: output bit7=0, select bit7=1 */
    uint8_t req[] = { 0x10, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00 };
    run(req, sizeof(req));
    CHECK(mock_swd_ll_nreset_calls == 1);
    CHECK(mock_swd_ll_last_nreset);   /* asserted (driven low) */
}

static void test_unknown_command(void) {
    setup();
    uint8_t req[] = { 0x7E };
    size_t n = run(req, sizeof(req));
    CHECK(n == 1);
    CHECK(resp[0] == 0xFF);   /* DAP_Invalid */
}

/* The read counts are the host's: a request for more words than the packet holds must return a
 * short count, never write past the response buffer.  (Before the bounds, a 65535-word block read
 * or a 255-read DAP_Transfer overran it; the LAN API is unauthenticated.) */
static struct { uint8_t r[DAP_PACKET_SIZE]; uint8_t guard[64]; } g;

static bool guard_intact(void) {
    for (size_t i = 0; i < sizeof(g.guard); i++) if (g.guard[i] != 0x5A) return false;
    return true;
}

static void test_reads_never_overrun_the_response(void) {
    setup();
    mock_swd_ll_dp_value = 0x12345678;
    memset(g.guard, 0x5A, sizeof(g.guard));
    uint8_t blk[] = { 0x06, 0x00, 0xFF, 0xFF, (uint8_t)SWD_REQ_RnW };   /* 65535 DP reads */
    size_t n = dap_process(blk, sizeof(blk), g.r, DAP_PACKET_SIZE);
    uint16_t done = (uint16_t)(g.r[1] | (g.r[2] << 8));
    CHECK(n <= DAP_PACKET_SIZE);
    CHECK(done == (DAP_PACKET_SIZE - 4) / 4);
    CHECK(n == 4 + 4u * done);
    CHECK(le32(&g.r[4]) == 0x12345678);
    CHECK(guard_intact());

    setup();
    mock_swd_ll_dp_value = 0x0BADF00D;
    memset(g.guard, 0x5A, sizeof(g.guard));
    uint8_t xf[3 + 255];
    xf[0] = 0x05; xf[1] = 0x00; xf[2] = 255;                           /* 255 DP reads */
    memset(xf + 3, SWD_REQ_RnW, 255);
    n = dap_process(xf, sizeof(xf), g.r, DAP_PACKET_SIZE);
    CHECK(n <= DAP_PACKET_SIZE);
    CHECK(n == 3 + 4u * g.r[1]);
    CHECK(g.r[1] > 200);                                                 /* most of them still ran */
    CHECK(guard_intact());

    /* DAP_SWD_Sequence: 255 input sequences of 64 bits would return 2040 bytes. */
    setup();
    memset(g.guard, 0x5A, sizeof(g.guard));
    uint8_t sq[2 + 255];
    sq[0] = 0x1D; sq[1] = 255;
    memset(sq + 2, 0x80, 255);                                           /* input, 0 = 64 bits */
    n = dap_process(sq, sizeof(sq), g.r, DAP_PACKET_SIZE);
    CHECK(n <= DAP_PACKET_SIZE);
    CHECK(g.r[1] == 0xFF);                                               /* DAP_ERROR: refused the rest */
    CHECK(guard_intact());
}

/* A FAULT in the middle of a batched DAP_Transfer must answer like the one-by-one loop breaking
 * there: the count of transfers before it, FAULT, and only the data read before it. */
static void test_fault_mid_transfer_same_answer(void) {
    uint8_t want[DAP_PACKET_SIZE], got_n_one, got_n_batch;
    size_t n1, n2;
    /* DP read, DP write, DP read, DP read: the third bus op FAULTs */
    uint8_t req[] = { 0x05, 0x00, 4, SWD_REQ_RnW, 0x08, 1, 2, 3, 4, SWD_REQ_RnW, SWD_REQ_RnW };
    for (int b = 0; b < 2; b++) {
        mock_swd_ll_reset(); dap_reset(); mock_swd_ll_batch = (b == 1);
        mock_swd_ll_dp_value = 0x11223344;
        mock_swd_ll_fail_at = 3; mock_swd_ll_fail_ack = SWD_ACK_FAULT;
        size_t n = run(req, sizeof(req));
        if (b == 0) { memcpy(want, resp, n); n1 = n; got_n_one = resp[1]; }
        else {
            n2 = n; got_n_batch = resp[1];
            CHECK(n2 == n1);
            CHECK(memcmp(want, resp, n) == 0);
            CHECK(got_n_batch == got_n_one && got_n_batch == 2);
            CHECK(resp[2] == SWD_ACK_FAULT);
            CHECK(mock_swd_ll_batch_calls == 1);
        }
    }
}

static void run_suite(void) {
    test_info_capabilities();
    test_info_packet_size();
    test_reads_never_overrun_the_response();
    test_connect_swd();
    test_connect_jtag_rejected();
    test_transfer_configure();
    test_transfer_dp_read();
    test_transfer_dp_write();
    test_transfer_ap_read_posted();
    test_transfer_fault_stops();
    test_transfer_wait_retries();
    test_block_read_ap();
    test_block_write_ap();
    test_swj_pins_nreset();
    test_unknown_command();
}

/* A target that answers WAIT forever, with the host's retry count at its maximum: one packet must
   come back after a bounded number of tries (with WAIT), not hold the pod for minutes. */
static void test_endless_wait_is_bounded(void) {
    setup();
    uint8_t cfg[] = { 0x04, 0x00, 0xFF, 0xFF, 0xFF, 0xFF };   /* wait retry 65535, match 65535 */
    run(cfg, sizeof(cfg));
    mock_swd_ll_wait_before_ok = 1 << 30;
    uint8_t xf[3 + 255];
    xf[0] = 0x05; xf[1] = 0x00; xf[2] = 255;
    memset(xf + 3, SWD_REQ_RnW, 255);                      /* 255 DP reads */
    run(xf, sizeof(xf));
    CHECK(resp[2] == SWD_ACK_WAIT);
    CHECK(mock_swd_ll_xfer_calls <= 2048 + 255);
}

int main(void) {
    g_batch = false; run_suite();
    g_batch = true;  run_suite();
    test_fault_mid_transfer_same_answer();
    g_batch = false; test_endless_wait_is_bounded();
    if (failures == 0) { printf("PASS — all dap tests\n"); return 0; }
    printf("FAILED — %d check(s)\n", failures);
    return 1;
}
