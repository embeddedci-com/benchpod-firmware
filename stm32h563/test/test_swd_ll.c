/* test_swd_ll.c — host unit tests for the SWD line-protocol codec (swd_ll.c).
 * A fake fpga_swd_feed() records the emitted remote_bitbang bytes and feeds back
 * programmed sample bits, so we can check that a transfer encodes the request and
 * decodes ACK + data correctly — without any FPGA.  Ported from
 * rp2350/test/test_swd_ll.c to the lightweight CHECK() harness (no Unity).
 */
#include "swd_ll.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

/* ---- fake FPGA feed ----------------------------------------------------- */
#define REC_MAX 4096
static uint8_t rec[REC_MAX];
static size_t  rec_n;
static int     samp_q[512];   /* programmed sample bits, consumed per 'c' */
static int     samp_n, samp_idx;

static void fake_reset(void) { rec_n = 0; samp_n = 0; samp_idx = 0; }
static void push_bit(int b)  { samp_q[samp_n++] = b & 1; }

/* ACK lands s_turnaround samples into the read stream — the standard SWD
 * turnaround, no sampling-latency compensation (w_read_bit samples after the
 * rising edge, so reads are in phase; see swd_ll.c).  With the default turnaround
 * of 1 that's 1 leading sample before ACK; the codec samples and discards it.
 * Push it so ACK and data land where swd_ll.c reads them. */
#define ACK_LEAD 1
static void push_lead(void) { for (int k = 0; k < ACK_LEAD; k++) push_bit(1); }
static void push_ack(uint8_t ack) {   /* 3 bits, LSB first */
    push_bit(ack >> 0); push_bit(ack >> 1); push_bit(ack >> 2);
}
static void push_word(uint32_t v) {   /* 32 data bits LSB-first + even parity */
    int parity = 0;
    for (int k = 0; k < 32; k++) { int b = (v >> k) & 1; push_bit(b); parity += b; }
    push_bit(parity & 1);
}

/* matches signal_engine.h prototype; swd_ll.c links against this */
size_t fpga_swd_feed(const uint8_t *in, size_t len, char *reply, size_t reply_cap,
                     size_t *reply_len, bool *exit_to_json, bool *quit) {
    *exit_to_json = false; *quit = false;
    size_t r = 0;
    for (size_t k = 0; k < len; k++) {
        uint8_t b = in[k];
        if (rec_n < REC_MAX) rec[rec_n++] = b;
        if (b == 'c') {
            int bit = (samp_idx < samp_n) ? samp_q[samp_idx++] : 0;
            if (reply && r < reply_cap) reply[r] = bit ? '1' : '0';
            r++;
        }
    }
    *reply_len = r;
    return len;
}

/* ---- fake NRST pin ------------------------------------------------------ */
/* Since rev3, swd_ll_nreset() drives the pod's dedicated /NRST_CONTROL pin
 * (nrst_ctrl.c) instead of emitting 's'/'r' to the FPGA SWD engine.  Record the
 * calls so the tests can assert that nRESET never reaches the wire stream. */
static int  nrst_calls;
static bool nrst_state;
void nrst_ctrl_assert(bool asserted) { nrst_calls++; nrst_state = asserted; }

static void setup(void) { fake_reset(); nrst_calls = 0; nrst_state = false; swd_ll_reset(); }

/* ---- tests -------------------------------------------------------------- */

static void test_request_encoding(void) {
    setup();
    push_lead();
    push_ack(SWD_ACK_OK);
    push_word(0);
    uint32_t data = 0;
    swd_ll_transfer(SWD_REQ_RnW, &data);
    /* phase 1 starts: drive ('O'), then start bit = write_bit(1) = 'e','g' */
    CHECK(rec[0] == 'O');
    CHECK(rec[1] == 'e');
    CHECK(rec[2] == 'g');
}

static void test_read_returns_value(void) {
    setup();
    push_lead();
    push_ack(SWD_ACK_OK);
    push_word(0xA5A50F0F);
    uint32_t data = 0;
    uint8_t ack = swd_ll_transfer(SWD_REQ_RnW, &data);
    CHECK(ack == SWD_ACK_OK);
    CHECK(data == 0xA5A50F0F);
}

static void test_read_parity_error(void) {
    setup();
    push_lead();
    push_ack(SWD_ACK_OK);
    /* push a word but flip the parity bit -> swd_ll flags SWD_ACK_PERR */
    for (int k = 0; k < 32; k++) push_bit((0xA5A50F0F >> k) & 1);
    int parity = 0; for (int k = 0; k < 32; k++) parity += (0xA5A50F0F >> k) & 1;
    push_bit((parity & 1) ^ 1);   /* wrong parity */
    uint32_t data = 0;
    uint8_t ack = swd_ll_transfer(SWD_REQ_RnW, &data);
    CHECK(ack & SWD_ACK_PERR);
}

static void test_wait_ack(void) {
    setup();
    push_lead();
    push_ack(SWD_ACK_WAIT);       /* no data phase samples needed */
    uint32_t data = 0;
    uint8_t ack = swd_ll_transfer(SWD_REQ_RnW, &data);
    CHECK(ack == SWD_ACK_WAIT);
}

static void test_write_encodes_value(void) {
    setup();
    push_lead();
    push_ack(SWD_ACK_OK);         /* write needs only the 3 ACK samples */
    uint32_t val = 0xC3C30FF0;
    uint8_t ack = swd_ll_transfer(0 /* DP write addr0 */, &val);
    CHECK(ack == SWD_ACK_OK);

    /* decode the 32 write bits after the last 'O' (drive-enable before data). */
    size_t last_O = 0;
    for (size_t k = 0; k < rec_n; k++) if (rec[k] == 'O') last_O = k;
    size_t j = last_O + 1;
    uint32_t got = 0;
    for (int b = 0; b < 32; b++) {
        /* each bit is a pair: 'e'/'g' => 1, 'd'/'f' => 0 */
        int one = (rec[j] == 'e');
        if (one) got |= (uint32_t)1 << b;
        j += 2;
    }
    CHECK(got == 0xC3C30FF0);
}

/* nRESET is a dedicated GPIO now, not an LA channel: swd_ll_nreset() must reach
 * nrst_ctrl_assert() and must NOT put anything on the remote_bitbang stream. */
static void test_nreset_uses_dedicated_pin(void) {
    setup();
    swd_ll_nreset(true);
    CHECK(nrst_calls == 1);
    CHECK(nrst_state == true);
    CHECK(rec_n == 0);          /* nothing emitted to the FPGA */

    swd_ll_nreset(false);
    CHECK(nrst_calls == 2);
    CHECK(nrst_state == false);
    CHECK(rec_n == 0);
}

int main(void) {
    test_nreset_uses_dedicated_pin();
    test_request_encoding();
    test_read_returns_value();
    test_read_parity_error();
    test_wait_ack();
    test_write_encodes_value();
    if (failures == 0) { printf("PASS — all swd_ll tests\n"); return 0; }
    printf("FAILED — %d check(s)\n", failures);
    return 1;
}
