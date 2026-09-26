/* swd_ll.c — see swd_ll.h.  Bit-level SWD over the FPGA remote_bitbang engine. */

#include "swd_ll.h"
#include "signal_engine.h"   /* fpga_swd_feed() */
#include "nrst_ctrl.h"       /* the dedicated /NRST_CONTROL pin (rev3+) */

#include <string.h>

/* remote_bitbang command bytes understood by swd_engine.v:
 *   'd'/'e'/'f'/'g' = (SWCLK,SWDIO) = (0,0)/(0,1)/(1,0)/(1,1)
 *   'O' / 'o'       = drive SWDIO / release SWDIO (turnaround)
 *   'c'             = sample SWDIO -> one ASCII '0'/'1' in the reply
 *   's' / 'r'       = assert / deassert nRESET (active low) — NO LONGER USED,
 *                     nRESET is the pod's own PF4 pin, not an LA channel.
 */

/* Worst case in one phase: 32 data bits + parity read = 33 *3 bytes, plus a
 * turnaround, drive toggle and up to 255 idle clocks (idle*2). 768 covers it. */
#define SWD_WIRE_MAX 768

static uint8_t s_turnaround = 1;   /* 1..4 */
static bool    s_data_phase = false;
static uint8_t s_idle       = 0;

/* ---- remote_bitbang byte-stream builder ---------------------------------- */

typedef struct {
    uint8_t buf[SWD_WIRE_MAX];
    size_t  n;
} wire_t;

static inline void w_byte(wire_t *w, uint8_t b) {
    if (w->n < sizeof(w->buf)) w->buf[w->n++] = b;
}

/* Drive one bit: SWCLK low then high, SWDIO held at `bit` (target samples on
 * the rising edge). */
static inline void w_write_bit(wire_t *w, int bit) {
    if (bit) { w_byte(w, 'e'); w_byte(w, 'g'); }   /* (0,1) (1,1) */
    else     { w_byte(w, 'd'); w_byte(w, 'f'); }   /* (0,0) (1,0) */
}

/* Read one bit: clock SWCLK low then high, and sample SWDIO AFTER the rising
 * edge.  swd_engine.v registers SWCLK but samples SWDIO combinationally, so
 * sampling on the high phase (after 'f') puts the sample in phase with the clock
 * the target sees — no one-bit lag, and no extra clock per transfer.  SWDIO must
 * already be released ('o'); the 'd'/'f' SWDIO value is don't-care while released. */
static inline void w_read_bit(wire_t *w) {
    w_byte(w, 'd');   /* SWCLK low                       */
    w_byte(w, 'f');   /* SWCLK high (rising edge)        */
    w_byte(w, 'c');   /* sample SWDIO after the rising edge */
}

/* One clock with no data meaning: turnaround / idle / dummy. */
static inline void w_clock(wire_t *w) {
    w_byte(w, 'd');
    w_byte(w, 'f');
}

static inline void w_drive(wire_t *w)   { w_byte(w, 'O'); }
static inline void w_release(wire_t *w) { w_byte(w, 'o'); }

/* Feed the built wire stream to the FPGA, collecting every 'c' sample as an
 * ASCII '0'/'1' into samples[].  Loops in case fpga_swd_feed() chunks. */
static size_t swd_feed(const wire_t *w, char *samples, size_t cap) {
    size_t off = 0, got = 0;
    while (off < w->n) {
        size_t reply_len = 0;
        bool   exit_json = false, quit = false;
        size_t consumed = fpga_swd_feed(w->buf + off, w->n - off,
                                        samples + got, cap - got,
                                        &reply_len, &exit_json, &quit);
        off += consumed;
        got += reply_len;
        if (consumed == 0 || quit) {   /* stuck feed, or the SWD engine is no longer armed */
            /* Read the missing samples as line-high: an all-ones ACK is SWD's "no response", so
               the DAP layer reports a failed transfer instead of parsing whatever was here. */
            if (got < cap) memset(samples + got, '1', cap - got);
            break;
        }
    }
    return got;
}

/* ---- public API ---------------------------------------------------------- */

/* Minimum idle SWCLK cycles appended after every transfer.  The AHB-AP needs a
 * few clocks to retire a write before it can ACK the next access; with none, an
 * AP read issued right after an AP write hits WAIT (AP still busy) and our
 * read path — which clocks the whole data window in one feed — then desyncs.
 * The CMSIS-DAP host (OpenOCD) requests 0 idle cycles, so enforce a floor.
 * (2 already works on the bench STM32F4; 8 leaves margin for faster targets.) */
#define SWD_MIN_IDLE 8u

void swd_ll_reset(void) {
    s_turnaround = 1;
    s_data_phase = false;
    s_idle       = SWD_MIN_IDLE;
}

void swd_ll_configure(uint8_t turnaround, bool data_phase) {
    if (turnaround < 1) turnaround = 1;
    if (turnaround > 4) turnaround = 4;
    s_turnaround = turnaround;
    s_data_phase = data_phase;
}

void swd_ll_set_idle(uint8_t idle_cycles) {
    s_idle = (idle_cycles < SWD_MIN_IDLE) ? SWD_MIN_IDLE : idle_cycles;
}

/* nRESET is the pod's dedicated /NRST_CONTROL pin (PF4 -> 330 R -> J1 pin 22),
 * driven open-drain: asserted pulls the DUT's reset net low, released is Hi-Z.
 * It is independent of the FPGA SWD engine, so it works whether or not the
 * engine is armed — and on a v2 board (no such pin) it is a silent no-op, the
 * same "no reset line" behaviour the LA-channel version had when unset. */
void swd_ll_nreset(bool asserted) {
    nrst_ctrl_assert(asserted);
}

/* ACK lands s_turnaround samples into the sampled read stream — the standard SWD
 * turnaround, with no extra clock.  w_read_bit samples on the high phase (after
 * the rising edge), so a sampled read is in phase with the clock the target sees
 * and no longer lags by a bit; the transfer therefore clocks the exact SWD bit
 * count and consecutive transfers stay in sync.  The turnaround is clocked with
 * the SAME read-bit primitive as the data so every sample stays on one phase. */
uint8_t swd_ll_transfer(uint8_t request, uint32_t *data) {
    wire_t w = { .n = 0 };
    char   samp[64];
    int    parity, bit, i;
    uint8_t ack;
    const int ack_lead = s_turnaround;

    /* ---- request: drive 8-bit packet, then release the line ----------- */
    w_drive(&w);                       /* guarantee SWDIO driven for the request */
    parity = 0;
    w_write_bit(&w, 1);                /* start */
    bit = (request >> 0) & 1; w_write_bit(&w, bit); parity += bit;   /* APnDP */
    bit = (request >> 1) & 1; w_write_bit(&w, bit); parity += bit;   /* RnW   */
    bit = (request >> 2) & 1; w_write_bit(&w, bit); parity += bit;   /* A2    */
    bit = (request >> 3) & 1; w_write_bit(&w, bit); parity += bit;   /* A3    */
    w_write_bit(&w, parity & 1);       /* parity */
    w_write_bit(&w, 0);                /* stop   */
    w_write_bit(&w, 1);                /* park   */
    swd_feed(&w, NULL, 0);             /* FEED 1: clock the request out on its own.  The
                                        * resulting processing/SPI gap before we sample the
                                        * ACK lets a slow AP latch its read result first.
                                        * Sampled back-to-back with the request, an AP read
                                        * ACKs OK before its (posted) data is ready and the
                                        * data phase comes back corrupt (parity error); DP
                                        * reads answer instantly and don't need the gap.
                                        * Mirrors DAP_SWD_Sequence: request out, then read. */
    w.n = 0;
    w_release(&w);                     /* turnaround to input */

    if (request & SWD_REQ_RnW) {
        /* ---- read: clock turnaround + ACK + 32 data + parity as ONE continuous
         * feed, then turn the line back and idle.  The ACK and the data MUST stay
         * in the same sampled stream: splitting the data into a second feed makes
         * an AP read's data phase straddle a feed boundary and come back corrupt
         * (DP reads survive the split, AP reads do not).  We therefore always
         * clock the data window — safe here because the request went out in its
         * own feed above, so the AP has already answered and the ACK is OK; we
         * don't reach the WAIT case where the extra 33 clocks would desync the DP.
         * Mirrors the proven DAP_SWD_Sequence path (request out, then one read). */
        for (i = 0; i < ack_lead + 3 + 33; i++) w_read_bit(&w);
        for (i = 0; i < s_turnaround; i++) w_clock(&w);    /* turnaround back */
        w_drive(&w);
        for (i = 0; i < s_idle; i++) w_clock(&w);          /* idle (drives 0) */

        size_t got = swd_feed(&w, samp, sizeof(samp));
        if (got < (size_t)(ack_lead + 3)) return SWD_ACK_NO_ACK;
        ack = (uint8_t)(((samp[ack_lead]     - '0') << 0) |
                        ((samp[ack_lead + 1] - '0') << 1) |
                        ((samp[ack_lead + 2] - '0') << 2));
        if (ack == SWD_ACK_OK) {
            if (got < (size_t)(ack_lead + 3 + 33)) return SWD_ACK_NO_ACK;
            const char *d = samp + ack_lead + 3;
            uint32_t val = 0; int dp = 0;
            for (i = 0; i < 32; i++) {
                int b = d[i] - '0';
                val |= (uint32_t)b << i;
                dp  += b;
            }
            if ((dp & 1) != (d[32] - '0')) ack |= SWD_ACK_PERR;
            if (data) *data = val;
        }
        return ack;
    }

    /* ---- write: sample turnaround + ACK, then drive the data phase ---- */
    for (i = 0; i < ack_lead + 3; i++) w_read_bit(&w);
    {
        size_t got = swd_feed(&w, samp, sizeof(samp));
        if (got < (size_t)(ack_lead + 3)) return SWD_ACK_NO_ACK;
        ack = (uint8_t)(((samp[ack_lead]     - '0') << 0) |
                        ((samp[ack_lead + 1] - '0') << 1) |
                        ((samp[ack_lead + 2] - '0') << 2));
    }

    w.n = 0;
    if (ack == SWD_ACK_OK) {
        /* turnaround back, then write 32 data bits + parity */
        for (i = 0; i < s_turnaround; i++) w_clock(&w);
        w_drive(&w);
        uint32_t val = data ? *data : 0;
        parity = 0;
        for (i = 0; i < 32; i++) {
            bit = (int)(val & 1); val >>= 1;
            w_write_bit(&w, bit); parity += bit;
        }
        w_write_bit(&w, parity & 1);
        for (i = 0; i < s_idle; i++) w_clock(&w);          /* idle (drives 0) */
        swd_feed(&w, NULL, 0);
    } else if (ack == SWD_ACK_WAIT || ack == SWD_ACK_FAULT) {
        for (i = 0; i < s_turnaround; i++) w_clock(&w);    /* turnaround back */
        w_drive(&w);
        if (s_data_phase)
            for (i = 0; i < 33; i++) w_clock(&w);          /* dummy write (0) */
        swd_feed(&w, NULL, 0);
    } else {
        /* no-ack / protocol error: flush the line back to a driven idle */
        for (i = 0; i < s_turnaround + 33; i++) w_clock(&w);
        w_drive(&w);
        swd_feed(&w, NULL, 0);
    }
    return ack;
}

void swd_ll_seq_out(const uint8_t *data, uint32_t nbits) {
    wire_t w = { .n = 0 };
    if (nbits > 256) nbits = 256;
    w_drive(&w);
    for (uint32_t k = 0; k < nbits; k++) {
        int bit = (data[k >> 3] >> (k & 7)) & 1;
        w_write_bit(&w, bit);
    }
    swd_feed(&w, NULL, 0);
}

void swd_ll_seq_in(uint8_t *data, uint32_t nbits) {
    wire_t w = { .n = 0 };
    char   samp[256];
    if (nbits > 256) nbits = 256;
    memset(data, 0, (nbits + 7) / 8);
    w_release(&w);
    for (uint32_t k = 0; k < nbits; k++) w_read_bit(&w);
    size_t got = swd_feed(&w, samp, sizeof(samp));
    for (uint32_t k = 0; k < got && k < nbits; k++)
        if (samp[k] != '0') data[k >> 3] |= (uint8_t)(1u << (k & 7));
}
