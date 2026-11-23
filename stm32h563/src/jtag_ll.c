/* jtag_ll.c — see jtag_ll.h.  Splits a sequence stream into engine runs and aligns TDO bytes. */
#include "jtag_ll.h"
#include "signal_engine.h"   /* fpga_jtag_*, JTAG_QUEUE_MAX, fpga_la_set */
#include "fpga_config.h"     /* FPGA_HFOSC_HZ */

#include <string.h>

static unsigned s_trst_ch;   /* LA channel of nTRST (1..14), 0 = none: set by dap_start */

void jtag_ll_set_trst_channel(unsigned ch) { s_trst_ch = ch; }

static unsigned nbits_of(uint8_t info) {
    unsigned n = info & 0x3Fu;
    return n ? n : 64u;
}

long jtag_ll_tdo_len(const uint8_t *seq, size_t len) {
    size_t i = 0;
    long out = 0;
    while (i < len) {
        uint8_t  info   = seq[i];
        unsigned nbytes = (nbits_of(info) + 7u) / 8u;
        if (i + 1u + nbytes > len) return -1;
        if (info & 0x80u) out += (long)nbytes;
        i += 1u + nbytes;
    }
    return out;
}

/* The gateware shifts TDO in at the top of its byte register, so a capturing sequence's last
   byte of k < 8 bits holds its TDO bits at the top: shift them down. */
static void align(const uint8_t *seq, size_t len, uint8_t *tdo) {
    size_t i = 0, o = 0;
    while (i < len) {
        uint8_t  info   = seq[i];
        unsigned n      = nbits_of(info);
        unsigned nbytes = (n + 7u) / 8u;
        if (info & 0x80u) {
            unsigned k = n & 7u;
            if (k) tdo[o + nbytes - 1u] = (uint8_t)(tdo[o + nbytes - 1u] >> (8u - k));
            o += nbytes;
        }
        i += 1u + nbytes;
    }
}

int jtag_ll_run(const uint8_t *seq, size_t len, uint8_t *tdo, size_t tdo_cap) {
    long total = jtag_ll_tdo_len(seq, len);
    if (total < 0 || (size_t)total > tdo_cap) return -1;
    size_t i = 0, o = 0;
    while (i < len) {
        /* the longest run of whole sequences that fits the queue and the reply buffer */
        size_t start = i, nreply = 0;
        while (i < len) {
            uint8_t  info   = seq[i];
            unsigned nbytes = (nbits_of(info) + 7u) / 8u;
            size_t   r      = (info & 0x80u) ? nbytes : 0u;
            if (i + 1u + nbytes - start > JTAG_QUEUE_MAX || nreply + r > JTAG_QUEUE_MAX) break;
            nreply += r;
            i += 1u + nbytes;
        }
        if (fpga_jtag_run(seq + start, i - start, tdo + o, nreply) != 0) return -2;
        align(seq + start, i - start, tdo + o);
        o += nreply;
    }
    return 0;
}

void jtag_ll_set_clock(uint32_t hz) {
    const uint32_t clk = FPGA_HFOSC_HZ;          /* 24 MHz */
    uint32_t half = hz ? (clk + 2u * hz - 1u) / (2u * hz) : SPI_HALF_MAX;
    fpga_jtag_set_half((unsigned)half);          /* clamps to 2..63 */
}

void jtag_ll_trst(bool asserted) {
    if (s_trst_ch) fpga_la_set(s_trst_ch, asserted ? 0 : 1);   /* 0 = low, 1 = high */
}
