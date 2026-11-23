/* dap_jtag.c — see dap_jtag.h.
 *
 * The JTAG side of the CMSIS-DAP processor, after the semantics of ARM's CMSIS-DAP reference
 * firmware (DAP.c / JTAG_DP.c, Apache-2.0): the device chain set by DAP_JTAG_Configure (device 0
 * is the one nearest TDO), IR scans that put every other device in BYPASS, and JTAG-DP transfers
 * as 35-bit DPACC/APACC scans whose captured value is the ACK and the result of the PREVIOUS
 * transaction (so reads are posted and a packet ends with an RDBUFF read).
 *
 * Every operation is built as a DAP_JTAG_Sequence stream in s_sq and run with jtag_ll_run().  An
 * IR scan has no answer to look at, so it stays queued and runs together with the DR scan that
 * follows it; each DR scan runs on its own, because its ACK decides what happens next (a WAIT
 * means the target ignored the request, so the same scan is sent again).
 */
#include "dap_jtag.h"
#include "dap.h"
#include "jtag_ll.h"
#include "swd_ll.h"      /* SWD_ACK_*, SWD_REQ_*: the CMSIS-DAP transfer bits are the same */

#include <string.h>

#define DAP_OK     0x00u
#define DAP_ERROR  0xFFu
#define XFER_MATCH_VALUE  (1u << 4)
#define XFER_MATCH_MASK   (1u << 5)

/* JTAG-DP instructions (ADIv5) */
#define IR_ABORT   0x8u
#define IR_DPACC   0xAu
#define IR_APACC   0xBu
#define IR_IDCODE  0xEu
#define IR_NONE    0xFFFFFFFFu

#define DP_RDBUFF_READ  (SWD_REQ_RnW | SWD_REQ_A2 | SWD_REQ_A3)

/* ---- the chain (DAP_JTAG_Configure) ---- */
static uint8_t  s_count;
static uint8_t  s_ir_len[DAP_JTAG_DEV_MAX];
static uint16_t s_ir_before[DAP_JTAG_DEV_MAX];   /* IR bits of the devices nearer TDO */
static uint16_t s_ir_after[DAP_JTAG_DEV_MAX];    /* IR bits of the devices nearer TDI */
static uint8_t  s_index;                         /* the device a transfer addresses */
static uint32_t s_ir;                            /* its IR as this packet left it (IR_NONE: unknown) */

void dap_jtag_reset(void) {
    s_count = 0;
    s_index = 0;
    s_ir    = IR_NONE;
}

/* ---- the sequence stream ---- */
/* One IR scan + one DR scan, with every other device of an 8-device chain in BYPASS, fits; a
   longer stream (IR lengths far past what real parts use) fails the scan instead of overrunning. */
#define SQ_MAX 512u
static uint8_t s_sq[SQ_MAX];
static size_t  s_sqn;
static bool    s_sq_full;
/* What one stream captures: a DR scan's 35 bits as a 34-bit and a 1-bit sequence (6 bytes), or
   an IDCODE's 32 (5). */
static uint8_t s_tdo[8];

static void sq_one(unsigned n, bool tms, bool cap, uint64_t tdi) {   /* n = 1..64 */
    unsigned nb = (n + 7u) / 8u;
    if (s_sqn + 1u + nb > SQ_MAX) { s_sq_full = true; return; }
    s_sq[s_sqn++] = (uint8_t)((n & 0x3Fu) | (tms ? 0x40u : 0u) | (cap ? 0x80u : 0u));
    for (unsigned b = 0; b < nb; b++) s_sq[s_sqn++] = (uint8_t)(tdi >> (8u * b));
}

/* Bits i.. of a value whose bits past 63 are `hi`. */
static uint64_t chunk(uint64_t lo, bool hi, uint32_t i) {
    if (i >= 64u) return hi ? ~0ull : 0ull;
    uint64_t v = lo >> i;
    if (i && hi) v |= ~0ull << (64u - i);
    return v;
}

/* n bits of (lo, then `hi` past bit 63), TMS low except on the last bit when tms_last. */
static void sq_word(uint32_t n, bool tms_last, bool cap, uint64_t lo, bool hi) {
    if (n == 0) return;
    uint32_t body = tms_last ? n - 1u : n, i = 0;
    while (i < body) {
        unsigned k = (body - i) > 64u ? 64u : (unsigned)(body - i);
        sq_one(k, false, cap, chunk(lo, hi, i));
        i += k;
    }
    if (tms_last) sq_one(1, true, cap, chunk(lo, hi, i));
}

static void sq_fill(uint32_t n, bool tdi, bool tms_last) {
    sq_word(n, tms_last, false, tdi ? ~0ull : 0ull, tdi);
}

/* TMS bits[0..n) (bit 0 first), TDI high; a run of equal bits is one sequence. */
static void sq_tms_bits(const uint8_t *bits, uint32_t n) {
    uint32_t i = 0;
    while (i < n) {
        bool t = (bits[i / 8u] >> (i % 8u)) & 1u;
        unsigned k = 1;
        while (i + k < n && k < 64u && (((bits[(i + k) / 8u] >> ((i + k) % 8u)) & 1u) == t)) k++;
        sq_one(k, t, false, ~0ull);
        i += k;
    }
}

static void sq_tms(uint32_t v, unsigned n) {
    uint8_t b[4] = { (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24) };
    sq_tms_bits(b, n);
}

/* Run the stream; the captured TDO bits, in order, go to bits[] (LSB first, up to max_bits). */
static int sq_run(uint8_t *bits, unsigned max_bits) {
    bool full = s_sq_full;
    size_t n = s_sqn;
    s_sqn = 0; s_sq_full = false;
    if (full) return -1;
    if (n == 0) return 0;
    if (jtag_ll_run(s_sq, n, s_tdo, sizeof(s_tdo)) != 0) return -1;
    if (!bits) return 0;
    memset(bits, 0, (max_bits + 7u) / 8u);
    size_t i = 0, o = 0;
    unsigned out = 0;
    while (i < n) {
        uint8_t  info = s_sq[i];
        unsigned k    = (info & 0x3Fu) ? (info & 0x3Fu) : 64u;
        unsigned nb   = (k + 7u) / 8u;
        if (info & 0x80u) {
            for (unsigned b = 0; b < k && out < max_bits; b++, out++)
                if ((s_tdo[o + b / 8u] >> (b % 8u)) & 1u) bits[out / 8u] |= (uint8_t)(1u << (out % 8u));
            o += nb;
        }
        i += 1u + nb;
    }
    return 0;
}

static uint32_t bits32(const uint8_t *bits, unsigned at) {
    uint32_t v = 0;
    for (unsigned b = 0; b < 32u; b++)
        if ((bits[(at + b) / 8u] >> ((at + b) % 8u)) & 1u) v |= 1u << b;
    return v;
}

/* ---- scans (from Run-Test/Idle back to Run-Test/Idle) ---- */

/* IR scan of the addressed device, every other device in BYPASS (all ones).  Queued only. */
static void jir(uint32_t ir) {
    if (s_ir == ir) return;
    s_ir = ir;
    unsigned len   = s_ir_len[s_index];
    uint32_t after = s_ir_after[s_index];
    sq_tms(0x3u, 4);                          /* Select-DR, Select-IR, Capture-IR, Shift-IR */
    sq_fill(s_ir_before[s_index], true, false);
    sq_word(len, after == 0, false, (uint64_t)ir | (~0ull << 32), true);
    sq_fill(after, true, true);               /* -> Exit1-IR */
    sq_tms(0x1u, 2);                          /* Update-IR, Run-Test/Idle */
}

/* One DPACC/APACC scan: request bits (RnW, A2, A3) and 32 data bits in; ACK and the previous
   transaction's data out.  Returns the CMSIS-DAP ACK (OK for JTAG's OK/FAULT, WAIT), or
   SWD_ACK_NO_ACK when the engine failed. */
static uint8_t jdr(uint8_t request, uint32_t wdata, uint32_t *rdata, uint8_t idle) {
    uint32_t after = (uint32_t)s_count - s_index - 1u;
    uint64_t v = ((uint64_t)wdata << 3) | ((request >> 1) & 7u);
    sq_tms(0x1u, 3);                          /* Select-DR, Capture-DR, Shift-DR */
    sq_fill(s_index, true, false);            /* the BYPASS bits of the devices nearer TDO */
    sq_word(35, after == 0, true, v, false);
    sq_fill(after, true, true);               /* -> Exit1-DR */
    sq_tms(0x1u, 2);                          /* Update-DR, Run-Test/Idle */
    sq_fill(idle, false, false);              /* idle cycles in Run-Test/Idle */
    uint8_t bits[5];
    if (sq_run(bits, 35) != 0) { s_ir = IR_NONE; return SWD_ACK_NO_ACK; }
    /* ACK[2:0] comes out bit 0 first: 0b010 OK/FAULT, 0b001 WAIT; CMSIS-DAP's code is
       ACK[0] << 1 | ACK[1] | ACK[2] << 2 (OK = 1, WAIT = 2). */
    uint8_t a = bits[0] & 7u;
    uint8_t ack = (uint8_t)(((a & 1u) << 1) | ((a >> 1) & 1u) | (a & 4u));
    if (rdata) *rdata = bits32(bits, 3);
    return ack;
}

/* jdr with the WAIT retries of DAP_TransferConfigure, bounded by the packet's budget. */
static uint8_t jxfer(const dap_jtag_cfg_t *c, uint8_t request, uint32_t *data) {
    uint32_t w = (data && !(request & SWD_REQ_RnW)) ? *data : 0u;
    uint32_t r = 0;
    uint16_t retry = c->wait_retry;
    uint8_t  ack;
    for (;;) {
        ack = jdr(request, w, &r, c->idle_cycles);
        if (ack != SWD_ACK_WAIT || retry == 0 || *c->budget == 0) break;
        retry--;
        (*c->budget)--;
    }
    if (data && (request & SWD_REQ_RnW)) *data = r;
    return ack;
}

static void put_le32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint32_t get_le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ---- DAP_JTAG_Sequence (0x14): [count] then sequences; TDO bytes back ---- */
size_t dap_jtag_sequence(const uint8_t *req, size_t req_len, uint8_t *resp) {
    resp[1] = DAP_OK;
    uint8_t count = (req_len > 1) ? req[1] : 0;
    const uint8_t *p = req + 2, *end = req + req_len;
    size_t i = 0, out = 0;
    for (uint8_t s = 0; s < count; s++) {
        if (p + i >= end) { resp[1] = DAP_ERROR; return 2; }
        uint8_t  info = p[i];
        unsigned nb   = (((info & 0x3Fu) ? (info & 0x3Fu) : 64u) + 7u) / 8u;
        if (p + i + 1u + nb > end) { resp[1] = DAP_ERROR; return 2; }
        if (info & 0x80u) out += nb;
        i += 1u + nb;
    }
    if (2u + out > DAP_PACKET_SIZE) { resp[1] = DAP_ERROR; return 2; }
    if (i && jtag_ll_run(p, i, resp + 2, DAP_PACKET_SIZE - 2u) != 0) { resp[1] = DAP_ERROR; return 2; }
    s_ir = IR_NONE;            /* the host moved the TAPs itself */
    return 2u + out;
}

/* ---- DAP_JTAG_Configure (0x15): [count] [IR length]... ---- */
size_t dap_jtag_configure(const uint8_t *req, size_t req_len, uint8_t *resp) {
    uint8_t count = (req_len > 1) ? req[1] : 0;
    if (count > DAP_JTAG_DEV_MAX || req_len < 2u + count) { resp[1] = DAP_ERROR; return 2; }
    uint32_t bits = 0;
    for (uint8_t n = 0; n < count; n++) {
        s_ir_len[n]    = req[2 + n];
        s_ir_before[n] = (uint16_t)bits;
        bits += req[2 + n];
    }
    for (uint8_t n = 0; n < count; n++) {
        bits -= s_ir_len[n];
        s_ir_after[n] = (uint16_t)bits;
    }
    s_count = count;
    s_index = 0;
    s_ir    = IR_NONE;
    resp[1] = DAP_OK;
    return 2;
}

/* ---- DAP_JTAG_IDCODE (0x16): [index] -> [status][IDCODE] ---- */
size_t dap_jtag_idcode(const uint8_t *req, size_t req_len, uint8_t *resp) {
    uint8_t index = (req_len > 1) ? req[1] : 0;
    if (index >= s_count) { resp[1] = DAP_ERROR; return 2; }
    s_index = index;
    s_ir    = IR_NONE;
    jir(IR_IDCODE);
    uint32_t after = (uint32_t)s_count - s_index - 1u;
    sq_tms(0x1u, 3);                          /* Select-DR, Capture-DR, Shift-DR */
    sq_fill(s_index, true, false);
    sq_word(32, after == 0, true, 0, false);
    sq_fill(after, true, true);
    sq_tms(0x1u, 2);                          /* Update-DR, Run-Test/Idle */
    uint8_t bits[4];
    if (sq_run(bits, 32) != 0) { s_ir = IR_NONE; resp[1] = DAP_ERROR; return 2; }
    resp[1] = DAP_OK;
    put_le32(resp + 2, bits32(bits, 0));
    return 6;
}

/* ---- DAP_WriteABORT (0x08) over JTAG: [index][data] ---- */
size_t dap_jtag_write_abort(const uint8_t *req, size_t req_len, uint8_t *resp) {
    uint8_t index = (req_len > 1) ? req[1] : 0;
    if (req_len < 6 || index >= s_count) { resp[1] = DAP_ERROR; return 2; }
    s_index = index;
    s_ir    = IR_NONE;
    jir(IR_ABORT);
    /* ABORT ignores the ACK; the request bits are 0 (a write to address 0). */
    resp[1] = (jdr(0, get_le32(req + 2), NULL, 0) == SWD_ACK_NO_ACK) ? DAP_ERROR : DAP_OK;
    return 2;
}

/* ---- DAP_Transfer (0x05) over JTAG ---- */
size_t dap_jtag_transfer(const uint8_t *req, size_t req_len, uint8_t *resp, const dap_jtag_cfg_t *c) {
    const uint8_t *p   = req + 1;
    const uint8_t *end = req + req_len;
    uint8_t index = (p < end) ? *p++ : 0;
    uint8_t count = (p < end) ? *p++ : 0;
    uint8_t *rp   = resp + 3;
    uint8_t  done = 0;
    uint8_t  ack  = 0;
    uint32_t data;
    bool     post_read = false;

    if (index >= s_count) { resp[1] = 0; resp[2] = 0; return 3; }
    s_index = index;
    s_ir    = IR_NONE;

    for (; count > 0; count--) {
        if (p >= end) break;
        if (rp + 8 > resp + DAP_PACKET_SIZE) break;   /* room for a read and the final flush */
        uint8_t  request = *p++;
        uint32_t ir      = (request & SWD_REQ_APnDP) ? IR_APACC : IR_DPACC;

        if (request & SWD_REQ_RnW) {                          /* ---- read ---- */
            if (post_read) {
                if (s_ir == ir && !(request & XFER_MATCH_VALUE)) {
                    ack = jxfer(c, request, &data);           /* previous out, this one posted */
                } else {
                    jir(IR_DPACC);
                    ack = jxfer(c, DP_RDBUFF_READ, &data);
                    post_read = false;
                }
                if (ack != SWD_ACK_OK) break;
                put_le32(rp, data); rp += 4;
            }
            if (request & XFER_MATCH_VALUE) {
                if (p + 4 > end) break;
                uint32_t match = get_le32(p); p += 4;
                jir(ir);
                ack = jxfer(c, request, NULL);                /* post the read */
                if (ack != SWD_ACK_OK) break;
                uint16_t retry = c->match_retry;
                for (;;) {
                    ack = jxfer(c, request, &data);           /* previous out, posted again */
                    if (ack != SWD_ACK_OK) break;
                    if ((data & *c->match_mask) == match || retry == 0 || *c->budget == 0) break;
                    retry--;
                    (*c->budget)--;
                }
                if (ack != SWD_ACK_OK) break;
                if ((data & *c->match_mask) != match) { ack |= XFER_MATCH_VALUE; break; }
            } else if (!post_read) {
                jir(ir);
                ack = jxfer(c, request, NULL);                /* post the read */
                if (ack != SWD_ACK_OK) break;
                post_read = true;
            }
        } else {                                              /* ---- write ---- */
            if (post_read) {
                jir(IR_DPACC);
                ack = jxfer(c, DP_RDBUFF_READ, &data);
                if (ack != SWD_ACK_OK) break;
                put_le32(rp, data); rp += 4;
                post_read = false;
            }
            if (p + 4 > end) break;
            uint32_t value = get_le32(p); p += 4;
            if (request & XFER_MATCH_MASK) {
                *c->match_mask = value;
                ack = SWD_ACK_OK;
            } else {
                jir(ir);
                ack = jxfer(c, request, &value);
                if (ack != SWD_ACK_OK) break;
            }
        }
        done++;
    }

    /* the last posted read, or a check that the last write went through */
    if (ack == SWD_ACK_OK || done == 0) {
        if (post_read) {
            jir(IR_DPACC);
            ack = jxfer(c, DP_RDBUFF_READ, &data);
            if (ack == SWD_ACK_OK) { put_le32(rp, data); rp += 4; }
        } else if (done > 0) {
            jir(IR_DPACC);
            ack = jxfer(c, DP_RDBUFF_READ, NULL);
        }
    }
    resp[1] = done;
    resp[2] = ack;
    return (size_t)(rp - resp);
}

/* ---- DAP_TransferBlock (0x06) over JTAG ---- */
size_t dap_jtag_transfer_block(const uint8_t *req, size_t req_len, uint8_t *resp, const dap_jtag_cfg_t *c) {
    const uint8_t *p   = req + 1;
    const uint8_t *end = req + req_len;
    uint8_t *rp   = resp + 4;
    uint16_t done = 0;
    uint8_t  ack  = 0;
    uint32_t data;

    if (p + 4 > end) goto out;
    uint8_t  index   = *p++;
    uint16_t count   = (uint16_t)(p[0] | (p[1] << 8)); p += 2;
    uint8_t  request = *p++;
    if (index >= s_count || count == 0) { ack = (index >= s_count) ? 0 : SWD_ACK_OK; goto out; }
    s_index = index;
    s_ir    = IR_NONE;
    uint32_t ir = (request & SWD_REQ_APnDP) ? IR_APACC : IR_DPACC;

    if (request & SWD_REQ_RnW) {
        if (count > (DAP_PACKET_SIZE - 4u) / 4u) count = (DAP_PACKET_SIZE - 4u) / 4u;
        jir(ir);
        ack = jxfer(c, request, NULL);                        /* post the first read */
        if (ack != SWD_ACK_OK) goto out;
        while (count > 0) {
            uint8_t rq = request;
            if (--count == 0) { jir(IR_DPACC); rq = DP_RDBUFF_READ; }   /* the last: RDBUFF */
            ack = jxfer(c, rq, &data);
            if (ack != SWD_ACK_OK) goto out;
            put_le32(rp, data); rp += 4;
            done++;
        }
    } else {
        jir(ir);
        while (count > 0) {
            if (p + 4 > end) break;
            uint32_t value = get_le32(p); p += 4;
            ack = jxfer(c, request, &value);
            if (ack != SWD_ACK_OK) goto out;
            done++;
            count--;
        }
        jir(IR_DPACC);
        ack = jxfer(c, DP_RDBUFF_READ, NULL);                 /* did the last write go through */
    }
out:
    resp[1] = (uint8_t)(done & 0xFF);
    resp[2] = (uint8_t)(done >> 8);
    resp[3] = ack;
    return (size_t)(rp - resp);
}

int dap_jtag_tms(const uint8_t *bits, uint32_t nbits) {
    sq_tms_bits(bits, nbits);
    s_ir = IR_NONE;
    return sq_run(NULL, 0);
}
