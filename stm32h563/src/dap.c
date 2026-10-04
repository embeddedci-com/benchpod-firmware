/* dap.c — see dap.h.  CMSIS-DAP v1 command processor, SWD subset. */

#include "dap.h"
#include "swd_ll.h"

#include <string.h>

/* ---- CMSIS-DAP command IDs ----------------------------------------------- */
#define ID_DAP_Info              0x00u
#define ID_DAP_HostStatus        0x01u
#define ID_DAP_Connect           0x02u
#define ID_DAP_Disconnect        0x03u
#define ID_DAP_TransferConfigure 0x04u
#define ID_DAP_Transfer          0x05u
#define ID_DAP_TransferBlock     0x06u
#define ID_DAP_WriteABORT        0x08u
#define ID_DAP_Delay             0x09u
#define ID_DAP_ResetTarget       0x0Au
#define ID_DAP_SWJ_Pins          0x10u
#define ID_DAP_SWJ_Clock         0x11u
#define ID_DAP_SWJ_Sequence      0x12u
#define ID_DAP_SWD_Configure     0x13u
#define ID_DAP_SWD_Sequence      0x1Du
#define ID_DAP_Invalid           0xFFu

#define DAP_OK     0x00u
#define DAP_ERROR  0xFFu

/* DAP_Info IDs */
#define INFO_FW_VER        0x04u
#define INFO_CAPABILITIES  0xF0u
#define INFO_PACKET_COUNT  0xFEu
#define INFO_PACKET_SIZE   0xFFu

/* Transfer request flags (extra bits beyond SWD_REQ_*) */
#define XFER_MATCH_VALUE   (1u << 4)
#define XFER_MATCH_MASK    (1u << 5)

/* DP RDBUFF read: APnDP=0, RnW=1, A[3:2]=0b11 (reg 0xC) */
#define DP_RDBUFF_READ  (SWD_REQ_RnW | SWD_REQ_A2 | SWD_REQ_A3)
/* DP ABORT write: APnDP=0, RnW=0, A=0 */
#define DP_ABORT_WRITE  (0x00u)

/* ---- processor state ----------------------------------------------------- */
static uint8_t  s_idle_cycles;
static uint16_t s_wait_retry;
static uint16_t s_match_retry;
/* WAIT and match-value retries one packet may spend in total. The host sets the per-transfer
   counts (up to 65535 each, x up to 255 transfers per packet), and a target that answers WAIT
   forever then held the hw task for minutes: no other session ran and the watchdog reset the
   pod. ~0.65 ms per queued transfer keeps a packet near 1-2 s; the host (OpenOCD) retries a
   WAIT answer itself, so ending early only hands the decision back to it. */
#define DAP_RETRY_BUDGET     2048u
/* Cap on the batched engine's per-transfer WAIT retries (swd_ll), for the same reason. */
#define DAP_BATCH_WAIT_MAX   64u
static uint32_t s_budget;
static uint32_t s_match_mask;
static uint16_t s_packet_size  = DAP_PACKET_DEFAULT;
static uint8_t  s_packet_count = 1;

/* batching state (see the batching section below) */
#define REC_MAX 128u
static swd_op_t  s_ops[REC_MAX];
static uint8_t  *s_dst[REC_MAX];     /* where a read's value goes (NULL: discarded) */
static uint16_t  s_cnt_at[REC_MAX];  /* the command's transfer count when the op was issued */
static uint8_t  *s_rp_at[REC_MAX];   /* ... and its response write position */
static uint32_t  s_rdata[REC_MAX];
static size_t    s_rec_n;
static bool      s_recording;
static uint16_t  s_cur_cnt;          /* kept current by the transfer loops */
static uint8_t  *s_cur_rp;
static bool      s_failed;
static uint16_t  s_fail_cnt;
static uint8_t  *s_fail_rp;
static uint8_t   s_fail_ack;

void dap_configure(unsigned packet_size, unsigned packet_count) {
    if (packet_size < 64u) packet_size = 64u;
    if (packet_size > DAP_PACKET_SIZE) packet_size = DAP_PACKET_SIZE;
    if (packet_count < 1u) packet_count = 1u;
    if (packet_count > DAP_PACKET_COUNT_MAX) packet_count = DAP_PACKET_COUNT_MAX;
    s_packet_size  = (uint16_t)packet_size;
    s_packet_count = (uint8_t)packet_count;
}

void dap_reset(void) {
    s_idle_cycles = 0;
    s_wait_retry  = 100;
    s_match_retry = 0;
    s_match_mask  = 0;
    s_packet_size  = DAP_PACKET_DEFAULT;
    s_packet_count = 1;
    swd_ll_reset();
    swd_ll_set_idle(0);
    swd_ll_set_wait_retry(100);
    s_recording = false;
    s_rec_n     = 0;
    s_failed    = false;
}

/* ---- little-endian helpers ----------------------------------------------- */
static uint32_t get_le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void put_le32_(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v); p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

/* ---- batching ----------------------------------------------------------------------------
 * With the gateware's SWD queue (swd_ll_batch), DAP_Transfer and DAP_TransferBlock RECORD their
 * transfers instead of running each one: xfer() appends an op and answers OK, store() notes
 * where a read's value goes.  The ops run as one batch when the list fills and when the command
 * ends, and the values are written into the response afterwards.  Each op also remembers the
 * command's transfer count and response position when it was issued, so a batch that stops at
 * op k (FAULT, WAIT retries used up, no ACK, parity) yields exactly the response the one-by-one
 * loop gives when it breaks there.  A DAP_Transfer with a match-value read branches on read
 * data, so it keeps the one-by-one path. */

static uint8_t rec_flush(void) {
    if (s_failed) return s_fail_ack;
    if (s_rec_n == 0) return SWD_ACK_OK;
    uint8_t ack = SWD_ACK_OK;
    size_t done = swd_ll_batch(s_ops, s_rec_n, s_rdata, &ack);
    size_t r = 0;
    for (size_t i = 0; i < done; i++)
        if (s_ops[i].req & SWD_REQ_RnW) { if (s_dst[i]) put_le32_(s_dst[i], s_rdata[r]); r++; }
    if (done < s_rec_n) {
        s_failed   = true;
        s_fail_cnt = s_cnt_at[done];
        s_fail_rp  = s_rp_at[done];
        s_fail_ack = ack;
    }
    s_rec_n = 0;
    return s_failed ? s_fail_ack : SWD_ACK_OK;
}

static void rec_begin(void) { s_recording = true; s_rec_n = 0; s_failed = false; }

/* Run what is left; true when the batch stopped early (the caller then answers with
   s_fail_cnt / s_fail_ack / s_fail_rp). */
static bool rec_end(void) {
    rec_flush();
    s_recording = false;
    return s_failed;
}

/* A read's value into the response (one-by-one), or a note of where it goes (recording). */
static void store(uint8_t *rp, uint32_t v) {
    if (s_recording) { if (s_rec_n) s_dst[s_rec_n - 1] = rp; s_cur_rp = rp + 4; }
    else put_le32_(rp, v);
}

/* One transfer with WAIT-retry (DAP_TransferConfigure wait-retry count). */
static uint8_t xfer(uint8_t request, uint32_t *data) {
    if (s_recording) {
        if (s_failed) return s_fail_ack;
        if (s_rec_n == REC_MAX && rec_flush() != SWD_ACK_OK) return s_fail_ack;
        s_ops[s_rec_n].req  = request & 0x0Fu;
        s_ops[s_rec_n].data = (data && !(request & SWD_REQ_RnW)) ? *data : 0;
        s_dst[s_rec_n]    = NULL;
        s_cnt_at[s_rec_n] = s_cur_cnt;
        s_rp_at[s_rec_n]  = s_cur_rp;
        s_rec_n++;
        if (data && (request & SWD_REQ_RnW)) *data = 0;   /* filled in after the batch */
        return SWD_ACK_OK;
    }
    uint16_t retry = s_wait_retry;
    uint8_t  ack;
    for (;;) {
        ack = swd_ll_transfer(request, data);
        if (ack != SWD_ACK_WAIT || retry == 0 || s_budget == 0) break;
        retry--;
        s_budget--;
    }
    return ack;
}

/* Does this DAP_Transfer contain a match-value read?  (Its loop branches on read data.) */
static bool has_match_read(const uint8_t *req, size_t req_len) {
    const uint8_t *p = req + 2, *end = req + req_len;
    uint8_t count = (p < end) ? *p++ : 0;
    for (; count > 0 && p < end; count--) {
        uint8_t r = *p++;
        if ((r & SWD_REQ_RnW) && (r & XFER_MATCH_VALUE)) return true;
        if (!(r & SWD_REQ_RnW) || (r & XFER_MATCH_VALUE)) p += 4;
    }
    return false;
}

/* ---- DAP_Transfer -------------------------------------------------------- */
static size_t do_transfer(const uint8_t *req, size_t req_len, uint8_t *resp) {
    const uint8_t *p   = req + 1;          /* skip command id   */
    const uint8_t *end = req + req_len;
    p++;                                   /* skip DAP index    */
    uint8_t count = (p < end) ? *p++ : 0;

    uint8_t *rp         = resp + 3;        /* [cmd][count][ack] then read data */
    uint8_t  xfer_count = 0;               /* transfers completed (reads + writes) */
    uint8_t  ack        = DAP_OK;
    bool     post_read = false;
    uint32_t data;

    for (; count > 0; count--) {
        if (p >= end) break;
        /* Room for what this request can add (a finished posted read + a DP read) plus the
           final posted-read flush after the loop.  A host that asks for more reads than the
           advertised packet size holds gets a short count, not a buffer overrun. */
        if (rp + 12 > resp + DAP_PACKET_SIZE) break;
        s_cur_cnt = xfer_count; s_cur_rp = rp;
        uint8_t request = *p++;

        if (request & SWD_REQ_RnW) {                 /* ---- read ---- */
            if (post_read) {
                /* finish the previously posted AP read */
                if ((request & (SWD_REQ_APnDP | XFER_MATCH_VALUE)) == SWD_REQ_APnDP) {
                    ack = xfer(request, &data);      /* returns prev, posts new */
                    if (ack != SWD_ACK_OK) break;    /* (post_read stays true)  */
                } else {
                    ack = xfer(DP_RDBUFF_READ, &data);
                    if (ack != SWD_ACK_OK) break;
                    post_read = false;
                }
                store(rp, data); rp += 4;
            }
            if (request & XFER_MATCH_VALUE) {
                if (p + 4 > end) break;
                uint32_t match_value = get_le32(p); p += 4;
                if (request & SWD_REQ_APnDP) {
                    ack = xfer(request, NULL);       /* post AP read */
                    if (ack != SWD_ACK_OK) break;
                }
                uint16_t retry = s_match_retry;
                for (;;) {
                    ack = xfer((request & SWD_REQ_APnDP) ? DP_RDBUFF_READ : request, &data);
                    if (ack != SWD_ACK_OK) break;
                    if ((data & s_match_mask) == match_value || retry == 0 || s_budget == 0) break;
                    retry--;
                    s_budget--;
                }
                if (ack != SWD_ACK_OK) break;
                if ((data & s_match_mask) != match_value) ack |= XFER_MATCH_VALUE;
            } else if (request & SWD_REQ_APnDP) {
                if (!post_read) {
                    ack = xfer(request, NULL);       /* post AP read */
                    if (ack != SWD_ACK_OK) break;
                    post_read = true;
                }
            } else {
                ack = xfer(request, &data);          /* DP read: immediate */
                if (ack != SWD_ACK_OK) break;
                store(rp, data); rp += 4;
            }
        } else {                                     /* ---- write ---- */
            if (post_read) {
                ack = xfer(DP_RDBUFF_READ, &data);
                if (ack != SWD_ACK_OK) break;
                store(rp, data); rp += 4;
                post_read = false;
            }
            if (p + 4 > end) break;
            uint32_t value = get_le32(p); p += 4;
            if (request & XFER_MATCH_MASK) {
                s_match_mask = value;                /* config only, no bus op */
            } else {
                ack = xfer(request, &value);
                if (ack != SWD_ACK_OK) break;
            }
        }
        xfer_count++;   /* count every fully-completed transfer (read or write) */
    }

    /* flush a trailing posted read (data only — its transfer was already counted) */
    s_cur_cnt = xfer_count; s_cur_rp = rp;
    if (post_read && ack == SWD_ACK_OK) {
        ack = xfer(DP_RDBUFF_READ, &data);
        if (ack == SWD_ACK_OK) { store(rp, data); rp += 4; }
    }

    if (s_recording && rec_end()) { xfer_count = (uint8_t)s_fail_cnt; ack = s_fail_ack; rp = s_fail_rp; }
    resp[1] = xfer_count;
    resp[2] = ack;
    return (size_t)(rp - resp);
}

/* ---- DAP_TransferBlock --------------------------------------------------- */
static size_t do_transfer_block(const uint8_t *req, size_t req_len, uint8_t *resp) {
    const uint8_t *p   = req + 1;          /* skip command id */
    const uint8_t *end = req + req_len;
    p++;                                   /* skip DAP index  */
    if (p + 3 > end) { resp[1] = 0; resp[2] = 0; resp[3] = 0; return 4; }
    uint16_t count   = (uint16_t)(p[0] | (p[1] << 8)); p += 2;
    uint8_t  request = *p++;

    uint8_t *rp   = resp + 4;              /* [cmd][cnt_lo][cnt_hi][ack] */
    uint16_t done = 0;
    uint8_t  ack  = SWD_ACK_OK;
    uint32_t data;

    if (count == 0) { resp[1] = 0; resp[2] = 0; resp[3] = SWD_ACK_OK; return 4; }

    if (request & SWD_REQ_RnW) {                          /* block read */
        /* Never more words than the response holds (the count is the host's). */
        if (count > (DAP_PACKET_SIZE - 4u) / 4u) count = (DAP_PACKET_SIZE - 4u) / 4u;
        s_cur_cnt = 0; s_cur_rp = rp;
        if (request & SWD_REQ_APnDP) {
            ack = xfer(request, NULL);                    /* prime posted read */
            if (ack != SWD_ACK_OK) goto done;
        }
        while (count > 0) {
            uint8_t rq = request;
            if ((request & SWD_REQ_APnDP) && count == 1)
                rq = DP_RDBUFF_READ;                      /* last: flush via RDBUFF */
            s_cur_cnt = done; s_cur_rp = rp;
            ack = xfer(rq, &data);
            if (ack != SWD_ACK_OK) goto done;
            store(rp, data); rp += 4; done++;
            count--;
        }
    } else {                                              /* block write */
        while (count > 0) {
            if (p + 4 > end) break;
            uint32_t value = get_le32(p); p += 4;
            s_cur_cnt = done; s_cur_rp = rp;
            ack = xfer(request, &value);
            if (ack != SWD_ACK_OK) goto done;
            done++;
            count--;
        }
    }
done:
    if (s_recording && rec_end()) { done = s_fail_cnt; ack = s_fail_ack; rp = s_fail_rp; }
    resp[1] = (uint8_t)(done & 0xFF);
    resp[2] = (uint8_t)(done >> 8);
    resp[3] = ack;
    return (size_t)(rp - resp);
}

/* ---- DAP_Info ------------------------------------------------------------ */
static size_t do_info(const uint8_t *req, size_t req_len, uint8_t *resp) {
    uint8_t id = (req_len > 1) ? req[1] : 0;
    switch (id) {
        case INFO_CAPABILITIES:
            resp[1] = 1; resp[2] = 0x01;            /* bit0 = SWD */
            return 3;
        case INFO_PACKET_COUNT:
            resp[1] = 1; resp[2] = s_packet_count;
            return 3;
        case INFO_PACKET_SIZE:
            resp[1] = 2;
            resp[2] = (uint8_t)(s_packet_size & 0xFF);
            resp[3] = (uint8_t)(s_packet_size >> 8);
            return 4;
        case INFO_FW_VER: {
            static const char ver[] = "1.2.0";      /* CMSIS-DAP protocol version */
            resp[1] = (uint8_t)(sizeof(ver));        /* includes NUL */
            memcpy(resp + 2, ver, sizeof(ver));
            return 2 + sizeof(ver);
        }
        default:                                     /* strings / unsupported */
            resp[1] = 0;
            return 2;
    }
}

/* ---- DAP_SWD_Sequence ---------------------------------------------------- */
static size_t do_swd_sequence(const uint8_t *req, size_t req_len, uint8_t *resp) {
    uint8_t        seqs = (req_len > 1) ? req[1] : 0;
    const uint8_t *p    = req + 2;
    const uint8_t *end  = req + req_len;
    uint8_t       *rp   = resp + 2;          /* [cmd][status] then input data */
    resp[1] = DAP_OK;

    for (uint8_t s = 0; s < seqs; s++) {
        if (p >= end) break;
        uint8_t  info   = *p++;
        uint32_t nbits  = info & 0x3Fu;
        if (nbits == 0) nbits = 64;
        uint32_t nbytes = (nbits + 7) / 8;
        if (info & 0x80u) {                  /* input */
            /* Each input sequence costs one request byte but returns up to 8: 255 of them
               would write ~2 KB into a DAP_PACKET_SIZE response. Refuse the rest. */
            if ((size_t)(rp - resp) + nbytes > DAP_PACKET_SIZE) { resp[1] = DAP_ERROR; break; }
            uint8_t tmp[8] = {0};
            swd_ll_seq_in(tmp, nbits);
            memcpy(rp, tmp, nbytes); rp += nbytes;
        } else {                             /* output */
            if (p + nbytes > end) break;
            swd_ll_seq_out(p, nbits); p += nbytes;
        }
    }
    return (size_t)(rp - resp);
}

/* ---- top level ----------------------------------------------------------- */
size_t dap_process(const uint8_t *req, size_t req_len, uint8_t *resp, size_t resp_cap) {
    if (req_len == 0 || resp_cap < DAP_PACKET_SIZE) {
        resp[0] = ID_DAP_Invalid;
        return 1;
    }
    uint8_t cmd = req[0];
    resp[0] = cmd;
    s_budget = DAP_RETRY_BUDGET;

    switch (cmd) {
        case ID_DAP_Info:
            return do_info(req, req_len, resp);

        case ID_DAP_HostStatus:                  /* LED — accept and ignore */
            resp[1] = DAP_OK;
            return 2;

        case ID_DAP_Connect:                     /* req[1]=port (1=SWD,2=JTAG) */
            if (req_len > 1 && req[1] == 2) { resp[1] = 0; return 2; }  /* JTAG: fail */
            resp[1] = 1;                         /* connected as SWD */
            return 2;

        case ID_DAP_Disconnect:
            resp[1] = DAP_OK;
            return 2;

        case ID_DAP_TransferConfigure:
            if (req_len >= 6) {
                s_idle_cycles = req[1];
                s_wait_retry  = (uint16_t)(req[2] | (req[3] << 8));
                swd_ll_set_wait_retry(s_wait_retry < DAP_BATCH_WAIT_MAX ? s_wait_retry
                                                                        : (uint16_t)DAP_BATCH_WAIT_MAX);
                s_match_retry = (uint16_t)(req[4] | (req[5] << 8));
                swd_ll_set_idle(s_idle_cycles);
            }
            resp[1] = DAP_OK;
            return 2;

        case ID_DAP_Transfer:
            if (swd_ll_batch_supported() && !has_match_read(req, req_len)) rec_begin();
            return do_transfer(req, req_len, resp);

        case ID_DAP_TransferBlock:
            if (swd_ll_batch_supported()) rec_begin();
            return do_transfer_block(req, req_len, resp);

        case ID_DAP_WriteABORT: {
            uint8_t ack = SWD_ACK_OK;
            if (req_len >= 6) {
                uint32_t v = get_le32(req + 2);
                ack = xfer(DP_ABORT_WRITE, &v);
            }
            resp[1] = (ack == SWD_ACK_OK) ? DAP_OK : DAP_ERROR;
            return 2;
        }

        case ID_DAP_Delay:                       /* best-effort: no-op */
            resp[1] = DAP_OK;
            return 2;

        case ID_DAP_ResetTarget:
            resp[1] = DAP_OK;
            resp[2] = 0;                         /* no device-specific reset seq */
            return 3;

        case ID_DAP_SWJ_Pins: {                  /* req[1]=out, req[2]=select, [3..6]=wait */
            uint8_t out = (req_len > 1) ? req[1] : 0;
            uint8_t sel = (req_len > 2) ? req[2] : 0;
            if (sel & 0x80u)                     /* nRESET (active low) */
                swd_ll_nreset((out & 0x80u) == 0);
            resp[1] = out;                       /* best-effort pin readback */
            return 2;
        }

        case ID_DAP_SWJ_Clock:                   /* req[1..4] = Hz: the batch SWCLK */
            if (req_len >= 5) swd_ll_set_clock(get_le32(req + 1));
            resp[1] = DAP_OK;
            return 2;

        case ID_DAP_SWJ_Sequence: {              /* req[1]=bit count (0 => 256) */
            uint32_t nbits = (req_len > 1) ? req[1] : 0;
            if (nbits == 0) nbits = 256;
            if (req_len >= 2) swd_ll_seq_out(req + 2, nbits);
            resp[1] = DAP_OK;
            return 2;
        }

        case ID_DAP_SWD_Configure: {             /* req[1]=config */
            uint8_t cfg = (req_len > 1) ? req[1] : 0;
            swd_ll_configure((uint8_t)((cfg & 0x03u) + 1), (cfg & 0x04u) != 0);
            resp[1] = DAP_OK;
            return 2;
        }

        case ID_DAP_SWD_Sequence:
            return do_swd_sequence(req, req_len, resp);

        default:
            resp[0] = ID_DAP_Invalid;
            return 1;
    }
}
