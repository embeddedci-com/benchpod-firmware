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
static uint32_t s_match_mask;

void dap_reset(void) {
    s_idle_cycles = 0;
    s_wait_retry  = 100;
    s_match_retry = 0;
    s_match_mask  = 0;
    swd_ll_reset();
    swd_ll_set_idle(0);
}

/* ---- little-endian helpers ----------------------------------------------- */
static uint32_t get_le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void put_le32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v); p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

/* One transfer with WAIT-retry (DAP_TransferConfigure wait-retry count). */
static uint8_t xfer(uint8_t request, uint32_t *data) {
    uint16_t retry = s_wait_retry;
    uint8_t  ack;
    do { ack = swd_ll_transfer(request, data); }
    while (ack == SWD_ACK_WAIT && retry--);
    return ack;
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
                put_le32(rp, data); rp += 4;
            }
            if (request & XFER_MATCH_VALUE) {
                if (p + 4 > end) break;
                uint32_t match_value = get_le32(p); p += 4;
                if (request & SWD_REQ_APnDP) {
                    ack = xfer(request, NULL);       /* post AP read */
                    if (ack != SWD_ACK_OK) break;
                }
                uint16_t retry = s_match_retry;
                do {
                    ack = xfer((request & SWD_REQ_APnDP) ? DP_RDBUFF_READ : request, &data);
                    if (ack != SWD_ACK_OK) break;
                } while ((data & s_match_mask) != match_value && retry--);
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
                put_le32(rp, data); rp += 4;
            }
        } else {                                     /* ---- write ---- */
            if (post_read) {
                ack = xfer(DP_RDBUFF_READ, &data);
                if (ack != SWD_ACK_OK) break;
                put_le32(rp, data); rp += 4;
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
    if (post_read && ack == SWD_ACK_OK) {
        ack = xfer(DP_RDBUFF_READ, &data);
        if (ack == SWD_ACK_OK) { put_le32(rp, data); rp += 4; }
    }

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
        if (request & SWD_REQ_APnDP) {
            ack = xfer(request, NULL);                    /* prime posted read */
            if (ack != SWD_ACK_OK) goto done;
        }
        while (count > 0) {
            uint8_t rq = request;
            if ((request & SWD_REQ_APnDP) && count == 1)
                rq = DP_RDBUFF_READ;                      /* last: flush via RDBUFF */
            ack = xfer(rq, &data);
            if (ack != SWD_ACK_OK) goto done;
            put_le32(rp, data); rp += 4; done++;
            count--;
        }
    } else {                                              /* block write */
        while (count > 0) {
            if (p + 4 > end) break;
            uint32_t value = get_le32(p); p += 4;
            ack = xfer(request, &value);
            if (ack != SWD_ACK_OK) goto done;
            done++;
            count--;
        }
    }
done:
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
            resp[1] = 1; resp[2] = 1;
            return 3;
        case INFO_PACKET_SIZE:
            resp[1] = 2;
            resp[2] = (uint8_t)(DAP_PACKET_SIZE & 0xFF);
            resp[3] = (uint8_t)(DAP_PACKET_SIZE >> 8);
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
                s_match_retry = (uint16_t)(req[4] | (req[5] << 8));
                swd_ll_set_idle(s_idle_cycles);
            }
            resp[1] = DAP_OK;
            return 2;

        case ID_DAP_Transfer:
            return do_transfer(req, req_len, resp);

        case ID_DAP_TransferBlock:
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

        case ID_DAP_SWJ_Clock:                   /* rate is SPI-bound; accept */
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
