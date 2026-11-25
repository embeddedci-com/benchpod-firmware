/*
 * dap_id.c — target ID reads for `dap_start` "op":"id" (see dap_id.h).
 *
 * Runs on the hw worker task, on an engine command_handler_dap.c has just armed.
 */
#include "dap_id.h"
#include "swd_ll.h"
#include "jtag_ll.h"
#include "watchdog.h"
#include "pico_compat.h"     /* sleep_ms (yields to FreeRTOS) */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/time.h"

/* DP registers by their A[3:2] request bits; SELECT's DPBANKSEL picks the register behind A2. */
#define DP_ABORT      0u
#define DP_CTRLSTAT   SWD_REQ_A2
#define DP_SELECT     SWD_REQ_A3
#define DP_RDBUFF     (SWD_REQ_A2 | SWD_REQ_A3)
#define DP_TARGETID_BANK 2u

#define ABORT_CLEAR_ALL  0x1Eu         /* STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR */
#define CTRLSTAT_PWRUP   0x50000000u   /* CSYSPWRUPREQ | CDBGPWRUPREQ */
#define CTRLSTAT_PWRACK  0xA0000000u   /* CSYSPWRUPACK | CDBGPWRUPACK */
#define CTRLSTAT_STICKY  0x000000A2u   /* STICKYERR | WDATAERR | STICKYORUN */
#define AP_CSW           0u
#define AP_TAR           SWD_REQ_A2
#define AP_DRW           (SWD_REQ_A2 | SWD_REQ_A3)
#define AP_IDR_BANK      0x000000F0u   /* SELECT: APSEL 0, APBANKSEL 0xF; the IDR is register 0xC */
#define CSW_32BIT_NOINC  0xA2000002u   /* DbgSwEnable, HPROT, 32-bit, no auto-increment */
#define IDR_CLASS_MEMAP  8u            /* IDR bits 16:13 */
#define CPUID_ADDR       0xE000ED00u

/* WAIT means the target's bus is busy: an MCU sleeping in WFI between interrupts stalls AHB-AP
   accesses to its system bus (DBGMCU, flash) until the next wake-up, while the core's own SCS
   (CPUID) still answers.  64 back-to-back retries are a few microseconds, far shorter than a 1 ms
   SysTick, so retry for up to ID_WAIT_MS instead (a busy loop: short, and the worker keeps its
   heartbeat through the callers' sleeps). */
#define ID_WAIT_MS 20

/* swd_ll_transfer clocks a read's whole data window in one feed (see swd_ll.c), so a read that
   gets WAIT leaves 33 stray clocks on the line and the DP out of step: the next transfer gets no
   ACK.  A line reset and a DPIDR read put it back in step (SELECT survives a line reset), and the
   read is then sent again.  The DAP session avoids this through the gateware's SWD queue; this
   path talks to swd_ll directly. */
static void resync(void) {
    static const uint8_t line_reset[7] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };  /* 56 ones */
    static const uint8_t idle[1] = { 0x00 };
    uint32_t dpidr;
    swd_ll_seq_out(line_reset, 56);
    swd_ll_seq_out(idle, 8);
    (void)swd_ll_transfer(SWD_REQ_RnW /* DPIDR */, &dpidr);
}

static uint8_t xfer(uint8_t req, uint32_t *data) {
    absolute_time_t deadline = make_timeout_time_ms(ID_WAIT_MS);
    uint8_t ack;
    for (;;) {
        ack = swd_ll_transfer(req, data);
        if (ack != SWD_ACK_WAIT) return ack;
        if (req & SWD_REQ_RnW) resync();
        if (time_reached(deadline)) return ack;
    }
}

/* ACK OK only counts when the read's parity was good too (OK|PERR is not SWD_ACK_OK). */
static bool dp_wr(uint8_t a, uint32_t v) { return xfer(a, &v) == SWD_ACK_OK; }
static bool dp_rd(uint8_t a, uint32_t *v) { return xfer(SWD_REQ_RnW | a, v) == SWD_ACK_OK; }
static bool ap_wr(uint8_t a, uint32_t v) { return xfer(SWD_REQ_APnDP | a, &v) == SWD_ACK_OK; }
/* An AP read is posted: the transfer returns the previous value, RDBUFF the one asked for. */
static bool ap_rd(uint8_t a, uint32_t *v) {
    uint32_t stale;
    return xfer(SWD_REQ_APnDP | SWD_REQ_RnW | a, &stale) == SWD_ACK_OK && dp_rd(DP_RDBUFF, v);
}

/* A word of target memory through the MEM-AP (CSW already set, SELECT on bank 0).  A bus error
   on the target's side comes back as an OK ack with STICKYERR set, so look at CTRL/STAT too;
   either way the sticky flags are cleared before the next read. */
static bool mem_rd(uint32_t addr, uint32_t *v) {
    uint32_t cs = 0;
    bool ok = ap_wr(AP_TAR, addr) && ap_rd(AP_DRW, v) && dp_rd(DP_CTRLSTAT, &cs) &&
              !(cs & CTRLSTAT_STICKY);
    if (!ok) dp_wr(DP_ABORT, ABORT_CLEAR_ALL);
    return ok;
}

/* Line reset, JTAG-to-SWD, line reset, idle (dap_wait_for_target's sequence), then DPIDR. */
static bool read_dpidr(uint32_t wait_ms, uint32_t *dpidr) {
    static const uint8_t line_reset[7] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };  /* 56 ones */
    static const uint8_t jtag_to_swd[2] = { 0x9E, 0xE7 };
    static const uint8_t idle[1] = { 0x00 };
    absolute_time_t deadline = make_timeout_time_ms(wait_ms);
    for (;;) {
        swd_ll_seq_out(line_reset, 56);
        swd_ll_seq_out(jtag_to_swd, 16);
        swd_ll_seq_out(line_reset, 56);
        swd_ll_seq_out(idle, 8);
        /* A DPIDR of 0 is a floating or stuck line that happened to parse, not a DP. */
        if (dp_rd(0 /* DPIDR: A[3:2] = 0 */, dpidr) && *dpidr != 0) return true;
        if (time_reached(deadline)) return false;
        watchdog_heartbeat(WD_TASK_WORKER, "dap-id");
        sleep_ms(20);
    }
}

void dap_id_swd(uint32_t wait_ms, const uint32_t *addr, size_t n, dap_id_swd_t *r) {
    memset(r, 0, sizeof(*r));
    if (n > DAP_ID_MAX_READS) n = DAP_ID_MAX_READS;
    r->n_reads = n;
    for (size_t i = 0; i < n; i++) r->addr[i] = addr[i];

    swd_ll_reset();   /* turnaround 1, no data phase: whatever the last DAP session left */
    if (!read_dpidr(wait_ms, &r->dpidr)) return;
    r->found = true;
    dp_wr(DP_ABORT, ABORT_CLEAR_ALL);   /* a sticky error from an earlier session fails every access */

    if (((r->dpidr >> 12) & 0xFu) >= 2u) {   /* DP version 2: TARGETID is DP bank 2, register 4 */
        r->has_targetid = dp_wr(DP_SELECT, DP_TARGETID_BANK) && dp_rd(DP_CTRLSTAT, &r->targetid);
        dp_wr(DP_SELECT, 0);
    }

    /* The AP needs the debug domain powered. */
    bool powered = false;
    if (dp_wr(DP_CTRLSTAT, CTRLSTAT_PWRUP)) {
        for (int i = 0; i < 20 && !powered; i++) {
            uint32_t cs;
            if (dp_rd(DP_CTRLSTAT, &cs) && (cs & CTRLSTAT_PWRACK) == CTRLSTAT_PWRACK) powered = true;
            else sleep_ms(5);
        }
    }
    if (powered) {
        r->has_apidr = dp_wr(DP_SELECT, AP_IDR_BANK) && ap_rd(SWD_REQ_A2 | SWD_REQ_A3, &r->apidr);
        dp_wr(DP_SELECT, 0);
        if (r->has_apidr && ((r->apidr >> 13) & 0xFu) == IDR_CLASS_MEMAP &&
            ap_wr(AP_CSW, CSW_32BIT_NOINC)) {
            r->has_cpuid = mem_rd(CPUID_ADDR, &r->cpuid);
            for (size_t i = 0; i < n; i++)
                r->value_ok[i] = mem_rd(r->addr[i], &r->value[i]);
        }
    }
    dp_wr(DP_CTRLSTAT, 0);   /* give the power-up request back: the target keeps no debug state */
}

/* ---- JTAG ---- */

size_t dap_id_jtag_parse(const uint8_t *tdo, size_t nbits, uint32_t *id, size_t cap) {
    size_t n = 0, pos = 0, zeros = 0;
    while (pos < nbits && n < cap) {
        if (!((tdo[pos / 8] >> (pos % 8)) & 1u)) {   /* BYPASS device: one 0 bit */
            if (++zeros >= 32) break;
            pos++;
            continue;
        }
        zeros = 0;
        if (pos + 32 > nbits) break;
        uint32_t v = 0;
        for (unsigned b = 0; b < 32; b++) v |= (uint32_t)((tdo[(pos + b) / 8] >> ((pos + b) % 8)) & 1u) << b;
        if (v == 0xFFFFFFFFu) break;   /* TDI = 1 coming back out: the chain is over */
        id[n++] = v;
        pos += 32;
    }
    return n;
}

size_t dap_id_jtag(uint32_t wait_ms, uint32_t *id, size_t cap) {
    /* DAP_JTAG_Sequence stream (jtag_ll.h): Test-Logic-Reset, Run-Test/Idle, Select-DR,
       Capture-DR + Shift-DR, 4 x 64 captured clocks with TDI = 1 (the TAPs' IDCODE register is
       what Test-Logic-Reset selects), then back to Test-Logic-Reset and Run-Test/Idle. */
    /* An SWJ-DP (every Cortex-M with JTAG) that an SWD session switched over, our own SWD id
       read included, stays in SWD until power-on reset, so lead with the SWD-to-JTAG switch:
       56 TMS 1, then 0xE73C on TMS (LSB first, as runs of equal bits).  On a JTAG-only chain it
       walks the TAPs through Run-Test/Idle and Select-IR back to Test-Logic-Reset, no Update. */
    static const uint8_t head[] = {
        0x78, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,       /* 56 x TMS 1 */
        0x02, 0xFF, 0x44, 0xFF, 0x02, 0xFF,                   /* 0x3C: 0 0 1 1 1 1 0 0 */
        0x43, 0xFF, 0x02, 0xFF, 0x43, 0xFF,                   /* 0xE7: 1 1 1 0 0 1 1 1 */
        0x45, 0xFF, 0x01, 0x00, 0x41, 0xFF, 0x02, 0x00 };
    static const uint8_t tail[] = { 0x45, 0xFF, 0x01, 0x00 };
    static const uint8_t scan[9] = { 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    uint8_t seq[sizeof(head) + 4 * sizeof(scan) + sizeof(tail)];
    uint8_t *p = seq;
    memcpy(p, head, sizeof(head)); p += sizeof(head);
    for (int i = 0; i < 4; i++) { memcpy(p, scan, sizeof(scan)); p += sizeof(scan); }
    memcpy(p, tail, sizeof(tail));

    if (cap > DAP_ID_MAX_IDCODES) cap = DAP_ID_MAX_IDCODES;
    absolute_time_t deadline = make_timeout_time_ms(wait_ms);
    for (;;) {
        uint8_t tdo[32] = { 0 };
        if (jtag_ll_run(seq, sizeof(seq), tdo, sizeof(tdo)) == 0) {
            size_t n = dap_id_jtag_parse(tdo, sizeof(tdo) * 8, id, cap);
            if (n) return n;
        }
        if (time_reached(deadline)) return 0;
        watchdog_heartbeat(WD_TASK_WORKER, "dap-id");
        sleep_ms(20);
    }
}

/* ---- request / reply ---- */

int dap_id_parse_reads(const char *json, uint32_t *addr, size_t cap) {
    const char *p = strstr(json, "\"read\"");
    if (!p) return 0;
    p = strchr(p + 6, '[');
    if (!p) return -1;
    size_t n = 0;
    for (p++;;) {
        while (*p == ' ' || *p == ',') p++;
        if (*p == ']') return (int)n;
        if (*p == '"') p++;                      /* the addresses come as "0x..." strings */
        char *end;
        unsigned long v = strtoul(p, &end, 0);
        if (end == p || n >= cap) return -1;
        addr[n++] = (uint32_t)v;
        p = end;
        if (*p == '"') p++;
    }
}

static size_t put_hex_or_null(char *out, size_t cap, const char *key, bool ok, uint32_t v) {
    int n = ok ? snprintf(out, cap, "\"%s\":\"0x%08x\"", key, (unsigned)v)
               : snprintf(out, cap, "\"%s\":null", key);
    return (n < 0 || (size_t)n >= cap) ? 0 : (size_t)n;
}

size_t dap_id_fmt_swd(char *out, size_t cap, const dap_id_swd_t *r) {
    if (!r->found) {
        int n = snprintf(out, cap, "{\"port\":\"swd\",\"found\":false}");
        return (n < 0 || (size_t)n >= cap) ? 0 : (size_t)n;
    }
    size_t o = 0;
    int n = snprintf(out, cap, "{\"port\":\"swd\",\"found\":true,\"dpidr\":\"0x%08x\",", (unsigned)r->dpidr);
    if (n < 0 || (size_t)n >= cap) return 0;
    o = (size_t)n;
    struct { const char *key; bool ok; uint32_t v; } f[3] = {
        { "targetid", r->has_targetid, r->targetid }, { "apidr", r->has_apidr, r->apidr },
        { "cpuid", r->has_cpuid, r->cpuid } };
    for (int i = 0; i < 3; i++) {
        size_t k = put_hex_or_null(out + o, cap - o, f[i].key, f[i].ok, f[i].v);
        if (!k || o + k + 1 >= cap) return 0;
        o += k;
        out[o++] = ',';
    }
    n = snprintf(out + o, cap - o, "\"reads\":[");
    if (n < 0 || (size_t)n >= cap - o) return 0;
    o += (size_t)n;
    for (size_t i = 0; i < r->n_reads; i++) {
        n = r->value_ok[i]
            ? snprintf(out + o, cap - o, "%s{\"addr\":\"0x%08x\",\"value\":\"0x%08x\"}", i ? "," : "",
                       (unsigned)r->addr[i], (unsigned)r->value[i])
            : snprintf(out + o, cap - o, "%s{\"addr\":\"0x%08x\",\"value\":null}", i ? "," : "",
                       (unsigned)r->addr[i]);
        if (n < 0 || (size_t)n >= cap - o) return 0;
        o += (size_t)n;
    }
    n = snprintf(out + o, cap - o, "]}");
    if (n < 0 || (size_t)n >= cap - o) return 0;
    return o + (size_t)n;
}

size_t dap_id_fmt_jtag(char *out, size_t cap, const uint32_t *id, size_t n) {
    if (n == 0) {
        int k = snprintf(out, cap, "{\"port\":\"jtag\",\"found\":false}");
        return (k < 0 || (size_t)k >= cap) ? 0 : (size_t)k;
    }
    int k = snprintf(out, cap, "{\"port\":\"jtag\",\"found\":true,\"idcodes\":[");
    if (k < 0 || (size_t)k >= cap) return 0;
    size_t o = (size_t)k;
    for (size_t i = 0; i < n; i++) {
        k = snprintf(out + o, cap - o, "%s\"0x%08x\"", i ? "," : "", (unsigned)id[i]);
        if (k < 0 || (size_t)k >= cap - o) return 0;
        o += (size_t)k;
    }
    k = snprintf(out + o, cap - o, "]}");
    if (k < 0 || (size_t)k >= cap - o) return 0;
    return o + (size_t)k;
}
