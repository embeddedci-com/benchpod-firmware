#include "mock_jtag_ll.h"
#include "jtag_ll.h"
#include <string.h>

int      mock_jtag_ll_wait;
bool     mock_jtag_ll_fail;
uint32_t mock_jtag_ll_mem[MOCK_JTAG_MEM_WORDS];
uint32_t mock_jtag_ll_ctrl_stat;
uint32_t mock_jtag_ll_abort;
int      mock_jtag_ll_aborts;
int      mock_jtag_ll_runs;
long     mock_jtag_ll_tcks;
int      mock_jtag_ll_dp_execs;
bool     mock_jtag_ll_trst;
int      mock_jtag_ll_trst_calls;
uint32_t mock_jtag_ll_clock_hz;

enum { TLR, RTI, SDR, CDR, SHDR, E1DR, PDR, E2DR, UDR, SIR, CIR, SHIR, E1IR, PIR, E2IR, UIR };

typedef struct {
    unsigned irlen;
    uint32_t idcode, idcode_ir;
    bool     dp;
    int      st;
    uint64_t ir, irsh;
    uint64_t dr;         /* the selected data register while shifting */
    unsigned drlen;
    bool     waited;     /* this DPACC/APACC scan captured WAIT: ignore its update */
} tap_t;

static tap_t    s_tap[MOCK_JTAG_DEV_MAX];
static unsigned s_n;
/* the DP / MEM-AP */
static uint32_t s_select, s_result, s_csw, s_tar;

void mock_jtag_ll_reset(void) {
    memset(s_tap, 0, sizeof(s_tap));
    s_n = 0;
    mock_jtag_ll_wait = 0;
    mock_jtag_ll_fail = false;
    memset(mock_jtag_ll_mem, 0, sizeof(mock_jtag_ll_mem));
    mock_jtag_ll_ctrl_stat = 0;
    mock_jtag_ll_abort = 0;
    mock_jtag_ll_aborts = 0;
    mock_jtag_ll_runs = 0;
    mock_jtag_ll_tcks = 0;
    mock_jtag_ll_dp_execs = 0;
    mock_jtag_ll_trst = false;
    mock_jtag_ll_trst_calls = 0;
    mock_jtag_ll_clock_hz = 0;
    s_select = s_result = s_csw = s_tar = 0;
}

void mock_jtag_ll_add(unsigned irlen, uint32_t idcode, uint32_t idcode_ir, bool dp) {
    tap_t *t = &s_tap[s_n++];
    t->irlen = irlen; t->idcode = idcode; t->idcode_ir = idcode_ir; t->dp = dp;
    t->st = TLR; t->ir = idcode_ir;
}

int mock_jtag_ll_state(unsigned dev) { return s_tap[dev].st; }

static bool is_acc(const tap_t *t) { return t->dp && (t->ir == 0xA || t->ir == 0xB); }

static void capture_dr(tap_t *t) {
    if (t->ir == t->idcode_ir) { t->dr = t->idcode; t->drlen = 32; return; }
    if (t->dp && t->ir == 0x8) { t->dr = 0; t->drlen = 35; return; }          /* ABORT */
    if (is_acc(t)) {
        t->drlen = 35;
        if (mock_jtag_ll_wait > 0) { mock_jtag_ll_wait--; t->waited = true; t->dr = ((uint64_t)s_result << 3) | 1u; }
        else                       { t->waited = false; t->dr = ((uint64_t)s_result << 3) | 2u; }
        return;
    }
    t->dr = 0; t->drlen = 1;                                                     /* BYPASS */
}

static uint32_t ap_read(unsigned a) {
    unsigned bank = (s_select >> 4) & 0xFu;
    unsigned reg  = (bank << 4) | a;
    switch (reg) {
    case 0x00: return s_csw;
    case 0x04: return s_tar;
    case 0x0C: { uint32_t v = mock_jtag_ll_mem[(s_tar / 4u) % MOCK_JTAG_MEM_WORDS]; s_tar += 4; return v; }
    case 0xFC: return 0x24770011u;                                               /* an AHB-AP IDR */
    default:   return 0;
    }
}

static void ap_write(unsigned a, uint32_t v) {
    unsigned bank = (s_select >> 4) & 0xFu;
    unsigned reg  = (bank << 4) | a;
    switch (reg) {
    case 0x00: s_csw = v; break;
    case 0x04: s_tar = v; break;
    case 0x0C: mock_jtag_ll_mem[(s_tar / 4u) % MOCK_JTAG_MEM_WORDS] = v; s_tar += 4; break;
    default: break;
    }
}

static void update_dr(tap_t *t) {
    if (t->dp && t->ir == 0x8) { mock_jtag_ll_abort = (uint32_t)(t->dr >> 3); mock_jtag_ll_aborts++; return; }
    if (!is_acc(t) || t->waited) return;
    bool     rnw  = t->dr & 1u;
    unsigned a    = (unsigned)((t->dr >> 1) & 3u) << 2;
    uint32_t data = (uint32_t)(t->dr >> 3);
    mock_jtag_ll_dp_execs++;
    if (t->ir == 0xB) {                                                          /* APACC */
        if (rnw) s_result = ap_read(a);
        else     { ap_write(a, data); s_result = 0; }
    } else {                                                                     /* DPACC */
        if (rnw) {
            s_result = (a == 0x4) ? mock_jtag_ll_ctrl_stat : 0;                  /* RDBUFF and 0x0 read 0 */
        } else {
            if (a == 0x4) mock_jtag_ll_ctrl_stat = data;
            if (a == 0x8) s_select = data;
            s_result = 0;
        }
    }
}

/* One TCK cycle; returns TDO as it was before the rising edge. */
static int clock_bit(int tms, int tdi) {
    int out[MOCK_JTAG_DEV_MAX];
    for (unsigned i = 0; i < s_n; i++) {
        tap_t *t = &s_tap[i];
        out[i] = (t->st == SHDR) ? (int)(t->dr & 1u) : (t->st == SHIR) ? (int)(t->irsh & 1u) : 0;
    }
    int tdo = s_n ? out[0] : tdi;
    for (unsigned i = 0; i < s_n; i++) {
        tap_t *t = &s_tap[i];
        int in = (i + 1 < s_n) ? out[i + 1] : tdi;     /* device s_n-1 is nearest TDI */
        if (t->st == SHDR) t->dr   = (t->dr >> 1)   | ((uint64_t)(in & 1) << (t->drlen - 1));
        if (t->st == SHIR) t->irsh = (t->irsh >> 1) | ((uint64_t)(in & 1) << (t->irlen - 1));
        switch (t->st) {
        case TLR:  t->st = tms ? TLR : RTI; t->ir = t->idcode_ir; break;
        case RTI:  t->st = tms ? SDR : RTI; break;
        case SDR:  t->st = tms ? SIR : CDR; break;
        case CDR:  capture_dr(t); t->st = tms ? E1DR : SHDR; break;
        case SHDR: t->st = tms ? E1DR : SHDR; break;
        case E1DR: t->st = tms ? UDR : PDR; break;
        case PDR:  t->st = tms ? E2DR : PDR; break;
        case E2DR: t->st = tms ? UDR : SHDR; break;
        case UDR:  update_dr(t); t->st = tms ? SDR : RTI; break;
        case SIR:  t->st = tms ? TLR : CIR; break;
        case CIR:  t->irsh = 1u; t->st = tms ? E1IR : SHIR; break;
        case SHIR: t->st = tms ? E1IR : SHIR; break;
        case E1IR: t->st = tms ? UIR : PIR; break;
        case PIR:  t->st = tms ? E2IR : PIR; break;
        case E2IR: t->st = tms ? UIR : SHIR; break;
        case UIR:  t->ir = t->irsh; t->st = tms ? SDR : RTI; break;
        }
    }
    mock_jtag_ll_tcks++;
    return tdo;
}

long jtag_ll_tdo_len(const uint8_t *seq, size_t len) {
    size_t i = 0; long out = 0;
    while (i < len) {
        unsigned n = (seq[i] & 0x3Fu) ? (seq[i] & 0x3Fu) : 64u, nb = (n + 7u) / 8u;
        if (i + 1u + nb > len) return -1;
        if (seq[i] & 0x80u) out += (long)nb;
        i += 1u + nb;
    }
    return out;
}

int jtag_ll_run(const uint8_t *seq, size_t len, uint8_t *tdo, size_t tdo_cap) {
    mock_jtag_ll_runs++;
    long total = jtag_ll_tdo_len(seq, len);
    if (total < 0 || (size_t)total > tdo_cap) return -1;
    if (mock_jtag_ll_fail) return -2;
    size_t i = 0, o = 0;
    while (i < len) {
        uint8_t  info = seq[i];
        unsigned n = (info & 0x3Fu) ? (info & 0x3Fu) : 64u, nb = (n + 7u) / 8u;
        const uint8_t *d = seq + i + 1;
        if (info & 0x80u) memset(tdo + o, 0, nb);
        for (unsigned b = 0; b < n; b++) {
            int bit = clock_bit((info >> 6) & 1, (d[b / 8u] >> (b % 8u)) & 1);
            if ((info & 0x80u) && bit) tdo[o + b / 8u] |= (uint8_t)(1u << (b % 8u));
        }
        if (info & 0x80u) o += nb;
        i += 1u + nb;
    }
    return 0;
}

void jtag_ll_set_clock(uint32_t hz) { mock_jtag_ll_clock_hz = hz; }
void jtag_ll_set_trst_channel(unsigned ch) { (void)ch; }
void jtag_ll_trst(bool asserted) { mock_jtag_ll_trst = asserted; mock_jtag_ll_trst_calls++; }
