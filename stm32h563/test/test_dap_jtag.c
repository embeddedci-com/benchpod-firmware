/* test_dap_jtag.c — host tests of the CMSIS-DAP processor's JTAG side (dap.c + dap_jtag.c),
 * against mocks/mock_jtag_ll.c: a bit-level TAP chain like the NUCLEO-F446RE's (an ARM JTAG-DP,
 * IR 4, IDCODE 0x4BA00477, nearest TDO; a boundary-scan TAP, IR 5, IDCODE 0x06413041) with a
 * MEM-AP over a small memory.  Ports and DAP_Connect, DAP_JTAG_Sequence / Configure / IDCODE,
 * DP and AP transfers with the posted-read pipeline, blocks, WAIT retries, match reads, ABORT,
 * TMS sequences and nTRST.  The same transfers also run with the DP as device 1 (behind the other
 * TAP) to check the bypass bits on both sides.
 */
#include "dap.h"
#include "mocks/mock_jtag_ll.h"
#include "mocks/mock_swd_ll.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

static uint8_t resp[DAP_PACKET_SIZE];

static size_t run(const uint8_t *req, size_t n) {
    memset(resp, 0xEE, sizeof(resp));
    return dap_process(req, n, resp, sizeof(resp));
}
static uint32_t le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }

static int  g_switches;
static bool g_switch_ok = true;
static bool sw(uint8_t port) { (void)port; g_switches++; return g_switch_ok; }

static unsigned g_dp;   /* the DP's device index: 0 (nearest TDO) or 1 */

/* the F446 chain; the DP at index g_dp; JTAG connected, chain configured */
static void setup(void) {
    mock_swd_ll_reset();
    mock_jtag_ll_reset();
    if (g_dp == 0) {
        mock_jtag_ll_add(4, 0x4BA00477u, 0xE, true);
        mock_jtag_ll_add(5, 0x06413041u, 0x1, false);
    } else {
        mock_jtag_ll_add(5, 0x06413041u, 0x1, false);
        mock_jtag_ll_add(4, 0x4BA00477u, 0xE, true);
    }
    dap_reset();
    dap_set_ports(DAP_PORTS_JTAG, DAP_PORT_JTAG, NULL);
    uint8_t con[] = { 0x02, 0x02 };
    run(con, sizeof(con));
    uint8_t cfg[] = { 0x15, 2, g_dp == 0 ? 4 : 5, g_dp == 0 ? 5 : 4 };
    run(cfg, sizeof(cfg));
    /* Test-Logic-Reset, then Run-Test/Idle: 5 ones and a zero on TMS */
    uint8_t swj[] = { 0x12, 6, 0x1F };
    run(swj, sizeof(swj));
}

/* ---- ports ---- */
static void test_ports(void) {
    mock_jtag_ll_reset(); mock_swd_ll_reset();
    dap_reset();
    uint8_t info[] = { 0x00, 0xF0 };
    run(info, sizeof(info));
    CHECK(resp[1] == 1 && resp[2] == 0x01);               /* SWD only by default */
    uint8_t cj[] = { 0x02, 0x02 };
    run(cj, sizeof(cj));
    CHECK(resp[1] == 0);                                  /* no JTAG pins: refused */
    uint8_t c0[] = { 0x02, 0x00 };
    run(c0, sizeof(c0));
    CHECK(resp[1] == 1);                                  /* default: SWD */

    dap_set_ports(DAP_PORTS_SWD | DAP_PORTS_JTAG, DAP_PORT_SWD, sw);
    run(info, sizeof(info));
    CHECK(resp[2] == 0x03);
    g_switches = 0; g_switch_ok = true;
    run(cj, sizeof(cj));
    CHECK(resp[1] == 2 && g_switches == 1 && dap_port() == DAP_PORT_JTAG);
    run(cj, sizeof(cj));
    CHECK(resp[1] == 2 && g_switches == 1);               /* already JTAG: no re-arm */
    g_switch_ok = false;
    uint8_t cs[] = { 0x02, 0x01 };
    run(cs, sizeof(cs));
    CHECK(resp[1] == 0 && dap_port() == DAP_PORT_JTAG);   /* the switch failed: stays JTAG */
    g_switch_ok = true;
    run(cs, sizeof(cs));
    CHECK(resp[1] == 1 && dap_port() == DAP_PORT_SWD);

    /* JTAG commands need the JTAG port */
    uint8_t seq[] = { 0x14, 1, 0x05, 0x00 };
    run(seq, sizeof(seq));
    CHECK(resp[1] == 0xFF);
    uint8_t id[] = { 0x16, 0 };
    run(id, sizeof(id));
    CHECK(resp[1] == 0xFF);

    /* SWJ_Clock reaches the JTAG engine on connect and while connected */
    uint8_t clk[] = { 0x11, 0x40, 0x42, 0x0F, 0x00 };   /* 1 MHz */
    run(clk, sizeof(clk));
    CHECK(mock_jtag_ll_clock_hz == 0);
    run(cj, sizeof(cj));
    CHECK(mock_jtag_ll_clock_hz == 1000000u);
}

/* ---- DAP_JTAG_Sequence: read both IDCODEs from Shift-DR ---- */
static void test_sequence_idcodes(void) {
    setup();
    /* RTI -> Select-DR (1), Capture-DR (0), Shift-DR (0); 63 bits; 1 bit to Exit1; Update, RTI */
    uint8_t req[] = { 0x14, 7,
                      0x41, 0xFF,            /* 1 cycle TMS 1 */
                      0x02, 0xFF,            /* 2 cycles TMS 0 */
                      0xBF, 0,0,0,0,0,0,0,0, /* 63 cycles, capture */
                      0xC1, 0x00,            /* 1 cycle TMS 1, capture */
                      0x41, 0xFF,            /* Update-DR */
                      0x01, 0xFF,            /* Run-Test/Idle */
                      0x00, 0,0,0,0,0,0,0,0 }; /* 64 cycles (count 0) in RTI */
    size_t n = run(req, sizeof(req));
    CHECK(n == 2 + 8 + 1);
    CHECK(resp[1] == 0x00);
    uint32_t lo = le32(resp + 2), hi = le32(resp + 6);
    hi = (hi & 0x7FFFFFFFu) | ((uint32_t)(resp[10] & 1u) << 31);
    uint32_t want_lo = g_dp == 0 ? 0x4BA00477u : 0x06413041u;
    uint32_t want_hi = g_dp == 0 ? 0x06413041u : 0x4BA00477u;
    CHECK(lo == want_lo);
    CHECK(hi == want_hi);
    CHECK(mock_jtag_ll_state(0) == 1 && mock_jtag_ll_state(1) == 1);

    /* malformed: a sequence runs past the end */
    uint8_t bad[] = { 0x14, 1, 0x10, 0x00 };   /* 16 cycles want 2 bytes, 1 given */
    run(bad, sizeof(bad));
    CHECK(resp[1] == 0xFF);
    /* the count says 2, one is there */
    uint8_t bad2[] = { 0x14, 2, 0x01, 0x00 };
    run(bad2, sizeof(bad2));
    CHECK(resp[1] == 0xFF);
    /* an engine failure is an error */
    mock_jtag_ll_fail = true;
    uint8_t ok[] = { 0x14, 1, 0x01, 0x00 };
    run(ok, sizeof(ok));
    CHECK(resp[1] == 0xFF);
}

/* ---- DAP_JTAG_Configure / DAP_JTAG_IDCODE ---- */
static void test_configure_idcode(void) {
    setup();
    /* Like ARM's reference firmware, DAP_JTAG_IDCODE loads the ARM IDCODE instruction (0xE): it
       reads the DP's IDCODE; another TAP's answer depends on what 0xE selects there. */
    uint8_t iddp[] = { 0x16, (uint8_t)g_dp }, idother[] = { 0x16, (uint8_t)(1u - g_dp) }, id2[] = { 0x16, 2 };
    size_t n = run(iddp, sizeof(iddp));
    CHECK(n == 6 && resp[1] == 0x00);
    CHECK(le32(resp + 2) == 0x4BA00477u);
    n = run(idother, sizeof(idother));
    CHECK(n == 6 && resp[1] == 0x00);
    run(id2, sizeof(id2));
    CHECK(resp[1] == 0xFF);

    uint8_t too_many[] = { 0x15, 9, 4,4,4,4,4,4,4,4,4 };
    run(too_many, sizeof(too_many));
    CHECK(resp[1] == 0xFF);
    uint8_t short_req[] = { 0x15, 3, 4, 5 };
    run(short_req, sizeof(short_req));
    CHECK(resp[1] == 0xFF);
}

/* ---- DAP_Transfer: DP + AP writes, posted reads ---- */
static void test_transfer(void) {
    setup();
    uint8_t req[64]; size_t k = 0;
    req[k++] = 0x05; req[k++] = (uint8_t)g_dp; req[k++] = 6;
    req[k++] = 0x04; put32(req + k, 0x50000000u); k += 4;   /* DP CTRL/STAT write */
    req[k++] = 0x08; put32(req + k, 0x00000000u); k += 4;   /* DP SELECT = 0 */
    req[k++] = 0x01; put32(req + k, 0x23000002u); k += 4;   /* AP CSW */
    req[k++] = 0x05; put32(req + k, 0x00000100u); k += 4;   /* AP TAR */
    req[k++] = 0x0D; put32(req + k, 0xCAFEF00Du); k += 4;   /* AP DRW */
    req[k++] = 0x0D; put32(req + k, 0x12345678u); k += 4;   /* AP DRW */
    run(req, k);
    CHECK(resp[1] == 6 && resp[2] == 0x01);
    CHECK(mock_jtag_ll_ctrl_stat == 0x50000000u);
    CHECK(mock_jtag_ll_mem[0x100 / 4] == 0xCAFEF00Du);
    CHECK(mock_jtag_ll_mem[0x104 / 4] == 0x12345678u);

    /* read back: TAR write, two DRW reads (posted), a DP read, the CSW read */
    k = 0;
    req[k++] = 0x05; req[k++] = (uint8_t)g_dp; req[k++] = 5;
    req[k++] = 0x05; put32(req + k, 0x00000100u); k += 4;   /* AP TAR */
    req[k++] = 0x0F;                                        /* AP DRW read */
    req[k++] = 0x0F;                                        /* AP DRW read */
    req[k++] = 0x06;                                        /* DP CTRL/STAT read */
    req[k++] = 0x03;                                        /* AP CSW read */
    size_t n = run(req, k);
    CHECK(resp[1] == 5 && resp[2] == 0x01);
    CHECK(n == 3 + 4 * 4);
    CHECK(le32(resp + 3)  == 0xCAFEF00Du);
    CHECK(le32(resp + 7)  == 0x12345678u);
    CHECK(le32(resp + 11) == 0x50000000u);
    CHECK(le32(resp + 15) == 0x23000002u);

    /* a device index past the chain */
    uint8_t bad[] = { 0x05, 2, 1, 0x06 };
    n = run(bad, sizeof(bad));
    CHECK(n == 3 && resp[1] == 0 && resp[2] == 0);
}

/* ---- DAP_TransferBlock ---- */
static void test_transfer_block(void) {
    setup();
    uint8_t tar[] = { 0x05, (uint8_t)g_dp, 2, 0x08, 0,0,0,0, 0x05, 0x00,0x02,0,0 };   /* SELECT 0, TAR 0x200 */
    run(tar, sizeof(tar));
    CHECK(resp[1] == 2 && resp[2] == 0x01);

    static uint8_t wr[5 + 4 * 200];
    size_t k = 0;
    wr[k++] = 0x06; wr[k++] = (uint8_t)g_dp; wr[k++] = 200; wr[k++] = 0; wr[k++] = 0x0D;
    for (uint32_t i = 0; i < 200; i++) { put32(wr + k, 0xA5000000u + i * 7u); k += 4; }
    size_t n = run(wr, k);
    CHECK(n == 4 && resp[1] == 200 && resp[2] == 0 && resp[3] == 0x01);
    CHECK(mock_jtag_ll_mem[0x200 / 4] == 0xA5000000u);
    CHECK(mock_jtag_ll_mem[0x200 / 4 + 199] == 0xA5000000u + 199u * 7u);

    run(tar, sizeof(tar));
    uint8_t rd[] = { 0x06, (uint8_t)g_dp, 200, 0, 0x0F };
    n = run(rd, sizeof(rd));
    CHECK(resp[1] == 200 && resp[3] == 0x01 && n == 4 + 800);
    bool same = true;
    for (uint32_t i = 0; i < 200; i++) if (le32(resp + 4 + 4 * i) != 0xA5000000u + i * 7u) same = false;
    CHECK(same);
    /* exactly 200 AP reads: the last word came through RDBUFF, not one more DRW read */
    uint8_t rtar[] = { 0x05, (uint8_t)g_dp, 1, 0x07 };
    n = run(rtar, sizeof(rtar));
    CHECK(n == 7 && resp[1] == 1 && le32(resp + 3) == 0x200u + 800u);

    /* count 0 and a bad index */
    uint8_t zero[] = { 0x06, (uint8_t)g_dp, 0, 0, 0x0F };
    n = run(zero, sizeof(zero));
    CHECK(n == 4 && resp[1] == 0 && resp[3] == 0x01);
    uint8_t badix[] = { 0x06, 5, 1, 0, 0x0F };
    n = run(badix, sizeof(badix));
    CHECK(n == 4 && resp[1] == 0 && resp[3] == 0);
}

/* ---- WAIT: retried (the DP ignored the scan), then given up ---- */
static void test_wait(void) {
    setup();
    uint8_t cfg[] = { 0x04, 0, 5, 0, 0, 0 };   /* TransferConfigure: idle 0, 5 WAIT retries */
    run(cfg, sizeof(cfg));
    uint8_t wr[] = { 0x05, (uint8_t)g_dp, 3,
                     0x08, 0,0,0,0,                 /* SELECT 0 */
                     0x05, 0x40,0,0,0,              /* TAR 0x40 */
                     0x0D, 0x11,0x22,0x33,0x44 };   /* DRW */
    mock_jtag_ll_wait = 3;
    run(wr, sizeof(wr));
    CHECK(resp[1] == 3 && resp[2] == 0x01);
    CHECK(mock_jtag_ll_mem[0x40 / 4] == 0x44332211u);
    CHECK(mock_jtag_ll_mem[0x44 / 4] == 0);      /* written once: the WAITed scans were ignored */

    mock_jtag_ll_wait = 100;                       /* more than the retries */
    uint8_t rd[] = { 0x05, (uint8_t)g_dp, 1, 0x06 };
    run(rd, sizeof(rd));
    CHECK(resp[1] == 0 && resp[2] == 0x02);
}

/* ---- match-value read, ABORT, TMS sequence, nTRST ---- */
static void test_misc(void) {
    setup();
    uint8_t set[] = { 0x05, (uint8_t)g_dp, 2,
                      0x20, 0x00,0x00,0x00,0xF0,    /* match mask 0xF0000000 */
                      0x04, 0x00,0x00,0x00,0xF0 };  /* CTRL/STAT = 0xF0000000 */
    run(set, sizeof(set));
    CHECK(resp[1] == 2 && resp[2] == 0x01);
    uint8_t match[] = { 0x05, (uint8_t)g_dp, 1, 0x16, 0x00,0x00,0x00,0xF0 };
    size_t n = run(match, sizeof(match));
    CHECK(n == 3 && resp[1] == 1 && resp[2] == 0x01);
    uint8_t nomatch[] = { 0x05, (uint8_t)g_dp, 1, 0x16, 0x00,0x00,0x00,0x30 };
    run(nomatch, sizeof(nomatch));
    CHECK(resp[1] == 0 && resp[2] == (0x01 | 0x10));

    uint8_t ab[] = { 0x08, (uint8_t)g_dp, 0x1E, 0, 0, 0 };
    run(ab, sizeof(ab));
    CHECK(resp[1] == 0x00 && mock_jtag_ll_aborts == 1 && mock_jtag_ll_abort == 0x1Eu);

    uint8_t tlr[] = { 0x12, 5, 0x1F };
    run(tlr, sizeof(tlr));
    CHECK(resp[1] == 0x00 && mock_jtag_ll_state(0) == 0 && mock_jtag_ll_state(1) == 0);

    uint8_t trst[] = { 0x10, 0x00, 0x20, 0,0,0,0 };
    run(trst, sizeof(trst));
    CHECK(mock_jtag_ll_trst_calls == 1 && mock_jtag_ll_trst);
    uint8_t untrst[] = { 0x10, 0x20, 0x20, 0,0,0,0 };
    run(untrst, sizeof(untrst));
    CHECK(mock_jtag_ll_trst_calls == 2 && !mock_jtag_ll_trst);

    /* SWD_Sequence is refused in JTAG mode */
    uint8_t swdseq[] = { 0x1D, 1, 0x08, 0xFF };
    run(swdseq, sizeof(swdseq));
    CHECK(resp[1] == 0xFF);
}

int main(void) {
    test_ports();
    for (g_dp = 0; g_dp < 2; g_dp++) {
        test_sequence_idcodes();
        test_configure_idcode();
        test_transfer();
        test_transfer_block();
        test_wait();
        test_misc();
    }
    if (failures) { printf("test_dap_jtag: %d failure(s)\n", failures); return 1; }
    printf("test_dap_jtag: all passed\n");
    return 0;
}
