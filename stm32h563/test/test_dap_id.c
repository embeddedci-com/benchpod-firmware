/*
 * test_dap_id.c — host tests for dap_start "op":"id" (src/dap_id.c): the SWD ID read against a
 * model of an ADIv5 DP + MEM-AP (posted AP reads, SELECT banks, CTRL/STAT power-up handshake,
 * a bus error that sets STICKYERR), the JTAG chain parser, the "read" list parser and the reply
 * formatting.
 */
#include <stdio.h>
#include <string.h>

#include "dap_id.h"
#include "swd_ll.h"
#include "jtag_ll.h"
#include "watchdog.h"
#include "pico_compat.h"

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static uint64_t g_now_us;
uint64_t time_us_64(void) { return g_now_us; }
void sleep_ms(uint32_t ms) { g_now_us += (uint64_t)ms * 1000u; }
void watchdog_heartbeat(int task, const char *name) { (void)task; (void)name; }

/* ---- a DP + one AP behind swd_ll ------------------------------------------------------- */
static struct {
    bool     present;          /* answers at all (else NO_ACK) */
    int      up_after_ms;      /* DPIDR answers only from this time on (a booting target) */
    uint32_t dpidr, targetid, apidr;
    bool     memap;
    uint32_t select, ctrlstat, csw, tar, posted;
    int      waits;            /* the next N transfers answer WAIT */
    uint32_t mem_ok[4], mem_val[4];   /* readable words; any other address is a bus error */
    int      nmem;
    bool     fault_dpidr_ack;
    /* observed */
    uint32_t csw_written, ctrlstat_final;
    int      writes_to_mem;    /* DRW writes: must stay 0 */
    int      aborts;
    int      ap_read_waits;    /* the next N AP reads answer WAIT, like a sleeping bus */
    bool     desynced;         /* a read got WAIT: swd_ll's stray data clocks, see swd_ll.c */
} m;
static void model_reset(void) {
    memset(&m, 0, sizeof(m));
    m.present = true;
    m.dpidr = 0x2ba01477; m.apidr = 0x24770011; m.memap = true;
    m.mem_ok[0] = 0xE000ED00; m.mem_val[0] = 0x410fc241;
    m.mem_ok[1] = 0xE0042000; m.mem_val[1] = 0x10006421;
    m.nmem = 2;
}
void swd_ll_reset(void) {}
void swd_ll_seq_out(const uint8_t *d, uint32_t nbits) {
    (void)d;
    if (nbits >= 50) m.desynced = false;   /* a line reset */
}
static bool mem_find(uint32_t a, uint32_t *v) {
    for (int i = 0; i < m.nmem; i++) if (m.mem_ok[i] == a) { *v = m.mem_val[i]; return true; }
    return false;
}
uint8_t swd_ll_transfer(uint8_t req, uint32_t *data) {
    g_now_us += 10;   /* a transfer takes time: the WAIT retry is bounded by the clock */
    if (!m.present || (uint64_t)m.up_after_ms * 1000u > g_now_us) return SWD_ACK_NO_ACK;
    if (m.desynced) return SWD_ACK_NO_ACK;
    if (m.waits > 0) { m.waits--; return SWD_ACK_WAIT; }
    bool ap = req & SWD_REQ_APnDP, rd = req & SWD_REQ_RnW;
    if (ap && rd && m.ap_read_waits > 0) { m.ap_read_waits--; m.desynced = true; return SWD_ACK_WAIT; }
    unsigned a = req & (SWD_REQ_A2 | SWD_REQ_A3);
    if (!ap) {
        if (rd) {
            switch (a) {
            case 0: *data = m.dpidr; if (m.fault_dpidr_ack) return SWD_ACK_FAULT; return SWD_ACK_OK;
            case SWD_REQ_A2: *data = (m.select & 0xF) == 2 ? m.targetid : m.ctrlstat; return SWD_ACK_OK;
            default: *data = m.posted; return SWD_ACK_OK;   /* RDBUFF */
            }
        }
        switch (a) {
        case 0: m.aborts++; m.ctrlstat &= ~0xA2u; return SWD_ACK_OK;
        case SWD_REQ_A2:
            m.ctrlstat = (m.ctrlstat & 0xA2u) | (*data & 0x50000000u);
            m.ctrlstat |= (*data & 0x10000000u) << 1;   /* CDBGPWRUPACK */
            m.ctrlstat |= (*data & 0x40000000u) << 1;   /* CSYSPWRUPACK */
            m.ctrlstat &= ~(((~*data) & 0x50000000u) << 1);
            m.ctrlstat_final = *data;
            return SWD_ACK_OK;
        case SWD_REQ_A3: m.select = *data; return SWD_ACK_OK;
        default: return SWD_ACK_FAULT;
        }
    }
    if ((m.select >> 24) != 0) return SWD_ACK_NO_ACK;   /* only AP0 exists */
    unsigned bank = (m.select >> 4) & 0xF;
    uint32_t prev = m.posted;
    if (rd) {
        if (bank == 0xF && a == (SWD_REQ_A2 | SWD_REQ_A3)) m.posted = m.apidr;
        else if (bank == 0 && a == 0) m.posted = m.csw;
        else if (bank == 0 && a == SWD_REQ_A2) m.posted = m.tar;
        else if (bank == 0 && a == (SWD_REQ_A2 | SWD_REQ_A3)) {
            uint32_t v;
            if (mem_find(m.tar, &v)) m.posted = v;
            else { m.posted = 0; m.ctrlstat |= 0x20; }   /* bus error: STICKYERR, ack still OK */
        } else m.posted = 0;
        *data = prev;
        return SWD_ACK_OK;
    }
    if (bank == 0 && a == 0) { m.csw = *data; m.csw_written = *data; }
    else if (bank == 0 && a == SWD_REQ_A2) m.tar = *data;
    else if (bank == 0 && a == (SWD_REQ_A2 | SWD_REQ_A3)) m.writes_to_mem++;
    return SWD_ACK_OK;
}

/* ---- jtag_ll: a chain model that just returns canned TDO ---- */
static uint8_t g_tdo[32];
static int g_jruns;
int  jtag_ll_run(const uint8_t *seq, size_t len, uint8_t *tdo, size_t cap) {
    (void)seq; (void)len; g_jruns++; memcpy(tdo, g_tdo, cap); return 0;
}
long jtag_ll_tdo_len(const uint8_t *seq, size_t len) { (void)seq; (void)len; return 0; }
void jtag_ll_set_clock(uint32_t hz) { (void)hz; }
void jtag_ll_set_trst_channel(unsigned ch) { (void)ch; }
void jtag_ll_trst(bool a) { (void)a; }

/* ---- helpers ---- */
static void put_bits(uint8_t *buf, size_t *pos, uint32_t v, unsigned n) {
    for (unsigned i = 0; i < n; i++, (*pos)++)
        if ((v >> i) & 1u) buf[*pos / 8] |= (uint8_t)(1u << (*pos % 8));
}
/* a chain: IDCODE devices (and 1-bit bypass for id 0) in TDO-nearest order, then ones */
static size_t build_chain(const uint32_t *ids, size_t n) {
    memset(g_tdo, 0, sizeof(g_tdo));
    size_t pos = 0;
    for (size_t i = 0; i < n; i++) {
        if (ids[i] == 0) pos++;          /* bypass: a 0 bit */
        else put_bits(g_tdo, &pos, ids[i], 32);
    }
    while (pos < sizeof(g_tdo) * 8) { g_tdo[pos / 8] |= (uint8_t)(1u << (pos % 8)); pos++; }
    return sizeof(g_tdo) * 8;
}

/* ---- tests ---- */
static void test_swd_full(void) {
    model_reset();
    uint32_t addrs[2] = { 0xE0042000, 0x5C001000 };   /* the second one bus-errors */
    dap_id_swd_t r;
    dap_id_swd(200, addrs, 2, &r);
    CHECK(r.found && r.dpidr == 0x2ba01477, "dpidr 0x%08x", (unsigned)r.dpidr);
    CHECK(!r.has_targetid, "DP v1 has no TARGETID");
    CHECK(r.has_apidr && r.apidr == 0x24770011, "apidr");
    CHECK(r.has_cpuid && r.cpuid == 0x410fc241, "cpuid 0x%08x", (unsigned)r.cpuid);
    CHECK(r.n_reads == 2 && r.value_ok[0] && r.value[0] == 0x10006421, "first read");
    CHECK(!r.value_ok[1], "bus-error read must be null");
    CHECK(m.aborts >= 2, "sticky error cleared (aborts %d)", m.aborts);
    CHECK((m.csw_written & 7) == 2 && !(m.csw_written & 0x30), "CSW 32-bit, no increment: 0x%08x", (unsigned)m.csw_written);
    CHECK(m.ctrlstat_final == 0, "power-up request not given back: 0x%08x", (unsigned)m.ctrlstat_final);
    CHECK(m.writes_to_mem == 0, "wrote target memory");
    char out[640];
    size_t n = dap_id_fmt_swd(out, sizeof(out), &r);
    const char *want = "{\"port\":\"swd\",\"found\":true,\"dpidr\":\"0x2ba01477\",\"targetid\":null,"
                       "\"apidr\":\"0x24770011\",\"cpuid\":\"0x410fc241\",\"reads\":["
                       "{\"addr\":\"0xe0042000\",\"value\":\"0x10006421\"},"
                       "{\"addr\":\"0x5c001000\",\"value\":null}]}";
    CHECK(n == strlen(want) && strcmp(out, want) == 0, "reply: %s", out);
}

static void test_swd_dp_v2_targetid(void) {
    model_reset();
    m.dpidr = 0x6ba02477; m.targetid = 0x0ba03477;
    dap_id_swd_t r;
    dap_id_swd(200, NULL, 0, &r);
    CHECK(r.has_targetid && r.targetid == 0x0ba03477, "targetid 0x%08x", (unsigned)r.targetid);
    CHECK(r.has_cpuid, "the SELECT bank went back to 0 for the AP");
    char out[640];
    dap_id_fmt_swd(out, sizeof(out), &r);
    CHECK(strstr(out, "\"targetid\":\"0x0ba03477\"") && strstr(out, "\"reads\":[]"), "reply: %s", out);
}

static void test_swd_not_memap(void) {
    model_reset();
    m.apidr = 0x14760001;   /* class 0 (no MEM-AP): no CSW, no memory */
    uint32_t addr = 0xE0042000;
    dap_id_swd_t r;
    dap_id_swd(200, &addr, 1, &r);
    CHECK(r.has_apidr && !r.has_cpuid && !r.value_ok[0] && m.csw_written == 0, "non-MEM-AP is left alone");
}

static void test_swd_absent_and_wait(void) {
    model_reset();
    m.present = false;
    dap_id_swd_t r;
    uint64_t t0 = g_now_us;
    dap_id_swd(200, NULL, 0, &r);
    CHECK(!r.found, "nothing on the wire");
    CHECK(g_now_us - t0 >= 200000, "retried for the wait window (%llu us)", (unsigned long long)(g_now_us - t0));
    char out[64];
    dap_id_fmt_swd(out, sizeof(out), &r);
    CHECK(strcmp(out, "{\"port\":\"swd\",\"found\":false}") == 0, "reply: %s", out);

    model_reset();
    m.up_after_ms = 120;   /* a target still booting */
    dap_id_swd(200, NULL, 0, &r);
    CHECK(r.found, "a target that comes up within the window is found");

    model_reset();
    m.waits = 5;   /* WAIT is retried, not an error */
    dap_id_swd(200, NULL, 0, &r);
    CHECK(r.found && r.has_cpuid, "WAIT acks are retried");

    model_reset();
    m.ap_read_waits = 3;   /* a read that got WAIT knocks the DP out of step until a line reset */
    {
        uint32_t want = 0xE0042000;
        dap_id_swd(200, &want, 1, &r);
    }
    CHECK(r.found && r.has_cpuid && r.cpuid == 0x410fc241 && r.value_ok[0] && r.value[0] == 0x10006421,
          "a WAIT on an AP read is retried after a line reset");

    model_reset();
    m.waits = 1000000;   /* a target stuck on WAIT: bounded, no hang */
    dap_id_swd(0, NULL, 0, &r);
    CHECK(!r.found, "stuck WAIT gives up");

    model_reset();
    m.dpidr = 0;   /* a floating line that parsed */
    dap_id_swd(0, NULL, 0, &r);
    CHECK(!r.found, "DPIDR 0 is not a DP");
}

static void test_jtag_parse(void) {
    uint32_t id[8];
    uint32_t two[2] = { 0x4ba00477, 0x06421041 };
    size_t nb = build_chain(two, 2);
    CHECK(dap_id_jtag_parse(g_tdo, nb, id, 8) == 2 && id[0] == 0x4ba00477 && id[1] == 0x06421041,
          "two-device chain, TDO-nearest first");
    uint32_t byp[3] = { 0, 0x4ba00477, 0 };   /* bypass devices around an IDCODE one */
    nb = build_chain(byp, 3);
    CHECK(dap_id_jtag_parse(g_tdo, nb, id, 8) == 1 && id[0] == 0x4ba00477, "bypass devices skipped");
    memset(g_tdo, 0xFF, sizeof(g_tdo));
    CHECK(dap_id_jtag_parse(g_tdo, 256, id, 8) == 0, "all ones: no chain");
    memset(g_tdo, 0x00, sizeof(g_tdo));
    CHECK(dap_id_jtag_parse(g_tdo, 256, id, 8) == 0, "all zeros: no chain");
    uint32_t eight[8];
    for (int i = 0; i < 8; i++) eight[i] = 0x10000001u + ((uint32_t)i << 12);
    nb = build_chain(eight, 8);
    CHECK(dap_id_jtag_parse(g_tdo, nb, id, 8) == 8 && id[7] == eight[7], "eight devices fill the scan");
    CHECK(dap_id_jtag_parse(g_tdo, nb, id, 3) == 3, "cap respected");
    uint32_t one[1] = { 0x4ba00477 };
    build_chain(one, 1);
    CHECK(dap_id_jtag_parse(g_tdo, 40, id, 8) == 1, "short capture still parses a whole IDCODE");
    CHECK(dap_id_jtag_parse(g_tdo, 20, id, 8) == 0, "a partial IDCODE is not reported");
}

static void test_jtag_scan_and_format(void) {
    uint32_t id[8];
    uint32_t two[2] = { 0x4ba00477, 0x06421041 };
    build_chain(two, 2);
    g_jruns = 0;
    size_t n = dap_id_jtag(200, id, 8);
    CHECK(n == 2 && g_jruns == 1, "scan found %zu in %d runs", n, g_jruns);
    char out[256];
    dap_id_fmt_jtag(out, sizeof(out), id, n);
    CHECK(strcmp(out, "{\"port\":\"jtag\",\"found\":true,\"idcodes\":[\"0x4ba00477\",\"0x06421041\"]}") == 0,
          "reply: %s", out);
    memset(g_tdo, 0xFF, sizeof(g_tdo));
    uint64_t t0 = g_now_us;
    n = dap_id_jtag(100, id, 8);
    CHECK(n == 0 && g_now_us - t0 >= 100000, "empty chain retried for the window");
    dap_id_fmt_jtag(out, sizeof(out), id, 0);
    CHECK(strcmp(out, "{\"port\":\"jtag\",\"found\":false}") == 0, "reply: %s", out);
}

static void test_reads_and_size(void) {
    uint32_t a[8];
    CHECK(dap_id_parse_reads("{\"cmd\":\"dap_start\"}", a, 8) == 0, "no read key");
    CHECK(dap_id_parse_reads("{\"read\":[\"0xE0042000\",\"0x5C001000\", 4096]}", a, 8) == 3 &&
          a[0] == 0xE0042000 && a[1] == 0x5C001000 && a[2] == 4096, "three addresses");
    CHECK(dap_id_parse_reads("{\"read\":[]}", a, 8) == 0, "empty list");
    CHECK(dap_id_parse_reads("{\"read\":[1,2,3,4,5,6,7,8,9]}", a, 8) == -1, "nine addresses rejected");
    CHECK(dap_id_parse_reads("{\"read\":[1,2,3,4,5,6,7,8]}", a, 8) == 8, "eight accepted");
    CHECK(dap_id_parse_reads("{\"read\":[\"zz\"]}", a, 8) == -1, "not a number");
    /* the worst case reply fits the handler's buffer and one cloud frame (1792 B) */
    dap_id_swd_t r;
    memset(&r, 0, sizeof(r));
    r.found = r.has_targetid = r.has_apidr = r.has_cpuid = true;
    r.n_reads = 8;
    for (int i = 0; i < 8; i++) { r.addr[i] = 0xE0042000u + (uint32_t)i; r.value[i] = 0xFFFFFFFFu; r.value_ok[i] = true; }
    char out[640];
    size_t n = dap_id_fmt_swd(out, sizeof(out), &r);
    CHECK(n > 0 && n < 640 && n < 1792 - 64, "worst case %zu bytes", n);
    CHECK(dap_id_fmt_swd(out, 40, &r) == 0, "too small a buffer is reported, not clipped");
}

int main(void) {
    test_swd_full();
    test_swd_dp_v2_targetid();
    test_swd_not_memap();
    test_swd_absent_and_wait();
    test_jtag_parse();
    test_jtag_scan_and_format();
    test_reads_and_size();
    if (fails) { printf("FAIL %d\n", fails); return 1; }
    printf("PASS test_dap_id\n");
    return 0;
}
