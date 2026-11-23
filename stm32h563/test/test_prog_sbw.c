/* Host test for prog_sbw.c against a bit-level model of an MSP430 Spy-Bi-Wire target (prog_sim.h).
 *
 * The model is written from docs/design/spy-bi-wire.md (SLAU320AJ, the family guides, the datasheets
 * and TI's Replicator430FR source), not from prog_sbw.c. It works on the two pins only:
 *  - SBW off while TEST is low. TEST high, then after tSBW,En (110 us) the first falling edge with
 *    SBWTDIO high is the first clock and selects SBW (low selects 4-wire JTAG: not modeled, counted).
 *    A TEST low phase of 100 us or more exits SBW; a low phase longer than the limit (7 us, SLAU320 2.2.3.4)
 *    but under 100 us deactivates the SBW logic.
 *  - Slots TMS, TDI, TDO after the first clock. TMS and TDI are captured on the falling edge; the chip
 *    drives SBWTDIO only while SBWTCK is low in the TDO slot. The internal TCK (a TAP step) comes at the
 *    end of the TDO slot with the captured TMS/TDI, and the TDO bit shown in a cycle is the output before
 *    that cycle's step (a BYPASS register returns its input one clock late).
 *  - TCLK: while the TAP stays in Run-Test/Idle (TMS captured 0) it follows SBWTDIO at the rising edge of
 *    the TMS slot and of the TDI slot, so a TMS slot that drives SBWTDIO low while TCLK is high glitches it
 *    (TMSLDH exists to prevent that). Rising TCLK edge: with R/W = read and DATA_TO_ADDR loaded the data
 *    of MAB goes on MDB. Falling edge: with R/W = write and MDB loaded, MDB is written to MAB.
 *  - IEEE 1149.1 TAP; IR 8 bit LSB first, the JTAG ID MSB first on every IR scan; DR MSB first;
 *    the 20 bit address/device-pointer registers shift out as bits 15..0 then 19..16.
 *  - CPUXv2 control register (sync bit 9, CPUSUSP bit 8, ...), watchdog, SYSCFG0 / MPU / FR59xx write
 *    protection, FRAM, the JTAG lock (every instruction but the mailbox is BYPASS), the JTAG mailbox
 *    (works while the chip is held in reset by the RST-low entry) with the 0xA55A + 0x1A1A mass erase,
 *    which runs when SBW is left, and the FR2xx/FR4xx and FR5xxx variants.
 * The pins are also checked: nobody drives SBWTDIO together with the chip, the pod drives it in the TMS
 * and TDI slots, data never changes in the same command as a clock edge (setup 4 ns / hold 19 ns need
 * their own command), the longest SBWTCK low phase, and a flush never lands with SBWTCK low. */
#include "prog_flash.h"
#include "prog_sbw.h"
#include "prog_sim.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

#define TCK_BYTE_FAST_NS 2048u      /* one SPI byte at 3.9 MHz */
#define TCK_BYTE_BOOT_NS 4096u      /* at 1.95 MHz */
#define LOW_LIMIT_NS     7000u      /* SLAU320 2.2.3.4 */
#define EN_NS            110000u    /* tSBW,En, FR2xx/FR4xx/FR5xxx datasheets */
#define EXIT_NS          100000u    /* SLAU320 2.3.1.1: TEST low for more than 100 us */

enum { TLR, RTI, SDR, CDR, SHDR, E1DR, PDR, E2DR, UDR, SIR, CIR, SHIR, E1IR, PIR, E2IR, UIR };
static const uint8_t k_next[16][2] = {
    [TLR] = {RTI, TLR}, [RTI] = {RTI, SDR}, [SDR] = {CDR, SIR}, [CDR] = {SHDR, E1DR},
    [SHDR] = {SHDR, E1DR}, [E1DR] = {PDR, UDR}, [PDR] = {PDR, E2DR}, [E2DR] = {SHDR, UDR},
    [UDR] = {RTI, SDR}, [SIR] = {CIR, TLR}, [CIR] = {SHIR, E1IR}, [SHIR] = {SHIR, E1IR},
    [E1IR] = {PIR, UIR}, [PIR] = {PIR, E2IR}, [E2IR] = {SHIR, UIR}, [UIR] = {RTI, SDR},
};

typedef struct {
    /* configuration */
    bool     silent;               /* no chip: nothing ever answers */
    bool     fr5;                  /* FR5994-like (JTAG ID 0x99) instead of FR2433-like (0x98) */
    uint8_t  jid;
    bool     locked;               /* JTAG lock signatures in effect */
    unsigned sync_polls;           /* control register reads before TCE0 shows; 9999 = never */
    uint64_t low_limit_ns;         /* SBWTCK low phase beyond this deactivates SBW */
    int      corrupt_addr;         /* FRAM word at this address stores a flipped bit 0, -1 none */

    /* pins and SBW */
    int      prev_clk, prev_raw;
    bool     test_high; uint64_t test_high_since;
    bool     halted;               /* RST was low when TEST rose: boot code stopped, mailbox works */
    bool     sbw_on, first_clock, tdo_driving;
    int      slot, cap_tms, cap_tdi;
    uint64_t low_since;
    /* observation */
    unsigned entries, early_clock, bad_select, deactivated, exits, overlap, undriven, simultaneous;
    uint64_t max_low_ns;
    unsigned tclk_rise, tclk_fall, tclk_in_tms, erases;

    /* TAP and registers */
    int      tap, ir_cur;
    uint8_t  ir_out[8], ir_in[8]; unsigned ir_pos, ir_in_n;
    uint8_t  dr_out[24], dr_in[24]; unsigned dr_out_n, dr_pos, dr_in_n;
    bool     dr_bypass; uint8_t byp;
    int      tdo_bit;

    /* CPU */
    uint16_t cntrl; bool synced, cpususp; unsigned sync_reads, susp_rises;
    uint32_t mab; uint16_t mdb; bool mdb_loaded; int tclk;
    unsigned por_count, releases;
    bool     wdt_held; unsigned tm_writes[2];
    uint16_t syscfg0, frwp, mpuctl0;
    unsigned blocked_writes, bad_password;
    /* mailbox */
    unsigned jmb_expect, jmb_n; uint16_t jmb_w[2]; bool jmb32, erase_pending, got_erase_msg;
    uint8_t  mem[0x10000];
} m430_t;

static uint16_t rd16(const m430_t *m, uint32_t a) { return (uint16_t)(m->mem[a & 0xFFFF] | (m->mem[(a + 1) & 0xFFFF] << 8)); }
static void wr16m(m430_t *m, uint32_t a, uint16_t v) { m->mem[a & 0xFFFF] = (uint8_t)v; m->mem[(a + 1) & 0xFFFF] = (uint8_t)(v >> 8); }

static bool in_fram(uint32_t a) { return a >= 0xC400 && a <= 0xFFFF; }
static bool in_info(uint32_t a) { return a >= 0x1800 && a <= 0x19FF; }

static uint16_t mem_read(const m430_t *m, uint32_t a)
{
    if (a == 0x0160) return m->syscfg0;
    if (a == 0x0140 && m->fr5) return m->frwp;
    if (a == 0x05A0) return m->mpuctl0;
    return rd16(m, a);
}

static void mem_write(m430_t *m, uint32_t a, uint16_t v)
{
    uint32_t wdt = m->jid == 0x98 ? 0x01CC : 0x015C;
    if (a == wdt) { if (v == 0x5A80) m->wdt_held = true; return; }
    if (a == 0x0160 && !m->fr5) {
        if ((v >> 8) != 0xA5) { m->bad_password++; return; }          /* a wrong password is a PUC */
        m->syscfg0 = (uint16_t)(0x9600 | (v & 0xFF));
        return;
    }
    if (a == 0x0140 && m->fr5) {
        if ((v >> 8) != 0xA5) { m->bad_password++; return; }
        m->frwp = (uint16_t)(0x9600 | (v & 0xFF));
        return;
    }
    if (a == 0x05A0) {
        if ((v >> 8) != 0xA5) { m->bad_password++; return; }
        m->mpuctl0 = (uint16_t)(0x9600 | (v & 0xFF));
        return;
    }
    if (a == 0x06 || a == 0x08) { m->tm_writes[a == 8]++; wr16m(m, a, v); return; }
    if (in_fram(a) || in_info(a)) {
        bool prot = m->fr5 ? (m->frwp & 1) : in_info(a) ? (m->syscfg0 & 2) : (m->syscfg0 & 1);
        if (prot) { m->blocked_writes++; return; }
        if ((int)a == m->corrupt_addr) v ^= 1;
        wr16m(m, a, v);
        return;
    }
    if (a >= 0x2000 && a < 0x3000) wr16m(m, a, v);
}

/* ---- TCLK ---- */

static void set_tclk(m430_t *m, int v)
{
    if (v == m->tclk) return;
    m->tclk = v;
    if (v) {
        m->tclk_rise++;
        if (m->cpususp) m->susp_rises++;
        if ((m->cntrl & 1u) && m->ir_cur == 0x85) m->mdb = mem_read(m, m->mab);   /* read: MDB shows MAB's data */
    } else {
        m->tclk_fall++;
        if (!(m->cntrl & 1u) && m->mdb_loaded && m->ir_cur == 0x85) {            /* write on the falling edge */
            mem_write(m, m->mab, m->mdb);
            m->mdb_loaded = false;
        }
    }
}

/* ---- registers ---- */

static uint16_t cntrl_read(m430_t *m)
{
    if (m->cntrl & 0x0400) { if (++m->sync_reads > m->sync_polls) m->synced = true; }
    uint16_t r = (uint16_t)(m->cntrl & 0x0C01);
    if (m->cpususp && m->susp_rises >= 1) r |= 0x0100;
    if (m->synced) r |= 0x0200;
    return r;
}

static void cntrl_write(m430_t *m, uint16_t v)
{
    if ((m->cntrl & 0x0800) && !(v & 0x0800)) m->por_count++;
    bool susp = (v & 0x0100) != 0;
    if (susp && !m->cpususp) m->susp_rises = 0;
    m->cpususp = susp;
    if ((v & 0x0400) && !(m->cntrl & 0x0400)) { m->sync_reads = 0; m->synced = false; }
    if (!(v & 0x0400)) m->synced = false;
    m->cntrl = v;
}

static void bits_msb(uint8_t *dst, unsigned n, uint32_t v) { for (unsigned i = 0; i < n; i++) dst[i] = (uint8_t)((v >> (n - 1 - i)) & 1u); }

static uint32_t from_bits_msb(const uint8_t *b, unsigned n) { uint32_t v = 0; for (unsigned i = 0; i < n; i++) v = (v << 1) | b[i]; return v; }

static void scrambled20(uint8_t *dst, uint32_t v)       /* bits 15..0, then 19..16 */
{
    bits_msb(dst, 16, v & 0xFFFF);
    bits_msb(dst + 16, 4, (v >> 16) & 0xF);
}

static void dr_capture(m430_t *m)
{
    m->dr_in_n = 0; m->dr_pos = 0; m->dr_bypass = false;
    if (m->locked && m->ir_cur != 0x61) m->dr_bypass = true;
    switch (m->ir_cur) {
    case 0x13: case 0x14: bits_msb(m->dr_out, 16, cntrl_read(m)); m->dr_out_n = 16; break;
    case 0x83: case 0x84: scrambled20(m->dr_out, m->mab); m->dr_out_n = 20; break;
    case 0x85: case 0x41: case 0x42: case 0x43: bits_msb(m->dr_out, 16, m->mdb); m->dr_out_n = 16; break;
    case 0x17: bits_msb(m->dr_out, 16, 0x1234); m->dr_out_n = 16; break;             /* core ID, not zero */
    case 0x87: scrambled20(m->dr_out, 0x1A00); m->dr_out_n = 20; break;              /* the device descriptor pointer */
    case 0x61: bits_msb(m->dr_out, 16, (m->halted && m->jmb_expect == 0) ? 1u : 0u); m->dr_out_n = 16; break;
    default:   m->dr_bypass = true; break;
    }
    if (m->dr_bypass) { m->byp = 0; m->dr_out_n = 1; m->dr_out[0] = 0; }
}

static void dr_update(m430_t *m)
{
    unsigned n = m->dr_in_n;
    if (m->dr_bypass) return;
    uint32_t v = from_bits_msb(m->dr_in, n);
    switch (m->ir_cur) {
    case 0x13: if (n == 16) cntrl_write(m, (uint16_t)v); break;
    case 0x83: if (n == 20) m->mab = v; else if (n == 16) m->mab = (m->mab & 0xF0000u) | v; break;
    case 0x85: if (n == 16) { m->mdb = (uint16_t)v; m->mdb_loaded = true; } break;
    case 0x41: if (n == 16) m->mdb = (uint16_t)v; break;
    case 0x61:
        if (n != 16 || !m->halted) break;
        if (m->jmb_expect) {
            m->jmb_w[m->jmb_n++] = (uint16_t)v;
            if (--m->jmb_expect == 0 && m->jmb32 && m->jmb_w[0] == 0xA55A && m->jmb_w[1] == 0x1A1A) {
                m->erase_pending = true; m->got_erase_msg = true;
            }
        } else if (v & 1u) {                                         /* INREQ */
            m->jmb32 = (v & 0x10u) != 0;
            m->jmb_expect = m->jmb32 ? 2 : 1; m->jmb_n = 0;
        }
        break;
    default: break;
    }
}

static void ir_update(m430_t *m)
{
    unsigned v = 0;
    for (unsigned k = 0; k < 8; k++) v |= (unsigned)m->ir_in[k] << k;            /* first bit in = LSB */
    m->ir_cur = (int)v;
    if (v == 0x15) m->releases++;                                                /* IR_CNTRL_SIG_RELEASE */
}

static void tap_step(m430_t *m, int tms, int tdi)
{
    switch (m->tap) {
    case CIR: for (unsigned i = 0; i < 8; i++) m->ir_out[i] = (uint8_t)((m->jid >> (7 - i)) & 1u);   /* the JTAG ID, MSB first */
              m->ir_pos = 0; m->ir_in_n = 0; break;
    case SHIR: if (m->ir_in_n < 8) m->ir_in[m->ir_in_n] = (uint8_t)tdi;
               m->ir_in_n++; m->ir_pos++; break;
    case CDR: dr_capture(m); break;
    case SHDR:
        if (m->dr_bypass) { m->byp = (uint8_t)tdi; }
        if (m->dr_in_n < sizeof m->dr_in) m->dr_in[m->dr_in_n] = (uint8_t)tdi;
        m->dr_in_n++; m->dr_pos++;
        break;
    default: break;
    }
    int nx = k_next[m->tap][tms];
    if (nx == UDR && m->tap != UDR) dr_update(m);
    if (nx == UIR && m->tap != UIR) ir_update(m);
    m->tap = nx;
    if (nx == TLR) { m->ir_cur = 0xFF; }
    if (nx == SHDR) m->tdo_bit = m->dr_bypass ? m->byp : (m->dr_pos < m->dr_out_n ? m->dr_out[m->dr_pos] : 0);
    else if (nx == SHIR) m->tdo_bit = m->ir_pos < 8 ? m->ir_out[m->ir_pos] : 0;
    else m->tdo_bit = 0;
}

/* ---- the pins ---- */

static void sbw_off(m430_t *m) { m->sbw_on = false; m->first_clock = false; m->tdo_driving = false; }

static void sbw_exit(m430_t *m)
{
    m->exits++;
    sbw_off(m);
    if (m->erase_pending) {                      /* the boot code runs the user-code erase after the BOR */
        for (uint32_t a = 0xC400; a <= 0xFFFF; a++) m->mem[a] = 0xFF;
        for (uint32_t a = 0x1800; a <= 0x19FF; a++) m->mem[a] = 0xFF;
        m->locked = false; m->erase_pending = false; m->erases++;
    }
    m->halted = false;
}

static bool chip_drives(const m430_t *m) { return m->sbw_on && !m->first_clock && m->slot == 2 && m->tdo_driving; }

static void m_on_pins(prog_sim_t *s, int clk, int pod)
{
    m430_t *m = s->chip;
    uint64_t now = s->ns;
    if (m->silent) { m->prev_clk = clk; if (clk) { m->test_high = true; } return; }
    if (chip_drives(m) && pod != -1) m->overlap++;
    if (clk == m->prev_clk) { m->prev_raw = s->pod_data; return; }
    if (m->sbw_on && s->pod_data != m->prev_raw) m->simultaneous++;
    m->prev_raw = s->pod_data;
    m->prev_clk = clk;
    int lvl = pod < 0 ? 1 : pod;                  /* released: the pullup */

    if (clk == 0) {                                /* ---- falling edge ---- */
        m->low_since = now;
        if (!m->sbw_on) {
            if (m->test_high && now - m->test_high_since >= EN_NS) {
                m->entries++;
                if (lvl) { m->sbw_on = true; m->first_clock = true; m->slot = 0; m->tclk = 1; m->tap = TLR; m->ir_cur = 0xFF; }
                else m->bad_select++;
            } else if (m->test_high) m->early_clock++;
            m->test_high = false;
            return;
        }
        if (m->first_clock) return;
        if (m->slot == 0) { if (pod < 0) m->undriven++; m->cap_tms = lvl; }
        else if (m->slot == 1) { if (pod < 0) m->undriven++; m->cap_tdi = lvl; }
        else m->tdo_driving = true;
        return;
    }
    /* ---- rising edge ---- */
    uint64_t low = now - m->low_since;
    if (!m->sbw_on) {
        m->test_high = true; m->test_high_since = now;
        m->halted = (pod == 0);
        return;
    }
    if (low >= EXIT_NS) { sbw_exit(m); m->test_high = true; m->test_high_since = now; m->halted = (pod == 0); return; }
    if (low > m->low_limit_ns) { m->deactivated++; sbw_off(m); m->test_high = true; m->test_high_since = now; return; }
    if (low > m->max_low_ns) m->max_low_ns = low;
    if (m->first_clock) { m->first_clock = false; m->slot = 0; return; }
    m->tdo_driving = false;
    bool stay = (m->tap == RTI && m->cap_tms == 0);
    if (m->slot == 0 && stay) { int before = m->tclk; set_tclk(m, lvl); if (m->tclk != before) m->tclk_in_tms++; }
    else if (m->slot == 1 && stay) set_tclk(m, lvl);
    else if (m->slot == 2) tap_step(m, m->cap_tms, m->cap_tdi);
    m->slot = (m->slot + 1) % 3;
}

static int m_chip_data(prog_sim_t *s)
{
    m430_t *m = s->chip;
    if (m->silent) return -1;
    return chip_drives(m) ? m->tdo_bit : -1;
}

/* Time passing with TEST low: 100 us or more leaves SBW, without waiting for the next rising edge. */
static void m_on_time(prog_sim_t *s, uint32_t us)
{
    (void)us;
    m430_t *m = s->chip;
    if (!m->silent && m->sbw_on && s->clk == 0 && s->ns - m->low_since >= EXIT_NS) sbw_exit(m);
}

static const prog_sim_model_t k_model = { .on_pins = m_on_pins, .chip_data = m_chip_data, .on_time = m_on_time };

/* ---- harness ---- */

#define IMG_SZ 0x400u

typedef struct {
    prog_sim_t sim; prog_wire_ops_t ops; m430_t chip;
    prog_ctx_t ctx;
    uint8_t img[IMG_SZ]; uint8_t rd[IMG_SZ];
} rig_t;

static int rig_src(void *job, uint32_t off, uint8_t *buf, uint32_t n) {
    rig_t *r = job;
    if (off + n > sizeof r->img) return -1;
    memcpy(buf, r->img + off, n);
    return 0;
}
static int rig_dst(void *job, uint32_t off, const uint8_t *buf, uint32_t n) {
    rig_t *r = job;
    if (off + n > sizeof r->rd) return -1;
    memcpy(r->rd + off, buf, n);
    return 0;
}


/* prog_wire.c flushes by itself: at 192 queued bytes, when a clock-high set() finds 176 queued, and before a
   sample when nsamples + nfeed >= 128. This layer repeats those three rules over the sim's ops, so a protocol
   that would let one of them land in a slot's low phase shows up as low_flush, and the 40 us a flush costs then
   exceeds the chip's low limit (docs 14). */
#define WM_FEED_MAX 192u
#define WM_FEED_SOFT 176u
#define WM_SAMPLES_MAX 128u
static struct { prog_wire_ops_t orig; unsigned nfeed, nsaved, low_flush; bool in_delay; uint8_t saved[WM_SAMPLES_MAX]; } g_wm;

static void wm_flush_now(void) {
    uint8_t s[WM_SAMPLES_MAX];
    prog_sim_t *sim = g_wm.orig.io;
    bool slot_low = sim->clk == 0 && ((m430_t *)sim->chip)->sbw_on;   /* with SBW off a low TEST is the entry/exit */
    if (slot_low && !g_wm.in_delay) g_wm.low_flush++;    /* a flush before a delay is the entry/exit, not a slot */
    int n = g_wm.orig.flush(g_wm.orig.io, s, sizeof s);
    if (g_wm.nfeed) g_wm.orig.delay_us(g_wm.orig.io, 40);              /* a flush takes tens of us on the pod */
    for (int i = 0; i < n && g_wm.nsaved < WM_SAMPLES_MAX; i++) g_wm.saved[g_wm.nsaved++] = s[i];
    g_wm.nfeed = 0;
}
static void wm_guard(bool sample) {
    if (g_wm.nfeed >= WM_FEED_MAX || (sample && g_wm.nsaved + g_wm.nfeed >= WM_SAMPLES_MAX)) wm_flush_now();
}
static void wm_set(void *io, int clk, int d) {
    wm_guard(false); g_wm.orig.set(io, clk, d); g_wm.nfeed++;
    if (clk && g_wm.nfeed >= WM_FEED_SOFT) wm_flush_now();
}
static void wm_drive(void *io, bool on) { wm_guard(false); g_wm.orig.data_drive(io, on); g_wm.nfeed++; }
static void wm_sample(void *io) { wm_guard(true); g_wm.orig.sample(io); g_wm.nfeed++; }
static int wm_flush(void *io, uint8_t *samples, size_t cap) {
    (void)io;
    wm_flush_now();
    size_t n = g_wm.nsaved < cap ? g_wm.nsaved : cap;
    if (samples && n) memcpy(samples, g_wm.saved, n);
    g_wm.nsaved = 0;
    return (int)n;
}
static void wm_delay(void *io, uint32_t us) {
    g_wm.in_delay = true;
    if (g_wm.nfeed) wm_flush_now();
    g_wm.in_delay = false;
    g_wm.orig.delay_us(io, us);
}

static void rig_init(rig_t *r, unsigned byte_ns, bool fr5) {
    memset(r, 0, sizeof *r);
    m430_t *m = &r->chip;
    m->fr5 = fr5; m->jid = fr5 ? 0x99 : 0x98;
    m->sync_polls = 3; m->low_limit_ns = LOW_LIMIT_NS; m->corrupt_addr = -1;
    m->prev_clk = 1; m->prev_raw = 1; m->tclk = 1; m->ir_cur = 0xFF; m->tdo_bit = 0;
    memset(m->mem, 0xA5, sizeof m->mem);
    m->mem[0x1A04] = 0x40; m->mem[0x1A05] = 0x82;                 /* device ID, FR2433 (docs 8) */
    m->syscfg0 = 0x9603;                                          /* PFWP and DFWP set after reset */
    m->frwp = 0x9601;
    m->mpuctl0 = fr5 ? 0x9601 : 0x9600;                           /* FR5 variant: MPU enabled, not locked */
    prog_sim_init(&r->sim, &r->ops, &k_model, m);
    r->sim.byte_ns = byte_ns;
    memset(&g_wm, 0, sizeof g_wm);
    g_wm.orig = r->ops;
    r->ops.set = wm_set; r->ops.data_drive = wm_drive; r->ops.sample = wm_sample;
    r->ops.flush = wm_flush; r->ops.delay_us = wm_delay;
    r->ctx.w = &r->ops;
    r->ctx.src = rig_src; r->ctx.dst = rig_dst; r->ctx.job = r;
    for (unsigned i = 0; i < sizeof r->img; i++) r->img[i] = (uint8_t)((i * 131u + 7u) ^ (i >> 5));
}

static int run(int (*step)(prog_ctx_t *), prog_ctx_t *ctx, unsigned *calls) {
    unsigned n = 0;
    int rc;
    do { rc = step(ctx); n++; } while (rc == PROG_RUNNING && n < 100000);
    if (calls) *calls = n;
    return rc;
}

/* The pin rules that hold for every test with a working link. */
static void check_pins(rig_t *r) {
    CHECK(r->sim.contention == 0);
    CHECK(g_wm.low_flush == 0);
    CHECK(r->chip.overlap == 0);
    CHECK(r->chip.undriven == 0);
    CHECK(r->chip.simultaneous == 0);
    CHECK(r->chip.deactivated == 0);
    CHECK(r->chip.bad_select == 0);
    CHECK(r->chip.early_clock == 0);
    CHECK(r->chip.bad_password == 0);
    CHECK(r->chip.tclk_in_tms == 0);               /* TMSLDH keeps TCLK out of the TMS slot (docs 5.1) */
    CHECK(r->chip.max_low_ns <= LOW_LIMIT_NS);
    CHECK(r->chip.max_low_ns >= TCK_BYTE_FAST_NS);        /* the model did see slots */
}

#define PROTO prog_proto_sbw

/* ---- tests ---- */

static void test_enter(void) {
    rig_t *r = malloc(sizeof *r); rig_init(r, TCK_BYTE_FAST_NS, false);
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    CHECK(r->ctx.id_len == 3);
    CHECK(r->ctx.id[0] == 0x98 && r->ctx.id[1] == 0x40 && r->ctx.id[2] == 0x82);
    CHECK(r->chip.entries == 1);
    CHECK(r->chip.synced && r->chip.cpususp);              /* CPU taken over, Full-Emulation-State */
    CHECK(r->chip.por_count >= 1);
    CHECK(r->chip.wdt_held);
    CHECK(r->chip.tm_writes[0] == 0);                      /* 0x06/0x08 only for JTAG ID 0x91/0x99 */
    check_pins(r);
    /* a TDO slot with a read is two byte times low, nothing is longer */
    CHECK(r->chip.max_low_ns == 2u * TCK_BYTE_FAST_NS);
    CHECK(r->sim.clk == 1);
    free(r);

    r = malloc(sizeof *r); rig_init(r, TCK_BYTE_FAST_NS, true);   /* FR5994-like */
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    CHECK(r->ctx.id[0] == 0x99 && r->chip.wdt_held);
    CHECK(r->chip.tm_writes[0] == 1 && r->chip.tm_writes[1] == 1);
    check_pins(r);
    free(r);
}

static void test_tclk_edges(void) {
    /* Scans do not move TCLK (they restore TCLK_saved) and ClrTCLK/SetTCLK each give one edge. A word read
       is ClrTCLK SetTCLK ClrTCLK SetTCLK ClrTCLK SetTCLK ClrTCLK (docs 11): TCLK is high after enter's
       SetTCLK, so the first ClrTCLK is a fall, and the pairs give 3 falls and 3 rises. */
    rig_t *r = malloc(sizeof *r); rig_init(r, TCK_BYTE_FAST_NS, false);
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    unsigned rise0 = r->chip.tclk_rise, fall0 = r->chip.tclk_fall;
    r->ctx.mem = PROG_MEM_FLASH; r->ctx.addr = 0xC400; r->ctx.len = 2;
    CHECK(run(PROTO.read, &r->ctx, NULL) == PROG_OK);
    CHECK(r->chip.tclk == 1);
    CHECK(r->chip.tclk_rise - rise0 == 3 && r->chip.tclk_fall - fall0 == 3);
    check_pins(r);
    free(r);
}

static void test_write_read(void) {
    rig_t *r = malloc(sizeof *r); rig_init(r, TCK_BYTE_FAST_NS, false);
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->ctx.mem = PROG_MEM_FLASH; r->ctx.addr = 0xC400; r->ctx.len = 100;   /* 50 words: 4 blocks */
    unsigned calls = 0;
    CHECK(run(PROTO.write, &r->ctx, &calls) == PROG_OK);
    CHECK(calls == 4 && r->ctx.done == 4 && r->ctx.total == 4);
    CHECK(memcmp(&r->chip.mem[0xC400], r->img, 100) == 0);
    CHECK((r->chip.syscfg0 & 3) == 0);                      /* the write protection was lifted first */
    CHECK(r->chip.blocked_writes == 0);
    memset(r->rd, 0, sizeof r->rd);
    CHECK(run(PROTO.read, &r->ctx, NULL) == PROG_OK);
    CHECK(memcmp(r->rd, r->img, 100) == 0);
    /* information FRAM (PROG_MEM_CONFIG), the same accesses */
    r->ctx.mem = PROG_MEM_CONFIG; r->ctx.addr = 0x1800; r->ctx.len = 8;
    CHECK(run(PROTO.write, &r->ctx, NULL) == PROG_OK);
    CHECK(memcmp(&r->chip.mem[0x1800], r->img, 8) == 0);
    /* argument checks */
    r->ctx.mem = PROG_MEM_EEPROM; CHECK(run(PROTO.write, &r->ctx, NULL) == PROG_E_ARGS);
    r->ctx.mem = PROG_MEM_FLASH;
    r->ctx.addr = 0xC401; CHECK(run(PROTO.write, &r->ctx, NULL) == PROG_E_ARGS);
    r->ctx.addr = 0xC400; r->ctx.len = 3; CHECK(run(PROTO.write, &r->ctx, NULL) == PROG_E_ARGS);
    r->ctx.len = 0; CHECK(run(PROTO.write, &r->ctx, NULL) == PROG_E_ARGS);
    r->ctx.addr = 0xFFFFE; r->ctx.len = 4; CHECK(run(PROTO.write, &r->ctx, NULL) == PROG_E_ARGS);
    check_pins(r);
    free(r);

    r = malloc(sizeof *r); rig_init(r, TCK_BYTE_FAST_NS, true);   /* FR59xx: MPU and write protect register */
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->ctx.mem = PROG_MEM_FLASH; r->ctx.addr = 0xC400; r->ctx.len = 40;
    CHECK(run(PROTO.write, &r->ctx, NULL) == PROG_OK);
    CHECK(memcmp(&r->chip.mem[0xC400], r->img, 40) == 0);
    CHECK((r->chip.mpuctl0 & 1) == 0 && (r->chip.frwp & 1) == 0);
    check_pins(r);
    free(r);
}

static void test_verify_fail(void) {
    rig_t *r = malloc(sizeof *r); rig_init(r, TCK_BYTE_FAST_NS, false);
    r->chip.corrupt_addr = 0xC410;
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->ctx.mem = PROG_MEM_FLASH; r->ctx.addr = 0xC400; r->ctx.len = 64;
    CHECK(run(PROTO.write, &r->ctx, NULL) == PROG_E_VERIFY && r->ctx.at_addr == 0xC410);
    check_pins(r);
    free(r);

    /* the image source runs out: the last block asks for bytes past the staged image */
    r = malloc(sizeof *r); rig_init(r, TCK_BYTE_FAST_NS, false);
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->ctx.mem = PROG_MEM_FLASH; r->ctx.addr = 0xC400; r->ctx.len = IMG_SZ + 2;
    CHECK(run(PROTO.write, &r->ctx, NULL) == PROG_E_SRC);
    check_pins(r);
    free(r);
}

static void test_erase_range(void) {
    rig_t *r = malloc(sizeof *r); rig_init(r, TCK_BYTE_FAST_NS, false);
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->ctx.mem = PROG_MEM_FLASH; r->ctx.addr = 0xC800; r->ctx.len = 48;
    r->ctx.erase = PROG_ERASE_NONE;
    CHECK(run(PROTO.erase, &r->ctx, NULL) == PROG_OK);
    CHECK(r->chip.mem[0xC800] == 0xA5);                    /* untouched */
    r->ctx.erase = PROG_ERASE_PAGE;
    CHECK(run(PROTO.erase, &r->ctx, NULL) == PROG_OK);
    for (unsigned i = 0; i < 48; i++) CHECK(r->chip.mem[0xC800 + i] == 0xFF);
    CHECK(r->chip.mem[0xC800 + 48] == 0xA5 && r->chip.mem[0xC7FF] == 0xA5);
    check_pins(r);
    free(r);
}

static void test_locked(void) {
    rig_t *r = malloc(sizeof *r); rig_init(r, TCK_BYTE_FAST_NS, false);
    r->chip.locked = true;
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    CHECK(r->ctx.id_len == 1 && r->ctx.id[0] == 0x98);     /* the JTAG ID still answers: locked, not silent */
    r->ctx.mem = PROG_MEM_FLASH; r->ctx.addr = 0xC400; r->ctx.len = 32;
    CHECK(run(PROTO.read, &r->ctx, NULL) == PROG_E_LOCKED);
    CHECK(run(PROTO.write, &r->ctx, NULL) == PROG_E_LOCKED);
    r->ctx.erase = PROG_ERASE_PAGE;
    CHECK(run(PROTO.erase, &r->ctx, NULL) == PROG_E_LOCKED);
    CHECK(r->chip.mem[0xC400] == 0xA5);

    /* chip erase: the mailbox in the RST-low entry, then the chip is open again */
    r->ctx.erase = PROG_ERASE_CHIP;
    CHECK(run(PROTO.erase, &r->ctx, NULL) == PROG_OK);
    CHECK(r->chip.got_erase_msg && r->chip.erases == 1 && !r->chip.locked);
    CHECK(r->chip.mem[0xC400] == 0xFF && r->chip.mem[0xFFFF] == 0xFF);
    CHECK(r->ctx.id_len == 3 && r->ctx.id[1] == 0x40);
    r->ctx.erase = PROG_ERASE_NONE;
    CHECK(run(PROTO.write, &r->ctx, NULL) == PROG_OK);
    CHECK(memcmp(&r->chip.mem[0xC400], r->img, 32) == 0);
    CHECK(r->chip.blocked_writes == 0);                    /* protection lifted again after the BOR */
    CHECK(r->sim.contention == 0 && r->chip.overlap == 0 && r->chip.undriven == 0 && r->chip.simultaneous == 0);
    CHECK(g_wm.low_flush == 0 && r->chip.deactivated == 0 && r->chip.max_low_ns <= LOW_LIMIT_NS);
    free(r);

    /* a chip erase on a chip that is not locked also works and leaves it unlocked */
    r = malloc(sizeof *r); rig_init(r, TCK_BYTE_FAST_NS, false);
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->ctx.erase = PROG_ERASE_CHIP;
    CHECK(run(PROTO.erase, &r->ctx, NULL) == PROG_OK && r->chip.erases == 1);
    free(r);
}

static void test_no_answer(void) {
    rig_t *r = malloc(sizeof *r); rig_init(r, TCK_BYTE_FAST_NS, false);
    r->chip.silent = true;
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_E_NO_ANSWER);
    CHECK(r->sim.contention == 0 && g_wm.low_flush == 0);
    free(r);

    /* flash part (JTAG ID 0x89): answers, but not supported here */
    r = malloc(sizeof *r); rig_init(r, TCK_BYTE_FAST_NS, false);
    r->chip.jid = 0x89;
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_E_UNSUPPORTED && r->ctx.id[0] == 0x89);
    free(r);

    /* the chip never syncs */
    r = malloc(sizeof *r); rig_init(r, TCK_BYTE_FAST_NS, false);
    r->chip.sync_polls = 9999;
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_E_TIMEOUT);
    free(r);
}

static void test_link_rate(void) {
    /* The fast SPI link: a TDO slot with a read is 2 x 2.05 us = 4.1 us low: inside 7 us (docs 14.1). */
    /* The boot link: 2 x 4.1 us = 8.2 us: over SLAU320's 7 us, so a model that enforces it loses the chip... */
    rig_t *r = malloc(sizeof *r); rig_init(r, TCK_BYTE_BOOT_NS, false);
    CHECK(run(PROTO.enter, &r->ctx, NULL) != PROG_OK);
    CHECK(r->chip.deactivated > 0);
    free(r);
    /* ...and works where the datasheets' 15 us applies */
    r = malloc(sizeof *r); rig_init(r, TCK_BYTE_BOOT_NS, false);
    r->chip.low_limit_ns = 15000;
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    CHECK(r->chip.deactivated == 0 && r->chip.max_low_ns == 2u * TCK_BYTE_BOOT_NS);
    CHECK(g_wm.low_flush == 0 && r->chip.overlap == 0 && r->chip.simultaneous == 0);
    free(r);
}

static void test_leave(void) {
    rig_t *r = malloc(sizeof *r); rig_init(r, TCK_BYTE_FAST_NS, false);
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    PROTO.leave(&r->ctx, true);
    CHECK(r->chip.releases == 1);                          /* IR_CNTRL_SIG_RELEASE after the reset */
    CHECK(r->chip.por_count >= 2);
    CHECK(r->sim.clk == 0);                                /* TEST ends low: high would re-enable SBW */
    CHECK(r->sim.ns - r->sim.clk_low_since_ns >= EXIT_NS); /* ...for more than 100 us (tSBW,Ret) */
    CHECK(!r->sim.pod_drives);                             /* RST released */
    CHECK(r->chip.overlap == 0 && r->chip.undriven == 0 && r->chip.simultaneous == 0);
    free(r);

    /* leave without run: the chip stays in reset (RST held low) */
    r = malloc(sizeof *r); rig_init(r, TCK_BYTE_FAST_NS, false);
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    PROTO.leave(&r->ctx, false);
    CHECK(r->chip.releases == 0);
    CHECK(r->sim.clk == 0 && r->sim.pod_drives && r->sim.pod_data == 0);
    CHECK(r->sim.ns - r->sim.clk_low_since_ns >= EXIT_NS);
    free(r);

    /* leave after a failed enter: pins released, TEST low */
    r = malloc(sizeof *r); rig_init(r, TCK_BYTE_FAST_NS, false);
    r->chip.silent = true;
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_E_NO_ANSWER);
    PROTO.leave(&r->ctx, true);
    CHECK(r->sim.clk == 0 && !r->sim.pod_drives);
    free(r);
}

int main(void) {
    CHECK(strcmp(PROTO.name, "sbw") == 0);
    CHECK(!PROTO.uart && !PROTO.uses_nrst);
    CHECK((PROTO.mems & (1u << PROG_MEM_FLASH)) && (PROTO.mems & (1u << PROG_MEM_CONFIG)));
    test_enter();
    test_tclk_edges();
    test_write_read();
    test_verify_fail();
    test_erase_range();
    test_locked();
    test_no_answer();
    test_link_rate();
    test_leave();
    printf("test_prog_sbw: %s\n", failures ? "FAILED" : "ok");
    return failures ? 1 : 0;
}
