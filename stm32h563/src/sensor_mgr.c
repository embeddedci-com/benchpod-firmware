/* ============================================================================
 * sensor_mgr.c — the emulated-sensor manager (sensor_mgr.h).
 *
 * Pure C on top of three seams: the engine (sensor_def.h), the emulation gateware (fpga_emu.h)
 * and the pod hooks (sensor_port.h), so the host tests run it end to end against a fake FPGA.
 * All state is on the heap while it is in use; the static part is the slot table and the upload
 * and stimulus bookkeeping.
 *
 * Register images: the manager keeps `live`, its copy of the 256 bytes the DUT reads. The DUT's
 * writes come back through the gateware's write log (32 entries per target, addressed by the
 * target's wr_count): each one goes into `live` (unless the byte is read-only), then through
 * sd_regs_after_write, and the bytes the engine changed are loaded back as runs of changed bytes
 * only, so a byte the DUT writes between the log read and the load is never overwritten. More
 * than 32 writes between two polls lose entries: the image is read back whole and rebuilt.
 * ========================================================================== */
#include "sensor_mgr.h"
#include "sensor_port.h"
#include "fpga_emu.h"
#include "la_pins.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define STATUS_PERIOD_US   2000u     /* write-log watch per target */
#define TICK_PERIOD_US     100000u   /* time-driven rebuild (expressions of t, states) */
#define CATCHUP_MAX_US     50000u    /* a timer this far behind restarts instead of bursting */

typedef struct {
    bool        on;
    bool        stim;        /* switched by the stimulus (its index in stim_idx) */
    uint8_t     stim_idx;
    uint64_t    end_us;      /* 0 = until cleared */
    sd_fault_t  f;
} sm_fault_t;

typedef struct {
    uint8_t  stage[SM_UART_STAGE];   /* bytes going out of UART2 */
    uint16_t len, off;
    uint8_t  epoch[SM_UART_STAGE];   /* the epoch being built */
    uint8_t  ans[SD_MAX_MATCH * 4];  /* an answer waiting for its delay */
    uint16_t ans_len;
    bool     ans_pending;
    uint64_t ans_due_us;
    uint32_t sent, received, messages, answers, overruns, dropped, late;
    uint32_t baud_now;
    uint64_t garbage_next_us;
    uint32_t rng;
} sm_uart_t;

typedef struct {
    char       name[SM_NAME_MAX + 1];
    sd_kind_t  kind;
    bool       running;
    uint8_t   *prog;
    uint32_t   prog_len;
    sd_inst_t *in;
    /* i2c / spi */
    uint8_t   *live;          /* 256 bytes */
    uint8_t    ro[SD_REGIMAGE / 8];
    uint8_t    target;        /* gateware target 0..3, 4 = spi */
    uint8_t    addr7;
    uint8_t    wr_seen;
    uint32_t   writes, log_lost, rebuilds;
    int16_t    last_at;
    uint8_t    last_val;
    bool       dirty;
    uint64_t   next_status_us, next_tick_us;
    /* data ready */
    bool       drdy_pin;
    uint8_t    drdy_on;       /* 0 conversion, 1 sample, 2 rate */
    double     drdy_hz;
    bool       drdy_en, drdy_changed;
    uint64_t   drdy_next_us;
    uint32_t   drdy_count;
    /* fifo */
    bool       has_fifo, fifo_en;
    double     fifo_hz;
    uint8_t    fifo_len;
    uint16_t   fifo_cap;      /* bytes */
    uint64_t   fifo_next_us;
    uint32_t   fifo_frames, fifo_drops;
    unsigned   fifo_level_frames;
    /* uart */
    sm_uart_t *u;
    uint32_t   baud;
    /* analog */
    sm_out_t   out;
    double     out_max, out_last;
    bool       out_set;
    /* common */
    sm_pins_t  pins;
    uint64_t   t0_us;
    sm_fault_t faults[SM_FAULTS];
} sm_inst_t;

static sm_inst_t *s_slot[SM_MAX_INST];
static bool s_rearm;

/* one upload at a time */
static struct {
    bool     active, stim;
    char     inst[SM_NAME_MAX + 1];
    uint8_t *buf;
    uint32_t total, got;
    uint64_t last_us;
} s_up;

/* the stimulus */
static struct {
    uint8_t  *prog;
    uint32_t  len;
    sd_stim_t st;
    bool      running;
    uint64_t  t0_us, next_us;
    double    rate_hz, t_s;
    int8_t    sig_slot[SD_STIM_SIGNALS], sig_param[SD_STIM_SIGNALS];
    int8_t    fault_slot[SD_STIM_FAULTS];
} s_stim;

/* ---------------------------------------------------------------------------------------------
 * helpers
 * ------------------------------------------------------------------------------------------- */

static void seterr(char *err, size_t cap, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
#include <stdarg.h>
static void seterr(char *err, size_t cap, const char *fmt, ...) {
    if (!err || !cap) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(err, cap, fmt, ap);
    va_end(ap);
}

static const char *kind_name(sd_kind_t k) {
    switch (k) {
    case SD_KIND_I2C: return "i2c";
    case SD_KIND_SPI: return "spi";
    case SD_KIND_UART: return "uart";
    case SD_KIND_ANALOG: return "analog";
    default: return "none";
    }
}

/* An unsigned 64-bit value in decimal (newlib-nano printf has no %lld). */
static void u64_dec(unsigned long long v, char *out, size_t cap, unsigned min_digits) {
    char t[24];
    unsigned n = 0;
    do { t[n++] = (char)('0' + (int)(v % 10u)); v /= 10u; } while (v && n < sizeof(t));
    while (n < min_digits && n < sizeof(t)) t[n++] = '0';
    size_t i = 0;
    while (n && i + 1 < cap) out[i++] = t[--n];
    out[i] = '\0';
}

/* A double as JSON: integers as integers, else up to 6 decimals, trailing zeros cut
   (newlib-nano printf has neither %f nor %lld). NaN/inf become null. */
static void emit_num(bp_emit_t *e, double v) {
    if (!(v == v) || v > 9.0e15 || v < -9.0e15) { bp_emit_raw(e, "null"); return; }
    bool neg = v < 0;
    double a = neg ? -v : v;
    unsigned long long whole = (unsigned long long)floor(a);
    unsigned long long frac = (unsigned long long)llround((a - (double)whole) * 1e6);
    if (frac >= 1000000u) { whole++; frac -= 1000000u; }
    char w[24], f[24];
    u64_dec(whole, w, sizeof(w), 1);
    if (frac == 0) { bp_emit(e, "%s%s", (neg && whole) ? "-" : "", w); return; }
    u64_dec(frac, f, sizeof(f), 6);
    for (int i = 5; i > 0 && f[i] == '0'; i--) f[i] = '\0';
    bp_emit(e, "%s%s.%s", neg ? "-" : "", w, f);
}

static void emit_strn(bp_emit_t *e, const char *s, size_t n) {
    char tmp[136];
    if (n >= sizeof(tmp)) n = sizeof(tmp) - 1;
    memcpy(tmp, s, n);
    tmp[n] = '\0';
    bp_emit_jstr(e, tmp);
}

static bool name_ok(const char *s) {
    size_t n = s ? strlen(s) : 0;
    if (n == 0 || n > SM_NAME_MAX) return false;
    for (size_t i = 0; i < n; i++) {
        char c = s[i];
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_')) return false;
    }
    return true;
}

static int find_slot(const char *name) {
    for (unsigned i = 0; i < SM_MAX_INST; i++)
        if (s_slot[i] && strcmp(s_slot[i]->name, name) == 0) return (int)i;
    return -1;
}

const char *sm_slot_name(unsigned slot) {
    return (slot < SM_MAX_INST && s_slot[slot]) ? s_slot[slot]->name : NULL;
}

static bool is_regs(const sm_inst_t *s) { return s->kind == SD_KIND_I2C || s->kind == SD_KIND_SPI; }
static bool is_digital(const sm_inst_t *s) { return s->kind != SD_KIND_ANALOG; }

static void set_time(sm_inst_t *s, uint64_t now) {
    sd_set_time_ms(s->in, (int64_t)((now - s->t0_us) / 1000u));
}

static uint32_t xorshift(uint32_t *st) {
    uint32_t x = *st ? *st : 0x9E3779B9u;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *st = x;
    return x;
}

static void slot_text(const sm_inst_t *s, char *out, size_t cap) {
    switch (s->kind) {
    case SD_KIND_I2C: snprintf(out, cap, "i2c%u", (unsigned)s->target); break;
    case SD_KIND_SPI: snprintf(out, cap, "spi0"); break;
    case SD_KIND_UART: snprintf(out, cap, "uart0"); break;
    case SD_KIND_ANALOG: snprintf(out, cap, "analog%u", s->out == SM_OUT_CURRENT ? 1u : 0u); break;
    default: snprintf(out, cap, "-"); break;
    }
}

static void engine_err(char *err, size_t cap, const char *what, const sd_inst_t *in) {
    char m[160];
    const sd_err_t *e = sd_last_error(in);
    if (e && e->code) sd_err_format(e, m, sizeof(m));
    else snprintf(m, sizeof(m), "evaluation failed");
    seterr(err, cap, "%s: %s", what, m);
}

/* ---------------------------------------------------------------------------------------------
 * pins
 * ------------------------------------------------------------------------------------------- */

typedef struct { uint8_t la; la_fn_t fn; } pin_use_t;

static unsigned pin_list(const sm_inst_t *s, pin_use_t out[8]) {
    unsigned n = 0;
    const sm_pins_t *p = &s->pins;
    switch (s->kind) {
    case SD_KIND_I2C:
        out[n++] = (pin_use_t){ p->sda, LA_FN_I2C_SDA };
        out[n++] = (pin_use_t){ p->scl, LA_FN_I2C_SCL };
        if (p->drdy) out[n++] = (pin_use_t){ p->drdy, LA_FN_SENSOR_DRDY };
        break;
    case SD_KIND_SPI:
        out[n++] = (pin_use_t){ p->sclk, LA_FN_SENSOR_SCK };
        out[n++] = (pin_use_t){ p->mosi, LA_FN_SENSOR_MOSI };
        out[n++] = (pin_use_t){ p->miso, LA_FN_SENSOR_MISO };
        out[n++] = (pin_use_t){ p->cs, LA_FN_SENSOR_CS };
        break;
    case SD_KIND_UART:
        out[n++] = (pin_use_t){ p->tx, LA_FN_SENSOR_TX };
        if (p->rx) out[n++] = (pin_use_t){ p->rx, LA_FN_SENSOR_RX };
        break;
    default:
        break;
    }
    return n;
}

/* May slot `me` share an I2C line `la` that already carries `fn`? Only with I2C instances on
   exactly the same SDA/SCL pair. */
static bool i2c_share_ok(unsigned me, const sm_inst_t *s, unsigned la, la_fn_t fn) {
    if (fn != LA_FN_I2C_SDA && fn != LA_FN_I2C_SCL) return false;
    if (la_pins_fn(la) != fn) return false;
    uint8_t own = la_pins_owner(la);
    if (!own) return false;
    for (unsigned i = 0; i < SM_MAX_INST; i++) {
        if (!(own & (1u << i)) || i == me) continue;
        const sm_inst_t *o = s_slot[i];
        if (!o || o->kind != SD_KIND_I2C || o->pins.sda != s->pins.sda || o->pins.scl != s->pins.scl)
            return false;
    }
    return true;
}

static bool pins_check(unsigned me, const sm_inst_t *s, char *err, size_t cap) {
    pin_use_t p[8];
    unsigned n = pin_list(s, p);
    for (unsigned i = 0; i < n; i++) {
        if (p[i].la < 1 || p[i].la > LA_PINS_COUNT) {
            seterr(err, cap, "pin %s: LA%u is not a pin (use 1..14)", la_fn_name(p[i].fn), p[i].la);
            return false;
        }
        for (unsigned j = 0; j < i; j++)
            if (p[j].la == p[i].la) {
                seterr(err, cap, "pins %s and %s are both LA%u", la_fn_name(p[j].fn), la_fn_name(p[i].fn), p[i].la);
                return false;
            }
        if (i2c_share_ok(me, s, p[i].la, p[i].fn)) continue;
        char m[LA_PINS_ERR_MAX];
        if (!la_pins_check_claim(p[i].fn, LA_GPIO_NONE, &p[i].la, 1, 0, sensor_port_pull_mask(), m, sizeof(m))) {
            seterr(err, cap, "%s", m);
            return false;
        }
    }
    return true;
}

static void pins_claim(unsigned me, const sm_inst_t *s) {
    pin_use_t p[8];
    unsigned n = pin_list(s, p);
    for (unsigned i = 0; i < n; i++) {
        uint8_t own = la_pins_owner(p[i].la);
        if (!(la_pins_fn(p[i].la) == p[i].fn && own && la_fn_is_sensor(p[i].fn)))
            { la_pins_claim(p[i].fn, LA_GPIO_NONE, 0, &p[i].la, 1); own = 0; }
        la_pins_set_owner(p[i].la, (uint8_t)(own | (1u << me)));
    }
}

static void pins_release(unsigned me, const sm_inst_t *s) {
    pin_use_t p[8];
    unsigned n = pin_list(s, p);
    for (unsigned i = 0; i < n; i++) {
        if (!la_fn_is_sensor(la_pins_fn(p[i].la))) continue;
        uint8_t own = (uint8_t)(la_pins_owner(p[i].la) & ~(1u << me));
        if (own) la_pins_set_owner(p[i].la, own);
        else la_pins_release(p[i].la);
    }
}

/* Every emulated output pin, from the running instances (EMU_PINMAP). */
static void pinmap_update(void) {
    uint8_t codes[14] = { 0 };
    for (unsigned i = 0; i < SM_MAX_INST; i++) {
        const sm_inst_t *s = s_slot[i];
        if (!s || !s->running) continue;
        const sm_pins_t *p = &s->pins;
        if (s->kind == SD_KIND_I2C) {
            codes[p->sda - 1] = EMU_PIN_I2C;
            codes[p->scl - 1] = EMU_PIN_I2C;
            if (p->drdy) codes[p->drdy - 1] = EMU_PIN_DRDY(s->target);
        } else if (s->kind == SD_KIND_SPI) {
            codes[p->miso - 1] = EMU_PIN_SPI_MISO;
        } else if (s->kind == SD_KIND_UART) {
            codes[p->tx - 1] = EMU_PIN_UART2_TX;
        }
    }
    (void)fpga_emu_pinmap(codes);
}

/* ---------------------------------------------------------------------------------------------
 * faults: what the gateware or the manager does
 * ------------------------------------------------------------------------------------------- */

static bool fault_on(const sm_inst_t *s, sd_fault_type_t t, const sm_fault_t **which) {
    for (unsigned i = 0; i < SM_FAULTS; i++)
        if (s->faults[i].on && s->faults[i].f.type == t) { if (which) *which = &s->faults[i]; return true; }
    return false;
}

static bool engine_fault(sd_fault_type_t t) {
    return t == SD_FAULT_STUCK || t == SD_FAULT_OFFSET || t == SD_FAULT_GAIN || t == SD_FAULT_CORRUPT ||
           t == SD_FAULT_OPEN || t == SD_FAULT_SHORT;
}

/* ---------------------------------------------------------------------------------------------
 * register images
 * ------------------------------------------------------------------------------------------- */

static bool ro_bit(const sm_inst_t *s, unsigned a) { return (s->ro[a >> 3] >> (a & 7u)) & 1u; }

/* Load the changed bytes back, as runs, so bytes nobody changed are never rewritten. */
static int load_changed(sm_inst_t *s, const uint8_t changed[SD_REGIMAGE / 8], bool all) {
    if (all) return fpga_emu_load_regs(s->target, 0, s->live, SD_REGIMAGE);
    int rc = 0;
    unsigned a = 0;
    while (a < SD_REGIMAGE) {
        if (!((changed[a >> 3] >> (a & 7u)) & 1u)) { a++; continue; }
        unsigned b = a;
        while (b + 1 < SD_REGIMAGE && ((changed[(b + 1) >> 3] >> ((b + 1) & 7u)) & 1u)) b++;
        if (fpga_emu_load_regs(s->target, (uint8_t)a, &s->live[a], b - a + 1) != 0) rc = -1;
        a = b + 1;
    }
    return rc;
}

static void drdy_fifo_eval(sm_inst_t *s) {
    if (s->drdy_pin) {
        double hz = 0; bool en = true;
        if (sd_drdy_eval(s->in, s->live, &hz, &en) == SD_OK) {
            if (en && !s->drdy_en) s->drdy_next_us = 0;
            if (en != s->drdy_en) s->drdy_changed = true;
            s->drdy_hz = hz; s->drdy_en = en;
        }
    }
    if (s->has_fifo) {
        double hz = 0; bool en = true;
        if (sd_fifo_eval(s->in, s->live, &hz, &en) == SD_OK) {
            if (en && !s->fifo_en) s->fifo_next_us = 0;
            s->fifo_hz = hz; s->fifo_en = en;
        }
    }
}

static int regs_rebuild(sm_inst_t *s, uint64_t now) {
    uint8_t img[SD_REGIMAGE];
    sd_img_result_t r;
    set_time(s, now);
    if (s->has_fifo) sd_set_fifo_level(s->in, s->fifo_level_frames);
    if (sd_regs_rebuild(s->in, s->live, img, &r) != SD_OK) return -1;
    memcpy(s->live, img, SD_REGIMAGE);
    s->rebuilds++;
    s->dirty = false;
    int rc = (r.reset || r.n_changed) ? load_changed(s, r.changed, r.reset) : 0;
    drdy_fifo_eval(s);
    return rc;
}

/* The DUT's writes since the last poll, from the write log. */
static void regs_writes(sm_inst_t *s, uint64_t now) {
    uint8_t flags = 0, wr = 0;
    if (fpga_emu_status(s->target, &flags, &wr) != 0) return;
    uint8_t n = (uint8_t)(wr - s->wr_seen);
    if (n == 0) return;
    uint8_t img[SD_REGIMAGE];
    uint8_t changed[SD_REGIMAGE / 8] = { 0 };
    bool reset = false;
    sd_img_result_t r;
    set_time(s, now);
    if (s->has_fifo) sd_set_fifo_level(s->in, s->fifo_level_frames);
    if (n > FPGA_EMU_LOG_ENTRIES) {
        /* Entries were lost: take the whole image the DUT sees and run the rules once with
           `written` = -1 (they only know that something was written). */
        s->log_lost += (uint32_t)n - FPGA_EMU_LOG_ENTRIES;
        s->writes += n;
        s->wr_seen = wr;
        if (fpga_emu_read_regs(s->target, 0, s->live, SD_REGIMAGE) != 0) return;
        if (sd_regs_after_write(s->in, s->live, -1, -1, img, &r) != SD_OK) return;
        memcpy(s->live, img, SD_REGIMAGE);
        (void)load_changed(s, r.changed, r.reset);
        drdy_fifo_eval(s);
        return;
    }
    uint8_t pairs[2 * FPGA_EMU_LOG_ENTRIES];
    if (fpga_emu_read_log(s->target, (uint8_t)(s->wr_seen % FPGA_EMU_LOG_ENTRIES), n, pairs) != 0) return;
    s->wr_seen = wr;
    sd_bus_info_t bi;
    bool word = sd_bus_info(s->in, &bi) == SD_OK && bi.register_bytes == 2;
    for (unsigned k = 0; k < n; k++) {
        uint8_t a = pairs[2 * k], v = pairs[2 * k + 1];
        s->writes++;
        s->last_at = a;
        s->last_val = v;
        if (!ro_bit(s, a)) s->live[a] = v;
        /* word registers: the MSB and the LSB of one register are one write for the rules
           (they run once, at the LSB, with the whole word) */
        if (word && !(a & 1u) && k + 1 < n && pairs[2 * k + 2] == (uint8_t)(a + 1u)) {
            changed[a >> 3] |= (uint8_t)(1u << (a & 7u));
            continue;
        }
        if (sd_regs_after_write(s->in, s->live, a, v, img, &r) != SD_OK) continue;
        memcpy(s->live, img, SD_REGIMAGE);
        if (r.reset) reset = true;
        for (unsigned b = 0; b < SD_REGIMAGE / 8; b++) changed[b] |= r.changed[b];
        /* The DUT's byte goes back too: a load from an earlier poll (a reset's power-on image)
           can have overwritten it in the gateware after the DUT wrote it, when a driver writes
           its setup right after a soft reset. After every poll the gateware holds `live`. */
        changed[a >> 3] |= (uint8_t)(1u << (a & 7u));
    }
    (void)load_changed(s, changed, reset);
    s->rebuilds++;
    drdy_fifo_eval(s);
}

/* Attributes and conversion times from the definition's bus info and commands. */
static void regs_attrs(sm_inst_t *s, uint8_t attrs[SD_REGIMAGE], uint16_t times[FPGA_EMU_TIMES], unsigned *nt) {
    sd_bus_info_t bi;
    memset(attrs, 0, SD_REGIMAGE);
    memset(times, 0, FPGA_EMU_TIMES * sizeof(uint16_t));
    *nt = 0;
    memset(times, 0, FPGA_EMU_TIMES * sizeof(times[0]));
    if (sd_bus_info(s->in, &bi) != SD_OK) return;
    /* Slot 0 is the conversion's time; without a conversion all four slots go to commands. */
    times[0] = (uint16_t)(bi.conv_us > 0xFFFFu ? 0xFFFFu : bi.conv_us);
    if (bi.has_conversion) *nt = 1;
    if (bi.has_conversion) {
        attrs[bi.conv_trigger] |= EMU_ATTR_TRIG | EMU_ATTR_TIME(0);
        if (bi.conv_busy_mask) attrs[bi.conv_busy_at] |= EMU_ATTR_BUSY;
    }
    for (unsigned i = 0; i < bi.n_commands; i++) {
        sd_command_t c;
        if (sd_command(s->in, i, &c) != SD_OK || c.busy_us == 0) continue;   /* answers at once */
        uint16_t us = (uint16_t)(c.busy_us > 0xFFFFu ? 0xFFFFu : c.busy_us);
        unsigned idx = *nt;
        for (unsigned k = 0; k < *nt; k++) if (times[k] == us) { idx = k; break; }
        if (idx == *nt) {
            if (*nt < FPGA_EMU_TIMES) times[(*nt)++] = us;
            else {   /* more than 4 distinct times: the shortest one that is not shorter, else
                        the longest (a measurement never ends too early) */
                int best = -1;
                for (unsigned k = 0; k < FPGA_EMU_TIMES; k++)
                    if (times[k] >= us && (best < 0 || times[k] < times[best])) best = (int)k;
                if (best < 0) { best = 0; for (unsigned k = 1; k < FPGA_EMU_TIMES; k++) if (times[k] > times[best]) best = (int)k; }
                idx = (unsigned)best;
            }
        }
        attrs[c.at] |= EMU_ATTR_CMD | EMU_ATTR_TIME(idx);
    }
    if (bi.has_fifo) attrs[bi.fifo_at] |= EMU_ATTR_FIFO;
    if (bi.has_drdy && bi.drdy_level && bi.drdy_clear_on_read >= 0 && bi.drdy_clear_on_read < 256)
        attrs[bi.drdy_clear_on_read] |= EMU_ATTR_CLR;
    for (unsigned a = 0; a < SD_REGIMAGE; a++) if (ro_bit(s, a)) attrs[a] |= EMU_ATTR_RO;
    if (s->kind == SD_KIND_SPI)   /* the SPI target honours FIFO and RO only */
        for (unsigned a = 0; a < SD_REGIMAGE; a++) attrs[a] &= (EMU_ATTR_FIFO | EMU_ATTR_RO);
}

/* The I2C_CONFIG / SPIT_CONFIG of an instance, faults included. */
static int regs_config(sm_inst_t *s, bool enable) {
    sd_bus_info_t bi;
    if (sd_bus_info(s->in, &bi) != SD_OK) return -1;
    bool mute = fault_on(s, SD_FAULT_NO_RESPONSE, NULL);
    if (s->kind == SD_KIND_SPI) {
        uint8_t f = enable ? EMU_SPI_ENABLE : 0;
        if (bi.spi_mode & 1u) f |= EMU_SPI_CPHA;
        if (bi.spi_mode & 2u) f |= EMU_SPI_CPOL;
        if (mute) f |= EMU_SPI_MUTE;
        f |= EMU_SPI_DUMMY(bi.dummy_bytes);
        uint8_t rb = (uint8_t)((bi.read_bit & 7u) | (bi.read_val ? 0x80u : 0u));
        uint8_t ib = bi.has_inc_bit ? (uint8_t)((bi.inc_bit & 7u) | (bi.inc_val ? 0x80u : 0u) | 0x40u) : 0u;
        return fpga_emu_spi_config((uint8_t)(s->pins.cs - 1), (uint8_t)(s->pins.sclk - 1),
                                   (uint8_t)(s->pins.mosi - 1), f, rb, ib, bi.address_mask);
    }
    if (!enable) return fpga_emu_i2c_disable(s->target);
    /* Commands with busy_us are not acknowledged while busy (format 3.1); a definition with only
       commands and no conversion block therefore runs in nack mode. */
    uint8_t wb = bi.while_busy;
    if (!bi.has_conversion && bi.n_commands) wb = 1u;
    uint8_t f = EMU_I2C_ENABLE | (uint8_t)((wb & 3u) << 2);
    if (bi.command_map) f |= EMU_I2C_CMAP;            /* two pointer bytes, no page */
    else if (bi.addr_bits == 16) f |= EMU_I2C_ADDR16;
    if (bi.register_bytes == 2) f |= EMU_I2C_WORD;
    if (mute) f |= EMU_I2C_MUTE;
    if (fault_on(s, SD_FAULT_BUS_STUCK, NULL)) f |= EMU_I2C_BUS_STUCK;
    /* With its `enabled` condition false the pin stays configured (so it holds its inactive
       level) but never asserts: no assert at the end of a conversion, and the firmware sends no
       asserts (drdy_poll, fifo_poll and the stimulus check drdy_en). */
    uint8_t drdy = 0;
    if (s->drdy_pin && bi.has_drdy) {
        drdy = bi.drdy_level ? EMU_DRDY_LEVEL : EMU_DRDY_PULSE;
        if (bi.drdy_active_high) drdy |= EMU_DRDY_ACTIVE_HIGH;
        if (bi.drdy_on == 0 && s->drdy_en) drdy |= EMU_DRDY_ON_CONV;
    }
    return fpga_emu_i2c_config(s->target, s->addr7, (uint8_t)(s->pins.sda - 1), (uint8_t)(s->pins.scl - 1),
                               f, (uint8_t)(bi.page >> 8), bi.conv_busy_mask, drdy);
}

/* Configure the gateware for an instance: image, attributes, times, config. */
static int regs_arm(sm_inst_t *s, bool power_on) {
    if (power_on) {
        sd_img_result_t r;
        set_time(s, sensor_port_now_us());
        if (sd_regs_power_on(s->in, s->live, &r) != SD_OK) return -1;
    }
    uint8_t attrs[SD_REGIMAGE];
    uint16_t times[FPGA_EMU_TIMES];
    unsigned nt = 0;
    regs_attrs(s, attrs, times, &nt);
    drdy_fifo_eval(s);
    if (s->kind == SD_KIND_I2C) (void)fpga_emu_i2c_disable(s->target);
    int rc = fpga_emu_load_regs(s->target, 0, s->live, SD_REGIMAGE);
    if (rc == 0) rc = fpga_emu_load_attrs(s->target, 0, attrs, SD_REGIMAGE);
    if (rc == 0 && s->kind == SD_KIND_I2C)
        for (unsigned i = 0; i < FPGA_EMU_TIMES && rc == 0; i++) rc = fpga_emu_times(s->target, (uint8_t)i, times[i]);
    if (rc == 0 && s->drdy_pin) {
        sd_bus_info_t bi;
        if (sd_bus_info(s->in, &bi) == SD_OK && bi.drdy_pulse_us)
            rc = fpga_emu_config((uint8_t)(bi.drdy_pulse_us > 255u ? 255u : bi.drdy_pulse_us), false);
    }
    if (rc == 0 && s->has_fifo) rc = fpga_emu_config(50, true);   /* an empty FIFO for its user */
    if (rc == 0) rc = regs_config(s, true);
    uint8_t flags = 0, wr = 0;
    if (rc == 0) rc = fpga_emu_status(s->target, &flags, &wr);
    s->wr_seen = wr;
    s->next_status_us = s->next_tick_us = 0;
    return rc;
}

/* ---------------------------------------------------------------------------------------------
 * UART
 * ------------------------------------------------------------------------------------------- */

static uint32_t uart_baud_now(const sm_inst_t *s) {
    const sm_fault_t *f;
    double b = (double)s->baud;
    if (fault_on(s, SD_FAULT_BAUD_ERROR, &f) && f->f.has_value) b *= 1.0 + f->f.value / 100.0;
    if (b < 50) b = 50;
    return (uint32_t)llround(b);
}

static int uart_arm(sm_inst_t *s) {
    s->u->baud_now = uart_baud_now(s);
    s->u->len = s->u->off = 0;
    s->u->ans_pending = false;
    return fpga_emu_uart2_config(s->pins.rx ? (uint8_t)(s->pins.rx - 1) : 0xFFu, (uint8_t)(s->pins.tx - 1),
                                 s->u->baud_now, true);
}

/* pod -> DUT: the staged bytes into UART2's FIFO, a FIFO's worth whenever it is empty. */
static void uart_pump(sm_uart_t *u) {
    uint8_t flags = 0;
    if (u->off < u->len && fpga_emu_uart2_status(NULL, &flags) == 0 && (flags & EMU_U2_TX_EMPTY)) {
        size_t n = (size_t)(u->len - u->off);
        if (n > FPGA_EMU_UART2_FIFO) n = FPGA_EMU_UART2_FIFO;
        if (fpga_emu_uart2_write(&u->stage[u->off], n) == 0) {
            u->off = (uint16_t)(u->off + n);
            u->sent += (uint32_t)n;
        }
    }
}

static void uart_poll(sm_inst_t *s, uint64_t now) {
    sm_uart_t *u = s->u;
    int64_t now_ms = (int64_t)((now - s->t0_us) / 1000u);
    bool mute = fault_on(s, SD_FAULT_NO_RESPONSE, NULL);
    const sm_fault_t *gf = NULL, *df = NULL;
    bool garbage = fault_on(s, SD_FAULT_GARBAGE, &gf);
    bool drop = fault_on(s, SD_FAULT_DROP, &df);
    if (u->baud_now != uart_baud_now(s)) (void)uart_arm(s);   /* baud_error on/off */

    /* DUT -> pod: requests and their answers */
    uint16_t avail = 0;
    uint8_t flags = 0;
    if (s->pins.rx && fpga_emu_uart2_status(&avail, &flags) == 0) {
        if (flags & EMU_U2_RX_OVF) u->overruns++;
        if (avail) {
            uint8_t rx[FPGA_EMU_UART2_FIFO];
            if (fpga_emu_uart2_read(rx, avail) == 0) {
                for (unsigned i = 0; i < avail; i++) {
                    u->received++;
                    size_t len = 0;
                    uint32_t delay = 0;
                    int r = sd_uart_rx(s->in, now_ms, rx[i], u->ans, sizeof(u->ans), &len, &delay);
                    if (r == 1 && !mute) {
                        if (u->ans_pending) u->overruns++;
                        u->ans_len = (uint16_t)len;
                        u->ans_pending = true;
                        u->ans_due_us = now + (uint64_t)delay * 1000u;
                    }
                }
            }
        }
    }
    if (u->ans_pending && now >= u->ans_due_us && u->off >= u->len) {
        memcpy(u->stage, u->ans, u->ans_len);
        u->len = u->ans_len;
        u->off = 0;
        u->ans_pending = false;
        u->answers++;
    }

    /* the messages, at their times: first finish what is staged, so a poll that came late (a
       capture held the loop) sends the waiting epoch instead of skipping the next one */
    uart_pump(u);
    int64_t due = sd_uart_next_ms(s->in);
    if (due >= 0 && now_ms >= due) {
        size_t len = 0;
        int64_t at = 0;
        set_time(s, now);
        int erc = sd_uart_epoch(s->in, (char *)u->epoch, sizeof(u->epoch), &len, &at);
        /* A late poll (a capture held the loop) finds several epochs due: send the newest, as a
           receiver that was busy would, instead of a burst that overruns the FIFO. */
        for (int64_t nx = sd_uart_next_ms(s->in); erc == SD_OK && nx >= 0 && now_ms >= nx;
             nx = sd_uart_next_ms(s->in)) {
            erc = sd_uart_epoch(s->in, (char *)u->epoch, sizeof(u->epoch), &len, &at);
            u->late++;
        }
        if (erc == SD_OK && len) {
            bool dropped = mute || garbage ||
                           (drop && df->f.has_value && (double)(xorshift(&u->rng) % 1000000u) < df->f.value * 1e6);
            if (dropped) u->dropped++;
            else if (u->off < u->len) u->overruns++;   /* still sending the last one */
            else {
                memcpy(u->stage, u->epoch, len);
                u->len = (uint16_t)len;
                u->off = 0;
                u->messages++;
            }
        }
    }

    /* garbage: random bytes at rate_hz instead of the messages */
    if (garbage && u->off >= u->len) {
        double hz = gf->f.rate_hz > 0 ? gf->f.rate_hz : 100.0;
        uint64_t period = (uint64_t)(1e6 / hz);
        if (period == 0) period = 1;
        if (u->garbage_next_us == 0 || now - u->garbage_next_us > CATCHUP_MAX_US) u->garbage_next_us = now;
        unsigned n = 0;
        while (u->garbage_next_us <= now && n < sizeof(u->stage)) {
            u->stage[n++] = (uint8_t)xorshift(&u->rng);
            u->garbage_next_us += period;
        }
        u->len = (uint16_t)n;
        u->off = 0;
    }

    /* pod -> DUT */
    uart_pump(u);
}

/* ---------------------------------------------------------------------------------------------
 * analog
 * ------------------------------------------------------------------------------------------- */

static int analog_apply(sm_inst_t *s, bool force, char *err, size_t cap) {
    double v = 0;
    if (sd_analog_eval(s->in, 0.0, &v) != SD_OK) { engine_err(err, cap, "analog value", s->in); return -1; }
    s->dirty = false;
    if (!force && s->out_set && fabs(v - s->out_last) < 1e-9) return 0;
    char m[SM_ERR_MAX];
    if (sensor_port_analog_set(s->out, v, s->out_max, m, sizeof(m)) != 0) { seterr(err, cap, "%s", m); return -1; }
    s->out_last = v;
    s->out_set = true;
    return 0;
}

/* ---------------------------------------------------------------------------------------------
 * instance life cycle
 * ------------------------------------------------------------------------------------------- */

static void inst_free(unsigned i) {
    sm_inst_t *s = s_slot[i];
    if (!s) return;
    sensor_port_free(s->u);
    sensor_port_free(s->live);
    sensor_port_free(s->in);
    sensor_port_free(s->prog);
    sensor_port_free(s);
    s_slot[i] = NULL;
}

/* Stop the hardware side of a running instance and release its pins. */
static void inst_halt(unsigned i, bool touch_hw) {
    sm_inst_t *s = s_slot[i];
    if (!s || !s->running) return;
    s->running = false;
    if (touch_hw) {
        if (is_regs(s)) {
            (void)regs_config(s, false);
            if (s->has_fifo) (void)fpga_emu_config(50, true);
        } else if (s->kind == SD_KIND_UART) {
            (void)fpga_emu_uart2_config(0xFFu, (uint8_t)(s->pins.tx - 1), s->baud ? s->baud : 9600u, false);
        } else if (s->kind == SD_KIND_ANALOG && s->out_set) {
            sensor_port_analog_release(s->out);
        }
    }
    s->out_set = false;
    pins_release(i, s);
    if (touch_hw && is_digital(s)) pinmap_update();
    printf("[sensor] %s stopped\n", s->name);
}

/* The stimulus refers to instances by slot: drop it when one of them goes. */
static void stim_forget_slot(unsigned i) {
    if (!s_stim.running) return;
    for (unsigned k = 0; k < SD_STIM_SIGNALS; k++)
        if (s_stim.sig_slot[k] == (int8_t)i) { s_stim.running = false; break; }
    for (unsigned k = 0; k < SD_STIM_FAULTS; k++)
        if (s_stim.fault_slot[k] == (int8_t)i) { s_stim.running = false; break; }
}

static void upload_drop(void) {
    sensor_port_free(s_up.buf);
    memset(&s_up, 0, sizeof(s_up));
}

void sm_reset(void) {
    for (unsigned i = 0; i < SM_MAX_INST; i++) { inst_halt(i, false); inst_free(i); }
    upload_drop();
    sensor_port_free(s_stim.prog);
    memset(&s_stim, 0, sizeof(s_stim));
    s_rearm = false;
    la_pins_set_owner_namer(sm_slot_name);
}

/* ---------------------------------------------------------------------------------------------
 * uploads
 * ------------------------------------------------------------------------------------------- */

uint32_t sm_upload_received(void) { return s_up.got; }

/* 0 = taken, 1 = complete (s_up.buf holds total bytes), <0 = error. */
static int upload_chunk(bool stim, const char *inst, uint32_t offset, uint32_t total, const uint8_t *data,
                        size_t n, uint32_t max, char *err, size_t cap) {
    if (total == 0 || total > max) { seterr(err, cap, "total must be 1..%lu bytes", (unsigned long)max); return -1; }
    if (offset == 0) {
        upload_drop();
        s_up.buf = sensor_port_alloc(total + 1u);
        if (!s_up.buf) {
            seterr(err, cap, "out of memory: the upload needs %lu bytes, %lu free",
                   (unsigned long)total + 1u, (unsigned long)sensor_port_heap_free());
            return -1;
        }
        s_up.active = true;
        s_up.stim = stim;
        snprintf(s_up.inst, sizeof(s_up.inst), "%s", inst ? inst : "");
        s_up.total = total;
    } else if (!s_up.active || s_up.stim != stim || strcmp(s_up.inst, inst ? inst : "") != 0 ||
               s_up.total != total || offset != s_up.got) {
        seterr(err, cap, "chunk at offset %lu, expected %lu (start again at offset 0)", (unsigned long)offset,
               (unsigned long)((s_up.active && s_up.stim == stim) ? s_up.got : 0u));
        if (s_up.active && s_up.stim == stim && strcmp(s_up.inst, inst ? inst : "") == 0) upload_drop();
        return -1;
    }
    if ((uint64_t)offset + n > total) { seterr(err, cap, "chunk runs past total (%lu bytes)", (unsigned long)total); upload_drop(); return -1; }
    memcpy(s_up.buf + offset, data, n);
    s_up.got = offset + (uint32_t)n;
    s_up.last_us = sensor_port_now_us();
    if (s_up.got < total) return 0;
    s_up.buf[total] = '\0';
    return 1;
}

static void emit_params(bp_emit_t *e, sd_inst_t *in) {
    unsigned np = sd_n_params(in);
    bp_emit_raw(e, ",\"params\":[");
    for (unsigned i = 0; i < np; i++) {
        sd_param_info_t pi;
        if (sd_param_info(in, i, &pi) != SD_OK) continue;
        /* the reply must stay whole: cut the list (and say so) before it would not fit */
        if (e->cap - e->len < 160u) { bp_emit_raw(e, "],\"params_cut\":true"); return; }
        bp_emit_raw(e, i ? ",{\"key\":" : "{\"key\":");
        emit_strn(e, pi.key, pi.key_len);
        bp_emit_raw(e, ",\"unit\":");
        emit_strn(e, pi.unit ? pi.unit : "", pi.unit ? pi.unit_len : 0);
        bp_emit_raw(e, ",\"min\":"); emit_num(e, pi.min);
        bp_emit_raw(e, ",\"max\":"); emit_num(e, pi.max);
        bp_emit_raw(e, ",\"default\":"); emit_num(e, pi.def);
        bp_emit_raw(e, "}");
    }
    bp_emit_raw(e, "]");
}

int sm_define_chunk(const char *inst, uint32_t offset, uint32_t total, const uint8_t *data, size_t n,
                    bool dry_run, bp_emit_t *e, char *err, size_t cap) {
    la_pins_set_owner_namer(sm_slot_name);
    if (!name_ok(inst)) { seterr(err, cap, "instance must be 1..%u characters of a-z, 0-9 and _", SM_NAME_MAX); return -1; }
    int rc = upload_chunk(false, inst, offset, total, data, n, SD_JSON_MAX, err, cap);
    if (rc <= 0) return rc;

    /* compile: the program and the compiler's scratch only for this call */
    void *prog = sensor_port_alloc(SD_PROG_MAX);
    void *work = sensor_port_alloc(SD_COMPILE_WORK);
    sd_inst_t *in = sensor_port_alloc(sizeof(sd_inst_t));
    if (!prog || !work || !in) {
        sensor_port_free(prog); sensor_port_free(work); sensor_port_free(in);
        seterr(err, cap, "out of memory: compiling needs %lu bytes, %lu free",
               (unsigned long)(SD_PROG_MAX + SD_COMPILE_WORK + sizeof(sd_inst_t)), (unsigned long)sensor_port_heap_free());
        upload_drop();
        return -1;
    }
    size_t plen = 0;
    sd_err_t se;
    rc = sd_compile((const char *)s_up.buf, total, prog, SD_PROG_MAX, work, SD_COMPILE_WORK, &plen, &se);
    sensor_port_free(work);
    upload_drop();
    if (rc != SD_OK) {
        char m[160];
        sd_err_format(&se, m, sizeof(m));
        seterr(err, cap, "invalid definition: %s", m);
        sensor_port_free(prog); sensor_port_free(in);
        return -1;
    }
    /* the program at its own size */
    uint8_t *p = sensor_port_alloc(plen);
    if (!p) {
        seterr(err, cap, "out of memory: the program needs %lu bytes", (unsigned long)plen);
        sensor_port_free(prog); sensor_port_free(in);
        return -1;
    }
    memcpy(p, prog, plen);
    sensor_port_free(prog);
    if (sd_inst_init(in, p, plen, &se) != SD_OK) {
        char m[160];
        sd_err_format(&se, m, sizeof(m));
        seterr(err, cap, "invalid definition: %s", m);
        sensor_port_free(p); sensor_port_free(in);
        return -1;
    }
    sd_kind_t kind = sd_kind(in);

    /* the reply */
    size_t nl = 0, ll = 0;
    const char *nm = sd_name(in, &nl), *lb = sd_label(in, &ll);
    bp_emit_raw(e, "\"instance\":");
    bp_emit_jstr(e, inst);
    bp_emit(e, ",\"kind\":\"%s\",\"name\":", kind_name(kind));
    emit_strn(e, nm, nl);
    bp_emit_raw(e, ",\"label\":");
    emit_strn(e, lb, ll);
    bp_emit(e, ",\"program_bytes\":%lu", (unsigned long)plen);
    if (dry_run) bp_emit_raw(e, ",\"dry_run\":true");
    if (kind == SD_KIND_I2C || kind == SD_KIND_SPI) {
        sd_bus_info_t bi;
        if (sd_bus_info(in, &bi) == SD_OK && kind == SD_KIND_I2C) {
            bp_emit(e, ",\"i2c\":{\"address\":\"0x%02X\",\"alt_addresses\":[", bi.address);
            for (unsigned k = 0; k < bi.n_alt; k++) bp_emit(e, "%s\"0x%02X\"", k ? "," : "", bi.alt[k]);
            bp_emit_raw(e, "]}");
        } else if (kind == SD_KIND_SPI) {
            bp_emit(e, ",\"spi\":{\"mode\":%u}", bi.spi_mode);
        }
    } else if (kind == SD_KIND_UART) {
        sd_uart_info_t ui;
        if (sd_uart_info(in, &ui) == SD_OK)
            bp_emit(e, ",\"uart\":{\"baud\":%lu,\"format\":\"%u%c%u\"}", (unsigned long)ui.baud, ui.data_bits,
                    ui.parity ? ui.parity : 'N', ui.stop_bits);
    } else if (kind == SD_KIND_ANALOG) {
        sd_analog_info_t ai;
        if (sd_analog_info(in, &ai) == SD_OK) {
            bp_emit(e, ",\"analog\":{\"output\":\"%s\",\"min\":", ai.current ? "current" : "voltage");
            emit_num(e, ai.min);
            bp_emit_raw(e, ",\"max\":");
            emit_num(e, ai.max);
            bp_emit_raw(e, "}");
        }
    }
    emit_params(e, in);

    if (dry_run) { sensor_port_free(p); sensor_port_free(in); return 1; }

    /* keep it: a running instance of that name is replaced */
    int slot = find_slot(inst);
    if (slot >= 0) {
        stim_forget_slot((unsigned)slot);
        inst_halt((unsigned)slot, true);
        inst_free((unsigned)slot);
    }
    if (slot < 0)
        for (unsigned i = 0; i < SM_MAX_INST; i++) if (!s_slot[i]) { slot = (int)i; break; }
    sm_inst_t *s = slot >= 0 ? sensor_port_alloc(sizeof(sm_inst_t)) : NULL;
    if (!s) {
        if (slot < 0) seterr(err, cap, "%u instances are defined already: sensor_stop one first", SM_MAX_INST);
        else seterr(err, cap, "out of memory for the instance");
        sensor_port_free(p); sensor_port_free(in);
        e->len = 0; e->ok = true; if (e->cap) e->buf[0] = '\0';
        return -1;
    }
    memset(s, 0, sizeof(*s));
    snprintf(s->name, sizeof(s->name), "%s", inst);
    s->kind = kind;
    s->prog = p;
    s->prog_len = (uint32_t)plen;
    s->in = in;
    s->last_at = -1;
    s_slot[slot] = s;
    printf("[sensor] defined %s (%s, %lu program bytes)\n", inst, kind_name(kind), (unsigned long)plen);
    return 1;
}

int sm_stim_chunk(uint32_t offset, uint32_t total, const uint8_t *data, size_t n, bool dry_run,
                  bp_emit_t *e, char *err, size_t cap) {
    int rc = upload_chunk(true, "", offset, total, data, n, SD_JSON_MAX, err, cap);
    if (rc <= 0) return rc;
    void *prog = sensor_port_alloc(SD_STIM_MAX);
    if (!prog) { upload_drop(); seterr(err, cap, "out of memory: compiling needs %u bytes", SD_STIM_MAX); return -1; }
    size_t plen = 0;
    sd_err_t se;
    rc = sd_stim_compile((const char *)s_up.buf, total, prog, SD_STIM_MAX, &plen, &se);
    upload_drop();
    if (rc != SD_OK) {
        char m[160];
        sd_err_format(&se, m, sizeof(m));
        seterr(err, cap, "invalid stimulus: %s", m);
        sensor_port_free(prog);
        return -1;
    }
    sd_stim_t st;
    if (sd_stim_init(&st, prog, plen) != SD_OK) { sensor_port_free(prog); seterr(err, cap, "invalid stimulus"); return -1; }
    bp_emit(e, "\"program_bytes\":%lu,\"signals\":[", (unsigned long)plen);
    unsigned ns = sd_stim_n_signals(&st);
    for (unsigned i = 0; i < ns; i++) {
        const char *ip, *pp;
        size_t il, pl;
        if (sd_stim_signal(&st, i, &ip, &il, &pp, &pl) != SD_OK) continue;
        char sig[72];
        snprintf(sig, sizeof(sig), "%.*s%s%.*s", (int)il, ip ? ip : "", il ? "." : "", (int)pl, pp);
        if (i) bp_emit_raw(e, ",");
        bp_emit_jstr(e, sig);
    }
    bp_emit(e, "],\"faults\":%u,\"duration_s\":", sd_stim_n_faults(&st));
    emit_num(e, sd_stim_duration_s(&st));
    bp_emit(e, ",\"repeat\":%s", sd_stim_repeats(&st) ? "true" : "false");
    if (dry_run) { bp_emit_raw(e, ",\"dry_run\":true"); sensor_port_free(prog); return 1; }
    uint8_t *p = sensor_port_alloc(plen);
    if (p) { memcpy(p, prog, plen); sensor_port_free(prog); } else p = prog;
    sensor_port_free(s_stim.prog);
    memset(&s_stim, 0, sizeof(s_stim));
    s_stim.prog = p;
    s_stim.len = (uint32_t)plen;
    (void)sd_stim_init(&s_stim.st, p, plen);
    return 1;
}

/* ---------------------------------------------------------------------------------------------
 * start / stop
 * ------------------------------------------------------------------------------------------- */

static int take_target(sd_kind_t k) {
    if (k == SD_KIND_SPI) {
        for (unsigned i = 0; i < SM_MAX_INST; i++)
            if (s_slot[i] && s_slot[i]->running && s_slot[i]->kind == SD_KIND_SPI) return -1;
        return FPGA_EMU_SPI_TARGET;
    }
    for (unsigned t = 0; t < FPGA_EMU_I2C_TARGETS; t++) {
        bool used = false;
        for (unsigned i = 0; i < SM_MAX_INST; i++)
            if (s_slot[i] && s_slot[i]->running && s_slot[i]->kind == SD_KIND_I2C && s_slot[i]->target == t) used = true;
        if (!used) return (int)t;
    }
    return -1;
}

static int apply_values(sm_inst_t *s, unsigned n, const char *const keys[], const double vals[], char *err, size_t cap) {
    int idx[SD_MAX_PARAMS];
    if (n > SD_MAX_PARAMS) { seterr(err, cap, "at most %u values", SD_MAX_PARAMS); return -1; }
    for (unsigned k = 0; k < n; k++) {
        idx[k] = sd_param_index(s->in, keys[k]);
        if (idx[k] < 0) {
            seterr(err, cap, "%s has no param '%s'", s->name, keys[k]);
            return -1;
        }
        sd_param_info_t pi;
        sd_param_info(s->in, (unsigned)idx[k], &pi);
        if (!(vals[k] >= pi.min && vals[k] <= pi.max)) {
            char lo[24], hi[24];
            bp_emit_t a, b;
            bp_emit_init(&a, lo, sizeof(lo)); emit_num(&a, pi.min);
            bp_emit_init(&b, hi, sizeof(hi)); emit_num(&b, pi.max);
            seterr(err, cap, "%s must be %s..%s %.*s", keys[k], lo, hi, (int)pi.unit_len, pi.unit ? pi.unit : "");
            return -1;
        }
    }
    for (unsigned k = 0; k < n; k++) (void)sd_set_param(s->in, (unsigned)idx[k], vals[k], false);
    s->dirty = true;
    return 0;
}

static bool dac_used_by_us(void) {
    for (unsigned i = 0; i < SM_MAX_INST; i++) {
        const sm_inst_t *s = s_slot[i];
        if (s && s->running && s->kind == SD_KIND_ANALOG &&
            (s->out == SM_OUT_DAC || sensor_port_current_shares_dac())) return true;
    }
    return false;
}

int sm_start(const char *inst, const sm_start_req_t *req, bp_emit_t *e, char *err, size_t cap) {
    la_pins_set_owner_namer(sm_slot_name);
    int slot = inst ? find_slot(inst) : -1;
    if (slot < 0) { seterr(err, cap, "unknown sensor instance '%s'", inst ? inst : ""); return -1; }
    sm_inst_t *s = s_slot[slot];
    if (s->running) { seterr(err, cap, "%s is running: sensor_stop it first", s->name); return -1; }
    sm_pins_t pins = req->pins;
    s->pins = pins;

    /* per kind checks, before anything is touched */
    int target = -1;
    sd_bus_info_t bi;
    memset(&bi, 0, sizeof(bi));
    if (is_regs(s)) {
        sd_bus_info(s->in, &bi);
        target = take_target(s->kind);
        if (target < 0) {
            seterr(err, cap, s->kind == SD_KIND_SPI ? "the SPI sensor slot is in use: sensor_stop it first"
                                                    : "all 4 I2C sensor slots are in use: sensor_stop one first");
            return -1;
        }
        if (bi.has_fifo)
            for (unsigned i = 0; i < SM_MAX_INST; i++)
                if (s_slot[i] && s_slot[i]->running && s_slot[i]->has_fifo) {
                    seterr(err, cap, "the sensor FIFO is used by sensor:%s (one FIFO sensor at a time)", s_slot[i]->name);
                    return -1;
                }
    }
    if (s->kind == SD_KIND_I2C) {
        if (!pins.sda || !pins.scl) { seterr(err, cap, "an I2C sensor needs pins sda and scl"); return -1; }
        int a = req->address >= 0 ? req->address : bi.address;
        bool ok = a == bi.address;
        for (unsigned k = 0; k < bi.n_alt; k++) ok = ok || a == bi.alt[k];
        if (!ok) { seterr(err, cap, "address 0x%02X is not one of this sensor's addresses", a); return -1; }
        for (unsigned i = 0; i < SM_MAX_INST; i++) {
            const sm_inst_t *o = s_slot[i];
            if (o && o->running && o->kind == SD_KIND_I2C && o->pins.sda == pins.sda && o->pins.scl == pins.scl &&
                o->addr7 == a) {
                seterr(err, cap, "address 0x%02X is used by sensor:%s on this bus", a, o->name);
                return -1;
            }
        }
        s->addr7 = (uint8_t)a;
        if (pins.drdy && !bi.has_drdy) { seterr(err, cap, "%s has no data_ready in its definition", s->name); return -1; }
    } else if (s->kind == SD_KIND_SPI) {
        if (!pins.sclk || !pins.mosi || !pins.miso || !pins.cs) { seterr(err, cap, "an SPI sensor needs pins sclk, mosi, miso and cs"); return -1; }
        if (pins.drdy) { seterr(err, cap, "the emulated SPI target has no data-ready pin"); return -1; }
    } else if (s->kind == SD_KIND_UART) {
        if (!pins.tx) { seterr(err, cap, "a UART sensor needs pin tx (pod to DUT)"); return -1; }
        for (unsigned i = 0; i < SM_MAX_INST; i++)
            if (s_slot[i] && s_slot[i]->running && s_slot[i]->kind == SD_KIND_UART) {
                seterr(err, cap, "the UART sensor slot is used by sensor:%s", s_slot[i]->name);
                return -1;
            }
        sd_uart_info_t ui;
        sd_uart_info(s->in, &ui);
        if (ui.data_bits != 8 || (ui.parity && ui.parity != 'N') || ui.stop_bits != 1) {
            seterr(err, cap, "the emulated UART is 8N1 only (the definition asks for %u%c%u)", ui.data_bits,
                   ui.parity ? ui.parity : 'N', ui.stop_bits);
            return -1;
        }
        s->baud = req->baud ? req->baud : ui.baud;
        if (s->baud < 300 || s->baud > 3000000) { seterr(err, cap, "baud must be 300..3000000"); return -1; }
    } else if (s->kind == SD_KIND_ANALOG) {
        sd_analog_info_t ai;
        sd_analog_info(s->in, &ai);
        sm_out_t want = ai.current ? SM_OUT_CURRENT : SM_OUT_DAC;
        if (pins.output && pins.output != want) {
            seterr(err, cap, "%s is a %s output: use output \"%s\"", s->name, ai.current ? "current" : "voltage",
                   ai.current ? "current" : "dac");
            return -1;
        }
        if (ai.excitation) { seterr(err, cap, "excitation: adc is not supported by the pod yet"); return -1; }
        for (unsigned i = 0; i < SM_MAX_INST; i++)
            if (s_slot[i] && s_slot[i]->running && s_slot[i]->kind == SD_KIND_ANALOG && s_slot[i]->out == want) {
                seterr(err, cap, "the %s output is used by sensor:%s", want == SM_OUT_CURRENT ? "current" : "dac", s_slot[i]->name);
                return -1;
            }
        bool shares = want == SM_OUT_DAC || sensor_port_current_shares_dac();
        if (shares && !dac_used_by_us() && sensor_port_dac_running()) {
            seterr(err, cap, "the DAC is playing a waveform (generate, replay or the control loop): dac_stop it first");
            return -1;
        }
        s->out = want;
        s->out_max = ai.max;
    }
    if (is_digital(s) && !sensor_port_la_voltage_set()) {
        seterr(err, cap, "la voltage not set; set it with la_voltage (mv 1800 or 3300) first");
        return -1;
    }
    if (is_digital(s) && !pins_check((unsigned)slot, s, err, cap)) return -1;

    /* the image */
    bool loaded = false;
    if (is_digital(s) && sensor_port_image() != 2) {
        if (sensor_port_dac_running() && !dac_used_by_us()) {
            seterr(err, cap, "loading the emulation image stops the DAC waveform that is playing: dac_stop it first");
            return -1;
        }
        char m[SM_ERR_MAX];
        if (sensor_port_load_emulation(m, sizeof(m)) != 0) { seterr(err, cap, "%s", m); return -1; }
        loaded = true;
        s_rearm = false;   /* nothing digital ran on the old image; analog outputs: below */
        for (unsigned i = 0; i < SM_MAX_INST; i++)
            if (s_slot[i] && s_slot[i]->running && s_slot[i]->kind == SD_KIND_ANALOG)
                (void)analog_apply(s_slot[i], true, m, sizeof(m));
    }

    /* start */
    if (sd_inst_init(s->in, s->prog, s->prog_len, NULL) != SD_OK) { seterr(err, cap, "%s: the program does not start", s->name); return -1; }
    if (req->n_values && apply_values(s, req->n_values, req->keys, req->values, err, cap) != 0) return -1;
    s->t0_us = sensor_port_now_us();
    memset(s->faults, 0, sizeof(s->faults));
    s->writes = s->log_lost = s->rebuilds = s->drdy_count = s->fifo_frames = s->fifo_drops = 0;
    s->last_at = -1;
    int rc = 0;
    if (is_regs(s)) {
        if (!s->live) s->live = sensor_port_alloc(SD_REGIMAGE);
        if (!s->live) { seterr(err, cap, "out of memory for the register image"); return -1; }
        sd_regs_ro_map(s->in, s->ro);
        s->target = (uint8_t)target;
        s->drdy_pin = pins.drdy != 0;
        s->drdy_on = bi.drdy_on;
        s->drdy_en = true;
        s->has_fifo = bi.has_fifo;
        s->fifo_len = bi.fifo_frame_len;
        uint32_t fb = (uint32_t)bi.fifo_depth * bi.fifo_frame_len;
        s->fifo_cap = (uint16_t)(fb > FPGA_EMU_FIFO_BYTES ? FPGA_EMU_FIFO_BYTES : fb);
        s->fifo_level_frames = 0;
        rc = regs_arm(s, true);
    } else if (s->kind == SD_KIND_UART) {
        if (!s->u) s->u = sensor_port_alloc(sizeof(sm_uart_t));
        if (!s->u) { seterr(err, cap, "out of memory for the UART buffers"); return -1; }
        memset(s->u, 0, sizeof(*s->u));
        s->u->rng = 0x2545F491u ^ (uint32_t)s->t0_us;
        rc = uart_arm(s);
    } else if (s->kind == SD_KIND_ANALOG) {
        rc = analog_apply(s, true, err, cap);
        if (rc) return -1;
    }
    if (rc != 0) { seterr(err, cap, "the gateware did not take the sensor configuration"); return -1; }
    if (is_digital(s)) pins_claim((unsigned)slot, s);
    s->running = true;
    if (is_digital(s)) pinmap_update();
    printf("[sensor] %s started (%s)\n", s->name, kind_name(s->kind));

    char st[12];
    slot_text(s, st, sizeof(st));
    bp_emit_raw(e, "\"instance\":");
    bp_emit_jstr(e, s->name);
    int img = sensor_port_image();
    bp_emit(e, ",\"slot\":\"%s\",\"image\":\"%s\",\"image_loaded\":%s", st,
            img == 2 ? "emulation" : img == 1 ? "deep" : img == 0 ? "loop" : "unknown", loaded ? "true" : "false");
    if (s->kind == SD_KIND_I2C) bp_emit(e, ",\"address\":\"0x%02X\"", s->addr7);
    if (s->kind == SD_KIND_UART) bp_emit(e, ",\"baud\":%lu", (unsigned long)s->baud);
    if (img == 2) bp_emit_raw(e, ",\"unavailable\":[\"dac_control_loop\",\"deep_replay\"]");
    return 0;
}

int sm_stop(const char *inst, char *err, size_t cap) {
    if (!inst) {
        for (unsigned i = 0; i < SM_MAX_INST; i++) { inst_halt(i, true); inst_free(i); }
        s_stim.running = false;
        return 0;
    }
    int slot = find_slot(inst);
    if (slot < 0) { seterr(err, cap, "unknown sensor instance '%s'", inst); return -1; }
    stim_forget_slot((unsigned)slot);
    inst_halt((unsigned)slot, true);
    inst_free((unsigned)slot);
    return 0;
}

int sm_set(const char *inst, unsigned n, const char *const keys[], const double vals[], char *err, size_t cap) {
    int slot = inst ? find_slot(inst) : -1;
    if (slot < 0) { seterr(err, cap, "unknown sensor instance '%s'", inst ? inst : ""); return -1; }
    if (n == 0) { seterr(err, cap, "values: give at least one param"); return -1; }
    return apply_values(s_slot[slot], n, keys, vals, err, cap);
}

/* ---------------------------------------------------------------------------------------------
 * faults
 * ------------------------------------------------------------------------------------------- */

static void bus_faults_changed(sm_inst_t *s) {
    if (!s->running) return;
    if (is_regs(s)) (void)regs_config(s, true);
    s->dirty = true;
}

static int fault_add(sm_inst_t *s, const sd_fault_t *f, bool stim, uint8_t stim_idx, char *err, size_t cap) {
    if (!sd_fault_allowed(f->type, s->kind)) {
        seterr(err, cap, "fault %s does not apply to a%s %s sensor", sd_fault_name(f->type),
               s->kind == SD_KIND_I2C || s->kind == SD_KIND_ANALOG ? "n" : "", kind_name(s->kind));
        return -1;
    }
    unsigned k = SM_FAULTS;
    for (unsigned i = 0; i < SM_FAULTS; i++) if (s->faults[i].on && s->faults[i].f.type == f->type && !engine_fault(f->type)) k = i;
    for (unsigned i = 0; i < SM_FAULTS && k == SM_FAULTS; i++) if (!s->faults[i].on) k = i;
    if (k == SM_FAULTS) { seterr(err, cap, "%s has %u faults on: sensor_fault_clear first", s->name, SM_FAULTS); return -1; }
    if (engine_fault(f->type) && sd_set_fault(s->in, f, true) != SD_OK) {
        seterr(err, cap, "fault %s: check its param, value, at and mask", sd_fault_name(f->type));
        return -1;
    }
    uint64_t now = sensor_port_now_us();
    double dur = f->duration_s;
    if (f->type == SD_FAULT_BUS_STUCK && (!(dur > 0) || dur > SM_BUS_STUCK_MAX_MS / 1000.0)) dur = SM_BUS_STUCK_MAX_MS / 1000.0;
    s->faults[k].on = true;
    s->faults[k].f = *f;
    s->faults[k].stim = stim;
    s->faults[k].stim_idx = stim_idx;
    s->faults[k].end_us = dur > 0 ? now + (uint64_t)(dur * 1e6) : 0;
    bus_faults_changed(s);
    return 0;
}

static void fault_remove(sm_inst_t *s, unsigned k) {
    if (!s->faults[k].on) return;
    if (engine_fault(s->faults[k].f.type)) (void)sd_set_fault(s->in, &s->faults[k].f, false);
    s->faults[k].on = false;
    bus_faults_changed(s);
}

int sm_fault(const char *inst, const sd_fault_t *f, char *err, size_t cap) {
    int slot = inst ? find_slot(inst) : -1;
    if (slot < 0) { seterr(err, cap, "unknown sensor instance '%s'", inst ? inst : ""); return -1; }
    if (!s_slot[slot]->running) { seterr(err, cap, "%s is not running: sensor_start it first", inst); return -1; }
    return fault_add(s_slot[slot], f, false, 0, err, cap);
}

int sm_fault_clear(const char *inst, char *err, size_t cap) {
    int slot = inst ? find_slot(inst) : -1;
    if (slot < 0) { seterr(err, cap, "unknown sensor instance '%s'", inst ? inst : ""); return -1; }
    sm_inst_t *s = s_slot[slot];
    for (unsigned k = 0; k < SM_FAULTS; k++) s->faults[k].on = false;
    sd_clear_faults(s->in);
    bus_faults_changed(s);
    return 0;
}

/* ---------------------------------------------------------------------------------------------
 * stimulus
 * ------------------------------------------------------------------------------------------- */

static int only_running(void) {
    int found = -1;
    for (unsigned i = 0; i < SM_MAX_INST; i++)
        if (s_slot[i] && s_slot[i]->running) { if (found >= 0) return -2; found = (int)i; }
    return found;
}

int sm_stim_start(double at_s, char *err, size_t cap) {
    if (!s_stim.prog) { seterr(err, cap, "no stimulus: sensor_stimulus it first"); return -1; }
    if (!(at_s >= 0)) at_s = 0;
    unsigned ns = sd_stim_n_signals(&s_stim.st);
    for (unsigned i = 0; i < ns; i++) {
        const char *ip, *pp;
        size_t il, pl;
        sd_stim_signal(&s_stim.st, i, &ip, &il, &pp, &pl);
        char iname[SM_NAME_MAX + 2] = { 0 }, pname[36] = { 0 };
        snprintf(iname, sizeof(iname), "%.*s", (int)(il > SM_NAME_MAX ? SM_NAME_MAX + 1 : il), ip ? ip : "");
        snprintf(pname, sizeof(pname), "%.*s", (int)pl, pp);
        int slot = il ? find_slot(iname) : only_running();
        if (slot == -2) { seterr(err, cap, "signal %s: several instances run, name one (instance.%s)", pname, pname); return -1; }
        if (slot < 0 || !s_slot[slot]->running) { seterr(err, cap, "signal %s%s%s: no running instance%s%s", iname, il ? "." : "", pname, il ? " " : "", iname); return -1; }
        int pi = sd_param_index(s_slot[slot]->in, pname);
        if (pi < 0) { seterr(err, cap, "signal %s%s%s: %s has no such param", iname, il ? "." : "", pname, s_slot[slot]->name); return -1; }
        s_stim.sig_slot[i] = (int8_t)slot;
        s_stim.sig_param[i] = (int8_t)pi;
    }
    unsigned nf = sd_stim_n_faults(&s_stim.st);
    for (unsigned i = 0; i < nf; i++) {
        sd_fault_t f;
        sd_stim_fault(&s_stim.st, i, &f);
        int slot = f.sensor[0] ? find_slot(f.sensor) : only_running();
        if (slot == -2) { seterr(err, cap, "faults[%u]: several instances run, give its sensor", i); return -1; }
        if (slot < 0 || !s_slot[slot]->running) { seterr(err, cap, "faults[%u]: no running instance %s", i, f.sensor); return -1; }
        if (!sd_fault_allowed(f.type, s_slot[slot]->kind)) {
            seterr(err, cap, "faults[%u]: %s does not apply to %s", i, sd_fault_name(f.type), s_slot[slot]->name);
            return -1;
        }
        s_stim.fault_slot[i] = (int8_t)slot;
    }
    (void)sd_stim_init(&s_stim.st, s_stim.prog, s_stim.len);
    double hz = sd_stim_rate_hz(&s_stim.st);
    if (!(hz > 0)) hz = 50.0;
    if (hz > SM_STIM_MAX_HZ) hz = SM_STIM_MAX_HZ;
    s_stim.rate_hz = hz;
    uint64_t now = sensor_port_now_us();
    s_stim.t0_us = now - (uint64_t)(at_s * 1e6);
    s_stim.next_us = now;
    s_stim.running = true;
    return 0;
}

static void stim_clear_faults(void) {
    for (unsigned i = 0; i < SM_MAX_INST; i++) {
        sm_inst_t *s = s_slot[i];
        if (!s) continue;
        for (unsigned k = 0; k < SM_FAULTS; k++) if (s->faults[k].on && s->faults[k].stim) fault_remove(s, k);
    }
}

void sm_stim_stop(void) {
    if (!s_stim.running) return;
    s_stim.running = false;
    stim_clear_faults();
}

static void stim_step(uint64_t now) {
    if (now < s_stim.next_us) return;
    uint64_t period = (uint64_t)(1e6 / s_stim.rate_hz);
    s_stim.next_us += period;
    if (s_stim.next_us + CATCHUP_MAX_US < now) s_stim.next_us = now + period;
    double t = (double)(now - s_stim.t0_us) / 1e6;
    double dur = sd_stim_duration_s(&s_stim.st);
    bool last = !sd_stim_repeats(&s_stim.st) && t >= dur;
    if (last) t = dur;
    s_stim.t_s = t;
    sd_stim_event_t ev[SD_STIM_SIGNALS + 2 * SD_STIM_FAULTS];
    int n = sd_stim_step(&s_stim.st, t, ev, sizeof(ev) / sizeof(ev[0]));
    uint8_t stepped = 0;
    for (int i = 0; i < n; i++) {
        if (ev[i].kind == SD_EV_VALUE) {
            int slot = s_stim.sig_slot[ev[i].index];
            sm_inst_t *s = slot >= 0 ? s_slot[slot] : NULL;
            if (!s) continue;
            (void)sd_set_param(s->in, (unsigned)s_stim.sig_param[ev[i].index], ev[i].value, true);
            s->dirty = true;
            stepped |= (uint8_t)(1u << slot);
        } else {
            int slot = s_stim.fault_slot[ev[i].index];
            sm_inst_t *s = slot >= 0 ? s_slot[slot] : NULL;
            if (!s) continue;
            if (ev[i].kind == SD_EV_FAULT_ON) {
                sd_fault_t f;
                char m[SM_ERR_MAX];
                if (sd_stim_fault(&s_stim.st, ev[i].index, &f) == SD_OK) {
                    f.duration_s = -1;   /* the stimulus switches it off */
                    (void)fault_add(s, &f, true, ev[i].index, m, sizeof(m));
                }
            } else {
                for (unsigned k = 0; k < SM_FAULTS; k++)
                    if (s->faults[k].on && s->faults[k].stim && s->faults[k].stim_idx == ev[i].index) fault_remove(s, k);
            }
        }
    }
    /* data ready on: sample also fires on each stimulus step */
    for (unsigned i = 0; i < SM_MAX_INST; i++) {
        sm_inst_t *s = s_slot[i];
        if (s && s->running && (stepped & (1u << i)) && s->drdy_pin && s->drdy_on == 1 && s->drdy_en &&
            s->kind == SD_KIND_I2C) {
            (void)fpga_emu_drdy(s->target, true);
            s->drdy_count++;
        }
    }
    if (last) s_stim.running = false;   /* signals hold their last value */
}

/* ---------------------------------------------------------------------------------------------
 * poll
 * ------------------------------------------------------------------------------------------- */

void sm_on_gateware_reconfigured(void) {
    for (unsigned i = 0; i < SM_MAX_INST; i++) if (s_slot[i] && s_slot[i]->running) s_rearm = true;
}

void sm_on_dac_stop(void) {
    for (unsigned i = 0; i < SM_MAX_INST; i++) {
        sm_inst_t *s = s_slot[i];
        if (!s || !s->running || s->kind != SD_KIND_ANALOG) continue;
        if (s->out == SM_OUT_DAC || sensor_port_current_shares_dac()) inst_halt(i, false);
    }
}

static void rearm_all(void) {
    s_rearm = false;
    int img = sensor_port_image();
    for (unsigned i = 0; i < SM_MAX_INST; i++) {
        sm_inst_t *s = s_slot[i];
        if (!s || !s->running) continue;
        int rc = 0;
        if (s->kind == SD_KIND_ANALOG) {
            char m[SM_ERR_MAX];
            rc = analog_apply(s, true, m, sizeof(m));
        } else if (img != 2) {
            printf("[sensor] the emulation image is gone: %s stopped\n", s->name);
            inst_halt(i, false);
            continue;
        } else if (is_regs(s)) {
            rc = regs_arm(s, false);   /* the live image, the DUT's configuration included */
        } else {
            rc = uart_arm(s);
        }
        if (rc != 0) {
            printf("[sensor] gateware reconfigured: could not re-arm %s, stopped\n", s->name);
            inst_halt(i, false);
        } else {
            printf("[sensor] gateware reconfigured: %s re-armed\n", s->name);
        }
    }
    if (img == 2) pinmap_update();
}

static void fifo_poll(sm_inst_t *s, uint64_t now) {
    if (!s->fifo_en || !(s->fifo_hz > 0) || !s->fifo_len) return;
    uint64_t period = (uint64_t)(1e6 / s->fifo_hz);
    if (period == 0) period = 1;
    if (s->fifo_next_us == 0 || s->fifo_next_us + CATCHUP_MAX_US < now) s->fifo_next_us = now;
    if (now < s->fifo_next_us) return;
    uint16_t level = 0;
    bool ovf = false, udf = false;
    if (fpga_emu_fifo_status(&level, &ovf, &udf) != 0) return;
    unsigned pushed = 0;
    uint8_t frame[256];
    while (s->fifo_next_us <= now && pushed < 16) {
        s->fifo_next_us += period;
        if ((unsigned)level + s->fifo_len > s->fifo_cap) { s->fifo_drops++; continue; }
        size_t len = 0;
        set_time(s, now);
        if (sd_fifo_frame(s->in, s->live, frame, sizeof(frame), &len) != SD_OK || !len) break;
        if (fpga_emu_fifo_write(frame, len) != 0) break;
        level = (uint16_t)(level + len);
        s->fifo_frames++;
        pushed++;
        if (s->drdy_pin && s->drdy_on == 1 && s->drdy_en && s->kind == SD_KIND_I2C) {
            (void)fpga_emu_drdy(s->target, true);
            s->drdy_count++;
        }
    }
    unsigned frames = level / s->fifo_len;
    if (frames != s->fifo_level_frames) { s->fifo_level_frames = frames; s->dirty = true; }
}

static void drdy_poll(sm_inst_t *s, uint64_t now) {
    if (!s->drdy_pin || s->drdy_on != 2 || !s->drdy_en || !(s->drdy_hz > 0) || s->kind != SD_KIND_I2C) return;
    uint64_t period = (uint64_t)(1e6 / s->drdy_hz);
    if (period == 0) period = 1;
    if (s->drdy_next_us == 0 || s->drdy_next_us + CATCHUP_MAX_US < now) s->drdy_next_us = now;
    if (now < s->drdy_next_us) return;
    while (s->drdy_next_us <= now) s->drdy_next_us += period;
    (void)fpga_emu_drdy(s->target, true);
    s->drdy_count++;
}

void sm_poll(void) {
    bool any = false;
    for (unsigned i = 0; i < SM_MAX_INST; i++) if (s_slot[i] && s_slot[i]->running) { any = true; break; }
    if (!any && !s_up.active) return;   /* idle: a few loads, no SPI, no clock read */
    uint64_t now = sensor_port_now_us();
    if (s_up.active && now - s_up.last_us > (uint64_t)SM_CHUNK_IDLE_MS * 1000u) {
        printf("[sensor] an upload was idle for %u s: dropped\n", SM_CHUNK_IDLE_MS / 1000u);
        upload_drop();
    }
    if (!any) return;
    if (s_rearm) rearm_all();
    for (unsigned i = 0; i < SM_MAX_INST; i++) {
        sm_inst_t *s = s_slot[i];
        if (!s || !s->running) continue;
        for (unsigned k = 0; k < SM_FAULTS; k++)
            if (s->faults[k].on && s->faults[k].end_us && now >= s->faults[k].end_us) fault_remove(s, k);
    }
    if (s_stim.running) stim_step(now);
    for (unsigned i = 0; i < SM_MAX_INST; i++) {
        sm_inst_t *s = s_slot[i];
        if (!s || !s->running) continue;
        if (is_regs(s)) {
            if (now >= s->next_status_us) {
                s->next_status_us = now + STATUS_PERIOD_US;
                regs_writes(s, now);
            }
            if (now >= s->next_tick_us) { s->next_tick_us = now + TICK_PERIOD_US; s->dirty = true; }
            if (s->has_fifo) fifo_poll(s, now);
            if (s->dirty) (void)regs_rebuild(s, now);
            if (s->drdy_changed) {
                s->drdy_changed = false;
                (void)regs_config(s, true);
                if (!s->drdy_en && s->kind == SD_KIND_I2C) (void)fpga_emu_drdy(s->target, false);   /* release a held level */
            }
            drdy_poll(s, now);
        } else if (s->kind == SD_KIND_UART) {
            uart_poll(s, now);
        } else if (s->kind == SD_KIND_ANALOG && s->dirty) {
            char m[SM_ERR_MAX];
            if (analog_apply(s, false, m, sizeof(m)) != 0) printf("[sensor] %s: %s\n", s->name, m);
        }
    }
}

/* ---------------------------------------------------------------------------------------------
 * read-backs and status
 * ------------------------------------------------------------------------------------------- */

int sm_regs(const char *inst, unsigned start, unsigned len, uint8_t *out, char *err, size_t cap) {
    int slot = inst ? find_slot(inst) : -1;
    if (slot < 0) { seterr(err, cap, "unknown sensor instance '%s'", inst ? inst : ""); return -1; }
    sm_inst_t *s = s_slot[slot];
    if (!is_regs(s)) { seterr(err, cap, "%s is not an I2C or SPI sensor", s->name); return -1; }
    if (!s->running) { seterr(err, cap, "%s is not running", s->name); return -1; }
    if (len == 0 || start + len > SD_REGIMAGE) { seterr(err, cap, "start/len out of range (0..255)"); return -1; }
    if (fpga_emu_read_regs(s->target, (uint8_t)start, out, len) != 0) { seterr(err, cap, "read regs failed"); return -1; }
    return 0;
}

int sm_i2c_pins(const char *inst, unsigned *sda, unsigned *scl, char *err, size_t cap) {
    int slot = inst ? find_slot(inst) : -1;
    if (slot < 0) { seterr(err, cap, "unknown sensor instance '%s'", inst ? inst : ""); return -1; }
    sm_inst_t *s = s_slot[slot];
    if (s->kind != SD_KIND_I2C) { seterr(err, cap, "sensor_la captures I2C sensors only; use la_capture for the pins of %s", s->name); return -1; }
    if (!s->running) { seterr(err, cap, "%s is not running", s->name); return -1; }
    *sda = s->pins.sda;
    *scl = s->pins.scl;
    return 0;
}

static void emit_inst(bp_emit_t *e, const sm_inst_t *s) {
    size_t nl = 0;
    const char *nm = sd_name(s->in, &nl);
    char st[12];
    slot_text(s, st, sizeof(st));
    bp_emit_raw(e, "{\"instance\":");
    bp_emit_jstr(e, s->name);
    bp_emit(e, ",\"kind\":\"%s\",\"name\":", kind_name(s->kind));
    emit_strn(e, nm, nl);
    bp_emit(e, ",\"slot\":\"%s\",\"running\":%s", s->running ? st : "", s->running ? "true" : "false");
    if (s->running) {
        const sm_pins_t *p = &s->pins;
        if (s->kind == SD_KIND_I2C) {
            bp_emit(e, ",\"address\":\"0x%02X\",\"pins\":{\"sda\":%u,\"scl\":%u", s->addr7, p->sda, p->scl);
            if (p->drdy) bp_emit(e, ",\"drdy\":%u", p->drdy);
            bp_emit_raw(e, "}");
        } else if (s->kind == SD_KIND_SPI) {
            bp_emit(e, ",\"pins\":{\"sclk\":%u,\"mosi\":%u,\"miso\":%u,\"cs\":%u}", p->sclk, p->mosi, p->miso, p->cs);
        } else if (s->kind == SD_KIND_UART) {
            bp_emit(e, ",\"baud\":%lu,\"pins\":{\"tx\":%u", (unsigned long)s->baud, p->tx);
            if (p->rx) bp_emit(e, ",\"rx\":%u", p->rx);
            bp_emit_raw(e, "}");
        } else {
            bp_emit(e, ",\"pins\":{\"output\":\"%s\"},\"output\":", s->out == SM_OUT_CURRENT ? "current" : "dac");
            if (s->out_set) emit_num(e, s->out_last); else bp_emit_raw(e, "null");
        }
    }
    bp_emit_raw(e, ",\"values\":{");
    unsigned np = sd_n_params(s->in);
    for (unsigned i = 0; i < np; i++) {
        sd_param_info_t pi;
        if (sd_param_info(s->in, i, &pi) != SD_OK) continue;
        if (i) bp_emit_raw(e, ",");
        emit_strn(e, pi.key, pi.key_len);
        bp_emit_raw(e, ":");
        emit_num(e, sd_get_param(s->in, i));
    }
    bp_emit_raw(e, "},\"faults\":[");
    bool first = true;
    uint64_t now = sensor_port_now_us();
    for (unsigned k = 0; k < SM_FAULTS; k++) {
        const sm_fault_t *f = &s->faults[k];
        if (!f->on) continue;
        bp_emit(e, "%s{\"type\":\"%s\"", first ? "" : ",", sd_fault_name(f->f.type));
        if (f->f.param[0]) { bp_emit_raw(e, ",\"param\":"); bp_emit_jstr(e, f->f.param); }
        if (f->f.has_value) { bp_emit_raw(e, ",\"value\":"); emit_num(e, f->f.value); }
        if (f->f.type == SD_FAULT_CORRUPT) bp_emit(e, ",\"at\":\"0x%02X\",\"mask\":\"0x%02X\"", (unsigned)f->f.at, f->f.mask);
        if (f->f.type == SD_FAULT_GARBAGE) { bp_emit_raw(e, ",\"rate_hz\":"); emit_num(e, f->f.rate_hz > 0 ? f->f.rate_hz : 100.0); }
        if (f->stim) bp_emit_raw(e, ",\"stimulus\":true");
        bp_emit_raw(e, ",\"left_s\":");
        if (f->end_us) emit_num(e, f->end_us > now ? (double)((f->end_us - now) / 1000u) / 1000.0 : 0.0);
        else bp_emit_raw(e, "null");
        bp_emit_raw(e, "}");
        first = false;
    }
    bp_emit_raw(e, "]");
    if (s->running && is_regs(s)) {
        /* transfers: null, the v54 gateware counts the DUT's data writes only (wr_count) */
        bp_emit(e, ",\"bus\":{\"writes\":%lu,\"transfers\":null,\"log_lost\":%lu", (unsigned long)s->writes,
                (unsigned long)s->log_lost);
        if (s->last_at >= 0) bp_emit(e, ",\"last_write\":{\"at\":\"0x%02X\",\"value\":\"0x%02X\"}", (unsigned)s->last_at, s->last_val);
        if (s->drdy_pin) bp_emit(e, ",\"drdy\":%lu", (unsigned long)s->drdy_count);
        if (s->has_fifo) bp_emit(e, ",\"fifo_frames\":%lu,\"fifo_drops\":%lu", (unsigned long)s->fifo_frames, (unsigned long)s->fifo_drops);
        bp_emit_raw(e, "}");
    } else if (s->running && s->kind == SD_KIND_UART) {
        const sm_uart_t *u = s->u;
        bp_emit(e, ",\"bus\":{\"sent_bytes\":%lu,\"received_bytes\":%lu,\"messages\":%lu,\"answers\":%lu,"
                   "\"overruns\":%lu,\"dropped\":%lu,\"late\":%lu}",
                (unsigned long)u->sent, (unsigned long)u->received, (unsigned long)u->messages,
                (unsigned long)u->answers, (unsigned long)u->overruns, (unsigned long)u->dropped,
                (unsigned long)u->late);
    }
    bp_emit_raw(e, "}");
}

int sm_status(const char *inst, unsigned first, bp_emit_t *e, char *err, size_t cap) {
    int img = sensor_port_image();
    bp_emit(e, "\"image\":\"%s\",\"stimulus\":{\"loaded\":%s,\"running\":%s",
            img == 2 ? "emulation" : img == 1 ? "deep" : img == 0 ? "loop" : "unknown",
            s_stim.prog ? "true" : "false", s_stim.running ? "true" : "false");
    if (s_stim.running) { bp_emit_raw(e, ",\"t_s\":"); emit_num(e, s_stim.t_s); }
    bp_emit_raw(e, "},\"instances\":[");
    int only = -1;
    if (inst) {
        only = find_slot(inst);
        if (only < 0) { seterr(err, cap, "unknown sensor instance '%s'", inst); return -1; }
    }
    unsigned listed = 0, idx = 0;
    int next = -1;
    for (unsigned i = 0; i < SM_MAX_INST; i++) {
        if (!s_slot[i] || (only >= 0 && (int)i != only)) continue;
        if (idx++ < first) continue;
        /* each instance whole, or not at all: page when the next one does not fit */
        size_t mark = e->len;
        if (listed) bp_emit_raw(e, ",");
        emit_inst(e, s_slot[i]);
        if (!bp_emit_ok(e) || e->cap - e->len < 48u) {
            e->len = mark; e->ok = true; e->buf[mark] = '\0';
            next = (int)(idx - 1);
            break;
        }
        listed++;
    }
    bp_emit_raw(e, "]");
    if (next >= 0) bp_emit(e, ",\"next\":%d", next);
    return 0;
}

/* ---------------------------------------------------------------------------------------------
 * other commands vs the sensors
 * ------------------------------------------------------------------------------------------- */

bool sensor_mgr_digital_names(char *out, size_t cap) {
    size_t n = 0;
    if (cap) out[0] = '\0';
    for (unsigned i = 0; i < SM_MAX_INST; i++) {
        const sm_inst_t *s = s_slot[i];
        if (!s || !s->running || !is_digital(s)) continue;
        int w = snprintf(out + n, cap > n ? cap - n : 0, "%s%s", n ? ", " : "", s->name);
        if (w > 0) n += (size_t)w;
        if (n >= cap) { n = cap ? cap - 1 : 0; break; }
    }
    return n > 0;
}

const char *sensor_mgr_command_conflict(const char *verb) {
    static char msg[128];
    if (!verb) return NULL;
    static const char *const dac_verbs[] = { "generate", "replay", "load", "load_bin", "dac_set", "dac_out",
                                             "dac_control_loop", "measure", NULL };
    bool dac = false, cur = !strcmp(verb, "current_out");
    for (const char *const *v = dac_verbs; *v; v++) if (!strcmp(verb, *v)) dac = true;
    if (!dac && !cur) return NULL;
    for (unsigned i = 0; i < SM_MAX_INST; i++) {
        const sm_inst_t *s = s_slot[i];
        if (!s || !s->running || s->kind != SD_KIND_ANALOG) continue;
        bool hit = cur ? (s->out == SM_OUT_CURRENT || sensor_port_current_shares_dac())
                       : (s->out == SM_OUT_DAC || sensor_port_current_shares_dac());
        if (hit) {
            snprintf(msg, sizeof(msg), "the %s output is played by sensor:%s; stop it first "
                     "({\"cmd\":\"sensor_stop\",\"instance\":\"%s\"})",
                     s->out == SM_OUT_CURRENT ? "current" : "dac", s->name, s->name);
            return msg;
        }
    }
    return NULL;
}
