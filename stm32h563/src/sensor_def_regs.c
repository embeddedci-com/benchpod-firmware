/* ============================================================================
 * sensor_def_regs.c — I2C / SPI register images of a sensor definition.
 *
 * The image is 256 bytes the gateware serves.  Every entry (registers, then
 * commands) covers some of them; where entries overlap, the later one owns the
 * byte (the owner map the compiler made).  A constant entry is loaded at power
 * on and on a reset, and after that the byte holds what the DUT writes; a
 * dynamic one (it reads a param, reg[], t, a state, ...) and a writable:false
 * one are rewritten on every rebuild.  Values saturate to their field type.
 * ========================================================================== */
#include "sensor_def_int.h"

#include <math.h>

static const struct { uint8_t bytes; bool sgn, be; } k_fmt[SDT_BYTES] = {
    [SDT_U8] = { 1, false, true },    [SDT_S8] = { 1, true, true },
    [SDT_U16BE] = { 2, false, true }, [SDT_U16LE] = { 2, false, false },
    [SDT_S16BE] = { 2, true, true },  [SDT_S16LE] = { 2, true, false },
    [SDT_U24BE] = { 3, false, true }, [SDT_U24LE] = { 3, false, false },
    [SDT_S24BE] = { 3, true, true },  [SDT_S24LE] = { 3, true, false },
    [SDT_U32BE] = { 4, false, true }, [SDT_U32LE] = { 4, false, false },
    [SDT_S32BE] = { 4, true, true },  [SDT_S32LE] = { 4, true, false },
};

static const sdp_entry_t *entries(const sd_inst_t *in) {
    return (const sdp_entry_t *)(const void *)(in->prog + sd_h(in)->entries);
}

static bool is_regs(const sd_inst_t *in) {
    return in && in->prog && (sd_h(in)->kind == SD_KIND_I2C || sd_h(in)->kind == SD_KIND_SPI);
}

static int eval_byte(sd_inst_t *in, uint16_t code, uint8_t *out) {
    double v;
    int rc = sd_vm_num(in, code, &v);
    if (rc) return rc;
    *out = (uint8_t)sd_sat(v, 8, false);
    return SD_OK;
}

/* Write entry e (index idx) into img.  owned: only the bytes this entry owns. */
static int write_entry(sd_inst_t *in, const sdp_entry_t *e, unsigned idx, uint8_t *img, bool owned) {
    const sdp_hdr_t *h = sd_h(in);
    const uint8_t *own = in->prog + h->owner;
    unsigned span = (unsigned)h->wrap + 1u;
    if (e->type < SDT_BYTES) {
        double v;
        int rc = sd_vm_num(in, e->code, &v);
        if (rc) return rc;
        unsigned nb = k_fmt[e->type].bytes;
        uint64_t u = (uint64_t)(int64_t)sd_sat(v, 8 * (int)nb, k_fmt[e->type].sgn);
        for (unsigned b = 0; b < nb; b++) {
            unsigned a = (e->at + b) % span;
            if (owned && own[a] != idx) continue;
            unsigned shift = k_fmt[e->type].be ? 8u * (nb - 1u - b) : 8u * b;
            img[a] = (uint8_t)(u >> shift);
        }
        return SD_OK;
    }
    for (unsigned b = 0; b < e->len; b++) {
        unsigned a = (e->at + b) % span;
        if (owned && own[a] != idx) continue;
        if (e->type == SDT_RAW) { img[a] = in->prog[e->code + b]; continue; }
        int rc = eval_byte(in, sdp_code_at(in->prog, e->code, b), &img[a]);
        if (rc) return rc;
    }
    return SD_OK;
}

static void diff(const sd_inst_t *in, const uint8_t *base, int16_t old_at, uint8_t old_mask,
                 const uint8_t *img, sd_img_result_t *res) {
    (void)in;
    memset(res, 0, sizeof(*res));
    for (unsigned i = 0; i < SD_REGIMAGE; i++) {
        uint8_t orig = (uint8_t)(base[i] ^ ((int)i == old_at ? old_mask : 0));
        if (orig != img[i]) {
            res->changed[i / 8] |= (uint8_t)(1u << (i % 8));
            if (!res->n_changed) res->lo = (uint8_t)i;
            res->hi = (uint8_t)i;
            res->n_changed++;
        }
    }
}

/* in->img_base = live without the corrupt fault's XOR; reg[] reads it. */
static void take_base(sd_inst_t *in, const uint8_t *live) {
    memcpy(in->img_base, live, SD_REGIMAGE);
    if (in->corrupt_applied_at >= 0) in->img_base[in->corrupt_applied_at] ^= in->corrupt_applied_mask;
    in->regsrc = in->img_base;
}

static void apply_corrupt(sd_inst_t *in, uint8_t *img) {
    in->corrupt_applied_at = in->corrupt_at;
    in->corrupt_applied_mask = in->corrupt_mask;
    if (in->corrupt_at >= 0) img[in->corrupt_at] ^= in->corrupt_mask;
}

/* Rewrite the dynamic and writable:false entries of img (img == the base image). */
static int rewrite(sd_inst_t *in, uint8_t *img) {
    const sdp_hdr_t *h = sd_h(in);
    const sdp_entry_t *e = entries(in);
    for (unsigned i = 0; i < h->n_entries; i++) {
        if (!(e[i].flags & (SDE_DYN | SDE_RO))) continue;
        int rc = write_entry(in, &e[i], i, img, true);
        if (rc) return rc;
    }
    return SD_OK;
}

static int power_on(sd_inst_t *in, uint8_t img[SD_REGIMAGE], sd_img_result_t *res) {
    const sdp_hdr_t *h = sd_h(in);
    const sdp_entry_t *e = entries(in);
    sd_vm_begin(in);
    /* phase 1: every entry over the fill (reg[] sees the fill) */
    memset(in->img_base, h->fill, SD_REGIMAGE);
    in->regsrc = in->img_base;
    int rc = sd_prepare(in);
    if (rc) return rc;
    uint8_t *tmp = img;
    memset(tmp, h->fill, SD_REGIMAGE);
    for (unsigned i = 0; i < h->n_entries; i++) {
        rc = write_entry(in, &e[i], i, tmp, true);
        if (rc) return rc;
    }
    /* phase 2: the data entries again, against the power-on configuration (only when an
       expression reads reg[]; otherwise phase 1 already gave the final values) */
    if (h->uses_reg) {
        memcpy(in->img_base, tmp, SD_REGIMAGE);
        rc = sd_prepare(in);
        if (rc) return rc;
        rc = rewrite(in, img);
        if (rc) return rc;
    }
    apply_corrupt(in, img);
    memset(res, 0, sizeof(*res));
    memset(res->changed, 0xFF, sizeof(res->changed));
    res->n_changed = SD_REGIMAGE;
    res->lo = 0;
    res->hi = 0xFF;
    res->reset = true;
    return SD_OK;
}

int sd_regs_power_on(sd_inst_t *in, uint8_t img[SD_REGIMAGE], sd_img_result_t *res) {
    if (!is_regs(in) || !img || !res) return SD_E_ARG;
    in->err.code = 0;
    return power_on(in, img, res);
}

static int rebuild(sd_inst_t *in, const uint8_t *live, uint8_t *img, sd_img_result_t *res, bool prepared) {
    int16_t old_at = in->corrupt_applied_at;
    uint8_t old_mask = in->corrupt_applied_mask;
    if (!prepared) {
        sd_vm_begin(in);
        take_base(in, live);
    }
    if (img != live) memcpy(img, in->img_base, SD_REGIMAGE);
    else if (old_at >= 0) img[old_at] ^= old_mask;
    if (in->frozen && in->started) {
        apply_corrupt(in, img);
        diff(in, in->img_base, old_at, old_mask, img, res);
        return SD_OK;
    }
    int rc = prepared ? SD_OK : sd_prepare(in);
    if (!rc) rc = rewrite(in, img);
    if (rc) return rc;
    apply_corrupt(in, img);
    diff(in, in->img_base, old_at, old_mask, img, res);
    return SD_OK;
}

int sd_regs_rebuild(sd_inst_t *in, const uint8_t live[SD_REGIMAGE], uint8_t img[SD_REGIMAGE],
                    sd_img_result_t *res) {
    if (!is_regs(in) || !live || !img || !res) return SD_E_ARG;
    in->err.code = 0;
    return rebuild(in, live, img, res, false);
}

int sd_regs_after_write(sd_inst_t *in, const uint8_t live[SD_REGIMAGE], int addr, int value,
                        uint8_t img[SD_REGIMAGE], sd_img_result_t *res) {
    if (!is_regs(in) || !live || !img || !res) return SD_E_ARG;
    const sdp_hdr_t *h = sd_h(in);
    in->err.code = 0;
    in->vars[SDV_BUILTIN + SDB_WRITTEN] = sdv_num(addr < 0 ? -1.0 : (double)(addr + h->page));
    in->vars[SDV_BUILTIN + SDB_VALUE] = sdv_num(value < 0 ? -1.0 : (double)value);
    sd_vm_begin(in);
    take_base(in, live);
    int rc = sd_prepare(in);
    if (rc) return rc;
    const sdp_rule_t *r = (const sdp_rule_t *)(const void *)(in->prog + h->rules);
    uint32_t sets = 0;
    for (unsigned i = 0; i < h->n_rules; i++) {
        double w;
        rc = sd_vm_num(in, r[i].when, &w);
        if (rc) return rc;
        if (w == 0) continue;
        if (r[i].reset) return power_on(in, img, res);
        sets |= 1u << i;
    }
    int16_t old_at = in->corrupt_applied_at;
    uint8_t old_mask = in->corrupt_applied_mask;
    rc = rebuild(in, live, img, res, true);
    if (rc) return rc;
    if (sets) {
        if (in->corrupt_applied_at >= 0) img[in->corrupt_applied_at] ^= in->corrupt_applied_mask;
        for (unsigned i = 0; i < h->n_rules; i++) {
            if (!(sets & (1u << i))) continue;
            rc = write_entry(in, &r[i].set, 0, img, false);
            if (rc) return rc;
        }
        if (in->corrupt_applied_at >= 0) img[in->corrupt_applied_at] ^= in->corrupt_applied_mask;
        diff(in, in->img_base, old_at, old_mask, img, res);
    }
    return SD_OK;
}

/* ---- bus info, data-ready, FIFO ---- */
int sd_bus_info(const sd_inst_t *in, sd_bus_info_t *o) {
    if (!is_regs(in) || !o) return SD_E_ARG;
    const sdp_hdr_t *h = sd_h(in);
    memset(o, 0, sizeof(*o));
    o->address = h->addr7;
    o->n_alt = h->n_alt;
    memcpy(o->alt, h->alt, sizeof(o->alt));
    o->addr_bits = h->addr_bits;
    o->page = h->page;
    o->auto_increment = h->auto_inc;
    o->wrap = h->wrap;
    o->spi_mode = h->spi_mode;
    o->read_bit = h->read_bit;
    o->read_val = h->read_val;
    o->has_inc_bit = h->has_inc;
    o->inc_bit = h->inc_bit;
    o->inc_val = h->inc_val;
    o->address_mask = h->address_mask;
    o->dummy_bytes = h->dummy_bytes;
    o->has_conversion = h->has_conv;
    o->conv_trigger = h->conv_trigger;
    o->conv_busy_at = h->conv_busy_at;
    o->conv_busy_mask = h->conv_busy_mask;
    o->while_busy = h->while_busy;
    o->conv_us = h->conv_us;
    o->has_drdy = h->has_drdy;
    o->drdy_active_high = h->drdy_high;
    o->drdy_level = h->drdy_level;
    o->drdy_on = h->drdy_on;
    o->drdy_pulse_us = h->drdy_pulse_us;
    o->drdy_clear_on_read = h->drdy_clear;
    o->has_fifo = h->has_fifo;
    o->fifo_at = h->fifo_at;
    o->fifo_frame_len = h->fifo_len;
    o->fifo_drop_newest = h->fifo_drop_newest;
    o->fifo_depth = h->fifo_depth;
    o->n_commands = h->n_cmds;
    return SD_OK;
}

int sd_regs_ro_map(const sd_inst_t *in, uint8_t ro[SD_REGIMAGE / 8]) {
    if (!is_regs(in) || !ro) return SD_E_ARG;
    const sdp_hdr_t *h = sd_h(in);
    const uint8_t *own = in->prog + h->owner;
    const sdp_entry_t *e = entries(in);
    memset(ro, 0, SD_REGIMAGE / 8);
    for (unsigned a = 0; a < SD_REGIMAGE; a++)
        if (own[a] < h->n_entries && (e[own[a]].flags & SDE_RO)) ro[a >> 3] |= (uint8_t)(1u << (a & 7u));
    return SD_OK;
}

int sd_command(const sd_inst_t *in, unsigned i, sd_command_t *out) {
    if (!is_regs(in) || !out) return SD_E_ARG;
    const sdp_hdr_t *h = sd_h(in);
    unsigned first = (unsigned)h->n_entries - h->n_cmds;
    if (i >= h->n_cmds) return SD_E_ARG;
    const sdp_entry_t *e = &entries(in)[first + i];
    out->at = e->at;
    out->busy_us = e->busy_us;
    return SD_OK;
}

static int eval_rate_enable(sd_inst_t *in, const uint8_t *live, uint16_t rate, uint16_t en,
                            double *hz, bool *enabled) {
    sd_vm_begin(in);
    in->err.code = 0;
    take_base(in, live);
    int rc = sd_prepare(in);
    if (rc) return rc;
    double r = 0, e = 1;
    if (rate && (rc = sd_vm_num(in, rate, &r))) return rc;
    if (en && (rc = sd_vm_num(in, en, &e))) return rc;
    if (!(r == r) || r < 0) r = 0;
    *hz = r;
    *enabled = e != 0;
    return SD_OK;
}

int sd_drdy_eval(sd_inst_t *in, const uint8_t live[SD_REGIMAGE], double *rate_hz, bool *enabled) {
    if (!is_regs(in) || !live || !rate_hz || !enabled || !sd_h(in)->has_drdy) return SD_E_ARG;
    return eval_rate_enable(in, live, sd_h(in)->drdy_rate, sd_h(in)->drdy_enabled, rate_hz, enabled);
}

int sd_fifo_eval(sd_inst_t *in, const uint8_t live[SD_REGIMAGE], double *rate_hz, bool *enabled) {
    if (!is_regs(in) || !live || !rate_hz || !enabled || !sd_h(in)->has_fifo) return SD_E_ARG;
    return eval_rate_enable(in, live, sd_h(in)->fifo_rate, sd_h(in)->fifo_enabled, rate_hz, enabled);
}

int sd_fifo_frame(sd_inst_t *in, const uint8_t live[SD_REGIMAGE], uint8_t *out, size_t cap, size_t *len) {
    if (!is_regs(in) || !live || !out || !len || !sd_h(in)->has_fifo) return SD_E_ARG;
    const sdp_hdr_t *h = sd_h(in);
    if (cap < h->fifo_len) return SD_E_SPACE;
    sd_vm_begin(in);
    in->err.code = 0;
    take_base(in, live);
    int rc = sd_prepare(in);
    if (rc) return rc;
    for (unsigned i = 0; i < h->fifo_len; i++)
        if ((rc = eval_byte(in, sdp_code_at(in->prog, h->fifo_frame, i), &out[i]))) return rc;
    *len = h->fifo_len;
    return SD_OK;
}

void sd_set_fifo_level(sd_inst_t *in, unsigned frames) {
    in->vars[SDV_BUILTIN + SDB_FIFO] = sdv_num((double)frames);
}
