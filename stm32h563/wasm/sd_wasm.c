/* ============================================================================
 * sd_wasm.c: a small, stable ABI over the sensor definition engine
 * (../src/sensor_def.h) for the WASM build: JSON in, JSON out, through linear
 * memory.  The same file builds on the host (golden_host.c), so the WASM output
 * can be compared byte for byte with a native build.
 *
 * Every call is stateless: it compiles the definition again, runs it and
 * writes one JSON document into the output buffer (out_ptr(); the call returns
 * its length).  The output stays valid until the next call.  README.md
 * describes each call and its JSON.
 * ========================================================================== */
#include "sd_wasm.h"

#include "sensor_def.h"
#include "sensor_def_int.h"

#if defined(__wasm__)
#define EXPORT(name) __attribute__((export_name(name)))
#else
#include <stdlib.h>
#define EXPORT(name)
#endif

#define SDW_ABI_VERSION 1
#define SDW_OUT_MAX    (4u << 20)     /* bytes of one JSON result */
#define SDW_MAX_POINTS 10000u         /* samples per signal in stimulus_sample */
#define SDW_MAX_EPOCHS 200000u        /* UART epochs preview runs to reach t */

/* ---- state of one call (the module is single-threaded; one call at a time) ---- */
static _Alignas(8) uint8_t g_prog[SD_PROG_MAX];
static _Alignas(8) uint8_t g_work[SD_COMPILE_WORK];
static _Alignas(8) uint8_t g_stim[SD_STIM_MAX];
static sd_inst_t g_in;
static char g_out[SDW_OUT_MAX];
static size_t g_out_len;
static bool g_out_over;

/* ---- memory ---- */
#if defined(__wasm__)
extern unsigned char __heap_base;
static uintptr_t g_heap_top;
static uint32_t g_live;

EXPORT("sd_alloc") void *sdw_alloc(uint32_t n) {
    if (!g_heap_top) g_heap_top = (uintptr_t)&__heap_base;
    uintptr_t p = (g_heap_top + 15u) & ~(uintptr_t)15u;
    uintptr_t end = p + (n ? n : 1u);
    uintptr_t have = (uintptr_t)__builtin_wasm_memory_size(0) * 65536u;
    if (end > have) {
        uintptr_t pages = (end - have + 65535u) / 65536u;
        if (__builtin_wasm_memory_grow(0, pages) == (size_t)-1) return NULL;
    }
    g_heap_top = end;
    g_live++;
    return (void *)p;
}

/* Allocations are a stack: memory comes back when every live allocation is freed. */
EXPORT("sd_free") void sdw_free(void *p) {
    if (!p || !g_live) return;
    if (--g_live == 0) g_heap_top = (uintptr_t)&__heap_base;
}
#else
void *sdw_alloc(uint32_t n) { return malloc(n ? n : 1u); }
void sdw_free(void *p) { free(p); }
#endif

EXPORT("out_ptr") const char *sdw_out_ptr(void) { return g_out; }
EXPORT("abi_version") uint32_t sdw_abi_version(void) { return SDW_ABI_VERSION; }

/* ---- JSON output ---- */
static void o_reset(void) { g_out_len = 0; g_out_over = false; }
static void o_ch(char c) {
    if (g_out_len + 1 < SDW_OUT_MAX) g_out[g_out_len++] = c;
    else g_out_over = true;
}
static void o_raw(const char *s, size_t n) { for (size_t i = 0; i < n; i++) o_ch(s[i]); }
static void o_lit(const char *s) { o_raw(s, strlen(s)); }

static const char k_hex[] = "0123456789abcdef";

/* A JSON string.  latin1: every byte >= 0x80 becomes \u00XX (one character per byte, for
   UART text that is not necessarily UTF-8); otherwise bytes pass through (UTF-8 text). */
static void o_strn(const char *s, size_t n, bool latin1) {
    o_ch('"');
    for (size_t i = 0; i < n; i++) {
        unsigned char c = (unsigned char)s[i];
        if (c == '"' || c == '\\') { o_ch('\\'); o_ch((char)c); }
        else if (c == '\n') o_lit("\\n");
        else if (c == '\r') o_lit("\\r");
        else if (c == '\t') o_lit("\\t");
        else if (c < 0x20 || c == 0x7f || (latin1 && c >= 0x80)) {
            o_lit("\\u00"); o_ch(k_hex[c >> 4]); o_ch(k_hex[c & 15]);
        } else o_ch((char)c);
    }
    o_ch('"');
}
static void o_str(const char *s) { o_strn(s, strlen(s), false); }
static void o_key(const char *k) { o_ch(','); o_str(k); o_ch(':'); }
static void o_key1(const char *k) { o_str(k); o_ch(':'); }

static void o_u64(uint64_t u) {
    char d[24];
    unsigned n = 0;
    do { d[n++] = (char)('0' + u % 10u); u /= 10u; } while (u);
    while (n) o_ch(d[--n]);
}
static void o_i64(int64_t v) {
    if (v < 0) { o_ch('-'); o_u64((uint64_t)0 - (uint64_t)v); }
    else o_u64((uint64_t)v);
}
static void o_bool(bool b) { o_lit(b ? "true" : "false"); }

/* ---- shortest round-trip double formatting (Steele & White / Burger & Dybvig, exact
   bignum arithmetic), printed the way JavaScript's Number.prototype.toString does. ---- */
#define BIG_WORDS 40   /* 1280 bits: the largest intermediate is below 2^1100 */
typedef struct { uint32_t w[BIG_WORDS]; unsigned n; } big_t;

static void big_set(big_t *b, uint64_t v) {
    b->w[0] = (uint32_t)v; b->w[1] = (uint32_t)(v >> 32);
    b->n = b->w[1] ? 2 : (b->w[0] ? 1 : 0);
}
static void big_shl(big_t *b, unsigned k) {
    unsigned ws = k / 32, bs = k % 32;
    if (!b->n) return;
    if (bs) {
        uint32_t carry = 0;
        for (unsigned i = 0; i < b->n; i++) {
            uint32_t x = b->w[i];
            b->w[i] = (x << bs) | carry;
            carry = x >> (32 - bs);
        }
        if (carry && b->n < BIG_WORDS) b->w[b->n++] = carry;
    }
    if (ws) {
        for (unsigned i = b->n; i-- > 0;) if (i + ws < BIG_WORDS) b->w[i + ws] = b->w[i];
        for (unsigned i = 0; i < ws; i++) b->w[i] = 0;
        b->n = b->n + ws > BIG_WORDS ? BIG_WORDS : b->n + ws;
    }
}
static void big_mul(big_t *b, uint32_t m) {
    uint64_t carry = 0;
    for (unsigned i = 0; i < b->n; i++) {
        uint64_t x = (uint64_t)b->w[i] * m + carry;
        b->w[i] = (uint32_t)x;
        carry = x >> 32;
    }
    if (carry && b->n < BIG_WORDS) b->w[b->n++] = (uint32_t)carry;
}
static int big_cmp(const big_t *a, const big_t *b) {
    if (a->n != b->n) return a->n < b->n ? -1 : 1;
    for (unsigned i = a->n; i-- > 0;)
        if (a->w[i] != b->w[i]) return a->w[i] < b->w[i] ? -1 : 1;
    return 0;
}
static void big_add(big_t *r, const big_t *a, const big_t *b) {
    unsigned n = a->n > b->n ? a->n : b->n;
    uint64_t carry = 0;
    for (unsigned i = 0; i < n; i++) {
        uint64_t x = carry + (i < a->n ? a->w[i] : 0) + (i < b->n ? b->w[i] : 0);
        r->w[i] = (uint32_t)x;
        carry = x >> 32;
    }
    r->n = n;
    if (carry && n < BIG_WORDS) r->w[r->n++] = (uint32_t)carry;
}
static void big_sub(big_t *a, const big_t *b) {   /* a -= b, a >= b */
    int64_t borrow = 0;
    for (unsigned i = 0; i < a->n; i++) {
        int64_t x = (int64_t)a->w[i] - (i < b->n ? b->w[i] : 0) - borrow;
        borrow = x < 0;
        a->w[i] = (uint32_t)(x + (borrow ? ((int64_t)1 << 32) : 0));
    }
    while (a->n && !a->w[a->n - 1]) a->n--;
}

/* Digits of the shortest decimal that reads back as v (v finite, > 0): v = 0.d1d2... * 10^*k. */
static unsigned shortest_digits(double v, char *dig, int *k_out) {
    uint64_t u;
    memcpy(&u, &v, 8);
    int be = (int)((u >> 52) & 0x7ff);
    uint64_t f = u & ((1ull << 52) - 1);
    int e;
    if (be) { f |= 1ull << 52; e = be - 1075; } else { e = -1074; }
    bool even = (f & 1) == 0;
    static big_t r, s, mp, mm, t;
    bool lower_close = be > 1 && f == (1ull << 52);   /* the gap below is half the gap above */
    if (e >= 0) {
        big_set(&r, f); big_shl(&r, (unsigned)e + (lower_close ? 2 : 1));
        big_set(&s, lower_close ? 4 : 2);
        big_set(&mp, 1); big_shl(&mp, (unsigned)e + (lower_close ? 1 : 0));
        big_set(&mm, 1); big_shl(&mm, (unsigned)e);
    } else {
        big_set(&r, f); big_shl(&r, lower_close ? 2 : 1);
        big_set(&s, 1); big_shl(&s, (unsigned)(-e) + (lower_close ? 2 : 1));
        big_set(&mp, lower_close ? 2 : 1);
        big_set(&mm, 1);
    }
    int k = 0;
    for (;;) {   /* scale so that (r + m+) / s < 1 (<= 1 for an even mantissa) */
        big_add(&t, &r, &mp);
        int c = big_cmp(&t, &s);
        if (c > 0 || (even && c == 0)) { big_mul(&s, 10); k++; } else break;
    }
    for (;;) {   /* and (r + m+) * 10 / s >= 1 */
        big_add(&t, &r, &mp);
        big_mul(&t, 10);
        int c = big_cmp(&t, &s);
        if (c < 0 || (!even && c == 0)) { big_mul(&r, 10); big_mul(&mp, 10); big_mul(&mm, 10); k--; }
        else break;
    }
    unsigned n = 0;
    for (;;) {
        big_mul(&r, 10); big_mul(&mp, 10); big_mul(&mm, 10);
        int d = 0;
        while (big_cmp(&r, &s) >= 0) { big_sub(&r, &s); d++; }
        int c1 = big_cmp(&r, &mm);
        bool low = c1 < 0 || (even && c1 == 0);
        big_add(&t, &r, &mp);
        int c2 = big_cmp(&t, &s);
        bool high = c2 > 0 || (even && c2 == 0);
        if (!low && !high && n < 30) { dig[n++] = (char)('0' + d); continue; }
        if (low && high) {
            big_t r2 = r;
            big_shl(&r2, 1);
            if (big_cmp(&r2, &s) >= 0) d++;
        } else if (high) {
            d++;
        }
        dig[n++] = (char)('0' + d);
        break;
    }
    *k_out = k;
    return n;
}

static void o_num(double v) {
    if (v != v || v == sd_inf() || v == -sd_inf()) { o_lit("null"); return; }
    if (v == 0) { o_ch('0'); return; }
    if (v == __builtin_trunc(v) && v > -9007199254740992.0 && v < 9007199254740992.0) {
        o_i64((int64_t)v);
        return;
    }
    if (v < 0) { o_ch('-'); v = -v; }
    char d[32];
    int k;
    unsigned n = shortest_digits(v, d, &k);
    if (k > 0 && k <= 21) {
        if ((int)n <= k) { o_raw(d, n); for (int i = (int)n; i < k; i++) o_ch('0'); }
        else { o_raw(d, (size_t)k); o_ch('.'); o_raw(d + k, n - (unsigned)k); }
    } else if (k <= 0 && k > -6) {
        o_lit("0.");
        for (int i = 0; i < -k; i++) o_ch('0');
        o_raw(d, n);
    } else {
        o_ch(d[0]);
        if (n > 1) { o_ch('.'); o_raw(d + 1, n - 1); }
        o_ch('e');
        o_ch(k - 1 < 0 ? '-' : '+');
        o_u64((uint64_t)(k - 1 < 0 ? 1 - k : k - 1));
    }
}

/* base64 (RFC 4648, with padding) */
static void o_b64(const uint8_t *p, size_t n) {
    static const char a[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    o_ch('"');
    for (size_t i = 0; i < n; i += 3) {
        uint32_t x = (uint32_t)p[i] << 16;
        if (i + 1 < n) x |= (uint32_t)p[i + 1] << 8;
        if (i + 2 < n) x |= p[i + 2];
        o_ch(a[x >> 18 & 63]); o_ch(a[x >> 12 & 63]);
        o_ch(i + 1 < n ? a[x >> 6 & 63] : '=');
        o_ch(i + 2 < n ? a[x & 63] : '=');
    }
    o_ch('"');
}

static uint32_t finish(void) {
    if (g_out_over) {
        o_reset();
        o_lit("{\"ok\":false,\"error\":\"the result is larger than 4 MB\",\"code\":-8}");
    }
    g_out[g_out_len] = 0;
    return (uint32_t)g_out_len;
}

/* {"ok":false,...} from an engine error */
static uint32_t fail_err(const sd_err_t *e) {
    char m[200];
    sd_err_format(e, m, sizeof(m));
    o_reset();
    o_lit("{\"ok\":false"); o_key("error"); o_str(m);
    o_key("code"); o_i64(e->code);
    o_key("path"); o_str(e->path);
    o_key("message"); o_str(e->msg);
    o_key("json_off"); o_u64(e->json_off);
    o_key("expr_off"); o_i64(e->expr_off);
    o_ch('}');
    return finish();
}

static uint32_t fail_msg(int code, const char *path, const char *msg) {
    sd_err_t e;
    memset(&e, 0, sizeof(e));
    e.code = code;
    e.expr_off = -1;
    size_t i;
    for (i = 0; path && path[i] && i + 1 < sizeof(e.path); i++) e.path[i] = path[i];
    e.path[i] = 0;
    for (i = 0; msg[i] && i + 1 < sizeof(e.msg); i++) e.msg[i] = msg[i];
    e.msg[i] = 0;
    return fail_err(&e);
}

static uint32_t fail_inst(void) { return fail_err(sd_last_error(&g_in)); }

/* ---- shared steps ---- */
static size_t g_prog_len;

static int compile_def(const char *json, uint32_t len, sd_err_t *e) {
    memset(e, 0, sizeof(*e));
    e->expr_off = -1;
    if (!json) return SD_E_ARG;
    int rc = sd_compile(json, len, g_prog, sizeof(g_prog), g_work, sizeof(g_work), &g_prog_len, e);
    if (rc) return rc;
    return sd_inst_init(&g_in, g_prog, g_prog_len, e);
}

static void o_pstr(uint16_t off) {   /* a string record of the program */
    uint8_t n = 0;
    const char *s = off ? sdp_str(g_prog, off, &n) : "";
    o_strn(s, n, false);
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

/* values: {"key": number, ...}; NULL or empty = the defaults.  The params are set without
   clamping, so a value out of range is an error. */
static uint32_t apply_values(const char *vals, uint32_t len, bool *failed) {
    *failed = false;
    if (!vals || !len) return 0;
    uint32_t eo;
    const char *why;
    if (sdj_validate(vals, len, &eo, &why) != 0) {
        *failed = true;
        return fail_msg(SD_E_JSON, "values", why ? why : "not valid JSON");
    }
    sdj_t j = { vals, len };
    uint32_t root = sdj_ws(&j, 0);
    if (sdj_type(&j, root) == 'z') return 0;
    if (sdj_type(&j, root) != '{') {
        *failed = true;
        return fail_msg(SD_E_SCHEMA, "values", "must be an object of param values");
    }
    uint32_t it = root, key, val;
    while (sdj_obj_next(&j, &it, &key, &val)) {
        char k[40], path[72];
        size_t kl = 0;
        if (!sdj_str(&j, key, k, sizeof(k) - 1, &kl)) kl = 0;
        k[kl] = 0;
        size_t pl = 0;
        for (const char *p = "values."; *p; p++) path[pl++] = *p;
        for (size_t i = 0; i < kl && pl + 1 < sizeof(path); i++) path[pl++] = k[i];
        path[pl] = 0;
        int idx = sd_param_index(&g_in, k);
        if (idx < 0) { *failed = true; return fail_msg(SD_E_NAME, path, "unknown param"); }
        double v;
        if (!sdj_num(&j, val, &v)) { *failed = true; return fail_msg(SD_E_SCHEMA, path, "must be a number"); }
        if (sd_set_param(&g_in, (unsigned)idx, v, false) != SD_OK) {
            *failed = true;
            return fail_msg(SD_E_RANGE, path, "outside the param's min..max");
        }
    }
    return 0;
}

static int64_t ms_of(double t_ms) {
    if (!(t_ms > 0)) return 0;
    if (t_ms > 9.0e15) t_ms = 9.0e15;
    return (int64_t)(t_ms + 0.5);   /* round half up (t_ms > 0) */
}

/* ---- validate ---- */
static void o_params(void) {
    o_key("params"); o_ch('[');
    for (unsigned i = 0; i < sd_n_params(&g_in); i++) {
        sd_param_info_t p;
        sd_param_info(&g_in, i, &p);
        if (i) o_ch(',');
        o_ch('{');
        o_key1("key"); o_strn(p.key, p.key_len, false);
        o_key("unit"); o_strn(p.unit ? p.unit : "", p.unit ? p.unit_len : 0, false);
        o_key("label"); o_strn(p.label ? p.label : "", p.label ? p.label_len : 0, false);
        o_key("min"); o_num(p.min);
        o_key("max"); o_num(p.max);
        o_key("default"); o_num(p.def);
        o_ch('}');
    }
    o_ch(']');
}

/* Image addresses rewritten on every rebuild (dynamic) and the ones the pod restores after a
   DUT write (writable: false). */
static void o_addr_lists(void) {
    const sdp_hdr_t *h = sdp_hdr(g_prog);
    const sdp_entry_t *e = (const sdp_entry_t *)(const void *)(g_prog + h->entries);
    uint8_t dyn[SD_REGIMAGE / 8], ro[SD_REGIMAGE / 8];
    memset(dyn, 0, sizeof(dyn));
    memset(ro, 0, sizeof(ro));
    for (unsigned i = 0; i < h->n_entries; i++) {
        if (e[i].flags & SDE_CMD) continue;
        for (unsigned b = 0; b < e[i].len && e[i].at + b < SD_REGIMAGE; b++) {
            unsigned a = e[i].at + b;
            if (e[i].flags & SDE_RO) ro[a / 8] |= (uint8_t)(1u << (a % 8));
        }
    }
    /* dynamic: by the owner map, command answers included (a command map sensor such as an
       SHT3x has nothing but commands, and its answers follow the params) */
    (void)sd_regs_dyn_map(&g_in, dyn);
    for (int list = 0; list < 2; list++) {
        const uint8_t *m = list ? ro : dyn;
        o_key(list ? "read_only" : "dynamic"); o_ch('[');
        bool first = true;
        for (unsigned a = 0; a < SD_REGIMAGE; a++) {
            if (!(m[a / 8] & (1u << (a % 8)))) continue;
            if (!first) o_ch(',');
            first = false;
            o_u64(a);
        }
        o_ch(']');
    }
}

static void o_bus(void) {
    sd_bus_info_t b;
    if (sd_bus_info(&g_in, &b) != SD_OK) return;
    bool spi = sd_kind(&g_in) == SD_KIND_SPI;
    o_key("bus"); o_ch('{');
    if (!spi) {
        o_key1("address"); o_u64(b.address);
        o_key("alt"); o_ch('[');
        for (unsigned i = 0; i < b.n_alt && i < 3; i++) { if (i) o_ch(','); o_u64(b.alt[i]); }
        o_ch(']');
        o_key("addr_bits"); o_u64(b.addr_bits);
        o_key("page"); o_u64(b.page);
        o_key("auto_increment"); o_bool(b.auto_increment);
        o_key("wrap"); o_u64(b.wrap);
        o_key("register_bytes"); o_u64(b.register_bytes);
        o_key("command_map"); o_bool(b.command_map);
        if (b.command_map) {   /* image[h] ^ l lands on `at`: the bytes of addr .. addr + span - 1 */
            o_key("map"); o_ch('[');
            for (unsigned i = 0; i < b.n_map_blocks; i++) {
                sd_map_block_t m;
                if (sd_map_block(&g_in, i, &m) != SD_OK) continue;
                if (i) o_ch(',');
                o_ch('{'); o_key1("addr"); o_u64(m.addr); o_key("at"); o_u64(m.at); o_key("span"); o_u64(m.span); o_ch('}');
            }
            o_ch(']');
        }
    } else {
        o_key1("spi_mode"); o_u64(b.spi_mode);
        o_key("read_bit"); o_u64(b.read_bit);
        o_key("read_val"); o_u64(b.read_val);
        o_key("inc_bit"); if (b.has_inc_bit) o_u64(b.inc_bit); else o_lit("null");
        o_key("inc_val"); o_u64(b.inc_val);
        o_key("address_mask"); o_u64(b.address_mask);
        o_key("dummy_bytes"); o_u64(b.dummy_bytes);
    }
    o_key("conversion");
    if (b.has_conversion) {
        static const char *const wb[] = { "flag", "nack", "stretch" };
        o_ch('{');
        o_key1("trigger"); o_u64(b.conv_trigger);
        o_key("busy_at"); o_u64(b.conv_busy_at);
        o_key("busy_mask"); o_u64(b.conv_busy_mask);
        o_key("while_busy"); o_str(b.while_busy < 3 ? wb[b.while_busy] : "flag");
        o_key("us"); o_u64(b.conv_us);
        o_ch('}');
    } else o_lit("null");
    o_key("data_ready");
    if (b.has_drdy) {
        static const char *const on[] = { "conversion", "sample", "rate" };
        o_ch('{');
        o_key1("active_high"); o_bool(b.drdy_active_high != 0);
        o_key("level"); o_bool(b.drdy_level != 0);
        o_key("on"); o_str(b.drdy_on < 3 ? on[b.drdy_on] : "conversion");
        o_key("pulse_us"); o_u64(b.drdy_pulse_us);
        o_key("clear_on_read"); if (b.drdy_clear_on_read >= 0) o_i64(b.drdy_clear_on_read); else o_lit("null");
        o_ch('}');
    } else o_lit("null");
    o_key("fifo");
    if (b.has_fifo) {
        o_ch('{');
        o_key1("at"); o_u64(b.fifo_at);
        o_key("frame_len"); o_u64(b.fifo_frame_len);
        o_key("depth"); o_u64(b.fifo_depth);
        o_key("drop"); o_str(b.fifo_drop_newest ? "newest" : "oldest");
        o_ch('}');
    } else o_lit("null");
    o_key("commands"); o_ch('[');
    for (unsigned i = 0; i < b.n_commands; i++) {
        sd_command_t c;
        if (sd_command(&g_in, i, &c) != SD_OK) continue;
        if (i) o_ch(',');
        o_ch('{'); o_key1("at"); o_u64(c.at); o_key("busy_us"); o_u64(c.busy_us); o_key("command"); o_u64(c.addr); o_ch('}');
    }
    o_ch(']');
    o_ch('}');
    o_addr_lists();
}

EXPORT("validate") uint32_t sdw_validate(const char *def, uint32_t def_len) {
    sd_err_t e;
    if (compile_def(def, def_len, &e)) return fail_err(&e);
    const sdp_hdr_t *h = sdp_hdr(g_prog);
    sd_kind_t k = sd_kind(&g_in);
    o_reset();
    o_lit("{\"ok\":true");
    o_key("kind"); o_str(kind_name(k));
    o_key("name"); o_pstr(h->name);
    o_key("label"); o_pstr(h->label);
    o_key("program_bytes"); o_u64(g_prog_len);
    o_params();
    if (k == SD_KIND_I2C || k == SD_KIND_SPI) o_bus();
    if (k == SD_KIND_UART) {
        sd_uart_info_t u;
        sd_uart_info(&g_in, &u);
        char par[2] = { (char)u.parity, 0 };
        o_key("uart"); o_ch('{');
        o_key1("baud"); o_u64(u.baud);
        o_key("data_bits"); o_u64(u.data_bits);
        o_key("parity"); o_str(par);
        o_key("stop_bits"); o_u64(u.stop_bits);
        o_key("messages"); o_u64(h->n_msgs);
        o_key("requests"); o_u64(h->n_reqs);
        o_ch('}');
    }
    if (k == SD_KIND_ANALOG) {
        sd_analog_info_t a;
        sd_analog_info(&g_in, &a);
        o_key("analog"); o_ch('{');
        o_key1("output"); o_str(a.current ? "current" : "voltage");
        o_key("excitation"); o_bool(a.excitation);
        o_key("min"); o_num(a.min);
        o_key("max"); o_num(a.max);
        o_ch('}');
    }
    o_ch('}');
    return finish();
}

/* ---- preview ---- */
static void o_bitlist(const uint8_t *bits, unsigned n) {
    o_ch('[');
    bool first = true;
    for (unsigned a = 0; a < n; a++) {
        if (!(bits[a / 8] & (1u << (a % 8)))) continue;
        if (!first) o_ch(',');
        first = false;
        o_u64(a);
    }
    o_ch(']');
}

EXPORT("preview") uint32_t sdw_preview(const char *def, uint32_t def_len, const char *vals, uint32_t vals_len,
                                       const uint8_t *live, uint32_t live_len, double t_ms, double input) {
    sd_err_t e;
    if (compile_def(def, def_len, &e)) return fail_err(&e);
    bool failed;
    uint32_t r = apply_values(vals, vals_len, &failed);
    if (failed) return r;
    int64_t t = ms_of(t_ms);
    sd_kind_t k = sd_kind(&g_in);
    if (k == SD_KIND_I2C || k == SD_KIND_SPI) {
        if (live && live_len != SD_REGIMAGE) return fail_msg(SD_E_ARG, "live_regs", "must be 256 bytes");
        static uint8_t base[SD_REGIMAGE], img[SD_REGIMAGE];
        sd_img_result_t res;
        if (live) {
            memcpy(base, live, SD_REGIMAGE);
        } else {
            sd_set_time_ms(&g_in, 0);
            if (sd_regs_power_on(&g_in, base, &res) != SD_OK) return fail_inst();
        }
        sd_set_time_ms(&g_in, t);
        if (sd_regs_rebuild(&g_in, base, img, &res) != SD_OK) return fail_inst();
        o_reset();
        o_lit("{\"ok\":true");
        o_key("kind"); o_str(kind_name(k));
        o_key("t_ms"); o_i64(t);
        o_key("power_on"); o_bool(!live);
        o_key("image_b64"); o_b64(img, SD_REGIMAGE);
        o_key("changed"); o_bitlist(res.changed, SD_REGIMAGE);
        o_addr_lists();
        o_ch('}');
        return finish();
    }
    if (k == SD_KIND_UART) {
        static char ep[SDW_OUT_MAX / 8];
        size_t n = 0;
        int64_t at = -1;
        bool have = false;
        unsigned epochs = 0;
        while (sd_uart_next_ms(&g_in) >= 0 && sd_uart_next_ms(&g_in) <= t) {
            if (++epochs > SDW_MAX_EPOCHS) return fail_msg(SD_E_LIMIT, "t_ms", "more than 200000 epochs before t");
            if (sd_uart_epoch(&g_in, ep, sizeof(ep), &n, &at) != SD_OK) return fail_inst();
            have = true;
        }
        o_reset();
        o_lit("{\"ok\":true");
        o_key("kind"); o_lit("\"uart\"");
        o_key("t_ms"); o_i64(t);
        o_key("epochs"); o_u64(epochs);
        o_key("at_ms"); if (have) o_i64(at); else o_lit("null");
        o_key("text"); o_strn(ep, have ? n : 0, true);
        o_key("bytes_b64"); o_b64((const uint8_t *)ep, have ? n : 0);
        o_key("next_ms"); o_i64(sd_uart_next_ms(&g_in));
        o_ch('}');
        return finish();
    }
    if (k == SD_KIND_ANALOG) {
        double v;
        sd_set_time_ms(&g_in, t);
        if (sd_analog_eval(&g_in, input, &v) != SD_OK) return fail_inst();
        sd_analog_info_t a;
        sd_analog_info(&g_in, &a);
        o_reset();
        o_lit("{\"ok\":true");
        o_key("kind"); o_lit("\"analog\"");
        o_key("t_ms"); o_i64(t);
        o_key("value"); o_num(v);
        o_key("unit"); o_str(a.current ? "mA" : "V");
        o_ch('}');
        return finish();
    }
    return fail_msg(SD_E_ARG, "kind", "unknown kind");
}

/* ---- uart_answer ---- */
EXPORT("uart_answer") uint32_t sdw_uart_answer(const char *def, uint32_t def_len, const char *vals, uint32_t vals_len,
                                               const uint8_t *req, uint32_t req_len, double t_ms) {
    sd_err_t e;
    if (compile_def(def, def_len, &e)) return fail_err(&e);
    if (sd_kind(&g_in) != SD_KIND_UART) return fail_msg(SD_E_ARG, "kind", "uart_answer needs a uart sensor");
    bool failed;
    uint32_t r = apply_values(vals, vals_len, &failed);
    if (failed) return r;
    int64_t t = ms_of(t_ms);
    static uint8_t ans[4096];
    /* the answers go to a second buffer first: the JSON is written after the run */
    static struct { uint32_t after, delay, off, len; } list[64];
    static uint8_t all[SDW_OUT_MAX / 8];
    unsigned na = 0;
    size_t used = 0;
    for (uint32_t i = 0; i < req_len; i++) {
        size_t n = 0;
        uint32_t d = 0;
        int rc = sd_uart_rx(&g_in, t, req ? req[i] : 0, ans, sizeof(ans), &n, &d);
        if (rc < 0) return fail_inst();
        if (rc == 1 && na < 64 && used + n <= sizeof(all)) {
            memcpy(all + used, ans, n);
            list[na].after = i; list[na].delay = d; list[na].off = (uint32_t)used; list[na].len = (uint32_t)n;
            used += n;
            na++;
        }
    }
    o_reset();
    o_lit("{\"ok\":true");
    o_key("answers"); o_ch('[');
    for (unsigned i = 0; i < na; i++) {
        if (i) o_ch(',');
        o_ch('{');
        o_key1("after_byte"); o_u64(list[i].after);
        o_key("delay_ms"); o_u64(list[i].delay);
        o_key("text"); o_strn((const char *)all + list[i].off, list[i].len, true);
        o_key("bytes_b64"); o_b64(all + list[i].off, list[i].len);
        o_ch('}');
    }
    o_ch(']');
    o_ch('}');
    return finish();
}

/* ---- stimulus ---- */
static sd_stim_t g_st;

static int compile_stim(const char *json, uint32_t len, sd_err_t *e) {
    memset(e, 0, sizeof(*e));
    e->expr_off = -1;
    if (!json) return SD_E_ARG;
    size_t n;
    int rc = sd_stim_compile(json, len, g_stim, sizeof(g_stim), &n, e);
    if (rc) return rc;
    g_prog_len = n;
    return sd_stim_init(&g_st, g_stim, n);
}

static void o_signal_name(unsigned i) {
    const char *in, *pa;
    size_t il, pl;
    sd_stim_signal(&g_st, i, &in, &il, &pa, &pl);
    char nm[80];
    size_t n = 0;
    for (size_t k = 0; k < il && n < sizeof(nm); k++) nm[n++] = in[k];
    if (il && n < sizeof(nm)) nm[n++] = '.';
    for (size_t k = 0; k < pl && n < sizeof(nm); k++) nm[n++] = pa[k];
    o_key1("name"); o_strn(nm, n, false);
    o_key("instance"); o_strn(il ? in : "", il, false);
    o_key("param"); o_strn(pa, pl, false);
}

static void o_fault(const sd_fault_t *f) {
    o_key1("type"); o_str(sd_fault_name(f->type));
    o_key("at_s"); o_num(f->at_s);
    o_key("duration_s"); if (f->duration_s < 0) o_lit("null"); else o_num(f->duration_s);
    o_key("sensor"); o_str(f->sensor);
    o_key("param"); o_str(f->param);
    o_key("value"); if (f->has_value) o_num(f->value); else o_lit("null");
    if (f->type == SD_FAULT_GARBAGE) { o_key("rate_hz"); o_num(f->rate_hz); }
    if (f->type == SD_FAULT_CORRUPT) {
        o_key("at"); if (f->at >= 0) o_i64(f->at); else o_lit("null");
        o_key("mask"); o_u64(f->mask);
    }
}

/* The stimulus's own name (optional in the file). */
static void o_stim_name(const char *json, uint32_t len) {
    sdj_t j = { json, len };
    uint32_t root = sdj_ws(&j, 0), v;
    char nm[40];
    size_t n = 0;
    if (sdj_type(&j, root) == '{' && sdj_find(&j, root, "name", &v) && sdj_type(&j, v) == '"' &&
        sdj_str(&j, v, nm, sizeof(nm), &n)) {
        o_key("name"); o_strn(nm, n, false);
    } else {
        o_key("name"); o_lit("null");
    }
}

EXPORT("stimulus_validate") uint32_t sdw_stimulus_validate(const char *json, uint32_t len) {
    sd_err_t e;
    if (compile_stim(json, len, &e)) return fail_err(&e);
    o_reset();
    o_lit("{\"ok\":true");
    o_key("kind"); o_lit("\"stimulus\"");
    o_stim_name(json, len);
    o_key("program_bytes"); o_u64(g_prog_len);
    o_key("rate_hz"); o_num(sd_stim_rate_hz(&g_st));
    o_key("duration_s"); o_num(sd_stim_duration_s(&g_st));
    o_key("repeat"); o_bool(sd_stim_repeats(&g_st));
    o_key("signals"); o_ch('[');
    for (unsigned i = 0; i < sd_stim_n_signals(&g_st); i++) {
        if (i) o_ch(',');
        o_ch('{'); o_signal_name(i); o_ch('}');
    }
    o_ch(']');
    o_key("faults"); o_ch('[');
    for (unsigned i = 0; i < sd_stim_n_faults(&g_st); i++) {
        sd_fault_t f;
        sd_stim_fault(&g_st, i, &f);
        if (i) o_ch(',');
        o_ch('{'); o_fault(&f); o_ch('}');
    }
    o_ch(']');
    o_ch('}');
    return finish();
}

EXPORT("stimulus_sample") uint32_t sdw_stimulus_sample(const char *json, uint32_t len, double from_s, double to_s,
                                                       double step_s) {
    sd_err_t e;
    if (compile_stim(json, len, &e)) return fail_err(&e);
    if (!(step_s > 0) || !(to_s >= from_s) || !(from_s >= 0) || to_s - from_s > step_s * (SDW_MAX_POINTS - 1) + step_s * 1e-9)
        return fail_msg(SD_E_ARG, "step_s", "need 0 <= from_s <= to_s, step_s > 0 and at most 10000 points");
    unsigned np = (unsigned)((to_s - from_s) / step_s + 1e-9) + 1;
    unsigned ns = sd_stim_n_signals(&g_st), nf = sd_stim_n_faults(&g_st);
    o_reset();
    o_lit("{\"ok\":true");
    o_key("from_s"); o_num(from_s);
    o_key("to_s"); o_num(to_s);
    o_key("step_s"); o_num(step_s);
    o_key("t"); o_ch('[');
    for (unsigned p = 0; p < np; p++) { if (p) o_ch(','); o_num(from_s + p * step_s); }
    o_ch(']');
    o_key("signals"); o_ch('[');
    for (unsigned i = 0; i < ns; i++) {
        if (i) o_ch(',');
        o_ch('{'); o_signal_name(i);
        o_key("values"); o_ch('[');
        for (unsigned p = 0; p < np; p++) { if (p) o_ch(','); o_num(sd_stim_value(&g_st, i, from_s + p * step_s)); }
        o_ch(']');
        o_ch('}');
    }
    o_ch(']');
    /* fault windows, as the player switches them at these sample times */
    o_key("faults"); o_ch('[');
    for (unsigned fi = 0; fi < nf; fi++) {
        sd_fault_t f;
        sd_stim_fault(&g_st, fi, &f);
        if (fi) o_ch(',');
        o_ch('{');
        o_key1("index"); o_u64(fi);
        o_ch(','); o_fault(&f);
        o_key("windows"); o_ch('[');
        sd_stim_t s;
        sd_stim_init(&s, g_stim, g_prog_len);
        sd_stim_event_t ev[SD_STIM_SIGNALS + SD_STIM_FAULTS];
        bool on = false, first = true;
        for (unsigned p = 0; p < np; p++) {
            double t = from_s + p * step_s;
            int k = sd_stim_step(&s, t, ev, SD_STIM_SIGNALS + SD_STIM_FAULTS);
            for (int x = 0; x < k; x++) {
                if (ev[x].kind == SD_EV_VALUE || ev[x].index != fi) continue;
                if (ev[x].kind == SD_EV_FAULT_ON && !on) {
                    if (!first) o_ch(',');
                    first = false;
                    o_ch('['); o_num(t); o_ch(',');
                    on = true;
                } else if (ev[x].kind == SD_EV_FAULT_OFF && on) {
                    o_num(t); o_ch(']');
                    on = false;
                }
            }
        }
        if (on) o_lit("null]");
        o_ch(']');
        o_ch('}');
    }
    o_ch(']');
    o_ch('}');
    return finish();
}
