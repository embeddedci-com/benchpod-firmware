/* ============================================================================
 * sensor_def_vm.c — the expression VM of the sensor definition engine.
 *
 * A stack machine over 8-byte values: a double, or a string NaN-boxed into one
 * (in the program, or in the instance's scratch).  Functions and fragments run
 * in frames on the same value stack (arguments, then their let values); a
 * template appends text to the caller's output (top level) or to the scratch
 * (inside a fragment, whose text becomes its string result).
 *
 * Every evaluation is bounded: SD_STEP_LIMIT ops, SD_STACK values, SD_FRAMES
 * calls, SD_SCRATCH bytes of strings, SD_SOLVE_MAX_ITER bisection steps.
 * ========================================================================== */
#include "sensor_def_int.h"

#include <math.h>   /* floor, ceil, trunc, round, fmod, sqrt, fabs: exact */

const sd_builtin_t sd_builtins[BI_N] = {
    [BI_ABS] = { "abs", 1, 1, false },      [BI_MIN] = { "min", 2, 16, false },
    [BI_MAX] = { "max", 2, 16, false },     [BI_CLAMP] = { "clamp", 3, 3, false },
    [BI_ROUND] = { "round", 1, 1, false },  [BI_FLOOR] = { "floor", 1, 1, false },
    [BI_CEIL] = { "ceil", 1, 1, false },    [BI_TRUNC] = { "trunc", 1, 1, false },
    [BI_SQRT] = { "sqrt", 1, 1, false },    [BI_EXP] = { "exp", 1, 1, false },
    [BI_LOG] = { "log", 1, 1, false },      [BI_LOG10] = { "log10", 1, 1, false },
    [BI_POW] = { "pow", 2, 2, false },      [BI_SIN] = { "sin", 1, 1, false },
    [BI_COS] = { "cos", 1, 1, false },      [BI_TAN] = { "tan", 1, 1, false },
    [BI_ASIN] = { "asin", 1, 1, false },    [BI_ACOS] = { "acos", 1, 1, false },
    [BI_ATAN] = { "atan", 1, 1, false },    [BI_ATAN2] = { "atan2", 2, 2, false },
    [BI_RAD] = { "rad", 1, 1, false },      [BI_DEG] = { "deg", 1, 1, false },
    [BI_IDIV] = { "idiv", 2, 2, false },    [BI_IMOD] = { "imod", 2, 2, false },
    [BI_U8] = { "u8", 1, 1, false },        [BI_S8] = { "s8", 1, 1, false },
    [BI_U16] = { "u16", 1, 1, false },      [BI_S16] = { "s16", 1, 1, false },
    [BI_U24] = { "u24", 1, 1, false },      [BI_S24] = { "s24", 1, 1, false },
    [BI_U32] = { "u32", 1, 1, false },      [BI_S32] = { "s32", 1, 1, false },
    [BI_SAT_U8] = { "sat_u8", 1, 1, false },   [BI_SAT_S8] = { "sat_s8", 1, 1, false },
    [BI_SAT_U16] = { "sat_u16", 1, 1, false }, [BI_SAT_S16] = { "sat_s16", 1, 1, false },
    [BI_SAT_U24] = { "sat_u24", 1, 1, false }, [BI_SAT_S24] = { "sat_s24", 1, 1, false },
    [BI_SAT_U32] = { "sat_u32", 1, 1, false }, [BI_SAT_S32] = { "sat_s32", 1, 1, false },
    [BI_BIT] = { "bit", 2, 2, false },      [BI_BITS] = { "bits", 3, 3, false },
    [BI_BYTE] = { "byte", 2, 2, false },    [BI_CRC8] = { "crc8", 3, 64, false },
    [BI_CRC16] = { "crc16", 5, 64, false }, [BI_SUM8] = { "sum8", 1, 64, false },
    [BI_XOR8] = { "xor8", 1, 64, false },   [BI_RANDOM] = { "random", 1, 1, false },
    [BI_HHMMSS] = { "hhmmss", 1, 1, true }, [BI_DDMMYY] = { "ddmmyy", 1, 1, true },
    [BI_NMEA_LAT] = { "nmea_lat", 1, 1, true }, [BI_NMEA_LON] = { "nmea_lon", 1, 1, true },
};

int sd_builtin_find(const char *name, size_t len) {
    for (int i = 0; i < BI_N; i++)
        if (strlen(sd_builtins[i].name) == len && memcmp(sd_builtins[i].name, name, len) == 0) return i;
    return -1;
}

/* ---- pure numeric operations ---- */
#define NAN_ sd_nan()

int64_t sd_to_i64(double v) {
    if (v != v) return 0;
    if (v >= 9223372036854775807.0) return INT64_MAX;
    if (v <= -9223372036854775808.0) return INT64_MIN;
    return (int64_t)v;   /* truncates toward zero */
}

double sd_sat(double v, int bits, bool is_signed) {
    if (v != v) return 0;
    double r = round(v);
    double lo = is_signed ? -ldexp(1.0, bits - 1) : 0.0;
    double hi = is_signed ? ldexp(1.0, bits - 1) - 1.0 : ldexp(1.0, bits) - 1.0;
    return r < lo ? lo : (r > hi ? hi : r);
}

double sd_wrap(double v, int bits, bool is_signed) {
    uint64_t u = (uint64_t)sd_to_i64(v);
    uint64_t mask = bits >= 64 ? ~0ull : ((1ull << bits) - 1u);
    u &= mask;
    if (is_signed && bits < 64 && (u >> (bits - 1)) & 1u) return (double)(int64_t)(u | ~mask);
    return (double)u;
}

static double i2d(int64_t i) { return (double)i; }

static int64_t shr_arith(int64_t a, int64_t n) {
    if (n <= 0) return a;
    if (n > 63) return a < 0 ? -1 : 0;
    return a < 0 ? ~(~a >> n) : a >> n;   /* floor division by 2^n, also for negatives */
}

double sd_op_binary(int op, double a, double b) {
    switch (op) {
    case OP_ADD: return a + b;
    case OP_SUB: return a - b;
    case OP_MUL: return a * b;
    case OP_DIV: return a / b;
    case OP_MOD: return b == 0 ? NAN_ : fmod(a, b);
    case OP_POW: return sd_pow(a, b);
    case OP_SHL: {
        int64_t n = sd_to_i64(b);
        if (n < 0) return i2d(sd_to_i64(a));
        if (n > 63) return 0;
        return i2d((int64_t)((uint64_t)sd_to_i64(a) << n));
    }
    case OP_SHR: return i2d(shr_arith(sd_to_i64(a), sd_to_i64(b)));
    case OP_BAND: return i2d(sd_to_i64(a) & sd_to_i64(b));
    case OP_BXOR: return i2d(sd_to_i64(a) ^ sd_to_i64(b));
    case OP_BOR: return i2d(sd_to_i64(a) | sd_to_i64(b));
    case OP_LT: return a < b;
    case OP_LE: return a <= b;
    case OP_GT: return a > b;
    case OP_GE: return a >= b;
    case OP_EQ: return a == b;
    case OP_NE: return a != b;
    default: return NAN_;
    }
}

double sd_op_unary(int op, double a) {
    switch (op) {
    case OP_NEG: return -a;
    case OP_NOT: return a == 0 ? 1 : 0;
    case OP_BNOT: return i2d(~sd_to_i64(a));
    default: return NAN_;
    }
}

static uint64_t splitmix(uint64_t x) {
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

int sd_builtin_num(int id, const double *a, unsigned n, double *out) {
    double r = 0;
    switch (id) {
    case BI_ABS: r = fabs(a[0]); break;
    case BI_MIN: r = a[0]; for (unsigned i = 1; i < n; i++) if (a[i] < r || a[i] != a[i]) r = a[i]; break;
    case BI_MAX: r = a[0]; for (unsigned i = 1; i < n; i++) if (a[i] > r || a[i] != a[i]) r = a[i]; break;
    case BI_CLAMP: r = a[0] < a[1] ? a[1] : (a[0] > a[2] ? a[2] : a[0]); break;
    case BI_ROUND: r = round(a[0]); break;
    case BI_FLOOR: r = floor(a[0]); break;
    case BI_CEIL: r = ceil(a[0]); break;
    case BI_TRUNC: r = trunc(a[0]); break;
    case BI_SQRT: r = a[0] < 0 ? NAN_ : sqrt(a[0]); break;
    case BI_EXP: r = sd_exp(a[0]); break;
    case BI_LOG: r = sd_log(a[0]); break;
    case BI_LOG10: r = sd_log(a[0]) * 0.43429448190325182765; break;
    case BI_POW: r = sd_pow(a[0], a[1]); break;
    case BI_SIN: r = sd_sin(a[0]); break;
    case BI_COS: r = sd_cos(a[0]); break;
    case BI_TAN: r = sd_tan(a[0]); break;
    case BI_ASIN: r = sd_asin(a[0]); break;
    case BI_ACOS: r = sd_acos(a[0]); break;
    case BI_ATAN: r = sd_atan(a[0]); break;
    case BI_ATAN2: r = sd_atan2(a[0], a[1]); break;
    case BI_RAD: r = a[0] * (SD_PI / 180.0); break;
    case BI_DEG: r = a[0] * (180.0 / SD_PI); break;
    case BI_IDIV: case BI_IMOD: {
        int64_t x = sd_to_i64(a[0]), y = sd_to_i64(a[1]);
        if (y == 0 || (x == INT64_MIN && y == -1)) r = 0;
        else r = id == BI_IDIV ? i2d(x / y) : i2d(x % y);
        break;
    }
    case BI_U8: r = sd_wrap(a[0], 8, false); break;
    case BI_S8: r = sd_wrap(a[0], 8, true); break;
    case BI_U16: r = sd_wrap(a[0], 16, false); break;
    case BI_S16: r = sd_wrap(a[0], 16, true); break;
    case BI_U24: r = sd_wrap(a[0], 24, false); break;
    case BI_S24: r = sd_wrap(a[0], 24, true); break;
    case BI_U32: r = sd_wrap(a[0], 32, false); break;
    case BI_S32: r = sd_wrap(a[0], 32, true); break;
    case BI_SAT_U8: r = sd_sat(a[0], 8, false); break;
    case BI_SAT_S8: r = sd_sat(a[0], 8, true); break;
    case BI_SAT_U16: r = sd_sat(a[0], 16, false); break;
    case BI_SAT_S16: r = sd_sat(a[0], 16, true); break;
    case BI_SAT_U24: r = sd_sat(a[0], 24, false); break;
    case BI_SAT_S24: r = sd_sat(a[0], 24, true); break;
    case BI_SAT_U32: r = sd_sat(a[0], 32, false); break;
    case BI_SAT_S32: r = sd_sat(a[0], 32, true); break;
    case BI_BIT: {
        int64_t b = sd_to_i64(a[1]);
        r = (b < 0 || b > 63) ? 0 : (double)(((uint64_t)sd_to_i64(a[0]) >> b) & 1u);
        break;
    }
    case BI_BITS: {
        int64_t hi = sd_to_i64(a[1]), lo = sd_to_i64(a[2]);
        if (lo < 0 || hi > 63 || hi < lo) { r = 0; break; }
        uint64_t v = (uint64_t)sd_to_i64(a[0]) >> lo;
        int w = (int)(hi - lo + 1);
        r = (double)(w >= 64 ? v : (v & ((1ull << w) - 1u)));
        break;
    }
    case BI_BYTE: {
        int64_t b = sd_to_i64(a[1]);
        r = (b < 0 || b > 7) ? 0 : (double)(((uint64_t)sd_to_i64(a[0]) >> (8 * b)) & 0xFFu);
        break;
    }
    case BI_CRC8: {
        uint8_t poly = (uint8_t)sd_to_i64(a[0]), crc = (uint8_t)sd_to_i64(a[1]);
        for (unsigned i = 2; i < n; i++) {
            crc ^= (uint8_t)sd_to_i64(a[i]);
            for (int k = 0; k < 8; k++) crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ poly) : (uint8_t)(crc << 1);
        }
        r = crc;
        break;
    }
    case BI_CRC16: {
        uint16_t poly = (uint16_t)sd_to_i64(a[0]), crc = (uint16_t)sd_to_i64(a[1]);
        bool refl = sd_to_i64(a[2]) != 0;
        uint16_t xo = (uint16_t)sd_to_i64(a[3]);
        uint16_t rp = 0;
        for (int k = 0; k < 16; k++) if (poly & (1u << k)) rp |= (uint16_t)(1u << (15 - k));
        for (unsigned i = 4; i < n; i++) {
            uint8_t b = (uint8_t)sd_to_i64(a[i]);
            if (refl) {
                crc ^= b;
                for (int k = 0; k < 8; k++) crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ rp) : (uint16_t)(crc >> 1);
            } else {
                crc ^= (uint16_t)(b << 8);
                for (int k = 0; k < 8; k++) crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ poly) : (uint16_t)(crc << 1);
            }
        }
        r = (uint16_t)(crc ^ xo);
        break;
    }
    case BI_SUM8: case BI_XOR8: {
        uint8_t acc = 0;
        for (unsigned i = 0; i < n; i++) {
            uint8_t b = (uint8_t)sd_to_i64(a[i]);
            acc = id == BI_SUM8 ? (uint8_t)(acc + b) : (uint8_t)(acc ^ b);
        }
        r = acc;
        break;
    }
    case BI_RANDOM: {
        uint64_t u;
        double s = a[0] == 0 ? 0.0 : a[0];   /* -0 and 0 seed the same */
        memcpy(&u, &s, 8);
        r = (double)(splitmix(u) >> 11) * (1.0 / 9007199254740992.0);
        break;
    }
    default: return SD_E_EVAL;
    }
    *out = r;
    return SD_OK;
}

/* ---- number formatting (templates) ---- */
typedef struct { char *o; size_t cap, n; } wbuf_t;
static void wch(wbuf_t *w, char c) { if (w->n < w->cap) w->o[w->n] = c; w->n++; }
static void wstr(wbuf_t *w, const char *s, size_t n) { for (size_t i = 0; i < n; i++) wch(w, s[i]); }

/* Decimal digits of u into d (reversed); returns the count. */
static unsigned udigits(uint64_t u, char *d) {
    unsigned n = 0;
    do { d[n++] = (char)('0' + u % 10u); u /= 10u; } while (u);
    return n;
}

static const double P10[] = { 1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9, 1e10, 1e11, 1e12,
                              1e13, 1e14, 1e15 };

/* body: the digits (and point) of |v| without sign; returns length.  From 1e18 on, the
   exponent form d.dddddde+NN (one fixed way on every platform). */
static size_t fixed_body(double a, unsigned prec, char *body, size_t cap) {
    char d[24];
    size_t n = 0;
    double sc = a * P10[prec];
    if (a < 1e18 && sc < 9.2e18) {
        uint64_t s = (uint64_t)llround(sc);
        uint64_t ip = s / (uint64_t)P10[prec], fp = s % (uint64_t)P10[prec];
        unsigned k = udigits(ip, d);
        for (unsigned i = 0; i < k && n < cap; i++) body[n++] = d[k - 1 - i];
        if (prec) {
            if (n < cap) body[n++] = '.';
            k = udigits(fp, d);
            for (unsigned i = k; i < prec && n < cap; i++) body[n++] = '0';
            for (unsigned i = 0; i < k && n < cap; i++) body[n++] = d[k - 1 - i];
        }
    } else {
        unsigned e = 0;
        while (a >= 10.0 && e < 400) { a /= 10.0; e++; }
        n = fixed_body(a, 6, body, cap > 8 ? cap - 6 : cap);
        if (n + 6 <= cap) {
            body[n++] = 'e'; body[n++] = '+';
            body[n++] = (char)('0' + e / 100); body[n++] = (char)('0' + e / 10 % 10); body[n++] = (char)('0' + e % 10);
        }
    }
    return n;
}

size_t sd_fmt_num(double v, uint8_t flags, uint8_t width, uint8_t prec, uint8_t conv, char *out,
                  size_t cap) {
    char body[64];
    size_t bn = 0;
    bool neg = false;
    if (v != v) {
        memcpy(body, "nan", 3); bn = 3;
    } else if (v == sd_inf() || v == -sd_inf()) {
        neg = v < 0;
        memcpy(body, "inf", 3); bn = 3;
    } else if (conv == SDC_D) {
        int64_t i = sd_to_i64(v);
        neg = i < 0;
        uint64_t u = neg ? (uint64_t)0 - (uint64_t)i : (uint64_t)i;
        char d[24];
        unsigned k = udigits(u, d);
        for (unsigned q = 0; q < k; q++) body[bn++] = d[k - 1 - q];
    } else if (conv == SDC_X || conv == SDC_XU) {
        uint64_t u = (uint64_t)sd_to_i64(v);
        const char *hx = conv == SDC_X ? "0123456789abcdef" : "0123456789ABCDEF";
        char d[20];
        unsigned k = 0;
        do { d[k++] = hx[u & 15u]; u >>= 4; } while (u);
        for (unsigned q = 0; q < k; q++) body[bn++] = d[k - 1 - q];
    } else if (conv == SDC_F) {
        neg = v < 0;
        bn = fixed_body(neg ? -v : v, (flags & SDF_PREC) ? (prec > 15 ? 15 : prec) : 6, body, sizeof(body));
    } else {
        /* default: a whole number as an integer, else up to 6 decimals without trailing zeros */
        neg = v < 0;
        double a = neg ? -v : v;
        if (a == floor(a) && a < 1e15) {
            bn = fixed_body(a, 0, body, sizeof(body));
        } else {
            bn = fixed_body(a, 6, body, sizeof(body));
            if (a < 1e18) {
                while (bn > 1 && body[bn - 1] == '0') bn--;
                if (bn > 1 && body[bn - 1] == '.') bn--;
            }
        }
        if (bn == 1 && body[0] == '0') neg = false;   /* -0.0000001 prints 0 */
    }
    wbuf_t w = { out, cap, 0 };
    char sign = neg ? '-' : ((flags & SDF_PLUS) ? '+' : 0);
    size_t len = bn + (sign ? 1u : 0u);
    size_t pad = width > len ? width - len : 0;
    if (!(flags & SDF_MINUS) && !(flags & SDF_ZERO)) for (size_t i = 0; i < pad; i++) wch(&w, ' ');
    if (sign) wch(&w, sign);
    if (!(flags & SDF_MINUS) && (flags & SDF_ZERO)) for (size_t i = 0; i < pad; i++) wch(&w, '0');
    wstr(&w, body, bn);
    if (flags & SDF_MINUS) for (size_t i = 0; i < pad; i++) wch(&w, ' ');
    return w.n > cap ? cap + 1 : w.n;
}

/* ---- string built-ins ---- */
static void civil_from_days(int64_t z, int64_t *y, unsigned *m, unsigned *d) {
    z += 719468;   /* Howard Hinnant's days -> civil date */
    int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    unsigned doe = (unsigned)(z - era * 146097);
    unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    unsigned mp = (5 * doy + 2) / 153;
    *d = doy - (153 * mp + 2) / 5 + 1;
    *m = mp < 10 ? mp + 3 : mp - 9;
    *y = (int64_t)yoe + era * 400 + (*m <= 2);
}

static size_t put2(char *o, unsigned v) { o[0] = (char)('0' + v / 10 % 10); o[1] = (char)('0' + v % 10); return 2; }

/* Builds the text of a string built-in into o (cap >= 24); returns its length. */
static size_t str_builtin(int id, double x, char *o) {
    size_t n = 0;
    if (id == BI_HHMMSS || id == BI_DDMMYY) {
        double ms_d = round(x * 1000.0);
        int64_t ms = sd_to_i64(ms_d);
        int64_t day = ms / 86400000, rem = ms % 86400000;
        if (rem < 0) { rem += 86400000; day--; }
        if (id == BI_HHMMSS) {
            unsigned s = (unsigned)(rem / 1000), cs = (unsigned)(rem % 1000) / 10u;
            n += put2(o + n, s / 3600u);
            n += put2(o + n, s / 60u % 60u);
            n += put2(o + n, s % 60u);
            o[n++] = '.';
            n += put2(o + n, cs);
        } else {
            int64_t y; unsigned m, d;
            civil_from_days(day, &y, &m, &d);
            n += put2(o + n, d);
            n += put2(o + n, m);
            int64_t yy = y % 100;
            n += put2(o + n, (unsigned)(yy < 0 ? yy + 100 : yy));
        }
        return n;
    }
    /* nmea_lat: ddmm.mmmmm,N; nmea_lon: dddmm.mmmmm,E */
    bool lat = id == BI_NMEA_LAT;
    char hemi = x < 0 ? (lat ? 'S' : 'W') : (lat ? 'N' : 'E');
    double v = fabs(x);
    if (v != v || v > 360.0) v = 0;
    uint64_t mmin = (uint64_t)llround(v * 60.0 * 100000.0);   /* 1e-5 minutes */
    uint64_t deg = mmin / 6000000u, rem = mmin % 6000000u;
    if (!lat) o[n++] = (char)('0' + deg / 100 % 10);
    n += put2(o + n, (unsigned)(deg % 100));
    n += put2(o + n, (unsigned)(rem / 100000u));
    o[n++] = '.';
    unsigned f = (unsigned)(rem % 100000u);
    for (int k = 4; k >= 0; k--) { o[n + (size_t)k] = (char)('0' + f % 10); f /= 10; }
    n += 5;
    o[n++] = ',';
    o[n++] = hemi;
    return n;
}

/* ---- the VM ---- */
void sd_set_err(sd_inst_t *in, int code, const char *msg) {
    if (in->err.code) return;   /* keep the first */
    in->err.code = code;
    size_t l = strlen(msg);
    if (l >= sizeof(in->err.msg)) l = sizeof(in->err.msg) - 1;
    memcpy(in->err.msg, msg, l);
    in->err.msg[l] = '\0';
}

void sd_vm_begin(sd_inst_t *in) {
    if (in->steps > in->max_steps_seen) in->max_steps_seen = in->steps;
    in->steps = 0;
    in->scratch_top = 0;
    in->sp = 0;
    in->depth = 0;
}

static const uint8_t *str_ptr(const sd_inst_t *in, uint64_t v) {
    return ((v >> 48) == SDS_TAG_PROG) ? in->prog + sdv_str_off(v) : in->scratch + sdv_str_off(v);
}

static int16_t rd16(const uint8_t *p) { return (int16_t)(uint16_t)(p[0] | (p[1] << 8)); }

#define FAIL(c, m) do { sd_set_err(in, (c), (m)); in->err.expr_off = (int32_t)(pc - in->prog); return (c); } while (0)
#define PUSH(v) do { if (in->sp >= SD_STACK) FAIL(SD_E_EVAL, "expression too deep (stack)"); in->stack[in->sp++] = (v); } while (0)
#define NEED(k) do { if (in->sp < (k) + base) FAIL(SD_E_EVAL, "stack underflow"); } while (0)

/* Append text to the current target: in->out (top level) or the scratch at *tend (fragment). */
static int append(sd_inst_t *in, bool frag, uint16_t *tend, const uint8_t *src, size_t n) {
    if (frag) {
        if ((size_t)*tend + n > SD_SCRATCH) return SD_E_SPACE;
        memmove(in->scratch + *tend, src, n);
        *tend = (uint16_t)(*tend + n);
        if (in->scratch_top < *tend) in->scratch_top = *tend;
        return SD_OK;
    }
    if (!in->out) return SD_OK;
    if (in->out_len + n > in->out_cap) { in->out_over = true; n = in->out_cap - in->out_len; }
    memcpy(in->out + in->out_len, src, n);
    in->out_len += n;
    return SD_OK;
}

static int run(sd_inst_t *in, uint16_t code, uint16_t base, bool frag, uint64_t *result);

/* Leaf helpers with the bigger buffers, so run()'s own frame (it recurses) stays small. */
static SD_NOINLINE int do_builtin(sd_inst_t *in, uint8_t id, uint8_t argc) {
    double a[64];
    for (unsigned i = 0; i < argc; i++) {
        uint64_t v = in->stack[in->sp - argc + i];
        if (sdv_is_str(v)) { sd_set_err(in, SD_E_EVAL, "a string as a number argument"); return SD_E_EVAL; }
        a[i] = sdv_d(v);
    }
    in->sp = (uint16_t)(in->sp - argc);
    if (sd_builtins[id].str_result) {
        char tmp[32];
        size_t n = str_builtin(id, a[0], tmp);
        if ((size_t)in->scratch_top + n > SD_SCRATCH) { sd_set_err(in, SD_E_SPACE, "string scratch full"); return SD_E_SPACE; }
        memcpy(in->scratch + in->scratch_top, tmp, n);
        in->stack[in->sp++] = sdv_str(SDS_TAG_SCRATCH, in->scratch_top, (uint32_t)n);
        in->scratch_top = (uint16_t)(in->scratch_top + n);
        return SD_OK;
    }
    double r;
    if (sd_builtin_num(id, a, argc, &r) != SD_OK) { sd_set_err(in, SD_E_EVAL, "bad built-in"); return SD_E_EVAL; }
    in->stack[in->sp++] = sdv_num(r);
    return SD_OK;
}

static SD_NOINLINE int do_txtv(sd_inst_t *in, bool frag, uint16_t *tend, uint64_t v, const uint8_t *f) {
    if (sdv_is_str(v)) return append(in, frag, tend, str_ptr(in, v), sdv_str_len(v));
    char tmp[96];
    size_t n = sd_fmt_num(sdv_d(v), f[0], f[1], f[2], f[3], tmp, sizeof(tmp));
    if (n > sizeof(tmp)) n = sizeof(tmp);
    return append(in, frag, tend, (const uint8_t *)tmp, n);
}

static int call_func(sd_inst_t *in, unsigned f, uint16_t base_args, uint64_t *res) {
    const sdp_hdr_t *h = sdp_hdr(in->prog);
    const sdp_func_t *fn = (const sdp_func_t *)(const void *)(in->prog + h->funcs) + f;
    return run(in, fn->code, base_args, false, res);
}

/* solve(f, target, lo, hi): the integer x in [lo, hi] whose f(x) is closest to target, by
   binary search (an exact hit stops it), as the C models did. */
#define SFAIL(c, m) do { sd_set_err(in, (c), (m)); return (c); } while (0)
#define SPUSH(v) do { if (in->sp >= SD_STACK) SFAIL(SD_E_EVAL, "expression too deep (stack)"); in->stack[in->sp++] = (v); } while (0)
#define SNEED(k) do { if (in->sp < (k) + base) SFAIL(SD_E_EVAL, "stack underflow"); } while (0)
static SD_NOINLINE int do_solve(sd_inst_t *in, uint8_t f, uint8_t pos, uint16_t base) {
            const sdp_hdr_t *h = sdp_hdr(in->prog);
            if (f >= h->n_funcs) SFAIL(SD_E_EVAL, "bad solve");
            const sdp_func_t *fn = (const sdp_func_t *)(const void *)(in->prog + h->funcs) + f;
            unsigned nfixed = pos == 0xFF ? 0u : (unsigned)fn->nargs - 1u;
            if (pos == 0xFF) pos = 0;
            SNEED(nfixed + 3);
            double v3[3];
            for (unsigned i = 0; i < 3; i++) {
                uint64_t v = in->stack[in->sp - 3 + i];
                if (sdv_is_str(v)) SFAIL(SD_E_EVAL, "a string in solve()");
                v3[i] = sdv_d(v);
            }
            in->sp = (uint16_t)(in->sp - 3);
            uint16_t fixed_at = (uint16_t)(in->sp - nfixed);
            double target = v3[0];
            if (!(v3[1] == v3[1]) || !(v3[2] == v3[2]) || target != target) SFAIL(SD_E_SOLVE, "solve(): NaN argument");
            double lod = ceil(v3[1]), hid = floor(v3[2]);
            if (lod > hid) SFAIL(SD_E_SOLVE, "solve(): empty range");
            if (hid - lod > 4.0e18 || fabs(lod) > 4.0e18 || fabs(hid) > 4.0e18) SFAIL(SD_E_SOLVE, "solve(): range too large");
            int64_t lo = (int64_t)lod, hi = (int64_t)hid;
            /* f(x): push the arguments with x at pos, call, read the result */
            #define SOLVE_EVAL(x, out) do { \
                uint16_t nb_ = in->sp; \
                for (unsigned q_ = 0, k_ = 0; q_ < fn->nargs; q_++) \
                    SPUSH(q_ == pos ? sdv_num((double)(x)) : in->stack[fixed_at + k_++]); \
                uint64_t r_; int rc_ = call_func(in, f, nb_, &r_); \
                if (rc_) return rc_; \
                in->sp = nb_; \
                if (sdv_is_str(r_)) SFAIL(SD_E_SOLVE, "solve(): the function gives a string"); \
                (out) = sdv_d(r_); } while (0)
            double flo, fhi;
            SOLVE_EVAL(lo, flo);
            SOLVE_EVAL(hi, fhi);
            bool rising = !(flo > fhi);
            double vlo = flo, vhi = fhi;
            int64_t best = lo;
            double best_err = sd_inf();
            for (unsigned it = 0; lo <= hi && it < SD_SOLVE_MAX_ITER; it++) {
                int64_t mid = lo + (hi - lo) / 2;
                double v;
                SOLVE_EVAL(mid, v);
                if (!(v == v) || v == sd_inf() || v == -sd_inf()) { hi = mid - 1; continue; }
                /* a monotonic function stays between the values at the bracket's ends */
                if ((vlo == vlo && vhi == vhi) && fabs(vlo) != sd_inf() && fabs(vhi) != sd_inf() &&
                    (rising ? (v < vlo || v > vhi) : (v > vlo || v < vhi)))
                    SFAIL(SD_E_SOLVE, "solve(): the function is not monotonic");
                double e = fabs(v - target);
                if (e < best_err) { best_err = e; best = mid; }
                if (v == target) break;
                if (rising ? v < target : v > target) { lo = mid + 1; vlo = v; }
                else { hi = mid - 1; vhi = v; }
            }
            #undef SOLVE_EVAL
            in->sp = fixed_at;
            SPUSH(sdv_num((double)best));
            return SD_OK;
}
#undef SFAIL
#undef SPUSH
#undef SNEED

static int run(sd_inst_t *in, uint16_t code, uint16_t base, bool frag, uint64_t *result) {
    const uint8_t *pc = in->prog + code;
    const uint8_t *end = in->prog + in->prog_len;
    uint16_t tend = in->scratch_top;      /* fragment text end */
    uint16_t top_mark = in->scratch_top;  /* top-level template: temporaries above this */
    if (++in->depth > SD_FRAMES) FAIL(SD_E_EVAL, "calls nested too deep");
    for (;;) {
        if (pc >= end) FAIL(SD_E_EVAL, "code runs past the program");
        if (++in->steps > SD_STEP_LIMIT) FAIL(SD_E_EVAL, "step limit");
        uint8_t op = *pc++;
        switch (op) {
        case OP_RET:
            if (frag) {
                *result = sdv_str(SDS_TAG_SCRATCH, top_mark, (uint32_t)(tend - top_mark));
                in->scratch_top = tend;
            } else {
                *result = in->sp > base ? in->stack[in->sp - 1] : sdv_num(0);
            }
            in->depth--;
            return SD_OK;
        case OP_I8: PUSH(sdv_num((int8_t)*pc)); pc++; break;
        case OP_I16: PUSH(sdv_num(rd16(pc))); pc += 2; break;
        case OP_F64: { double d; memcpy(&d, pc, 8); pc += 8; PUSH(sdv_num(d)); break; }
        case OP_STR: PUSH(sdv_str(SDS_TAG_PROG, (uint32_t)(pc + 1 - in->prog), *pc)); pc += 1u + *pc; break;
        case OP_VAR:
            if (*pc >= SDV_N) FAIL(SD_E_EVAL, "bad variable");
            PUSH(in->vars[*pc]); pc++; break;
        case OP_LOCAL:
            if (base + *pc >= in->sp) FAIL(SD_E_EVAL, "bad local");
            PUSH(in->stack[base + *pc]); pc++; break;
        case OP_REG: case OP_REG16BE: case OP_REG16LE: {
            NEED(1);
            uint64_t a = in->stack[in->sp - 1];
            if (sdv_is_str(a)) FAIL(SD_E_EVAL, "a string as a register address");
            const sdp_hdr_t *ph = sdp_hdr(in->prog);
            int64_t ra = sd_to_i64(sdv_d(a));
            /* word registers: reg[p] is the 16-bit register (reg16le[p] its bytes swapped) */
            unsigned k = (op == OP_REG && ph->reg_bytes != 2) ? 1u : 2u, v = 0;
            if (ph->reg_bytes == 2 || ph->cmap) {
                unsigned ix;
                bool ok = sd_map_index(in->prog, ra, &ix);
                for (unsigned i = 0; i < k; i++) {
                    unsigned b = (in->regsrc && ok) ? in->regsrc[(ix + i) & 0xFFu] : 0u;
                    v = op == OP_REG16LE ? (v | (b << (8 * i))) : ((v << 8) | b);
                }
            } else {
                int64_t addr = ra - (int64_t)ph->page;
                for (unsigned i = 0; i < k; i++) {
                    int64_t ad = addr + (int64_t)i;
                    unsigned b = (in->regsrc && ad >= 0 && ad < 256) ? in->regsrc[ad] : 0u;
                    v = op == OP_REG16LE ? (v | (b << (8 * i))) : ((v << 8) | b);
                }
            }
            in->stack[in->sp - 1] = sdv_num(v);
            break;
        }
        case OP_NEG: case OP_NOT: case OP_BNOT: {
            NEED(1);
            uint64_t a = in->stack[in->sp - 1];
            if (sdv_is_str(a)) FAIL(SD_E_EVAL, "a string in arithmetic");
            in->stack[in->sp - 1] = sdv_num(sd_op_unary(op, sdv_d(a)));
            break;
        }
        case OP_ADD: case OP_SUB: case OP_MUL: case OP_DIV: case OP_MOD: case OP_POW: case OP_SHL:
        case OP_SHR: case OP_BAND: case OP_BXOR: case OP_BOR: case OP_LT: case OP_LE: case OP_GT:
        case OP_GE: case OP_EQ: case OP_NE: {
            NEED(2);
            uint64_t b = in->stack[--in->sp], a = in->stack[in->sp - 1];
            if (sdv_is_str(a) || sdv_is_str(b)) {
                if (op != OP_EQ && op != OP_NE) FAIL(SD_E_EVAL, "a string in arithmetic");
                bool eq = sdv_is_str(a) && sdv_is_str(b) && sdv_str_len(a) == sdv_str_len(b) &&
                          memcmp(str_ptr(in, a), str_ptr(in, b), sdv_str_len(a)) == 0;
                in->stack[in->sp - 1] = sdv_num((op == OP_EQ) == eq ? 1 : 0);
                break;
            }
            in->stack[in->sp - 1] = sdv_num(sd_op_binary(op, sdv_d(a), sdv_d(b)));
            break;
        }
        case OP_JZ: {
            NEED(1);
            uint64_t c = in->stack[--in->sp];
            if (sdv_is_str(c)) FAIL(SD_E_EVAL, "a string as a condition");
            int16_t off = rd16(pc);
            pc += 2;
            if (sdv_d(c) == 0) pc += off;
            break;
        }
        case OP_JMP: { int16_t off = rd16(pc); pc += 2; pc += off; break; }
        case OP_ANDJ: case OP_ORJ: {
            NEED(1);
            uint64_t c = in->stack[in->sp - 1];
            if (sdv_is_str(c)) FAIL(SD_E_EVAL, "a string as a condition");
            int16_t off = rd16(pc);
            pc += 2;
            bool t = sdv_d(c) != 0;
            if (op == OP_ANDJ && !t) { in->stack[in->sp - 1] = sdv_num(0); pc += off; }
            else if (op == OP_ORJ && t) { in->stack[in->sp - 1] = sdv_num(1); pc += off; }
            else in->sp--;
            break;
        }
        case OP_BOOL: {
            NEED(1);
            uint64_t c = in->stack[in->sp - 1];
            if (sdv_is_str(c)) FAIL(SD_E_EVAL, "a string as a condition");
            in->stack[in->sp - 1] = sdv_num(sdv_d(c) != 0 ? 1 : 0);
            break;
        }
        case OP_BUILTIN: {
            uint8_t id = pc[0], argc = pc[1];
            pc += 2;
            if (id >= BI_N || argc > 64 || argc < sd_builtins[id].min_args) FAIL(SD_E_EVAL, "bad built-in");
            NEED(argc);
            if (do_builtin(in, id, argc)) { in->err.expr_off = (int32_t)(pc - in->prog); return in->err.code; }
            break;
        }
        case OP_CALL: case OP_FRAG: {
            const sdp_hdr_t *h = sdp_hdr(in->prog);
            uint8_t f = *pc++;
            if (f >= (op == OP_CALL ? h->n_funcs : h->n_frags)) FAIL(SD_E_EVAL, "bad call");
            const sdp_func_t *fn = (const sdp_func_t *)(const void *)(in->prog + (op == OP_CALL ? h->funcs : h->frags)) + f;
            NEED(fn->nargs);
            uint16_t nb = (uint16_t)(in->sp - fn->nargs);
            uint64_t r;
            int rc = run(in, fn->code, nb, op == OP_FRAG, &r);
            if (rc) return rc;
            in->sp = nb;
            PUSH(r);
            break;
        }
        case OP_SOLVE: {
            uint8_t f = pc[0], pos = pc[1];
            pc += 2;
            int rc = do_solve(in, f, pos, base);
            if (rc) { in->err.expr_off = (int32_t)(pc - in->prog); return rc; }
            break;
        }
        case OP_CTAB: {
            uint8_t n = *pc++;
            NEED(1);
            uint64_t iv = in->stack[in->sp - 1];
            if (sdv_is_str(iv)) FAIL(SD_E_EVAL, "a string as a table index");
            int64_t i = sd_to_i64(sdv_d(iv));
            double d = 0;
            if (i >= 0 && i < n) memcpy(&d, pc + 8 * i, 8);
            pc += 8u * n;
            in->stack[in->sp - 1] = sdv_num(d);
            break;
        }
        case OP_TAB: {
            uint8_t n = *pc++;
            NEED((unsigned)n + 1u);
            uint64_t iv = in->stack[--in->sp];
            if (sdv_is_str(iv)) FAIL(SD_E_EVAL, "a string as a table index");
            int64_t i = sd_to_i64(sdv_d(iv));
            uint64_t v = (i >= 0 && i < n) ? in->stack[in->sp - n + i] : sdv_num(0);
            in->sp = (uint16_t)(in->sp - n);
            PUSH(v);
            break;
        }
        case OP_TXT: {
            uint8_t n = *pc++;
            if (append(in, frag, &tend, pc, n)) FAIL(SD_E_SPACE, "string scratch full");
            pc += n;
            break;
        }
        case OP_TXTV: {
            uint8_t fl = pc[0], wd = pc[1], pr = pc[2], cv = pc[3];
            pc += 4;
            NEED(1);
            uint64_t v = in->stack[--in->sp];
            (void)fl; (void)wd; (void)pr; (void)cv;
            if (do_txtv(in, frag, &tend, v, pc - 4)) FAIL(SD_E_SPACE, "string scratch full");
            in->scratch_top = frag ? tend : top_mark;   /* the expression's temporaries are done */
            break;
        }
        default:
            FAIL(SD_E_EVAL, "bad opcode");
        }
    }
}

int sd_vm_eval(sd_inst_t *in, uint16_t code, uint64_t *result) {
    uint64_t r = sdv_num(0);
    in->sp = 0;
    in->depth = 0;
    int rc = run(in, code, 0, false, &r);
    if (rc) { in->sp = 0; return rc; }
    in->sp = 0;
    if (result) *result = r;
    return SD_OK;
}

int sd_vm_num(sd_inst_t *in, uint16_t code, double *result) {
    uint64_t r;
    int rc = sd_vm_eval(in, code, &r);
    if (rc) return rc;
    if (sdv_is_str(r)) { sd_set_err(in, SD_E_EVAL, "a string where a number is needed"); return SD_E_EVAL; }
    *result = sdv_d(r);
    return SD_OK;
}
