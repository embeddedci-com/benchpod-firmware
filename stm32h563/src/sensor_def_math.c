/* ============================================================================
 * sensor_def_math.c — the engine's own transcendental functions and number
 * parsing.
 *
 * libm's sin/exp/log/pow differ in the last bit between newlib, glibc, Apple's
 * libm and WASM, and a register byte computed from them could differ with
 * them.  These use only + - * /, sqrt and exact bit operations, with fixed
 * polynomial lengths, so every platform gets the same bits.  They are accurate
 * to a few units in the last place, plenty for sensor transfer functions.
 * ========================================================================== */
#include "sensor_def_int.h"

#include <math.h>   /* sqrt, floor, fabs: exact operations, the same everywhere */

static uint64_t bits_of(double d) { uint64_t u; memcpy(&u, &d, 8); return u; }
static double of_bits(uint64_t u) { double d; memcpy(&d, &u, 8); return d; }
static bool is_nan(double x) { return x != x; }

/* x * 2^k, exactly (one rounding at most when the result is subnormal). */
static double scale2(double x, int k) {
    while (k > 1000) { x *= of_bits((uint64_t)(1023 + 1000) << 52); k -= 1000; if (k < 0) break; }
    while (k < -1000) { x *= of_bits((uint64_t)(1023 - 1000) << 52); k += 1000; }
    return x * of_bits((uint64_t)(1023 + k) << 52);
}

static const double LN2_HI = 6.93147180369123816490e-01;   /* low bits zero: k * LN2_HI exact */
static const double LN2_LO = 1.90821492927058770002e-10;
static const double INV_LN2 = 1.44269504088896338700e+00;

double sd_exp(double x) {
    if (is_nan(x)) return x;
    if (x > 709.782712893384) return of_bits(0x7FF0000000000000ull);
    if (x < -745.1332191019412) return 0.0;
    double kd = floor(x * INV_LN2 + 0.5);
    int k = (int)kd;
    double r = (x - kd * LN2_HI) - kd * LN2_LO;
    /* Taylor to r^13 on |r| <= 0.35 */
    double p = 1.0 / 6227020800.0;
    static const double c[] = { 1.0 / 479001600.0, 1.0 / 39916800.0, 1.0 / 3628800.0,
                                1.0 / 362880.0, 1.0 / 40320.0, 1.0 / 5040.0, 1.0 / 720.0,
                                1.0 / 120.0, 1.0 / 24.0, 1.0 / 6.0, 0.5, 1.0, 1.0 };
    for (unsigned i = 0; i < sizeof(c) / sizeof(c[0]); i++) p = p * r + c[i];
    return scale2(p, k);
}

double sd_log(double x) {
    if (is_nan(x) || x < 0) return of_bits(0x7FF8000000000000ull);
    if (x == 0) return -of_bits(0x7FF0000000000000ull);
    if (x == of_bits(0x7FF0000000000000ull)) return x;
    int e = 0;
    if (x < 2.2250738585072014e-308) { x *= 18014398509481984.0; e = -54; }   /* subnormal: * 2^54 */
    uint64_t u = bits_of(x);
    e += (int)((u >> 52) & 0x7FF) - 1023;
    double m = of_bits((u & 0x000FFFFFFFFFFFFFull) | 0x3FF0000000000000ull);   /* [1, 2) */
    if (m > 1.4142135623730951) { m *= 0.5; e++; }
    double s = (m - 1.0) / (m + 1.0), s2 = s * s;
    /* 2 * (s + s^3/3 + ... + s^25/25) on |s| <= 0.172 */
    double p = 1.0 / 25.0;
    for (int k = 23; k >= 1; k -= 2) p = p * s2 + 1.0 / (double)k;
    double lm = 2.0 * s * p;
    return (double)e * LN2_HI + (lm + (double)e * LN2_LO);
}

double sd_pow(double x, double y) {
    if (y == 0) return 1.0;
    if (is_nan(x) || is_nan(y)) return of_bits(0x7FF8000000000000ull);
    if (x == 1.0) return 1.0;
    if (y == floor(y) && fabs(y) <= 2147483648.0) {
        /* integer powers by squaring: exact while the result fits 53 bits */
        uint64_t n = (uint64_t)fabs(y);
        double r = 1.0, b = x;
        for (int i = 0; i < 32 && n; i++) {
            if (n & 1u) r *= b;
            b *= b;
            n >>= 1;
        }
        return y < 0 ? 1.0 / r : r;
    }
    if (x < 0) return of_bits(0x7FF8000000000000ull);
    if (x == 0) return y > 0 ? 0.0 : of_bits(0x7FF0000000000000ull);
    return sd_exp(y * sd_log(x));
}

/* sin and cos of r, |r| <= pi/4: Taylor to r^17 / r^18 */
static double k_sin(double r) {
    double r2 = r * r;
    double p = 1.0 / 355687428096000.0;
    p = p * r2 - 1.0 / 1307674368000.0;
    p = p * r2 + 1.0 / 6227020800.0;
    p = p * r2 - 1.0 / 39916800.0;
    p = p * r2 + 1.0 / 362880.0;
    p = p * r2 - 1.0 / 5040.0;
    p = p * r2 + 1.0 / 120.0;
    p = p * r2 - 1.0 / 6.0;
    return r + r * r2 * p;
}
static double k_cos(double r) {
    double r2 = r * r;
    double p = -1.0 / 6402373705728000.0;
    p = p * r2 + 1.0 / 20922789888000.0;
    p = p * r2 - 1.0 / 87178291200.0;
    p = p * r2 + 1.0 / 479001600.0;
    p = p * r2 - 1.0 / 3628800.0;
    p = p * r2 + 1.0 / 40320.0;
    p = p * r2 - 1.0 / 720.0;
    p = p * r2 + 1.0 / 24.0;
    p = p * r2 - 0.5;
    return 1.0 + r2 * p;
}

/* x = n * pi/2 + r, |r| <= pi/4 (three-part pi/2, Cody and Waite). */
static int reduce(double x, double *r) {
    static const double TWO_OVER_PI = 6.36619772367581382433e-01;
    static const double P1 = 1.57079632673412561417e+00, P2 = 6.07710050630396597660e-11,
                        P3 = 2.02226624871116645580e-21;
    double n = floor(x * TWO_OVER_PI + 0.5);
    *r = ((x - n * P1) - n * P2) - n * P3;
    double q = n - 4.0 * floor(n / 4.0);   /* n mod 4, exact */
    return (int)q;
}

static double inf(void) { return of_bits(0x7FF0000000000000ull); }
static double nan_(void) { return of_bits(0x7FF8000000000000ull); }

double sd_sin(double x) {
    if (is_nan(x) || fabs(x) == inf()) return nan_();
    if (fabs(x) > 1e15) return nan_();   /* no meaningful phase left */
    double r;
    switch (reduce(x, &r)) {
    case 0: return k_sin(r);
    case 1: return k_cos(r);
    case 2: return -k_sin(r);
    default: return -k_cos(r);
    }
}

double sd_cos(double x) {
    if (is_nan(x) || fabs(x) == inf()) return nan_();
    if (fabs(x) > 1e15) return nan_();
    double r;
    switch (reduce(x, &r)) {
    case 0: return k_cos(r);
    case 1: return -k_sin(r);
    case 2: return -k_cos(r);
    default: return k_sin(r);
    }
}

double sd_tan(double x) {
    if (is_nan(x) || fabs(x) == inf() || fabs(x) > 1e15) return nan_();
    double r;
    int q = reduce(x, &r);
    double s = k_sin(r), c = k_cos(r);
    return (q & 1) ? -c / s : s / c;
}

static const double PI_2 = 1.57079632679489661923;
static const double PI_ = 3.14159265358979323846;

double sd_atan(double x) {
    if (is_nan(x)) return x;
    double a = fabs(x);
    bool inv = a > 1.0;
    if (inv) a = 1.0 / a;
    /* halve the angle twice: atan(a) = 2 atan(a / (1 + sqrt(1 + a^2))), then |t| <= 0.2 */
    double t = a / (1.0 + sqrt(1.0 + a * a));
    t = t / (1.0 + sqrt(1.0 + t * t));
    double t2 = t * t;
    /* t - t^3/3 + t^5/5 - ... - t^27/27, Horner in t^2 */
    double p = 0.0;
    for (int k = 13; k >= 0; k--) p = p * t2 + ((k & 1) ? -1.0 : 1.0) / (double)(2 * k + 1);
    double r = 4.0 * t * p;
    if (inv) r = PI_2 - r;
    return x < 0 ? -r : r;
}

double sd_atan2(double y, double x) {
    if (is_nan(x) || is_nan(y)) return nan_();
    if (x == 0) return y > 0 ? PI_2 : (y < 0 ? -PI_2 : 0.0);
    double a = sd_atan(y / x);
    if (x > 0) return a;
    return y >= 0 ? a + PI_ : a - PI_;
}

double sd_asin(double x) {
    if (is_nan(x) || x > 1.0 || x < -1.0) return nan_();
    return sd_atan2(x, sqrt((1.0 - x) * (1.0 + x)));
}

double sd_acos(double x) {
    if (is_nan(x) || x > 1.0 || x < -1.0) return nan_();
    return sd_atan2(sqrt((1.0 - x) * (1.0 + x)), x);
}

/* ---- numbers: 25, -1.5, 1e5, 0x58, 0b1010 ----
   Up to 19 significant digits and |exponent| <= 22 convert with one rounding (the
   correctly rounded result, as strtod gives); longer forms round in a fixed way. */
double sd_parse_number(const char *s, size_t n, size_t *used) {
    size_t i = 0;
    bool neg = false;
    *used = 0;
    if (i < n && s[i] == '-') { neg = true; i++; }
    if (i + 1 < n && s[i] == '0' && (s[i + 1] == 'x' || s[i + 1] == 'X' || s[i + 1] == 'b' || s[i + 1] == 'B')) {
        unsigned base = (s[i + 1] == 'x' || s[i + 1] == 'X') ? 16u : 2u;
        size_t j = i + 2;
        double v = 0;
        while (j < n) {
            char c = s[j];
            int d = -1;
            if (c >= '0' && c <= '9') d = c - '0';
            else if (base == 16 && c >= 'a' && c <= 'f') d = c - 'a' + 10;
            else if (base == 16 && c >= 'A' && c <= 'F') d = c - 'A' + 10;
            if (d < 0 || d >= (int)base) break;
            v = v * (double)base + (double)d;
            j++;
        }
        if (j == i + 2) return 0;
        *used = j;
        return neg ? -v : v;
    }
    uint64_t m = 0;
    int digits = 0, e10 = 0;
    bool any = false;
    while (i < n && s[i] >= '0' && s[i] <= '9') {
        any = true;
        if (digits < 19) { m = m * 10u + (uint64_t)(s[i] - '0'); if (m) digits++; }
        else e10++;
        i++;
    }
    if (i < n && s[i] == '.') {
        size_t j = i + 1;
        while (j < n && s[j] >= '0' && s[j] <= '9') {
            any = true;
            if (digits < 19) { m = m * 10u + (uint64_t)(s[j] - '0'); if (m) digits++; e10--; }
            j++;
        }
        if (j > i + 1 || any) i = j;
    }
    if (!any) return 0;
    if (i < n && (s[i] == 'e' || s[i] == 'E')) {
        size_t j = i + 1;
        bool eneg = false;
        if (j < n && (s[j] == '+' || s[j] == '-')) { eneg = s[j] == '-'; j++; }
        if (j < n && s[j] >= '0' && s[j] <= '9') {
            int ev = 0;
            while (j < n && s[j] >= '0' && s[j] <= '9') { if (ev < 100000) ev = ev * 10 + (s[j] - '0'); j++; }
            e10 += eneg ? -ev : ev;
            i = j;
        }
    }
    *used = i;
    static const double p10[] = { 1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9, 1e10, 1e11,
                                  1e12, 1e13, 1e14, 1e15, 1e16, 1e17, 1e18, 1e19, 1e20, 1e21, 1e22 };
    double v = (double)m;
    if (m == 0) v = 0;
    else if (e10 >= 0 && e10 <= 22 && m <= (1ull << 53)) v = v * p10[e10];
    else if (e10 < 0 && e10 >= -22 && m <= (1ull << 53)) v = v / p10[-e10];
    else {
        int e = e10;
        while (e > 22) { v *= 1e22; e -= 22; if (v == inf()) break; }
        while (e < -22) { v /= 1e22; e += 22; if (v == 0) break; }
        v = e >= 0 ? v * p10[e] : v / p10[-e];
    }
    return neg ? -v : v;
}
