/* shim_math_test.c: the WASM build's round/llround/fmod/ldexp (libc/shim.c) against the host
 * libm, bit for bit, on edge cases and random doubles.  Run by `make test`. */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

double sdw_round(double x);
long long sdw_llround(double x);
double sdw_fmod(double x, double y);
double sdw_ldexp(double x, int n);

static uint64_t bits(double d) { uint64_t u; memcpy(&u, &d, 8); return u; }
static double from_bits(uint64_t u) { double d; memcpy(&d, &u, 8); return d; }
static int same(double a, double b) { return (a != a && b != b) || bits(a) == bits(b); }

static uint64_t rng = 0x9E3779B97F4A7C15ull;
static uint64_t next(void) { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return rng; }

static double any_double(void) {
    switch (next() % 4) {
    case 0: return from_bits(next());                                     /* any bit pattern */
    case 1: return ((double)(int64_t)(next() % 2000001) - 1000000) / 8;   /* halves, quarters */
    case 2: return ((double)(int64_t)(next() >> 11) - 4503599627370496.0) * 1e-6;
    default: return ldexp((double)(next() >> 11), (int)(next() % 200) - 150);
    }
}

int main(void) {
    int fails = 0;
    static const double edge[] = { 0.0, -0.0, 0.5, -0.5, 1.5, -1.5, 2.5, -2.5, 0.49999999999999994,
        -0.49999999999999994, 4503599627370495.5, -4503599627370495.5, 4503599627370496.0,
        9007199254740993.0, 1e308, -1e308, 5e-324, -5e-324, 2.2250738585072014e-308, 1.0 / 3.0 };
    double inf = 1.0 / 0.0, nan = 0.0 / 0.0;
    for (unsigned i = 0; i < sizeof(edge) / sizeof(edge[0]); i++) {
        double x = edge[i];
        if (!same(sdw_round(x), round(x))) { printf("FAIL round(%.17g)\n", x); fails++; }
        for (unsigned k = 0; k < sizeof(edge) / sizeof(edge[0]); k++)
            if (!same(sdw_fmod(x, edge[k]), fmod(x, edge[k]))) { printf("FAIL fmod(%.17g, %.17g)\n", x, edge[k]); fails++; }
        if (!same(sdw_fmod(x, inf), fmod(x, inf))) { printf("FAIL fmod(%.17g, inf)\n", x); fails++; }
        if (!same(sdw_fmod(inf, x), fmod(inf, x)) || !same(sdw_fmod(x, nan), fmod(x, nan))) {
            printf("FAIL fmod inf/nan %.17g\n", x); fails++;
        }
    }
    for (int n = -1100; n <= 1100; n++) {
        if (!same(sdw_ldexp(1.0, n), ldexp(1.0, n)) || !same(sdw_ldexp(-3.75, n), ldexp(-3.75, n)) ||
            !same(sdw_ldexp(5e-324, n), ldexp(5e-324, n))) {
            printf("FAIL ldexp(.., %d)\n", n); fails++;
        }
    }
    for (long i = 0; i < 2000000; i++) {
        double x = any_double(), y = any_double();
        if (!same(sdw_round(x), round(x))) { if (fails++ < 20) printf("FAIL round(%.17g)\n", x); }
        if (fabs(x) < 9e18 && sdw_llround(x) != llround(x)) { if (fails++ < 20) printf("FAIL llround(%.17g)\n", x); }
        if (!same(sdw_fmod(x, y), fmod(x, y))) { if (fails++ < 20) printf("FAIL fmod(%.17g, %.17g)\n", x, y); }
        int n = (int)(next() % 2400) - 1200;
        if (!same(sdw_ldexp(x, n), ldexp(x, n))) { if (fails++ < 20) printf("FAIL ldexp(%.17g, %d)\n", x, n); }
    }
    printf("%s: shim math (round, llround, fmod, ldexp) against libm, %d failure(s)\n", fails ? "FAIL" : "PASS", fails);
    return fails != 0;
}
