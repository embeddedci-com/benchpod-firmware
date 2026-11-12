/* shim.c: the few libc functions the sensor definition engine and the WASM wrapper need, for
 * the freestanding (-nostdlib) WASM build.  Compiled with -fno-builtin so the compiler does not
 * turn these loops back into calls to themselves.
 *
 * The math part (sdw_*) builds on the host too (shim_math_test.c compares it with libm).
 * fmod and ldexp follow musl (MIT license, Copyright 2005-2020 Rich Felker et al.). */
#include <stddef.h>
#include <stdint.h>

#ifndef SHIM_MATH_ONLY
#include <string.h>

void *memcpy(void *restrict d, const void *restrict s, size_t n) {
    unsigned char *a = d;
    const unsigned char *b = s;
    while (n--) *a++ = *b++;
    return d;
}

void *memmove(void *d, const void *s, size_t n) {
    unsigned char *a = d;
    const unsigned char *b = s;
    if (a == b || n == 0) return d;
    if (a < b) {
        while (n--) *a++ = *b++;
    } else {
        while (n) { n--; a[n] = b[n]; }
    }
    return d;
}

void *memset(void *d, int c, size_t n) {
    unsigned char *a = d;
    while (n--) *a++ = (unsigned char)c;
    return d;
}

int memcmp(const void *a, const void *b, size_t n) {
    const unsigned char *x = a, *y = b;
    for (; n; n--, x++, y++) if (*x != *y) return *x - *y;
    return 0;
}

void *memchr(const void *s, int c, size_t n) {
    const unsigned char *p = s;
    for (; n; n--, p++) if (*p == (unsigned char)c) return (void *)p;
    return NULL;
}

size_t strlen(const char *s) {
    const char *p = s;
    while (*p) p++;
    return (size_t)(p - s);
}

char *strchr(const char *s, int c) {
    for (;; s++) {
        if (*s == (char)c) return (char *)s;
        if (!*s) return NULL;
    }
}

int strcmp(const char *a, const char *b) {
    while (*a && *a == *b) { a++; b++; }
    return (unsigned char)*a - (unsigned char)*b;
}

int strncmp(const char *a, const char *b, size_t n) {
    for (; n; n--, a++, b++) {
        if (*a != *b) return (unsigned char)*a - (unsigned char)*b;
        if (!*a) return 0;
    }
    return 0;
}
#endif

typedef union { double f; uint64_t i; } dbits_t;

/* Round half away from zero.  For |x| < 2^52, x - trunc(x) is exact. */
double sdw_round(double x) {
    if (!(__builtin_fabs(x) < 4503599627370496.0)) return x;   /* already an integer, inf, nan */
    double t = __builtin_trunc(x), d = x - t;
    if (d >= 0.5) t += 1.0;
    else if (d <= -0.5) t -= 1.0;
    return t;
}

long long sdw_llround(double x) { return (long long)sdw_round(x); }

double sdw_ldexp(double x, int n) {
    dbits_t u;
    double y = x;
    if (n > 1023) {
        y *= 0x1p1023;
        n -= 1023;
        if (n > 1023) {
            y *= 0x1p1023;
            n -= 1023;
            if (n > 1023) n = 1023;
        }
    } else if (n < -1022) {
        /* keep the final n < -53 to avoid double rounding in the subnormal range */
        y *= 0x1p-1022 * 0x1p53;
        n += 1022 - 53;
        if (n < -1022) {
            y *= 0x1p-1022 * 0x1p53;
            n += 1022 - 53;
            if (n < -1022) n = -1022;
        }
    }
    u.i = (uint64_t)(0x3ff + n) << 52;
    return y * u.f;
}

double sdw_fmod(double x, double y) {
    dbits_t ux = { x }, uy = { y };
    int ex = (int)(ux.i >> 52 & 0x7ff);
    int ey = (int)(uy.i >> 52 & 0x7ff);
    int sx = (int)(ux.i >> 63);
    uint64_t i, uxi = ux.i;

    if (uy.i << 1 == 0 || y != y || ex == 0x7ff) return (x * y) / (x * y);
    if (uxi << 1 <= uy.i << 1) {
        if (uxi << 1 == uy.i << 1) return 0 * x;
        return x;
    }
    /* normalize x and y */
    if (!ex) {
        for (i = uxi << 12; i >> 63 == 0; ex--, i <<= 1) {}
        uxi <<= -ex + 1;
    } else {
        uxi &= -1ULL >> 12;
        uxi |= 1ULL << 52;
    }
    if (!ey) {
        for (i = uy.i << 12; i >> 63 == 0; ey--, i <<= 1) {}
        uy.i <<= -ey + 1;
    } else {
        uy.i &= -1ULL >> 12;
        uy.i |= 1ULL << 52;
    }
    /* x mod y */
    for (; ex > ey; ex--) {
        i = uxi - uy.i;
        if (i >> 63 == 0) {
            if (i == 0) return 0 * x;
            uxi = i;
        }
        uxi <<= 1;
    }
    i = uxi - uy.i;
    if (i >> 63 == 0) {
        if (i == 0) return 0 * x;
        uxi = i;
    }
    for (; uxi >> 52 == 0; uxi <<= 1, ex--) {}
    /* scale the result */
    if (ex > 0) {
        uxi -= 1ULL << 52;
        uxi |= (uint64_t)ex << 52;
    } else {
        uxi >>= -ex + 1;
    }
    uxi |= (uint64_t)sx << 63;
    ux.i = uxi;
    return ux.f;
}
