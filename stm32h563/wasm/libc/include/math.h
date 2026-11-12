/* math.h for the freestanding WASM build of the sensor definition engine.
 *
 * Only the exact operations the engine uses (it has its own transcendental functions):
 * floor, ceil, trunc, sqrt and fabs are single WASM instructions (correctly rounded, the
 * same as any IEEE 754 libm); round, llround, fmod and ldexp are exact algorithms in
 * libc/shim.c.  shim_math_test.c checks them against the host libm. */
#pragma once

double sdw_round(double x);
long long sdw_llround(double x);
double sdw_fmod(double x, double y);
double sdw_ldexp(double x, int n);

static inline double floor(double x) { return __builtin_floor(x); }
static inline double ceil(double x) { return __builtin_ceil(x); }
static inline double trunc(double x) { return __builtin_trunc(x); }
static inline double sqrt(double x) { return __builtin_sqrt(x); }
static inline double fabs(double x) { return __builtin_fabs(x); }
static inline double round(double x) { return sdw_round(x); }
static inline long long llround(double x) { return sdw_llround(x); }
static inline double fmod(double x, double y) { return sdw_fmod(x, y); }
static inline double ldexp(double x, int n) { return sdw_ldexp(x, n); }

#define NAN      (__builtin_nan(""))
#define INFINITY (__builtin_inf())
#define HUGE_VAL (__builtin_huge_val())
#define isnan(x) __builtin_isnan(x)
#define isinf(x) __builtin_isinf(x)
#define isfinite(x) __builtin_isfinite(x)
