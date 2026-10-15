/*
 * wave16.h — the generator's waveforms at the DAC8551's full 16 bits.
 *
 * The parametric `generate` command originally took 8-bit levels (0..255) and built an 8-bit
 * table that the DAC played in its high byte: 256 steps. Its calibrated form takes volts (or
 * microamps on the 4-20 mA output), turns them into 16-bit codes with the same per-path fits
 * dac_out and current_out use (cal_data.h), and fills the table in 16 bits.
 *
 * Pure: no HAL, so the whole file runs in the host tests.
 */
#ifndef WAVE16_H
#define WAVE16_H

#include <stdint.h>

typedef enum { WAVE16_SINE, WAVE16_SQUARE, WAVE16_SAWTOOTH } wave16_shape_t;

/* "sine" | "square" | "sawtooth" -> *shape. 0 on success, -1 for an unknown name. */
int wave16_shape_from_name(const char *name, wave16_shape_t *shape);

/* One period of `shape` in n samples around `mid` with peak `amp`, both in 16-bit DAC codes
   (fractional, so rounding happens once per sample). `amp` may be negative: on a path whose
   output falls as the code rises, the waveform still comes out the right way up in volts.
   Samples are rounded and clamped to 0..65535. */
void wave16_fill(wave16_shape_t shape, uint32_t n, double amp, double mid, uint16_t *out);

/* Volts on DAC path `path` (cal_data.h dac[] index: 0 = 3v3, 1 = 5v, 2 = 12v) -> *amp and *mid
   in 16-bit codes for wave16_fill. NULL on success, else why the request is refused (the peaks
   fall outside what the path can output, or the amplitude is not positive). */
const char *wave16_codes_for_volts(int path, double amplitude_v, double offset_v,
                                   double *amp, double *mid);

/* The same for the 4-20 mA output, in microamps (current_out.h). */
const char *wave16_codes_for_ua(double amplitude_ua, double offset_ua, double *amp, double *mid);

#endif /* WAVE16_H */
