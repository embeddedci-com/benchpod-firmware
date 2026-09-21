#ifndef DAC_LOOP_PARAMS_H
#define DAC_LOOP_PARAMS_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Pure, host-testable core of the closed-loop DAC control command
 * ({"cmd":"dac_control_loop"} -> handle_dac_control_loop): the curve UPSAMPLE and the
 * PARAMETER VALIDATION.  Extracted from command_handler.c so test/test_dac_loop_params.c
 * can pin the exact index mapping the gateware (ice40/src/dac_loop.v) and every client's
 * expectation depend on — a silent change here mis-shapes the emulated panel curve on a
 * live bench, and only a hardware run would notice.  See docs/dac-control-loop.md. */

/* The gateware curve LUT is a fixed table indexed by `adc >> SHIFT` with SHIFT=5, i.e.
 * 2048 entries of 16-bit LE (dac_loop.v: .ADDR_W(12), .SHIFT(5) -> 4 KB of sample_buf). */
#define DAC_LOOP_CURVE_POINTS  2048u
#define DAC_LOOP_CURVE_BYTES   (DAC_LOOP_CURVE_POINTS * 2u)

/* The loop's Q15 damping coefficient is used as k[14:0] in fabric (dac_loop.v S_MUL takes
 * the magnitude), so 32767 == "full step to the curve target" and anything above it would
 * silently wrap to a small k.  Clamped, not rejected, so a client that thinks Q15 is 32768
 * still gets the fastest loop rather than an error. */
#define DAC_LOOP_K_MAX         32767u
#define DAC_LOOP_K_MIN         1u        /* k=0 freezes the output at vmin — never intended */
/* Control period floor in clk48 cycles: below ~8 the pipelined tick (2 BRAM reads + MAC +
 * add + clamp) has not retired the previous pass. */
#define DAC_LOOP_TICK_MIN      8u

/* Where the loop's INPUT comes from (DAC_LOOP_SRC 0x1A, gateware >= v29).  The curve, the
 * damping and the clamp are identical in all three; only the value that indexes the curve
 * differs.  The two OPEN-loop sources take the ADC and the whole analog input path out of
 * the picture, which is how the DAC/output side gets validated on its own before anyone
 * trusts a closed loop (hold a point, meter the output, move to the next). */
typedef enum {
    DAC_LOOP_SRC_ADC   = 0,   /* live ADC — closed loop (default; pre-v29 behaviour) */
    DAC_LOOP_SRC_FIXED = 1,   /* host-held constant — open loop, one curve point at a time */
    DAC_LOOP_SRC_SWEEP = 2,   /* input += step every tick (mod 65536) — open loop, f(time) */
} dac_loop_src_t;
#define DAC_LOOP_SRC_MAX  2u

/* Loop parameters as they go to the gateware (START_DAC_LOOP 0x15 + DAC_LOOP_SRC 0x1A). */
typedef struct {
    uint16_t k_q15;
    uint16_t vmin;
    uint16_t vmax;
    uint16_t tick_div;
    uint8_t  src;         /* dac_loop_src_t */
    uint16_t in_fixed;    /* SRC_FIXED value; also the SRC_SWEEP start point */
    uint16_t sweep_step;  /* SRC_SWEEP increment per tick */
} dac_loop_params_t;

/* Why a parameter set was refused (dac_loop_params_validate / dac_loop_inmap_derive). */
typedef enum {
    DAC_LOOP_PARAMS_OK = 0,
    DAC_LOOP_PARAMS_ERR_CLAMP_INVERTED,   /* vmin > vmax */
    DAC_LOOP_PARAMS_ERR_BAD_SOURCE,       /* src outside dac_loop_src_t */
    DAC_LOOP_PARAMS_ERR_SWEEP_STEP_ZERO,  /* SRC_SWEEP with step 0 = a frozen "sweep" */
    DAC_LOOP_PARAMS_ERR_INPUT_GAIN,       /* sense chain has no (or negative) gain */
    DAC_LOOP_PARAMS_ERR_INPUT_RANGE,      /* range max <= min */
    DAC_LOOP_PARAMS_ERR_INPUT_SPAN,       /* range spans >= half the ADC: the count wraps
                                             ambiguously and cannot be indexed */
} dac_loop_params_err_t;

/* ---- INPUT MAP (gateware >= v30, DAC_LOOP_INMAP 0x1C) ---------------------------------
 *
 * The gateware indexes the curve with the raw input count.  Nothing about a raw count is
 * meaningful to whoever wired the bench, and on this board the analog front end is
 * inverting, offset ~65.79 V and wrapping near 0 V — so a real measurement window sits
 * backwards inside a few percent of the curve.  v30 puts an affine map in front of the
 * index; deriving its registers is THIS module's job, because the firmware is the only
 * place that knows both the front-end calibration (cal_data.h) and the request.
 *
 * That is the whole reason the map is derived here and not on the host: a client sends the
 * bench in ENGINEERING UNITS and never has to carry a copy of the ADC calibration, which
 * would then be free to drift away from the board's own. */

/* One linear ADC fit: volts = a + b*count, b negative on this (inverting) front end.
 * Deliberately a local type rather than cal_data.h's, so this module stays pure and a
 * caller can map a different analog path by passing that path's fit. */
typedef struct { float a, b; } dac_loop_adc_cal_t;

/* The bench's sense chain, as the client describes it: mV = mv_at_zero + value*mv_per_unit.
 * For a 0.04 ohm shunt into an INA282 (fixed 50 V/V, REF to GND) that is 0 and 2.0 mV/mA.
 * `range_min`/`range_max` bound the axis the curve is authored over — they are what maps
 * onto the LUT, and inputs outside them saturate to the curve's ends. */
typedef struct {
    float mv_at_zero;
    float mv_per_unit;
    float range_min;
    float range_max;
    float trip;        /* trip level, same unit; only read when trip_en */
    bool  trip_en;
} dac_loop_input_t;

/* What goes on the wire to DAC_LOOP_INMAP 0x1C. */
typedef struct {
    uint16_t in_zero;   /* count that maps to curve index 0 */
    int16_t  in_gain;   /* signed Q15; negative on an inverting front end */
    uint16_t in_trip;   /* trip threshold as a curve index */
    uint16_t idx_max;   /* index range_max lands on — NOT sent, but the curve must be
                           upsampled over it (see dac_loop_upsample_curve_mapped) */
    bool     map_en;
    bool     trip_en;
} dac_loop_inmap_t;

/* Derive the gateware's input-map registers from the bench description and the analog
 * path's calibration.  Rejects a window that spans half the ADC or more: past that the
 * 16-bit modular subtract the fabric relies on stops being an unambiguous distance, so two
 * different readings would index the same curve point. */
dac_loop_params_err_t dac_loop_inmap_derive(const dac_loop_input_t *in,
                                            dac_loop_adc_cal_t cal,
                                            dac_loop_inmap_t *out);

/* Validate + normalise in place.  CLAMPS the fields whose out-of-range values have an
 * obvious safe reading (k into [1,32767], tick_div up to >= 8) and REJECTS the one that
 * does not: an inverted output window (vmin > vmax).  The gateware's S_CLAMP tests vmin
 * first, so an inverted window silently pins the DAC at vmin and the caller's ceiling is
 * never applied — exactly the kind of "armed fine, did something else" failure a bench
 * emulator must not have.  Returns DAC_LOOP_PARAMS_OK when the (possibly clamped) set is
 * usable. */
dac_loop_params_err_t dac_loop_params_validate(dac_loop_params_t *p);

/* Human-readable reason for a non-OK validate result (stable, client-facing text). */
const char *dac_loop_params_err_str(dac_loop_params_err_t e);

/* Wire name <-> dac_loop_src_t for the JSON `source` field ("adc" | "fixed" | "sweep").
 * Parsing returns false on an unknown name rather than silently falling back to the ADC:
 * a client asking for an open-loop run that quietly got a closed one would be metering a
 * value the loop never produced.  A NULL/empty name means "unspecified" -> false. */
bool        dac_loop_src_parse(const char *name, uint8_t *out_src);
const char *dac_loop_src_str(uint8_t src);

/* Expand a COMPACT uploaded curve to the full DAC_LOOP_CURVE_POINTS gateware LUT.
 *
 * A curve-bearing command has to fit one command frame (BP_CLOUD_CMD_IN_MAX), so clients
 * upload a small curve (the webapp: 256 points) and we nearest-neighbour upsample it here
 * so it spans the whole ADC range and no stale upper entries survive from a previous load.
 * `in_bytes` is 16-bit LE point data; an upload of >= DAC_LOOP_CURVE_POINTS points passes
 * through (truncated to the LUT depth).  `out` must have room for DAC_LOOP_CURVE_BYTES.
 *
 * Index mapping (what every client predicts against): out point j takes in point
 * (j * in_pts) / 2048, and the gateware then reads out[adc >> 5] — so for a 256-point
 * upload the live ADC selects source point `adc >> 8`.
 *
 * Returns the number of bytes written to `out` (always DAC_LOOP_CURVE_BYTES), or 0 if the
 * input is unusable (no points) or `out_cap` is too small. */
size_t dac_loop_upsample_curve(const uint8_t *in, size_t in_bytes,
                               uint8_t *out, size_t out_cap);

/* Expand a compact curve for a MAPPED loop (gateware >= v30).
 *
 * Two differences from dac_loop_upsample_curve(), both consequences of the conditioner:
 *
 *  - The authored curve spans the client's [range_min, range_max], which the map lands on
 *    indices 0..`idx_max` — not 0..2047.  Spreading it over the whole LUT the old way would
 *    compress the curve into the part of the table the input can actually reach.  Entries
 *    above idx_max hold the curve's last point, which is also what the fabric's saturation
 *    lands on for an over-range input (it pins at 2047), so "past the end of the range"
 *    reads as "the end of the curve" rather than a stale entry.
 *
 *  - It LINEARLY INTERPOLATES between uploaded points instead of taking the nearest.  With
 *    the window now spanning the LUT, one uploaded point covers several entries, and
 *    nearest-neighbour would turn a smooth curve into an audible staircase on the output.
 *    Interpolating costs nothing here and removes the need to upload a bigger curve.
 *
 * Returns bytes written (DAC_LOOP_CURVE_BYTES), or 0 if the input or idx_max is unusable. */
size_t dac_loop_upsample_curve_mapped(const uint8_t *in, size_t in_bytes, uint16_t idx_max,
                                      uint8_t *out, size_t out_cap);

#endif /* DAC_LOOP_PARAMS_H */
