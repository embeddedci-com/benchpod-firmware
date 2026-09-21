/*
 * adc_scale.h — reduce a raw MCP33131 sample burst to ONE calibrated reading.
 *
 * Why this is its own module: the v2 analog front end (OPA810 + THS4551) is
 * INVERTING with a large DC offset, so `volts = a + b*count` has b NEGATIVE and
 * 0 V sits at count ~65538 — i.e. just PAST the 16-bit top.  A near-zero or
 * negative input therefore WRAPS to a small count (0, 1, 2, ...) and has to be
 * unwrapped (count += 65536) before the affine fit means anything.  See
 * cal_data.h and the server's adcScaling (benchpod_capture.go), which unwraps
 * every capture sample individually.
 *
 * The bug this module exists to prevent (2026-07-29): handle_adc_read averaged
 * the 16 RAW counts and unwrapped only the AVERAGE.  Every analog source's quiet
 * operating point sits WITHIN A FEW COUNTS OF THAT WRAP (measured on pod
 * 192.168.1.215: ext 24..38, cal1@dac0 65524..65530, amp 0..9 — the wrap
 * boundary IS the 0 V point), so as soon as one sample in a burst lands on the
 * other side of the cut, the raw mean lands mid-scale, gets unwrapped as if it
 * were a single huge negative reading, and adc_read returns a PLAUSIBLE number
 * that is wrong by tens of volts — quantised in steps of 65536/16 = 4096 counts.
 * It never errored, so a calibration sweep silently recorded garbage.
 *
 * The fix is to treat the count as what it physically is — a CIRCULAR 16-bit
 * value — and average it on the circle (shortest-arc deltas from a reference
 * sample) BEFORE applying the branch cut.  That is correct for a burst sitting
 * anywhere, including straddling the cut.
 *
 * The unwrap is unconditional, i.e. a property of the shared front end rather
 * than of the selected source.  It cannot misfire: with b < 0 a count below
 * half-scale that is NOT wrapped would decode to > +32 V, which no path on this
 * board can present.  Doing it for every source also fixes `amp` and `cal1`,
 * which were scaled with the unwrap DISABLED and so reported ~65.8 V (!) for a
 * 0 V input.
 */
#ifndef ADC_SCALE_H
#define ADC_SCALE_H

#include <stdint.h>
#include <stddef.h>

/* Widest wrap-corrected peak-to-peak spread (in ADC counts) a burst may show and
 * still be reported as a settled DC reading.  On this front end 1 count ~= 1 mV,
 * and a burst spans only ~40 us, so 1024 counts (~1 V pk-pk) is far outside any
 * real noise floor (measured: 5..15 counts) yet trips immediately on an input
 * that is still MOVING — e.g. a DAC left driving the node by a preceding
 * `measure`/`generate`.  Such a burst has no single voltage, so adc_read must
 * refuse it instead of averaging it into a plausible-looking number. */
#define ADC_BURST_MAX_SPAN 1024u

typedef struct {
    int      valid;    /* 0 => not a settled DC level; volts/count are meaningless */
    float    count;    /* mean count folded back into 0..65535 (what the reply reports) */
    float    count_uw; /* mean count UNWRAPPED (32768..98303): what the fit is applied to */
    float    volts;    /* a + b*count_uw */
    uint32_t span;     /* wrap-corrected peak-to-peak of the burst, in counts */
} adc_reading_t;

/* Average `n` raw counts on the 16-bit circle, unwrap, and apply volts = a + b*c.
 * `n` must be >= 1; raw must be non-NULL.  A degenerate call returns valid=0. */
adc_reading_t adc_scale_burst(const uint16_t *raw, size_t n, float a, float b);

#endif /* ADC_SCALE_H */
