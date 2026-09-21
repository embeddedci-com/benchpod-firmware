/* capture_format.h — correlated ADC + I2C-LA capture record format (v1).
 *
 * The FPGA `capture_merge` block serialises two fixed-rate sample streams (the
 * MCP33131 ADC and the I2C-LA sampler) into ONE byte stream written to PSRAM by
 * psram_writer.  Firmware reads the PSRAM region back and walks it with the
 * decoder below to recover a single correlated timeline.
 *
 * Records are forward-parseable: the lead tag byte fixes the record length, so
 * raw payload bytes (any value) are never mistaken for tags.
 *
 *   0x00 lo hi      ADC sample, 16-bit little-endian        (3 bytes)
 *   0x01 b          I2C-LA sample byte (4 packed bus samples) (2 bytes)
 *   0x02 t0 t1 t2   timestamp marker, 24-bit little-endian tick (4 bytes)
 *
 * A timestamp marker is emitted at capture start and whenever the active source
 * switches, so every run of same-source samples is time-anchored.  Within a run
 * the samples are exactly one source-period apart, so sample i of the run lands
 * at  ts + i * period_ticks(source)  — the period comes from the capture config
 * (cap_divider / i2c_la_divider), which firmware already knows.
 *
 * v1 note: the 24-bit tick wraps (~0.7 s at a 24 MHz raw tick, ~16 s at a 1 us
 * tick).  For longer captures, emit periodic in-run markers and handle wrap in
 * firmware; the format already allows extra 0x02 records mid-run.
 *
 * Header-only and dependency-free so the same decoder compiles into both the
 * rp2350 and stm32h563 firmware and the host round-trip test.
 */
#ifndef CAPTURE_FORMAT_H
#define CAPTURE_FORMAT_H

#include <stdint.h>
#include <stddef.h>

#define CAP_TAG_ADC 0x00u
#define CAP_TAG_LA  0x01u
#define CAP_TAG_TS  0x02u

#define CAP_REC_ADC_LEN 3u
#define CAP_REC_LA_LEN  2u
#define CAP_REC_TS_LEN  4u

/* Reconstructed sample handed to the caller's callback. */
typedef struct {
    uint32_t tick;     /* absolute tick (anchor + index*period) */
    uint8_t  source;   /* CAP_TAG_ADC or CAP_TAG_LA */
    uint16_t value;    /* ADC: 16-bit sample; LA: low 8 bits = packed byte */
} cap_sample_t;

typedef void (*cap_sample_cb)(void *ctx, const cap_sample_t *s);

/* Walk buf[0..len), invoking cb for each reconstructed sample in order.
 * period_adc / period_la are the tick spacing between consecutive samples of
 * each source.  max_samples bounds decoding (0 = unbounded) — callers that read
 * back a worst-case-sized PSRAM region pass the known sample count so trailing
 * bytes past the real data aren't mis-parsed.  Returns the number of samples
 * decoded, or -1 on a malformed (truncated) record. */
static inline int cap_decode(const uint8_t *buf, size_t len,
                             uint32_t period_adc, uint32_t period_la,
                             int max_samples, cap_sample_cb cb, void *ctx)
{
    size_t   i = 0;
    int      n = 0;
    uint32_t anchor = 0;   /* tick of the current run's first sample */
    uint32_t idx    = 0;   /* sample index within the current run */

    while (i < len) {
        if (max_samples && n >= max_samples) break;
        uint8_t tag = buf[i];
        if (tag == CAP_TAG_TS) {
            if (i + CAP_REC_TS_LEN > len) return -1;
            anchor = (uint32_t)buf[i + 1]
                   | ((uint32_t)buf[i + 2] << 8)
                   | ((uint32_t)buf[i + 3] << 16);
            idx = 0;
            i += CAP_REC_TS_LEN;
        } else if (tag == CAP_TAG_ADC) {
            if (i + CAP_REC_ADC_LEN > len) return -1;
            cap_sample_t s;
            s.source = CAP_TAG_ADC;
            s.value  = (uint16_t)(buf[i + 1] | (buf[i + 2] << 8));
            s.tick   = anchor + idx * period_adc;
            if (cb) cb(ctx, &s);
            idx++; n++;
            i += CAP_REC_ADC_LEN;
        } else if (tag == CAP_TAG_LA) {
            if (i + CAP_REC_LA_LEN > len) return -1;
            cap_sample_t s;
            s.source = CAP_TAG_LA;
            s.value  = buf[i + 1];
            s.tick   = anchor + idx * period_la;
            if (cb) cb(ctx, &s);
            idx++; n++;
            i += CAP_REC_LA_LEN;
        } else {
            return -1;   /* unknown tag = stream corruption */
        }
    }
    return n;
}

#endif /* CAPTURE_FORMAT_H */
