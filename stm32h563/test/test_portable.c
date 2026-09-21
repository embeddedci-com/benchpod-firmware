/*
 * test_portable.c — host unit tests for the platform-independent firmware logic
 * (no STM32 HAL).  Compiled + run natively (see test/Makefile, `make test`).
 *
 * Covers: b64url codec roundtrip + known vectors, BMP280 compensation register
 * image, and the v2 sample-rate divider arithmetic shared with the gateware.
 */
#include "b64url.h"
#include "sensor_sim.h"
#include "sensor_bmp280.h"
#include "esp_hosted_frame.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

/* ---- b64url ---- */
static void test_b64url_roundtrip(void)
{
    /* every byte value survives encode->decode */
    uint8_t in[256];
    for (int i = 0; i < 256; i++) in[i] = (uint8_t)i;
    char   enc[400];
    uint8_t dec[256];
    size_t n = b64url_encode(in, sizeof(in), enc, sizeof(enc));
    CHECK(n > 0);
    /* base64url uses only [A-Za-z0-9-_], no padding */
    for (size_t i = 0; i < n; i++) {
        char c = enc[i];
        int ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                 (c >= '0' && c <= '9') || c == '-' || c == '_';
        CHECK(ok);
    }
    size_t dl = 0;
    CHECK(b64url_decode(enc, dec, sizeof(dec), &dl) == 0);
    CHECK(dl == sizeof(in));
    CHECK(memcmp(in, dec, sizeof(in)) == 0);
}

static void test_b64url_vectors(void)
{
    /* RFC 4648 base64url, no padding: "foobar" */
    char enc[16];
    size_t n = b64url_encode((const uint8_t *)"foobar", 6, enc, sizeof(enc));
    enc[n] = '\0';
    CHECK(strcmp(enc, "Zm9vYmFy") == 0);

    uint8_t dec[16];
    size_t dl = 0;
    CHECK(b64url_decode("Zm9vYmFy", dec, sizeof(dec), &dl) == 0);
    CHECK(dl == 6 && memcmp(dec, "foobar", 6) == 0);

    /* a value that exercises the - and _ alphabet (0xFB 0xFF -> "-_8") */
    uint8_t raw[2] = { 0xFB, 0xFF };
    n = b64url_encode(raw, 2, enc, sizeof(enc));
    enc[n] = '\0';
    CHECK(strcmp(enc, "-_8") == 0);
}

/* ---- BMP280 model: register image is sane ---- */
static void test_bmp280_regimage(void)
{
    sensor_model_t *m = sensor_bmp280_model();
    CHECK(m != NULL);
    CHECK(strcmp(m->name, "bmp280") == 0);
    CHECK(m->default_addr7 == 0x76);

    uint8_t img[SENSOR_REGIMAGE_LEN];
    memset(img, 0xAA, sizeof(img));
    m->reset(m);
    m->build_regimage(m, img);
    /* chip-id register (0xD0) reports BMP280 = 0x58 */
    CHECK(img[0xD0] == 0x58);
    /* setting a temperature changes the temp ADC registers (0xFA..0xFC) */
    uint8_t before[3] = { img[0xFA], img[0xFB], img[0xFC] };
    CHECK(m->set_param(m, "temperature_c", 42.0f) == 0);
    m->build_regimage(m, img);
    CHECK(memcmp(before, &img[0xFA], 3) != 0);
}

/* ---- divider arithmetic (mirrors signal_engine divider_from_rate_hz) ---- */
static uint32_t divider_from_rate_hz(float rate, uint32_t hfosc)
{
    if (rate <= 0.0f) return 2;
    uint32_t d = (uint32_t)((float)hfosc / rate + 0.5f);
    if (d < 2)     d = 2;
    if (d > 65535) d = 65535;
    return d;
}
static void test_divider(void)
{
    CHECK(divider_from_rate_hz(0.0f, 24000000u) == 2);          /* max rate */
    CHECK(divider_from_rate_hz(1000000.0f, 24000000u) == 24);   /* 1 MSPS */
    CHECK(divider_from_rate_hz(1.0f, 24000000u) == 65535);      /* clamp */
}

/* ---- DAC generate frequency accuracy (mirrors compute_divider_and_period) ----
   The DAC8551 sequencer spends DAC_SEQ_OVERHEAD_CLK extra clocks/sample, so the
   real rate is HFOSC/(divider+K) and generate() must compute the period from THAT
   or the played frequency saturates far below the request. */
#define T_HFOSC      48000000u   /* DAC engine runs on clk48 (gateware >= 13) */
#define T_DAC_K      51u
#define T_DAC_MIN    3u          /* DAC_MIN_DIVIDER: DAC8551 t9 frame gap (gateware >= v34 floors to it) */
#define T_SIGNAL_MAX 2048u
static float t_dac_eff_rate(uint32_t divider)
{
    return (float)T_HFOSC / (float)(divider + T_DAC_K);
}
static uint32_t t_compute_dp(float freq, float forced, uint32_t *out_period)
{
    if (freq <= 0.0f) { *out_period = 0; return 0; }
    uint32_t divider;
    if (forced > 0.0f) {
        float d = (float)T_HFOSC / forced - (float)T_DAC_K;
        divider = (d < (float)T_DAC_MIN) ? T_DAC_MIN : (d > 65535.0f) ? 65535u : (uint32_t)(d + 0.5f);
    } else {
        float dmin = (float)T_HFOSC / ((float)T_SIGNAL_MAX * freq) - (float)T_DAC_K;
        divider = (dmin < (float)T_DAC_MIN) ? T_DAC_MIN : (dmin >= 65535.0f) ? 65535u : (uint32_t)dmin + 1u;
    }
    float    eff    = t_dac_eff_rate(divider);
    uint32_t period = (uint32_t)(eff / freq + 0.5f);
    if (period == 0) period = 1;
    if (period > T_SIGNAL_MAX) period = T_SIGNAL_MAX;
    *out_period = period;
    return divider;
}
static void test_dac_freq_accuracy(void)
{
    /* Auto-rate: generate(freq) plays within a couple % of freq across the range,
       and the period always fits the 16-bit waveform buffer (<= 2048 samples). */
    const float freqs[] = { 100.f, 250.f, 500.f, 1000.f, 2000.f, 5000.f, 10000.f };
    for (size_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); i++) {
        uint32_t period = 0, divider = t_compute_dp(freqs[i], 0.0f, &period);
        CHECK(divider >= T_DAC_MIN && period >= 1 && period <= T_SIGNAL_MAX);
        float actual = t_dac_eff_rate(divider) / (float)period;
        float err = (actual - freqs[i]) / freqs[i];
        CHECK((err < 0 ? -err : err) < 0.02f);          /* within 2% */
    }
    /* The fastest generator setting and a forced rate above the peak both stop at the floor. */
    uint32_t pf = 0;
    CHECK(t_compute_dp(10000.f, 0.0f, &pf) == T_DAC_MIN);
    CHECK(t_compute_dp(1000.f, 5000000.f, &pf) == T_DAC_MIN);
    /* Forced effective rate: output frequency still lands on the request. */
    uint32_t p = 0, d = t_compute_dp(1000.f, 100000.f, &p);
    float a = t_dac_eff_rate(d) / (float)p;
    CHECK(a > 980.f && a < 1020.f);
    /* Regression: 1 kHz at a nominal 1 MS/s must NOT saturate to ~310 Hz anymore. */
    d = t_compute_dp(1000.f, 1000000.f, &p);
    a = t_dac_eff_rate(d) / (float)p;
    CHECK(a > 950.f && a < 1050.f);
}

/* ---- esp-hosted transport framing ---- */
static void test_esp_hosted_frame_roundtrip(void)
{
    /* 12-byte header is the contract with the ESP32-C3 slave. */
    CHECK(ESP_HOSTED_HDR_LEN == 12);

    const uint8_t payload[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03 };
    uint8_t frame[64];
    size_t n = esp_hosted_frame_build(frame, sizeof(frame),
                                      ESP_STA_IF, 0, MORE_FRAGMENT,
                                      0, 0x1234, payload, sizeof(payload));
    CHECK(n == ESP_HOSTED_HDR_LEN + sizeof(payload));

    /* Wire layout: if-byte, len(LE), offset(LE) at the expected positions. */
    CHECK(frame[0] == ESP_STA_IF);               /* if_num=0, if_type=STA */
    CHECK(frame[1] == MORE_FRAGMENT);
    CHECK((frame[2] | (frame[3] << 8)) == sizeof(payload));
    CHECK((frame[4] | (frame[5] << 8)) == ESP_HOSTED_HDR_LEN);   /* offset */
    CHECK((frame[8] | (frame[9] << 8)) == 0x1234);               /* seq_num */

    esp_hosted_rx_t rx;
    CHECK(esp_hosted_frame_parse(frame, n, &rx) == 0);
    CHECK(rx.if_type == ESP_STA_IF);
    CHECK(rx.if_num == 0);
    CHECK(rx.flags == MORE_FRAGMENT);
    CHECK(rx.seq_num == 0x1234);
    CHECK(rx.payload_len == sizeof(payload));
    CHECK(memcmp(rx.payload, payload, sizeof(payload)) == 0);
}

static void test_esp_hosted_frame_checksum(void)
{
    uint8_t payload[] = { 0x10, 0x20, 0x30, 0x40 };
    uint8_t frame[32];
    size_t n = esp_hosted_frame_build(frame, sizeof(frame),
                                      ESP_SERIAL_IF, 1, 0, 0, 7, payload, sizeof(payload));
    CHECK(n > 0);
    esp_hosted_rx_t rx;
    CHECK(esp_hosted_frame_parse(frame, n, &rx) == 0);
    CHECK(rx.if_type == ESP_SERIAL_IF && rx.if_num == 1);

    /* Corrupting any byte must fail the checksum. */
    frame[ESP_HOSTED_HDR_LEN + 2] ^= 0xFF;
    CHECK(esp_hosted_frame_parse(frame, n, &rx) == -1);
}

static void test_esp_hosted_frame_priv_and_reject(void)
{
    /* A priv-interface frame carries a packet type in the union byte
       (0x33 == ESP_PACKET_TYPE_EVENT in the transport header). */
    const uint8_t kPktTypeEvent = 0x33;
    uint8_t frame[24];
    size_t n = esp_hosted_frame_build(frame, sizeof(frame),
                                      ESP_PRIV_IF, 0, 0,
                                      kPktTypeEvent, 0, NULL, 0);
    CHECK(n == ESP_HOSTED_HDR_LEN);
    esp_hosted_rx_t rx;
    CHECK(esp_hosted_frame_parse(frame, n, &rx) == 0);
    CHECK(rx.if_type == ESP_PRIV_IF);
    CHECK(rx.priv_pkt_type == kPktTypeEvent);
    CHECK(rx.payload_len == 0);

    /* Too-short buffer and an out-of-range offset are rejected. */
    CHECK(esp_hosted_frame_parse(frame, 4, &rx) == -1);
    frame[4] = 0xFF; frame[5] = 0xFF;            /* offset way past the buffer */
    CHECK(esp_hosted_frame_parse(frame, n, &rx) == -1);
}

int main(void)
{
    test_b64url_roundtrip();
    test_b64url_vectors();
    test_bmp280_regimage();
    test_divider();
    test_dac_freq_accuracy();
    test_esp_hosted_frame_roundtrip();
    test_esp_hosted_frame_checksum();
    test_esp_hosted_frame_priv_and_reject();
    if (failures == 0) { printf("PASS — all portable tests\n"); return 0; }
    printf("FAILED — %d check(s)\n", failures);
    return 1;
}
