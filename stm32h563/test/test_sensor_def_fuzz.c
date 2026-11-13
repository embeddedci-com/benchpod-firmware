/*
 * test_sensor_def_fuzz.c — robustness of the sensor definition engine, built with ASan and
 * UBSan: truncated, mutated, huge and hostile definitions and stimuli must be refused or run
 * within their limits, never crash, never write out of bounds, never exceed SD_PROG_MAX.
 * Deterministic (a fixed PRNG seed), so a failure reproduces.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sensor_def.h"

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static _Alignas(8) uint8_t g_prog[SD_PROG_MAX];
static _Alignas(8) uint8_t g_work[SD_COMPILE_WORK];
static _Alignas(8) uint8_t g_stim[SD_STIM_MAX];
static sd_inst_t g_in;

static uint64_t rng = 0x243F6A8885A308D3ull;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return (uint32_t)(rng >> 16); }
static double rndd(void) {
    static const double v[] = { 0, 1, -1, 0.5, 1e9, -1e9, 1e300, -1e300, 255, 256, 65535, 1e-300, 3.3, 100 };
    return (rnd() & 1) ? v[rnd() % 14] : ((double)rnd() / 4294967296.0 - 0.5) * 1e4;
}

static unsigned g_compiled, g_tried;

/* Run whatever compiled, a few times, with random params, images and input. */
static void exercise(void) {
    sd_err_t e;
    if (sd_inst_init(&g_in, g_prog, ((const uint32_t *)(const void *)g_prog)[1], &e) != SD_OK) {
        CHECK(0, "a compiled program does not start: %s", e.msg);
        return;
    }
    unsigned np = sd_n_params(&g_in);
    for (int round = 0; round < 3; round++) {
        for (unsigned i = 0; i < np; i++) (void)sd_set_param(&g_in, i, rndd(), rnd() & 1);
        sd_set_time_ms(&g_in, (int64_t)(rnd() % 100000));
        switch (sd_kind(&g_in)) {
        case SD_KIND_I2C: case SD_KIND_SPI: {
            uint8_t img[256], live[256];
            sd_img_result_t r;
            (void)sd_regs_power_on(&g_in, img, &r);
            for (int k = 0; k < 256; k++) live[k] = (uint8_t)rnd();
            (void)sd_regs_rebuild(&g_in, live, img, &r);
            (void)sd_regs_after_write(&g_in, live, (int)(rnd() % 257) - 1, (int)(rnd() % 256), img, &r);
            (void)sd_regs_after_write(&g_in, img, -1, -1, img, &r);
            double hz;
            bool en;
            uint8_t fr[64];
            size_t n;
            (void)sd_drdy_eval(&g_in, img, &hz, &en);
            (void)sd_fifo_eval(&g_in, img, &hz, &en);
            (void)sd_fifo_frame(&g_in, img, fr, sizeof(fr), &n);
            sd_fault_t f = { .type = SD_FAULT_CORRUPT, .at = (int16_t)(rnd() % 256), .mask = 0x5A, .duration_s = -1 };
            (void)sd_set_fault(&g_in, &f, true);
            (void)sd_regs_rebuild(&g_in, img, img, &r);
            break;
        }
        case SD_KIND_UART: {
            static char out[2048];
            size_t n;
            int64_t at;
            uint8_t ans[256];
            uint32_t d;
            for (int k = 0; k < 3; k++) {
                int rc = sd_uart_epoch(&g_in, out, (rnd() & 3) ? sizeof(out) : rnd() % 64, &n, &at);
                CHECK(rc != SD_OK || n <= sizeof(out), "epoch length");
            }
            for (int k = 0; k < 64; k++)
                (void)sd_uart_rx(&g_in, (int64_t)k * (rnd() % 150), (uint8_t)((rnd() & 1) ? rnd() : "AT+READ?\r\n\xFF\x01\x86"[k % 13]),
                                 ans, (rnd() & 3) ? sizeof(ans) : rnd() % 8, &n, &d);
            break;
        }
        case SD_KIND_ANALOG: {
            double v;
            (void)sd_analog_eval(&g_in, rndd(), &v);
            break;
        }
        default: break;
        }
    }
    CHECK(sd_max_steps(&g_in) <= SD_STEP_LIMIT + 1, "step limit held (%u)", sd_max_steps(&g_in));
}

static int try_def(const char *j, size_t n) {
    size_t len = 0;
    sd_err_t e;
    g_tried++;
    int rc = sd_compile(j, n, g_prog, sizeof(g_prog), g_work, sizeof(g_work), &len, &e);
    if (rc == SD_OK) {
        g_compiled++;
        CHECK(len > 0 && len <= SD_PROG_MAX, "program size %zu", len);
        exercise();
    } else {
        char m[256];
        size_t k = sd_err_format(&e, m, sizeof(m));
        CHECK(k < sizeof(m) && e.msg[0], "an error has a message (rc %d)", rc);
    }
    return rc;
}

static char *read_file(const char *path, size_t *n) {
    static char bufs[6][SD_JSON_MAX];
    static int k;
    FILE *f = fopen(path, "rb");
    if (!f) { printf("FAIL: cannot open %s\n", path); fails++; *n = 0; return NULL; }
    char *b = bufs[k++ % 6];
    *n = fread(b, 1, SD_JSON_MAX, f);
    fclose(f);
    return b;
}

static void mutate(char *b, size_t *n, size_t cap) {
    static const char pieces[] = "{}[],:\"0123456789+-*/()?'xe_.\\ !~<>=&|^%";
    int kind = (int)(rnd() % 6);
    size_t at = *n ? rnd() % *n : 0;
    switch (kind) {
    case 0: if (*n) b[at] = (char)rnd(); break;                            /* any byte */
    case 1: if (*n) b[at] = pieces[rnd() % (sizeof(pieces) - 1)]; break;   /* a JSON / expression char */
    case 2: if (*n + 1 < cap) { memmove(b + at + 1, b + at, *n - at); b[at] = pieces[rnd() % (sizeof(pieces) - 1)]; (*n)++; } break;
    case 3: { size_t l = rnd() % 16; if (at + l > *n) l = *n - at; memmove(b + at, b + at + l, *n - at - l); *n -= l; break; }
    case 4: { size_t l = rnd() % 64, from = *n ? rnd() % *n : 0; if (from + l > *n) l = *n - from;   /* duplicate a span */
              if (*n + l < cap) { memmove(b + at + l, b + at, *n - at); memmove(b + at, b + from + (from >= at ? l : 0), l); *n += l; } break; }
    default: { /* a digit run into a big number */
        if (*n + 8 < cap) { memmove(b + at + 8, b + at, *n - at); memcpy(b + at, "99999999", 8); *n += 8; } break; }
    }
}

/* Change one character inside a JSON string (an expression, a template, a name): the JSON stays
   valid, so the compiler and the VM get exercised rather than the JSON reader. */
static void mutate_in_string(char *b, size_t n) {
    static const char pieces[] = "0123456789+-*/()?:'xe_. !~<>=&|^%,[]{}_tpi";
    for (int tries = 0; tries < 32; tries++) {
        size_t at = n ? rnd() % n : 0;
        size_t q = 0;
        for (size_t k = 0; k < at; k++) if (b[k] == '"' && (k == 0 || b[k - 1] != '\\')) q++;
        if (!(q & 1) || b[at] == '"' || b[at] == '\\') continue;
        b[at] = pieces[rnd() % (sizeof(pieces) - 1)];
        return;
    }
}

static void test_fixtures(void) {
    static const char *names[] = { "bmp280", "bme280", "sht4x", "mpu6050", "gps_nmea", "word_tmp117", "cmap_scd4x" };
    static char b[SD_JSON_MAX + 64];
    for (unsigned i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
        char path[96];
        snprintf(path, sizeof(path), "fixtures/sensor_defs/%s.json", names[i]);
        size_t n;
        char *j = read_file(path, &n);
        if (!j) continue;
        CHECK(try_def(j, n) == SD_OK, "%s compiles", names[i]);
        /* every truncation */
        for (size_t k = 0; k < n; k++) CHECK(try_def(j, k) != SD_OK || k + 2 >= n, "%s cut at %zu compiled", names[i], k);
        /* random mutations, a few at a time */
        for (int it = 0; it < 3000; it++) {
            size_t m = n;
            memcpy(b, j, n);
            int muts = 1 + (int)(rnd() % 4);
            for (int q = 0; q < muts; q++) mutate(b, &m, SD_JSON_MAX);
            (void)try_def(b, m);
        }
        for (int it = 0; it < 6000; it++) {
            memcpy(b, j, n);
            int muts = 1 + (int)(rnd() % 3);
            for (int q = 0; q < muts; q++) mutate_in_string(b, n);
            (void)try_def(b, n);
        }
    }
}

static size_t cat(char *o, size_t cap, size_t n, const char *s) {
    size_t l = strlen(s);
    if (n + l < cap) { memcpy(o + n, s, l); n += l; o[n] = 0; }
    return n;
}

static void analog_expr(char *o, size_t cap, const char *extra, const char *expr) {
    snprintf(o, cap, "{\"format\":1,\"kind\":\"analog\",\"name\":\"x\",\"range\":{\"min\":-1e15,\"max\":1e15},"
                     "\"params\":[{\"key\":\"p\",\"min\":-1e9,\"max\":1e9,\"default\":3}]%s%s,\"value\":\"%s\"}",
             extra[0] ? "," : "", extra, expr);
}

static void test_hostile(void) {
    static char j[SD_JSON_MAX + 4096], tmp[SD_JSON_MAX];
    size_t n;
    /* too large */
    memset(j, ' ', SD_JSON_MAX + 1);
    j[0] = '{'; j[SD_JSON_MAX] = '}';
    CHECK(try_def(j, SD_JSON_MAX + 1) == SD_E_LIMIT, "33 KB refused");
    /* nesting */
    for (n = 0; n < 20000; n++) j[n] = '[';
    CHECK(try_def(j, n) == SD_E_JSON, "deep JSON refused");
    /* no document, junk, NULs */
    CHECK(try_def("", 0) == SD_E_JSON && try_def("null", 4) != SD_OK && try_def("{\"a\":1}\0", 8) == SD_E_JSON, "junk refused");
    CHECK(try_def("{\"format\":1,\"kind\":\"i2c\",\"name\":\"\\u0000\"}", 39) != SD_OK, "a NUL in a name");
    /* far too many registers */
    n = 0;
    n = cat(j, sizeof(j), n, "{\"format\":1,\"kind\":\"i2c\",\"name\":\"r\",\"i2c\":{\"address\":16},\"registers\":[");
    for (int i = 0; i < 300; i++) { char e[48]; snprintf(e, sizeof(e), "%s{\"at\":%d,\"u8\":\"%d\"}", i ? "," : "", i % 256, i); n = cat(j, sizeof(j), n, e); }
    n = cat(j, sizeof(j), n, "]}");
    CHECK(try_def(j, n) == SD_E_LIMIT, "300 registers refused");
    /* the command map's worst case: 255 one-byte commands, each in its own high byte (every
       base byte taken), then 256 that cannot all fit */
    for (int lim = 255; lim <= 256; lim++) {
        n = 0;
        n = cat(j, sizeof(j), n, "{\"format\":1,\"kind\":\"i2c\",\"name\":\"m\",\"i2c\":{\"address\":16,\"register_address_bits\":16},\"commands\":[");
        for (int i = 0; i < lim && i < 255; i++) { char e[64]; snprintf(e, sizeof(e), "%s{\"command\":%d,\"answer\":[%d]}", i ? "," : "", i * 256 + 7, i); n = cat(j, sizeof(j), n, e); }
        if (lim == 256) n = cat(j, sizeof(j), n, ",{\"command\":1800,\"answer\":[1,2]}");
        n = cat(j, sizeof(j), n, "]}");
        int mrc = try_def(j, n);
        CHECK(lim == 255 ? mrc == SD_E_LIMIT || mrc == SD_OK : mrc != SD_OK, "command map, %d commands: %d", lim, mrc);
    }
    /* a program that would not fit 8 KB: many long templates */
    n = 0;
    n = cat(j, sizeof(j), n, "{\"format\":1,\"kind\":\"uart\",\"name\":\"u\",\"messages\":[");
    for (int i = 0; i < 30; i++) {
        n = cat(j, sizeof(j), n, i ? ",{\"every_ms\":1,\"text\":\"" : "{\"every_ms\":1,\"text\":\"");
        for (int k = 0; k < 60; k++) n = cat(j, sizeof(j), n, "{1+2:05d}x");
        n = cat(j, sizeof(j), n, "\"}");
    }
    n = cat(j, sizeof(j), n, "]}");
    int rc = try_def(j, n);
    CHECK(rc == SD_E_LIMIT || rc == SD_E_SPACE, "an oversized program is refused (%d)", rc);
    /* the epoch output is bounded: a message that repeats 64 times with 1000 chars */
    n = 0;
    n = cat(j, sizeof(j), n, "{\"format\":1,\"kind\":\"uart\",\"name\":\"u\",\"messages\":[{\"every_ms\":1,\"repeat\":1e9,\"text\":\"");
    for (int k = 0; k < 100; k++) n = cat(j, sizeof(j), n, "{i:064d}");
    n = cat(j, sizeof(j), n, "\"}]}");
    rc = try_def(j, n);
    CHECK(rc == SD_E_SPACE || rc == SD_OK, "huge epoch: %d", rc);
    /* strings: scratch overflow through nested fragments */
    n = 0;
    n = cat(j, sizeof(j), n, "{\"format\":1,\"kind\":\"uart\",\"name\":\"u\",\"fragments\":{\"f0\":{\"text\":\"0123456789012345678901234567890123456789\"}");
    for (int i = 1; i < 16; i++) { char e[96]; snprintf(e, sizeof(e), ",\"f%d\":{\"text\":\"{f%d()}{f%d()}{f%d()}\"}", i, i - 1, i - 1, i - 1); n = cat(j, sizeof(j), n, e); }
    n = cat(j, sizeof(j), n, "},\"messages\":[{\"every_ms\":1,\"text\":\"{f15()}\"}]}");
    rc = try_def(j, n);
    CHECK(rc == SD_E_SPACE || rc == SD_E_EVAL, "fragment blow-up refused (%d)", rc);
    /* expressions */
    static const char *const hostile[] = {
        "1 / 0", "0 / 0", "sat_s32(1 / 0)", "u32(0 / 0)", "-9223372036854775808 >> 1", "idiv(-9223372036854775808, -1)",
        "imod(-9223372036854775808, -1)", "1e308 * 1e308", "bits(p, 63, 0)", "bits(p, 0, 63)", "bits(p, 100, -5)",
        "byte(p, 9)", "bit(p, -1)", "pow(-8, 1 / 3)", "pow(0, -1)", "log(-1)", "sqrt(-1)", "asin(2)", "exp(1000)",
        "sin(1e300)", "tan(rad(90))", "[1, 2, 3][1e300]", "[1, 2, 3][-1]", "[p, p, p][0 / 0]", "1 << -1", "1 << 1e300",
        "-1 >> 1e300", "crc8(1, 2, 1e300, -1e300, 0 / 0)", "random(0 / 0)", "random(-0)", "0x", "0b2", "1e", "1e+", ".5",
        "5.", "1..2", "''", "'unterminated", "(((1)))", "p ? : 1", "p ?", "p :", "&&", "1 &&", "()", "[]", "[1][", "[1]",
        "reg[0]", "reg16be[255]", "reg16le[-1]", "solve(1, 2, 3, 4)", "solve(p, 1, 2, 3)", "min(1)", "clamp(1)",
        "hhmmss(1e300)", "ddmmyy(-1e300)", "nmea_lat(0 / 0)", "nmea_lon(1e300)", "hhmmss(-1)", "ddmmyy(-86400.5)",
        "t + utc + dt + written + value + count + fifo_level + i", "p == 'x'", "'x' < 1", "!'x'", "-'x'", "'x' ? 1 : 2",
        "1 ? 'a' : 'b'", "~1e300", "~(0/0)", "0 || 0 / 0", "(0 / 0) && 1", "1 ** 1e300", "(-1) ** 0.5", "2 ** 1024",
        "2 ** -1080", "1e-320 * 1e-10",
    };
    for (unsigned i = 0; i < sizeof(hostile) / sizeof(hostile[0]); i++) {
        analog_expr(j, sizeof(j), "", hostile[i]);
        (void)try_def(j, strlen(j));
    }
    /* functions: exponential calls hit the step limit; solve over a huge range is refused */
    n = (size_t)snprintf(tmp, sizeof(tmp), "\"functions\":{\"f0\":{\"args\":[\"x\"],\"return\":\"x + 1\"}");
    for (int i = 1; i < 16; i++)
        n += (size_t)snprintf(tmp + n, sizeof(tmp) - n, ",\"f%d\":{\"args\":[\"x\"],\"return\":\"f%d(x) + f%d(x) + solve(f%d, x, 0, 1e6)\"}", i, i - 1, i - 1, i - 1);
    snprintf(tmp + n, sizeof(tmp) - n, "}");
    analog_expr(j, sizeof(j), tmp, "f15(p)");
    CHECK(try_def(j, strlen(j)) == SD_E_EVAL, "step limit");
    analog_expr(j, sizeof(j), "\"functions\":{\"f\":{\"args\":[\"x\"],\"return\":\"x\"}}", "solve(f, 1, -1e300, 1e300)");
    CHECK(try_def(j, strlen(j)) == SD_E_SOLVE, "solve range too large");
    analog_expr(j, sizeof(j), "\"functions\":{\"f\":{\"args\":[\"x\"],\"return\":\"x\"}}", "solve(f, 1, -2e18, 2e18)");
    CHECK(try_def(j, strlen(j)) == SD_OK, "solve over 2^62 is fine (at most 64 steps)");
    analog_expr(j, sizeof(j), "\"functions\":{\"f\":{\"args\":[\"x\"],\"return\":\"0 / 0\"}}", "solve(f, 1, 0, 1e6)");
    (void)try_def(j, strlen(j));
    /* random expressions from a token soup */
    static const char *const toks[] = { "p", "1", "0", "-", "+", "*", "/", "%", "**", "<<", ">>", "&", "|", "^", "==",
        "!=", "<", ">=", "&&", "||", "!", "~", "?", ":", "(", ")", "[", "]", ",", "'s'", "min(", "crc8(", "sat_u8(",
        "bits(", "nmea_lat(", "reg[", "t", "1e300", "0x7FFFFFFFFFFFFFFF", "_", "solve(" };
    for (int it = 0; it < 20000; it++) {
        char e[256];
        size_t w = 0;
        int k = 1 + (int)(rnd() % 24);
        for (int q = 0; q < k; q++) { const char *t = toks[rnd() % (sizeof(toks) / sizeof(toks[0]))]; size_t l = strlen(t); if (w + l + 2 < sizeof(e)) { memcpy(e + w, t, l); w += l; e[w++] = ' '; } }
        e[w] = 0;
        analog_expr(j, sizeof(j), "", e);
        (void)try_def(j, strlen(j));
    }
    /* random templates */
    static const char *const tt[] = { "{", "}", "{{", "}}", "{p}", "{p:05.2f}", "{p:x}", "{'a':d}", "{p ? 'a' : 'b'}",
        "{p:}", "{:d}", "{p:99d}", "{p:.20f}", "{p:-+0d}", "{nmea_lon(p)}", "{hhmmss(utc)}", "text", ",", "{i}", "{count:X}" };
    for (int it = 0; it < 5000; it++) {
        char e[512];
        size_t w = 0;
        int k = 1 + (int)(rnd() % 12);
        for (int q = 0; q < k; q++) { const char *t = tt[rnd() % (sizeof(tt) / sizeof(tt[0]))]; size_t l = strlen(t); if (w + l + 1 < sizeof(e)) { memcpy(e + w, t, l); w += l; } }
        e[w] = 0;
        snprintf(j, sizeof(j), "{\"format\":1,\"kind\":\"uart\",\"name\":\"u\",\"params\":[{\"key\":\"p\",\"min\":-1e300,\"max\":1e300,\"default\":-12.5},{\"key\":\"utc_start\",\"default\":0}],"
                               "\"messages\":[{\"every_ms\":\"p\",\"repeat\":\"p\",\"enabled\":1,\"text\":\"%s\",\"end\":\"nmea\"}],"
                               "\"requests\":[{\"match_line\":\"A\",\"text\":\"%s\"},{\"match\":[\"*\"],\"answer\":[\"p\",\"1e300\",\"-1\"]}]}", e, e);
        (void)try_def(j, strlen(j));
    }
}

static void test_stim_fuzz(void) {
    const char *base =
        "{\"format\":1,\"kind\":\"stimulus\",\"name\":\"s\",\"rate_hz\":50,\"repeat\":true,\"signals\":{"
        "\"a\":[{\"shape\":\"hold\",\"value\":22,\"duration_s\":5},{\"shape\":\"ramp\",\"end\":35,\"duration_s\":30},"
        "{\"shape\":\"sine\",\"offset\":35,\"amplitude\":1.5,\"period_s\":10,\"phase_deg\":90,\"duration_s\":40},"
        "{\"shape\":\"noise\",\"sigma\":0.2,\"duration_s\":20,\"seed\":7},{\"shape\":\"step\",\"value\":25}],"
        "\"e.b\":[{\"shape\":\"data\",\"points\":[[0,1],[10,2],[60,3]],\"interpolate\":\"step\"}],"
        "\"c\":[{\"shape\":\"repeat\",\"count\":3,\"segments\":[{\"shape\":\"saw\",\"low\":0,\"high\":1,\"period_s\":0.5,\"duration_s\":1},"
        "{\"shape\":\"pulse\",\"low\":0,\"high\":1,\"period_s\":0.1,\"duty\":0.5,\"duration_s\":1}]}]},"
        "\"faults\":[{\"at_s\":45,\"duration_s\":5,\"type\":\"stuck\"},{\"at_s\":1,\"type\":\"drop\",\"value\":0.5}]}";
    size_t bn = strlen(base), len;
    static char b[8192];
    sd_err_t e;
    CHECK(sd_stim_compile(base, bn, g_stim, sizeof(g_stim), &len, &e) == SD_OK, "base stimulus: %s", e.msg);
    unsigned ok = 0;
    for (int it = 0; it < 20000; it++) {
        size_t m = bn;
        memcpy(b, base, bn);
        if (it < (int)bn) m = (size_t)it;   /* every truncation first */
        else for (int q = 0, k = 1 + (int)(rnd() % 3); q < k; q++) mutate(b, &m, sizeof(b));
        if (sd_stim_compile(b, m, g_stim, sizeof(g_stim), &len, &e) != SD_OK) continue;
        ok++;
        sd_stim_t s;
        CHECK(sd_stim_init(&s, g_stim, len) == SD_OK && len <= SD_STIM_MAX, "a compiled stimulus starts");
        sd_stim_event_t ev[64];
        static const double ts[] = { -1, 0, 1e-9, 4.999999, 5, 35, 1e6, 1e15, 1e300 };
        for (unsigned q = 0; q < sizeof(ts) / sizeof(ts[0]); q++) {
            int k = sd_stim_step(&s, ts[q], ev, (rnd() & 1) ? 64 : rnd() % 4);
            CHECK(k >= 0 && k <= 64, "events");
        }
        for (unsigned q = 0; q < sd_stim_n_faults(&s); q++) { sd_fault_t f; (void)sd_stim_fault(&s, q, &f); }
    }
    /* far too many segments, points, signals */
    static char j[65536];
    size_t n = 0;
    n = cat(j, sizeof(j), n, "{\"format\":1,\"kind\":\"stimulus\",\"signals\":{\"a\":[");
    for (int i = 0; i < 300; i++) n = cat(j, sizeof(j), n, i ? ",{\"shape\":\"step\",\"value\":1}" : "{\"shape\":\"step\",\"value\":1}");
    n = cat(j, sizeof(j), n, "]}}");
    CHECK(sd_stim_compile(j, n, g_stim, sizeof(g_stim), &len, &e) == SD_E_LIMIT, "300 segments refused");
    n = 0;
    n = cat(j, sizeof(j), n, "{\"format\":1,\"kind\":\"stimulus\",\"signals\":{\"a\":[{\"shape\":\"data\",\"points\":[");
    for (int i = 0; i < 1000; i++) { char p[32]; snprintf(p, sizeof(p), "%s[%d,%d]", i ? "," : "", i, i); n = cat(j, sizeof(j), n, p); }
    n = cat(j, sizeof(j), n, "]}]}}");
    CHECK(sd_stim_compile(j, n, g_stim, sizeof(g_stim), &len, &e) == SD_E_LIMIT, "16 KB of points do not fit 8 KB");
    printf("  stimulus fuzz: %u of 20000 mutated stimuli compiled and played\n", ok);
}

int main(void) {
    test_fixtures();
    test_hostile();
    test_stim_fuzz();
    printf("  definition fuzz: %u of %u inputs compiled and ran\n", g_compiled, g_tried);
    if (fails) { printf("test_sensor_def_fuzz: %d FAILED\n", fails); return 1; }
    printf("test_sensor_def_fuzz: all passed\n");
    return 0;
}
