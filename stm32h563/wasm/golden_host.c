/* golden_host.c: runs the ABI calls of sd_wasm.c in a native build and prints one JSON line
 * per call (the call, its arguments and the result).  test_wasm.mjs replays every line
 * against sensor_def.wasm and checks the results are byte for byte the same.
 *
 *   ./build/golden_host ../test/fixtures/sensor_defs > build/golden.jsonl
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sd_wasm.h"
#include "sensor_def.h"

static const char *g_dir;

/* ---- JSON line output ---- */
static void put_str(const char *s, size_t n) {
    putchar('"');
    for (size_t i = 0; i < n; i++) {
        unsigned char c = (unsigned char)s[i];
        if (c == '"' || c == '\\') { putchar('\\'); putchar(c); }
        else if (c < 0x20) printf("\\u%04x", c);
        else putchar(c);
    }
    putchar('"');
}
static void put_cstr(const char *s) { put_str(s, strlen(s)); }
static void put_bytes(const uint8_t *p, size_t n) {   /* as a JSON array of numbers */
    putchar('[');
    for (size_t i = 0; i < n; i++) printf(i ? ",%u" : "%u", p[i]);
    putchar(']');
}

typedef struct { const char *fixture; const char *text; size_t len; } def_t;

static def_t fixture(const char *name) {
    static char buf[16][SD_JSON_MAX];
    static int k;
    char path[512];
    snprintf(path, sizeof(path), "%s/%s.json", g_dir, name);
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(1); }
    char *b = buf[k++ % 16];
    size_t n = fread(b, 1, SD_JSON_MAX, f);
    fclose(f);
    return (def_t){ name, b, n };
}
static def_t inline_def(const char *text) { return (def_t){ NULL, text, strlen(text) }; }

static void put_def(const def_t *d) {
    if (d->fixture) { printf("\"fixture\":"); put_cstr(d->fixture); }
    else { printf("\"text\":"); put_str(d->text, d->len); }
}
static void put_out(uint32_t n) {
    printf(",\"out\":");
    put_str(sdw_out_ptr(), n);
    printf("}\n");
}

static void c_validate(const def_t *d) {
    printf("{\"fn\":\"validate\",");
    put_def(d);
    put_out(sdw_validate(d->text, (uint32_t)d->len));
}

static void c_preview(const def_t *d, const char *vals, const uint8_t *live, double t_ms, double input) {
    printf("{\"fn\":\"preview\",");
    put_def(d);
    printf(",\"values\":");
    if (vals) put_cstr(vals); else printf("null");
    printf(",\"live\":");
    if (live) put_bytes(live, 256); else printf("null");
    printf(",\"t_ms\":%.17g,\"input\":%.17g", t_ms, input);
    put_out(sdw_preview(d->text, (uint32_t)d->len, vals, vals ? (uint32_t)strlen(vals) : 0, live, live ? 256 : 0,
                        t_ms, input));
}

static void c_answer(const def_t *d, const char *vals, const char *req, size_t req_len, double t_ms) {
    printf("{\"fn\":\"uart_answer\",");
    put_def(d);
    printf(",\"values\":");
    if (vals) put_cstr(vals); else printf("null");
    printf(",\"request\":");
    put_bytes((const uint8_t *)req, req_len);
    printf(",\"t_ms\":%.17g", t_ms);
    put_out(sdw_uart_answer(d->text, (uint32_t)d->len, vals, vals ? (uint32_t)strlen(vals) : 0,
                            (const uint8_t *)req, (uint32_t)req_len, t_ms));
}

static void c_stim_validate(const char *json) {
    printf("{\"fn\":\"stimulus_validate\",\"text\":");
    put_cstr(json);
    put_out(sdw_stimulus_validate(json, (uint32_t)strlen(json)));
}

static void c_stim_sample(const char *json, double from, double to, double step) {
    printf("{\"fn\":\"stimulus_sample\",\"text\":");
    put_cstr(json);
    printf(",\"from_s\":%.17g,\"to_s\":%.17g,\"step_s\":%.17g", from, to, step);
    put_out(sdw_stimulus_sample(json, (uint32_t)strlen(json), from, to, step));
}

/* {"key": a point inside [min, max]} for every param, from validate's output. */
static const char *mid_values(const def_t *d, double frac) {
    static char out[2048];
    sdw_validate(d->text, (uint32_t)d->len);
    const char *o = sdw_out_ptr();
    size_t n = 0;
    out[n++] = '{';
    const char *p = strstr(o, "\"params\":[");
    bool first = true;
    while (p && (p = strstr(p, "{\"key\":\"")) != NULL) {
        p += 8;
        const char *q = strchr(p, '"');
        char key[40];
        snprintf(key, sizeof(key), "%.*s", (int)(q - p), p);
        double mn = strtod(strstr(q, "\"min\":") + 6, NULL);
        double mx = strtod(strstr(q, "\"max\":") + 6, NULL);
        if (!strcmp(key, "utc_start")) mx = 1900000000;   /* keep the dates readable */
        double v = mn + (mx - mn) * frac;
        n += (size_t)snprintf(out + n, sizeof(out) - n, "%s\"%s\":%.17g", first ? "" : ",", key, v);
        first = false;
    }
    snprintf(out + n, sizeof(out) - n, "}");
    return out;
}

static const char *k_analog =
    "{\"format\":1,\"kind\":\"analog\",\"name\":\"ntc\",\"label\":\"NTC divider\",\"output\":\"voltage\",\"excitation\":\"adc\","
    "\"params\":[{\"key\":\"temperature_c\",\"unit\":\"C\",\"min\":-40,\"max\":150,\"default\":25}],"
    "\"let\":[{\"r_ntc\":\"10000 * exp(3950 * (1 / (temperature_c + 273.15) - 1 / 298.15))\"}],"
    "\"value\":\"input * r_ntc / (r_ntc + 10000)\",\"range\":{\"min\":0,\"max\":3.3}}";

static const char *k_uart_req =
    "{\"format\":1,\"kind\":\"uart\",\"name\":\"probe\",\"label\":\"AT probe\",\"uart\":{\"baud\":115200,\"format\":\"8N1\"},"
    "\"params\":[{\"key\":\"level\",\"min\":-10,\"max\":10,\"default\":-3.14159}],"
    "\"messages\":[{\"every_ms\":250,\"text\":\"$X,{level:.3f},{count}\",\"end\":\"nmea\"}],"
    "\"requests\":[{\"match_line\":\"AT+READ?\",\"text\":\"+READ: {level:.1f}\",\"end\":\"crlf\"},"
    "{\"match\":[0,\"*\",1],\"answer\":[255,134,\"byte(count, 0)\",133],\"delay_ms\":10}]}";

static const char *k_stim =
    "{\"format\":1,\"kind\":\"stimulus\",\"name\":\"hot\",\"rate_hz\":20,\"signals\":{"
    "\"temperature_c\":[{\"shape\":\"hold\",\"value\":22,\"duration_s\":5},{\"shape\":\"ramp\",\"end\":35,\"duration_s\":30},"
    "{\"shape\":\"sine\",\"offset\":35,\"amplitude\":1.5,\"period_s\":10,\"duration_s\":40},"
    "{\"shape\":\"noise\",\"mean\":35,\"sigma\":0.2,\"duration_s\":20,\"seed\":7},{\"shape\":\"step\",\"value\":25}],"
    "\"env.pressure_pa\":[{\"shape\":\"data\",\"points\":[[0,101325],[10,100900],[60,99000]],\"interpolate\":\"linear\"}],"
    "\"x\":[{\"shape\":\"repeat\",\"count\":3,\"segments\":[{\"shape\":\"ramp\",\"start\":0,\"end\":10,\"duration_s\":1},{\"shape\":\"hold\",\"value\":10,\"duration_s\":1}]},"
    "{\"shape\":\"saw\",\"low\":0,\"high\":4,\"period_s\":2,\"duration_s\":4},{\"shape\":\"pulse\",\"low\":1,\"high\":2,\"period_s\":1,\"duty\":0.25,\"duration_s\":2}]},"
    "\"faults\":[{\"at_s\":45,\"duration_s\":5,\"type\":\"stuck\"},{\"at_s\":70,\"duration_s\":2,\"type\":\"no_response\"},"
    "{\"at_s\":1,\"type\":\"corrupt\",\"at\":250,\"mask\":1,\"sensor\":\"env\"}]}";

static const char *k_stim_repeat =
    "{\"format\":1,\"kind\":\"stimulus\",\"repeat\":true,\"signals\":{\"a\":[{\"shape\":\"ramp\",\"start\":0,\"end\":10,\"duration_s\":10}]},"
    "\"faults\":[{\"at_s\":2,\"duration_s\":1,\"type\":\"stuck\"}]}";

int main(int argc, char **argv) {
    g_dir = argc > 1 ? argv[1] : "../test/fixtures/sensor_defs";
    static const char *const names[] = { "bme280", "bmp280", "sht4x", "mpu6050", "gps_nmea" };
    uint8_t live[256];
    for (unsigned i = 0; i < 256; i++) live[i] = (uint8_t)(i * 37u + 11u);
    for (unsigned i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
        def_t d = fixture(names[i]);
        c_validate(&d);
        bool uart = !strcmp(names[i], "gps_nmea");
        static const double ts[] = { 0, 1000, 1500, 10000, 60000 };
        for (unsigned t = 0; t < sizeof(ts) / sizeof(ts[0]); t++) {
            c_preview(&d, NULL, NULL, ts[t], 0);
            c_preview(&d, mid_values(&d, 0.25), NULL, ts[t], 0);
            c_preview(&d, mid_values(&d, 0.8), NULL, ts[t], 0);
        }
        if (!uart) {
            c_preview(&d, mid_values(&d, 0.5), live, 0, 0);
            c_preview(&d, mid_values(&d, 0.5), live, 2500, 0);
        } else {
            c_answer(&d, NULL, "AT\r\n", 4, 0);
        }
        c_preview(&d, "{\"no_such_param\":1}", NULL, 0, 0);
    }
    /* analog, uart requests */
    def_t a = inline_def(k_analog);
    c_validate(&a);
    static const double temps[] = { -40, 0, 25, 99.5, 150 };
    for (unsigned i = 0; i < 5; i++) {
        char v[64];
        snprintf(v, sizeof(v), "{\"temperature_c\":%.17g}", temps[i]);
        c_preview(&a, v, NULL, 0, 3.3);
        c_preview(&a, v, NULL, 0, 5.0);
    }
    c_preview(&a, "{\"temperature_c\":151}", NULL, 0, 3.3);
    def_t u = inline_def(k_uart_req);
    c_validate(&u);
    c_preview(&u, NULL, NULL, 0, 0);
    c_preview(&u, "{\"level\":7.25}", NULL, 1000, 0);
    const char req[] = { 'j', 'u', 'n', 'k', '\r', 'A', 'T', '+', 'R', 'E', 'A', 'D', '?', '\r', '\n', 0, (char)0xFF, 1 };
    c_answer(&u, NULL, req, sizeof(req), 0);
    c_answer(&u, "{\"level\":-9.99}", req, sizeof(req), 4000);
    /* broken definitions: the error and its path */
    def_t b1 = fixture("bme280");
    static char broken[SD_JSON_MAX];
    memcpy(broken, b1.text, b1.len);
    char *hit = strstr(broken, "\"registers\"");
    if (hit) { hit[1] = 'R'; }
    def_t bad1 = { NULL, broken, b1.len };
    c_validate(&bad1);
    def_t bad2 = inline_def("{\"format\":1,\"kind\":\"analog\",\"name\":\"x\",\"label\":\"x\",\"value\":\"1 +* 2\",\"range\":{\"min\":0,\"max\":1}}");
    c_validate(&bad2);
    def_t bad3 = inline_def("{not json");
    c_validate(&bad3);
    /* stimulus */
    c_stim_validate(k_stim);
    c_stim_sample(k_stim, 0, 100, 0.5);
    c_stim_sample(k_stim, 44, 52, 0.05);
    c_stim_validate(k_stim_repeat);
    c_stim_sample(k_stim_repeat, 0, 35, 0.25);
    c_stim_validate("{\"format\":1,\"kind\":\"stimulus\",\"signals\":{\"a\":[{\"shape\":\"wobble\"}]}}");
    c_stim_sample(k_stim, 0, 100000, 0.001);
    return 0;
}
