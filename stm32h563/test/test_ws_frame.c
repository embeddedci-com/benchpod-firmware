/*
 * test_ws_frame.c — server->pod WebSocket framing (src/ws_frame.c) and the id scan used on a
 * frame too large to copy (bp_json_scan_str), FW-12.
 *
 * A fragmented message used to be taken piecewise: its first frame acted on as a whole message,
 * the continuation frames dropped. It is now refused explicitly. An oversized tunnel.data frame
 * resets its tunnel, which needs the tunnel id from the whole frame: the server's JSON puts it
 * after the base64 data.
 */
#include "ws_frame.h"
#include "bp_json.h"

#include <stdio.h>
#include <string.h>

uint64_t get_rand_64(void) { return 0x0123456789abcdefull; }

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

/* An unmasked server frame: b0 (FIN/RSV/opcode) + payload. */
static size_t server_frame(uint8_t b0, const char *payload, uint8_t *out) {
    size_t n = strlen(payload), h = 2;
    out[0] = b0;
    if (n < 126) out[1] = (uint8_t)n;
    else { out[1] = 126; out[2] = (uint8_t)(n >> 8); out[3] = (uint8_t)n; h = 4; }
    memcpy(out + h, payload, n);
    return h + n;
}

static void test_whole_frames(void) {
    uint8_t b[64];
    ws_frame_t f;
    size_t n = server_frame(0x81, "{\"type\":\"ping\"}", b);
    CHECK(ws_parse_frame(b, n, &f) == (int)n);
    CHECK(f.opcode == WS_OP_TEXT && f.fin && f.payload_len == 15);
    CHECK(ws_parse_frame(b, n - 1, &f) == 0);              /* incomplete: wait */
    n = server_frame(0x89, "", b);                          /* ping control frame */
    CHECK(ws_parse_frame(b, n, &f) == (int)n && f.opcode == WS_OP_PING);
}

static void test_fragments_refused(void) {
    uint8_t b[64];
    ws_frame_t f;
    size_t n = server_frame(0x01, "{\"type\":\"tunn", b);   /* text, FIN clear */
    CHECK(ws_parse_frame(b, n, &f) == WS_PARSE_FRAGMENTED);
    n = server_frame(0x80, "el.data\"}", b);                /* final continuation */
    CHECK(ws_parse_frame(b, n, &f) == WS_PARSE_FRAGMENTED);
    n = server_frame(0x00, "middle", b);                    /* middle continuation */
    CHECK(ws_parse_frame(b, n, &f) == WS_PARSE_FRAGMENTED);
    /* Refused from the header alone, before the payload has arrived. */
    CHECK(ws_parse_frame(b, 2, &f) == WS_PARSE_FRAGMENTED);
}

static void test_bad_frames_refused(void) {
    uint8_t b[64];
    ws_frame_t f;
    size_t n = server_frame(0xC1, "{}", b);                 /* RSV1: compressed */
    CHECK(ws_parse_frame(b, n, &f) == -1);
    n = server_frame(0x81, "{}", b);
    b[1] |= 0x80;                                           /* masked server frame */
    CHECK(ws_parse_frame(b, n, &f) == -1);
}

static void test_scan_ids_after_data(void) {
    /* What the server's encoding/json sends: keys sorted, data first. */
    static char frame[4096];
    char data[3000];
    memset(data, 'A', sizeof(data) - 1);
    data[sizeof(data) - 1] = '\0';
    int n = snprintf(frame, sizeof(frame),
                     "{\"data_b64\":\"%s\",\"tunnel_id\":\"t-42\",\"type\":\"tunnel.data\"}", data);
    char v[64];
    CHECK(bp_json_scan_str(frame, (size_t)n, "type", v, sizeof(v)) && strcmp(v, "tunnel.data") == 0);
    CHECK(bp_json_scan_str(frame, (size_t)n, "tunnel_id", v, sizeof(v)) && strcmp(v, "t-42") == 0);
    CHECK(!bp_json_scan_str(frame, (size_t)n, "request_id", v, sizeof(v)) && v[0] == '\0');
    /* Not NUL-terminated and cut inside the value: not found. */
    char *cut = strstr(frame, "t-42");
    CHECK(!bp_json_scan_str(frame, (size_t)(cut + 2 - frame), "tunnel_id", v, sizeof(v)));
    /* A value longer than the output is refused, not truncated. */
    char small[3];
    CHECK(!bp_json_scan_str(frame, (size_t)n, "type", small, sizeof(small)));
    /* Whitespace around the colon. */
    const char *sp = "{\"type\" : \"command.request\", \"request_id\":\"r1\"}";
    CHECK(bp_json_scan_str(sp, strlen(sp), "type", v, sizeof(v)) && strcmp(v, "command.request") == 0);
    CHECK(bp_json_scan_str(sp, strlen(sp), "request_id", v, sizeof(v)) && strcmp(v, "r1") == 0);
}

int main(void) {
    test_whole_frames();
    test_fragments_refused();
    test_bad_frames_refused();
    test_scan_ids_after_data();
    if (failures) { printf("test_ws_frame: %d FAILED\n", failures); return 1; }
    printf("test_ws_frame: all passed\n");
    return 0;
}
