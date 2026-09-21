#ifndef WS_FRAME_H
#define WS_FRAME_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ---- Minimal RFC 6455 WebSocket framing ------------------------------------
 *
 * Just enough for the bench pod's cloud client: encode masked client->server
 * text/control frames, and parse (unmasked) server->client frames. Fragmentation
 * and 64-bit huge payloads are not needed — our frames are small JSON.
 * ---------------------------------------------------------------------------*/

#define WS_OP_CONT  0x0
#define WS_OP_TEXT  0x1
#define WS_OP_BIN   0x2
#define WS_OP_CLOSE 0x8
#define WS_OP_PING  0x9
#define WS_OP_PONG  0xA

typedef struct {
    uint8_t        opcode;
    bool           fin;
    const uint8_t *payload;   /* points into the caller's buffer */
    size_t         payload_len;
} ws_frame_t;

/* Build a masked client frame for `opcode` carrying `payload` into `out`.
   mask_key is 4 random bytes (client frames MUST be masked). Returns the total
   frame length, or 0 if `out_cap` is too small. Supports payloads up to 65535 B. */
size_t ws_build_frame(uint8_t opcode, const uint8_t *payload, size_t len,
                      const uint8_t mask_key[4], uint8_t *out, size_t out_cap);

/* Parse one frame from buf[0..len). On a complete frame, fills *out (payload
   points into buf) and returns the number of bytes consumed (header + payload).
   Returns 0 if the buffer does not yet hold a full frame, or -1 on a frame we
   refuse (a masked server frame, or a payload larger than 65535 B). */
int ws_parse_frame(const uint8_t *buf, size_t len, ws_frame_t *out);

/* Generate a Sec-WebSocket-Key header value: 16 random bytes, standard base64
   (24 chars + NUL). out must be >= 25 bytes. */
void ws_make_sec_key(char out[25]);

#endif /* WS_FRAME_H */
