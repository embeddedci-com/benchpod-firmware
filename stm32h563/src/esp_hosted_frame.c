#include "esp_hosted_frame.h"

#include <string.h>

/* The wire layout below is byte-exact for the vendored packed struct; assert the
   size so an upstream header change is caught at compile time. */
_Static_assert(sizeof(struct esp_payload_header) == 12,
               "esp_payload_header must be 12 bytes on the wire");

/* Byte offsets within esp_payload_header (little-endian multi-byte fields):
     [0] if_num:4 | if_type:4   [1] flags        [2..3] len      [4..5] offset
     [6..7] checksum            [8..9] seq_num    [10] throttle   [11] priv/hci type */
enum {
    OFF_IFBYTE = 0, OFF_FLAGS = 1, OFF_LEN = 2, OFF_OFFSET = 4,
    OFF_CKSUM = 6, OFF_SEQ = 8, OFF_THROTTLE = 10, OFF_PRIV = 11,
};

static inline uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline void     wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

uint16_t esp_hosted_checksum(const uint8_t *frame, uint16_t len) {
    uint16_t s = 0;
    for (uint16_t i = 0; i < len; i++) s += frame[i];
    return s;
}

size_t esp_hosted_frame_build(uint8_t *out, size_t out_cap,
                              uint8_t if_type, uint8_t if_num, uint8_t flags,
                              uint8_t priv_pkt_type, uint16_t seq_num,
                              const uint8_t *payload, uint16_t len) {
    size_t total = ESP_HOSTED_HDR_LEN + (size_t)len;
    if (out_cap < total) return 0;

    memset(out, 0, ESP_HOSTED_HDR_LEN);
    out[OFF_IFBYTE] = (uint8_t)((if_num << 4) | (if_type & 0x0F));
    out[OFF_FLAGS]  = flags;
    wr16(out + OFF_LEN, len);
    wr16(out + OFF_OFFSET, (uint16_t)ESP_HOSTED_HDR_LEN);
    wr16(out + OFF_CKSUM, 0);                 /* zeroed for the checksum pass */
    wr16(out + OFF_SEQ, seq_num);
    out[OFF_THROTTLE] = 0;
    out[OFF_PRIV]     = priv_pkt_type;
    if (len && payload) memcpy(out + ESP_HOSTED_HDR_LEN, payload, len);

    wr16(out + OFF_CKSUM, esp_hosted_checksum(out, (uint16_t)total));
    return total;
}

int esp_hosted_frame_parse(const uint8_t *buf, size_t len, esp_hosted_rx_t *out) {
    if (len < ESP_HOSTED_HDR_LEN) return -1;

    uint16_t offset = rd16(buf + OFF_OFFSET);
    uint16_t plen   = rd16(buf + OFF_LEN);
    if (offset < ESP_HOSTED_HDR_LEN) return -1;

    size_t total = (size_t)offset + (size_t)plen;
    if (total > len) return -1;                 /* frame doesn't fit the received buffer */

    /* Checksum is the sum of all frame bytes with the checksum field zeroed. */
    uint16_t stored = rd16(buf + OFF_CKSUM);
    uint16_t calc   = (uint16_t)(esp_hosted_checksum(buf, (uint16_t)total)
                                 - buf[OFF_CKSUM] - buf[OFF_CKSUM + 1]);
    if (calc != stored) return -1;

    out->if_type       = (uint8_t)(buf[OFF_IFBYTE] & 0x0F);
    out->if_num        = (uint8_t)(buf[OFF_IFBYTE] >> 4);
    out->flags         = buf[OFF_FLAGS];
    out->priv_pkt_type = buf[OFF_PRIV];
    out->seq_num       = rd16(buf + OFF_SEQ);
    out->payload       = buf + offset;
    out->payload_len   = plen;
    return 0;
}
