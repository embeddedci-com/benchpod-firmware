#ifndef ESP_HOSTED_FRAME_H
#define ESP_HOSTED_FRAME_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ---- ESP-Hosted transport framing (portable, no HAL) -----------------------
 *
 * Builds and parses the esp-hosted `esp_payload_header` that prefixes every
 * frame exchanged with the ESP32-C3 over SPI (see lib/esp_hosted/common/). The
 * STM32 host puts 802.3 frames on ESP_STA_IF and RPC control on ESP_SERIAL_IF;
 * the slave announces itself with a priv event on ESP_PRIV_IF at boot.
 *
 * This module is hardware-independent so it can be unit-tested on the host
 * (test/test_portable.c); the SPI transaction that carries these frames lives
 * in esp_hosted_spi.c.
 * ---------------------------------------------------------------------------*/

#include "esp_hosted_header.h"      /* struct esp_payload_header, flags (vendored) */
#include "esp_hosted_interface.h"   /* esp_hosted_if_type_t (vendored) */

/* Fixed 12-byte header (compile-time asserted in the .c). */
#define ESP_HOSTED_HDR_LEN   ((size_t)sizeof(struct esp_payload_header))

/* Parsed view of one received frame (payload points into the caller's buffer). */
typedef struct {
    uint8_t        if_type;       /* esp_hosted_if_type_t */
    uint8_t        if_num;
    uint8_t        flags;
    uint8_t        priv_pkt_type; /* union byte — valid for ESP_PRIV_IF */
    uint16_t       seq_num;
    const uint8_t *payload;
    uint16_t       payload_len;
} esp_hosted_rx_t;

/* esp-hosted checksum: 16-bit sum of all frame bytes with the checksum field
   zeroed (matches upstream compute_checksum()). Exposed for the unit tests. */
uint16_t esp_hosted_checksum(const uint8_t *frame, uint16_t len);

/* Build a frame into out[]: writes the header (offset=ESP_HOSTED_HDR_LEN, the
   given if_type/if_num/flags/priv_pkt_type, a caller-supplied seq_num and the
   computed checksum) followed by payload. Returns the total frame length
   (header + payload), or 0 if it does not fit in out_cap or len is too large. */
size_t esp_hosted_frame_build(uint8_t *out, size_t out_cap,
                              uint8_t if_type, uint8_t if_num, uint8_t flags,
                              uint8_t priv_pkt_type, uint16_t seq_num,
                              const uint8_t *payload, uint16_t len);

/* Parse one frame from buf[0..len). Validates the header offset, that
   offset+payload_len fits within len, and the checksum. On success fills *out
   (payload points into buf) and returns 0; returns -1 on a malformed/short/
   bad-checksum frame. */
int esp_hosted_frame_parse(const uint8_t *buf, size_t len, esp_hosted_rx_t *out);

#endif /* ESP_HOSTED_FRAME_H */
