// Small pure helpers shared by the firmware and the host tests: console line tokenizer, Wi-Fi
// disconnect-reason mapping, OTA chunk sequencing and image identification.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Splits a console line in place. Words are separated by spaces or tabs; "double quotes" group
// words, and a backslash escapes the next character (\" \\ \ ). Returns argc, or -1 for an
// unterminated quote, -2 for more than max_args words.
int bu_tokenize(char *line, char **argv, int max_args);

// Maps an ESP-IDF Wi-Fi disconnect reason to a bridge state (bwp_state_t): AUTH_FAILED for a
// wrong passphrase, NO_AP when the SSID is not found, JOINING for everything else (retry).
uint8_t bu_state_for_reason(uint16_t reason);

// OTA chunk sequencing: chunks must arrive in order. A chunk at the previous offset is a
// retransmission (the pod lost our ack) and is acknowledged again without writing.
typedef struct {
    bool active;
    uint32_t size;      // announced image size
    uint32_t next;      // next expected offset
    uint32_t last_off;  // offset of the last chunk written
    uint32_t last_len;
} bu_ota_seq_t;

typedef enum {
    BU_OTA_WRITE = 0,   // new chunk: write it
    BU_OTA_DUPLICATE,   // repeat of the last chunk: ack, do not write
    BU_OTA_BAD_OFFSET,  // gap or rewind: reply with the expected offset
    BU_OTA_TOO_LARGE,   // past the announced size
    BU_OTA_INACTIVE,
} bu_ota_chunk_t;

void bu_ota_begin(bu_ota_seq_t *s, uint32_t size);
bu_ota_chunk_t bu_ota_check(const bu_ota_seq_t *s, uint32_t offset, uint32_t len);
void bu_ota_advance(bu_ota_seq_t *s, uint32_t offset, uint32_t len);
bool bu_ota_complete(const bu_ota_seq_t *s);

// Reads the project name and version from the start of an ESP32 app image (image header,
// first segment header, esp_app_desc_t). Needs the first BU_IMG_DESC_END bytes.
#define BU_IMG_DESC_END 112
bool bu_image_ident(const uint8_t *img, size_t len, char project[33], char version[33]);

// HELLO pacing: every interval while the Ethernet link is up, and at once when it comes up. A
// HELLO carries this boot's nonce; the pod signs nothing after a link drop until it has one, so
// the first HELLO after a reboot must not wait for the timer.
typedef struct {
    bool link;       // the link as of the last call
    int64_t next_us; // when the next HELLO is due
} bu_hello_timer_t;

// True when a HELLO is due now (and schedules the next one). Never while the link is down.
bool bu_hello_due(bu_hello_timer_t *t, bool link_up, int64_t now_us, int64_t interval_us);

// Formats a MAC as aa:bb:cc:dd:ee:ff (out: 18 bytes).
void bu_mac_str(const uint8_t mac[6], char out[18]);

#ifdef __cplusplus
}
#endif
