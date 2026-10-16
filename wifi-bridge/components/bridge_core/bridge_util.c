#include "bridge_util.h"

#include <stdio.h>
#include <string.h>

#include "bwp.h"

int bu_tokenize(char *line, char **argv, int max_args)
{
    int argc = 0;
    char *r = line, *w = line;
    while (*r) {
        while (*r == ' ' || *r == '\t') {
            r++;
        }
        if (!*r) {
            break;
        }
        if (argc == max_args) {
            return -2;
        }
        argv[argc++] = w;
        bool quoted = false;
        while (*r) {
            char c = *r;
            if (c == '\\' && r[1]) {
                *w++ = r[1];
                r += 2;
            } else if (c == '"') {
                quoted = !quoted;
                r++;
            } else if (!quoted && (c == ' ' || c == '\t')) {
                r++;
                break;
            } else {
                *w++ = c;
                r++;
            }
        }
        if (quoted) {
            return -1;
        }
        *w++ = '\0';
    }
    return argc;
}

// Reason codes from ESP-IDF esp_wifi_types_generic.h (wifi_err_reason_t).
uint8_t bu_state_for_reason(uint16_t reason)
{
    switch (reason) {
    case 2:    // AUTH_EXPIRE
    case 14:   // MIC_FAILURE
    case 15:   // 4WAY_HANDSHAKE_TIMEOUT
    case 202:  // AUTH_FAIL
    case 204:  // HANDSHAKE_TIMEOUT
        return BWP_ST_AUTH_FAILED;
    case 201:  // NO_AP_FOUND
    case 210:  // NO_AP_FOUND_W_COMPATIBLE_SECURITY
    case 211:  // NO_AP_FOUND_IN_AUTHMODE_THRESHOLD
    case 212:  // NO_AP_FOUND_IN_RSSI_THRESHOLD
        return BWP_ST_NO_AP;
    default:
        return BWP_ST_JOINING;
    }
}

void bu_ota_begin(bu_ota_seq_t *s, uint32_t size)
{
    memset(s, 0, sizeof(*s));
    s->active = true;
    s->size = size;
}

bu_ota_chunk_t bu_ota_check(const bu_ota_seq_t *s, uint32_t offset, uint32_t len)
{
    if (!s->active) {
        return BU_OTA_INACTIVE;
    }
    if (s->last_len && offset == s->last_off && len == s->last_len) {
        return BU_OTA_DUPLICATE;
    }
    if (offset != s->next) {
        return BU_OTA_BAD_OFFSET;
    }
    if (len > s->size - s->next) {
        return BU_OTA_TOO_LARGE;
    }
    return BU_OTA_WRITE;
}

void bu_ota_advance(bu_ota_seq_t *s, uint32_t offset, uint32_t len)
{
    s->last_off = offset;
    s->last_len = len;
    s->next = offset + len;
}

bool bu_ota_complete(const bu_ota_seq_t *s)
{
    return s->active && s->next == s->size;
}

// esp_image_header_t (24 bytes) + esp_image_segment_header_t (8) + esp_app_desc_t:
// magic_word u32 @32, secure_version @36, reserv1[2] @40, version[32] @48, project_name[32] @80.
#define IMG_MAGIC 0xE9
#define APP_DESC_MAGIC 0xABCD5432u

bool bu_image_ident(const uint8_t *img, size_t len, char project[33], char version[33])
{
    if (len < BU_IMG_DESC_END || img[0] != IMG_MAGIC) {
        return false;
    }
    uint32_t magic = (uint32_t)img[32] | ((uint32_t)img[33] << 8) | ((uint32_t)img[34] << 16) |
                     ((uint32_t)img[35] << 24);
    if (magic != APP_DESC_MAGIC) {
        return false;
    }
    memcpy(version, img + 48, 32);
    version[32] = '\0';
    memcpy(project, img + 80, 32);
    project[32] = '\0';
    return true;
}

void bu_mac_str(const uint8_t m[6], char out[18])
{
    snprintf(out, 18, "%02x:%02x:%02x:%02x:%02x:%02x", m[0], m[1], m[2], m[3], m[4], m[5]);
}
