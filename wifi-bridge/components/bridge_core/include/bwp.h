// BenchPod Wi-Fi bridge control protocol (BWP): frame encode/decode.
//
// Wire format: docs/PROTOCOL.md. Plain C, no ESP-IDF dependencies, so the same code runs in the
// host unit tests (test/) and could be lifted into the pod firmware.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BWP_ETHERTYPE   0x88B5
#define BWP_MAGIC0      0x42  // 'B'
#define BWP_MAGIC1      0x57  // 'W'
#define BWP_VERSION     1
#define BWP_ETH_HDR_LEN 14
#define BWP_HDR_LEN     8
#define BWP_MIN_FRAME   60    // Ethernet minimum without FCS; shorter frames are zero-padded
#define BWP_MAX_FRAME   1514  // 14 + 1500
#define BWP_MAX_BODY    (BWP_MAX_FRAME - BWP_ETH_HDR_LEN - BWP_HDR_LEN)
#define BWP_OTA_CHUNK_MAX 1024
#define BWP_SCAN_MAX_APS  20
#define BWP_RESPONSE    0x80  // response type = request type | BWP_RESPONSE

// Message types
enum {
    BWP_HELLO = 0x01,
    BWP_STATUS = 0x02,
    BWP_SCAN = 0x03,
    BWP_SET_WIFI = 0x04,
    BWP_CLEAR_WIFI = 0x05,
    BWP_OTA_BEGIN = 0x06,
    BWP_OTA_DATA = 0x07,
    BWP_OTA_END = 0x08,
    BWP_REBOOT = 0x09,
    BWP_FACTORY_RESET = 0x0A,
    BWP_OTA_ABORT = 0x0B,
    BWP_PAIR = 0x0C,
};

// TLV tags
enum {
    BWP_T_RESULT = 0x01,       // u8, bwp_result_t
    BWP_T_MODEL = 0x02,        // string
    BWP_T_FW_VERSION = 0x03,   // string
    BWP_T_HW_REV = 0x04,       // string
    BWP_T_STATE = 0x05,        // u8, bwp_state_t
    BWP_T_BRIDGE_MAC = 0x06,   // 6 bytes
    BWP_T_POD_MAC = 0x07,      // 6 bytes
    BWP_T_SSID = 0x08,         // 1..32 bytes
    BWP_T_PASSPHRASE = 0x09,   // cleartext passphrase: refused on the wire, use ENC_PASSPHRASE
    BWP_T_CHANNEL = 0x0A,      // u8
    BWP_T_RSSI = 0x0B,         // i8, dBm
    BWP_T_BSSID = 0x0C,        // 6 bytes
    BWP_T_AUTHMODE = 0x0D,     // u8, bwp_auth_t
    BWP_T_AP = 0x0E,           // nested TLVs: SSID, RSSI, CHANNEL, AUTHMODE, BSSID
    BWP_T_IMAGE_SIZE = 0x0F,   // u32
    BWP_T_OFFSET = 0x10,       // u32
    BWP_T_DATA = 0x11,         // bytes
    BWP_T_UPTIME = 0x12,       // u32, seconds
    BWP_T_REASON = 0x13,       // u8, last Wi-Fi disconnect reason (802.11 / ESP-IDF code)
    BWP_T_COUNTERS = 0x14,     // 7 x u32, see bwp_counters_t
    BWP_T_MESSAGE = 0x15,      // string, human-readable detail for an error
    BWP_T_FLAGS = 0x16,        // u8, BWP_F_*
    BWP_T_NONCE = 0x17,        // 8 bytes, random per boot (HELLO); part of every AUTH
    BWP_T_PUBKEY = 0x18,       // 32 bytes, X25519 public key: the pod's in PAIR, the bridge's in
                               // the PAIR answer and in HELLO while paired
    BWP_T_COUNTER = 0x19,      // u32, request counter, strictly increasing per nonce
    BWP_T_AUTH = 0x1A,         // 16 bytes, truncated HMAC-SHA256; must be the last TLV
    BWP_T_ENC_PASSPHRASE = 0x1B,  // 89 bytes, sealed passphrase (bwp_crypto.h)
    BWP_T_SIGNATURE = 0x1C,    // 128 bytes, the pod's signed manifest of an OTA image (fwsig.h)
};

#define BWP_KEY_LEN   32
#define BWP_NONCE_LEN 8
#define BWP_AUTH_LEN  16

enum {
    BWP_F_CONFIGURED = 0x01,   // Wi-Fi credentials stored
    BWP_F_ETH_LINK = 0x02,     // Ethernet link up
    BWP_F_OTA_ACTIVE = 0x04,   // an OTA transfer is in progress
    BWP_F_POD_KNOWN = 0x08,    // pod MAC learned
    BWP_F_PENDING_VERIFY = 0x10,  // running a new OTA image that the pod has not confirmed yet
    BWP_F_PAIRED = 0x20,       // a pod's key is stored: requests must carry COUNTER + AUTH
};

typedef enum {
    BWP_OK = 0,
    BWP_E_BAD_REQUEST = 1,
    BWP_E_UNSUPPORTED = 2,
    BWP_E_BUSY = 3,
    BWP_E_STORAGE = 4,
    BWP_E_OTA_NOT_STARTED = 5,
    BWP_E_OTA_BAD_OFFSET = 6,
    BWP_E_OTA_VERIFY = 7,
    BWP_E_OTA_TOO_LARGE = 8,
    BWP_E_INTERNAL = 9,
    BWP_E_OTA_WRONG_IMAGE = 10,
    BWP_E_NOT_PAIRED = 11,      // the request needs a paired bridge: send PAIR first
    BWP_E_AUTH = 12,            // missing or wrong AUTH, or a replayed COUNTER (response has NONCE)
    BWP_E_ALREADY_PAIRED = 13,  // PAIR without valid AUTH while paired: press PAIR on the bridge
} bwp_result_t;

typedef enum {
    BWP_ST_UNCONFIGURED = 0,   // no Wi-Fi credentials stored
    BWP_ST_WAIT_POD = 1,       // credentials stored, pod MAC not learned yet
    BWP_ST_JOINING = 2,
    BWP_ST_CONNECTED = 3,
    BWP_ST_AUTH_FAILED = 4,    // wrong passphrase (keeps retrying with backoff)
    BWP_ST_NO_AP = 5,          // SSID not found (keeps retrying with backoff)
} bwp_state_t;

typedef enum {
    BWP_AUTH_OPEN = 0,
    BWP_AUTH_WEP = 1,
    BWP_AUTH_WPA_PSK = 2,
    BWP_AUTH_WPA2_PSK = 3,
    BWP_AUTH_WPA_WPA2_PSK = 4,
    BWP_AUTH_WPA2_ENTERPRISE = 5,
    BWP_AUTH_WPA3_PSK = 6,
    BWP_AUTH_WPA2_WPA3_PSK = 7,
    BWP_AUTH_OTHER = 255,
} bwp_auth_t;

typedef struct {
    uint32_t eth_rx;        // frames received on Ethernet (all)
    uint32_t wifi_tx;       // frames forwarded Ethernet -> Wi-Fi
    uint32_t wifi_tx_err;   // Wi-Fi transmit failures (queue full, not connected)
    uint32_t wifi_rx;       // frames received on Wi-Fi (all)
    uint32_t eth_tx;        // frames forwarded Wi-Fi -> Ethernet
    uint32_t eth_tx_err;    // Ethernet transmit failures
    uint32_t dropped;       // frames dropped by the forwarding rules
} bwp_counters_t;

// ---- Writer: builds one Ethernet frame carrying one BWP message.

typedef struct {
    uint8_t *buf;
    size_t cap;
    size_t len;
    size_t nest_at;  // offset of an open nested TLV's length field, 0 if none
    bool overflow;
} bwp_writer_t;

void bwp_begin(bwp_writer_t *w, uint8_t *buf, size_t cap, const uint8_t dst[6], const uint8_t src[6],
               uint8_t type, uint16_t seq);
void bwp_put(bwp_writer_t *w, uint8_t tag, const void *val, size_t len);
void bwp_put_u8(bwp_writer_t *w, uint8_t tag, uint8_t v);
void bwp_put_u32(bwp_writer_t *w, uint8_t tag, uint32_t v);
void bwp_put_str(bwp_writer_t *w, uint8_t tag, const char *s);
void bwp_nest_begin(bwp_writer_t *w, uint8_t tag);
void bwp_nest_end(bwp_writer_t *w);
// Patches the body length, zero-pads to 60 bytes. Returns the frame length, 0 on overflow.
size_t bwp_finish(bwp_writer_t *w);
// Appends COUNTER and AUTH = HMAC-SHA256(key, nonce || frame[14 .. start of AUTH TLV])[0..16].
// Call last, right before bwp_finish.
void bwp_sign(bwp_writer_t *w, const uint8_t key[BWP_KEY_LEN], const uint8_t nonce[BWP_NONCE_LEN],
              uint32_t counter);
// Signs an already finished frame (e.g. from bwp_build_status) in place. Returns the new length,
// 0 if it does not fit.
size_t bwp_sign_frame(uint8_t *buf, size_t cap, const uint8_t key[BWP_KEY_LEN],
                      const uint8_t nonce[BWP_NONCE_LEN], uint32_t counter);

// ---- Reader

typedef struct {
    const uint8_t *dst;
    const uint8_t *src;
    uint8_t version;
    uint8_t type;
    uint16_t seq;
    const uint8_t *body;
    uint16_t body_len;
} bwp_msg_t;

typedef enum {
    BWP_PARSE_OK = 0,
    BWP_PARSE_NOT_BWP,      // wrong EtherType or magic: not ours, ignore silently
    BWP_PARSE_BAD_VERSION,  // ours, but a protocol version we do not speak
    BWP_PARSE_MALFORMED,    // truncated header, body length or TLV chain
} bwp_parse_t;

// Validates the Ethernet + BWP headers and the whole TLV chain of the body.
bwp_parse_t bwp_parse(const uint8_t *frame, size_t len, bwp_msg_t *out);
// True if the TLV chain in body is well formed (every TLV fits).
bool bwp_tlv_valid(const uint8_t *body, size_t len);

typedef struct {
    const uint8_t *p;
    size_t left;
} bwp_iter_t;

void bwp_iter_init(bwp_iter_t *it, const uint8_t *body, size_t len);
// Next TLV; false at the end or on a truncated TLV.
bool bwp_iter_next(bwp_iter_t *it, uint8_t *tag, const uint8_t **val, uint16_t *len);
// First TLV with the tag.
bool bwp_find(const uint8_t *body, size_t len, uint8_t tag, const uint8_t **val, uint16_t *vlen);
bool bwp_get_u8(const uint8_t *body, size_t len, uint8_t tag, uint8_t *out);
bool bwp_get_u32(const uint8_t *body, size_t len, uint8_t tag, uint32_t *out);
// Copies a string TLV into out (NUL-terminated). False if missing or longer than cap - 1.
bool bwp_get_str(const uint8_t *body, size_t len, uint8_t tag, char *out, size_t cap);

typedef enum {
    BWP_VERIFY_OK = 0,
    BWP_VERIFY_MISSING,  // no AUTH or no COUNTER
    BWP_VERIFY_BAD,      // wrong length, not the last TLV, or wrong MAC
} bwp_verify_t;

// Checks a parsed frame's AUTH (see bwp_sign) and returns its COUNTER.
bwp_verify_t bwp_verify(const uint8_t *frame, const bwp_msg_t *m, const uint8_t key[BWP_KEY_LEN],
                        const uint8_t nonce[BWP_NONCE_LEN], uint32_t *counter);

// ---- Messages

#define BWP_MODEL "benchpod-wifi-bridge"

typedef struct {
    const char *fw_version;
    const char *hw_rev;
    uint8_t state;          // bwp_state_t
    uint8_t flags;          // BWP_F_*
    uint8_t bridge_mac[6];
    uint8_t pod_mac[6];     // sent when flags has BWP_F_POD_KNOWN
    uint8_t nonce[BWP_NONCE_LEN];
    uint8_t bridge_pub[32]; // sent when flags has BWP_F_PAIRED: the pod recomputes the key from it
} bwp_hello_t;

typedef struct {
    uint8_t state;
    uint8_t flags;
    char ssid[33];          // empty: not configured
    uint8_t channel;        // 0: not connected
    int8_t rssi;
    uint8_t bssid[6];
    uint8_t reason;         // last disconnect reason, 0: none
    uint32_t uptime_s;
    uint8_t pod_mac[6];
    bwp_counters_t counters;
} bwp_status_t;

typedef struct {
    char ssid[33];
    int8_t rssi;
    uint8_t channel;
    uint8_t authmode;       // bwp_auth_t
    uint8_t bssid[6];
} bwp_ap_t;

size_t bwp_build_hello(uint8_t *buf, size_t cap, const uint8_t dst[6], const bwp_hello_t *h);
size_t bwp_build_status(uint8_t *buf, size_t cap, const uint8_t dst[6], const uint8_t src[6], uint16_t seq,
                        const bwp_status_t *s);
size_t bwp_build_scan_result(uint8_t *buf, size_t cap, const uint8_t dst[6], const uint8_t src[6], uint16_t seq,
                             uint8_t result, const bwp_ap_t *aps, size_t n);
// Generic response: RESULT plus an optional MESSAGE (msg may be NULL).
size_t bwp_build_result(uint8_t *buf, size_t cap, const uint8_t dst[6], const uint8_t src[6], uint8_t type,
                        uint16_t seq, uint8_t result, const char *msg);

typedef struct {
    char ssid[33];
    size_t ssid_len;
    char passphrase[65];
} bwp_set_wifi_t;

// Decodes and validates SET_WIFI. The passphrase must come sealed (ENC_PASSPHRASE) under the
// pairing key; a cleartext PASSPHRASE TLV is refused. No passphrase at all: an open network.
// Returns BWP_OK or BWP_E_BAD_REQUEST.
bwp_result_t bwp_parse_set_wifi(const bwp_msg_t *m, const uint8_t key[BWP_KEY_LEN], bwp_set_wifi_t *out);

// ---- Shared validation (also used by the UART console)

bool bwp_ssid_valid(const uint8_t *ssid, size_t len);
// Empty (open network), 8..63 printable ASCII, or exactly 64 hex digits (raw PSK).
bool bwp_passphrase_valid(const char *pass, size_t len);

#ifdef __cplusplus
}
#endif
