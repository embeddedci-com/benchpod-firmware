#ifndef WIFI_BRIDGE_H
#define WIFI_BRIDGE_H
/*
 * wifi_bridge — the pod's side of the Wi-Fi bridge control channel (EtherType 0x88B5).
 *
 * BenchPod v4 has no radio; a small external box bridges the pod's Ethernet port to Wi-Fi
 * (benchpod-firmware/wifi-bridge, protocol in wifi-bridge/docs/PROTOCOL.md). This module is the
 * protocol client: it tracks the bridge's HELLO, pairs with it on first contact, signs every
 * request (HMAC-SHA256, per-bridge-boot nonce, counter), retries, and polls STATUS every 10 s.
 * The frame codec and the HMAC are the bridge's own code (wifi-bridge/components/bridge_core),
 * linked into this firmware, so both ends encode the same bytes.
 *
 * Plain C with no lwIP, HAL or RTOS: the caller passes frames in (wb_rx), calls wb_poll with the
 * time, and supplies send() and the pairing-key derivation. wifi_bridge_net.c is that glue on
 * the net task. Host-tested in test/test_wifi_bridge.c against the bridge's pairing logic.
 *
 * Built only when BENCHPOD_WIFI_BRIDGE is defined (dev builds, see the Makefile).
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bwp.h"

#define WB_HELLO_TIMEOUT_MS   6000u   /* bridge gone after three missed HELLOs */
#define WB_STATUS_EVERY_MS    10000u  /* background STATUS while paired */
#define WB_PAIR_RETRY_MS      5000u   /* auto-pair attempts at most this often */
#define WB_TRIES              3       /* sends per request (each re-signed with a new counter) */
#define WB_MSG_MAX            64
#define WB_SCAN_MAX           BWP_SCAN_MAX_APS
/* Request bodies the pod sends today: SET_WIFI is the largest (3+32 + 3+64 bytes). OTA relay
   (1024-byte chunks) needs this raised; RAM is tight on the STM32 (see the Makefile size). */
#define WB_REQ_MAX            128
#define WB_TX_MAX             (BWP_ETH_HDR_LEN + BWP_HDR_LEN + WB_REQ_MAX + 7 + 3 + BWP_AUTH_LEN)

typedef struct {
    /* Send one Ethernet frame on the pod's wired port. */
    void (*send)(const uint8_t *frame, size_t len, void *ctx);
    /* The pairing key for a bridge (deterministic per pod and bridge MAC). 0 = ok. */
    int (*derive_key)(const uint8_t bridge_mac[6], uint8_t key[BWP_KEY_LEN], void *ctx);
    void *ctx;
} wb_io_t;

typedef enum {
    WB_REQ_IDLE = 0,
    WB_REQ_PENDING,
    WB_REQ_DONE,     /* answered: see wb_reply_t.result */
    WB_REQ_TIMEOUT,  /* no (valid) answer after WB_TRIES sends */
} wb_req_state_t;

typedef struct {
    uint8_t result;              /* bwp_result_t */
    char message[WB_MSG_MAX];    /* MESSAGE from the bridge, or our own explanation */
    uint32_t offset;             /* OTA_DATA: next expected offset */
    bwp_ap_t aps[WB_SCAN_MAX];   /* SCAN */
    uint8_t n_aps;
} wb_reply_t;

/* What the pod knows about the bridge, for wifi_status / console / caps. */
typedef struct {
    bool present;                /* a HELLO within WB_HELLO_TIMEOUT_MS */
    uint8_t mac[6];
    char fw_version[16];
    char hw_rev[8];
    uint8_t state;               /* bwp_state_t, from the latest HELLO or STATUS */
    uint8_t flags;               /* BWP_F_* */
    bool paired;                 /* the bridge accepts our key (a signed request succeeded) */
    const char *problem;         /* NULL, or why pairing / requests fail */
    bool have_status;            /* fields below come from a signed STATUS */
    char ssid[33];
    uint8_t channel;
    int8_t rssi;
    uint8_t reason;
    uint32_t uptime_s;
    uint32_t status_age_ms;
    bwp_counters_t counters;
} wb_info_t;

typedef struct {
    wb_io_t io;
    uint8_t pod_mac[6];
    bool link_up;

    /* bridge */
    bool seen;
    uint32_t last_hello;
    uint8_t mac[6];
    uint8_t nonce[BWP_NONCE_LEN];
    bool have_nonce;
    uint8_t key[BWP_KEY_LEN];
    bool have_key;
    uint32_t counter;            /* last COUNTER sent with this nonce */
    uint16_t seq;
    wb_info_t info;
    uint32_t status_at;          /* when info.have_status was refreshed */
    uint32_t next_status;
    uint32_t next_pair;

    /* the one request in flight */
    wb_req_state_t req_state;
    bool req_internal;           /* our own PAIR / STATUS, not a caller's */
    uint8_t req_type;
    uint16_t req_seq;
    uint32_t req_counter;        /* counter of the latest send (to check the signed reply) */
    bool req_signed;
    uint8_t req_tlvs[WB_REQ_MAX];
    uint16_t req_len;
    uint32_t req_timeout;
    uint32_t req_sent;
    uint8_t req_tries;
    bool req_resynced;           /* already retried after a REPLAY resync */
    wb_reply_t reply;
    uint8_t tx[WB_TX_MAX < BWP_MIN_FRAME ? BWP_MIN_FRAME : WB_TX_MAX];
} wb_t;

void wb_init(wb_t *b, const uint8_t pod_mac[6], const wb_io_t *io);
void wb_set_link(wb_t *b, bool up, uint32_t now);
/* One received frame (any EtherType; non-BWP frames are ignored). */
void wb_rx(wb_t *b, const uint8_t *frame, size_t len, uint32_t now);
/* Timeouts, retries, auto-pair and the STATUS poll. Call often (every few ms is fine). */
void wb_poll(wb_t *b, uint32_t now);

/* Start a caller's request (body = TLVs, without COUNTER/AUTH). 0: started; -1: another one is in
   flight (retry shortly); -2: no bridge present; -3: not usable (not paired, see info.problem). */
int wb_request(wb_t *b, uint8_t type, const uint8_t *tlvs, size_t len, uint32_t timeout_ms, uint32_t now);
wb_req_state_t wb_request_state(const wb_t *b);
const wb_reply_t *wb_request_reply(const wb_t *b);
/* The caller has read the reply: free the slot. */
void wb_request_release(wb_t *b);

void wb_get_info(const wb_t *b, uint32_t now, wb_info_t *out);
const char *wb_state_str(uint8_t state);
const char *wb_result_str(uint8_t result);

#endif /* WIFI_BRIDGE_H */
