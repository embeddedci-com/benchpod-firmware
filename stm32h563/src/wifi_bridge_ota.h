#ifndef WIFI_BRIDGE_OTA_H
#define WIFI_BRIDGE_OTA_H
/*
 * wifi_bridge_ota — keeps the Wi-Fi bridge's firmware up to date from the W25Q slot "bridge".
 *
 * When the slot holds a newer image than the bridge runs (the signed manifest's version against
 * HELLO's FW_VERSION), the pod relays it on its own: OTA_BEGIN with the manifest, 1 KB OTA_DATA
 * chunks in order (resuming at the bridge's offset after a loss), OTA_END, REBOOT, then waits for
 * HELLO to show the new version. The bridge checks the manifest itself and, after the reboot,
 * runs the image on trial until the pod's next request confirms it. A version that did not come
 * back (refused, failed or rolled back) is not tried again until the pod reboots or `bridge
 * update` forces it.
 *
 * One step per wbo_poll call, never a wait: it runs from the hw worker's loop like the ESP32-C3
 * flasher. Plain C with callbacks, host-tested in test/test_wifi_bridge.c. Built only with
 * BENCHPOD_WIFI_BRIDGE.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "wifi_bridge.h"

#define WBO_CHUNK          1024u
#define WBO_CHECK_EVERY_MS 5000u
#define WBO_COME_BACK_MS   180000u   /* the bridge reboots, maybe runs a trial and rolls back */
#define WBO_MANIFEST_LEN   128u

typedef struct {
    /* Start a request with a borrowed body (wb_request_ext). 0 = sent, -1 = slot busy (try again
       on the next poll), other = cannot. */
    int (*start)(void *ctx, uint8_t type, const uint8_t *tlvs, size_t len, uint32_t timeout_ms);
    /* The request's state; when DONE or TIMEOUT, copies the reply and frees the slot. */
    wb_req_state_t (*take)(void *ctx, wb_reply_t *out);
    /* Image bytes from the slot. 0 = ok. */
    int (*read)(void *ctx, uint32_t off, uint8_t *buf, uint32_t n);
    void *(*alloc)(size_t n);
    void (*release)(void *p);
    void *ctx;
} wbo_io_t;

/* What the pod's slot holds. */
typedef struct {
    bool present;            /* an image with a manifest for target "bridge" of its size */
    uint32_t size;
    uint32_t version;        /* the manifest's, major << 16 | minor << 8 | patch */
    uint8_t manifest[WBO_MANIFEST_LEN];
} wbo_image_t;

typedef enum {
    WBO_IDLE = 0,
    WBO_BEGIN,       /* OTA_BEGIN sent: the bridge erases its slot (seconds) */
    WBO_DATA,        /* sending chunks */
    WBO_END,         /* OTA_END sent: the bridge verifies */
    WBO_REBOOT,      /* REBOOT sent */
    WBO_WAIT_BACK,   /* waiting for HELLO with the new version */
} wbo_state_t;

typedef struct {
    wbo_io_t io;
    wbo_state_t state;
    uint32_t size, off, want, failed, next_check, deadline;
    bool forced, resend;
    uint8_t *buf;            /* allocated while an update runs: the request body */
    wb_reply_t *reply;       /* only read in the step that took it: may be shared (wbo_init) */
    char status[80];
} wbo_t;

void wbo_init(wbo_t *o, const wbo_io_t *io, wb_reply_t *reply);
/* One step. `bus_free`: nothing else holds the bus the image is read over (start only then). */
void wbo_poll(wbo_t *o, const wb_info_t *bridge, const wbo_image_t *img, bool bus_free, uint32_t now);
/* An update is moving data: the image bus is taken (block other bus users). */
bool wbo_busy(const wbo_t *o);
/* `bridge update`: try now, even the same or an older version, or one that failed. */
void wbo_force(wbo_t *o);
/* "idle", "updating to 0.2.0: 45%", "updated to 0.2.0", "failed: ...", ... */
const char *wbo_status(const wbo_t *o);
/* "X.Y.Z" -> X << 16 | Y << 8 | Z, 0 if it is not one. */
uint32_t wbo_pack_version(const char *s);
void wbo_version_str(uint32_t v, char out[16]);

#endif /* WIFI_BRIDGE_OTA_H */
