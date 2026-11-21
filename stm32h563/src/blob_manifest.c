/*
 * blob_manifest.c — the blobs this firmware build goes with, generated at build time by
 * tools/blob_manifest.py (see blob_store.h).
 */
#include "blob_store.h"
#include "blob_manifest_gen.h"
#include "xip_image.h"

#include <string.h>

/* gw0, gw1, esp, gw2: the order tools/blob_manifest.py writes them in. */
static const blob_manifest_t k_manifest[BLOB_COUNT + 1] = BLOB_MANIFEST_INIT;

const blob_manifest_t *blob_manifest(blob_id_t id)
{
    if (id < BLOB_COUNT) return &k_manifest[id];
    if (id == BLOB_XIP) {
        /* The W25Q image this firmware was linked with (patched in after the link). */
        static blob_manifest_t xip;
        const xip_hash_t *h = xip_image_expected();
        xip.len = h->magic == XIP_HASH_MAGIC ? h->len : 0u;
        xip.version = 0u;
        memcpy(xip.sha256, h->sha256, 32);
        return &xip;
    }
    return id == BLOB_GW2 ? &k_manifest[BLOB_COUNT] : NULL;
}

uint32_t blob_manifest_gw_version(void) { return BLOB_MANIFEST_GW_VERSION; }
