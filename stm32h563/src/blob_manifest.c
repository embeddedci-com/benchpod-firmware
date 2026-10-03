/*
 * blob_manifest.c — the blobs this firmware build goes with, generated at build time by
 * tools/blob_manifest.py (see blob_store.h).
 */
#include "blob_store.h"
#include "blob_manifest_gen.h"

static const blob_manifest_t k_manifest[BLOB_COUNT] = BLOB_MANIFEST_INIT;

const blob_manifest_t *blob_manifest(blob_id_t id)
{
    return id < BLOB_COUNT ? &k_manifest[id] : NULL;
}

uint32_t blob_manifest_gw_version(void) { return BLOB_MANIFEST_GW_VERSION; }
