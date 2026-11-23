#ifndef MOCK_OTA_PSRAM_H
#define MOCK_OTA_PSRAM_H

#include <stdint.h>

/* Large enough for the OTA images the tests stage (the real part is 8 MB). */
#define MOCK_PSRAM_BYTES (256u * 1024u)

extern uint8_t mock_psram[MOCK_PSRAM_BYTES];
extern int mock_psram_acquired;         /* nesting depth; 0 when the bus is free */
extern int mock_psram_acquire_count;    /* total acquires, for "the bus was used" checks */
extern int mock_psram_fail_write_at;    /* make the write covering this offset fail (-1 = never) */
extern int mock_psram_fail_read_at;     /* make the read covering this offset fail (-1 = never) */
extern int mock_quiesce_calls;
extern uint32_t mock_flash_size;
extern int mock_installed[4];          /* blob_store_on_installed calls per blob id */      /* what flash_layout_size() reports (default 2 MB) */

void mock_ota_psram_reset(void);
/* The xip image the firmware was "linked with" (blob_manifest(BLOB_XIP)). */
#include "blob_store.h"
extern blob_manifest_t mock_xip_manifest;
extern blob_manifest_t mock_xip_prog_manifest;   /* the same for BLOB_XIP_PROG */

#endif /* MOCK_OTA_PSRAM_H */
