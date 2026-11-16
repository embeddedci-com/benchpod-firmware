/*
 * c3_image.h — which esp-hosted image the ESP32-C3 runs.
 *
 * Firmware up to 3.10.x kept a copy of the C3's image in the W25Q slot "esp" (4 MB) on every pod
 * and reported that slot in blob_status. The pod no longer keeps the image: an esp update is
 * staged in PSRAM and written straight into the C3 (c3_update.c), and this record says what the
 * C3 then holds, so blob_status can still tell an installer whether to send a new one.
 *
 * The record sits in the last 4 KB sector of the old esp slot (C3_IMAGE_REC_ADDR); an image in
 * that slot (about 1.1 MB) never reaches it. A copy left in the slot by older firmware stays
 * readable: until the first update writes a record, what the slot holds is what the C3 was
 * flashed from (the pod's own automatic reflash and flash-esp32 used it), so it is reported, and
 * a C3 that does not start is still reinstalled from it.
 *
 * Talks to the flash only through w25q.h (a RAM model in the host tests). Every function that
 * touches the flash needs the bus held (w25q_open or w25q_session_open).
 */
#ifndef C3_IMAGE_H
#define C3_IMAGE_H

#include <stdbool.h>
#include <stdint.h>
#include "blob_store.h"

#define C3_IMAGE_REC_ADDR   0x5FF000u   /* last sector of the retired esp slot (0x200000-0x600000) */
#define C3_IMAGE_REC_MAGIC  0x33435042u /* "BPC3" */
#define C3_IMAGE_REC_VER    1u

/* Read the record (boot, after blob_store_load). Bus held. */
void c3_image_load(void);

/* What the C3 holds, as blob_status reports it for "esp": the record, else the legacy slot's
   header (older firmware flashed the C3 from it), else nothing (present = false). Also nothing
   while c3_image_set_missing(true): the C3 did not start and the pod has no copy to reinstall. */
const blob_info_t *c3_image_info(void);

/* The C3's NVS partition (where esp-hosted firmware up to pod firmware 3.7.0 kept Wi-Fi
   credentials), as recorded when the image was written. false when not recorded. */
bool c3_image_nvs(uint32_t *off, uint32_t *size);

/* The C3 now runs this image. Bus held. 0 = recorded. nvs_size 0 = NVS location unknown. */
int  c3_image_save(uint32_t len, uint32_t version, const uint8_t sha256[32],
                   uint32_t nvs_off, uint32_t nvs_size);

/* The C3 holds no usable image (a write into it failed). Bus held. 0 = recorded. */
int  c3_image_forget(void);

/* In memory only: the C3 did not start and nothing on the pod can reinstall it, so report "esp"
   as missing and the next installer sends it. Cleared by a successful write (c3_image_save). */
void c3_image_set_missing(bool missing);

/* For listings (blob_status, the console): esp from c3_image_info, the other slots from the
   blob store. */
const blob_info_t *blob_report_info(blob_id_t id);
blob_state_t       blob_report_state(blob_id_t id);

#endif /* C3_IMAGE_H */
