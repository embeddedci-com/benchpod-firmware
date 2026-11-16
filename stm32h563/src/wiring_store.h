#ifndef WIRING_STORE_H
#define WIRING_STORE_H
/*
 * wiring_store: the pod's wiring profiles (wiring_doc.h) kept in two W25Q slots, WIRING-A and
 * WIRING-B (blob_store.h), and in RAM.
 *
 * Two copies so a power loss during a save keeps the previous profiles: a save writes the slot that
 * does not hold the newest copy, with the next sequence number in the slot header's version field,
 * and boot loads the present slot with the highest sequence (its data hash is checked). Until the
 * new slot's commit word lands it reads as empty, so the other one is still the newest.
 *
 * The whole document lives in RAM (heap, its actual size, at most WIRING_DOC_MAX), so reading a
 * profile never touches the bus: a running DAC replay or capture is never disturbed by a read.
 * Saving does need the bus (w25q_session_open quiesces the gateware's PSRAM masters, which stops a
 * DAC), so the caller refuses a save while the DAC runs or a capture or upload holds the bus.
 *
 * All calls on the hw worker.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "wiring_doc.h"

/* Boot, bus free (next to cloud_extras_load): read the newest valid copy into RAM. */
void wiring_store_load(void);

/* The cached index and document (doc may be NULL when there are no profiles). */
const wiring_index_t *wiring_store_index(void);
const uint8_t        *wiring_store_doc(void);

typedef enum {
    WIRING_STORE_OK = 0,
    WIRING_STORE_REFUSED,   /* wiring_doc_check said no: see *why */
    WIRING_STORE_IO,        /* the W25Q did not answer, the write failed or out of memory */
} wiring_store_rc_t;

/* Apply op: check, build the new document, write it to the other slot, then switch RAM to it.
   On anything but OK the profiles stay as they were. The caller has made sure the bus is free. */
wiring_store_rc_t wiring_store_apply(const wiring_op_t *op, wiring_result_t *why);

#endif /* WIRING_STORE_H */
