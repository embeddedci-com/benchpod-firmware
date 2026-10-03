/*
 * upload_rx.h — raw chunk receive for uploads over the USB console.
 *
 * The console is a text console (96-character lines, 1 KB receive ring that drops when full),
 * so a blob or firmware image cannot travel as text lines at any useful speed. Instead:
 *
 *   upload-begin <target> <size> <sha256> [version]   -> "upload-begin ok" | "... error <why>"
 *   upload-data <offset> <len> <crc32 hex>\n           then exactly <len> raw bytes
 *                                                      -> "upload-data ok <offset>"
 *                                                         | "upload-data retry crc|timeout"
 *                                                         | "upload-data busy"
 *   upload-end                                         -> "upload-end ok" | "... error <why>"
 *   upload-commit                                      -> "upload-commit ok" (blob) |
 *                                                         "upload-commit resetting" (firmware)
 *   upload-status, upload-abort
 *
 * One chunk at a time, at most UPLOAD_CHUNK_MAX bytes so it always fits the receive ring; the
 * sender waits for the reply before the next chunk and resends on retry/busy. The upload-data
 * line must end in a single '\n' or '\r' (not CR-LF: the LF would be taken as data). The data
 * goes through the same staging and SHA-256 check as an OTA (ota.h); target is "firmware",
 * "gw0", "gw1" or "esp".
 *
 * This file is the byte-level part of upload-data, kept free of console and HAL so it is host
 * tested; console.c feeds it.
 */
#ifndef UPLOAD_RX_H
#define UPLOAD_RX_H

#include <stdint.h>
#include <stdbool.h>

#define UPLOAD_CHUNK_MAX     512u
#define UPLOAD_RX_TIMEOUT_MS 2000u

typedef struct {
    bool     active;          /* receiving raw bytes */
    uint32_t offset;
    uint32_t len;
    uint32_t crc;             /* expected CRC-32 (IEEE) of the chunk */
    uint32_t got;
    uint32_t last_ms;         /* time of the last byte (or the start) */
    uint8_t  buf[UPLOAD_CHUNK_MAX];
} upload_rx_t;

/* Parse "<offset> <len> <crc32 hex>" and start receiving. 0 = ok, -1 = bad arguments. */
int  upload_rx_start(upload_rx_t *u, const char *args, uint32_t now_ms);
/* Feed one byte. Returns true when the chunk is complete (receiving stops). */
bool upload_rx_byte(upload_rx_t *u, uint8_t b, uint32_t now_ms);
/* The completed chunk matches its CRC. */
bool upload_rx_crc_ok(const upload_rx_t *u);
/* No byte for UPLOAD_RX_TIMEOUT_MS while receiving: stops receiving and returns true. */
bool upload_rx_timed_out(upload_rx_t *u, uint32_t now_ms);

uint32_t upload_crc32(const uint8_t *p, uint32_t n);

#endif /* UPLOAD_RX_H */
