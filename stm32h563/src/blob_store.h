/*
 * blob_store.h — the ESP32-C3 image and the iCE40 gateware images, kept in slots of the W25Q
 * (the iCE40 config flash) instead of in the STM32's internal flash.
 *
 * W25Q layout (8 MB W25Q64; a 16 MB W25Q128 has the same layout plus free space):
 *
 *   0x000000  the gateware the iCE40 boots from (ice40_flash.c rewrites it)
 *   0x100000  slot GW0: gateware image 0 (closed loop)     256 KB
 *   0x140000  slot GW1: gateware image 1 (deep replay)     256 KB
 *   0x180000  free for two more gateware slots
 *   0x200000  slot ESP: ESP32-C3 esp-hosted image          4 MB (the C3's own flash size)
 *   0x600000  free
 *
 * A slot is a 4 KB header sector followed by the data. A write erases the slot (header first,
 * so it reads as empty from that moment), programs and read-back-hashes the data, then programs
 * the header and, last, a commit word. Power lost anywhere before the commit word leaves an
 * empty slot, never a half-written one that looks valid.
 *
 * Headers are read once at boot (blob_store_init) and cached, so asking what a slot holds never
 * touches the shared bus. Everything that reads or writes slot data needs the bus held
 * (w25q_open), on the hw worker task.
 */
#ifndef BLOB_STORE_H
#define BLOB_STORE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum {
    BLOB_GW0 = 0,     /* gateware image 0: closed loop */
    BLOB_GW1 = 1,     /* gateware image 1: deep replay */
    BLOB_ESP = 2,     /* ESP32-C3 esp-hosted slave image */
    BLOB_COUNT
} blob_id_t;

#define BLOB_HDR_SIZE      0x1000u
#define BLOB_HDR_MAGIC     0x4C425042u   /* "BPBL" */
#define BLOB_HDR_VERSION   1u
#define BLOB_COMMIT_OFF    0x80u         /* commit word, inside the header sector */
#define BLOB_COMMIT_MAGIC  0x544D4F43u   /* "COMT" */

typedef struct {
    uint32_t magic;          /* BLOB_HDR_MAGIC */
    uint16_t hdr_version;    /* BLOB_HDR_VERSION */
    uint16_t id;             /* blob_id_t */
    uint32_t len;            /* data bytes */
    uint32_t version;        /* gateware version for GW slots, 0 when not known */
    uint8_t  sha256[32];     /* of the data */
    uint32_t reserved[4];
} blob_hdr_t;

/* What a slot holds, from the cached header. */
typedef struct {
    bool     present;        /* a committed header */
    uint32_t len;
    uint32_t version;
    uint8_t  sha256[32];
} blob_info_t;

/* Slot geometry. */
uint32_t    blob_slot_base(blob_id_t id);
uint32_t    blob_slot_capacity(blob_id_t id);    /* data bytes the slot can hold */
const char *blob_name(blob_id_t id);             /* "gw0", "gw1", "esp" */
int         blob_from_name(const char *name);    /* blob_id_t, or -1 */

/* Read the headers into the cache. Bus held by the caller. Returns the number of slots present,
   or -1 when no usable W25Q answers (then every slot reads as empty). */
int  blob_store_load(void);
/* blob_store_load inside w25q_open/close; call where the bus is free (boot, the hw worker). */
int  blob_store_init(void);

/* Cached slot state. */
const blob_info_t *blob_store_info(blob_id_t id);
bool blob_store_present(blob_id_t id);

/* Where a write's data comes from: n <= 256 bytes at offset `off`. */
typedef int (*blob_src_fn)(void *ctx, uint32_t off, uint8_t *buf, uint32_t n);

/* Write a whole slot from `src`, hash the written data back against `sha256`, then commit the
   header. Bus held by the caller. 0 = ok; <0 = failed, slot left empty. Updates the cache. */
int  blob_store_write(blob_id_t id, uint32_t len, uint32_t version, const uint8_t sha256[32],
                      blob_src_fn src, void *ctx);

/* Read slot data. Bus held by the caller. Same shape as blob_src_fn and ice40_src_read_fn, with
   ctx = (void *)(uintptr_t)id, so a slot can feed ice40_flash_program_src directly. */
int  blob_store_read(void *ctx, uint32_t off, uint8_t *buf, uint32_t n);

/* Hash a present slot's data against its header. Bus held. 0 = intact. */
int  blob_store_verify(blob_id_t id);

/* A blob was just installed into its slot (OTA): put it to use. Defined by the firmware
   (blob_hooks.c): a gateware slot may update the running gateware, an ESP slot lets Wi-Fi retry
   flashing a blank C3. */
void blob_store_on_installed(blob_id_t id);

#endif /* BLOB_STORE_H */
