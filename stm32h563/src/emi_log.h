/*
 * emi_log.h — the EMI test build's run log (emi_test.h): what every run did, kept in the W25Q
 * (the iCE40 config flash) so a lab run that nobody watched can still be read afterwards.
 *
 * Where: the free range between slot PROXY and slot ESP (blob_store.h), 0x192000-0x200000,
 * 440 KB = 110 sectors. Only the EMI test build writes there and nothing reserves it: a blob slot
 * added there later would erase the log (and this log would corrupt it), so moving a slot into
 * this range means moving the log first.
 *
 * A run is one power-on (or reset). The log is a ring of 64-byte records (EMI_LOG_REC):
 *
 *   RUN_START   at boot: run number, firmware version, reset cause, core clock, DAC mode
 *   EVENT       on a change: LED state (green -> red, red -> blue, ...), Ethernet link lost or
 *               back, the activity started or stopped
 *   CHECKPOINT  captures, errors, frames sent and dropped, link losses, LED state, the longest
 *               DAC gap a log write caused (0: the outputs keep running through log writes,
 *               emi_test.c). Written when a value that shows trouble changed (errors, drops,
 *               link losses, LED, link, DAC gap, lost records), at most every
 *               EMI_LOG_CHECKPOINT_MIN_MS, and every EMI_LOG_HEARTBEAT_MS when nothing changed
 *               (emi_log_checkpoint_due). Captures and frames sent grow all the time: they are in
 *               every checkpoint but do not trigger one
 *
 * Every record carries a sequence number that only grows (across runs), the run number, the
 * uptime and a CRC-32 (zlib's) over its first 60 bytes. Erased flash (0xFF) is an empty slot; a
 * slot that is neither empty nor a valid record (power lost while it was programmed) is skipped.
 * At boot the whole range is scanned: the newest record is the one with the highest sequence
 * number, the next write goes to the slot after it. Slots are written in order; before the write
 * position reaches a sector that is not empty, that sector (the 64 oldest records) is erased.
 *
 * Capacity: 110 sectors x 64 = 7,040 slots. emi_test.c keeps 16 sectors (1,024 slots) erased
 * ahead of the write position, erased at boot, and about 6,000 records of history. A quiet hour
 * takes about 60 records (the heartbeat), an hour with trouble all the time at most 360 (one
 * checkpoint every 10 s), plus the events. So a lab day writes into flash erased at boot and
 * never erases while it runs, unless it fails for more than about 2.8 hours in all; then a sector
 * is erased between two captures (that capture waits about 60 ms, the outputs keep running). The
 * history is about 100 quiet hours, or 16 hours of trouble.
 *
 * This file is the record format and the ring logic only, over three flash callbacks, so it runs
 * in the host tests. emi_test.c gives it the W25Q (w25q_open/close after
 * signal_engine_quiesce_psram_keep_dac, on the hw worker) and decides when the bus may be taken.
 */
#ifndef EMI_LOG_H
#define EMI_LOG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define EMI_LOG_BASE        0x192000u
#define EMI_LOG_END         0x200000u
#define EMI_LOG_SECTOR      4096u
#define EMI_LOG_REC         64u
#define EMI_LOG_PER_SECTOR  (EMI_LOG_SECTOR / EMI_LOG_REC)
/* EMI_LOG_TEST_SECTORS=<n> (make EMI_LOG_SECTORS=<n>) shrinks the ring to its first n sectors, to
   see it wrap on hardware without writing 7,040 records. */
#if defined(EMI_LOG_TEST_SECTORS)
#define EMI_LOG_SECTORS     ((uint32_t)(EMI_LOG_TEST_SECTORS))
#else
#define EMI_LOG_SECTORS     ((EMI_LOG_END - EMI_LOG_BASE) / EMI_LOG_SECTOR)   /* 110 */
#endif
#define EMI_LOG_MAX_SLOTS   (((EMI_LOG_END - EMI_LOG_BASE) / EMI_LOG_SECTOR) * EMI_LOG_PER_SECTOR)

#define EMI_LOG_MAGIC       0x4C45u   /* "EL" as stored (little-endian) */
#define EMI_LOG_CHECKPOINT_MIN_MS  (10u * 1000u)   /* a changed value: at most this often */
#define EMI_LOG_HEARTBEAT_MS       (60u * 1000u)   /* nothing changed: this often */

typedef enum {
    EMI_REC_RUN_START  = 1,
    EMI_REC_EVENT      = 2,
    EMI_REC_CHECKPOINT = 3,
} emi_rec_type_t;

typedef enum {
    EMI_EV_LED        = 1,   /* the LED state changed: from -> to (status_led_emi_t values) */
    EMI_EV_LINK_LOST  = 2,   /* the Ethernet link went down */
    EMI_EV_LINK_BACK  = 3,   /* the Ethernet link came back */
    EMI_EV_STARTED    = 4,   /* the activity started (boot start or `emi start`) */
    EMI_EV_STOPPED    = 5,   /* `emi stop` */
} emi_event_t;

/* A record, decoded. Fields a type does not use are 0. */
typedef struct {
    uint8_t  type;           /* emi_rec_type_t */
    uint8_t  led;            /* status_led_emi_t when the record was made */
    uint32_t seq;            /* set by emi_log_append */
    uint32_t run;            /* set by emi_log_append */
    uint32_t uptime_ms;
    union {
        struct {
            char     fw[16];        /* FIRMWARE_VERSION */
            char     reset[16];     /* fault_last_reset_str() */
            uint16_t core_mhz;
            uint8_t  dac;           /* emi_dac_mode_t */
            uint8_t  net_mbit;
            uint32_t gw_version;
        } start;
        struct {
            uint8_t  kind;          /* emi_event_t */
            uint8_t  from, to;      /* EMI_EV_LED: LED states */
            uint32_t captures, errors, net_sent;
            char     text[28];      /* the last error, cut */
        } ev;
        struct {
            uint32_t captures, errors, net_sent, net_drops, link_losses;
            uint32_t rec_dropped;   /* records the RAM queue had no room for */
            uint32_t glitch_us_max; /* the longest DAC interruption a log write caused, 0 normally */
            uint8_t  dac, net_mbit, running, link;
            char     text[12];      /* the last error, cut */
        } cp;
    } u;
} emi_rec_t;

/* What decides whether a checkpoint is due: the values that show trouble. */
typedef struct {
    uint32_t errors, net_drops, link_losses, rec_dropped, glitch_us_max;
    uint8_t  led, dac, running, link;
} emi_cp_sig_t;
/* A checkpoint is due when `since_ms` (since the last one) reached EMI_LOG_HEARTBEAT_MS, or
   reached EMI_LOG_CHECKPOINT_MIN_MS and `now` differs from `last`. */
bool emi_log_checkpoint_due(const emi_cp_sig_t *last, const emi_cp_sig_t *now, uint32_t since_ms);

/* 64 bytes with the magic and CRC. */
void emi_log_encode(const emi_rec_t *r, uint8_t out[EMI_LOG_REC]);
/* true when `in` is a valid record (magic, CRC, a known type). */
bool emi_log_decode(const uint8_t in[EMI_LOG_REC], emi_rec_t *r);
/* All 0xFF: an erased slot. */
bool emi_log_blank(const uint8_t in[EMI_LOG_REC]);

/* One line, no newline: "#123 run 45 +12.3s START fw 3.9.6 reset power-on core 250 MHz ...". */
size_t emi_log_format(const emi_rec_t *r, char *buf, size_t cap);
const char *emi_log_led_name(uint8_t led);

/* The flash under the ring. addr is absolute. Each returns 0 on success. */
typedef struct {
    int (*read)(void *ctx, uint32_t addr, uint8_t *buf, uint32_t len);
    int (*program)(void *ctx, uint32_t addr, const uint8_t *buf, uint32_t len);
    int (*erase_sector)(void *ctx, uint32_t addr);
    void *ctx;
} emi_log_io_t;

typedef struct {
    emi_log_io_t io;
    uint32_t base;
    uint32_t slots;          /* sectors * EMI_LOG_PER_SECTOR */
    bool     ok;             /* scanned */
    bool     any;            /* the scan found a valid record */
    uint32_t newest_seq, newest_run;   /* of the newest valid record (when any) */
    uint32_t next_seq;
    uint32_t run;            /* this boot's run number (newest_run + 1, from 1) */
    uint32_t write_slot;     /* the next slot to program */
    uint32_t ready;          /* empty slots from write_slot on */
    uint32_t valid;          /* valid records in the ring */
    uint32_t erases;         /* sectors erased since boot */
    uint32_t write_errors;   /* programs that did not read back */
    uint8_t  blank[(EMI_LOG_MAX_SLOTS + 7) / 8];   /* per slot: erased */
    uint8_t  good[(EMI_LOG_MAX_SLOTS + 7) / 8];    /* per slot: a valid record */
} emi_log_t;

/* Scan `sectors` sectors from `base`: newest record, write position, run number. 0 = ok. */
int  emi_log_scan(emi_log_t *l, const emi_log_io_t *io, uint32_t base, uint32_t sectors);
/* Erase sectors ahead of the write position until at least `min_ready` slots are empty (never
   the sector of the newest record; at most all but one sector). Returns the sectors erased, or
   <0 on a flash error. */
int  emi_log_prepare(emi_log_t *l, uint32_t min_ready);
/* Program `r` (seq and run filled in) at the write position. -1 = no empty slot (prepare first),
   -2 = flash error. A program that does not read back marks the slot bad and tries the next one. */
int  emi_log_append(emi_log_t *l, emi_rec_t *r);

/* The slot index of the oldest slot in ring order (the one after the empty run ahead of the
   write position): walking from here for emi_log->slots slots visits records oldest first. */
uint32_t emi_log_oldest_slot(const emi_log_t *l);
bool emi_log_slot_good(const emi_log_t *l, uint32_t slot);
/* The slot `n` valid records back from the newest (n = 1: the newest); slots when there are
   fewer than n, then the walk starts at emi_log_oldest_slot. */
uint32_t emi_log_slot_back(const emi_log_t *l, uint32_t n);
uint32_t emi_log_slot_addr(const emi_log_t *l, uint32_t slot);
/* The slot of the first record newer than sequence number `after` (the oldest slot when `after`
   is older than every record) and, in *newer, how many records are newer (0: none, the slot is
   then write_slot). For a viewer that keeps a copy and asks only for what it does not have. */
uint32_t emi_log_slot_after(const emi_log_t *l, uint32_t after, uint32_t *newer);

#endif /* EMI_LOG_H */
