#ifndef MSC_FLASH_H
#define MSC_FLASH_H

/* msc_flash — flashing a board that shows up as a USB drive (the logic; xip_mod_msc_flash.c runs
 * from the W25Q, usb_flash.c drives it).
 *
 * Two modes, both writing 512-byte sectors through the ops below:
 *  - UF2: the UF2 bootloaders (Raspberry Pi Pico, Adafruit/Seeed boards, tinyuf2) take UF2 blocks
 *    written to any sector. Each 512-byte block of the file is checked (magic numbers, block count)
 *    and written as one sector into the drive's data area, in order.
 *  - file: drag-and-drop loaders (DAPLink on NUCLEO, nRF DK, micro:bit, ...) look for a file. A
 *    small FAT12/FAT16/FAT32 writer puts the data into a run of free clusters, then writes the FAT
 *    chain (every FAT copy), then the root directory entry, so the entry with the final size comes
 *    last. FAT32 also grows a full root directory by one cluster, frees the chain of a file it
 *    replaces and updates the FSInfo sector (when its signatures are valid). exFAT: the file goes into a
 *    contiguous run of free clusters (NoFatChain) found in the allocation bitmap, then the bitmap bits,
 *    then the directory entry set last; a replaced file's set is marked unused and its clusters freed.
 *
 * The context holds all the state (the code has no static data: it runs from the W25Q). One step
 * does a bounded amount of work, so the hw worker keeps answering commands between steps.
 * Every number the drive reports is checked before it is used: the drive is under test.
 */

#include <stdbool.h>
#include <stdint.h>

#define MSC_FLASH_SECTOR 512u

typedef struct {
    void *io;
    /* Sector read and write on the drive: count sectors of 512 bytes. 0 = ok. */
    int (*read)(void *io, uint32_t lba, uint8_t *buf, uint32_t count);
    int (*write)(void *io, uint32_t lba, const uint8_t *buf, uint32_t count);
    /* The staged file: n bytes at offset off. 0 = ok. */
    int (*src)(void *io, uint32_t off, uint8_t *buf, uint32_t n);
    uint32_t block_count;   /* sectors the drive reports */
    uint32_t block_size;    /* bytes per sector it reports (only 512 is taken) */
} msc_flash_ops_t;

/* READ and VERIFY run after a flash, on the drive that came back (usb_flash.c does the waiting):
 *  - READ: find `name83` in the root directory and read its first bytes into `out` (found = 1), e.g.
 *    FAIL.TXT, which DAPLink loaders write after a failed copy. Not found is a result, not an error.
 *  - VERIFY: the staged UF2 file against the drive's CURRENT.UF2 (the loader's flash contents as
 *    UF2 blocks): each block of the file is looked up by its target address and its payload compared.
 *    MSC_FLASH_E_NOFILE when the drive has no such file (the board left its loader). */
typedef enum { MSC_FLASH_UF2 = 0, MSC_FLASH_FILE = 1, MSC_FLASH_READ = 2, MSC_FLASH_VERIFY = 3 } msc_flash_mode_t;

typedef enum {
    MSC_FLASH_RUNNING = 1,
    MSC_FLASH_DONE = 0,
    MSC_FLASH_E_IO = -1,          /* a sector read or write failed */
    MSC_FLASH_E_SRC = -2,         /* reading the staged file failed */
    MSC_FLASH_E_NOT_UF2 = -3,     /* a block is not a UF2 block, or the file is not whole blocks */
    MSC_FLASH_E_SECTOR = -4,      /* the drive's sector size is not 512 */
    MSC_FLASH_E_FS = -5,          /* no FAT12/FAT16/FAT32/exFAT file system the writer understands */
    MSC_FLASH_E_FULL = -6,        /* no free directory entry, or no run of free clusters long enough */
    MSC_FLASH_E_ARGS = -7,        /* bad mode, name or length */
    MSC_FLASH_E_ABORTED = -8,     /* stopped by the client (usb_flash.c, not the steps) */
    MSC_FLASH_E_EXFAT = -9,       /* READ/VERIFY on an exFAT drive (not supported) */
    MSC_FLASH_E_VERIFY = -10,     /* VERIFY: the drive holds something else than the file */
    MSC_FLASH_E_NOFILE = -11,     /* VERIFY: the drive has no CURRENT.UF2 */
    MSC_FLASH_E_LOADER = -12,     /* the loader reported a failure (usb_flash.c, from FAIL.TXT) */
} msc_flash_rc_t;

typedef enum {
    MSC_PH_START = 0, MSC_PH_UF2, MSC_PH_DIR_SCAN, MSC_PH_FAT_SCAN, MSC_PH_DATA, MSC_PH_FAT,
    MSC_PH_DIR, MSC_PH_DONE,
    MSC_PH_DIR_ZERO, MSC_PH_FREE_OLD, MSC_PH_FSINFO,    /* FAT32 only */
    MSC_PH_FIND, MSC_PH_READ, MSC_PH_VERIFY, MSC_PH_WAIT,  /* READ and VERIFY; WAIT: usb_flash.c */
    MSC_PH_XBITMAP, MSC_PH_XLINK, MSC_PH_XUNUSE, MSC_PH_XSET, MSC_PH_XFREE   /* exFAT only */
} msc_flash_phase_t;

typedef struct {
    /* in */
    msc_flash_ops_t ops;
    uint8_t  mode;               /* msc_flash_mode_t */
    char     name83[11];         /* FILE: "FIRMWAREBIN" (8.3, space padded, upper case) */
    uint32_t len;                /* bytes of the staged file */
    uint32_t unix_time;          /* FILE: seconds since 1970 (UTC) for the new entry's dates, 0 = not known (2026-01-01) */
    /* out */
    uint8_t  phase;              /* msc_flash_phase_t */
    int      result;             /* msc_flash_rc_t once done */
    uint32_t done, total;        /* sectors written / to write (data only) */
    /* FAT geometry (FILE, and UF2 when the drive has one) */
    uint32_t vol_start;          /* first sector of the volume (an MBR partition, else 0) */
    uint8_t  fat_bits;           /* 12, 16 or 32 */
    uint8_t  nfats;
    uint16_t spc;                /* sectors per cluster */
    uint32_t fat_start, fat_secs, root_start, root_secs, data_start, clusters;
    /* work */
    uint32_t pos;                /* phase cursor */
    uint32_t data_base;          /* UF2: first sector written */
    uint32_t dir_lba, dir_off;   /* FILE: the free root entry */
    uint32_t need_clusters, run_start, run_len;
    uint32_t fat_cached;         /* LBA of the first sector in buf (FAT window), UINT32_MAX none */
    bool     fat_dirty;          /* buf holds FAT changes not written yet */
    uint8_t  fat_copy;           /* FAT phase: the copy being written */
    /* FAT32 */
    uint8_t  fat_rd, fat_wr_end; /* the FAT copy to read; copies fat_rd..fat_wr_end-1 are written */
    uint8_t  fsi_ok;             /* the FSInfo sector is there and its signatures are valid */
    uint8_t  dir_extend;         /* 1: the root directory is full, the run starts with a new cluster */
    uint8_t  wrapped, scan_end;  /* free-space search: second pass; directory scan: finished */
    uint32_t root_clus, fsi_lba, fsi_free, hint;   /* root cluster; FSInfo sector, count, next free */
    uint32_t dir_clus, dir_last, dir_n;            /* root chain: current, last, clusters seen */
    uint32_t old_first, free_cur, free_n, freed;   /* a replaced file's chain, clusters freed */
    /* exFAT (the FAT window is the second half of buf, the working sector the first) */
    uint8_t  is_exfat, x_active; /* an exFAT volume; its active FAT and bitmap (0 or 1) */
    uint8_t  x_rem, x_idx, x_ok, x_cnt, x_file;   /* directory scan: entries left in the set, position, name matches, secondaries, not a directory */
    uint8_t  x_zero, x_mk, old_flags, old_cnt;    /* new entry: 1 zero the sector after it, 2 zero it all; end marker seen */
    uint8_t  bm_dirty;
    uint16_t x_hash, x_nlen, x_pos;               /* name hash and length; characters compared */
    uint32_t bm_first, bm_len, bm_clus, bm_ci, bm_i, bm_lba;   /* allocation bitmap: chain start, bytes, cursor, sector in buf */
    uint32_t cs_clus, cs_sec, cs_off;             /* start of the set being scanned */
    uint32_t old_clus, old_sec, old_off, old_len; /* the set (and file) being replaced */
    uint32_t mk_lba, mk_off, mk_sec, pad_lba, pad_off;
    /* READ and VERIFY */
    uint8_t  found;              /* READ: the file is in the root directory */
    uint8_t  vf_stage;           /* VERIFY: 0 file head, 1 next block, 2 walk to its cluster, 3 compare */
    uint16_t out_len;            /* READ: bytes in out */
    uint32_t f_clus, f_size;     /* the file found: first cluster, bytes */
    uint32_t vf_base, vf_plen, vf_sec;   /* VERIFY: address of the file's first block, payload size,
                                            sector of the block being checked */
    uint32_t vf_clus, vf_idx;    /* VERIFY: the cluster walked to, its index in the file */
    uint8_t  out[128];           /* READ: the file's first bytes */
    uint8_t  buf[2 * MSC_FLASH_SECTOR];   /* two sectors: a FAT12 entry can straddle them */
} msc_flash_ctx_t;

/* Fill in the inputs and call until it does not return MSC_FLASH_RUNNING. */
int  msc_flash_step(msc_flash_ctx_t *c);
const char *msc_flash_rc_str(int rc);
const char *msc_flash_phase_str(int phase);
/* "firmware.bin" -> "FIRMWAREBIN"; false for a name that is not a plain 8.3 name. */
bool msc_flash_name83(const char *name, char out[11]);

#endif /* MSC_FLASH_H */
