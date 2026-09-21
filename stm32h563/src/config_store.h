#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <stdint.h>
#include <stddef.h>

/* ---- Persistent config layout ----------------------------------------------
 *
 * Stored in the LAST 4 KB sector of flash (offset CONFIG_FLASH_OFFSET).
 *
 * CMakeLists.txt sets PICO_FLASH_SIZE_BYTES = CONFIG_FLASH_OFFSET so the
 * linker never places firmware there, and OpenOCD's `program ... verify
 * reset exit` only writes ELF-loadable segments — so `make flash` does NOT
 * touch this sector.  The config survives firmware updates.
 *
 * To detect blank flash (all 0xFF) vs. a valid config we use a magic word
 * and a schema version.  We always write the whole struct atomically into a
 * single 256 B page, so no CRC is needed for tear-resistance: either the
 * write completed (magic + version present) or it did not (sector blank).
 * ---------------------------------------------------------------------------*/

#define CONFIG_FLASH_OFFSET   0x1FE000u   /* STM32H5: bank2 sector 127 (8 KB) */
#define CONFIG_MAGIC          0xC0FFEE01u
#define CONFIG_VERSION        1u

#define CONFIG_SSID_MAX       64
#define CONFIG_PASSWORD_MAX   64

typedef struct {
    uint32_t magic;                          /* CONFIG_MAGIC */
    uint32_t version;                        /* CONFIG_VERSION */
    char     ssid[CONFIG_SSID_MAX];          /* null-terminated */
    char     password[CONFIG_PASSWORD_MAX];  /* null-terminated */
    uint32_t reserved[8];                    /* headroom for future fields */
} config_t;
/* sizeof(config_t) = 4 + 4 + 64 + 64 + 32 = 168 B  (fits in one 256 B page) */

/* Returns 0 if a valid config was found and copied into *out.
   Returns -1 if the sector is blank or contains an unknown/old schema. */
int  config_load(config_t *out);

/* Erase the config sector and write *cfg.  Atomic at the sector level:
   either the new config is fully written or the sector is blank.
   Returns 0 on success, -1 on failure. */
int  config_save(const config_t *cfg);

/* Erase the config sector — equivalent to "factory reset" for WiFi creds. */
void config_clear(void);

#endif /* CONFIG_STORE_H */
