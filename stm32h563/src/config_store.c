#include "config_store.h"

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/regs/addressmap.h"   /* XIP_BASE */

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* Compile-time check: struct must fit in one program page. */
_Static_assert(sizeof(config_t) <= FLASH_PAGE_SIZE,
               "config_t must fit in a single 256 B flash page");

#if PICO_NO_FLASH
/* ---- RAM-only (no_flash) build -------------------------------------------
 * Running entirely from SRAM (e.g. a board with an unusable flash chip — see
 * 'make flash SRAM=ON').  There is no flash to persist to, so back the config
 * with a volatile RAM shadow: it survives within a boot but not a power cycle.
 * ---------------------------------------------------------------------------*/
static config_t s_ram_cfg;
static bool     s_ram_valid = false;

int config_load(config_t *out) {
    if (!s_ram_valid) {
        printf("[cfg] (sram) no config in RAM shadow\n");
        return -1;
    }
    if (out) memcpy(out, &s_ram_cfg, sizeof(*out));
    return 0;
}

int config_save(const config_t *cfg) {
    if (!cfg) return -1;
    memcpy(&s_ram_cfg, cfg, sizeof(s_ram_cfg));
    s_ram_valid = true;
    printf("[cfg] (sram) saved to RAM shadow (volatile — lost on power cycle)\n");
    return 0;
}

void config_clear(void) {
    s_ram_valid = false;
    printf("[cfg] (sram) RAM shadow cleared\n");
}

#else  /* normal flash-backed build */

/* Read pointer to the in-flash config (memory-mapped via XIP). */
static const config_t *flash_config(void) {
    return (const config_t *)(XIP_BASE + CONFIG_FLASH_OFFSET);
}

int config_load(config_t *out) {
    const config_t *p = flash_config();
    if (p->magic != CONFIG_MAGIC) {
        printf("[cfg] no valid config in flash (magic=0x%08lx, expected 0x%08lx)\n",
               (unsigned long)p->magic, (unsigned long)CONFIG_MAGIC);
        return -1;
    }
    if (p->version != CONFIG_VERSION) {
        printf("[cfg] config schema mismatch (version=%lu, expected %lu) — ignoring\n",
               (unsigned long)p->version, (unsigned long)CONFIG_VERSION);
        return -1;
    }
    if (out) memcpy(out, p, sizeof(config_t));
    return 0;
}

int config_save(const config_t *cfg) {
    if (!cfg) return -1;

    /* flash_range_program requires a buffer that is a multiple of
       FLASH_PAGE_SIZE (256 B).  Copy our struct into a page-sized stack
       buffer and pad with 0xFF (the erased-flash value). */
    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    memcpy(page, cfg, sizeof(config_t));

    printf("[cfg] saving to flash offset 0x%08x (%u B + pad to %u B)...\n",
           (unsigned)CONFIG_FLASH_OFFSET,
           (unsigned)sizeof(config_t),
           (unsigned)FLASH_PAGE_SIZE);

    /* Disable interrupts around the flash operation.  The RP2350 cannot
       execute XIP code while flash is being erased/programmed; the SDK's
       flash helpers run from RAM, but any IRQ handler resident in flash
       would crash.  Single-core firmware — no multicore lockout needed. */
    uint32_t irqs = save_and_disable_interrupts();
    flash_range_erase(CONFIG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CONFIG_FLASH_OFFSET, page, FLASH_PAGE_SIZE);
    restore_interrupts(irqs);

    /* Verify by reading back via XIP. */
    const config_t *p = flash_config();
    if (p->magic != CONFIG_MAGIC || p->version != CONFIG_VERSION) {
        printf("[cfg] ERROR: write verification failed\n");
        return -1;
    }
    printf("[cfg] saved OK\n");
    return 0;
}

void config_clear(void) {
    printf("[cfg] erasing flash sector at 0x%08x...\n",
           (unsigned)CONFIG_FLASH_OFFSET);
    uint32_t irqs = save_and_disable_interrupts();
    flash_range_erase(CONFIG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    restore_interrupts(irqs);
    printf("[cfg] sector erased\n");
}

#endif /* PICO_NO_FLASH */
