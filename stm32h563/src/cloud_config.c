#include "cloud_config.h"

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/regs/addressmap.h"   /* XIP_BASE == FLASH_BASE on STM32H5 */

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* Compile-time check: struct must fit in one program page. */
_Static_assert(sizeof(cloud_config_t) <= FLASH_PAGE_SIZE,
               "cloud_config_t must fit in a single 256 B flash page");

#if PICO_NO_FLASH
/* RAM-only build: back the config with a volatile RAM shadow (lost on power
   cycle).  STM32H5 normally runs from internal flash, so this branch is unused;
   kept for parity with config_store.c. */
static cloud_config_t s_ram_cfg;
static bool           s_ram_valid = false;

int cloud_config_load(cloud_config_t *out) {
    if (!s_ram_valid) return -1;
    if (out) memcpy(out, &s_ram_cfg, sizeof(*out));
    return 0;
}

int cloud_config_save(const cloud_config_t *cfg) {
    if (!cfg) return -1;
    memcpy(&s_ram_cfg, cfg, sizeof(s_ram_cfg));
    s_ram_valid = true;
    printf("[cloud-cfg] (sram) saved to RAM shadow (volatile — lost on power cycle)\n");
    return 0;
}

void cloud_config_clear(void) {
    s_ram_valid = false;
    printf("[cloud-cfg] (sram) RAM shadow cleared\n");
}

#else  /* normal flash-backed build */

/* Read pointer to the in-flash config (memory-mapped at FLASH_BASE). */
static const cloud_config_t *flash_cloud_config(void) {
    return (const cloud_config_t *)(XIP_BASE + CLOUD_CONFIG_FLASH_OFFSET);
}

int cloud_config_load(cloud_config_t *out) {
    const cloud_config_t *p = flash_cloud_config();
    if (p->magic != CLOUD_CONFIG_MAGIC) {
        return -1;
    }
    if (p->version != CLOUD_CONFIG_VERSION) {
        printf("[cloud-cfg] schema mismatch (version=%lu, expected %lu) — ignoring\n",
               (unsigned long)p->version, (unsigned long)CLOUD_CONFIG_VERSION);
        return -1;
    }
    if (out) {
        memcpy(out, p, sizeof(cloud_config_t));
        /* TLS always verifies now (the verify=0 bring-up mode was removed). Upgrade
           any config persisted by older firmware so a stored verify=0 can never
           downgrade a TLS link to encrypted-but-unauthenticated. */
        if (out->tls) out->verify = 1;
    }
    return 0;
}

int cloud_config_save(const cloud_config_t *cfg) {
    if (!cfg) return -1;

    /* flash_range_program wants a buffer that is a multiple of FLASH_PAGE_SIZE
       (256 B).  Copy our struct into a page-sized stack buffer, pad with 0xFF. */
    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    memcpy(page, cfg, sizeof(cloud_config_t));

    printf("[cloud-cfg] saving to flash offset 0x%08x (%u B)...\n",
           (unsigned)CLOUD_CONFIG_FLASH_OFFSET, (unsigned)sizeof(cloud_config_t));

    uint32_t irqs = save_and_disable_interrupts();
    flash_range_erase(CLOUD_CONFIG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CLOUD_CONFIG_FLASH_OFFSET, page, FLASH_PAGE_SIZE);
    restore_interrupts(irqs);

    const cloud_config_t *p = flash_cloud_config();
    if (p->magic != CLOUD_CONFIG_MAGIC || p->version != CLOUD_CONFIG_VERSION) {
        printf("[cloud-cfg] ERROR: write verification failed\n");
        return -1;
    }
    printf("[cloud-cfg] saved OK\n");
    return 0;
}

void cloud_config_clear(void) {
    printf("[cloud-cfg] erasing flash sector at 0x%08x...\n",
           (unsigned)CLOUD_CONFIG_FLASH_OFFSET);
    uint32_t irqs = save_and_disable_interrupts();
    flash_range_erase(CLOUD_CONFIG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    restore_interrupts(irqs);
    printf("[cloud-cfg] sector erased\n");
}

#endif /* PICO_NO_FLASH */
