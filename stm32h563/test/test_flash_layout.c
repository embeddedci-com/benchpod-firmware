/* Host unit test for flash_layout: the 2 MB persistence offsets on a 2 MB part stay exactly
   where they are, and on a 1 MB part they keep the same distance from the end of flash. */
#include "flash_layout.h"
#include "adc_cal.h"
#include "dac_limits.h"
#include "cloud_config.h"
#include "config_store.h"
#include "pod_policy.h"
#include <stdio.h>

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* Every persistence offset, as written for the 2 MB part. The identity record and the OTA
   self-test scratch are private to their .c files; their values are repeated here. */
static const struct { const char *name; uint32_t off; } k_stores[] = {
    { "pod-policy B",   POD_POLICY_SLOT_B_OFFSET },
    { "pod-policy A",   POD_POLICY_SLOT_A_OFFSET },
    { "adc-cal B",      ADC_CAL_SLOT_B_OFFSET },
    { "adc-cal A",      ADC_CAL_SLOT_A_OFFSET },
    { "dac-limits B",   DAC_LIMITS_SLOT_B_OFFSET },
    { "dac-limits A",   DAC_LIMITS_SLOT_A_OFFSET },
    { "ota scratch",    0x1F0000u },
    { "cloud B",        CLOUD_CONFIG_SLOT_B_OFFSET },
    { "cloud A",        CLOUD_CONFIG_SLOT_A_OFFSET },
    { "wifi B",         CONFIG_SLOT_B_OFFSET },
    { "wifi A",         CONFIG_SLOT_A_OFFSET },
    { "cloud legacy",   CLOUD_CONFIG_FLASH_OFFSET },
    { "identity",       0x1FC000u },
    { "wifi legacy",    CONFIG_FLASH_OFFSET },
};
#define N_STORES (sizeof(k_stores) / sizeof(k_stores[0]))

int main(void) {
    for (unsigned i = 0; i < N_STORES; i++) {
        uint32_t off = k_stores[i].off;
        uint32_t two = flash_layout_map(off, FLASH_LAYOUT_SIZE_2MB);
        uint32_t one = flash_layout_map(off, FLASH_LAYOUT_SIZE_1MB);

        CHECK(off >= FLASH_LAYOUT_STORE_BASE && off < FLASH_LAYOUT_REF_SIZE,
              "%s 0x%06x is outside the 2 MB persistence area", k_stores[i].name, (unsigned)off);
        CHECK(two == off, "%s moved on a 2 MB part: 0x%06x -> 0x%06x",
              k_stores[i].name, (unsigned)off, (unsigned)two);
        CHECK(one == off - 0x100000u, "%s on a 1 MB part is 0x%06x, want 0x%06x",
              k_stores[i].name, (unsigned)one, (unsigned)(off - 0x100000u));
        CHECK(one >= 0x0E4000u && one < FLASH_LAYOUT_SIZE_1MB && (one % 0x2000u) == 0u,
              "%s on a 1 MB part is not a sector in its top 112 KB: 0x%06x",
              k_stores[i].name, (unsigned)one);
        for (unsigned j = 0; j < i; j++)
            CHECK(k_stores[j].off != off, "%s and %s share a sector", k_stores[i].name, k_stores[j].name);
    }

    /* Offsets below the persistence area are not remapped. */
    CHECK(flash_layout_map(0x000000u, FLASH_LAYOUT_SIZE_1MB) == 0x000000u, "offset 0 remapped");
    CHECK(flash_layout_map(0x1E3FF0u, FLASH_LAYOUT_SIZE_1MB) == 0x1E3FF0u, "offset below the store remapped");
    /* The older records did not move: ADC cal B is still where pods have it. */
    CHECK(flash_layout_map(ADC_CAL_SLOT_B_OFFSET, FLASH_LAYOUT_SIZE_2MB) == 0x1E8000u &&
          flash_layout_map(ADC_CAL_SLOT_B_OFFSET, FLASH_LAYOUT_SIZE_1MB) == 0x0E8000u, "ADC cal moved");
    /* The lowest store sits right above the 1 MB part's 912 KB code end. */
    CHECK(flash_layout_map(FLASH_LAYOUT_STORE_BASE, FLASH_LAYOUT_SIZE_1MB) == 0x0E4000u, "1 MB store base");

    if (fails) { printf("test_flash_layout: %d FAILED\n", fails); return 1; }
    printf("test_flash_layout: all passed\n");
    return 0;
}
