/*
 * cloud_config.c — the cloud-connection config, kept in the power-loss-safe A/B
 * record store (config_store.c / config_store.h).
 */
#include "cloud_config.h"
#include "config_store.h"   /* ab_store_* */

#include <string.h>
#include <stdio.h>

_Static_assert(sizeof(cloud_config_t) <= AB_STORE_MAX_PAYLOAD,
               "cloud_config_t must fit in an A/B record");

static const ab_store_t s_cloud_store = {
    .tag          = "cloud-cfg",
    .slot_off     = { CLOUD_CONFIG_SLOT_A_OFFSET, CLOUD_CONFIG_SLOT_B_OFFSET },
    .magic        = CLOUD_CONFIG_RECORD_MAGIC,
    .version      = CLOUD_CONFIG_VERSION,
    .legacy_off   = CLOUD_CONFIG_FLASH_OFFSET,
    .legacy_magic = CLOUD_CONFIG_MAGIC,
};

int cloud_config_load(cloud_config_t *out) {
    cloud_config_t tmp;
    if (ab_store_load(&s_cloud_store, &tmp, sizeof(tmp)) != 0) return -1;
    if (tmp.magic != CLOUD_CONFIG_MAGIC) return -1;
    if (tmp.version != CLOUD_CONFIG_VERSION) {
        printf("[cloud-cfg] schema mismatch (version=%lu, expected %lu), ignoring\n",
               (unsigned long)tmp.version, (unsigned long)CLOUD_CONFIG_VERSION);
        return -1;
    }
    /* TLS always verifies now (the verify=0 bring-up mode was removed). Upgrade
       any config persisted by older firmware so a stored verify=0 can never
       downgrade a TLS link to encrypted-but-unauthenticated. */
    if (tmp.tls) tmp.verify = 1;
    if (out) memcpy(out, &tmp, sizeof(tmp));
    return 0;
}

int cloud_config_save(const cloud_config_t *cfg) {
    if (!cfg) return -1;
    cloud_config_t tmp;
    memcpy(&tmp, cfg, sizeof(tmp));
    tmp.magic   = CLOUD_CONFIG_MAGIC;
    tmp.version = CLOUD_CONFIG_VERSION;
    return ab_store_save(&s_cloud_store, &tmp, sizeof(tmp));
}

void cloud_config_clear(void) {
    ab_store_clear(&s_cloud_store);
}
