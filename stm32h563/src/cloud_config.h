#ifndef CLOUD_CONFIG_H
#define CLOUD_CONFIG_H

#include <stdint.h>
#include <stddef.h>

/* ---- Persistent cloud-connection config ------------------------------------
 *
 * Tells the firmware how to reach the embeddedci-server and which device it is,
 * so it can open the control WebSocket itself and reconnect on every boot —
 * no host/CLI present at runtime. Provisioned once by `benchpod register`
 * (the `cloud_set` command).
 *
 * Stored with the power-loss-safe A/B record store (config_store.h): two slots,
 * bank2 sectors 122 (0x1F4000) and 121 (0x1F2000). The old single-sector layout
 * at 0x1FA000 (sector 125) is read only when neither slot holds a record, so pods
 * provisioned by older firmware keep their config; the next save migrates it.
 * These sectors sit above the linker's FLASH_BLOBS end and the OTA scratch sector
 * (0x1F0000), so neither `make flash` nor an OTA touches them. A Wi-Fi
 * factory-reset (config_clear) does NOT clear this store.
 * ---------------------------------------------------------------------------*/

#define CLOUD_CONFIG_SLOT_A_OFFSET 0x1F4000u   /* bank2 sector 122 */
#define CLOUD_CONFIG_SLOT_B_OFFSET 0x1F2000u   /* bank2 sector 121 */
#define CLOUD_CONFIG_FLASH_OFFSET  0x1FA000u   /* legacy single-sector layout, bank2 s125 */
#define CLOUD_CONFIG_RECORD_MAGIC  0xAB5C0F03u
#define CLOUD_CONFIG_MAGIC        0xC0FFEE03u   /* distinct from config (..01) / identity (..02) */
#define CLOUD_CONFIG_VERSION      1u

#define CLOUD_HOST_MAX        64
#define CLOUD_DEVICE_ID_MAX   40   /* a UUID is 36 chars + NUL */

typedef struct {
    uint32_t magic;                       /* CLOUD_CONFIG_MAGIC */
    uint32_t version;                     /* CLOUD_CONFIG_VERSION */
    char     host[CLOUD_HOST_MAX];        /* server host, e.g. "api.embeddedci.com" (direct endpoint) */
    char     device_id[CLOUD_DEVICE_ID_MAX];
    uint16_t port;                        /* 443 (tls) / 8080 (plain) / ... */
    uint8_t  tls;                         /* 1 = wss/https, 0 = ws/http */
    uint8_t  enabled;                     /* 1 = auto-connect on boot */
    uint8_t  verify;                      /* Always 1 when tls=1: verify server cert chain +
                                             hostname against the embedded ISRG roots (cloud_ca.h).
                                             The verify=0 bring-up mode was removed; cloud_config_load
                                             upgrades any stored verify=0 to 1 when tls is set. */
    uint8_t  rsvd8[3];                    /* keep reserved[] 4-byte aligned */
    uint32_t reserved[4];                 /* headroom for future fields */
} cloud_config_t;

/* Returns 0 and fills *out if a valid config exists; -1 if none/cleared/unknown schema. */
int  cloud_config_load(cloud_config_t *out);

/* Store *cfg (magic/version are filled in). Power-loss safe: after a power loss the
   loader sees either the new config or the previous one. 0 on success. */
int  cloud_config_save(const cloud_config_t *cfg);

/* Forget the cloud config (disables auto-connect). */
void cloud_config_clear(void);

#endif /* CLOUD_CONFIG_H */
