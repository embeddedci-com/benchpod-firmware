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
 * Stored in its OWN 8 KB sector at 0x1FA000 (bank2 sector 125) — the sector
 * just below the device-identity sector (0x1FC000) and the WiFi-config sector
 * (0x1FE000, the last sector).  Survives `make flash` for the same reason those
 * do: the firmware is ~120 KB (nowhere near the bank-2 tail), and OpenOCD only
 * programs loadable ELF segments, so this sector is never touched.  A WiFi
 * factory-reset (config_clear) does NOT erase it.
 *
 * Same magic/version + single-page write discipline as config_store.c; the
 * Pico-style flash API maps onto STM32H5 internal flash via flash_compat.c.
 * ---------------------------------------------------------------------------*/

#define CLOUD_CONFIG_FLASH_OFFSET 0x1FA000u    /* STM32H5: bank2 sector 125 (8 KB) */
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

/* Returns 0 and fills *out if a valid config exists; -1 if blank/unknown schema. */
int  cloud_config_load(cloud_config_t *out);

/* Erase the sector and write *cfg. Atomic at the sector level. 0 on success. */
int  cloud_config_save(const cloud_config_t *cfg);

/* Erase the cloud-config sector (disables auto-connect). */
void cloud_config_clear(void);

#endif /* CLOUD_CONFIG_H */
