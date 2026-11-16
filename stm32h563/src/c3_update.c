/*
 * c3_update.c — write an esp update into the ESP32-C3 (ota.h, ota_apply_esp_staged).
 *
 * The pod keeps no copy of the C3's image any more (c3_image.h). An esp update is staged in PSRAM
 * like any other, verified by ota_end, and at ota_commit written from there straight into the C3
 * over its ROM loader: the same background flash as the pod's own automatic reflash
 * (esp_rom_flash.c), with the image source swapped. The session stays "applying" meanwhile and
 * the busy gate holds off captures and uploads, so the staging is not overwritten.
 *
 * The write starts C3_UPDATE_DELAY_MS after the commit: when the commit came in over Wi-Fi, it
 * came through the C3, and its reply (or the cloud's ota.status "applying") should leave before
 * the C3 goes into reset. The client then waits for "installed" or "error" (over Wi-Fi after the
 * link is back, about 2-3 minutes later).
 *
 * hw worker only: ota_apply_esp_staged runs inside ota_commit, c3_update_poll in the worker loop.
 */
#include "c3_update.h"
#include "ota.h"
#include "c3_image.h"
#include "esp_rom_flash.h"
#include "esp_rom_job.h"      /* esp_rom_result_str */
#include "esp_wifi_ctrl.h"
#include "net_server.h"       /* net_wifi_hold_for_flash */
#include "w25q.h"
#include "bp_log.h"
#include "stm32h5xx_hal.h"

#include <stdio.h>
#include <string.h>

#define C3_UPDATE_DELAY_MS  1000u

static bool     s_pending;
static uint32_t s_start_at;
static uint32_t s_len, s_version, s_nvs_off, s_nvs_size;
static uint8_t  s_sha[32];

int ota_apply_esp_staged(uint32_t size, uint32_t version, const uint8_t sha256[32],
                         char *why, size_t cap)
{
    if (!esp_wifi_ctrl_module_present()) {
        snprintf(why, cap, "this BenchPod has no ESP32-C3");
        return -1;
    }
    if (s_pending || esp_rom_flash_busy()) {
        snprintf(why, cap, "an ESP32-C3 flash is already running");
        return -1;
    }
    s_len = size;
    s_version = version;
    memcpy(s_sha, sha256, 32);
    s_start_at = HAL_GetTick() + C3_UPDATE_DELAY_MS;
    s_pending = true;
    return 0;
}

static void flash_done(int rc)
{
    bool ok = rc >= 0;   /* ESP_ROM_UNVERIFIED (no MD5 came back) still booted the C3 */
    w25q_session_open();
    int rec = ok ? c3_image_save(s_len, s_version, s_sha, s_nvs_off, s_nvs_size) : c3_image_forget();
    w25q_session_close();
    if (rec != 0) log_printf("[c3] the update ran, but the pod could not record it\n");
    net_wifi_hold_for_flash(false);   /* restart the Wi-Fi join with the new image, or retry */
    char why[64];
    snprintf(why, sizeof(why), "ESP32-C3 flash failed: %s", esp_rom_result_str(rc));
    ota_apply_finished(ok, ok ? NULL : why);
}

void c3_update_poll(void)
{
    if (!s_pending || (int32_t)(HAL_GetTick() - s_start_at) < 0) return;
    s_pending = false;
    s_nvs_off = s_nvs_size = 0;
    if (!esp_rom_flash_staged_nvs(&s_nvs_off, &s_nvs_size))
        log_printf("[c3] no NVS partition in the image's partition table: wifi-clear will not know it\n");
    log_printf("[c3] writing the esp update into the ESP32-C3 (%lu bytes, about 2 minutes); "
               "Wi-Fi is down meanwhile\n", (unsigned long)s_len);
    net_wifi_hold_for_flash(true);    /* the Wi-Fi control must not drive EN/BOOT meanwhile */
    if (esp_rom_flash_start_staged(s_len, flash_done) != 0) {
        net_wifi_hold_for_flash(false);
        ota_apply_finished(false, "ESP32-C3 flash could not start");
    }
}

bool c3_update_pending(void) { return s_pending; }
