// Firmware update over the control channel, relayed by the pod.
//
// OTA_BEGIN must carry the pod's signed manifest (components/bridge_core/fwsig.h): an Ed25519
// signature by one of the pod's release keys (plus the developer key in development builds),
// for target "bridge" and this exact size. Chunks arrive in order (bridge_util.c: bu_ota_*) and
// are hashed as they are written; OTA_END compares the SHA-256 with the manifest before the image
// is marked bootable. The first chunk must also identify a benchpod-wifi-bridge app. In
// production builds secure boot v2 adds its own RSA-3072 check in esp_ota_end.
//
// Rollback: the pod is the only management path, so a new image confirms itself only once it
// has handled a valid control request from the pod (Ethernet, EMAC rx and the control channel
// all work). Without one within CONFIG_BRIDGE_CONFIRM_TIMEOUT_S it reboots, and the bootloader
// goes back to the previous slot.
#include <stdio.h>
#include <string.h>

#include "bridge.h"
#include "bridge_util.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "fw_keys_gen.h"
#include "fwsig.h"
#include "hmac_sha256.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "ota";
static SemaphoreHandle_t s_lock;
static esp_ota_handle_t s_handle;
static const esp_partition_t *s_part;
static bu_ota_seq_t s_seq;
static int64_t s_last_us;
static fwsig_t s_sig;              // the manifest of the transfer in progress
static sha256_ctx_t s_hash;        // of the bytes written so far

// The pod's signing keys: keys/release-N.pub, plus its developer key in development builds
// (build.sh generates fw_keys_gen.h from stm32h563/keys with tools/fwsign.py keys-header).
static const uint8_t k_keys[FW_SIGN_KEY_COUNT][32] = FW_SIGN_KEYS_INIT;

void ota_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_app_desc_t *d = esp_app_get_description();
    ESP_LOGI(TAG, "running %s %s from %s", d->project_name, d->version, run ? run->label : "?");
}

bool ota_active(void)
{
    return s_seq.active;
}

static void abort_locked(void)
{
    if (s_seq.active) {
        esp_ota_abort(s_handle);
        memset(&s_seq, 0, sizeof(s_seq));
        ESP_LOGW(TAG, "transfer aborted");
    }
}

void ota_abort(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    abort_locked();
    xSemaphoreGive(s_lock);
}

void ota_poll(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_seq.active && esp_timer_get_time() - s_last_us > 60 * 1000000LL) {
        abort_locked();
    }
    xSemaphoreGive(s_lock);
}

bwp_result_t ota_begin(uint32_t size, const uint8_t *sig, size_t sig_len, const char **msg)
{
    static char why[40];
    bwp_result_t r = BWP_OK;
    *msg = NULL;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    abort_locked();
    s_part = esp_ota_get_next_update_partition(NULL);
    fwsig_result_t v = fwsig_check(sig, sig_len, k_keys, FW_SIGN_KEY_COUNT, FWSIG_TARGET_BRIDGE, size, &s_sig);
    if (v != FWSIG_OK) {
        r = BWP_E_OTA_VERIFY;
        snprintf(why, sizeof(why), "signature: %s", sig ? fwsig_result_str(v) : "none");
        *msg = why;
        ESP_LOGW(TAG, "begin refused: %s", why);
    } else if (!s_part) {
        r = BWP_E_INTERNAL;
        *msg = "no OTA partition";
    } else if (size < BU_IMG_DESC_END || size > s_part->size) {
        r = BWP_E_OTA_TOO_LARGE;
        *msg = "image size out of range";
    } else {
        ESP_LOGI(TAG, "begin: %lu bytes into %s", (unsigned long)size, s_part->label);
        esp_err_t err = esp_ota_begin(s_part, size, &s_handle);  // erases: can take seconds
        if (err != ESP_OK) {
            r = BWP_E_STORAGE;
            *msg = esp_err_to_name(err);
        } else {
            bu_ota_begin(&s_seq, size);
            sha256_init(&s_hash);
            s_last_us = esp_timer_get_time();
        }
    }
    xSemaphoreGive(s_lock);
    return r;
}

bwp_result_t ota_data(uint32_t offset, const uint8_t *data, size_t len, uint32_t *next, const char **msg)
{
    bwp_result_t r = BWP_OK;
    *msg = NULL;
    if (len == 0 || len > BWP_OTA_CHUNK_MAX) {
        *next = s_seq.next;
        return BWP_E_BAD_REQUEST;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    switch (bu_ota_check(&s_seq, offset, len)) {
    case BU_OTA_INACTIVE:
        r = BWP_E_OTA_NOT_STARTED;
        break;
    case BU_OTA_DUPLICATE:
        break;  // already written, the pod missed our ack
    case BU_OTA_BAD_OFFSET:
        r = BWP_E_OTA_BAD_OFFSET;
        break;
    case BU_OTA_TOO_LARGE:
        r = BWP_E_OTA_TOO_LARGE;
        break;
    case BU_OTA_WRITE: {
        if (offset == 0) {
            char project[33], version[33];
            if (!bu_image_ident(data, len, project, version) ||
                strcmp(project, esp_app_get_description()->project_name) != 0) {
                r = BWP_E_OTA_WRONG_IMAGE;
                *msg = "not a benchpod-wifi-bridge image";
                break;
            }
            ESP_LOGI(TAG, "image %s %s", project, version);
        }
        esp_err_t err = esp_ota_write(s_handle, data, len);
        if (err != ESP_OK) {
            r = BWP_E_STORAGE;
            *msg = esp_err_to_name(err);
            break;
        }
        sha256_update(&s_hash, data, len);
        bu_ota_advance(&s_seq, offset, len);
        break;
    }
    }
    if (s_seq.active) {
        s_last_us = esp_timer_get_time();
    }
    *next = s_seq.next;
    xSemaphoreGive(s_lock);
    return r;
}

bwp_result_t ota_end(const char **msg)
{
    bwp_result_t r = BWP_OK;
    *msg = NULL;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!s_seq.active) {
        r = BWP_E_OTA_NOT_STARTED;
    } else if (!bu_ota_complete(&s_seq)) {
        r = BWP_E_OTA_BAD_OFFSET;
        *msg = "image incomplete";
    } else {
        uint8_t h[32];
        sha256_final(&s_hash, h);
        memset(&s_seq, 0, sizeof(s_seq));
        if (memcmp(h, s_sig.sha256, sizeof(h)) != 0) {
            esp_ota_abort(s_handle);
            xSemaphoreGive(s_lock);
            *msg = "image does not match its signed manifest";
            return BWP_E_OTA_VERIFY;
        }
        esp_err_t err = esp_ota_end(s_handle);  // the image's own checks (+ RSA with secure boot)
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            r = BWP_E_OTA_VERIFY;
            *msg = "image verification failed (hash or signature)";
        } else if (err != ESP_OK) {
            r = BWP_E_INTERNAL;
            *msg = esp_err_to_name(err);
        } else if ((err = esp_ota_set_boot_partition(s_part)) != ESP_OK) {
            r = BWP_E_STORAGE;
            *msg = esp_err_to_name(err);
        } else {
            ESP_LOGI(TAG, "image verified, boots from %s after reboot", s_part->label);
        }
    }
    xSemaphoreGive(s_lock);
    return r;
}

static volatile bool s_pending;
static esp_timer_handle_t s_confirm_timer;

bool ota_pending_verify(void)
{
    return s_pending;
}

void ota_confirm(void)
{
    if (!s_pending) {
        return;
    }
    if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
        s_pending = false;
        esp_timer_stop(s_confirm_timer);
        ESP_LOGI(TAG, "new image confirmed by the pod");
    }
}

static void rollback_task(void *arg)
{
    board_reboot();  // does not return
}

static void confirm_timeout(void *arg)
{
    // esp_timer task: hand the reboot to a short-lived task (board_reboot sleeps).
    ESP_LOGE(TAG, "no control request from the pod in %d s: rebooting to roll back",
             CONFIG_BRIDGE_CONFIRM_TIMEOUT_S);
    xTaskCreate(rollback_task, "rollback", 2048, NULL, 10, NULL);
}

void ota_check_pending(void)
{
    esp_ota_img_states_t st;
    const esp_partition_t *run = esp_ota_get_running_partition();
    if (esp_ota_get_state_partition(run, &st) != ESP_OK || st != ESP_OTA_IMG_PENDING_VERIFY) {
        return;
    }
    s_pending = true;
    const esp_timer_create_args_t args = {.callback = confirm_timeout, .name = "ota_confirm"};
    if (esp_timer_create(&args, &s_confirm_timer) == ESP_OK) {
        esp_timer_start_once(s_confirm_timer, (uint64_t)CONFIG_BRIDGE_CONFIRM_TIMEOUT_S * 1000000ULL);
    }
    ESP_LOGW(TAG, "new image on trial: waiting up to %d s for the pod", CONFIG_BRIDGE_CONFIRM_TIMEOUT_S);
}
