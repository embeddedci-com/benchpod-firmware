// Firmware update, shared by the control channel (relayed by the pod) and the UART console.
//
// Chunks arrive in order (bridge_util.c: bu_ota_*). The first chunk must identify a
// benchpod-wifi-bridge app. esp_ota_end checks the SHA-256 and the RSA-3072 signature: with
// CONFIG_SECURE_SIGNED_ON_UPDATE_NO_SECURE_BOOT (dev) or secure boot v2 (production) only an
// image signed with our key is accepted. A new image must run for
// CONFIG_BRIDGE_MARK_VALID_DELAY_S with Ethernet started, or the bootloader rolls back.
#include <string.h>

#include "bridge.h"
#include "bridge_util.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

static const char *TAG = "ota";
static SemaphoreHandle_t s_lock;
static esp_ota_handle_t s_handle;
static const esp_partition_t *s_part;
static bu_ota_seq_t s_seq;
static int64_t s_last_us;

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

bwp_result_t ota_begin(uint32_t size, const char **msg)
{
    bwp_result_t r = BWP_OK;
    *msg = NULL;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    abort_locked();
    s_part = esp_ota_get_next_update_partition(NULL);
    if (!s_part) {
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
        memset(&s_seq, 0, sizeof(s_seq));
        esp_err_t err = esp_ota_end(s_handle);  // verifies hash and signature
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

static void mark_valid(void *arg)
{
    esp_ota_img_states_t st;
    const esp_partition_t *run = esp_ota_get_running_partition();
    if (esp_ota_get_state_partition(run, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGI(TAG, "new image marked valid");
    }
}

void ota_schedule_mark_valid(void)
{
    esp_timer_handle_t t;
    const esp_timer_create_args_t args = {.callback = mark_valid, .name = "ota_valid"};
    if (esp_timer_create(&args, &t) == ESP_OK) {
        esp_timer_start_once(t, (uint64_t)CONFIG_BRIDGE_MARK_VALID_DELAY_S * 1000000ULL);
    }
}
