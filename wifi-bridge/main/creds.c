// Wi-Fi credentials in NVS. Production builds encrypt the NVS partition (flash-encryption based
// NVS encryption, keys in the encrypted nvs_keys partition): see docs/SECURITY.md.
#include <string.h>

#include "bridge.h"
#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "creds";
static const char *NS = "bridge";
static volatile int s_present = -1;  // cached: -1 unknown

bool creds_load(char ssid[33], size_t *ssid_len, char pass[65])
{
    nvs_handle_t h;
    bool ok = false;
    if (nvs_open(NS, NVS_READONLY, &h) == ESP_OK) {
        size_t n = 32, p = 65;
        if (nvs_get_blob(h, "ssid", ssid, &n) == ESP_OK && n >= 1 && nvs_get_str(h, "pass", pass, &p) == ESP_OK) {
            ssid[n] = '\0';
            *ssid_len = n;
            ok = true;
        }
        nvs_close(h);
    }
    s_present = ok;
    return ok;
}

bool creds_present(void)
{
    if (s_present < 0) {
        char ssid[33], pass[65];
        size_t n;
        creds_load(ssid, &n, pass);
        memset(pass, 0, sizeof(pass));
    }
    return s_present == 1;
}

esp_err_t creds_store(const char *ssid, size_t ssid_len, const char *pass)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(h, "ssid", ssid, ssid_len);
    if (err == ESP_OK) {
        err = nvs_set_str(h, "pass", pass);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    s_present = -1;
    return err;
}

esp_err_t creds_clear(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    nvs_erase_key(h, "ssid");
    nvs_erase_key(h, "pass");
    err = nvs_commit(h);
    nvs_close(h);
    s_present = 0;
    return err;
}

bool pairkey_load(uint8_t key[BWP_KEY_LEN])
{
    nvs_handle_t h;
    bool ok = false;
    if (nvs_open(NS, NVS_READONLY, &h) == ESP_OK) {
        size_t n = BWP_KEY_LEN;
        ok = nvs_get_blob(h, "pairkey", key, &n) == ESP_OK && n == BWP_KEY_LEN;
        nvs_close(h);
    }
    return ok;
}

esp_err_t pairkey_store(const uint8_t key[BWP_KEY_LEN])
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(h, "pairkey", key, BWP_KEY_LEN);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t pairkey_clear(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    nvs_erase_key(h, "pairkey");
    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

esp_err_t bridge_set_wifi(const char *ssid, size_t ssid_len, const char *pass)
{
    esp_err_t err = creds_store(ssid, ssid_len, pass);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "store: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Wi-Fi credentials stored (SSID %.*s)", (int)ssid_len, ssid);
    wifi_creds_changed();
    return ESP_OK;
}

esp_err_t bridge_clear_wifi(void)
{
    esp_err_t err = creds_clear();
    ESP_LOGI(TAG, "Wi-Fi credentials cleared");
    wifi_creds_changed();
    return err;
}
