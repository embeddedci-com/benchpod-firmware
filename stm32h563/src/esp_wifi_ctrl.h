#ifndef ESP_WIFI_CTRL_H
#define ESP_WIFI_CTRL_H

#include <stdint.h>
#include <stdbool.h>

#include "wifi_scan.h"

/* ---- Wi-Fi control over the esp-hosted RPC channel -------------------------
 *
 * Drives the ESP32-C3 through the esp-hosted protobuf RPC (on ESP_SERIAL_IF) to
 * associate with the configured AP, and reflects the link state into the Wi-Fi
 * netif (esp_netif). Hand-rolled minimal RPC (Init → SetStorage(RAM) → SetMode →
 * SetConfig(STA) → Start → Connect → GetMac) — see esp_wifi_ctrl.c. Credentials
 * come from config_store (ssid/password) and are re-sent on every bring-up; the
 * C3 holds them in RAM only. Polled from the net task.
 *
 * If no SSID is configured, this stays idle and the ESP32 is never started, so
 * an un-provisioned unit runs Ethernet-only.
 * ---------------------------------------------------------------------------*/

void esp_wifi_ctrl_init(void);

/* The board has the ESP32-C3 at all (board_variant.h board_has_wifi_module; digital v3r3 has
   none).  Set before esp_wifi_ctrl_init().  Without it: never configured (USB networking stays
   on), the C3 pins are never driven, nothing is flashed, the state is "absent" and a scan ends
   with ESP_WIFI_NO_MODULE. */
#define ESP_WIFI_NO_MODULE "no Wi-Fi module: this BenchPod has no built-in Wi-Fi"
void esp_wifi_ctrl_set_module_present(bool present);
bool esp_wifi_ctrl_module_present(void);

/* Advance the connect state machine: brings the ESP32 up + runs the RPC sequence
   when configured, retries on failure/disconnect. No-op when unconfigured. */
void esp_wifi_ctrl_poll(void);

/* True if a Wi-Fi SSID is provisioned (config_store) and the C3 is fitted. */
bool esp_wifi_ctrl_configured(void);

/* True once associated (sta-connected event seen). */
bool esp_wifi_ctrl_connected(void);

/* Re-read credentials and restart the sequence (after a wifi-set). */
void esp_wifi_ctrl_reload(void);

/* C3 commands that use the ROM loader (flash-esp32, flash-esp32-sync, wifi-clear) own the C3's EN/BOOT straps while they
   run: pause(true) stops the state machine touching them, pause(false) releases it. */
void esp_wifi_ctrl_pause(bool paused);

/* Worker task: the C3 flash that Wi-Fi control asked for has finished (ok = flashed and
   verified).  Safe to call from any task. */
void esp_wifi_ctrl_flash_done(bool ok);

/* Human-readable state for the `status` command. */
const char *esp_wifi_ctrl_state_str(void);

/* Last measured STA RSSI in dBm (polled while connected, and captured at each
   disconnect). Returns true and fills *dbm iff a reading is available. */
bool esp_wifi_ctrl_rssi(int *dbm);

/* How many times the link was torn down and the C3 restarted because the C3 itself
   went away while connected or connecting: `no_response` = RSSI_MAX_MISSES GetRssi
   requests in a row went unanswered (crashed/hung C3, stuck HANDSHAKE); `rebooted` =
   the C3 announced a new boot mid-session.  Since power-on.  Either pointer may be NULL. */
void esp_wifi_ctrl_slave_lost_counts(uint32_t *no_response, uint32_t *rebooted);

/* A new ESP32-C3 image was installed into its W25Q slot: a C3 that never booted gets another
   automatic flash attempt. */
void esp_wifi_ctrl_image_installed(void);

/* ---- Wi-Fi scan through the ESP32-C3 ----
   Joined: the C3 scans alongside the connection (it stays up).  Not configured, or after a
   failed connect: the C3 is started for the scan alone and held in reset again afterwards.
   Mid-connect: the request waits for the connect to end (up to 30 s).  The scan runs on the net
   task (esp_wifi_ctrl_poll); asking and reading never wait. */
typedef enum {
    ESP_WIFI_SCAN_UNKNOWN = 0,   /* no such request */
    ESP_WIFI_SCAN_RUNNING,
    ESP_WIFI_SCAN_DONE,          /* *list holds the networks */
    ESP_WIFI_SCAN_FAILED,        /* *err says why */
} esp_wifi_scan_state_t;

/* Worker task: ask for a scan.  Returns its id (never 0); a request made while another is
   waiting or running shares that one's id and answer. */
uint32_t esp_wifi_ctrl_scan_request(void);

/* Worker task: where request `id` stands.  The list stays valid until the next request. */
esp_wifi_scan_state_t esp_wifi_ctrl_scan_result(uint32_t id, const wifi_scan_list_t **list,
                                                const char **err);

#endif /* ESP_WIFI_CTRL_H */
