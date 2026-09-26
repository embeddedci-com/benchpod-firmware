#ifndef ESP_WIFI_CTRL_H
#define ESP_WIFI_CTRL_H

#include <stdint.h>
#include <stdbool.h>

/* ---- Wi-Fi control over the esp-hosted RPC channel -------------------------
 *
 * Drives the ESP32-C3 through the esp-hosted protobuf RPC (on ESP_SERIAL_IF) to
 * associate with the configured AP, and reflects the link state into the Wi-Fi
 * netif (esp_netif). Hand-rolled minimal RPC (Init → SetConfig(STA) → Start →
 * Connect → GetMac) — see esp_wifi_ctrl.c. Credentials come from config_store
 * (ssid/password). Polled from the net task.
 *
 * If no SSID is configured, this stays idle and the ESP32 is never started, so
 * an un-provisioned unit runs Ethernet-only.
 * ---------------------------------------------------------------------------*/

void esp_wifi_ctrl_init(void);

/* Advance the connect state machine: brings the ESP32 up + runs the RPC sequence
   when configured, retries on failure/disconnect. No-op when unconfigured. */
void esp_wifi_ctrl_poll(void);

/* True if a Wi-Fi SSID is provisioned (config_store). */
bool esp_wifi_ctrl_configured(void);

/* True once associated (sta-connected event seen). */
bool esp_wifi_ctrl_connected(void);

/* Re-read credentials and restart the sequence (after a wifi-set). */
void esp_wifi_ctrl_reload(void);

/* Console C3 commands (flash-esp32, flash-esp32-sync) own the C3's EN/BOOT straps while they
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

#endif /* ESP_WIFI_CTRL_H */
