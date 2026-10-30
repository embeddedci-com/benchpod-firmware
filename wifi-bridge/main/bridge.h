// BenchPod Wi-Fi bridge: interfaces between the firmware modules.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bwp.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "l2fwd.h"

// ---- Hardware pin map (benchpod-private pcb/wifi-bridge)
#define PIN_OSC_OE      32  // 50 MHz oscillator output enable, 10k pull-down (R14)
#define PIN_PHY_NRST    4   // LAN8742A nRST, 10k pull-down (R16)
#define PIN_MDC         23
#define PIN_MDIO        18
#define PIN_STATUS      13  // STATUS test pad TP8 (no LEDs on the board); GPIO14 and GPIO33 unused
#define PIN_BTN_FACTORY 34  // input only, 10k pull-up (R20), active low
#define PIN_BTN_PAIR    35  // input only, 10k pull-up (R41) + 100 nF (C27), active low

// ---- Shared forwarding state (main.c)
typedef struct {
    portMUX_TYPE lock;  // guards fwd
    l2fwd_t fwd;
    // Counters: each one is written by one task only (see bwp_counters_t for meanings).
    volatile uint32_t eth_rx, wifi_tx, wifi_tx_err, drop_eth;  // Ethernet rx task
    volatile uint32_t wifi_rx, eth_tx, eth_tx_err, drop_wifi;  // Wi-Fi task
} bridge_t;

extern bridge_t g_bridge;

// Copies the learned pod MAC; false if none yet.
bool bridge_pod_mac(uint8_t mac[6]);
void bridge_counters(bwp_counters_t *c);
// Collects state, flags, Wi-Fi details and counters for HELLO, STATUS and the console.
void bridge_status(bwp_status_t *s);
uint8_t bridge_flags(void);

// ---- board.c: buttons, STATUS test pad, RMII clock, safe reboot
void board_init(void);
void board_rmii_clock(bool on);
void board_reboot(void);         // stops the RMII clock first (GPIO0 is a boot strap)
void board_start_tasks(void);    // buttons + STATUS pad task

// ---- eth.c
esp_err_t eth_start(void);
esp_err_t eth_send(const uint8_t *frame, size_t len);
bool eth_link_up(void);
void eth_get_mac(uint8_t mac[6]);

// ---- wifi.c
esp_err_t wifi_start(void);
void wifi_pod_mac_changed(void);  // learned or changed pod MAC: (re)start the station with it
void wifi_pod_gone(void);         // Ethernet link down: Wi-Fi goes off if no pod comes back
void wifi_pod_back(void);         // the pod spoke on the control channel after a link-down
void wifi_creds_changed(void);    // stored credentials changed or were cleared
bool wifi_is_connected(void);
uint8_t wifi_state(void);         // bwp_state_t
// Fills SSID, channel, RSSI, BSSID and the last disconnect reason.
void wifi_fill_status(bwp_status_t *s);
// Blocking scan, strongest first. Returns BWP_OK or BWP_E_BUSY / BWP_E_INTERNAL.
bwp_result_t wifi_scan(bwp_ap_t *aps, size_t max, size_t *n);

// ---- creds.c: Wi-Fi credentials in NVS (encrypted NVS in production builds)
bool creds_load(char ssid[33], size_t *ssid_len, char pass[65]);
bool creds_present(void);
esp_err_t creds_store(const char *ssid, size_t ssid_len, const char *pass);
esp_err_t creds_clear(void);
// Clears credentials and tells the Wi-Fi module (button, console, control channel).
esp_err_t bridge_clear_wifi(void);
esp_err_t bridge_set_wifi(const char *ssid, size_t ssid_len, const char *pass);
// Pairing (control channel), same NVS namespace: the key shared with the pod and our X25519
// public key from that pairing (sent in HELLO, so the pod can recompute the key after a reboot).
bool pairkey_load(uint8_t key[BWP_KEY_LEN], uint8_t pub[32]);
esp_err_t pairkey_store(const uint8_t key[BWP_KEY_LEN], const uint8_t pub[32]);
esp_err_t pairkey_clear(void);

// ---- ota.c: shared by the control channel and the console
void ota_init(void);
bwp_result_t ota_begin(uint32_t size, const uint8_t *sig, size_t sig_len, const char **msg);
bwp_result_t ota_data(uint32_t offset, const uint8_t *data, size_t len, uint32_t *next, const char **msg);
bwp_result_t ota_end(const char **msg);
void ota_abort(void);
bool ota_active(void);
void ota_poll(void);              // aborts a transfer idle for more than 60 s
// Rollback guard: on a new image's first boot, starts the confirmation timer.
void ota_check_pending(void);
void ota_confirm(void);           // a valid control request from the pod arrived
bool ota_pending_verify(void);

// ---- control.c
void control_start(void);
void control_rx(uint8_t *frame, size_t len);  // takes ownership of the malloc'd frame
void control_notify(void);                    // state changed: send an unsolicited STATUS
void control_forget_pairing(void);            // PAIR button: the next pod that offers a key wins
bool control_paired(void);

// ---- console.c
void console_start(void);
