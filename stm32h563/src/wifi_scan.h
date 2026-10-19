#ifndef WIFI_SCAN_H
#define WIFI_SCAN_H

/*
 * wifi_scan: one list of Wi-Fi networks, whichever radio found them (the built-in ESP32-C3 or,
 * on development builds, a Wi-Fi bridge box), so `wifi_scan` and the console `wifi-scan` answer
 * the same way on every pod.
 *
 * The list holds the strongest WIFI_SCAN_MAX networks, strongest first, one entry per SSID (the
 * strongest access point of a network that has several); hidden networks (empty SSID) are kept
 * apart. Portable (host-tested in test_wifi_scan.c).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bp_json.h"

#define WIFI_SCAN_MAX 20u   /* the Wi-Fi bridge's own cap (BWP_SCAN_MAX_APS) */

typedef struct {
    char    ssid[33];
    int8_t  rssi;           /* dBm */
    uint8_t channel;
    uint8_t authmode;       /* ESP-IDF wifi_auth_mode_t (the bridge uses the same numbers) */
} wifi_scan_ap_t;

typedef struct {
    uint8_t        n;
    uint16_t       seen;    /* access points reported, before the cap and the merge per SSID */
    wifi_scan_ap_t ap[WIFI_SCAN_MAX];
} wifi_scan_list_t;

void wifi_scan_list_clear(wifi_scan_list_t *l);

/* Adds one access point: merged into its SSID's entry (the stronger wins), kept sorted by RSSI,
   the weakest dropped once the list is full. */
void wifi_scan_list_add(wifi_scan_list_t *l, const char *ssid, size_t ssid_len, int rssi,
                        unsigned channel, unsigned authmode);

/* "open", "wep", "wpa", "wpa2", "wpa/wpa2", "wpa2-ent", "wpa3", "wpa2/wpa3", "wapi", "owe" or
   "other". */
const char *wifi_scan_auth_str(unsigned authmode);

/* Appends "networks":[{"ssid":..,"rssi":..,"channel":..,"auth":..},...] to e, stopping before an
   entry that would take e past `limit` bytes (the cloud reply cap minus what the caller still
   appends after the list). Returns how many it wrote. */
unsigned wifi_scan_emit_networks(bp_emit_t *e, const wifi_scan_list_t *l, size_t limit);

#endif /* WIFI_SCAN_H */
