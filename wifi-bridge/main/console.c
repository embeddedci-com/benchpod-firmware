// Line-based console on UART0 (CP2102N, USB-C), 115200 8N1: setup without a pod and recovery.
// Every command ends with a line "OK" or "ERR <message>"; data lines come before it. Log lines
// (starting "I (", "W (", "E (") can interleave; `log off` silences them for scripted use.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bridge.h"
#include "bridge_util.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "mbedtls/base64.h"
#include "sdkconfig.h"

#define UART      CONFIG_ESP_CONSOLE_UART_NUM
#define CON_LINE_MAX  1600  // fits "ota-data <offset> <base64 of 1024 bytes>"
#define MAX_ARGS  4

static char s_line[CON_LINE_MAX];
static uint8_t s_bin[BWP_OTA_CHUNK_MAX + 4];
static bool s_echo = true;

static const char *STATE_NAMES[] = {"unconfigured", "wait-pod", "joining", "connected", "auth-failed", "no-ap"};

static const char *result_name(bwp_result_t r)
{
    static const char *names[] = {"ok", "bad-request", "unsupported", "busy", "storage", "ota-not-started",
                                  "ota-bad-offset", "ota-verify", "ota-too-large", "internal", "ota-wrong-image"};
    return r < sizeof(names) / sizeof(names[0]) ? names[r] : "error";
}

static void ok(void)
{
    printf("OK\n");
}

static void err(const char *fmt, const char *a)
{
    printf("ERR ");
    printf(fmt, a ? a : "");
    printf("\n");
}

static void result(bwp_result_t r, const char *msg)
{
    if (r == BWP_OK) {
        ok();
    } else {
        printf("ERR %s%s%s\n", result_name(r), msg ? ": " : "", msg ? msg : "");
    }
}

static void cmd_status(void)
{
    bwp_status_t s;
    char m[18];
    bridge_status(&s);
    printf("state: %s\n", s.state < 6 ? STATE_NAMES[s.state] : "?");
    printf("ssid: %s\n", s.ssid);
    if (s.state == BWP_ST_CONNECTED) {
        bu_mac_str(s.bssid, m);
        printf("channel: %u\nrssi: %d\nbssid: %s\n", s.channel, s.rssi, m);
    }
    if (s.reason) {
        printf("last-disconnect-reason: %u\n", s.reason);
    }
    printf("ethernet: %s\n", (s.flags & BWP_F_ETH_LINK) ? "up" : "down");
    if (s.flags & BWP_F_POD_KNOWN) {
        bu_mac_str(s.pod_mac, m);
        printf("pod-mac: %s\n", m);
    } else {
        printf("pod-mac: none\n");
    }
    uint8_t mac[6];
    eth_get_mac(mac);
    bu_mac_str(mac, m);
    printf("bridge-mac: %s\n", m);
    printf("ota: %s\n", (s.flags & BWP_F_OTA_ACTIVE) ? "in-progress" : "idle");
    printf("uptime: %lu\n", (unsigned long)s.uptime_s);
    printf("counters: eth_rx=%lu wifi_tx=%lu wifi_tx_err=%lu wifi_rx=%lu eth_tx=%lu eth_tx_err=%lu dropped=%lu\n",
           (unsigned long)s.counters.eth_rx, (unsigned long)s.counters.wifi_tx,
           (unsigned long)s.counters.wifi_tx_err, (unsigned long)s.counters.wifi_rx,
           (unsigned long)s.counters.eth_tx, (unsigned long)s.counters.eth_tx_err,
           (unsigned long)s.counters.dropped);
    ok();
}

static void cmd_scan(void)
{
    static bwp_ap_t aps[BWP_SCAN_MAX_APS];
    static const char *auth[] = {"open", "wep", "wpa", "wpa2", "wpa/wpa2", "wpa2-ent", "wpa3", "wpa2/wpa3"};
    size_t n;
    bwp_result_t r = wifi_scan(aps, BWP_SCAN_MAX_APS, &n);
    if (r != BWP_OK) {
        result(r, NULL);
        return;
    }
    char m[18];
    for (size_t i = 0; i < n; i++) {
        bu_mac_str(aps[i].bssid, m);
        printf("%4d %2u %-9s %s %s\n", aps[i].rssi, aps[i].channel,
               aps[i].authmode < 8 ? auth[aps[i].authmode] : "other", m, aps[i].ssid);
    }
    ok();
}

static void cmd_set_wifi(int argc, char **argv)
{
    if (argc < 2 || argc > 3) {
        err("usage: set-wifi <ssid> [passphrase]  (quote values with spaces)", NULL);
        return;
    }
    const char *pass = argc == 3 ? argv[2] : "";
    size_t ssid_len = strlen(argv[1]);
    if (!bwp_ssid_valid((const uint8_t *)argv[1], ssid_len)) {
        err("bad-request: SSID must be 1-32 bytes", NULL);
    } else if (!bwp_passphrase_valid(pass, strlen(pass))) {
        err("bad-request: passphrase must be empty, 8-63 printable ASCII or 64 hex digits", NULL);
    } else if (ota_active()) {
        err("busy: OTA in progress", NULL);
    } else if (bridge_set_wifi(argv[1], ssid_len, pass) != ESP_OK) {
        err("storage", NULL);
    } else {
        ok();
    }
}

static void cmd_ota(int argc, char **argv)
{
    const char *msg = NULL;
    bwp_result_t r;
    if (strcmp(argv[0], "ota-begin") == 0 && argc == 2) {
        r = ota_begin(strtoul(argv[1], NULL, 0), &msg);
    } else if (strcmp(argv[0], "ota-data") == 0 && argc == 3) {
        size_t n = 0;
        uint32_t next = 0;
        if (mbedtls_base64_decode(s_bin, sizeof(s_bin), &n, (const unsigned char *)argv[2], strlen(argv[2])) != 0 ||
            n > BWP_OTA_CHUNK_MAX) {
            err("bad-request: DATA must be base64 of 1-1024 bytes", NULL);
            return;
        }
        r = ota_data(strtoul(argv[1], NULL, 0), s_bin, n, &next, &msg);
        printf("next: %lu\n", (unsigned long)next);
    } else if (strcmp(argv[0], "ota-end") == 0 && argc == 1) {
        r = ota_end(&msg);
    } else if (strcmp(argv[0], "ota-abort") == 0 && argc == 1) {
        ota_abort();
        r = BWP_OK;
    } else {
        err("usage: ota-begin <size> | ota-data <offset> <base64> | ota-end | ota-abort", NULL);
        return;
    }
    result(r, msg);
}

static void cmd_help(void)
{
    printf("help                        this list\n"
           "version                     model, firmware version, hardware revision\n"
           "status                      Wi-Fi state, SSID, RSSI, pod MAC, counters\n"
           "scan                        nearby networks: rssi channel auth bssid ssid\n"
           "set-wifi <ssid> [pass]      store credentials and connect (quote values with spaces)\n"
           "clear-wifi                  erase credentials and disconnect\n"
           "reboot                      restart the bridge\n"
           "factory-reset               erase credentials and restart\n"
           "log on|off                  show or hide log lines\n"
           "echo on|off                 echo typed characters\n"
           "ota-begin <size>            firmware update over this console (signed image)\n"
           "ota-data <offset> <base64>  up to 1024 bytes per line, in order\n"
           "ota-end                     verify the image; boots after reboot\n"
           "ota-abort\n");
    ok();
}

static void execute(char *line)
{
    char *argv[MAX_ARGS];
    int argc = bu_tokenize(line, argv, MAX_ARGS);
    if (argc == 0) {
        return;
    }
    if (argc < 0) {
        err(argc == -1 ? "unterminated quote" : "too many arguments", NULL);
        return;
    }
    const char *c = argv[0];
    if (strcmp(c, "help") == 0 || strcmp(c, "?") == 0) {
        cmd_help();
    } else if (strcmp(c, "version") == 0) {
        printf("%s %s hw %s\n", BWP_MODEL, esp_app_get_description()->version, CONFIG_BRIDGE_HW_REV);
        ok();
    } else if (strcmp(c, "status") == 0) {
        cmd_status();
    } else if (strcmp(c, "scan") == 0) {
        cmd_scan();
    } else if (strcmp(c, "set-wifi") == 0) {
        cmd_set_wifi(argc, argv);
    } else if (strcmp(c, "clear-wifi") == 0) {
        bridge_clear_wifi() == ESP_OK ? ok() : err("storage", NULL);
    } else if (strcmp(c, "reboot") == 0) {
        ok();
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(100));
        board_reboot();
    } else if (strcmp(c, "factory-reset") == 0) {
        creds_clear() == ESP_OK ? ok() : err("storage", NULL);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(100));
        board_reboot();
    } else if (strcmp(c, "log") == 0 && argc == 2) {
        esp_log_level_set("*", strcmp(argv[1], "off") == 0 ? ESP_LOG_NONE : ESP_LOG_INFO);
        ok();
    } else if (strcmp(c, "echo") == 0 && argc == 2) {
        s_echo = strcmp(argv[1], "off") != 0;
        ok();
    } else if (strncmp(c, "ota-", 4) == 0) {
        cmd_ota(argc, argv);
    } else {
        err("unknown command %s (try help)", c);
    }
}

static void console_task(void *arg)
{
    size_t n = 0;
    bool overflow = false;
    for (;;) {
        uint8_t ch;
        if (uart_read_bytes(UART, &ch, 1, portMAX_DELAY) != 1) {
            continue;
        }
        if (ch == '\r' || ch == '\n') {
            if (s_echo) {
                printf("\n");
            }
            s_line[n] = '\0';
            if (overflow) {
                err("line too long", NULL);
            } else {
                execute(s_line);
            }
            memset(s_line, 0, n);  // may hold a passphrase
            n = 0;
            overflow = false;
            fflush(stdout);
        } else if (ch == 0x08 || ch == 0x7f) {
            if (n) {
                n--;
                if (s_echo) {
                    printf("\b \b");
                    fflush(stdout);
                }
            }
        } else if (ch >= 0x20 && ch < 0x7f) {
            if (n < CON_LINE_MAX - 1) {
                s_line[n++] = (char)ch;
                if (s_echo) {
                    putchar(ch);
                    fflush(stdout);
                }
            } else {
                overflow = true;
            }
        }
    }
}

void console_start(void)
{
    ESP_ERROR_CHECK(uart_driver_install(UART, 2048, 0, 0, NULL, 0));
    uart_vfs_dev_use_driver(UART);
    setvbuf(stdin, NULL, _IONBF, 0);
    xTaskCreate(console_task, "console", 4096, NULL, 3, NULL);
}
