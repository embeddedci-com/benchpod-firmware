/*
 * console.c — bench-pod STM32 bring-up console.
 *
 * console_exec() runs one command line and writes results through an output
 * sink, so the same command set serves both the local USB/USART console and the
 * TCP command server (net_server.c).  The full RP2350 command_handler/SCPI/JSON
 * surface lands in later phases.
 */
#include "console.h"
#include "console_moved.h"
#include "usb_device.h"
#include "usb_netif.h"
#include "adc_pool.h"   /* gated diagnostics use the shared pool's scratch */
#include "console_io.h"
#include "signal_engine.h"
#include "board_info.h"       /* board caps (ADC/DAC bits, channels) for `status` */
#include "fpga_config.h"      /* FPGA_DAC_REPLAY_MAX_SAMPLES for `status` */
#include "version.h"          /* FIRMWARE_VERSION for `status` */
#include "i2c_bus.h"
#include "ina238.h"
#include "cal_data.h"
#include <math.h>
#include "target_power.h"
#include "can_bus.h"
#include "psram.h"
#include "ice40_flash.h"
#include "boot_guard.h"
#include "board_uid.h"
#include "FreeRTOS.h"
#include "task.h"

bool clock_on_hsi(void);   /* main.c */
#include "command_handler.h"   /* mirror re-sync after a reconfiguration */
#include "command_handler_internal.h"   /* heavy_in_flight: calibrate must not cut a capture */
#include "esp_rom_flash.h"
#include "esp_rom_job.h"     /* esp_rom_result_str */
#include "bp_log.h"
#include "esp_hosted_spi.h"   /* stop the Wi-Fi transport before flashing the C3 */
#include "esp_wifi_ctrl.h"    /* wifi-set / wifi-show provisioning */
#include "config_store.h"
#include "stm32h5xx_hal.h"
#include "hw_lock.h"
#include "dfu_boot.h"
#include "fault.h"
#include "watchdog.h"
#include "sys_health.h"
#include "board_rev.h"
#include "usb_cc.h"
#include "nrst_ctrl.h"
#include "hw_worker.h"
#include "la_pins.h"          /* la-voltage refusal while pins are in use */
#include "net_server.h"       /* net_ip_str() for `status` */
#include "cloud_config.h"     /* provisioned cloud registration, for `status` */
#include "cloud_client.h"     /* cloud_client_state_str() for `status` */
#include "board_pins.h"      /* ESP_EN (esp-reset-pulse) */
#include "pico/time.h"
#include "pico/rand.h"        /* get_rand_32 — hardware RNG liveness check */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "dac_limits.h"
#include "adc_cal.h"
#include "current_out.h"
#include <stdlib.h>

#include "blob_store.h"
#include "board_variant.h"
#include "device_identity.h"   /* identity / identity-wipe */
#include "b64url.h"
#include "w25q.h"
#include "ota.h"
#include "flash_layout.h"
#include "pod_policy.h"
#include "cloud_extras.h"
#include "fw_sign.h"
#include "lan_tls.h"
#include "lwip/stats.h"    /* lwIP memory high-water marks in status */
#include "lwip/memp.h"
#include "status_led.h"

#if defined(BENCHPOD_WIFI_BRIDGE)
#include "wifi_bridge_net.h"
#include "wifi_manager.h"
#endif


/* Reprogram the config flash with a specific gateware image and reconfigure — the runtime
 * IMAGE SWITCH.  SB_WARMBOOT cannot reconfigure at runtime on this board (its config-SPI
 * pins are the shared PSRAM bus), so switching copies the selected image from its W25Q blob
 * slot (blob_store.h) to offset 0 of the same flash via the proven ice40_flash_program +
 * CRESET-reconfig path (~2 s).  n: 0=closed-loop, 1=deep-DAC-replay.  Returns 0 ok, -1 fail. */
uint8_t ice40_embedded_gw_version(void) { return (uint8_t)blob_manifest_gw_version(); }

int ice40_reflash_image(int n)
{
    blob_id_t id = (n == 1) ? BLOB_GW1 : BLOB_GW0;
    if (!blob_store_present(id)) {
        printf("[ice40] no gateware image %d in the W25Q (slot %s is empty)\n", n, blob_name(id));
        return -1;
    }
    /* Never copy a damaged slot over the image the iCE40 boots from. */
    w25q_open();
    int intact = blob_store_verify(id) == 0;
    w25q_close();
    if (!intact) {
        printf("[ice40] gateware slot %s does not match its checksum: not flashed\n", blob_name(id));
        return -1;
    }
    /* Runtime image-swap sequence.  The DEEP image's PSRAM-write wedge is fixed IN THE GATEWARE
       (gw v26: the deep reader/arbiter are held in reset while the STM32 owns the bus, so the
       psram_init() bus-yank below can no longer desync them).  So keep the firmware ordering that
       reads the config with the PSRAM in SPI — the safest state for the shared-bus bitstream read:
         psram_reset_to_spi() -> program+boot (config read, PSRAM in SPI) -> psram_init() (QPI for
         the new gateware) -> psram_bus_release() (hand the bus to the iCE40).
       NB: an earlier attempt to run psram_init() BEFORE the boot (so the config read saw a QPI
       PSRAM) intermittently CORRUPTED the deep bitstream — reverted. */
    psram_reset_to_spi();
    int rc = ice40_flash_program_src(blob_store_read, (void *)(uintptr_t)id, blob_store_info(id)->len);
    psram_init();                       /* re-enter QPI for the new gateware (bus-yank now safe: gw v26) */
    psram_bus_handover();               /* hand the shared bus back to the iCE40 */
    /* The fabric just reset every register it owns; the firmware's MIRRORS of those registers did
       not.  Re-sync them here — this is the one point every reconfiguration passes through (image
       swap, this console command, the boot/OTA reflash).  Unconditional: even a FAILED program
       leaves the part reconfigured or held in reset, so the mirrors are stale either way. */
    signal_engine_on_gateware_reconfigured();
    command_handler_on_gateware_reconfigured();
    return rc;
}

#define CONSOLE_LINE_MAX 128

/* Formatted output to a sink. */
static void op(console_out_t out, void *ctx, const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) out(ctx, buf);
}

static void cmd_help(console_out_t out, void *ctx)
{
    /* Emit straight through out() (not op()): the list is longer than op()'s 160-byte format
       buffer. Split into a few writes to stay well under the USB-CDC TX ring.
       The console keeps what needs physical presence or works without the network; everything
       else is the JSON API (docs/API.md), on the USB network adapter too. */
    out(ctx,
        "commands (the rest is the JSON API on :8080, also over the USB network):\r\n"
        "  help                 this list\r\n"
        "  status               pod report: network, USB, PSRAM, gateware, safe mode, health\r\n"
        "  ping                 SPI ping the iCE40 (expect 0xA5 + version)\r\n"
        "  uid                  the chip's unique ID\r\n"
        "  selftest             silicon health check (clocks/timer/sram/rng)\r\n"
        "  reboot               system reset\r\n"
        "  dfu                  reboot into the USB DFU bootloader to reflash firmware\r\n"
        "  usb-reattach         re-enumerate on USB (console + network adapter)\r\n"
        "  identity | identity-wipe <id|unknown>      show the device key; erase it and make a new one\r\n"
        "  sig-policy [audit|permissive|required]    what OTA accepts (USB may loosen it)\r\n"
        "  lan-policy [open|locked|off]               what the external LAN may do (USB stays on)\r\n"
        "  lan-tls [https-only on|off | regenerate]   the LAN API's TLS certificate (port 8443)\r\n"
        "  cloud-ca | cloud-ca-clear                  company CA (install it over the network: benchpod cloud ca set)\r\n"
        "  cloud-proxy | cloud-proxy-set <host:port> [user pass] | cloud-proxy-clear   HTTP proxy for the cloud link\r\n"
        "                       (ca, ca-clear, proxy, proxy-set, proxy-clear still work)\r\n");
    out(ctx,
        "  eth <stop|start|restart|stats>  the wired link: down/up (PHY reset + DHCP), counters\r\n"
        "  wifi-set \"<ssid>\" \"<pass>\"  save Wi-Fi credentials + (re)connect the C3 (USB networking off)\r\n"
        "  wifi-show            show stored SSID, Wi-Fi state, IP\r\n"
        "  wifi-scan            networks the pod's Wi-Fi sees: scan <rssi> <channel> <auth> <ssid>, scan done <n>\r\n"
        "  wifi-clear           erase stored Wi-Fi credentials (pod + ESP32-C3; USB networking on)\r\n"
        "  wifi-static <ip> <mask> <gw>  static Wi-Fi address (diagnostic)\r\n"
        "  esp-reset-pulse      diagnostic: reset the C3 unannounced (Wi-Fi must recover)\r\n"
        "  esp-mon [ms]         print the C3's UART output\r\n"
        "  flash-esp32-sync     C3: strap download mode + SYNC (wiring test)\r\n"
        "  flash-esp32          C3: flash the esp-hosted image from the W25Q (background, ~2 min)\r\n"
        "  flash-ice40          reflash the iCE40 with gateware image 0 from the W25Q\r\n"
        "  psram-selftest       layered STM32/iCE40->PSRAM + /CE-net reach test (boot test)\r\n"
        "  led <colour|mask>    show red, green, blue, cyan, magenta, yellow, white, off or a 0-7 mask for 5 s\r\n"
        "  psram-test           bring up + pattern-test the XSPI PSRAM\r\n"
        "  blobs                what the W25Q blob slots (gw0, gw1, esp) hold\r\n"
        "  (firmware and blob uploads: the JSON API, also over the USB network; recovery: dfu)\r\n");
#if !defined(BENCHPOD_RELEASE)
    out(ctx,
        "development build only:\r\n"
        "  test-bootloop [net|hw] yes | test-hang net|hw yes | test-crash yes   prove safe mode / crash reports\r\n"
        "  i2c-scan | ina | expdump   I2C bus scan, INA monitors, expander dump\r\n"
        "  eth speed|refclk|loopback  wired-link diagnostics\r\n"
        "  psram | psram-addrtest | psram-bench [kb] | psram-clk <p> | psram-chunk <b>   PSRAM diagnostics\r\n"
        "  cap-selftest [n] | capture-psram [n] | dualcap [an] [ln] | lastress [n]   capture-path diagnostics\r\n"
        "  spi-clk <128|256> | spi-diag [n] | adc-spi [n] | adcraw | flash-id | ca-damage   low-level debug\r\n"
        "  ice40-erase [gw0] yes   blank the iCE40 config flash (and slot gw0), to test a new board's boot\r\n");
#endif
#if defined(BENCHPOD_WIFI_BRIDGE)
    out(ctx,
        "Wi-Fi bridge (development build only; wifi-set/-show/-clear/-scan use it when one is plugged in):\r\n"
        "  bridge | bridge scan | bridge update | bridge reboot | bridge factory-reset yes\r\n"
        "  (the pod updates the bridge by itself when blob slot \"bridge\" holds a newer image)\r\n");
#endif
}

#if !defined(BENCHPOD_RELEASE)   /* development builds only: `ina` */
static void cmd_ina(console_out_t out, void *ctx)
{
    int bus_mv, sh_uv, cur_ua;
    uint16_t id;
    if (ina238_read_id(I2C_ADDR_INA238_INTERNAL, &id) == 0 &&
        ina238_read(I2C_ADDR_INA238_INTERNAL, &bus_mv, &sh_uv, &cur_ua) == 0)
        op(out, ctx, "  int(0x40) %s id=0x%04x  bus=%d mV  shunt=%d uV  I=%d uA\r\n",
           ina_chip_name(ina238_chip(I2C_ADDR_INA238_INTERNAL)), id, bus_mv, sh_uv, cur_ua);
    else
        op(out, ctx, "  int(0x40) no response\r\n");
    if (ina238_read(I2C_ADDR_INA238_EXTERNAL, &bus_mv, &sh_uv, &cur_ua) == 0)
        op(out, ctx, "  ext(0x44) %s bus=%d mV  shunt=%d uV  I=%d uA\r\n",
           ina_chip_name(ina238_chip(I2C_ADDR_INA238_EXTERNAL)), bus_mv, sh_uv, cur_ua);
    else
        op(out, ctx, "  ext(0x44) no response (needs external supply)\r\n");
    if (ina_pod_present()) {
        if (ina238_read(I2C_ADDR_INA_POD, &bus_mv, &sh_uv, &cur_ua) == 0)
            op(out, ctx, "  pod(0x41) %s bus=%d mV  shunt=%d uV  I=%d uA\r\n",
               ina_chip_name(ina238_chip(I2C_ADDR_INA_POD)), bus_mv, sh_uv, cur_ua);
        else
            op(out, ctx, "  pod(0x41) no response\r\n");
    }
}
#endif

/* ---- silicon self-test --------------------------------------------------
 * 'selftest' — a quick health check of the STM32H563 silicon itself: the HSE
 * crystal, PLL1 lock, the 250 MHz system clock, the SysTick timebase, on-chip
 * SRAM, and the hardware RNG.  Each line is PASS/FAIL with a measured value, and
 * a summary at the end reports overall health.  Intended for bringing up a
 * suspect board; it deliberately does NOT touch the FPGA/PSRAM/flash (use 'ping'
 * / 'psram-test' for those).  Mirrors the RP2350 console selftest.
 * --------------------------------------------------------------------------*/

/* Walking-pattern test over a small static SRAM buffer: catches stuck/aliased
   data bits without disturbing the rest of RAM. */
static bool selftest_sram(void)
{
    static volatile uint32_t buf[64];
    const uint32_t pats[] = { 0x00000000u, 0xFFFFFFFFu, 0xAAAAAAAAu, 0x55555555u };
    for (size_t p = 0; p < sizeof(pats) / sizeof(pats[0]); p++) {
        for (size_t i = 0; i < 64; i++) buf[i] = pats[p];
        for (size_t i = 0; i < 64; i++) if (buf[i] != pats[p]) return false;
    }
    /* Address-unique pattern: catches address aliasing. */
    for (size_t i = 0; i < 64; i++) buf[i] = (uint32_t)(i * 0x9E3779B1u);
    for (size_t i = 0; i < 64; i++) if (buf[i] != (uint32_t)(i * 0x9E3779B1u)) return false;
    return true;
}

static void cmd_selftest(console_out_t out, void *ctx)
{
    int pass = 0, fail = 0;

#define ST(name, cond, ...) do {                                       \
        bool _ok = (cond);                                             \
        op(out, ctx, "  %-4s %-7s ", _ok ? "PASS" : "FAIL", name);     \
        op(out, ctx, __VA_ARGS__);                                     \
        op(out, ctx, "\r\n");                                          \
        if (_ok) pass++; else fail++;                                  \
    } while (0)

    op(out, ctx, "STM32H563 self-test:\r\n");

    /* 1. HSE crystal (25 MHz) ready — the whole clock tree hangs off it. */
    bool hse = __HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != 0;
    ST("hse", hse, "25 MHz crystal %s", hse ? "ready" : "NOT ready");

    /* 2. PLL1 locked. */
    bool pll = __HAL_RCC_GET_FLAG(RCC_FLAG_PLL1RDY) != 0;
    ST("pll1", pll, "%s", pll ? "locked" : "NOT locked");

    /* 3. SYSCLK driven by PLL1 at ~250 MHz (+/-1%).  This is only correct if the
          25 MHz HSE crystal itself is correct, so it doubles as a crystal check. */
    uint32_t sysclk  = HAL_RCC_GetSysClockFreq();
    bool     src_pll = __HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK;
    uint32_t want = 250000000u, tol = want / 100u;
    bool sys_ok = src_pll &&
                  (sysclk > want ? sysclk - want : want - sysclk) <= tol;
    ST("sysclk", sys_ok, "%lu Hz (expect ~250M, src=%s)",
       (unsigned long)sysclk, src_pll ? "PLL1" : "other");

    /* 4. SysTick timebase is advancing (not stuck). */
    uint32_t t0 = HAL_GetTick();
    volatile uint32_t spin = 0;
    for (uint32_t i = 0; i < 2000000u; i++) spin++;
    uint32_t t1 = HAL_GetTick();
    ST("timer", t1 != t0, "tick advanced %lu ms over busy loop",
       (unsigned long)(t1 - t0));

    /* 5. On-chip SRAM read/write integrity (small region). */
    ST("sram", selftest_sram(), "walking-bit + address pattern");

    /* 6. Hardware RNG liveness — samples must not be all-zero or all-identical
          (also brings up HSI48, the RNG kernel clock). */
    uint32_t r0 = get_rand_32();
    bool all_zero = (r0 == 0u), all_same = true;
    for (int i = 0; i < 7; i++) {
        uint32_t r = get_rand_32();
        if (r != 0u) all_zero = false;
        if (r != r0) all_same = false;
    }
    ST("rng", !all_zero && !all_same, "sample 0x%08lx", (unsigned long)r0);

#undef ST
    op(out, ctx, "selftest: %d passed, %d failed -- %s\r\n",
       pass, fail, fail == 0 ? "HEALTHY" : "PROBLEMS DETECTED");
}

/* Extract one argument, honoring "double quotes" (so an SSID/password may
   contain spaces).  The benchpod-cli sends both args double-quoted. Returns the
   position just past the parsed argument. */
static bool s_parse_truncated;   /* the last parse_quoted value did not fit */

static const char *parse_quoted(const char *p, char *out, size_t n)
{
    while (*p == ' ' || *p == '\t') p++;
    size_t i = 0, len = 0;
    if (*p == '"') {
        p++;
        while (*p && *p != '"') { if (i < n - 1) out[i++] = *p; p++; len++; }
        if (*p == '"') p++;
    } else {
        while (*p && *p != ' ' && *p != '\t') { if (i < n - 1) out[i++] = *p; p++; len++; }
    }
    out[i] = '\0';
    s_parse_truncated = (len > i);
    return p;
}

#if defined(BENCHPOD_WIFI_BRIDGE)

/* ---- Wi-Fi bridge (dev builds): wifi-set / wifi-show / wifi-clear go to the external bridge box
   when one says HELLO on the Ethernet port; `bridge` shows it and runs scan / reboot / reset. */

static bool bridge_failed(console_out_t out, void *ctx, const char *what, int rc, const char *err)
{
    if (rc != 0) {
        op(out, ctx, "  %s: %s\r\n", what, err);
        return true;
    }
    if (g_wifi_bridge_reply.result != BWP_OK) {
        op(out, ctx, "  %s: %s%s%s\r\n", what, wb_result_str(g_wifi_bridge_reply.result),
           g_wifi_bridge_reply.message[0] ? ": " : "", g_wifi_bridge_reply.message);
        return true;
    }
    return false;
}

static void bridge_show(console_out_t out, void *ctx)
{
    wb_info_t i;
    wifi_bridge_get_info(&i);
    op(out, ctx, "  ssid: %s\r\n", i.have_status ? i.ssid : "");
    op(out, ctx, "  state: %s\r\n", i.present ? wb_state_str(i.state) : "no bridge");
    op(out, ctx, "  ip: %s\r\n", wifi_get_ip());
    if (i.have_status && i.state == BWP_ST_CONNECTED)
        op(out, ctx, "  rssi: %d dBm\r\n", i.rssi);
    else
        op(out, ctx, "  rssi: \r\n");
    op(out, ctx, "  via: wifi-bridge %02x:%02x:%02x:%02x:%02x:%02x fw %s hw %s, %s\r\n", i.mac[0], i.mac[1],
       i.mac[2], i.mac[3], i.mac[4], i.mac[5], i.fw_version, i.hw_rev,
       i.paired ? "paired" : "not paired");
    if (i.problem) op(out, ctx, "  problem: %s\r\n", i.problem);
    uint32_t iv;
    if (wifi_bridge_image(&iv))
        op(out, ctx, "  pod's bridge image: %u.%u.%u  update: %s\r\n", (unsigned)(iv >> 16) & 0xff,
           (unsigned)(iv >> 8) & 0xff, (unsigned)iv & 0xff, wifi_bridge_ota_status());
    else
        op(out, ctx, "  pod's bridge image: none (slot \"bridge\" empty)  update: %s\r\n", wifi_bridge_ota_status());
    if (i.have_status) {
        op(out, ctx, "  channel: %u  last disconnect reason: %u  bridge uptime: %lu s  (status %lu ms old)\r\n",
           i.channel, i.reason, (unsigned long)i.uptime_s, (unsigned long)i.status_age_ms);
        op(out, ctx, "  frames: eth_rx %lu wifi_tx %lu (err %lu) wifi_rx %lu eth_tx %lu (err %lu) dropped %lu\r\n",
           (unsigned long)i.counters.eth_rx, (unsigned long)i.counters.wifi_tx,
           (unsigned long)i.counters.wifi_tx_err, (unsigned long)i.counters.wifi_rx,
           (unsigned long)i.counters.eth_tx, (unsigned long)i.counters.eth_tx_err,
           (unsigned long)i.counters.dropped);
    }
}

static void cmd_bridge(int argc, char **argv, console_out_t out, void *ctx)
{
    static const char *const auth[] = {"open", "wep", "wpa", "wpa2", "wpa/wpa2", "wpa2-ent", "wpa3", "wpa2/wpa3"};
    const char *err = NULL;
    int rc;
    if (argc == 1) {
        if (!wifi_bridge_present()) op(out, ctx, "  no Wi-Fi bridge on the Ethernet port\r\n");
        else bridge_show(out, ctx);
    } else if (!strcmp(argv[1], "scan")) {
        rc = wifi_bridge_call(BWP_SCAN, NULL, 0, 10000, &g_wifi_bridge_reply, &err);
        if (bridge_failed(out, ctx, "bridge scan", rc, err)) return;
        for (unsigned k = 0; k < g_wifi_bridge_reply.n_aps; k++) {
            const bwp_ap_t *a = &g_wifi_bridge_reply.aps[k];
            op(out, ctx, "  %4d dBm  ch %2u  %-9s  %s\r\n", a->rssi, a->channel,
               a->authmode < 8 ? auth[a->authmode] : "other", a->ssid);
        }
        op(out, ctx, "  %u networks\r\n", g_wifi_bridge_reply.n_aps);
    } else if (!strcmp(argv[1], "update")) {
        wifi_bridge_ota_force();   /* the hw worker's loop runs it, after this line returns */
        op(out, ctx, "  bridge update requested: follow it with `bridge` (it reports the progress)\r\n");
    } else if (!strcmp(argv[1], "reboot")) {
        rc = wifi_bridge_call(BWP_REBOOT, NULL, 0, 1000, &g_wifi_bridge_reply, &err);
        if (!bridge_failed(out, ctx, "bridge reboot", rc, err)) op(out, ctx, "  the Wi-Fi bridge restarts\r\n");
    } else if (!strcmp(argv[1], "factory-reset") && argc == 3 && !strcmp(argv[2], "yes")) {
        rc = wifi_bridge_call(BWP_FACTORY_RESET, NULL, 0, 1000, &g_wifi_bridge_reply, &err);
        if (!bridge_failed(out, ctx, "bridge factory-reset", rc, err))
            op(out, ctx, "  the Wi-Fi bridge forgot its Wi-Fi and its pairing, and restarts\r\n");
    } else {
        op(out, ctx, "  usage: bridge | bridge scan | bridge update | bridge reboot | bridge factory-reset yes\r\n");
    }
}
#endif

/* wifi-set "<ssid>" "<password>" — save creds + kick the ESP32 (re)connect.
   Prints the markers the benchpod-cli parses ("[cfg] credentials written to
   flash").  Non-blocking: the association RPC runs on the net task, so the
   result is read back with wifi-show (blocking here would stall that RPC, which
   runs under the same hw_lock this handler holds). */
static void cmd_wifi_set(const char *args, console_out_t out, void *ctx)
{
    config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.magic = CONFIG_MAGIC; cfg.version = CONFIG_VERSION;
    const char *p = parse_quoted(args, cfg.ssid, sizeof(cfg.ssid));
    bool too_long = s_parse_truncated;
    parse_quoted(p, cfg.password, sizeof(cfg.password));   /* open AP: empty */
    if (too_long || s_parse_truncated) {                   /* never save credentials cut short */
        op(out, ctx, "  wifi-set: %s too long\r\n", too_long ? "ssid" : "password");
        return;
    }

    if (cfg.ssid[0] == '\0') {
        op(out, ctx, "  usage: wifi-set \"<ssid>\" \"<password>\"\r\n");
        return;
    }
#if defined(BENCHPOD_WIFI_BRIDGE)
    if (wifi_bridge_present()) {   /* the bridge box keeps the credentials, not the pod */
        const char *err = NULL;   /* the passphrase goes out sealed under the pairing key */
        int rc = wifi_bridge_set_wifi(cfg.ssid, cfg.password, &g_wifi_bridge_reply, &err);
        memset(cfg.password, 0, sizeof(cfg.password));
        if (bridge_failed(out, ctx, "wifi-set", rc, err)) return;
        op(out, ctx, "[cfg] credentials written to flash\r\n");   /* the marker benchpod-cli waits for */
        op(out, ctx, "  saved SSID \"%s\" on the Wi-Fi bridge: it joins in the background; run wifi-show for status\r\n",
           cfg.ssid);
        return;
    }
#endif
    if (config_save(&cfg) != 0) {
        op(out, ctx, "  wifi-set: config save failed\r\n");
        return;
    }
    op(out, ctx, "[cfg] credentials written to flash\r\n");
    net_wifi_reload();          /* re-read creds, (re)start the ESP32 + connect */
    op(out, ctx, "  saved SSID \"%s\" — connecting in the background; run wifi-show for status\r\n",
       cfg.ssid);
}

static void cmd_wifi_show(console_out_t out, void *ctx)
{
#if defined(BENCHPOD_WIFI_BRIDGE)
    if (wifi_bridge_present()) { bridge_show(out, ctx); return; }
#endif
    config_t cfg;
    bool have = (config_load(&cfg) == 0 && cfg.ssid[0] != '\0');
    op(out, ctx, "  ssid: %s\r\n", have ? cfg.ssid : "");
    op(out, ctx, "  state: %s\r\n", esp_wifi_ctrl_state_str());
    uint32_t done = 0, total = 0;
    if (esp_rom_flash_progress(&done, &total))
        op(out, ctx, "  c3 flash: block %lu/%lu\r\n", (unsigned long)done, (unsigned long)total);
    op(out, ctx, "  ip: %s\r\n", wifi_sta_ip());
    int rssi;
    if (esp_wifi_ctrl_rssi(&rssi)) op(out, ctx, "  rssi: %d dBm\r\n", rssi);
    else                           op(out, ctx, "  rssi: \r\n");
    uint32_t noresp = 0, reboot = 0;
    esp_wifi_ctrl_slave_lost_counts(&noresp, &reboot);
    op(out, ctx, "  c3 restarts: %lu no response, %lu rebooted while up\r\n",
       (unsigned long)noresp, (unsigned long)reboot);
}

/* The end of a console flash-esp32 (hw worker): the result line, then Wi-Fi gets the C3 back. */
static void flash_esp32_done(int result)
{
    if (result >= 0)
        log_printf("flash-esp32: ESP32-C3 flashed + booted (%s)\n", esp_rom_result_str(result));
    else
        log_printf("flash-esp32: failed, %s (C3 left in reset; see log)\n", esp_rom_result_str(result));
    net_wifi_hold_for_flash(false); /* restart the Wi-Fi join (fresh image, or retry) */
}

/* `ca`: one line per company CA certificate (cloud_extras_ca_each). */
typedef struct { console_out_t out; void *ctx; } console_line_t;
static bool console_ca_cert(void *c, const char *subject, const char *sha256_hex)
{
    console_line_t *cl = c;
    op(cl->out, cl->ctx, "ca %s %.64s\r\n", subject, sha256_hex);
    return true;
}

/* Console commands that capture into the PSRAM or drive the shared bus.  They go through the
   same heavy gate as the JSON and SCPI captures: run on the worker between two polls of a LAN
   capture_dual, a console `adc` used to overwrite the capture the iCE40 was still writing (or a
   staged OTA image) and read back garbage itself. */
static bool console_needs_heavy_gate(const char *verb)
{
    static const char *const gated[] = {
        "capture-psram", "psram-test", "dualcap", "lastress", "psram",
        "psram-addrtest", "psram-bench", "psram-clk", "psram-selftest", "cap-selftest",
        "flash-ice40",
    };
    for (size_t i = 0; i < sizeof(gated) / sizeof(gated[0]); i++)
        if (!strcmp(verb, gated[i])) return true;
    return false;
}

static void console_exec_locked(char *cmd, console_out_t out, void *ctx)
{
    /* wifi-set needs quote-aware parsing (SSID/password may contain spaces), so
       intercept it before the whitespace tokenizer mangles the line. */
    if (!strncmp(cmd, "wifi-set", 8) && (cmd[8] == ' ' || cmd[8] == '\t')) {
        cmd_wifi_set(cmd + 8, out, ctx);
        return;
    }

    /* Up to 11 tokens so `can write <id> b0..b7` (2 + 1 + 8) fits; other verbs
       use far fewer. */
    char *argv[11] = {0};
    int argc = 0;
    char *tok = strtok(cmd, " \t");
    while (tok && argc < 11) { argv[argc++] = tok; tok = strtok(NULL, " \t"); }
    if (argc == 0) return;
    /* The cloud-link settings carry the JSON verbs' names (cloud_ca, cloud_proxy) on the console
       too; the short names they started with keep working, and replies keep the short form. */
    static const char *const aliases[][2] = {
        { "cloud-ca", "ca" }, { "cloud-ca-clear", "ca-clear" }, { "cloud-proxy", "proxy" },
        { "cloud-proxy-set", "proxy-set" }, { "cloud-proxy-clear", "proxy-clear" },
    };
    for (size_t i = 0; i < sizeof(aliases) / sizeof(aliases[0]); i++)
        if (!strcmp(argv[0], aliases[i][0])) { argv[0] = (char *)aliases[i][1]; break; }

    const bool gated = console_needs_heavy_gate(argv[0]);
    if (gated && !command_handler_acquire_adc(CH_CONSOLE_TEXT_OWNER)) {
        op(out, ctx, "  %s refused: busy: a capture, upload or update is using the PSRAM bus; "
                     "try again when it ends\r\n", argv[0]);
        return;
    }

    if (!strcmp(argv[0], "help")) {
        cmd_help(out, ctx);
    } else if (!strcmp(argv[0], "ping")) {
        uint8_t ver = 0;
        if (fpga_ping(2, &ver) == 0) op(out, ctx, "  PING ok  version=0x%02x\r\n", ver);
        else                         op(out, ctx, "  PING failed\r\n");
    } else if (!strcmp(argv[0], "status")) {
        /* Stable identifier so a host can tell a real bench pod apart from other
           USB-serial devices: tooling just greps the output for "benchpod". */
        op(out, ctx, "  device : benchpod\r\n");
        op(out, ctx, "  mac    : %s\r\n", net_mac_str());
        op(out, ctx, "  ip     : %s\r\n", net_ip_str());
        {
            usb_net_stats_t us;
            usb_net_stats(&us);
            op(out, ctx, "  usb    : net %s pod %s host %s link %s rx %lu (drop %lu) tx %lu (full %lu)\r\n",
               usb_net_on() ? "on" : !usb_net_setting_on() ? "off (setting)" : "off (Wi-Fi configured)",
               usb_netif_pod_ip(), usb_netif_host_ip(), usb_net_link_up() ? "up" : "down",
               (unsigned long)us.rx_frames, (unsigned long)us.rx_drops,
               (unsigned long)us.tx_frames, (unsigned long)us.tx_full);
        }
        op(out, ctx, "  dma    : spi1=%s spi4=%s\r\n",
           signal_engine_dma_active() ? "on" : "off",
           esp_hosted_spi_dma_active() ? "on" : "off");
        /* PSRAM datapath health from the boot self-test (STM32<->PSRAM, iCE40 write,
           iCE40 /CE-net reach).  Re-run on demand with 'psram-selftest'. */
        op(out, ctx, "  psram  : %s\r\n", psram_selftest_str());
        /* Shared-bus ownership (psram.h): a non-zero count is a firmware bug worth reporting. */
        op(out, ctx, "  psbus  : depth %u, violations %lu\r\n", psram_bus_depth(),
           (unsigned long)psram_bus_violations());
        /* Capabilities — the full set advertised to the cloud (cl_send_capabilities),
           surfaced here so they're visible from the serial console without decoding the
           cloud handshake. The gateware version is LIVE-read (not the boot snapshot) so a
           runtime flash-ice40 is reflected, and shows UNREACHABLE when the FPGA is mute.
           The FPGA STATUS register is folded into the fpga line (not a bare "STATUS=0x.."
           top-level line) so a `key : value` parser of this output isn't tripped up. */
        bool     fpga_ok = signal_engine_refresh_version();
        uint8_t  gw      = signal_engine_fpga_version();
        bool     deep    = gw >= DAC_DEEP_REPLAY_MIN_GW;
        uint8_t  streg   = 0;
        bool     streg_ok = (fpga_status_read(&streg) == 0);
        op(out, ctx, "  board  : %s  fw v%s  rev %s  nrst_pin=%s  usb_cc=%s\r\n",
           BOARD_NAME, FIRMWARE_VERSION, board_rev_str(),
           nrst_ctrl_supported() ? "yes" : "no",
           usb_cc_supported() ? "yes" : "no");
        if (streg_ok)
            op(out, ctx, "  fpga   : gateware v%u (%s)  status_reg=0x%02x\r\n",
               gw, fpga_ok ? "reachable" : "UNREACHABLE", streg);
        else
            op(out, ctx, "  fpga   : gateware v%u (%s)  status_reg=read-failed\r\n",
               gw, fpga_ok ? "reachable" : "UNREACHABLE");
        if (ice40_embedded_gw_version() != 0 && gw != ice40_embedded_gw_version())
            op(out, ctx, "  gw     : firmware built for v%u, running v%u (updated at the next boot, or run flash-ice40)\r\n",
               ice40_embedded_gw_version(), gw);
        /* Separate keys ("hint", "safe", "clock", "gw"), so a host parsing "fpga : gateware vN"
           keeps working. */
        if (!fpga_ok && boot_hw_ready() && !ice40_is_configured())
            op(out, ctx, "  hint   : the iCE40 never configured (blank config flash?): run flash-ice40\r\n");
        if (boot_guard_report()[0] != '\0')
            op(out, ctx, "  safe   : %s\r\n", boot_guard_report());
        if (clock_on_hsi())
            op(out, ctx, "  clock  : the 25 MHz crystal did not start; running from the internal HSI\r\n");
        /* The digital board has the gateware's ADC/DAC engines but nothing behind them
           (board_variant.h), so it names no ADC or DAC, like its status (analog:false). */
        if (!board_has_analog()) {
            op(out, ctx, "  analog : none (digital board: no ADC, DAC or 4-20 mA)\r\n");
        } else {
            op(out, ctx, "  adc    : %d-bit x%d, %d mV full-scale\r\n",
               ADC_BITS, ADC_CHANNELS, ADC_FULLSCALE_MV);
            op(out, ctx, "  dac    : ac=%s replay=%s dc=%s  gen %d-bit / replay %d-bit x%d, %d mV FS\r\n",
               DAC_AC ? "yes" : "no", DAC_REPLAY ? "yes" : "no", DAC_DC ? "yes" : "no",
               DAC_BITS, DAC_REPLAY_BITS, DAC_CHANNELS, DAC_FULLSCALE_MV);
            op(out, ctx, "  replay : deep=%s (needs v%u), max %lu samples (%s)\r\n",
               deep ? "yes" : "no", DAC_DEEP_REPLAY_MIN_GW,
               (unsigned long)(deep ? FPGA_DAC_REPLAY_MAX_SAMPLES : SIGNAL_MAX_SAMPLES),
               deep ? "PSRAM" : "BRAM");
        }
        /* Cloud registration, so `benchpod discover` can report whether this pod is already
           claimed WITHOUT the pod being on the network — the TCP/JSON `cloud_status` command
           can only answer once there is a network to answer over, and a pod fresh out of its
           box has none. A pre-registered pod that someone was sent is the case that matters:
           over USB alone, this line is the whole answer. Read straight from the provisioned
           config, so nothing here depends on the link being up. */
        {
            cloud_config_t ccfg;
            if (cloud_config_load(&ccfg) == 0 && ccfg.enabled) {
                char last_error[80];
                cloud_client_last_error(last_error, sizeof(last_error));
                op(out, ctx, "  cloud  : registered  state=%s  device_id=%s%s%s\r\n",
                   cloud_client_state_str(), ccfg.device_id,
                   last_error[0] ? "  last_error=" : "", last_error);
            } else {
                op(out, ctx, "  cloud  : not registered\r\n");
            }
        }
        op(out, ctx, "  reset  : %s\r\n", fault_last_reset_str());
        op(out, ctx, "  crash  : %s\r\n", fault_last_crash_str());
        op(out, ctx, "  boot   : id %08lx  unclean resets not yet reported %lu\r\n",
           (unsigned long)fault_boot_id(), (unsigned long)fault_unclean_resets());
        {
            sys_health_task_t t[SYS_HEALTH_MAX_TASKS];
            int n = sys_health_tasks(t, SYS_HEALTH_MAX_TASKS);
            for (int i = 0; i < n; i++)
                op(out, ctx, "  stack  : %-7s %u B free (min)\r\n", t[i].name, t[i].stack_free);
        }
        op(out, ctx, "  flash  : %lu KB\r\n", (unsigned long)(flash_layout_size() / 1024u));
        op(out, ctx, "  lwip   : mem max %lu of %lu B, pbuf pool max %u of %u\r\n",
           (unsigned long)lwip_stats.mem.max, (unsigned long)lwip_stats.mem.avail,
           (unsigned)lwip_stats.memp[MEMP_PBUF_POOL]->max, (unsigned)lwip_stats.memp[MEMP_PBUF_POOL]->avail);
        op(out, ctx, "  heap   : %u B free, %u B min\r\n",
           sys_health_heap_free(), sys_health_heap_min_free());
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "spi-clk") && argc >= 2) {
        uint32_t div = (uint32_t)atoi(argv[1]);
        uint32_t hz = signal_engine_spi_set_prescaler(div);
        if (hz) op(out, ctx, "  SPI1 SCK = %lu Hz (/%lu)\r\n", (unsigned long)hz, (unsigned long)div);
        else    op(out, ctx, "  bad divisor (use 128 or 256; faster than /128 exceeds the iCE40's MOSI sampling)\r\n");
#endif
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "spi-diag")) {
        int n = (argc >= 2) ? atoi(argv[1]) : 50;
        signal_engine_spi_diag(n);
        op(out, ctx, "  (spi-diag printed on the device log)\r\n");
#endif
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "adc-spi")) {
        /* Read the live ADC sample DIRECTLY over SPI (CMD_ADC_PROBE), skipping
           PSRAM + the 24->48 CDC — isolates 0x5555 to engine/silicon vs path. */
        int n = (argc >= 2) ? atoi(argv[1]) : 16;
        signal_engine_adc_spi_diag(n);
        op(out, ctx, "  (adc-spi printed on the device log)\r\n");
#endif
    } else if (!strcmp(argv[0], "led")) {
        /* Light the status LED in one colour for 5 s (checks the LED and its pinout), then it
           goes back to showing the pod's state (status_led.h). */
        static const struct { const char *name; uint8_t mask; } colours[] = {
            {"off", STATUS_LED_OFF}, {"red", STATUS_LED_RED}, {"green", STATUS_LED_GREEN},
            {"blue", STATUS_LED_BLUE}, {"cyan", STATUS_LED_CYAN}, {"magenta", STATUS_LED_MAGENTA},
            {"yellow", STATUS_LED_RED | STATUS_LED_GREEN},
            {"white", STATUS_LED_RED | STATUS_LED_GREEN | STATUS_LED_BLUE},
        };
        int mask = -1;
        if (argc >= 2) {
            for (size_t i = 0; i < sizeof(colours) / sizeof(colours[0]); i++)
                if (!strcmp(argv[1], colours[i].name)) mask = colours[i].mask;
            if (mask < 0 && argv[1][0] >= '0' && argv[1][0] <= '7' && argv[1][1] == '\0')
                mask = argv[1][0] - '0';
        }
        if (mask < 0) {
            op(out, ctx, "usage: led red|green|blue|cyan|magenta|yellow|white|off|<0-7>\r\n");
        } else {
            status_led_override((uint8_t)mask, 5000u);
            op(out, ctx, "led %u for 5 s\r\n", (unsigned)mask);
        }
    } else if (!strcmp(argv[0], "psram-selftest")) {
        /* Layered PSRAM datapath test: STM32<->PSRAM, iCE40->PSRAM write, and (if
           that fails) whether the iCE40 even reaches the /CE net.  Same test run
           once at boot. */
        psram_boot_selftest();
        op(out, ctx, "  (psram-selftest printed on the device log)\r\n");
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "cap-selftest")) {
        /* End-to-end ADC->PSRAM path test: iCE40 streams a known ramp through the
           real capture datapath (CDC+writer+PSRAM+readback).  PASS => the path is
           healthy; FAIL => iCE40 capture/CDC timing error, NOT the analog ADC. */
        int n = (argc >= 2) ? atoi(argv[1]) : 64;
        int bad = -1;
        int rc = signal_engine_capture_selftest(n, &bad);
        op(out, ctx, "  cap-selftest: %s (see device log)\r\n",
           rc == 0 ? "PASS — ADC->PSRAM path OK" : "FAIL — iCE40 capture/CDC timing error");
#endif
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "i2c-scan")) {
        i2c_bus_status();   /* prints to the local console log */
        op(out, ctx, "  (scan printed on the device log)\r\n");
#endif
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "ina")) {
        cmd_ina(out, ctx);
#endif
    } else if (!strcmp(argv[0], "usb-reattach")) {
        /* macOS ignores a new network interface while the screen is locked; this makes
           the host see the device again without unplugging it. */
        op(out, ctx, "usb-reattach ok\r\n");
        usb_reattach_request();
    } else if (!strcmp(argv[0], "uid")) {
        /* uid: the 96-bit unique ID.  Words only: a byte read of this area is a precise bus
           fault on the H5 (proven on the v3, bfar=0x08fff800; see board_uid.h). */
        uint32_t w[3];
        board_uid_words(w);
        op(out, ctx, "  uid (words): %08lx %08lx %08lx\r\n",
           (unsigned long)w[0], (unsigned long)w[1], (unsigned long)w[2]);
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "test-bootloop")) {
        /* Prove the safe-mode safeguard: the next two boots crash in the net task (net) or the
           iCE40/PSRAM bring-up (hw), the third comes up in safe mode with only that part off.
           A power cycle then returns to normal.  `test-bootloop yes` is the net test. */
        const char *target = argc >= 3 ? argv[1] : "net";
        const char *confirm = argc >= 3 ? argv[2] : (argc >= 2 ? argv[1] : "");
        uint32_t sub = !strcmp(target, "net") ? BOOT_SUB_NET
                     : !strcmp(target, "hw")  ? BOOT_SUB_HW : 0;
        if (!sub || strcmp(confirm, "yes")) {
            op(out, ctx, "  test-bootloop net yes : crash the next 2 boots in the net task; the 3rd comes up in safe mode, network off\r\n");
            op(out, ctx, "  test-bootloop hw yes  : same, in the iCE40/PSRAM bring-up; the 3rd has iCE40/PSRAM off\r\n");
        } else {
            boot_guard_arm_test_loop(BOOT_GUARD_SAFE_AFTER, sub);
            op(out, ctx, "  armed: resetting now. Expect ~1 minute, then `status` shows safe mode\r\n");
            vTaskDelay(pdMS_TO_TICKS(200));
            NVIC_SystemReset();
        }
#endif
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "ice40-erase")) {
        /* Make this pod a new board again, to prove the blank-board gateware load: erase the
           start of the iCE40 config flash (its image begins at 0) and hold the iCE40 in reset,
           so the next boot finds it unconfigured. "gw0" also empties that slot, like a pod
           whose blobs arrive after its firmware. */
        const bool gw0 = argc >= 3 && !strcmp(argv[1], "gw0");
        if (argc < 2 || strcmp(argv[argc - 1], "yes") || (argc >= 3 && !gw0)) {
            op(out, ctx, "  ice40-erase [gw0] yes : blank the iCE40 config flash (and slot gw0); then reboot\r\n");
        } else if (w25q_session_open() != 0) {
            op(out, ctx, "  ice40-erase: the W25Q flash does not answer\r\n");
        } else {
            int rc = w25q_erase_block(0);
            if (rc == 0 && gw0) rc = blob_store_clear(BLOB_GW0);
            w25q_session_close();
            ice40_hold_off_bus();
            op(out, ctx, rc == 0 ? "  iCE40 config flash erased%s, iCE40 held in reset: reboot to test the blank-board path\r\n"
                                 : "  ice40-erase failed%s (see device log)\r\n",
               gw0 ? " (and slot gw0)" : "");
        }
#endif
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "test-crash")) {
        /* Prove the crash report: a real fault (UsageFault/HardFault) in the console task. */
        if (argc < 2 || strcmp(argv[1], "yes")) {
            op(out, ctx, "  test-crash yes : fault on purpose; the next boot reports it as the last crash\r\n");
        } else {
            op(out, ctx, "  crashing now\r\n");
            vTaskDelay(pdMS_TO_TICKS(100));
            __builtin_trap();
        }
#endif
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "test-hang")) {
        /* Prove the watchdog report: the named task spins, the IWDG early warning records it
           (~30 s), the pod resets, and the next boot reports the stall as its last crash. */
        int task = argc >= 2 && !strcmp(argv[1], "net") ? WD_TASK_NET
                 : argc >= 2 && !strcmp(argv[1], "hw")  ? WD_TASK_WORKER : -1;
        if (task < 0 || argc < 3 || strcmp(argv[2], "yes")) {
            op(out, ctx, "  test-hang net yes : hang the net task; the watchdog resets the pod in ~30 s\r\n");
            op(out, ctx, "  test-hang hw yes  : hang the hw worker; the watchdog resets the pod in ~90 s\r\n");
        } else {
            watchdog_test_hang(task);
            op(out, ctx, "  armed: the %s task hangs now\r\n", argv[1]);
        }
#endif
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "expdump")) {
        analog_switch_dump();
#endif
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "adcraw")) {
        /* adcraw — single raw ADC probe byte (debug). */
        uint8_t val = 0;
        if (adc_probe_one(&val) == 0) op(out, ctx, "  ADC=%u (0x%02x)\r\n", val, val);
        else                          op(out, ctx, "  adc-probe failed\r\n");
#endif
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "psram")) {
        /* Diagnostic path: reports bus/CS/arbitration state and re-probes the ID
           with the iCE40 held off the shared bus (see psram_diag). */
        if (psram_diag() == 0) op(out, ctx, "  PSRAM ID ok with iCE40 in reset (arbitration is the issue)\r\n");
        else                   op(out, ctx, "  PSRAM ID failed even with iCE40 in reset (see device log)\r\n");
#endif
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "psram-bench")) {
        uint32_t kb = (argc >= 2) ? (uint32_t)atoi(argv[1]) : 256;
        psram_bench(kb);
        op(out, ctx, "  (psram-bench printed on the device log)\r\n");
#endif
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "psram-clk") && argc >= 2) {
        uint32_t p = (uint32_t)atoi(argv[1]);
        uint32_t hz = psram_set_prescaler(p);
        if (hz) op(out, ctx, "  OCTOSPI SCLK = %lu Hz (presc %lu)\r\n", (unsigned long)hz, (unsigned long)p);
        else    op(out, ctx, "  psram-clk failed\r\n");
#endif
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "psram-chunk") && argc >= 2) {
        uint32_t n = psram_set_chunk((uint32_t)atoi(argv[1]));
        op(out, ctx, "  PSRAM tCEM chunk = %lu bytes\r\n", (unsigned long)n);
#endif
    } else if (!strcmp(argv[0], "psram-test")) {
        if (psram_init() == 0 && psram_test() == 0) op(out, ctx, "  PSRAM ok\r\n");
        else                                        op(out, ctx, "  PSRAM failed (see device log)\r\n");
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "psram-addrtest")) {
        /* STM32-standalone full-8MB address integrity test: write an ADDRESS-TAGGED
           256-B block at points spanning the whole device, then read them ALL back and
           verify.  Catches address aliasing (a dropped high address bit would have one
           write clobber another -> tag mismatch) and per-address read HAL failures.
           This isolates the STM32/XSPI/chip path from the iCE40 write path. */
        static const uint32_t A[] = {
            0x000000u, 0x010000u, 0x080000u, 0x100000u, 0x200000u,
            0x300000u, 0x400000u, 0x600000u, 0x7FFF00u
        };
        const int NADDR = (int)(sizeof(A) / sizeof(A[0]));
        uint8_t *b = adc_pool_scratch();   /* 256 B; gated, so the pool is ours */
        int errs = 0;
        psram_bus_acquire();
        for (int a = 0; a < NADDR; a++) {                 /* phase 1: write tagged blocks */
            for (int i = 0; i < 256; i++) b[i] = (uint8_t)((A[a] >> 12) + (uint32_t)i * 7u + 0x5Au);
            if (psram_write(A[a], b, 256) != 0) { op(out, ctx, "  write @%06x HAL-FAIL\r\n", (unsigned)A[a]); errs++; }
        }
        for (int a = 0; a < NADDR; a++) {                 /* phase 2: read back + verify */
            for (int i = 0; i < 256; i++) b[i] = 0;
            if (psram_read(A[a], b, 256) != 0) { op(out, ctx, "  read  @%06x HAL-FAIL\r\n", (unsigned)A[a]); errs++; continue; }
            int bad = -1;
            for (int i = 0; i < 256; i++)
                if (b[i] != (uint8_t)((A[a] >> 12) + (uint32_t)i * 7u + 0x5Au)) { bad = i; break; }
            if (bad < 0) op(out, ctx, "  @%06x OK\r\n", (unsigned)A[a]);
            else { op(out, ctx, "  @%06x MISMATCH i=%d got=%02x\r\n", (unsigned)A[a], bad, b[bad]); errs++; }
        }
        psram_bus_release();
        op(out, ctx, "  psram-addrtest: %s (%d error(s) across %d addrs to 8MB)\r\n",
           errs ? "FAIL" : "PASS", errs, NADDR);
#endif
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "capture-psram")) {
        unsigned n = (argc >= 2) ? (unsigned)strtoul(argv[1], NULL, 0) : 16u;
        if (n > 64) n = 64;
        uint16_t s[64];
        if (adc_capture_psram(s, n, 0.0f) == 0) {
            op(out, ctx, "  %u samples:", n);
            for (unsigned i = 0; i < n; i++) op(out, ctx, " %u", s[i]);
            op(out, ctx, "\r\n");
        } else {
            op(out, ctx, "  capture-psram failed (v2 gateware? see log)\r\n");
        }
#endif
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "dualcap")) {
        /* Unified simultaneous ADC + raw-LA capture (one trigger).  `dualcap [an]
           [ln]` captures `an` ADC + `ln` LA samples into their two PSRAM regions and
           prints the first few of each.  ADC slow (div 240 = 100 kS/s), LA fast
           (div 24 = 1 MS/s) to exercise the independent-rate datapath. */
        unsigned an = (argc >= 2) ? (unsigned)strtoul(argv[1], NULL, 0) : 64u;
        unsigned ln = (argc >= 3) ? (unsigned)strtoul(argv[2], NULL, 0) : 64u;
        if (an > 256) an = 256;
        if (ln > 256) ln = 256;
        uint16_t *adcb = (uint16_t *)adc_pool_scratch();   /* gated: the pool's scratch, 256 each */
        uint16_t *lab  = adcb + 256;
        int rc = fpga_dual_capture(an ? adcb : NULL, (uint16_t)an, 240,
                                   ln ? lab : NULL, (uint16_t)ln, 24);
        if (rc != 0) {
            op(out, ctx, "  dualcap failed (v2 gateware? see log)\r\n");
        } else {
            op(out, ctx, "  dualcap: ADC=%u @100kS/s  LA=%u @1MS/s  (one trigger)\r\n", an, ln);
            if (an) op(out, ctx, "    ADC[0..3]: %5u %5u %5u %5u\r\n",
                       adcb[0], adcb[1], adcb[2], adcb[3]);
            if (ln) op(out, ctx, "    LA [0..3]: 0x%03x 0x%03x 0x%03x 0x%03x\r\n",
                       lab[0] & 0xFFFu, lab[1] & 0xFFFu, lab[2] & 0xFFFu, lab[3] & 0xFFFu);
        }
#endif
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "lastress")) {
        /* Deep-LA capture + drain stress test.  Captures `n` LA samples (up to the
           full 8 MB PSRAM = LA_PSRAM_MAX_SAMPLES) at the MAX rate and checks: no ring
           overflow (STATUS bit5, the 48 MHz DDR-drain acid test), AND that the
           read-back works at BOTH the start AND a HIGH offset near the end — which is
           the acid test for the 8 MB addressing (a deep capture writes far past the
           old 1 MB DEVSIZE wall). */
        unsigned n = (argc >= 2) ? (unsigned)strtoul(argv[1], NULL, 0) : 65535u;
        if (n < 1u) n = 1u;
        if (n > LA_PSRAM_MAX_SAMPLES) n = LA_PSRAM_MAX_SAMPLES;
        static uint16_t lo[4], hi[4];
        if (fpga_la_capture_psram_start((size_t)n, 0.0f /*max rate*/, NULL) != 0) {
            op(out, ctx, "  lastress: arm failed (v2 gateware / LA voltage? see log)\r\n");
        } else {
            int r = 0; uint32_t spin = 0;
            while ((r = fpga_la_capture_psram_wait()) == 0 && spin < 200000000u) spin++;
            if (r == 1) {
                uint32_t hi_off = (n >= 4u) ? (uint32_t)(n - 4u) * 2u : 0u; /* byte offset of last 4 samples */
                int rr = fpga_la_psram_read(0, (uint8_t *)lo, sizeof(lo));
                rr |= fpga_la_psram_read(hi_off, (uint8_t *)hi, sizeof(hi));
                fpga_la_psram_release();
                if (rr == 0)
                    op(out, ctx, "  lastress: %u LA samples @max rate => PASS (no overflow; "
                                 "read-back OK to 0x%06lx); LA[0..3]=0x%03x.. LA[hi]=0x%03x..\r\n",
                       n, (unsigned long)(0x010000u + hi_off), lo[0] & 0xFFFu, hi[0] & 0xFFFu);
                else
                    op(out, ctx, "  lastress: %u LA samples => FAIL (read-back HAL error at high "
                                 "offset 0x%06lx)\r\n", n, (unsigned long)(0x010000u + hi_off));
            } else {
                op(out, ctx, "  lastress: %u LA samples @max rate => FAIL "
                             "(overflow/timeout; STATUS bit5 — see log)\r\n", n);
            }
        }
#endif
#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
    } else if (!strcmp(argv[0], "flash-id")) {
        uint8_t id[3] = {0};
        if (ice40_flash_read_id(id) == 0)
            op(out, ctx, "  iCE40 flash JEDEC ID: %02x %02x %02x (W25Q64 = ef 40 17)\r\n",
               id[0], id[1], id[2]);
        else
            op(out, ctx, "  flash-id failed (see device log)\r\n");
#endif
    } else if (!strcmp(argv[0], "flash-ice40")) {
        if (!blob_store_present(BLOB_GW0)) {
            op(out, ctx, "  no gateware image 0 in the W25Q (install blob gw0 first)\r\n");
        } else {
            /* Quiesce first: ice40_flash_program grabs the shared bus (psram_bus_acquire); a
               live DAC replay/reader mid-burst would wedge the PSRAM the same way the runtime
               image swap does. */
            signal_engine_quiesce_psram_masters();
            /* Through ice40_reflash_image like every other reconfig: it puts the PSRAM in SPI
               for the config read, re-inits and hands the bus back, and re-syncs the firmware's
               mirrors of the fabric's registers.  Calling ice40_flash_program directly skipped
               all of that (stale capture bases = the "sentinel survived" false wedge). */
            if (ice40_reflash_image(0) == 0) {
                /* Re-read the gateware version so status/capabilities reflect the just-
                   flashed bitstream instead of the boot-time value. */
                bool ok = signal_engine_refresh_version();
                cloud_client_request_caps_resend();   /* the running image may have changed */
                op(out, ctx, "  iCE40 reflashed + reconfigured (%u bytes), gateware v%u%s\r\n",
                   (unsigned)blob_store_info(BLOB_GW0)->len, signal_engine_fpga_version(),
                   ok ? "" : " (FPGA unreachable — check gateware)");
            } else {
                op(out, ctx, "  flash-ice40 failed (see device log)\r\n");
            }
        }
    } else if (!strcmp(argv[0], "sig-policy")) {
        /* The USB console may set any value: it is the way back from "required" (pod_policy.h). */
        const char *why = NULL;
        if (argc >= 2) {
            int p = pod_policy_sig_from_name(argv[1]);
            why = p < 0 ? "use audit, permissive or required" : pod_policy_set_sig((fw_sig_policy_t)p, POLICY_SRC_USB);
        }
        if (why) op(out, ctx, "sig-policy error %s\r\n", why);
        else     op(out, ctx, "sig-policy %s %u\r\n", fw_sign_policy_name(pod_policy_sig()),
                    (unsigned)fw_sign_key_count());
#if !defined(BENCHPOD_RELEASE)
    } else if (!strcmp(argv[0], "ca-damage")) {
        /* Development builds: damage the company CA slot to try the built-in-roots fallback. */
        const char *why = cloud_extras_ca_damage();
        if (why) op(out, ctx, "ca-damage error %s\r\n", why);
        else     op(out, ctx, "ca-damage ok (ca error %s)\r\n",
                    cloud_extras_ca_error() ? cloud_extras_ca_error() : "none");
#endif
    } else if (!strcmp(argv[0], "ca") || !strcmp(argv[0], "ca-clear")) {
        if (!strcmp(argv[0], "ca-clear")) {
            const char *why = bus_busy_reason();   /* the W25Q write takes the shared bus */
            if (!why) why = cloud_extras_ca_clear();
            if (why) op(out, ctx, "ca-clear error %s\r\n", why);
            else     op(out, ctx, "ca-clear ok\r\n");
        } else {
            /* One line per certificate: "ca <subject> <sha256>", or "ca none". */
            console_line_t cl = { out, ctx };
            int n = cloud_extras_ca_each(console_ca_cert, &cl);
            if (n == 0) op(out, ctx, "ca none\r\n");
            if (cloud_extras_ca_error()) op(out, ctx, "ca error %s\r\n", cloud_extras_ca_error());
        }
    } else if (!strcmp(argv[0], "proxy") || !strcmp(argv[0], "proxy-set") || !strcmp(argv[0], "proxy-clear")) {
        const char *why = NULL;
        /* proxy-set/-clear write the W25Q, which takes the shared bus; plain `proxy` only reads */
        const char *busy = strcmp(argv[0], "proxy") ? bus_busy_reason() : NULL;
        if (busy)
            why = busy;
        else if (!strcmp(argv[0], "proxy-set"))
            why = argc < 2 ? "usage: proxy-set <host:port> [user password]"
                           : cloud_extras_proxy_set(argv[1], argc >= 4 ? argv[2] : NULL, argc >= 4 ? argv[3] : NULL);
        else if (!strcmp(argv[0], "proxy-clear"))
            why = cloud_extras_proxy_set(NULL, NULL, NULL);
        if (why) {
            op(out, ctx, "proxy error %s\r\n", why);
        } else {
            cloud_proxy_t p;
            cloud_extras_proxy_get(&p);
            if (p.host[0]) op(out, ctx, "proxy %s:%u %s\r\n", p.host, (unsigned)p.port, p.user[0] ? "auth" : "noauth");
            else           op(out, ctx, "proxy none\r\n");
            memset(&p, 0, sizeof(p));
        }
    } else if (!strcmp(argv[0], "usb-network")) {
        /* The USB network adapter on or off (pod_policy.h): off, the pod re-enumerates as its
           console only. The console is the way back on. */
        const char *why = NULL;
        if (argc >= 2) {
            if (!strcmp(argv[1], "on") || !strcmp(argv[1], "off"))
                why = pod_policy_set_usb_net(!strcmp(argv[1], "on"), POLICY_SRC_USB, false);
            else
                why = "use on or off";
        }
        if (why) op(out, ctx, "usb-network error %s\r\n", why);
        else     op(out, ctx, "usb-network %s\r\n", pod_policy_usb_net() ? "on" : "off");
    } else if (!strcmp(argv[0], "lan-tls")) {
        /* The LAN API over TLS (lan_tls.h): show the certificate, set HTTPS-only, regenerate. */
        const char *why = NULL;
        if (argc >= 3 && !strcmp(argv[1], "https-only") && (!strcmp(argv[2], "on") || !strcmp(argv[2], "off")))
            why = pod_policy_set_https_only(!strcmp(argv[2], "on"), POLICY_SRC_USB, false, false);
        else if (argc >= 2 && !strcmp(argv[1], "regenerate"))
            why = lan_tls_regenerate();
        else if (argc >= 2)
            why = "use https-only on|off or regenerate";
        static lan_tls_info_t in;   /* console task only */
        lan_tls_get_info(&in);
        if (why)            op(out, ctx, "lan-tls error %s\r\n", why);
        else if (!in.ready) op(out, ctx, "lan-tls port %d https-only %s none %s\r\n", LAN_TLS_PORT,
                               pod_policy_https_only() ? "on" : "off", in.error);
        else                op(out, ctx, "lan-tls port %d https-only %s sha256 %s %s \"%s\" \"%s\"\r\n", LAN_TLS_PORT,
                               pod_policy_https_only() ? "on" : "off", in.sha256, in.origin, in.key, in.subject);
    } else if (!strcmp(argv[0], "lan-policy")) {
        const char *why = NULL;
        if (argc >= 2) {
            int p = pod_policy_lan_from_name(argv[1]);
            why = p < 0 ? "use open, locked or off" : pod_policy_set_lan((pod_lan_policy_t)p, POLICY_SRC_USB);
        }
        if (why) op(out, ctx, "lan-policy error %s\r\n", why);
        else     op(out, ctx, "lan-policy %s\r\n", pod_policy_lan_name(pod_policy_lan()));
    } else if (!strcmp(argv[0], "blobs")) {
        /* One line per W25Q slot: name, state against this firmware, what it holds. */
        for (int i = 0; i < BLOB_SLOT_COUNT; i++) {
            if (!blob_listed((blob_id_t)i)) continue;
            const blob_info_t *in = blob_store_info((blob_id_t)i);
            char hex[65] = "-";
            if (in->present)
                for (int k = 0; k < 32; k++) snprintf(hex + k * 2, 3, "%02x", in->sha256[k]);
            op(out, ctx, "blob %s %s %lu %lu %s\r\n", blob_name((blob_id_t)i),
               blob_state_str(blob_state((blob_id_t)i)), (unsigned long)in->len,
               (unsigned long)in->version, hex);
        }
        /* The firmware copy the last install left in the W25Q (for the bootloader planned next).
           Not a "blob" line: installers parse those and must not see an internal slot. */
        const blob_info_t *fw = blob_store_info(BLOB_FW);
        if (fw && fw->present)
            op(out, ctx, "fw-copy %lu bytes, sha256 %02x%02x%02x%02x...\r\n", (unsigned long)fw->len,
               fw->sha256[0], fw->sha256[1], fw->sha256[2], fw->sha256[3]);
        else
            op(out, ctx, "fw-copy none\r\n");
    } else if (esp_rom_flash_busy() &&
               (!strcmp(argv[0], "flash-esp32-sync") || !strcmp(argv[0], "flash-esp32") ||
                !strcmp(argv[0], "esp-reset-pulse") || !strcmp(argv[0], "esp-mon"))) {
        uint32_t done = 0, total = 0;
        (void)esp_rom_flash_progress(&done, &total);
        op(out, ctx, "  %s refused: the ESP32-C3 is being flashed (block %lu/%lu); wait for the "
                     "\"[espflash] done\" line\r\n", argv[0], (unsigned long)done, (unsigned long)total);
    } else if (!strcmp(argv[0], "flash-esp32-sync")) {
        net_wifi_hold_for_flash(true);  /* Wi-Fi control must not drive EN/BOOT meanwhile */
        uint32_t magic = 0;
        if (esp_rom_flash_sync(&magic) == 0)
            op(out, ctx, "  C3 ROM synced (chip magic 0x%08lx)\r\n", (unsigned long)magic);
        else
            op(out, ctx, "  flash-esp32-sync failed — no C3 ROM response (see log)\r\n");
        net_wifi_hold_for_flash(false); /* the C3 was left in its ROM loader: restart Wi-Fi */
    } else if (!strcmp(argv[0], "flash-esp32") && bus_busy_reason()) {
        /* ~140 s on the shared bus: not under a capture, upload or update */
        op(out, ctx, "  flash-esp32 refused: %s\r\n", bus_busy_reason());
    } else if (!strcmp(argv[0], "flash-esp32")) {
        /* Runs in the background (~2 min): the console and the API keep answering, and the log
           carries the progress and the result (flash_esp32_done). */
        net_wifi_hold_for_flash(true);  /* Wi-Fi control must not drive EN/BOOT meanwhile */
        int rc = esp_rom_flash_start(flash_esp32_done);
        if (rc == 0) {
            op(out, ctx, "  flashing the ESP32-C3 in the background (%u bytes, about 2 minutes);\r\n"
                         "  the log shows the progress and ends with \"flash-esp32: ...\"\r\n",
               (unsigned)blob_store_info(BLOB_ESP)->len);
        } else {
            op(out, ctx, "  flash-esp32 failed: %s\r\n",
               rc == ESP_FLASH_START_NO_IMAGE ? "no ESP32-C3 image in the W25Q (install blob esp first)"
               : rc == ESP_FLASH_START_BAD_SLOT ? "the esp slot does not match its checksum"
               : "a flash is already running");
            net_wifi_hold_for_flash(false);
        }
    } else if (!strcmp(argv[0], "wifi-set")) {
        op(out, ctx, "  usage: wifi-set \"<ssid>\" \"<password>\"\r\n");
    } else if (!strcmp(argv[0], "esp-reset-pulse")) {
        /* Diagnostic: reset the ESP32-C3 behind the Wi-Fi control's back (EN low 50 ms), which
           is what a C3 crash or brown-out looks like from here.  The control must notice (the
           C3's new boot event, or its RSSI answers stopping), drop the link and rejoin; see
           wifi_status c3_lost_reboot / c3_lost_noresp. */
        HAL_GPIO_WritePin(ESP_EN_PORT, ESP_EN_PIN, GPIO_PIN_RESET);
        sleep_ms(50);
        HAL_GPIO_WritePin(ESP_EN_PORT, ESP_EN_PIN, GPIO_PIN_SET);
        op(out, ctx, "  C3 reset pulsed (50 ms); watch wifi-show\r\n");
    } else if (!strcmp(argv[0], "esp-mon")) {
        uint32_t ms = (argc >= 2) ? (uint32_t)atoi(argv[1]) : 5000;
        if (ms > 20000) ms = 20000;
        esp_uart_monitor(ms);
    } else if (!strcmp(argv[0], "wifi-show")) {
        cmd_wifi_show(out, ctx);
    } else if (!strcmp(argv[0], "wifi-static")) {
        if (argc >= 4) net_wifi_static(argv[1], argv[2], argv[3]);
        else op(out, ctx, "  usage: wifi-static <ip> <netmask> <gateway>  (e.g. 172.20.10.9 255.255.255.240 172.20.10.1)\r\n");
    } else if (!strcmp(argv[0], "eth")) {
        /* Manually bring the wired interface down/up (PHY reset + DHCP re-acquire)
           for debugging a stuck link without a power cycle. */
        if      (argc >= 2 && !strcmp(argv[1], "stop"))    { net_eth_stop();    op(out, ctx, "  eth: stop requested\r\n"); }
        else if (argc >= 2 && !strcmp(argv[1], "start"))   { net_eth_start();   op(out, ctx, "  eth: start requested (PHY reset + DHCP)\r\n"); }
        else if (argc >= 2 && !strcmp(argv[1], "restart")) { net_eth_restart(); op(out, ctx, "  eth: restart requested (PHY reset + DHCP)\r\n"); }
        else if (argc >= 2 && !strcmp(argv[1], "stats")) {
            static eth_diag_t d;      /* static: the console stack has ~1.3 KB of headroom */
            static char line[384];
            net_eth_diag(&d);
            eth_diag_format(&d, line, sizeof(line));
            op(out, ctx, "  eth: ");
            out(ctx, line);            /* not op(): its 160-byte buffer would cut the line */
            op(out, ctx, "\r\n");
        }
#if !defined(BENCHPOD_RELEASE)   /* development builds only: wired-link diagnostics (JSON eth has them) */
        else if (argc >= 2 && !strcmp(argv[1], "refclk")) {
            uint32_t seq = net_eth_refclk_seq(), hz = 0;
            net_eth_refclk_measure();
            /* The net task runs it (~200 ms of gate plus a PHY restart); wait, don't guess. */
            for (int i = 0; i < 40 && !net_eth_refclk_result(seq, &hz); i++)
                vTaskDelay(pdMS_TO_TICKS(100));
            if (hz == 0) {
                op(out, ctx, "  eth refclk: no clock on PA1 — the PHY is not driving RMII\r\n");
            } else {
                long ppm = ((long long)hz - 50000000LL) * 1000000LL / 50000000LL;
                op(out, ctx, "  eth refclk: %lu Hz, %+ld ppm vs 50 MHz%s\r\n",
                   (unsigned long)hz, ppm,
                   clock_on_hsi() ? " (UNRELIABLE: the MCU crystal did not start, so the"
                                    " reference this is measured against is the internal HSI)" : "");
                op(out, ctx, "  (100BASE-TX needs +/-50 ppm, 10BASE-T tolerates far more."
                             " This compares the PHY clock with the MCU crystal, so read it"
                             " next to a known-good pod.)\r\n");
            }
        }
        else if (argc >= 3 && !strcmp(argv[1], "loopback")) {
            int mbit = atoi(argv[2]);
            uint32_t n = (argc >= 4) ? (uint32_t)atoi(argv[3]) : 200u;
            if (mbit != 10 && mbit != 100) {
                op(out, ctx, "  usage: eth loopback <10|100> [n]\r\n");
            } else {
                if (n == 0) n = 1;
                if (n > 1000) n = 1000;
                uint32_t seq = net_eth_loopback_seq();
                static eth_loopback_result_t r;   /* static: console stack headroom */
                net_eth_loopback(mbit, n);
                bool done = false;
                for (int i = 0; i < 150 && !(done = net_eth_loopback_result(seq, &r)); i++)
                    vTaskDelay(pdMS_TO_TICKS(100));
                if (!done) {
                    op(out, ctx, "  eth loopback: no result (timed out)\r\n");
                } else {
                    op(out, ctx, "  eth loopback %dM: sent %lu, came back %lu, intact %lu, corrupt %lu,"
                                 " crc %lu, align %lu, tx fail %lu\r\n",
                       r.mbit, (unsigned long)r.sent, (unsigned long)r.received,
                       (unsigned long)r.intact, (unsigned long)r.corrupt,
                       (unsigned long)r.crc, (unsigned long)r.align, (unsigned long)r.tx_fail);
                    op(out, ctx, "  %s\r\n",
                       r.intact == r.sent
                         ? "all frames intact: the RMII lines and the PHY's digital side are fine at this speed"
                         : "frames lost or damaged without ever reaching the jack: suspect the RMII lines"
                           " (TXD0/TXD1/TX_EN, RXD, CRS_DV) or the PHY's digital side");
                }
            }
        }
        else if (argc >= 3 && !strcmp(argv[1], "speed")) {
            int full = (argc >= 4 && !strcmp(argv[3], "full"));
            if (!strcmp(argv[2], "auto")) {
                net_eth_force_speed(0, 0);
                op(out, ctx, "  eth: autonegotiation restored (re-acquiring link + DHCP)\r\n");
            } else if (!strcmp(argv[2], "10") || !strcmp(argv[2], "100")) {
                int mbit = atoi(argv[2]);
                net_eth_force_speed(mbit, full);
                op(out, ctx, "  eth: forcing %d M %s (re-acquiring link + DHCP)\r\n",
                   mbit, full ? "full" : "half");
                if (full)
                    op(out, ctx, "  note: a switch port that autonegotiates parallel-detects HALF,"
                                 " so forced full is a duplex mismatch\r\n");
            } else {
                op(out, ctx, "  usage: eth speed <auto|10|100> [full]\r\n");
            }
        }
#endif
        else op(out, ctx, "  usage: eth <stop|start|restart|stats>\r\n");
    } else if (!strcmp(argv[0], "identity")) {
        /* "identity <short id> <public key>", or "identity unknown <why>" (device_identity.h). */
        char id[DEVICE_ID_SHORT_LEN + 1];
        device_identity_short_id(id);
        uint8_t pub[DEVICE_ID_PUBLIC_LEN];
        if (id[0] && device_identity_get_public(pub) == 0) {
            char b64[B64URL_ENCODED_LEN(DEVICE_ID_PUBLIC_LEN) + 1];
            b64url_encode(pub, sizeof(pub), b64, sizeof(b64));
            op(out, ctx, "identity %s %s\r\n", id, b64);
        } else {
            op(out, ctx, "identity unknown %s\r\n", device_identity_problem());
        }
    } else if (!strcmp(argv[0], "identity-wipe")) {
        /* The way back for a pod whose identity record the firmware will not replace: erase it and
           make a new key. USB console only (this console is physical presence); the token must be
           the current short id, or "unknown" when there is none. */
        const char *why = device_identity_wipe(argc >= 2 ? argv[1] : NULL);
        if (why) {
            op(out, ctx, "identity-wipe error %s\r\n", why);
        } else {
            char id[DEVICE_ID_SHORT_LEN + 1];
            device_identity_short_id(id);
            op(out, ctx, "identity-wipe ok %s\r\n", id);
            net_cloud_reload();   /* log in again with the new key (the server needs a re-register) */
        }
#if defined(BENCHPOD_WIFI_BRIDGE)
    } else if (!strcmp(argv[0], "wifi-clear") && wifi_bridge_present()) {
        const char *err = NULL;
        int rc = wifi_bridge_call(BWP_CLEAR_WIFI, NULL, 0, 1000, &g_wifi_bridge_reply, &err);
        if (!bridge_failed(out, ctx, "wifi-clear", rc, err))
            op(out, ctx, "  Wi-Fi credentials cleared on the Wi-Fi bridge\r\n");
    } else if (!strcmp(argv[0], "bridge")) {
        cmd_bridge(argc, argv, out, ctx);
#endif
    } else if (!strcmp(argv[0], "wifi-scan")) {
        /* For a Wi-Fi picker (the CLI): "scan <rssi> <channel> <auth> <ssid>" per network,
           strongest first, then "scan done <n>"; or "wifi-scan error <why>". Through a Wi-Fi
           bridge when one is plugged in, else the built-in Wi-Fi, whose list follows the prompt
           when the scan ends (the worker does not wait for it). */
        wifi_scan_console();
    } else if (!strcmp(argv[0], "wifi-clear")) {
        /* Drops Wi-Fi and wipes both copies: config_store, then the ESP32-C3's NVS (~3 s). */
        config_clear();
        net_wifi_reload();          /* drop Wi-Fi now, whatever happens to the C3 */
        const char *why = wifi_clear_c3_now();
        if (!why)
            op(out, ctx, "  Wi-Fi credentials cleared (pod flash and the ESP32-C3's NVS)\r\n");
        else
            op(out, ctx, "  Wi-Fi credentials cleared on the pod, but the ESP32-C3's NVS was not\r\n"
                         "  erased (%s): an old copy may remain there; run wifi-clear again or flash-esp32\r\n", why);
    } else if (!strcmp(argv[0], "dfu")) {
        op(out, ctx, "  entering DFU (USB bootloader) — flash with dfu-util / `benchpod flash-self`...\r\n");
        HAL_Delay(50);          /* let the notice drain before USB re-enumerates */
        dfu_boot_request();     /* arms the flag + system reset; never returns */
    } else if (!strcmp(argv[0], "selftest")) {
        cmd_selftest(out, ctx);
    } else if (!strcmp(argv[0], "reboot")) {
        op(out, ctx, "  rebooting...\r\n");
        HAL_Delay(50);
        NVIC_SystemReset();
    } else if (console_moved_to(argv[0])) {
        /* An older benchpod CLI / embeddedci SDK: say what to update, not "unknown command". */
        static char moved[224];
        console_moved_line(argv[0], moved, sizeof(moved));
        out(ctx, moved);
        out(ctx, "\r\n");
    } else {
        op(out, ctx, "  unknown command '%s' (try 'help')\r\n", argv[0]);
    }
    if (gated) command_handler_release_adc(CH_CONSOLE_TEXT_OWNER);
}

void console_exec(char *cmd, console_out_t out, void *ctx)
{
    hw_lock();
    console_exec_locked(cmd, out, ctx);
    hw_unlock();
}

/* ---- local interactive console ----------------------------------------- */

static char line[CONSOLE_LINE_MAX];
static size_t line_len;

static void local_out(void *ctx, const char *s)
{
    (void)ctx;
    console_io_write((const uint8_t *)s, strlen(s));
}

/* Run one console command line on the hw worker task (the console task only does
   line editing + submission now).  Executes the command, then reprints the prompt
   AFTER the output so the "> " lands below the result rather than before it. */
void console_run_line(const char *line_in)
{
    char buf[CONSOLE_LINE_MAX];
    strncpy(buf, line_in, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    console_exec(buf, local_out, NULL);
    printf("> ");
}

void console_init(void)
{
    line_len = 0;
    printf("\r\nconsole ready — type 'help'\r\n> ");
}

void console_poll(void)
{
    /* Persists across polls: a CR and its trailing LF can arrive in separate
       drains.  Swallowing the LF of a CR-LF stops the prompt firing twice (the
       "> on its own line" artifact with terminals that send CR-LF). */
    static int last_was_cr = 0;
    int c;
    while ((c = console_io_getc()) >= 0) {
        if (c == '\n' && last_was_cr) { last_was_cr = 0; continue; }  /* LF of a CR-LF */
        if (c == '\r' || c == '\n') {
            last_was_cr = (c == '\r');
            printf("\r\n");
            line[line_len] = '\0';
            if (line_len > 0) {
                /* Execute on the hw worker (it owns the instrument hardware); the
                   worker reprints the prompt after the command output. */
                if (!hw_worker_submit_console(line)) {
                    printf("busy — try again\r\n> ");   /* worker queue full */
                }
            } else {
                printf("> ");
            }
            line_len = 0;
        } else if (c == 0x08 || c == 0x7F) {
            last_was_cr = 0;
            if (line_len > 0) { line_len--; printf("\b \b"); }
        } else if (c >= 0x20 && c < 0x7F && line_len < CONSOLE_LINE_MAX - 1) {
            last_was_cr = 0;
            uint8_t ch = (uint8_t)c;
            line[line_len++] = (char)ch;
            console_io_write(&ch, 1);
        }
    }
}
