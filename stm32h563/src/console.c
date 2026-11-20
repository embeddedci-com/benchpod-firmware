/*
 * console.c — bench-pod STM32 bring-up console.
 *
 * console_exec_hw() runs one command line and writes results through an output
 * sink, so the same command set serves both the local USB/USART console and the
 * TCP command server (net_server.c).  The full RP2350 command_handler/SCPI/JSON
 * surface lands in later phases.
 */
#include "console.h"
#include "cloud_caps.h"     /* cloud_caps_wifi_mac: status wifimac */
#include "bp_json.h"         /* bp_hex_nibble */
#include "usb_device.h"
#include "usb_netif.h"
#include "adc_pool.h"   /* gated diagnostics use the shared pool's scratch */
#include "console_io.h"
#include "signal_engine.h"
#include "gw_caps.h"
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
#include "c3_image.h"
#include "board_variant.h"
#include "device_identity.h"   /* identity / identity-wipe */
#include "b64url.h"
#include "w25q.h"
#include "ota.h"
#include "flash_layout.h"
#include "flash_compat.h"
#include "pod_policy.h"
#include "cloud_extras.h"
#include "fw_sign.h"
#include "lan_tls.h"
#include "lwip/stats.h"    /* lwIP memory high-water marks in status */
#include "lwip/memp.h"
#include "status_led.h"
#if defined(BENCHPOD_EMI_TEST)
#include "emi_test.h"
#endif

#if defined(BENCHPOD_WIFI_BRIDGE)
#include "wifi_bridge_net.h"
#include "wifi_manager.h"
#endif


/* Reprogram the config flash with a specific gateware image and reconfigure — the runtime
 * IMAGE SWITCH.  SB_WARMBOOT cannot reconfigure at runtime on this board (its config-SPI
 * pins are the shared PSRAM bus), so switching copies the selected image from its W25Q blob
 * slot (blob_store.h) to offset 0 of the same flash via the proven ice40_flash_program +
 * CRESET-reconfig path (~2 s).  n: 0=closed-loop, 1=deep-DAC-replay, 2=sensor emulation.
 * Returns 0 ok, -1 fail. */
uint8_t ice40_embedded_gw_version(void) { return (uint8_t)blob_manifest_gw_version(); }

int ice40_reflash_image(int n)
{
    blob_id_t id = (blob_id_t)ice40_image_slot(n);
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
    /* Record which image this is (the slot and its header's sha256), so the boot check sees a new
       image of the same gateware version (boot_policy_gateware_action). */
    gw_ident_t ident;
    gw_ident_make(&ident, (uint16_t)id, blob_store_info(id)->len, blob_store_info(id)->sha256);
    psram_reset_to_spi();
    int rc = ice40_flash_program_src_ident(blob_store_read, (void *)(uintptr_t)id, blob_store_info(id)->len, &ident);
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

/* Reload the iCE40 from its config flash as it is: no erase, no write (ice40_flash.h). The same
   PSRAM handling around the configuration as the image switch above. */
int ice40_reload(void)
{
    psram_reset_to_spi();
    int rc = ice40_flash_boot();
    psram_init();
    psram_bus_handover();
    signal_engine_on_gateware_reconfigured();
    command_handler_on_gateware_reconfigured();
    return rc;
}

/* Compare the config flash with gateware slot n and record its identity when equal (ice40_flash.h).
   The same PSRAM handling around the reconfiguration as ice40_reload. */
int ice40_image_matches_slot(int n)
{
    blob_id_t id = (blob_id_t)ice40_image_slot(n);
    if (!blob_store_present(id)) return -1;
    gw_ident_t ident;
    gw_ident_make(&ident, (uint16_t)id, blob_store_info(id)->len, blob_store_info(id)->sha256);
    psram_reset_to_spi();
    int same = ice40_flash_matches_src_ident(blob_store_read, (void *)(uintptr_t)id, blob_store_info(id)->len, &ident);
    psram_init();
    psram_bus_handover();
    signal_engine_on_gateware_reconfigured();
    command_handler_on_gateware_reconfigured();
    return same;
}

/* The W25Q slot of gateware image n (0 loop, 1 deep, 2 emulation). */
int ice40_image_slot(int n)
{
    return n == 2 ? (int)BLOB_GW2 : n == 1 ? (int)BLOB_GW1 : (int)BLOB_GW0;
}

/* The image the iCE40 boots from equals gateware slot gw0, gw1 or gw2 (a read compare, no write). */
int ice40_boot_image_intact(void)
{
    int known = 0;
    for (int n = 0; n < 3; n++) {
        blob_id_t id = (blob_id_t)ice40_image_slot(n);
        if (!blob_store_present(id)) continue;
        known = 1;
        psram_reset_to_spi();
        int same = ice40_flash_matches_src(blob_store_read, (void *)(uintptr_t)id, blob_store_info(id)->len);
        psram_init();
        psram_bus_handover();
        signal_engine_on_gateware_reconfigured();
        command_handler_on_gateware_reconfigured();
        if (same == 1) return 1;
        if (same < 0) return -1;
    }
    return known ? 0 : -1;
}

#include "console_prompt.h"   /* CONSOLE_LINE_MAX */

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
       The console keeps what needs physical presence or works without the network (firmware
       4.0.0); everything else is the JSON API (docs/API.md), on the USB network adapter too. */
    out(ctx,
        "recovery console (everything else is the JSON API: :8443 TLS, :8080 over the USB network;\r\n"
        "the commands and their replies: docs/usb-serial-interface.md):\r\n"
        "  help | status | ping | selftest | reboot | dfu\r\n"
        "  identity                                   the device key's short id and public key\r\n"
        "  usb-network [on|off]                       the USB network adapter\r\n"
        "  lan-policy [open|locked|off]               what Ethernet and Wi-Fi may do\r\n"
        "  lan-tls [https-only on|off | scpi on|off | regenerate]   TLS, plaintext :8080, SCPI\r\n"
        "  net-reset                                  forget Wi-Fi, Ethernet back to autonegotiation\r\n"
        "  log [error|warn|info|debug]                console log level\r\n");
#if !defined(BENCHPOD_RELEASE)
    out(ctx,
        "development build only:\r\n"
        "  uid | led <color|mask> | i2c-scan | ina | expdump   MCU id, status LED, I2C bus, INA monitors, expanders\r\n"
        "  test-bootloop [net|hw] yes | test-hang net|hw yes | test-crash yes   prove safe mode / crash reports\r\n"
        "  test-ice40-hold yes  hold the iCE40 in reset: the idle check reloads it by itself\r\n"
        "  psram-selftest | psram-test | psram | psram-addrtest | psram-bench [kb] | psram-clk <p> | psram-chunk <b>\r\n"
        "  cap-selftest [n] | capture-psram [n] | dualcap [an] [ln] | lastress [n]   capture-path diagnostics\r\n"
        "  spi-clk <64|128|256> | spi-diag [n] | adc-spi [n] | adcraw | flash-id | ca-damage   low-level debug\r\n"
        "  flash-ice40 | ice40-erase [gw0] yes   rewrite / blank the iCE40 config flash\r\n"
        "  wifi-static <ip> <mask> <gw> | esp-reset-pulse | esp-mon [ms] | flash-esp32-sync   Wi-Fi module\r\n");
#endif
#if defined(BENCHPOD_EMI_TEST)
    out(ctx,
        "EMI test build (runs the EMC activity loop from boot):\r\n"
        "  emi [status] | emi start | emi stop   the loop's state, start it, stop it (outputs and power off)\r\n"
        "  emi dac all|sine|dc|current|off       what the DAC plays: 1 kHz sine on 3.3 V, 3 V DC on 5 V, 12 mA,\r\n"
        "                                        or all three (quad-DAC boards, their default)\r\n"
        "  emi uart <1-14>|off                   NMEA at 115200 baud out of an LA pin (drives a DUT pin)\r\n"
        "  emi log [n|all]                       the run log in the W25Q: its state, and the last n records\r\n"
        "                                        (default 20) or all of them, printed on the device log\r\n"
        "  emi fault ice40|ice40-stuck|off       test the recovery: hold the iCE40 in reset once, or after\r\n"
        "                                        every reload and across resets (until `off` or power-off)\r\n");
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
    uint32_t want = configCPU_CLOCK_HZ, tol = want / 100u;
    bool sys_ok = src_pll &&
                  (sysclk > want ? sysclk - want : want - sysclk) <= tol;
    ST("sysclk", sys_ok, "%lu Hz (expect ~%luM, src=%s)",
       (unsigned long)sysclk, (unsigned long)(want / 1000000u), src_pll ? "PLL1" : "other");

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


#if defined(BENCHPOD_WIFI_BRIDGE)

/* ---- Wi-Fi bridge: net-reset clears the Wi-Fi on the external bridge box when one says HELLO
   on the Ethernet port, unless built-in Wi-Fi is configured (it wins, wifi_bridge_ignored). */

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

#endif

/* `ca`: one line per company CA certificate (cloud_extras_ca_each). */

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

/* Not a console command. (Firmware 3.9.0 to 3.10.13 answered the commands it had moved to the
   JSON API with a line saying so; 4.0.0 treats them like any other.) */
static void con_fallback(const char *verb, console_out_t out, void *ctx)
{
    op(out, ctx, "  unknown command '%s' (try 'help')\r\n", verb);
}

static void con_help(int argc, char **argv, console_out_t out, void *ctx)
{
    cmd_help(out, ctx);
}

static void con_ping(int argc, char **argv, console_out_t out, void *ctx)
{
    uint8_t ver = 0;
    if (fpga_ping(2, &ver) == 0) op(out, ctx, "  PING ok  version=0x%02x\r\n", ver);
    else                         op(out, ctx, "  PING failed\r\n");
}

static void con_status(int argc, char **argv, console_out_t out, void *ctx)
{
    /* Stable identifier so a host can tell a real bench pod apart from other
       USB-serial devices: tooling just greps the output for "benchpod". */
    op(out, ctx, "  device : benchpod\r\n");
    op(out, ctx, "  mac    : %s\r\n", net_mac_str());
    {
        /* The MAC the pod uses on Wi-Fi, for a network that registers devices first. */
        char wm[18];
        if (cloud_caps_wifi_mac(wm))
            op(out, ctx, "  wifimac: %s\r\n", wm[0] ? wm : "not known yet (wifi-scan or a join tells it)");
    }
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
    {
        uint32_t rd_retried = 0, rd_failed = 0; int rd_rc = 0;
        signal_engine_psram_read_stats(&rd_retried, &rd_failed, &rd_rc);
        op(out, ctx, "  psram rd : %lu read-backs retried, %lu failed (last rc %d)\r\n",
           (unsigned long)rd_retried, (unsigned long)rd_failed, rd_rc);
    }
    op(out, ctx, "  ice40boot : %s\r\n", ice40_boot_note());
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
    bool     deep    = gw_caps()->dac_deep_replay;
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
    if (esp_wifi_ctrl_module_present()) {
        const c3_restarts_t *r = esp_wifi_ctrl_restarts();
        op(out, ctx, "  c3     : %lu restarts since power-on%s%s\r\n", (unsigned long)r->total,
           r->total ? ", last: " : "", r->total ? r->last_reason : "");
    }
    {
        sys_health_task_t t[SYS_HEALTH_MAX_TASKS];
        int n = sys_health_tasks(t, SYS_HEALTH_MAX_TASKS);
        for (int i = 0; i < n; i++)
            op(out, ctx, "  stack  : %-7s %u B free (min)\r\n", t[i].name, t[i].stack_free);
    }
    op(out, ctx, "  flash  : %lu KB%s\r\n", (unsigned long)(flash_layout_size() / 1024u),
       flash_compat_refused() ? ", RECORD WRITES REFUSED (outside the record area: a firmware bug)" : "");
    op(out, ctx, "  lwip   : mem max %lu of %lu B, pbuf pool max %u of %u\r\n",
       (unsigned long)lwip_stats.mem.max, (unsigned long)lwip_stats.mem.avail,
       (unsigned)lwip_stats.memp[MEMP_PBUF_POOL]->max, (unsigned)lwip_stats.memp[MEMP_PBUF_POOL]->avail);
    op(out, ctx, "  heap   : %u B free, %u B min\r\n",
       sys_health_heap_free(), sys_health_heap_min_free());
}

#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
static void con_spi_clk(int argc, char **argv, console_out_t out, void *ctx)
{
    if (argc < 2) { con_fallback(argv[0], out, ctx); return; }   /* needs an argument */
    uint32_t div = (uint32_t)atoi(argv[1]);
    uint32_t hz = signal_engine_spi_set_prescaler(div);
    if (hz) op(out, ctx, "  SPI1 SCK = %lu Hz (/%lu)\r\n", (unsigned long)hz, (unsigned long)div);
    else    op(out, ctx, "  bad divisor (128 or 256; 64 needs gateware >= 52; /32 and faster exceed the iCE40's SCK sampling)\r\n");
}

static void con_spi_diag(int argc, char **argv, console_out_t out, void *ctx)
{
    int n = (argc >= 2) ? atoi(argv[1]) : 50;
    signal_engine_spi_diag(n);
    op(out, ctx, "  (spi-diag printed on the device log)\r\n");
}

static void con_adc_spi(int argc, char **argv, console_out_t out, void *ctx)
{
    /* Read the live ADC sample DIRECTLY over SPI (CMD_ADC_PROBE), skipping
       PSRAM + the 24->48 CDC — isolates 0x5555 to engine/silicon vs path. */
    int n = (argc >= 2) ? atoi(argv[1]) : 16;
    signal_engine_adc_spi_diag(n);
    op(out, ctx, "  (adc-spi printed on the device log)\r\n");
}
#endif

#if defined(BENCHPOD_EMI_TEST)
static void con_emi(int argc, char **argv, console_out_t out, void *ctx)
{
    /* The EMC test build's activity loop (emi_test.h). */
    const char *sub = argc >= 2 ? argv[1] : "status";
    int mode = argc >= 3 ? emi_test_dac_from_name(argv[2]) : -1;
    int la   = argc >= 3 && strcmp(argv[2], "off") ? atoi(argv[2]) : 0;
    if (!strcmp(sub, "start"))                     emi_test_start();
    else if (!strcmp(sub, "stop"))                 emi_test_stop();
    else if (!strcmp(sub, "dac") && mode >= 0) {
        if (!emi_test_set_dac((emi_dac_mode_t)mode)) {
            op(out, ctx, "  emi dac: %s\r\n", board_has_analog()
                ? "all needs the quad DAC (analog v3r2 or the v4 analog module)"
                : "this board has no DAC (digital-only board)");
            return;
        }
    }
    else if (!strcmp(sub, "uart") && la >= 0 && la <= 14 && (la > 0 || !strcmp(argv[2], "off")))
        emi_test_set_uart(la);
    else if (!strcmp(sub, "net") && argc >= 3 && ((argv[2][0] >= '0' && argv[2][0] <= '9') || !strcmp(argv[2], "off"))) {
        if (!emi_test_set_net_rate(strcmp(argv[2], "off") ? (unsigned)atoi(argv[2]) : 0u)) {
            op(out, ctx, "  emi net: at most %u Mbit/s\r\n", EMI_NET_MAX_MBIT);
            return;
        }
    }
    else if (!strcmp(sub, "log")) {
        /* The run log (emi_log.h): its state here, the records on the device log. */
        char info[320];
        emi_test_log_describe(info, sizeof(info));
        op(out, ctx, "  %.150s\r\n", info);
        if (strlen(info) > 150) op(out, ctx, "  %s\r\n", info + 150);
#if defined(EMI_LOG_TEST_SECTORS)
        if (argc >= 4 && !strcmp(argv[2], "fill")) {   /* a small-ring test build only */
            emi_test_log_fill((uint32_t)atol(argv[3]));
            return;
        }
#endif
        const bool all = argc >= 3 && !strcmp(argv[2], "all");
        const long n = argc >= 3 && !all ? atol(argv[2]) : 20;
        if (!all && n <= 0) { op(out, ctx, "usage: emi log [n|all]\r\n"); return; }
        if (emi_test_log_print(all ? 0u : (uint32_t)n))
            op(out, ctx, "  (records printed on the device log)\r\n");
        return;
    }
    else if (!strcmp(sub, "fault")) {
        /* Test the recovery ladder (recovery.h) with a failure made on purpose. */
        const char *f = argc >= 3 ? argv[2] : "";
        if (!emi_test_fault(f)) { op(out, ctx, "usage: emi fault ice40|ice40-stuck|off\r\n"); return; }
        op(out, ctx, "  emi fault %s\r\n", f);
        return;
    }
    else if (strcmp(sub, "status")) {
        op(out, ctx, "usage: emi [status] | emi start | emi stop | emi dac all|sine|dc|current|off"
                     " | emi uart <1-14>|off | emi net <mbit>|off | emi log [n|all]"
                     " | emi fault ice40|ice40-stuck|off\r\n");
        return;
    }
    char line[256];
    emi_test_describe(line, sizeof(line));
    op(out, ctx, "  %s\r\n", line);
}
#endif

#if !defined(BENCHPOD_RELEASE)   /* development builds only (firmware 4.0.0) */
static void con_led(int argc, char **argv, console_out_t out, void *ctx)
{
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
}
#endif

#if !defined(BENCHPOD_RELEASE)   /* development builds only (firmware 4.0.0) */
static void con_psram_selftest(int argc, char **argv, console_out_t out, void *ctx)
{
    /* Layered PSRAM datapath test: STM32<->PSRAM, iCE40->PSRAM write, and (if
       that fails) whether the iCE40 even reaches the /CE net.  Same test run
       once at boot. */
    psram_boot_selftest();
    op(out, ctx, "  (psram-selftest printed on the device log)\r\n");
}
#endif

#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
static void con_cap_selftest(int argc, char **argv, console_out_t out, void *ctx)
{
    /* End-to-end ADC->PSRAM path test: iCE40 streams a known ramp through the
       real capture datapath (CDC+writer+PSRAM+readback).  PASS => the path is
       healthy; FAIL => iCE40 capture/CDC timing error, NOT the analog ADC. */
    int n = (argc >= 2) ? atoi(argv[1]) : 64;
    int bad = -1;
    int rc = signal_engine_capture_selftest(n, &bad);
    op(out, ctx, "  cap-selftest: %s (see device log)\r\n",
       rc == 0 ? "PASS — ADC->PSRAM path OK" : "FAIL — iCE40 capture/CDC timing error");
}

static void con_i2c_scan(int argc, char **argv, console_out_t out, void *ctx)
{
    i2c_bus_status();   /* prints to the local console log */
    op(out, ctx, "  (scan printed on the device log)\r\n");
}

static void con_ina(int argc, char **argv, console_out_t out, void *ctx)
{
    cmd_ina(out, ctx);
}
#endif

#if !defined(BENCHPOD_RELEASE)   /* development builds only (firmware 4.0.0) */
static void con_uid(int argc, char **argv, console_out_t out, void *ctx)
{
    /* uid: the 96-bit unique ID.  Words only: a byte read of this area is a precise bus
       fault on the H5 (proven on the v3, bfar=0x08fff800; see board_uid.h). */
    uint32_t w[3];
    board_uid_words(w);
    op(out, ctx, "  uid (words): %08lx %08lx %08lx\r\n",
       (unsigned long)w[0], (unsigned long)w[1], (unsigned long)w[2]);
}
#endif

#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
static void con_test_bootloop(int argc, char **argv, console_out_t out, void *ctx)
{
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
}

static void con_ice40_erase(int argc, char **argv, console_out_t out, void *ctx)
{
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
}

static void con_test_crash(int argc, char **argv, console_out_t out, void *ctx)
{
    /* Prove the crash report: a real fault (UsageFault/HardFault) in the console task. */
    if (argc < 2 || strcmp(argv[1], "yes")) {
        op(out, ctx, "  test-crash yes : fault on purpose; the next boot reports it as the last crash\r\n");
    } else {
        op(out, ctx, "  crashing now\r\n");
        vTaskDelay(pdMS_TO_TICKS(100));
        __builtin_trap();
    }
}

static void con_test_hang(int argc, char **argv, console_out_t out, void *ctx)
{
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
}

static void con_test_ice40_hold(int argc, char **argv, console_out_t out, void *ctx)
{
    /* Prove the idle iCE40 check (hw_worker.c, recovery.h): hold the iCE40 in reset as an ESD
       hit that unconfigured it would leave it; the pod reloads it from its flash by itself. */
    if (argc < 2 || strcmp(argv[1], "yes")) {
        op(out, ctx, "  test-ice40-hold yes : hold the iCE40 in reset; the idle check reloads it (~10 s)\r\n");
    } else {
        ice40_hold_off_bus();
        op(out, ctx, "  the iCE40 is held in reset\r\n");
    }
}

static void con_expdump(int argc, char **argv, console_out_t out, void *ctx)
{
    analog_switch_dump();
}

static void con_adcraw(int argc, char **argv, console_out_t out, void *ctx)
{
    /* adcraw — single raw ADC probe byte (debug). */
    uint8_t val = 0;
    if (adc_probe_one(&val) == 0) op(out, ctx, "  ADC=%u (0x%02x)\r\n", val, val);
    else                          op(out, ctx, "  adc-probe failed\r\n");
}

static void con_psram(int argc, char **argv, console_out_t out, void *ctx)
{
    /* Diagnostic path: reports bus/CS/arbitration state and re-probes the ID
       with the iCE40 held off the shared bus (see psram_diag). */
    if (psram_diag() == 0) op(out, ctx, "  PSRAM ID ok with iCE40 in reset (arbitration is the issue)\r\n");
    else                   op(out, ctx, "  PSRAM ID failed even with iCE40 in reset (see device log)\r\n");
}

static void con_psram_bench(int argc, char **argv, console_out_t out, void *ctx)
{
    uint32_t kb = (argc >= 2) ? (uint32_t)atoi(argv[1]) : 256;
    psram_bench(kb);
    op(out, ctx, "  (psram-bench printed on the device log)\r\n");
}

static void con_psram_clk(int argc, char **argv, console_out_t out, void *ctx)
{
    if (argc < 2) { con_fallback(argv[0], out, ctx); return; }   /* needs an argument */
    uint32_t p = (uint32_t)atoi(argv[1]);
    uint32_t hz = psram_set_prescaler(p);
    if (hz) op(out, ctx, "  OCTOSPI SCLK = %lu Hz (presc %lu)\r\n", (unsigned long)hz, (unsigned long)p);
    else    op(out, ctx, "  psram-clk failed\r\n");
}

static void con_psram_chunk(int argc, char **argv, console_out_t out, void *ctx)
{
    if (argc < 2) { con_fallback(argv[0], out, ctx); return; }   /* needs an argument */
    uint32_t n = psram_set_chunk((uint32_t)atoi(argv[1]));
    op(out, ctx, "  PSRAM tCEM chunk = %lu bytes\r\n", (unsigned long)n);
}
#endif

#if !defined(BENCHPOD_RELEASE)   /* development builds only (firmware 4.0.0) */
static void con_psram_test(int argc, char **argv, console_out_t out, void *ctx)
{
    if (psram_init() == 0 && psram_test() == 0) op(out, ctx, "  PSRAM ok\r\n");
    else                                        op(out, ctx, "  PSRAM failed (see device log)\r\n");
}
#endif

#if !defined(BENCHPOD_RELEASE)   /* development builds only: bench diagnostics */
static void con_psram_addrtest(int argc, char **argv, console_out_t out, void *ctx)
{
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
}

static void con_capture_psram(int argc, char **argv, console_out_t out, void *ctx)
{
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
}

static void con_dualcap(int argc, char **argv, console_out_t out, void *ctx)
{
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
}

static void con_lastress(int argc, char **argv, console_out_t out, void *ctx)
{
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
}

static void con_flash_id(int argc, char **argv, console_out_t out, void *ctx)
{
    uint8_t id[3] = {0};
    if (ice40_flash_read_id(id) == 0)
        op(out, ctx, "  iCE40 flash JEDEC ID: %02x %02x %02x (W25Q64 = ef 40 17)\r\n",
           id[0], id[1], id[2]);
    else
        op(out, ctx, "  flash-id failed (see device log)\r\n");
}
#endif

#if !defined(BENCHPOD_RELEASE)   /* development builds only (firmware 4.0.0) */
static void con_flash_ice40(int argc, char **argv, console_out_t out, void *ctx)
{
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
}
#endif

#if !defined(BENCHPOD_RELEASE)
static void con_ca_damage(int argc, char **argv, console_out_t out, void *ctx)
{
    /* Development builds: damage the company CA slot to try the built-in-roots fallback. */
    const char *why = cloud_extras_ca_damage();
    if (why) op(out, ctx, "ca-damage error %s\r\n", why);
    else     op(out, ctx, "ca-damage ok (ca error %s)\r\n",
                cloud_extras_ca_error() ? cloud_extras_ca_error() : "none");
}
#endif

static void con_usb_network(int argc, char **argv, console_out_t out, void *ctx)
{
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
}

static void con_lan_tls(int argc, char **argv, console_out_t out, void *ctx)
{
    /* The LAN API over TLS (lan_tls.h): show the certificate, set HTTPS-only (on by default from
       4.0.0) and SCPI (off by default), regenerate. */
    const char *why = NULL;
    const bool onoff = argc >= 3 && (!strcmp(argv[2], "on") || !strcmp(argv[2], "off"));
    if (onoff && !strcmp(argv[1], "https-only"))
        why = pod_policy_set_https_only(!strcmp(argv[2], "on"), POLICY_SRC_USB, false);
    else if (onoff && !strcmp(argv[1], "scpi"))
        why = pod_policy_set_scpi(!strcmp(argv[2], "on"), POLICY_SRC_USB, false);
    else if (argc >= 2 && !strcmp(argv[1], "regenerate"))
        why = lan_tls_regenerate();
    else if (argc >= 2)
        why = "use https-only on|off, scpi on|off or regenerate";
    static lan_tls_info_t in;   /* console task only */
    lan_tls_get_info(&in);
    if (why)            op(out, ctx, "lan-tls error %s\r\n", why);
    /* "scpi on|off" (4.0.0) goes last, so a reader of the older line still finds every field. */
    else if (!in.ready) op(out, ctx, "lan-tls port %d https-only %s none %s scpi %s\r\n", LAN_TLS_PORT,
                           pod_policy_https_only() ? "on" : "off", in.error, pod_policy_scpi() ? "on" : "off");
    else {
        static char line[256];   /* console task only; longer than op()'s 160 B (sha256 + subject) */
        snprintf(line, sizeof(line), "lan-tls port %d https-only %s sha256 %s %s \"%s\" \"%s\" scpi %s\r\n",
                 LAN_TLS_PORT, pod_policy_https_only() ? "on" : "off", in.sha256, in.origin, in.key,
                 in.subject, pod_policy_scpi() ? "on" : "off");
        out(ctx, line);
    }
}

static void con_log(int argc, char **argv, console_out_t out, void *ctx)
{
    const char *why = NULL;
    if (argc >= 2) {
        int lv = log_level_from_name(argv[1]);
        why = lv < 0 ? "use error, warn, info or debug" : pod_policy_set_log_level((bp_log_level_t)lv);
    }
    if (why) op(out, ctx, "log error %s\r\n", why);
    else     op(out, ctx, "log %s\r\n", log_level_name(log_level()));
}

static void con_lan_policy(int argc, char **argv, console_out_t out, void *ctx)
{
    const char *why = NULL;
    if (argc >= 2) {
        int p = pod_policy_lan_from_name(argv[1]);
        why = p < 0 ? "use open, locked or off" : pod_policy_set_lan((pod_lan_policy_t)p, POLICY_SRC_USB);
    }
    if (why) op(out, ctx, "lan-policy error %s\r\n", why);
    else     op(out, ctx, "lan-policy %s\r\n", pod_policy_lan_name(pod_policy_lan()));
}

#if !defined(BENCHPOD_RELEASE)   /* development builds only (firmware 4.0.0) */
static void con_wifi_static(int argc, char **argv, console_out_t out, void *ctx)
{
    if (argc >= 4) net_wifi_static(argv[1], argv[2], argv[3]);
    else op(out, ctx, "  usage: wifi-static <ip> <netmask> <gateway>  (e.g. 172.20.10.9 255.255.255.240 172.20.10.1)\r\n");
}
#endif

static void con_identity(int argc, char **argv, console_out_t out, void *ctx)
{
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
}

static void con_selftest(int argc, char **argv, console_out_t out, void *ctx)
{
    cmd_selftest(out, ctx);
}

static void con_reboot(int argc, char **argv, console_out_t out, void *ctx)
{
    op(out, ctx, "  rebooting...\r\n");
    HAL_Delay(50);
    boot_guard_planned_reset(true);
    NVIC_SystemReset();
}

#if !defined(BENCHPOD_RELEASE)   /* development builds only (firmware 4.0.0) */
/* The ESP32-C3 commands' common refusals: no C3 on this board, or a flash of it is running.
   True = refused (the reply is sent). */
static bool esp_refused(const char *verb, console_out_t out, void *ctx)
{
    if (!esp_wifi_ctrl_module_present()) {
        op(out, ctx, "  %s: " ESP_WIFI_NO_MODULE "\r\n", verb);
        return true;
    }
    if (esp_rom_flash_busy()) {
        uint32_t done = 0, total = 0;
        (void)esp_rom_flash_progress(&done, &total);
        op(out, ctx, "  %s refused: the ESP32-C3 is being flashed (block %lu/%lu); wait for the "
                     "\"[espflash] done\" line\r\n", verb, (unsigned long)done, (unsigned long)total);
        return true;
    }
    return false;
}
#endif

#if !defined(BENCHPOD_RELEASE)   /* development builds only (firmware 4.0.0) */
static void con_flash_esp32_sync(int argc, char **argv, console_out_t out, void *ctx)
{
    if (esp_refused(argv[0], out, ctx)) return;
    net_wifi_hold_for_flash(true);  /* Wi-Fi control must not drive EN/BOOT meanwhile */
    uint32_t magic = 0;
    if (esp_rom_flash_sync(&magic) == 0)
        op(out, ctx, "  C3 ROM synced (chip magic 0x%08lx)\r\n", (unsigned long)magic);
    else
        op(out, ctx, "  flash-esp32-sync failed — no C3 ROM response (see log)\r\n");
    net_wifi_hold_for_flash(false); /* the C3 was left in its ROM loader: restart Wi-Fi */
}
#endif

#if !defined(BENCHPOD_RELEASE)   /* development builds only (firmware 4.0.0) */
static void con_esp_reset_pulse(int argc, char **argv, console_out_t out, void *ctx)
{
    if (esp_refused(argv[0], out, ctx)) return;
    /* Diagnostic: reset the ESP32-C3 behind the Wi-Fi control's back (EN low 50 ms), which
       is what a C3 crash or brown-out looks like from here.  The control must notice (the
       C3's new boot event, or its RSSI answers stopping), drop the link and rejoin; see
       wifi_status c3_lost_reboot / c3_lost_noresp. */
    HAL_GPIO_WritePin(ESP_EN_PORT, ESP_EN_PIN, GPIO_PIN_RESET);
    sleep_ms(50);
    HAL_GPIO_WritePin(ESP_EN_PORT, ESP_EN_PIN, GPIO_PIN_SET);
    op(out, ctx, "  C3 reset pulsed (50 ms); watch wifi_status\r\n");
}
#endif

#if !defined(BENCHPOD_RELEASE)   /* development builds only (firmware 4.0.0) */
static void con_esp_mon(int argc, char **argv, console_out_t out, void *ctx)
{
    if (esp_refused(argv[0], out, ctx)) return;
    uint32_t ms = (argc >= 2) ? (uint32_t)atoi(argv[1]) : 5000;
    if (ms > 20000) ms = 20000;
    esp_uart_monitor(ms);
}
#endif

/* net-reset: the way back to a pod that no network reaches (firmware 4.0.0; wifi-clear up to
   3.10.13 did the Wi-Fi part). Forgets the Wi-Fi credentials (the pod's flash and the ESP32-C3's
   NVS, or the Wi-Fi bridge's), which turns the USB network adapter back on, and puts Ethernet back
   to autonegotiation with a PHY reset and a new DHCP lease. The policies, the cloud settings and the
   device key stay as they are (lan-policy and usb-network have their own commands). */
static void con_net_reset(int argc, char **argv, console_out_t out, void *ctx)
{
    net_eth_force_speed(0, 0);   /* autonegotiation, PHY reset, DHCP again */
    op(out, ctx, "net-reset eth autonegotiation, link and DHCP again\r\n");
#if defined(BENCHPOD_WIFI_BRIDGE)
    if (wifi_bridge_in_use()) {
        const char *err = NULL;
        int rc = wifi_bridge_call(BWP_CLEAR_WIFI, NULL, 0, 1000, &g_wifi_bridge_reply, &err);
        if (!bridge_failed(out, ctx, "net-reset wifi", rc, err))
            op(out, ctx, "net-reset wifi cleared on the Wi-Fi bridge\r\n");
        return;
    }
#endif
    /* Drops Wi-Fi and wipes both copies: config_store, then the ESP32-C3's NVS (~3 s). */
    config_clear();
    net_wifi_reload();          /* drop Wi-Fi now, whatever happens to the C3 */
    const char *why = esp_wifi_ctrl_module_present() ? wifi_clear_c3_now() : NULL;
    if (!esp_wifi_ctrl_module_present())   /* digital v3r3: no C3, no second copy */
        op(out, ctx, "net-reset wifi cleared (pod flash; this BenchPod has no ESP32-C3)\r\n");
    else if (!why)
        op(out, ctx, "net-reset wifi cleared (pod flash and the ESP32-C3's NVS)\r\n");
    else
        op(out, ctx, "net-reset wifi cleared on the pod, but the ESP32-C3's NVS was not erased (%s): "
                     "run net-reset again, or install the esp image again\r\n", why);
}

static void con_dfu(int argc, char **argv, console_out_t out, void *ctx)
{
    /* Firmware 4.0.0: always allowed. The ROM bootloader writes any image, signed or not, and
       that is on purpose: with signatures always required on every OTA path, USB DFU is the one
       way to install firmware you built yourself, and it needs the pod on a USB cable. (3.10.7
       to 3.10.13 refused it under "required", which the console could loosen.) */
    op(out, ctx, "  entering DFU (USB bootloader) — flash with dfu-util / `benchpod flash-self`...\r\n");
    HAL_Delay(50);          /* let the notice drain before USB re-enumerates */
    boot_guard_planned_reset(true);
    dfu_boot_request();     /* arms the flag + system reset; never returns */
}

/* One row per console command (check_docs_coverage.py reads the names from here).  A name that is
   not here goes to con_fallback ("unknown command").
   Development-only commands are not in a release build's table at all.  `help` is its own text:
   it groups several commands per line, which a per-row help string would not reproduce. */
typedef void (*console_fn_t)(int argc, char **argv, console_out_t out, void *ctx);
typedef struct {
    const char  *name;
    console_fn_t fn;
} console_cmd_t;

static const console_cmd_t k_console_cmds[] = {
    /* Release builds (firmware 4.0.0): a recovery console. What needs someone at the pod, or works
       when no network reaches it; everything else is the JSON API, on the USB network adapter too. */
    { "help", con_help },
    { "status", con_status },
    { "ping", con_ping },
    { "selftest", con_selftest },
    { "reboot", con_reboot },
    { "dfu", con_dfu },
    { "identity", con_identity },
    { "usb-network", con_usb_network },
    { "lan-policy", con_lan_policy },
    { "lan-tls", con_lan_tls },
    { "net-reset", con_net_reset },
    { "log", con_log },
#if !defined(BENCHPOD_RELEASE)
    /* development builds only: bench diagnostics */
    { "uid", con_uid },
    { "led", con_led },
    { "spi-clk", con_spi_clk },
    { "spi-diag", con_spi_diag },
    { "adc-spi", con_adc_spi },
    { "adcraw", con_adcraw },
    { "i2c-scan", con_i2c_scan },
    { "ina", con_ina },
    { "expdump", con_expdump },
    { "psram-selftest", con_psram_selftest },
    { "psram-test", con_psram_test },
    { "psram", con_psram },
    { "psram-addrtest", con_psram_addrtest },
    { "psram-bench", con_psram_bench },
    { "psram-clk", con_psram_clk },
    { "psram-chunk", con_psram_chunk },
    { "cap-selftest", con_cap_selftest },
    { "capture-psram", con_capture_psram },
    { "dualcap", con_dualcap },
    { "lastress", con_lastress },
    { "flash-id", con_flash_id },
    { "flash-ice40", con_flash_ice40 },
    { "ice40-erase", con_ice40_erase },
    { "test-bootloop", con_test_bootloop },
    { "test-crash", con_test_crash },
    { "test-hang", con_test_hang },
    { "test-ice40-hold", con_test_ice40_hold },
    { "ca-damage", con_ca_damage },
    { "wifi-static", con_wifi_static },
    { "esp-reset-pulse", con_esp_reset_pulse },
    { "esp-mon", con_esp_mon },
    { "flash-esp32-sync", con_flash_esp32_sync },
#endif
#if defined(BENCHPOD_EMI_TEST)
    { "emi", con_emi },
#endif
};

/* Runs on the hw worker (hw_worker_submit_console), the one task that drives the instrument
   hardware, so it takes no lock of its own: the worker runs one job at a time. */
void console_exec_hw(char *cmd, console_out_t out, void *ctx)
{
    /* Up to 11 tokens so `can write <id> b0..b7` (2 + 1 + 8) fits; other verbs
       use far fewer. */
    char *argv[11] = {0};
    int argc = 0;
    char *tok = strtok(cmd, " \t");
    while (tok && argc < 11) { argv[argc++] = tok; tok = strtok(NULL, " \t"); }
    if (argc == 0) return;

    const bool gated = console_needs_heavy_gate(argv[0]);
    if (gated && !command_handler_acquire_adc(CH_CONSOLE_TEXT_OWNER)) {
        op(out, ctx, "  %s refused: busy: a capture, upload or update is using the PSRAM bus; "
                     "try again when it ends\r\n", argv[0]);
        return;
    }

    const console_cmd_t *c = NULL;
    for (size_t i = 0; i < sizeof(k_console_cmds) / sizeof(k_console_cmds[0]) && !c; i++)
        if (!strcmp(argv[0], k_console_cmds[i].name)) c = &k_console_cmds[i];
    if (c) c->fn(argc, argv, out, ctx);
    else   con_fallback(argv[0], out, ctx);
    if (gated) command_handler_release_adc(CH_CONSOLE_TEXT_OWNER);
}

/* ---- local interactive console ----------------------------------------- */

/* The prompt and the typed line live in console_io (console_prompt.h), under the console lock,
   so background log lines never land after a bare prompt or inside a typed line. */

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
    console_exec_hw(buf, local_out, NULL);
    console_io_prompt();
}

static volatile bool s_console_ready;

void console_init(void)
{
    /* Nothing yet: the first prompt waits for the iCE40/PSRAM bring-up (console_boot_done), so
       the boot lines come before it, not after. Input typed meanwhile waits in the RX ring. */
}

void console_boot_done(void)
{
    if (s_console_ready) return;
    log_reply("\r\nconsole ready — type 'help'\r\n");
    console_io_prompt();
    s_console_ready = true;
}

void console_poll(void)
{
    /* Persists across polls: a CR and its trailing LF can arrive in separate
       drains.  Swallowing the LF of a CR-LF stops the prompt firing twice (the
       "> on its own line" artifact with terminals that send CR-LF). */
    static int last_was_cr = 0;
    /* Before the bring-up ends the prompt is not up; also give up waiting after 30 s, so a
       bring-up that hangs (until the watchdog) never leaves the console deaf. */
    if (!s_console_ready) {
        if (HAL_GetTick() < 30000u) return;
        console_boot_done();
    }
    int c;
    while ((c = console_io_getc()) >= 0) {
        if (c == '\n' && last_was_cr) { last_was_cr = 0; continue; }  /* LF of a CR-LF */
        if (c == '\r' || c == '\n') {
            last_was_cr = (c == '\r');
            char line[CONSOLE_LINE_MAX];
            size_t n = console_io_edit_enter(line, sizeof(line));
            if (n > 0) {
                /* Execute on the hw worker (it owns the instrument hardware); the
                   worker reprints the prompt after the command output. */
                if (!hw_worker_submit_console(line)) {
                    console_io_write((const uint8_t *)"busy — try again\r\n", strlen("busy — try again\r\n"));   /* worker queue full */
                    console_io_prompt();
                }
            } else {
                console_io_prompt();
            }
        } else if (c == 0x08 || c == 0x7F) {
            last_was_cr = 0;
            console_io_edit_backspace();
        } else if (c >= 0x20 && c < 0x7F) {
            last_was_cr = 0;
            console_io_edit_char((char)c);
        }
    }
}
