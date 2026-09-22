/*
 * console.c — bench-pod STM32 bring-up console.
 *
 * console_exec() runs one command line and writes results through an output
 * sink, so the same command set serves both the local USB/USART console and the
 * TCP command server (net_server.c).  The full RP2350 command_handler/SCPI/JSON
 * surface lands in later phases.
 */
#include "console.h"
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
#include "esp_rom_flash.h"
#include "esp_hosted_spi.h"   /* stop the Wi-Fi transport before flashing the C3 */
#include "esp_wifi_ctrl.h"    /* wifi-set / wifi-show provisioning */
#include "config_store.h"
#include "stm32h5xx_hal.h"
#include "hw_lock.h"
#include "dfu_boot.h"
#include "fault.h"
#include "sys_health.h"
#include "board_rev.h"
#include "usb_cc.h"
#include "nrst_ctrl.h"
#include "hw_worker.h"
#include "la_pins.h"          /* la-voltage refusal while pins are in use */
#include "net_server.h"       /* net_ip_str() for `status` */
#include "cloud_config.h"     /* provisioned cloud registration, for `status` */
#include "cloud_client.h"     /* cloud_client_state_str() for `status` */
#include "pico/rand.h"        /* get_rand_32 — hardware RNG liveness check */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#include "fpga_bitstream.h"   /* generated: fpga_image0/1[] + _len (or fpga_bitstream[]) */

/* Reprogram the config flash with a specific gateware image and reconfigure — the runtime
 * IMAGE SWITCH.  SB_WARMBOOT cannot reconfigure at runtime on this board (its config-SPI
 * pins are the shared PSRAM bus), so switching reprograms the selected single image via the
 * proven ice40_flash_program + CRESET-reconfig path (~2 s).  n: 0=closed-loop,
 * 1=deep-DAC-replay.  Returns 0 ok, -1 fail. */
int ice40_reflash_image(int n)
{
#ifdef FPGA_HAVE_IMAGES
    const unsigned char *img = (n == 1) ? fpga_image1     : fpga_image0;
    unsigned int         len = (n == 1) ? fpga_image1_len : fpga_image0_len;
#else
    const unsigned char *img = fpga_bitstream;
    unsigned int         len = fpga_bitstream_len;
    (void)n;
    if (len == 0) return -1;
#endif
    /* Runtime image-swap sequence.  The DEEP image's PSRAM-write wedge is fixed IN THE GATEWARE
       (gw v26: the deep reader/arbiter are held in reset while the STM32 owns the bus, so the
       psram_init() bus-yank below can no longer desync them).  So keep the firmware ordering that
       reads the config with the PSRAM in SPI — the safest state for the shared-bus bitstream read:
         psram_reset_to_spi() -> program+boot (config read, PSRAM in SPI) -> psram_init() (QPI for
         the new gateware) -> psram_bus_release() (hand the bus to the iCE40).
       NB: an earlier attempt to run psram_init() BEFORE the boot (so the config read saw a QPI
       PSRAM) intermittently CORRUPTED the deep bitstream — reverted. */
    psram_reset_to_spi();
    int rc = ice40_flash_program(img, (size_t)len);
    psram_init();                       /* re-enter QPI for the new gateware (bus-yank now safe: gw v26) */
    psram_bus_release();                /* hand the shared bus back to the iCE40 */
    /* The fabric just reset every register it owns; the firmware's MIRRORS of those registers did
       not.  Re-sync them here — this is the one point every reconfiguration passes through (image
       swap, this console command, the boot/OTA reflash).  Unconditional: even a FAILED program
       leaves the part reconfigured or held in reset, so the mirrors are stale either way. */
    signal_engine_on_gateware_reconfigured();
    command_handler_on_gateware_reconfigured();
    return rc;
}

#define LINE_MAX 96

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
    /* Emit straight through out() (not op()): the full list is ~830 bytes and
       would be truncated by op()'s 160-byte format buffer — that is what cut
       `help` off mid-line.  Split into a couple of writes to stay well under the
       USB-CDC TX ring. */
    out(ctx,
        "commands:\r\n"
        "  help                 this list\r\n"
        "  ping                 SPI ping the iCE40 (expect 0xA5 + version)\r\n"
        "  status               read the iCE40 STATUS byte\r\n"
        "  i2c-scan             scan the I2C0 power/IO bus\r\n"
        "  ina                  read both INA238 monitors\r\n"
        "  power <1|2> <on|off> enable/disable an eFuse\r\n"
        "  pstat                show eFuse status\r\n"
        "  la-voltage [1800|3300] set/show the LA I/O-bank voltage (required for LA ops)\r\n"
        "  usb-cc              USB-C CC lines: orientation + source current (v3)\r\n"
        "  nrst [assert|release|<ms>]  drive the target reset pin, J1 pin 22 (v3)\r\n"
        "  uid                  the chip's unique ID\r\n"
        "  test-bootloop [net|hw] yes  crash 2 boots on purpose to prove safe mode\r\n"
        "  dac <off|3v3|5v|12v> [volts]  route DAC output + set a calibrated voltage\r\n"
        "  adc [ext|cal1|cal2|amp]       route ADC source + read calibrated mV (def ext)\r\n"
        "  measure              read the ADC input SMA in volts (= adc ext, ÷12)\r\n"
        "  path <name>          apply a named analog path (routing only)\r\n");
    out(ctx,
        "  dacraw <0-255> [div] raw DAC code, no routing/cal (debug)\r\n"
        "  adcraw               single raw ADC sample byte (debug)\r\n"
        "  adc-spi [n]          n live ADC reads DIRECT over SPI, no PSRAM (debug)\r\n"
        "  cap-selftest [n]     ramp through ADC->PSRAM path: PASS=path OK, FAIL=iCE40 CDC\r\n"
        "  psram-selftest       layered STM32/iCE40->PSRAM + /CE-net reach test (boot test)\r\n"
        "  dacmux/calsw/expdump low-level mux/relay control + dump (debug)\r\n"
        "  psram                diagnose PSRAM bus/CS/arbitration (0xFF ID)\r\n"
        "  psram-test           bring up + pattern-test the XSPI PSRAM\r\n"
        "  psram-addrtest       STM32-standalone address-tagged R/W across the full 8 MB\r\n"
        "  capture-psram [n]    v2: iCE40 captures n ADC samples to PSRAM, read back\r\n"
        "  dualcap [an] [ln]    v2: simultaneous an ADC + ln raw-LA samples (one trigger)\r\n"
        "  lastress [n]         v2: deep LA capture (n<=65535) @12 MS/s — DDR-drain stress test\r\n"
        "  flash-id             read the iCE40 config-flash JEDEC ID\r\n"
        "  flash-ice40          reflash the iCE40 from the embedded v2 bitstream\r\n"
        "  flash-esp32-sync     C3: strap download mode + SYNC (wiring test)\r\n"
        "  flash-esp32          C3: flash the embedded esp-hosted slave image\r\n"
        "  wifi-set \"<ssid>\" \"<pass>\"  save Wi-Fi credentials + (re)connect the C3\r\n"
        "  wifi-show            show stored SSID, Wi-Fi state, IP\r\n"
        "  wifi-clear           erase stored Wi-Fi credentials\r\n"
        "  eth <stop|start|restart>  bring the wired link down/up (PHY reset + DHCP re-acquire)\r\n"
        "  eth stats            wired link: negotiated mode, MAC mode, error + drop counters\r\n"
        "  dfu                  reboot into the USB DFU bootloader to reflash firmware\r\n"
        "  selftest             silicon health check (clocks/timer/sram/rng)\r\n"
        "  reboot               system reset\r\n");
}

static void cmd_ina(console_out_t out, void *ctx)
{
    int bus_mv, sh_uv, cur_ua;
    uint16_t id;
    if (ina238_read_id(I2C_ADDR_INA238_INTERNAL, &id) == 0 &&
        ina238_read(I2C_ADDR_INA238_INTERNAL, &bus_mv, &sh_uv, &cur_ua) == 0)
        op(out, ctx, "  int(0x40) id=0x%04x  bus=%d mV  shunt=%d uV  I=%d uA\r\n",
           id, bus_mv, sh_uv, cur_ua);
    else
        op(out, ctx, "  int(0x40) no response\r\n");
    if (ina238_read(I2C_ADDR_INA238_EXTERNAL, &bus_mv, &sh_uv, &cur_ua) == 0)
        op(out, ctx, "  ext(0x44) bus=%d mV  shunt=%d uV  I=%d uA\r\n", bus_mv, sh_uv, cur_ua);
    else
        op(out, ctx, "  ext(0x44) no response (needs external supply)\r\n");
}

static void cmd_pstat(console_out_t out, void *ctx)
{
    target_power_status_t s;
    for (int e = 1; e <= 2; e++)
        if (target_power_get_status(e, &s) == 0)
            op(out, ctx, "  eFuse%d: en=%d valid=%d fault=%d\r\n", e, s.enabled, s.valid, s.fault);
}

/* ---- silicon self-test --------------------------------------------------
 * 'selftest' — a quick health check of the STM32H563 silicon itself: the HSE
 * crystal, PLL1 lock, the 250 MHz system clock, the SysTick timebase, on-chip
 * SRAM, and the hardware RNG.  Each line is PASS/FAIL with a measured value, and
 * a summary at the end reports overall health.  Intended for bringing up a
 * suspect board; it deliberately does NOT touch the FPGA/PSRAM/flash (use 'ping'
 * / 'psram' / 'flash-id' for those).  Mirrors the RP2350 console selftest.
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
static const char *parse_quoted(const char *p, char *out, size_t n)
{
    while (*p == ' ' || *p == '\t') p++;
    size_t i = 0;
    if (*p == '"') {
        p++;
        while (*p && *p != '"') { if (i < n - 1) out[i++] = *p; p++; }
        if (*p == '"') p++;
    } else {
        while (*p && *p != ' ' && *p != '\t') { if (i < n - 1) out[i++] = *p; p++; }
    }
    out[i] = '\0';
    return p;
}

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
    parse_quoted(p, cfg.password, sizeof(cfg.password));   /* open AP: empty */

    if (cfg.ssid[0] == '\0') {
        op(out, ctx, "  usage: wifi-set \"<ssid>\" \"<password>\"\r\n");
        return;
    }
    if (config_save(&cfg) != 0) {
        op(out, ctx, "  wifi-set: config save failed\r\n");
        return;
    }
    op(out, ctx, "[cfg] credentials written to flash\r\n");
    esp_wifi_ctrl_reload();     /* re-read creds, (re)start the ESP32 + connect */
    op(out, ctx, "  saved SSID \"%s\" — connecting in the background; run wifi-show for status\r\n",
       cfg.ssid);
}

static void cmd_wifi_show(console_out_t out, void *ctx)
{
    config_t cfg;
    bool have = (config_load(&cfg) == 0 && cfg.ssid[0] != '\0');
    op(out, ctx, "  ssid: %s\r\n", have ? cfg.ssid : "");
    op(out, ctx, "  state: %s\r\n", esp_wifi_ctrl_state_str());
    op(out, ctx, "  ip: %s\r\n", wifi_sta_ip());
    int rssi;
    if (esp_wifi_ctrl_rssi(&rssi)) op(out, ctx, "  rssi: %d dBm\r\n", rssi);
    else                           op(out, ctx, "  rssi: \r\n");
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
        op(out, ctx, "  dma    : spi1=%s spi4=%s\r\n",
           signal_engine_dma_active() ? "on" : "off",
           esp_hosted_spi_dma_active() ? "on" : "off");
        /* PSRAM datapath health from the boot self-test (STM32<->PSRAM, iCE40 write,
           iCE40 /CE-net reach).  Re-run on demand with 'psram-selftest'. */
        op(out, ctx, "  psram  : %s\r\n", psram_selftest_str());
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
        /* Separate keys ("hint", "safe", "clock"), so a host parsing "fpga : gateware vN"
           keeps working. */
        if (!fpga_ok && boot_hw_ready() && !ice40_is_configured())
            op(out, ctx, "  hint   : the iCE40 never configured (blank config flash?): run flash-ice40\r\n");
        if (boot_guard_report()[0] != '\0')
            op(out, ctx, "  safe   : %s\r\n", boot_guard_report());
        if (clock_on_hsi())
            op(out, ctx, "  clock  : the 25 MHz crystal did not start; running from the internal HSI\r\n");
        op(out, ctx, "  adc    : %d-bit x%d, %d mV full-scale\r\n",
           ADC_BITS, ADC_CHANNELS, ADC_FULLSCALE_MV);
        op(out, ctx, "  dac    : ac=%s replay=%s dc=%s  gen %d-bit / replay %d-bit x%d, %d mV FS\r\n",
           DAC_AC ? "yes" : "no", DAC_REPLAY ? "yes" : "no", DAC_DC ? "yes" : "no",
           DAC_BITS, DAC_REPLAY_BITS, DAC_CHANNELS, DAC_FULLSCALE_MV);
        op(out, ctx, "  replay : deep=%s (needs v%u), max %lu samples (%s)\r\n",
           deep ? "yes" : "no", DAC_DEEP_REPLAY_MIN_GW,
           (unsigned long)(deep ? FPGA_DAC_REPLAY_MAX_SAMPLES : SIGNAL_MAX_SAMPLES),
           deep ? "PSRAM" : "BRAM");
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
        for (int i = 0; i < sys_health_task_count(); i++)
            op(out, ctx, "  stack  : %-7s %u B free (min)\r\n",
               sys_health_task_name(i), sys_health_task_stack_free(i));
        op(out, ctx, "  heap   : %u B free, %u B min\r\n",
           sys_health_heap_free(), sys_health_heap_min_free());
    } else if (!strcmp(argv[0], "spi-clk") && argc >= 2) {
        uint32_t div = (uint32_t)atoi(argv[1]);
        uint32_t hz = signal_engine_spi_set_prescaler(div);
        if (hz) op(out, ctx, "  SPI1 SCK = %lu Hz (/%lu)\r\n", (unsigned long)hz, (unsigned long)div);
        else    op(out, ctx, "  bad divisor (use 2,4,8,16,32,64,128,256)\r\n");
    } else if (!strcmp(argv[0], "spi-diag")) {
        int n = (argc >= 2) ? atoi(argv[1]) : 50;
        signal_engine_spi_diag(n);
        op(out, ctx, "  (spi-diag printed on the device log)\r\n");
    } else if (!strcmp(argv[0], "adc-spi")) {
        /* Read the live ADC sample DIRECTLY over SPI (CMD_ADC_PROBE), skipping
           PSRAM + the 24->48 CDC — isolates 0x5555 to engine/silicon vs path. */
        int n = (argc >= 2) ? atoi(argv[1]) : 16;
        signal_engine_adc_spi_diag(n);
        op(out, ctx, "  (adc-spi printed on the device log)\r\n");
    } else if (!strcmp(argv[0], "psram-selftest")) {
        /* Layered PSRAM datapath test: STM32<->PSRAM, iCE40->PSRAM write, and (if
           that fails) whether the iCE40 even reaches the /CE net.  Same test run
           once at boot. */
        psram_boot_selftest();
        op(out, ctx, "  (psram-selftest printed on the device log)\r\n");
    } else if (!strcmp(argv[0], "cap-selftest")) {
        /* End-to-end ADC->PSRAM path test: iCE40 streams a known ramp through the
           real capture datapath (CDC+writer+PSRAM+readback).  PASS => the path is
           healthy; FAIL => iCE40 capture/CDC timing error, NOT the analog ADC. */
        int n = (argc >= 2) ? atoi(argv[1]) : 64;
        int bad = -1;
        int rc = signal_engine_capture_selftest(n, &bad);
        op(out, ctx, "  cap-selftest: %s (see device log)\r\n",
           rc == 0 ? "PASS — ADC->PSRAM path OK" : "FAIL — iCE40 capture/CDC timing error");
    } else if (!strcmp(argv[0], "i2c-scan")) {
        i2c_bus_status();   /* prints to the local console log */
        op(out, ctx, "  (scan printed on the device log)\r\n");
    } else if (!strcmp(argv[0], "ina")) {
        cmd_ina(out, ctx);
    } else if (!strcmp(argv[0], "power") && argc >= 3) {
        int e = atoi(argv[1]);
        bool on = !strcmp(argv[2], "on") || !strcmp(argv[2], "1");
        if (target_power_enable(e, on) == 0) op(out, ctx, "  eFuse%d %s\r\n", e, on ? "ON" : "OFF");
        else                                 op(out, ctx, "  bad eFuse index\r\n");
    } else if (!strcmp(argv[0], "pstat")) {
        cmd_pstat(out, ctx);
    } else if (!strcmp(argv[0], "la-voltage")) {
        /* Select the LA I/O-bank voltage (TPS2116 mux) — must be set before any
           LA op (la / la_capture / dap_start / uart / i2c-sensor). */
        if (argc >= 2) {
            /* Same refusal as the JSON la_voltage: not while any LA pin has a function. */
            char why[LA_PINS_ERR_MAX];
            int rc = la_pins_check_voltage_change(la_vccio_get_mv(), atoi(argv[1]), why, sizeof(why))
                         ? la_vccio_set_mv(atoi(argv[1])) : -3;
            if (rc == -3) {
                out(ctx, "  ");   /* straight through out(): op()'s 160-byte buffer would cut it */
                out(ctx, why);
                out(ctx, "\r\n");
            } else if (rc == 0)
                op(out, ctx, "  LA VCCIO = %d mV (st=%d)\r\n",
                   la_vccio_get_mv(), la_vccio_status_pin());
            else if (rc == -2)
                op(out, ctx, "  1.8 V needs a v3 pod; this board is %s "
                             "(its TPS2116 has no 1.8 V setting)\r\n",
                   board_rev_str());
            else
                op(out, ctx, "  usage: la-voltage <1800|3300>\r\n");
        } else {
            int mv = la_vccio_get_mv();
            op(out, ctx, "  LA VCCIO = %s (st=%d)\r\n",
               mv ? (mv == LA_VCCIO_1V8 ? "1800 mV" : "3300 mV") : "UNSET",
               la_vccio_status_pin());
        }
    } else if (!strcmp(argv[0], "usb-cc")) {
        /* USB-C CC lines: cable orientation + the source's current advertisement. */
        usb_cc_t cc;
        if (usb_cc_read(&cc) != 0) {
            op(out, ctx, "  usb-cc: unavailable (board %s)\r\n", board_rev_str());
        } else {
            static const char *const orient[] = { "none", "CC1", "CC2" };
            op(out, ctx, "  CC1=%d mV  CC2=%d mV  orientation=%s  source=%s (%d mA)\r\n",
               cc.cc1_mv, cc.cc2_mv, orient[cc.orientation], cc.advertised,
               cc.advertised_ma);
        }
    } else if (!strcmp(argv[0], "uid")) {
        /* uid: the 96-bit unique ID.  Words only: a byte read of this area is a precise bus
           fault on the H5 (proven on the v3, bfar=0x08fff800; see board_uid.h). */
        uint32_t w[3];
        board_uid_words(w);
        op(out, ctx, "  uid (words): %08lx %08lx %08lx\r\n",
           (unsigned long)w[0], (unsigned long)w[1], (unsigned long)w[2]);
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
    } else if (!strcmp(argv[0], "nrst")) {
        /* Drive /NRST_CONTROL (J1 pin 22): nrst [assert|release|<pulse ms>]. */
        if (!nrst_ctrl_supported()) {
            op(out, ctx, "  nrst: no reset pin on this board (%s)\r\n", board_rev_str());
        } else {
            if (argc >= 2) {
                if      (!strcmp(argv[1], "assert"))  nrst_ctrl_assert(true);
                else if (!strcmp(argv[1], "release")) nrst_ctrl_assert(false);
                else                                  nrst_ctrl_pulse((uint32_t)atoi(argv[1]));
            }
            op(out, ctx, "  nRST %s\r\n",
               nrst_ctrl_is_asserted() ? "ASSERTED (low)" : "released (Hi-Z)");
        }
    } else if (!strcmp(argv[0], "dacraw") && argc >= 2) {
        /* dacraw <code 0..255> [div] — raw DAC code, no routing/cal (debug). */
        uint8_t v = (uint8_t)atoi(argv[1]);
        uint32_t div = (argc >= 3) ? (uint32_t)strtoul(argv[2], NULL, 0) : 240u;
        if (dac_set_constant(v, div) == 0) op(out, ctx, "  DAC=%u div=%lu\r\n", v, (unsigned long)div);
        else                               op(out, ctx, "  dac-set failed\r\n");
    } else if (!strcmp(argv[0], "dacmux") && argc >= 3) {
        /* dacmux <en> <sel> [en2 sel2] — LOW-LEVEL U55 mux (prefer `dac`/`path`).
           CTRL1(U47) sel: 0=3V3 1=5V 2=12V 3=12V_ADC; CTRL2(U48): 0=12V_VMID 1=ADC_VMID */
        int r = dacmux_set_ctrl1((bool)atoi(argv[1]), (uint8_t)(atoi(argv[2]) & 3));
        if (argc >= 5) r |= dacmux_set_ctrl2((bool)atoi(argv[3]), (uint8_t)(atoi(argv[4]) & 3));
        op(out, ctx, r == 0 ? "  dacmux set\r\n" : "  dacmux failed\r\n");
    } else if (!strcmp(argv[0], "expdump")) {
        analog_switch_dump();
    } else if (!strcmp(argv[0], "calsw") && argc >= 5) {
        /* calsw <cal1> <cal2> <amp_measure> <cal_path> — LOW-LEVEL U58 relays
           (prefer `path`/`adc`).  CAL1=5V->ADC, CAL2=diff->ADC, CAL_PATH=ADC<-cal node */
        int r = calsw_set((bool)atoi(argv[1]), (bool)atoi(argv[2]),
                          (bool)atoi(argv[3]), (bool)atoi(argv[4]));
        op(out, ctx, r == 0 ? "  calsw set\r\n" : "  calsw failed (cal1&cal2 exclusive?)\r\n");
    } else if (!strcmp(argv[0], "path") && argc >= 2) {
        /* path <name> — apply a named analog path (routing only; switches flip
           automatically). names: off dac_3v3|3v3 dac_5v|5v dac_12v|12v
           adc_ext|ext|sma cal1 cal2 amp */
        analog_path_t p;
        if (analog_path_from_name(argv[1], &p) != 0) {
            op(out, ctx, "  unknown path (off|3v3|5v|12v|ext|cal1|cal2|amp)\r\n");
        } else {
            int r = analog_path_set(p);
            uint8_t u55 = 0, u58 = 0; dacmux_read(&u55); calsw_read(&u58);
            op(out, ctx, "  path=%s%s  U55=0x%02x U58=0x%02x\r\n",
               analog_path_name(p), r ? " (I2C ERR)" : "", u55, u58);
        }
    } else if (!strcmp(argv[0], "dac") && argc >= 2) {
        /* dac <off|3v3|5v|12v> [volts] — route the DAC output path (switches flip
           automatically) and, if volts given, set a CALIBRATED voltage. */
        analog_path_t p;
        int idx = -1;
        if      (analog_path_from_name(argv[1], &p) != 0) p = ANALOG_PATH__COUNT;
        if      (p == ANALOG_PATH_DAC_3V3) idx = 0;
        else if (p == ANALOG_PATH_DAC_5V)  idx = 1;
        else if (p == ANALOG_PATH_DAC_12V) idx = 2;
        if (p != ANALOG_PATH_OFF && idx < 0) {
            op(out, ctx, "  usage: dac <off|3v3|5v|12v> [volts]\r\n");
        } else {
            analog_path_set(p);
            if (argc >= 3 && idx >= 0) {
                float v = (float)atof(argv[2]);
                float a = DAC_CAL[idx].a, b = DAC_CAL[idx].b;
                long code = lroundf((v - a) / b);
                if (code < 0) code = 0;
                if (code > 255) code = 255;
                dac_set_constant((uint8_t)code, 240);
                int got_mv = (int)lroundf((a + b * (float)code) * 1000.0f);
                op(out, ctx, "  dac %s = %d mV (code=%d)\r\n",
                   analog_path_name(p), got_mv, (int)code);
            } else {
                op(out, ctx, "  dac routed %s\r\n", analog_path_name(p));
            }
        }
    } else if (!strcmp(argv[0], "adc") || !strcmp(argv[0], "measure")) {
        /* adc [ext|cal1|cal2|amp] / measure — route the ADC source (switches flip
           automatically), read, and print CALIBRATED millivolts.  'ext' (default,
           and `measure`) applies the front-SMA ÷12 divider so the value is the
           true voltage at the ADC input SMA.  CAL2 unwraps the 16-bit count. */
        analog_path_t p = ANALOG_PATH_ADC_EXT;   /* default + `measure` */
        bool ok = true;
        if (!strcmp(argv[0], "adc") && argc >= 2) {
            if (analog_path_from_name(argv[1], &p) != 0 ||
                (p != ANALOG_PATH_ADC_EXT && p != ANALOG_PATH_CAL1 &&
                 p != ANALOG_PATH_CAL2 && p != ANALOG_PATH_AMP)) {
                op(out, ctx, "  usage: adc [ext|cal1|cal2|amp]\r\n"); ok = false;
            }
        }
        if (ok) {
            analog_path_set(p);
            HAL_Delay(20);           /* let the G6K relays (~4ms) + front-end RC settle */
            uint16_t s16[16] = {0};
            if (adc_capture_psram(s16, 16, 0.0f) == 0) {
                uint32_t sum = 0;
                for (int i = 0; i < 16; i++) sum += s16[i];
                float count = (float)sum / 16.0f, cnt = count;
                /* pick the per-source cal; bipolar sources (cal2, ext) wrap */
                cal_lin_t c = ADC_CAL_CAL1;
                bool wrap = false;
                if      (p == ANALOG_PATH_CAL2)    { c = ADC_CAL_CAL2; wrap = true; }
                else if (p == ANALOG_PATH_ADC_EXT) { c = ADC_CAL_EXT;  wrap = true; }
                if (wrap && cnt < 32768.0f) cnt += 65536.0f;
                float v = c.a + c.b * cnt;
                op(out, ctx, "  %s %s: count=%d -> %d mV\r\n",
                   argv[0], analog_path_name(p), (int)lroundf(count), (int)lroundf(v * 1000.0f));
            } else {
                op(out, ctx, "  adc read failed\r\n");
            }
        }
    } else if (!strcmp(argv[0], "adcraw")) {
        /* adcraw — single raw ADC probe byte (debug). */
        uint8_t val = 0;
        if (adc_probe_one(&val) == 0) op(out, ctx, "  ADC=%u (0x%02x)\r\n", val, val);
        else                          op(out, ctx, "  adc-probe failed\r\n");
    } else if (!strcmp(argv[0], "psram")) {
        /* Diagnostic path: reports bus/CS/arbitration state and re-probes the ID
           with the iCE40 held off the shared bus (see psram_diag). */
        if (psram_diag() == 0) op(out, ctx, "  PSRAM ID ok with iCE40 in reset (arbitration is the issue)\r\n");
        else                   op(out, ctx, "  PSRAM ID failed even with iCE40 in reset (see device log)\r\n");
    } else if (!strcmp(argv[0], "psram-bench")) {
        uint32_t kb = (argc >= 2) ? (uint32_t)atoi(argv[1]) : 256;
        psram_bench(kb);
        op(out, ctx, "  (psram-bench printed on the device log)\r\n");
    } else if (!strcmp(argv[0], "psram-clk") && argc >= 2) {
        uint32_t p = (uint32_t)atoi(argv[1]);
        uint32_t hz = psram_set_prescaler(p);
        if (hz) op(out, ctx, "  OCTOSPI SCLK = %lu Hz (presc %lu)\r\n", (unsigned long)hz, (unsigned long)p);
        else    op(out, ctx, "  psram-clk failed\r\n");
    } else if (!strcmp(argv[0], "psram-chunk") && argc >= 2) {
        uint32_t n = psram_set_chunk((uint32_t)atoi(argv[1]));
        op(out, ctx, "  PSRAM tCEM chunk = %lu bytes\r\n", (unsigned long)n);
    } else if (!strcmp(argv[0], "psram-test")) {
        if (psram_init() == 0 && psram_test() == 0) op(out, ctx, "  PSRAM ok\r\n");
        else                                        op(out, ctx, "  PSRAM failed (see device log)\r\n");
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
        static uint8_t b[256];
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
    } else if (!strcmp(argv[0], "dualcap")) {
        /* Unified simultaneous ADC + raw-LA capture (one trigger).  `dualcap [an]
           [ln]` captures `an` ADC + `ln` LA samples into their two PSRAM regions and
           prints the first few of each.  ADC slow (div 240 = 100 kS/s), LA fast
           (div 24 = 1 MS/s) to exercise the independent-rate datapath. */
        unsigned an = (argc >= 2) ? (unsigned)strtoul(argv[1], NULL, 0) : 64u;
        unsigned ln = (argc >= 3) ? (unsigned)strtoul(argv[2], NULL, 0) : 64u;
        if (an > 256) an = 256;
        if (ln > 256) ln = 256;
        static uint16_t adcb[256], lab[256];
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
        if (fpga_la_capture_psram_start((size_t)n, 0.0f /*max rate*/) != 0) {
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
    } else if (!strcmp(argv[0], "flash-id")) {
        uint8_t id[3] = {0};
        if (ice40_flash_read_id(id) == 0)
            op(out, ctx, "  iCE40 flash JEDEC ID: %02x %02x %02x (W25Q64 = ef 40 17)\r\n",
               id[0], id[1], id[2]);
        else
            op(out, ctx, "  flash-id failed (see device log)\r\n");
    } else if (!strcmp(argv[0], "flash-ice40")) {
        if (fpga_bitstream_len == 0) {
            op(out, ctx, "  no embedded bitstream (build the v2 iCE40 .bin first)\r\n");
        } else {
            /* Quiesce first: ice40_flash_program grabs the shared bus (psram_bus_acquire); a
               live DAC replay/reader mid-burst would wedge the PSRAM the same way the runtime
               image swap does. */
            signal_engine_quiesce_psram_masters();
            if (ice40_flash_program(fpga_bitstream, fpga_bitstream_len) == 0) {
                /* Re-read the gateware version so status/capabilities reflect the just-
                   flashed bitstream instead of the boot-time value. */
                bool ok = signal_engine_refresh_version();
                op(out, ctx, "  iCE40 reflashed + reconfigured (%u bytes), gateware v%u%s\r\n",
                   (unsigned)fpga_bitstream_len, signal_engine_fpga_version(),
                   ok ? "" : " (FPGA unreachable — check gateware)");
            } else {
                op(out, ctx, "  flash-ice40 failed (see device log)\r\n");
            }
        }
    } else if (!strcmp(argv[0], "flash-esp32-sync")) {
        esp_wifi_ctrl_pause(true);      /* Wi-Fi control must not drive EN/BOOT meanwhile */
        esp_hosted_spi_stop();
        uint32_t magic = 0;
        if (esp_rom_flash_sync(&magic) == 0)
            op(out, ctx, "  C3 ROM synced (chip magic 0x%08lx)\r\n", (unsigned long)magic);
        else
            op(out, ctx, "  flash-esp32-sync failed — no C3 ROM response (see log)\r\n");
        esp_wifi_ctrl_pause(false);
        esp_wifi_ctrl_reload();         /* the C3 was left in its ROM loader: restart Wi-Fi */
    } else if (!strcmp(argv[0], "flash-esp32")) {
        esp_wifi_ctrl_pause(true);      /* Wi-Fi control must not drive EN/BOOT meanwhile */
        esp_hosted_spi_stop();
        if (esp_slave_fw_len == 0) {
            op(out, ctx, "  no embedded C3 image (build esp32-hosted-slave first)\r\n");
        } else if (esp_rom_flash_program(esp_slave_fw, esp_slave_fw_len, 0) == 0) {
            op(out, ctx, "  ESP32-C3 flashed + booted (%u bytes)\r\n",
               (unsigned)esp_slave_fw_len);
        } else {
            op(out, ctx, "  flash-esp32 failed (C3 left in reset; see log)\r\n");
        }
        esp_wifi_ctrl_pause(false);
        esp_wifi_ctrl_reload();         /* restart the Wi-Fi join (fresh image, or retry) */
    } else if (!strcmp(argv[0], "wifi-set")) {
        op(out, ctx, "  usage: wifi-set \"<ssid>\" \"<password>\"\r\n");
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
        else op(out, ctx, "  usage: eth <stop|start|restart|stats>\r\n");
    } else if (!strcmp(argv[0], "wifi-clear")) {
        config_clear();
        esp_wifi_ctrl_reload();     /* drop Wi-Fi; ESP32 returns to reset */
        op(out, ctx, "  Wi-Fi credentials cleared (reboot to fully apply)\r\n");
    } else if (!strcmp(argv[0], "can")) {
        /* can config <bitrate> <normal|internal|external|listen> [term] |
           can write <id> [b0 b1 ...] | can read | can status |
           can term <0|1> | can off — classic CAN over FDCAN1 (TCAN1044).
           Use internal/external loopback to self-test on a single pod. */
        if (argc >= 3 && !strcmp(argv[1], "config")) {
            uint32_t br = (uint32_t)strtoul(argv[2], NULL, 0);
            can_mode_t m = CAN_MODE_NORMAL;
            if (argc >= 4 && can_mode_from_name(argv[3], &m) != 0) {
                op(out, ctx, "  usage: can config <bitrate> <normal|internal|external|listen> [term]\r\n");
            } else {
                bool term = (argc >= 5) && (!strcmp(argv[4], "1") || !strcmp(argv[4], "term") || !strcmp(argv[4], "on"));
                int rc = can_configure(br, m, false, term);
                op(out, ctx, rc == 0 ? "  CAN up: %lu bit/s %s term=%d\r\n"
                                     : "  CAN config failed (rc=%d)\r\n",
                   rc == 0 ? (unsigned long)br : (unsigned long)rc,
                   rc == 0 ? can_mode_name(m) : "", term);
            }
        } else if (argc >= 3 && !strcmp(argv[1], "write")) {
            can_frame_t f = {0};
            f.id = (uint32_t)strtoul(argv[2], NULL, 0);
            int n = 0;
            for (int i = 3; i < argc && n < 8; i++) f.data[n++] = (uint8_t)strtoul(argv[i], NULL, 0);
            f.dlc = (uint8_t)n;
            int rc = can_tx(&f);
            op(out, ctx, rc == 0 ? "  TX id=0x%lx dlc=%d\r\n" : "  TX failed (rc=%d)\r\n",
               rc == 0 ? (unsigned long)f.id : (unsigned long)rc, f.dlc);
        } else if (argc >= 2 && !strcmp(argv[1], "read")) {
            can_frame_t fr[8];
            int n = can_rx_pop(fr, 8);
            if (n == 0) op(out, ctx, "  (no frames)\r\n");
            for (int i = 0; i < n; i++) {
                op(out, ctx, "  RX id=0x%lx%s dlc=%d:", (unsigned long)fr[i].id,
                   fr[i].ext ? "(ext)" : "", fr[i].dlc);
                for (int b = 0; b < fr[i].dlc; b++) op(out, ctx, " %02x", fr[i].data[b]);
                op(out, ctx, "\r\n");
            }
        } else if (argc >= 2 && !strcmp(argv[1], "status")) {
            can_status_t s; can_get_status(&s);
            op(out, ctx, "  enabled=%d mode=%s bitrate=%lu term=%d tec=%u rec=%u busoff=%d rx=%lu ovf=%lu resp=%lu hits=%lu\r\n",
               s.enabled, can_mode_name(s.mode), (unsigned long)s.bitrate, s.term,
               s.tec, s.rec, s.bus_off, (unsigned long)s.rx_pending, (unsigned long)s.rx_overflow,
               (unsigned long)s.responder_rules, (unsigned long)s.responder_hits);
        } else if (argc >= 3 && !strcmp(argv[1], "respond")) {
            /* can respond <match_id> <reply_id> [bytes...] | can respond clear */
            if (!strcmp(argv[2], "clear")) {
                can_responder_clear();
                op(out, ctx, "  responder rules cleared\r\n");
            } else if (argc >= 4) {
                can_frame_t reply = {0};
                reply.id = (uint32_t)strtoul(argv[3], NULL, 0);
                int n = 0;
                for (int i = 4; i < argc && n < 8; i++) reply.data[n++] = (uint8_t)strtoul(argv[i], NULL, 0);
                reply.dlc = (uint8_t)n;
                int idx = can_responder_add((uint32_t)strtoul(argv[2], NULL, 0), false, &reply);
                op(out, ctx, idx >= 0 ? "  rule %d: id 0x%lx -> reply 0x%lx dlc=%d\r\n"
                                      : "  responder table full\r\n",
                   idx, (unsigned long)strtoul(argv[2], NULL, 0), (unsigned long)reply.id, reply.dlc);
            } else {
                op(out, ctx, "  usage: can respond <match_id> <reply_id> [bytes...] | can respond clear\r\n");
            }
        } else if (argc >= 3 && !strcmp(argv[1], "term")) {
            bool on = !strcmp(argv[2], "1") || !strcmp(argv[2], "on");
            can_set_term(on);
            op(out, ctx, "  CAN termination %s\r\n", on ? "ON" : "OFF");
        } else if (argc >= 2 && !strcmp(argv[1], "off")) {
            can_disable();
            op(out, ctx, "  CAN disabled\r\n");
        } else {
            op(out, ctx, "  usage: can config <bitrate> <normal|internal|external|listen> [term] | write <id> [bytes] | read | status | respond <match_id> <reply_id> [bytes] | respond clear | term <0|1> | off\r\n");
        }
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
    } else {
        op(out, ctx, "  unknown command '%s' (try 'help')\r\n", argv[0]);
    }
}

void console_exec(char *cmd, console_out_t out, void *ctx)
{
    hw_lock();
    console_exec_locked(cmd, out, ctx);
    hw_unlock();
}

/* ---- local interactive console ----------------------------------------- */

static char line[LINE_MAX];
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
    char buf[LINE_MAX];
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
        } else if (c >= 0x20 && c < 0x7F && line_len < LINE_MAX - 1) {
            last_was_cr = 0;
            uint8_t ch = (uint8_t)c;
            line[line_len++] = (char)ch;
            console_io_write(&ch, 1);
        }
    }
}
