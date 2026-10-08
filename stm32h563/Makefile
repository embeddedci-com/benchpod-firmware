# bench-pod STM32H563ZIT6 firmware — Makefile
#
# `make` builds, `make test` runs the host tests, `make flash-dfu` / `make flash`
# program a pod, `make clean`, `make help`.
# Uses the STM32CubeH5 HAL (and its mbed-crypto) referenced in place from $(CUBE),
# plus FreeRTOS, LwIP, the ST USB device stack, scpi-parser and Monocypher
# as submodules under lib/.

TARGET            := bench_pod_stm32
BUILD_DIR         := build

# Split DFU images (built by `all`; recipes further below). Defined up here so the
# `all:` prerequisite list — read before those recipes — sees them.

# STM32CubeH5 SDK location (override on the command line if installed elsewhere).
CUBE              ?= /opt/STM32CubeH5

# Programmer: OpenOCD interface cfg (stlink, cmsis-dap, jlink, ...).
OPENOCD_INTERFACE ?= stlink

# USB DFU flashing (no SWD probe): host tool + the STM32 main-flash base. Used by
# the `flash-dfu` target, which writes over the device's own USB-FS port once it
# is in the ROM bootloader (BOOT0 high at reset, or `benchpod dfu` on a running pod).
DFU_UTIL          ?= dfu-util
DFU_FLASH_ADDR    ?= 0x08000000

#######################################
# Toolchain
#######################################
PREFIX  := arm-none-eabi-
CC      := $(PREFIX)gcc
AS      := $(PREFIX)gcc -x assembler-with-cpp
CP      := $(PREFIX)objcopy
SZ      := $(PREFIX)size
NM      := $(PREFIX)nm
HEX     := $(CP) -O ihex
BIN     := $(CP) -O binary -S

#######################################
# SDK / library paths
#######################################
CMSIS_CORE  := $(CUBE)/Drivers/CMSIS/Include
CMSIS_DEV   := $(CUBE)/Drivers/CMSIS/Device/ST/STM32H5xx
HAL         := $(CUBE)/Drivers/STM32H5xx_HAL_Driver
FREERTOS    := lib/FreeRTOS-Kernel
FREERTOS_PORT := $(FREERTOS)/portable/GCC/ARM_CM33_NTZ/non_secure
USBD        := lib/stm32_mw_usb_device
LWIP        := lib/stm32_mw_lwip
LAN8742     := $(CUBE)/Drivers/BSP/Components/lan8742
MBEDTLS     := $(CUBE)/Middlewares/Third_Party/mbed-crypto

#######################################
# Sources
#######################################
C_SOURCES := \
  src/main.c \
  src/system_stm32h5xx.c \
  src/stm32h5xx_it.c \
  src/stm32h5xx_hal_timebase_tim.c \
  src/signal_engine.c \
  src/la_rate.c \
  src/la_pins.c \
  src/power_profile.c \
  src/dac_loop_params.c \
  src/dac_rearm.c \
  src/adc_scale.c \
  src/mcu_adc.c \
  src/board_rev.c \
  src/board_variant.c \
  src/usb_cc.c \
  src/nrst_ctrl.c \
  src/psram_alloc.c \
  src/bus_owner.c \
  src/psram_regions.c \
  src/xfer.c \
  src/dma_wait.c \
  src/dap.c \
  src/swd_ll.c \
  src/i2c_bus.c \
  src/ina238.c \
  src/tca9554.c \
  src/target_power.c \
  src/can_bus.c \
  src/console_io.c \
  src/bp_log.c \
  src/console.c \
  src/usbd_conf.c \
  src/usbd_desc.c \
  src/usbd_cdc_if.c \
  src/usb_device.c \
  src/ethernetif.c \
  src/net_server.c \
  src/net_dhcp.c \
  src/net_route.c \
  src/eth_diag.c \
  src/esp_hosted_frame.c \
  src/esp_hosted_spi.c \
  src/esp_hosted_pump.c \
  src/esp_netif.c \
  src/esp_wifi_ctrl.c \
  src/cloud_config.c \
  src/cloud_client.c \
  src/cloud_rx.c \
  src/cloud_caps.c \
  src/cloud_reply_cap.c \
  src/ws_frame.c \
  src/mbedtls_port.c \
  src/command_handler.c \
  src/command_handler_can.c \
  src/command_handler_ota.c \
  src/command_handler_pins.c \
  src/command_handler_power.c \
  src/scpi_server.c \
  src/sensor_sim.c \
  src/sensor_bmp280.c \
  src/b64url.c \
  src/bp_json.c \
  src/conn_tx.c \
  src/hw_worker.c \
  src/ota.c \
  src/ota_commit.c \
  src/fault.c \
  src/watchdog.c \
  src/boot_guard.c \
  src/boot_policy.c \
  src/sys_health.c \
  src/device_identity.c \
  src/config_store.c \
  src/dac_limits.c \
  src/dac_limits_hw.c \
  src/cal_data.c \
  src/adc_cal.c \
  src/adc_cal_hw.c \
  src/current_out.c \
  src/flash_compat.c \
  src/flash_layout.c \
  src/fw_info.c \
  src/fw_info_block.c \
  src/fw_sign.c \
  src/cmd_tier.c \
  src/cmd_gate.c \
  src/pod_policy.c \
  src/lease_gate.c \
  src/cloud_extras.c \
  src/altcp_bp_proxy.c \
  src/command_handler_policy.c \
  src/rng.c \
  src/hw_lock.c \
  src/psram.c \
  src/ice40_flash.c \
  src/w25q.c \
  src/blob_store.c \
  src/blob_hooks.c \
  src/blob_manifest.c \
  src/upload_rx.c \
  src/spi_flash.c \
  src/spi_stream.c \
  src/command_handler_spi.c \
  src/esp_rom_flash.c \
  src/dfu_boot.c \
  $(LAN8742)/lan8742.c \
  port/pico_compat.c

# libscpi (SCPI parser) + monocypher (Ed25519) — vendored submodules.
SCPI := lib/scpi-parser/libscpi
C_SOURCES += $(addprefix $(SCPI)/src/,error.c expression.c fifo.c ieee488.c \
             lexer.c minimal.c parser.c units.c utils.c)
C_SOURCES += lib/monocypher/src/monocypher.c \
             lib/monocypher/src/optional/monocypher-ed25519.c

# HAL driver sources (curated; unused funcs are GC'd anyway).
HAL_C := stm32h5xx_hal.c stm32h5xx_hal_cortex.c stm32h5xx_hal_rcc.c \
         stm32h5xx_hal_rcc_ex.c stm32h5xx_hal_gpio.c stm32h5xx_hal_pwr.c \
         stm32h5xx_hal_pwr_ex.c stm32h5xx_hal_flash.c stm32h5xx_hal_flash_ex.c \
         stm32h5xx_hal_dma.c stm32h5xx_hal_dma_ex.c stm32h5xx_hal_uart.c \
         stm32h5xx_hal_uart_ex.c stm32h5xx_hal_tim.c stm32h5xx_hal_tim_ex.c \
         stm32h5xx_hal_exti.c stm32h5xx_hal_spi.c stm32h5xx_hal_spi_ex.c \
         stm32h5xx_hal_i2c.c stm32h5xx_hal_i2c_ex.c stm32h5xx_hal_pcd.c \
         stm32h5xx_hal_pcd_ex.c stm32h5xx_ll_usb.c stm32h5xx_hal_eth.c \
         stm32h5xx_hal_rng.c stm32h5xx_hal_xspi.c stm32h5xx_hal_fdcan.c \
         stm32h5xx_hal_iwdg.c stm32h5xx_hal_adc.c stm32h5xx_hal_adc_ex.c
C_SOURCES += $(addprefix $(HAL)/Src/,$(HAL_C))

# ST USB Device middleware (Core + CDC class).
C_SOURCES += \
  $(USBD)/Core/Src/usbd_core.c \
  $(USBD)/Core/Src/usbd_ctlreq.c \
  $(USBD)/Core/Src/usbd_ioreq.c \
  $(USBD)/Class/CDC/Src/usbd_cdc.c

# LwIP (NO_SYS=1): core + IPv4 + ethernet netif + DNS + altcp(+TLS) + mDNS.
# igmp is needed for the mDNS responder (joins the 224.0.0.251 multicast group).
LWIP_CORE := init def inet_chksum ip mem memp netif pbuf raw tcp tcp_in \
             tcp_out timeouts udp sys stats dns altcp altcp_alloc altcp_tcp
LWIP_V4   := ip4 ip4_addr ip4_frag etharp icmp dhcp acd igmp
C_SOURCES += $(addprefix $(LWIP)/src/core/,$(addsuffix .c,$(LWIP_CORE)))
C_SOURCES += $(addprefix $(LWIP)/src/core/ipv4/,$(addsuffix .c,$(LWIP_V4)))
C_SOURCES += $(LWIP)/src/netif/ethernet.c
# altcp_tls_mbedtls: our patched copy (src/altcp_tls_bp/, see the top of altcp_tls_mbedtls_bp.c).
C_SOURCES += src/altcp_tls_bp/altcp_tls_mbedtls_bp.c \
             src/altcp_tls_bp/altcp_tls_mbedtls_mem.c
C_SOURCES += $(LWIP)/src/apps/mdns/mdns.c \
             $(LWIP)/src/apps/mdns/mdns_out.c \
             $(LWIP)/src/apps/mdns/mdns_domain.c

# FreeRTOS kernel + Cortex-M33 (no TrustZone) port + heap_4.
C_SOURCES += \
  $(FREERTOS)/tasks.c $(FREERTOS)/queue.c $(FREERTOS)/list.c \
  $(FREERTOS)/timers.c $(FREERTOS)/event_groups.c $(FREERTOS)/stream_buffer.c \
  $(FREERTOS_PORT)/port.c $(FREERTOS_PORT)/portasm.c \
  $(FREERTOS)/portable/MemMang/heap_4.c

ASM_SOURCES := config/startup_stm32h563zitx.s
# The ESP32-C3 esp-hosted slave image is not embedded: the pod keeps it in a W25Q blob slot
# (src/blob_store.h) and flashes the C3 from there; an installer sends it from build/blobs/.
ESP_SLAVE_BIN := ../esp32-hosted-slave/prebuilt/esp32c3-hosted-slave-merged.bin

#######################################
# Flags
#######################################
MCU := -mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard

DEFS := -DUSE_HAL_DRIVER -DSTM32H563xx -DHSE_VALUE=25000000 \
        -DUSE_FULL_ERROR_LIST=1 -DBLAKE2_NO_UNROLLING=1 \
        -DMBEDTLS_CONFIG_FILE='"mbedtls_bench_config.h"'

# RELEASE=1: trust only keys/release-N.pub (see "Firmware signing" below) and refuse a plain
# (non-TLS) cloud connection (cloud-hardening.md section 1). This must come before CFLAGS below
# (`:=` expands DEFS there): appended further down, -DBENCHPOD_RELEASE=1 never reached the compiler.
RELEASE ?= 0
ifeq ($(RELEASE),1)
DEFS += -DBENCHPOD_RELEASE=1
endif

INCLUDES := \
  -Iconfig -Ibsp -Isrc -Iport -I$(BUILD_DIR) \
  -Ilib/scpi-parser/libscpi/inc \
  -Ilib/monocypher/src -Ilib/monocypher/src/optional \
  -I$(CMSIS_CORE) \
  -I$(CMSIS_DEV)/Include \
  -I$(HAL)/Inc \
  -I$(HAL)/Inc/Legacy \
  -I$(FREERTOS)/include \
  -I$(FREERTOS_PORT) \
  -I$(USBD)/Core/Inc \
  -I$(USBD)/Class/CDC/Inc \
  -I$(LWIP)/src/include \
  -I$(LWIP)/system \
  -I$(LAN8742) \
  -I$(MBEDTLS)/include \
  -I$(MBEDTLS)/library \
  -Ilib/esp_hosted/common

OPT := -Og -g3

# -MMD -MP emits a per-object .d file of header deps (included below) so editing a
# header forces a recompile — without it, editing e.g. la_rate.h left a stale la_rate.o.
CFLAGS := $(MCU) $(DEFS) $(INCLUDES) $(OPT) -std=gnu11 \
          -Wall -Wextra -Wno-unused-parameter \
          -ffunction-sections -fdata-sections -fno-common -fstack-usage -MMD -MP

ASFLAGS := $(MCU) $(OPT) -Wall -ffunction-sections -fdata-sections

LDSCRIPT := config/STM32H563ZITX_FLASH.ld
# --no-warn-rwx-segments: .data is RWX on purpose. It carries .RamFunc (ota_commit.c's
# RAM-resident flash copy code), so the RAM LOAD segment must be executable.
LDFLAGS := $(MCU) -specs=nano.specs -specs=nosys.specs -T$(LDSCRIPT) \
           -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
           -Wl,--gc-sections -Wl,--print-memory-usage -Wl,--no-warn-rwx-segments -lc -lm

#######################################
# Build rules
#######################################
OBJECTS := $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# mbedTLS: compile the whole library/ tree into a dedicated object subdir.
# A subdir (not the flat vpath path the rest of the build uses) is required so
# mbedTLS's library/error.c doesn't collide with libscpi's error.c at link time;
# unused source files compile to (near-)empty objects under our config and are
# dropped by --gc-sections.
MBEDTLS_SRCS := $(wildcard $(MBEDTLS)/library/*.c)
MBEDTLS_OBJS := $(addprefix $(BUILD_DIR)/mbedtls/,$(notdir $(MBEDTLS_SRCS:.c=.o)))
OBJECTS += $(MBEDTLS_OBJS)

$(BUILD_DIR)/mbedtls:
	mkdir -p $@

$(BUILD_DIR)/mbedtls/%.o: $(MBEDTLS)/library/%.c | $(BUILD_DIR)/mbedtls
	$(CC) -c $(CFLAGS) $< -o $@

.DEFAULT_GOAL := all

# The iCE40 gateware images are not embedded: the pod keeps them in W25Q blob slots
# (src/blob_store.h), and an installer sends them from build/blobs/ (below).
ICE40_LOOP_BIN := ../ice40/bench_pod_fpga_vbench_pod_loop.bin
ICE40_DEEP_BIN := ../ice40/bench_pod_fpga_vbench_pod_deep.bin

# The blobs that go with this firmware, as the pod's W25Q slots hold them (src/blob_store.h):
# build/blobs/blob-{gw0,gw1,esp}.bin + blobs-manifest.json for installers, and blob_manifest_gen.h so the
# firmware knows which blobs it was built with.
BLOB_DIR      := $(BUILD_DIR)/blobs
GEN_BLOB_MANIFEST := $(BUILD_DIR)/blob_manifest_gen.h
$(GEN_BLOB_MANIFEST): tools/blob_manifest.py $(wildcard $(ICE40_LOOP_BIN)) $(wildcard $(ICE40_DEEP_BIN)) $(wildcard $(ICE40_LOOP_BIN).gwversion) $(wildcard $(ESP_SLAVE_BIN)) | $(BUILD_DIR)
	@v=$$(cat $(ICE40_LOOP_BIN).gwversion 2>/dev/null); \
	 python3 tools/blob_manifest.py --gw0 $(ICE40_LOOP_BIN) --gw1 $(ICE40_DEEP_BIN) --esp $(ESP_SLAVE_BIN) \
	   --gw-version $${v:-0} --out-dir $(BLOB_DIR) --header $@
	@echo "blobs: $$(ls $(BLOB_DIR) | tr '\n' ' ')"
$(BUILD_DIR)/blob_manifest.o: $(GEN_BLOB_MANIFEST)


# Firmware signing (docs/design/firmware-signing.md). The build trusts the release keys in
# keys/release-N.pub and, unless RELEASE=1, the developer key from `make dev-key` too. A dev build
# signs its own image and blobs with that key (build/*.sig), so OTA, install-blobs and hwe2e can
# send a signed update to a pod running a dev build. CI signs releases itself (release-stm32.yml).
DEV_SIGN_KEY  ?= $(HOME)/.config/benchpod/fw-signing-dev.key
FWSIGN        := python3 ../tools/fwsign.py
FW_VERSION    := $(shell sed -n 's/^\#define FIRMWARE_VERSION "\(.*\)"/\1/p' src/version.h)
SIGN_PUBS     := $(sort $(wildcard keys/release-*.pub))
ifneq ($(RELEASE),1)
DEV_SIGN_PUB  := $(if $(wildcard $(DEV_SIGN_KEY)),$(BUILD_DIR)/dev-signing.pub)
endif
GEN_SIGN_KEYS := $(BUILD_DIR)/fw_sign_keys_gen.h
# Rewritten only when its content changes (keys-header compares), so the FORCE does not rebuild
# anything unless RELEASE or the dev key changed.
$(GEN_SIGN_KEYS): FORCE $(SIGN_PUBS) $(DEV_SIGN_PUB) | $(BUILD_DIR)
	@$(FWSIGN) keys-header $(foreach k,$(SIGN_PUBS) $(DEV_SIGN_PUB),--pub $(k)) -o $@
$(BUILD_DIR)/dev-signing.pub: $(DEV_SIGN_KEY) | $(BUILD_DIR)
	@$(FWSIGN) pubkey $< | cut -d' ' -f1 > $@
$(BUILD_DIR)/fw_sign.o: $(GEN_SIGN_KEYS)
.PHONY: FORCE
FORCE:

# A dev build signs what it built: the image and the three blobs, as `<file>.sig`.
SIGNED_OUT := $(if $(DEV_SIGN_PUB),$(BUILD_DIR)/$(TARGET).bin.sig)
$(BUILD_DIR)/$(TARGET).bin.sig: $(BUILD_DIR)/$(TARGET).bin $(GEN_BLOB_MANIFEST) $(DEV_SIGN_KEY)
	@$(FWSIGN) sign --key $(DEV_SIGN_KEY) --target firmware --release $(FW_VERSION) $<
	@$(FWSIGN) sign-blobs --key $(DEV_SIGN_KEY) --release $(FW_VERSION) $(BLOB_DIR)/blobs-manifest.json

# One developer signing key per machine (the private seed never leaves ~/.config/benchpod/).
.PHONY: dev-key
dev-key:
	@if [ -f $(DEV_SIGN_KEY) ]; then $(FWSIGN) pubkey $(DEV_SIGN_KEY); \
	 else $(FWSIGN) keygen $(DEV_SIGN_KEY); fi

# The embedded C3 slave image is a ~1.07 MB vendored binary, intentionally NOT in
# git (caught by *.bin). If it's absent, fail the build loudly with the command to
# produce it — rather than silently shipping a pod that can't flash the C3.
$(ESP_SLAVE_BIN):
	@printf '\nERROR: missing binary build dependency:\n         %s\n\n' '$(ESP_SLAVE_BIN)' >&2
	@printf '  The ESP32-C3 esp-hosted slave image is not committed. Produce it with:\n\n' >&2
	@printf '         make fetch\n\n' >&2
	@printf '  (reproducible ESP-IDF Docker build; or set ESP_SLAVE_URL=<asset-url> to\n' >&2
	@printf '   download a prebuilt image instead). See ../esp32-hosted-slave/README.md\n\n' >&2
	@exit 1

# Fetch/produce the out-of-tree binary build dependencies (currently just the
# ESP32-C3 esp-hosted slave image). Default: ESP-IDF Docker build, reproducible
# byte for byte: the image is pinned by digest (esp32-hosted-slave/idf-image),
# the component versions by esp32-hosted-slave/dependencies.lock, and the build
# uses CONFIG_APP_REPRODUCIBLE_BUILD. Override with ESP_SLAVE_URL to download a
# prebuilt asset instead of running Docker.
#
# Either way the result must match the committed
# esp32-hosted-slave/prebuilt/esp32c3-hosted-slave-merged.bin.sha256, or this
# fails. (It used to be advisory on a Docker build: with the floating
# espressif/idf:release-v5.5 tag a rebuild could differ in the self-referential
# app_elf_sha256 field and the image trailer. With the digest pinned, a rebuild
# reproduces the committed hash.) Changing the recipe, the lock or the image
# means committing the new hash: make fetch prints it on a mismatch.
ESP_SLAVE_DIR := ../esp32-hosted-slave
ESP_SLAVE_URL ?=
ESP_IDF_IMAGE := $(shell cat $(ESP_SLAVE_DIR)/idf-image)
fetch:
	@if [ -n "$(ESP_SLAVE_URL)" ]; then \
	   echo "==> Downloading esp-hosted slave image: $(ESP_SLAVE_URL)"; \
	   mkdir -p $(ESP_SLAVE_DIR)/prebuilt; \
	   curl -fL "$(ESP_SLAVE_URL)" -o $(ESP_SLAVE_BIN); \
	 else \
	   echo "==> Building ESP32-C3 esp-hosted slave image (ESP-IDF Docker $(ESP_IDF_IMAGE))..."; \
	   ( cd $(ESP_SLAVE_DIR) && docker run --rm \
	       -v "$$PWD:/board" -v "$$PWD/work:/work" -v "$$PWD/prebuilt:/out" \
	       "$(ESP_IDF_IMAGE)" bash /board/build.sh ) || exit 1; \
	 fi
	@echo "==> Verifying sha256 against the committed hash..."
	@cd $(ESP_SLAVE_DIR)/prebuilt && if ! shasum -a 256 -c esp32c3-hosted-slave-merged.bin.sha256; then \
	   echo "fetch: the ESP32-C3 slave image does not match esp32c3-hosted-slave-merged.bin.sha256." >&2; \
	   echo "       Got: $$(shasum -a 256 esp32c3-hosted-slave-merged.bin)" >&2; \
	   echo "       If the recipe, the lock or idf-image changed on purpose, commit the new hash." >&2; \
	   exit 1; \
	 fi
	@echo "==> OK: $(ESP_SLAVE_BIN)"

.PHONY: all flash flash-dfu flash-dfu-code clean help test fetch
all: check-uid $(GEN_BLOB_MANIFEST) $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex \
     $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).syms $(SIGNED_OUT)

# The unique-ID area must be read as whole words (a byte read is a bus fault on the H5 and
# once left a pod crashing before USB came up).  Only board_uid.h may touch UID_BASE.
.PHONY: check-uid
check-uid:
	@if grep -rn 'UID_BASE' src port bsp | grep -v '^src/board_uid.h:' ; then \
	  echo 'check-uid: read the unique ID through board_uid_words() (src/board_uid.h), never UID_BASE directly' >&2; \
	  exit 1; \
	fi

# Host unit tests for the platform-independent logic (no STM32 HAL, native cc).
test:
	$(MAKE) -C test

# The LwIP altcp_tls_mbedtls.c (our copy, src/altcp_tls_bp/altcp_tls_mbedtls_bp.c) is mbedTLS 2.x-era: it reads the now-private
# ssl_context.out_left (needs MBEDTLS_ALLOW_PRIVATE_ACCESS) and the renamed-away
# MBEDTLS_SSL_MAX_CONTENT_LEN (now split into IN/OUT) — alias it back for this file only.
$(BUILD_DIR)/altcp_tls_mbedtls_bp.o: CFLAGS += -DMBEDTLS_ALLOW_PRIVATE_ACCESS \
                                               -DMBEDTLS_SSL_MAX_CONTENT_LEN=16384

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

# The compile flags and defines are part of every object: RELEASE=1 adds -DBENCHPOD_RELEASE=1, a
# different CUBE moves the include paths. make only compares file times, so without this a
# `make RELEASE=1` after a dev build relinked the dev objects into a "release" image. The stamp is
# rewritten only when its content changes, so an unchanged build rebuilds nothing.
DEFS_STAMP := $(BUILD_DIR)/flags.stamp
DEFS_STAMP_TEXT := $(CC) $(CFLAGS) | $(AS) $(ASFLAGS) | $(LDFLAGS)
$(DEFS_STAMP): FORCE | $(BUILD_DIR)
	@printf '%s\n' '$(subst ','\'',$(DEFS_STAMP_TEXT))' > $@.tmp; \
	 if cmp -s $@.tmp $@; then rm -f $@.tmp; \
	 else if [ -f $@ ]; then echo "compile flags changed: rebuilding every object"; fi; mv $@.tmp $@; fi
$(OBJECTS): $(DEFS_STAMP)

# Header-dependency files emitted by -MMD (one .d per .o); the leading '-' ignores
# them on a first/clean build when they don't exist yet.
-include $(OBJECTS:.o=.d)

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(BIN) $< $@

# Function symbol map ("address size type name", sorted), published with each release so the
# server can name the function a crash report's pc and lr point into.
$(BUILD_DIR)/%.syms: $(BUILD_DIR)/%.elf
	$(NM) -n -S --defined-only $< | awk '$$3 ~ /^[tTwW]$$/' > $@

$(BUILD_DIR):
	mkdir -p $@

# Flash via OpenOCD/ST-Link SWD (PA13/PA14).
flash: all
	openocd -f interface/$(OPENOCD_INTERFACE).cfg -f target/stm32h5x.cfg \
		-c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

# Flash over USB DFU — no SWD probe. The device must already be in the ROM
# bootloader: hold BOOT0 high at reset on a blank board, or run `benchpod dfu`
# (or the `dfu` console command) on a pod already running this firmware. The
# trailing `:leave` makes the bootloader start the new firmware when done.
#
# dfu-util exits non-zero (EX_IOERR = 74) on the post-`:leave` get_status: the
# STM32 resets into the app and detaches before dfu-util can read the final
# status. That is benign — the write already succeeded — so we judge success on
# the "File downloaded successfully" marker rather than dfu-util's exit code, and
# only fail on a real download error.
#
# $(call dfu_write,<bin>,<flash-address>) writes one image and leaves into the app.
define dfu_write
	@echo "$(DFU_UTIL) -a 0 --dfuse-address $(2):leave -D $(1)"
	@$(DFU_UTIL) -a 0 --dfuse-address $(2):leave -D $(1) 2>&1 \
		| tee $(BUILD_DIR)/dfu-flash.log; \
	if grep -q "File downloaded successfully" $(BUILD_DIR)/dfu-flash.log; then \
		echo "flash-dfu: OK ($(notdir $(1)) -> $(2); benign post-':leave' get_status / Error 74 ignored)"; \
	else \
		echo "flash-dfu: FAILED — see dfu-util output above"; exit 1; \
	fi
endef

# The firmware image (~0.5 MB). The gateware and ESP32-C3 images are not in it: they live in
# the W25Q blob slots and go in with `benchpod flash-self` (or any OTA) after this write.
flash-dfu: all
	@python3 tools/dfu_fit_check.py $(BUILD_DIR)/$(TARGET).bin $(DFU_UTIL)
	$(call dfu_write,$(BUILD_DIR)/$(TARGET).bin,$(DFU_FLASH_ADDR))

# Same image; kept for the old split-image habit.
flash-dfu-code: flash-dfu

clean:
	rm -rf $(BUILD_DIR)

help:
	@echo "Targets: all (default), flash, clean, fetch"
	@echo "  flash          : SWD flash via OpenOCD (needs a probe; verifies)"
	@echo "  flash-dfu      : USB-DFU write of the firmware (~0.5 MB); blobs go to the W25Q separately"
	@echo "  fetch          : build/download the ESP32-C3 slave image (ESP_SLAVE_URL=<url> to download)"
	@echo "  build/blobs/   : blob-{gw0,gw1,esp}.bin + blobs-manifest.json for the W25Q blob slots"
	@echo "  dev-key        : make (or show) this machine's firmware signing key; dev builds then sign"
	@echo "                   build/*.bin.sig and trust it (RELEASE=1 builds trust only keys/release-N.pub)"
	@echo "Vars:    CUBE=$(CUBE)  OPENOCD_INTERFACE=$(OPENOCD_INTERFACE)"
	@echo "         DFU_UTIL=$(DFU_UTIL)  DFU_FLASH_ADDR=$(DFU_FLASH_ADDR)"
