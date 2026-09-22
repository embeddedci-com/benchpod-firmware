# bench-pod STM32H563ZIT6 firmware — Makefile
#
# Mirrors the RP2350 build ergonomics: `make`, `make flash`, `make clean`.
# Uses the STM32CubeH5 HAL referenced in place from $(CUBE) (like the RP2350
# build referenced /opt/pico-sdk), plus FreeRTOS vendored under lib/.

TARGET            := bench_pod_stm32
BUILD_DIR         := build

# Split DFU images (built by `all`; recipes further below). Defined up here so the
# `all:` prerequisite list — read before those recipes — sees them.
CODE_BIN          := $(BUILD_DIR)/$(TARGET)-code.bin
BLOBS_BIN         := $(BUILD_DIR)/$(TARGET)-blobs.bin

# STM32CubeH5 SDK location (override on the command line if installed elsewhere).
CUBE              ?= /opt/STM32CubeH5

# Programmer: OpenOCD interface cfg (stlink, cmsis-dap, jlink, ...).
OPENOCD_INTERFACE ?= stlink

# USB DFU flashing (no SWD probe): host tool + the STM32 main-flash base. Used by
# the `flash-dfu` target, which writes over the device's own USB-FS port once it
# is in the ROM bootloader (BOOT0 high at reset, or `benchpod dfu` on a running pod).
DFU_UTIL          ?= dfu-util
DFU_FLASH_ADDR    ?= 0x08000000
# Base of the FLASH_BLOBS region (embedded ESP-C3 + iCE40 blobs). MUST match the
# FLASH_BLOBS origin in config/STM32H563ZITX_FLASH.ld. Used by `flash-dfu-blobs`.
DFU_BLOBS_ADDR    ?= 0x080A0000

#######################################
# Toolchain
#######################################
PREFIX  := arm-none-eabi-
CC      := $(PREFIX)gcc
AS      := $(PREFIX)gcc -x assembler-with-cpp
CP      := $(PREFIX)objcopy
SZ      := $(PREFIX)size
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
  src/adc_scale.c \
  src/mcu_adc.c \
  src/board_rev.c \
  src/usb_cc.c \
  src/nrst_ctrl.c \
  src/psram_alloc.c \
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
  src/console.c \
  src/usbd_conf.c \
  src/usbd_desc.c \
  src/usbd_cdc_if.c \
  src/usb_device.c \
  src/ethernetif.c \
  src/net_server.c \
  src/net_dhcp.c \
  src/eth_diag.c \
  src/esp_hosted_frame.c \
  src/esp_hosted_spi.c \
  src/esp_hosted_pump.c \
  src/esp_netif.c \
  src/esp_wifi_ctrl.c \
  src/cloud_config.c \
  src/cloud_client.c \
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
  src/flash_compat.c \
  src/rng.c \
  src/hw_lock.c \
  src/psram.c \
  src/ice40_flash.c \
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
C_SOURCES += $(LWIP)/src/apps/altcp_tls/altcp_tls_mbedtls.c \
             $(LWIP)/src/apps/altcp_tls/altcp_tls_mbedtls_mem.c
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
# Embedded ESP32-C3 esp-hosted slave image (.incbin blob for flash-esp32).
ESP_SLAVE_BIN := ../esp32-hosted-slave/prebuilt/esp32c3-hosted-slave-merged.bin
ASM_SOURCES += src/esp_slave_fw.s

#######################################
# Flags
#######################################
MCU := -mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard

DEFS := -DUSE_HAL_DRIVER -DSTM32H563xx -DHSE_VALUE=25000000 \
        -DUSE_FULL_ERROR_LIST=1 -DBLAKE2_NO_UNROLLING=1 \
        -DMBEDTLS_CONFIG_FILE='"mbedtls_bench_config.h"'

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
LDFLAGS := $(MCU) -specs=nano.specs -specs=nosys.specs -T$(LDSCRIPT) \
           -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
           -Wl,--gc-sections -Wl,--print-memory-usage -lc -lm

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

# Embed the iCE40 bitstream(s) so the STM32 can (re)flash them (flash-ice40) and switch
# between gateware images at runtime.  SB_WARMBOOT can't reconfigure at runtime on this
# board (config-SPI pins are shared with the PSRAM bus), so image switching reprograms the
# config flash with the selected SINGLE image via the proven ice40_flash_program +
# CRESET-reconfig path.  Embed BOTH singles when the gateware `make multiboot` built them
# (slot0=closed-loop image, slot1=deep-DAC-replay); `fpga_bitstream` aliases image 0 so the
# existing flash-ice40 default is unchanged.  Fall back to the single default bitstream.
ICE40_LOOP_BIN := ../ice40/bench_pod_fpga_vbench_pod_loop.bin
ICE40_DEEP_BIN := ../ice40/bench_pod_fpga_vbench_pod_deep.bin
GEN_BITSTREAM := $(BUILD_DIR)/fpga_bitstream.h
# Place the (large, rarely-changing) bitstream arrays in their own section so the
# linker collects them into the FLASH_BLOBS region (.blobs) instead of .rodata —
# this is what lets `flash-dfu-code` skip them. `used` pins them past --gc-sections.
FPGA_BLOB_ATTR := __attribute__((section(".blob_fpga"),used))
# Prefer the two-image embed (loop + deep) when both single images exist; otherwise
# fall back to a single ../ice40/bench_pod_fpga_vbench_pod.bin if present. The single
# fallback is intentionally NOT a hard prerequisite (it may not exist in a two-image build).
$(GEN_BITSTREAM): $(wildcard $(ICE40_LOOP_BIN)) $(wildcard $(ICE40_DEEP_BIN)) $(wildcard ../ice40/bench_pod_fpga_vbench_pod.bin) $(wildcard $(ICE40_LOOP_BIN).gwversion) | $(BUILD_DIR)
	@if [ -f $(ICE40_LOOP_BIN) ] && [ -f $(ICE40_DEEP_BIN) ]; then \
	   printf '$(FPGA_BLOB_ATTR) static const unsigned char fpga_image0[] = {\n' > $@; \
	   xxd -i < $(ICE40_LOOP_BIN) >> $@; \
	   printf '};\nstatic const unsigned int fpga_image0_len = sizeof(fpga_image0);\n' >> $@; \
	   printf '$(FPGA_BLOB_ATTR) static const unsigned char fpga_image1[] = {\n' >> $@; \
	   xxd -i < $(ICE40_DEEP_BIN) >> $@; \
	   printf '};\nstatic const unsigned int fpga_image1_len = sizeof(fpga_image1);\n' >> $@; \
	   printf '#define FPGA_HAVE_IMAGES 1\n#define fpga_bitstream fpga_image0\n#define fpga_bitstream_len fpga_image0_len\n' >> $@; \
	 elif [ -f ../ice40/bench_pod_fpga_vbench_pod.bin ]; then \
	   printf '$(FPGA_BLOB_ATTR) static const unsigned char fpga_bitstream[] = {\n' > $@; \
	   xxd -i < ../ice40/bench_pod_fpga_vbench_pod.bin >> $@; \
	   printf '};\nstatic const unsigned int fpga_bitstream_len = sizeof(fpga_bitstream);\n' >> $@; \
	 else \
	   printf '$(FPGA_BLOB_ATTR) static const unsigned char fpga_bitstream[1]={0};\nstatic const unsigned int fpga_bitstream_len=0;\n' > $@; \
	 fi
	@# The GATEWARE_VERSION the gateware build recorded next to the image (0 = unknown, which
	@# turns the boot-time gateware update off).
	@v=$$(cat $(ICE40_LOOP_BIN).gwversion 2>/dev/null); \
	 if [ -f $(ICE40_LOOP_BIN) ] && [ -n "$$v" ]; then printf '#define FPGA_EMBEDDED_GW_VERSION %s\n' "$$v" >> $@; \
	 else printf '#define FPGA_EMBEDDED_GW_VERSION 0\n' >> $@; fi
$(BUILD_DIR)/console.o: $(GEN_BITSTREAM)

# Re-assemble the embedded C3 image whenever the blob changes. The .incbin path
# in src/esp_slave_fw.s is relative to this Makefile's directory.
$(BUILD_DIR)/esp_slave_fw.o: $(ESP_SLAVE_BIN)

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
# ESP32-C3 esp-hosted slave image). Default: ESP-IDF Docker build, pinned by
# esp32-hosted-slave/dependencies.lock and CONFIG_APP_REPRODUCIBLE_BUILD to be
# byte-for-byte reproducible for the actual firmware content. Override with
# ESP_SLAVE_URL to download a prebuilt asset (e.g. in CI) instead of running
# Docker.
#
# The committed sha256 is hard-enforced on a DOWNLOAD (ESP_SLAVE_URL) — that's
# a real transfer-integrity check. It's advisory-only on a fresh Docker build:
# ESP-IDF embeds a self-referential app_elf_sha256 field (esp_app_desc_t) whose
# computation isn't fully covered by CONFIG_APP_REPRODUCIBLE_BUILD, so a build
# can differ from the committed hash by exactly that field (plus the knock-on
# whole-image trailer checksum) even when the actual compiled firmware is
# byte-identical (verified: only those two 32-byte fields differed between a
# local and a CI build of the same source/lock). Failing the build over that
# would block every release for a difference with no functional effect.
ESP_SLAVE_DIR := ../esp32-hosted-slave
ESP_SLAVE_URL ?=
fetch:
	@if [ -n "$(ESP_SLAVE_URL)" ]; then \
	   echo "==> Downloading esp-hosted slave image: $(ESP_SLAVE_URL)"; \
	   mkdir -p $(ESP_SLAVE_DIR)/prebuilt; \
	   curl -fL "$(ESP_SLAVE_URL)" -o $(ESP_SLAVE_BIN); \
	   echo "==> Verifying sha256 (download integrity)..."; \
	   ( cd $(ESP_SLAVE_DIR)/prebuilt && shasum -a 256 -c esp32c3-hosted-slave-merged.bin.sha256 ); \
	 else \
	   echo "==> Building ESP32-C3 esp-hosted slave image (ESP-IDF Docker; needs Docker)..."; \
	   ( cd $(ESP_SLAVE_DIR) && docker run --rm \
	       -v "$$PWD:/board" -v "$$PWD/work:/work" -v "$$PWD/prebuilt:/out" \
	       espressif/idf:release-v5.5 bash /board/build.sh ); \
	   echo "==> Checking sha256 (advisory — see comment above `fetch`)..."; \
	   ( cd $(ESP_SLAVE_DIR)/prebuilt && shasum -a 256 -c esp32c3-hosted-slave-merged.bin.sha256 ) \
	     || echo "==> sha256 mismatch on a fresh build — expected (see comment above), continuing"; \
	 fi
	@echo "==> OK: $(ESP_SLAVE_BIN)"

.PHONY: all flash flash-dfu flash-dfu-code flash-dfu-blobs clean help test fetch
all: check-uid $(GEN_BITSTREAM) $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex \
     $(BUILD_DIR)/$(TARGET).bin $(CODE_BIN) $(BLOBS_BIN)

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

# The vendored LwIP altcp_tls_mbedtls.c is mbedTLS 2.x-era: it reads the now-private
# ssl_context.out_left (needs MBEDTLS_ALLOW_PRIVATE_ACCESS) and the renamed-away
# MBEDTLS_SSL_MAX_CONTENT_LEN (now split into IN/OUT) — alias it back for this file only.
$(BUILD_DIR)/altcp_tls_mbedtls.o: CFLAGS += -DMBEDTLS_ALLOW_PRIVATE_ACCESS \
                                            -DMBEDTLS_SSL_MAX_CONTENT_LEN=16384

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

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

# Split images for fast DFU. The big embedded blobs (ESP-C3 slave image + iCE40
# bitstreams) live in the dedicated high-flash .blobs region and rarely change,
# so a routine firmware edit only needs the ~0.4 MB code image re-flashed instead
# of the full ~1.7 MB image (~4x faster over USB-FS DFU).
#   code.bin  — everything EXCEPT .blobs → compact image based at $(DFU_FLASH_ADDR)
#   blobs.bin — ONLY .blobs             → image based at $(DFU_BLOBS_ADDR)
# --remove-section drops .blobs so the highest remaining section (code/rodata) sets
# the image length; --only-section keeps just .blobs so the image starts at its base.
# (CODE_BIN / BLOBS_BIN are defined near the top so `all` can list them as prereqs.)
$(CODE_BIN): $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O binary -S --remove-section=.blobs $< $@
	@echo "code image:  $$(wc -c < $@) bytes  -> $(DFU_FLASH_ADDR)"

$(BLOBS_BIN): $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O binary -S --only-section=.blobs $< $@
	@echo "blobs image: $$(wc -c < $@) bytes  -> $(DFU_BLOBS_ADDR)"

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

# Full image (code + blobs) — first flash of a board, or when both changed. The
# blob region sits above the code region with a gap that objcopy zero-fills, so
# this one write primes everything (required before the first flash-dfu-code).
flash-dfu: all
	$(call dfu_write,$(BUILD_DIR)/$(TARGET).bin,$(DFU_FLASH_ADDR))

# Firmware only (~0.4 MB, ~4x faster) — the routine dev-loop write. Leaves the
# blob region untouched, so the blobs must already be in flash (run `flash-dfu`
# once on a fresh board first).
flash-dfu-code: $(CODE_BIN)
	$(call dfu_write,$(CODE_BIN),$(DFU_FLASH_ADDR))

# Blobs only (~1.3 MB) — after a gateware rebuild (new iCE40 bitstreams) or to
# refresh the embedded ESP-C3 image, without rewriting the firmware.
flash-dfu-blobs: $(BLOBS_BIN)
	$(call dfu_write,$(BLOBS_BIN),$(DFU_BLOBS_ADDR))

clean:
	rm -rf $(BUILD_DIR)

help:
	@echo "Targets: all (default), flash, clean, fetch"
	@echo "  flash          : SWD flash via OpenOCD (needs a probe; verifies)"
	@echo "  flash-dfu      : full USB-DFU write (code + blobs) — first flash / clean image"
	@echo "  flash-dfu-code : DFU-write firmware only (~0.4 MB, ~4x faster) — routine dev loop"
	@echo "                   (blobs must already be flashed; run flash-dfu once on a fresh board)"
	@echo "  flash-dfu-blobs: DFU-write blobs only (~1.3 MB) — after a gateware/ESP-C3 rebuild"
	@echo "  fetch          : build/download the embedded ESP32-C3 slave image (ESP_SLAVE_URL=<url> to download)"
	@echo "Vars:    CUBE=$(CUBE)  OPENOCD_INTERFACE=$(OPENOCD_INTERFACE)"
	@echo "         DFU_UTIL=$(DFU_UTIL)  DFU_FLASH_ADDR=$(DFU_FLASH_ADDR)  DFU_BLOBS_ADDR=$(DFU_BLOBS_ADDR)"
