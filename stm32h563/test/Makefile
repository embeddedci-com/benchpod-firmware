# Host unit tests for the platform-independent firmware logic.
# Run with:  make            (builds + runs)
CC      ?= cc
CFLAGS  := -std=gnu11 -Wall -Wextra -O1 -I../src -I../port -I. -I../lib/esp_hosted/common

# Every test binary depends on every header (see the $(TESTS) rule near the bottom).  The rules
# below list only .c prerequisites, so before this a header edit left stale binaries behind and
# `make test` reported a PASS for code that was no longer being compiled — or a FAIL for a fix
# that was.  These suites link in seconds, so depending on all headers is cheaper than being
# wrong; it is also why there is no -MMD dependency machinery to get subtly incomplete.
HEADERS := $(wildcard ../src/*.h) $(wildcard ../port/*.h) $(wildcard *.h) $(wildcard mocks/*.h)

PORTABLE_SRCS := test_portable.c ../src/b64url.c ../src/sensor_bmp280.c ../src/esp_hosted_frame.c
DAP_SRCS      := test_dap.c ../src/dap.c mocks/mock_swd_ll.c
SWDLL_SRCS    := test_swd_ll.c ../src/swd_ll.c
NET_DHCP_SRCS := test_net_dhcp.c ../src/net_dhcp.c
XFER_SRCS     := test_xfer.c ../src/xfer.c
TGTPWR_SRCS   := test_target_power.c ../src/target_power.c ../src/tca9554.c
LARATE_SRCS   := test_la_rate.c ../src/la_rate.c
PSRAMALLOC_SRCS := test_psram_alloc.c ../src/psram_alloc.c
JSON_SRCS     := test_json.c ../src/bp_json.c
CONNTX_SRCS   := test_conn_tx.c ../src/conn_tx.c
CLOUDRX_SRCS  := test_cloud_rx.c
UARTSTAT_SRCS := test_uart_status.c
DACLOOP_SRCS  := test_dac_loop_params.c ../src/dac_loop_params.c
ADCSCALE_SRCS := test_adc_scale.c ../src/adc_scale.c
ESPPUMP_SRCS  := test_esp_pump.c ../src/esp_hosted_pump.c
LAPINS_SRCS   := test_la_pins.c ../src/la_pins.c ../src/bp_json.c
ETHDIAG_SRCS  := test_eth_diag.c ../src/eth_diag.c
PWRPROF_SRCS  := test_power_profile.c ../src/power_profile.c ../src/ina238.c ../src/bp_json.c
BOOTPOL_SRCS  := test_boot_policy.c ../src/boot_policy.c

# OTA staging/verify (src/ota.c). It hashes with mbedTLS, which lives in the STM32Cube tree the
# firmware build already needs, so this one suite is conditional: without Cube the rest of the
# host tests still build and run, and `make test` says why this was skipped rather than failing.
CUBE     ?= /opt/STM32CubeH5
MBEDTLS  := $(CUBE)/Middlewares/Third_Party/mbed-crypto
MBEDTLS_SHA256 := $(MBEDTLS)/library/sha256.c $(MBEDTLS)/library/platform_util.c
OTA_SRCS := test_ota.c ../src/ota.c mocks/mock_ota_psram.c $(MBEDTLS_SHA256)
HAVE_MBEDTLS := $(wildcard $(MBEDTLS)/library/sha256.c)
ifneq ($(HAVE_MBEDTLS),)
OTA_TARGET := build/test_ota
endif

.PHONY: all clean run
all: run

build/test_portable: $(PORTABLE_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(PORTABLE_SRCS) -lm -o $@

build/test_dap: $(DAP_SRCS) mocks/mock_swd_ll.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(DAP_SRCS) -lm -o $@

build/test_swd_ll: $(SWDLL_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(SWDLL_SRCS) -lm -o $@

build/test_net_dhcp: $(NET_DHCP_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(NET_DHCP_SRCS) -lm -o $@

build/test_xfer: $(XFER_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(XFER_SRCS) -lm -o $@

build/test_target_power: $(TGTPWR_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(TGTPWR_SRCS) -lm -o $@

build/test_la_rate: $(LARATE_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(LARATE_SRCS) -lm -o $@

build/test_psram_alloc: $(PSRAMALLOC_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(PSRAMALLOC_SRCS) -lm -o $@

build/test_json: $(JSON_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(JSON_SRCS) -lm -o $@

build/test_conn_tx: $(CONNTX_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(CONNTX_SRCS) -lm -o $@

build/test_cloud_rx: $(CLOUDRX_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(CLOUDRX_SRCS) -lm -o $@

build/test_uart_status: $(UARTSTAT_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(UARTSTAT_SRCS) -lm -o $@

build/test_dac_loop_params: $(DACLOOP_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(DACLOOP_SRCS) -lm -o $@

build/test_adc_scale: $(ADCSCALE_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(ADCSCALE_SRCS) -lm -o $@

build/test_esp_pump: $(ESPPUMP_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(ESPPUMP_SRCS) -lm -o $@

build/test_la_pins: $(LAPINS_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(LAPINS_SRCS) -lm -o $@

build/test_eth_diag: $(ETHDIAG_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(ETHDIAG_SRCS) -lm -o $@

build/test_power_profile: $(PWRPROF_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(PWRPROF_SRCS) -lm -o $@

build/test_boot_policy: $(BOOTPOL_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(BOOTPOL_SRCS) -lm -o $@

build/test_ota: $(OTA_SRCS) mocks/mock_ota_psram.h
	@mkdir -p build
	$(CC) $(CFLAGS) -I$(MBEDTLS)/include $(OTA_SRCS) -lm -o $@

TESTS := build/test_portable build/test_dap build/test_swd_ll build/test_net_dhcp build/test_xfer build/test_target_power build/test_la_rate build/test_psram_alloc build/test_json build/test_conn_tx build/test_cloud_rx build/test_uart_status build/test_dac_loop_params build/test_adc_scale build/test_esp_pump build/test_la_pins build/test_eth_diag build/test_power_profile build/test_boot_policy $(OTA_TARGET)

# Header edits must rebuild every suite (see HEADERS above).
$(TESTS): $(HEADERS)

run: $(TESTS)
	@./build/test_portable
	@./build/test_dap
	@./build/test_swd_ll
	@./build/test_net_dhcp
	@./build/test_xfer
	@./build/test_target_power
	@./build/test_la_rate
	@./build/test_psram_alloc
	@./build/test_json
	@./build/test_conn_tx
	@./build/test_cloud_rx
	@./build/test_uart_status
	@./build/test_dac_loop_params
	@./build/test_adc_scale
	@./build/test_esp_pump
	@./build/test_la_pins
	@./build/test_eth_diag
	@./build/test_power_profile
	@./build/test_boot_policy
ifneq ($(HAVE_MBEDTLS),)
	@./build/test_ota
else
	@echo "SKIP test_ota (mbedTLS not found under $(MBEDTLS); set CUBE=<path to STM32CubeH5>)"
endif

clean:
	rm -rf build
