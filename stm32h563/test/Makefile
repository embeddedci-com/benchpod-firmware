# Host unit tests for the platform-independent firmware logic.
# Run with:  make            (builds + runs)
CC      ?= cc
CFLAGS  := -std=gnu11 -Wall -Wextra -O1 -I../src -I../port -I. -I../lib/esp_hosted/common
# Code under test logs through plain printf (bp_log.h); test_bp_log tests log_printf itself.
CFLAGS  += -DLOG_PRINTF_IS_PRINTF
# The command table (cmd_tier.c) without its handlers, so it links without command_handler.c.
CFLAGS  += -DCMD_TABLE_NO_HANDLERS

# Every test binary depends on every header (see the $(TESTS) rule near the bottom).  The rules
# below list only .c prerequisites, so before this a header edit left stale binaries behind and
# `make test` reported a PASS for code that was no longer being compiled — or a FAIL for a fix
# that was.  These suites link in seconds, so depending on all headers is cheaper than being
# wrong; it is also why there is no -MMD dependency machinery to get subtly incomplete.
HEADERS := $(wildcard ../src/*.h) $(wildcard ../port/*.h) $(wildcard *.h) $(wildcard mocks/*.h) \
           $(wildcard shim/lwip/*.h) $(wildcard shim_altcp/*.h) $(wildcard shim_altcp/arch/*.h) \
           $(wildcard shim_ch/*.h)

PORTABLE_SRCS := test_portable.c ../src/b64url.c ../src/esp_hosted_frame.c
# The C sensor models and the NMEA builder the engine replaced, kept in ref/ as references.
GPS_SRCS      := test_gps.c ref/gps_nmea.c
# The sensor definition engine (pure C11), against the C models it replaces.
SENSORDEF_CORE := ../src/sensor_def_json.c ../src/sensor_def_math.c ../src/sensor_def_vm.c ../src/sensor_def_expr.c \
                  ../src/sensor_def_compile.c ../src/sensor_def_inst.c ../src/sensor_def_regs.c \
                  ../src/sensor_def_uart.c ../src/sensor_def_analog.c ../src/sensor_stim.c
SENSORDEF_SRCS := test_sensor_def.c $(SENSORDEF_CORE) ref/sensor_bmp280.c ref/sensor_sht4x.c \
                  ref/sensor_mpu6050.c ref/gps_nmea.c
# The sensor manager and its commands end to end against a fake emulation gateware.
SENSORMGR_SRCS := test_sensor_mgr.c fake_fpga_emu.c $(SENSORDEF_CORE) ../src/sensor_mgr.c ../src/sensor_cmd.c \
                  ../src/la_pins.c ../src/la_pull.c fake_pull.c ../src/pull_exp_logic.c ../src/board_caps.c ../src/bp_json.c ../src/bp_errors.c
SENSORDEF_FUZZ_SRCS := test_sensor_def_fuzz.c $(SENSORDEF_CORE)
SANITIZE := -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all
DAP_SRCS      := test_dap.c ../src/dap.c ../src/dap_jtag.c mocks/mock_swd_ll.c mocks/mock_jtag_ll.c
DAPJ_SRCS     := test_dap_jtag.c ../src/dap.c ../src/dap_jtag.c mocks/mock_swd_ll.c mocks/mock_jtag_ll.c
DAPID_SRCS   := test_dap_id.c ../src/dap_id.c
SWDLL_SRCS    := test_swd_ll.c ../src/gw_caps.c ../src/swd_ll.c
NET_DHCP_SRCS := test_net_dhcp.c ../src/net_dhcp.c
NET_ROUTE_SRCS := test_net_route.c ../src/net_route.c
USB_DHCPD_SRCS := test_usb_dhcpd.c ../src/usb_dhcpd.c
XFER_SRCS     := test_xfer.c ../src/xfer.c
TGTPWR_SRCS   := test_target_power.c ../src/target_power.c ../src/tca9554.c ../src/efuse_flt.c
STATUSLED_SRCS := test_status_led.c ../src/status_led.c
EMILOG_SRCS   := test_emi_log.c ../src/emi_log.c
EMIHTTP_SRCS  := test_emi_http.c ../src/emi_http.c
EMISCHED_SRCS := test_emi_sched.c ../src/emi_sched.c
RECOVERY_SRCS := test_recovery.c ../src/recovery.c
LARATE_SRCS   := test_la_rate.c ../src/la_rate.c
PSRAMALLOC_SRCS := test_psram_alloc.c ../src/psram_alloc.c
BUSOWNER_SRCS := test_bus_owner.c ../src/bus_owner.c
XSPIGUARD_SRCS := test_xspi_guard.c ../src/xspi_guard.c
USBFLASHWAIT_SRCS := test_usb_flash_wait.c
MSCFLASH_SRCS := test_msc_flash.c ../src/xip_mod_msc_flash.c ../src/msc_flash_names.c
CTRLFLASH_SRCS := test_ctrl_flash.c ../src/xip_mod_ctrl_flash.c ../src/ctrl_flash_names.c
PICOBOOT_SRCS := test_picoboot.c ../src/xip_mod_ctrl_flash.c ../src/ctrl_flash_names.c
USBHID_SRCS   := test_usb_hid.c ../src/usb_hid_core.c
PROGUPDI_SRCS := test_prog_updi.c prog_sim.c ../src/prog_updi.c
PROGPIC_SRCS := test_prog_pic.c prog_sim.c ../src/prog_pic.c
PROGC2_SRCS := test_prog_c2.c prog_sim.c ../src/prog_c2.c
PROGTPI_SRCS := test_prog_tpi.c prog_sim.c ../src/prog_tpi.c
PROGPDI_SRCS := test_prog_pdi.c prog_sim.c ../src/prog_pdi.c
PROGDESC_SRCS := test_prog_desc.c ../src/prog_desc.c ../src/bp_json.c
PROGSBW_SRCS := test_prog_sbw.c prog_sim.c ../src/prog_sbw.c
SERFLASH_SRCS := test_ser_flash.c ../src/xip_mod_ser_flash.c ../src/ser_flash_names.c
PSRAMREG_SRCS := test_psram_regions.c ../src/psram_regions.c
JSON_SRCS     := test_json.c ../src/bp_json.c
BP_ERRORS_SRCS := test_bp_errors.c ../src/bp_errors.c ../src/bp_json.c
WIRING_DOC_SRCS := test_wiring_doc.c ../src/wiring_doc.c
CONNTX_SRCS   := test_conn_tx.c ../src/conn_tx.c
CLOUDRX_SRCS  := test_cloud_rx.c ../src/cloud_rx.c
CLOUDCAPS_SRCS := test_cloud_caps.c ../src/cloud_caps.c ../src/bp_json.c
CLOUDREPLY_SRCS := test_cloud_reply_cap.c ../src/cloud_reply_cap.c ../src/bp_errors.c ../src/bp_json.c
DEVID_SRCS     = test_device_identity.c ../src/device_identity.c ../src/b64url.c mocks/mock_flash.c $(MONOCYPHER)
WSFRAME_SRCS  := test_ws_frame.c ../src/ws_frame.c ../src/bp_json.c
UARTSTAT_SRCS := test_uart_status.c
DACLOOP_SRCS  := test_dac_loop_params.c ../src/dac_loop_params.c
DACREARM_SRCS := test_dac_rearm.c ../src/dac_rearm.c
ADCSCALE_SRCS := test_adc_scale.c ../src/adc_scale.c
ESPPUMP_SRCS  := test_esp_pump.c ../src/esp_hosted_pump.c
SPIFLASH_SRCS := test_spi_flash.c ../src/spi_flash.c
SPISTREAM_SRCS := test_spi_stream.c ../src/spi_stream.c
I2CTXN_SRCS   := test_i2c_txn.c ../src/i2c_txn.c
LAPINS_SRCS   := test_la_pins.c ../src/la_pins.c ../src/la_pull.c fake_pull.c ../src/pull_exp_logic.c ../src/board_caps.c ../src/bp_json.c ../src/bp_errors.c
ETHDIAG_SRCS  := test_eth_diag.c ../src/eth_diag.c
# Wi-Fi bridge client, tested against the bridge firmware's own codec and pairing logic.
BRIDGE_CORE := ../../wifi-bridge/components/bridge_core
WIFIBRIDGE_SRCS := test_wifi_bridge.c ../src/wifi_bridge.c ../src/wifi_bridge_ota.c $(BRIDGE_CORE)/bwp.c $(BRIDGE_CORE)/hmac_sha256.c $(BRIDGE_CORE)/pairing.c \
                   $(BRIDGE_CORE)/bwp_crypto.c $(BRIDGE_CORE)/bwp_id.c ../src/fw_sign.c ../lib/monocypher/src/monocypher.c \
                   ../lib/monocypher/src/optional/monocypher-ed25519.c
PWRPROF_SRCS  := test_power_profile.c ../src/power_profile.c ../src/ina238.c ../src/bp_json.c
BOOTPOL_SRCS  := test_boot_policy.c ../src/boot_policy.c
ICE40SW_SRCS  := test_ice40_switch.c ../src/ice40_switch.c ../src/bp_json.c
# hw_worker.c is #included by its test; shim_worker has the FreeRTOS + HAL headers it needs.
HWWORKER_SRCS := test_hw_worker.c ../src/hw_worker.c ../src/net_reload.c
# boot_guard.c is #included by its test (shim_hal stands in for the HAL header).
BOOTGUARD_SRCS := test_boot_guard.c ../src/boot_guard.c ../src/boot_policy.c
FLASHLAYOUT_SRCS := test_flash_layout.c
BOARDVAR_SRCS := test_board_variant.c ../src/board_variant.c ../src/board_caps.c
BOARDCAPS_SRCS := test_board_caps.c ../src/board_caps.c
USBHOST_SRCS  := test_usb_host_desc.c ../src/usb_host_desc.c ../src/usb_host_reply.c
USBSERCON_SRCS := test_usb_serial_con.c ../src/usb_serial_con.c ../src/bp_json.c ../src/b64url.c
USBCHECK_SRCS := test_usb_check.c ../src/usb_check_core.c ../src/bp_json.c
GWCAPS_SRCS := test_gw_caps.c ../src/gw_caps.c
LAIO_SRCS := test_la_io.c ../src/la_io.c ../src/board_caps.c
VREF_SRCS     := test_vref.c ../src/vref_logic.c ../src/bp_json.c
PULLEXP_SRCS  := test_pull_exp.c ../src/pull_exp_logic.c
# pull_exp.c is #included by its state test (shim_pull stands in for the HAL, I2C, pin 21, power).
PULLEXPSTATE_SRCS := test_pull_exp_state.c ../src/pull_exp.c ../src/pull_exp_logic.c ../src/board_caps.c
MODEE_SRCS := test_module_eeprom.c ../src/module_eeprom.c
DACLINK_SRCS := test_dac_link.c ../src/dac_link.c ../src/module_eeprom.c ../src/board_caps.c ../src/bp_json.c
BUSREC_SRCS   := test_bus_recovery.c
CANTIM_SRCS   := test_can_timing.c ../src/can_timing.c ../src/can_ring.c
CFGSTORE_SRCS := test_config_store.c ../src/config_store.c ../src/cloud_config.c mocks/mock_flash.c
DACLIM_SRCS   := test_dac_limits.c ../src/dac_limits.c ../src/dac_quad.c ../src/analog_path.c ../src/cal_data.c ../src/board_caps.c ../src/config_store.c ../src/bp_json.c mocks/mock_flash.c
ADCCAL_SRCS  := test_adc_cal.c ../src/adc_cal.c ../src/cal_data.c ../src/board_caps.c ../src/adc_scale.c ../src/config_store.c mocks/mock_flash.c
ANAPATH_SRCS := test_analog_path.c ../src/analog_path.c ../src/dac_quad.c ../src/dac_limits.c ../src/cal_data.c ../src/board_caps.c ../src/config_store.c ../src/bp_json.c mocks/mock_flash.c
DACQUAD_SRCS := test_dac_quad.c ../src/dac_quad.c ../src/analog_path.c ../src/dac_limits.c ../src/cal_data.c ../src/board_caps.c ../src/config_store.c ../src/bp_json.c mocks/mock_flash.c
CURROUT_SRCS := test_current_out.c ../src/current_out.c ../src/cal_data.c ../src/board_caps.c
DACCODE16_SRCS := test_dac_code16.c ../src/cal_data.c ../src/board_caps.c
WAVE16_SRCS := test_wave16.c ../src/wave16.c ../src/sensor_def_math.c ../src/cal_data.c ../src/board_caps.c ../src/current_out.c
# Wi-Fi control state machine, compiled unmodified against a fake transport/C3.  shim/ stands in
# for the two lwIP headers esp_netif.h pulls in (only the netif type name is needed).
WIFICTRL_SRCS := test_esp_wifi_ctrl.c ../src/esp_wifi_ctrl.c ../src/log_level.c ../src/c3_restarts.c ../src/wifi_scan.c ../src/bp_json.c
C3RESTARTS_SRCS := test_c3_restarts.c ../src/c3_restarts.c
WIFISCAN_SRCS := test_wifi_scan.c ../src/wifi_scan.c ../src/bp_json.c
# Where wifi-clear erases on the ESP32-C3: the NVS partition, looked up in the slave image.
ESPPART_SRCS  := test_esp_part_table.c ../src/esp_part_table.c
# SCPI reply pacing: scpi_server.c on the real libscpi, instrument + conn ring faked.
SCPI_LIB      := ../lib/scpi-parser/libscpi
SCPI_SRCS     := test_scpi_paced.c ../src/scpi_server.c ../src/b64url.c \
                 ../src/cmd_gate.c ../src/cmd_tier.c ../src/bp_json.c ../src/bp_errors.c ../src/adc_pool.c \
                 $(addprefix $(SCPI_LIB)/src/,error.c expression.c fifo.c ieee488.c \
                   lexer.c minimal.c parser.c units.c utils.c)

# OTA staging/verify (src/ota.c). It hashes with mbedTLS (the lib/stm32cube/stm32-mw-mbedtls
# submodule), so this one suite is conditional: without the submodule checked out the rest of the
# host tests still build and run, and `make test` says why this was skipped rather than failing.
STM32CUBE ?= $(abspath ../lib/stm32cube)
MBEDTLS  := $(STM32CUBE)/stm32-mw-mbedtls
MBEDTLS_SHA256 := $(MBEDTLS)/library/sha256.c $(MBEDTLS)/library/platform_util.c
# Signed-manifest check (src/fw_sign.c) on the real Monocypher; fw_sign_keys_gen.h here holds the
# test vectors' key (vectors/, made by tools/fwsign.py vectors).
MONOCYPHER := ../lib/monocypher/src/monocypher.c ../lib/monocypher/src/optional/monocypher-ed25519.c
MONO_INC   := -I../lib/monocypher/src -I../lib/monocypher/src/optional
PODPOL_SRCS := test_pod_policy.c ../src/pod_policy.c ../src/log_level.c ../src/fw_sign.c ../src/config_store.c mocks/mock_flash.c $(MONOCYPHER) ../src/bp_errors.c ../src/bp_json.c
NETRELOAD_SRCS := test_net_reload.c ../src/net_reload.c
LEASE_SRCS := test_lease_gate.c ../src/lease_gate.c ../src/cmd_tier.c ../src/bp_json.c
LEASELOCAL_SRCS := test_lease_local.c ../src/lease_local.c ../src/cmd_tier.c ../src/bp_json.c
ADCPOOL_SRCS := test_adc_pool.c ../src/adc_pool.c
CMDTIER_SRCS := test_cmd_tier.c ../src/cmd_tier.c ../src/bp_json.c
LANCLIENTS_SRCS := test_lan_clients.c ../src/lan_clients.c ../src/bp_json.c
CMDGATE_SRCS := test_cmd_gate.c ../src/cmd_gate.c ../src/cmd_tier.c ../src/bp_json.c ../src/bp_errors.c
# The command_handler modules, each on its own with the modules around it faked in the test
# (shim_ch: the HAL tick/reset they call).
CH_INC := -Ishim_ch
CHTRANSPORT_SRCS := test_ch_transport.c ../src/command_handler_transport.c ../src/lease_local.c
CHDAP_SRCS := test_ch_dap.c ../src/command_handler_dap.c ../src/dap_id.c ../src/bp_json.c ../src/bp_errors.c
CHUART_SRCS := test_ch_uart.c ../src/gw_caps.c ../src/command_handler_uart.c ../src/bp_json.c ../src/board_caps.c ../src/la_pins.c \
              ../src/la_pull.c fake_pull.c ../src/pull_exp_logic.c ../src/bp_errors.c
CHDAC_SRCS := test_ch_dac.c ../src/gw_caps.c ../src/command_handler_dac.c ../src/bp_json.c ../src/b64url.c ../src/adc_pool.c \
              ../src/psram_regions.c ../src/dac_loop_params.c ../src/cal_data.c ../src/board_caps.c \
              ../src/wave16.c ../src/sensor_def_math.c ../src/current_out.c ../src/adc_cal.c ../src/config_store.c mocks/mock_flash.c ../src/bp_errors.c
CHCAL_SRCS := test_ch_calibrate.c ../src/command_handler_analog.c ../src/adc_cal.c ../src/cal_data.c ../src/board_caps.c ../src/adc_scale.c \
              ../src/config_store.c ../src/bp_json.c ../src/dac_limits.c ../src/dac_quad.c ../src/current_out.c mocks/mock_flash.c
CHCAPTURE_SRCS := test_ch_capture.c ../src/gw_caps.c ../src/command_handler_capture.c ../src/bp_json.c ../src/b64url.c \
                  ../src/adc_pool.c ../src/psram_regions.c ../src/la_pins.c ../src/la_pull.c fake_pull.c ../src/pull_exp_logic.c ../src/board_caps.c ../src/psram_alloc.c ../src/la_rate.c ../src/bp_errors.c
# The LA pull commands and the pull driver on every board revision (shim_pull: pull_exp.c's HAL).
CHPINS_SRCS := test_ch_pins.c ../src/gw_caps.c ../src/command_handler_pins.c ../src/la_pull.c ../src/pull_tca9554.c ../src/la_pins.c ../src/board_caps.c \
               ../src/bp_json.c ../src/pull_exp.c ../src/pull_exp_logic.c ../src/bp_errors.c
# status: the real handler with its inputs faked at their longest, the caps[] list from cloud_caps.c.
CHSTATUS_SRCS := test_ch_status.c ../src/command_handler_status.c ../src/ice40_switch.c ../src/cloud_caps.c ../src/bp_json.c
CHSTATUS_DEFS := -DBENCHPOD_WIFI_BRIDGE=1 -I../../wifi-bridge/components/bridge_core/include \
                 -DLWIP_STATS=1 -DMEM_STATS=1 -DMEMP_STATS=1
CHNET_SRCS := test_ch_net.c ../src/command_handler_net.c ../src/c3_restarts.c ../src/wifi_scan.c ../src/bp_json.c ../src/config_store.c \
              ../src/cloud_config.c ../src/eth_diag.c mocks/mock_flash.c ../src/bp_errors.c
FWSIGN_SRCS := test_fw_sign.c ../src/fw_sign.c $(MONOCYPHER)
OTA_SRCS := test_ota.c ../src/ota.c ../src/bp_json.c ../src/bp_errors.c ../src/fw_info.c ../src/fw_sign.c ../src/b64url.c ../src/blob_store.c ../src/psram_regions.c mocks/mock_ota_psram.c mocks/mock_w25q.c $(MBEDTLS_SHA256) $(MONOCYPHER)
HAVE_MBEDTLS := $(wildcard $(MBEDTLS)/library/sha256.c)
BLOB_SRCS := test_blob_store.c ../src/blob_store.c mocks/mock_w25q.c $(MBEDTLS_SHA256)
# Which image the ESP32-C3 runs (src/c3_image.c), over the same W25Q model.
C3IMG_SRCS := test_c3_image.c ../src/c3_image.c ../src/blob_store.c mocks/mock_w25q.c $(MBEDTLS_SHA256)
# The company CA load (src/cloud_extras.c) parses X.509, so it links the whole mbedTLS library
# (default config), built once into an archive.
LANTLSPEM_SRCS := test_lan_tls_pem.c ../src/lan_tls_pem.c
# The LAN TLS resumption policy against a real mbedTLS server/client built with the pod's own
# configuration (config/mbedtls_bench_config.h), once into an archive.
LANTLSRESUME_SRCS := test_lan_tls_resume.c ../src/lan_tls_resume.c ../src/lan_clients.c ../src/bp_json.c
BENCHTLS_CFLAGS_ABS := -DMBEDTLS_CONFIG_FILE='"mbedtls_bench_config.h"' -I$(abspath ../config) -I$(abspath $(MBEDTLS))/include -I$(abspath $(MBEDTLS))/library
BENCHTLS_CFLAGS := -DMBEDTLS_CONFIG_FILE='"mbedtls_bench_config.h"' -I../config -I$(MBEDTLS)/include -I$(MBEDTLS)/library
CLOUDEXTRAS_SRCS := test_cloud_extras.c ../src/cloud_ca.c ../src/blob_store.c mocks/mock_w25q.c
MBEDTLS_ALL := $(wildcard $(MBEDTLS)/library/*.c)
# The ESP32-C3 ROM-loader write job (src/esp_rom_job.c) checks its write with the C3's MD5.
ESPROMJOB_SRCS := test_esp_rom_job.c ../src/esp_rom_job.c $(MBEDTLS)/library/md5.c $(MBEDTLS)/library/platform_util.c
ifneq ($(HAVE_MBEDTLS),)
OTA_TARGET := build/test_ota build/test_blob_store build/test_c3_image build/test_cloud_extras build/test_esp_rom_job
endif

.PHONY: all clean run docs-check
all: run

build/test_portable: $(PORTABLE_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(PORTABLE_SRCS) -lm -o $@

build/test_gps: $(GPS_SRCS) ref/gps_nmea.h
	@mkdir -p build
	$(CC) $(CFLAGS) -Iref $(GPS_SRCS) -lm -o $@

build/test_sensor_def: $(SENSORDEF_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) -Iref -ffp-contract=off $(SENSORDEF_SRCS) -lm -o $@

# Under ASan + UBSan, like the fuzz suite: the manager juggles heap blocks and FIFO/log indexes.
build/test_sensor_mgr: $(SENSORMGR_SRCS) fake_fpga_emu.h
	@mkdir -p build
	$(CC) $(CFLAGS) -O1 -g -ffp-contract=off $(SANITIZE) $(SENSORMGR_SRCS) -lm -o $@

# Malformed, truncated, huge and hostile definitions under ASan + UBSan.
build/test_sensor_def_fuzz: $(SENSORDEF_FUZZ_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) -O1 -g -ffp-contract=off $(SANITIZE) $(SENSORDEF_FUZZ_SRCS) -lm -o $@

build/test_dap: $(DAP_SRCS) mocks/mock_swd_ll.h mocks/mock_jtag_ll.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(DAP_SRCS) -lm -o $@

build/test_dap_jtag: $(DAPJ_SRCS) mocks/mock_swd_ll.h mocks/mock_jtag_ll.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(DAPJ_SRCS) -lm -o $@

build/test_dap_id: $(DAPID_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(DAPID_SRCS) -o $@

build/test_swd_ll: $(SWDLL_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(SWDLL_SRCS) -lm -o $@

build/test_net_dhcp: $(NET_DHCP_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(NET_DHCP_SRCS) -lm -o $@

build/test_net_route: $(NET_ROUTE_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(NET_ROUTE_SRCS) -lm -o $@

build/test_usb_dhcpd: $(USB_DHCPD_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(USB_DHCPD_SRCS) -lm -o $@

build/test_xfer: $(XFER_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(XFER_SRCS) -lm -o $@

build/test_target_power: $(TGTPWR_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(TGTPWR_SRCS) -lm -o $@

build/test_emi_log: $(EMILOG_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(EMILOG_SRCS) -o $@

build/test_emi_http: $(EMIHTTP_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(EMIHTTP_SRCS) -o $@

build/test_emi_sched: $(EMISCHED_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(EMISCHED_SRCS) -o $@

build/test_recovery: $(RECOVERY_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(RECOVERY_SRCS) -o $@

build/test_status_led: $(STATUSLED_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(STATUSLED_SRCS) -lm -o $@

build/test_la_rate: $(LARATE_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(LARATE_SRCS) -lm -o $@

build/test_psram_alloc: $(PSRAMALLOC_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(PSRAMALLOC_SRCS) -lm -o $@

build/test_bus_owner: $(BUSOWNER_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(BUSOWNER_SRCS) -lm -o $@

build/test_prog_updi: $(PROGUPDI_SRCS) ../src/prog_flash.h ../src/prog_wire.h prog_sim.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SANITIZE) $(PROGUPDI_SRCS) -o $@

build/test_prog_pic: $(PROGPIC_SRCS) ../src/prog_flash.h ../src/prog_wire.h prog_sim.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SANITIZE) $(PROGPIC_SRCS) -o $@

build/test_prog_c2: $(PROGC2_SRCS) ../src/prog_flash.h ../src/prog_wire.h prog_sim.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SANITIZE) $(PROGC2_SRCS) -o $@

build/test_prog_desc: $(PROGDESC_SRCS) ../src/prog_desc.h ../src/prog_pic.h ../src/prog_avr12.h ../src/prog_c2.h ../src/bp_json.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SANITIZE) $(PROGDESC_SRCS) -o $@

build/test_prog_tpi: $(PROGTPI_SRCS) ../src/prog_flash.h ../src/prog_wire.h prog_sim.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SANITIZE) $(PROGTPI_SRCS) -o $@

build/test_prog_pdi: $(PROGPDI_SRCS) ../src/prog_flash.h ../src/prog_wire.h prog_sim.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SANITIZE) $(PROGPDI_SRCS) -o $@

build/test_prog_sbw: $(PROGSBW_SRCS) ../src/prog_flash.h ../src/prog_wire.h ../src/prog_sbw.h prog_sim.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SANITIZE) $(PROGSBW_SRCS) -o $@

build/test_ser_flash: $(SERFLASH_SRCS) ../src/ser_flash.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SANITIZE) $(SERFLASH_SRCS) -o $@

build/test_ctrl_flash: $(CTRLFLASH_SRCS) ../src/ctrl_flash.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SANITIZE) $(CTRLFLASH_SRCS) -o $@

build/test_usb_flash_wait: $(USBFLASHWAIT_SRCS) ../src/usb_flash_wait.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SANITIZE) $(USBFLASHWAIT_SRCS) -o $@

build/test_picoboot: $(PICOBOOT_SRCS) ../src/ctrl_flash.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SANITIZE) $(PICOBOOT_SRCS) -o $@

build/test_usb_hid: $(USBHID_SRCS) ../src/usb_hid_core.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SANITIZE) $(USBHID_SRCS) -o $@

build/test_msc_flash: $(MSCFLASH_SRCS) ../src/msc_flash.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SANITIZE) $(MSCFLASH_SRCS) -o $@

build/test_xspi_guard: $(XSPIGUARD_SRCS) ../src/xspi_guard.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SANITIZE) $(XSPIGUARD_SRCS) -o $@

build/test_psram_regions: $(PSRAMREG_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(PSRAMREG_SRCS) -lm -o $@

build/test_json: $(JSON_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(JSON_SRCS) -lm -o $@

build/test_bp_errors: $(BP_ERRORS_SRCS) ../src/bp_errors.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(BP_ERRORS_SRCS) -lm -o $@

build/test_wiring_doc: $(WIRING_DOC_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(SANITIZE) $(WIRING_DOC_SRCS) -o $@

# The proxy layer compiles against lwIP's real altcp headers; the altcp core and the layers
# around it are fakes in the test (shim_altcp holds the host lwipopts.h and arch/cc.h).
PROXY_SRCS := test_altcp_bp_proxy.c ../src/altcp_bp_proxy.c
PROXY_INC  := -Ishim_altcp -I../lib/stm32_mw_lwip/src/include
build/test_altcp_bp_proxy: $(PROXY_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(PROXY_INC) $(PROXY_SRCS) -lm -o $@

# NSEC answers to AAAA (src/mdns_nsec*.c) against lwIP's real mDNS responder: a NO_SYS IPv4/UDP/IGMP
# stack on a fake netif (shim_mdns holds the host lwipopts.h; arch/cc.h comes from shim_altcp).
LWIP_SRC   := ../lib/stm32_mw_lwip/src
MDNS_SRCS  := test_mdns_nsec.c ../src/mdns_nsec.c ../src/mdns_nsec_lwip.c \
              $(addprefix $(LWIP_SRC)/core/,init.c def.c inet_chksum.c ip.c mem.c memp.c netif.c pbuf.c \
                                            udp.c timeouts.c stats.c sys.c) \
              $(addprefix $(LWIP_SRC)/core/ipv4/,ip4.c ip4_addr.c ip4_frag.c igmp.c) \
              $(addprefix $(LWIP_SRC)/apps/mdns/,mdns.c mdns_out.c mdns_domain.c)
MDNS_INC   := -Ishim_mdns -Ishim_altcp -I$(LWIP_SRC)/include
build/test_mdns_nsec: $(MDNS_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(SANITIZE) $(MDNS_INC) $(MDNS_SRCS) -o $@

build/test_conn_tx: $(CONNTX_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(CONNTX_SRCS) -lm -o $@

build/test_cloud_rx: $(CLOUDRX_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(CLOUDRX_SRCS) -lm -o $@

build/test_cloud_caps: $(CLOUDCAPS_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(CLOUDCAPS_SRCS) -lm -o $@

build/test_cloud_reply_cap: $(CLOUDREPLY_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(CLOUDREPLY_SRCS) -lm -o $@

build/test_device_identity: $(DEVID_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) -Imocks $(MONO_INC) $(DEVID_SRCS) -lm -o $@

build/test_ws_frame: $(WSFRAME_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(WSFRAME_SRCS) -lm -o $@

build/test_uart_status: $(UARTSTAT_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(UARTSTAT_SRCS) -lm -o $@

build/test_dac_loop_params: $(DACLOOP_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(DACLOOP_SRCS) -lm -o $@

build/test_dac_rearm: $(DACREARM_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(DACREARM_SRCS) -lm -o $@

build/test_adc_scale: $(ADCSCALE_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(ADCSCALE_SRCS) -lm -o $@

build/test_esp_pump: $(ESPPUMP_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(ESPPUMP_SRCS) -lm -o $@

build/test_spi_flash: $(SPIFLASH_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(SPIFLASH_SRCS) -lm -o $@

build/test_spi_stream: $(SPISTREAM_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(SPISTREAM_SRCS) -lm -o $@

build/test_bp_log: test_bp_log.c ../src/bp_log.c ../src/log_level.c
	@mkdir -p build
	$(CC) $(CFLAGS) -ULOG_PRINTF_IS_PRINTF test_bp_log.c ../src/bp_log.c ../src/log_level.c -o $@

build/test_log_level: test_log_level.c ../src/log_level.c
	@mkdir -p build
	$(CC) $(CFLAGS) test_log_level.c ../src/log_level.c -o $@

build/test_console_prompt: test_console_prompt.c ../src/console_prompt.c
	@mkdir -p build
	$(CC) $(CFLAGS) test_console_prompt.c ../src/console_prompt.c -o $@

build/test_i2c_txn: $(I2CTXN_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(I2CTXN_SRCS) -o $@

build/test_la_pins: $(LAPINS_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(LAPINS_SRCS) -lm -o $@

build/test_eth_diag: $(ETHDIAG_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(ETHDIAG_SRCS) -lm -o $@

build/test_wifi_bridge: $(WIFIBRIDGE_SRCS) ../src/wifi_bridge.h ../src/wifi_bridge_ota.h $(wildcard $(BRIDGE_CORE)/include/*.h)
	@mkdir -p build
	$(CC) $(CFLAGS) -I$(BRIDGE_CORE)/include -I../lib/monocypher/src -I../lib/monocypher/src/optional -I. $(WIFIBRIDGE_SRCS) -o $@

build/test_power_profile: $(PWRPROF_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(PWRPROF_SRCS) -lm -o $@

build/test_board_variant: $(BOARDVAR_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(BOARDVAR_SRCS) -lm -o $@

build/test_board_caps: $(BOARDCAPS_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(BOARDCAPS_SRCS) -lm -o $@

build/test_usb_host_desc: $(USBHOST_SRCS) ../src/usb_host.h ../src/usb_host_desc.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(USBHOST_SRCS) -o $@

build/test_usb_serial_con: $(USBSERCON_SRCS) ../src/usb_serial_con.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(USBSERCON_SRCS) -o $@
build/test_usb_check: $(USBCHECK_SRCS) ../src/usb_check_core.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(USBCHECK_SRCS) -o $@

build/test_gw_caps: $(GWCAPS_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(GWCAPS_SRCS) -lm -o $@

# la_io.c with host stand-ins (shim_la_io/) for the HAL and the bank mux pins.
build/test_la_io: $(LAIO_SRCS) $(wildcard shim_la_io/*.h)
	@mkdir -p build
	$(CC) $(CFLAGS) -Ishim_la_io $(LAIO_SRCS) -lm -o $@

build/test_vref: $(VREF_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(VREF_SRCS) -lm -o $@

build/test_pull_exp: $(PULLEXP_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(PULLEXP_SRCS) -lm -o $@

build/test_pull_exp_state: $(PULLEXPSTATE_SRCS) $(wildcard shim_pull/*.h)
	@mkdir -p build
	$(CC) $(CFLAGS) -Ishim_pull test_pull_exp_state.c ../src/pull_exp_logic.c ../src/board_caps.c -lm -o $@

build/test_module_eeprom: $(MODEE_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(MODEE_SRCS) -lm -o $@

build/test_dac_link: $(DACLINK_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(DACLINK_SRCS) -lm -o $@

build/test_flash_layout: $(FLASHLAYOUT_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(FLASHLAYOUT_SRCS) -lm -o $@

build/test_boot_policy: $(BOOTPOL_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(BOOTPOL_SRCS) -lm -o $@

build/test_ice40_switch: $(ICE40SW_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(ICE40SW_SRCS) -lm -o $@

build/test_hw_worker: $(HWWORKER_SRCS) $(wildcard shim_worker/*.h)
	@mkdir -p build
	$(CC) $(CFLAGS) -Ishim_worker test_hw_worker.c ../src/net_reload.c -lm -o $@

build/test_boot_guard: $(BOOTGUARD_SRCS) shim_hal/stm32h5xx_hal.h
	@mkdir -p build
	$(CC) $(CFLAGS) -Ishim_hal test_boot_guard.c ../src/boot_policy.c -lm -o $@

build/test_wifi_scan: $(WIFISCAN_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(WIFISCAN_SRCS) -lm -o $@

build/test_esp_wifi_ctrl: $(WIFICTRL_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) -Ishim $(WIFICTRL_SRCS) -lm -o $@

build/test_c3_restarts: $(C3RESTARTS_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(C3RESTARTS_SRCS) -o $@

build/test_esp_part_table: $(ESPPART_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(ESPPART_SRCS) -lm -o $@

# -Wno-missing-field-initializers: scpi_server.c's command table omits libscpi's `tag` field.
build/test_scpi_paced: $(SCPI_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) -Wno-missing-field-initializers -I$(SCPI_LIB)/inc $(SCPI_SRCS) -lm -o $@

build/test_lan_tls_pem: $(LANTLSPEM_SRCS)
	$(CC) $(CFLAGS) $(LANTLSPEM_SRCS) -o $@

build/test_config_store: $(CFGSTORE_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(CFGSTORE_SRCS) -lm -o $@

build/test_dac_limits: $(DACLIM_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(DACLIM_SRCS) -lm -o $@

build/test_analog_path: $(ANAPATH_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(ANAPATH_SRCS) -lm -o $@

build/test_dac_quad: $(DACQUAD_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(DACQUAD_SRCS) -lm -o $@

build/test_adc_cal: $(ADCCAL_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(ADCCAL_SRCS) -lm -o $@

build/test_current_out: $(CURROUT_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(CURROUT_SRCS) -lm -o $@

build/test_dac_code16: $(DACCODE16_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(DACCODE16_SRCS) -lm -o $@

build/test_wave16: $(WAVE16_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(WAVE16_SRCS) -lm -o $@

build/test_bus_recovery: $(BUSREC_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(BUSREC_SRCS) -lm -o $@

build/test_can_timing: $(CANTIM_SRCS) ../src/can_timing.h ../src/can_ring.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(CANTIM_SRCS) -o $@

build/test_blob_store: $(BLOB_SRCS) mocks/mock_w25q.h
	@mkdir -p build
	$(CC) $(CFLAGS) -I$(MBEDTLS)/include $(BLOB_SRCS) -lm -o $@

build/test_c3_image: $(C3IMG_SRCS) mocks/mock_w25q.h
	@mkdir -p build
	$(CC) $(CFLAGS) -I$(MBEDTLS)/include $(C3IMG_SRCS) -lm -o $@

build/mbedtls/libmbedtls_host.a: $(MBEDTLS_ALL)
	@mkdir -p build/mbedtls
	cd build/mbedtls && $(CC) -O1 -w -c -I$(MBEDTLS)/include -I$(MBEDTLS)/library $(MBEDTLS_ALL)
	ar rcs $@ build/mbedtls/*.o

build/mbedtls_bench/libmbedtls_bench.a: $(MBEDTLS_ALL) ../config/mbedtls_bench_config.h
	@rm -rf build/mbedtls_bench && mkdir -p build/mbedtls_bench
	@cd build/mbedtls_bench && $(CC) -O1 -w -c $(BENCHTLS_CFLAGS_ABS) $(MBEDTLS_ALL)
	ar rcs $@ build/mbedtls_bench/*.o

build/test_lan_tls_resume: $(LANTLSRESUME_SRCS) build/mbedtls_bench/libmbedtls_bench.a
	@mkdir -p build
	$(CC) $(CFLAGS) $(BENCHTLS_CFLAGS) $(LANTLSRESUME_SRCS) build/mbedtls_bench/libmbedtls_bench.a -o $@

build/test_cloud_extras: $(CLOUDEXTRAS_SRCS) build/mbedtls/libmbedtls_host.a mocks/mock_w25q.h
	@mkdir -p build
	$(CC) $(CFLAGS) -Ishim_rtos -Imocks -I$(MBEDTLS)/include $(CLOUDEXTRAS_SRCS) build/mbedtls/libmbedtls_host.a -lm -o $@

build/test_esp_rom_job: $(ESPROMJOB_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) -I$(MBEDTLS)/include $(ESPROMJOB_SRCS) -lm -o $@

build/test_ota: $(OTA_SRCS) mocks/mock_ota_psram.h mocks/mock_w25q.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(MONO_INC) -I$(MBEDTLS)/include $(OTA_SRCS) -lm -o $@

build/test_pod_policy: $(PODPOL_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(MONO_INC) $(PODPOL_SRCS) -lm -o $@

build/test_lease_gate: $(LEASE_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(LEASE_SRCS) -lm -o $@

build/test_lease_local: $(LEASELOCAL_SRCS) ../src/lease_local.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(LEASELOCAL_SRCS) -lm -o $@

build/test_net_reload: $(NETRELOAD_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(NETRELOAD_SRCS) -lm -o $@

build/test_adc_pool: $(ADCPOOL_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(ADCPOOL_SRCS) -lm -o $@

build/test_cmd_tier: $(CMDTIER_SRCS) vectors/cmd_tier_vectors.json
	@mkdir -p build
	$(CC) $(CFLAGS) $(CMDTIER_SRCS) -lm -o $@

build/test_lan_clients: $(LANCLIENTS_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(LANCLIENTS_SRCS) -o $@

build/test_cmd_gate: $(CMDGATE_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(CMDGATE_SRCS) -lm -o $@

build/test_ch_transport: $(CHTRANSPORT_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(CH_INC) $(CHTRANSPORT_SRCS) -lm -o $@

build/test_ch_dap: $(CHDAP_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(CH_INC) $(CHDAP_SRCS) -lm -o $@

build/test_ch_uart: $(CHUART_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(CH_INC) $(CHUART_SRCS) -lm -o $@

build/test_ch_calibrate: $(CHCAL_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(CH_INC) $(CHCAL_SRCS) -lm -o $@

build/test_ch_dac: $(CHDAC_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(CH_INC) -Ishim_rtos $(CHDAC_SRCS) -lm -o $@

build/test_ch_capture: $(CHCAPTURE_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(CH_INC) $(CHCAPTURE_SRCS) -lm -o $@

build/test_ch_pins: $(CHPINS_SRCS) $(wildcard shim_pull/*.h)
	@mkdir -p build
	$(CC) $(CFLAGS) -Ishim_pull $(CHPINS_SRCS) -lm -o $@

build/test_ch_status: $(CHSTATUS_SRCS) ../src/command_handler_status.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(CH_INC) $(PROXY_INC) $(CHSTATUS_DEFS) $(CHSTATUS_SRCS) -lm -o $@

# command_handler_net.c reaches lwIP's types through net_server.h (as test_altcp_bp_proxy does).
build/test_ch_net: $(CHNET_SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(CH_INC) $(PROXY_INC) $(CHNET_SRCS) -lm -o $@

build/test_fw_sign: $(FWSIGN_SRCS) vectors/fwsign_vectors.h fw_sign_keys_gen.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(MONO_INC) $(FWSIGN_SRCS) -lm -o $@

TESTS := build/test_portable build/test_sensor_def build/test_sensor_mgr build/test_sensor_def_fuzz build/test_gps build/test_dap build/test_dap_jtag build/test_dap_id build/test_swd_ll build/test_net_dhcp build/test_net_route build/test_usb_dhcpd build/test_xfer build/test_target_power build/test_status_led build/test_la_rate build/test_psram_alloc build/test_bus_owner build/test_xspi_guard build/test_msc_flash build/test_usb_flash_wait build/test_ctrl_flash build/test_ser_flash build/test_prog_updi build/test_prog_pic build/test_prog_c2 build/test_prog_tpi build/test_prog_desc build/test_prog_pdi build/test_prog_sbw build/test_psram_regions build/test_json build/test_bp_errors build/test_wiring_doc build/test_conn_tx build/test_cloud_rx build/test_cloud_caps build/test_cloud_reply_cap build/test_device_identity build/test_ws_frame build/test_uart_status build/test_dac_loop_params build/test_dac_rearm build/test_adc_scale build/test_esp_pump build/test_la_pins build/test_spi_flash build/test_spi_stream build/test_i2c_txn build/test_eth_diag build/test_wifi_bridge build/test_power_profile build/test_boot_policy build/test_ice40_switch build/test_boot_guard build/test_hw_worker build/test_esp_wifi_ctrl build/test_c3_restarts build/test_wifi_scan build/test_esp_part_table build/test_scpi_paced build/test_bus_recovery build/test_can_timing build/test_config_store build/test_lan_tls_pem build/test_dac_limits build/test_dac_quad build/test_analog_path build/test_adc_cal build/test_current_out build/test_dac_code16 build/test_wave16 build/test_flash_layout build/test_board_variant build/test_board_caps build/test_usb_host_desc build/test_usb_check build/test_picoboot build/test_usb_hid build/test_usb_serial_con build/test_gw_caps build/test_la_io build/test_vref build/test_pull_exp build/test_pull_exp_state build/test_module_eeprom build/test_dac_link build/test_fw_sign build/test_adc_pool build/test_cmd_tier build/test_lan_clients build/test_cmd_gate build/test_pod_policy build/test_lease_gate build/test_lease_local build/test_net_reload build/test_altcp_bp_proxy build/test_mdns_nsec build/test_ch_transport build/test_ch_dap build/test_ch_uart build/test_ch_pins build/test_ch_dac build/test_ch_calibrate build/test_ch_capture build/test_ch_net build/test_ch_status build/test_lan_tls_resume $(OTA_TARGET)

TESTS += build/test_bp_log build/test_log_level build/test_console_prompt build/test_emi_log build/test_emi_http build/test_emi_sched build/test_recovery

# Header edits must rebuild every suite (see HEADERS above).
$(TESTS): $(HEADERS)

run: $(TESTS)
	@./build/test_portable
	@./build/test_sensor_def
	@./build/test_sensor_mgr
	@./build/test_sensor_def_fuzz
	@./build/test_gps
	@./build/test_dap
	@./build/test_dap_jtag
	@./build/test_dap_id
	@./build/test_swd_ll
	@./build/test_net_dhcp
	@./build/test_net_route
	@./build/test_usb_dhcpd
	@./build/test_xfer
	@./build/test_target_power
	@./build/test_la_rate
	@./build/test_emi_log
	@./build/test_emi_http
	@./build/test_emi_sched
	@./build/test_recovery
	@./build/test_psram_alloc
	@./build/test_bus_owner
	@./build/test_xspi_guard
	@./build/test_msc_flash
	@./build/test_usb_flash_wait
	@./build/test_ctrl_flash
	@./build/test_picoboot
	@./build/test_usb_hid
	@./build/test_ser_flash
	@./build/test_prog_updi
	@./build/test_prog_pic
	@./build/test_prog_c2
	@./build/test_prog_tpi
	@./build/test_prog_desc
	@./build/test_prog_pdi
	@./build/test_prog_sbw
	@./build/test_psram_regions
	@./build/test_json
	@./build/test_bp_errors
	@./build/test_wiring_doc
	@./build/test_conn_tx
	@./build/test_altcp_bp_proxy
	@./build/test_mdns_nsec
	@./build/test_cloud_rx
	@./build/test_cloud_caps
	@./build/test_cloud_reply_cap > build/test_cloud_reply_cap.log || { cat build/test_cloud_reply_cap.log; exit 1; }
	@tail -n 1 build/test_cloud_reply_cap.log
	@./build/test_device_identity > build/test_device_identity.log || { cat build/test_device_identity.log; exit 1; }
	@tail -n 1 build/test_device_identity.log
	@./build/test_ws_frame
	@./build/test_uart_status
	@./build/test_dac_loop_params
	@./build/test_dac_rearm
	@./build/test_adc_scale
	@./build/test_esp_pump
	@./build/test_la_pins
	@./build/test_spi_flash
	@./build/test_spi_stream
	@./build/test_i2c_txn
	@./build/test_bp_log
	@./build/test_log_level
	@./build/test_console_prompt
	@./build/test_eth_diag
	@./build/test_wifi_bridge
	@./build/test_power_profile
	@./build/test_boot_policy
	@./build/test_ice40_switch
	@./build/test_boot_guard > build/test_boot_guard.log || { cat build/test_boot_guard.log; exit 1; }
	@tail -n 1 build/test_boot_guard.log
	@./build/test_hw_worker
	@./build/test_bus_recovery
	@./build/test_can_timing
	@./build/test_flash_layout
	@./build/test_board_variant
	@./build/test_board_caps
	@./build/test_usb_host_desc
	@./build/test_usb_serial_con
	@./build/test_usb_check
	@./build/test_gw_caps
	@./build/test_la_io
	@./build/test_vref
	@./build/test_pull_exp
	@./build/test_pull_exp_state
	@./build/test_module_eeprom
	@./build/test_dac_link
	@./build/test_fw_sign
	@./build/test_adc_pool
	@./build/test_cmd_tier
	@./build/test_lan_clients
	@./build/test_lan_tls_resume
	@./build/test_cmd_gate
	@./build/test_pod_policy > build/test_pod_policy.log || { cat build/test_pod_policy.log; exit 1; }
	@tail -n 1 build/test_pod_policy.log
	@./build/test_lease_gate
	@./build/test_lease_local
	@./build/test_net_reload
	@./build/test_config_store > build/test_config_store.log || { cat build/test_config_store.log; exit 1; }
	@tail -n 1 build/test_config_store.log
	@./build/test_lan_tls_pem > build/test_lan_tls_pem.log || { cat build/test_lan_tls_pem.log; exit 1; }
	@tail -n 1 build/test_lan_tls_pem.log
	@./build/test_dac_limits > build/test_dac_limits.log || { cat build/test_dac_limits.log; exit 1; }
	@tail -n 1 build/test_dac_limits.log
	@./build/test_dac_quad > build/test_dac_quad.log || { cat build/test_dac_quad.log; exit 1; }
	@tail -n 1 build/test_dac_quad.log
	@./build/test_analog_path > build/test_analog_path.log || { cat build/test_analog_path.log; exit 1; }
	@tail -n 1 build/test_analog_path.log
	@./build/test_adc_cal > build/test_adc_cal.log || { cat build/test_adc_cal.log; exit 1; }
	@tail -n 1 build/test_adc_cal.log
	@./build/test_current_out > build/test_current_out.log || { cat build/test_current_out.log; exit 1; }
	@tail -n 1 build/test_current_out.log
	@./build/test_dac_code16 > build/test_dac_code16.log || { cat build/test_dac_code16.log; exit 1; }
	@tail -n 1 build/test_dac_code16.log
	@./build/test_wave16 > build/test_wave16.log || { cat build/test_wave16.log; exit 1; }
	@tail -n 1 build/test_wave16.log
	@./build/test_esp_wifi_ctrl
	@./build/test_c3_restarts
	@./build/test_wifi_scan
	@./build/test_esp_part_table
	@./build/test_ch_transport > build/test_ch_transport.log || { cat build/test_ch_transport.log; exit 1; }
	@tail -n 1 build/test_ch_transport.log
	@./build/test_ch_dap > build/test_ch_dap.log || { cat build/test_ch_dap.log; exit 1; }
	@tail -n 1 build/test_ch_dap.log
	@./build/test_ch_uart > build/test_ch_uart.log || { cat build/test_ch_uart.log; exit 1; }
	@tail -n 1 build/test_ch_uart.log
	@./build/test_ch_pins > build/test_ch_pins.log || { cat build/test_ch_pins.log; exit 1; }
	@tail -n 1 build/test_ch_pins.log
	@./build/test_ch_dac > build/test_ch_dac.log || { cat build/test_ch_dac.log; exit 1; }
	@tail -n 1 build/test_ch_dac.log
	@./build/test_ch_calibrate > build/test_ch_calibrate.log || { cat build/test_ch_calibrate.log; exit 1; }
	@tail -n 1 build/test_ch_calibrate.log
	@./build/test_ch_capture > build/test_ch_capture.log || { cat build/test_ch_capture.log; exit 1; }
	@tail -n 1 build/test_ch_capture.log
	@./build/test_ch_net > build/test_ch_net.log || { cat build/test_ch_net.log; exit 1; }
	@tail -n 1 build/test_ch_net.log
	@./build/test_ch_status
	@./build/test_scpi_paced
ifneq ($(HAVE_MBEDTLS),)
	@./build/test_ota
	@./build/test_blob_store
	@./build/test_c3_image
	@./build/test_cloud_extras > build/test_cloud_extras.log || { cat build/test_cloud_extras.log; exit 1; }
	@tail -n 1 build/test_cloud_extras.log
	@./build/test_esp_rom_job > build/test_esp_rom_job.log || { cat build/test_esp_rom_job.log; exit 1; }
	@tail -n 1 build/test_esp_rom_job.log
else
	@echo "SKIP test_ota, test_blob_store, test_cloud_extras and test_esp_rom_job (mbedTLS not found under $(MBEDTLS); run git submodule update --init stm32h563/lib/stm32cube/stm32-mw-mbedtls)"
endif
	@$(MAKE) --no-print-directory docs-check

# Every JSON verb, SCPI command and console command must be in docs/ (check_docs_coverage.py), and
# the console replies host tools parse must not change (check_console_contract.py), only
# board_rev/board_caps read the board revision (check_board_rev_gates.py), every indirect OCTOSPI1
# command goes through xspi_guard (check_xspi_guard.py), the command table's
# hand-kept copies (tier vectors, the API.md index) match k_tiers (check_cmd_table.py), and the
# feature table's outputs must be current and its docs links must name real anchors (gen_features.py).
PYTHON ?= python3
docs-check:
	@$(PYTHON) ../tools/emi/test_emi_tools.py
	@$(PYTHON) check_docs_coverage.py
	@$(PYTHON) check_console_contract.py
	@$(PYTHON) check_board_rev_gates.py
	@$(PYTHON) check_xspi_guard.py
	@$(PYTHON) check_cmd_table.py
	@$(PYTHON) ../../tools/gen_features.py --check

clean:
	rm -rf build
