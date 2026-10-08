#ifndef COMMAND_HANDLER_INTERNAL_H
#define COMMAND_HANDLER_INTERNAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "la_pins.h"

/*
 * command_handler_internal.h — package-private seam shared by the command-handler
 * translation units.
 *
 * command_handler.c owns the dispatch loop and the tightly-coupled instrument
 * machinery (capture/DAC/PSRAM/load/bulk/DAP), whose state is intentionally kept
 * file-local. The self-contained subsystem handlers that only talk to their own
 * driver module (CAN -> can_bus.c, OTA -> ota.c) live in their own files to keep
 * command_handler.c smaller; this header exposes the few helpers they need plus
 * their prototypes so dispatch_line() can reach them. Not part of the public
 * command_handler.h API.
 */

/* Reply helpers (defined in command_handler.c). */
void send_error(int conn_id, const char *message);            /* {"status":"error","message":...} */
void send_ok_str(int conn_id, const char *payload_json);      /* {"status":"ok","data":<payload>}  */

/* True while a capture/measure/LA/DAC op owns the shared PSRAM bus — an OTA (which
   also stages into PSRAM) must refuse while it is set. Defined in command_handler.c. */
bool heavy_in_flight(void);
/* heavy_in_flight(), or a connection holding the gate (an upload waiting for its replay) or a
   PSRAM waveform upload in progress: the test for starting an OTA, which takes the PSRAM bus. */
bool heavy_or_claimed(void);
/* NULL when the shared PSRAM/W25Q bus is free for a runtime W25Q write (company CA, proxy) or a
   console diagnostic, else the "busy: ..." refusal to send. Same test as heavy_or_claimed. */
const char *bus_busy_reason(void);
/* heavy_or_claimed() without the OTA session itself: a capture, bulk send, upload or a held gate. */
bool capture_or_upload_busy(void);

/* Every LA-bank operation needs the LA voltage chosen first; false = the error was sent. */
bool require_la_voltage(int conn_id);

/* The command handlers in command_handler.c. Every handler has this one signature so the command
   table (cmd_tier.c) can reach it; the ones that take no arguments ignore `json`. */
void handle_generate(int conn_id, const char *json);
void handle_capture(int conn_id, const char *json);
void handle_capture_dual(int conn_id, const char *json);
void handle_capture_read(int conn_id, const char *json);
void handle_stream(int conn_id, const char *json);
void handle_ping(int conn_id, const char *json);
void handle_test(int conn_id, const char *json);
void handle_measure(int conn_id, const char *json);
void handle_load(int conn_id, const char *json);
void handle_load_bin(int conn_id, const char *json);
void handle_replay(int conn_id, const char *json);
void handle_dac_stop(int conn_id, const char *json);
void handle_dac_limits(int conn_id, const char *json);
void handle_dac_set(int conn_id, const char *json);
void handle_la(int conn_id, const char *json);
void handle_target_power(int conn_id, const char *json);
void handle_target_status(int conn_id, const char *json);
void handle_power_status(int conn_id, const char *json);
void handle_status(int conn_id, const char *json);
void handle_cloud_set(int conn_id, const char *json);
void handle_cloud_status(int conn_id, const char *json);
void handle_cloud_clear(int conn_id, const char *json);
void handle_wifi_set(int conn_id, const char *json);
void handle_wifi_clear(int conn_id, const char *json);
void handle_eth(int conn_id, const char *json);
void handle_speedtest(int conn_id, const char *json);
void handle_wifi_status(int conn_id, const char *json);
void handle_identity_public(int conn_id, const char *json);
void handle_identity_pop(int conn_id, const char *json);
void handle_dap_start(int conn_id, const char *json);
void handle_sensor_start(int conn_id, const char *json);
void handle_sensor_set(int conn_id, const char *json);
void handle_sensor_stop(int conn_id, const char *json);
void handle_sensor_status(int conn_id, const char *json);
void handle_sensor_regs(int conn_id, const char *json);
void handle_sensor_la(int conn_id, const char *json);
void handle_la_capture(int conn_id, const char *json);
void handle_uart_proxy_start(int conn_id, const char *json);
void handle_la_voltage(int conn_id, const char *json);
void handle_usb_cc(int conn_id, const char *json);
void handle_nrst(int conn_id, const char *json);
void handle_dac_mux(int conn_id, const char *json);
void handle_cal_switch(int conn_id, const char *json);
void handle_analog_path(int conn_id, const char *json);
void handle_dac_out(int conn_id, const char *json);
void handle_current_out(int conn_id, const char *json);
void handle_adc_read(int conn_id, const char *json);
void handle_calibrate(int conn_id, const char *json);
void handle_dac_control_loop(int conn_id, const char *json);
void handle_dac_loop_input(int conn_id, const char *json);
void handle_dac_loop_probe(int conn_id, const char *json);
void handle_fpga_image(int conn_id, const char *json);
void handle_psram_recover(int conn_id, const char *json);
void handle_psram_ping(int conn_id, const char *json);
void handle_identity_wipe(int conn_id, const char *json);

/* LA pin ownership glue + handlers (command_handler_pins.c). */
uint16_t la_pull_mask_now(void);              /* engaged LA pulls, bit la-1 */
void     la_pins_drive_mask(uint16_t mask);   /* GPIO_SET each pin in mask to its table state */
/* May `fn` claim las[0..n)?  Pins owned by a function in free_fns count as free.  false = the
   pin/pull conflict error was sent.  Commit with la_pins_claim once the FPGA side succeeded. */
bool     la_claim_or_error(int conn_id, la_fn_t fn, const uint8_t *las, size_t n, uint16_t free_fns);
/* Start an `la` step train with ownership.  0 ok; otherwise err holds the reply text and the
   return is -1 bad arguments, -2 a train is already running ("busy"), -3 pin conflict. */
int      la_step_begin(unsigned la, uint32_t steps, uint32_t delay_us, unsigned dir_la, int dir,
                       char *err, size_t cap);
void     la_step_poll(void);                  /* releases the train's pins once STEP_BUSY drops */
void     la_pins_on_gateware_reconfigured(void);
void     handle_la_pins(int conn_id, const char *json);
void     handle_gpio(int conn_id, const char *json);

/* SPI master on the LA pins (command_handler_spi.c). */
void handle_spi_start(int conn_id, const char *json);
void handle_spi_stop(int conn_id, const char *json);
void handle_spi_xfer(int conn_id, const char *json);
void handle_spi_flash(int conn_id, const char *json);
void handle_spi_stream(int conn_id, const char *json);
/* The PSRAM bytes the last `load_bin` with "psram" staged: false when none. */
bool command_handler_psram_stage(uint32_t *base, uint32_t *len);
void handle_spi_status(int conn_id, const char *json);
void spi_session_end(void);
void spi_on_gateware_reconfigured(void);

/* power_profile (command_handler_power.c). */
void handle_power_profile(int conn_id, const char *json);
void power_profile_service(void);            /* one-shot completion + paced result frames */
void power_profile_conn_closed(int conn_id);

/* CAN subsystem handlers (command_handler_can.c). */
void handle_can_config(int conn_id, const char *json);
void handle_can_write(int conn_id, const char *json);
void handle_can_read(int conn_id, const char *json);
void handle_can_status(int conn_id, const char *json);
void handle_can_respond(int conn_id, const char *json);
void handle_can_term(int conn_id, const char *json);
void handle_can_disable(int conn_id, const char *json);

/* OTA subsystem handlers (command_handler_ota.c). */
/* Policy commands (command_handler_policy.c). */
void handle_sig_policy(int conn_id, const char *json);
/* POLICY_SRC_USB / _CLOUD / _LAN for a connection id (pod_policy.h). */
#include "pod_policy.h"
policy_src_t command_handler_policy_src(int conn_id);
void handle_lan_policy(int conn_id, const char *json);
void handle_cloud_ca(int conn_id, const char *json);
void handle_cloud_proxy(int conn_id, const char *json);

/* The OTA session owner for a connection (ota.h): the cloud, the USB console or one LAN conn. */
#include "ota.h"
ota_owner_t ota_owner_for_conn(int conn_id);
/* The one test every transport runs before an OTA begin: NULL = go ahead, else the refusal
   (another transport's session, or a capture/upload on the PSRAM bus). */
const char *ota_begin_gate(ota_owner_t who);

void handle_ota_begin(int conn_id, const char *json);
void handle_ota_data(int conn_id, const char *json);
void handle_ota_end(int conn_id, const char *json);
void handle_ota_status(int conn_id, const char *json);
void handle_ota_abort(int conn_id, const char *json);
void handle_ota_selftest(int conn_id, const char *json);
void handle_ota_commit(int conn_id, const char *json);
void handle_blob_status(int conn_id, const char *json);

#endif /* COMMAND_HANDLER_INTERNAL_H */
