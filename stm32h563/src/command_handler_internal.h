#ifndef COMMAND_HANDLER_INTERNAL_H
#define COMMAND_HANDLER_INTERNAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "command_handler.h"   /* the pseudo-connection ids */
#include "cmd_table.h"         /* cmd_desc_t */
#include "la_pins.h"
#include "bp_json.h"   /* bp_emit_t */
#include "b64url.h"
#include "gw_caps.h"
#include "bp_errors.h"  /* require_gw refuses with BP0017 */   /* gw_caps_t, require_gw */

/*
 * command_handler_internal.h — package-private seam shared by the command-handler
 * translation units.
 *
 * command_handler.c is the glue: replies, the heavy gate, dispatch_line and the
 * poll / conn_closed / gateware hooks. Each command_handler_*.c module owns its
 * state (file-local) and exposes only its handlers and hooks here:
 *   _transport  line assembly, the per-connection protocol and its raw modes
 *   _capture    captures and the paced bulk read-back
 *   _dac        DAC output, uploads (load / load_bin), replay, the control loop
 *   _uart       the UART proxy        _dap      the on-pod CMSIS-DAP probe
 *   _net        network provisioning and the cloud speed test
 *   _device     power, identity, the iCE40   _status   the status reply
 *   _analog     the analog front end
 *   _pins, _spi, _power, _can, _ota, _policy   LA pins, SPI, power profile, CAN, OTA, policy
 * A module with per-connection state has a <module>_conn_closed(conn_id) hook.
 * Not part of the public command_handler.h API.
 */

/* ---- Connections and their protocol ----
   The TCP conns are 0..CH_MAX_CONN-1 (net_server.c); the pseudo-connections are in
   command_handler.h. */
#define CH_MAX_CONN   5

/* Per-connection protocol, decided from the first non-whitespace byte:
   '{' -> JSON, anything else -> SCPI.  Lets a VISA/SCPI client and a JSON
   client share the single ESP-AT TCP server.  PROTO_DAP is entered only by an
   explicit JSON "dap_start" command and routes the connection's raw bytes to
   the on-pod CMSIS-DAP processor as length-framed packets until a zero-length
   frame leaves DAP mode. */
/* PROTO_REFUSED: the protocol picked is not served on this connection (command_handler_transport.c:
   SCPI while it is off, JSON on plaintext LAN under HTTPS-only); its bytes are dropped until the
   close. */
typedef enum { PROTO_UNKNOWN, PROTO_JSON, PROTO_SCPI, PROTO_UART, PROTO_DAP, PROTO_LOAD, PROTO_SPEEDTEST,
               PROTO_REFUSED } proto_t;

/* A connection's protocol (PROTO_UNKNOWN for an id outside the table) and setting it
   (command_handler_transport.c). */
proto_t conn_proto(int conn_id);
void    conn_proto_set(int conn_id, proto_t p);
void    transport_conn_closed(int conn_id);   /* drop the partial line, back to PROTO_UNKNOWN */

/* Run one complete, NUL-terminated JSON command line through the gates and its handler
   (command_handler.c). */
void dispatch_line(int conn_id, const char *buf);

/* is_tunnel_conn: true for any of the N cloud byte-tunnel pseudo-connections. */
static inline bool is_tunnel_conn(int conn_id) {
    return conn_id >= CH_CLOUD_TUNNEL_CONN && conn_id <= CH_CLOUD_TUNNEL_CONN_LAST;
}

/* conn_runs_state_machine: true for the TCP conns and the cloud tunnels — the connections whose raw
   bytes flow through command_handler_process()'s proto state machine (JSON line assembly plus the
   DAP/UART raw modes). The console/cloud-command pseudo-conns dispatch single lines instead. */
static inline bool conn_runs_state_machine(int conn_id) {
    return (conn_id >= 0 && conn_id < CH_MAX_CONN) || is_tunnel_conn(conn_id);
}

/* conn_has_socket: true only for real LwIP TCP conns — the ones with a pcb backing tcp_nodelay.
   The tunnel reuses the state machine but has no socket, so socket-only calls (at_set_tcp_nodelay)
   must be skipped for it. */
static inline bool conn_has_socket(int conn_id) {
    return conn_id >= 0 && conn_id < CH_MAX_CONN;
}

/* Reply helpers (defined in command_handler.c). */
void send_error(int conn_id, const char *message);            /* {"status":"error","message":...} */
void send_ok_str(int conn_id, const char *payload_json);      /* {"status":"ok","data":<payload>}  */
/* The same for a payload that may not fit send_ok_str's 256-byte reply buffer (up to
   BP_CLOUD_REPLY_MAX - 64): the reply is built on the heap. */
void send_ok_large(int conn_id, const char *payload_json);

/* A gateware gate (a gw_caps() field): true when the running gateware has it, else the handler
   refuses with gateware_too_old (BP0017: feature needs gateware v`min`) and returns. */
static inline bool require_gw(int conn_id, bool have, const char *feature, int min) {
    if (!have) bp_err_gateware_too_old(conn_id, feature, (int32_t)min);
    return have;
}

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

/* One raw newline-delimited JSON line on a (tunnel) conn: load_bin acks and speed-test marks. */
void cloud_send_json_line(int conn_id, const char *json);

/* The heavy gate (command_handler.c): one owner of the RAM sample pool and the capture path.
   heavy_begin refuses with "busy" (sent) when a heavy op runs or another conn holds it. */
bool heavy_try_claim(int conn_id);
void heavy_release(int conn_id);
bool heavy_begin(int conn_id);
int  heavy_owner_conn(void);                  /* the conn holding the gate, or -1 */

/* An optional "sample_rate_hz" field; 0.0f (= auto) when absent or not positive. */
float parse_sample_rate_hz(const char *json);

/* Every LA-bank operation needs the LA voltage chosen first; false = the error was sent. */
bool require_la_voltage(int conn_id);

/* The command handlers in command_handler.c. Every handler has this one signature so the command
   table (cmd_tier.c) can reach it; the ones that take no arguments ignore `json`. */
void handle_ping(int conn_id, const char *json);
/* The wiring profiles stored on the pod (command_handler_wiring.c, wiring_store.h). */
void handle_wiring_list(int conn_id, const char *json);
void handle_wiring_get(int conn_id, const char *json);
void handle_wiring_put(int conn_id, const char *json);
void handle_wiring_delete(int conn_id, const char *json);
void handle_wiring_default(int conn_id, const char *json);

/* Network provisioning and the cloud speed test (command_handler_net.c). */
void   handle_cloud_set(int conn_id, const char *json);
void   handle_cloud_status(int conn_id, const char *json);
void   handle_cloud_clear(int conn_id, const char *json);
void   handle_wifi_set(int conn_id, const char *json);
void   handle_wifi_clear(int conn_id, const char *json);
void   handle_wifi_status(int conn_id, const char *json);
/* wifi_clear's second half, the ESP32-C3's NVS partition (esp_rom_flash_erase_nvs). Worker task.
   wifi_clear_c3_now: erase it now; NULL = erased and verified blank, else why not.
   wifi_clear_poll: run the erase a JSON wifi_clear scheduled, once its reply is out. */
const char *wifi_clear_c3_now(void);
void   wifi_clear_poll(void);
void   handle_wifi_scan(int conn_id, const char *json);
void   handle_eth(int conn_id, const char *json);
void   handle_speedtest(int conn_id, const char *json);
void   speedtest_pump(void);                  /* the "up" direction's paced byte source */
/* The PROTO_SPEEDTEST receive path: counts and drops the "down" bytes, acks, returns to JSON when
   all arrived. Returns the bytes taken from buf. */
size_t speedtest_receive(int conn_id, const uint8_t *buf, size_t len);
void   speedtest_conn_closed(int conn_id);

/* The pod itself: power, identity, the iCE40 (command_handler_device.c); status
   (command_handler_status.c). */
void handle_target_power(int conn_id, const char *json);
void handle_target_status(int conn_id, const char *json);
void handle_power_status(int conn_id, const char *json);
void handle_status(int conn_id, const char *json);
void handle_diag(int conn_id, const char *json);
#if defined(BENCHPOD_EMI_TEST)
void handle_emi_log(int conn_id, const char *json);   /* emi_test.c: the EMI build's run log */
#endif
void handle_identity_public(int conn_id, const char *json);
void handle_identity_pop(int conn_id, const char *json);
void handle_identity_wipe(int conn_id, const char *json);   /* command_handler_recovery.c */
void handle_usb_reattach(int conn_id, const char *json);
void handle_wifi_bridge(int conn_id, const char *json);
void handle_usb_cc(int conn_id, const char *json);
/* USB host mode (command_handler_usb.c). */
void handle_usb_host(int conn_id, const char *json);
/* Flash a board that shows up as a USB drive (command_handler_usb.c, USB_HOST=1 builds). */
void handle_usb_flash(int conn_id, const char *json);
void handle_usb_hid(int conn_id, const char *json);

/* Digital v3r3's analog module link (command_handler_module.c, module_link.h). */
void handle_module(int conn_id, const char *json);
void handle_module_power(int conn_id, const char *json);
/* The module object: {"present":..,"analog":..,"power":..,"bridge_power":..,"eeprom":..}, and
   with `id` the EEPROM's ID fields (the `module` reply; status takes the short form). */
void module_emit_state(bp_emit_t *e, bool id);
void handle_nrst(int conn_id, const char *json);
void handle_fpga_image(int conn_id, const char *json);
void handle_psram_recover(int conn_id, const char *json);
void handle_psram_ping(int conn_id, const char *json);
void device_poll(void);                       /* the deferred psram_recover reboot */

/* Analog front end (command_handler_analog.c). */
void handle_dac_mux(int conn_id, const char *json);
void handle_cal_switch(int conn_id, const char *json);
void handle_analog_path(int conn_id, const char *json);
void handle_dac_out(int conn_id, const char *json);
void handle_current_out(int conn_id, const char *json);
void handle_adc_read(int conn_id, const char *json);
void handle_calibrate(int conn_id, const char *json);
void handle_adc_range(int conn_id, const char *json);

/* Captures and the paced bulk read-back (command_handler_capture.c). */
void handle_capture(int conn_id, const char *json);
void handle_capture_dual(int conn_id, const char *json);
void handle_capture_read(int conn_id, const char *json);
void handle_test(int conn_id, const char *json);
void handle_measure(int conn_id, const char *json);
void handle_la_capture(int conn_id, const char *json);
void handle_sensor_regs(int conn_id, const char *json);
void handle_sensor_la(int conn_id, const char *json);   /* an I2C instance's two pins, packed */
bool capture_busy(void);                      /* an async capture or a bulk send is running */
void capture_poll(void);                      /* trigger timeouts, completions, bulk read-back */
void capture_on_gateware_reconfigured(void);
void capture_conn_closed(int conn_id);

/* DAC output, uploads, replay and the control loop (command_handler_dac.c). */
void   handle_generate(int conn_id, const char *json);
void   handle_load(int conn_id, const char *json);
void   handle_load_bin(int conn_id, const char *json);
void   handle_replay(int conn_id, const char *json);
void   handle_dac_stop(int conn_id, const char *json);
void   handle_dac_limits(int conn_id, const char *json);
void   handle_dac_set(int conn_id, const char *json);
void   handle_dac_control_loop(int conn_id, const char *json);
void   handle_dac_loop_input(int conn_id, const char *json);
void   handle_dac_loop_probe(int conn_id, const char *json);
/* The PROTO_LOAD receive path: stores the upload, replies and returns to JSON once all of it
   arrived. Returns the bytes taken from buf. */
size_t load_bin_receive(int conn_id, const uint8_t *buf, size_t len);
int    load_bin_owner(void);                  /* the conn streaming a load_bin upload, or -1 */
void   dac_trace_note_capture(size_t samples);/* a RAM capture is now the trace to replay */
void   dac_poll(void);                        /* upload gate hold + load_bin stall guard */
void   dac_on_gateware_reconfigured(void);    /* the loop's input mirrors */
void   dac_conn_closed(int conn_id);

/* On-pod CMSIS-DAP probe (command_handler_dap.c). */
void   handle_dap_start(int conn_id, const char *json);
/* The PROTO_DAP receive path: runs each whole frame, returns the bytes taken from buf. */
size_t dap_proxy_receive(int conn_id, const uint8_t *buf, size_t len);
void   dap_poll(void);                        /* frees the pins of an SWD session that timed out */
void   swd_disarm_and_release(void);          /* disarm SWD and give SWCLK/SWDIO back */
void   dap_conn_closed(int conn_id);

/* UART transparent proxy (command_handler_uart.c). */
void   handle_uart_proxy_start(int conn_id, const char *json);
bool   uart_frame_parse(const char *json, uint8_t *frame);   /* parity / stop_bits -> UART_FRAME_* */
/* uart_proxy_start's "entry" object: the boot-mode entry the pod runs before the ack. */
#define UART_ENTRY_MS_MAX   1000
#define UART_ENTRY_HEX_MAX  32
typedef struct {
    bool     on, tx_low;
    uint16_t reset_ms, hold_ms, idle_ms;
    uint8_t  nhex;
    uint8_t  hex[UART_ENTRY_HEX_MAX];
} uart_entry_t;
bool   uart_entry_parse(const char *json, uart_entry_t *en, const char **bad, int *max);
/* A uart_proxy_start request, parsed and checked (uart_cfg_parse). */
typedef struct {
    unsigned     rx, tx;      /* LA channels; rx == tx on one wire (half duplex) */
    unsigned     de;          /* RS-485 DE channel, 0 = none (gateware v58) */
    bool         de_inv;      /* DE active low */
    uint32_t     baud;
    uint8_t      frame;       /* UART_FRAME_* bits (0 = 8N1), UART_FRAME_HALFDUPLEX on one wire */
    uart_entry_t entry;       /* the boot-mode entry (entry.on false: none) */
} uart_cfg_t;
/* Why uart_cfg_parse refused a request (uart_cfg_refuse sends the reply). */
typedef enum {
    UART_CFG_OK = 0,
    UART_CFG_MISSING_RX, UART_CFG_MISSING_TX,
    UART_CFG_BAD_FRAME,                       /* parity / stop_bits */
    UART_CFG_GW_FRAME,                        /* anything but 8N1 on gateware < v56 */
    UART_CFG_GW_HALF_DUPLEX,                  /* half_duplex on gateware < v57 */
    UART_CFG_HALF_DUPLEX_TWO_PINS,            /* half_duplex with rx != tx */
    UART_CFG_SAME_PIN,                        /* rx == tx without half_duplex */
    UART_CFG_ENTRY_INVALID,                   /* an entry argument out of range (bad, max) */
    UART_CFG_ENTRY_NO_RESET_PIN,              /* entry on a pod without the reset pin */
    UART_CFG_GW_DE,                           /* de on gateware < v58 */
    UART_CFG_DE_PIN,                          /* de on rx or tx */
} uart_cfg_err_kind_t;
typedef struct { uart_cfg_err_kind_t kind; const char *bad; int max; } uart_cfg_err_t;
/* Parse and check {"cmd":"uart_proxy_start",...} against the gateware version `gw` and whether the
   pod has the reset pin.  Pure (host tests).  The checks run in the order the replies always came
   in, so a request with several problems gets the same refusal as before. */
bool   uart_cfg_parse(const char *json, const gw_caps_t *gw, bool has_reset_pin, uart_cfg_t *cfg, uart_cfg_err_t *err);
void   uart_cfg_refuse(int conn_id, const uart_cfg_t *cfg, const uart_cfg_err_t *err);
/* The LA pins one user of the soft UART holds (the proxy, esp_dut_flash), and its RX pull-up. */
typedef struct {
    uint8_t         rx, tx, de;   /* de 0 = none */
    bool            one_wire;     /* tx is claimed as uart_hd, rx is the same pin */
    bool            claimed;
    la_pull_token_t rx_hold;      /* la_pull_hold on rx, put back by uart_pins_release */
} uart_pins_t;
/* Check every pin first (a `pin conflict:` / `pull conflict:` message in err, nothing claimed),
   then claim them all into *p. */
bool   uart_pins_claim(uart_pins_t *p, unsigned rx, unsigned tx, unsigned de, bool one_wire, char *err, size_t cap);
/* Hold RX high (la_pull_hold; 2.2k on one wire, else by the pin's function).  Logs when a pull
   that should engage does not.  After uart_pins_claim. */
void   uart_pins_hold_rx(uart_pins_t *p);
/* Put RX's pull back and release exactly the pins in *p (only where they still hold the UART
   function).  Safe on a *p that holds nothing. */
void   uart_pins_release(uart_pins_t *p);
/* The PROTO_UART receive path: forwards buf to the DUT and watches for the +++ escape. */
size_t uart_proxy_receive(int conn_id, const uint8_t *buf, size_t len);
void   uart_proxy_poll(void);                 /* DUT -> client drain and the escape's guard */
void   uart_rearm_poll(void);                 /* re-arm a session kept across a reconfiguration */
void   uart_proxy_on_gateware_reconfigured(void);
void   uart_proxy_conn_closed(int conn_id);

/* LA pin ownership glue + the LA pin and I2C-sensor handlers (command_handler_pins.c). */
/* The pull masks, la_pull_mask_now / la_pull_mask_live: la_pull.h (through la_pins.h). */
void     la_pins_drive_mask(uint16_t mask);   /* GPIO_SET each pin in mask to its table state */
/* May `fn` claim las[0..n)?  Pins owned by a function in free_fns count as free.  false = the
   pin/pull conflict error was sent.  Commit with la_pins_claim once the FPGA side succeeded. */
bool     la_claim_or_error(int conn_id, la_fn_t fn, const uint8_t *las, size_t n, uint32_t free_fns);
/* Start an `la` step train with ownership.  0 ok; otherwise err holds the reply text and the
   return is -1 bad arguments, -2 a train is already running ("busy"), -3 pin conflict. */
int      la_step_begin(unsigned la, uint32_t steps, uint32_t delay_us, unsigned dir_la, int dir,
                       char *err, size_t cap);
void     la_step_poll(void);                  /* releases the train's pins once STEP_BUSY drops */
void     la_pins_on_gateware_reconfigured(void);
void     handle_la_pins(int conn_id, const char *json);
void     handle_gpio(int conn_id, const char *json);
void     handle_la_step_cmd(int conn_id, const char *json);
void     handle_la_pull_cmd(int conn_id, const char *json);
void     handle_la_voltage(int conn_id, const char *json);
/* Emulated sensors (command_handler_sensors.c; the JSON is in sensor_cmd.c). */
void     handle_sensor(int conn_id, const char *json);   /* every sensor_* verb but sensor_la */

/* SPI master on the LA pins (command_handler_spi.c). */
void handle_spi_start(int conn_id, const char *json);
void handle_spi_stop(int conn_id, const char *json);
void handle_spi_xfer(int conn_id, const char *json);
void handle_spi_flash(int conn_id, const char *json);
void handle_spi_stream(int conn_id, const char *json);
/* ESP32 DUT flashing over its UART ROM loader (command_handler_esp.c). */
void handle_esp_flash(int conn_id, const char *json);
/* The PSRAM bytes the last `load_bin` with "psram" staged: false when none. */
bool command_handler_psram_stage(uint32_t *base, uint32_t *len);
void handle_spi_status(int conn_id, const char *json);
void spi_session_end(void);
void spi_on_gateware_reconfigured(void);
/* command_handler_i2c.c: the I2C master (gateware v55) */
void handle_la_dir(int conn_id, const char *json);
/* Send and capture (command_handler_bus_send.c): la_capture's optional "send". */
#define BUS_SEND_MAX         32u    /* bytes written / read by a capture's send */
#define BUS_SEND_RESULT_MAX  128u   /* its result text: {"bus":"i2c","ack":true,"rx":"<44>"} */
bool   bus_send_prepare(int conn_id, const char *json);   /* false: the error was sent */
void   bus_send_run(void);
void   bus_send_cancel(void);
size_t bus_send_take(char *out, size_t cap);              /* ",\"send\":{...}" or 0 */
int    i2c_capture_prepare(const char *obj, char *err, size_t cap);
void   i2c_capture_run(char *out, size_t cap);
int    spi_capture_prepare(const char *obj, char *err, size_t cap);
void   spi_capture_run(char *out, size_t cap);
void handle_i2c_start(int conn_id, const char *json);
void handle_i2c_stop(int conn_id, const char *json);
void handle_i2c_status(int conn_id, const char *json);
void handle_i2c_xfer(int conn_id, const char *json);
void handle_i2c_scan(int conn_id, const char *json);
void i2c_session_end(void);
void i2c_on_gateware_reconfigured(void);

/* power_profile (command_handler_power.c). */
void handle_power_profile(int conn_id, const char *json);
void power_profile_service(void);            /* one-shot completion + paced result frames */
void power_profile_conn_closed(int conn_id);

/* CAN subsystem handlers (command_handler_can.c). */
void handle_can_start(int conn_id, const char *json);
void handle_can_write(int conn_id, const char *json);
void handle_can_read(int conn_id, const char *json);
void handle_can_status(int conn_id, const char *json);
void handle_can_respond(int conn_id, const char *json);
void handle_can_term(int conn_id, const char *json);
void handle_can_stop(int conn_id, const char *json);

/* The pod's own lease (command_handler_lease.c, lease_local.h). */
void handle_lease_acquire(int conn_id, const char *json);
void handle_lease_renew(int conn_id, const char *json);
void handle_lease_release(int conn_id, const char *json);
/* dispatch_line, every command: true = may run; false = refused (busy, or lease lost), reply sent. */
bool command_handler_lease_gate(int conn_id, const cmd_desc_t *d, const char *json);

/* OTA subsystem handlers (command_handler_ota.c). */
/* Policy commands (command_handler_policy.c). */
void handle_sig_policy(int conn_id, const char *json);
/* POLICY_SRC_USB / _CLOUD / _LAN for a connection id (pod_policy.h). */
#include "pod_policy.h"
policy_src_t command_handler_policy_src(int conn_id);
void handle_lan_policy(int conn_id, const char *json);
void handle_usb_network(int conn_id, const char *json);
void handle_log_level(int conn_id, const char *json);
void handle_lan_tls(int conn_id, const char *json);
void handle_lan_clients(int conn_id, const char *json);
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
/* Code that runs from the W25Q (command_handler_xip.c). */
void handle_xip(int conn_id, const char *json);

/* Append base64url of n bytes to an emitter (bounds-tracked like bp_emit: too little room sets
   e->ok false and appends nothing). */
static inline void bp_emit_b64url(bp_emit_t *e, const uint8_t *p, size_t n) {
    if (!e->ok || n == 0) return;
    size_t w = b64url_encode(p, n, e->buf + e->len, e->cap - e->len);
    if (w == 0) { e->ok = false; return; }
    e->len += w;
}

#endif /* COMMAND_HANDLER_INTERNAL_H */
