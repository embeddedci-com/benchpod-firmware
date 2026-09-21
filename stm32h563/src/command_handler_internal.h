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

/* Every LA-bank operation needs the LA voltage chosen first; false = the error was sent. */
bool require_la_voltage(int conn_id);

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
void     handle_la_pins(int conn_id);
void     handle_gpio(int conn_id, const char *json);

/* power_profile (command_handler_power.c). */
void handle_power_profile(int conn_id, const char *json);
void power_profile_service(void);            /* one-shot completion + paced result frames */
void power_profile_conn_closed(int conn_id);

/* CAN subsystem handlers (command_handler_can.c). */
void handle_can_config(int conn_id, const char *json);
void handle_can_write(int conn_id, const char *json);
void handle_can_read(int conn_id, const char *json);
void handle_can_status(int conn_id);
void handle_can_respond(int conn_id, const char *json);
void handle_can_term(int conn_id, const char *json);
void handle_can_disable(int conn_id);

/* OTA subsystem handlers (command_handler_ota.c). */
void handle_ota_begin(int conn_id, const char *json);
void handle_ota_data(int conn_id, const char *json);
void handle_ota_end(int conn_id);
void handle_ota_status(int conn_id);
void handle_ota_abort(int conn_id);
void handle_ota_selftest(int conn_id);
void handle_ota_commit(int conn_id);

#endif /* COMMAND_HANDLER_INTERNAL_H */
