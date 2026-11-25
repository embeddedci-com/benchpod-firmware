#ifndef ESP_ROM_RUN_H
#define ESP_ROM_RUN_H

/* esp_rom_run — how the firmware runs an esp_rom_job_t (esp_rom_job.h). Internal flash.
 *
 * The ROM-loader state machine is code of the usb image in the W25Q (xip_mod_esp_rom.c), so a job
 * runs inside an XIP session (xip_session.h): the caller opens it before it touches any pin or
 * strap (esp_rom_run_open) and closes it when the job ends (esp_rom_run_close), on every path.
 * Everything that is not the protocol (the UART, the straps, the image source, the reply to the
 * client) stays in the callers. Runs on the hw worker only, like every session.
 *
 *   esp_rom_work_t *w;
 *   int e = esp_rom_run_open(&w);                       // 0, or why not (esp_rom_run_why)
 *   esp_rom_job_prepare(&job, w, &io, len, offset, &opts, false);
 *   esp_rom_run_begin(&job);
 *   while (esp_rom_run_step(&job)) ...                  // one step per hw worker loop
 *   esp_rom_run_close(&w);
 */

#include <stddef.h>
#include <stdbool.h>
#include "esp_rom_job.h"

/* esp_rom_run_open's failures are xip_session.h's xip_err_t, and this one: no memory for the buffers. */
#define ESP_RUN_E_MEM   100

/* Open a session of the usb image and allocate the job's buffers. 0 = open, *w set; else the
   reason (an xip_err_t or ESP_RUN_E_MEM) and nothing is open. */
int  esp_rom_run_open(esp_rom_work_t **w);
/* End the session and free the buffers (a NULL *w: nothing to do). */
void esp_rom_run_close(esp_rom_work_t **w);

/* The slot of the image holds this firmware's code (xip_image_check_slot): 0, or the reason (an
   xip_err_t) that esp_rom_run_open would give, without opening anything. For refusing early. */
int  esp_rom_run_ready(void);

/* Another session or a capture holds what the image needs right now: trying again shortly can work. */
bool esp_rom_run_transient(int e);
/* The reason in a sentence ("busy: ..." when transient; the xip slot text, with the install-blobs fix, otherwise). */
void esp_rom_run_why(int e, char *buf, size_t cap);

/* esp_rom_job_begin and esp_rom_job_step through the session's table: 1 = still running. A job
   that is stepped outside a session ends with ESP_ROM_ERR_ARGS instead of running. */
int esp_rom_run_begin(esp_rom_job_t *j);
int esp_rom_run_step(esp_rom_job_t *j);

#endif /* ESP_ROM_RUN_H */
