#ifndef ESP_ROM_RUN_H
#define ESP_ROM_RUN_H

/* esp_rom_run — how the firmware runs an esp_rom_job_t (esp_rom_job.h). Internal flash.
 *
 * The ROM-loader state machine is code of the usb image in the W25Q (xip_mod_esp_rom.c): a job runs
 * inside an XIP session (xip_session.h) that the caller opens before it touches any pin and closes
 * when the job ends, and every call into the machine goes through the image's table. */

#include "esp_rom_job.h"

/* The job's buffers. */
esp_rom_work_t *esp_rom_run_work(void);

/* esp_rom_job_begin and esp_rom_job_step for a prepared job (esp_rom_job_prepare). */
int esp_rom_run_begin(esp_rom_job_t *j);
int esp_rom_run_step(esp_rom_job_t *j);

#endif /* ESP_ROM_RUN_H */
