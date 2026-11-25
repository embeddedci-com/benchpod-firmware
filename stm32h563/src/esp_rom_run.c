/* esp_rom_run.c — see esp_rom_run.h. Internal flash. */
#include "esp_rom_run.h"

static esp_rom_work_t s_work;   /* one job at a time: the C3 flash, esp_flash and usb_flash exclude each other */

esp_rom_work_t *esp_rom_run_work(void) { return &s_work; }
int esp_rom_run_begin(esp_rom_job_t *j) { return esp_rom_job_begin(j); }
int esp_rom_run_step(esp_rom_job_t *j)  { return esp_rom_job_step(j); }
