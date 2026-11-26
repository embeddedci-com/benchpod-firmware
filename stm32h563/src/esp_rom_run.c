/* esp_rom_run.c — see esp_rom_run.h. Internal flash. */
#include "esp_rom_run.h"
#include "xip_session.h"
#include "xip_image.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

int esp_rom_run_open(esp_rom_work_t **w)
{
    *w = NULL;
    xip_err_t xe = xip_session_begin(XIP_IMG_USB);
    if (xe != XIP_OK) return (int)xe;
    esp_rom_work_t *p = pvPortMalloc(sizeof(*p));
    if (!p) { xip_session_end(); return ESP_RUN_E_MEM; }
    memset(p, 0, sizeof(*p));
    *w = p;
    return 0;
}

void esp_rom_run_close(esp_rom_work_t **w)
{
    if (!*w) return;
    xip_session_end();
    vPortFree(*w);
    *w = NULL;
}

int esp_rom_run_ready(void) { return (int)xip_image_check_slot(XIP_IMG_USB); }

bool esp_rom_run_transient(int e)
{
    return e == XIP_ERR_ACTIVE || e == XIP_ERR_POOL_BUSY || e == ESP_RUN_E_MEM;
}

void esp_rom_run_why(int e, char *buf, size_t cap)
{
    if (e == XIP_ERR_ACTIVE)
        snprintf(buf, cap, "busy: a flash over USB, a chip flash (prog_flash) or an ESP32 flash is running; try again when it ends");
    else if (e == XIP_ERR_POOL_BUSY)
        snprintf(buf, cap, "%s", xip_err_text(XIP_ERR_POOL_BUSY));
    else if (e == ESP_RUN_E_MEM)
        snprintf(buf, cap, "busy: out of memory for the ESP32 flash buffers; try again");
    else
        snprintf(buf, cap, "ESP32 flashing runs from the xip image: %s", xip_img_err_text(XIP_IMG_USB, (xip_err_t)e));
}

static int not_in_session(esp_rom_job_t *j)
{
    j->failed_phase = j->phase;
    j->result = ESP_ROM_ERR_ARGS;
    j->phase = ESP_ROM_DONE;
    return 0;
}

int esp_rom_run_begin(esp_rom_job_t *j)
{
    if (!xip_session_active() || xip_session_image() != XIP_IMG_USB) return not_in_session(j);
    return xip_session_call_ctx(XIP_FN_ESP_ROM_BEGIN, j) == 1;
}

int esp_rom_run_step(esp_rom_job_t *j)
{
    if (j->phase == ESP_ROM_IDLE || j->phase == ESP_ROM_DONE) return 0;
    if (!xip_session_active() || xip_session_image() != XIP_IMG_USB) return not_in_session(j);
    return xip_session_call_ctx(XIP_FN_ESP_ROM_STEP, j) == 1;
}
