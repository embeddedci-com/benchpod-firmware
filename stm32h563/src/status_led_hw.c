/* status_led_hw — feeds status_led the pod's state and writes SET_LED (status_led.h).
   Everything here runs on the hw worker, the only task that talks to the iCE40. */
#include "status_led.h"

#include "boot_guard.h"
#include "command_handler_internal.h"   /* heavy_or_claimed */
#include "ice40_flash.h"
#include "ota.h"
#include "signal_engine.h"
#include "target_power.h"
#include "stm32h5xx_hal.h"              /* HAL_GetTick */

#define SAMPLE_MS 20u   /* the CDONE read reconfigures a GPIO: not on every 1 ms worker pass */

static bool     s_boot_done;
static bool     s_sampled;
static uint32_t s_sample_ms;
static bool     s_override;
static uint8_t  s_override_mask;
static uint32_t s_override_until;

void status_led_boot_finished(void) { s_boot_done = true; }

void status_led_note_command(bool drives) {
    if (drives) status_led_command(HAL_GetTick());
}

void status_led_override(uint8_t mask, uint32_t hold_ms) {
    s_override = true;
    s_override_mask = mask & 0x07u;
    s_override_until = HAL_GetTick() + hold_ms;
    fpga_set_led(s_override_mask);
}

void status_led_service(void) {
    const uint32_t now = HAL_GetTick();
    if (s_override) {
        if ((int32_t)(now - s_override_until) < 0) return;
        s_override = false;
        s_sampled = false;   /* resend the status at once */
        status_led_reset_write();
    }
    if (s_sampled && (uint32_t)(now - s_sample_ms) < SAMPLE_MS) return;
    s_sampled = true;
    s_sample_ms = now;

    bool on1 = false, on2 = false, flt1 = false, flt2 = false;
    (void)target_power_get_cached(1, &on1, &flt1);
    (void)target_power_get_cached(2, &on2, &flt2);
    const ota_state_t o = ota_get_state();
    status_led_in_t in = {
        .boot_done   = s_boot_done,
        .hw_ok       = !boot_guard_skip_hw() && ice40_is_configured() && signal_engine_psram_operable(),
        .power_fault = flt1 || flt2,
        .power_on    = on1 || on2,
        .updating    = o == OTA_RECEIVING || o == OTA_VERIFIED,
        .busy        = heavy_or_claimed(),
    };
    const int w = status_led_next_write(status_led_color(&in, now), now);
    if (w >= 0) fpga_set_led((uint8_t)w);
}
