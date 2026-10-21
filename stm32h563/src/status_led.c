/* status_led — the RGB status LED's colour from the pod's state (status_led.h). */
#include "status_led.h"

static bool     s_boot_seen;       /* boot_done observed: the boot blinks are timed from here */
static uint32_t s_boot_ms;
static bool     s_cmd_armed;
static uint32_t s_cmd_ms;
static bool     s_written;         /* a mask went out at least once */
static uint8_t  s_last;
static uint32_t s_last_ms;

void status_led_reset(void) {
    s_boot_seen = false;
    s_cmd_armed = false;
    s_written = false;
}

void status_led_command(uint32_t now_ms) {
    s_cmd_armed = true;
    s_cmd_ms = now_ms;
}

/* Wrap-safe "less than ms have passed since t". */
static bool within(uint32_t now_ms, uint32_t t, uint32_t ms) { return (uint32_t)(now_ms - t) < ms; }

uint8_t status_led_color(const status_led_in_t *in, uint32_t now_ms) {
    if (!in->boot_done) return STATUS_LED_OFF;
    if (!s_boot_seen) { s_boot_seen = true; s_boot_ms = now_ms; }
    if (!in->hw_ok) return STATUS_LED_RED;
    if (in->power_fault)
        return ((now_ms / STATUS_LED_FAULT_BLINK_MS) & 1u) ? STATUS_LED_OFF : STATUS_LED_RED;

    const uint32_t since_boot = now_ms - s_boot_ms;
    if (since_boot < STATUS_LED_BOOT_BLINKS * 2u * STATUS_LED_BOOT_BLINK_MS)
        return ((since_boot / STATUS_LED_BOOT_BLINK_MS) & 1u) ? STATUS_LED_OFF : STATUS_LED_GREEN;

    if (in->updating) return STATUS_LED_MAGENTA;
    if (in->busy) return STATUS_LED_BLUE;
    if (s_cmd_armed) {
        if (within(now_ms, s_cmd_ms, STATUS_LED_CMD_FLASH_MS)) return STATUS_LED_CYAN;
        s_cmd_armed = false;
    }
    return in->power_on ? STATUS_LED_GREEN : STATUS_LED_OFF;
}

void status_led_reset_write(void) { s_written = false; }

int status_led_next_write(uint8_t color, uint32_t now_ms) {
    if (s_written && color == s_last && within(now_ms, s_last_ms, STATUS_LED_REFRESH_MS)) return -1;
    s_written = true;
    s_last = color;
    s_last_ms = now_ms;
    return color;
}
