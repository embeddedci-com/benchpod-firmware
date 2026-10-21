/*
 * test_status_led.c — host unit test for the status LED's colour logic (status_led.h): boot
 * blinks, failure red, fault blink, update/busy/command colours and their order, and that only
 * changes (plus a slow refresh) go out over SPI.
 */
#include "status_led.h"

#include <stdio.h>

static int failures;
#define CHECK(c) do { if(!(c)){printf("FAIL %s:%d  %s\n",__FILE__,__LINE__,#c);failures++;} } while(0)

static const status_led_in_t BOOTED = { .boot_done = true, .hw_ok = true };

/* Booting is dark; a good boot blinks green three times, then goes dark (idle). */
static void test_boot_ok_blinks_green(void)
{
    status_led_reset();
    status_led_in_t booting = { 0 };
    CHECK(status_led_color(&booting, 0) == STATUS_LED_OFF);
    CHECK(status_led_color(&booting, 5000) == STATUS_LED_OFF);

    const uint32_t t0 = 10000;
    int greens = 0;
    uint8_t prev = STATUS_LED_OFF;
    for (uint32_t t = t0; t < t0 + 3000; t += 10) {
        uint8_t c = status_led_color(&BOOTED, t);
        CHECK(c == STATUS_LED_GREEN || c == STATUS_LED_OFF);
        if (c == STATUS_LED_GREEN && prev != STATUS_LED_GREEN) greens++;
        prev = c;
    }
    CHECK(greens == (int)STATUS_LED_BOOT_BLINKS);
    CHECK(status_led_color(&BOOTED, t0) == STATUS_LED_GREEN);
    CHECK(status_led_color(&BOOTED, t0 + 3000) == STATUS_LED_OFF);
}

/* A failed bring-up (or a later wedge) is solid red, and wins over everything else. */
static void test_failure_is_solid_red(void)
{
    status_led_reset();
    status_led_in_t bad = { .boot_done = true, .hw_ok = false, .power_on = true, .busy = true };
    for (uint32_t t = 0; t < 3000; t += 50) CHECK(status_led_color(&bad, t) == STATUS_LED_RED);

    status_led_reset();
    (void)status_led_color(&BOOTED, 0);
    bad.busy = false;
    CHECK(status_led_color(&bad, 60000) == STATUS_LED_RED);   /* runtime wedge */
}

/* A target-power fault blinks red, even during the boot blinks. */
static void test_power_fault_blinks_red(void)
{
    status_led_reset();
    status_led_in_t f = BOOTED;
    f.power_fault = true;
    f.power_on = true;
    CHECK(status_led_color(&f, 0) == STATUS_LED_RED);
    CHECK(status_led_color(&f, STATUS_LED_FAULT_BLINK_MS) == STATUS_LED_OFF);
    CHECK(status_led_color(&f, 2 * STATUS_LED_FAULT_BLINK_MS) == STATUS_LED_RED);
}

/* After the boot blinks: update > busy > command flash > power on > off. */
static void test_activity_colours_and_order(void)
{
    status_led_reset();
    (void)status_led_color(&BOOTED, 0);
    const uint32_t t = 5000;   /* boot blinks long over */

    status_led_in_t in = BOOTED;
    CHECK(status_led_color(&in, t) == STATUS_LED_OFF);
    in.power_on = true;
    CHECK(status_led_color(&in, t) == STATUS_LED_GREEN);

    status_led_command(t);
    CHECK(status_led_color(&in, t) == STATUS_LED_CYAN);
    CHECK(status_led_color(&in, t + STATUS_LED_CMD_FLASH_MS - 1) == STATUS_LED_CYAN);
    CHECK(status_led_color(&in, t + STATUS_LED_CMD_FLASH_MS) == STATUS_LED_GREEN);

    status_led_command(t + 1000);
    in.busy = true;
    CHECK(status_led_color(&in, t + 1000) == STATUS_LED_BLUE);
    in.updating = true;
    CHECK(status_led_color(&in, t + 1000) == STATUS_LED_MAGENTA);
    in.updating = in.busy = false;
    CHECK(status_led_color(&in, t + 1001) == STATUS_LED_CYAN);

    /* The command flash waits behind the boot blinks rather than cutting them short. */
    status_led_reset();
    (void)status_led_color(&BOOTED, 0);
    status_led_command(50);
    CHECK(status_led_color(&BOOTED, 50) == STATUS_LED_GREEN);
}

/* The flash survives a tick counter wrap. */
static void test_command_flash_across_wrap(void)
{
    const uint32_t t = 0xFFFFFFF0u;
    status_led_reset();
    (void)status_led_color(&BOOTED, t - 5000u);   /* booted well before the wrap */
    status_led_command(t);
    CHECK(status_led_color(&BOOTED, t + 20u) == STATUS_LED_CYAN);
    CHECK(status_led_color(&BOOTED, t + STATUS_LED_CMD_FLASH_MS) == STATUS_LED_OFF);
}

/* Only a change goes out, plus a refresh every STATUS_LED_REFRESH_MS (gateware loads reset it). */
static void test_writes_only_on_change_or_refresh(void)
{
    status_led_reset();
    CHECK(status_led_next_write(STATUS_LED_OFF, 0) == STATUS_LED_OFF);   /* first one always */
    CHECK(status_led_next_write(STATUS_LED_OFF, 10) == -1);
    CHECK(status_led_next_write(STATUS_LED_GREEN, 20) == STATUS_LED_GREEN);
    CHECK(status_led_next_write(STATUS_LED_GREEN, 20 + STATUS_LED_REFRESH_MS - 1) == -1);
    CHECK(status_led_next_write(STATUS_LED_GREEN, 20 + STATUS_LED_REFRESH_MS) == STATUS_LED_GREEN);
    status_led_reset_write();
    CHECK(status_led_next_write(STATUS_LED_GREEN, 21 + STATUS_LED_REFRESH_MS) == STATUS_LED_GREEN);
}

int main(void)
{
    test_boot_ok_blinks_green();
    test_failure_is_solid_red();
    test_power_fault_blinks_red();
    test_activity_colours_and_order();
    test_command_flash_across_wrap();
    test_writes_only_on_change_or_refresh();
    if (failures) { printf("test_status_led: %d FAILED\n", failures); return 1; }
    printf("test_status_led: all passed\n");
    return 0;
}
