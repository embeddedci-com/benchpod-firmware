// Board support for hardware rev A: RMII clock enable, PHY reset line, LEDs, FACTORY and PAIR buttons.
#include "bridge.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "board";
static volatile int64_t s_activity_until;

void board_init(void)
{
    // Keep the PHY in reset and the oscillator off until eth_start (R14/R16 pull both low).
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_OSC_OE) | (1ULL << PIN_PHY_NRST) | (1ULL << PIN_LED_WIFI) |
                        (1ULL << PIN_LED_ACT) | (1ULL << PIN_LED_STAT),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_set_level(PIN_OSC_OE, 0);
    gpio_set_level(PIN_PHY_NRST, 0);
    gpio_config(&out);
    gpio_set_level(PIN_LED_WIFI, 0);
    gpio_set_level(PIN_LED_ACT, 0);
    gpio_set_level(PIN_LED_STAT, 1);  // red until Ethernet is up

    gpio_config_t in = {.pin_bit_mask = (1ULL << PIN_BTN_FACTORY) | (1ULL << PIN_BTN_PAIR),
                        .mode = GPIO_MODE_INPUT};
    gpio_config(&in);  // GPIO34/35 have no internal pulls; R20 and R41 pull them up
}

void board_rmii_clock(bool on)
{
#if CONFIG_BRIDGE_RMII_CLK_EXT_OSC
    gpio_set_level(PIN_OSC_OE, on);
    if (on) {
        vTaskDelay(pdMS_TO_TICKS(10));  // oscillator start-up (datasheet: < 10 ms)
    }
#else
    (void)on;  // GPIO17 option: the EMAC driver starts the APLL clock itself
#endif
}

void board_activity(void)
{
    s_activity_until = esp_timer_get_time() + 30000;
}

void board_reboot(void)
{
    // GPIO0 carries the 50 MHz clock and is the download-mode strap: stop the clock and hold the
    // PHY in reset so a reset that resamples the straps sees GPIO0 high (R11).
    gpio_set_level(PIN_PHY_NRST, 0);
    gpio_set_level(PIN_OSC_OE, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_restart();
}

// Blink pattern helper: on for `on_ms` every `period_ms`.
static int blink(int64_t now_ms, int period_ms, int on_ms)
{
    return (now_ms % period_ms) < on_ms;
}

static void led_task(void *arg)
{
    int64_t pressed_since = 0;
    bool reset_done = false;
    int64_t reset_flash_until = 0;
    int64_t pair_since = 0, pair_flash_until = 0;
    bool pair_done = false;
    for (;;) {
        int64_t now_us = esp_timer_get_time();
        int64_t now = now_us / 1000;

        // Factory-reset button: hold to clear the Wi-Fi credentials.
        bool pressed = gpio_get_level(PIN_BTN_FACTORY) == 0;
        if (pressed && !pressed_since) {
            pressed_since = now;
            reset_done = false;
        } else if (!pressed) {
            pressed_since = 0;
        }
        if (pressed_since && !reset_done && now - pressed_since >= CONFIG_BRIDGE_FACTORY_RESET_HOLD_MS) {
            ESP_LOGW(TAG, "factory-reset button held: clearing Wi-Fi credentials");
            bridge_clear_wifi();
            reset_done = true;
            reset_flash_until = now + 1000;
        }

        // PAIR button: a press (50 ms, debounced; C27 filters bounce too) forgets the pairing.
        // Separate from FACTORY so pairing can never wipe settings.
        bool pair = gpio_get_level(PIN_BTN_PAIR) == 0;
        if (pair && !pair_since) {
            pair_since = now;
            pair_done = false;
        } else if (!pair) {
            pair_since = 0;
        }
        if (pair_since && !pair_done && now - pair_since >= 50) {
            ESP_LOGW(TAG, "PAIR button: forgetting the paired pod");
            control_forget_pairing();
            pair_done = true;
            pair_flash_until = now + 900;
        }

        // Green: Wi-Fi state.
        int wifi;
        switch (wifi_state()) {
        case BWP_ST_CONNECTED: wifi = 1; break;
        case BWP_ST_JOINING: wifi = blink(now, 1000, 500); break;
        case BWP_ST_WAIT_POD: wifi = blink(now, 2000, 100); break;
        case BWP_ST_AUTH_FAILED:
        case BWP_ST_NO_AP: wifi = blink(now, 250, 125); break;
        default: wifi = 0; break;
        }
        gpio_set_level(PIN_LED_WIFI, wifi);

        // Yellow: forwarded traffic.
        gpio_set_level(PIN_LED_ACT, now_us < s_activity_until);

        // Red: Ethernet link down (solid), OTA (slow blink), button held (fast blink), reset done.
        int stat;
        if (now < reset_flash_until) {
            stat = 1;
        } else if (now < pair_flash_until) {
            stat = blink(now, 300, 150);  // three blinks: pairing forgotten
        } else if (pressed_since && now - pressed_since >= 1000 && !reset_done) {
            stat = blink(now, 100, 50);
        } else if (ota_active()) {
            stat = blink(now, 500, 250);
        } else {
            stat = !eth_link_up();
        }
        gpio_set_level(PIN_LED_STAT, stat);

        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

void board_start_tasks(void)
{
    xTaskCreate(led_task, "leds", 3072, NULL, 2, NULL);
}
