// Board support for hardware rev 1 and rev 2: RMII clock enable, PHY reset line, FACTORY and PAIR buttons,
// and the STATUS test pad (TP8). The board has no LEDs; Ethernet link/activity are the RJ45's.
#include "bridge.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "board";

void board_init(void)
{
    // Keep the PHY in reset and the oscillator off until eth_start (R14/R16 pull both low).
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_OSC_OE) | (1ULL << PIN_PHY_NRST) | (1ULL << PIN_STATUS),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_set_level(PIN_OSC_OE, 0);
    gpio_set_level(PIN_PHY_NRST, 0);
    gpio_config(&out);
    gpio_set_level(PIN_STATUS, 0);

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

void board_reboot(void)
{
    // GPIO0 carries the 50 MHz clock and is the download-mode strap: stop the clock and hold the
    // PHY in reset so a reset that resamples the straps sees GPIO0 high (R11).
    gpio_set_level(PIN_PHY_NRST, 0);
    gpio_set_level(PIN_OSC_OE, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_restart();
}

// Square wave helper: high for `on_ms` of every `period_ms`.
static int wave(int64_t now_ms, int period_ms, int on_ms)
{
    return (now_ms % period_ms) < on_ms;
}

// Level on the STATUS test pad (TP8), for a scope or logic analyser. Highest priority first.
static int status_level(int64_t now, bool button_feedback)
{
    if (button_feedback || !control_paired()) {
        return wave(now, 100, 50);  // 10 Hz: button acted / held, or waiting for a pod to pair
    }
    if (ota_active()) {
        return wave(now, 500, 250);  // 2 Hz: OTA transfer in progress
    }
    switch (wifi_state()) {
    case BWP_ST_CONNECTED: return 1;
    case BWP_ST_JOINING: return wave(now, 1000, 500);       // 1 Hz, 50 %
    case BWP_ST_AUTH_FAILED:
    case BWP_ST_NO_AP: return wave(now, 1000, 100);         // 1 Hz, 10 %
    case BWP_ST_WAIT_POD: return wave(now, 2000, 100);      // 100 ms every 2 s
    default: return 0;                                      // no Wi-Fi credentials
    }
}

static void button_task(void *arg)
{
    int64_t factory_since = 0, pair_since = 0, feedback_until = 0;
    bool factory_done = false, pair_done = false;
    for (;;) {
        int64_t now = esp_timer_get_time() / 1000;

        // FACTORY button held while running: clear the Wi-Fi credentials (the bootloader handles
        // the power-up gesture).
        bool factory = gpio_get_level(PIN_BTN_FACTORY) == 0;
        if (factory && !factory_since) {
            factory_since = now;
            factory_done = false;
        } else if (!factory) {
            factory_since = 0;
        }
        if (factory_since && !factory_done && now - factory_since >= CONFIG_BRIDGE_FACTORY_RESET_HOLD_MS) {
            ESP_LOGW(TAG, "FACTORY button held: clearing Wi-Fi credentials");
            bridge_clear_wifi();
            factory_done = true;
            feedback_until = now + 2000;
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
            feedback_until = now + 2000;
        }

        bool held = factory_since && !factory_done && now - factory_since >= 1000;
        gpio_set_level(PIN_STATUS, status_level(now, held || now < feedback_until));
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

void board_start_tasks(void)
{
    xTaskCreate(button_task, "buttons", 3072, NULL, 2, NULL);
}
