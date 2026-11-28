/* dut_supply.c — see dut_supply.h.  Digital v4 only (board_caps()->supply_3v3_1v8). */
#include "dut_supply.h"
#include "board_caps.h"
#include "board_pins.h"
#include "stm32h5xx_hal.h"
#include "bp_log.h"

static bool s_present;
static bool s_on[2];
static struct { bool armed, on; uint32_t at; } s_pend[2];

static void drive(dut_supply_t r, bool on)
{
    GPIO_TypeDef *port = (r == DUT_SUPPLY_3V3) ? DUT_3V3_EN_PORT : DUT_1V8_EN_PORT;
    uint32_t pin = (r == DUT_SUPPLY_3V3) ? DUT_3V3_EN_PIN : DUT_1V8_EN_PIN;
    HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    s_on[r] = on;
}

void dut_supply_init(void)
{
    if (!board_caps()->supply_3v3_1v8) return;
    DUT_EN_CLK_EN();
    /* Level first (the pull-downs hold the rails off until here), then the output mode. */
    drive(DUT_SUPPLY_3V3, true);
    drive(DUT_SUPPLY_1V8, true);
    GPIO_InitTypeDef gp = {0};
    gp.Mode  = GPIO_MODE_OUTPUT_PP;
    gp.Pull  = GPIO_NOPULL;
    gp.Speed = GPIO_SPEED_FREQ_LOW;
    if (DUT_3V3_EN_PORT == DUT_1V8_EN_PORT) {
        gp.Pin = DUT_3V3_EN_PIN | DUT_1V8_EN_PIN;
        HAL_GPIO_Init(DUT_3V3_EN_PORT, &gp);
    } else {   /* two ports: DUT_EN_CLK_EN() must then enable both clocks */
        gp.Pin = DUT_3V3_EN_PIN;
        HAL_GPIO_Init(DUT_3V3_EN_PORT, &gp);
        gp.Pin = DUT_1V8_EN_PIN;
        HAL_GPIO_Init(DUT_1V8_EN_PORT, &gp);
    }
    s_present = true;
    log_printf("[dut] J1 pin 17 (3.3 V) and pin 18 (1.8 V) on\n");
}

bool dut_supply_present(void) { return s_present; }

int dut_supply_set(dut_supply_t r, bool on)
{
    if (!s_present || (r != DUT_SUPPLY_3V3 && r != DUT_SUPPLY_1V8)) return -1;
    s_pend[r].armed = false;
    drive(r, on);
    return 0;
}

int dut_supply_schedule(dut_supply_t r, bool on, uint32_t delay_ms)
{
    if (!s_present || (r != DUT_SUPPLY_3V3 && r != DUT_SUPPLY_1V8)) return -1;
    if (delay_ms == 0) {
        s_pend[r].armed = false;
        drive(r, on);
        return 0;
    }
    s_pend[r].on = on;
    s_pend[r].at = HAL_GetTick() + delay_ms;
    s_pend[r].armed = true;
    return 0;
}

void dut_supply_poll(void)
{
    if (!s_present) return;
    for (int i = 0; i < 2; i++) {
        if (s_pend[i].armed && (int32_t)(HAL_GetTick() - s_pend[i].at) >= 0) {
            s_pend[i].armed = false;
            drive((dut_supply_t)i, s_pend[i].on);
        }
    }
}

bool dut_supply_get(dut_supply_t r)
{
    return s_present && (r == DUT_SUPPLY_3V3 || r == DUT_SUPPLY_1V8) && s_on[r];
}
