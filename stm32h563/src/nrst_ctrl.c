/* nrst_ctrl.c — see nrst_ctrl.h.  Open-drain PF4, pulled low or left Hi-Z. */
#include "nrst_ctrl.h"
#include "board_rev.h"
#include "stm32h5xx_hal.h"
#include "board_pins.h"
#include "pico_compat.h"   /* sleep_ms — yields to FreeRTOS once it is running */

#define NRST_PULSE_MIN_MS   1u
#define NRST_PULSE_MAX_MS   1000u

static bool s_ready;
static bool s_asserted;

void nrst_ctrl_init(void)
{
    if (!board_rev_is_v3()) return;

    NRST_CTRL_CLK_EN();

    /* Set the output latch to 1 (open-drain released = Hi-Z) BEFORE switching
       the pin out of its analog reset state, so the line is never momentarily
       pulled low by a stale latch value. */
    HAL_GPIO_WritePin(NRST_CTRL_PORT, NRST_CTRL_PIN, GPIO_PIN_SET);

    GPIO_InitTypeDef gp = {0};
    gp.Mode  = GPIO_MODE_OUTPUT_OD;   /* never sources current — see the header */
    gp.Pull  = GPIO_NOPULL;           /* R7 on the board is the only pull-up    */
    gp.Speed = GPIO_SPEED_FREQ_LOW;
    gp.Pin   = NRST_CTRL_PIN;
    HAL_GPIO_Init(NRST_CTRL_PORT, &gp);

    s_ready    = true;
    s_asserted = false;
}

bool nrst_ctrl_supported(void) { return s_ready; }

void nrst_ctrl_assert(bool asserted)
{
    if (!s_ready) return;
    HAL_GPIO_WritePin(NRST_CTRL_PORT, NRST_CTRL_PIN,
                      asserted ? GPIO_PIN_RESET : GPIO_PIN_SET);
    s_asserted = asserted;
}

bool nrst_ctrl_is_asserted(void) { return s_asserted; }

void nrst_ctrl_pulse(uint32_t hold_ms)
{
    if (!s_ready) return;
    if (hold_ms < NRST_PULSE_MIN_MS) hold_ms = NRST_PULSE_MIN_MS;
    if (hold_ms > NRST_PULSE_MAX_MS) hold_ms = NRST_PULSE_MAX_MS;
    nrst_ctrl_assert(true);
    /* sleep_ms, not HAL_Delay: a 1 s busy-wait here would block the hw worker
       (and everything queued behind it) for the whole hold. */
    sleep_ms(hold_ms);
    nrst_ctrl_assert(false);
}
