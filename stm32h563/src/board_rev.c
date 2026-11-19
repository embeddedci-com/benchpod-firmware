/* board_rev.c — see board_rev.h.  PA3 strap: digital NC test, then ADC bucket, then (v3r4 band
   only) the PC3 strap. */
#include "board_rev.h"
#include "mcu_adc.h"
#include "stm32h5xx_hal.h"
#include "board_pins.h"
#include "bp_log.h"

#include <stdio.h>

/* A pulled-down NC pad settles well under this; the rev3 divider loaded by the
   internal pull-down still sits near 2.9 V, so the margin either side is >1 V. */
#define STRAP_NC_LEVEL_IS_LOW   GPIO_PIN_RESET

/* The bands are in board_rev.h (board_rev_decide): v3 2850..3400 mV, v3r2 2450..2849 mV, v3r4
   3130..3400 mV with the ratio and the second strap. */

static int s_rev      = BOARD_REV_UNKNOWN;
static int s_strap_mv = -1;
static int s_rev2_mv  = -1;   /* PC3, read only when PA3 says v3r4 */

/* The second strap (digital v3r4+).  Only called when PA3 already reads the v3r4 band, so PC3
   is never touched on an older board.  PC3 is in analog mode from reset; set it again in case. */
static int read_rev2_mv(void)
{
    BOARD_REV2_CLK_EN();
    GPIO_InitTypeDef gp = {0};
    gp.Mode = GPIO_MODE_ANALOG;
    gp.Pull = GPIO_NOPULL;
    gp.Pin  = BOARD_REV2_PIN;
    HAL_GPIO_Init(BOARD_REV2_PORT, &gp);
    int mv = -1;
    if (mcu_adc_read_mv_avg(BOARD_REV2_ADC_CH, 8, &mv) != 0) return -1;
    return mv;
}

void board_rev_init(void)
{
    BOARD_REV_CLK_EN();

    /* Step 1 — digital NC test.  Drive the internal pull-down and see whether
       the pin follows it.  mcu_adc_init() left PA3 in analog mode (which
       disconnects the pull resistors), so flip it back to a digital input for
       the duration of the test. */
    GPIO_InitTypeDef gp = {0};
    gp.Mode = GPIO_MODE_INPUT;
    gp.Pull = GPIO_PULLDOWN;
    gp.Pin  = BOARD_REV_PIN;
    HAL_GPIO_Init(BOARD_REV_PORT, &gp);
    HAL_Delay(2);                       /* 10 n on nothing, but let it settle */
    GPIO_PinState lvl = HAL_GPIO_ReadPin(BOARD_REV_PORT, BOARD_REV_PIN);

    /* Back to analog for the measurement (and to stop the pull-down loading the
       divider for the rest of the pod's life). */
    gp.Mode = GPIO_MODE_ANALOG;
    gp.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BOARD_REV_PORT, &gp);

    if (lvl == STRAP_NC_LEVEL_IS_LOW) {
        s_rev = BOARD_REV_V2;
        log_printf("[board] revision v2 (PA3 strap not fitted)\r\n");
        return;
    }

    /* Step 2 — the pin is driven by something; identify it by voltage. */
    int mv = -1;
    if (mcu_adc_ready() && mcu_adc_read_mv_avg(BOARD_REV_ADC_CH, 8, &mv) == 0) {
        s_strap_mv = mv;
        /* Step 3, digital v3r4 or later only (a ratio no older board reaches): both straps must
           agree, else it runs as v3.  Before the mV band check, which a v3r4 strap read with a
           high VREF+ can leave. */
        const int vref = mcu_adc_vref_mv();
        if (board_rev_strap_says_v3r4(mv, vref)) {
            s_rev2_mv = read_rev2_mv();
            s_rev = board_rev_decide(mv, vref, s_rev2_mv);
            if (s_rev != BOARD_REV_V3R4) {
                log_printf("[board] WARNING: strap %d mV says v3r4 but the second strap reads %d mV "
                       "(expected ~%d mV); running as v3\r\n", mv, s_rev2_mv, BOARD_REV2_V3R4_NOMINAL_MV);
                return;
            }
            log_printf("[board] revision %s (straps %d / %d mV, VREF+ %d mV)\r\n",
                   board_rev_str(), mv, s_rev2_mv, vref);
            return;
        }
        s_rev = board_rev_from_strap_mv(mv);
        if (!board_rev_strap_known(mv)) {
            /* Driven, but not a ratio we know.  It is certainly not a v2 board, so run as v3
               (not v3r2: that would send DAC8554 frames to a DAC8551) and make it loud. */
            log_printf("[board] WARNING: unrecognised revision strap %d mV "
                   "(expected ~%d mV for v3, ~%d for v3r2, ~%d for v3r4); running as v3\r\n",
                   mv, BOARD_REV_V3_NOMINAL_MV, BOARD_REV_V3R2_NOMINAL_MV, BOARD_REV_V3R4_NOMINAL_MV);
            return;
        }
        log_printf("[board] revision %s (strap %d mV, VREF+ %d mV)\r\n",
               board_rev_str(), mv, mcu_adc_vref_mv());
        return;
    }

    s_rev = BOARD_REV_V3;
    log_printf("[board] revision v3 (strap fitted; ADC unavailable, no voltage check)\r\n");
}

int  board_rev_get(void)      { return s_rev; }
bool board_rev_is_v3(void)    { return s_rev >= BOARD_REV_V3; }
int  board_rev_strap_mv(void) { return s_strap_mv; }
int  board_rev2_strap_mv(void) { return s_rev2_mv; }

const char *board_rev_str(void)
{
    switch (s_rev) {
    case BOARD_REV_V2: return "v2";
    case BOARD_REV_V3: return "v3";
    case BOARD_REV_V3R2: return "v3r2";
    case BOARD_REV_V3R4: return "v3r4";
    default:           return "unknown";
    }
}
