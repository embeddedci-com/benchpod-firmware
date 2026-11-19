/* vref_out.c — see vref_out.h.  Digital v3r4 only (board_caps()->vref_out). */
#include "vref_out.h"
#include "board_caps.h"
#include "board_pins.h"
#include "mcu_adc.h"
#include "la_io.h"   /* la_vccio_get_mv */
#include "stm32h5xx_hal.h"
#include "bp_log.h"

static bool         s_present;
static vref_src_t   s_src = VREF_SRC_BANK;
static int          s_mv = -1;
static vref_state_t s_state = VREF_UNKNOWN;
static bool         s_up;
static uint32_t     s_next_ms;
static void       (*s_hook)(bool up);

static void pin_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode, uint32_t pull)
{
    GPIO_InitTypeDef gp = {0};
    gp.Pin   = pin;
    gp.Mode  = mode;
    gp.Pull  = pull;
    gp.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &gp);
}

/* PR1 released: R208 holds it high, U67 passes VIN1 (the bank). */
static void sel_bank(void) { pin_mode(VREF_SEL_PORT, VREF_SEL_PIN, GPIO_MODE_INPUT, GPIO_NOPULL); }

/* PR1 driven low: U67 passes VIN2 (the 5 V switch).  Level first, then the mode. */
static void sel_5v(void)
{
    HAL_GPIO_WritePin(VREF_SEL_PORT, VREF_SEL_PIN, GPIO_PIN_RESET);
    pin_mode(VREF_SEL_PORT, VREF_SEL_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
}

static void sw_5v(bool on)
{
    HAL_GPIO_WritePin(VREF_5V_EN_PORT, VREF_5V_EN_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void vref_out_init(void)
{
    if (!board_caps()->vref_out) return;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    /* The 5 V switch off first (R209 already holds it off), then pin 21 on the bank. */
    sw_5v(false);
    pin_mode(VREF_5V_EN_PORT, VREF_5V_EN_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    sel_bank();
    pin_mode(VREF_SENSE_PORT, VREF_SENSE_PIN, GPIO_MODE_ANALOG, GPIO_NOPULL);
    s_present = true;
    s_src = VREF_SRC_BANK;
    (void)vref_out_measure_mv();
    log_printf("[vref] pin 21 on the bank, %d mV (%s)\r\n", s_mv, vref_state_name(s_state));
}

bool vref_out_present(void) { return s_present; }

int vref_out_set_source(vref_src_t src)
{
    if (!s_present) return -1;
    if (src == s_src) return 0;
    if (src == VREF_SRC_5V) {
        sw_5v(true);
        HAL_Delay(3);        /* the switch's soft start (datasheet, not measured), before U67 is pointed at it */
        sel_5v();
    } else {
        sel_bank();
        HAL_Delay(1);        /* U67 back on VIN1 before VIN2 goes away */
        sw_5v(false);
    }
    s_src = src;
    return 0;
}

vref_src_t vref_out_source(void) { return s_src; }

int vref_out_measure_mv(void)
{
    if (!s_present) return -1;
    int pin = -1;
    if (mcu_adc_read_mv_avg(VREF_SENSE_ADC_CH, 8, &pin) != 0) {
        s_mv = -1;
        s_state = VREF_UNKNOWN;
        return -1;
    }
    s_mv = pin * VREF_SENSE_MUL;
    s_state = vref_classify(s_src, vref_setpoint_mv(s_src, la_vccio_get_mv()), s_mv);
    bool up = vref_rail_up(s_up, s_mv);
    if (up != s_up) {
        s_up = up;
        if (s_hook) s_hook(up);
    }
    return s_mv;
}

void vref_out_poll(uint32_t now_ms)
{
    if (!s_present || (int32_t)(now_ms - s_next_ms) < 0) return;
    s_next_ms = now_ms + VREF_POLL_MS;
    vref_state_t was = s_state;
    (void)vref_out_measure_mv();
    if (s_state != was) {
        int set = vref_setpoint_mv(s_src, la_vccio_get_mv());
        if (s_state == VREF_HIGH)
            LOG_W("[vref] pin 21 at %d mV, above its %d mV setting: a DUT signal above "
                  "the I/O voltage lifts it through the pulls; select 5 V or remove the signal\n",
                  s_mv, set);
        else
            log_printf("[vref] pin 21 %s: %d mV (setting %d mV, %s)\r\n",
                       vref_state_name(s_state), s_mv, set, vref_src_name(s_src));
    }
}

int          vref_out_last_mv(void) { return s_mv; }
vref_state_t vref_out_state(void)   { return s_present ? s_state : VREF_UNKNOWN; }
bool         vref_out_rail_up(void) { return s_present && s_up; }
void         vref_out_set_rail_hook(void (*hook)(bool up)) { s_hook = hook; }

void vref_out_emit(bp_emit_t *e)
{
    vref_emit_fields(e, s_src, vref_setpoint_mv(s_src, la_vccio_get_mv()), s_mv,
                     vref_out_state(), vref_out_rail_up());
}
