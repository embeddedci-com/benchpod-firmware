/*
 * mcu_adc.c — see mcu_adc.h.  Single-shot polled conversions on ADC1.
 *
 * Kernel clock: HSI (64 MHz) divided by 4 in the ADC common prescaler, so the
 * ADC runs at 16 MHz regardless of how SystemClock_Config() leaves the PLL —
 * the same reason i2c_bus.c sources I2C1 from HSI.  With ADC_SAMPLETIME_247CYCLES_5
 * a 12-bit conversion takes ~16 us, and the 10 k source impedance on the CC
 * taps (R178/R179) settles comfortably inside that window.
 */
#include "mcu_adc.h"
#include "stm32h5xx_hal.h"
#include "board_pins.h"

#include <stdio.h>

/* VREF+ is VDDA through R1 (47 R) with no DC load, so nominally +3V3. */
#define MCU_ADC_VREF_NOMINAL_MV  3300
#define MCU_ADC_FULL_SCALE       4095u        /* 12-bit */
#define MCU_ADC_TIMEOUT_MS       10u

static ADC_HandleTypeDef s_adc;
static bool s_ready;
static int  s_vref_mv = MCU_ADC_VREF_NOMINAL_MV;

/* Select `channel` on regular rank 1 and run one conversion. */
static int convert_raw(uint32_t channel, uint32_t *out_raw)
{
    ADC_ChannelConfTypeDef ch = {0};
    ch.Channel      = channel;
    ch.Rank         = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    ch.SingleDiff   = ADC_SINGLE_ENDED;
    ch.OffsetNumber = ADC_OFFSET_NONE;
    ch.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&s_adc, &ch) != HAL_OK) return -1;

    if (HAL_ADC_Start(&s_adc) != HAL_OK) return -1;
    if (HAL_ADC_PollForConversion(&s_adc, MCU_ADC_TIMEOUT_MS) != HAL_OK) {
        HAL_ADC_Stop(&s_adc);
        return -1;
    }
    *out_raw = HAL_ADC_GetValue(&s_adc);
    HAL_ADC_Stop(&s_adc);
    return 0;
}

int mcu_adc_init(void)
{
    if (s_ready) return 0;

    /* ADC/DAC kernel clock from HSI so the sampling window is PLL-independent. */
    RCC_PeriphCLKInitTypeDef pclk = {0};
    pclk.PeriphClockSelection = RCC_PERIPHCLK_ADCDAC;
    pclk.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) return -1;

    __HAL_RCC_ADC_CLK_ENABLE();

    /* The three sensed pins as analog inputs.  Analog mode disconnects the
       Schmitt trigger and the pull resistors, which is what we want for a
       measurement — board_rev.c temporarily flips PA3 back to a digital input
       when it needs the internal pull-down for its NC test. */
    BOARD_REV_CLK_EN();
    USB_CC_CLK_EN();
    GPIO_InitTypeDef gp = {0};
    gp.Mode = GPIO_MODE_ANALOG;
    gp.Pull = GPIO_NOPULL;
    gp.Pin  = BOARD_REV_PIN;
    HAL_GPIO_Init(BOARD_REV_PORT, &gp);
    gp.Pin  = USB_CC1_PIN | USB_CC2_PIN;   /* PC0 + PC2, same port */
    HAL_GPIO_Init(USB_CC1_PORT, &gp);

    s_adc.Instance                      = ADC1;
    s_adc.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV4;
    s_adc.Init.Resolution               = ADC_RESOLUTION_12B;
    s_adc.Init.DataAlign                = ADC_DATAALIGN_RIGHT;
    s_adc.Init.ScanConvMode             = ADC_SCAN_DISABLE;
    s_adc.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    s_adc.Init.LowPowerAutoWait         = DISABLE;
    s_adc.Init.ContinuousConvMode       = DISABLE;
    s_adc.Init.NbrOfConversion          = 1;
    s_adc.Init.DiscontinuousConvMode    = DISABLE;
    s_adc.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    s_adc.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
    s_adc.Init.DMAContinuousRequests    = DISABLE;
    s_adc.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
    s_adc.Init.OversamplingMode         = DISABLE;
    if (HAL_ADC_Init(&s_adc) != HAL_OK) return -1;

    if (HAL_ADCEx_Calibration_Start(&s_adc, ADC_SINGLE_ENDED) != HAL_OK) return -1;

    s_ready = true;

    /* Latch VREF+ from the factory-trimmed VREFINT.  A failure here is not
       fatal: fall back to the 3300 mV nominal and carry on. */
    uint32_t raw = 0;
    if (convert_raw(ADC_CHANNEL_VREFINT, &raw) == 0 && raw != 0) {
        s_vref_mv = (int)__LL_ADC_CALC_VREFANALOG_VOLTAGE(raw, LL_ADC_RESOLUTION_12B);
    } else {
        printf("[adc] VREFINT read failed; assuming VREF+ = %d mV\r\n",
               MCU_ADC_VREF_NOMINAL_MV);
    }
    return 0;
}

bool mcu_adc_ready(void) { return s_ready; }

int mcu_adc_vref_mv(void) { return s_vref_mv; }

int mcu_adc_read_mv(uint32_t channel, int *out_mv)
{
    return mcu_adc_read_mv_avg(channel, 1, out_mv);
}

int mcu_adc_read_mv_avg(uint32_t channel, unsigned n, int *out_mv)
{
    if (!s_ready || out_mv == NULL) return -1;
    if (n < 1)  n = 1;
    if (n > 64) n = 64;

    uint32_t acc = 0;
    for (unsigned i = 0; i < n; i++) {
        uint32_t raw = 0;
        if (convert_raw(channel, &raw) != 0) return -1;
        acc += raw;
    }
    /* Round to nearest: (acc * vref) / (n * full_scale). */
    uint32_t denom = (uint32_t)n * MCU_ADC_FULL_SCALE;
    *out_mv = (int)(((uint64_t)acc * (uint32_t)s_vref_mv + denom / 2) / denom);
    return 0;
}
