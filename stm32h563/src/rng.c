/*
 * rng.c — STM32H5 hardware RNG backing the Pico-style get_rand_64/32().
 * Used by device_identity.c to seed the Ed25519 key on first boot.
 * RNG kernel clock = HSI48 (enabled here if not already on for USB).
 */
#include "pico/rand.h"
#include "stm32h5xx_hal.h"

static RNG_HandleTypeDef hrng;
static int s_ready;

static void rng_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_PeriphCLKInitTypeDef pclk = {0};

    /* Ensure HSI48 is running and route it to the RNG kernel clock. */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    osc.HSI48State = RCC_HSI48_ON;
    osc.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&osc);

    pclk.PeriphClockSelection = RCC_PERIPHCLK_RNG;
    pclk.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
    HAL_RCCEx_PeriphCLKConfig(&pclk);

    __HAL_RCC_RNG_CLK_ENABLE();
    hrng.Instance = RNG;
    if (HAL_RNG_Init(&hrng) == HAL_OK) s_ready = 1;
}

uint32_t get_rand_32(void)
{
    if (!s_ready) rng_init();
    uint32_t v = 0;
    if (s_ready) HAL_RNG_GenerateRandomNumber(&hrng, &v);
    return v;
}

uint64_t get_rand_64(void)
{
    uint64_t hi = get_rand_32();
    uint64_t lo = get_rand_32();
    return (hi << 32) | lo;
}
