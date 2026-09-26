/*
 * rng.c — STM32H5 hardware RNG behind rng_fill() (rng.h) and the Pico-style
 * get_rand_64/32().  rng_fill() feeds TLS (mbedtls_hardware_poll) and the
 * first-boot Ed25519 key (device_identity.c), so it reports errors instead of
 * returning zeros.
 * RNG kernel clock = HSI48 (enabled here if not already on for USB).
 */
#include "rng.h"
#include "pico/rand.h"
#include "stm32h5xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

static RNG_HandleTypeDef hrng;
static int s_ready;
static uint32_t s_errors;
static uint32_t s_logged_errors;

/* Callers run on several tasks (net: TLS + WebSocket masks, console, boot).
   HAL_RNG's lock makes a concurrent call fail with BUSY rather than wait, so
   keep each read atomic w.r.t. task switches.  Interrupts stay on: the HAL
   timeout counts on the TIM-based HAL tick. */
static inline void rng_enter(void) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) vTaskSuspendAll();
}
static inline void rng_leave(void) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED) xTaskResumeAll();
}

static void rng_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_PeriphCLKInitTypeDef pclk = {0};

    s_ready = 0;

    /* Ensure HSI48 is running and route it to the RNG kernel clock (a clock
       error, CEIS, means this clock stopped or is too slow). */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    osc.HSI48State = RCC_HSI48_ON;
    osc.PLL.PLLState = RCC_PLL_NONE;
    (void)HAL_RCC_OscConfig(&osc);

    pclk.PeriphClockSelection = RCC_PERIPHCLK_RNG;
    pclk.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
    (void)HAL_RCCEx_PeriphCLKConfig(&pclk);

    __HAL_RCC_RNG_CLK_ENABLE();
    hrng.Instance = RNG;
    if (HAL_RNG_Init(&hrng) == HAL_OK) s_ready = 1;
}

/* Recovery after a failed read.  The HAL's own seed-error recovery (clear SEIS,
   RM0481 "Error management") runs inside HAL_RNG_GenerateRandomNumber; when
   it fails it returns with the handle still locked and BUSY, so nothing short
   of a fresh handle gets the HAL working again.  Reset the peripheral through
   RCC (clears SEIS/CEIS, CONDRST state and the conditioning stage) and init
   from scratch. */
static void rng_reset(void *ctx)
{
    (void)ctx;
    rng_enter();
    __HAL_RCC_RNG_FORCE_RESET();
    __HAL_RCC_RNG_RELEASE_RESET();
    memset(&hrng, 0, sizeof(hrng));      /* State = RESET, Lock = UNLOCKED */
    rng_init();
    rng_leave();
}

static int rng_word(void *ctx, uint32_t *out)
{
    (void)ctx;
    uint32_t v = 0;
    HAL_StatusTypeDef st = HAL_ERROR;
    rng_enter();
    if (!s_ready) rng_init();
    if (s_ready) st = HAL_RNG_GenerateRandomNumber(&hrng, &v);
    rng_leave();
    if (st != HAL_OK) return -1;
    *out = v;
    return 0;
}

static void rng_log_errors(void)
{
    uint32_t e = s_errors;
    if (e != s_logged_errors) {
        printf("[rng] %lu read error(s) since boot (last HAL error 0x%lx)\n",
               (unsigned long)e, (unsigned long)hrng.ErrorCode);
        s_logged_errors = e;
    }
}

int rng_fill(void *buf, size_t len)
{
    int rc = rng_fill_policy(buf, len, rng_word, rng_reset, NULL,
                             RNG_WORD_ATTEMPTS, &s_errors);
    rng_log_errors();
    if (rc != 0) printf("[rng] ERROR: TRNG failed %u times in a row, no entropy\n",
                        (unsigned)RNG_WORD_ATTEMPTS);
    return rc;
}

int rng_get32(uint32_t *out)
{
    return rng_fill(out, sizeof(*out));
}

uint32_t rng_error_count(void) { return s_errors; }

/* Non-key callers only (see rng.h).  Still retried; returns 0 if the RNG is
   dead, as before, but now says so on the console. */
uint32_t get_rand_32(void)
{
    uint32_t v = 0;
    (void)rng_get32(&v);
    return v;
}

uint64_t get_rand_64(void)
{
    uint64_t hi = get_rand_32();
    uint64_t lo = get_rand_32();
    return (hi << 32) | lo;
}
