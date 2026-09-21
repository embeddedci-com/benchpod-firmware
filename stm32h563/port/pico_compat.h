/*
 * pico_compat.h — minimal Raspberry Pi Pico SDK time/delay API on STM32H5.
 *
 * The ported bench-pod modules (signal_engine.c, target_power.c, console.c, …)
 * use a small set of Pico SDK timing primitives.  Re-implementing just those
 * here lets those files keep their timing code verbatim; the bus/GPIO transport
 * is ported to STM32 HAL inside each module.
 *
 * absolute_time_t is microseconds since boot (monotonic 64-bit, from TIM2).
 */
#ifndef PICO_COMPAT_H
#define PICO_COMPAT_H

#include <stdint.h>
#include <stdbool.h>

typedef uint64_t absolute_time_t;

/* Bring up the microsecond timebase (TIM2 free-running @ 1 MHz) and the DWT
   cycle counter used by busy_wait_us().  Call once, early in main(). */
void pico_compat_init(void);

/* Monotonic microseconds / milliseconds since boot. */
uint64_t time_us_64(void);
uint32_t time_us_32(void);
uint32_t to_ms_since_boot(absolute_time_t t);

static inline absolute_time_t get_absolute_time(void) { return time_us_64(); }
static inline int64_t absolute_time_diff_us(absolute_time_t from, absolute_time_t to)
{
    return (int64_t)(to - from);
}
static inline absolute_time_t make_timeout_time_ms(uint32_t ms)
{
    return time_us_64() + (uint64_t)ms * 1000ull;
}
static inline absolute_time_t make_timeout_time_us(uint64_t us)
{
    return time_us_64() + us;
}
static inline bool time_reached(absolute_time_t t)
{
    return time_us_64() >= t;
}

/* Delays.  sleep_ms() yields to FreeRTOS once the scheduler is running, and
   busy-waits (HAL_Delay) before that.  busy_wait_us() always spins. */
void sleep_ms(uint32_t ms);
void sleep_us(uint64_t us);
void busy_wait_us(uint64_t us);

static inline void tight_loop_contents(void) { __asm volatile("nop"); }

#endif /* PICO_COMPAT_H */
