/* Pico SDK <pico/rand.h> shim -> STM32 hardware RNG (see src/rng.c). */
#ifndef PICO_RAND_SHIM_H
#define PICO_RAND_SHIM_H
#include <stdint.h>
uint64_t get_rand_64(void);
uint32_t get_rand_32(void);
#endif
