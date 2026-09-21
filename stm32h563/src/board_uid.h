#ifndef BOARD_UID_H
#define BOARD_UID_H

#include <stdint.h>
#include "stm32h5xx.h"

/* The STM32's 96-bit factory unique ID, as three 32-bit words.
 *
 * ALWAYS read it through here, as whole words.  A byte read of this area is a precise bus
 * fault on the STM32H5 (proven on the v3 bring-up, 2026-09-21: ldrb at 0x08fff800,
 * cfsr=0x00008200, bfar=0x08fff800).  The Ethernet MAC code once did exactly that in the net
 * task: the pod crashed on every boot before USB came up and had to be rescued with BOOT0.
 * `make` refuses any other use of UID_BASE (check-uid). */
static inline void board_uid_words(uint32_t w[3])
{
    w[0] = *(const volatile uint32_t *)(UID_BASE);
    w[1] = *(const volatile uint32_t *)(UID_BASE + 4U);
    w[2] = *(const volatile uint32_t *)(UID_BASE + 8U);
}

#endif /* BOARD_UID_H */
