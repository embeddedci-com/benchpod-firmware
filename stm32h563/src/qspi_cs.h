#ifndef QSPI_CS_H
#define QSPI_CS_H

/* qspi_cs — the chip-selects of the shared quad bus: the iCE40 config flash (/ICE_SS) and the
 * PSRAM (/ICE_SS_PSRAM).
 *
 * Up to digital v3r3 they are PE3 and PE4.  Digital v3r4 moves them to PB6 and PC11, the two
 * free pins that can also carry OCTOSPI1_NCS (board_caps()->qspi_cs_ncs_pins), so a later driver
 * can let the controller frame a transfer itself (one memory at a time: the H563 has one NCS).
 * The firmware still drives both as GPIOs.
 *
 * The pins live in RAM (.data), not in a const table: ota_commit.c frames PSRAM reads from
 * RAM-resident code while the internal flash is erased.  They hold the PE3/PE4 pins until
 * qspi_cs_init() runs right after board_rev_init(), before anything touches the bus.
 * board_pins.h's ICE_FLASH_CS_* and PSRAM_CS_* read them. */

#include <stdint.h>
#include "stm32h5xx_hal.h"

typedef struct {
    GPIO_TypeDef *flash_port;   /* /ICE_SS */
    uint16_t      flash_pin;
    GPIO_TypeDef *psram_port;   /* /ICE_SS_PSRAM */
    uint16_t      psram_pin;
} qspi_cs_pins_t;

extern qspi_cs_pins_t qspi_cs;

/* Pick the pins for this board revision (board_caps() must be latched) and enable their GPIO
   clocks.  Leaves the pins themselves alone: they stay inputs until a driver takes them. */
void qspi_cs_init(void);

#endif /* QSPI_CS_H */
