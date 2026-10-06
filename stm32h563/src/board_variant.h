/*
 * board_variant.h — does this BenchPod have the analog front end?
 *
 * The digital-only board (digital-v3, digital-v3r2) is the analog board without its analog
 * section: no ADC, DAC, ±14 V supplies or relays. It carries the same revision strap as the
 * v3 analog board, so board_rev cannot tell them apart.
 *
 * The analog section is detected by its two I2C expanders: U55 (DAC mux, 0x24) and U58
 * (analog switching, 0x26). Both sit on the main +3V3 rail and the I2C0 bus, so they answer
 * from power-up, and they exist only on analog boards. An I2C ACK is definitive, needs no
 * gateware and switches nothing. The ADC and DAC themselves cannot be detected: the DAC8551
 * has no data output, and the MCP33131 has no ID register (its data line just floats).
 *
 * The digital board's expansion connector (CN1) can take an analog add-on later. An add-on with
 * the same two expanders at the same addresses is detected the same way, at the next boot: the
 * pod has to be restarted after one is fitted.
 *
 * Probed once at boot, after i2c_bus_init(); accessors are plain RAM reads afterwards.
 */
#ifndef BOARD_VARIANT_H
#define BOARD_VARIANT_H

#include <stdbool.h>
#include <stdint.h>

#define BOARD_VARIANT_PROBE_TRIES 3u

/* The decision itself, with the probe injected (host tests). True when either expander ACKs
   on any of BOARD_VARIANT_PROBE_TRIES attempts: one missed ACK must not turn an analog board
   into a digital one. *u55 / *u58 report which expander answered (may be NULL). */
bool board_variant_detect(bool (*probe)(uint8_t addr), bool *u55, bool *u58);

/* Probe the I2C0 bus and latch the result. Call once after i2c_bus_init(). */
void board_variant_init(void);

/* The board has the analog front end. True until board_variant_init() runs, so nothing is
   hidden on an analog board by calling it early. */
bool board_has_analog(void);

/* "analog" or "digital". */
const char *board_variant_str(void);

#endif /* BOARD_VARIANT_H */
