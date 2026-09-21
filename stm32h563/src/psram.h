#ifndef PSRAM_H
#define PSRAM_H

#include <stdint.h>

/* APS6404L 8 MB QSPI PSRAM on the STM32H5 OCTOSPI1.  Shares the quad bus
 * (PF6..PF10) with the iCE40 config flash; PE4 (PSRAM CS) and PE3 (flash CS) are
 * plain GPIO chip-selects (PE4 is not an OCTOSPI-NCS pin), so transfers are
 * OCTOSPI indirect, framed by the STM32 toggling PE4.  PG0 hands the shared bus
 * to the iCE40 (which writes captures into the PSRAM) and back. */
#define PSRAM_SIZE   0x00800000u   /* 8 MB */

/* Bring up OCTOSPI1, reset + identify the APS6404L, enter QPI.  Returns 0 if the
   JEDEC ID matched (MFID 0x0D / KGD 0x5D), <0 otherwise. */
int psram_init(void);

/* Read `len` bytes from PSRAM byte-address `addr` (QPI fast read, chunked under
   tCEM).  Requires the bus acquired.  Returns 0 on success, <0 on error. */
int psram_read(uint32_t addr, uint8_t *buf, uint32_t len);

/* Write `len` bytes to PSRAM (QPI 0x38, chunked under tCEM).  Requires the bus
   acquired (psram_bus_acquire).  Used for the capture no-write sentinel. */
int psram_write(uint32_t addr, const uint8_t *buf, uint32_t len);

/* Write + read-back pattern self-test.  Returns 0 on success, <0 on mismatch. */
int psram_test(void);

/* Read-throughput benchmark: read total_kb KB (0 => 256) sequentially and report
   the effective MB/s + a 4 KB integrity check.  Returns 0 if integrity ok. */
int psram_bench(uint32_t total_kb);

/* Re-init OCTOSPI1 at a new clock prescaler (SCLK = HCLK/(presc+1)); returns the
   resulting SCLK Hz, or 0 on failure.  For the psram-clk read-clock sweep. */
uint32_t psram_set_prescaler(uint32_t presc);

/* Set the tCEM CS-low burst size in bytes (clamped 1..1024); returns the value.
   For the psram-chunk sweep — bigger chunks cut the per-chunk HAL overhead. */
uint32_t psram_set_chunk(uint32_t n);

/* Bus/CS/arbitration diagnostic for the '0xFF ID' failure: reports arbitration
   pin state, samples who drives the shared bus with the iCE40 running vs. held
   in reset, then re-probes the PSRAM ID with the iCE40 off the bus.  Returns the
   psram_init() result of that isolated probe.  Restores the iCE40 afterwards. */
int psram_diag(void);

/* The shared OCTOSPI1 handle (initialised by psram_init), so the iCE40
   bitstream-reflash path can reuse the controller with its own GPIO chip-select
   (PE3) for the W25Q64 config flash on the same quad bus. */
void *psram_xspi(void);

/* Shared-bus arbitration for v2 capture.  The quad bus is shared with the iCE40
   (which writes capture samples into the PSRAM).  acquire() drives PG0 high and
   restores the OCTOSPI pins so the STM32 can read; release() tristates the
   OCTOSPI pins and drops PG0 so the iCE40 can drive the bus during a capture. */
void psram_bus_acquire(void);
void psram_bus_release(void);
/* Hold the PSRAM /CS driven high (deselected) with SCLK/IO tristated — for an iCE40
   warmboot so the PSRAM doesn't corrupt the shared-bus config-flash read. */
void psram_cs_park_high(void);
/* Reset the PSRAM to standard SPI (exit QPI) — call BEFORE an iCE40 warmboot so a
   QPI-latched PSRAM can't drive the shared SO line into the config-flash read.  Follow
   with psram_init() after the warmboot to restore QPI. */
void psram_reset_to_spi(void);

/* Boot /CE-net probe: hand the shared bus to the iCE40 but sense PE4 (PSRAM /CS on
   the /CE net) as an input+pull-up.  _read() returns 1 if the net is high (the
   iCE40 does NOT reach it) or 0 if the iCE40 pulled it low.  See signal_engine's
   psram_boot_selftest / CMD_PSRAM_CS. */
void psram_ce_probe_begin(void);
int  psram_ce_probe_read(void);
void psram_ce_probe_end(void);
/* Read all six shared PSRAM-bus lines: b0=/CS b1=SCLK b2=IO0 b3=IO1 b4=IO2 b5=IO3. */
uint8_t psram_bus_read_lines(void);

#endif /* PSRAM_H */
