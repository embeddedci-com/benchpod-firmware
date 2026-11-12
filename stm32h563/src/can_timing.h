#ifndef CAN_TIMING_H
#define CAN_TIMING_H

/* ============================================================================
 * can_timing — FDCAN bit-timing solver and CAN FD length helpers.
 *
 * Pure integer logic (no HAL), host-tested in test/test_can_timing.c.
 *
 * Kernel clocks (can_bus.c picks one per configuration):
 *   classic CAN: HSE, 25 MHz.  Every classic rate the pod had before keeps the
 *                exact same timing.
 *   CAN FD:      PLL2Q, 80 MHz (HSE / 5 * 80 / 5).  25 MHz cannot time the most
 *                common data rate, 2 Mbit/s (12.5 tq per bit); 80 MHz is one of
 *                the clocks CiA 601-3 recommends and times 1, 2, 4, 5 and
 *                8 Mbit/s exactly.
 * ========================================================================== */

#include <stdint.h>
#include <stdbool.h>

#define CAN_CLASSIC_KERNEL_HZ  25000000u
#define CAN_FD_KERNEL_HZ       80000000u

/* CAN FD data phase limits: the TCAN1044 transceiver is rated to 8 Mbit/s. */
#define CAN_FD_DATA_MAX_BPS    8000000u
#define CAN_FD_MAX_LEN         64u

typedef struct {
    uint32_t presc;   /* kernel-clock prescaler                                */
    uint32_t seg1;    /* time segment 1 in tq (prop + phase1, excludes sync)   */
    uint32_t seg2;    /* time segment 2 in tq                                  */
    uint32_t sjw;     /* resynchronisation jump width in tq                    */
    uint32_t tdco;    /* data phase only: transmitter delay compensation offset */
                      /* in kernel clocks (0 = TDC off)                         */
} can_timing_t;

/* Nominal (arbitration) bit timing.
 *   fd = false: the classic solver: largest total of 8..25 tq with an integer
 *               prescaler, ~87.5 % sample point (unchanged from before FD).
 *   fd = true:  smallest prescaler (most tq per bit) within the FDCAN nominal
 *               limits, 80 % sample point, SJW = seg2 (CiA 601-3).
 * Returns 0, or -1 if the rate has no exact timing on this clock. */
int can_timing_nominal(uint32_t fclk, uint32_t bitrate, bool fd, can_timing_t *out);

/* Data-phase bit timing for CAN FD with bit-rate switching.  Smallest prescaler
 * (1..32) whose tq count fits the data limits (1 + 32 + 16), sample point 80 %
 * up to 2 Mbit/s and 75 % above, SJW = seg2.  Sets tdco to the sample point in
 * kernel clocks when the prescaler is 1 or 2 (the FDCAN TDC limit).
 * Returns 0, -1 if the rate has no exact timing on this clock, -2 if the rate
 * is above CAN_FD_DATA_MAX_BPS. */
int can_timing_data(uint32_t fclk, uint32_t bitrate, can_timing_t *out);

/* Sample point of a timing, in per mille. */
uint32_t can_timing_sample_permille(const can_timing_t *t);

/* DLC code (0..15) -> payload bytes. */
uint8_t can_dlc_to_len(uint32_t dlc);

/* Payload bytes -> DLC code.  Classic: 0..8.  FD: 0..8, 12, 16, 20, 24, 32,
 * 48, 64; another length is rounded up to the next FD length (the caller pads
 * with zeros).  Returns the DLC code, or -1 if too long for the frame type. */
int can_len_to_dlc(uint32_t len, bool fd);

#endif /* CAN_TIMING_H */
