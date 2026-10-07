#ifndef DAC_REARM_H
#define DAC_REARM_H
#include <stdbool.h>
#include <stdint.h>

/* Pure, host-testable core of "stop the DAC before it is armed again" (signal_engine.c).
 *
 * The gateware's DAC source modes are LEVELS: dac_psram_mode (deep replay, START_DAC_PSRAM)
 * and dac_loop_mode (closed loop, START_DAC_LOOP) stay 1 until STOP_DAC.  Re-sending the same
 * start opcode while one runs only pulses the DAC engine's start, so the engine restarts at
 * its LOW-byte pop while its byte source keeps going:
 *   - deep replay: the PSRAM reader keeps its old address, remaining count and prefetch FIFO
 *     (it reloads base/len and flushes the FIFO only while `run` is low), so the new replay
 *     plays the tail of the old one and can come out byte-swapped;
 *   - closed loop: dac_loop's byte_sel resets only on disarm, so a restart between the two
 *     byte pops swaps the bytes of every value after it.
 * STOP_DAC drops the level; the next start then reloads the reader (or realigns the loop) from
 * a clean state.  The low phase only has to last a few clk48 cycles (the reader's 2-FF run
 * sync), and the STATUS poll plus the 9-byte start opcode keep it low for microseconds.
 *
 * STOP_DAC also cancels a staged co-trigger (DAC_ARM_ON_CAPTURE), so a pending one is
 * staged again after the stop: the start that follows must still wait for the capture's t0. */

#define DAC_REARM_STATUS_DAC_RUN  (1u << 0)   /* STATUS bit 0, must match the gateware */
#define DAC_REARM_MAX_POLLS       8u          /* the engine drops `running` within ~0.2 us */

typedef struct {
    int  (*status_read)(uint8_t *st);   /* CMD_STATUS; 0 = valid read */
    void (*stop)(void);                 /* CMD_STOP_DAC (cancels a staged co-trigger) */
    bool (*arm_cotrig)(void);           /* CMD_DAC_ARM_ON_CAPTURE */
} dac_rearm_ops_t;

/* Quiesce the DAC before a new start.
 *   force          stop even when STATUS says idle: a deep replay staged on a co-trigger has
 *                  the reader streaming (mode high) while DAC_RUN is still 0.
 *   cotrig_pending a co-trigger is staged for the start that follows; staged again after the stop.
 * Without `force` an unreadable STATUS means "not running" (the old fpga_load_wave rule).
 * Returns 0 when the DAC is idle (or was never running), -1 if DAC_RUN never cleared. */
int dac_rearm_quiesce(const dac_rearm_ops_t *ops, bool force, bool cotrig_pending);

#endif
