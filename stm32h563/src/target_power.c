/*
 * target_power.c — dual eFuse control via the TCA9554 power expander (0x22).
 *
 * Ported from the RP2350B target_power.c: same public API and scheduling
 * behaviour, but EN/V/FLT now live on expander pins instead of native GPIO.
 */
#include "target_power.h"
#include "i2c_bus.h"
#include "tca9554.h"
#include "signal_engine.h"
#include "pico_compat.h"
#include "hw_lock.h"   /* serialize I2C access vs the net task */
#include <stdio.h>

#define PWR_ADDR  I2C_ADDR_TCA9554_PWR

/* TCA9554 register indices (1 = input dir, 0 = output dir in CONFIG). */
#define TCA_REG_OUTPUT  0x01u
#define TCA_REG_CONFIG  0x03u

typedef struct { uint8_t en, v, flt; } efuse_pins_t;

/* Index 0 = eFuse1 (internal), index 1 = eFuse2 (external). */
static const efuse_pins_t efuses[2] = {
    { 7, 6, 5 },   /* eFuse1: EN=P7, V=P6, FLT=P5 */
    { 0, 1, 2 },   /* eFuse2: EN=P0, V=P1, FLT=P2 */
};

/* The two EN pins (P7 = eF1, P0 = eF2) are the only outputs; V/FLT are inputs.
   CONFIG: 1 = input, 0 = output, so outputs get a 0 bit. */
#define PWR_EN_MASK   ((uint8_t)((1u << 7) | (1u << 0)))
#define PWR_CONFIG    ((uint8_t)~PWR_EN_MASK)   /* 0x7E: P7,P0 output; rest input */

static bool            enabled_state[2];
static bool            last_fault[2];        /* last FLT level sampled in poll, for edge detection */
static struct {
    bool            armed;
    bool            on;
    absolute_time_t at;
} pending[2];

/* Optional async change notification (cloud-agnostic; NULL until registered). */
static target_power_event_fn event_cb;

void target_power_set_event_cb(target_power_event_fn cb)
{
    event_cb = cb;
}

/* Read the current FLT (active-low) for the eFuse at index idx.  Returns false
   if the read fails (treat unknown as "no fault" to avoid spurious events). */
static bool efuse_fault_now(int idx)
{
    bool flt = false;
    if (tca9554_read_pin(PWR_ADDR, efuses[idx].flt, &flt) != 0) return false;
    return (flt == false);   /* active-low: asserted when the pin reads 0 */
}

static int efuse_index(int efuse)
{
    if (efuse == 1) return 0;
    if (efuse == 2) return 1;
    return -1;
}

/* Reflect eFuse state on the iCE40 green status LED: any eFuse ON -> green,
   both OFF -> dark.  A visual confirmation that target power is live. */
static void power_update_led(void)
{
    bool any_on = enabled_state[0] || enabled_state[1];
    fpga_set_led(any_on ? 0x01u : 0x00u);   /* bit0 = green */
}

void target_power_init(void)
{
    for (int i = 0; i < 2; i++) { enabled_state[i] = false; last_fault[i] = false; pending[i].armed = false; }

    /* Glitch-free bring-up.  The TCA9554 powers up with ALL pins as inputs
       (high-Z) and the output latch = 0xFF, so from power-on until we run, the
       eFuse EN pins are undriven — a DUT rail can float on if the EN net isn't
       held low in hardware.  Minimise that: (1) main() calls us FIRST after the
       I2C bus is up; (2) write the WHOLE output register to the safe value (both
       EN latches = 0) in one unconditional write — not a read-modify-write of the
       0xFF default — so the OFF state can't depend on a correct read; then (3)
       switch the EN pins to outputs.  Order is critical: OUTPUT-latch-low BEFORE
       CONFIG-as-output, and if the latch write fails we must NOT enable the
       outputs (that would drive the 0xFF default HIGH = eFuse ON), so leave the
       pins as inputs (POR) and warn.
       NOTE: the pre-firmware POR window still needs a hardware pulldown on each
       EN net for full elimination — same as the U53 relay note in i2c_bus.c. */
    if (tca9554_write_reg(PWR_ADDR, TCA_REG_OUTPUT, 0x00) != 0) {
        printf("[pwr] WARN: TCA9554@0x%02x output preset failed — leaving EN pins "
               "as inputs (both eFuses stay OFF)\n", PWR_ADDR);
        return;   /* do not enable outputs off a possibly-0xFF latch */
    }
    if (tca9554_write_reg(PWR_ADDR, TCA_REG_CONFIG, PWR_CONFIG) != 0) {
        printf("[pwr] WARN: TCA9554@0x%02x config write failed\n", PWR_ADDR);
        return;
    }
    printf("[pwr] eFuse control via TCA9554@0x%02x (eF1 EN=P7, eF2 EN=P0), both OFF\n",
           PWR_ADDR);
}

int target_power_enable(int efuse, bool on)
{
    int idx = efuse_index(efuse);
    if (idx < 0) return -1;
    if (tca9554_write_pin(PWR_ADDR, efuses[idx].en, on) != 0) return -1;
    enabled_state[idx] = on;
    power_update_led();
    /* Notify only on a confirmed EN change (I2C write succeeded).  Report the
       current FLT alongside the new EN state. */
    if (event_cb) event_cb(efuse, on, efuse_fault_now(idx));
    return 0;
}

int target_power_schedule(int efuse, bool on, uint32_t delay_ms)
{
    int idx = efuse_index(efuse);
    if (idx < 0) return -1;
    if (delay_ms == 0) return target_power_enable(efuse, on);
    pending[idx].armed = true;
    pending[idx].on = on;
    pending[idx].at = make_timeout_time_ms(delay_ms);
    return 0;
}

/* Runs in the console task; target_power_enable() drives the TCA9554 over I2C,
   the same bus the net task uses — so take hw_lock around the deferred enable to
   serialize it (same reasoning as signal_engine_poll's SPI1 ops). */
void target_power_poll(void)
{
    for (int i = 0; i < 2; i++) {
        if (pending[i].armed && time_reached(pending[i].at)) {
            pending[i].armed = false;
            hw_lock();
            target_power_enable(i + 1, pending[i].on);
            hw_unlock();
        }
    }

    /* Sample FLT (active-low) every 10 ms and notify on a rising edge — there is
       no other periodic FLT sampling, so this is what makes a fault async.  The
       reads hit the same I2C bus the net task uses, so serialize with hw_lock.
       Not every worker pass: the two reads cost ~1 ms of I2C at 100 kHz, which held the
       power_profile sampler (same bus, same task) to ~400 reads/s.  FLT stays asserted
       through the eFuse's ~110 ms auto-retry, so 10 ms sampling still sees every trip. */
    static uint64_t next_flt_us;
    uint64_t now_us = time_us_64();
    if (now_us < next_flt_us) return;
    next_flt_us = now_us + 10000u;
    for (int i = 0; i < 2; i++) {
        hw_lock();
        bool flt = efuse_fault_now(i);
        hw_unlock();
        if (flt && !last_fault[i] && event_cb)
            event_cb(i + 1, enabled_state[i], true);
        last_fault[i] = flt;
    }
}

int target_power_get_status(int efuse, target_power_status_t *out)
{
    int idx = efuse_index(efuse);
    if (idx < 0 || !out) return -1;
    const efuse_pins_t *e = &efuses[idx];
    bool v = false, flt = false;

    out->enabled = enabled_state[idx];
    out->status_supported = true;
    (void)tca9554_read_pin(PWR_ADDR, e->v, &v);
    (void)tca9554_read_pin(PWR_ADDR, e->flt, &flt);
    out->valid = v;             /* active-high */
    out->fault = (flt == false); /* active-low: asserted when the pin reads 0 */
    return 0;
}

int target_power_get_cached(int efuse, bool *enabled, bool *fault)
{
    int idx = efuse_index(efuse);
    if (idx < 0) return -1;
    /* Plain RAM reads — no I2C — so this is safe from the net task.  enabled_state is
       the last commanded EN (authoritative even when a disabled rail reads power-good
       low); last_fault is the FLT sampled by the most recent target_power_poll pass. */
    if (enabled) *enabled = enabled_state[idx];
    if (fault)   *fault   = last_fault[idx];
    return 0;
}
