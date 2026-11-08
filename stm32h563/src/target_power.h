#ifndef TARGET_POWER_H
#define TARGET_POWER_H

#include <stdint.h>
#include <stdbool.h>

/* Two target-power eFuses (TPS259470A), controlled through the TCA9554 power
   expander at I2C 0x22:
     eFuse1 — internal 5 V rail   (EN=P7, V=P6, FLT=P5)
     eFuse2 — external 5 V..20 V  (EN=P0, V=P1, FLT=P2)
   EN is active-high, V (power-good) active-high, FLT active-low. */

typedef struct {
    bool enabled;          /* current EN output state (tracked)            */
    bool fault;            /* a confirmed fault: FLT held 30 ms and not reverse-current blocking */
    bool flt_pin;          /* the FLT pin asserted right now (active-low input) */
    uint32_t reverse_blocks; /* held FLTs that were reverse-current blocking, since boot */
    bool valid;            /* pin 3 high: on the TPS259470A that is AUXOFF (input valid, inrush
                              done), not output power good */
    bool status_supported; /* true — V/FLT are wired via the expander      */
} target_power_status_t;

/* Configure the expander pins (EN outputs driven off, V/FLT inputs).
   Requires i2c_bus_init() first. */
void target_power_init(void);

/* Enable/disable an eFuse (efuse is 1 or 2).  Returns 0, -1 on bad index. */
int target_power_enable(int efuse, bool on);

/* Schedule an eFuse change after delay_ms (0 = immediate).  A new schedule for
   the same eFuse replaces a pending one.  Returns 0, -1 on bad index. */
int target_power_schedule(int efuse, bool on, uint32_t delay_ms);

/* Apply scheduled changes whose delay elapsed.  Call once per main-loop iter. */
void target_power_poll(void);

/* Fill *out with eFuse state.  efuse is 1 or 2.  Returns 0, -1 on error.
   Reads V/FLT over I2C, which only the hw worker task drives: do NOT call it from the
   net task (target_power_get_cached is the RAM-only read for that). */
int target_power_get_status(int efuse, target_power_status_t *out);

/* Read the tracked EN state and last-sampled FLT for an eFuse (1 or 2) WITHOUT any
   I2C — plain RAM reads of state target_power_enable()/poll() maintain, so it is safe
   to call from any task (e.g. the net task pushing an efuse.event on connect).
   enabled/fault may be NULL.  Returns 0, -1 on bad index. */
int target_power_get_cached(int efuse, bool *enabled, bool *fault);

/* Async eFuse-change notification.  Invoked (efuse is 1 or 2):
     - on a SUCCESSFUL target_power_enable() (immediate or delayed), with the new
       EN state and the current FLT for that eFuse;
     - on a rising FLT edge sampled in target_power_poll(), with enabled=current
       EN state and fault=true.
   Kept cloud-agnostic: cloud_client registers a callback that queues an
   efuse.event WS frame.  The callback may run on either the console or net task,
   so it must be brief and do its own cross-task guarding. */
typedef void (*target_power_event_fn)(int efuse, bool enabled, bool fault);
void target_power_set_event_cb(target_power_event_fn cb);

/* What a held FLT means. The TPS259470A pulls FLT low for real faults (overtemperature, a
   persistent overcurrent in current limit, ILM open or shorted) and ALSO while it blocks reverse
   current: when VOUT - VIN > 29 mV (VREVTH) it turns its blocking FET off within 1 us and only
   turns it on again once VIN - VOUT > 104 mV (VFWDTH; datasheet SLVSFC9C, 7.3.7 and Table 7-3).
   With a light load on the output, a dip of the input rail (the pod's +5V_IN when a Wi-Fi bridge
   on the USB-A port transmits) leaves VOUT above VIN, and FLT stays low until VOUT has leaked
   down 104 mV: tens of ms with no load. That is no fault. The check gets the eFuse (1 or 2)
   and returns true for a real fault, false for reverse-current blocking; without a check (host
   tests) every held FLT is a fault. It runs on the hw worker (I2C). */
typedef bool (*target_power_flt_check_fn)(int efuse);
void target_power_set_flt_check(target_power_flt_check_fn fn);
/* How many held FLTs the check called reverse-current blocking, per eFuse since boot. */
uint32_t target_power_reverse_blocks(int efuse);

#endif /* TARGET_POWER_H */
