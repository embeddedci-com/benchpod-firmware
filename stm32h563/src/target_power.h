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
    bool fault;            /* FLT asserted (active-low input)              */
    bool valid;            /* V/power-good asserted (active-high input)    */
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
   Reads V/FLT over I2C, so callers must serialize it with hw_lock (see
   target_power_poll's note) — do NOT call it from the net task. */
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

#endif /* TARGET_POWER_H */
