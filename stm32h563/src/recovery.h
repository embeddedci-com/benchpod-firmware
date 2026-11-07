#ifndef RECOVERY_H
#define RECOVERY_H

/*
 * recovery — when the pod gets stuck (an ESD hit that hangs the iCE40 or wedges a bus), it gets
 * itself running again, so a failure that would need an operator becomes one it recovers from.
 * This file is the decision only (which step, when, how often), so it runs in the host tests;
 * the steps themselves are in the callers (emi_test.c for the EMI build's activity loop,
 * hw_worker.c for the idle iCE40 check).
 *
 * The ladder, for captures that keep failing (the iCE40 / PSRAM side):
 *   1. RECOVERY_ICE40_RELOAD after RECOVERY_FAIL_MS of failing: reset the iCE40 and let it read
 *      its bitstream from its flash again. Never a write: the flash is rewritten only by the boot
 *      self-test, and only when the image in it no longer matches a gateware slot.
 *      Again after RECOVERY_RELOAD_GAP_MS if it still fails, up to RECOVERY_RELOADS_PER_EPISODE.
 *   2. RECOVERY_MCU_RESET when the reloads did not help, RECOVERY_RELOAD_GAP_MS after the last:
 *      reset the STM32 (the iCE40 is reloaded at boot when it fails the self-test).
 *
 * For the Ethernet link that stays down: RECOVERY_ETH_RESTART after RECOVERY_LINK_DOWN_MS (reset
 * the PHY and ask DHCP again; harmless when the cable is just out). Never an STM32 reset for it.
 *
 * Limits, so a pod that stays broken shows red instead of cycling forever, and nothing wears:
 *   - at most RECOVERY_RELOADS_PER_HOUR iCE40 reloads in any hour;
 *   - at most RECOVERY_RESETS_MAX STM32 resets until the pod ran RECOVERY_HEALTHY_MS without a
 *     failure, or was powered off and on (the count survives the resets, recovery_noinit);
 *   - at most RECOVERY_ETH_PER_HOUR PHY restarts in any hour, RECOVERY_ETH_GAP_MS apart.
 * A hard hang of the STM32 itself is the independent watchdog's (watchdog.h).
 */

#include <stdbool.h>
#include <stdint.h>

#define RECOVERY_FAIL_MS             10000u   /* captures failing this long: start the ladder */
#define RECOVERY_RELOAD_GAP_MS       10000u   /* between reloads, and before the reset */
/* So failing captures from t0 get reloads at t0+10 s and t0+20 s and the reset at t0+30 s; the
   boot and the EMI loop's quick start after a recovery reset (EMI_TEST_RECOVERY_START_MS) have it
   running again by about t0+45 s, inside the 60 s of the lab's criterion B. (Was 30 s apart:
   the reset came at t0+70 s.) */
#define RECOVERY_RELOADS_PER_EPISODE 2u
#define RECOVERY_RELOADS_PER_HOUR    6u
#define RECOVERY_RESETS_MAX          2u
#define RECOVERY_HEALTHY_MS          (10u * 60u * 1000u)
#define RECOVERY_LINK_DOWN_MS        20000u
#define RECOVERY_ETH_GAP_MS          60000u
#define RECOVERY_ETH_PER_HOUR        10u
#define RECOVERY_HOUR_MS             (60u * 60u * 1000u)

typedef enum {
    RECOVERY_NONE = 0,
    RECOVERY_ICE40_RELOAD,
    RECOVERY_MCU_RESET,
    RECOVERY_ETH_RESTART,
} recovery_action_t;

typedef struct {
    /* captures */
    uint32_t episode_reloads;      /* reloads since the captures started failing */
    uint32_t last_reload_ms;
    uint32_t reload_times[RECOVERY_RELOADS_PER_HOUR];   /* ring of the last reloads */
    uint32_t reload_n;             /* reloads ever (index into the ring) */
    uint32_t healthy_since_ms;     /* when captures last started working */
    bool     healthy_valid;
    /* Ethernet */
    uint32_t last_eth_ms;
    uint32_t eth_times[RECOVERY_ETH_PER_HOUR];
    uint32_t eth_n;
    /* what was refused for a limit, for the log (once per episode) */
    bool     limit_noted;
} recovery_t;

void recovery_init(recovery_t *r, uint32_t now_ms);

/* One check. fail_ms: how long captures have been failing without one success (0 = working).
   link_down_ms: how long the Ethernet link has been down (0 = up, or not expected).
   *resets: the persisted reset count; cleared here after RECOVERY_HEALTHY_MS of health.
   Returns the step to take now; the caller does it and calls recovery_done. */
recovery_action_t recovery_poll(recovery_t *r, uint32_t now_ms, uint32_t fail_ms,
                                uint32_t link_down_ms, uint32_t *resets);

/* The caller took `a` at now_ms (counts it against the limits). */
void recovery_done(recovery_t *r, recovery_action_t a, uint32_t now_ms, uint32_t *resets);

/* True once per failure episode when the ladder is out of steps (limits reached): log it. */
bool recovery_exhausted(recovery_t *r, uint32_t now_ms, uint32_t fail_ms, uint32_t resets);

const char *recovery_action_name(recovery_action_t a);

#endif /* RECOVERY_H */
