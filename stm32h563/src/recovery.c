/*
 * recovery.c — the recovery ladder's decisions (recovery.h). Pure: no hardware, no clock.
 */
#include "recovery.h"

#include <string.h>

void recovery_init(recovery_t *r, uint32_t now_ms) {
    memset(r, 0, sizeof(*r));
    r->healthy_since_ms = now_ms;
}

static uint32_t in_last_hour(const uint32_t *ring, uint32_t size, uint32_t n, uint32_t now_ms) {
    uint32_t c = 0;
    const uint32_t have = n < size ? n : size;
    for (uint32_t i = 0; i < have; i++)
        if (now_ms - ring[i] < RECOVERY_HOUR_MS) c++;
    return c;
}

static bool reload_allowed(const recovery_t *r, uint32_t now_ms) {
    return in_last_hour(r->reload_times, RECOVERY_RELOADS_PER_HOUR, r->reload_n, now_ms) < RECOVERY_RELOADS_PER_HOUR;
}

/* The reset's turn: the episode's reloads are done (or none is allowed now), and the last step
   was RECOVERY_RELOAD_GAP_MS ago (with no reload, failing that much longer). */
static bool reset_due(const recovery_t *r, uint32_t now_ms, uint32_t fail_ms) {
    if (r->episode_reloads == 0) return fail_ms >= RECOVERY_FAIL_MS + RECOVERY_RELOAD_GAP_MS;
    return now_ms - r->last_reload_ms >= RECOVERY_RELOAD_GAP_MS;
}

recovery_action_t recovery_poll(recovery_t *r, uint32_t now_ms, uint32_t fail_ms,
                                uint32_t link_down_ms, uint32_t *resets) {
    if (fail_ms == 0) {
        if (!r->healthy_valid) {
            r->healthy_valid = true;
            r->healthy_since_ms = now_ms;
            r->episode_reloads = 0;
            r->limit_noted = false;
        }
        if (*resets && now_ms - r->healthy_since_ms >= RECOVERY_HEALTHY_MS) *resets = 0;
    } else {
        r->healthy_valid = false;
    }

    if (fail_ms >= RECOVERY_FAIL_MS) {
        const bool more_reloads = r->episode_reloads < RECOVERY_RELOADS_PER_EPISODE && reload_allowed(r, now_ms);
        if (more_reloads) {
            if (r->episode_reloads == 0 || now_ms - r->last_reload_ms >= RECOVERY_RELOAD_GAP_MS)
                return RECOVERY_ICE40_RELOAD;
        } else if (*resets < RECOVERY_RESETS_MAX && reset_due(r, now_ms, fail_ms)) {
            return RECOVERY_MCU_RESET;
        }
    }

    if (link_down_ms >= RECOVERY_LINK_DOWN_MS &&
        (r->eth_n == 0 || now_ms - r->last_eth_ms >= RECOVERY_ETH_GAP_MS) &&
        in_last_hour(r->eth_times, RECOVERY_ETH_PER_HOUR, r->eth_n, now_ms) < RECOVERY_ETH_PER_HOUR)
        return RECOVERY_ETH_RESTART;
    return RECOVERY_NONE;
}

void recovery_done(recovery_t *r, recovery_action_t a, uint32_t now_ms, uint32_t *resets) {
    switch (a) {
    case RECOVERY_ICE40_RELOAD:
        r->episode_reloads++;
        r->last_reload_ms = now_ms;
        r->reload_times[r->reload_n % RECOVERY_RELOADS_PER_HOUR] = now_ms;
        r->reload_n++;
        break;
    case RECOVERY_MCU_RESET:
        (*resets)++;
        break;
    case RECOVERY_ETH_RESTART:
        r->last_eth_ms = now_ms;
        r->eth_times[r->eth_n % RECOVERY_ETH_PER_HOUR] = now_ms;
        r->eth_n++;
        break;
    default:
        break;
    }
}

bool recovery_exhausted(recovery_t *r, uint32_t now_ms, uint32_t fail_ms, uint32_t resets) {
    if (r->limit_noted || fail_ms < RECOVERY_FAIL_MS) return false;
    const bool more_reloads = r->episode_reloads < RECOVERY_RELOADS_PER_EPISODE && reload_allowed(r, now_ms);
    if (more_reloads || resets < RECOVERY_RESETS_MAX) return false;
    r->limit_noted = true;
    return true;
}

const char *recovery_action_name(recovery_action_t a) {
    switch (a) {
    case RECOVERY_ICE40_RELOAD: return "iCE40 reload";
    case RECOVERY_MCU_RESET:    return "STM32 reset";
    case RECOVERY_ETH_RESTART:  return "Ethernet restart";
    default:                    return "none";
    }
}
