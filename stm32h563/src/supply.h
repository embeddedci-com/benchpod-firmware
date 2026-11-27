#ifndef SUPPLY_H
#define SUPPLY_H

#include <stdbool.h>
#include <stddef.h>

/* ============================================================================
 * supply — the one name for what the DUT gets from the pod.  Pure (no hardware), host-tested.
 *
 *   "5v"        the pod's 5 V through eFuse 1 (the DUT INT terminal); metered
 *   "external"  the user's own supply through eFuse 2 (the DUT EXT terminal); metered
 *   "3v3"       the switched 3.3 V on J1 pin 17, BenchPod v4 only; not metered
 *   "1v8"       the switched 1.8 V on J1 pin 18, BenchPod v4 only; not metered
 *
 * The legacy request key `efuse` (1 or 2) means "5v" or "external".  Commands (target_power,
 * power_profile) refuse a request that has both keys; a wiring profile may have both when they
 * agree.  Every place that reads a request or a profile's supply goes through these functions.
 * ==========================================================================*/

typedef enum { SUPPLY_5V = 0, SUPPLY_EXTERNAL = 1, SUPPLY_3V3 = 2, SUPPLY_1V8 = 3 } supply_t;

#define SUPPLY_LIST "5v, external, 3v3, 1v8"

/* The wire name; "?" for a value outside the enum. */
const char *supply_name(supply_t s);
/* Parse a wire name (exactly one of the four).  -1 when unknown or NULL. */
int         supply_from_name(const char *name);
/* The eFuse index of 5v (1) and external (2); 0 for 3v3 and 1v8. */
int         supply_efuse(supply_t s);
/* 5v -> 1, external -> 2 (the legacy efuse number) as a supply; -1 for anything else. */
int         supply_from_efuse(int efuse);
/* 3v3 or 1v8: the switched J1 supplies (only boards with board_caps()->supply_3v3_1v8). */
bool        supply_is_j1(supply_t s);

/* Whether this pod has the supply: 5v and external always, 3v3 and 1v8 only with
   board_caps()->supply_3v3_1v8 (has_j1). */
bool        supply_on_board(supply_t s, bool has_j1);

typedef enum {
    SUPPLY_REQ_OK = 0,        /* *out is set */
    SUPPLY_REQ_ABSENT,        /* neither key given */
    SUPPLY_REQ_BOTH,          /* both given where only one is allowed (commands) */
    SUPPLY_REQ_CONFLICT,      /* both given and they name different supplies (profiles) */
    SUPPLY_REQ_BAD_SUPPLY,    /* supply is not one of the four */
    SUPPLY_REQ_BAD_EFUSE,     /* efuse is not 1 or 2 */
} supply_req_t;

/* Resolve a request's two keys.  supply_s / efuse_s are the raw value texts, NULL when the key is
   absent.  agree_ok: both keys are allowed when they name the same supply (a wiring profile);
   false: both is SUPPLY_REQ_BOTH (a command). */
supply_req_t supply_resolve(const char *supply_s, const char *efuse_s, bool agree_ok, supply_t *out);

#endif /* SUPPLY_H */
