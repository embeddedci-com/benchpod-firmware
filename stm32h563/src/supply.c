/* supply.c — see supply.h. */
#include "supply.h"

#include <stdlib.h>
#include <string.h>

static const char *const k_names[] = { "5v", "external", "3v3", "1v8" };

const char *supply_name(supply_t s) { return (unsigned)s < 4u ? k_names[s] : "?"; }

int supply_from_name(const char *name)
{
    if (!name) return -1;
    for (int i = 0; i < 4; i++)
        if (strcmp(name, k_names[i]) == 0) return i;
    return -1;
}

int supply_efuse(supply_t s) { return s == SUPPLY_5V ? 1 : s == SUPPLY_EXTERNAL ? 2 : 0; }

int supply_from_efuse(int efuse) { return efuse == 1 ? SUPPLY_5V : efuse == 2 ? SUPPLY_EXTERNAL : -1; }

bool supply_is_j1(supply_t s) { return s == SUPPLY_3V3 || s == SUPPLY_1V8; }

bool supply_on_board(supply_t s, bool has_j1) { return !supply_is_j1(s) || has_j1; }

supply_req_t supply_resolve(const char *supply_s, const char *efuse_s, bool agree_ok, supply_t *out)
{
    int a = -1, b = -1;
    if (supply_s) {
        a = supply_from_name(supply_s);
        if (a < 0) return SUPPLY_REQ_BAD_SUPPLY;
    }
    if (efuse_s) {
        char *end = NULL;
        long v = strtol(efuse_s, &end, 10);
        b = (end != efuse_s && *end == '\0') ? supply_from_efuse((int)v) : -1;
        if (b < 0) return SUPPLY_REQ_BAD_EFUSE;
    }
    if (a >= 0 && b >= 0) {
        if (!agree_ok) return SUPPLY_REQ_BOTH;
        if (a != b) return SUPPLY_REQ_CONFLICT;
    }
    if (a < 0 && b < 0) return SUPPLY_REQ_ABSENT;
    if (out) *out = (supply_t)(a >= 0 ? a : b);
    return SUPPLY_REQ_OK;
}
