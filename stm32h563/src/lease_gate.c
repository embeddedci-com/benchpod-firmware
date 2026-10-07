/*
 * lease_gate.c — see lease_gate.h.
 */
#include "lease_gate.h"

#include <string.h>

static volatile bool     s_held;
static volatile uint32_t s_deadline_ms;
static char              s_holder[LEASE_GATE_HOLDER_MAX];

void lease_gate_update(bool held, const char *holder, uint32_t expires_in_s, uint32_t now_ms) {
    if (!held || expires_in_s == 0) { lease_gate_clear(); return; }
    if (expires_in_s > LEASE_GATE_MAX_S) expires_in_s = LEASE_GATE_MAX_S;
    /* A label is for people; keep printable ASCII only, so it is safe in any reply. */
    size_t n = 0;
    for (const char *p = holder ? holder : ""; *p && n < sizeof(s_holder) - 1; p++)
        s_holder[n++] = (*p >= 0x20 && *p < 0x7f && *p != '"' && *p != '\\') ? *p : '?';
    s_holder[n] = '\0';
    s_deadline_ms = now_ms + expires_in_s * 1000u;
    s_held = true;
}

void lease_gate_clear(void) {
    s_held = false;
    s_holder[0] = '\0';
}

bool lease_gate_active(uint32_t now_ms, uint32_t *left_s) {
    if (!s_held) { if (left_s) *left_s = 0; return false; }
    int32_t left = (int32_t)(s_deadline_ms - now_ms);   /* wrap-safe */
    if (left <= 0) { lease_gate_clear(); if (left_s) *left_s = 0; return false; }
    if (left_s) *left_s = ((uint32_t)left + 999u) / 1000u;
    return true;
}

const char *lease_gate_holder(void) { return s_holder; }
