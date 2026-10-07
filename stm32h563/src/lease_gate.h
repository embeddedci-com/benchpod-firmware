#ifndef LEASE_GATE_H
#define LEASE_GATE_H

/*
 * lease_gate — the LAN yields while a cloud consumer holds the pod (access-control.md F3,
 * docs/design/cloud-hardening.md section 2). Always on.
 *
 * The server sends lease.state frames: held (with a holder label and seconds left) on acquire and
 * every renew, not held on release. The pod keeps its own deadline, so a vanished server cannot
 * lock the LAN out for longer than the lease it last announced, and a dropped cloud link ends it
 * at once. While held, LAN connections may only run light reads (cmd_tier_light()).
 *
 * Times are milliseconds from a caller-supplied clock, so this file builds on the host.
 */

#include <stdbool.h>
#include <stdint.h>

#define LEASE_GATE_HOLDER_MAX 41      /* 40 chars + NUL */
#define LEASE_GATE_MAX_S      600u    /* never trust a longer lease than the server's maximum */

/* A lease.state frame: held with `expires_in_s` left, or not held. */
void lease_gate_update(bool held, const char *holder, uint32_t expires_in_s, uint32_t now_ms);
/* The cloud link dropped: whatever the server said no longer holds. */
void lease_gate_clear(void);

/* Is a lease in force at now_ms? *left_s (may be NULL) gets the seconds left, rounded up. */
bool lease_gate_active(uint32_t now_ms, uint32_t *left_s);
/* The holder label of the current lease ("" when none). */
const char *lease_gate_holder(void);

#endif /* LEASE_GATE_H */
