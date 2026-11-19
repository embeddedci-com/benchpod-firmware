#ifndef LEASE_LOCAL_H
#define LEASE_LOCAL_H

/*
 * lease_local: the pod's own lease, taken by a client on any transport (LAN, USB network, the
 * cloud tunnels and command channel) with lease_acquire, and kept in RAM only (a reboot clears it).
 *
 * One lease: an id (the token a client shares with its child processes), a holder label, a
 * purpose and a deadline. Every connection that sent lease_acquire with the held id is an owner.
 * While the lease is held, driving commands (anything but cmd_tier_light()) from any other
 * connection are refused as busy (BP0025); the lease_* verbs themselves always reach their
 * handler. An owner's connection closing drops its owner flag, not the lease: the lease ends at
 * its deadline, on lease_release, or on a lease.break frame from the server, after which every
 * owner's next command fails once with "lease lost" (BP0026).
 *
 * A client that opens a connection per command (the SDK's TCP transport) proves ownership per
 * command instead: any command may carry "lease":"<id>". The held id makes that command (and its
 * connection) an owner; any other id counts as no id. A command never moves the deadline: only
 * lease_acquire with the held id and lease_renew do.
 *
 * A broken lease stays broken: its id (and who broke it) is kept for LEASE_LOCAL_BROKEN_S after
 * the break, and lease_acquire, lease_renew or any command carrying it gets "lease lost" again,
 * so a run that reconnects cannot take the pod back by accident. An expired lease is not kept:
 * its id may be acquired again.
 *
 * The cloud lease mirror (lease_gate.h) is separate and unchanged: lease_acquire while it is
 * held is refused as busy with the cloud holder (command_handler_lease.c checks it).
 *
 * Tasks: everything but lease_local_break and lease_local_snapshot runs on the hw worker (the
 * writer). lease_local_break (net task) only posts a request that the writer applies at its next
 * call. lease_local_snapshot (net task, the upstream lease.local report) reads under a sequence
 * counter and gives up rather than spin when the writer is mid-update.
 *
 * Times are milliseconds from a caller-supplied clock, so this file builds on the host
 * (test/test_lease_local.c).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LEASE_LOCAL_ID_MAX       65u    /* 64 chars + NUL */
#define LEASE_LOCAL_HOLDER_MAX   41u    /* 40 chars + NUL */
#define LEASE_LOCAL_PURPOSE_MAX  65u    /* 64 chars + NUL */
#define LEASE_LOCAL_TTL_MIN_S    10u
#define LEASE_LOCAL_TTL_MAX_S    1800u
#define LEASE_LOCAL_TTL_DEFAULT_S 120u  /* lease_acquire / lease_renew without ttl_s */
#define LEASE_LOCAL_MAX_CONN     32     /* connection ids 0..31 can own it */
#define LEASE_LOCAL_BROKEN_S     1800u  /* a broken id is refused this long (the longest ttl) */

typedef enum {
    LEASE_LOCAL_OK = 0,
    LEASE_LOCAL_BUSY,       /* another id holds it */
    LEASE_LOCAL_NOT_HELD,   /* renew: nothing is held */
    LEASE_LOCAL_BAD_ID,     /* id empty, too long or not printable ASCII */
    LEASE_LOCAL_LOST,       /* the id of a lease the server broke: lease_local_broken_by() says who */
} lease_local_rc_t;

typedef struct {
    bool     held;
    bool     mine;                              /* the asking connection is an owner */
    uint32_t left_s;                            /* seconds left, rounded up */
    char     id[LEASE_LOCAL_ID_MAX];
    char     holder[LEASE_LOCAL_HOLDER_MAX];
    char     purpose[LEASE_LOCAL_PURPOSE_MAX];
} lease_local_view_t;

/* The ttl a request asks for (0 = not given) clamped to LEASE_LOCAL_TTL_MIN_S..MAX_S. */
uint32_t lease_local_clamp_ttl(uint32_t ttl_s);

/* lease_acquire: take a free lease, or refresh the held one when `id` matches; either way `conn`
   becomes an owner. Another id while held: LEASE_LOCAL_BUSY. `view` (may be NULL) gets the lease
   after the call (the holder's on BUSY). */
lease_local_rc_t lease_local_acquire(int conn, const char *id, const char *holder,
                                     const char *purpose, uint32_t ttl_s, uint32_t now_ms,
                                     lease_local_view_t *view);
/* lease_renew: push the deadline of the held lease with this id. Ownership does not change. */
lease_local_rc_t lease_local_renew(int conn, const char *id, uint32_t ttl_s, uint32_t now_ms,
                                   lease_local_view_t *view);
/* lease_release: free the held lease with this id. OK when nothing is held (a broken id too);
   BUSY for another id. */
lease_local_rc_t lease_local_release(int conn, const char *id, uint32_t now_ms,
                                     lease_local_view_t *view);

/* A connection closed: it is no longer an owner (the lease itself stays). */
void lease_local_conn_closed(int conn);

/* Writer, every poll: end a lease past its deadline. */
void lease_local_expire(uint32_t now_ms);

/* The lease as `conn` sees it. Returns view->held. */
bool lease_local_get(int conn, uint32_t now_ms, lease_local_view_t *view);

typedef enum {
    LEASE_LOCAL_PASS = 0,
    LEASE_LOCAL_REFUSE_BUSY,   /* someone else holds it: view has holder, purpose, left_s */
    LEASE_LOCAL_REFUSE_LOST,   /* this owner's lease was broken: by[] has who (reported once) */
} lease_local_gate_t;

/* The gate dispatch_line runs for every command: `lease_verb` (a lease_* command) is never
   refused as busy, `light` (cmd_tier_light) is never refused as busy. `cmd_id` is the command's
   "lease" field (NULL or "" = none): the held id passes and makes `conn` an owner. "Lease lost"
   comes first: once per owner connection after a break, and for every command carrying the
   broken id (lease verbs included; their own id is checked by acquire/renew). */
lease_local_gate_t lease_local_check(int conn, bool lease_verb, bool light, const char *cmd_id,
                                     uint32_t now_ms, lease_local_view_t *view, char *by,
                                     size_t by_cap);

/* Who broke the lease that LEASE_LOCAL_LOST / LEASE_LOCAL_REFUSE_LOST is about. */
const char *lease_local_broken_by(void);

/* Server frame lease.break {by}: end the lease. Any task: the writer applies it at its next call.
   The owners at that moment get one LEASE_LOCAL_REFUSE_LOST each. */
void lease_local_break(const char *by);

/* For the upstream report (any task): a counter that moves on every change of held, id, holder
   or purpose (acquire of a free lease, release, expiry, break). */
uint32_t lease_local_generation(void);
/* A consistent copy of the lease for the net task. false = the writer was mid-update: try again at
   the next poll. On true, *gen is the generation the copy belongs to. */
bool lease_local_snapshot(uint32_t now_ms, lease_local_view_t *view, uint32_t *gen);

/* Host tests: forget everything. */
void lease_local_reset(void);

#endif /* LEASE_LOCAL_H */
