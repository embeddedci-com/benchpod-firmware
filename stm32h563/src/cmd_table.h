#ifndef CMD_TABLE_H
#define CMD_TABLE_H

/*
 * cmd_table — the one table of JSON commands (defined in cmd_tier.c).
 *
 * Each row holds a verb, its tier (cmd_tier.h), what the gates need to know about it, its
 * handler and, for a verb whose tier depends on its arguments, the test for its write form.
 * dispatch_line() looks the verb up here once and hands the row on: the tier gate
 * (cmd_tier_of), the lease gates (cmd_tier_light_of), the safe-mode and digital-board gates
 * (cmd_gate), the cloud command channel (no streaming verbs) and the console trace (no noisy
 * polls) all read the same row, so a new command is one line here and nothing else.
 *
 * Host tests compile cmd_tier.c with CMD_TABLE_NO_HANDLERS: the rows then carry no handler, so
 * the table links without command_handler.c.
 */

#include <stddef.h>
#include <stdint.h>

#include "cmd_tier.h"

typedef void (*cmd_fn_t)(int conn_id, const char *json);
/* A mixed verb's write form (cmd_tier.h): nonzero when `json` (never NULL) asks to change
   something, which makes the command T2.  The row's tier is the read form's. */
typedef int (*cmd_writes_fn_t)(const char *json);

enum {
    /* Runs with the iCE40/PSRAM off (boot_guard safe mode): network config, status, target
       power, so the operator can see what happened and recover. Every other verb needs the
       hardware; leaving the flag out is the safe default. */
    CMD_F_NO_HW  = 1u << 0,
    /* Needs the analog front end (DAC, ADC, relays, the 4-20 mA terminals): refused on the
       digital-only board. */
    CMD_F_ANALOG = 1u << 1,
    /* Streams chunks or switches the connection into a raw protocol (SWD, UART, upload): cannot be
       carried over the single-reply cloud command channel. */
    CMD_F_STREAM = 1u << 2,
    /* A T0 read that takes the capture hardware (PSRAM, LA, ADC, sensor bus): not a light read,
       so a LAN client cannot run it while a cloud job holds the pod (lease_gate.h). */
    CMD_F_HEAVY  = 1u << 3,
    /* Polled on a timer by the web UI: not traced on the serial console. */
    CMD_F_NOISY  = 1u << 4,
    /* Needs the analog front end only when it asks for ADC samples ("adc_samples" > 0):
       capture_dual, whose LA-only form runs on the digital-only board. */
    CMD_F_ADC_SAMPLES = 1u << 5,
    /* lease_acquire / lease_renew / lease_release (lease_local.h): always reach their handler, so a
       busy lease answers with its holder instead of a gate refusal. */
    CMD_F_LEASE  = 1u << 6,
    /* Runs for a client below min_client_api (cmd_gate_client_api): what a client needs to find
       out that it is too old and to say so, and to let go of the pod. Keep the list short. */
    CMD_F_ANY_CLIENT = 1u << 7,
    /* Answers a LAN computer that is not paired with the pod (lan_clients.h, cmd_gate.h): the
       few reads that tell it what the pod is and how to pair. Everything else needs pairing. */
    CMD_F_UNPAIRED = 1u << 8,
};

/* The {name, tier} pair is what embeddedci-server mirrors (testdata/shared/cmd_tier_vectors.json)
   and its api/shared_vectors_test.go reads out of cmd_tier.c as `{ "verb", CMD_TIER_Tn }`, so the
   pair keeps its own braces. */
typedef struct {
    struct {
        const char *name;
        uint8_t     tier;   /* cmd_tier_t */
    } key;
    uint16_t flags;         /* CMD_F_* */
    cmd_fn_t fn;            /* NULL under CMD_TABLE_NO_HANDLERS */
    cmd_writes_fn_t writes; /* NULL: the tier does not depend on the arguments */
} cmd_desc_t;

/* The row for `name`, NULL for an unknown verb. */
const cmd_desc_t *cmd_find(const char *name);
/* Every row, for tests and listings. */
const cmd_desc_t *cmd_table_rows(size_t *count);

/* The tier of row `d` (NULL = unknown verb, T3) with its full JSON line `json`. */
cmd_tier_t cmd_tier_of(const cmd_desc_t *d, const char *json);
/* cmd_tier_light() for row `d` (NULL = unknown verb: not light). */
int cmd_tier_light_of(const cmd_desc_t *d, const char *json);

static inline int cmd_has(const cmd_desc_t *d, unsigned flag) { return d && (d->flags & flag); }

#endif /* CMD_TABLE_H */
