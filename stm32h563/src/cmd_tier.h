#ifndef CMD_TIER_H
#define CMD_TIER_H

/*
 * cmd_tier — how much a JSON command can change, for the access-control work
 * (docs/design/access-control.md section 3, docs/design/firmware-signing.md).
 *
 *   T0  read-only (including captures, which read the DUT)
 *   T1  instrument control: live pod or DUT state, nothing that outlives a reboot
 *   T2  persisted config, network identity, safety limits, calibration
 *   T3  firmware and gateware (OTA)
 *
 * dispatch_line logs the tier with each command and gates on it: a "locked" LAN policy refuses
 * T2/T3 on LAN connections, and a cloud tunnel refuses anything above its max_tier.
 *
 * The tier depends on the arguments for the verbs that read and write: `dac_limits` (a write
 * has "path" or "enabled":false), `calibrate` ("source" or "clear":true), `eth` (only
 * "stats" and "refclk" are reads), `sig_policy` and `lan_policy` (a write has "set"). An unknown verb counts as T3, so a new command is never
 * under-classified; test_cmd_tier fails when dispatch_line gains a verb this table lacks.
 */

typedef enum {
    CMD_TIER_T0 = 0,
    CMD_TIER_T1,
    CMD_TIER_T2,
    CMD_TIER_T3,
} cmd_tier_t;

/* The tier of `cmd` with its full JSON line `json` (may be NULL: the verb's base tier). */
cmd_tier_t cmd_tier(const char *cmd, const char *json);
/* Is `cmd` in the table at all? */
int cmd_tier_known(const char *cmd);
/* A light read: T0 and not one of the reads that take the capture hardware. The only thing a LAN
   client may run while a cloud job holds the pod (lease_gate.h). */
int cmd_tier_light(const char *cmd, const char *json);
const char *cmd_tier_name(cmd_tier_t t);   /* "T0".."T3" */

#endif /* CMD_TIER_H */
