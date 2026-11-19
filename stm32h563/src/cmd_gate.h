#ifndef CMD_GATE_H
#define CMD_GATE_H

/*
 * cmd_gate — the checks dispatch_line() runs before a JSON command reaches its handler, as pure
 * functions of the command and a snapshot of the pod's state, so they are host-tested
 * (test_cmd_gate.c).  command_handler.c fills the snapshot (policy source, LAN policy, lease,
 * tunnel max tier, safe mode, board variant) and sends the refusal this returns.
 *
 * Before these, cmd_gate_client_api: a command from a client older than the firmware's
 * min_client_api is refused with BP0030 client_too_old (features.toml), on every JSON transport.
 *
 * Order, first refusal wins (docs/design/policy-commands.md):
 *   1. tier: a LAN connection under a locked LAN policy gets T0/T1 only
 *   2. lease: while a cloud job holds the pod, a LAN connection gets light reads only
 *   3. tunnel: a cloud tunnel never goes above the tier the server allowed its user
 *   4. safe mode: with the iCE40/PSRAM off, only the commands that need neither
 *   5. board: the digital-only board refuses analog commands (an ADC capture_dual included)
 *
 * The DAC output limits (dac_limits_check_command) run after these, in dispatch_line.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cmd_table.h"   /* cmd_desc_t */
#include "cmd_tier.h"
#include "pod_policy.h"

typedef struct {
    policy_src_t     src;              /* where the command came from */
    pod_lan_policy_t lan_policy;       /* pod_policy_lan(): the external LAN (Ethernet, Wi-Fi) */
    bool             usb_link;         /* a LAN connection on the USB network adapter: physical
                                          presence, so the LAN policy does not apply to it (it
                                          stays LAN tier: what the LAN never may, it never may) */
    bool             lease_active;     /* lease_gate_active(); only looked at for POLICY_SRC_LAN */
    uint32_t         lease_left_s;     /* seconds left on that lease */
    const char      *lease_holder;     /* lease_gate_holder() ("" or NULL = "cloud") */
    int              tunnel_max_tier;  /* a cloud tunnel's max tier, -1 = not a tunnel */
    bool             skip_hw;          /* boot_guard_skip_hw(): safe mode */
    bool             has_analog;       /* board_has_analog() */
} cmd_gate_ctx_t;

/* The start of the one refusal for a cloud job holding the pod. dispatch_line sends that one as
   lease_busy (BP0025) with holder "a cloud job" and the cloud holder as its purpose; hosts that
   predate the codes read the text ("busy: ..."), so it stays as it was. */
#define CMD_GATE_CLOUD_LEASE_HOLDER "a cloud job"
#define CMD_GATE_CLOUD_LEASE_TEXT   "busy: " CMD_GATE_CLOUD_LEASE_HOLDER " holds this pod"

/* NULL = `cmd` (its row `d` from cmd_find, NULL for an unknown verb, with its full JSON line
   `json`, already classified as `tier`) may run; else the refusal to send, either a constant or
   written into why[why_cap].  `cmd` is only the name the refusals quote. */
const char *cmd_gate_check(const cmd_desc_t *d, const char *cmd, const char *json, cmd_tier_t tier,
                           const cmd_gate_ctx_t *ctx, char *why, size_t why_cap);

/* The client API check (features.toml `min_client_api`): true = `json` (row `d`, NULL = unknown
   verb) may run for a firmware that takes client_api >= min_api. *sent is the command's
   "client_api" (top level; absent, negative or not a number = 0, a value past INT32_MAX is
   INT32_MAX). The rows with CMD_F_ANY_CLIENT (status, diag, ping, lease_release) always run. The
   firmware passes BP_MIN_CLIENT_API; tests pass any minimum. */
bool cmd_gate_client_api(const cmd_desc_t *d, const char *json, uint32_t min_api, int32_t *sent);

/* The refusal text for BP0030 (an old client shows it verbatim) into why[why_cap], with the
   command's optional "client" ("sdk 3.0.6", at most CMD_GATE_CLIENT_MAX characters, printable
   ASCII; anything else is left out). Returns why. */
#define CMD_GATE_CLIENT_MAX 32
const char *cmd_gate_client_api_text(const char *json, int32_t sent, uint32_t min_api,
                                     char *why, size_t why_cap);

/* Checks 4 and 5 alone (safe mode, the digital-only board), for the transports that have no
   tier, lease or tunnel of their own: SCPI asks with the JSON verb its command stands for
   (command_handler_device_gate). NULL = may run, else the refusal (a constant). */
const char *cmd_gate_device(const char *cmd, const char *json, bool skip_hw, bool has_analog);
/* The same for a row already looked up (NULL = unknown verb). */
const char *cmd_gate_device_row(const cmd_desc_t *d, const char *json, bool skip_hw, bool has_analog);

/* Commands that run with the iCE40/PSRAM off (safe mode). */
bool cmd_gate_ok_without_hw(const char *cmd);
/* Commands that need the analog front end. */
bool cmd_gate_needs_analog(const char *cmd);

/* A SCPI line with any non-query part (every ';'-separated part must hold a '?' to be a read).
   A LAN SCPI line like that is refused while a cloud job holds the pod. */
bool cmd_gate_scpi_line_writes(const char *line);

#endif /* CMD_GATE_H */
