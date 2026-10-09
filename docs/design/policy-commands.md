# Pod policies: signature policy, LAN policy, tunnel tier limit

Status: built and released in firmware 3.6.0 (2026-10-07), with the server and CLI changes; all
three parts below shipped. This was the spec for branch `policy-enforce`, 2026-10-06.
Builds on [firmware-signing.md](firmware-signing.md) and [access-control.md](access-control.md).

Backward compatibility rules, for all three parts:

- Firmware defaults never lock anyone out: `sig_policy` defaults to `audit`, `lan_policy` to `open`,
  and a tunnel without `max_tier` gets everything (as today).
- A pod advertises each feature with a capability flag. Servers and CLIs only use a feature on a pod
  that advertises it; older pods behave exactly as before.
- The USB console can always change both policies. It is the way back.
- Every refusal is a normal JSON error reply (`{"status":"error","message":...}`) before anything is
  changed. Nothing half-applies.

## Command tiers

`src/cmd_tier.c` (firmware) and `api/benchpod_command_tier.go` (server) classify every JSON verb:
T0 read, T1 instrument control, T2 persisted config, T3 firmware. `sig_policy` and `lan_policy`
are T2 when they set something, T0 when they only read.

## 1. Signature policy

Values, in order of strictness: `audit` < `permissive` < `required` (firmware-signing.md).
Persisted in the pod's config flash. Missing record = `audit`.

JSON (LAN, cloud command channel, cloud tunnel):

    {"cmd":"sig_policy"}                      -> {"status":"ok","data":{"policy":"audit","enforces":true,"keys":3}}
    {"cmd":"sig_policy","set":"required"}     -> same reply with the new policy

Who may set it:

| Transport | May set |
|---|---|
| USB console | anything (the way back) |
| Cloud (command channel or tunnel) | only a stricter value than the current one ("ratchet") |
| LAN TCP | nothing: `sig_policy: change it from the cloud or the USB console` |

Loosening from the cloud: `sig_policy: only the USB console can loosen the policy (now required)`.

Console: `sig-policy` prints `sig-policy <policy> <keys>`; `sig-policy <audit|permissive|required>`
sets it and prints the same line, or `sig-policy error <why>`.

Capabilities frame and JSON `status`: `"sig_policy":"<policy>"` (exists since the report-only
release) and new `"sig_policy_cmd":true`.

Firmware that has this command sets `FW_INFO_FLAG_ENFORCES_SIG` in its fw_info block. A pod on
`required` refuses firmware without that flag (the downgrade guard).

## 2. LAN policy

Values: `open` (today), `locked` (LAN TCP gets T0 and T1 only), `off` (no LAN TCP listener and no
mDNS advertisement). Persisted in config flash. Missing record = `open`.

JSON:

    {"cmd":"lan_policy"}                    -> {"status":"ok","data":{"policy":"open"}}
    {"cmd":"lan_policy","set":"locked"}     -> same reply with the new policy

Who may set it: the USB console and the cloud (command channel or tunnel), both directions. LAN TCP
never: `lan_policy: change it from the cloud or the USB console`.

Effect, immediately:

- `locked`: a T2 or T3 command on a LAN TCP connection gets
  `locked: <verb> needs the cloud or the USB console`. SCPI is T0/T1 only and keeps working.
- `off`: the TCP 8080 listener stops, open LAN connections close, mDNS stops advertising
  `_benchpod._tcp`. The cloud link and USB are unaffected. Setting `open` or `locked` again starts the
  listener and mDNS again. The reply to `set:"off"` goes out before the listener stops.

Console: `lan-policy` prints `lan-policy <policy>`; `lan-policy <open|locked|off>` sets it.

Capabilities and JSON `status`: `"lan_policy":"<policy>"`, `"lan_policy_cmd":true`.

## 3. Tunnel tier limit

The server is the policy point for cloud users (role gate, access-control.md section 5). Cloud tunnels
carry raw JSON lines from the user to the pod, so the server cannot see every command. It tells the pod
the highest tier the user behind a tunnel may use:

    {"type":"tunnel.open","tunnel_id":"...","max_tier":1}

`max_tier` 0..3; missing = 3 (today's behavior, and what old servers send). A command above it on that
tunnel gets `forbidden: <verb> needs an organization owner or admin`. The firmware advertises
`"tunnel_max_tier":true` in its capabilities. On a pod without it, the server must not offer
T2/T3-capable tunnels to callers who would be denied; it can only gate the REST command and OTA
routes.

## 4. Server role gate (enforced)

T2/T3 commands (REST `POST /benchpod/devices/{id}/command`, the cloud command routes, calibrate) and
every OTA route need:

- a web session of an organization `owner` or `admin`, or
- an API key with the `benchpod:admin` scope whose owner is still an org owner or admin.

Anything else gets HTTP 403 `this needs an organization owner or admin (API keys: the benchpod:admin
scope)`. Tunnels opened for other callers carry `max_tier: 1`.

## 5. Require signed firmware (org setting)

Organization setting `require_signed_firmware`, default off, changeable by owners/admins. When on, the
server sends `{"cmd":"sig_policy","set":"required"}` to a pod of that org when all of these hold:

- the pod advertises `sig_policy_cmd`,
- the pod is not already `required`,
- the pod has reported at least one firmware or blob install whose own check said `sig: ok`,
- the latest published firmware release is fully signed.

Turning the setting off cannot loosen pods (ratchet); the UI says so and points to the USB console.
