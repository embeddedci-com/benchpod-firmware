# BenchPod access control: proposal

Status (firmware 3.7.0, 2026-10-08): the first phase of section 7 is built and released.
Firmware 3.6.0 shipped the command tiers (`cmd_tier.c`, server `api/benchpod_command_tier.go`),
option A `lan_policy` (`open|locked|off`, so C's `off` too), the server role gate and audit log,
option D signing ([firmware-signing.md](firmware-signing.md),
[policy-commands.md](policy-commands.md)), the host-bound cloud login for F2
(`benchpod-ws-auth:v2`) and the LAN yielding to cloud jobs for F3
([cloud-hardening.md](cloud-hardening.md)). Firmware 3.7.0 added the F2 quick fix: release
builds refuse `tls=false` in `cloud_set`. Not built: `lan_iface=eth`, option B and a `strict`
policy, retiring the v1 login (the server still accepts it), the rollback floor, B2, E and
secure boot.

The rest is the original proposal, written against firmware `main` at 054ca7c and
embeddedci-server `main` before any of this. Line numbers are from those trees.

## 1. Current state (verified in code)

| Surface | Where | Authentication today |
|---|---|---|
| JSON + SCPI over TCP 8080 | `net_server.c:61`, `tcp_bind(pcb, IP_ADDR_ANY, ...)` at `net_server.c:436`; protocol chosen by the first byte (`{` = JSON, else SCPI) at `command_handler.c:4096` | None. `dispatch_line` (`command_handler.c:3740`) only applies the safe-mode filter (`cmd_ok_without_hw`, `:3720`) and the DAC limits check (`:3757`) before the verb table (`:3766-3844`). |
| CMSIS-DAP bridge | Same TCP connection, entered by `dap_start` (`command_handler.c:2697`, `PROTO_DAP` at `:2750`) | None (inherits the connection). |
| mDNS | `net_server.c:703-718`: `_benchpod._tcp` with `id=<full Ed25519 pubkey>`, `port=8080`, on both eth and Wi-Fi netifs (`:749`, `:756`) | Public by design. |
| USB CDC console | `console.c`: text commands incl. `dfu`, `upload-begin/-commit` (OTA and blobs via `upload_rx.c`), `wifi-set`, `flash-ice40`, `reboot` | Physical presence only. The console never runs JSON verbs (the uncalled `command_handler_dispatch_console` bridge was removed), so USB has no JSON path; it shares the heavy gate and the sample pool with JSON. |
| Cloud WSS | `cloud_client.c`: pod dials out, TLS verified against embedded ISRG roots, Ed25519 challenge-response (`cl_sign_nonce`, `cloud_client.c:1125-1131`) | Strong, but see finding F2. Commands from the server go through the same `dispatch_line` (`command_handler.c:3880`; tunnel bytes via the per-connection state machine, `:4084`). |
| OTA (firmware, `gw0`, `gw1`, `esp`) | `command_handler_ota.c:51-125`, `ota.c:96-196`, `ota_commit.c:210` | SHA-256 integrity only. The hash comes from the same client that sends the image (`ota.c:102`). `blob_store.c:144` also checks only the caller's hash. |

Server side: API keys carry scopes (`api/scopes.go:10`, `requireScope` at `:38`). The raw
command route `POST /benchpod/devices/{id}/command` (`api/routes.go:181`) and the OTA route
(`:188`) need only `benchpod:control` plus org membership (`benchpodOwnsOrShares`,
`api/scope_benchpod.go:333`). `runBenchpodDeviceCommand` (`api/benchpod_command.go:118`)
filters nothing except the output-stage guard (`:140`). Org roles exist
(`requireOrgRole`, `api/server.go:432`) but are not used for device verbs. The OTA upload
accepts any uploaded body (`api/benchpod_ota.go:330-336`); its own comment says the device
"verifies only the SHA-256" (`:35-36`). The release workflow publishes only a `.sha256`
next to the binary (`.github/workflows/release-stm32.yml:126-129`).

`docs/API.md:35` already documents "The LAN API is unauthenticated" and asks users to put
the pod on a lab VLAN. That is acceptable for a small team; it is not acceptable as the only
control for a large company's shared lab network.

### Findings worth fixing regardless of the chosen option

- **F1. Any LAN host can install arbitrary firmware.** `ota_begin` + `ota_data` + `ota_end` +
  `ota_commit` from TCP 8080 replaces the STM32 image. That firmware then owns the device
  identity key (`device_identity.c:26`, a flash sector the app can read) and the cloud link.
  The compromise persists through any later policy change.
- **F2. `cloud_set` allows plaintext and the WS-auth signature does not bind the server name.**
  `handle_cloud_set` stores `tls=false` when asked (`command_handler.c:2380-2391`), and the
  pod signs only `"benchpod-ws-auth:v1" || 0 || nonce` (`cloud_client.c:1131`). A LAN attacker
  can point the pod at their own host, fetch a real challenge for that `device_id` from
  embeddedci.com, relay it, and log in to embeddedci.com as the pod. CI jobs would then receive
  attacker-made capture results. A valid Let's Encrypt certificate for the attacker's own domain
  works too, so forcing TLS alone does not fix it; binding the host into the signature does.
- **F3. LAN bypasses the cloud lease.** The DB-backed lease (`internal/db/benchpod_leases.go`)
  only arbitrates cloud clients. A colleague on the LAN can drive outputs in the middle of a
  leased CI run.
- **F4. Cloud members can run every verb.** Any org member or `benchpod:control` key can send
  `cloud_set`, `wifi_set`, `dac_limits {"enabled":false}` or an uploaded firmware image.

## 2. Threat model for a corporate lab network

**Who is on the network:** lab engineers and their PCs, shared CI runners (hwe2e, GitHub
self-hosted runners), contractors and visitors on the same VLAN, any compromised workstation
(the realistic attacker: malware with LAN reach), and, when Wi-Fi is configured, everyone on
that WLAN. Physical attackers (USB, SWD, BOOT0/DFU) are out of scope for this release: the H563
runs with TZEN=0 and no RDP (`dfu_boot.c:56`), so physical access already wins.

**Assets, in priority order:** (1) the pod's firmware and its identity key, (2) the hardware
attached to it (DUT, external output stages, power rails), (3) the integrity of test results
reported to CI, (4) confidentiality of the DUT (`spi_flash read`, SWD memory reads, UART logs
can expose pre-release customer firmware), (5) availability of the bench.

| Actor | Plausible action | Impact |
|---|---|---|
| Careless colleague | Runs a script against the wrong IP; clears `dac_limits`; sets `la_voltage` 3.3 V on a 1.8 V DUT; powers a rail mid-test | Damaged DUT or output stage, broken CI run |
| Careless colleague | `fpga_image`, `psram_recover`, `eth speed 10` | Bench unavailable until someone notices |
| Compromised PC | `ota_*` with a malicious image (F1) | Persistent implant; key theft; pivot to cloud |
| Compromised PC | `cloud_set` to own host (F2), `wifi_set` to a rogue AP | Pod impersonation, traffic capture |
| Compromised PC | `spi_flash read`, `dap_start`, `uart_proxy_start` | DUT firmware exfiltration |
| Passive sniffer | Reads plaintext TCP 8080 | Sees DUT data and any bearer secret (relevant to option B) |

**Destructive vs observational.** "Read-only" is not "harmless": reads leak DUT data and the
heavy captures hold the single hardware gate (DoS). But the irreversible harms come from
firmware writes and persisted configuration, and those are a small set of verbs. That is the
lever this proposal uses.

## 3. Command classification

Tiers: **T0** read-only, **T1** instrument control (changes live pod or DUT state, nothing that
outlives a reboot except what the instrument does to the DUT), **T2** destructive config
(persisted settings, network identity, safety limits, calibration), **T3** firmware (writes
code or gateware). Every verb in `dispatch_line` (`command_handler.c:3766-3843`):

| Tier | Verbs |
|---|---|
| T0 | `ping`, `status`, `cloud_status`, `wifi_status`, `la_pins`, `usb_cc`, `target_status`, `power_status`, `identity_public`, `identity_pop`, `spi_status`, `sensor_status`, `can_status`, `ota_status`, `blob_status`, `dac_loop_probe`; read forms of mixed verbs: `dac_limits` (no `path`/`enabled`), `calibrate` (no `source`/`clear`), `eth action=stats|refclk` |
| T0 (observational, leaks DUT data) | `capture`, `capture_dual`, `capture_read`, `stream`, `la_capture`, `sensor_regs`, `sensor_la`, `can_read`, `psram_ping`, `test` |
| T1 | `generate`, `measure`, `load`, `load_bin`, `replay`, `dac_stop`, `dac_set`, `dac_mux`, `cal_switch`, `analog_path`, `dac_out`, `current_out`, `adc_read` (flips relays), `dac_control_loop`, `dac_loop_input`, `la`, `gpio`, `la_voltage`, `nrst`, `target_power`, `power_profile`, `dap_start`, `uart_proxy_start`, `spi_start`, `spi_stop`, `spi_xfer`, `spi_stream`, `spi_flash` (DUT flash, all ops), `sensor_start`, `sensor_set`, `sensor_stop`, `can_config`, `can_write`, `can_term`, `can_respond`, `can_disable`, `speedtest`, `fpga_image` (selects one of the two installed images), `psram_recover` (reboot only) |
| T2 | `cloud_set`, `cloud_clear`, `wifi_set`, `wifi_clear`, `eth action=stop|start|restart|speed|loopback`, `dac_limits` set/clear, `calibrate` run/clear (persists, changes everyone's readings) |
| T3 | `ota_begin`, `ota_data`, `ota_end`, `ota_commit`, `ota_abort`, `ota_selftest` (erases a scratch flash sector) |

SCPI (`scpi_server.c:744-816`) has only T0 queries and T1 setters (`SOURce`, `OUTPut`,
`DIGital`, `OUTPut:POWer`), so it needs the T1 gate only. The USB console is physical access
and is treated as T3-capable.

Implementation note: the tier is a function of `(cmd, buf)`, not of `cmd` alone, because
`eth`, `dac_limits` and `calibrate` mix reads and writes. One table plus three sub-verb checks
replaces the ad hoc `cmd_ok_without_hw` list style. A unit test should fail when a verb is
added to `dispatch_line` without a tier.

## 4. Options

### A. Admin lock (persisted policy flag)

A persisted `lan_policy` in the config sector: `open` (today), `locked` (LAN gets T0 + T1;
T2/T3 return `"locked: <verb> needs the cloud or the USB console"`), and `off` (see C). The
check sits in `dispatch_line` next to the DAC limits check, keyed on the transport of
`conn_id` (TCP 0..4, console 5, cloud 6, tunnels 7..9, `command_handler.h:15-34`).

The policy can be changed only from the cloud command channel or the USB console, never from
the LAN. The web app shows a per-device toggle and an org default; the CLI gets
`benchpod lan-policy show|set` (cloud or USB).

- Pros: closes F1 and most of F2/F4 from the LAN with no client crypto; T0/T1 clients do not change.
- Cons: no LAN identity, so an admin on the LAN cannot do T2/T3 without the cloud or USB.
  Air-gapped labs need USB for every config change.
- Provisioning: a pod with no cloud config stays `open` (the CLI claims over LAN with
  `cloud_set`, `benchpod-cli/cmd/benchpod-cli/commands_cloud.go:349`). Once the server sees it
  connect, it pushes the org's default policy. `cloud_clear` is T2, so a locked pod cannot be
  reset to "unclaimed" from the LAN.
- Effort: firmware 3 days, server + web app 3 days, CLI/SDK error mapping 1 day.

### B. Per-pod LAN credential

Gives authorized LAN clients T2/T3 (and, in a `strict` policy, makes T1 require auth too).

**Provisioning.** The pod generates a 32-byte key K from the TRNG (`rng.c`).
- Cloud (default): the server sends `lan_key_rotate` over the authenticated WS, the pod returns
  K inside that TLS channel, the server stores it encrypted at rest. Users with the right role
  fetch it with their existing login or API key (`benchpod lan-key pull <pod>`, SDK
  `BENCHPOD_LAN_KEY`, MCP from its existing auth). Rotation on staff changes is one click.
- USB console: `lan-key show|rotate` for air-gapped labs.
- Printed label: rejected. A per-unit static secret cannot be rotated, gets photographed, and
  must be injected at the factory. At most a label could carry a one-time claim code.

**Protocol.** HMAC-SHA256 challenge-response per connection (SHA-256 is already linked):
`auth_hello` returns a pod nonce; the client sends a client nonce and
`HMAC(K, "benchpod-lan-auth:v1" || pod_nonce || client_nonce || role)`. The reply carries an
Ed25519 signature over the transcript under a new `benchpod-lan-auth:v1` context from the device
identity, so the client can pin the pod (public key from the cloud registry or mDNS `id=`).
Because TCP 8080 is plaintext, the handshake alone does not stop a man in the middle who
injects into an authenticated session. Derive a session key and require a per-line MAC
(`"seq"` + `"mac"` fields) on T2/T3 lines. That gives integrity without TLS.

A bearer token sent in the clear is not recommended: anyone on a mirror port or doing ARP
spoofing reads it once and keeps it. It protects only against the careless colleague, which
option A already does.

**Variant B2:** per-user Ed25519 client keys pushed from the cloud as an allow-list (up to 16
on the pod). Better audit and per-user revocation, about twice the work. Worth it later.

- Effort: firmware 5 days, server + web app 4 days, Python SDK + Go CLI + MCP 4 days, hwe2e 1 day.

### C. LAN API off, cloud-only, or bound to one interface

- `lan_policy=off`: do not call `server_start` (`net_server.c:432`) and drop the mDNS service.
  The pod stays fully usable through the cloud tunnels.
- Bind to an interface: `tcp_bind_netif` (lwIP 2.1) on the eth netif only, so a Wi-Fi uplink
  (often a corporate WLAN) never exposes 8080. Useful as `lan_iface=eth|wifi|both`.
- Pros: trivial (1 to 2 days), and the strongest LAN posture.
- Cons: LAN-only users lose access: hwe2e `TestHW` (`BENCHPOD_HW_ADDR`), local SCPI/VISA
  tools, and high-rate captures that are faster on the LAN than through the tunnel. Fits a
  customer that already drives everything through embeddedci.com.

### D. Signed firmware and blobs

This is the only option that also protects the cloud path (F4) and a compromised server
account, so it is the most valuable for a large customer.

- **Format.** A detached 128-byte signed manifest per asset: magic `BPSG`, format 1, target
  (`fw|gw0|gw1|esp`), size, SHA-256, monotonic build number, `key_id` (first 8 bytes of
  SHA-256 of the public key), Ed25519 signature (64 bytes) over the preceding fields.
- **Pod.** `ota_begin` gains `"sig":"<b64url>"` (console: an extra `upload-begin` argument;
  WS: a field in `ota.begin`). The pod checks the signature with Monocypher
  `crypto_ed25519_check` against embedded release keys and checks that `size`, `sha256` and
  `target` match the manifest, all before staging starts. The existing SHA-256 check at
  `ota_end` (`ota.c:191`) and the re-verify before the flash write (`ota_commit.c:210`) then
  bind the data to the signed hash. No new code runs over the 2 MB image; the cost is one
  verify of a ~64-byte message.
- **Blobs.** Pinning blobs to the running firmware's manifest (`blob_manifest.c`) is not
  enough: the server installs a release's blobs before its firmware when slots exist
  (`api/benchpod_ota.go:333-336`), so the new blobs never match the old firmware's manifest.
  Sign all four assets.
- **Keys and rotation.** Firmware embeds two release public keys (current and next). Each
  release may add the next key and drop a revoked one; the pod also persists a minimum key
  generation once it boots a release that raises it. Optional later: a persisted rollback floor
  on the build number.
- **Release CI.** `release-stm32.yml` gains a signing step after "Checksum". Preferred: a KMS
  key with Ed25519 support (for example Google Cloud KMS) reached through GitHub OIDC workload
  identity, gated by a protected `release` environment with required reviewers, so no private
  key sits in a GitHub secret. Publish `*.sig` next to each asset; the server's
  `FirmwareReleaseWatcher` downloads and forwards them. Uploaded `.bin` files in the web app
  need a `.sig` too.
- **Dev builds.** A build without `RELEASE=1` also trusts a developer key generated by
  `make dev-key` (kept in `~/.config/benchpod/`), and `make` signs its own output, so
  `install-blobs` and LAN OTA keep working on dev pods. Release firmware never trusts dev keys.
  To move a release pod to dev firmware, use USB DFU (`dfu` console command, `make flash-dfu`),
  which is physical presence. hwe2e OTA tests (`hwe2e/benchpod_ota_hw_test.go`) sign with the
  dev key.
- Limits: without H5 secure boot (OEM-iRoT and product state "closed") a physical attacker can
  still flash anything. Bring-up showed `BOOT_UBE=OEM-iRoT` is easy to get wrong, so that is a
  later, separate project.
- Effort: firmware 4 days, CI + key setup 2 days, server and CLI plumbing 3 days, tests 2 days.

### E. TLS on the LAN API

- **Memory.** Static RAM is ~631 KB of 640 KB (`.data`+`.bss`+`.noinit`, 96%). mbedTLS uses the
  248 KB FreeRTOS heap (`config/FreeRTOSConfig.h:33`), with ~197 KB free at runtime that already
  backs two cloud TLS sessions (`config/mbedtls_bench_config.h:18-28`). A server session needs
  the 16 KB input record buffer (`:138`; Python's `ssl` and Go's `crypto/tls` cannot negotiate a
  smaller max fragment length) plus 4 KB output and ~10-15 KB of context and handshake state:
  roughly 30-40 KB each. `NET_MAX_CONN` is 5 (`net_server.c:62`), so at most 1-2 TLS LAN
  sessions fit, and only when the bulk cloud WS is idle.
- **Code and CPU.** `MBEDTLS_SSL_SRV_C` is off (only `SSL_CLI_C`, `:125`); adding it costs
  ~20 KB flash. The H563 has no AES or PKA accelerator, so software AES-GCM at 250 MHz would
  cap LAN captures well below what the 100M link carries today, and a P-256 ECDHE + ECDSA
  handshake costs a few hundred milliseconds.
- **Keys.** mbedTLS cannot do Ed25519 in TLS, so the pod needs a second (P-256) key and a
  certificate: self-signed with the fingerprint pinned through the cloud, or issued by an
  embeddedci CA at claim time. Clients need pinning logic.
- Effort: 3 to 4 weeks including clients, and LAN throughput drops. Only worth it later as an
  optional `secure_lan` mode; the cloud tunnel already gives confidentiality.

### Comparison

| | A lock | B LAN credential | C LAN off / bind | D signing | E TLS |
|---|---|---|---|---|---|
| Stops LAN firmware implant (F1) | yes | yes (if T3 needs auth) | yes | yes, also via cloud | no by itself |
| Stops careless T1 mistakes | no | only in `strict` | yes | no | no |
| Protects cloud path (F4) | with server role gate | no | no | yes | no |
| Confidentiality on LAN | no | no | yes (no LAN) | no | yes |
| Client changes | error messages | SDK, CLI, MCP | none (use cloud) | signed releases | SDK, CLI, MCP |
| Effort (eng. days) | ~7 | ~14 | ~2 | ~11 | ~15-20 |

## 5. Server-side policy (pairs with A)

The pod trusts the authenticated cloud channel at T3, so the server is the policy point for
cloud traffic. Add a verb-tier table to `runBenchpodDeviceCommand` (mirroring the firmware
table, generated from one source) and require org `owner`/`admin` (`requireOrgRole`) or a new
`benchpod:admin` API-key scope for T2/T3 commands and for the OTA routes. Default API keys
keep `benchpod:control` only. Audit-log every T2/T3 command with the user or token. Also push
lease state to the pod (`lease_state`) so a `locked` pod refuses LAN T1 while a cloud lease is
held (F3).

## 6. Migration and compatibility

| Client | Impact | Plan |
|---|---|---|
| Python SDK, MCP, Go CLI (T0/T1) | none | Map the new `locked:` error to a typed exception with the fix in the message |
| CLI `cloud register` | needs `cloud_set` on LAN | allowed while unclaimed; afterwards via cloud |
| CLI `install-blobs`, `flash-self` | USB console and DFU: unaffected by A | send `.sig` once D lands (old firmware ignores the extra field) |
| MCP `calibrate` (run) over LAN | T2, refused when locked | works over `embeddedci:<name>` for admins |
| hwe2e LAN suite | uses `ota_*`, `dac_limits`, `calibrate` | bench pods stay `open` (dev builds default to `open`) |
| Older firmware | no `lan_policy` capability | server hides the toggle when the capability flag is absent and shows "update firmware to lock LAN access" |
| Older server, new firmware | does not push policy | pod keeps its stored policy; default `open` for release N, see below |
| Server WS auth (F2) | new `benchpod-ws-auth:v2` context signs `host || nonce` | server accepts v1 and v2 for two releases, then v1 only for allow-listed legacy devices |

Default policy: `open` for unclaimed pods in all builds; for claimed pods, the org default,
which the server sets to `locked` for new orgs and which this customer's org will have set.

## 7. Recommended phased plan

**This release (about 3 weeks of one engineer, firmware + server):**
1. Command tier table + unit test (1 day).
2. Option A `lan_policy` with `open|locked|off`, settable only from cloud and USB; server push,
   web app toggle, org default, CLI `lan-policy` (6 days). Includes C's `off` mode and
   `lan_iface=eth` because they are a few lines on top (1 day).
3. Server role gate for T2/T3 and OTA routes, plus audit log (3 days).
4. Option D signing for firmware and all blobs, KMS-backed CI step, dev-key flow (11 days).
5. F2 quick fix: release builds refuse `tls=false` in `cloud_set` (half a day). The v2 signature
   context lands with the server change in the same release if time allows, otherwise next.
6. Update `docs/API.md` Security model to describe tiers and policies.

**Next release:** option B (HMAC challenge-response with per-line MAC, cloud-distributed key,
USB fallback), `strict` policy, lease mirroring to the pod (F3), WS-auth v2 enforcement,
rollback floor.

**Later:** B2 per-user client keys, optional TLS LAN mode (E) on a pod with spare heap, H5 secure
boot and product state for physical-attack resistance, and customer-owned signing keys if the
customer asks to sign its own builds.
