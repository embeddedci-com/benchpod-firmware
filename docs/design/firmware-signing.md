# Signed firmware and blobs

Status: plan, 2026-10-06. Nothing implemented. Expands option D of
[access-control.md](access-control.md) and adds the rollout order within the access-control work.
Written against firmware 3.5.1 (`bcad10d`).

## Why

Every install path checks only a SHA-256 that the same client sends: cloud WS `ota.*`, LAN JSON
`ota_*` on TCP 8080, USB console `upload-*`, and the server's release install
(`api/benchpod_ota.go`). So any LAN host, or anyone who controls the server or an org account,
can install arbitrary firmware (access-control.md F1, F4). That firmware then owns the device
identity key and the cloud link, and it survives any later policy change. Signing is the only
control that covers the LAN and the cloud path at once.

The pod already links Monocypher's RFC 8032 Ed25519 for the device identity
(`device_identity.c`), so a verify costs about a millisecond and almost no flash.

## Hard requirement: backward compatible

- Pods on 3.5.1 and older keep updating from the new server and CLI as they do today.
- New firmware keeps updating from an old server or CLI until a pod is deliberately switched to
  `required`.
- Image bytes do not change: DFU, SWD and old firmware's SHA-256 check see the same file.
- Dev builds, hwe2e and self-built firmware keep a path.

## Format: a detached signed manifest (128 bytes per asset)

Published as `<asset>.sig` next to `bench_pod_stm32.bin`, `blob-gw0.bin`, `blob-gw1.bin` and
`blob-esp.bin`. All fields little-endian.

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | magic `BPSG` |
| 4 | 2 | format = 1 |
| 6 | 1 | target: 0 firmware, 1 gw0, 2 gw1, 3 esp (same numbering as `ota_target_t`) |
| 7 | 1 | flags (0) |
| 8 | 4 | image size in bytes |
| 12 | 32 | SHA-256 of the image |
| 44 | 4 | version, packed `major<<16 \| minor<<8 \| patch` (gateware: its version) |
| 48 | 4 | build number, monotonic across releases |
| 52 | 2 | fw_info layout (firmware only, else 0) |
| 54 | 2 | fw_info min_flash_kb (firmware only, else 0) |
| 56 | 8 | key_id: first 8 bytes of SHA-256(public key) |
| 64 | 64 | Ed25519 signature over `"benchpod-fw-sign:v1" \|\| 0x00 \|\| bytes 0..63` |

The context string follows the `DEVICE_ID_CTX_*` convention in `device_identity.h`. Binding the
target means a signed gateware blob can never be installed as firmware. Bump `:v1` and `format`
together on any breaking change.

## Pod side

- `sig_verify.c`: key table, `crypto_ed25519_check`, manifest parse. Host-tested with vectors
  produced by the signing tool.
- `ota_begin_target` gets an optional manifest. It checks magic, format, key_id against the
  table, the signature, then that `size`, `sha256` and `target` equal the begin arguments, and for
  firmware that layout and min_flash_kb fit this pod. All of that runs **before staging**, so a
  refusal never touches PSRAM or flash. The existing hash check at `ota_end` and the re-verify
  before the flash write then bind the received bytes to the signed hash. No new code runs over
  the image.
- Transports:
  - WS `ota.begin` and LAN `ota_begin`: new field `"sig":"<b64url>"` (`b64url.c` exists).
    `bp_json_get` and `cl_json_str` look fields up by name, so old firmware ignores it.
  - USB console: `CONSOLE_LINE_MAX` is 128 and `upload-begin` already uses about 95 characters,
    so the 171-character manifest goes on its own lines first: `upload-sig 0 <b64>`,
    `upload-sig 1 <b64>`. `upload-begin` uses it when present. Clients send these only when the
    pod reports `ota_sig`.
- Keys: release builds embed two public keys (current and an offline spare for rotation).
  Builds without `RELEASE=1` also trust a developer key from `make dev-key`
  (`~/.config/benchpod/`), and `make` signs its own output. Release firmware never trusts it.
- Policy, persisted in the config sector:
  - `permissive`: unsigned accepted (reported `sig:"none"`), a bad signature always refused.
  - `required`: unsigned refused.
  - **Ratchet:** the cloud and the USB console may set `required`; only the USB console may go
    back to `permissive`. A compromised server or LAN host can therefore never loosen it. The
    LAN cannot change it at all.
  - The default comes from the build: `permissive` in the first release.
- Downgrade guard: fw_info `reserved[0]` bit 0 = "this image verifies signatures". A `required`
  pod refuses firmware without that bit, so installing 3.5.1 cannot silently drop verification.
  Going back on purpose stays possible over USB DFU.
- Reporting: capabilities `ota_sig:1` and `sig_policy`; `ota.status` and the JSON status carry
  the last install's `sig: ok|none|bad` and key_id.
- Blobs in the W25Q: the signature is checked at install time like firmware. Store the manifest
  in the slot header's reserved bytes so the planned bootloader can check the `fw` slot at boot.

## Compatibility matrix

| Combination | Result |
|---|---|
| Old firmware (<=3.5.1) + new server | `sig` field ignored, install works as today |
| Old firmware + new CLI over USB | no `ota_sig` capability, so no `upload-sig`; works as today |
| New firmware + old server or CLI | no signature: `permissive` accepts, `required` refuses at begin with a clear error (nothing staged, no brick risk) |
| New firmware + tampered image or unknown key | refused at begin in both policies |
| Installing an older release | 3.5.0 and 3.5.1 get `.sig` files afterwards: installable on `permissive`, refused on `required` (no verify bit) |
| USB DFU, SWD | unchanged. `flash-self` checks the signature on the host and needs `--allow-unsigned` for an unsigned file |
| Web app "Upload a file" | optional `.sig` field; without one it works on `permissive` pods only |
| Dev builds, hwe2e | dev key on dev builds; bench pods on release firmware set to `permissive` over USB |

The first update *into* the signing release is installed by the old firmware's code, without a
signature check. That is unavoidable and happens once per pod.

## Keys and CI

- Recommended: the Ed25519 private key in a protected GitHub Environment `release` with the
  owner as required reviewer; the spare key generated offline and kept offline (for example
  1Password), never in CI.
- Stronger option: a cloud key service with Ed25519 (Google Cloud KMS; AWS KMS, to be
  confirmed) reached through GitHub OIDC, so no private key exists in GitHub at all. Worth it if
  the server already lives in that cloud.
- `release-stm32.yml` signs all four assets after the "Checksum" step and uploads `*.sig`.
- Add `.sig` files to the 3.5.0 and 3.5.1 releases afterwards with `gh release upload`. Their
  build numbers are assigned in the order of the releases.
- Rotation: a release adds the next key and can drop a revoked one; the pod persists a minimum
  key generation once it boots a release that raises it.

## Server and CLI

- Server: `FirmwareReleaseWatcher` downloads the `.sig` files and verifies them itself before it
  lists a release as installable (a release without valid signatures stays visible but cannot be
  installed on a `required` pod). `ota.begin` forwards `sig` for firmware and blobs. The web app
  upload takes an optional `.sig`. After a pod reports its first `sig:"ok"` install, the server
  may push `sig_policy required` (org setting, on by default for new orgs).
- CLI: `flash-self` checks the signature before DFU; `install-blobs` and the post-DFU blob
  install send `upload-sig` when the pod has `ota_sig`.
- hwe2e: synthetic OTA images signed with the dev key; a new test for each refusal.

## Implementation order within the access-control work

The principle: **firmware integrity first.** Every other pod-side control (LAN policy, tiers,
TLS-only cloud) is enforced by firmware, so as long as anyone with LAN or cloud access can
replace that firmware, those controls can be removed by whoever they are meant to stop. And an
implant installed before the controls land survives them. Server-only fixes go even earlier
because they cost no firmware release.

| Step | What | Ships as | Needs |
|---|---|---|---|
| 1 | Server role gate: OTA routes and T2/T3 verbs need org owner/admin or a new `benchpod:admin` scope, plus an audit log (access-control.md section 5) | server deploy, days | nothing |
| 2 | Signing infrastructure: format spec + vectors, signing tool, Go verify package, keys, CI signing, `.sig` files added to 3.5.0/3.5.1 | CI + release assets | nothing; runs in parallel with step 1 |
| 3 | Server + CLI forward and check signatures (both are no-ops for old firmware) | server deploy, CLI release | step 2 |
| 4 | **Firmware release N**, commits in this order: (a) command tier table, (b) signature check + policy + ratchet + downgrade bit + `upload-sig`, (c) release builds refuse `cloud_set tls=false` (F2 quick fix), (d) `lan_policy open\|locked\|off`, default `open` | one firmware release | step 3 deployed **before** the tag |
| 5 | Server turns it on: pushes `sig_policy required` to pods after their first verified install, pushes the org's `lan_policy` default, UI toggles | server deploy | step 4 |
| 6 | Bootloader (ota-fallback.md): installed as a signed combined image, then verifies the W25Q `fw` slot manifest at boot | next firmware release | step 4 |
| 7 | WS-auth v2 (`host \|\| nonce`, F2), LAN credential (option B), lease mirroring (F3), rollback floor on the build number | next release(s) | steps 4, 5 |
| 8 | H5 secure boot / RDP for physical attackers; customer-owned signing keys if asked | later | step 6 |

Notes on the order:

- Within release N the tier table comes first because it decides which verbs `sig_policy`,
  `lan_policy` and `cloud_set` belong to (all T3). Signing comes before `lan_policy` because
  `lan_policy` alone leaves the cloud path open (F4), while signing closes both.
- Thanks to the ratchet, enforcement does not need a second firmware release: release N ships
  `permissive`, and the server switches each pod to `required` once that pod has proven it
  accepts signed images. If the verify code had a bug, pods stay `permissive` and keep updating.
- Steps 2 and 3 must be live before step 4 is tagged, so the first verifying firmware already
  receives signed images. Restart the server after tagging so its release list picks up N.
- The bootloader goes after signing so that the one risky combined-image install, and every
  bootloader-driven restore after it, is signed.

## Effort

| Part | Days |
|---|---|
| Spec, signing tool, Go verify package, keys | 2 |
| Release CI + `.sig` files added to old releases | 1 |
| Server (watcher, forward, upload, policy push) | 2 to 3 |
| CLI (`flash-self` check, `upload-sig`) | 2 |
| Firmware (verify, policy, ratchet, console, fw_info bit, host tests) | 4 |
| hwe2e + hardware runs (cloud, LAN, USB; good and bad signatures) | 1 to 2 |
| **Total** | **12 to 14** |

## Open decisions

1. Key storage: GitHub Environment secret (recommended) or a cloud key service.
2. Default `sig_policy required` for existing orgs too, or only new orgs with opt-in for the rest.
3. Self-built firmware over the network: is "set `permissive` over USB once" enough, or should
   users be able to enroll their own public key (over USB only)?
4. Add `.sig` files only to 3.5.0 and 3.5.1 (blobs in the W25Q), or to every 3.x release?
