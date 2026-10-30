# Wi-Fi bridge: what is left to release it

State on 2026-10-30. Test results: [BRINGUP-REV2.md](BRINGUP-REV2.md).

## Decided (2026-10-30)

- **The plain build goes in the pod's `bridge` slot** (`RELEASE=1 ./build.sh`) for now. A bridge with
  secure boot fuses burned would refuse it, so fused bridges need another plan first. Secure boot and
  flash encryption have never run on hardware.
- **Pods in the field need nothing for the bridge:** they have built-in Wi-Fi. They get the image on
  their second update (old firmware does not list the slot); that is fine and stays as it is. Their
  built-in Wi-Fi must keep working after the upgrade (test below).

## Release steps, in this order

1. Deploy the server (`make deploy-prod`, list other sessions' commits on main first). It must know
   `blob-bridge.bin.sig` before a release carries it, or the pod refuses the unsigned blob.
2. Set the bridge version: `PROJECT_VER` in `wifi-bridge/CMakeLists.txt` is 0.1.1. Bump the pod's
   `FIRMWARE_VERSION` (patch).
3. Tag the firmware. CI builds and signs everything, including `blob-bridge.bin`.
4. Restart the server after tagging, so it sees the release.
5. Tag the CLI so users have `set-wifi` with the SSID check (benchpod-cli PR #47, merged, not
   released). `bringup.sh` needs CLI 0.8.3+.

## Test on the release (cannot be done before the tag)

The test unit runs dev builds signed with the developer key; a release pod trusts only release keys.

1. Install the signed release on the pod: `benchpod flash-self` from the release, then check
   `blob_status` lists `bridge` as present.
2. Bridge on an older version: the pod updates it by itself. Note the bench bridge runs test build
   0.1.9, above the release's 0.1.1, so it will not update: use `bridge update` to force it, or
   reflash it first. It must hold a build that trusts the release keys to accept the update.
3. Over the cloud: install from the web app on a second pod and check the bridge part shows as
   "Wi-Fi bridge image". Then the same install again: the blob is skipped.
4. **Built-in Wi-Fi after the upgrade (must pass).** Update a pod on its own Wi-Fi (ESP32-C3) from
   the current release over the cloud: it comes back on Wi-Fi with its credentials, on LAN and cloud.
   Run hwe2e on it. Checked once on a dev build with the client (2026-10-30, not an upgrade):
   `set-wifi` joined through the C3 at -64 dBm, the LAN API answered, pod to host 2.9 Mbit/s, pings 72
   to 316 ms, `clear-wifi` worked. No hwe2e, no cloud check, and no baseline to compare those numbers with.
5. A pod on Ethernet with no bridge: the client does nothing there. Run hwe2e.
6. A pod with built-in Wi-Fi configured, then a bridge plugged in: built-in Wi-Fi wins (see the
   results in `BRINGUP-REV2.md`). Repeat on the release build.
7. Overnight soak through the bridge (`.podlock/soak/bridge_soak.py`). Stopped after a short run on
   2026-10-30, not needed unless wanted.

## Before bridges go to customers

- Factory flow: the factory image goes over the TC2030 pads. Only done with a dev build on one board.
- More than one board: every result so far is one rev 2 board on one bench. Includes supply margin
  (no reset down to 3.6 V).
- RF and EMC: not covered by these tests.
- Router case on the final build: bridge on a router forwards nothing (tested on 0.1.7), and Wi-Fi
  goes off after 30 s (written in 0.1.9, not tested on a router).

## Small things, not blocking

- For its first second after losing an AP, the bridge can report `auth-failed` before `no-ap`.
- The pod's first request after a bridge reboot uses the old nonce: one AUTH refusal, then it recovers.
- After a longer outage the cloud waits out its 35 s pong timeout before reconnecting.
- The pod's "bridge found" log line can show the version from before an update.
- Host-to-pod TCP is about 2.3 Mbit/s even wired (pod TCP receive window, about 11.7 KB).
- `docs/API.md` mentions `benchpod firmware install`, which CLI 0.8.4 does not have.
