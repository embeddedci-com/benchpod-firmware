# Wi-Fi bridge: what is left to release it

State on 2026-10-30. Test results: [BRINGUP-REV2.md](BRINGUP-REV2.md).

## Decide first

1. **Secure boot or not for the image in the pod's `bridge` slot.** The release blob is the plain
   `RELEASE=1 ./build.sh` build. A bridge with secure boot fuses burned refuses it: it needs the
   RSA-signed image. Secure boot and flash encryption have never run on hardware, and testing them
   burns fuses on that board for good. Plain build for now means fused bridges need another plan
   (a second slot image, or the pod choosing by what the bridge reports).
2. **Close the gap for pods in the field?** The server asks the pod which slots it has before an
   update, and old firmware does not list `bridge`, so those pods get the image on their second
   update. Fine unless a pod gets a bridge before that (then `benchpod install-blobs`). A server
   change could send it right after the first update.

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
4. A pod with no bridge: a normal release pod on Ethernet and one on its own Wi-Fi, to confirm the
   client does nothing there. Run hwe2e on them.
5. Overnight soak through the bridge (`.podlock/soak/bridge_soak.py`). Stopped after a short run on
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
