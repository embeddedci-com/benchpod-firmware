# Wi-Fi bridge control protocol (BWP v1)

How the BenchPod (or a laptop, for bring-up) talks to the Wi-Fi bridge over the Ethernet cable
between them. This is the bridge's only management path once it is flashed: its USB-C is power
only, and the UART is on factory pads. The bridge implements it in `components/bridge_core/bwp.c`; the pod's client is
`stm32h563/src/wifi_bridge.c` (development builds), which links the same codec. `tools/bridgectl.py` is a reference client for Linux.

## Transport

- Raw Ethernet frames, **EtherType 0x88B5** (IEEE 802 "local experimental"). No IP.
- Accepted **only on the bridge's Ethernet port**, only from the pod (the learned MAC, see
  "Forwarding"), and only when sent to the bridge's own Ethernet MAC or to a group address
  (broadcast works before the pod knows the bridge's MAC). Never forwarded to Wi-Fi in either
  direction; 0x88B5 frames arriving from Wi-Fi are dropped.
- The bridge's Ethernet MAC is its factory `ESP_MAC_ETH` address. It is in every HELLO
  (`BRIDGE_MAC` and the frame source). It is *not* the MAC the bridge uses on Wi-Fi.
- One message per frame, at most 1514 bytes (no FCS). Frames under 60 bytes are zero-padded; the
  body length field says where the message ends.
- Request/response: the pod sends a request with a sequence number of its choice; the bridge
  answers with `type | 0x80` and the same `seq`, to the request's source MAC. Requests are
  handled one at a time, in order. A lost frame is the pod's to retry (suggested timeout 1 s,
  10 s for SCAN, 30 s for OTA_BEGIN which erases flash). Every request is idempotent, or made
  so by the OTA offset rule below. When paired, a retry is signed again with a new COUNTER (a
  repeated counter is refused as a replay).
- Unsolicited messages from the bridge use `seq = 0`: HELLO every 2 s, and STATUS (0x82)
  whenever the state changes.

## Frame layout

```
offset size
 0     6   destination MAC
 6     6   source MAC
12     2   EtherType 0x88B5
14     2   magic 0x42 0x57 ("BW")
16     1   version = 1
17     1   type
18     2   seq, big-endian
20     2   body length, big-endian
22     n   body: TLVs
```

The magic tells our frames apart from anyone else's 0x88B5 experiments; frames without it are
ignored silently. A frame with another `version` is ignored too (a future v2 adds a new magic
or version and keeps v1 working). Malformed frames (body length past the frame end, a TLV running
past the body) are dropped without an answer.

**TLV:** `tag` (1 byte), `length` (2 bytes, big-endian), `value`. All integers are big-endian.
Strings are UTF-8 without a terminating NUL. Unknown tags must be ignored, so new fields can
be added without a version bump. A tag that appears twice: the first one counts.

## Tags

| Tag | Name | Value |
|---|---|---|
| 0x01 | RESULT | u8, see "Result codes" |
| 0x02 | MODEL | string, `benchpod-wifi-bridge` |
| 0x03 | FW_VERSION | string, e.g. `0.1.0` |
| 0x04 | HW_REV | string, e.g. `A` |
| 0x05 | STATE | u8, see "States" |
| 0x06 | BRIDGE_MAC | 6 bytes |
| 0x07 | POD_MAC | 6 bytes, the learned pod MAC (also the bridge's Wi-Fi MAC) |
| 0x08 | SSID | 1..32 bytes |
| 0x09 | PASSPHRASE | reserved: a cleartext passphrase is refused on the wire (use ENC_PASSPHRASE) |
| 0x0A | CHANNEL | u8 |
| 0x0B | RSSI | i8, dBm |
| 0x0C | BSSID | 6 bytes |
| 0x0D | AUTHMODE | u8: 0 open, 1 WEP, 2 WPA-PSK, 3 WPA2-PSK, 4 WPA/WPA2-PSK, 5 WPA2-Enterprise, 6 WPA3-PSK, 7 WPA2/WPA3-PSK, 255 other |
| 0x0E | AP | nested TLVs: SSID, RSSI, CHANNEL, AUTHMODE, BSSID |
| 0x0F | IMAGE_SIZE | u32, bytes |
| 0x10 | OFFSET | u32, bytes |
| 0x11 | DATA | 1..1024 bytes |
| 0x12 | UPTIME | u32, seconds since boot |
| 0x13 | REASON | u8, last Wi-Fi disconnect reason (ESP-IDF `wifi_err_reason_t`, low byte) |
| 0x14 | COUNTERS | 7 x u32: eth_rx, wifi_tx, wifi_tx_err, wifi_rx, eth_tx, eth_tx_err, dropped |
| 0x15 | MESSAGE | string, human-readable detail for an error |
| 0x16 | FLAGS | u8: 0x01 credentials stored, 0x02 Ethernet link up, 0x04 OTA in progress, 0x08 pod MAC learned, 0x10 new image on trial (not confirmed yet), 0x20 paired |
| 0x17 | NONCE | 8 bytes, random, new on every boot of the bridge (HELLO, refusals) |
| 0x18 | PUBKEY | 32 bytes, X25519 public key: the pod's in PAIR, the bridge's in the PAIR answer and in HELLO while paired |
| 0x19 | COUNTER | u32, request counter, strictly increasing for one NONCE |
| 0x1A | AUTH | 16 bytes, truncated HMAC-SHA256; always the last TLV, see "Pairing" |
| 0x1B | ENC_PASSPHRASE | 89 bytes, the sealed passphrase, see "Encrypted passphrase" |
| 0x1C | SIGNATURE | 128 bytes, the pod's signed manifest of an OTA image (`tools/fwsign.py`, target `bridge`) |

## States

| Value | State | Meaning |
|---|---|---|
| 0 | UNCONFIGURED | no Wi-Fi credentials stored |
| 1 | WAIT_POD | credentials stored, no frame from the pod yet (its MAC is unknown, so the bridge cannot join) |
| 2 | JOINING | associating / authenticating, or retrying after a disconnect |
| 3 | CONNECTED | associated; frames flow |
| 4 | AUTH_FAILED | the AP rejected the passphrase; the bridge keeps retrying with backoff |
| 5 | NO_AP | SSID not found; the bridge keeps retrying with backoff |

## Result codes

| Value | Name | When |
|---|---|---|
| 0 | OK | |
| 1 | BAD_REQUEST | missing or invalid TLV |
| 2 | UNSUPPORTED | unknown request type |
| 3 | BUSY | e.g. SCAN while another scan runs, SET_WIFI during OTA |
| 4 | STORAGE | writing NVS failed |
| 5 | OTA_NOT_STARTED | OTA_DATA/END without OTA_BEGIN |
| 6 | OTA_BAD_OFFSET | chunk not at the expected offset; the response carries the expected OFFSET |
| 7 | OTA_VERIFY | image hash or signature check failed at OTA_END |
| 8 | OTA_TOO_LARGE | image larger than the OTA slot, or a chunk past IMAGE_SIZE |
| 9 | INTERNAL | anything else; see MESSAGE |
| 10 | OTA_WRONG_IMAGE | the image is not a `benchpod-wifi-bridge` app |
| 11 | NOT_PAIRED | the request needs pairing: send PAIR first |
| 12 | AUTH | paired, and AUTH is missing or wrong, or COUNTER did not increase; the response carries NONCE |
| 13 | ALREADY_PAIRED | PAIR from a pod that is not the paired one: the user must press PAIR on the bridge |

## Messages

| Type | Name | Direction | Request body | Response body |
|---|---|---|---|---|
| 0x01 | HELLO | bridge → pod, every 2 s, seq 0 | MODEL, FW_VERSION, HW_REV, STATE, FLAGS, BRIDGE_MAC, POD_MAC (if learned), NONCE, PUBKEY (if paired) | (none) |
| 0x02 | STATUS | pod → bridge | (empty) | RESULT, STATE, FLAGS, SSID (if configured), CHANNEL + RSSI + BSSID (if connected), REASON (if any), UPTIME, POD_MAC (if learned), COUNTERS |
| 0x03 | SCAN | pod → bridge | (empty) | RESULT, up to 20 AP, strongest first |
| 0x04 | SET_WIFI | pod → bridge | SSID, ENC_PASSPHRASE (omit for an open network) | RESULT, after the credentials are stored |
| 0x05 | CLEAR_WIFI | pod → bridge | (empty) | RESULT, after the credentials are erased; Wi-Fi disconnects |
| 0x06 | OTA_BEGIN | pod → bridge | IMAGE_SIZE, SIGNATURE | RESULT, after the manifest checks out and the slot is erased (can take seconds) |
| 0x07 | OTA_DATA | pod → bridge | OFFSET, DATA | RESULT, OFFSET = next expected offset |
| 0x08 | OTA_END | pod → bridge | (empty) | RESULT, after hash + signature verification; the new image boots on the next REBOOT |
| 0x09 | REBOOT | pod → bridge | (empty) | RESULT, then the bridge restarts after ~200 ms |
| 0x0A | FACTORY_RESET | pod → bridge | (empty) | RESULT, then credentials and pairing are erased and the bridge restarts |
| 0x0B | OTA_ABORT | pod → bridge | (empty) | RESULT; drops a partial transfer |
| 0x0C | PAIR | pod → bridge | PUBKEY (the pod's) | RESULT, PUBKEY (the bridge's), after the key is stored |

Every request except HELLO carries COUNTER + AUTH once the bridge is paired (FLAGS 0x20), and
the bridge then signs its response the same way (echoing the COUNTER). Before pairing only
STATUS, SCAN and PAIR are accepted; anything else gets NOT_PAIRED.

Notes:

- **HELLO destination:** the learned pod MAC, or broadcast until the bridge has seen a frame
  from the pod. The pod learns the bridge's MAC from it.
- **STATUS** with `seq = 0` is also sent unsolicited when the state changes, so the pod does not
  have to poll.
- **SET_WIFI** replaces the stored network and reconnects. The passphrase is never sent back by
  any message or console command.
- **SCAN** briefly interrupts traffic while the radio leaves the AP's channel (a few hundred
  milliseconds per channel set). If the bridge is retrying a failing network, it pauses the
  retries for the scan.
- **OTA:** chunks must arrive in order. OTA_DATA at the offset of the previous chunk with the
  same length is a retransmission (our ack was lost): acknowledged again, not written. Any other
  offset gets OTA_BAD_OFFSET with the expected OFFSET, so the pod can resume. The first chunk
  must hold at least the first 112 bytes of the image: the bridge checks the app descriptor's
  project name (`benchpod-wifi-bridge`) and refuses anything else. OTA_BEGIN must carry the pod's
  signed manifest (SIGNATURE): an Ed25519 signature by one of the pod's own release keys over
  target `bridge`, size, SHA-256 and version (the format of `tools/fwsign.py`); anything else is
  answered OTA_VERIFY with `signature: <reason>`. OTA_END compares the received image's SHA-256
  with the manifest (and, in production builds, secure boot checks its RSA-3072 signature, see
  [SECURITY.md](SECURITY.md)) before it marks it bootable.
  After REBOOT the new image is **on trial** (FLAGS 0x10): the first valid request from the pod
  (any type) confirms it. Without one within 120 s, or if it crashes first, the bridge reboots
  into the previous image. So after REBOOT the pod waits for HELLO, sends STATUS, and compares
  FW_VERSION: the old version means the update rolled back. A request is handled only after
  the confirmation, so an OTA_BEGIN to a bridge on trial confirms the running image first and
  never erases the fallback slot.
- **Who may send requests:** only the paired pod (next section), only from the Ethernet side,
  never from Wi-Fi. No secret crosses the cable in the clear: the pairing key is computed on both
  sides (X25519) and the Wi-Fi passphrase travels encrypted. OTA images are signed on top, so
  even the paired pod cannot install its own firmware.

## Pairing

The bridge obeys one pod. The pod proves it is that pod with a 32-byte key they share. The key
is agreed with X25519 and never sent: someone recording the cable learns nothing from PAIR.

```
 unpaired --PAIR(PUBKEY), unsigned, from any pod----------> paired     (trust on first use)
 paired   --PAIR(PUBKEY), signed with the current key-----> paired     (key rotation)
 paired   --PAIR button / FACTORY_RESET / power-up restore--> unpaired (next pod that offers wins)
```

- **First contact:** a pod that sees a HELLO without FLAGS 0x20 sends PAIR with its X25519 public
  key `pod_pub`. The bridge makes a fresh key pair (`bridge_secret` from its RNG,
  `bridge_pub`), computes the key, stores it and `bridge_pub` in NVS (encrypted in production
  builds), forgets `bridge_secret`, and answers OK with `bridge_pub`. Both sides then hold

  ```
  key = BLAKE2b-256(key = X25519(own secret, other side's public key),
                    "BWP pairing v1" || pod_pub || bridge_pub)
  ```

  An all-zero X25519 result (a bogus public key) is refused. A rotation (signed PAIR from the
  paired pod) is answered under the old key, then the new one applies.
- **The pod's secret** is derived, not stored: `SHA-256(Ed25519 signature by the pod's device key
  over "benchpod-wifi-bridge-pair:v1" || 0x00 || bridge MAC)` (`device_identity_sign_ctx`).
  Ed25519 signatures are deterministic, so the pod gets the same secret after every reboot
  without writing flash, a different one for every bridge, and nobody without the pod's device
  key can compute it. With `bridge_pub` from HELLO (sent while paired) it computes the key again.
- **Requests while paired:** the pod adds `COUNTER` (u32) and then `AUTH` as the last TLV:

  ```
  AUTH = HMAC-SHA256(key, NONCE || frame[14 .. start of the AUTH TLV])[0..16]
  ```

  i.e. over the bridge's current NONCE (from HELLO) and every byte from the magic up to the
  AUTH TLV's tag, with the header's body length already counting the AUTH TLV (19 bytes).
  Ethernet addresses and padding are not covered. COUNTER must be higher than the last one the
  bridge accepted for this NONCE; the pod starts again at 1 whenever HELLO shows a new NONCE
  (the bridge rebooted). Gaps are fine.
- **Responses to signed requests** are signed the same way by the bridge, with the request's
  COUNTER, so the pod knows the answer came from its bridge. HELLO and unsolicited STATUS are
  not signed (HELLO has to be readable to learn the NONCE).
- **Refusals** (NOT_PAIRED, AUTH, ALREADY_PAIRED) are unsigned and carry NONCE and a MESSAGE.
  When the AUTH was right but the COUNTER was not above the last accepted one (a replay, or a
  pod that rebooted and started again at 1), the refusal also carries COUNTER = the last
  accepted value, and the pod continues above it. On AUTH the pod first resyncs its nonce and
  counter and retries once; if that fails too, its key is not the paired one (the bridge was
  re-paired with another pod) and it tells the user to press PAIR on the bridge.
- **PAIR button** (GPIO35, a press): the bridge forgets the key, clears FLAGS 0x20 in the next
  HELLO, and accepts the next PAIR from whichever pod sends it. It also re-learns the pod MAC
  from the next frame, so a new pod can take over without unplugging the cable. Wi-Fi settings
  are kept. The FACTORY button never touches pairing, except through the power-up factory
  restore, which erases NVS and with it the key.
- **Trust on first use** means a device on the cable before the pod (or right after a PAIR
  press) could pair instead, or sit in the middle of the first pairing (an active attack; just
  recording is not enough). The pod notices the first case: its PAIR gets ALREADY_PAIRED, which
  it reports. After pairing, a device on the cable can no longer change settings, reboot the
  bridge or start an update; it can still read HELLO and disturb the link physically.

## Encrypted passphrase

SET_WIFI carries the passphrase only as ENC_PASSPHRASE (89 bytes); a cleartext PASSPHRASE TLV is
answered BAD_REQUEST. The sender picks 24 random bytes `seal_nonce` (a fresh value per message) and
sends

```
seal_nonce (24) || XChaCha20(pass_key, seal_nonce, [len] || passphrase || zero padding)   (65 bytes)
pass_key = BLAKE2b-256(key = pairing key, "BWP passphrase v1")
```

The padding to 64 hides the passphrase's length. XChaCha20 is Monocypher's `crypto_chacha20_x`
(HChaCha20 sub-key, then ChaCha20 with a 64-bit block counter starting at 0). There is no separate
tag: the request's AUTH covers the ciphertext, so it cannot be changed unnoticed, and the bridge
only decrypts after AUTH has passed. An open network has no ENC_PASSPHRASE at all. Code:
`components/bridge_core/bwp_crypto.c` (both sides) and `tools/bridgectl.py` (pure Python).

## Forwarding (data frames, for reference)

The bridge is a layer-2 pipe for one MAC:

- **Learning:** the source of the first valid Ethernet frame (any EtherType, control frames
  included) becomes the pod MAC. Multicast and all-zero sources and the bridge's own MAC never
  count. After an Ethernet link-down the next valid source replaces it (another pod was plugged
  in); the Wi-Fi station then restarts with the new MAC.
- **Ethernet → Wi-Fi:** frames from the pod MAC, not addressed to the bridge, while Wi-Fi is
  connected. Everything else is dropped.
- **Wi-Fi → Ethernet:** frames to the pod MAC or to a group address, except frames whose source
  is the pod MAC (the AP echoing the pod's own broadcasts, which would break IPv6 duplicate
  address detection).
- The Wi-Fi station uses the pod MAC, so DHCP, ARP, mDNS and the cloud connection see the pod.
  The bridge itself has no IP address and no open ports on either side.

## Example frames

From `test/build/test_bridge --dump`. The C and Python tests check the HELLO, SET_WIFI and
CLEAR_WIFI bytes (the Python side with its own HMAC, BLAKE2b, X25519 and XChaCha20).

```
HELLO, bridge 02:bb:00:00:00:01 → broadcast, state WAIT_POD, flags 0x0b, pod 00:80:e1:12:34:56,
nonce 01 02 03 04 05 06 07 08
ff ff ff ff ff ff 02 bb 00 00 00 01 88 b5 42 57
01 01 00 00 00 48 02 00 14 62 65 6e 63 68 70 6f
64 2d 77 69 66 69 2d 62 72 69 64 67 65 03 00 05
30 2e 31 2e 30 04 00 01 41 05 00 01 01 16 00 01
0b 06 00 06 02 bb 00 00 00 01 07 00 06 00 80 e1
12 34 56 17 00 08 01 02 03 04 05 06 07 08

SET_WIFI, pod → bridge, seq 0x0102, SSID "lab", passphrase "secret12" sealed (key 00 01 .. 1f,
seal_nonce 01 .. 18), COUNTER 6, AUTH with bridge nonce 01 .. 08
02 bb 00 00 00 01 00 80 e1 12 34 56 88 b5 42 57
01 04 01 02 00 7c 08 00 03 6c 61 62 1b 00 59 01
02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11
12 13 14 15 16 17 18 e7 ad e5 fd 4e 5a 39 fe 31
09 ac d1 36 c8 43 9e 42 6b 3b fc 20 83 65 69 84
87 81 83 25 23 0a aa 59 c2 cf c7 63 e0 12 f3 61
e2 94 3a 04 00 37 fc 02 ab 33 f0 d1 28 f8 4a 3b
21 0a 1e 9b 1b d8 7a 9b 19 00 04 00 00 00 06 1a
00 10 e4 47 0d d1 19 d1 59 70 a7 7b 3f 6e 31 50
ea 3f

Response: SET_WIFI | 0x80, seq 0x0102, RESULT OK, signed with the request's COUNTER
00 80 e1 12 34 56 02 bb 00 00 00 01 88 b5 42 57
01 84 01 02 00 1e 01 00 01 00 19 00 04 00 00 00
06 1a 00 10 05 59 f3 f4 95 76 80 62 54 b3 5a e6
86 cc 2d 9c 00 00 00 00 00 00 00 00

CLEAR_WIFI, seq 0x0103, paired: key 00 01 .. 1f, nonce 01 .. 08, COUNTER 5, AUTH
02 bb 00 00 00 01 00 80 e1 12 34 56 88 b5 42 57
01 05 01 03 00 1a 19 00 04 00 00 00 05 1a 00 10
8c 32 e6 4d 4c 63 82 df c6 53 ef fe e8 5b 08 37
00 00 00 00 00 00 00 00 00 00 00 00
```

Pairing vectors (X25519 secrets 10 11 .. 2f for the pod, 80 81 .. 9f for the bridge): pod_pub
`d89e3bad…67ff66`, bridge_pub `493e82fc…0d531d`, key `dc8611ac…34fce541` (full values in
`tools/test_bridgectl.py`).

## The pod's side

`stm32h563/src/wifi_bridge.c` (protocol client, host-tested against this repo's pairing code)
and `wifi_bridge_net.c` (lwIP `LWIP_HOOK_UNKNOWN_ETH_PROTOCOL` on the Ethernet netif, frames
out through `netif->linkoutput`). Built only in development firmware for now (`WIFI_BRIDGE` in
`stm32h563/Makefile`; off with `RELEASE=1`). What it does:

- Tracks HELLO (bridge MAC, NONCE, state, flags); the bridge counts as present while HELLOs keep
  coming on a live Ethernet link (6 s).
- Pairs on first contact (X25519, secret derived as above), seals the passphrase in SET_WIFI,
  signs every request, verifies every signed answer, retries three times with a new COUNTER,
  and resyncs after either side reboots.
- Polls STATUS every 10 s (which also confirms a bridge firmware update on trial).
- `wifi_set` / `wifi_clear` / `wifi_status` (JSON) and `wifi-set` / `wifi-clear` / `wifi-show`
  (USB console) go to the bridge while one is present; `wifi-scan` lists networks for a picker;
  the console's `bridge` command shows it and runs `scan`, `update`, `reboot` and
  `factory-reset yes`.
- Reports `"uplink":"wifi-bridge"` with `"uplink_rssi_dbm"` in the cloud capabilities when the
  cloud link runs over Ethernet into a connected bridge.
- Keeps the bridge firmware in its W25Q blob slot `bridge` with the signed manifest, and updates
  the bridge by itself when that image is newer than HELLO's FW_VERSION
  (`stm32h563/src/wifi_bridge_ota.c`): OTA_BEGIN with SIGNATURE, 1 KB OTA_DATA in order, OTA_END,
  REBOOT, then HELLO with the new version (its STATUS poll confirms the trial). A failed or
  rolled-back version is not retried until the pod restarts or `bridge update` forces it.

Still to do on the pod:

- Power control of the always-on USB-A port (v4 board) to power-cycle a bridge that stops
  sending HELLO.
