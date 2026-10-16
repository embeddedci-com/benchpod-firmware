# Wi-Fi bridge control protocol (BWP v1)

How the BenchPod (or a laptop, for bring-up) talks to the Wi-Fi bridge over the Ethernet cable
between them. The bridge implements it in `components/bridge_core/bwp.c`; the pod side is a
follow-up and implements the same spec. `tools/bridgectl.py` is a reference client for Linux.

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
  so by the OTA offset rule below.
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
| 0x09 | PASSPHRASE | empty (open network), 8..63 printable ASCII, or 64 hex digits (raw PSK) |
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
| 0x16 | FLAGS | u8: 0x01 credentials stored, 0x02 Ethernet link up, 0x04 OTA in progress, 0x08 pod MAC learned |

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

## Messages

| Type | Name | Direction | Request body | Response body |
|---|---|---|---|---|
| 0x01 | HELLO | bridge → pod, every 2 s, seq 0 | MODEL, FW_VERSION, HW_REV, STATE, FLAGS, BRIDGE_MAC, POD_MAC (if learned) | (none) |
| 0x02 | STATUS | pod → bridge | (empty) | RESULT, STATE, FLAGS, SSID (if configured), CHANNEL + RSSI + BSSID (if connected), REASON (if any), UPTIME, POD_MAC (if learned), COUNTERS |
| 0x03 | SCAN | pod → bridge | (empty) | RESULT, up to 20 AP, strongest first |
| 0x04 | SET_WIFI | pod → bridge | SSID, PASSPHRASE (omit or empty for open) | RESULT, after the credentials are stored |
| 0x05 | CLEAR_WIFI | pod → bridge | (empty) | RESULT, after the credentials are erased; Wi-Fi disconnects |
| 0x06 | OTA_BEGIN | pod → bridge | IMAGE_SIZE | RESULT, after the slot is erased (can take seconds) |
| 0x07 | OTA_DATA | pod → bridge | OFFSET, DATA | RESULT, OFFSET = next expected offset |
| 0x08 | OTA_END | pod → bridge | (empty) | RESULT, after hash + signature verification; the new image boots on the next REBOOT |
| 0x09 | REBOOT | pod → bridge | (empty) | RESULT, then the bridge restarts after ~200 ms |
| 0x0A | FACTORY_RESET | pod → bridge | (empty) | RESULT, then credentials are erased and the bridge restarts |
| 0x0B | OTA_ABORT | pod → bridge | (empty) | RESULT; drops a partial transfer |

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
  project name (`benchpod-wifi-bridge`) and refuses anything else. OTA_END verifies the image
  (SHA-256 and the RSA-3072 signature, see [SECURITY.md](SECURITY.md)) before it marks it bootable.
  After REBOOT the new image must come up and run for 30 s with Ethernet started, or the
  bootloader rolls back to the previous one.
- **Who may send requests:** physical access to the cable is the trust boundary. The bridge
  accepts requests from the device it learned as the pod, never from Wi-Fi. SET_WIFI carries the
  passphrase in the clear over that cable, the same as typing it into the console. OTA images
  are signed, so a device on the cable cannot install its own firmware.

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

From `test/build/test_bridge --dump` (the unit tests check the HELLO bytes):

```
HELLO, bridge 02:bb:00:00:00:01 → broadcast, state WAIT_POD, flags 0x0b, pod 00:80:e1:12:34:56
ff ff ff ff ff ff 02 bb 00 00 00 01 88 b5 42 57
01 01 00 00 00 3d 02 00 14 62 65 6e 63 68 70 6f
64 2d 77 69 66 69 2d 62 72 69 64 67 65 03 00 05
30 2e 31 2e 30 04 00 01 41 05 00 01 01 16 00 01
0b 06 00 06 02 bb 00 00 00 01 07 00 06 00 80 e1
12 34 56

SET_WIFI, pod → bridge, seq 0x0102, SSID "lab", passphrase "secret12" (padded to 60 bytes)
02 bb 00 00 00 01 00 80 e1 12 34 56 88 b5 42 57
01 04 01 02 00 11 08 00 03 6c 61 62 09 00 08 73
65 63 72 65 74 31 32 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00

Response: SET_WIFI | 0x80, seq 0x0102, RESULT OK
00 80 e1 12 34 56 02 bb 00 00 00 01 88 b5 42 57
01 84 01 02 00 04 01 00 01 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00
```

## What the pod needs (follow-up, not in this repo change)

- lwIP: a hook for EtherType 0x88B5 on the Ethernet netif (`LWIP_HOOK_UNKNOWN_ETH_PROTOCOL`),
  sending raw frames with `netif->linkoutput`.
- A small client: remember the bridge MAC from HELLO, request/response with seq and retries,
  report STATUS as the `wifi-bridge` uplink (state, SSID, RSSI) in `status`.
- Map `set-wifi`, the web Settings tab and a Wi-Fi picker (SCAN) to SET_WIFI / CLEAR_WIFI / SCAN
  when a bridge says HELLO.
- Relay bridge firmware: OTA_BEGIN, 1024-byte OTA_DATA in order, OTA_END, REBOOT.
- Power control of the always-on USB-A port to power-cycle a bridge that stops sending HELLO.
