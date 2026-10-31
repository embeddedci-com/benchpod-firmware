# Wi-Fi bridge rev 2: first bring-up results

First rev 2 board, tested 2026-10-29 on benchpod-baea06 (pod Ethernet to the bridge, bridge on
home Wi-Fi, 2.4 GHz channel 1, -54 to -66 dBm). Pod: dev build `WIFI_BRIDGE=1` from PR #75.
Bridge: 0.1.4 test build (a version bump of 0.1.0, to exercise updates). Bridge console over an
Olimex ESP-PROG, pod console over USB, both logged during every test.

## Results

| Test | Result |
|---|---|
| Boot, RMII, PHY | Clean. 100M full duplex, no CRC errors. |
| Pairing | Pod pairs on its own within 1 s of the bridge appearing. |
| Wi-Fi join | Joins in about 3 s; pod status shows `bridge connected` and the rssi over LAN and cloud. |
| Pod to bridge update (OTA relay) | 725 KB in about 4.5 s, confirmed about 2.3 s after the new image boots. |
| Update cut off mid-transfer (EN held low 2 s) | Bridge keeps its old image. Pod retries on its own (fixed in PR #75) and finishes 12 s after the cut. |
| New image that never confirms | Bridge rolls back after 120 s. Pod reports the rollback once and does not reinstall (fixed in PR #75). |
| Bridge reboot / 2 s / 10 s power cut | Pod back after 10.4 / 11.7 / 19.9 s (was 17 / 19 / 27 s before the DHCP fix in PR #75). Cloud survives the two short cases. |
| Pod hwe2e over the bridge (LAN) | 69 pass, 6 expected skips. |
| Pod firmware update over the bridge (LAN OTA) | 676 KB in 10.8 s, installed, back in 18 s. |
| PAIR button | Bridge forgets the pod, pod re-pairs 3.3 s later; Wi-Fi and traffic not interrupted. |
| FACTORY held 5 s while running | Wi-Fi credentials erased, pairing kept, pod reports `bridge unconfigured`. `set-wifi` restores it in 3 s. |
| FACTORY held at power-up | Bootloader erases nvs + otadata and boots factory 0.1.0; pod re-pairs at once and updates it to 0.1.4; confirmed 20 s after power-up. Wi-Fi must be set again (only the bridge stores it). |
| Missing AP (credentials for an SSID that does not exist) | Bridge goes `no-ap` and retries every 1 to 10 s; pod reports `bridge no-ap`. Real AP loss and return (router off) not tested yet. |
| 10 rejoins (`set-wifi` with the same credentials) | Each about 3 s, no reset. |

## Built-in Wi-Fi and a bridge together (2026-10-30, bridge 0.1.9)

Pod with built-in Wi-Fi configured and joined, then the bridge plugged in, with the bridge also joined:

| Test | Result |
|---|---|
| Bridge plugged in | Pod takes no address on Ethernet: .221 (built-in) answers, .220 does not. `wifi_status` reports the built-in Wi-Fi with `bridge_ignored`. `bridge` prints `not in use`, and pairing still works. |
| `wifi_scan` | Runs on the built-in Wi-Fi (`via` built-in, 14 networks). |
| Pod reboot | Comes back on built-in Wi-Fi with the cloud connected; no lease on Ethernet. |
| `wifi-clear` | Pod answers through the bridge at .220 after 3.7 s, cloud connected; C3 NVS erased. |
| Built-in network gone (`bench-no-such-ap`), bridge joined | Pod stays offline by design: nothing answers on .220 or .221. `wifi-clear` over USB brings it back through the bridge at .220 after 3.7 s. |

Hardware tests on this build (dev build, not a release; skips need fixtures or env settings not set up):

| Path | LAN + cloud result |
|---|---|
| Through the bridge | LAN 70 pass, 0 fail, 6 skip; cloud 52 pass, 0 fail, 16 skip |
| Built-in Wi-Fi only (Ethernet port empty) | 122 pass, 0 fail, 22 skip |

With Ethernet to a router and built-in Wi-Fi joined, both addresses answer and `status` reports the Ethernet one, as intended.

## Throughput (Mac on Wi-Fi to the pod, LAN JSON `speedtest`)

| Direction | Through the bridge | Pod wired | Bridge cost |
|---|---|---|---|
| Pod to host | 7.9 to 8.7 Mbit/s | 10.9 Mbit/s | about 20% |
| Host to pod | 1.9 Mbit/s | 2.3 Mbit/s | about 15% |

The slow host-to-pod direction is the pod's own network stack (TCP receive window about 11.7 KB),
not the bridge.

## Power (bench PSU on the USB-C input, 5 V)

| State | USB input |
|---|---|
| Idle, joined | 0.154 A, 0.77 W |
| Joining | 0.157 A shown for about 1 s (short peaks too fast for the PSU display) |
| 7.9 Mbit/s traffic | 0.200 A, 1.0 W |
| No pod for 30 s, Wi-Fi off | 0.061 A, 0.31 W |
| Modem sleep (tried, not used) | 0.109 A idle, but pings of 370 ms on average, 1.8 s at worst, against about 8 ms |

The ESP32 moved from the pod to the bridge, so the load is about what the pod carried before; there
is no separate power budget. Wi-Fi power save stays off in normal use.

Supply margin: under full traffic the bridge did not reset with the PSU stepped from 5 V down to
3.6 V.

## Open issues

1. **The bridge clones any device on its Ethernet port.** Fixed and HW-verified: the pod MAC is learned from control frames only, and nothing goes from Wi-Fi to Ethernet after a link-down until the pod is back. On the router for 60 s: `eth_rx` +149, `wifi_tx` +0, `eth_tx` +0, pod MAC unchanged. Plugged into a router instead of the pod,
   it logged `pod MAC changed to 34:60:f9:80:77:bf`, restarted its station with that MAC and
   forwarded about 85,000 frames in 90 s (1,425 dropped) between the wired LAN and Wi-Fi: a
   duplicate MAC on the network and a possible loop. It should forward only the paired pod's MAC
   (`l2fwd` `L2_EV_CHANGED` and `wifi_pod_mac_changed`), or at least not adopt a new MAC while paired.
2. **`benchpod set-wifi` reports a join that did not happen.** Fixed in the bridge, pod and CLI, HW-verified: a missing network and a wrong password are reported as such, and the right password as joined, three rounds in a row. With an SSID that does not exist it
   printed `The Wi-Fi bridge joined "bench-no-such-ap"`, because the pod still showed the previous
   network's `connected` status. Fix on both sides: the pod drops the old status (state joining,
   no rssi) when SET_WIFI is accepted, and the CLI only accepts a join or failure for the SSID it set
   (`setWifiThroughBridge` in benchpod-cli `commands_serial.go`).
3. **Brownout message: not a fault.** `E BOD: Brownout detector was triggered` prints as the rail
   collapses at power-off (raw console timestamps: 17 s before the next boot, nothing during that boot).
4. **Duplicate MAC with a wired pod: fixed.** The bridge stayed on Wi-Fi with the pod's MAC after the
   pod moved to a wired LAN. It now leaves Wi-Fi 30 s after a link-down without a pod; back about 2 s after the pod's first control frame. A pod reboot does not trigger it.
5. **Minor:**
   - The pod's "bridge found" log line shows the version from before an update.
   - The pod's first request after a bridge reboot uses the old nonce (one AUTH refusal, then it recovers).
   - After a longer outage the cloud waits out its 35 s pong timeout before reconnecting.
   - The pod still shows its old IP while the bridge is off Wi-Fi (harmless, the link stays up).
6. **Not tested yet:**
   - real AP loss and return;
   - the cloud hwe2e suite (needs a new API key);
   - join current peaks (needs a scope or the pod's `measure_power`);
   - the overnight soak (stopped after 2.5 h; it ran through a planned bridge unplug).
