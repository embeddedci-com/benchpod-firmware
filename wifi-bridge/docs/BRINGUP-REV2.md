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

## Throughput (Mac on Wi-Fi to the pod, LAN JSON `speedtest`)

| Direction | Through the bridge | Pod wired | Bridge cost |
|---|---|---|---|
| Pod to host | 7.9 to 8.7 Mbit/s | 10.9 Mbit/s | about 20% |
| Host to pod | 1.9 Mbit/s | 2.3 Mbit/s | about 15% |

The slow host-to-pod direction is the pod's own network stack (TCP receive window about 11.7 KB),
not the bridge.

## Power (bench PSU on the USB-C input, 5 V)

| State | USB input | 3V3 if the regulator is linear |
|---|---|---|
| Idle, joined | 0.154 A, 0.77 W | about 0.51 W |
| Joining | 0.157 A shown for about 1 s (short peaks too fast for the PSU display) | about 0.52 W |
| 7.9 Mbit/s traffic | 0.200 A, 1.0 W | about 0.66 W |
| Ethernet into a router by mistake (see below) | 0.177 A | about 0.58 W |

Over the 0.5 W budget: just at idle, about 30% under traffic. Wi-Fi power save is off
(`Set ps type: 0`); modem sleep would lower idle at some latency cost. The LAN8742A at 100M is a
fixed share of 40 to 50 mA.

## Open issues

1. **The bridge clones any device on its Ethernet port.** Plugged into a router instead of the pod,
   it logged `pod MAC changed to 34:60:f9:80:77:bf`, restarted its station with that MAC and
   forwarded about 85,000 frames in 90 s (1,425 dropped) between the wired LAN and Wi-Fi: a
   duplicate MAC on the network and a possible loop. It should forward only the paired pod's MAC
   (`l2fwd` `L2_EV_CHANGED` and `wifi_pod_mac_changed`), or at least not adopt a new MAC while paired.
2. **`benchpod set-wifi` reports a join that did not happen.** With an SSID that does not exist it
   printed `The Wi-Fi bridge joined "bench-no-such-ap"`, because the pod still showed the previous
   network's `connected` status. Fix on both sides: the pod drops the old status (state joining,
   no rssi) when SET_WIFI is accepted, and the CLI only accepts a join or failure for the SSID it set
   (`setWifiThroughBridge` in benchpod-cli `commands_serial.go`).
3. **Power over budget** (above).
4. **Brownout message at every power-up from the bench PSU** (`E BOD` before the reset line, then a
   normal boot). Not seen on reboots. Check on a normal USB supply and look at the 3V3 ramp.
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
