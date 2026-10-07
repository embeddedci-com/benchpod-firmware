# Cloud hardening: bound login, LAN yields to cloud jobs, company CA and proxy

Status: spec for branch `cloud-hardening` in benchpod-firmware, embeddedci-server and
benchpod-cli, 2026-10-06. Findings F2 and F3 are from [access-control.md](access-control.md).

Every part is backward compatible: a pod only gets a new frame when it advertises the matching
capability, and a server that does not know a new field ignores it.

## 1. Cloud login bound to the server name (F2)

Today the pod signs `benchpod-ws-auth:v1 || 0x00 || nonce`. A LAN attacker who points the pod at
their own host (with a valid certificate for it) can fetch a real challenge for the pod from
embeddedci.com, hand it to the pod, and log in as the pod with the signature.

**v2 message:** `benchpod-ws-auth:v2 || 0x00 || host || 0x00 || nonce`, where `host` is the cloud
host the pod connected to, exactly as stored in its cloud config (lowercase, no port), the same
name it used for SNI and the certificate check.

**Firmware:** signs v2 and adds `&auth=v2` to the WS URL query. Capabilities: `"ws_auth_v2":true`.
Device-identity context constant `DEVICE_ID_CTX_WS_AUTH_V2 "benchpod-ws-auth:v2"`.

**Server:**

- `auth=v2`: rebuild the message with each of the server's own host names and accept if any
  verifies. The names come from config, never from the request (an attacker controls the relayed
  request's Host header): env `BENCHPOD_WS_AUTH_HOSTS` (comma-separated), defaulting to the host of
  `BASE_URL` plus its `www.` / apex twin.
- No `auth` or `auth=v1`: verify v1 as today, unless the device has logged in with v2 before
  (device parameter `ws_auth.v2 = true`, set on the first v2 success). Then refuse: HTTP 401 with
  `this pod logged in with the host-bound signature before; v1 is refused`.
- Audit-log a refused v1 attempt from a v2 device (action `ws_auth`, denied).

**Also in this release (F2 quick fix):** release firmware refuses `cloud_set` with `tls=false` (and
verification off). Dev builds keep allowing plain ws for local servers.

## 2. LAN yields to cloud jobs (F3)

Always on, no setting. While a cloud consumer (CI run, web UI, SDK) holds the device lease, a LAN
client cannot drive outputs or compete for the hardware.

**Server -> pod frame** (only to pods with `"lease_state":true` in their capabilities):

    {"type":"lease.state","held":true,"holder":"CI: org/repo","expires_in_s":118}
    {"type":"lease.state","held":false}

Sent on acquire, on every renew, on release, and right after a pod (re)connects (current state).
`holder` is a short human label (<= 40 chars, ASCII), never a secret. `expires_in_s` is capped at
600.

**Firmware:**

- Keeps `held` and a local deadline (now + `expires_in_s`). Held ends at `held:false`, at the
  deadline (a vanished server cannot lock the LAN forever), or when the cloud link drops.
- While held, LAN TCP connections may only run *light reads*: tier T0 verbs that are not
  observational (`ping`, `status`, `*_status`, `la_pins`, `identity_public`, `blob_status`,
  `ota_status`, ...). Captures (they own the PSRAM bus), T1, T2 and T3 get
  `busy: a cloud job holds this pod (<holder>, <N> s left)`.
- SCPI on the LAN: queries pass, setters get the same busy error.
- The cloud's own connections and the USB console are unaffected.
- Capabilities and status: `"lease_state":true`; status also shows `"lease":{"held":..,"holder":..,"left_s":..}`.

## 3. Company CA certificate and HTTP proxy

### Company CA

Corporate TLS-inspecting proxies re-sign HTTPS with a company root. The pod trusts only the
embedded ISRG roots, so the cloud link fails there.

- Stored in a new W25Q slot `ca` at 0x180000 (64 KB slot, PEM up to 16 KB), outside the release
  blob list (like the `fw` copy slot). Written with the existing upload path: OTA target `ca`
  (`ota_begin` `"target":"ca"`, console `upload-begin ca <size> <sha256>`, WS `ota.begin`). No
  signature needed (it is configuration, not code); T2. Never from the LAN (see "Who may change
  the CA and proxy" below).
- Accepted only if it parses as one or more X.509 CA certificates; refused otherwise.
- Used in addition to the ISRG roots (hostname and chain checks stay on).
- JSON `{"cmd":"cloud_ca"}` -> `{"present":true,"certs":[{"subject":"...","sha256":"<hex>"}]}`;
  `{"cmd":"cloud_ca","clear":true}` removes it. Console: `ca` (show), `ca-clear`.
- Takes effect at the next cloud connect (the pod reconnects after a change).

### HTTP proxy

- Config: host, port, optional user/password, stored in a W25Q slot `proxy` at 0x190000 (4 KB).
- JSON `{"cmd":"cloud_proxy"}` -> `{"host":"...","port":3128,"auth":true}` (never the password);
  `{"cmd":"cloud_proxy","set":"proxy.corp:3128","user":"u","password":"p"}`;
  `{"cmd":"cloud_proxy","clear":true}`. Console: `proxy`, `proxy-set <host:port> [user password]`,
  `proxy-clear`. T2 (so a locked LAN refuses it).
- When set, the cloud client opens TCP to the proxy, sends
  `CONNECT <host>:<port> HTTP/1.1` (+ `Proxy-Authorization: Basic` when set), requires a `200`
  reply, then starts TLS over the same connection (lwIP `altcp_tls_wrap`). Both the challenge
  request and the WS go through it.
- Capabilities and status: `"cloud_ca":true`, `"cloud_proxy":true`.

### Who may change the CA and proxy

Like `sig_policy`, only the USB console and the cloud (WS `ota.begin` target `ca`, the cloud
command channel and cloud tunnels) may install or clear the CA or set or clear the proxy. A LAN TCP
connection is refused whatever the LAN policy (`open` included), before anything changes:
`cloud_ca: change it from the cloud or the USB console` (for `ota_begin` target `ca` and
`cloud_ca` `clear`) and `cloud_proxy: change it from the cloud or the USB console` (`set`,
`clear`). Reads (`cloud_ca`, `cloud_proxy` without `set`/`clear`) stay allowed on the LAN.

Why: with both, a LAN attacker can point the pod at their own proxy and present a certificate for
the real server name signed by their own CA. The pod then does a v2 login over the attacker's TLS,
and the attacker relays the challenge and signature to the real server: the host binding of
section 1 no longer helps, even with signatures `required`.

### CLI

`benchpod cloud ca show|set <file.pem>|clear` and `benchpod cloud proxy show|set <host:port>
[--user U --password P]|clear`, over USB (console commands above) or the cloud. Over the LAN (JSON)
only `show` works; the pod refuses `set` and `clear` there.
