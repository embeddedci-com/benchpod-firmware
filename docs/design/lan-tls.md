# LAN API over TLS (port 8443) and the HTTPS-only setting

Status: design, 2026-10-16, branch `lan-tls`. Motivation: Wi-Fi moves out of the pod into a
separate Ethernet-to-Wi-Fi bridge (benchpod-private `enclosure/wifi-bridge.md`, "Security"). The
bridge must only ever carry encrypted traffic. The cloud link already is (TLS + Ed25519 device
key); the LAN JSON/SCPI API on TCP 8080 is plaintext with no authentication. This closes the
plaintext part. It does not add client authentication (see "Not in scope").

## What the pod does

- A second listener, **TCP 8443**, speaks TLS 1.2 and carries exactly the same protocol as 8080
  (JSON lines, SCPI, the raw DAP/UART/bulk modes). Same 5 connection slots, same LAN policy
  (`off` refuses it on Ethernet/Wi-Fi, `locked` limits it to T0/T1), same USB rules.
- Both listeners are on by default. **HTTPS-only** (default off) turns plaintext 8080 off for the
  external LAN (Ethernet, Wi-Fi): new connections are refused and open ones closed. 8080 keeps
  answering on the USB network adapter, which is physical presence and never crosses the bridge;
  that is also the way back if a TLS client is lost.
- At most **2 TLS sessions at once** (each takes ~25-30 KB of FreeRTOS heap: 16 KB IN record
  buffer, 4 KB OUT, handshake state). A third TLS client is refused at accept, and so is any TLS
  client while the heap is below a floor that keeps the cloud link's room. Plaintext slots are
  unaffected.
- mDNS: the `_benchpod._tcp` TXT gains `tls=8443` and `https_only=0|1`. The service port stays
  8080 (old clients). `_scpi-raw` stays on 8080.

### Certificate

- Generated on the pod the first time the TLS listener starts without a stored one (first boot of
  this firmware): ECDSA P-256 key from the TRNG-seeded CTR_DRBG, self-signed X.509 v3 cert,
  SHA-256 signature, random 64-bit serial, subject `CN=benchpod-xxxxxx`, SAN
  `DNS:benchpod-xxxxxx`, `DNS:benchpod-xxxxxx.local` (the mDNS name). Validity 2025-01-01 to
  9999-12-31 (RFC 5280 "no expiry"; the pod has no wall clock). basicConstraints CA:false,
  keyUsage digitalSignature, extKeyUsage serverAuth.
- Replaceable by the user: a PEM private key (EC P-256/P-384 or RSA 2048-4096) plus a PEM
  certificate (optionally followed by its chain). The pod checks that both parse and that the key
  matches the certificate before it stores anything.
- Regenerate on request (new key, new cert).
- Stored in **internal config flash**, a new A/B record store (the same power-loss-safe layout as
  the other records) at **0x1E0000 / 0x1E2000** (sectors 112, 113), below the pod policy. The
  persistence area grows by 16 KB: `FLASH_LAYOUT_STORE_BASE` 0x1E4000 -> 0x1E0000, linker FLASH
  912K -> 896K, `OTA_MAX_SIZE` follows. The image is ~560 KB. The record is a small header
  (origin, key length, cert length) followed by the key PEM and the cert PEM, at most 7.5 KB.
  The A/B store gets a variable-length variant for this (the existing records are <= 240 B).
- On a 1 MB part the same 16 KB come off the code area (0x0E0000).

### JSON (LAN, cloud command channel, cloud tunnel)

    {"cmd":"lan_tls"}
      -> {"port":8443,"https_only":false,"sessions":0,"origin":"generated","key":"EC P-256",
          "subject":"CN=benchpod-baea06","san":["benchpod-baea06","benchpod-baea06.local"],
          "not_after":"9999-12-31","sha256":"<64 hex of the DER cert>"}
    {"cmd":"lan_tls","https_only":true|false}   -> same reply
    {"cmd":"lan_tls","regenerate":true}         -> same reply, new cert
    upload: ota_begin/ota_data/ota_end/ota_commit with "target":"tls"; the payload is the PEM
            certificate (chain) followed by the PEM key. ota_end checks it, ota_commit stores it
            and the TLS listener uses it for new connections (open TLS sessions keep the old one).

`status` gains `"lan_tls":{"port":8443,"https_only":false,"sha256":"..."}` and the capability
`lan_tls`. USB console: `tls`, `tls https-only on|off`, `tls regenerate`, and the existing upload
path with target `tls`.

### Who may change what

Reading works from anywhere. Changes (`https_only` off, `regenerate`, the upload):

| From | May change |
|---|---|
| USB console, USB network adapter | yes |
| Cloud (command channel, tunnel) | yes (T2: organization owner/admin on the server) |
| LAN over TLS (8443) | yes, unless the LAN policy is `locked` (T2) |
| LAN plaintext (8080, Ethernet/Wi-Fi) | no: `lan_tls: change it over TLS (port 8443), over USB or from the cloud` |

Turning `https_only` **on** is a tightening and is allowed from anywhere, plaintext LAN included.
The plaintext refusal exists because an active man-in-the-middle on 8080 could otherwise swap the
certificate and then also intercept TLS. A pinned TLS client cannot be spoofed that way.

The setting lives in the pod policy record (a reserved byte, `https_only`). A record from older
firmware reads as off.

## Clients (CLI, Python SDK, MCP)

- Connection `host` or `host:8080` = **auto**: try TLS on 8443 first; if the port is refused
  (older firmware) and the client has no pin for this host, use plaintext 8080. With a pin, a
  refused or failing TLS connection is an error, never a silent downgrade.
- `host:8443` = TLS only. Any other explicit port = plaintext as given (port forwards).
- `usb` / USB network adapter connections stay plaintext (physical link).
- Trust: trust on first use, pinned by the SHA-256 of the pod's certificate (DER) per address in a
  file the CLI and the SDK share: `~/.config/benchpod/known_pods.json` (the CLI's config dir). A
  mismatch is an error that names both fingerprints and how to accept the new one
  (`benchpod tls trust <host>`). When the CLI or SDK replaces or regenerates the cert itself, it
  updates the pin from the reply. With a CA bundle (`--tls-ca`, `BENCHPOD_TLS_CA`), the client
  verifies chain and host name instead of pinning (for user certificates signed by a company CA).
- A pod that refuses plaintext (https_only) with the client on an explicit `:8080` gets a clear
  error that says to use `host` or `host:8443`.

CLI commands (match `benchpod cloud ca ...` style):

    benchpod tls [show]                         cert subject, SAN, origin, SHA-256, https-only, port
    benchpod tls set-cert --cert x.pem --key y.key
    benchpod tls regenerate
    benchpod tls https-only on|off
    benchpod tls trust [<host>]                 pin the pod's current cert (after a deliberate change)
    benchpod tls forget [<host>]                drop the pin

SDK: `BenchPod.lan_tls()`, `set_lan_tls_https_only()`, `set_lan_tls_cert()`, `regenerate_lan_tls()`;
TLS transport options `tls=None|True|False`, `tls_ca=`. MCP: `connect` follows the SDK; a `lan_tls`
tool shows and changes the setting.

## Web app

Pod Settings tab: an "HTTPS-only API" switch next to "Network access without sign-in", plus the
certificate fingerprint (read-only). Shown only when the pod advertises `lan_tls`. Driven through
the cloud command channel (`lan_tls`), T2 like `lan_policy`.

## VISA / raw socket

8080 stays the raw-socket port. With HTTPS-only on, raw-socket tools need a TLS wrapper on the
computer, for example `socat TCP-LISTEN:5025,reuseaddr,fork OPENSSL:benchpod-xxxxxx.local:8443,cafile=pod.pem`
with `pod.pem` from `benchpod tls show --pem`.

## Resource budget (to be measured on hardware)

- Flash: + mbedTLS server side (ssl_srv), x509write, pk_write, PEM write, ECDSA sign paths.
- RAM: static +~1 KB; heap per TLS session ~25-30 KB, parsed cert + key ~2 KB while listening.
- Handshake CPU on the net task (ECDHE P-256 + ECDSA sign): measure; session tickets if it hurts
  per-command connections.

## Not in scope

- Client authentication (API key or client certificates) on the LAN API. TLS here gives
  confidentiality and integrity against the network path (the bridge), not access control; the
  LAN policy remains the access control.
- TLS 1.3.
