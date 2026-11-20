# LAN pairing

Status: firmware 4.0.1. Who may use the JSON API from Ethernet and Wi-Fi.

## The rule

- **USB is always trusted.** The USB console and the USB network adapter mean someone has the
  pod's cable. They need no pairing.
- **The cloud is trusted.** embeddedci.com checks the account, the API key or the GitHub OIDC
  token, and the pod checks the server (TLS + its device key).
- **A computer on Ethernet or Wi-Fi must be paired.** Until it is, it gets only `ping`, `status`,
  `diag`, `cloud_status`, `identity_public` and `lan_tls` (reads: what the pod is, and how to reach it over TLS).
  Everything else gets `BP0081` (`lan_not_paired`). `identity_pop` needs pairing too: `register`
  uses it, and an unpaired computer that could register an unregistered pod to its own account
  would unlock it through the cloud.
- A paired computer gets what the LAN policy allows: `open` (the default) is every command,
  `locked` keeps T2/T3 for USB and the cloud. Updates from the LAN still need a good signature.
- **Who is paired is decided over USB or from the cloud, never from the LAN**, paired or not:
  `lan_clients` changes, `lan_policy` `pairing`, and the console's `lan-pairing` / `lan-clients`.
  A leaked client key can be removed; it cannot add others or turn pairing off.

Pairing is on by default, and on every pod updated from older firmware (its policy record has the
byte at 0). `lan_policy` `"pairing":false` turns it off for a lab that wants the old behavior.

## How a computer proves who it is

Mutual TLS on port 8443, without a CA:

1. The computer makes an EC P-256 key and a self-signed certificate once, and keeps them (the CLI,
   the SDK and the desktop app share one per user).
2. Pairing sends the SHA-256 of the certificate's DER SubjectPublicKeyInfo to the pod
   (`lan_clients` `add`) over the USB link, or the server sends it over the pod's cloud link for a
   registered pod. At the same time the computer stores the pod's certificate SHA-256 (`lan_tls`),
   so later LAN connections check they reach the real pod.
3. Port 8443 asks every client for a certificate (mbedTLS `VERIFY_OPTIONAL`, no CA chain). A client
   without one still connects, unpaired. A client that sends one must also send a good
   CertificateVerify, so it holds the key. The chain and usage checks fail for every self-signed
   certificate; OPTIONAL keeps the connection, and the pod trusts the key only through its list.
4. `net_server` hashes the peer's public key once, on the connection's first data (application data
   only arrives after the handshake), and every command checks that hash against the list. Removing
   a computer takes effect on its next command, also on a connection that is already open.

Resumed sessions (session cache and tickets) keep the peer certificate
(`MBEDTLS_SSL_KEEP_PEER_CERTIFICATE`), so a resumed connection is paired the same way.

SCPI has no pairing step of its own: on the external LAN it answers only on a paired TLS
connection while pairing is on. Plaintext 8080 is never paired.

## Storage

The list lives in the LAN TLS record (0x1E0000 / 0x1E2000, `lan_tls.h`), as record version 2: the
key PEM, the certificate PEM, then the computers (32 bytes of hash and a 24-byte name each). The
persistence area has no free sector, and the list belongs with the certificate it pairs to. Up to
32 computers fit next to the generated certificate; a large user certificate leaves fewer (8 with a
7.6 KB bundle). Regenerating or replacing the certificate keeps the list.

Version 1 records (firmware 4.0.0 and older) are read once; the next change writes version 2.
Firmware 4.0.0 does not read version 2: rolled back, it generates a new certificate, and the pod
forgets its paired computers. Updating again starts with an empty list.

The list is in RAM on the FreeRTOS heap (1.8 KB) and changed only on the net task; other tasks read
it inside a critical section.

## Commands

- `lan_clients`: list (12 a reply, `offset`), `add` (`name`, `spki_sha256`), `remove` (a name or
  8+ hex digits of a key), `clear`. API.md has the details.
- `lan_policy` `pairing` true|false.
- `status`: `lan_pairing` and `lan_paired` (`null` when the asking connection needs no pairing).
- mDNS carries nothing new: a client learns it is unpaired from `status`.
- Console: `lan-pairing [on|off]`, `lan-clients [remove <x> | clear]`.

## Not done

- A button on the pod to pair without a cable (v4 hardware question).
- Per-computer rights. Every paired computer has the same rights; the LAN policy sets them.
