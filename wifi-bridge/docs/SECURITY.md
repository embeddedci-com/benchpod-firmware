# Wi-Fi bridge security

The claim we want to be able to make (design: benchpod-private `enclosure/wifi-bridge.md`):

> The bridge only forwards encrypted traffic, runs only our signed firmware, keeps your Wi-Fi
> password in encrypted storage, and has no address or open ports on your network.

This page says how the firmware gets there, what is on in dev builds, and what production must
turn on. EN 18031-1/-2 (RED delegated act) and the US Cyber Trust Mark (NIST IR 8425) ask for
roughly the same things: secure update, protected credentials, minimal attack surface.

## What the firmware does in every build

| Measure | Where |
|---|---|
| No IP stack on either port: no esp_netif, no lwIP interface, no open ports, no SoftAP code (`CONFIG_ESP_WIFI_SOFTAP_SUPPORT=n`), no Espressif cloud services | `main/wifi.c`, `main/eth.c`, `sdkconfig.defaults` |
| Frames only for the one learned pod MAC; everything else dropped | `components/bridge_core/l2fwd.c` |
| Control channel accepted only from the Ethernet port, only from the pod; never from Wi-Fi, never forwarded | `l2fwd.c`, `docs/PROTOCOL.md` |
| Paired pod only: trust on first use, then every request and its response carry a truncated HMAC-SHA256 over a per-boot nonce and a strictly increasing counter (no replay across requests or reboots). The pairing key is stored in NVS (encrypted in production). A dedicated PAIR button, not the FACTORY button, forgets it | `components/bridge_core/pairing.c`, `main/control.c`, PROTOCOL.md "Pairing" |
| Wi-Fi: no WEP or WPA1/TKIP; WPA2-PSK minimum unless the user stores no passphrase (open network); WPA3-SAE and PMF when the AP offers them | `main/wifi.c` |
| Fixed regulatory domain (`CONFIG_BRIDGE_COUNTRY`, default `US`), no 802.11d adoption | `main/wifi.c` |
| Passphrase is write-only: no message or console command returns it; buffers holding it are zeroed after use | `main/control.c`, `main/console.c`, `main/wifi.c` |
| Wi-Fi library keeps no copy of the credentials (`WIFI_STORAGE_RAM`, `CONFIG_ESP_WIFI_NVS_ENABLED=n`); our app stores them once, in NVS | `main/creds.c` |
| OTA images must be signed (RSA-3072, secure boot v2 signature block) and named `benchpod-wifi-bridge`; two app slots, and a new image stays only once the pod has talked to it (else rollback) |
| No user-reachable serial port: USB-C is power only, UART0 is on TC2030 pads inside the case, the console cannot update firmware | hardware, `main/console.c` | `main/ota.c` |
| Factory reset erases the credentials: button held 5 s while running, control channel, debug console; button held at power-up also restores the factory image (bootloader) | `main/board.c` |

## Dev builds (unlocked)

`sdkconfig.defaults` leaves the chip open: no eFuses burned, flash readable, JTAG and ROM
download mode enabled, NVS in plain text. What it does turn on is **signed OTA without secure
boot** (`CONFIG_SECURE_SIGNED_APPS_NO_SECURE_BOOT`, `CONFIG_SECURE_SIGNED_ON_UPDATE_NO_SECURE_BOOT`,
RSA scheme, so `CONFIG_ESP32_REV_MIN_3`): the running app checks an update's signature against the
public key in its own signature block. `build.sh` signs with `keys/dev_signing_key.pem`, created
on the first build and git-ignored. So a dev bridge only takes OTA images from the checkout that
built it; flash over the TC2030 pads with esptool to switch keys.

## Production

`./build.sh production` builds with `sdkconfig.defaults.production` on top, into `build-prod/`
(checked: it builds, bootloader 0xC000 bytes with factory reset, fits in front of the partition
table at 0x10000).
It turns on:

1. **Secure boot v2** (RSA-3072). The first boot burns the SHA-256 of our release public key into
   eFuse BLOCK2 and write-protects it; from then on the ROM only runs a bootloader signed with
   that key, and the bootloader only an app signed with it. OTA updates are checked against the
   same key. ESP32 (unlike the S3/C3) supports a single secure-boot key and no revocation, so
   the release key has to be protected for the life of the product.
2. **Flash encryption, release mode.** The first boot generates a per-chip AES key in eFuse
   BLOCK1 (unreadable by software) and encrypts the bootloader, partition table, app partitions
   and `nvs_keys` in place. Release mode disables the UART bootloader's encrypt/decrypt and
   cache access, so the flash cannot be read back in plain text.
3. **NVS encryption** (flash-encryption based, `CONFIG_NVS_SEC_KEY_PROTECT_USING_FLASH_ENC`): on
   the first boot `nvs_flash_init` generates XTS keys into the encrypted `nvs_keys` partition and
   encrypts the `nvs` partition with them. This is what protects the Wi-Fi passphrase: flash
   encryption alone does not cover NVS.
4. **ROM download mode disabled** (`CONFIG_SECURE_DISABLE_ROM_DL_MODE`): **a decision to
   confirm.** The UART is only on the TC2030 pads inside the case, so a user cannot reach it
   either way. With download mode disabled, a returned bridge whose OTA path is broken cannot be
   reflashed at all (rollback should prevent that state). With it enabled, esptool over the pads
   can only write flash the chip then decrypts into garbage or rejects (secure boot), so the risk
   is low; ESP32 (unlike S2/S3/C3) has no "secure download mode" in between. Enabled makes RMA
   repairs possible; disabled is the stricter story for EN 18031.

JTAG is disabled by secure boot and flash encryption release mode (default Kconfig).

### Signing (release key off the build machine)

`CONFIG_SECURE_BOOT_BUILD_SIGNED_BINARIES=n`: the build produces unsigned images and the release
key stays on the signing machine (an HSM, via `espsecure.py`'s PKCS#11 support, or an offline
machine):

```sh
espsecure.py sign_data --version 2 --keyfile release_signing_key.pem \
  --output bootloader-signed.bin build-prod/bootloader/bootloader.bin
espsecure.py sign_data --version 2 --keyfile release_signing_key.pem \
  --output benchpod-wifi-bridge-signed.bin build-prod/benchpod-wifi-bridge.bin
```

The signed app is both the factory image and the OTA image the pod relays.

### Factory flashing order

1. Flash the signed bootloader (0x1000), partition table (0x10000), `ota_data_initial.bin`
   (0x17000) and signed app (0x20000) in plain text, over the TC2030 pads.
2. Power up once. The bootloader enables secure boot (burns the key
   digest), generates the flash-encryption key, encrypts flash in place and resets. The app's
   first `nvs_flash_init` generates the NVS keys. This takes a few seconds; do not cut power.
3. Check with `espefuse.py summary` *before* step 2 on the first boards of a batch (ROM download
   mode is gone afterwards if it is disabled), and `version`/`status` on the console after.

### Factory image and the factory-reset button

The factory partition holds an ordinary signed app: with secure boot the bootloader verifies it
like any OTA image, and flash encryption encrypts it on the first boot like the others. The
bootloader's power-up factory reset (GPIO34 held 5 s) works the same in dev and production
builds: it erases `nvs` (Wi-Fi credentials and the pairing key) and `otadata` and boots the
factory image; it does not touch eFuses,
`nvs_keys` (the NVS keys are reused, the erased NVS starts empty) or the signature checks. The
factory image is as old as the factory flash, so the pod should update a bridge that comes back
with an old FW_VERSION. Anti-rollback conflicts with this: once a newer image raises the eFuse
secure version above the factory image's, the bootloader refuses the factory image and the
button reset leaves nothing bootable. Do not enable anti-rollback without redesigning that.

### Still open

- **Anti-rollback** (`CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK`, secure version in eFuse BLOCK3):
  stops an attacker from installing an older, validly signed image with a known bug. Needs a
  decision on eFuse BLOCK3 use on ESP32; not enabled yet.
- **Key management:** generate the release key in an HSM (there is only one key and no
  revocation on ESP32), document who can sign.
- **Vulnerability disclosure and update policy** (EN 18031 / Cyber Trust Mark paperwork): the
  repo's top-level `SECURITY.md` covers disclosure; support period still to define.
- **Pairing is trust on first use:** a device on the cable before the pod (or right after a PAIR
  press) could pair instead; the pod sees ALREADY_PAIRED and should tell the user. The key
  crosses the cable once, in the clear, at pairing. Passphrases in SET_WIFI are integrity-
  protected but not encrypted. Good enough for a point-to-point cable; for a shared switch,
  encrypt SET_WIFI with a key derived from the pairing key.
- None of the above has been run on a real chip: eFuse burning is irreversible, so try it on a
  sacrificial rev A board first.
