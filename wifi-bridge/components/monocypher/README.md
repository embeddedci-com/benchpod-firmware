# Monocypher 4.0.2 (vendored)

`monocypher.c`, `monocypher.h` and the optional `monocypher-ed25519.c`/`.h` are copied
unmodified from [Monocypher](https://monocypher.org) 4.0.2 (upstream commit 0d85f98, the same
version the pod firmware uses as a submodule in `../../../stm32h563/lib/monocypher`), so the bridge
and the pod run the same X25519, XChaCha20, BLAKE2b and Ed25519 code. Dual-licensed BSD-2-Clause / CC0, see `LICENCE.md`.

Vendored rather than referenced because the bridge's Docker build only mounts `wifi-bridge/`.
Update both copies together.
