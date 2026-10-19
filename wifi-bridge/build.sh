#!/usr/bin/env bash
# Build the Wi-Fi bridge firmware in the pinned ESP-IDF Docker image (idf-image).
#
#   ./build.sh              dev build -> build/benchpod-wifi-bridge.bin (+ bootloader, partition table)
#   ./build.sh production   production build (secure boot v2, flash encryption) -> build-prod/, unsigned
#   ./build.sh <idf.py args> anything else for idf.py, e.g. ./build.sh menuconfig, ./build.sh size
#
# Updates: the bridge installs only images that come with the pod's signed manifest (Ed25519, see
# components/bridge_core/fwsig.h). The keys it trusts are compiled in from main/fw_keys_gen.h,
# generated here on the host from ../stm32h563/keys/release-*.pub plus, unless RELEASE=1, the
# pod's developer key (~/.config/benchpod/fw-signing-dev.key, the one `make -C ../stm32h563
# dev-key` makes). Production builds add ESP32 secure boot v2 (RSA-3072, see docs/SECURITY.md).
set -euo pipefail
cd "$(dirname "$0")"

if [ "${1:-}" = "--in-container" ]; then
  shift
  if [ "${1:-}" = "production" ]; then
    # Separate build dir and sdkconfig, so dev and production settings never mix.
    idf.py -B build-prod -D SDKCONFIG=build-prod/sdkconfig \
      -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.production" build
    echo "=== unsigned production artifacts: sign them on the release signing machine ==="
    ls -l build-prod/benchpod-wifi-bridge.bin build-prod/bootloader/bootloader.bin \
      build-prod/partition_table/partition-table.bin
  elif [ $# -eq 0 ]; then
    idf.py build
    idf.py size
    echo "=== artifacts ==="
    ls -l build/benchpod-wifi-bridge.bin build/bootloader/bootloader.bin build/partition_table/partition-table.bin
  else
    idf.py "$@"
  fi
  exit 0
fi

# Trusted update keys -> main/fw_keys_gen.h (host side: the container only sees wifi-bridge/).
pubs=(../stm32h563/keys/release-*.pub)
dev_key="${DEV_SIGN_KEY:-$HOME/.config/benchpod/fw-signing-dev.key}"
if [ "${RELEASE:-0}" != 1 ] && [ -f "$dev_key" ]; then
  mkdir -p build-keys
  if python3 ../tools/fwsign.py pubkey "$dev_key" 2>/dev/null | cut -d' ' -f1 > build-keys/dev-signing.pub &&
     [ -s build-keys/dev-signing.pub ]; then
    pubs+=(build-keys/dev-signing.pub)
  else
    echo "warning: could not read the developer key $dev_key (pip3 install cryptography); release keys only" >&2
  fi
fi
args=()
for p in "${pubs[@]}"; do args+=(--pub "$p"); done
python3 ../tools/fwsign.py keys-header "${args[@]}" -o main/fw_keys_gen.h
echo "=== trusted update keys: ${#pubs[@]} (${pubs[*]##*/}) ==="

tty=()
[ -t 0 ] && tty=(-it)
docker run --rm "${tty[@]}" -v "$PWD:/project" -w /project "$(cat idf-image)" \
  bash /project/build.sh --in-container "$@"

# A dev build signs its image with the pod's developer key, so a pod (or bridgectl) can send it
# as an update: build/benchpod-wifi-bridge.bin.sig. Release images are signed by the release job.
version=$(sed -n 's/^set(PROJECT_VER "\(.*\)")/\1/p' CMakeLists.txt)
if [ $# -eq 0 ] && [ "${RELEASE:-0}" != 1 ] && [ -f "$dev_key" ] && [ -f build/benchpod-wifi-bridge.bin ]; then
  python3 ../tools/fwsign.py sign --key "$dev_key" --target bridge --release "$version" \
    build/benchpod-wifi-bridge.bin || echo "warning: not signed (pip3 install cryptography)" >&2
fi
