#!/usr/bin/env bash
# Build the Wi-Fi bridge firmware in the pinned ESP-IDF Docker image (idf-image).
#
#   ./build.sh              dev build -> build/benchpod-wifi-bridge.bin (+ bootloader, partition table)
#   ./build.sh production   production build (secure boot v2, flash encryption) -> build-prod/, unsigned
#   ./build.sh emc          EMC test build (README "EMC test build") -> build-emc/; development only:
#                           refused with RELEASE=1, never signed, never released
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
  elif [ "${1:-}" = "emc" ]; then
    idf.py -B build-emc -D SDKCONFIG=build-emc/sdkconfig -D BRIDGE_EMC_BUILD=1 \
      -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.emc;build-emc/overrides" build
    echo "=== EMC test build (development only): flash with ./flash.sh --emc ==="
    ls -l build-emc/benchpod-wifi-bridge.bin build-emc/bootloader/bootloader.bin \
      build-emc/partition_table/partition-table.bin
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

if [ "${1:-}" = "emc" ] && [ "${RELEASE:-0}" = 1 ]; then
  echo "the EMC test build is development only and cannot be combined with RELEASE=1" >&2
  exit 2
fi

# EMC build options (README "EMC test build"): EMC_RADIO=0 (radio silent at boot), EMC_ETH_MBIT=<n>
# (Ethernet stream at boot). Written as an sdkconfig overlay; the build's sdkconfig is regenerated
# every time so a previous build's options never stick.
if [ "${1:-}" = "emc" ]; then
  mkdir -p build-emc
  rm -f build-emc/sdkconfig
  {
    [ "${EMC_RADIO:-1}" = 0 ] && echo "# CONFIG_BRIDGE_EMC_RADIO is not set"
    echo "CONFIG_BRIDGE_EMC_ETH_MBIT=${EMC_ETH_MBIT:-0}"
  } > build-emc/overrides
  echo "=== EMC build: radio $([ "${EMC_RADIO:-1}" = 0 ] && echo silent || echo transmitting) at boot," \
    "Ethernet stream ${EMC_ETH_MBIT:-0} Mbit/s ==="
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
# The EMC test build too, so the pod can send it (README "EMC test build"): only dev images trust
# the dev key. Asset version 0: the pod never installs it on its own, only with `bridge update`.
# build-emc/blobs/ holds it as the pod's blob slot "bridge" for `benchpod install-blobs --dir`.
if [ "${1:-}" = emc ] && [ -f "$dev_key" ] && [ -f build-emc/benchpod-wifi-bridge.bin ]; then
  mkdir -p build-emc/blobs
  cp build-emc/benchpod-wifi-bridge.bin build-emc/blobs/blob-bridge.bin
  if python3 ../tools/fwsign.py sign --key "$dev_key" --target bridge --release "$version" --version 0 \
       build-emc/blobs/blob-bridge.bin; then
    python3 - build-emc/blobs <<'PY'
import hashlib, json, os, sys
d = sys.argv[1]
data = open(os.path.join(d, "blob-bridge.bin"), "rb").read()
json.dump({"format": 1, "blobs": [{"name": "bridge", "file": "blob-bridge.bin", "size": len(data),
           "sha256": hashlib.sha256(data).hexdigest(), "version": 0}]},
          open(os.path.join(d, "blobs-manifest.json"), "w"), indent=2)
PY
  else
    echo "warning: not signed (pip3 install cryptography)" >&2
  fi
fi
