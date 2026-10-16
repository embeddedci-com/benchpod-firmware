#!/usr/bin/env bash
# Build the Wi-Fi bridge firmware in the pinned ESP-IDF Docker image (idf-image).
#
#   ./build.sh              dev build -> build/benchpod-wifi-bridge.bin (+ bootloader, partition table)
#   ./build.sh <idf.py args> anything else for idf.py, e.g. ./build.sh menuconfig, ./build.sh size
#
# Dev builds sign the app with keys/dev_signing_key.pem (RSA-3072), generated on the first build
# and never committed. A bridge only accepts OTA images signed with the key of the image it runs.
set -euo pipefail
cd "$(dirname "$0")"

if [ "${1:-}" = "--in-container" ]; then
  shift
  if [ ! -f keys/dev_signing_key.pem ]; then
    echo "=== generating dev signing key keys/dev_signing_key.pem (not for production) ==="
    mkdir -p keys
    python -m espsecure generate_signing_key --version 2 --scheme rsa3072 keys/dev_signing_key.pem
  fi
  if [ $# -eq 0 ]; then
    idf.py build
    idf.py size
    echo "=== artifacts ==="
    ls -l build/benchpod-wifi-bridge.bin build/bootloader/bootloader.bin build/partition_table/partition-table.bin
  else
    idf.py "$@"
  fi
  exit 0
fi

tty=()
[ -t 0 ] && tty=(-it)
exec docker run --rm "${tty[@]}" -v "$PWD:/project" -w /project "$(cat idf-image)" \
  bash /project/build.sh --in-container "$@"
