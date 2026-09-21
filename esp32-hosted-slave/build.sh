#!/usr/bin/env bash
set -euo pipefail
echo "=== IDF version ==="; idf.py --version || true
cd /work

# Materialize the slave example, pinning esp_hosted to the version at commit 8f0770d.
if [ ! -d slave ]; then
  echo "=== create-project-from-example (pinned esp_hosted==2.12.9) ==="
  idf.py create-project-from-example "espressif/esp_hosted==2.12.9:slave" \
    || idf.py create-project-from-example "espressif/esp_hosted^2.12.9:slave" \
    || idf.py create-project-from-example "espressif/esp_hosted:slave"
fi

cd slave
cp /board/sdkconfig.defaults.board .
export SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.esp32c3;sdkconfig.defaults.board"

# Pin transitive component versions (mqtt/iperf/ping-cmd/wifi-cmd/... pulled in by
# the upstream example) to the committed lock, instead of re-resolving from the
# component registry on every build. Without this, CONFIG_APP_REPRODUCIBLE_BUILD
# alone isn't enough for a byte-identical rebuild: it only removes build-instruction
# nondeterminism (timestamps, paths), not "the registry served a newer patch
# version this time". The component manager honors an existing lock that matches
# the current manifests instead of re-reconciling.
cp /board/dependencies.lock .

echo "=== set-target esp32c3 ==="
idf.py set-target esp32c3

echo "=== verify board SPI symbols resolved ==="
grep -E "CONFIG_ESP_SPI_HOST_INTERFACE|CONFIG_ESP_HOSTED_SPI_GPIO_(HANDSHAKE|DATA_READY|MOSI|MISO|CLK|CS)|CONFIG_ESP_HOSTED_SPI_MODE" sdkconfig | sort || true

echo "=== build ==="
idf.py build

echo "=== merge to single flashable image ==="
mkdir -p /out
idf.py merge-bin -o /out/esp32c3-hosted-slave-merged.bin || true
cp -v build/flash_args /out/ 2>/dev/null || true
cp -v build/bootloader/bootloader.bin /out/ 2>/dev/null || true
cp -v build/partition_table/partition-table.bin /out/ 2>/dev/null || true
cp -v build/network_adapter.bin /out/ 2>/dev/null || true
cp -v build/*.bin /out/ 2>/dev/null || true
echo "=== artifacts ==="; ls -la /out
