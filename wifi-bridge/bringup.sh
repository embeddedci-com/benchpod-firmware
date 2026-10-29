#!/usr/bin/env bash
# Bring up a new Wi-Fi bridge board and the pod it plugs into, in one go.
#
#   ./bringup.sh               bridge + pod: build both, flash the bridge, flash the pod
#   ./bringup.sh bridge        build and flash the bridge only (ESP-Prog + TC2030 on J2)
#   ./bringup.sh pod           build and flash the pod only (USB), with the bridge client and image
#   ./bringup.sh -n ...        print the commands instead of running them
#
# Options:
#   --no-build                 flash what is already built
#   --no-erase                 keep the bridge's flash (Wi-Fi settings, pairing); default: erase it
#   -p PORT                    the ESP-Prog's UART port (default: auto-detected by flash.sh)
#   --connection C             how benchpod reaches the pod (default: usb / the saved connection)
#
# Bridge: ./build.sh (Docker), then ./flash.sh --erase. Pod: a development build of ../stm32h563
# (WIFI_BRIDGE=1, the default outside release builds), which carries the bridge client and puts
# this bridge build into its blob slot "bridge"; then `benchpod flash-self` writes the firmware
# over USB DFU and installs the blobs from the same build. Release pod firmware has no bridge
# client yet, so a pod on a release does not see the bridge.
#
# Afterwards: cable the bridge to the pod's RJ45 and power both; the steps from "Boot" on in the
# README's bring-up checklist say what to look for. Needs esptool, dfu-util, benchpod and Docker.
set -euo pipefail
cd "$(dirname "$0")"

what=all dry=0 build=1 erase=1 port="" conn=()
while [ $# -gt 0 ]; do
  case "$1" in
    bridge|pod|all) what="$1"; shift ;;
    -n|--dry-run) dry=1; shift ;;
    --no-build) build=0; shift ;;
    --no-erase) erase=0; shift ;;
    -p|--port) port="$2"; shift 2 ;;
    --connection) conn=(--connection "$2"); shift 2 ;;
    -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
    *) echo "unknown option $1 (try --help)" >&2; exit 2 ;;
  esac
done

run() {
  echo "+ $*"
  [ "$dry" = 1 ] || "$@"
}

need() {
  for t in "$@"; do
    command -v "$t" >/dev/null 2>&1 || { echo "$t not found on PATH" >&2; [ "$dry" = 1 ] || exit 1; }
  done
}

pod_dir=../stm32h563
pod_bin=$pod_dir/build/bench_pod_stm32.bin
pod_blobs=$pod_dir/build/blobs
want_rev=$(sed -n '/config BRIDGE_HW_REV/,/default/s/.*default "\(.*\)"/\1/p' main/Kconfig.projbuild)

bridge_build() {
  need docker
  run ./build.sh
  # sdkconfig keeps the value from the first build, so a Kconfig default changed later does not
  # reach an old tree: check what this image will report in HELLO.
  if [ "$dry" = 0 ] && [ -f sdkconfig ]; then
    rev=$(sed -n 's/^CONFIG_BRIDGE_HW_REV="\(.*\)"/\1/p' sdkconfig)
    if [ "$rev" != "$want_rev" ]; then
      echo "sdkconfig says hardware rev \"$rev\", the board is rev $want_rev: rm sdkconfig and rebuild" >&2
      exit 1
    fi
    echo "=== bridge image reports hardware rev $rev ==="
  fi
}

bridge_flash() {
  need esptool
  args=(--no-monitor)
  [ "$erase" = 1 ] && args+=(--erase)
  [ -n "$port" ] && args+=(-p "$port")
  run ./flash.sh "${args[@]}"
}

pod_build() {
  # After the bridge build, so build/benchpod-wifi-bridge.bin joins the pod's blobs.
  [ "$dry" = 1 ] || [ -f build/benchpod-wifi-bridge.bin ] ||
    { echo "no bridge build (build/benchpod-wifi-bridge.bin): run ./bringup.sh bridge first" >&2; exit 1; }
  # What the pod build needs besides its sources, as CI sets it up (stm32-build.yml): the
  # submodules, the released gateware (copied from ../ice40/release/) and the ESP32-C3 image.
  [ -f "$pod_dir/lib/FreeRTOS-Kernel/include/FreeRTOS.h" ] ||
    run git -C .. submodule update --init --recursive
  [ -f ../ice40/bench_pod_fpga_vbench_pod_loop.bin ] || run make -C ../ice40 release-images
  [ "$dry" = 1 ] || [ -f ../esp32-hosted-slave/prebuilt/esp32c3-hosted-slave-merged.bin ] ||
    { echo "no ESP32-C3 image: make -C $pod_dir fetch (builds it in Docker, slow), then rerun" >&2; exit 1; }
  run make -C "$pod_dir" WIFI_BRIDGE=1
  # flash-self installs what the manifest lists; a missing gateware or C3 image would leave the
  # pod's slot as it was, so insist on all four.
  [ "$dry" = 1 ] || python3 - "$pod_blobs/blobs-manifest.json" <<'PY'
import json, sys
m = json.load(open(sys.argv[1]))
missing = [b["name"] for b in m["blobs"] if not b.get("file")]
want = {"gw0", "gw1", "esp", "bridge"} - {b["name"] for b in m["blobs"]}
if missing or want:
    sys.exit("the pod build lacks blobs: %s" % ", ".join(sorted(set(missing) | want)))
print("=== pod blobs: " + ", ".join(b["name"] for b in m["blobs"]) + " ===")
PY
}

pod_flash() {
  need benchpod dfu-util
  run benchpod "${conn[@]}" flash-self "$pod_bin" --blobs-dir "$pod_blobs" --enter-dfu
}

case "$what" in
  bridge) [ "$build" = 1 ] && bridge_build; bridge_flash ;;
  pod)    [ "$build" = 1 ] && pod_build; pod_flash ;;
  all)    if [ "$build" = 1 ]; then bridge_build; pod_build; fi; bridge_flash; pod_flash ;;
esac

cat <<'EOF'

=== done. Next ===
  bridge log:   ./flash.sh --monitor     (expect "hardware rev 2", "eth: started", "PHY addr 0")
  cable the bridge to the pod's RJ45, then on the pod console:
                bridge                   (found, paired)
                wifi-set "<ssid>" "<passphrase>"
                wifi-show                (via: wifi-bridge)
  checklist:    README.md, "Bring-up checklist"
EOF
