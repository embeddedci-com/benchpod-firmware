#!/usr/bin/env bash
# Flash a Wi-Fi bridge over its Tag-Connect pads with an Espressif ESP-Prog or an Olimex ESP-PROG.
#
#   ./flash.sh                 flash build/ (from ./build.sh) and open the log
#   ./flash.sh -p PORT         pick the serial port (default: the programmer's UART, auto-detected)
#   ./flash.sh --erase         erase the whole flash first: no Wi-Fi settings, no pairing, factory state
#   ./flash.sh --no-monitor    flash only
#   ./flash.sh --monitor       only open the log (115200 8N1, Ctrl-] quits)
#
# Wiring: README.md, "Flash". Espressif ESP-Prog: TC2030-IDC cable into its PROG header, pin 1 =
# EN on both. Olimex ESP-PROG: the IDC's wires to its 0.1" header, TX and RX crossed, 3.3V left
# open, the bridge powered from USB-C. Either programmer drives EN and IO0 itself (DTR/RTS
# auto-reset), so no buttons are needed.
#
# Runs on the host, not in Docker (Docker on macOS cannot reach USB serial ports). Needs esptool:
#   pip3 install esptool     (or: pipx install esptool)
set -euo pipefail
cd "$(dirname "$0")"

port="" erase=0 flash=1 monitor=1 baud=460800
while [ $# -gt 0 ]; do
  case "$1" in
    -p|--port) port="$2"; shift 2 ;;
    -b|--baud) baud="$2"; shift 2 ;;
    --erase) erase=1; shift ;;
    --no-monitor) monitor=0; shift ;;
    --monitor) flash=0; shift ;;
    -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
    *) echo "unknown option $1 (try --help)" >&2; exit 2 ;;
  esac
done

esptool=()
if command -v esptool >/dev/null 2>&1; then  # esptool v5 (Homebrew, pipx)
  esptool=(esptool)
elif python3 -m esptool version >/dev/null 2>&1; then
  esptool=(python3 -m esptool)
elif command -v esptool.py >/dev/null 2>&1; then  # esptool v4
  esptool=(esptool.py)
else
  echo "esptool not found: brew install esptool, or pip3 install esptool" >&2
  exit 1
fi

if [ -z "$port" ]; then
  # Espressif's ESP-Prog is an FT2232H with two ports: channel A is JTAG, channel B (the second)
  # is the UART wired to the PROG header. macOS: /dev/cu.usbserial-<serial>0 / ...1; Linux:
  # ttyUSB0 / 1. The Olimex ESP-PROG is a CH340 with one port: /dev/cu.wchusbserial<loc> with
  # WCH's macOS driver (which can also leave a /dev/cu.usbserial-<loc> for the same chip),
  # /dev/cu.usbserial-<loc> with Apple's.
  ports=()
  for p in /dev/cu.wchusbserial* /dev/cu.usbserial-* /dev/ttyUSB*; do
    [ -e "$p" ] || continue
    case "$p" in /dev/cu.usbserial-*) [ -e "/dev/cu.wchusbserial${p#/dev/cu.usbserial-}" ] && continue ;; esac
    ports+=("$p")
  done
  case ${#ports[@]} in
    0) echo "no programmer found (no /dev/cu.wchusbserial*, /dev/cu.usbserial-* or /dev/ttyUSB*): plugged in? pass -p PORT" >&2; exit 1 ;;
    1) port="${ports[0]}" ;;
    2) port="${ports[1]}" ;;  # an ESP-Prog, sorted by the shell glob: channel B
    *) echo "several serial adapters found, pass one with -p:" >&2; printf '  %s\n' "${ports[@]}" >&2; exit 1 ;;
  esac
  echo "=== using $port ==="
fi

if [ "$flash" = 1 ]; then
  if [ ! -f build/flash_args ]; then
    echo "build/flash_args missing: run ./build.sh first" >&2
    exit 1
  fi
  if [ "$erase" = 1 ]; then
    echo "=== erasing the whole flash ==="
    "${esptool[@]}" --chip esp32 -p "$port" -b "$baud" erase_flash
  fi
  echo "=== flashing bootloader, partition table, otadata, app (factory slot) ==="
  # Defaults: reset into the ROM loader via DTR/RTS before, hard reset after (esptool v4 and v5).
  (cd build && "${esptool[@]}" --chip esp32 -p "$port" -b "$baud" write_flash @flash_args)
fi

if [ "$monitor" = 1 ]; then
  if python3 -c "import serial" 2>/dev/null; then
    echo "=== log on $port at 115200 (Ctrl-] quits; DTR/RTS released so the bridge keeps running) ==="
    exec python3 -m serial.tools.miniterm --raw --rts 0 --dtr 0 "$port" 115200
  fi
  # No pyserial: screen asserts DTR and RTS together, which the ESP-Prog's auto-reset ignores.
  echo "=== log on $port at 115200 (screen: Ctrl-A then K quits; pip3 install pyserial for miniterm) ==="
  exec screen "$port" 115200
fi
