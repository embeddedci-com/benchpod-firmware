#!/usr/bin/env bash
# Sign the assets of a release that was published before CI signed releases
# (docs/design/firmware-signing.md), so older firmware stays installable once pods check.
#
#   tools/sign_published_release.sh <tag> <key file> [--upload]
#
# Downloads bench_pod_stm32.bin + the blobs, checks them against the release's own
# bench_pod_stm32.bin.sha256 and blobs-manifest.json (what was published, not a rebuild),
# signs each into <asset>.sig, verifies the result against keys/release-N.pub, and with
# --upload attaches the .sig files to the release. Without --upload nothing leaves this machine.
set -euo pipefail

TAG=${1:?usage: $0 <tag> <key file> [--upload]}
KEY=${2:?usage: $0 <tag> <key file> [--upload]}
UPLOAD=${3:-}
REPO=${REPO:-embeddedci-com/benchpod-firmware}
HERE=$(cd "$(dirname "$0")" && pwd)
FWSIGN="python3 $HERE/fwsign.py"
PUBS="--pub $HERE/../stm32h563/keys/release-1.pub --pub $HERE/../stm32h563/keys/release-2.pub"
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
cd "$WORK"

gh release download "$TAG" --repo "$REPO" --pattern 'bench_pod_stm32.bin*' --pattern 'blob-*.bin' \
  --pattern 'blobs-manifest.json'
shasum -a 256 -c bench_pod_stm32.bin.sha256
python3 - <<'PY'
import hashlib, json, os, sys
m = json.load(open("blobs-manifest.json"))
for b in m["blobs"]:
    if not os.path.exists(b["file"]):
        sys.exit("%s is listed but not published" % b["file"])
    got = hashlib.sha256(open(b["file"], "rb").read()).hexdigest()
    if got != b["sha256"]:
        sys.exit("%s does not match blobs-manifest.json" % b["file"])
    print("%s: matches blobs-manifest.json" % b["file"])
PY

$FWSIGN sign --key "$KEY" --target firmware --release "${TAG#v}" bench_pod_stm32.bin
$FWSIGN sign-blobs --key "$KEY" --release "${TAG#v}" blobs-manifest.json
$FWSIGN verify $PUBS --target firmware bench_pod_stm32.bin >/dev/null
for b in gw0 gw1 esp; do $FWSIGN verify $PUBS --target $b blob-$b.bin >/dev/null; done
echo "all four signatures verify against keys/release-N.pub"

SIGS="bench_pod_stm32.bin.sig blob-gw0.bin.sig blob-gw1.bin.sig blob-esp.bin.sig"
if [ "$UPLOAD" = "--upload" ]; then
  gh release upload "$TAG" --repo "$REPO" $SIGS
  echo "uploaded to $TAG: $SIGS"
else
  OUT=${OUT:-$HERE/../stm32h563/build/signed-$TAG}
  mkdir -p "$OUT"
  cp $SIGS "$OUT/"
  echo "not uploaded (pass --upload); the .sig files are in $OUT"
fi
