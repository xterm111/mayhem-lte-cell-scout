#!/usr/bin/env bash
# Build v2.4.0 with LTE Cell Scout in the project container, then run the gate.
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd -P)"
mkdir -p "$root/cellscout/build"
# The packager does not notice a changed baseband image by itself: it runs as
# part of the step that writes application.bin, so force that step.
rm -f "$root/build/firmware/application/application.bin" "$root/build/firmware/application/ltecellscout.ppma"
log="$root/cellscout/build/build.log"

if ! podman run --rm --userns=keep-id -u "$(id -u):$(id -g)" \
    -e VERSION_STRING=v2.4.0 -v "$root:/havoc:Z" portapack-dev ninja -j"$(nproc)" >"$log" 2>&1; then
    grep -E -A6 "error|Error|FAILED|overflow|exceed" "$log" | head -60
    exit 1
fi
grep -E "ltecellscout.*warning|warning:.*ltecellscout" "$log" | head -20 || true
python3 -I "$root/cellscout/tools/gate.py" "$root"

# lay out what goes on the SD card (only reached if the gate passed)
dist="$root/cellscout/dist"
rm -rf "$dist"
mkdir -p "$dist/APPS" "$dist/CELLSCOUT"
cp "$root/build/firmware/application/ltecellscout.ppma" "$dist/APPS/"
cp "$root"/sdcard/CELLSCOUT/*.TXT "$dist/CELLSCOUT/"
echo "SD card files: $dist"
