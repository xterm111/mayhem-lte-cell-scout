#!/usr/bin/env python3
"""Check that a build still suits the official v2.4.0 firmware.

An SD card app calls into the firmware at fixed addresses, so the application
image built next to it must equal the official one. Only the compile date and
time strings may differ.

    gate.py <repo root>
"""
import re
import struct
import sys
from pathlib import Path

DATE_OR_TIME = re.compile(rb"(?:[A-Z][a-z]{2} [ 1-3]\d 20\d\d|\d\d:\d\d:\d\d)\x00")
APP_SLOT = 32 * 1024


def main(root):
    root = Path(root)
    reference = (root / "cellscout/reference/application.bin").read_bytes()
    build = root / "build/firmware/application"
    mine = (build / "application.bin").read_bytes()
    failures = []

    if len(mine) != len(reference):
        failures.append(f"application.bin is {len(mine)} bytes, official is {len(reference)}")
    a, b = bytearray(reference), bytearray(mine)
    for match in DATE_OR_TIME.finditer(reference):
        start, end = match.span()
        if DATE_OR_TIME.fullmatch(bytes(b[start:end])):
            a[start:end] = b[start:end] = bytes(end - start)
    different = [i for i, (x, y) in enumerate(zip(a, b)) if x != y]
    if different:
        failures.append(f"{len(different)} bytes differ from official, first at {different[0]:#x}")

    app = build / "ltecellscout.ppma"
    if not app.exists():
        failures.append("ltecellscout.ppma was not built")
    else:
        data = app.read_bytes()
        m4_offset = struct.unpack_from("<I", data, 80)[0]
        # file layout: app image, baseband image, checksum word(s)
        baseband = root / "build/firmware/baseband/PLCS.bin"
        m4_size = baseband.stat().st_size if baseband.exists() else 0
        print(f"app code {m4_offset:6} bytes")
        print(f"baseband {m4_size:6} bytes")
        print(f"total    {len(data):6} bytes of {APP_SLOT} ({APP_SLOT - len(data)} free)")
        if len(data) > APP_SLOT:
            failures.append("app and baseband exceed 32 KB")
        if not m4_offset or len(data) < m4_offset + m4_size:
            failures.append("baseband was not bundled, or the package is stale")

    for failure in failures:
        print("FAIL:", failure)
    if not failures:
        print("PASS: application image matches official v2.4.0")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
