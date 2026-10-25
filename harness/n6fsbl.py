#!/usr/bin/env python3
"""Say which first-stage boot loader a Neuro N6 runs, and whether this BSP
can rely on it. Read-only.

    python harness/n6fsbl.py

The board is sent to its bootloader if it runs an application; the bootloader
reports the CRC32 of the start of the flash for each published release's
length, and a match names the release. A loader signed again, or built by
this repository's FSBL projects, is matched on the code after its 0x400-byte
header. An unpublished loader after v2.6.2 is known by reporting FSBLLOCK.
Nothing is written; the board is left in the bootloader.

This BSP needs v2.5.0 or later: v2.0.2 to v2.4.0 switch the flash interface
to 200 MHz at a sampling point where reads lose bits, so the radio refuses
its firmware and the NPU reads wrong weights, with no error reported (see
docs/BOOT-CONTRACT.md). v2.6.2's open findings: docs/BOOT-CONTRACT.md, "Known
issues in v2.6.2".

Exit code 0 when this BSP can rely on the loader, 1 when it is recognised but
too old, 2 when it is not recognised, 3 when there is no bootloader to ask,
its port will not open or it does not answer, and 4 when the link to it fails
part-way.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: BSD-3-Clause
# Licensed under the terms in the LICENSE file at the root of this
# repository.

import sys

from n6weights import fail, open_session, to_bootloader
from utils.dfu import DfuError

REQUIRED = (2, 5, 0)

# The published releases (binaries/ in the Neuro-N6-Bootloader repository):
# version, bytes, CRC32 of the image, header included.
RELEASES = [
    ("2.6.2", 83744, 0x3C092073),
    ("2.6.1", 83744, 0xA1CBFD65),
    ("2.6.0", 83680, 0x2E77445C),
    ("2.5.0", 84672, 0x933535FB),
    ("2.4.0", 82944, 0x5A5F9417),
    ("2.3.1", 82944, 0xBA47754D),
    ("2.2.2", 82048, 0xDBC5FE2B),
    ("2.2.0", 81984, 0xFCDB2624),
    ("2.1.0", 80960, 0x0C0B0A2A),
    ("2.0.2", 79136, 0x306B8D2B),
    ("2.0.1", 79136, 0x674E7585),
]

# Code after the image header: version, bytes, CRC32, where it comes from.
CODE = [
    ("2.6.2", 82720, 0x92C1BDF4, "the release's code, signed again"),
    ("2.6.1", 82720, 0x1FC52467, "the release's code, signed again"),
    ("2.6.1", 82784, 0xF64F1266, "built by this repository's FSBL projects"),
    ("2.6.0", 82656, 0x6287BE95, "the release's code, signed again"),
    ("2.6.0", 82720, 0x188FB8C8, "built by this repository's FSBL projects"),
    ("2.5.0", 83648, 0xEF2E45AF, "the release's code, signed again"),
]
HEADER = 0x400


def version(v):
    return tuple(int(x) for x in v.split("."))


def identify(ses):
    for ver, n, crc in RELEASES:
        if ses.crc32(0, n) == crc:
            return ver, "the published image"
    for ver, n, crc, what in CODE:
        if ses.crc32(HEADER, n) == crc:
            return ver, what
    return None, None


def main():
    port = to_bootloader()
    if not port:
        fail(3, "no bootloader on the bus")
    print("bootloader on %s" % port)

    ses = open_session(port)
    try:
        ses.probe_info()
        if not ses.has_cap("CRC32"):
            print("this bootloader cannot report a CRC32, so it is older than any release this "
                  "tool knows; this BSP needs v%d.%d.%d or later" % REQUIRED)
            return 1
        ver, what = identify(ses)
        after_261 = ver is None and ses.has_cap("FSBLLOCK")
    except DfuError as e:
        fail(4, "bootloader: %s" % e)
    finally:
        ses.close()

    if after_261:
        print("the next boot loader, after v2.6.2, which is not a published release: "
              "this BSP can rely on it")
        return 0
    if ver is None:
        print("not a boot loader this tool knows: neither a published release nor a known build")
        return 2
    ok = version(ver) >= REQUIRED
    print("FSBL v%s, %s: %s" % (ver, what,
          "this BSP can rely on it" if ok else
          "too old; this BSP needs v%d.%d.%d or later" % REQUIRED))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
