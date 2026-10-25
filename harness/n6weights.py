#!/usr/bin/env python3
"""Check, or write, a network's weights in the Neuro N6's external flash.

    python harness/n6weights.py Middlewares/NeuroN6/Models/Nia416/model.json
    python harness/n6weights.py Middlewares/NeuroN6/Models/Nia416/model.json --write
    python harness/n6weights.py Middlewares/NeuroN6/MayaW476/firmware/firmware.json

Weights, and the radio firmware (whose firmware.json has the same shape),
have their own flash region at the address the JSON gives; flashing an
application never touches them. By default nothing is written: the board is
sent to its bootloader, which reports the region's CRC32 for comparison, and
is left there. --write programs the region, checks it the same way and resets
the board; it needs the bootloader's CRC32 and MULTI capabilities. The region
must lie wholly inside the weights pool (0x90300000 to 0x91C7FFFF) or the
radio firmware's region (from 0x91E00000).

Exit code 0 when the weights on the board match the file.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: BSD-3-Clause
# Licensed under the terms in the LICENSE file at the root of this
# repository.

import argparse
import json
import os
import sys
import time
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "uploader"))

import n6flash  # noqa: E402
from utils.dfu import DfuSession, DfuError, FLASH_BASE, FLASH_END, enter_bootloader  # noqa: E402

# The regions weights may go to (Middlewares/NeuroN6/N6OTA/n6_ota_layout.h):
# the weights pool, and the radio firmware's region. Application slot B lies
# between them.
REGIONS = ((0x90300000, 0x91C80000), (0x91E00000, FLASH_END + 1))


def fail(code, msg):
    print(msg, file=sys.stderr)
    sys.exit(code)


def to_bootloader(timeout=30.0):
    several = n6flash.several_boards()
    if several:
        fail(3, several)
    pid, port = n6flash.bus()
    if pid == n6flash.PID_BOOTLOADER:
        return port
    if pid == n6flash.PID_APP and port:
        # 'b' on the console, then the 1200-baud touch, for an application
        # that ignores 'b' (a build without a debug console).
        return enter_bootloader(port, log=lambda *a: None, timeout_sec=timeout)
    port, _ = n6flash.wait_for(n6flash.PID_BOOTLOADER, timeout)
    return port


def open_session(port, grace=5.0):
    """Open a session on the bootloader's port, allowing a few seconds for a
    port Windows has just listed. A port that will not open, or a bootloader
    that does not answer, exits with code 3, as no bootloader does."""
    deadline = time.time() + grace
    while True:
        try:
            return DfuSession(port, log=print).open()
        except DfuError as e:
            if time.time() >= deadline:
                fail(3, "bootloader on %s: %s" % (port, e))
            time.sleep(0.3)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("model", help="the network's model.json, as harness/n6model.py writes it")
    ap.add_argument("--write", action="store_true",
                    help="program the weights, then check them")
    args = ap.parse_args()

    try:
        with open(args.model, encoding="utf-8") as f:
            w = json.load(f)["weights"]
        path = os.path.join(os.path.dirname(os.path.abspath(args.model)), w["file"])
        address = int(w["address"], 16)
        want = int(w["crc32"], 16)
        size = int(w["size"])
        with open(path, "rb") as f:
            data = f.read()
    except KeyError as e:
        fail(2, "%s has no %s" % (args.model, e))
    except (OSError, ValueError, TypeError) as e:
        fail(2, "cannot use %s: %s" % (args.model, e))
    have = zlib.crc32(data) & 0xFFFFFFFF
    if len(data) != size or have != want:
        fail(2, "%s is %d bytes, CRC32 0x%08X; model.json says %d bytes, 0x%08X"
             % (path, len(data), have, size, want))
    end = address + len(data)
    if not data or not any(lo <= address and end <= hi for lo, hi in REGIONS):
        fail(2, "refusing: 0x%08X..0x%08X is not inside the weights pool or the "
             "radio firmware's region" % (address, end - 1))
    print("%s: %d bytes for 0x%08X, CRC32 0x%08X" % (w["file"], len(data), address, want))

    port = to_bootloader()
    if not port:
        fail(3, "no bootloader on the bus")
    print("bootloader on %s" % port)

    ses = open_session(port)
    try:
        ses.probe_info()
        if not ses.has_cap("CRC32"):
            fail(4, "this bootloader cannot report a CRC32; nothing checked or written")
        offset = address - FLASH_BASE
        if args.write:
            if not ses.has_cap("MULTI"):
                fail(4, "this bootloader cannot check an image before resetting; nothing written")
            ses.program(data, offset, w["file"], next_image=True)
        got = ses.crc32(offset, len(data))
        if got is None:
            fail(5, "the bootloader gave no CRC32")
        match = (got == want)
        print("on the board: CRC32 0x%08X - %s" % (got, "MATCH" if match else "DIFFERENT"))
        if args.write:
            ses.reset_device()
            print("board reset")
        return 0 if match else 1
    except DfuError as e:
        fail(6, "bootloader: %s" % e)
    finally:
        ses.close()


if __name__ == "__main__":
    sys.exit(main())
