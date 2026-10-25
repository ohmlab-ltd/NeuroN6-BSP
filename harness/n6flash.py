#!/usr/bin/env python3
"""Flash a Neuro N6 application image through the bootloader and prove it runs.

    python harness/n6flash.py <app>_Trusted.bin [--no-reboot-test] [--timeout S]

Stops at the first step that fails, and names it:
  1. Needs exactly one Neuro N6, in its bootloader (VID 0x1209 / PID 0x8009).
  2. Uploads the image to 0x90100000 with harness/uploader. The address is
     fixed: the tool never writes the bootloader at 0x90000000, nor a boot
     loader's image (*_FSBL_Trusted.bin) anywhere.
  3. Waits for the application to enumerate (PID 0x8008).
  4. Unless --no-reboot-test: sends 'b' on the application's port and waits
     for the bootloader to come back, so the board can be flashed again
     without a hand on it.

Exit code 0 only when every step passed.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: BSD-3-Clause
# Licensed under the terms in the LICENSE file at the root of this
# repository.

import argparse
import os
import re
import subprocess
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial is required: pip install pyserial", file=sys.stderr)
    sys.exit(2)

HERE = os.path.dirname(os.path.abspath(__file__))
UPLOADER = os.path.join(HERE, "uploader", "neuro-n6-uploader-cli.py")
sys.path.insert(0, os.path.join(HERE, "uploader"))

from utils.dfu import looks_like_fsbl  # noqa: E402

VID = "1209"
PID_APP = "8008"
PID_BOOTLOADER = "8009"
APP_ADDRESS = "0x90100000"      # the application slot, and the only one written
UPLOAD_TIMEOUT_S = 300          # far beyond any application slot's erase and write


def devices():
    """Every Neuro N6 console present, as (pid, com_port) pairs. The console's
    interface number varies by build, so any interface carrying the serial
    port matches."""
    out = subprocess.run(
        ["powershell", "-NoProfile", "-Command",
         "Get-PnpDevice -PresentOnly -EA SilentlyContinue | "
         "?{ $_.InstanceId -match 'VID_%s&PID_(%s|%s)&MI_' } | "
         "%%{ $_.InstanceId + ' ' + $_.FriendlyName }" % (VID, PID_APP, PID_BOOTLOADER)],
        capture_output=True, text=True).stdout
    found = []
    for line in out.splitlines():
        m = re.search(r"\((COM\d+)\)", line)
        if not m:
            continue
        if ("PID_" + PID_APP) in line:
            found.append((PID_APP, m.group(1)))
        elif ("PID_" + PID_BOOTLOADER) in line:
            found.append((PID_BOOTLOADER, m.group(1)))
    return found


def bus():
    """Return (pid, com_port) for the Neuro N6 console present, or
    (None, None). With several boards connected this is the first;
    several_boards() says so beforehand."""
    found = devices()
    return found[0] if found else (None, None)


def several_boards():
    """None with at most one Neuro N6 connected; otherwise a message naming
    each, for a tool to refuse with rather than pick one of them."""
    found = devices()
    if len(found) < 2:
        return None
    return ("more than one Neuro N6 is connected (%s); connect one at a time"
            % ", ".join("%s, %s" % (port, "application" if pid == PID_APP else "bootloader")
                        for pid, port in found))


def wait_for(pid, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        p, port = bus()
        if p == pid and port:
            return port, time.time() - t0
        time.sleep(0.4)
    return None, timeout


def step(n, text):
    print("[%d] %s" % (n, text))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("image", help="application image (<app>_Trusted.bin)")
    ap.add_argument("--no-reboot-test", action="store_true",
                    help="stop after the application enumerates; do not send 'b'")
    ap.add_argument("--timeout", type=float, default=20.0,
                    help="seconds to wait for each enumeration (default 20)")
    args = ap.parse_args()

    image = os.path.abspath(args.image)
    if not os.path.isfile(image):
        print("no such image: %s" % image, file=sys.stderr); return 2
    if not image.endswith("_Trusted.bin"):
        print("refusing: %s is not a boot image (expected *_Trusted.bin)" % image,
              file=sys.stderr); return 2
    if not os.path.isfile(UPLOADER):
        print("uploader not found: %s" % UPLOADER, file=sys.stderr); return 2
    with open(image, "rb") as f:
        head = f.read(0x408)
    if looks_like_fsbl(head):
        print("refusing: %s is a boot loader's image, not an application's" % image,
              file=sys.stderr); return 2

    t_all = time.time()

    step(1, "bootloader present?")
    several = several_boards()
    if several:
        print("    no: %s" % several, file=sys.stderr)
        return 3
    pid, port = bus()
    if pid != PID_BOOTLOADER or not port:
        print("    no: bus shows %s - put the board in bootloader mode first"
              % (pid or "nothing from the board"), file=sys.stderr)
        return 3
    print("    yes: %s" % port)

    step(2, "upload %s (%d bytes) to %s" % (os.path.basename(image),
                                            os.path.getsize(image), APP_ADDRESS))
    try:
        r = subprocess.run([sys.executable, UPLOADER, "--port", port, "--bin", image,
                            "--address", APP_ADDRESS, "--no-reboot"],
                           capture_output=True, text=True, timeout=UPLOAD_TIMEOUT_S)
    except subprocess.TimeoutExpired as e:
        tail = e.stdout or ""
        if isinstance(tail, bytes):
            tail = tail.decode(errors="ignore")
        print("    upload failed: no result after %d s\n%s" % (UPLOAD_TIMEOUT_S, tail[-1500:]),
              file=sys.stderr)
        return 4
    wrote = next((l.strip() for l in r.stdout.splitlines() if "Wrote" in l), "")
    if r.returncode != 0:
        print("    upload failed (rc=%d)\n%s" % (r.returncode, (r.stdout + r.stderr)[-1500:]),
              file=sys.stderr)
        return 4
    print("    %s" % (wrote or "done"))

    step(3, "application enumerates?")
    port, dt = wait_for(PID_APP, args.timeout)
    if not port:
        p, _ = bus()
        print("    no: after %.0fs the bus shows %s" % (args.timeout, p or "nothing"),
              file=sys.stderr)
        return 5
    print("    yes: %s after %.1fs" % (port, dt))

    if args.no_reboot_test:
        print("done in %.1fs" % (time.time() - t_all)); return 0

    step(4, "'b' on %s returns the bootloader?" % port)
    try:
        # Windows lists the CDC port before it can be opened, longer while the
        # application is busy starting: retry, and say how long a port that
        # never opens (USB not being serviced) was tried.
        s = None; t0 = time.time(); last = ""
        while s is None and time.time() - t0 < args.timeout:
            try:
                s = serial.Serial(port, 115200, timeout=1, write_timeout=1,
                                  dsrdtr=False, rtscts=False)
            except serial.SerialException as e:
                last = str(e); time.sleep(0.5)
        if s is None:
            raise serial.SerialException("%s not openable in %.0fs: %s" % (port, args.timeout, last))
        print("    port opened after %.1fs" % (time.time() - t0))
        s.dtr = True; s.rts = True; time.sleep(0.1)
        s.reset_input_buffer(); s.reset_output_buffer()
        s.write(b"b\n"); s.flush(); time.sleep(0.5)
        try:
            s.close()
        except Exception:
            pass                     # on a good reboot the port vanishes while open
    except serial.SerialException as e:
        print("    could not send: %s" % e, file=sys.stderr); return 6
    port, dt = wait_for(PID_BOOTLOADER, args.timeout)
    if not port:
        p, _ = bus()
        print("    no: after %.0fs the bus shows %s" % (args.timeout, p or "nothing"),
              file=sys.stderr)
        return 7
    print("    yes: %s after %.1fs" % (port, dt))

    print("ROUND TRIP OK in %.1fs, no hands on the board" % (time.time() - t_all))
    return 0


if __name__ == "__main__":
    sys.exit(main())
