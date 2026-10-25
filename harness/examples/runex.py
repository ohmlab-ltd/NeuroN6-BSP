"""Flash one example, watch it run, and prove 'b' still brings the bootloader back.

    python runex.py Blink [--seconds 4] [--send "text"]

An image that ignores 'b' cannot be replaced without a hand on the board, so
every example goes through this; the exit code says whether it came back.
Only the board's own vendor id is opened, so another USB serial device is
never mistaken for the board.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: BSD-3-Clause
# Licensed under the terms in the LICENSE file at the root of this
# repository.

import argparse
import os
import subprocess
import sys
import time

import serial
import serial.tools.list_ports as lp

HARNESS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
CUBE = os.environ.get("NEURON6_CUBE_DIR", "")
sys.path.insert(0, HARNESS)
import n6flash  # noqa: E402

APP, BOOT = "8008", "8009"


def port_of(pid):
    for p in lp.comports():
        if ("VID:PID=1209:%s" % pid) in (p.hwid or "").upper().replace("VID_", "VID:").replace("&PID_", ":"):
            return p.device
        if ("1209:%s" % pid) in (p.hwid or ""):
            return p.device
    return None


def wait_pid(pid, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        p, port = n6flash.bus()
        if p == pid and port:
            return port
        time.sleep(0.4)
    return None


def to_bootloader():
    p, port = n6flash.bus()
    if p == BOOT:
        return port
    if p == APP and port:
        try:
            with serial.Serial(port, 115200, timeout=1, write_timeout=2) as s:
                s.dtr = True
                s.write(b"b\n")
                s.flush()
        except serial.SerialException:
            pass
    return wait_pid(BOOT, 30)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name")
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--send", default=None)
    a = ap.parse_args()

    # The repository's own project first (harness/examples/cubebuild.sh, or
    # CubeIDE, builds it there), then a CubeMX workspace's clone of it.
    img = os.path.join(os.path.dirname(HARNESS), "Examples", a.name, "STM32CubeIDE", "Appli",
                       "Debug", a.name + "_Appli_Trusted.bin")
    if not os.path.isfile(img) and CUBE:
        proj = "NeuroN6 - Ex " + a.name
        img = os.path.join(CUBE, proj, "Appli", "Debug", proj + "_Appli_Trusted.bin")
    if not os.path.isfile(img):
        print("no image:", img)
        return 2

    if not to_bootloader():
        print("FAIL: the bootloader is not on the bus and could not be reached")
        return 3

    r = subprocess.run([sys.executable, os.path.join(HARNESS, "n6flash.py"), img,
                        "--no-reboot-test"], capture_output=True, text=True)
    print("\n".join("  " + l for l in r.stdout.splitlines()[-4:]))
    if r.returncode != 0:
        p, _ = n6flash.bus()
        print("FAIL: flash/enumerate rc=%d; bus now shows %s" % (r.returncode, p or "nothing"))
        print(r.stderr[-600:])
        return 4

    port = wait_pid(APP, 15)
    if not port:
        print("FAIL: application did not enumerate")
        return 5

    # Watch it run.
    lines = []
    s = None
    t0 = time.time()
    while s is None and time.time() - t0 < 10:
        try:
            s = serial.Serial(port, 115200, timeout=0.2, write_timeout=2,
                              dsrdtr=False, rtscts=False)
        except serial.SerialException:
            time.sleep(0.3)
    if s is None:
        print("FAIL: %s never became openable" % port)
        return 6
    try:
        s.dtr = True
        s.rts = True
        time.sleep(0.2)
        s.reset_input_buffer()
        if a.send is not None:
            s.write(a.send.encode())
            s.flush()
        buf = b""
        t0 = time.time()
        while time.time() - t0 < a.seconds:
            chunk = s.read(512)
            if chunk:
                buf += chunk
        for l in buf.decode(errors="replace").splitlines():
            if l.strip():
                lines.append(l.rstrip("\r"))
        # And bring it back.
        s.write(b"b\n")
        s.flush()
        time.sleep(0.3)
    finally:
        try:
            s.close()
        except Exception:
            pass

    back = wait_pid(BOOT, 20)

    print("---- output (%d lines in %.0fs) ----" % (len(lines), a.seconds))
    for l in lines[:40]:
        print("  " + l)
    if len(lines) > 40:
        print("  ... %d more" % (len(lines) - 40))
    if not back:
        p, _ = n6flash.bus()
        print("FAIL: 'b' did not return the bootloader; bus shows %s" % (p or "nothing"))
        return 7
    print("OK: %s ran on %s and 'b' returned the bootloader on %s" % (a.name, port, back))
    return 0


if __name__ == "__main__":
    sys.exit(main())
