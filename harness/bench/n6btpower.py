#!/usr/bin/env python3
"""Check BLE transmit power on a board running
Examples/WiFiApi_FreeRTOS against the 8 dBm cap (Windows).

    python harness/bench/n6btpower.py --port PORT [--json out.json] [--log console.log]

The board reads its controller's transmit power back (`btpower`) for:
advertising, before and after NXP's power command asking 20 dBm with its
front-end loss flag set (neuron6_bt sends 8 dBm, flag cleared); extended
advertising on 1M/1M, 1M/2M and Coded/Coded, asking for no preference and for
20 dBm; and a connection from this PC (n6radio.exe, from build_n6radio.py),
its current and highest levels, overall and on each PHY.

Every level in use must be 8 dBm or less. The highest levels the controller
reports for a connection are the commands' generic range, not this module's
(its calibration sets the most the standard commands give: NXP AN13006 rev 3,
section 3.5), so they are reported, not judged.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT

import argparse
import json
import os
import re
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from n6wifiapi import Shell   # noqa: E402

CAP_DBM = 8


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="the board's console port (COMn)")
    ap.add_argument("--seconds", type=int, default=25,
                    help="how long the board waits for this PC to connect")
    ap.add_argument("--log", default=os.path.join(tempfile.gettempdir(), "n6btpower.log"))
    ap.add_argument("--json", default=None)
    a = ap.parse_args()

    exe = os.path.join(tempfile.gettempdir(), "n6bench", "n6radio.exe")
    checks, notes = [], []

    def check(name, ok, detail=""):
        checks.append({"check": name, "ok": bool(ok), "detail": detail})
        print("%s  %s%s" % ("PASS" if ok else "FAIL", name, ("  -- " + detail) if detail else ""))

    sh = Shell(a.port, a.log)
    gatt = None
    try:
        mark = sh.mark()
        sh.log.write(">> btpower %d\n" % a.seconds)
        sh.s.write(("btpower %d\n" % a.seconds).encode())
        line = sh.wait_for(r"\[api\] btpower addr ([0-9A-F:]{17})", 40, since=mark)
        if line and os.path.exists(exe):
            addr = re.search(r"([0-9A-F:]{17})", line).group(1)
            gatt = subprocess.Popen([exe, "gatt", addr, "--seconds", "8"],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        done = sh.wait_for(r"\[api\] (ok|fail) btpower", a.seconds + 40, since=mark)
        lines = [l[len("[api] "):] for l in sh.lines[mark:] if l.startswith("[api] btpower")]
    finally:
        if gatt is not None:
            try:
                gatt.wait(timeout=30)
            except subprocess.TimeoutExpired:
                gatt.kill()
        sh.close()

    check("the board ran btpower", bool(done) and "ok btpower" in done, done or "no answer")

    for l in lines:
        m = re.match(r"btpower adv default=(-?\d+) dBm r=(-?\d+)", l)
        if m:
            check("advertising, by default, %s dBm" % m.group(1),
                  m.group(2) == "0" and int(m.group(1)) <= CAP_DBM)
        m = re.match(r"btpower adv after-fc87-20=(-?\d+) dBm fc87=(-?\d+) r=(-?\d+)", l)
        if m:
            check("advertising after the power command asked 20 dBm, %s dBm" % m.group(1),
                  m.group(3) == "0" and int(m.group(1)) <= CAP_DBM, "power command r=%s" % m.group(2))
        m = re.match(r"btpower ext primary=(\d) secondary=(\d) ask=(-?\d+) selected=(-?\d+) dBm r=(-?\d+)", l)
        if m:
            phy = {"1": "1M", "2": "2M", "3": "Coded"}
            check("extended advertising %s/%s asking %s, selected %s dBm"
                  % (phy[m.group(1)], phy[m.group(2)], "no preference" if m.group(3) == "127" else m.group(3) + " dBm",
                     m.group(4)),
                  m.group(5) == "0" and int(m.group(4)) <= CAP_DBM)
        m = re.match(r"btpower conn handle=(\d+) (current|max)=(-?\d+) dBm r=(-?\d+)", l)
        if m:
            if m.group(2) == "current":
                check("connected, the level in use, %s dBm" % m.group(3),
                      m.group(4) == "0" and int(m.group(3)) <= CAP_DBM)
            else:
                notes.append("the controller's highest level for the connection: %s dBm (r=%s)"
                             % (m.group(3), m.group(4)))
        m = re.match(r"btpower conn phy=(\d) current=(-?\d+) max=(-?\d+) dBm r=(-?\d+)", l)
        if m:
            phy = {"1": "1M", "2": "2M", "3": "Coded S8", "4": "Coded S2"}[m.group(1)]
            check("connected on %s, the level in use, %s dBm" % (phy, m.group(2)),
                  m.group(4) == "0" and int(m.group(2)) <= CAP_DBM)
            notes.append("on %s, the controller's highest level: %s dBm" % (phy, m.group(3)))
        if l.startswith("btpower conn none"):
            check("this PC connected to the board", False, l)

    for n in notes:
        print("NOTE  " + n)
    passed = sum(1 for c in checks if c["ok"])
    print("\n%d passed, %d failed" % (passed, len(checks) - passed))
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump({"checks": checks, "notes": notes, "lines": lines}, f, indent=2)
    return 0 if passed == len(checks) else 1


if __name__ == "__main__":
    sys.exit(main())
