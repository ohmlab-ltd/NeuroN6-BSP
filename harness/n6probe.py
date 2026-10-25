#!/usr/bin/env python3
"""Watch the streaming image's console live while it starts its stream.

    python harness/n6probe.py [--seconds 12] [--start]

Asks the running image for its one-line report ('?'), optionally sends 's'
(--start), then repeats the report every 300 ms, timestamped, until the port
goes away or time is up. The open has a deadline: an image not servicing USB
blocks it.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: BSD-3-Clause
# Licensed under the terms in the LICENSE file at the root of this
# repository.

import argparse
import os
import sys
import threading
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import n6flash  # noqa: E402


def open_console(port, deadline):
    box = {}

    def go():
        try:
            box["s"] = serial.Serial(port, 115200, timeout=0.2, write_timeout=1)
        except Exception as e:
            box["e"] = str(e)[:100]

    th = threading.Thread(target=go, daemon=True)
    th.start(); th.join(deadline)
    if "s" in box:
        return box["s"], None
    return None, box.get("e", "open blocked for %.0fs (image not servicing USB)" % deadline)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--start", action="store_true", help="send 's' after the first reports")
    args = ap.parse_args()

    t0 = time.time()
    pid, port = n6flash.bus()
    if pid != n6flash.PID_APP or not port:
        print("no application on the bus (%s)" % pid); return 3
    s, err = open_console(port, 10.0)
    if s is None:
        print("console %s: %s" % (port, err)); return 4
    s.dtr = True; s.rts = True
    time.sleep(0.2); s.reset_input_buffer()

    def stamp(): return "%6.2fs" % (time.time() - t0)

    def ask(cmd, wait=0.3):
        try:
            s.write(cmd); s.flush()
        except Exception as e:
            print(stamp(), "write failed:", str(e)[:60]); return False
        end = time.time() + wait
        while time.time() < end:
            try:
                line = s.readline()
            except Exception as e:
                print(stamp(), "port gone:", str(e)[:60]); return False
            if line:
                print(stamp(), line.decode(errors="replace").rstrip())
        return True

    for _ in range(3):
        if not ask(b"?\n"): return 5
    if args.start:
        print(stamp(), ">> s (start stream)")
        if not ask(b"s\n"): return 5
    while time.time() - t0 < args.seconds:
        if not ask(b"?\n"): return 5
        pid, _ = n6flash.bus()
        if pid != n6flash.PID_APP:
            print(stamp(), "bus now shows", pid); return 5
    print(stamp(), "still alive at the end of the watch")
    return 0


if __name__ == "__main__":
    sys.exit(main())
