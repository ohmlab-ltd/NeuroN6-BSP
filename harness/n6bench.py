#!/usr/bin/env python3
"""Run the whole bench unattended: wait for the bootloader, then test everything.

    python harness/n6bench.py --test <BSPTest>_Trusted.bin --stream <JpegStream>_Trusted.bin
                              [--wait 28800] [--only cmd,cmd] [--skip cmd,cmd]

Each step is logged to harness/board-tests/bench-<timestamp>.log:
  1. Wait (up to --wait seconds) for the bootloader, VID 0x1209 / PID 0x8009.
     This repository's images send every watchdog reset and fault there.
  2. Flash the test console and run its commands (n6test.py).
  3. Flash the streaming image, read its stream for a few seconds
     (n6stream.py), save the last frame, then send 'b', or wait for the
     watchdog if its console never opens.
  4. Flash the test image again and read the streaming image's breadcrumbs
     in the backup registers (bkp).

Exit code 0 only if every step ran; each failing step is named in the log.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: BSD-3-Clause
# Licensed under the terms in the LICENSE file at the root of this
# repository.

import argparse
import datetime
import os
import queue
import subprocess
import sys
import threading
import time

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import n6flash  # noqa: E402

LOG = None


def log(msg):
    line = "%s %s" % (datetime.datetime.now().strftime("%H:%M:%S"), msg)
    print(line, flush=True)
    if LOG:
        LOG.write(line + "\n"); LOG.flush()


def run(args, timeout):
    """Run a tool, stream its output into the log, return its exit code. A
    tool still running at the deadline is killed, whether or not it prints."""
    log("$ " + " ".join(args))
    p = subprocess.Popen([sys.executable, "-u"] + args, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True, errors="replace")
    t0 = time.time()
    lines = queue.Queue()

    def reader():
        for line in p.stdout:
            lines.put(line)
        lines.put(None)

    threading.Thread(target=reader, daemon=True).start()
    while True:
        left = timeout - (time.time() - t0)
        try:
            line = lines.get(timeout=max(0.0, min(left, 1.0)))
        except queue.Empty:
            line = ""
        if line is None:
            break
        if line:
            log("  | " + line.rstrip())
        if time.time() - t0 > timeout:
            p.kill(); p.wait(); log("  killed after %ds" % timeout); return 124
    try:
        p.wait(timeout=max(1.0, timeout - (time.time() - t0)))
    except subprocess.TimeoutExpired:
        p.kill(); p.wait(); log("  killed after %ds" % timeout); return 124
    log("  -> rc=%d in %.0fs" % (p.returncode, time.time() - t0))
    return p.returncode


def ask_for_bootloader(port, deadline=15.0):
    """Send 'b' on an application's console, opening it on a thread with a
    deadline: a wedged image makes CreateFile on its CDC port block."""
    def send():
        try:
            with serial.Serial(port, 115200, timeout=0.3, write_timeout=1) as s:
                s.write(b"\r\nb\r\n")
        except Exception as e:                           # noqa: BLE001 - report and wait
            log("  'b' did not go out (%s)" % e)

    th = threading.Thread(target=send, daemon=True)
    th.start()
    th.join(deadline)
    if th.is_alive():
        log("  opening %s hung %.0fs; waiting for the watchdog" % (port, deadline))


def wait_bootloader(seconds):
    t0 = time.time(); last = None; asked = False
    while time.time() - t0 < seconds:
        pid, port = n6flash.bus()
        if pid == n6flash.PID_BOOTLOADER and port:
            log("bootloader on %s after %.0fs" % (port, time.time() - t0)); return port
        # An application from this repository answers 'b': ask once rather
        # than wait out the timeout. One that does not answer is still waited for.
        if (pid == n6flash.PID_APP) and port and not asked:
            log("application on %s: asking it for the bootloader" % port)
            ask_for_bootloader(port)
            asked = True
        state = "%s %s" % (pid, port)
        if state != last:
            log("bus: %s (waiting for the bootloader)" % state); last = state
        time.sleep(3)
    return None


def main():
    global LOG
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--test", required=True, help="BSP test console image (_Trusted.bin)")
    ap.add_argument("--stream", default=None, help="JPEG streaming image (_Trusted.bin)")
    ap.add_argument("--wait", type=float, default=8 * 3600, help="seconds to wait for the bootloader")
    ap.add_argument("--only", default="")
    ap.add_argument("--skip", default="")
    ap.add_argument("--stream-seconds", type=float, default=6.0)
    args = ap.parse_args()

    out_dir = os.path.join(HERE, "board-tests"); os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    LOG = open(os.path.join(out_dir, "bench-%s.log" % stamp), "w", encoding="utf-8")
    log("bench start; test=%s stream=%s" % (os.path.basename(args.test), os.path.basename(args.stream or "-")))
    failures = []

    if not wait_bootloader(args.wait):
        log("no bootloader within the wait; nothing run"); return 3

    # 2. the console suite
    extra = []
    if args.only: extra += ["--only", args.only]
    if args.skip: extra += ["--skip", args.skip]
    rc = run([os.path.join(HERE, "n6test.py"), args.test, "--timeout", "30",
              "--out", os.path.join(out_dir, "results-%s.md" % stamp)] + extra, timeout=900)
    if rc != 0:
        failures.append("n6test rc=%d" % rc)
    if not wait_bootloader(60):
        failures.append("bootloader did not return after the console suite"); log("stopping"); return 4

    if args.stream:
        # 3. the stream
        rc = run([os.path.join(HERE, "n6flash.py"), args.stream, "--no-reboot-test", "--timeout", "30"], timeout=120)
        if rc != 0:
            failures.append("stream flash rc=%d" % rc)
        else:
            time.sleep(2.0)
            rc = run([os.path.join(HERE, "n6stream.py"), "--seconds", str(args.stream_seconds),
                      "--save", os.path.join(out_dir, "frame-%s.jpg" % stamp), "--console"], timeout=60)
            if rc != 0:
                failures.append("n6stream rc=%d" % rc)
            # Back to the bootloader: 'b' if the console answers, else the
            # watchdog. The open has a deadline, as a wedged image blocks it.
            def send_b():
                try:
                    import serial
                    pid, port = n6flash.bus()
                    if pid == n6flash.PID_APP and port:
                        s = serial.Serial(port, 115200, timeout=1, write_timeout=1)
                        s.dtr = True; s.rts = True; time.sleep(0.1); s.write(b"b\n"); s.flush(); time.sleep(0.5)
                        try: s.close()
                        except Exception: pass
                        log("sent 'b' on %s" % port)
                except Exception as e:
                    log("console not usable (%s); waiting for the watchdog" % str(e)[:80])
            th = threading.Thread(target=send_b, daemon=True); th.start(); th.join(15)
            if th.is_alive():
                log("console open hung 15s (image not servicing USB); waiting for the watchdog")
            if not wait_bootloader(90):
                failures.append("bootloader did not return after the stream"); log("stopping"); return 5

        # 4. breadcrumbs
        rc = run([os.path.join(HERE, "n6test.py"), args.test, "--only", "bkp", "--timeout", "30",
                  "--out", os.path.join(out_dir, "breadcrumbs-%s.md" % stamp)], timeout=180)
        if rc != 0:
            failures.append("breadcrumb read rc=%d" % rc)
        wait_bootloader(60)

    log("bench done; failures: %s" % (failures or "none"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
