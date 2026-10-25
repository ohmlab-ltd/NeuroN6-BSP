#!/usr/bin/env python3
"""Drive the BSP test console on a Neuro N6 and record what it reports.

    python harness/n6test.py <image>_Trusted.bin [--only cmd,cmd] [--skip cmd,cmd]
                             [--out results.md] [--no-flash] [--keep]

The test image (Examples/BSPTest, bare metal or FreeRTOS) has one console
command per BSP module. Each prints INFO lines, then exactly one of

    OK   <cmd> [key=value ...]
    FAIL <cmd> <reason>
    SKIP <cmd> <reason>

and `END <cmd>`. This flashes the image (through n6flash.py, which writes only
the application slot), runs every command or the subset asked for, writes a
Markdown table and, unless --keep, sends 'b' to return to the bootloader.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: BSD-3-Clause
# Licensed under the terms in the LICENSE file at the root of this
# repository.

import argparse
import datetime
import os
import re
import subprocess
import sys
import time

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import n6flash  # noqa: E402  (bus(), wait_for(), PID_*)

# Console commands with generous timeouts: a missing sensor fails through its
# own I2C timeout, which must not look like a hung board.
COMMANDS = [
    ("bkp",    10),
    ("id",    10), ("clk",   10), ("led",   10), ("btn",   10), ("i2c",   15),
    ("imu",   20), ("mag",   20), ("magcont", 25), ("i2cref", 15), ("adc",   15), ("io",    15), ("hdr",   30),
    # The ranging sensor, on the OV5640 carriers only: both skip without it.
    ("tof",   20), ("tofgrid", 30),
    ("pwm",   15), ("tone",  15), ("hdredge", 20), ("wdg",   15), ("rtos",  30), ("npu",   15), ("mic",   30),
    # Configuration surfaces (sensor tuning, audio, motion ranges, PWM rates,
    # small entry points); each checks an invariant, not only a return code.
    ("imucfg", 30), ("audcfg", 40), ("pwmcfg", 20), ("miscapi", 20),
    # camcarrier tries both lane mappings (a wrong one is silent: no frame,
    # registers still answer) and keeps the one that captures, so it runs
    # before every camera test.
    ("camcarrier", 90),
    ("camcfg", 60), ("camsusp", 40), ("camisp", 60), ("cammode", 60),
    # lcd and lcdbmp check the controller and frame buffer, so they pass with
    # or without a panel.
    ("lcd",   30), ("lcdbmp", 20), ("reg16",  20), ("audset", 60),
    # camcore checks the camera core itself - cache maintenance, pitches,
    # stops and restarts - on the sensor's colour bars, so it needs no light.
    ("camcore", 60),
    # campic reads the sensor's shared picture-control registers back.
    ("campic", 30),
    # camxga is left out: it measures how often XGA fails to start (see
    # BSP_CAMERA_R1024x768), failing about one run in two; use --only.
    # jpegres applies the documented retry. audloop needs a speaker.
    ("spk",   20), ("audloop", 20), ("cam",   40), ("camres", 90), ("vencreg", 15), ("rif", 10),
    ("jpeg",  60), ("jpegres", 180),
    # jpegdump must follow jpegmem directly: jpeggap and jpegrun reuse its
    # buffer, and the dump would read as a truncated, longer picture.
    ("jpegmem", 30), ("jpegdump", 120), ("jpeggap", 40), ("jpegrun", 40), ("jpegtorn", 30),
    ("thermal", 20), ("vospi", 15),
]

# Not in the default run: the MAYA-W476 driver is FreeRTOS only, so the Wi-Fi
# example exercises it. With --only, "wifi" stops at SDIO enumeration and "bt"
# reports SKIP: the controller runs only once the Wi-Fi stack has loaded its
# firmware (see board-tests/STATUS.md).
WIRELESS = [("wifi", 40), ("bt", 30)]


# The eight bars jpegmem draws, left to right, as the RGB a decoder should
# produce from their limited-range YCbCr (white, yellow, cyan, green,
# magenta, red, blue, black). Tolerance is wide because the encoder is at
# quality 2 and the decoder's colour conversion is its own.
BARS = [(235, 235, 235), (235, 235, 16), (16, 235, 235), (16, 235, 16),
        (235, 16, 235), (235, 16, 16), (16, 16, 235), (16, 16, 16)]


def check_jpegdump(lines, path):
    """Rebuild the JPEG from the INFO HEX lines, save it, decode and measure.

    Returns (ok, detail). The picture is 640 x 480 with 80-pixel bars; each
    bar is sampled at its centre column, mid-height, as a 16 x 16 mean.
    """
    hexdata = "".join(l[len("INFO HEX "):].strip() for l in lines if l.startswith("INFO HEX "))
    if not hexdata:
        return False, "no HEX lines"
    try:
        data = bytes.fromhex(hexdata)
    except ValueError as e:
        return False, "hex decode: %s" % e
    with open(path, "wb") as f:
        f.write(data)
    try:
        from PIL import Image
    except ImportError:
        return True, "saved %d bytes (PIL not installed, picture not checked)" % len(data)
    try:
        im = Image.open(path).convert("RGB")
    except Exception as e:
        return False, "saved %d bytes but the host cannot decode it: %s" % (len(data), e)
    if im.size != (640, 480):
        return False, "decoded size %sx%s, expected 640x480" % im.size
    bad = []
    for i, want in enumerate(BARS):
        x0, y0 = 40 + 80 * i - 8, 240 - 8
        px = [im.getpixel((x, y)) for x in range(x0, x0 + 16) for y in range(y0, y0 + 16)]
        got = tuple(sum(c[k] for c in px) // len(px) for k in range(3))
        if any(abs(got[k] - want[k]) > 48 for k in range(3)):
            bad.append("bar%d=%s" % (i, got))
    if bad:
        return False, "decoded 640x480, %d/8 bars off: %s" % (len(bad), " ".join(bad))
    return True, "decoded 640x480, 8/8 colour bars within tolerance, %d bytes" % len(data)


def open_console(port, timeout):
    t0 = time.time(); last = ""
    while time.time() - t0 < timeout:
        try:
            s = serial.Serial(port, 115200, timeout=0.2, write_timeout=2,
                              dsrdtr=False, rtscts=False)
            s.dtr = True; s.rts = True
            time.sleep(0.2)
            s.reset_input_buffer()
            return s
        except serial.SerialException as e:
            last = str(e); time.sleep(0.5)
    raise RuntimeError("%s not openable in %.0fs: %s" % (port, timeout, last))


def run_command(s, cmd, timeout):
    """Send one command and gather its lines up to END or the timeout. A
    watchdog reset under the open handle makes serial calls raise: that is
    reported as a result, not a crash."""
    lines = []; buf = b""; t0 = time.time(); verdict = None
    try:
        s.reset_input_buffer()
        s.write((cmd + "\n").encode()); s.flush()
    except (serial.SerialException, OSError):
        return ("FAIL", "WEDGED: the console was gone before this command"), lines, 0.0
    while time.time() - t0 < timeout:
        try:
            chunk = s.read(256)
        except (serial.SerialException, OSError):
            return ("FAIL", "WEDGED: the board reset and the console vanished"), lines, time.time() - t0
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode(errors="replace").rstrip("\r")
                if not text:
                    continue
                lines.append(text)
                m = re.match(r"^(OK|FAIL|SKIP)\s+(\S+)(?:\s+(.*))?$", text)
                if m and m.group(2) == cmd:
                    verdict = (m.group(1), m.group(3) or "")
                if text.strip() == "END " + cmd:
                    return verdict or ("FAIL", "no verdict before END"), lines, time.time() - t0
    return (verdict or ("FAIL", "timeout after %.0fs (board may be wedged)" % timeout)), lines, time.time() - t0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("image")
    ap.add_argument("--only", default="", help="comma list of commands to run")
    ap.add_argument("--skip", default="", help="comma list of commands to skip")
    ap.add_argument("--out", default=None, help="results Markdown path")
    ap.add_argument("--no-flash", action="store_true", help="board already runs the image")
    ap.add_argument("--keep", action="store_true", help="leave the application running (no 'b')")
    ap.add_argument("--timeout", type=float, default=30.0)
    args = ap.parse_args()

    only = set(filter(None, args.only.split(",")))
    skip = set(filter(None, args.skip.split(",")))
    todo = [(c, t) for c, t in COMMANDS if (not only or c in only) and c not in skip]
    todo += [(c, dict(WIRELESS).get(c, 20)) for c in only if c not in dict(COMMANDS)]  # wireless, bkpclr, wdgfire

    several = n6flash.several_boards()
    if several:
        print(several, file=sys.stderr); return 3
    if not args.no_flash:
        r = subprocess.run([sys.executable, os.path.join(HERE, "n6flash.py"), args.image,
                            "--no-reboot-test", "--timeout", str(args.timeout)])
        if r.returncode != 0:
            print("flash failed (rc=%d)" % r.returncode, file=sys.stderr); return r.returncode

    port, dt = n6flash.wait_for(n6flash.PID_APP, args.timeout)
    if not port:
        print("application not on the bus", file=sys.stderr); return 5
    s = open_console(port, args.timeout)
    print("console: %s" % port)

    results = []
    for cmd, tmo in todo:
        verdict, lines, took = run_command(s, cmd, tmo)
        # A test that wedged the board is reset by the watchdog into the
        # bootloader: record that as the verdict.
        if verdict[0] == "FAIL" and "wedged" in verdict[1]:
            pid, _ = n6flash.bus()
            if pid == n6flash.PID_BOOTLOADER:
                verdict = ("FAIL", "WEDGED: no answer, watchdog reset landed in the bootloader")
            else:
                verdict = ("FAIL", "WEDGED: no answer in %.0fs and no watchdog reset seen (bus=%s)" % (tmo, pid))
        if cmd == "jpegdump" and verdict[0] == "OK":
            jpg = os.path.join(os.path.dirname(args.out) if args.out else os.path.join(HERE, "board-tests"),
                               "jpegmem-%s.jpg" % datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
            good, detail = check_jpegdump(lines, jpg)
            verdict = ("OK" if good else "FAIL", detail + " -> " + os.path.basename(jpg))
            lines = [l for l in lines if not l.startswith("INFO HEX ")] + ["(hex lines omitted; picture saved as %s)" % os.path.basename(jpg)]
        results.append((cmd, verdict, lines, took))
        flag = {"OK": "ok  ", "FAIL": "FAIL", "SKIP": "skip"}[verdict[0]]
        print("  %s %-8s %5.1fs  %s" % (flag, cmd, took, verdict[1][:100]))
        if verdict[1].startswith("WEDGED"):
            # Reflash and carry on, so one hanging test does not cost the rest.
            print("  %s wedged the board; reflashing to continue" % cmd, file=sys.stderr)
            try:
                s.close()
            except Exception:
                pass
            n6flash.wait_for(n6flash.PID_BOOTLOADER, args.timeout)
            r = subprocess.run([sys.executable, os.path.join(HERE, "n6flash.py"), args.image,
                                "--no-reboot-test", "--timeout", str(args.timeout)],
                               capture_output=True, text=True)
            if r.returncode != 0:
                print("  could not bring the board back: rc=%d" % r.returncode, file=sys.stderr)
                break
            port2, _ = n6flash.wait_for(n6flash.PID_APP, args.timeout)
            if not port2:
                print("  the board did not come back", file=sys.stderr)
                break
            try:
                s = open_console(port2, args.timeout)
            except RuntimeError as e:
                print("  console did not reopen: %s" % e, file=sys.stderr)
                break
            print("  console back on %s" % port2)

    out = args.out or os.path.join(HERE, "board-tests",
                                   "results-%s.md" % datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
    if os.path.dirname(out):
        os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        f.write("# Neuro N6 BSP board test, %s\n\n" % datetime.datetime.now().isoformat(timespec="seconds"))
        f.write("Image: `%s`\n\n" % os.path.basename(args.image))
        f.write("| test | result | time | detail |\n|---|---|---|---|\n")
        for cmd, (v, detail), lines, took in results:
            f.write("| `%s` | **%s** | %.1fs | %s |\n" % (cmd, v, took, detail.replace("|", "\\|")))
        f.write("\n## Console output\n")
        for cmd, _, lines, _ in results:
            f.write("\n### %s\n```\n%s\n```\n" % (cmd, "\n".join(lines)))
    print("results: %s" % out)

    if not args.keep:
        try:
            s.write(b"b\n"); s.flush(); time.sleep(0.5); s.close()
        except Exception:
            pass
        port, dt = n6flash.wait_for(n6flash.PID_BOOTLOADER, args.timeout)
        print("bootloader back on %s after %.1fs" % (port, dt) if port else "WARNING: bootloader did not return")
    n_fail = sum(1 for _, (v, _), _, _ in results if v == "FAIL")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
