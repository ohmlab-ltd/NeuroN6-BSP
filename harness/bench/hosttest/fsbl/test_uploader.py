#!/usr/bin/env python3
"""The repository's uploader (harness/uploader) against the boot loader's own
source on the simulated board (fsbl_serve), over a pseudo-terminal.

    python3 test_uploader.py <path to fsbl_serve> [test]

run.sh builds fsbl_serve and runs this. Needs Linux (or WSL), Python 3.9 or
later and pyserial.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT

import contextlib
import importlib.util
import io
import os
import random
import shutil
import struct
import subprocess
import sys
import tempfile
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
UPLOADER = os.path.normpath(os.path.join(HERE, "..", "..", "..", "uploader"))
sys.path.insert(0, UPLOADER)

from utils.dfu import (DfuError, DfuSession, erase_size_for, host_crc32,  # noqa: E402
                       looks_like_fsbl)
from utils.pack import make_packet  # noqa: E402

_spec = importlib.util.spec_from_file_location(
    "n6cli", os.path.join(UPLOADER, "neuro-n6-uploader-cli.py"))
cli = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cli)

SLOT_A = 0x00100000
SLOT_B = 0x01C80000
WEIGHTS = 0x00300000
CB = 0x00280000
APP_PC = 0x34010469          # a reset vector in the window an application is copied to
FSBL_PC = 0x34184171         # and one in the RAM the boot loader runs from
CONFIRMED = 0x434F4E46

SERVE = None
LAST = None                  # the board of the test that ran last, for its failure report


def image(size, seed, pc):
    """An image as the build makes one: the header's magic, and a
    vector table 0x400 in."""
    b = bytearray(random.Random(seed).randbytes(size))
    b[0:4] = b"STM2"
    struct.pack_into("<II", b, 0x400, 0x34200000, pc)
    return bytes(b)


def control_block(active, images):
    """A board that has been updated before: both slots confirmed."""
    cb = struct.pack("<IIBBBB", 0x4E364F54, 5, active, 0xFF, 0, 0)
    for img in images:
        cb += struct.pack("<IIII", zlib.crc32(img) & 0xFFFFFFFF, len(img), 1, CONFIRMED)
    cb += bytes(16)
    return cb + struct.pack("<I", zlib.crc32(cb) & 0xFFFFFFFF)


class Board:
    """One simulated board, started in its bootloader."""

    def __init__(self, loads=(), dumps=(), idle=3000, reboot=False):
        self.dir = tempfile.mkdtemp(prefix="fsbl_uploader_")
        args = [SERVE, "--idle", str(idle), "--boots", "6", "--said"] + (["--reboot"] if reboot else [])
        for i, (off, data) in enumerate(loads):
            path = os.path.join(self.dir, "load%d.bin" % i)
            with open(path, "wb") as f:
                f.write(data)
            args += ["--load", "%s@%x" % (path, off)]
        self.dumps = {}
        for off, length in dumps:
            self.dumps[off] = os.path.join(self.dir, "dump%x.bin" % off)
            args += ["--dump", "%s@%x:%x" % (self.dumps[off], off, length)]
        self.proc = subprocess.Popen(args, stdout=subprocess.PIPE, text=True)
        first = self.proc.stdout.readline().split()
        assert first and first[0] == "PTY", "fsbl_serve did not start: %r" % (first,)
        self.port = first[1]
        self.ends = []

    def file(self, name, data):
        path = os.path.join(self.dir, name)
        with open(path, "wb") as f:
            f.write(data)
        return path

    def session(self):
        return DfuSession(self.port, log=lambda *a: None).open()

    def finish(self):
        """Let the board run on by itself; returns how each boot ended."""
        out, _ = self.proc.communicate(timeout=120)
        self.ends = [dict([("end", l.split()[1])] + [kv.split("=") for kv in l.split()[2:]])
                     for l in out.splitlines() if l.startswith("END")]
        return [e["end"] for e in self.ends]

    def flash(self, off):
        with open(self.dumps[off], "rb") as f:
            return f.read()

    def close(self):
        """Stop the board, keeping what it said for a test that failed."""
        if self.proc.poll() is None:
            self.proc.kill()
        try:
            self.said = self.proc.communicate(timeout=10)[0] or ""
        except (OSError, ValueError, subprocess.TimeoutExpired):
            self.said = ""
        shutil.rmtree(self.dir, ignore_errors=True)
        global LAST
        LAST = self


@contextlib.contextmanager
def quiet():
    """The uploader prints its progress; keep it for a failure, out of the way."""
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            yield buf
    except BaseException:
        sys.stderr.write(buf.getvalue())
        raise


def cut_off(s, data, off, frames, half=False):
    """Start an upload of `data` at `off` and stop after `frames` data
    frames, as a tool does that is stopped part-way; with half, in the
    middle of the frame after them."""
    s._send(b"DFU START" + struct.pack("<I", off))
    s._wait_ack("DFU START", quiet_msgs=True)
    s._send(b"DFU_ERASE" + struct.pack("<I", erase_size_for(len(data))))
    s._wait_ack("DFU_ERASE", quiet_msgs=True)
    s._send(b"DFU_DATA")
    s._wait_ack("DFU_DATA", quiet_msgs=True)
    for i in range(0, frames * 4096, 4096):
        s._send(data[i:i + 4096])
        assert s._read_line() == "N6ACK"
        if (i + 4096) % 16384 == 0:                  # and the one more it sends every 16 KB
            assert s._read_line() == "N6ACK"
    if half:
        frame = make_packet(data[frames * 4096:(frames + 1) * 4096])
        s._write(frame[:len(frame) // 2])


def refused(fn, *words):
    """fn must fail with a DfuError that says each of `words`."""
    try:
        fn()
    except DfuError as e:
        for w in words:
            assert w in str(e), "refused, but without %r: %s" % (w, e)
        return str(e)
    raise AssertionError("was not refused")


# ----------------------------------------------------------------- the tests

def t_session_app():
    """An application through a v2 session: identified, written, checked by
    CRC, and booted."""
    app = image(300000, 1, APP_PC)
    b = Board(dumps=[(SLOT_A, len(app))])
    try:
        with quiet():
            s = b.session()
            info = s.probe_info()
            assert info and info["version"] == 2, info
            assert {"CRC32", "MULTI", "OTA-AB", "LOAD1536K"} <= info["caps"], info
            assert info["serial"] and len(info["serial"]) == 24, info
            s.program(app, SLOT_A, "app", next_image=True, progress=False)
            assert s.crc32(SLOT_A, len(app)) == host_crc32(app)
            s.reset_device()
            s.close()
        assert b.finish() == ["RESET", "APP"], b.ends
        assert int(b.ends[-1]["offset"], 16) == SLOT_A and b.flash(SLOT_A) == app
    finally:
        b.close()


def t_cli_app():
    """The same through the command-line tool's v1 flow, as n6flash.py runs it."""
    app = image(250001, 2, APP_PC)
    b = Board(dumps=[(SLOT_A, len(app))])
    try:
        with quiet():
            assert cli.run_dfu(b.port, b.file("app.bin", app), SLOT_A) is True
        assert b.finish() == ["RESET", "APP"], b.ends
        assert b.flash(SLOT_A) == app
    finally:
        b.close()


def t_weights_then_app():
    """Weights and an application in one session, each checked."""
    weights = random.Random(3).randbytes(700001)
    app = image(150000, 4, APP_PC)
    b = Board(dumps=[(WEIGHTS, len(weights)), (SLOT_A, len(app))])
    try:
        with quiet():
            s = b.session()
            s.probe_info()
            s.program(weights, WEIGHTS, "weights", next_image=True, progress=False)
            assert s.crc32(WEIGHTS, len(weights)) == host_crc32(weights)
            s.program(app, SLOT_A, "app", next_image=True, progress=False)
            assert s.crc32(SLOT_A, len(app)) == host_crc32(app)
            s.reset_device()
            s.close()
        assert b.finish() == ["RESET", "APP"], b.ends
        assert b.flash(WEIGHTS) == weights and b.flash(SLOT_A) == app
    finally:
        b.close()


def t_slot_b_on_updated_board():
    """On a board that has been updated over the air, an upload to slot B
    becomes the slot that runs."""
    a, old_b, new_b = image(200000, 5, APP_PC), image(200000, 6, APP_PC), image(260000, 7, APP_PC)
    b = Board(loads=[(SLOT_A, a), (SLOT_B, old_b), (CB, control_block(0, [a, old_b]))],
              dumps=[(SLOT_B, len(new_b)), (SLOT_A, len(a))])
    try:
        with quiet():
            assert cli.run_dfu(b.port, b.file("b.bin", new_b), SLOT_B) is True
        assert b.finish() == ["RESET", "APP"], b.ends
        assert int(b.ends[-1]["offset"], 16) == SLOT_B
        assert b.flash(SLOT_B) == new_b and b.flash(SLOT_A) == a
    finally:
        b.close()


def t_fsbl_needs_asking():
    """The boot loader's own flash: not written unless that is asked for, by
    the tools or, for a host that does not know to ask, by the bootloader."""
    old, new = image(84000, 8, FSBL_PC), image(83744, 9, FSBL_PC)
    b = Board(loads=[(0, old)], dumps=[(0, len(old))])
    try:
        with quiet() as out:
            s = b.session()
            info = s.probe_info()
            assert "FSBLLOCK" in info["caps"], info
            refused(lambda: s.program(new, 0, "fsbl", next_image=True, progress=False),
                    "boot loader's own flash")
            # A host unaware of the lock sends DFU START at 0.
            s._send(b"DFU START" + struct.pack("<I", 0))
            refused(lambda: s._wait_ack("DFU START", bad_replies=("FSBL_LOCKED", "START_BAD_ADDR"),
                                        quiet_msgs=True), "FSBL_LOCKED")
            s.close()
            assert cli.run_dfu(b.port, b.file("fsbl.bin", new), 0) is False
            assert "--fsbl" in out.getvalue()
        assert b.finish() == ["DFU_IDLE"], b.ends
        assert b.flash(0) == old
    finally:
        b.close()


def t_fsbl_update():
    """Asked for, a boot loader image is written and checks out."""
    old, new = image(84000, 8, FSBL_PC), image(83744, 9, FSBL_PC)
    assert looks_like_fsbl(new) and not looks_like_fsbl(image(83744, 9, APP_PC))
    b = Board(loads=[(0, old)], dumps=[(0, len(new))])
    try:
        with quiet():
            s = b.session()
            s.probe_info()
            s.program(new, 0, "fsbl", next_image=True, progress=False, fsbl=True)
            assert s.crc32(0, len(new)) == host_crc32(new)
            s.close()
        assert b.finish() == ["DFU_IDLE"], b.ends
        assert b.flash(0) == new
    finally:
        b.close()


def t_fsbl_update_cli():
    """The same with the command-line tool and --fsbl. The board is blank
    above the loader, so it comes back in its bootloader."""
    old, new = image(84000, 8, FSBL_PC), image(85000, 10, FSBL_PC)
    b = Board(loads=[(0, old)], dumps=[(0, len(new))])
    try:
        with quiet() as out:
            assert cli.run_dfu(b.port, b.file("fsbl.bin", new), 0, fsbl=True) is True
            assert "FSBL_UNLOCKED" in out.getvalue()
        assert b.finish() == ["RESET", "RESET", "DFU_IDLE"], b.ends
        assert b.flash(0) == new
    finally:
        b.close()


def t_fsbl_wrong_image():
    """An application sent as a boot loader erases nothing: the tools refuse
    it, and so does the bootloader for a tool that does not."""
    old, app = image(84000, 8, FSBL_PC), image(84000, 11, APP_PC)
    b = Board(loads=[(0, old)], dumps=[(0, len(old))])
    try:
        with quiet() as out:
            assert cli.run_dfu(b.port, b.file("app.bin", app), 0, fsbl=True) is False
            assert "not a boot loader image" in out.getvalue()
            s = b.session()
            refused(lambda: s.program(app, 0, "app", next_image=True, progress=False, fsbl=True),
                    "not a boot loader image")
            # A tool that does not check: refused at the first data.
            assert s.unlock_fsbl()
            s._send(b"DFU START" + struct.pack("<I", 0))
            s._wait_ack("DFU START", quiet_msgs=True)
            s._send(b"DFU_ERASE" + struct.pack("<I", erase_size_for(len(app))))
            s._wait_ack("DFU_ERASE", quiet_msgs=True)
            s._send(b"DFU_DATA")
            s._wait_ack("DFU_DATA", quiet_msgs=True)
            s._send(app[:4096])
            refused(lambda: s._wait_ack("data", bad_replies=("DATA_NOT_FSBL",), quiet_msgs=True),
                    "DATA_NOT_FSBL")
            s.close()
        assert b.finish() == ["DFU_IDLE"], b.ends
        assert b.flash(0) == old
    finally:
        b.close()


def t_fsbl_image_for_app_slot():
    """A boot loader's image, which every example builds next to its
    application, is not written to an application slot."""
    fsbl = image(84000, 19, FSBL_PC)
    b = Board(dumps=[(SLOT_A + 0x400, 8)])
    try:
        with quiet() as out:
            assert cli.run_dfu(b.port, b.file("Blink_FSBL_Trusted.bin", fsbl), SLOT_A) is False
            assert "boot loader's image" in out.getvalue()
            s = b.session()
            refused(lambda: s.program(fsbl, SLOT_A, "fsbl", next_image=True, progress=False),
                    "boot loader's image")
            s.close()
        assert b.finish() == ["DFU_IDLE"], b.ends
        assert b.flash(SLOT_A + 0x400) == b"\xff" * 8
    finally:
        b.close()


class FakePort:
    def __init__(self, device, vid, pid):
        self.device, self.vid, self.pid = device, vid, pid


def choose(ports, argv, comes_back=None):
    """Run the command-line tool's main() over a made-up bus; returns
    (exit code, ports rebooted, port uploaded to). A reboot of an
    application's port takes it off the bus and puts `comes_back`, a
    bootloader, on it."""
    bus = list(ports)
    rebooted, uploaded = [], []

    def reboot(port):
        rebooted.append(port)
        bus[:] = [p for p in bus if p.device != port]
        if comes_back:
            bus.append(FakePort(comes_back, 0x1209, 0x8009))

    def upload(port, *rest):
        uploaded.append(port)
        return True

    saved = (cli.serial.tools.list_ports.comports, cli._reboot_port, cli.upload, sys.argv)
    cli.serial.tools.list_ports.comports = lambda: list(bus)
    cli._reboot_port, cli.upload = reboot, upload
    sys.argv = ["neuro-n6-uploader-cli.py"] + argv
    try:
        with quiet():
            cli.main()
        code = 0
    except SystemExit as e:
        code = e.code
    finally:
        (cli.serial.tools.list_ports.comports, cli._reboot_port, cli.upload, sys.argv) = saved
    return code, rebooted, uploaded


def t_cli_chooses_the_board():
    """With two boards connected, one running its application and the other
    in its bootloader, the tool uploads to the board it is given, and to
    neither when it is not told which."""
    tmp = tempfile.mkdtemp(prefix="fsbl_uploader_")
    try:
        app = os.path.join(tmp, "app.bin")
        with open(app, "wb") as f:
            f.write(image(70000, 20, APP_PC))
        a, b = FakePort("COM7", 0x1209, 0x8008), FakePort("COM9", 0x1209, 0x8009)
        # Board A named by its application's port: A is rebooted, and A is written.
        assert choose([a, b], ["--port", "COM7", "--bin", app], "COM8") == (0, ["COM7"], ["COM8"])
        # Board B named by its bootloader's port: written at once.
        assert choose([a, b], ["--port", "COM9", "--bin", app]) == (0, [], ["COM9"])
        # Neither named: nothing rebooted, nothing written.
        assert choose([a, b], ["--bin", app]) == (1, [], [])
        # One board, in its bootloader: written at once.
        assert choose([b], ["--bin", app]) == (0, [], ["COM9"])
        # A mistake is found before the board is touched.
        fsbl = os.path.join(tmp, "Blink_FSBL_Trusted.bin")
        with open(fsbl, "wb") as f:
            f.write(image(84000, 21, FSBL_PC))
        assert choose([a], ["--bin", fsbl]) == (1, [], [])
        assert choose([a], ["--bin", os.path.join(tmp, "missing.bin")]) == (1, [], [])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def t_refusal_then_upload():
    """A refusal ends at the next image: the same session uploads afterwards."""
    app = image(120000, 12, APP_PC)
    b = Board(dumps=[(SLOT_A, len(app))])
    try:
        with quiet():
            s = b.session()
            s._send(b"DFU START" + struct.pack("<I", SLOT_A))
            s._wait_ack("DFU START", quiet_msgs=True)
            s._send(b"DFU_DATA")                         # no erase
            refused(lambda: s._wait_ack("DFU_DATA", bad_replies=("DATA_NO_ERASE",),
                                        quiet_msgs=True), "DATA_NO_ERASE")
            s.ser.reset_input_buffer()
            s.probe_info()
            s.program(app, SLOT_A, "app", next_image=True, progress=False)
            assert s.crc32(SLOT_A, len(app)) == host_crc32(app)
            s.reset_device()
            s.close()
        assert b.finish() == ["RESET", "APP"], b.ends
        assert b.flash(SLOT_A) == app
    finally:
        b.close()


def t_upload_cut_short():
    """A tool that stops in the middle of an upload leaves the board in its
    bootloader, not in half an application."""
    app = image(300000, 13, APP_PC)
    b = Board(dumps=[(SLOT_A + 0x400, 8)], reboot=True)
    try:
        with quiet():
            s = b.session()
            s._send(b"DFU START" + struct.pack("<I", SLOT_A))
            s._wait_ack("DFU START", quiet_msgs=True)
            s._send(b"DFU_ERASE" + struct.pack("<I", 0x4A000))
            s._wait_ack("DFU_ERASE", quiet_msgs=True)
            s._send(b"DFU_DATA")
            s._wait_ack("DFU_DATA", quiet_msgs=True)
            for i in range(0, 40 * 4096, 4096):
                s._send(app[i:i + 4096])
                assert s._read_line() == "N6ACK"
                if (i + 4096) % 16384 == 0:              # and the one more it sends every 16 KB
                    assert s._read_line() == "N6ACK"
            s.close()                                    # and the tool goes away
        # Left alone, then reset: nothing to jump into, so the bootloader again.
        assert b.finish() == ["DFU_IDLE", "RESET", "DFU_IDLE"], b.ends
        assert b.flash(SLOT_A + 0x400) == b"\xff" * 8
    finally:
        b.close()


def t_session_after_cut():
    """A session after one that broke off mid-image: the bootloader, still
    taking image data, is brought back to idle first, so the upload goes in."""
    old, app = image(300000, 14, APP_PC), image(200000, 15, APP_PC)
    b = Board(dumps=[(SLOT_A, len(app))])
    try:
        with quiet():
            s = b.session()
            cut_off(s, old, SLOT_A, 10)
            s.close()
            s = b.session()
            info = s.probe_info()
            assert info and info["version"] == 2, info
            s.program(app, SLOT_A, "app", next_image=True, progress=False)
            assert s.crc32(SLOT_A, len(app)) == host_crc32(app)
            s.reset_device()
            s.close()
        assert b.finish() == ["RESET", "APP"], b.ends
        assert b.flash(SLOT_A) == app
    finally:
        b.close()


def t_cli_after_half_frame():
    """The command-line tool after a session that stopped mid-frame: it
    completes the held half frame with bytes the bootloader discards, so its
    first command is not taken for the rest."""
    old, app = image(300000, 16, APP_PC), image(250000, 17, APP_PC)
    b = Board(dumps=[(SLOT_A, len(app))])
    try:
        with quiet():
            s = b.session()
            cut_off(s, old, SLOT_A, 5, half=True)
            s.close()
            assert cli.run_dfu(b.port, b.file("app.bin", app), SLOT_A) is True
        assert b.finish() == ["RESET", "APP"], b.ends
        assert b.flash(SLOT_A) == app
    finally:
        b.close()


def t_fsbl_retry_after_cut():
    """A boot loader update that broke off part-way, retried with the
    command-line tool before a reset: the retry's unlock must be read as a
    command, not written into the half-written loader."""
    old, new = image(84000, 8, FSBL_PC), image(85000, 18, FSBL_PC)
    b = Board(loads=[(0, old)], dumps=[(0, len(new))])
    try:
        with quiet() as out:
            s = b.session()
            assert s.unlock_fsbl()
            cut_off(s, new, 0, 6)
            s.close()
            assert cli.run_dfu(b.port, b.file("fsbl.bin", new), 0, fsbl=True) is True, \
                out.getvalue()
        assert b.finish() == ["RESET", "RESET", "DFU_IDLE"], b.ends
        assert b.flash(0) == new
    finally:
        b.close()


TESTS = [
    ("session", "an application through a v2 session is written, checked and booted", t_session_app),
    ("cli", "the command-line tool writes an application, and it boots", t_cli_app),
    ("weights", "weights and an application in one session", t_weights_then_app),
    ("slotb", "an upload to slot B of an updated board becomes the running slot", t_slot_b_on_updated_board),
    ("locked", "the boot loader's flash is not written unless that is asked for", t_fsbl_needs_asking),
    ("fsbl", "asked for, a boot loader image is written and checks out", t_fsbl_update),
    ("fsblcli", "the command-line tool with --fsbl writes a boot loader image", t_fsbl_update_cli),
    ("wrong", "an application sent as a boot loader erases nothing", t_fsbl_wrong_image),
    ("again", "after a refusal the same session uploads the next image", t_refusal_then_upload),
    ("cut", "an upload stopped half way leaves the board in its bootloader", t_upload_cut_short),
    ("aftercut", "a session after one that broke off part-way uploads", t_session_after_cut),
    ("halfframe", "the command-line tool after a frame cut in half uploads", t_cli_after_half_frame),
    ("fsblretry", "a boot loader update that broke off is completed on retry", t_fsbl_retry_after_cut),
    ("fsblslot", "a boot loader's image is not written to an application slot", t_fsbl_image_for_app_slot),
    ("choose", "with two boards connected the tool writes the one it is given", t_cli_chooses_the_board),
]

# v2.6.2's open findings (docs/BOOT-CONTRACT.md, "Known issues in v2.6.2"):
# the loader's lock on its own flash (F5) and an upload cut short (F4). Each
# fails until fixed; a pass fails the run, so the mark is removed.
OPEN = {"locked", "fsblcli", "wrong", "again", "cut", "fsblretry"}


def main():
    global SERVE
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    SERVE = sys.argv[1]
    only = sys.argv[2] if len(sys.argv) > 2 else None
    passed = failed = opened = 0
    for name, what, fn in TESTS:
        if only and only != name:
            continue
        try:
            fn()
        except Exception as e:                     # a failed check, or the tool failing
            if name in OPEN:
                opened += 1
                print("OPEN  %-9s %s" % (name, what))
                continue
            failed += 1
            print("FAIL  %-9s %s\n      %s: %s" % (name, what, type(e).__name__, e))
            if LAST is not None and (LAST.ends or LAST.said):
                tail = [l for l in LAST.said.splitlines() if not l.startswith("SAID ERASE_PROGRESS")][-12:]
                print("      the board: %s %s" % (LAST.ends, " | ".join(tail)))
        else:
            if name in OPEN:
                failed += 1
                print("FAIL  %-9s %s: passes, but is listed as an open finding" % (name, what))
            else:
                passed += 1
                print("PASS  %-9s %s" % (name, what))
    print("%d passed, %d failed, %d open findings reproduced (the uploader against the loader)"
          % (passed, failed, opened))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
