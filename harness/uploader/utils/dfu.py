# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
# Licensed under the terms in harness/uploader/LICENSE.txt.

# utils/dfu.py - Neuro N6 bootloader (FSBL) session + entry helpers.
#
# Wire protocol (both directions framed as STX | LEN16-LE | DATA | XOR | ETX,
# replies are CRLF text lines):
#
#   v1 (legacy FSBL, 0xCAFE:0x4013):
#     "DFU START" + u32 offset   -> "ADDR_SET 0x..." , "N6ACK"
#     "DFU_ERASE" + u32 size     -> "ERASE_START", "ERASE_PROGRESS 0x...", ..., "N6ACK"
#     "DFU_DATA"                 -> "N6ACK"; then raw 4096-byte data frames,
#                                   one "N6ACK" per frame (+1 extra per 16 KB)
#     "DFU_END"                  -> "DFU_DONE" + immediate device reset
#     anything unknown           -> "N6ACK" (!), so capability probing works
#                                   by looking for the extra reply line, never
#                                   by ACK/NAK.
#
#   v2 additions (FSBL >= 2.0, 0x1209:0x8009):
#     "DFU_INFO"                     -> "N6INFO <ver> <CAP>... [UID <serial>]", "N6ACK"
#     "DFU_CRC32" + u32 off + u32 n  -> "CRC32 0x%08X", "N6ACK"      (idle only)
#     "DFU_NEXT"                     -> flush partial page, leave data mode,
#                                       "NEXT_OK", "N6ACK"           (multi-image)
#     "DFU_RESET"                    -> "RESETTING", device reset    (idle only)
#
#   A boot loader that reports FSBLLOCK in its N6INFO line also has:
#     "DFU_FSBL_UNLOCK"              -> "FSBL_UNLOCKED", "N6ACK"     (idle only)
#         The next "DFU START", and no other, may then be at offset 0, the
#         boot loader's own flash; nowhere else below the application slot is
#         ever accepted. An older boot loader has no such lock and answers
#         with "N6ACK" alone, as it does anything it does not know.
#     Refusals, each a line with no "N6ACK" after it:
#       "FSBL_LOCKED", "START_BAD_ADDR"   DFU START below the application slot
#                                          without the unlock, or not at 0
#       "ERASE_FLASH_MODE"                 the flash did not answer in octal mode
#       "DATA_NO_ERASE"                    DFU_DATA with no DFU_ERASE before it
#       "DATA_PAST_ERASE"                  a data frame that would end past the erase
#       "DATA_NOT_FSBL"                    the first data frame of an image for the
#                                          boot loader's flash is not a boot loader's
#       "DFU_REFUSED"                      anything but DFU_INFO, DFU_CRC32, DFU_RESET
#                                          and DFU_FSBL_UNLOCK, from a refusal until
#                                          the next "DFU START"
#     An upload to the start of an application slot has the page holding its
#     vectors programmed when DFU_NEXT or DFU_END arrives, so one that is cut
#     short leaves the board in the bootloader.
#
#   Between DFU_DATA and DFU_NEXT or DFU_END every frame is image data except
#   those two and DFU START, which ends the image unfinished; a frame is held
#   until all the bytes its length promises have arrived. A session that
#   broke off can leave the bootloader in either state, so every session
#   begins with resync().
#
# Addresses on the wire are OFFSETS from the external-flash base (0x90000000).

import struct
import sys
import time
import zlib

import serial
import serial.tools.list_ports

from utils.pack import make_packet
from utils.device import (APP_IDENTITIES, find_app_port, find_bootloader_port,
                          is_bootloader_family)

FLASH_BASE = 0x90000000
FLASH_END = 0x91FFFFFF
XSPI_SECTOR_SIZE = 0x1000
CHUNK_SIZE = 4096
BAUDRATE = 115200

# Below this offset is the boot loader itself; from it on, the application.
# The second slot an updated board may boot from starts at SLOT_B_OFFSET.
APP_SLOT_OFFSET = 0x00100000
SLOT_B_OFFSET = 0x01C80000
# Where the boot loader runs: the RAM above the 1.5 MB window an application
# is copied to. An image's vectors, 0x400 in, say which of the two it is.
IMAGE_VECTORS = 0x400
FSBL_RAM = (0x34180000 + IMAGE_VECTORS, 0x34200000)

# resync()'s DFU START names the start of the weights pool, which a DFU START
# on its own neither erases nor writes; its zeros are enough to complete the
# longest frame the bootloader can be holding.
RESYNC_OFFSET = 0x00300000
RESYNC_FILL = 8192


class DfuError(Exception):
    """Protocol/transport failure talking to the bootloader."""


def looks_like_fsbl(data):
    """True if `data` begins as a boot loader image does: the image header's
    magic, and a reset vector, in Thumb state, in the RAM the boot loader runs
    from. It is the check the boot loader itself makes on the first block of
    an image for its own flash."""
    if len(data) < IMAGE_VECTORS + 8 or data[:4] != b"STM2":
        return False
    sp, pc = struct.unpack_from("<II", data, IMAGE_VECTORS)
    stack_in_ram = (sp & 3) == 0 and 0x20000000 < sp <= 0x40000000
    return stack_in_ram and (pc & 1) == 1 and FSBL_RAM[0] <= pc < FSBL_RAM[1]


def human(n):
    if n >= 1024 * 1024:
        return "%.2f MB" % (n / (1024.0 * 1024.0))
    if n >= 1024:
        return "%.1f KB" % (n / 1024.0)
    return "%d B" % n


def raw(text):
    """Partial-line write (no newline) for progress that GROWS on one line.
    Append-only friendly, so it renders identically in real terminals and in
    a log panel that does not rewrite lines on \\r."""
    sys.stdout.write(text)
    sys.stdout.flush()


def kv(key, value):
    return "    %-13s %s" % (key, value)


def erase_size_for(n):
    return (n + XSPI_SECTOR_SIZE - 1) // XSPI_SECTOR_SIZE * XSPI_SECTOR_SIZE


def addr_to_offset(addr):
    if addr < FLASH_BASE or addr > FLASH_END:
        raise DfuError("Address 0x%08X outside external flash (0x%08X-0x%08X)"
                       % (addr, FLASH_BASE, FLASH_END))
    off = addr - FLASH_BASE
    if off % XSPI_SECTOR_SIZE:
        raise DfuError("Address 0x%08X is not sector (4 KB) aligned" % addr)
    return off


def resync(ser, timeout=5):
    """Bring the bootloader back to idle, whatever a session that broke off
    left it doing, and return whether it answered. A frame cut short is
    completed by zeros, fails its checksum and is thrown away, as are the
    zeros after it; an image cut short is ended by DFU START, where any
    other command would be written into it. Nothing is erased or written.
    Raises serial.SerialException if the bootloader takes no bytes at all."""
    ser.write(b"\x00" * RESYNC_FILL)
    ser.flush()
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.write(make_packet(b"DFU START" + struct.pack("<I", RESYNC_OFFSET)))
    old = ser.timeout
    ser.timeout = timeout
    try:
        while True:
            reply = ser.readline().decode(errors="ignore").strip()
            if reply == "N6ACK":
                return True
            if reply == "":
                return False
    finally:
        ser.timeout = old


class DfuSession(object):
    """One serial connection to a bootloader; supports several images per
    connection when the FSBL advertises MULTI."""

    def __init__(self, port, log=print):
        self.port = port
        self.log = log
        self.ser = None
        self.info = None          # dict once probe_info() ran: version/caps/serial

    # Transport.

    def open(self):
        # A write the bootloader has not taken in 30 s means it has stopped
        # reading; the write fails rather than waiting for ever.
        try:
            self.ser = serial.Serial(self.port, BAUDRATE, timeout=30,
                                     write_timeout=30)
        except serial.SerialException as e:
            raise DfuError("Cannot open bootloader port %s: %s" % (self.port, e))
        try:
            answered = resync(self.ser)
        except serial.SerialException as e:
            answered = False
            self.log(kv("port", str(e)))
        if not answered:
            self.close()
            raise DfuError("The bootloader on %s does not answer: reset the "
                           "board and try again" % self.port)
        return self

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def _write(self, frame):
        try:
            self.ser.write(frame)
        except serial.SerialException as e:
            raise DfuError("Write to the bootloader failed: %s" % e)

    def _send(self, payload):
        self._write(make_packet(payload))

    def _read_line(self, timeout=None):
        try:
            if timeout is not None:
                old = self.ser.timeout
                self.ser.timeout = timeout
                try:
                    return self.ser.readline().decode(errors="ignore").strip()
                finally:
                    self.ser.timeout = old
            return self.ser.readline().decode(errors="ignore").strip()
        except serial.SerialException as e:
            raise DfuError("Read from the bootloader failed: %s" % e)

    def _wait_ack(self, what, bad_replies=(), timeout=None, quiet_msgs=False):
        """Read lines until N6ACK. Returns list of informational lines seen.
        Raises DfuError on a listed bad reply or on timeout."""
        seen = []
        while True:
            resp = self._read_line(timeout=timeout)
            if resp == "N6ACK":
                return seen
            if resp == "":
                raise DfuError("No response from bootloader during %s" % what)
            if resp in bad_replies:
                raise DfuError("Bootloader rejected %s: %s" % (what, resp))
            seen.append(resp)
            if not quiet_msgs:
                self.log(kv("device", resp))

    # v2 capability probe.

    def probe_info(self):
        """Ask the FSBL to identify itself. Returns the info dict (also kept
        as self.info) or None on a legacy v1 FSBL (which just ACKs)."""
        self.ser.reset_input_buffer()
        self._send(b"DFU_INFO")
        try:
            seen = self._wait_ack("DFU_INFO", timeout=3, quiet_msgs=True)
        except DfuError:
            return None  # even the ACK went missing: treat as legacy
        for line in seen:
            if line.startswith("N6INFO"):
                parts = line.split()
                info = {"version": 1, "caps": set(), "serial": None}
                if len(parts) >= 2:
                    try:
                        info["version"] = int(parts[1])
                    except ValueError:
                        pass
                toks = parts[2:]
                i = 0
                while i < len(toks):
                    if toks[i] == "UID" and i + 1 < len(toks):
                        info["serial"] = toks[i + 1]
                        i += 2
                    else:
                        info["caps"].add(toks[i])
                        i += 1
                self.info = info
                return info
        return None

    def has_cap(self, cap):
        return bool(self.info) and cap in self.info["caps"]

    def crc32(self, offset, length):
        """Device-side CRC32 of `length` flash bytes at `offset`. Returns the
        CRC or None when unsupported/failed (callers fall back)."""
        if not self.has_cap("CRC32"):
            return None
        self._send(b"DFU_CRC32" + struct.pack("<II", offset, length))
        try:
            # 10 MB at even a leisurely 2 MB/s is 5 s; give it slack.
            seen = self._wait_ack("DFU_CRC32", timeout=30, quiet_msgs=True)
        except DfuError:
            return None
        for line in seen:
            if line.startswith("CRC32"):
                try:
                    return int(line.split()[1], 16) & 0xFFFFFFFF
                except (IndexError, ValueError):
                    return None
        return None

    # Image programming.

    def unlock_fsbl(self):
        """Tell the bootloader the next image is for its own flash. Returns
        True if it has that lock and has opened it, False for an older one,
        which has none and writes there without being asked."""
        self._send(b"DFU_FSBL_UNLOCK")
        seen = self._wait_ack("DFU_FSBL_UNLOCK", timeout=3, quiet_msgs=True)
        return "FSBL_UNLOCKED" in seen

    def program(self, data, offset, label, next_image=False, progress=True,
                fsbl=False):
        """START + ERASE + DATA + (DFU_NEXT | DFU_END). With next_image=True
        (v2 MULTI only) the session stays open for another image / CRC check;
        otherwise the device resets and this session is done.

        An offset below the application slot is the boot loader's own flash.
        It is written only with fsbl=True, which says so to the bootloader
        too, and only with an image that is a boot loader's (looks_like_fsbl),
        which an FSBLLOCK boot loader checks again. A boot loader's image is
        never written to the start of an application slot, where the
        bootloader would try to run it."""
        total = len(data)
        if total == 0:
            raise DfuError("Refusing to program an empty image (%s)" % label)
        if offset < APP_SLOT_OFFSET and not fsbl:
            raise DfuError("0x%08X is the boot loader's own flash; %s is not "
                           "written there unless that is asked for"
                           % (FLASH_BASE + offset, label))
        if fsbl and offset != 0:
            raise DfuError("A boot loader image goes at 0x%08X, not 0x%08X"
                           % (FLASH_BASE, FLASH_BASE + offset))
        if fsbl and not looks_like_fsbl(data):
            raise DfuError("%s is not a boot loader image; nothing was written" % label)
        if not fsbl and offset in (APP_SLOT_OFFSET, SLOT_B_OFFSET) and looks_like_fsbl(data):
            raise DfuError("%s is a boot loader's image, not an application's; "
                           "nothing was written" % label)
        if fsbl:
            self.unlock_fsbl()

        self._send(b"DFU START" + struct.pack("<I", offset))
        self._wait_ack("DFU START", bad_replies=("FSBL_LOCKED", "START_BAD_ADDR"),
                       quiet_msgs=True)

        erase = erase_size_for(total)
        raw(kv("erase", "%s bytes at 0x%08X ... " %
               ("{:,}".format(erase), FLASH_BASE + offset)))
        try:
            self._send(b"DFU_ERASE" + struct.pack("<I", erase))
            self._wait_ack("DFU_ERASE",
                           bad_replies=("ERASE_BAD_SIZE", "ERASE_FLASH_MODE",
                                        "DFU_REFUSED"),
                           quiet_msgs=True)
        except DfuError:
            raw("failed\n")
            raise
        raw("ok\n")

        self._send(b"DFU_DATA")
        self._wait_ack("DFU_DATA", bad_replies=("DATA_NO_ERASE", "DFU_REFUSED"),
                       quiet_msgs=True)

        # The bar grows on ONE line as bytes are written.
        BAR = 34
        written = 0
        since_16k = 0
        drawn = 0
        t0 = time.time()
        if progress:
            raw(kv("write", "["))
        try:
            for i in range(0, total, CHUNK_SIZE):
                chunk = data[i:i + CHUNK_SIZE]
                self._write(make_packet(chunk))
                ack = self._read_line()
                if ack != "N6ACK":
                    raise DfuError("Lost sync while writing %s at byte %d (got %r).\n"
                                   "Re-run the upload." % (label, written, ack))
                written += len(chunk)
                since_16k += len(chunk)
                if since_16k >= 16384:
                    ack = self._read_line()
                    if ack != "N6ACK":
                        raise DfuError("Lost sync (16 KB ack) while writing %s (got %r)"
                                       % (label, ack))
                    since_16k = 0
                if progress:
                    want = BAR * written // total
                    if want > drawn:
                        raw("=" * (want - drawn))
                        drawn = want
        except DfuError:
            if progress:
                raw("\n")   # never leave a broken partial line behind
            raise
        dt = time.time() - t0
        if progress:
            raw("]  %s bytes in %.1f s (%.0f KB/s)\n"
                % ("{:,}".format(total), dt, total / dt / 1024 if dt else 0))

        # An image that fills an application slot is recorded in the control
        # block an updated board boots by; the bootloader says when it could
        # not record it, and the board may then start another image.
        unrecorded = DfuError("%s was written, but the bootloader could not "
                              "record it (OTA_CB_NOT_UPDATED); the board may "
                              "start another image. Upload again." % label)
        if next_image:
            self._send(b"DFU_NEXT")
            seen = self._wait_ack("DFU_NEXT", bad_replies=("DFU_REFUSED",),
                                  quiet_msgs=True)
            if "OTA_CB_NOT_UPDATED" in seen:
                raise unrecorded
            if not any(l.startswith("NEXT_OK") for l in seen):
                raise DfuError("Bootloader did not confirm DFU_NEXT "
                               "(image boundary); aborting to stay safe.")
        else:
            self._send(b"DFU_END")
            replies = []
            try:
                while len(replies) < 4:
                    resp = self._read_line(timeout=2)
                    if not resp:
                        break
                    replies.append(resp)
                    if resp == "DFU_DONE":
                        break
                    self.log(kv("device", resp))
            except Exception:
                pass  # the port goes as the board resets
            if "OTA_CB_NOT_UPDATED" in replies:
                raise unrecorded

    def reset_device(self):
        """v2: reset from idle state (after a DFU_NEXT-terminated image)."""
        self._send(b"DFU_RESET")
        try:
            self._read_line(timeout=3)
        except Exception:
            pass


# Bootloader entry.

def _ports():
    return list(serial.tools.list_ports.comports())


def _touch_1200(port, log):
    """The 1200-bps touch: works even when the CDC debug console is off, the
    firmware watching for baud 1200 and a DTR drop."""
    try:
        s = serial.Serial()
        s.port = port
        s.baudrate = 1200
        s.dtr = True
        s.open()
        time.sleep(0.1)
        s.dtr = False
        s.close()
        log(kv("action", "1200-baud touch on %s" % port))
        return True
    except serial.SerialException:
        return False


def _send_b(port, log):
    try:
        s = serial.Serial(port, BAUDRATE, timeout=1, write_timeout=1)
        s.dtr = True
        s.rts = True
        time.sleep(0.1)
        s.reset_input_buffer()
        s.write(b"b\n")
        s.flush()
        time.sleep(0.3)
        s.close()
        log(kv("action", "asked app on %s to enter bootloader" % port))
        return True
    except serial.SerialException as e:
        if "denied" in str(e).lower() or "busy" in str(e).lower():
            log(kv("BLOCKED", "%s is held open by another program" % port))
            log(kv("", "(Serial Monitor / Neuro Studio / another terminal)"))
            log(kv("", "close it and upload again"))
            return False
        return True  # port vanished mid-reboot: that IS success
    except Exception:
        return False


def _wait_bootloader(timeout_sec, prev_app_port, log):
    """Same three-path wait as the classic CLI: distinct PID first, then the
    legacy shared-PID heuristics."""
    start = time.time()
    app_gone = False
    while (time.time() - start) < timeout_sec:
        port = find_bootloader_port()
        # find_bootloader_port also matches the ambiguous legacy 0xCAFE:0x4013;
        # only trust that match once the app port disappeared (or never existed).
        if port and (port != prev_app_port):
            return port
        devices = {p.device for p in _ports()}
        if prev_app_port and prev_app_port not in devices:
            app_gone = True
        if app_gone or prev_app_port is None:
            for p in _ports():
                if is_bootloader_family(p):
                    return p.device
        time.sleep(0.3)
    # Board may have been sitting in the legacy bootloader all along.
    if prev_app_port:
        for p in _ports():
            if p.device == prev_app_port and is_bootloader_family(p):
                return p.device
    return None


def enter_bootloader(preferred_port, log=print, timeout_sec=12):
    """Get the board into bootloader mode and return the bootloader port.
    Accepts whatever state the board is in: already in the bootloader (put
    there by the IDE's 1200-bps touch, say), running the app, or unplugged
    (returns None with guidance printed)."""
    boot = find_bootloader_port()
    if boot:
        log(kv("bootloader", "already present on %s" % boot))
        return boot

    app = None
    if preferred_port:
        for p in _ports():
            if p.device == preferred_port and (p.vid, p.pid) in APP_IDENTITIES:
                app = preferred_port
                break
    if app is None:
        app = find_app_port()
        if app and preferred_port and app != preferred_port:
            log(kv("port", "selected %s is not the Neuro N6 app; using %s"
                   % (preferred_port, app)))

    if app:
        _send_b(app, log)
        boot = _wait_bootloader(6, app, log)
        if boot:
            log(kv("bootloader", boot))
            return boot
        # The debug console may be off, in which case 'b' was ignored.
        # Try the 1200-baud touch instead.
        if app in {p.device for p in _ports()}:
            _touch_1200(app, log)
        boot = _wait_bootloader(timeout_sec, app, log)
        if boot:
            log(kv("bootloader", boot))
            return boot
    else:
        log(kv("app", "not found; waiting for a bootloader to appear"))
        log(kv("", "(double-tap the reset button if nothing happens)"))
        boot = _wait_bootloader(timeout_sec, preferred_port or None, log)
        if boot:
            log(kv("bootloader", boot))
            return boot
    return None


def bootloader_serial(port):
    """iSerial of the enumerated bootloader port (host-cache key fallback)."""
    for p in _ports():
        if p.device == port:
            return p.serial_number or None
    return None


def wait_for_app(timeout_sec=10, strict=False):
    """Wait for a running application to enumerate. strict=True matches only
    the current 0x1209:0x8008 identity - the legacy 0xCAFE:0x4013 is
    ambiguous (it is ALSO the legacy FSBL, which lingers during the
    reset/re-enumeration transition and would be misreported as 'app')."""
    from utils.device import APP_VID, APP_PID
    start = time.time()
    while (time.time() - start) < timeout_sec:
        if strict:
            for p in _ports():
                if (p.vid, p.pid) == (APP_VID, APP_PID):
                    return p.device
        else:
            port = find_app_port()
            if port:
                return port
        time.sleep(0.3)
    return None


def host_crc32(data):
    return zlib.crc32(data) & 0xFFFFFFFF
