#!/usr/bin/env python3
"""Neuro N6 DFU uploader - interactive command line.

    python neuro-n6-uploader-cli.py --bin firmware.bin
    python neuro-n6-uploader-cli.py --port COM7 --bin firmware.bin \\
                                    --address 0x90100000
    python neuro-n6-uploader-cli.py --bin bootloader.bin --fsbl

Uses the v1 DFU command set (with the unlock a boot loader image needs) and
does not read flash back; harness/n6weights.py verifies a region against the
bootloader's CRC32. With several boards connected, --port names one (its
application's or its bootloader's port).

An address below 0x90100000 is the boot loader's own flash: nothing is
written there without --fsbl, which puts a boot loader image at 0x90000000.
A wrong image there, or power lost while it is written, leaves a board that
no longer starts and that this tool cannot reach. Requires pyserial; see
README.md.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
# Licensed under the terms in harness/uploader/LICENSE.txt.

import argparse
import os
import sys
import time
import struct
import serial
import serial.tools.list_ports
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from utils.pack import pack, calculate_erase_size, make_packet

# DFU Constants
BAUDRATE = 115200
CHUNK_SIZE = 4096
XSPI_SECTOR_SIZE = 0x1000
FLASH_BASE = 0x90000000
FLASH_END = 0x91FFFFFF
DEFAULT_FLASH_OFFSET = 0x00100000  # Application slot A
BAR_WIDTH = 30


def human_bytes(n):
    if n >= 1024 * 1024:
        return f"{n / (1024 * 1024):.2f} MB"
    if n >= 1024:
        return f"{n / 1024:.1f} KB"
    return f"{n} B"


def bar(done, total, width=BAR_WIDTH):
    total = max(1, total)
    done = max(0, min(done, total))
    filled = int(width * done / total)
    pct = done * 100.0 / total
    return f"[{'#' * filled}{'-' * (width - filled)}] {pct:5.1f}%"


def line():
    print("+------------------------------------------------------------+")


def boxed(text):
    """One row of a banner box, padded to line()'s 62-column width."""
    print(f"| {text[:58]:<58} |")


def banner(title):
    line()
    boxed("Neuro N6 Upload")
    boxed(title)
    line()


def step(idx, total, text):
    print(f"  [{idx}/{total}] {text}")

# USB identities: utils/device.py. A board on the old 0xCAFE:0x4013 identity
# is reached by naming its port with --port.
from utils.device import (APP_IDENTITIES, BOOTLOADER_IDENTITIES,
                          is_bootloader_family, find_app_port)
from utils.dfu import APP_SLOT_OFFSET, looks_like_fsbl, resync
# The 1200-baud touch (open at 1200 baud, drop DTR) reboots even a build with
# no debug console, which has no 'b' handler.
from utils.dfu import _touch_1200


def neuro_ports():
    """Every port a Neuro N6 is on: (port, "app" or "bootloader") pairs.
    The old shared identity counts as an application, which is what an old
    board usually runs."""
    found = []
    for p in serial.tools.list_ports.comports():
        if (p.vid, p.pid) in BOOTLOADER_IDENTITIES[:2]:
            found.append((p.device, "bootloader"))
        elif (p.vid, p.pid) in APP_IDENTITIES:
            found.append((p.device, "app"))
    return found


def _reboot_port(port):
    """Reboot a specific COM port into the bootloader (best-effort): the
    1200-baud touch, then 'b', which works only with the debug console on."""
    print(f"  Port      : {port}")
    print("  Action    : 1200-baud touch")
    _touch_1200(port, lambda msg: None)  # already announced above; result shown by the wait step
    time.sleep(0.2)

    try:
        ser = serial.Serial(port, 115200, timeout=1, write_timeout=1)
        ser.dtr = True
        ser.rts = True
        time.sleep(0.1)
        ser.reset_input_buffer()
        print("  Action    : asking app to enter bootloader ('b')")
        ser.write(b'b\n')
        ser.flush()
        time.sleep(0.5)
        ser.close()
    except serial.SerialException:
        pass  # Port disappears as the device resets - expected
    except Exception as e:
        print(f"Note: Could not send reboot command: {e}")

def reboot_to_bootloader():
    """Reboot the auto-detected app into bootloader mode: the 1200-baud
    touch, then 'b', which works only with the debug console on."""
    port = find_app_port()

    if port is None:
        print("  App       : not found; board may already be in bootloader")
        return True

    print(f"  App       : Neuro N6 on {port}")

    print("  Action    : 1200-baud touch")
    _touch_1200(port, lambda msg: None)  # already announced above; result shown by the wait step
    time.sleep(0.2)

    try:
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=1,
            write_timeout=1,
        )
        ser.dtr = True
        ser.rts = True
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print("  Action    : asking app to enter bootloader ('b')")
        ser.write(b'b\n')
        ser.flush()
        time.sleep(0.5)
        ser.close()

        print("  Wait      : device is re-enumerating")
        time.sleep(2.0)
        return True

    except serial.SerialException as e:
        print(f"  Wait      : port disconnected for reboot ({e})")
        time.sleep(2.0)
        return True
    except Exception as e:
        print(f"ERROR: {e}")
        return False

def parse_address(addr_str):
    """Parse hex address string and return offset from flash base."""
    try:
        addr = int(addr_str, 16)

        if addr < FLASH_BASE or addr > FLASH_END:
            print(f"Error: Address 0x{addr:08X} outside flash range (0x{FLASH_BASE:08X} - 0x{FLASH_END:08X})")
            return None

        offset = addr - FLASH_BASE
        if offset % XSPI_SECTOR_SIZE != 0:
            print(f"Error: Address must be sector-aligned (0x{XSPI_SECTOR_SIZE:X} bytes)")
            return None

        return offset
    except ValueError:
        print(f"Error: Invalid hex address format: {addr_str}")
        return None


def find_bootloader_port(exclude=()):
    """Find the bootloader COM port, distinct PIDs only (wait_for_bootloader_port
    handles the ambiguous legacy 0xCAFE:0x4013), leaving out the ports in
    `exclude`."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if ((port.vid, port.pid) in BOOTLOADER_IDENTITIES[:2]    # 0x1209:0x8009, 0xCAFE:0x4014
                and port.device not in exclude):
            return port.device
    return None


def wait_for_bootloader_port(timeout_sec=10, prev_app_port=None, exclude=()):
    """Wait for the bootloader COM port to appear, by the first of:
      1. a distinct-PID FSBL (0x1209:0x8009 or 0xCAFE:0x4014) on any port;
      2. a legacy FSBL after reboot: the app port goes, then a 0xCAFE device
         appears (an old FSBL enumerates as 0xCAFE:0x4013);
      3. a legacy FSBL already in its bootloader ('b' ignored, the port never
         went): after the timeout the original port is taken as the bootloader.
    A bootloader in `exclude`, another board's, is never the one found.
    """
    print("  Scan      : waiting for bootloader (double-tap reset if needed)")
    start = time.time()
    app_port_gone = False

    while (time.time() - start) < timeout_sec:
        current_devices = {p.device for p in serial.tools.list_ports.comports()}

        # Path 1: distinct bootloader PID
        port = find_bootloader_port(exclude)
        if port:
            print(f"  Found     : bootloader on {port}")
            return port

        # Track whether the app port has disappeared (i.e. the device rebooted)
        if prev_app_port and prev_app_port not in current_devices:
            app_port_gone = True

        # Path 2: legacy FSBL. Once the app port is gone, accept any
        # bootloader-family device (anything on the legacy 0xCAFE VID)
        if app_port_gone:
            for p in serial.tools.list_ports.comports():
                if is_bootloader_family(p):
                    print(f"  Found     : legacy bootloader on {p.device}")
                    return p.device

        time.sleep(0.3)

    # Path 3: board was already in bootloader mode when upload started, so
    # the 'b' command was silently ignored and the port never left. If the
    # original port is still a bootloader-family device, assume bootloader.
    if prev_app_port:
        for p in serial.tools.list_ports.comports():
            if p.device == prev_app_port and is_bootloader_family(p):
                print(f"  Found     : assuming {prev_app_port} is legacy bootloader")
                return p.device

    return None


def check_image(bin_path, flash_offset, fsbl=False):
    """Read the image and check it may go where it is asked to, before the
    board is touched. Returns its bytes, or None after saying why not."""
    try:
        with open(bin_path, "rb") as f:
            data = f.read()
    except OSError as e:
        print(f"ERROR: Cannot read {bin_path}: {e}")
        return None
    if not data:
        print(f"ERROR: {bin_path} is empty")
        return None

    # The boot loader's own flash: only when asked, only at its start, only
    # with a boot loader's image, checked before the board is touched.
    if fsbl:
        if flash_offset != 0:
            print(f"ERROR: a boot loader image goes at 0x{FLASH_BASE:08X}, "
                  f"not 0x{FLASH_BASE + flash_offset:08X}")
            return None
        if not looks_like_fsbl(data):
            print(f"ERROR: {bin_path} is not a boot loader image; nothing was written")
            return None
    elif flash_offset < APP_SLOT_OFFSET:
        print(f"ERROR: 0x{FLASH_BASE + flash_offset:08X} is the boot loader's own flash.")
        print("       Nothing is written there without --fsbl and a boot loader image.")
        return None
    elif looks_like_fsbl(data):
        # Every example builds one next to its application: <name>_FSBL_Trusted.bin.
        print(f"ERROR: {bin_path} is a boot loader's image, not an application's;")
        print("       nothing was written. A boot loader goes to its own flash, with --fsbl.")
        return None
    return data


def run_dfu(port, bin_path, flash_offset, fsbl=False):
    """Run the DFU upload process. With fsbl, the image is a boot loader's
    and goes to the boot loader's own flash, at the start of the device."""

    file_data = check_image(bin_path, flash_offset, fsbl)
    if file_data is None:
        return False

    banner("DFU session")
    print(f"  Port      : {port}")
    print(f"  Firmware  : {bin_path}")
    print(f"  Address   : 0x{FLASH_BASE + flash_offset:08X}")
    print(f"  Protocol  : DFU v1 command set over CDC serial @ {BAUDRATE} baud")
    print()

    # Long timeout, for erases; a write not taken within it means the
    # bootloader stopped reading, a failed upload.
    try:
        ser = serial.Serial(port, BAUDRATE, timeout=30, write_timeout=30)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open serial port {port}: {e}")
        return False

    # A broken-off upload can leave the bootloader mid-frame or mid-image:
    # bring it back to idle first.
    if not resync(ser):
        print("ERROR: the bootloader does not answer; reset the board and try again")
        ser.close()
        return False

    # The image as it was read and checked above, cut into frames.
    total_size = len(file_data)
    packets = pack([file_data[i:i + CHUNK_SIZE] for i in range(0, total_size, CHUNK_SIZE)])
    erase_size = calculate_erase_size(total_size, XSPI_SECTOR_SIZE)
    # Host-side CRC32, printed to compare builds by eye: the v1 command set has
    # no read-back (harness/n6weights.py compares a device-side CRC32).
    host_crc = zlib.crc32(file_data) & 0xFFFFFFFF

    print(f"  Size      : {total_size:,} bytes ({human_bytes(total_size)})")
    print(f"  Erase     : {erase_size:,} bytes ({human_bytes(erase_size)})")
    print(f"  CRC32     : 0x{host_crc:08X} (host-side file checksum, not verified on device)")
    print()

    # DFU START (with address)
    step(1, 4, "Handshake")
    print(f"  Progress  : {bar(1, 4)}")

    # A bootloader that guards its own flash opens it for the next image on
    # this command; an older one has no guard and acknowledges it unread.
    if fsbl:
        ser.write(make_packet(b"DFU_FSBL_UNLOCK"))
        while True:
            resp = ser.readline().decode(errors="ignore").strip()
            if resp == "N6ACK":
                break
            elif resp == "":
                print("ERROR: no response to the boot loader unlock")
                ser.close()
                return False
            else:
                print(f"  Device    : {resp}")

    # Device expects 9 + 4 bytes: "DFU START" + flash_offset (uint32 LE)
    start_payload = b"DFU START" + struct.pack("<I", flash_offset)
    ser.write(make_packet(start_payload))

    # Device sends ADDR_SET message followed by N6ACK
    while True:
        resp = ser.readline().decode(errors="ignore").strip()
        if resp == "N6ACK":
            break
        elif resp == "FSBL_LOCKED":
            print("ERROR: DFU START failed - the bootloader keeps its own flash locked")
            ser.close()
            return False
        elif resp == "START_BAD_ADDR":
            print("ERROR: DFU START failed - invalid address")
            ser.close()
            return False
        elif resp == "":
            print("ERROR: DFU START failed - no response")
            ser.close()
            return False
        else:
            # Progress/info message (e.g., ADDR_SET)
            print(f"  Device    : {resp}")

    # ERASE
    print()
    step(2, 4, "Erase flash")
    print(f"  Progress  : {bar(2, 4)}")
    # Device expects exactly 9 + 4 bytes: "DFU_ERASE" + erase_size (uint32 LE)
    payload = b"DFU_ERASE" + struct.pack("<I", erase_size)
    ser.write(make_packet(payload))

    # Device may send progress messages before N6ACK
    while True:
        resp = ser.readline().decode(errors="ignore").strip()
        if resp == "N6ACK":
            break
        elif resp == "ERASE_BAD_SIZE":
            print("ERROR: Erase failed - bad size")
            ser.close()
            return False
        elif resp in ("ERASE_FLASH_MODE", "DFU_REFUSED"):
            print(f"ERROR: Erase refused by the bootloader ({resp})")
            ser.close()
            return False
        elif resp == "":
            print("ERROR: Erase timeout - no response")
            ser.close()
            return False
        else:
            # Progress message (e.g., ERASE_START)
            print(f"  Device    : {resp}")
    print("  Erase     : complete")

    # WRITE
    print()
    step(3, 4, "Program firmware")
    print(f"  Progress  : {bar(0, total_size)}  0 / {total_size:,} bytes")
    ser.write(make_packet(b"DFU_DATA"))

    resp = ser.readline().decode(errors="ignore").strip()
    if resp != "N6ACK":
        print(f"ERROR: DFU_DATA failed (got {resp!r})")
        ser.close()
        return False

    bytes_written = 0
    bytes_since_16k = 0
    start_time = time.time()
    last_progress = -1

    for packet in packets:
        payload_len = len(packet) - 5
        ser.write(packet)
        bytes_written += payload_len
        bytes_since_16k += payload_len

        # One ACK per frame; none for a frame that fails its checksum, which
        # is not written, so a missing ACK ends the upload (later frames would
        # land 4 KB short). The board stays in the bootloader.
        ack = ser.readline().decode(errors="ignore").strip()
        if ack != "N6ACK":
            print(f"ERROR: Write failed at byte {bytes_written - payload_len:,} - "
                  f"expected ACK, got {ack!r}")
            print("       The board is still in the bootloader; upload again.")
            ser.close()
            return False

        # Device also sends an extra ACK every 16KB from dfu_feed_data
        if bytes_since_16k >= 16384:
            ack = ser.readline().decode(errors="ignore").strip()
            if ack != "N6ACK":
                print(f"ERROR: Write failed at byte {bytes_written:,} - "
                      f"expected the 16 KB ACK, got {ack!r}")
                print("       The board is still in the bootloader; upload again.")
                ser.close()
                return False
            bytes_since_16k = 0

        pct = bytes_written * 100 // total_size
        if pct >= last_progress + 5 or bytes_written >= total_size:
            last_progress = pct
            print(f"  Progress  : {bar(bytes_written, total_size)}  {bytes_written:,} / {total_size:,} bytes")

    elapsed = time.time() - start_time
    speed = total_size / elapsed / 1024 if elapsed > 0 else 0
    print(f"  Write     : {total_size:,} bytes in {elapsed:.1f}s ({speed:.1f} KB/s)")

    # END WRITE - device will send DFU_DONE and immediately reset
    print()
    step(4, 4, "Finalize and reset")
    print(f"  Progress  : {bar(4, 4)}")
    ser.write(make_packet(b"DFU_END"))

    # The reset can come before DFU_DONE leaves. Ahead of it the bootloader
    # says if it could not record the image in the control block.
    replies = []
    try:
        ser.timeout = 2
        while len(replies) < 4:
            response = ser.readline().decode(errors="ignore").strip()
            if not response:
                break
            replies.append(response)
            if response == "DFU_DONE":
                break
    except Exception:
        pass  # the port goes as the board resets
    if "OTA_CB_NOT_UPDATED" in replies:
        print("ERROR: the image was written, but the bootloader could not record it;")
        print("       the board may start another image. Upload again.")
        ser.close()
        return False
    if "DFU_DONE" in replies:
        print("  Device    : acknowledged; resetting")

    ser.close()
    print()
    line()
    boxed("Upload complete")
    line()
    print(f"  Wrote     : {total_size:,} bytes to flash")
    print("  Status    : done; the board should reappear shortly")
    print("  Verify    : not performed - this path cannot read flash back")

    # Flush output and pause so a calling tool sees the completion before exit.
    sys.stdout.flush()
    sys.stderr.flush()
    time.sleep(0.5)

    return True


def upload(port, bin_path, flash_offset, fsbl=False):
    """run_dfu, with a port that fails under it - a write the bootloader
    never took, or the board reset or unplugged - reported on stdout as a
    failed upload, where a calling tool that keeps only stdout still sees
    why."""
    try:
        return run_dfu(port, bin_path, flash_offset, fsbl)
    except serial.SerialException as e:
        print(f"ERROR: The serial link to the bootloader failed: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Neuro N6 DFU Uploader (CLI)")
    parser.add_argument("--port", required=False,
                        help="The board's port, its application's or its bootloader's (e.g. COM3). "
                             "Found by itself with one board connected.")
    parser.add_argument("--bin", required=True, help="Path to binary file")
    parser.add_argument("--address", required=False, default=None,
                        help=f"Flash address in hex (default: 0x{FLASH_BASE + DEFAULT_FLASH_OFFSET:08X}, "
                             f"or 0x{FLASH_BASE:08X} with --fsbl)")
    parser.add_argument("--no-reboot", action="store_true", help="Skip auto-reboot (device already in bootloader)")
    parser.add_argument("--fsbl", action="store_true",
                        help="The image is a boot loader: write it to the boot loader's own flash. "
                             "A wrong image, or power lost during the write, leaves a board that no longer starts.")

    args = parser.parse_args()

    if args.address is None:
        args.address = f"0x{FLASH_BASE + (0 if args.fsbl else DEFAULT_FLASH_OFFSET):08X}"
    flash_offset = parse_address(args.address)
    if flash_offset is None:
        sys.exit(1)

    # Everything that can be checked without the board is checked first, so
    # that a mistake never leaves a board waiting in its bootloader.
    if check_image(args.bin, flash_offset, args.fsbl) is None:
        sys.exit(1)

    port = args.port
    boards = neuro_ports()
    kinds = dict(boards)
    if port is None and len(boards) > 1:
        print("ERROR: more than one Neuro N6 is connected:")
        for dev, kind in boards:
            print(f"         {dev} ({'bootloader' if kind == 'bootloader' else 'application'})")
        print("       Name the one to upload to with --port.")
        sys.exit(1)

    # A board already in its bootloader, named or the only one there, is
    # flashed at once: no reboot, no waiting.
    if port is not None:
        boot = port if kinds.get(port) == "bootloader" else None
    else:
        boot = boards[0][0] if boards and boards[0][1] == "bootloader" else None
    if boot is not None:
        banner("Bootloader already present")
        print(f"  Found     : {boot}")
        success = upload(boot, args.bin, flash_offset, args.fsbl)
        sys.exit(0 if success else 1)

    if port:
        # A port that is not a Neuro N6's is a stale selection, or the board
        # moved: with one board connected, that board is the one meant.
        if port not in kinds:
            apps = [dev for dev, kind in boards if kind == "app"]
            if len(apps) != 1:
                print(f"ERROR: {port} is not a Neuro N6's port")
                sys.exit(1)
            print(f"  Port      : selected {port} is not the Neuro N6 app; using {apps[0]}")
            port = apps[0]

        # After its reset the board comes back on a port of its own, which
        # is never another board's bootloader, one already present.
        others = {dev for dev, kind in boards if kind == "bootloader"}
        if not args.no_reboot:
            banner("Reboot to bootloader")
            _reboot_port(port)
        port = wait_for_bootloader_port(timeout_sec=10, prev_app_port=port, exclude=others)
        if port is None:
            print("ERROR: Bootloader port not found after reboot.")
            sys.exit(1)
    else:
        # No port specified - auto-detect
        if not args.no_reboot:
            banner("Auto-detect board")
            reboot_to_bootloader()

        # Try to find device port, wait if needed
        port = find_bootloader_port()
        if port is None:
            port = wait_for_bootloader_port(timeout_sec=15)
        if port is None:
            print("ERROR: No device found.")
            print("       Double-tap reset to enter bootloader mode, then upload again.")
            sys.exit(1)

    success = upload(port, args.bin, flash_offset, args.fsbl)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
