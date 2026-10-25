#!/usr/bin/env python3
"""Read the Neuro N6 JPEG stream from the vendor interface and judge it.

    python harness/n6stream.py [--seconds 5] [--save frame.jpg] [--console]

Frames are DNV0: a 20-byte header {magic 'DNV0', frame_id, meta_len,
payload_len, flags}, then meta_len + payload_len bytes; flags bit 0 marks a
JPEG payload. The network examples put detections in the metadata and
sometimes send metadata alone (no payload, flag clear), counted as `meta`.
WinUSB binds through the image's MS OS descriptors, so pyusb needs no driver.

Checks each frame's header, JPEG SOI and EOI markers and frame counter, and
that the stream is still sending at the end. Reports frames, bytes, frame rate
and failures per check. --save writes the last good frame; --console echoes
the image's once-a-second console report.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: BSD-3-Clause
# Licensed under the terms in the LICENSE file at the root of this
# repository.

import argparse
import struct
import sys
import threading
import time

try:
    import usb.core
    import usb.util
    import libusb_package
except ImportError:
    print("pyusb and libusb-package are required: pip install pyusb libusb-package", file=sys.stderr)
    sys.exit(2)

VID, PID = 0x1209, 0x8008
MAGIC = 0x30564E44
HDR = struct.Struct("<IIIII")


def open_vendor(retries=12):
    """Find the vendor interface and its IN endpoint, retrying while Windows
    binds WinUSB after enumeration (libusb says "Entity not found" until then)."""
    last = None
    for _ in range(retries):
        dev = libusb_package.find(idVendor=VID, idProduct=PID)
        if dev is None:
            last = "device 1209:8008 not on the bus"; time.sleep(0.5); continue
        try:
            cfg = dev.get_active_configuration()
            intf = cfg[(0, 0)]
            ep_in = next(e for e in intf
                         if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)
            return dev, ep_in
        except (usb.core.USBError, StopIteration, NotImplementedError) as e:
            last = str(e); time.sleep(0.5)
    print("could not open the vendor interface after %d tries: %s" % (retries, last), file=sys.stderr)
    return None, None


MAX_PLEN = 1536 * 1024  # the stream example's JPEG buffer, so no real frame exceeds it
MAX_MLEN = 64 * 1024    # far more metadata than a network example puts in front of a frame

# Anomalies this soon after the first frame are joining, not faults: a reader
# that attaches mid-frame must find the next header and sees the frame counter
# jump. Anything later fails the run.
JOIN_SECONDS = 1.5

# A stream with no frame in this long before the run ends has stopped,
# whatever it sent before; the slowest mode sends one every 40 ms.
STALL_SECONDS = 1.0


def find_header(buf):
    """Offset of the first byte of a header that is really a header, or -1
    when nothing in the buffer is decidable yet. 'DNV0' occurs in JPEG data,
    so a candidate also needs plausible lengths, the JPEG flag and a payload
    starting with SOI."""
    at = 0
    while True:
        i = buf.find(b"DNV0", at)
        if i < 0:
            return -1
        if len(buf) < i + HDR.size + 2:
            return -1                      # not enough to judge; wait for more
        _, _, mlen, plen, flags = HDR.unpack_from(buf, i)
        soi = buf[i + HDR.size + mlen:i + HDR.size + mlen + 2]
        if (mlen <= MAX_MLEN) and (4 <= plen <= MAX_PLEN) and (flags & 1) and (soi == b"\xff\xd8"):
            return i
        at = i + 1


def console_echo(stop):
    try:
        import serial
        sys.path.insert(0, __import__("os").path.dirname(__file__))
        import n6flash
        pid, port = n6flash.bus()
        if pid != n6flash.PID_APP or not port:
            return
        s = serial.Serial(port, 115200, timeout=0.2)
        s.dtr = True; s.rts = True
        s.write(b"?\n")
        while not stop.is_set():
            line = s.readline()
            if line:
                print("  console:", line.decode(errors="replace").rstrip())
        s.close()
    except Exception as e:
        print("  console: %s" % e)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--save", default=None, help="write the last good JPEG here")
    ap.add_argument("--console", action="store_true", help="echo the CDC console while streaming")
    args = ap.parse_args()

    dev, ep_in = open_vendor()
    if dev is None:
        print("no Neuro N6 application (1209:8008) on the bus", file=sys.stderr)
        return 3
    print("vendor IN endpoint 0x%02x, %d-byte packets" % (ep_in.bEndpointAddress, ep_in.wMaxPacketSize))

    stop = threading.Event()
    if args.console:
        threading.Thread(target=console_echo, args=(stop,), daemon=True).start()

    buf = b""; total = 0; frames = 0; bad_hdr = 0; bad_jpeg = 0; gaps = 0; meta = 0
    joining = 0
    last_id = None; last_good = None; sizes = []
    usb_error = None
    t0 = time.time(); t_first = None; t_last = None
    # The first bytes a reader sees are usually mid-frame: skip to a header.
    synced = False
    while time.time() - t0 < args.seconds:
        try:
            chunk = bytes(ep_in.read(64 * 1024, timeout=500))
        except usb.core.USBTimeoutError:
            continue
        except usb.core.USBError as e:
            usb_error = str(e); print("usb error: %s" % e); break
        if not chunk:
            continue
        total += len(chunk); buf += chunk
        if not synced:
            i = find_header(buf)
            if i < 0:
                buf = buf[-MAX_PLEN:]   # keep enough to finish judging a candidate
                continue
            buf = buf[i:]; synced = True
        while len(buf) >= HDR.size:
            joined = (t_first is not None) and ((time.time() - t_first) > JOIN_SECONDS)
            magic, fid, mlen, plen, flags = HDR.unpack_from(buf, 0)
            if magic != MAGIC:
                if joined:
                    bad_hdr += 1
                else:
                    joining += 1
                i = buf.find(b"DNV0", 1)
                buf = buf[i:] if i > 0 else b""
                continue
            need = HDR.size + mlen + plen
            if len(buf) < need:
                break
            payload = buf[HDR.size + mlen:need]; buf = buf[need:]
            if last_id is not None and fid != last_id + 1:
                if joined:
                    gaps += 1
                else:
                    joining += 1
            last_id = fid
            if (plen == 0) and not (flags & 1):
                meta += 1                  # metadata alone: not a picture
                continue
            if t_first is None:
                t_first = time.time()
            t_last = time.time()
            frames += 1; sizes.append(plen)
            ok = (flags & 1) and plen >= 4 and payload[:2] == b"\xff\xd8" and payload[-2:] == b"\xff\xd9"
            if ok:
                last_good = payload
            elif joined:
                bad_jpeg += 1
            else:
                joining += 1
    stop.set()
    t_end = time.time()

    dt = (time.time() - t_first) if t_first else 0
    fps = (frames - 1) / dt if frames > 1 and dt > 0 else 0.0
    print("frames=%d bytes=%d fps=%.1f size=%s..%s bad_header=%d bad_jpeg=%d id_gaps=%d joining=%d meta=%d"
          % (frames, total, fps, min(sizes) if sizes else "-", max(sizes) if sizes else "-",
             bad_hdr, bad_jpeg, gaps, joining, meta))
    if args.save and last_good:
        open(args.save, "wb").write(last_good)
        print("saved %s (%d bytes)" % (args.save, len(last_good)))
    if frames == 0:
        print("FAIL: no frames", file=sys.stderr); return 1
    if usb_error:
        print("FAIL: the link failed %.1f s into the run: %s" % (t_end - t0, usb_error),
              file=sys.stderr); return 1
    if t_end - t_last > STALL_SECONDS:
        print("FAIL: the stream stopped; no frame in the last %.1f s" % (t_end - t_last),
              file=sys.stderr); return 1
    if bad_jpeg or bad_hdr:
        print("FAIL: malformed frames", file=sys.stderr); return 1
    if gaps:
        print("FAIL: %d gap%s in the frame counter" % (gaps, "" if gaps == 1 else "s"),
              file=sys.stderr); return 1
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
