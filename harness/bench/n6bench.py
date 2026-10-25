#!/usr/bin/env python3
"""Neuro N6 bench helper: console reader, vendor-stream validator, command sender,
and the two-way Wi-Fi checks.

    n6bench.py ls
    n6bench.py console [--seconds N] [--send "text"] [--send-at S] [--log FILE]
    n6bench.py stream  [--tcp HOST[:PORT]] [--seconds N] [--log FILE] [--quiet]
    n6bench.py cmd     [--tcp HOST[:PORT]] --hex "52 45 ..." | --text "REBOOT_DFU"
    n6bench.py desc
    n6bench.py probe   [--addr ADDR] [--port 5000] [--timeout S]
    n6bench.py wifitest --host HOST [--port 5000] [--soak N] [--usb] [--any-app] [--json FILE]
    n6bench.py usbtest [--json FILE]
    n6bench.py wifiset --sta SSID | --ap SSID [--board-password] [--key-env VAR] [--security S]
                       [--channel N] [--hostname NAME] [--link-key-env VAR]
    n6bench.py wifiset --clear

console   the application's CDC port (VID 1209 PID 8008), DTR asserted, each
          line with a host-side millisecond timestamp; --send writes the text
          (plus CR) --send-at seconds after opening (default 1.0), pairs in
          order.
stream    the vendor stream, from bulk-IN 0x81 of interface 0 or, with --tcp,
          a board's Wi-Fi host server: checks the 20-byte
          [magic|frame_id|meta_len|payload_len|flags] framing, JPEG SOI/EOI,
          meta-only frames and desyncs, decodes a few TLVs, and prints a stats
          line a second and a summary.
cmd       one command to bulk-OUT 0x01 or, with --tcp, the Wi-Fi host server,
          framed as n6_host_contract.h describes.
desc      the configuration descriptor's bMaxPower and the interface list.
probe     a discovery broadcast to UDP --port; lists the boards that answer.
wifitest  the two-way Wi-Fi checks against the two-way Wi-Fi bench firmware,
          this PC on the same network. --usb adds the USB <-> Wi-Fi hand-over
          (the board's cable on this PC); --any-app keeps the checks any Wi-Fi
          application passes. usbtest runs the stream and control checks over
          USB, and also takes --any-app. Both exit 1 if anything failed.
wifiset   stores Wi-Fi settings over USB for MayaW476_BeginStored(): --sta
          joins that network, --ap hosts it (--board-password: with the
          board's own password). The passphrase comes from an environment
          variable (--key-env, default N6_WIFI_KEY), never the command line,
          and is never printed. WPA2 and WPA3 only. The board restarts to
          store them (an application without the Wi-Fi library ignores them);
          --clear removes them.

A board with a link key (MayaW476_SetLinkKey()) challenges every network host;
the Wi-Fi commands answer with the key in the environment variable
N6_LINK_KEY.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT

import argparse
import io
import json
import socket
import struct
import sys
import time
import zlib

VID = 0x1209
PID_APP = 0x8008
PID_FSBL = 0x8009
VND_MAGIC = 0x30564E44  # "DNV0" little-endian
HDR = struct.Struct("<IIIII")  # magic, frame_id, meta_len, payload_len, flags
NET_PORT = 5000

# CAM0 bits (NeuroN6_app.h)
CAM0_FLAG_AUTO_EXPOSURE = 0x01
CAM0_FLAG_AUTO_GAIN = 0x02
CAM0_APPLY_AUTO_EXPOSURE = 0x01
CAM0_APPLY_AUTO_GAIN = 0x02
CAM0_APPLY_EXPOSURE_US = 0x08
CAM0_APPLY_GAIN = 0x10


def _now_ms(t0):
    return int((time.perf_counter() - t0) * 1000)


# Serial console.
def find_port(pid):
    from serial.tools import list_ports
    for p in list_ports.comports():
        if p.vid == VID and p.pid == pid:
            return p.device
    return None


def wait_port(pid, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        dev = find_port(pid)
        if dev:
            return dev
        time.sleep(0.25)
    return None


def cmd_ls(args):
    from serial.tools import list_ports
    for p in list_ports.comports():
        vid = "%04x" % p.vid if p.vid else "----"
        pid = "%04x" % p.pid if p.pid else "----"
        print("PORT %-6s vid=%s pid=%s %s" % (p.device, vid, pid, p.description))
    try:
        dev = usb_open(required=False)
        if dev is not None:
            print("USB  vid=%04x pid=%04x bcdDevice=%04x" % (dev.idVendor, dev.idProduct, dev.bcdDevice))
    except Exception as e:  # noqa: BLE001
        print("USB  enumerate failed:", e)


def cmd_console(args):
    import serial
    port = args.port or wait_port(PID_APP, args.wait)
    if not port:
        print("console: no application port (pid 8008) within %ss" % args.wait)
        return 2
    ser = serial.Serial(port, 115200, timeout=0.05)
    ser.dtr = True
    ser.rts = True
    log = open(args.log, "a", encoding="utf-8", errors="replace") if args.log else None
    t0 = time.perf_counter()
    sends = list(zip(args.send or [], args.send_at or []))
    # pad send_at with 1.0 if fewer were given
    if args.send and (not args.send_at or len(args.send_at) < len(args.send)):
        base = list(args.send_at or [])
        while len(base) < len(args.send):
            base.append(1.0 if not base else base[-1] + 1.0)
        sends = list(zip(args.send, base))
    sends.sort(key=lambda s: s[1])
    pending = list(sends)
    buf = b""
    end = t0 + args.seconds
    lines = 0
    try:
        while time.perf_counter() < end:
            while pending and (time.perf_counter() - t0) >= pending[0][1]:
                text, _ = pending.pop(0)
                ser.write((text + "\r").encode())
                ser.flush()
                msg = "[%7d] >> %s" % (_now_ms(t0), text)
                print(msg)
                if log:
                    log.write(msg + "\n")
            chunk = ser.read(4096)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    s = line.decode("utf-8", errors="replace").rstrip("\r")
                    msg = "[%7d] %s" % (_now_ms(t0), s)
                    lines += 1
                    if not args.grep or any(g in s for g in args.grep):
                        print(msg)
                    if log:
                        log.write(msg + "\n")
                if log:
                    log.flush()
    except serial.SerialException as e:
        print("[%7d] console: port went away (%s)" % (_now_ms(t0), e))
    finally:
        try:
            ser.close()
        except Exception:  # noqa: BLE001
            pass
        if log:
            log.close()
    print("console: %d lines" % lines)
    return 0


# Vendor endpoint.
def usb_open(required=True, pid=PID_APP, wait=10.0):
    import usb.core
    import libusb_package
    backend = libusb_package.get_libusb1_backend()
    t0 = time.time()
    dev = None
    while time.time() - t0 < wait:
        dev = usb.core.find(backend=backend, idVendor=VID, idProduct=pid)
        if dev is not None:
            break
        if not required:
            break
        time.sleep(0.25)
    if dev is None:
        if required:
            raise SystemExit("usb: device %04x:%04x not found" % (VID, pid))
        return None
    return dev


def usb_claim(dev):
    import usb.util
    usb.util.claim_interface(dev, 0)   # never SET_CONFIGURATION: it bounces the composite device


def cmd_desc(args):
    dev = usb_open()
    cfg = dev.get_active_configuration()
    print("bcdUSB=%04x bcdDevice=%04x bNumConfigurations=%d" % (dev.bcdUSB, dev.bcdDevice, dev.bNumConfigurations))
    print("config %d: bmAttributes=0x%02x bMaxPower=%d mA" % (cfg.bConfigurationValue, cfg.bmAttributes, cfg.bMaxPower * 2))
    for intf in cfg:
        eps = ", ".join("0x%02x(%s,%d)" % (e.bEndpointAddress, "bulk" if (e.bmAttributes & 3) == 2 else "other", e.wMaxPacketSize) for e in intf)
        print("  itf %d alt %d class %02x/%02x/%02x eps: %s" % (intf.bInterfaceNumber, intf.bAlternateSetting, intf.bInterfaceClass, intf.bInterfaceSubClass, intf.bInterfaceProtocol, eps))
    return 0


# Host commands, and the two links that carry them.
def tun0(idx, value):
    return struct.pack("<4sBBi", b"TUN0", idx & 0xFF, 0, int(value))


def sub0(mask):
    return struct.pack("<4sI", b"SUB0", mask)


def cam0(flags, exposure_us=0, gain_x256=0, apply_mask=0):
    return struct.pack("<4sBIHB", b"CAM0", flags, exposure_us, gain_x256, apply_mask)


def frame_cmd(cmd):
    """One command as the network link carries it (n6_host_contract.h)."""
    return HDR.pack(VND_MAGIC, 0, 0, len(cmd), 0) + cmd


def host_port(spec, default_port=NET_PORT):
    host, _, port = spec.partition(":")
    return host, (int(port) if port else default_port)


class UsbLink:
    """The vendor endpoints. Commands go out raw: a bulk transfer delimits them."""
    name = "USB"

    def __init__(self):
        self.dev = usb_open()
        usb_claim(self.dev)
        self.pipe_errors = 0

    def read(self):
        import usb.core
        try:
            return bytes(self.dev.read(0x81, 1 << 16, timeout=300))
        except usb.core.USBTimeoutError:
            return b""
        except usb.core.USBError as e:
            if getattr(e, "errno", None) == 32 and self.pipe_errors < 5:
                self.pipe_errors += 1
                try:
                    self.dev.clear_halt(0x81)
                except Exception:  # noqa: BLE001
                    pass
                return b""
            raise ConnectionError("usb: %s" % e)

    def send(self, cmd):
        self.dev.write(0x01, cmd, timeout=1000)

    def close(self):
        import usb.util
        usb.util.dispose_resources(self.dev)


LINK_CONTEXT = b"N6LINK1"
LINK_CHALLENGE = 16


def link_answer(key, challenge):
    """The KEY0 command that proves `key` (the link key, n6_host_contract.h)."""
    import hashlib
    import hmac
    return b"KEY0" + hmac.new(key, LINK_CONTEXT + challenge, hashlib.sha256).digest()


def read_challenge(data):
    """The challenge, if `data` starts with a board's link-key challenge frame."""
    if len(data) < HDR.size + 3 + LINK_CHALLENGE:
        return None
    magic, _, meta_len, payload_len, _ = HDR.unpack_from(data)
    tag, ln = data[HDR.size], data[HDR.size + 1] | (data[HDR.size + 2] << 8)
    if magic != VND_MAGIC or meta_len != 3 + LINK_CHALLENGE or payload_len != 0 or \
            tag != 0x23 or ln != LINK_CHALLENGE:
        return None
    return bytes(data[HDR.size + 3:HDR.size + 3 + LINK_CHALLENGE])


class TcpLink:
    """A board's WiFi host server. Commands go out behind a frame header. A
    board with a link key challenges first; `link_key` (default: the
    environment variable N6_LINK_KEY) answers it, and `challenged` records
    that it asked."""
    name = "WiFi"

    def __init__(self, host, port=NET_PORT, timeout=3.0, rcvbuf=0, link_key=None):
        import os
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if rcvbuf:
            # Before connect, so the advertised window really is this small.
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
        self.sock.settimeout(timeout)
        self.sock.connect((host, port))
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(0.3)
        self.pending = b""
        self.challenged = False
        if link_key is None:
            link_key = os.environ.get("N6_LINK_KEY", "").encode()
        # A keyed board speaks first, with the challenge alone; any other board
        # starts streaming. Wait for enough bytes to tell which, briefly.
        end = time.time() + 1.0
        while len(self.pending) < HDR.size + 3 + LINK_CHALLENGE and time.time() < end:
            self.pending += self.read()
        challenge = read_challenge(self.pending)
        if challenge is not None:
            self.challenged = True
            self.pending = self.pending[HDR.size + 3 + LINK_CHALLENGE:]
            if not link_key:
                self.close()
                raise ConnectionError("the board wants a link key: set N6_LINK_KEY")
            self.send(link_answer(link_key, challenge))

    def read(self):
        if self.pending:
            data, self.pending = self.pending, b""
            return data
        try:
            data = self.sock.recv(1 << 16)
        except socket.timeout:
            return b""
        except OSError as e:
            raise ConnectionError(str(e))
        if not data:
            raise ConnectionError("the board closed the connection")
        return data

    def send(self, cmd):
        self.send_raw(frame_cmd(cmd))

    def send_raw(self, data):
        try:
            self.sock.sendall(data)
        except OSError as e:
            raise ConnectionError(str(e))

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


# Stream parsing.
class StreamStats:
    def __init__(self):
        self.frames = 0
        self.jpeg = 0
        self.jpeg_bad = 0
        self.meta_only = 0
        self.desync = 0
        self.id_gaps = 0
        self.bytes = 0
        self.tags = {}
        self.ipv4 = None
        self.board_name = None      # META_BOARD_NAME (0x21)
        self.board_keyed = False    # META_BOARD_LINK (0x22) bit 0
        self.debug = []
        self.last_id = None
        self.max_meta = 0
        self.pipe_errors = 0
        self.synced = False
        self.desync_pre = 0
        # numbered Terminal log (0x71): (number, text, host arrival time)
        self.log = []
        self.log_last = 0
        self.log_gaps = 0
        self.log_boot = None    # the record's boot byte; older boards send none
        self.log_boots = set()  # every boot byte seen
        self.tune = None
        self.cam_caps = None
        self.isp_mode = None
        self.vision_kind = None
        self.crops = 0
        self.crop_dims = None
        self.last_jpeg = None

    def line(self, t):
        top = ",".join("%02x:%d" % kv for kv in sorted(self.tags.items())[:10])
        return ("[%6.1fs] frames=%d jpeg=%d badjpeg=%d meta=%d desync=%d (pre-sync %d) idgap=%d pipe=%d MB=%.1f ipv4=%s tags=%s" %
                (t, self.frames, self.jpeg, self.jpeg_bad, self.meta_only, self.desync, self.desync_pre, self.id_gaps,
                 self.pipe_errors, self.bytes / 1e6, self.ipv4, top))


def parse_tune(v):
    """META_TUNE_DESC: u8 count, then per parameter u8 id, type, hint, name_len,
    the name, and i32 lo, hi, current."""
    out = []
    if not v:
        return out
    p = 1
    for _ in range(v[0]):
        if p + 4 > len(v):
            break
        pid, typ, _hint, nl = v[p], v[p + 1], v[p + 2], v[p + 3]
        p += 4
        name = bytes(v[p:p + nl]).decode("utf-8", errors="replace")
        p += nl
        if p + 12 > len(v):
            break
        lo, hi, cur = struct.unpack_from("<iii", v, p)
        p += 12
        out.append({"id": pid, "type": typ, "name": name, "lo": lo, "hi": hi, "value": cur})
    return out


def tune_value(st, name):
    for prm in st.tune or []:
        if prm["name"] == name:
            return prm["value"]
    return None


def parse_log(v, st):
    """META_DEBUG_LOG: u32 number of the first line, u8 count, then u8 length
    and text per line, then, from newer boards, a boot byte. Keeps each number
    once and counts the ones skipped; a new boot byte, or numbers going
    backwards, is a restart."""
    if len(v) < 5:
        return
    first, count = struct.unpack_from("<IB", v, 0)
    p, lines = 5, []
    for _ in range(count):
        if p >= len(v):
            break
        n = v[p]
        lines.append(bytes(v[p + 1:p + 1 + n]).decode("utf-8", errors="replace").rstrip("\r\n"))
        p += 1 + n
    if p < len(v):
        boot = v[p]
        if st.log_boot is not None and boot != st.log_boot:
            st.log_last = 0   # a new boot: its numbers start again
        st.log_boot = boot
        st.log_boots.add(boot)
    if lines and first + len(lines) - 1 < st.log_last:
        st.log_last = 0   # numbers went backwards: the board restarted
    now = time.perf_counter()
    for i, text in enumerate(lines):
        num = first + i
        if num <= st.log_last:
            continue
        if st.log_last and num > st.log_last + 1:
            st.log_gaps += num - st.log_last - 1
        st.log.append((num, text, now))
        st.log_last = num
    if len(st.log) > 2000:
        del st.log[:1000]


def parse_tlvs(meta, st):
    i = 0
    n = len(meta)
    while i + 3 <= n:
        tag = meta[i]
        ln = meta[i + 1] | (meta[i + 2] << 8)
        v0 = i + 3
        if v0 + ln > n:
            st.desync += 1
            return
        st.tags[tag] = st.tags.get(tag, 0) + 1
        v = meta[v0:v0 + ln]
        if tag == 0x20 and ln >= 4:
            st.ipv4 = "%d.%d.%d.%d:%d" % (v[0], v[1], v[2], v[3],
                                          (v[4] | (v[5] << 8)) if ln >= 6 else 0)
        elif tag == 0x21:
            st.board_name = bytes(v).decode("utf-8", errors="replace")
        elif tag == 0x22 and ln >= 1:
            st.board_keyed = bool(v[0] & 0x01)
        elif tag == 0x17:
            txt = bytes(v).decode("utf-8", errors="replace").rstrip("\r\n")
            st.debug.append(txt)
            if len(st.debug) > 200:
                del st.debug[:100]
        elif tag == 0x71:
            parse_log(v, st)
        elif tag == 0x40 and ln >= 1:
            st.isp_mode = v[0]
        elif tag == 0x60 and ln >= 1:
            st.vision_kind = v[0]
        elif tag == 0x64 and ln >= 1:
            st.cam_caps = v[0]
        elif tag == 0x65:
            st.tune = parse_tune(v)
        elif tag == 0x70 and ln >= 1:
            st.crops += v[0]
            if v[0] and ln >= 4:
                st.crop_dims = (v[2], v[3])
        i = v0 + ln


class FrameParser:
    """Splits a byte stream into vendor frames, resynchronising on the magic."""

    def __init__(self, st):
        self.st = st
        self.buf = bytearray()

    def feed(self, data):
        st, buf = self.st, self.buf
        buf += data
        st.bytes += len(data)
        while len(buf) >= 20:
            magic, fid, mlen, plen, _flags = HDR.unpack_from(buf, 0)
            if magic != VND_MAGIC or mlen > (4 << 20) or plen > (8 << 20):
                if st.synced:
                    st.desync += 1
                else:
                    st.desync_pre += 1
                idx = buf.find(b"DNV0", 1)
                if idx < 0:
                    buf.clear()
                    break
                del buf[:idx]
                continue
            total = 20 + mlen + plen
            if len(buf) < total:
                break
            meta = bytes(buf[20:20 + mlen])
            payload = bytes(buf[20 + mlen:total])
            st.frames += 1
            st.synced = True
            if st.last_id is not None and fid != st.last_id + 1 and fid != 0:
                st.id_gaps += 1
            st.last_id = fid
            st.max_meta = max(st.max_meta, mlen)
            if plen:
                st.jpeg += 1
                st.last_jpeg = payload
                if not (plen >= 4 and payload[0] == 0xFF and payload[1] == 0xD8 and payload[-2] == 0xFF and payload[-1] == 0xD9):
                    st.jpeg_bad += 1
            else:
                st.meta_only += 1
            if mlen:
                parse_tlvs(meta, st)
            del buf[:total]


class Session:
    """One host's view of the stream over a link."""

    def __init__(self, link):
        self.link = link
        self.st = StreamStats()
        self.parser = FrameParser(self.st)
        self.t0 = time.perf_counter()
        self.closed = False
        self.close_reason = ""

    def pump(self, seconds, until=None):
        """Read for up to `seconds`, returning early (True) once until(self)
        holds. Returns False if the board closed the link."""
        end = time.perf_counter() + seconds
        while True:
            if until is not None and until(self):
                return True
            if self.closed or time.perf_counter() >= end:
                break
            try:
                data = self.link.read()
            except ConnectionError as e:
                self.closed, self.close_reason = True, str(e)
                break
            if data:
                self.parser.feed(data)
        return (until(self) if until is not None else True) and not self.closed

    def send(self, cmd):
        try:
            self.link.send(cmd)
        except (ConnectionError, OSError) as e:
            self.closed, self.close_reason = True, str(e)

    def saw(self, prefix, after=0.0):
        return any(t.startswith(prefix) and at >= after for _, t, at in self.st.log)

    def close(self):
        self.link.close()


def cmd_stream(args):
    link = TcpLink(*host_port(args.tcp)) if args.tcp else UsbLink()
    sess = Session(link)
    st = sess.st
    t0 = sess.t0
    end = t0 + args.seconds
    next_print = t0 + 1.0
    log = open(args.log, "a", encoding="utf-8") if args.log else None
    # scheduled host->device packets: (at_seconds, bytes, label)
    sched = []
    for i, spec in enumerate(args.tun or []):
        idx, val = spec.split(":")
        at = args.tun_at[i] if args.tun_at and i < len(args.tun_at) else 2.0 + i
        sched.append((float(at), tun0(int(idx), int(val)), "TUN0 id=%s value=%s" % (idx, val)))
    for i, spec in enumerate(args.raw or []):
        at = args.raw_at[i] if args.raw_at and i < len(args.raw_at) else 2.0 + i
        data = bytes.fromhex(spec) if all(c in "0123456789abcdefABCDEF " for c in spec) else spec.encode()
        sched.append((float(at), data, "RAW %r" % data[:24]))
    sched.sort(key=lambda x: x[0])
    shown_log = 0
    last_debug = None

    def emit(line):
        print(line)
        if log:
            log.write(line + "\n")

    while time.perf_counter() < end and not sess.closed:
        while sched and (time.perf_counter() - t0) >= sched[0][0]:
            _at, data, label = sched.pop(0)
            try:
                link.send(data)
                print("[%6.1fs] >> %s" % (time.perf_counter() - t0, label))
            except Exception as e:  # noqa: BLE001
                print("[%6.1fs] >> %s FAILED: %s" % (time.perf_counter() - t0, label, e))
        sess.pump(0.3)
        now = time.perf_counter()
        # The numbered log when the board sends one, else the latest-line slot.
        for num, text, _at in st.log[shown_log:]:
            emit("[%6.1fs] LOG #%d %s" % (now - t0, num, text))
        shown_log = len(st.log)
        if not st.log and st.debug and st.debug[-1] != last_debug:
            last_debug = st.debug[-1]
            emit("[%6.1fs] DBG %s" % (now - t0, last_debug))
        if now >= next_print:
            next_print += 1.0
            line = st.line(now - t0)
            if not args.quiet:
                print(line)
            if log:
                log.write(line + "\n")
                log.flush()
    if sess.closed:
        print("stream: %s" % sess.close_reason)
    st.pipe_errors = getattr(link, "pipe_errors", 0)
    print("SUMMARY " + st.line(time.perf_counter() - t0) + " max_meta=%d" % st.max_meta)
    if st.debug and args.debug:
        print("--- last debug text ---")
        for d in st.debug[-args.debug:]:
            print("  " + d)
    if log:
        log.close()
    sess.close()
    return 0


def cmd_cmd(args):
    if args.hex:
        data = bytes.fromhex(args.hex.replace(" ", ""))
    else:
        data = args.text.encode()
    if args.tcp:
        link = TcpLink(*host_port(args.tcp))
        link.send(data)
        time.sleep(0.2)   # let it leave before the close
        link.close()
    else:
        dev = usb_open()
        usb_claim(dev)
        dev.write(0x01, data, timeout=2000)
    print("cmd: wrote %d bytes: %s" % (len(data), data[:32]))
    return 0


# Discovery.
def local_ipv4():
    """This PC's IPv4 addresses, loopback left out."""
    try:
        infos = socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)
    except OSError:
        return []
    return sorted({i[4][0] for i in infos if not i[4][0].startswith("127.")})


def discover(port=NET_PORT, addr="255.255.255.255", timeout=2.0, details=None):
    """Send one probe and collect the answers: {sender address: announced ip:port}.
    With a dict for `details`, also {sender: (name, keyed)} from the reply's
    META_BOARD_NAME and META_BOARD_LINK. A probe to 255.255.255.255 goes out
    of every interface, as Neuro Studio sends it: Windows sends a limited
    broadcast out of one interface only, the lowest metric."""
    socks = []
    for src in (local_ipv4() if addr == "255.255.255.255" else []) or [""]:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            s.bind((src, 0))
            s.sendto(HDR.pack(VND_MAGIC, 0, 0, 0, 0), (addr, port))
            s.setblocking(False)
            socks.append(s)
        except OSError as e:
            if src == "":
                print("probe: %s" % e)
            s.close()
    found = {}
    end = time.time() + timeout
    try:
        while time.time() < end and socks:
            for s in socks:
                try:
                    data, src = s.recvfrom(1024)
                except (BlockingIOError, OSError):   # nothing yet, or Windows' ICMP port-unreachable
                    continue
                st = StreamStats()
                FrameParser(st).feed(data)
                if st.ipv4:
                    found[src[0]] = st.ipv4
                    if details is not None:
                        details[src[0]] = (st.board_name, st.board_keyed)
            time.sleep(0.02)
    finally:
        for s in socks:
            s.close()
    return found


def cmd_probe(args):
    details = {}
    found = discover(args.port, args.addr, args.timeout, details)
    for src, ann in sorted(found.items()):
        name, keyed = details.get(src, (None, False))
        print("board at %s announces %s%s%s" % (src, ann, " name %s" % name if name else "",
                                              ", link key required" if keyed else ""))
    if not found:
        print("probe: no answer from %s:%d" % (args.addr, args.port))
        return 1
    return 0


# The Wi-Fi and USB checks.
class Report:
    def __init__(self):
        self.rows = []

    def add(self, status, name, detail=""):
        self.rows.append({"status": status, "check": name, "detail": detail})
        print("%-4s  %-52s %s" % (status, name, detail))
        sys.stdout.flush()

    def check(self, name, ok, detail=""):
        self.add("PASS" if ok else "FAIL", name, detail)
        return ok

    def info(self, name, detail=""):
        self.add("INFO", name, detail)

    def skip(self, name, detail=""):
        self.add("SKIP", name, detail)

    def finish(self, path):
        count = {k: sum(1 for r in self.rows if r["status"] == k) for k in ("PASS", "FAIL", "SKIP")}
        print("\n%d passed, %d failed, %d skipped" % (count["PASS"], count["FAIL"], count["SKIP"]))
        if path:
            with open(path, "w") as fh:
                json.dump({"summary": count, "checks": self.rows}, fh, indent=1)
        return 1 if count["FAIL"] else 0


def jpeg_brightness(jpeg):
    """Mean luma of a JPEG, or None without Pillow."""
    try:
        from PIL import Image, ImageStat
    except ImportError:
        return None
    try:
        return ImageStat.Stat(Image.open(io.BytesIO(jpeg)).convert("L")).mean[0]
    except Exception:  # noqa: BLE001
        return None


def stream_checks(sess, rep, min_fps, replay, bench=True):
    """bench: the board runs the two-way Wi-Fi bench firmware, whose tuning parameters and
    heartbeat the checks can rely on; otherwise only what any application
    sends."""
    st = sess.st
    via = sess.link.name
    ok = sess.pump(5, lambda s: s.st.jpeg >= 3)
    rep.check("%s: frames arrive" % via, ok,
              "%d JPEG, %d meta-only%s" % (st.jpeg, st.meta_only,
                                           (", " + sess.close_reason) if sess.closed else ""))
    if sess.closed:
        return False
    # A network connect starts a session (the board drops its queue), so every
    # Terminal number from here must arrive. Opening USB starts nothing, so over
    # USB only the numbers after the waiting backlog count.
    if not replay:
        sess.pump(1.5)
    gaps0 = 0 if replay else st.log_gaps
    t, j = time.perf_counter(), st.jpeg
    sess.pump(5)
    fps = (st.jpeg - j) / max(time.perf_counter() - t, 1e-3)
    rep.check("%s: frame rate" % via, fps >= min_fps, "%.1f fps (minimum %.1f)" % (fps, min_fps))
    rep.check("%s: stream is clean" % via, st.desync == 0 and st.jpeg_bad == 0,
              "desync %d, bad JPEG %d, frame-id gaps %d" % (st.desync, st.jpeg_bad, st.id_gaps))
    tune = ", ".join("%s=%d" % (p["name"], p["value"]) for p in st.tune or []) or "none"
    if bench:
        rep.check("%s: tuning announce (0x65)" % via, bool(st.tune), tune)
    else:
        rep.info("%s: tuning announce (0x65)" % via, tune)
    rep.check("%s: camera-control announce (0x64)" % via, st.cam_caps is not None,
              "caps 0x%02x" % st.cam_caps if st.cam_caps is not None else "none")
    rep.info("%s: ISP announce (0x40)" % via,
             "mode %d" % st.isp_mode if st.isp_mode is not None else "none (expected on an OV5640)")
    if replay:
        rep.check("%s: Terminal replays the boot log on connect" % via, sess.saw("[bootlog]"),
                  "%d numbered lines so far" % len(st.log))
    beats = sum(1 for _, t_, _a in st.log if t_.startswith("[beat]"))
    lost = st.log_gaps - gaps0
    enough = beats >= 3 if bench else len(st.log) >= 1
    boots = ", boot byte %s" % "/".join("0x%02x" % b for b in sorted(st.log_boots)) \
        if st.log_boots else ", no boot byte (an older board)"
    rep.check("%s: Terminal lines arrive numbered, none lost" % via, enough and lost == 0,
              "%d heartbeats, %d lines, %d missing%s%s" % (
                  beats, len(st.log), lost,
                  "" if replay else " (%d before the backlog cleared, not counted)" % gaps0,
                  boots))
    return True


def control_checks(sess, rep, fresh_session, bench=True):
    """fresh_session: the link started a session on the board (a network connect
    does; opening USB does not, so slots can still hold an earlier host's data)."""
    if bench:
        app_checks(sess, rep, fresh_session)
    else:
        rep.skip("%s: tuning and thumbnail checks" % sess.link.name, "need the bench firmware")
    camera_check(sess, rep)


def app_checks(sess, rep, fresh_session):
    st = sess.st
    via = sess.link.name

    # TUN0
    level = next((p for p in st.tune or [] if p["name"] == "level"), None)
    if level is None:
        rep.skip("%s: TUN0 changes a tuning value" % via, "no 'level' parameter: is WiFiTwoWay running?")
    else:
        new = 77 if level["value"] != 77 else 33
        t = time.perf_counter()
        sess.send(tun0(level["id"], new))
        ok = sess.pump(4, lambda s: tune_value(s.st, "level") == new)
        rep.check("%s: TUN0 changes a tuning value" % via, ok,
                  "level %d -> %s" % (level["value"], tune_value(st, "level")))
        ok = sess.pump(3, lambda s: s.saw("[tune] level=%d " % new, t))
        rep.check("%s: the board sees the new value" % via, ok, "'[tune] level=%d ...'" % new)

    # SUB0: thumbnails, the gate, and the lapse
    c0 = st.crops
    if fresh_session:
        rep.check("%s: no thumbnails before subscribing" % via, c0 == 0, "%d" % c0)
    else:
        rep.info("%s: thumbnails before subscribing" % via,
                 "%d (an earlier subscription's last one, still in its slot)" % c0 if c0 else "none")
    t_sub = last = time.perf_counter()
    sess.send(sub0(1))
    while time.perf_counter() - t_sub < 4 and not sess.closed:
        if time.perf_counter() - last >= 1.0:
            last = time.perf_counter()
            sess.send(sub0(1))
        sess.pump(0.1)
    rep.check("%s: SUB0 brings thumbnails" % via, st.crops > c0,
              "%d thumbnails%s" % (st.crops - c0, (", %dx%d" % st.crop_dims) if st.crop_dims else ""))
    rep.check("%s: the board sees the subscription" % via, sess.saw("[gate] crops wanted: yes", t_sub))
    ok = sess.pump(6, lambda s: s.saw("[gate] crops wanted: no", last))
    lapse = next((at - last for _, t_, at in st.log
                  if t_.startswith("[gate] crops wanted: no") and at >= last), None)
    rep.check("%s: the subscription lapses about 3 s after the last SUB0" % via,
              ok and lapse is not None and 2.5 <= lapse <= 4.5,
              "%.1f s" % lapse if lapse is not None else "no lapse seen")
    c1 = st.crops
    sess.pump(2)
    rep.info("%s: thumbnails still arriving after the lapse" % via,
             "%d (the last one stays in its slot; existing behaviour)" % (st.crops - c1))
    t = time.perf_counter()
    sess.send(sub0(1))
    sess.pump(1.5, lambda s: s.saw("[gate] crops wanted: yes", t))
    t = time.perf_counter()
    sess.send(sub0(0))
    ok = sess.pump(2, lambda s: s.saw("[gate] crops wanted: no", t))
    rep.check("%s: SUB0 with mask 0 stops it at once" % via, ok)



def camera_check(sess, rep):
    st = sess.st
    via = sess.link.name
    # CAM0: manual exposure at a fixed 4x gain (manual, or the gain loop undoes
    # the exposure), short then long, then back to auto. Each setting is sent a
    # few times: the board drops a CAM0 that arrives while its I2C bus is busy.
    if st.cam_caps is None or not st.cam_caps & 1:
        rep.skip("%s: CAM0 changes the exposure" % via, "the sensor announces no exposure control")
        return
    manual = (CAM0_APPLY_AUTO_EXPOSURE | CAM0_APPLY_AUTO_GAIN |
              CAM0_APPLY_EXPOSURE_US | CAM0_APPLY_GAIN)
    shots = {}
    for us in (500, 60000):
        for _ in range(3):
            sess.send(cam0(0, us, 4 * 256, manual))
            sess.pump(0.3)
        sess.pump(2.0)
        jpeg = st.last_jpeg or b""
        shots[us] = (jpeg_brightness(jpeg), len(jpeg))
    for _ in range(3):
        sess.send(cam0(CAM0_FLAG_AUTO_EXPOSURE | CAM0_FLAG_AUTO_GAIN, 0, 0,
                       CAM0_APPLY_AUTO_EXPOSURE | CAM0_APPLY_AUTO_GAIN))
        sess.pump(0.3)
    (b_short, n_short), (b_long, n_long) = shots[500], shots[60000]
    if b_short is None or b_long is None:
        rep.info("%s: CAM0 changes the exposure" % via,
                 "JPEG %d B at 0.5 ms, %d B at 60 ms (install Pillow to measure brightness)" % (n_short, n_long))
    else:
        rep.check("%s: CAM0 changes the exposure" % via,
                  b_long >= 2 * b_short and b_long - b_short >= 10,
                  "mean luma %.0f at 0.5 ms, %.0f at 60 ms (needs some light)" % (b_short, b_long))


def connect_when_free(host, port, seconds, rcvbuf=0):
    """Connect as soon as the board listens again; (link, seconds) or (None, seconds)."""
    t = time.perf_counter()
    while time.perf_counter() - t < seconds:
        try:
            return TcpLink(host, port, rcvbuf=rcvbuf), time.perf_counter() - t
        except OSError:
            time.sleep(0.25)
    return None, time.perf_counter() - t


def cmd_wifitest(args):
    rep = Report()
    host, port = args.host, args.port

    found = discover(port, args.probe_addr, 2.0)
    rep.check("discovery: the broadcast probe is answered", bool(found),
              ", ".join("%s says %s" % kv for kv in sorted(found.items()))
              or "no answer (a firewall or AP client isolation can block broadcasts)")
    direct = discover(port, host, 2.0).get(host)
    rep.check("discovery: a probe sent to the board is answered", direct is not None, direct or "no answer")
    if direct:
        rep.check("discovery: the announced address is the board's", direct == "%s:%d" % (host, port), direct)

    try:
        sess = Session(TcpLink(host, port))
    except OSError as e:
        rep.check("WiFi: connect to the host server", False, str(e))
        return rep.finish(args.json)
    rep.check("WiFi: connect to the host server", True, "%s:%d" % (host, port))
    bench = not args.any_app
    if stream_checks(sess, rep, args.min_fps, replay=True, bench=bench):
        control_checks(sess, rep, fresh_session=True, bench=bench)

    # Commands that need the cable must do nothing over Wi-Fi. The asset ones are
    # harmless if accepted; REBOOT_DFU is sent only on request, as a board that
    # obeyed it would sit in the bootloader.
    sess.send(struct.pack("<4sIII", b"AST0", 0, 0, 0))
    sess.send(struct.pack("<4sHB", b"AST1", 0, 4) + b"\0\0\0\0")
    sess.send(b"AST2")
    if args.try_dfu:
        sess.send(b"REBOOT_DFU")
    j = sess.st.jpeg
    ok = sess.pump(5) and sess.st.jpeg > j
    rep.check("WiFi: USB-only commands are ignored", ok,
              "stream kept going; REBOOT_DFU %s" % ("sent" if args.try_dfu else "not sent (use --try-dfu)"))

    # A second host is refused while this one is attached.
    try:
        second = TcpLink(host, port)
        other = Session(second)
        got = other.pump(3, lambda s: s.st.frames > 0)
        other.close()
        verdict = "connected and got frames" if got else "connected, no frames"
    except OSError as e:
        got, verdict = False, "refused (%s)" % e.__class__.__name__
    rep.check("WiFi: a second host is refused", not got, verdict)

    # Something that is not a command drops the host, and a new one gets a fresh session.
    try:
        sess.link.send_raw(b"NOT A NEURO HOST CMD")
    except ConnectionError:
        pass
    sess.pump(3)
    rep.check("WiFi: a malformed command drops the host", sess.closed, sess.close_reason or "still connected")
    sess.close()
    link, took = connect_when_free(host, port, 5)
    if link is None:
        rep.check("WiFi: reconnect after a drop", False, "no listener within 5 s")
    else:
        again = Session(link)
        ok = again.pump(5, lambda s: s.st.jpeg >= 1 and s.saw("[bootlog]"))
        rep.check("WiFi: reconnect after a drop", ok,
                  "listening again after %.2f s; replay %s" % (took, "seen" if again.saw("[bootlog]") else "missing"))
        again.close()

    # A host that stops reading is dropped, and the next one gets the stream.
    stalled, _ = connect_when_free(host, port, 5, rcvbuf=4096)
    if stalled is None:
        rep.check("WiFi: a host that stops reading is dropped", False, "could not connect")
    else:
        time.sleep(1.0)   # let its window fill
        link, took = connect_when_free(host, port, 30)
        ok = False
        if link is not None:
            nxt = Session(link)
            ok = nxt.pump(5, lambda s: s.st.jpeg >= 1)
            nxt.close()
        rep.check("WiFi: a host that stops reading is dropped", ok and took <= 20,
                  "the next host got the stream after %.1f s" % (took + 1.0))
        stalled.close()

    # Connect/disconnect soak.
    worst, done = 0.0, 0
    for _ in range(args.soak):
        link, _took = connect_when_free(host, port, 5)
        if link is None:
            break
        s = Session(link)
        t = time.perf_counter()
        ok = s.pump(5, lambda x: x.st.jpeg >= 3)
        s.close()
        if not ok:
            break
        worst = max(worst, time.perf_counter() - t)
        done += 1
    rep.check("WiFi: connect/disconnect soak", done == args.soak,
              "%d/%d cycles, slowest 3 frames in %.2f s" % (done, args.soak, worst))

    if args.usb:
        handover_checks(host, port, rep)
    else:
        rep.skip("hand-over between USB and WiFi", "run with --usb, with the board's cable on this PC")
    return rep.finish(args.json)


def handover_checks(host, port, rep):
    try:
        usb = Session(UsbLink())
    except (Exception, SystemExit) as e:  # noqa: BLE001
        rep.skip("hand-over between USB and WiFi", "no USB link: %s" % e)
        return
    ok = usb.pump(5, lambda s: s.st.jpeg >= 3)
    rep.check("hand-over: USB has the stream while no WiFi host is connected", ok, "%d frames" % usb.st.jpeg)
    link, _ = connect_when_free(host, port, 5)
    if link is None:
        rep.check("hand-over: WiFi takes the stream", False, "could not connect")
        usb.close()
        return
    wifi = Session(link)
    ok = wifi.pump(5, lambda s: s.st.jpeg >= 3)
    usb.pump(2)   # whatever was already on its way
    j = usb.st.jpeg
    usb.pump(3)
    rep.check("hand-over: WiFi takes the stream and USB stops", ok and usb.st.jpeg == j,
              "WiFi %d frames; USB %d in the last 3 s" % (wifi.st.jpeg, usb.st.jpeg - j))
    wifi.close()
    ok = usb.pump(6, lambda s: s.st.jpeg >= j + 3)
    rep.check("hand-over: the stream returns to USB", ok, "%d new USB frames" % (usb.st.jpeg - j))
    usb.close()


def cmd_usbtest(args):
    rep = Report()
    sess = Session(UsbLink())
    bench = not args.any_app
    if stream_checks(sess, rep, args.min_fps, replay=False, bench=bench):
        control_checks(sess, rep, fresh_session=False, bench=bench)
    sess.close()
    return rep.finish(args.json)


def touch_1200(port):
    import serial
    ser = serial.Serial(port, 1200, timeout=0.1)
    ser.dtr = True
    time.sleep(0.15)
    ser.dtr = False
    time.sleep(0.05)
    ser.close()


def cmd_dfu(args):
    """Get the board into the FSBL. Order: already there -> 1200-baud touch on
    a free app port -> (--kick) make the app reset via the vendor endpoint and
    grab the CDC port the instant it reappears, then touch."""
    fs = find_port(PID_FSBL)
    if fs:
        print("dfu: FSBL already on %s" % fs)
        return 0
    app = find_port(PID_APP)
    if app:
        try:
            touch_1200(app)
            print("dfu: touched %s at 1200 baud" % app)
            fs = wait_port(PID_FSBL, 15)
            if fs:
                print("dfu: FSBL on %s" % fs)
                return 0
            print("dfu: no FSBL after the touch")
        except Exception as e:  # noqa: BLE001
            print("dfu: cannot open %s (%s)" % (app, str(e)[:60]))
    if not args.kick:
        return 2
    # kick: the vendor REBOOT_DFU resets the app (it does not reach the FSBL);
    # the CDC port vanishes and comes back, and is opened again at once.
    for attempt in range(args.tries):
        dev = usb_open(required=False, wait=5)
        if dev is None:
            print("dfu: no app on the bus")
            return 2
        usb_claim(dev)
        dev.write(0x01, b"REBOOT_DFU", timeout=1000)
        print("dfu: sent REBOOT_DFU (attempt %d)" % (attempt + 1))
        t0 = time.time()
        while find_port(PID_APP) and time.time() - t0 < 10:
            time.sleep(0.02)
        gone_at = time.time()
        got = None
        while time.time() - gone_at < 20:
            fs = find_port(PID_FSBL)
            if fs:
                print("dfu: board came back as FSBL on %s after %.1fs" % (fs, time.time() - gone_at))
                return 0
            app = find_port(PID_APP)
            if app:
                try:
                    import serial
                    ser = serial.Serial(app, 1200, timeout=0.1)
                    got = (ser, app)
                    break
                except Exception:  # noqa: BLE001
                    time.sleep(0.005)
                    continue
            time.sleep(0.005)
        if got is None:
            print("dfu: could not grab the app port after the reset")
            continue
        ser, app = got
        print("dfu: grabbed %s %.1fs after the reset; touching" % (app, time.time() - gone_at))
        try:
            ser.dtr = True
            time.sleep(1.5)      # let the app finish booting so the line-state hook is live
            ser.dtr = False
            time.sleep(0.05)
        finally:
            ser.close()
        fs = wait_port(PID_FSBL, 15)
        if fs:
            print("dfu: FSBL on %s" % fs)
            return 0
        print("dfu: touch did not land; retrying")
    return 2


# Stored Wi-Fi settings: a record in the board's asset store (USB only).
REC_WIFI = 2
SECURITY = {"wpa2": 3, "wpa3": 5, "wpa2-wpa3": 6}


def wifi_record(mode, ssid, key=b"", security=None, channel=None, hostname="", link_key=b""):
    """The record MayaW476_BeginStored() reads: TLV, N6_WIFI_TLV_* in n6_host_contract.h.
    mode 1 joins the network, 2 hosts it."""
    def tlv(tag, value):
        return bytes([tag, len(value)]) + value
    out = tlv(1, bytes([mode])) + tlv(2, ssid.encode())
    if key:
        out += tlv(3, key)
    if security is not None:
        out += tlv(4, bytes([security]))
    if channel is not None:
        out += tlv(5, bytes([channel]))
    if hostname:
        out += tlv(6, hostname.encode())
    if link_key:
        out += tlv(7, link_key)
    return out


def record_cmds(rtype, payload, chunk=48):
    """One record transfer as commands: ASR0, the AST1 chunks, AST2."""
    cmds = [struct.pack("<4sIIB3x", b"ASR0", len(payload), zlib.crc32(payload) & 0xFFFFFFFF, rtype)]
    for off in range(0, len(payload), chunk):
        part = payload[off:off + chunk]
        cmds.append(struct.pack("<4sHB", b"AST1", off, len(part)) + part)
    cmds.append(b"AST2")
    return cmds


def wait_reset(timeout=25.0, leave_within=4.0):
    """True once the board has left the bus and come back: storing a record
    resets it so the bootloader can write flash."""
    import usb.core
    import libusb_package
    backend = libusb_package.get_libusb1_backend()
    t0, left = time.time(), False
    while time.time() - t0 < timeout:
        here = usb.core.find(backend=backend, idVendor=VID, idProduct=PID_APP) is not None
        if not here:
            left = True
        elif left:
            return True
        elif time.time() - t0 > leave_within:
            return False
        time.sleep(0.2)
    return False


def store_record(rtype, payload):
    link = UsbLink()
    try:
        for c in record_cmds(rtype, payload):
            link.send(c)
            time.sleep(0.02)
    finally:
        link.close()
    return wait_reset()


def env_secret(var, what, lo, hi):
    import os
    v = os.environ.get(var, "")
    if not v:
        raise SystemExit("wifiset: set the environment variable %s to the %s" % (var, what))
    b = v.encode()
    if not lo <= len(b) <= hi:
        raise SystemExit("wifiset: the %s in %s must be %d to %d characters" % (what, var, lo, hi))
    return b


def cmd_wifiset(args):
    import re
    if args.clear:
        payload, what = b"", "removing the stored WiFi settings"
    else:
        ssid = args.sta or args.ap
        if not ssid or (args.sta and args.ap):
            raise SystemExit("wifiset: give one of --sta SSID, --ap SSID or --clear")
        if not 1 <= len(ssid.encode()) <= 32:
            raise SystemExit("wifiset: an SSID is 1 to 32 bytes")
        if args.hostname and not re.fullmatch(r"[A-Za-z0-9-]{1,32}", args.hostname):
            raise SystemExit("wifiset: a hostname is letters, digits and hyphens, up to 32")
        if args.channel is not None and not (args.ap and 0 <= args.channel <= 11):
            raise SystemExit("wifiset: --channel is 0 (auto) to 11, and only with --ap")
        if args.board_password and not args.ap:
            raise SystemExit("wifiset: --board-password goes with --ap")
        security = SECURITY[args.security] if args.security else None
        key = b"" if args.board_password else env_secret(args.key_env, "network passphrase", 8, 63)
        link_key = env_secret(args.link_key_env, "link key", 8, 64) if args.link_key_env else b""
        payload = wifi_record(2 if args.ap else 1, ssid, key, security, args.channel, args.hostname, link_key)
        what = "%s \"%s\"%s%s%s" % ("hosting" if args.ap else "joining", ssid,
                                     ", with the board's own password" if not key else "",
                                     ", hostname %s" % args.hostname if args.hostname else "",
                                     ", with a link key" if link_key else "")
    print("wifiset: %s; the board restarts to write it" % what)
    if not store_record(REC_WIFI, payload):
        print("wifiset: FAILED - the board did not restart. Its firmware may not use the WiFi "
              "library, or it is running the bootloader.")
        return 1
    print("wifiset: done - the board restarted")
    return 0


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ls")
    c = sub.add_parser("console")
    c.add_argument("--port", default="")
    c.add_argument("--seconds", type=float, default=10.0)
    c.add_argument("--wait", type=float, default=15.0)
    c.add_argument("--send", action="append")
    c.add_argument("--send-at", action="append", type=float)
    c.add_argument("--log", default="")
    c.add_argument("--grep", action="append")
    s = sub.add_parser("stream")
    s.add_argument("--tcp", default="", help="HOST[:PORT] of a board's WiFi host server instead of USB")
    s.add_argument("--seconds", type=float, default=10.0)
    s.add_argument("--log", default="")
    s.add_argument("--quiet", action="store_true")
    s.add_argument("--debug", type=int, default=0)
    s.add_argument("--tun", action="append", help="id:value TUN0 packet to send (repeatable)")
    s.add_argument("--tun-at", action="append", type=float)
    s.add_argument("--raw", action="append", help="hex or text to send as one command (repeatable)")
    s.add_argument("--raw-at", action="append", type=float)
    m = sub.add_parser("cmd")
    m.add_argument("--tcp", default="", help="HOST[:PORT] of a board's WiFi host server instead of USB")
    m.add_argument("--hex", default="")
    m.add_argument("--text", default="")
    sub.add_parser("desc")
    p = sub.add_parser("probe")
    p.add_argument("--addr", default="255.255.255.255", help="where to send the probe")
    p.add_argument("--port", type=int, default=NET_PORT)
    p.add_argument("--timeout", type=float, default=2.0)
    w = sub.add_parser("wifitest")
    w.add_argument("--host", default="192.168.10.1", help="the board's address (SoftAP default 192.168.10.1)")
    w.add_argument("--port", type=int, default=NET_PORT)
    w.add_argument("--probe-addr", default="255.255.255.255")
    w.add_argument("--min-fps", type=float, default=5.0)
    w.add_argument("--soak", type=int, default=20, help="connect/disconnect cycles")
    w.add_argument("--usb", action="store_true", help="also check the USB <-> WiFi hand-over")
    w.add_argument("--try-dfu", action="store_true",
                   help="also send REBOOT_DFU over WiFi (a correct board ignores it)")
    w.add_argument("--any-app", action="store_true",
                   help="only the checks any WiFi application passes, not WiFiTwoWay's own")
    w.add_argument("--json", default="")
    u = sub.add_parser("usbtest")
    u.add_argument("--min-fps", type=float, default=5.0)
    u.add_argument("--any-app", action="store_true",
                   help="only the checks any application passes, not WiFiTwoWay's own")
    u.add_argument("--json", default="")
    ws = sub.add_parser("wifiset")
    ws.add_argument("--sta", default="", help="store this network to join")
    ws.add_argument("--ap", default="", help="store this network to host")
    ws.add_argument("--board-password", action="store_true",
                    help="--ap only: no key, so the board hosts with its own password")
    ws.add_argument("--key-env", default="N6_WIFI_KEY", help="environment variable holding the passphrase")
    ws.add_argument("--security", choices=sorted(SECURITY), help="default: wpa2")
    ws.add_argument("--channel", type=int, help="hosting channel, 1 to 11 or 0 (auto); default 6")
    ws.add_argument("--hostname", default="", help="the DHCP and mDNS name")
    ws.add_argument("--link-key-env", default="",
                    help="environment variable holding a link key hosts must prove they know")
    ws.add_argument("--clear", action="store_true", help="remove the stored settings")
    d = sub.add_parser("dfu")
    d.add_argument("--kick", action="store_true", help="reset the app via the vendor endpoint and grab the port")
    d.add_argument("--tries", type=int, default=3)
    args = ap.parse_args()
    return {"ls": cmd_ls, "console": cmd_console, "stream": cmd_stream, "cmd": cmd_cmd, "desc": cmd_desc,
            "probe": cmd_probe, "wifitest": cmd_wifitest, "usbtest": cmd_usbtest,
            "dfu": cmd_dfu, "wifiset": cmd_wifiset}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main() or 0)
