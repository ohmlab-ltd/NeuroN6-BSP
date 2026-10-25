#!/usr/bin/env python3
"""Check the Wi-Fi API on a board running Examples/WiFiApi_FreeRTOS (Windows).

    python harness/bench/n6wifiapi.py --port PORT [--phases pass,ap,sta,stored,link,wpa3]
                                      [--no-usb-store] [--json out.json]

pass   the board's own SoftAP password: removed, made, stored, kept across a
       restart, and an access point hosted with it (it replaces the board's
       password)
ap     refused security settings, then an access point this PC joins:
       addressing, scanning, DNS, NTP, TCP client and server, UDP,
       MayaW476_Config(), MayaW476_Disconnect() and MayaW476_End()
sta    the board as a station of this PC's Wi-Fi Direct group owner (n6radio
       wfd-ap), which must be on 2.4 GHz; or name a 2.4 GHz network in
       N6_STA_SSID and N6_STA_KEY (the connection-lost check is then skipped)
stored settings stored over USB (n6bench.py wifiset), then
       MayaW476_BeginStored(); the board restarts at each write
link   the link key: challenge, refusals, and the stream with the right key
wpa3   WPA3-Personal access points (WPA3 alone, and with WPA2), and the board
       as a WPA2/WPA3 and a WPA3 station

--no-usb-store leaves out the stored phase and the stored parts of link and
wpa3. The servers here run in python.exe, which the firewall must let in on
Public networks. No passphrase is printed, logged or kept.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT

import argparse
import hashlib
import json
import os
import re
import secrets
import socket
import string
import struct
import subprocess
import sys
import tempfile
import threading
import time

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import build_n6radio  # noqa: E402
import n6wlanscan  # noqa: E402

IFACE = "WiFi"
NO_USB_STORE = False   # --no-usb-store
TCP_ECHO, HTTP, UDP_SINK, BOARD_SRV, BOARD_UDP, MCAST_PORT = 5601, 5602, 5603, 5605, 5606, 5607
MCAST_GROUP = "239.255.0.7"
DNS_NAME, DNS_ANSWER = "bench.n6.test", "10.99.0.7"


def random_key(n=20):
    return "".join(secrets.choice(string.ascii_letters + string.digits) for _ in range(n))


# The board's shell.

class Shell:
    def __init__(self, port, log_path, prefix="[api]"):
        self.port = port
        self.prefix = prefix
        self.s = serial.Serial(port, 115200, timeout=0.05)
        self.log = open(log_path, "w", encoding="utf-8")
        self.lines = []
        self.buf = b""
        self.secrets = []

    def hide(self, text):
        text = re.sub(r"\bpass key=\S+", "pass key=<key>", text)
        text = re.sub(r'^(\[api\] net \d+ )ssid="[^"]*"', r"\1ssid=<hidden>", text)
        for k in self.secrets:
            text = text.replace(k, "<key>")
        return text

    def pump(self):
        data = self.s.read(8192)
        if not data:
            return
        self.buf += data
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            line = raw.decode("utf-8", errors="replace").rstrip("\r")
            self.lines.append(line)
            self.log.write("%.3f %s\n" % (time.time(), self.hide(line)))
        self.log.flush()

    def cmd(self, text, timeout=30.0):
        name = text.split()[0]
        self.log.write(">> %s\n" % self.hide(text))
        start = len(self.lines)
        self.s.write((text + "\n").encode())
        deadline = time.time() + timeout
        end = re.compile(r"^%s (ok|fail) %s\b" % (re.escape(self.prefix), re.escape(name)))
        while time.time() < deadline:
            self.pump()
            for i in range(start, len(self.lines)):
                m = end.match(self.lines[i])
                if m:
                    lead = self.prefix + " "
                    api = [l[len(lead):] for l in self.lines[start:i + 1]
                           if l.startswith(lead) and not l.startswith(lead + "beat ")]
                    return m.group(1) == "ok", api
            time.sleep(0.02)
        return False, ["timeout after %.0f s" % timeout]

    def wait_for(self, pattern, timeout, since=None):
        rx = re.compile(pattern)
        start = len(self.lines) if since is None else since
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.pump()
            for l in self.lines[start:]:
                if rx.search(l):
                    return l
            time.sleep(0.05)
        return None

    def mark(self):
        self.pump()
        return len(self.lines)

    def reopen(self, timeout=40.0):
        """The board restarted: its port comes back under the same name."""
        try:
            self.s.close()
        except serial.SerialException:
            pass
        self.log.write("-- board restart --\n")
        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(1.0)
            try:
                self.s = serial.Serial(self.port, 115200, timeout=0.05)
                self.buf = b""
                return True
            except serial.SerialException:
                pass
        return False

    def close(self):
        self.s.close()
        self.log.close()


def field(api_lines, key):
    for l in api_lines:
        m = re.search(r"\b%s=(\"[^\"]*\"|\S+)" % re.escape(key), l)
        if m:
            return m.group(1).strip('"')
    return None


def wait_restart(port, timeout=40.0):
    """The board restarted: its port left the list and came back."""
    from serial.tools import list_ports
    t0, left = time.time(), False
    while time.time() - t0 < timeout:
        here = any(p.device == port for p in list_ports.comports())
        if not here:
            left = True
        elif left:
            return True
        elif time.time() - t0 > 6.0:
            return False
        time.sleep(0.2)
    return False


def restart(sh, cmd):
    """A shell command that restarts the board, then its port reopened."""
    ok = sh.cmd(cmd, 20)[0]
    sh.s.close()
    return wait_restart(sh.port) and sh.reopen() and ok


def board_password(sh):
    """The board's own password, stored first when there is none."""
    for _ in range(2):
        ok, api = sh.cmd("pass", 20)
        if ok and "ok pass storing" in api:
            sh.s.close()
            if not (wait_restart(sh.port) and sh.reopen()):
                return None
            continue
        key = field(api, "key") if ok else None
        if key:
            sh.secrets.append(key)
        return key
    return None


def short_hash(key):
    return hashlib.sha256(key.encode()).hexdigest()[:8] if key else None


def server_answers(board_ip, pc_ip, name):
    """Which parts of the Neuro host server answer: TCP 5000, discovery on UDP
    5000, mDNS."""
    import n6bench
    try:
        socket.create_connection((board_ip, 5000), timeout=3).close()
        tcp = True
    except OSError:
        tcp = False
    details = {}
    n6bench.discover(5000, board_ip.rsplit(".", 1)[0] + ".255", 2.0, details)
    mdns = [v for (_, t, v) in mdns_query(name + ".local", pc_ip) if t == 1]
    return {"tcp": tcp, "udp": board_ip in details, "mdns": bool(mdns)}


def wait_server(board_ip, timeout=10.0):
    """Seconds until the host server answers discovery, or None. It comes up
    a moment after MayaW476_Start() returns, which only starts its task."""
    import n6bench
    t0 = time.time()
    while time.time() - t0 < timeout:
        if board_ip in n6bench.discover(5000, board_ip.rsplit(".", 1)[0] + ".255", 1.0):
            return round(time.time() - t0, 1)
    return None


def refused(sh, cmd, notice):
    """A command MayaW476 refuses, with exactly one line naming why."""
    m = sh.mark()
    ok, api = sh.cmd(cmd, 30)
    said = [l for l in sh.lines[m:] if l.startswith("[MayaW476]") and notice in l]
    return (not ok) and len(said) == 1, "; ".join(api[-1:] + said[:1])


# This PC's Wi-Fi.

PROFILE = """<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
  <name>{name}</name>
  <SSIDConfig><SSID><name>{ssid}</name></SSID></SSIDConfig>
  <connectionType>ESS</connectionType>
  <connectionMode>manual</connectionMode>
  <MSM><security>
    <authEncryption><authentication>{auth}</authentication><encryption>{enc}</encryption><useOneX>false</useOneX></authEncryption>
    {key}
  </security></MSM>
</WLANProfile>
"""


def netsh(*args):
    return subprocess.run(["netsh", "wlan"] + list(args), capture_output=True, text=True).stdout


def wlan_state():
    out = netsh("show", "interfaces")
    state = re.search(r"^\s*State\s*:\s*(\S+)", out, re.M)
    ssid = re.search(r"^\s*SSID\s*:\s*(.+)$", out, re.M)
    bssid = re.search(r"^\s*(?:AP )?BSSID\s*:\s*(\S+)", out, re.M)
    return (state.group(1) if state else "", ssid.group(1).strip() if ssid else "",
            bssid.group(1).upper() if bssid else "")


def wlan_auth():
    """This PC's link as Windows reports it: (authentication, cipher)."""
    out = netsh("show", "interfaces")
    a = re.search(r"^\s*Authentication\s*:\s*(.+?)\s*$", out, re.M)
    c = re.search(r"^\s*Cipher\s*:\s*(.+?)\s*$", out, re.M)
    return (a.group(1) if a else ""), (c.group(1) if c else "")


def scanned_auth(ssid, tries=4):
    """The authentication this PC's own scan reports for `ssid`, or None."""
    for _ in range(tries):
        n6wlanscan.scan_now()
        out = netsh("show", "networks", "mode=bssid")
        for block in re.split(r"^SSID \d+ : ", out, flags=re.M)[1:]:
            if block.split("\n", 1)[0].strip() == ssid:
                m = re.search(r"^\s*Authentication\s*:\s*(.+?)\s*$", block, re.M)
                return m.group(1) if m else ""
    return None


def wait_left(ssid, timeout=20):
    """Seconds until this PC no longer reports being joined to `ssid`, or None."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        state, cur, _ = wlan_state()
        if state != "connected" or cur != ssid:
            return round(time.time() - t0, 1)
        time.sleep(0.5)
    return None


def local_ip_towards(ip):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((ip, 9))
        a = s.getsockname()[0]
        s.close()
        return a
    except OSError:
        return None


class Join:
    """A temporary manual-connect profile for one network, removed on leave().
    `auth` is the profile's authentication: WPA2PSK by default with a key,
    WPA3SAE for WPA3-Personal."""

    def __init__(self, ssid, key, auth=None):
        self.ssid, self.key = ssid, key
        self.auth = auth or ("WPA2PSK" if key else "open")
        self.name = "n6bench-" + ssid

    def join(self, board_ip, timeout=40):
        key = ('<sharedKey><keyType>passPhrase</keyType><protected>false</protected>'
               '<keyMaterial>%s</keyMaterial></sharedKey>' % self.key) if self.key else ""
        xml = PROFILE.format(name=self.name, ssid=self.ssid, auth=self.auth,
                             enc="AES" if self.key else "none", key=key)
        fd, path = tempfile.mkstemp(suffix=".xml")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                f.write(xml)
            netsh("add", "profile", "filename=%s" % path, "interface=%s" % IFACE, "user=current")
        finally:
            os.remove(path)
        deadline = time.time() + timeout
        while time.time() < deadline:
            netsh("connect", "name=%s" % self.name, "interface=%s" % IFACE)
            for _ in range(20):
                time.sleep(0.5)
                state, ssid, _ = wlan_state()
                mine = local_ip_towards(board_ip)
                if state == "connected" and ssid == self.ssid and mine and \
                        mine.rsplit(".", 1)[0] == board_ip.rsplit(".", 1)[0]:
                    return mine
        return None

    def leave(self):
        netsh("disconnect", "interface=%s" % IFACE)
        netsh("delete", "profile", "name=%s" % self.name, "interface=%s" % IFACE)


# Servers on this PC.

class Servers:
    def __init__(self):
        self.udp_rx = []
        self.stop = False
        self.threads = []
        self.dns_sock = None
        self.ntp_error = None

    def start(self):
        for target in (self.tcp_echo, self.http, self.udp_sink, self.ntp):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self.threads.append(t)

    def dns_on(self, ip):
        """DNS answers only on this PC's address on the board's network, which
        is a different one in each phase."""
        if self.dns_sock is not None:
            if self.dns_sock.getsockname()[0] == ip:
                return True
            self.dns_sock.close()   # its thread ends on the error
            self.dns_sock = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind((ip, 53))
        except OSError as e:
            print("  (no DNS server here: %s)" % e)
            return False
        self.dns_sock = s
        threading.Thread(target=self.dns, args=(s,), daemon=True).start()
        return True

    def tcp_echo(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", TCP_ECHO))
        srv.listen(4)
        while not self.stop:
            c, _ = srv.accept()
            threading.Thread(target=self._echo, args=(c,), daemon=True).start()

    @staticmethod
    def _echo(c):
        try:
            while True:
                d = c.recv(65536)
                if not d:
                    break
                c.sendall(d)
        except OSError:
            pass
        c.close()

    def http(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", HTTP))
        srv.listen(4)
        body = bytes((i * 13) & 0xFF for i in range(5000))
        while not self.stop:
            c, _ = srv.accept()
            try:
                c.settimeout(5)
                req = b""
                while b"\r\n\r\n" not in req:
                    d = c.recv(4096)
                    if not d:
                        break
                    req += d
                c.sendall(b"HTTP/1.0 200 OK\r\nContent-Type: application/octet-stream\r\n"
                          b"Content-Length: %d\r\n\r\n" % len(body) + body)
            except OSError:
                pass
            c.close()

    def udp_sink(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("0.0.0.0", UDP_SINK))
        while not self.stop:
            d, a = s.recvfrom(2048)
            self.udp_rx.append((a, d))

    def ntp(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind(("0.0.0.0", 123))
        except OSError as e:
            # Windows' time service holds the port once it has started, which
            # it does on demand; then no server here can answer.
            self.ntp_error = "UDP port 123 on this PC is taken (%s)" % e
            s.close()
            return
        while not self.stop:
            try:
                d, a = s.recvfrom(512)
            except ConnectionResetError:
                continue   # Windows reports an earlier reply's ICMP port-unreachable here
            if len(d) < 48 or (d[0] & 7) != 3:
                continue
            now = time.time() + 2208988800
            ts = struct.pack(">II", int(now), int((now % 1) * 2 ** 32))
            reply = bytes([0x24, 2, 6, 0xEC]) + b"\0" * 8 + b"LOCL" + ts + d[40:48] + ts + ts
            s.sendto(reply, a)

    @staticmethod
    def dns(s):
        while True:
            try:
                d, a = s.recvfrom(512)
            except OSError:
                return
            if len(d) < 17:
                continue
            p, labels = 12, []
            while p < len(d) and d[p]:
                labels.append(d[p + 1:p + 1 + d[p]].decode(errors="replace"))
                p += 1 + d[p]
            qname, question = ".".join(labels).lower(), d[12:p + 5]
            if qname == DNS_NAME and d[p + 1:p + 3] == b"\0\1":
                ans = b"\xc0\x0c\0\1\0\1\0\0\0\x3c\0\x04" + socket.inet_aton(DNS_ANSWER)
                s.sendto(d[:2] + b"\x81\x80\0\1\0\1\0\0\0\0" + question + ans, a)
            else:
                s.sendto(d[:2] + b"\x81\x83\0\1\0\0\0\0\0\0" + question, a)


# mDNS, asked directly.

def _dns_name(data, off):
    """Read a possibly compressed DNS name; return (name, offset after it)."""
    labels, jumped, end = [], False, off
    for _ in range(64):
        n = data[off]
        if n == 0:
            off += 1
            break
        if n & 0xC0 == 0xC0:
            if not jumped:
                end = off + 2
            jumped, off = True, ((n & 0x3F) << 8) | data[off + 1]
            continue
        labels.append(data[off + 1:off + 1 + n].decode(errors="replace"))
        off += 1 + n
    return ".".join(labels), (end if jumped else off)


def mdns_query(name, from_ip, qtype=1, timeout=2.0):
    """One-shot (legacy unicast) mDNS query from an ephemeral port on this PC's
    address `from_ip`. Returns the answers as (name, type, value) tuples, where
    value is the address for A, the target for PTR, (port, target) for SRV."""
    qid = secrets.randbelow(65536)
    q = struct.pack(">HHHHHH", qid, 0, 1, 0, 0, 0)
    for label in name.split("."):
        q += bytes([len(label)]) + label.encode()
    q += b"\0" + struct.pack(">HH", qtype, 1)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((from_ip, 0))
    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(from_ip))
    s.settimeout(timeout)
    answers = []
    try:
        s.sendto(q, ("224.0.0.251", 5353))
        data, _ = s.recvfrom(2048)
        qd, an = struct.unpack(">HH", data[4:8])
        off = 12
        for _ in range(qd):
            _, off = _dns_name(data, off)
            off += 4
        for _ in range(an):
            rname, off = _dns_name(data, off)
            rtype, _, _, rdlen = struct.unpack(">HHIH", data[off:off + 10])
            off += 10
            rd = data[off:off + rdlen]
            if rtype == 1:
                value = socket.inet_ntoa(rd)
            elif rtype == 12:
                value = _dns_name(data, off)[0]
            elif rtype == 33:
                value = (struct.unpack(">H", rd[4:6])[0], _dns_name(data, off + 6)[0])
            else:
                value = rd
            answers.append((rname, rtype, value))
            off += rdlen
    except (socket.timeout, OSError, struct.error, IndexError):
        pass
    finally:
        s.close()
    return answers


def discovery_checks(sh, rep, board_ip, pc_ip, name, label):
    import n6bench
    details = {}
    bcast = board_ip.rsplit(".", 1)[0] + ".255"
    n6bench.discover(5000, bcast, 2.0, details)
    got = details.get(board_ip, (None, False))
    rep.check("%s: the discovery reply carries the board's name" % label, got[0] == name, got)

    a = [v for (_, t, v) in mdns_query(name + ".local", pc_ip) if t == 1]
    rep.check("%s: mDNS answers %s.local" % (label, name), a == [board_ip], a)

    ptr = mdns_query("_neuron6._tcp.local", pc_ip, qtype=12)
    srv = [v for (_, t, v) in ptr if t == 33]
    rep.check("%s: DNS-SD lists the host server as _neuron6._tcp" % label,
              any(v == "%s._neuron6._tcp.local" % name for (_, t, v) in ptr if t == 12) and
              srv and srv[0][0] == 5000, [(t, v) for (_, t, v) in ptr])

    try:
        win = socket.getaddrinfo(name + ".local", 5000, socket.AF_INET)[0][4][0]
    except OSError as e:
        win = str(e)
    rep.check("%s: Windows itself resolves %s.local" % (label, name), win == board_ip, win)


# Report.

class Report:
    def __init__(self):
        self.rows = []

    def check(self, name, ok, detail=""):
        self.rows.append({"check": name, "result": "PASS" if ok else "FAIL", "detail": str(detail)})
        print("%s  %s%s" % ("PASS" if ok else "FAIL", name, ("  -- " + str(detail)) if detail else ""))
        return ok

    def info(self, name, detail):
        self.rows.append({"check": name, "result": "INFO", "detail": str(detail)})
        print("INFO  %s  -- %s" % (name, detail))

    def finish(self, path):
        failed = [r for r in self.rows if r["result"] == "FAIL"]
        passed = [r for r in self.rows if r["result"] == "PASS"]
        print("\n%d passed, %d failed" % (len(passed), len(failed)))
        if path:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.rows, f, indent=1)
        return 1 if failed else 0


# Checks shared by both phases.

def client_checks(sh, rep, pc_ip, board_ip, label):
    ok, api = sh.cmd("tcp %s %d 200000" % (pc_ip, TCP_ECHO), 60)
    rep.check("%s: a TCP client echoes 200 kB through a server here" % label, ok,
              "; ".join(a for a in api if a.startswith("tcp ")))

    ok, api = sh.cmd("get %s %d /n6" % (pc_ip, HTTP), 30)
    rep.check("%s: a TCP client's HTTP GET" % label, ok and field(api, "body") == "5000",
              "; ".join(a for a in api if a.startswith("get ")))

    n0 = len(servers.udp_rx)
    ok, _ = sh.cmd("usend %s %d from-the-board" % (pc_ip, UDP_SINK))
    time.sleep(1.0)
    got = [d for a, d in servers.udp_rx[n0:] if a[0] == board_ip]
    rep.check("%s: UDP sends a datagram here" % label, ok and b"from-the-board" in got, got[:1])

    n0 = len(servers.udp_rx)
    ok, _ = sh.cmd("usend 255.255.255.255 %d board-broadcast" % UDP_SINK)
    time.sleep(1.0)
    got = [d for a, d in servers.udp_rx[n0:] if a[0] == board_ip]
    rep.check("%s: UDP broadcasts to 255.255.255.255" % label, ok and b"board-broadcast" in got, got[:1])

    ok, _ = sh.cmd("srv %d" % BOARD_SRV)
    good = False
    detail = "server did not start"
    if ok:
        try:
            c = socket.create_connection((board_ip, BOARD_SRV), timeout=5)
            c.settimeout(5)
            data = bytes((i * 31) & 0xFF for i in range(20000))
            back = b""
            for i in range(0, len(data), 1000):
                c.sendall(data[i:i + 1000])
                while len(back) < i + 1000:
                    d = c.recv(4096)
                    if not d:
                        break
                    back += d
            c.close()
            good, detail = back == data, "%d of %d bytes echoed" % (len(back), len(data))
        except OSError as e:
            detail = str(e)
    rep.check("%s: a TCP server echoes 20 kB from a client here" % label, good, detail)

    # A client resets while the board's echo is held up by a full window.
    # lwIP reports a reset once, so a loop asking SO_ERROR after it would
    # wait for ever.
    if ok:
        detail = ""
        try:
            c = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            c.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2048)
            c.settimeout(5)
            c.connect((board_ip, BOARD_SRV))
            c.settimeout(0.5)
            t0 = time.time()
            while time.time() - t0 < 8:
                try:
                    c.send(bytes(1400))
                except (socket.timeout, BlockingIOError):
                    break
            c.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
            c.close()
        except OSError as e:
            detail = str(e)
        time.sleep(1.0)
        t0 = time.time()
        alive, _ = sh.cmd("stat", 10)
        rep.check("%s: a client that resets mid-echo leaves the shell answering" % label, alive,
                  detail or "stat in %.1f s" % (time.time() - t0))

    # A client resets a quiet connection: only the server's next receive
    # meets it, and it must let the client go and take the next one.
    if ok:
        good, detail = False, ""
        try:
            c = socket.create_connection((board_ip, BOARD_SRV), timeout=5)
            time.sleep(0.5)
            c.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
            c.close()
            time.sleep(1.0)
            c = socket.create_connection((board_ip, BOARD_SRV), timeout=5)
            c.settimeout(5)
            c.sendall(b"after-a-reset")
            back = b""
            while len(back) < len(b"after-a-reset"):
                d = c.recv(64)
                if not d:
                    break
                back += d
            c.close()
            good, detail = back == b"after-a-reset", "%r echoed" % back
        except OSError as e:
            detail = str(e)
        rep.check("%s: after a client resets a quiet connection, the server echoes the next"
                  % label, good, detail)

    ok, _ = sh.cmd("udp %d" % BOARD_UDP)
    good, detail = False, "no socket"
    if ok:
        u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        u.settimeout(2)
        sizes, echoed = (1, 100, 1000, 1472), 0
        for n in sizes:
            payload = bytes((i + n) & 0xFF for i in range(n))
            for _ in range(3):
                u.sendto(payload, (board_ip, BOARD_UDP))
                try:
                    d, _ = u.recvfrom(2048)
                    if d == payload:
                        echoed += 1
                        break
                except socket.timeout:
                    pass
        u.close()
        good, detail = echoed == len(sizes), "%d of %d sizes (1..1472 bytes) echoed" % (echoed, len(sizes))
    rep.check("%s: UDP echoes datagrams from here" % label, good, detail)

    ok, _ = sh.cmd("mcast %s %d" % (MCAST_GROUP, MCAST_PORT))
    mark = sh.mark()
    m = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    m.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(pc_ip))
    m.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
    line = None
    for _ in range(5):
        m.sendto(b"n6-multicast", (MCAST_GROUP, MCAST_PORT))
        line = sh.wait_for(r"\[api\] mcast .*n6-multicast", 1.0, since=mark)
        if line:
            break
    m.close()
    rep.check("%s: UDP joins a multicast group" % label, ok and line is not None, line or "nothing heard")

    if servers.ntp_error:
        rep.info("%s: getTime from an NTP server here" % label, "not testable here: " + servers.ntp_error)
    else:
        ok, api = sh.cmd("time %s" % pc_ip, 20)
        epoch = int(field(api, "epoch") or 0)
        rep.check("%s: getTime from an NTP server here" % label, ok and abs(epoch - time.time()) < 3,
                  "board %d, PC %d" % (epoch, int(time.time())))

    ok, api = sh.cmd("resolve 10.1.2.3")
    rep.check("%s: hostByName takes a dotted quad" % label, ok and "-> 10.1.2.3" in " ".join(api), api[:1])

    if servers.dns_on(pc_ip):
        sh.cmd("dns %s" % pc_ip)
        ok, api = sh.cmd("resolve %s" % DNS_NAME, 20)
        rep.check("%s: hostByName asks the DNS server setDNS named" % label,
                  ok and ("-> %s" % DNS_ANSWER) in " ".join(api), api[:1])
        ok, api = sh.cmd("resolve nothing.n6.test", 20)
        rep.check("%s: hostByName reports a name that does not exist" % label, not ok, api[:1])


# Phases.

PASSWORD = re.compile(r"[a-kmnp-z2-9]{4}(-[a-kmnp-z2-9]{4}){3}")


def phase_pass(sh, rep):
    def appass():
        ok, api = sh.cmd("asset", 20)
        return field(api, "appass") if ok else None

    ok = restart(sh, "pclear")
    rep.check("pass: removing the board's password restarts the board", ok and appass() == "no")
    k1 = board_password(sh)
    rep.check("pass: MayaW476_DevicePassword() stores a new one, restarting the board once",
              k1 is not None and appass() == "yes", short_hash(k1))
    rep.check("pass: four groups of four from 32 letters and digits",
              k1 is not None and PASSWORD.fullmatch(k1) is not None, "%d characters" % len(k1 or ""))
    ok = restart(sh, "reboot")
    k = board_password(sh)
    rep.check("pass: the same after a restart", ok and k1 is not None and k == k1,
              "%s, then %s" % (short_hash(k1), short_hash(k)))
    ok = restart(sh, "pclear")
    k2 = board_password(sh)
    rep.check("pass: a different one once removed, as it is random, not derived from the chip",
              ok and k2 is not None and k2 != k1, "%s, then %s" % (short_hash(k1), short_hash(k2)))

    ssid = "N6PASS-%04d" % secrets.randbelow(10000)
    ok, api = sh.cmd("ap %s -" % ssid, 60)
    j = Join(ssid, k2 or "")
    try:
        pc_ip = j.join("192.168.10.1") if ok and k2 else None
        rep.check("pass: MayaW476_BeginAP() with no key hosts with it, and this PC joins", pc_ip is not None, pc_ip)
    finally:
        j.leave()
    sh.cmd("disc")


def phase_ap(sh, rep):
    # Nothing has started the host server since this restart.
    rep.check("ap: the board restarts", restart(sh, "reboot"))
    k = random_key()
    sh.secrets.append(k)
    for sec in ("open", "wep", "wpa", "wpa-wpa2"):
        ok, why = refused(sh, "ap N6REF-0000 %s 6 %s" % (k, sec), "only WPA2 and WPA3 are supported")
        rep.check("ap: MayaW476_BeginAP() refuses %s, with one line" % sec, ok, why)
        ok, why = refused(sh, "sta N6REF-0000 %s 2000 %s" % (k, sec), "only WPA2 and WPA3 are supported")
        rep.check("ap: MayaW476_Begin() refuses %s, with one line" % sec, ok, why)
    ok, why = refused(sh, "ap N6REF-0000 %s 6" % k[:7], "the passphrase must be 8 to 63 characters")
    rep.check("ap: a passphrase under 8 characters is refused", ok, why)
    ok, why = refused(sh, "ap N6REF-0000 %s 6 wpa3" % random_key(64), "the passphrase must be 8 to 63 characters")
    rep.check("ap: 64 characters under WPA3 are refused", ok, why)
    ok, why = refused(sh, "sta N6REF-0000 - 2000", "a passphrase is needed")
    rep.check("ap: MayaW476_Begin() with no passphrase is refused", ok, why)

    ssid, key = "N6API-%04d" % secrets.randbelow(10000), random_key()
    sh.secrets.append(key)
    sh.cmd("cfg 0.0.0.0")
    ok, api = sh.cmd("ap %s %s 6" % (ssid, key), 60)
    rep.check("ap: beginAP comes up", ok, "; ".join(api[-2:]))
    if not ok:
        return
    board_ip = "192.168.10.1"
    j = Join(ssid, key)
    try:
        pc_ip = j.join(board_ip)
        rep.check("ap: this PC joins with the passphrase", pc_ip is not None, pc_ip)
        if not pc_ip:
            return
        rep.check("ap: this PC's address is leased from the pool, .10 to .99",
                  10 <= int(pc_ip.rsplit(".", 1)[1]) <= 99, pc_ip)

        _, _, pc_bssid = wlan_state()
        ok, api = sh.cmd("stat")
        st = {k: field(api, k) for k in ("status", "ip", "mask", "gw", "ssid", "ch", "bssid", "mac", "host")}
        rep.check("ap: status is WL_AP_CONNECTED with this PC joined", st["status"] == "8", st["status"])
        rep.check("ap: localIP, subnetMask, gatewayIP", st["ip"] == board_ip and st["mask"] == "255.255.255.0"
                  and st["gw"] == board_ip, "%s/%s gw %s" % (st["ip"], st["mask"], st["gw"]))
        rep.check("ap: SSID and channel", st["ssid"] == ssid and st["ch"] == "6", "%s ch %s" % (st["ssid"], st["ch"]))
        rep.check("ap: BSSID is the access point this PC joined", (st["bssid"] or "").upper() == pc_bssid,
                  "board %s, PC sees %s" % (st["bssid"], pc_bssid))
        rep.check("ap: macAddress is the access point's", st["mac"] == st["bssid"], st["mac"])
        rep.check("ap: default host name neuron6-xxxxxx", re.fullmatch(r"neuron6-[0-9a-f]{6}", st["host"] or "") is not None,
                  st["host"])

        client_checks(sh, rep, pc_ip, board_ip, "ap")
        srv = server_answers(board_ip, pc_ip, st["host"])
        rep.check("ap: no host server until MayaW476_Start()", not any(srv.values()), srv)
        ok, api = sh.cmd("wstart 1", 30)
        up = wait_server(board_ip) if ok else None
        rep.check("ap: MayaW476_Start() starts it on the access point", ok and up is not None,
                  "%s; answering after %s s" % ("; ".join(api[-2:]), up))
        discovery_checks(sh, rep, board_ip, pc_ip, st["host"], "ap")

        ok, api = sh.cmd("scan", 40)
        n = int(field(api, "n") or -1)
        nets = [field([a], "ssid") for a in api if a.startswith("net ")]
        visible = set(re.findall(r"^SSID \d+ : (.+)$", netsh("show", "networks"), re.M))
        rep.check("ap: scanNetworks while hosting", ok and n >= 1,
                  "%d networks; %d also seen by this PC" % (n, len(visible & set(nets))))
        if n >= 1:
            rep.check("ap: scan results carry RSSI, channel, encryption and BSSID",
                      all(re.search(r"rssi=-\d+ enc=\d+ ch=\d+ bssid=[0-9A-F:]{17}", a) for a in api if a.startswith("net ")),
                      re.sub(r'ssid="[^"]*"', "ssid=<hidden>", next((a for a in api if a.startswith("net ")), "")))
    finally:
        j.leave()

    # config(): the access point on another subnet, then back
    ssid2, key2 = "N6API-%04d" % secrets.randbelow(10000), random_key()
    sh.secrets.append(key2)
    sh.cmd("disc")
    sh.cmd("cfg 192.168.23.1 192.168.23.1 192.168.23.1 255.255.255.0")
    sh.cmd("host n6bench-renamed")
    ok, api = sh.cmd("ap %s %s 1" % (ssid2, key2), 60)
    j2 = Join(ssid2, key2)
    try:
        pc_ip = j2.join("192.168.23.1") if ok else None
        rep.check("ap: config() moves the access point to 192.168.23.1", pc_ip is not None, pc_ip)
        if pc_ip:
            ok, api = sh.cmd("stat")
            rep.check("ap: ...and it hosts on channel 1", field(api, "ch") == "1", field(api, "ch"))
            a = [v for (_, t, v) in mdns_query("n6bench-renamed.local", pc_ip) if t == 1]
            rep.check("ap: setHostname() renames the board over mDNS", a == ["192.168.23.1"], a)
    finally:
        j2.leave()
    sh.cmd("cfg 0.0.0.0")
    sh.cmd("host -")

    # disconnect() and end()
    ssid3, key3 = "N6API-%04d" % secrets.randbelow(10000), random_key()
    sh.secrets.append(key3)
    ok, _ = sh.cmd("ap %s %s 6" % (ssid3, key3), 60)
    j3 = Join(ssid3, key3)
    try:
        pc_ip = j3.join("192.168.10.1") if ok else None
        if pc_ip:
            ok, api = sh.cmd("disc")
            gone = wait_left(ssid3)
            rep.check("ap: disconnect() stops the access point", field(api, "status") == "6" and gone is not None,
                      "board status %s, this PC dropped after %s s" % (field(api, "status"), gone))
            ok, api = sh.cmd("ap %s %s 6" % (ssid3, key3), 60)
            pc_ip = j3.join("192.168.10.1") if ok else None
            rep.check("ap: beginAP again after disconnect()", pc_ip is not None, pc_ip)
            ok, api = sh.cmd("end", 20)
            gone = wait_left(ssid3)
            ok2, api2 = sh.cmd("stat")
            rep.check("ap: end() turns the radio off", gone is not None and field(api2, "ps") == "2",
                      "this PC dropped after %s s, board ps=%s status=%s" % (gone, field(api2, "ps"), field(api2, "status")))
            ok, api = sh.cmd("ap %s %s 6" % (ssid3, key3), 60)
            pc_ip = j3.join("192.168.10.1") if ok else None
            ok2, api2 = sh.cmd("stat")
            rep.check("ap: beginAP after end() wakes the radio", pc_ip is not None and field(api2, "ps") == "0",
                      "%s, ps=%s" % (pc_ip, field(api2, "ps")))
    finally:
        j3.leave()
    sh.cmd("disc")


class GroupOwner:
    """This PC as a WPA2 access point: n6radio wfd-ap, for as long as it runs."""

    def __init__(self, ssid, key):
        self.ssid, self.key, self.p = ssid, key, None

    def start(self):
        exe = build_n6radio.build()
        self.p = subprocess.Popen([exe, "wfd-ap", "--ssid", self.ssid, "--key", self.key, "--seconds", "1800"],
                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        deadline = time.time() + 20
        while time.time() < deadline:
            line = self.p.stdout.readline()
            if "WFD ready" in line:
                return True
            if not line and self.p.poll() is not None:
                return False
        return False

    def stop(self):
        if self.p and self.p.poll() is None:
            self.p.terminate()
            try:
                self.p.wait(5)
            except subprocess.TimeoutExpired:
                self.p.kill()
        self.p = None


def ping_ms(ip, count=10):
    out = subprocess.run(["ping", "-n", str(count), "-w", "2000", ip], capture_output=True, text=True).stdout
    times = [int(t) for t in re.findall(r"time[=<](\d+)ms", out)]
    return sorted(times)[len(times) // 2] if times else None


def phase_sta(sh, rep):
    ext_ssid, ext_key = os.environ.get("N6_STA_SSID"), os.environ.get("N6_STA_KEY")
    ssid, key = (ext_ssid, ext_key) if ext_ssid else ("N6STA-%04d" % secrets.randbelow(10000), random_key())
    sh.secrets.append(key)
    sh.cmd("disc")
    sh.cmd("cfg 0.0.0.0")

    t0 = time.time()
    ok, api = sh.cmd("sta N6NoSuchNet-%04d %s" % (secrets.randbelow(10000), random_key()), 40)
    status = field(api, "status")
    while status != "1" and time.time() - t0 < 80:   # the driver rescans for 16 to 25 s
        time.sleep(1.0)
        status = field(sh.cmd("stat", 10)[1], "status")
    rep.check("sta: WL_NO_SSID_AVAIL for a network nobody hosts", status == "1",
              "after %.0f s" % (time.time() - t0))
    sh.cmd("disc")

    # Leaving a join mid-scan (begin() again, or disconnect()) must release
    # the driver's scan lock, or every later scan and join waits for ever.
    sh.cmd("sta N6NoSuchNet-%04d %s 1500" % (secrets.randbelow(10000), random_key()), 30)
    sh.cmd("sta N6NoSuchNet-%04d %s 1500" % (secrets.randbelow(10000), random_key()), 30)
    ok1, api1 = sh.cmd("scan", 60)
    sh.cmd("disc", 30)
    ok2, api2 = sh.cmd("scan", 60)
    rep.check("sta: begin() again or disconnect() mid-join leaves scanning working", ok1 and ok2,
              "%s, %s" % (field(api1, "n"), field(api2, "n")))

    # Windows opens a group owner on the channel its Wi-Fi already uses, else
    # often on 5 GHz, which the board cannot use. So this PC first joins the
    # board's access point on channel 1, the group owner starts there, and the
    # board moves across before Windows reconsiders.
    go = GroupOwner(ssid, key) if not ext_ssid else None
    steer = None
    if go is not None:
        steer_ssid, steer_key = "N6STEER-%04d" % secrets.randbelow(10000), random_key()
        sh.secrets.append(steer_key)
        sh.cmd("ap %s %s 1" % (steer_ssid, steer_key), 60)
        steer = Join(steer_ssid, steer_key)
        started = steer.join("192.168.10.1") is not None and go.start()
        if not rep.check("sta: this PC hosts a Wi-Fi Direct group owner on 2.4 GHz", started, ssid):
            steer.leave()
            go.stop()
            return
    def joined(timeout=60):
        """begin() returns after its timeout; the station keeper goes on
        trying, as an application that waits on status() would see."""
        ok, api = sh.cmd("sta %s %s" % (ssid, key), 60)
        t0 = time.time()
        while not ok and time.time() - t0 < timeout:
            time.sleep(2.0)
            _, api = sh.cmd("stat", 10)
            ok = field(api, "status") == "3"
        return ok, field(api, "ip"), time.time() - t0

    try:
        ok, board_ip, extra = joined()
        if steer is not None:
            steer.leave()
        rep.check("sta: begin() joins it and gets a DHCP lease", ok,
                  "%s%s" % (board_ip, ", on the station keeper's retry" if extra > 1 else ""))
        if not ok:
            return
        pc_ip = local_ip_towards(board_ip)
        ok, api = sh.cmd("stat")
        st = {k: field(api, k) for k in ("status", "ip", "mask", "gw", "dns", "rssi", "ch", "bssid", "mac", "host")}
        rep.check("sta: status WL_CONNECTED, gateway is this PC", st["status"] == "3" and st["gw"] == pc_ip,
                  "%s gw %s mask %s dns %s" % (st["ip"], st["gw"], st["mask"], st["dns"]))
        rep.check("sta: RSSI is a real reading", st["rssi"] is not None and -95 < int(st["rssi"]) < -5, st["rssi"])
        rep.check("sta: channel and BSSID", st["ch"] not in (None, "0") and st["bssid"] not in (None, "00:00:00:00:00:00")
                  and st["bssid"] != st["mac"], "ch %s bssid %s (board %s)" % (st["ch"], st["bssid"], st["mac"]))

        client_checks(sh, rep, pc_ip, board_ip, "sta")
        srv = server_answers(board_ip, pc_ip, st["host"])
        rep.check("sta: no host server on a joined network", not any(srv.values()), srv)
        ok, api = sh.cmd("wstart 2", 20)
        rep.check("sta: MayaW476_Start() refuses the station transport", not ok and field(api, "rc") == "-3",
                  "; ".join(api[-2:]))
        ok, api = sh.cmd("wstart 1", 20)
        rep.check("sta: ...and the access point's, while joined", not ok and field(api, "rc") == "-3",
                  "; ".join(api[-2:]))

        base = ping_ms(board_ip, 20)
        ok, _ = sh.cmd("lp 1")
        time.sleep(3)
        _, api = sh.cmd("stat")
        slept = ping_ms(board_ip, 20)
        rep.check("sta: lowPowerMode() puts the station in IEEE power save",
                  ok and field(api, "ps") in ("1", "3") and slept is not None and base is not None and slept > base,
                  "ps=%s, median ping %s ms awake, %s ms in power save" % (field(api, "ps"), base, slept))
        ok, _ = sh.cmd("lp 0")
        time.sleep(2)
        _, api = sh.cmd("stat")
        rep.check("sta: noLowPowerMode() wakes it", ok and field(api, "ps") == "0", "ps=%s, ping %s ms"
                  % (field(api, "ps"), ping_ms(board_ip, 10)))

        # A fixed address, while the group owner is still where the board can
        # reach it.
        prefix = board_ip.rsplit(".", 1)[0]
        fixed = prefix + ".77"
        sh.cmd("disc")
        sh.cmd("cfg %s %s %s 255.255.255.0" % (fixed, pc_ip, pc_ip))
        ok, ip, _ = joined()
        rep.check("sta: config() gives the station a fixed address", ok and ip == fixed
                  and ping_ms(fixed, 4) is not None, ip)
        sh.cmd("cfg 0.0.0.0")

        if go is not None:
            mark = sh.mark()
            go.stop()
            lost = sh.wait_for(r"\[api\] beat status=5", 40, since=mark)
            rep.check("sta: WL_CONNECTION_LOST when the network goes", lost is not None, lost or "status never 5")
            go = GroupOwner(ssid, key)
            go.start()
            mark = sh.mark()
            back = sh.wait_for(r"\[api\] beat status=3", 60, since=mark)
            if back is not None:
                rep.check("sta: rejoins by itself when it returns", True, back)
            else:
                _, api = sh.cmd("scan", 60)
                seen = any(field([a], "ssid") == ssid for a in api if a.startswith("net "))
                if seen:
                    rep.check("sta: rejoins by itself when it returns", False, "in range, not rejoined in 60 s")
                else:
                    rep.info("sta: rejoins by itself when it returns",
                             "not testable here: the restarted group owner came back off 2.4 GHz")
        sh.cmd("disc")
    finally:
        if go is not None:
            go.stop()


servers = Servers()


def phase_stored(sh, rep):
    import n6bench

    def restarted(send):
        """Close the port, write through `send`, and come back after the restart."""
        sh.s.close()
        ok = send()
        return sh.reopen() and ok

    def store(payload):
        return restarted(lambda: n6bench.store_record(n6bench.REC_WIFI, payload))

    def shell_restart(cmd):
        """A shell command that writes the asset store, restarting the board."""
        ok = sh.cmd(cmd, 20)[0]
        return restarted(n6bench.wait_reset) and ok

    def asset():
        ok, api = sh.cmd("asset", 20)
        return {k: field(api, k) for k in ("user", "text", "wifi", "fsbl")} if ok else {}

    tag = "%04d" % secrets.randbelow(10000)
    ok = store(b"")
    _, api = sh.cmd("stored", 20)
    rep.check("stored: removing the settings restarts the board", ok and field(api, "mode") == "0",
              "mode %s" % field(api, "mode"))
    ok, api = sh.cmd("sbegin", 60)
    rep.check("stored: begin() with nothing stored starts nothing", not ok and field(api, "status") == "1",
              "status %s (1 is WL_NO_SSID_AVAIL)" % field(api, "status"))

    ssid, key, host = "N6STORE-" + tag, random_key(), "n6-stored-" + tag
    sh.secrets.append(key)
    ok = store(n6bench.wifi_record(2, ssid, key.encode(), None, 6, host))
    _, api = sh.cmd("stored", 20)
    rep.check("stored: the board restarts with the new settings", ok and field(api, "mode") == "2" and
              field(api, "ssid") == ssid and field(api, "hostname") == host,
              "mode %s, %s, hostname %s" % (field(api, "mode"), field(api, "ssid"), field(api, "hostname")))
    rep.check("stored: the bootloader reports the write", asset().get("fsbl") == "written")

    ok, api = sh.cmd("sbegin", 90)
    rep.check("stored: begin() hosts the stored network", ok and field(api, "ip") == "192.168.10.1",
              "status %s, %s" % (field(api, "status"), field(api, "ssid")))
    j = Join(ssid, key)
    try:
        pc_ip = j.join("192.168.10.1") if ok else None
        rep.check("stored: this PC joins it with the stored passphrase", pc_ip is not None, pc_ip)
        if pc_ip:
            rep.check("stored: MayaW476_Start() starts the host server",
                      sh.cmd("wstart 1", 30)[0] and wait_server("192.168.10.1") is not None)
            a = [v for (_, t, v) in mdns_query(host + ".local", pc_ip) if t == 1]
            rep.check("stored: the stored hostname answers over mDNS", a == ["192.168.10.1"], a)
            details = {}
            n6bench.discover(5000, "192.168.10.255", 2.0, details)
            got = details.get("192.168.10.1", (None, False))
            rep.check("stored: discovery names the board by the stored hostname", got[0] == host, got)
    finally:
        j.leave()
        wait_left(ssid)

    ok = shell_restart("awrite n6-user-asset-" + tag)
    a = asset()
    rep.check("stored: a user asset written beside the settings", ok and a.get("user") == "yes" and
              a.get("text") == "n6-user-asset-" + tag and a.get("wifi") == "yes", a)
    _, api = sh.cmd("stored", 20)
    rep.check("stored: ...and the settings are still there", field(api, "ssid") == ssid)

    nope = "N6NOPE-" + tag
    ok = store(n6bench.wifi_record(1, nope, random_key().encode()))
    a = asset()
    rep.check("stored: new settings keep the user asset", ok and a.get("user") == "yes" and
              a.get("text") == "n6-user-asset-" + tag and a.get("wifi") == "yes", a)
    t0 = time.time()
    ok, api = sh.cmd("sbegin", 90)
    status = field(api, "status")
    while not ok and status != "1" and time.time() - t0 < 80:   # the driver rescans for 16 to 25 s
        time.sleep(1.0)
        status = field(sh.cmd("stat", 10)[1], "status")
    rep.check("stored: joining a network that is not there ends in WL_NO_SSID_AVAIL", not ok and status == "1",
              "status %s after %.0f s" % (status, time.time() - t0))

    ok = store(b"")
    a = asset()
    _, api = sh.cmd("stored", 20)
    rep.check("stored: removing the settings keeps the user asset", ok and a.get("user") == "yes" and
              a.get("wifi") == "no" and field(api, "mode") == "0", a)
    ok = shell_restart("aclear")
    a = asset()
    rep.check("stored: the user asset removed as well", ok and a.get("user") == "no" and a.get("wifi") == "no", a)


def phase_link(sh, rep):
    import n6bench
    board = "192.168.10.1"

    def keyed():
        d = {}
        n6bench.discover(5000, "192.168.10.255", 2.0, d)
        return d.get(board, (None, None))[1]

    def raw_challenge():
        """Connect without answering: the challenge, and whatever else arrives."""
        s = socket.create_connection((board, 5000), timeout=5)
        t0 = time.time()
        s.settimeout(1.5)
        got = b""
        try:
            while len(got) < 39:
                chunk = s.recv(4096)
                if not chunk:
                    break
                got += chunk
        except socket.timeout:
            pass
        return s, got, t0

    def closed_within(s, t0, seconds):
        """Seconds from connecting until the board closed or reset `s`, None
        if it stayed open or sent anything meanwhile. The one zero byte a
        refusal queues, so that lwIP resets rather than closes, does not count:
        it can arrive just before the reset."""
        s.settimeout(0.5)
        got = b""
        while time.time() - t0 < seconds:
            try:
                data = s.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return time.time() - t0
            if not data:
                return time.time() - t0
            got += data
            if got != b"\x00":
                return None
        return None

    def streams(link_key):
        """Connect as a host: was it challenged, and did the board attach it?
        (This example has no camera, so an attached host gets few frames; the
        board's console says what it did.)"""
        mark = sh.mark()
        try:
            t = n6bench.TcpLink(board, link_key=link_key)
        except (OSError, ConnectionError) as e:
            return False, False, str(e)
        said = sh.wait_for(r"Neuro host connected|a Neuro host was refused", 6, since=mark)
        t.close()
        sh.wait_for(r"listening again|was refused", 6, since=mark)   # the slot is free again
        return t.challenged, said is not None and "connected" in said, said

    tag = "%04d" % secrets.randbelow(10000)
    ssid, key, lk = "N6LINK-" + tag, random_key(), random_key(24)
    sh.secrets += [key, lk]
    sh.cmd("key -")
    ok, api = sh.cmd("ap %s %s" % (ssid, key), 60)
    j = Join(ssid, key)
    try:
        pc_ip = j.join(board) if ok else None
        if not rep.check("link: this PC joins the board's access point", pc_ip is not None, pc_ip):
            return
        rep.check("link: MayaW476_Start() starts the host server",
                  sh.cmd("wstart 1", 30)[0] and wait_server(board) is not None)
        rep.check("link: without a key, discovery says none is needed", keyed() is False)
        challenged, got, why = streams(b"")
        rep.check("link: without a key, a host is attached at once", not challenged and got, why)

        rep.check("link: MayaW476_SetLinkKey() refuses a key under 8 characters", not sh.cmd("key short")[0])
        rep.check("link: MayaW476_SetLinkKey() takes a key", sh.cmd("key " + lk)[0])
        rep.check("link: discovery says a key is needed", keyed() is True)

        s, got, t0 = raw_challenge()
        ch = n6bench.read_challenge(got)
        rep.check("link: a host is challenged first, with nothing else", ch is not None and len(got) == 39,
                  "%d bytes" % len(got))
        took = closed_within(s, t0, 14)
        s.close()
        rep.check("link: ...and dropped when it does not answer", took is not None and 9 <= took <= 12,
                  "closed after %s s" % ("%.1f" % took if took is not None else "never"))

        mark = sh.mark()
        s, got, t0 = raw_challenge()
        ch = n6bench.read_challenge(got)
        s.sendall(n6bench.frame_cmd(n6bench.link_answer(b"not the key at all", ch or b"")))
        took = closed_within(s, t0, 5)
        s.close()
        rep.check("link: a wrong key is refused", ch is not None and took is not None,
                  "closed after %.1f s" % took if took is not None else "stayed open")

        s, got, t0 = raw_challenge()
        s.sendall(n6bench.frame_cmd(n6bench.sub0(1)))
        took = closed_within(s, t0, 5)
        s.close()
        rep.check("link: a command instead of the answer is refused", took is not None,
                  "closed after %.1f s" % took if took is not None else "stayed open")
        rep.check("link: the board says why", sh.wait_for(r"refused: wrong link key", 3, since=mark) is not None)

        challenged, got, why = streams(lk.encode())
        rep.check("link: the right key is attached", challenged and got, why)

        sh.cmd("key -")
        challenged, got, why = streams(b"")
        rep.check("link: MayaW476_SetLinkKey(\"\") turns it off", keyed() is False and not challenged and got, why)
    finally:
        j.leave()
        wait_left(ssid)

    # A key stored with the Wi-Fi settings, over the USB cable.
    if NO_USB_STORE:
        rep.info("link: a key stored with the WiFi settings", "left out (--no-usb-store)")
        return
    lk2 = random_key(32)
    sh.secrets.append(lk2)
    sh.s.close()
    ok = n6bench.store_record(n6bench.REC_WIFI, n6bench.wifi_record(2, ssid, key.encode(), None, 6, "",
                                                                    lk2.encode()))
    ok = sh.reopen() and ok
    ok2, api = sh.cmd("sbegin", 90)
    j = Join(ssid, key)
    try:
        pc_ip = j.join(board) if ok and ok2 else None
        rep.check("link: stored settings with a link key come back up", pc_ip is not None, pc_ip)
        if pc_ip:
            rep.check("link: MayaW476_Start() starts the host server again",
                      sh.cmd("wstart 1", 30)[0] and wait_server(board) is not None)
            rep.check("link: the stored key is asked for", keyed() is True)
            challenged, got, why = streams(lk.encode())
            rep.check("link: the old key is refused", challenged and not got, why)
            challenged, got, why = streams(lk2.encode())
            rep.check("link: the stored key is attached", challenged and got, why)
    finally:
        j.leave()
        wait_left(ssid)
        sh.s.close()
        n6bench.store_record(n6bench.REC_WIFI, b"")
        sh.reopen()


def phase_wpa3(sh, rep):
    board_ip = "192.168.10.1"
    sh.cmd("disc")
    sh.cmd("cfg 0.0.0.0")

    def echo(pc_ip):
        ok, api = sh.cmd("tcp %s %d 20000" % (pc_ip, TCP_ECHO), 60)
        return ok, "; ".join(a for a in api if a.startswith("tcp "))

    # WPA3 alone.
    ssid, key = "N6W3-%04d" % secrets.randbelow(10000), random_key()
    sh.secrets.append(key)
    ok, api = sh.cmd("ap %s %s 6 wpa3" % (ssid, key), 60)
    if not rep.check("wpa3: beginAP with WPA3 alone comes up", ok and field(api, "sec") == "wpa3", "; ".join(api[-2:])):
        return
    seen = scanned_auth(ssid)
    rep.check("wpa3: this PC's scan shows WPA3-Personal", seen == "WPA3-Personal", seen)

    mark = sh.mark()
    j = Join(ssid, key, auth="WPA3SAE")
    try:
        t0 = time.time()
        pc_ip = j.join(board_ip)
        auth, cipher = wlan_auth()
        rep.check("wpa3: this PC joins with SAE", pc_ip is not None and auth == "WPA3-Personal",
                  "%s, %s %s, %.1f s" % (pc_ip, auth, cipher, time.time() - t0))
        if pc_ip:
            client_checks(sh, rep, pc_ip, board_ip, "wpa3")
            time.sleep(5)
            state, cur, _ = wlan_state()
            kicked = sh.wait_for(r"never finished the four-way handshake", 0.2, since=mark)
            rep.check("wpa3: the client stays joined, not sent away as unauthorised",
                      state == "connected" and cur == ssid and kicked is None, kicked or "%s to %s" % (state, cur))
    finally:
        j.leave()

    j = Join(ssid, key)
    try:
        pc_ip = j.join(board_ip, timeout=25)
        rep.check("wpa3: a WPA2 client cannot join WPA3 alone", pc_ip is None, pc_ip or "not joined")
    finally:
        j.leave()
    j = Join(ssid, random_key(), auth="WPA3SAE")
    try:
        pc_ip = j.join(board_ip, timeout=25)
        rep.check("wpa3: a wrong password is refused", pc_ip is None, pc_ip or "not joined")
    finally:
        j.leave()
    j = Join(ssid, key, auth="WPA3SAE")
    try:
        pc_ip = j.join(board_ip)
        rep.check("wpa3: ...and the right one still joins after it", pc_ip is not None, pc_ip)
    finally:
        j.leave()

    # A password longer than 63 characters: refused with WPA3 too.
    ssid_l, key_l = "N6W3L-%04d" % secrets.randbelow(10000), random_key(100)
    sh.secrets.append(key_l)
    for sec in ("wpa2-wpa3", "wpa3"):
        ok, why = refused(sh, "ap %s %s 6 %s" % (ssid_l, key_l, sec), "the passphrase must be 8 to 63 characters")
        rep.check("wpa3: %s refuses a 100-character password" % sec, ok, why)

    # WPA2 and WPA3 together.
    ssid2, key2 = "N6W23-%04d" % secrets.randbelow(10000), random_key()
    sh.secrets.append(key2)
    ok, api = sh.cmd("ap %s %s 6 wpa2-wpa3" % (ssid2, key2), 60)
    if not rep.check("wpa3: beginAP with WPA2 and WPA3 together comes up",
                     ok and field(api, "sec") == "wpa2-wpa3", "; ".join(api[-2:])):
        return
    rep.info("wpa3: this PC's scan shows WPA2/WPA3 as", scanned_auth(ssid2))
    for auth, want, how in (("WPA3SAE", "WPA3-Personal", "SAE"), ("WPA2PSK", "WPA2-Personal", "WPA2")):
        j = Join(ssid2, key2, auth=auth)
        try:
            pc_ip = j.join(board_ip)
            got, cipher = wlan_auth()
            ok, detail = echo(pc_ip) if pc_ip else (False, "not joined")
            rep.check("wpa3: WPA2/WPA3 joined with %s, and a TCP echo" % how, pc_ip is not None and got == want and ok,
                      "%s %s; %s" % (got, cipher, detail))
        finally:
            j.leave()

    # WPA3 alone in the settings a host stores over the USB cable (see
    # phase_stored); the board restarts at each write.
    if NO_USB_STORE:
        rep.info("wpa3: stored settings with WPA3 alone", "left out (--no-usb-store)")
        return wpa3_station(sh, rep)
    import n6bench

    def store(payload):
        sh.s.close()
        ok = n6bench.store_record(n6bench.REC_WIFI, payload)
        return sh.reopen() and ok

    ssid_s, key_s = "N6W3S-%04d" % secrets.randbelow(10000), random_key(40)
    sh.secrets.append(key_s)
    ok = store(n6bench.wifi_record(2, ssid_s, key_s.encode(), n6bench.SECURITY["wpa3"], 6))
    _, api = sh.cmd("stored", 20)
    rep.check("wpa3: stored settings with WPA3 alone", ok and field(api, "mode") == "2" and field(api, "ssid") == ssid_s,
              "mode %s, %s" % (field(api, "mode"), field(api, "ssid")))
    ok, api = sh.cmd("sbegin", 90)
    j = Join(ssid_s, key_s, auth="WPA3SAE")
    try:
        pc_ip = j.join(board_ip) if ok else None
        auth, cipher = wlan_auth()
        rep.check("wpa3: begin() hosts the stored network, and this PC joins with SAE",
                  pc_ip is not None and auth == "WPA3-Personal", "%s, %s %s" % (pc_ip, auth, cipher))
    finally:
        j.leave()
        wait_left(ssid_s)
    ok = store(b"")
    _, api = sh.cmd("stored", 20)
    rep.check("wpa3: the stored settings removed again", ok and field(api, "mode") == "0", "mode %s" % field(api, "mode"))
    wpa3_station(sh, rep)


def wpa3_station(sh, rep):
    # The board as a station, of this PC's WPA2 group owner (see phase_sta
    # for the channel steering).
    board_ip = "192.168.10.1"
    go_ssid, go_key = "N6W3STA-%04d" % secrets.randbelow(10000), random_key()
    steer_ssid, steer_key = "N6STEER-%04d" % secrets.randbelow(10000), random_key()
    sh.secrets += [go_key, steer_key]
    sh.cmd("disc")
    sh.cmd("ap %s %s 1" % (steer_ssid, steer_key), 60)
    go = GroupOwner(go_ssid, go_key)
    steer = Join(steer_ssid, steer_key)
    try:
        started = steer.join(board_ip) is not None and go.start()
        if not rep.check("wpa3: this PC hosts a WPA2 group owner on 2.4 GHz", started, go_ssid):
            return
        ok, api = sh.cmd("sta %s %s 20000 wpa2-wpa3" % (go_ssid, go_key), 60)
        t0 = time.time()
        while not ok and time.time() - t0 < 60:
            time.sleep(2.0)
            _, api = sh.cmd("stat", 10)
            ok = field(api, "status") == "3"
        steer.leave()
        rep.check("wpa3: a station asking for WPA2/WPA3 joins a WPA2 network", ok, field(api, "ip"))
        sh.cmd("disc")
        ok, api = sh.cmd("sta %s %s 20000 wpa3" % (go_ssid, go_key), 60)
        time.sleep(5)
        _, st = sh.cmd("stat", 10)
        rep.check("wpa3: a station asking for WPA3 alone does not join a WPA2 network",
                  not ok and field(st, "status") != "3", "status %s" % field(st, "status"))
        sh.cmd("disc")
    finally:
        steer.leave()
        go.stop()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="the board's console port (COMn on Windows)")
    ap.add_argument("--phases", default="ap,sta")
    ap.add_argument("--no-usb-store", action="store_true",
                    help="leave out what writes stored settings over the USB cable")
    ap.add_argument("--json")
    ap.add_argument("--log", default=os.path.join(tempfile.gettempdir(), "n6wifiapi.log"))
    args = ap.parse_args()
    global NO_USB_STORE
    NO_USB_STORE = args.no_usb_store

    servers.start()
    sh = Shell(args.port, args.log)
    rep = Report()
    try:
        ok, api = sh.cmd("stat", 20)
        if not rep.check("the board answers the shell", ok, api[:1]):
            return rep.finish(args.json)
        for ph in args.phases.split(","):
            if ph.strip() == "stored" and NO_USB_STORE:
                rep.info("stored", "left out (--no-usb-store)")
                continue
            {"pass": phase_pass, "ap": phase_ap, "sta": phase_sta, "stored": phase_stored,
             "link": phase_link, "wpa3": phase_wpa3}[ph.strip()](sh, rep)
    finally:
        sh.close()
        print("board console: %s" % args.log)
    return rep.finish(args.json)


if __name__ == "__main__":
    sys.exit(main())
