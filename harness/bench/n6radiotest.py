#!/usr/bin/env python3
"""Check the radio diagnostics and BLE on a board running
Examples/WiFiApi_FreeRTOS (Windows).

    python harness/bench/n6radiotest.py --port PORT [--phases ap,link,scan,adv] [--json out.json]

ap    the access point's beacon (visible, hidden, visible again) and the
      station that joins it through a temporary profile
link  the link rate, against the 802.11n table and what Windows reports, and
      the firmware's counters
scan  BLE scanning: the board must report this PC's nonce (company 0xFFFF,
      reserved for testing)
adv   BLE advertising: this PC must see the board's nonce at its address

Checks are anchored on the board's own BSSID or BLE address, so a phase
cannot pass because nothing was listening. The BLE phases need n6radio.exe,
which build_n6radio.py makes.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT

import argparse
import json
import os
import re
import secrets
import subprocess
import sys
import tempfile
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from n6wifiapi import IFACE, Shell, netsh, random_key          # noqa: E402
from n6wlanscan import look_for                                 # noqa: E402

N6RADIO = os.path.join(tempfile.gettempdir(), "n6bench", "n6radio.exe")
ADV_NAME = "N6ADV"

PROFILE = """<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
  <name>{name}</name>
  <SSIDConfig><SSID><name>{ssid}</name></SSID><nonBroadcast>{hidden}</nonBroadcast></SSIDConfig>
  <connectionType>ESS</connectionType>
  <connectionMode>manual</connectionMode>
  <MSM><security>
    <authEncryption><authentication>WPA2PSK</authentication><encryption>AES</encryption><useOneX>false</useOneX></authEncryption>
    <sharedKey><keyType>passPhrase</keyType><protected>false</protected><keyMaterial>{key}</keyMaterial></sharedKey>
  </security></MSM>
</WLANProfile>
"""

# 802.11n, one spatial stream, kbit/s, by [bandwidth][short guard][MCS].
HT_KBIT = {
    (20, 0): [6500, 13000, 19500, 26000, 39000, 52000, 58500, 65000],
    (20, 1): [7200, 14400, 21700, 28900, 43300, 57800, 65000, 72200],
    (40, 0): [13500, 27000, 40500, 54000, 81000, 108000, 121500, 135000],
    (40, 1): [15000, 30000, 45000, 60000, 90000, 120000, 135000, 150000],
}


def ht_rate(bandwidth, short_gi, mcs):
    row = HT_KBIT.get((bandwidth, short_gi))
    return row[mcs] if (row is not None) and (mcs < 8) else None


class Report:
    def __init__(self):
        self.rows = []

    def check(self, name, ok, detail=""):
        self.rows.append({"name": name, "ok": bool(ok), "detail": str(detail)})
        print("%-30s %s  %s" % (name, "PASS" if ok else "FAIL", detail))
        return bool(ok)

    def info(self, text):
        print("   %s" % text)

    def finish(self, path):
        bad = [r for r in self.rows if not r["ok"]]
        print("\n%d/%d pass" % (len(self.rows) - len(bad), len(self.rows)))
        if path:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.rows, f, indent=2)
        return 1 if bad else 0


class Join:
    """A temporary manual-connect profile, removed on leave()."""

    def __init__(self, ssid, key, hidden=False):
        self.ssid, self.key, self.hidden = ssid, key, hidden
        self.name = "n6bench-" + ssid

    def join(self, timeout=45):
        xml = PROFILE.format(name=self.name, ssid=self.ssid, key=self.key,
                             hidden="true" if self.hidden else "false")
        fd, path = tempfile.mkstemp(suffix=".xml")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                f.write(xml)
            netsh("add", "profile", "filename=%s" % path,
                  "interface=%s" % IFACE, "user=current")
        finally:
            os.remove(path)

        deadline = time.time() + timeout
        while time.time() < deadline:
            netsh("connect", "name=%s" % self.name, "interface=%s" % IFACE)
            for _ in range(16):
                time.sleep(0.75)
                out = netsh("show", "interfaces")
                state = re.search(r"^\s*State\s*:\s*(\S+)", out, re.M)
                ssid = re.search(r"^\s*SSID\s*:\s*(.+)$", out, re.M)
                if state and state.group(1) == "connected" and \
                        ssid and ssid.group(1).strip() == self.ssid:
                    return True
        return False

    def leave(self):
        netsh("disconnect", "interface=%s" % IFACE)
        netsh("delete", "profile", "name=%s" % self.name, "interface=%s" % IFACE)


class Advertiser:
    """This PC advertising a payload for as long as the block runs."""

    def __init__(self, payload_hex, seconds=90):
        self.hex, self.seconds, self.proc = payload_hex, seconds, None

    def __enter__(self):
        self.proc = subprocess.Popen(
            [N6RADIO, "advertise", "--mfg", self.hex, "--seconds", str(self.seconds)],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        threading.Thread(target=lambda: [None for _ in self.proc.stdout],
                         daemon=True).start()
        time.sleep(3.0)   # let the publisher start
        return self

    def __exit__(self, *a):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
        return False


def pc_scan(seconds=8):
    out = subprocess.run([N6RADIO, "scan", "--seconds", str(seconds)],
                         capture_output=True, text=True, timeout=seconds + 60).stdout
    return [l for l in out.splitlines() if l.startswith("ADV ")]


def pc_mac():
    """Read after joining: Windows randomises the address per network."""
    m = re.search(r"^\s*Physical address\s*:\s*(\S+)",
                  netsh("show", "interfaces"), re.M)
    return m.group(1).lower().replace("-", ":") if m else ""


def fields(text):
    return dict((k, int(v)) for k, v in re.findall(r"(\w+)=(-?\d+)", text))


def board_bssid(sh):
    _, api = sh.cmd("stat")
    m = re.search(r"bssid=(\S+)", " ".join(api))
    return m.group(1).lower() if m else ""


def start_ap(sh, ssid, key):
    ok, api = sh.cmd("ap %s %s 6" % (ssid, key), timeout=60)
    return ok, " | ".join(sh.hide(a) for a in api)


# Phases.

def phase_ap(sh, rep):
    ssid, key = "N6AP-" + secrets.token_hex(3), random_key()
    sh.secrets.append(key)

    sh.cmd("hidden 0")
    rep.check("ap starts visible", *start_ap(sh, ssid, key))
    bssid = board_bssid(sh)
    rep.check("the board gives a bssid", len(bssid) == 17, bssid)

    took, advertised = look_for(bssid)
    rep.check("visible: the bssid is seen", took is not None,
              "after %ss" % took if took is not None else "never, in 60 s")
    rep.check("visible: it advertises the ssid", advertised == ssid,
              "advertised %r" % advertised)

    ok, api = sh.cmd("hidden 1")
    rep.check("hidden is set", ok and any("hidden 1" in a for a in api),
              " | ".join(api))
    rep.check("ap restarts hidden", *start_ap(sh, ssid, key))

    took, advertised = look_for(board_bssid(sh))
    # The same radio has to stay visible, or this PC simply stopped scanning
    # and the missing SSID would say nothing about the beacon.
    rep.check("hidden: the bssid is still seen", took is not None,
              "after %ss" % took if took is not None
              else "the scan went blind, so this phase is void")
    rep.check("hidden: the ssid is withheld",
              (took is not None) and (advertised == ""),
              "advertised %r" % advertised)

    join = Join(ssid, key, hidden=True)
    try:
        joined = join.join()
        mine = pc_mac()
        rep.check("this pc joins it while hidden", joined, "mac %s" % mine)
        if joined:
            time.sleep(3.0)
            ok, api = sh.cmd("clients", timeout=30)
            text = " | ".join(api)
            n = re.search(r"clients n=(\d+)", text)
            rep.check("the board reports one client",
                      ok and n is not None and int(n.group(1)) == 1, text)
            macs = [m.lower() for m in re.findall(r"mac=(\S+)", text)]
            rep.check("that client is this pc", (mine != "") and (mine in macs),
                      "board %s, pc %s" % (macs, mine))
            rssi = re.search(r"rssi=(-?\d+)", text)
            rep.check("its signal is plausible",
                      bool(rssi) and -95 <= int(rssi.group(1)) <= -5,
                      "rssi=%s" % (rssi.group(1) if rssi else "none"))

            netsh("disconnect", "interface=%s" % IFACE)
            time.sleep(6.0)
            ok, api = sh.cmd("clients", timeout=30)
            text = " | ".join(api)
            n = re.search(r"clients n=(\d+)", text)
            rep.check("the list empties when it leaves",
                      ok and n is not None and int(n.group(1)) == 0, text)
    finally:
        join.leave()

    sh.cmd("hidden 0")
    rep.check("ap restarts visible", *start_ap(sh, ssid, key))
    took, advertised = look_for(board_bssid(sh))
    # The firmware keeps the setting across a stop, so the 0 has to be written.
    rep.check("the ssid comes back", advertised == ssid,
              "advertised %r after %ss" % (advertised, took))


def phase_link(sh, rep):
    ssid, key = "N6AP-" + secrets.token_hex(3), random_key()
    sh.secrets.append(key)
    sh.cmd("hidden 0")
    rep.check("ap starts", *start_ap(sh, ssid, key))

    ok, api = sh.cmd("counters")
    rep.check("counters read while hosting", ok, " | ".join(api))

    join = Join(ssid, key)
    try:
        if not rep.check("this pc joins it", join.join(), "mac %s" % pc_mac()):
            return
        board_ip = "192.168.10.1"
        subprocess.run(["ping", "-n", "20", "-l", "1400", board_ip],
                       capture_output=True, text=True)
        time.sleep(1.0)

        ok, api = sh.cmd("rate")
        text = " | ".join(api)
        f = fields(text)
        if rep.check("the rate reads back", ok and "tx" in f, text):
            rep.info("board tx %.1f Mbit/s, rx %.1f Mbit/s"
                     % (f["tx"] / 1000.0, f["rx"] / 1000.0))
            rep.check("the transmit rate matches the table",
                      f["tx"] == ht_rate(f["txbw"], f["txsgi"], f["txmcs"]),
                      "board %d, table %s"
                      % (f["tx"], ht_rate(f["txbw"], f["txsgi"], f["txmcs"])))
            rep.check("the receive rate matches the table",
                      f["rx"] == ht_rate(f["rxbw"], f["rxsgi"], f["rxmcs"]),
                      "board %d, table %s"
                      % (f["rx"], ht_rate(f["rxbw"], f["rxsgi"], f["rxmcs"])))

            # Windows reports the negotiated rate, not the last used, so it is
            # checked against the table rather than the board.
            out = netsh("show", "interfaces")
            m = re.search(r"^\s*Receive rate \(Mbps\)\s*:\s*([\d.]+)", out, re.M)
            negotiated = float(m.group(1)) if m else None
            cells = [(bw, gi, mcs) for bw in (20, 40) for gi in (0, 1)
                     for mcs in range(8)
                     if negotiated is not None
                     and abs(ht_rate(bw, gi, mcs) / 1000.0 - negotiated) < 0.05]
            rep.check("what windows reports is in the table", len(cells) > 0,
                      "%s Mbit/s = %s" % (negotiated, cells[0] if cells else "no cell"))
            rep.check("the rate used is at or below it",
                      negotiated is not None and (f["tx"] / 1000.0) <= negotiated + 0.05,
                      "used %.1f, negotiated %s" % (f["tx"] / 1000.0, negotiated))

        ok, api = sh.cmd("counters")
        before = fields(" | ".join(api))
        rep.check("counters read", ok and "tx" in before, " | ".join(api))
        subprocess.run(["ping", "-n", "30", "-l", "1400", board_ip],
                       capture_output=True, text=True)
        time.sleep(1.0)
        ok, api = sh.cmd("counters")
        after = fields(" | ".join(api))
        rep.check("counters read again", ok and "tx" in after, " | ".join(api))
        if ("tx" in before) and ("tx" in after):
            rep.info("frames %d to %d, retries %d to %d"
                     % (before["tx"], after["tx"], before["retry"], after["retry"]))
            rep.check("frames advanced with traffic", after["tx"] > before["tx"],
                      "+%d over 30 pings" % (after["tx"] - before["tx"]))
            rep.check("no counter went backwards",
                      all(after[k] >= before[k]
                          for k in ("tx", "retry", "ackfail", "failed")), "")
    finally:
        join.leave()


def phase_scan(sh, rep):
    nonce = secrets.token_hex(6)
    rep.info("nonce %s" % nonce)

    ok, api = sh.cmd("btscan 8", timeout=90)
    seen = re.search(r"btscan seen=(\d+)", " ".join(api))
    ambient = int(seen.group(1)) if seen else 0
    rep.check("the scan runs", ok, "%d advertisements in 8 s" % ambient)
    rep.check("the nonce is absent beforehand",
              not any(nonce.upper() in a for a in api), "")

    with Advertiser(nonce):
        ok, api = sh.cmd("btscan 12", timeout=120)
        hits = [a for a in api if nonce.upper() in a]
        rep.check("the board finds the nonce", len(hits) > 0,
                  hits[0] if hits else "not among the advertisements seen")
        if hits:
            payload = re.search(r"data=([0-9A-F]+)", hits[0])
            want = "FFFF" + nonce.upper()
            rep.check("company and payload are intact",
                      payload is not None and want in payload.group(1),
                      "looked for %s in %s"
                      % (want, payload.group(1) if payload else "nothing"))
            rssi = re.search(r"rssi=(-?\d+)", hits[0])
            rep.check("its signal is plausible",
                      bool(rssi) and -100 <= int(rssi.group(1)) <= -5,
                      "rssi=%s" % (rssi.group(1) if rssi else "none"))

    time.sleep(3.0)
    ok, api = sh.cmd("btscan 8", timeout=90)
    seen = re.search(r"btscan seen=(\d+)", " ".join(api))
    # Ambient advertisers are the liveness control: without them an absent
    # nonce would only mean the scan had stopped working.
    rep.check("it is gone once the advertiser stops",
              not any(nonce.upper() in a for a in api),
              "%s other advertisements still seen"
              % (seen.group(1) if seen else "0"))


def phase_adv(sh, rep):
    nonce = secrets.token_hex(6)
    name = ADV_NAME.encode()
    payload = (bytes([2, 0x01, 0x06]) +
               bytes([1 + len(name), 0x09]) + name +
               bytes([3 + 6, 0xFF, 0xFF, 0xFF]) + bytes.fromhex(nonce)).hex()
    rep.info("nonce %s, payload %s" % (nonce, payload))

    ok, api = sh.cmd("btinfo", timeout=90)
    m = re.search(r"addr=(\S+)", " ".join(api))
    addr = m.group(1).lower() if m else ""
    rep.check("the board gives its address", len(addr) == 17, addr)

    lines = pc_scan(8)
    rep.check("this pc's scanner works", len(lines) > 0,
              "%d advertisers in 8 s" % len(lines))
    rep.check("the nonce is absent beforehand",
              not any(nonce.lower() in l.lower() for l in lines), "")

    ok, api = sh.cmd("btadv %s 100" % payload, timeout=90)
    rep.check("the board starts advertising", ok, " | ".join(api))
    time.sleep(2.0)

    lines = pc_scan(10)
    hits = [l for l in lines if nonce.lower() in l.lower()]
    rep.check("this pc hears the nonce", len(hits) > 0,
              hits[0] if hits else "not among %d advertisers" % len(lines))
    if hits:
        rep.check("the name came through", ('name="%s"' % ADV_NAME) in hits[0], "")
        rep.check("the company is 0xFFFF", "mfg=ffff:" in hits[0].lower(), "")
        m = re.search(r"addr=(\S+)", hits[0])
        rep.check("the address is the board's",
                  bool(m) and m.group(1).lower() == addr,
                  "scanned %s, board %s" % (m.group(1) if m else "none", addr))

    ok, api = sh.cmd("btadv off", timeout=60)
    rep.check("the board stops advertising", ok, " | ".join(api))
    time.sleep(2.0)
    lines = pc_scan(8)
    rep.check("it is gone once stopped",
              not any(nonce.lower() in l.lower() for l in lines),
              "%d other advertisers still in range" % len(lines))


PHASES = {"ap": phase_ap, "link": phase_link, "scan": phase_scan, "adv": phase_adv}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="the board's console port (COMn on Windows)")
    ap.add_argument("--phases", default="ap,link,scan,adv")
    ap.add_argument("--log", default=os.path.join(tempfile.gettempdir(),
                                                  "n6bench", "n6radiotest.log"))
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    wanted = [p.strip() for p in args.phases.split(",") if p.strip()]
    if any(p in ("scan", "adv") for p in wanted) and not os.path.isfile(N6RADIO):
        print("n6radio.exe is missing: run python harness/bench/build_n6radio.py")
        return 2

    os.makedirs(os.path.dirname(args.log), exist_ok=True)
    sh = Shell(args.port, args.log)
    rep = Report()
    try:
        ok, api = sh.cmd("stat", 20)
        if not rep.check("the board answers the shell", ok, api[:1]):
            return rep.finish(args.json)
        for phase in wanted:
            print("\n== %s ==" % phase)
            PHASES[phase](sh, rep)
    finally:
        try:
            sh.cmd("hidden 0")
            sh.cmd("btadv off", timeout=30)
        except Exception:
            pass
        sh.close()
        print("board console: %s" % args.log)
    return rep.finish(args.json)


if __name__ == "__main__":
    sys.exit(main())
