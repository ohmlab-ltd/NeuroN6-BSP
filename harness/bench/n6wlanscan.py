#!/usr/bin/env python3
"""Force a Wi-Fi scan on this PC and read what came back (Windows).

`netsh wlan show networks` serves the adapter's cache, and an idle adapter
does not scan, so a beaconing network can be missing; this asks the driver to
scan first. The cache also keeps old names: one BSSID can be listed under
several SSIDs. So a BSSID's name is taken only from entries heard since the
question was asked (`ullHostTimestamp`), which `netsh` cannot tell.

Imported by n6radiotest.py.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT

import ctypes
import re
import subprocess
import time
from ctypes import wintypes

wlanapi = ctypes.windll.wlanapi


class GUID(ctypes.Structure):
    _fields_ = [("Data1", wintypes.DWORD), ("Data2", wintypes.WORD),
                ("Data3", wintypes.WORD), ("Data4", ctypes.c_ubyte * 8)]


class WLAN_INTERFACE_INFO(ctypes.Structure):
    _fields_ = [("InterfaceGuid", GUID),
                ("strInterfaceDescription", wintypes.WCHAR * 256),
                ("isState", wintypes.DWORD)]


class WLAN_INTERFACE_INFO_LIST(ctypes.Structure):
    _fields_ = [("dwNumberOfItems", wintypes.DWORD),
                ("dwIndex", wintypes.DWORD),
                ("InterfaceInfo", WLAN_INTERFACE_INFO * 8)]


class DOT11_SSID(ctypes.Structure):
    _fields_ = [("uSSIDLength", wintypes.ULONG), ("ucSSID", ctypes.c_ubyte * 32)]


class WLAN_RATE_SET(ctypes.Structure):
    _fields_ = [("uRateSetLength", wintypes.ULONG), ("usRateSet", wintypes.USHORT * 126)]


class WLAN_BSS_ENTRY(ctypes.Structure):
    _fields_ = [("dot11Ssid", DOT11_SSID), ("uPhyId", wintypes.ULONG),
                ("dot11Bssid", ctypes.c_ubyte * 6), ("dot11BssType", ctypes.c_int),
                ("dot11BssPhyType", ctypes.c_int), ("lRssi", wintypes.LONG),
                ("uLinkQuality", wintypes.ULONG), ("bInRegDomain", ctypes.c_ubyte),
                ("usBeaconPeriod", wintypes.USHORT), ("ullTimestamp", ctypes.c_ulonglong),
                ("ullHostTimestamp", ctypes.c_ulonglong),
                ("usCapabilityInformation", wintypes.USHORT),
                ("ulChCenterFrequency", wintypes.ULONG), ("wlanRateSet", WLAN_RATE_SET),
                ("ulIeOffset", wintypes.ULONG), ("ulIeSize", wintypes.ULONG)]


class WLAN_BSS_LIST(ctypes.Structure):
    _fields_ = [("dwTotalSize", wintypes.DWORD), ("dwNumberOfItems", wintypes.DWORD),
                ("wlanBssEntries", WLAN_BSS_ENTRY * 1)]


assert ctypes.sizeof(WLAN_BSS_ENTRY) == 360   # wlanapi.h's layout


def scan_now(settle=8.0):
    """Ask the driver for a fresh scan. True once it has had time to answer."""
    handle = wintypes.HANDLE()
    negotiated = wintypes.DWORD()

    if wlanapi.WlanOpenHandle(2, None, ctypes.byref(negotiated),
                              ctypes.byref(handle)) != 0:
        return False
    try:
        plist = ctypes.POINTER(WLAN_INTERFACE_INFO_LIST)()
        if wlanapi.WlanEnumInterfaces(handle, None, ctypes.byref(plist)) != 0:
            return False
        try:
            if plist.contents.dwNumberOfItems < 1:
                return False
            guid = plist.contents.InterfaceInfo[0].InterfaceGuid
            rc = wlanapi.WlanScan(handle, ctypes.byref(guid), None, None, None)
        finally:
            wlanapi.WlanFreeMemory(plist)
    finally:
        wlanapi.WlanCloseHandle(handle, None)

    if rc != 0:
        return False
    time.sleep(settle)   # results arrive asynchronously
    return True


def bss_entries():
    """Every entry in the driver's list: [(bssid, ssid, host timestamp)], or
    None when the list cannot be read."""
    handle = wintypes.HANDLE()
    negotiated = wintypes.DWORD()
    if wlanapi.WlanOpenHandle(2, None, ctypes.byref(negotiated),
                              ctypes.byref(handle)) != 0:
        return None
    try:
        plist = ctypes.POINTER(WLAN_INTERFACE_INFO_LIST)()
        if wlanapi.WlanEnumInterfaces(handle, None, ctypes.byref(plist)) != 0:
            return None
        try:
            if plist.contents.dwNumberOfItems < 1:
                return None
            guid = plist.contents.InterfaceInfo[0].InterfaceGuid
            pbss = ctypes.POINTER(WLAN_BSS_LIST)()
            # Every SSID and BSS type (3, dot11_BSS_type_any).
            if wlanapi.WlanGetNetworkBssList(handle, ctypes.byref(guid), None, 3, False,
                                             None, ctypes.byref(pbss)) != 0:
                return None
            try:
                n = pbss.contents.dwNumberOfItems
                first = ctypes.addressof(pbss.contents) + WLAN_BSS_LIST.wlanBssEntries.offset
                items = ctypes.cast(first, ctypes.POINTER(WLAN_BSS_ENTRY * n)).contents
                out = []
                for e in items:
                    bssid = ":".join("%02x" % b for b in e.dot11Bssid)
                    ssid = bytes(e.dot11Ssid.ucSSID[:e.dot11Ssid.uSSIDLength]).decode("utf-8", "replace")
                    out.append((bssid, ssid, e.ullHostTimestamp))
                return out
            finally:
                wlanapi.WlanFreeMemory(pbss)
        finally:
            wlanapi.WlanFreeMemory(plist)
    finally:
        wlanapi.WlanCloseHandle(handle, None)


def networks():
    """{bssid: ssid}, each BSSID's name as last heard. An empty ssid means
    hidden."""
    entries = bss_entries()
    if entries:
        newest = {}
        for bssid, ssid, heard in entries:
            if bssid not in newest or heard > newest[bssid][1]:
                newest[bssid] = (ssid, heard)
        return dict((b, s) for b, (s, _) in newest.items())
    out = subprocess.run(["netsh", "wlan", "show", "networks", "mode=bssid"],
                         capture_output=True, text=True).stdout
    found = {}
    for block in re.split(r"^SSID \d+ : ", out, flags=re.M)[1:]:
        ssid = block.splitlines()[0].strip()
        for bssid in re.findall(r"BSSID \d+\s*:\s*(\S+)", block):
            found[bssid.lower()] = ssid
    return found


def look_for(bssid, window=60.0):
    """Scan until `bssid` appears; returns (seconds taken, its SSID) or (None,
    None). Keyed on the BSSID, so a hidden network (empty SSID) is told from a
    scan that is not running. Only entries heard after the call count."""
    bssid = bssid.lower()
    started = time.time()
    since = int((started + 11644473600) * 1e7)   # the driver's clock: 100 ns from 1601, UTC
    while time.time() - started < window:
        if not scan_now():
            time.sleep(2.0)
            continue
        entries = bss_entries()
        if entries is None:
            found = networks()
            if bssid in found:
                return round(time.time() - started, 1), found[bssid]
            continue
        fresh = [(heard, ssid) for b, ssid, heard in entries if b == bssid and heard >= since]
        if fresh:
            return round(time.time() - started, 1), max(fresh)[1]
    return None, None
