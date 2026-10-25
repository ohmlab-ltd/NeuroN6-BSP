"""Neuro N6 USB identities and port discovery.

On the pid.codes open-hardware VID 0x1209:

    app        = 0x1209:0x8008   (Utilities/usb/usb_descriptors.c)
    bootloader = 0x1209:0x8009   (FSBL >= v2.0)

Older identities, recognised when the port is named:

    0xCAFE:0x4013 - an old application, and an old FSBL in bootloader mode
    0xCAFE:0x4014 - a bootloader identity no released firmware uses

No current application uses 0xCAFE, so a 0xCAFE device that appears once an
application's port has gone is a bootloader.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
# Licensed under the terms in harness/uploader/LICENSE.txt.

import time
import serial.tools.list_ports

# Current identities.
VID = 0x1209                # pid.codes open-hardware VID

APP_VID = VID
APP_PID = 0x8008            # running application (Neuro Studio / video stream)

BOOTLOADER_VID = VID
BOOTLOADER_PID = 0x8009     # FSBL DFU bootloader (v2.0+)

# Older identities.
LEGACY_VID = 0xCAFE
LEGACY_APP_PID = 0x4013         # old app; ALSO the old FSBL's identity
LEGACY_BOOTLOADER_PID = 0x4014  # old planned distinct-bootloader PID

# Match lists, preferred identity first.
APP_IDENTITIES = (
    (APP_VID, APP_PID),
    (LEGACY_VID, LEGACY_APP_PID),      # boards still running older firmware
)
BOOTLOADER_IDENTITIES = (
    (BOOTLOADER_VID, BOOTLOADER_PID),  # FSBL v2.0+
    (LEGACY_VID, LEGACY_BOOTLOADER_PID),
    (LEGACY_VID, LEGACY_APP_PID),      # old FSBL enumerates like the old app
)


def _find_port_multi(identities):
    for vid, pid in identities:
        for p in serial.tools.list_ports.comports():
            if p.vid == vid and p.pid == pid:
                return p.device
    return None


def find_app_port():
    """COM port of a running application (current or legacy IDs), or None.
    0xCAFE:0x4013 may also be an old FSBL in bootloader mode; callers handle
    that (the 'b' reboot is a no-op there)."""
    return _find_port_multi(APP_IDENTITIES)


def find_bootloader_port():
    """COM port of a DFU bootloader (current or legacy IDs), or None.

    Prefers the distinct-PID identities; 0xCAFE:0x4013 (an old FSBL) is
    matched LAST since it is ambiguous with the old app."""
    return _find_port_multi(BOOTLOADER_IDENTITIES)


def is_bootloader_family(p):
    """True if a pyserial port entry is plausibly the bootloader: a known
    bootloader identity, or anything on the legacy VID (no app uses it)."""
    return ((p.vid, p.pid) in BOOTLOADER_IDENTITIES) or (p.vid == LEGACY_VID)


def wait_for_port_gone(port, timeout_sec=5.0, poll=0.05):
    """Wait until `port` is no longer enumerated (the app disconnecting on reset).
    Returns True once gone (or immediately if `port` is None)."""
    if port is None:
        return True
    start = time.time()
    while (time.time() - start) < timeout_sec:
        if port not in [p.device for p in serial.tools.list_ports.comports()]:
            return True
        time.sleep(poll)
    return False


def wait_for_bootloader_port(timeout_sec=10.0, poll=0.1):
    """Poll until a bootloader identity enumerates; return its port or None."""
    start = time.time()
    while (time.time() - start) < timeout_sec:
        port = find_bootloader_port()
        if port:
            return port
        time.sleep(poll)
    return None
