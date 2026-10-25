"""Shared helpers for the Neuro N6 uploader tools.

    device.py  USB identities (VID/PID) and COM-port discovery.
    dfu.py     Bootloader session: capability probe, erase/write/verify, reset.
    pack.py    Wire framing - chunking and LEN16 packet construction.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
# Licensed under the terms in harness/uploader/LICENSE.txt.
