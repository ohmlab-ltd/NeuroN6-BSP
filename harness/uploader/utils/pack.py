"""Framing helpers for the Neuro N6 DFU wire protocol.

Wraps each piece of a firmware image in the LEN16 frame the bootloader
expects (see make_packet below for the layout). Shared by every uploader
entry point in this folder.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
# Licensed under the terms in harness/uploader/LICENSE.txt.

import struct

STX = 0x02
ETX = 0x03

# Packet format (LEN16):
# Byte 0        = STX (0x02)
# Byte 1..2     = payload length (uint16 LE)
# Byte 3..      = payload bytes
# Byte end-1    = checksum (XOR of payload bytes)
# Byte end      = ETX (0x03)
def make_packet(data: bytes):
    length = len(data)
    if length > 0xFFFF:
        raise ValueError(f"Payload too large for LEN16: {length} bytes")

    checksum = 0
    for b in data:
        checksum ^= b

    return bytes([STX]) + struct.pack("<H", length) + data + bytes([checksum, ETX])

def pack(chunks: list):
    return [make_packet(chunk) for chunk in chunks]

def calculate_erase_size(binary_size: int, sector_size: int):
    return ((binary_size + sector_size - 1) // sector_size) * sector_size
