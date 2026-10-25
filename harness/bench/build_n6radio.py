#!/usr/bin/env python3
"""Build n6radio.exe, the bench's BLE and Wi-Fi Direct fixture (Windows).

    python harness/bench/build_n6radio.py

Compiles n6radio.cs with the .NET Framework's C# compiler against the Windows
Runtime metadata in System32 (nothing is downloaded), to n6bench/n6radio.exe
in the temp folder. The bench scripts call this first; it returns at once
when the exe is up to date.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT

import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "n6radio.cs")
OUT = os.path.join(tempfile.gettempdir(), "n6bench", "n6radio.exe")

WINDIR = os.environ.get("WINDIR", r"C:\Windows")
FRAMEWORK = os.path.join(WINDIR, "Microsoft.NET", "Framework64", "v4.0.30319")
WINMD = os.path.join(WINDIR, "System32", "WinMetadata")


def build(force=False):
    if not force and os.path.exists(OUT) and os.path.getmtime(OUT) >= os.path.getmtime(SRC):
        return OUT
    csc = os.path.join(FRAMEWORK, "csc.exe")
    if not os.path.exists(csc):
        sys.exit("no C# compiler at %s" % csc)
    refs = [os.path.join(FRAMEWORK, n) for n in (
        "System.Runtime.dll", "System.Threading.Tasks.dll",
        "System.Runtime.InteropServices.WindowsRuntime.dll")]
    refs += [os.path.join(WINMD, n) for n in (
        "Windows.Foundation.winmd", "Windows.Devices.winmd", "Windows.Storage.winmd",
        "Windows.Security.winmd", "Windows.Networking.winmd")]
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    cmd = [csc, "/nologo", "/target:exe", "/optimize+", "/out:" + OUT]
    cmd += ["/r:" + r for r in refs] + [SRC]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        sys.stdout.write(r.stdout + r.stderr)
        sys.exit("n6radio.cs did not compile")
    return OUT


if __name__ == "__main__":
    print(build(force="--force" in sys.argv))
