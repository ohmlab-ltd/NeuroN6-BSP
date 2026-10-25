"""Make a CubeIDE project for one standalone example, and build it.

    python mkexample.py Blink
    python mkexample.py UvcStream --from "NeuroN6 - JPEG USB Vendor"

Clones a project whose recovery paths are known to work (the bench console
unless --from names another) into "NeuroN6 - Ex <Name>", drops build outputs,
renames the project, and puts the example's main.c and neuron6_conf.h in
place of the source project's. Everything else (fault handlers, startup file,
the whole BSP) stays as the source has it; the linker drops what the example
does not call. Running it again only refreshes main.c and rebuilds.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: BSD-3-Clause
# Licensed under the terms in the LICENSE file at the root of this
# repository.

import glob
import io
import os
import shutil
import subprocess
import sys

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
CUBE = os.environ.get("NEURON6_CUBE_DIR", "")
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SRC = "NeuroN6 - BSPTest"


def read(p):
    return io.open(p, encoding="utf-8", newline="").read()


def write(p, s):
    io.open(p, "w", encoding="utf-8", newline="").write(s)


def clone(dst_name, src_name):
    src = os.path.join(CUBE, src_name)
    dst = os.path.join(CUBE, dst_name)

    def ignore(d, names):
        return [n for n in names
                if n.endswith((".o", ".d", ".su", ".cyclo", ".elf", ".map", ".bin", ".list"))
                and n != "objects.list"]
    shutil.copytree(src, dst, ignore=ignore)

    def esc(n):
        return n.replace(" ", "\\ ")

    def rename_in(p):
        s = read(p)
        while esc(dst_name) in s:
            s = s.replace(esc(dst_name), esc(src_name))
        while dst_name in s:
            s = s.replace(dst_name, src_name)
        s = s.replace(esc(src_name), esc(dst_name)).replace(src_name, dst_name)
        write(p, s)

    old_ioc = os.path.join(dst, src_name + ".ioc")
    if os.path.exists(old_ioc):
        os.rename(old_ioc, os.path.join(dst, dst_name + ".ioc"))
    for rel in [".project", "Appli/.project", "Appli/.cproject", "FSBL/.project",
                "FSBL/.cproject", dst_name + ".ioc", ".mxproject"]:
        p = os.path.join(dst, rel)
        if os.path.exists(p):
            rename_in(p)
    for sub in ("Appli", "FSBL"):
        dbg = os.path.join(dst, sub, "Debug")
        for root, _d, files in os.walk(dbg):
            for f in files:
                if f.endswith(".mk") or f == "makefile":
                    rename_in(os.path.join(root, f))
    print("cloned %s -> %s" % (src_name, dst_name))


def main():
    if not CUBE:
        raise SystemExit("set NEURON6_CUBE_DIR to the directory holding the CubeIDE projects")
    argv = sys.argv[1:]
    src_name = DEFAULT_SRC
    if "--from" in argv:
        i = argv.index("--from")
        src_name = argv[i + 1]
        del argv[i:i + 2]
    name = argv[0]
    dst_name = "NeuroN6 - Ex " + name
    dst = os.path.join(CUBE, dst_name)
    if not os.path.exists(dst):
        clone(dst_name, src_name)

    # The example is its main.c and the neuron6_conf.h it is written against
    # (which decides, among other things, whether USB enumerates a camera).
    src_main = os.path.join(REPO, "Examples", name, "Core", "Src", "main.c")
    dst_main = os.path.join(dst, "Appli", "Core", "Src", "main.c")
    shutil.copyfile(src_main, dst_main)
    print("main.c in place")

    src_conf = os.path.join(REPO, "Examples", name, "Core", "Inc", "neuron6_conf.h")
    if os.path.exists(src_conf):
        shutil.copyfile(src_conf, os.path.join(dst, "Appli", "Core", "Inc", "neuron6_conf.h"))
        print("neuron6_conf.h in place")

    # The BSP itself, refreshed from the repository the way the console's is.
    subprocess.check_call([sys.executable, os.path.join(HERE, "sync2.py"), dst_name, name])

    # Middleware stacks the example lists in *_build.json files, which the
    # base projects lack.
    if glob.glob(os.path.join(REPO, "Examples", name, "*_build.json")):
        subprocess.check_call([sys.executable, os.path.join(HERE, "addstack.py"), dst_name, name])

    r = subprocess.run(["bash", os.path.join(HERE, "build.sh"), dst_name],
                       capture_output=True, text=True)
    tail = "\n".join(r.stdout.splitlines()[-4:])
    errs = [l for l in (r.stdout + r.stderr).splitlines() if " error" in l.lower()]
    if r.returncode != 0 or errs:
        print("BUILD FAILED")
        print("\n".join(errs[:12]))
        print(tail)
        sys.exit(1)
    img = os.path.join(dst, "Appli", "Debug", dst_name + "_Appli_Trusted.bin")
    print("built %s (%d bytes)" % (os.path.basename(img), os.path.getsize(img)))


main()
