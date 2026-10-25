#!/usr/bin/env python3
"""Generate a network for the Neuro N6 NPU with ST Edge AI Core.

    python harness/n6model.py model.onnx --name nia416 --out Middlewares/NeuroN6/Models/Nia416

Runs `stedgeai generate` for the STM32N6 Neural-ART NPU against the board's
memory pools in `Utilities/npu/NeuroN6_MemPool.mpool`, with this board's
compiler options and input contract (uint8, channels last) and the epoch
controller on. The weights land in the external-flash pool at 0x90300000
unless --address moves them; for a cascade, pack the second network after the
first, 64 KB aligned, so an erase of one cannot reach the other.

Writes to --out:
  <name>.c                       the network, as ST Edge AI generated it
  <name>_ecblobs.h               its epoch-controller blobs
  <name>_atonbuf.xSPI1.raw       its weights, for external flash
  <name>_generate_report.txt     ST Edge AI's report
  <name>_model.c                 the one translation unit a project compiles
  model.json                     where it all came from, and the weights'
                                 address, size and checksums

Work paths in the generated header comment and report become <work> and
<model>, so a regenerated network differs only where the network does. The
ST Edge AI install whose NPU runtime matches Middlewares/ST/AI_Runtime is
preferred (the generated code compiles against no other); NEURON6_STEDGEAI
names the executable.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: BSD-3-Clause
# Licensed under the terms in the LICENSE file at the root of this
# repository.

import argparse
import glob
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import zlib

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
MPOOL = os.path.join(REPO, "Utilities", "npu", "NeuroN6_MemPool.mpool")

# Networks are packed on an erase block, so writing one cannot disturb another.
WEIGHTS_ALIGN = 0x10000
LL_ATON_VERSION_H = os.path.join(REPO, "Middlewares", "ST", "AI_Runtime", "Npu", "ll_aton",
                                 "ll_aton_version.h")

# The Neural-ART compiler options and input contract. The camera feeds the
# network uint8, channels last.
NEURAL_ART_OPTIONS = ("--native-float --cache-maintenance --Ocache-opt "
                      "--enable-virtual-mem-pools --Os --optimization 3 "
                      "--Omax-ca-pipe 4 --Oauto-sched --all-buffers-info")
INTERFACE_OPTIONS = "--input-data-type uint8 --inputs-ch-position chlast"


def fail(msg):
    raise SystemExit("n6model: " + msg)


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def ll_aton_version(header):
    try:
        text = open(header, encoding="utf-8", errors="replace").read()
    except OSError:
        return None
    parts = []
    for field in ("MAJOR", "MINOR", "MICRO", "DEV"):
        m = re.search(r"#define\s+LL_ATON_VERSION_%s\s+\(?\s*(\d+)" % field, text)
        if not m:
            return None
        parts.append(m.group(1))
    return "%s.%s.%s-%s" % tuple(parts)


def find_stedgeai(want):
    cands = []
    env = os.environ.get("NEURON6_STEDGEAI")
    if env:
        if not os.path.isfile(env):
            fail("NEURON6_STEDGEAI names a file that does not exist: " + env)
        cands.append(env)
    if sys.platform.startswith("win"):
        pats = [r"C:\ST\STEdgeAI\*\Utilities\windows\stedgeai.exe",
                r"C:\Program Files\STMicroelectronics\STEdgeAI\*\Utilities\windows\stedgeai.exe"]
    elif sys.platform == "darwin":
        pats = ["/Applications/STEdgeAI/*/Utilities/mac/stedgeai"]
    else:
        pats = ["/opt/ST/STEdgeAI/*/Utilities/linux/stedgeai"]
    for p in pats:
        cands.extend(sorted(glob.glob(p), reverse=True))
    if shutil.which("stedgeai"):
        cands.append(shutil.which("stedgeai"))
    first = None
    for c in cands:
        if not os.path.isfile(c):
            continue
        first = first or c
        install = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(c))))
        bundled = ll_aton_version(os.path.join(install, "Middlewares", "ST", "AI", "Npu",
                                               "ll_aton", "ll_aton_version.h"))
        if bundled == want:
            return c
    if first:
        print("n6model: no ST Edge AI install bundles NPU runtime %s; using %s, whose "
              "network may not compile against Middlewares/ST/AI_Runtime" % (want, first))
        return first
    fail("ST Edge AI Core (stedgeai) not found; install it, or set NEURON6_STEDGEAI")


def stedgeai_version(exe):
    out = subprocess.run([exe, "--version"], capture_output=True, text=True, timeout=120).stdout
    m = re.search(r"ST Edge AI Core\s+(v[\w.\-]+)", out)
    return m.group(1) if m else out.strip().splitlines()[0]


def weights_pool(mpool):
    """The external-flash pool's base address and size in bytes."""
    for p in json.load(open(mpool, encoding="utf-8"))["memory"]["mempools"]:
        if p.get("fname") == "xSPI1":
            size = int(p["size"]["value"])
            if p["size"]["magnitude"].upper().startswith("KBYTE"):
                size *= 1024
            elif p["size"]["magnitude"].upper().startswith("MBYTE"):
                size *= 1024 * 1024
            return int(p["offset"]["value"], 16), size
    fail("no xSPI1 pool in " + mpool)


def rebase_mpool(mpool, base, size, address, out_path):
    """Write a copy of the pool file that starts at `address` (ST Edge AI
    places weights at the pool's own offset), its size shrunk by as much as
    its base moved so it does not run past the region."""
    data = json.load(open(mpool, encoding="utf-8"))
    for p in data["memory"]["mempools"]:
        if p.get("fname") == "xSPI1":
            p["offset"]["value"] = "0x%08X" % address
            p["size"]["value"] = str((size - (address - base)) // 1024)
            p["size"]["magnitude"] = "KBYTES"
    json.dump(data, open(out_path, "w", encoding="utf-8"), indent=1)


def epochs(report):
    counts = {}
    patterns = (("total", r"Total number of epochs\s+(\d+)"),
                ("sw", r"pure software \(SW\) epochs\s+(\d+)"),
                ("hybrid", r"hybrid epochs.*?(\d+)\s*$"),
                ("hw", r"pure hardware \(HW or EC\) epochs\s+(\d+)"))
    for line in open(report, encoding="utf-8", errors="replace").read().splitlines():
        for key, pat in patterns:
            m = re.search(pat, line)
            if m and key not in counts:
                counts[key] = int(m.group(1))
    return counts


def neutral(path, replacements):
    text = open(path, encoding="utf-8", errors="surrogateescape", newline="").read()
    for old, new in replacements:
        for form in {old, old.replace("\\", "/"), old.replace("/", "\\")}:
            text = text.replace(form, new)
    open(path, "w", encoding="utf-8", errors="surrogateescape", newline="").write(text)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("model", help="the quantised network, ONNX or TFLite")
    ap.add_argument("--name", required=True, help="the C name ST Edge AI gives the network")
    ap.add_argument("--out", required=True, help="the directory to write into")
    ap.add_argument("--address", default=None,
                    help="where the weights go, e.g. 0x90420000. The pool's own "
                         "base by default. Give the next 64 KB boundary after "
                         "the previous network's blob to pack a cascade.")
    args = ap.parse_args()

    model = os.path.abspath(args.model)
    if not os.path.isfile(model):
        fail("no such model: " + model)
    runtime = ll_aton_version(LL_ATON_VERSION_H)
    exe = find_stedgeai(runtime)
    version = stedgeai_version(exe)
    base, pool_size = weights_pool(MPOOL)
    address = base if args.address is None else int(args.address, 0)
    if not (base <= address < (base + pool_size)):
        fail("--address 0x%08X is outside the weights pool 0x%08X..0x%08X"
             % (address, base, base + pool_size - 1))
    if (address % WEIGHTS_ALIGN) != 0:
        fail("--address 0x%08X is not on a %d KB boundary; an erase of one "
             "network would reach another" % (address, WEIGHTS_ALIGN // 1024))
    print("ST Edge AI %s (%s), NPU runtime %s" % (version, exe, runtime))

    with tempfile.TemporaryDirectory(prefix="n6model-") as work:
        pool = os.path.join(work, "NeuroN6_MemPool.mpool")
        if address == base:
            shutil.copyfile(MPOOL, pool)
        else:
            rebase_mpool(MPOOL, base, pool_size, address, pool)
        profile = os.path.join(work, "neuron6_neural_art.json")
        json.dump({"Globals": {},
                   "Profiles": {"neuron6": {"memory_pool": "./NeuroN6_MemPool.mpool",
                                            "options": NEURAL_ART_OPTIONS}}},
                  open(profile, "w", encoding="utf-8"), indent=1)
        out_dir = os.path.join(work, "out")
        cmd = [exe, "generate", "-m", model, "--target", "stm32n6",
               "--st-neural-art", "neuron6@" + profile.replace("\\", "/"),
               "-n", args.name, "--enable-epoch-controller",
               "--output", out_dir, "--workspace", os.path.join(work, "ws")]
        cmd += INTERFACE_OPTIONS.split()
        print("generating %s (a minute or two)..." % args.name)
        r = subprocess.run(cmd, cwd=work, capture_output=True, text=True, timeout=1800)
        if r.returncode != 0:
            tail = "\n".join((r.stdout + r.stderr).splitlines()[-25:])
            fail("stedgeai failed (exit %d):\n%s" % (r.returncode, tail))

        names = {"c": args.name + ".c",
                 "ecblobs": args.name + "_ecblobs.h",
                 "report": args.name + "_generate_report.txt"}
        raws = sorted(glob.glob(os.path.join(out_dir, args.name + "_atonbuf.*.raw")))
        if [os.path.basename(p) for p in raws] != [args.name + "_atonbuf.xSPI1.raw"]:
            fail("expected exactly one weights file, for xSPI1; got %s" % raws)
        names["weights"] = os.path.basename(raws[0])
        for key, n in names.items():
            if not os.path.isfile(os.path.join(out_dir, n)):
                fail("stedgeai did not write " + n)

        os.makedirs(args.out, exist_ok=True)
        replacements = [(work, "<work>"), (os.path.dirname(model), "<model>")]
        for key, n in names.items():
            dst = os.path.join(args.out, n)
            shutil.copyfile(os.path.join(out_dir, n), dst)
            if key in ("c", "report"):
                neutral(dst, replacements)

    weights = os.path.join(args.out, names["weights"])
    data = open(weights, "rb").read()
    crc32 = zlib.crc32(data) & 0xFFFFFFFF
    unit = args.name + "_model.c"
    with open(os.path.join(args.out, unit), "w", encoding="utf-8", newline="\n") as f:
        f.write("/* The network's one translation unit, compiled the way the Neuro N6\n"
                " * generated network is compiled: its epoch-controller blobs\n"
                " * go to .ext_psram_init, which the startup copies to PSRAM before use.\n"
                " * Written by harness/n6model.py. */\n"
                "#define ECBLOB_CONST_SECTION __attribute__((section(\".ext_psram_init\")))\n"
                "\n"
                "#include \"%s\"\n"
                "\n"
                "/* The weights the network was generated against, as model.json records\n"
                " * them. They are not in the image: the NPU reads them in place from\n"
                " * external flash, so an application can check that the flash still holds\n"
                " * them before it runs the network. */\n"
                "const uint32_t %s_weights_address = 0x%08Xu;\n"
                "const uint32_t %s_weights_size    = %uu;\n"
                "const uint32_t %s_weights_crc32   = 0x%08Xu;\n"
                % (names["c"], args.name, address, args.name, len(data), args.name, crc32))
    info = {
        "name": args.name,
        "model": {"file": os.path.basename(model), "sha256": sha256(model)},
        "stedgeai": version,
        "npu_runtime": runtime,
        "target": "stm32n6",
        "options": NEURAL_ART_OPTIONS,
        "interface": INTERFACE_OPTIONS,
        "epoch_controller": True,
        "mempool": {"file": "Utilities/npu/NeuroN6_MemPool.mpool", "sha256": sha256(MPOOL)},
        "weights": {"file": names["weights"], "address": "0x%08X" % address,
                    "size": len(data), "sha256": hashlib.sha256(data).hexdigest(),
                    "crc32": "0x%08X" % crc32},
        "epochs": epochs(os.path.join(args.out, names["report"])),
        "files": {"c": names["c"], "ecblobs": names["ecblobs"],
                  "report": names["report"], "unit": unit},
    }
    with open(os.path.join(args.out, "model.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(info, f, indent=1)
        f.write("\n")
    print("wrote %s: weights %d bytes at %s, CRC32 %s, epochs %s"
          % (args.out, len(data), info["weights"]["address"], info["weights"]["crc32"],
             info["epochs"]))


main()
