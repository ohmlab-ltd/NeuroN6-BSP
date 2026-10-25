"""Copy the BSP and an example's application into a CubeIDE project.

    python sync2.py "NeuroN6 - JPEG USB Vendor" Camera_JpegStream_USB

Refreshes only files the project already has under Appli/BSP, Appli/Core,
Appli/Utilities and Appli/Middlewares; its other files (generated HAL, CubeMX
output) and its linker script are left alone. Missing middleware trees are
added by addstack.py, or by hand as Utilities/freertos/README.md describes.
An example's own USB stack (TinyUSB configuration and descriptors under
Utilities, patched TinyUSB under Middlewares) is copied over the shared one
afterwards.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: BSD-3-Clause
# Licensed under the terms in the LICENSE file at the root of this
# repository.

import filecmp
import os
import shutil
import sys

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
CUBE = os.environ.get("NEURON6_CUBE_DIR", "")


def copy_tree(src_root, dst_root, changed, skip=()):
    """Copy every file of dst_root that also exists under src_root."""
    for root, _dirs, files in os.walk(dst_root):
        for f in files:
            dst = os.path.join(root, f)
            rel = os.path.relpath(dst, dst_root)
            if rel in skip:
                continue
            src = os.path.join(src_root, rel)
            if not os.path.exists(src):
                continue
            if filecmp.cmp(src, dst, shallow=False):
                continue
            shutil.copyfile(src, dst)
            changed.append(rel)


def main():
    project, example = sys.argv[1], sys.argv[2]
    appli = os.path.join(CUBE, project, "Appli")
    changed = []
    copy_tree(REPO, os.path.join(appli, "BSP"), changed)
    # Examples/<name>/Core maps to Appli/Core (misaligned roots would silently
    # copy nothing). neuron6_conf.h is the project's own and is not copied.
    copy_tree(os.path.join(REPO, "Examples", example, "Core"),
              os.path.join(appli, "Core"), changed,
              skip=(os.path.normpath("Inc/neuron6_conf.h"),))
    # Utilities and Middlewares (FreeRTOS, TinyUSB, the NPU runtime and the
    # post-processing) come from the repository wherever the project has
    # them, except where the example carries its own copy of a file.
    for sub in ("Utilities", "Middlewares"):
        own = os.path.join(REPO, "Examples", example, sub)
        overridden = set()
        for root, _dirs, files in os.walk(own):
            overridden.update(os.path.relpath(os.path.join(root, f), own) for f in files)
        copy_tree(os.path.join(REPO, sub), os.path.join(appli, sub), changed, skip=overridden)
    for sub in ("Utilities", "Middlewares"):
        copy_tree(os.path.join(REPO, "Examples", example, sub),
                  os.path.join(appli, sub), changed)
    if not changed:
        print("project already matches the repository")
    for c in changed:
        print("updated %s" % c)


main()
