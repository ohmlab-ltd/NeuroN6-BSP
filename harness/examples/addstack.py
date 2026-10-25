"""Add an example's middleware stacks (the NPU runtime and its network, the
Wi-Fi stack) to its CubeIDE project, and wire the project's makefiles for
them.

    python addstack.py "NeuroN6 - Ex Camera_Nia416Stream_WiFi_FreeRTOS" Camera_Nia416Stream_WiFi_FreeRTOS
    python addstack.py --flags Camera_Nia416Stream_WiFi_FreeRTOS
    python addstack.py --sources Camera_Nia416Stream_WiFi_FreeRTOS

Each *_build.json in the example describes one stack: the middleware trees
and example files to copy, the files that compile and their flags, the
switches, include paths, libraries and linker flags the project needs, and
the BSP modules and HAL files (from the project's STM32Cube package, or the
patched copies under Utilities/hal). Each stack compiles in makefile
fragments of its own; existing compiles gain its switches, and its include
paths after their own. Running it again refreshes the files and adds only
switches and include paths a stack has gained since.

--flags and --sources copy nothing: they print the compile flags or the
stack sources, for harness/compile.sh.

Environment:
  NEURON6_CUBE_DIR       the directory holding the CubeIDE projects
  STM32CUBE_REPOSITORY   by default ~/STM32Cube/Repository
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: BSD-3-Clause
# Licensed under the terms in the LICENSE file at the root of this
# repository.

import filecmp
import glob
import io
import json
import os
import re
import shutil
import sys

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
CUBE = os.environ.get("NEURON6_CUBE_DIR", "")

# Marks the makefile fragments this script writes, so a later run knows them.
MARK = "# Written by harness/examples/addstack.py"

# Example files that are not this script's to place: the project's own
# configuration is copied by mkexample.py and left alone by sync2.py.
EXAMPLE_SKIP = {"Core/Inc/neuron6_conf.h"}
EXAMPLE_DIRS = ("Core", "Utilities", "Middlewares")

HAL_DIR = "Drivers/STM32N6xx_HAL_Driver"


def read(p):
    return io.open(p, encoding="utf-8", newline="").read()


def write(p, s):
    io.open(p, "w", encoding="utf-8", newline="").write(s)


def fail(msg):
    raise SystemExit("addstack: " + msg)


def ours(text):
    return MARK in text


def load(example):
    """The example's stacks, as (name, manifest), in name order."""
    paths = sorted(glob.glob(os.path.join(REPO, "Examples", example, "*_build.json")))
    if not paths:
        fail("no *_build.json in Examples/" + example)
    return [(os.path.basename(p)[:-len("_build.json")], json.loads(read(p))) for p in paths]


def cflags_of(manifest):
    return manifest.get("cflags") or manifest.get("npu_cflags")


def merged(stacks, key):
    """A list-valued key across the stacks, first mention first."""
    out = []
    for _name, m in stacks:
        for v in m.get(key, []):
            if v not in out:
                out.append(v)
    return out


def copy_file(src, dst, changed):
    if not os.path.isfile(src):
        fail("missing: " + src)
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    if os.path.exists(dst) and filecmp.cmp(src, dst, shallow=False):
        return
    shutil.copyfile(src, dst)
    changed.append(dst)


def copy_tree(src_root, dst_root, changed, skip=()):
    for d, dirs, files in os.walk(src_root):
        dirs.sort()
        for n in sorted(files):
            src = os.path.join(d, n)
            rel = os.path.relpath(src, src_root).replace("\\", "/")
            if rel in skip or "__pycache__" in rel:
                continue
            copy_file(src, os.path.join(dst_root, rel), changed)


def copy_parts(stacks, example, appli, changed):
    """The middleware trees, then the example's own files."""
    for tree in merged(stacks, "middlewares"):
        tree = tree.rstrip("/")
        copy_tree(os.path.join(REPO, tree), os.path.join(appli, tree), changed)
    own = os.path.join(REPO, "Examples", example)
    for sub in EXAMPLE_DIRS:
        if os.path.isdir(os.path.join(own, sub)):
            copy_tree(os.path.join(own, sub), os.path.join(appli, sub), changed,
                      skip={s[len(sub) + 1:] for s in EXAMPLE_SKIP if s.startswith(sub + "/")})


def cube_package(project_dir):
    """The STM32Cube package whose HAL the project was generated with."""
    proj_hal = os.path.join(project_dir, HAL_DIR, "Src", "stm32n6xx_hal.c")
    repo = os.environ.get("STM32CUBE_REPOSITORY") or os.path.join(
        os.path.expanduser("~"), "STM32Cube", "Repository")
    for pkg in sorted(glob.glob(os.path.join(repo, "STM32Cube_FW_N6_V*")), reverse=True):
        cand = os.path.join(pkg, HAL_DIR, "Src", "stm32n6xx_hal.c")
        if os.path.isfile(cand) and filecmp.cmp(cand, proj_hal, shallow=False):
            return pkg
    fail("no STM32Cube N6 package in %s has the project's HAL" % repo)


def hal_names(stacks):
    """Every HAL file the stacks need, as stm32n6xx_<name>: the modules, which
    also get enabled, then the other files."""
    names = ["hal_" + m for m in merged(stacks, "hal_modules")]
    names += [f for f in merged(stacks, "hal_files") if f not in names]
    return names


def add_hal(project_dir, appli, stacks, changed):
    names = hal_names(stacks)
    if not names:
        return None, []
    patched = {}
    for _name, m in stacks:
        patched.update({k: v for k, v in m.get("hal_patched", {}).items() if k != "about"})
    pkg = cube_package(project_dir)
    hal = os.path.join(pkg, HAL_DIR)
    sources = []
    for n in names:
        c_name = "stm32n6xx_%s.c" % n
        src = os.path.join(REPO, patched[c_name]) if c_name in patched else os.path.join(hal, "Src", c_name)
        copy_file(src, os.path.join(appli, HAL_DIR, c_name), changed)
        copy_file(os.path.join(hal, "Inc", "stm32n6xx_%s.h" % n),
                  os.path.join(project_dir, HAL_DIR, "Inc", "stm32n6xx_%s.h" % n), changed)
        sources.append(c_name)
    unknown = set(patched) - set(sources)
    if unknown:
        fail("patched HAL files no stack compiles: " + ", ".join(sorted(unknown)))
    conf_path = os.path.join(appli, "Core", "Inc", "stm32n6xx_hal_conf.h")
    conf = read(conf_path)
    new = conf
    for m in merged(stacks, "hal_modules"):
        name = "HAL_%s_MODULE_ENABLED" % m.upper()
        new = re.sub(r"/\*\s*#define\s+%s\s*\*/" % name, "#define %s" % name, new)
        if not re.search(r"^\s*#define\s+%s\b" % name, new, re.M):
            fail("could not enable %s in %s" % (name, conf_path))
    if new != conf:
        write(conf_path, new)
        changed.append(conf_path)
    return os.path.basename(pkg), sources


def add_bsp(appli, stacks, changed):
    for n in merged(stacks, "bsp_sources") + merged(stacks, "bsp_headers"):
        copy_file(os.path.join(REPO, n), os.path.join(appli, "BSP", n), changed)


# Makefiles.

def quote_define(d):
    """A switch as a makefile recipe passes it to the shell. One whose value
    is a quoted string, such as a header name, goes in single quotes, as
    CubeIDE writes it."""
    arg = "-D" + d
    if '"' in arg:
        return "'%s'" % arg
    return ('"%s"' % arg) if " " in arg else arg


def mk_tag(reldir):
    return reldir.replace("/", "-2f-")


def add_to_fragment(path, objdir, srcdir, names):
    """Append sources that the fragment's own pattern rule compiles."""
    s = read(path)
    names = [n for n in names if ("./%s/%s.o" % (objdir, n[:-2])) not in s]
    if not names:
        return
    s = s.rstrip("\n") + "\n\n"
    s += "C_SRCS += \\\n" + " \\\n".join("%s/%s" % (srcdir, n) for n in names) + " \n\n"
    s += "OBJS += \\\n" + " \\\n".join("./%s/%s.o" % (objdir, n[:-2]) for n in names) + " \n\n"
    s += "C_DEPS += \\\n" + " \\\n".join("./%s/%s.d" % (objdir, n[:-2]) for n in names) + " \n\n"
    write(path, s)


def patch_recipe(line, defines, includes):
    """Add the switches this recipe lacks after -DSTM32N657xx, and the include
    paths it lacks after its last one."""
    new_defs = [d for d in defines if (" " + d + " ") not in (" " + line + " ")]
    if new_defs:
        line = line.replace("-DSTM32N657xx", "-DSTM32N657xx " + " ".join(new_defs), 1)
    new = [i for i in includes if (" " + i + " ") not in (" " + line + " ")]
    if new:
        last = None
        for m in re.finditer(r"-I\S+", line):
            last = m
        if last is None:
            fail("no include path in: " + line[:120])
        line = line[:last.end()] + " " + " ".join(new) + line[last.end():]
    return line


def stack_recipe(line, cflags):
    line = line.replace("-std=gnu11", cflags["std"], 1)
    opt = cflags["optimisation"] + ((" " + cflags["extra"]) if cflags.get("extra") else "")
    line = re.sub(r" -O\S+", " " + opt, line, count=1)
    line = line.replace(" -DDEBUG", "", 1)
    line = line.replace(" -Wall", (" " + cflags["warnings"]) if cflags["warnings"] else "", 1)
    return line


def write_fragment(debug, reldir, names, recipe, stack):
    s = ("################################################################################\n"
         "%s: part of the %s stack,\n"
         "# compiled with that stack's own flags. Not generated by CubeIDE.\n"
         "################################################################################\n\n"
         % (MARK, stack))
    s += "C_SRCS += \\\n" + " \\\n".join("../%s/%s.c" % (reldir, n) for n in names) + " \n\n"
    s += "OBJS += \\\n" + " \\\n".join("./%s/%s.o" % (reldir, n) for n in names) + " \n\n"
    s += "C_DEPS += \\\n" + " \\\n".join("./%s/%s.d" % (reldir, n) for n in names) + " \n\n\n"
    s += "# Each subdirectory must supply rules for building sources it contributes\n"
    s += "%s/%%.o %s/%%.su %s/%%.cyclo: ../%s/%%.c %s/subdir.mk\n" % (
        reldir, reldir, reldir, reldir, reldir)
    s += recipe + "\n\n"
    tag = mk_tag(reldir)
    s += "clean: clean-%s\n\nclean-%s:\n\t-$(RM) %s\n\n.PHONY: clean-%s\n\n" % (
        tag, tag, " ".join("./%s/%s.%s" % (reldir, n, e)
                           for n in names for e in ("cyclo", "d", "o", "su")), tag)
    path = os.path.join(debug, reldir, "subdir.mk")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if not os.path.exists(path) or read(path) != s:
        write(path, s)
    return ["./%s/%s.o" % (reldir, n) for n in names]


def compile_units(manifest, root):
    """{directory relative to root: [source names]} for a stack's compile
    globs, which may use ** for a whole tree."""
    units = {}
    for pattern in manifest["compile"]:
        hits = sorted(glob.glob(os.path.join(root, *pattern.split("/")), recursive=True))
        if not hits:
            fail("nothing under %s matches %s" % (root, pattern))
        for h in hits:
            rel = os.path.relpath(h, root).replace("\\", "/")
            names = units.setdefault(os.path.dirname(rel), [])
            if os.path.basename(rel) not in names:
                names.append(os.path.basename(rel))
    return units


def wire_makefiles(appli, stacks, hal_sources):
    debug = os.path.join(appli, "Debug")
    defines = [quote_define(d) for d in merged(stacks, "defines")]
    includes = ["-I../" + d for d in merged(stacks, "include_dirs")]
    boards = [m["board_defines"] for _n, m in stacks if "board_defines" in m]

    # Every compile the project already has.
    core_src_recipe = None
    for d, _dirs, files in os.walk(debug):
        for n in files:
            if n != "subdir.mk":
                continue
            p = os.path.join(d, n)
            rel = os.path.relpath(p, debug).replace("\\", "/")
            s = read(p)
            if ours(s):
                continue
            lines = s.split("\n")
            for i, line in enumerate(lines):
                if line.startswith("\tarm-none-eabi-gcc") and "-DSTM32N657xx" in line:
                    line = patch_recipe(line, defines, includes)
                    for board in boards:
                        if rel == board["dir"] + "/subdir.mk":
                            line = patch_recipe(line, [quote_define(x) for x in board["defines"]], [])
                    lines[i] = line
                    if rel == "Core/Src/subdir.mk":
                        core_src_recipe = line
            t = "\n".join(lines)
            if t != s:
                write(p, t)
    if core_src_recipe is None:
        fail("no compile rule in Debug/Core/Src/subdir.mk")

    # Each stack's code, a fragment for each directory, with the stack's
    # flags. A directory CubeIDE already builds keeps its own rule and takes
    # the sources into it.
    new_objs = []
    fragments = []
    claimed = {}
    for name, m in stacks:
        recipe = stack_recipe(core_src_recipe, cflags_of(m))
        units = compile_units(m, appli)
        for reldir in sorted(units):
            if reldir in claimed and claimed[reldir] != name:
                fail("%s is compiled by both the %s and the %s stacks" % (reldir, claimed[reldir], name))
            claimed[reldir] = name
            names = sorted(n[:-2] for n in units[reldir])
            frag = os.path.join(debug, reldir, "subdir.mk")
            if os.path.exists(frag) and not ours(read(frag)):
                add_to_fragment(frag, reldir, "../" + reldir, [n + ".c" for n in names])
                new_objs += ["./%s/%s.o" % (reldir, n) for n in names]
            else:
                new_objs += write_fragment(debug, reldir, names, recipe, name)
                fragments.append(reldir)

    # The BSP modules and the HAL files, compiled by their folders' own rules.
    bsp = merged(stacks, "bsp_sources")
    add_to_fragment(os.path.join(debug, "BSP", "subdir.mk"), "BSP", "../BSP", bsp)
    new_objs += ["./BSP/%s.o" % n[:-2] for n in bsp]
    add_to_fragment(os.path.join(debug, *HAL_DIR.split("/"), "subdir.mk"),
                    HAL_DIR, "../" + HAL_DIR, hal_sources)
    new_objs += ["./%s/%s.o" % (HAL_DIR, n[:-2]) for n in hal_sources]

    # The top-level makefile reads the new fragments, and links with the
    # extra flags.
    mk_path = os.path.join(debug, "makefile")
    mk = read(mk_path)
    add = "".join("-include %s/subdir.mk\n" % d for d in fragments
                  if ("-include %s/subdir.mk" % d) not in mk)
    if add:
        mk = mk.replace("-include objects.mk\n", add + "-include objects.mk\n", 1)
    for _name, m in stacks:
        ld = m.get("ldflags")
        if ld and ld not in mk:
            mk, n = re.subn(r"(\n\tarm-none-eabi-gcc -o [^\n]*?--specs=nano\.specs)",
                            r"\1 " + ld, mk, count=1)
            if n != 1:
                fail("no link rule in " + mk_path)
    write(mk_path, mk)

    src_path = os.path.join(debug, "sources.mk")
    sm = read(src_path)
    add = "".join("%s \\\n" % d for d in fragments if ("\n%s \\" % d) not in sm)
    if add:
        sm = sm.replace("SUBDIRS := \\\n", "SUBDIRS := \\\n" + add, 1)
        write(src_path, sm)

    # The object list the link reads, and the libraries after it.
    ol_path = os.path.join(debug, "objects.list")
    ol = read(ol_path)
    have = set(l.strip() for l in ol.splitlines())
    extra = ["\"%s\"" % o for o in new_objs if ("\"%s\"" % o) not in have]
    if extra:
        write(ol_path, ol.rstrip("\n") + "\n" + "\n".join(extra) + "\n")

    om_path = os.path.join(debug, "objects.mk")
    om = read(om_path)
    libs = " ".join("../" + l for l in merged(stacks, "libs"))
    if libs and ("LIBS := " + libs + "\n") not in om:
        om, n = re.subn(r"^LIBS :=[^\n]*", "LIBS := " + libs, om, count=1, flags=re.M)
        if n != 1:
            fail("no LIBS in " + om_path)
        write(om_path, om)


def repo_root(example, path):
    """Stack paths name the repository's middleware and utilities, or the
    example's own files."""
    if path.startswith(("Middlewares/", "Utilities/")):
        return REPO
    return os.path.join(REPO, "Examples", example)


def gate_flags(example):
    """The switches and include paths for compiling the example's sources in
    this repository, as -isystem so every -I the gate gives comes first."""
    stacks = load(example)
    flags = ["-D" + d for d in merged(stacks, "defines")]
    for d in merged(stacks, "include_dirs"):
        p = os.path.join(repo_root(example, d), *d.split("/"))
        if not os.path.isdir(p):
            fail("no such directory: " + p)
        flags += ["-isystem", os.path.relpath(p, REPO).replace("\\", "/")]
    return " ".join(flags)


def gate_sources(example):
    """The files the example's stacks compile, as repository paths, each after
    the language flags it is compiled with and a '|'."""
    out = []
    for _name, m in load(example):
        cflags = cflags_of(m)
        lang = " ".join(f for f in (cflags["std"], cflags.get("extra")) if f)
        for pattern in m["compile"]:
            root = repo_root(example, pattern)
            hits = sorted(glob.glob(os.path.join(root, *pattern.split("/")), recursive=True))
            if not hits:
                fail("nothing in the repository matches " + pattern)
            out += ["%s|%s" % (lang, os.path.relpath(h, REPO).replace("\\", "/")) for h in hits]
    return out


def main():
    if len(sys.argv) == 3 and sys.argv[1] == "--flags":
        print(gate_flags(sys.argv[2]))
        return
    if len(sys.argv) == 3 and sys.argv[1] == "--sources":
        print("\n".join(gate_sources(sys.argv[2])))
        return
    if not CUBE:
        fail("set NEURON6_CUBE_DIR to the directory holding the CubeIDE projects")
    project, example = sys.argv[1], sys.argv[2]
    stacks = load(example)
    project_dir = os.path.join(CUBE, project)
    appli = os.path.join(project_dir, "Appli")

    changed = []
    copy_parts(stacks, example, appli, changed)
    pkg, hal_sources = add_hal(project_dir, appli, stacks, changed)
    add_bsp(appli, stacks, changed)
    wire_makefiles(appli, stacks, hal_sources)

    print("stacks: %s" % ", ".join(n for n, _m in stacks))
    if pkg:
        print("HAL files %s from %s" % (", ".join(hal_sources), pkg))
    print("%d files added or refreshed" % len(changed))
    for c in changed:
        print("  %s" % os.path.relpath(c, project_dir))


main()
