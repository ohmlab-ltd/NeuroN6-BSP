#!/bin/sh
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
#
# Build the boot loader's own source for the PC and run it on the simulated
# board (README.md). Linux, or WSL on Windows:
#
#   sh harness/bench/hosttest/fsbl/run.sh            everything
#   sh harness/bench/hosttest/fsbl/run.sh update     one test, by its name
#   sh harness/bench/hosttest/fsbl/run.sh fuzz [runs [seed [run]]]
#                                                    the loader driven at random
#   sh harness/bench/hosttest/fsbl/run.sh uploader [test]
#                                                    the host uploader against it
#   sh harness/bench/hosttest/fsbl/run.sh usb        its USB descriptors
#
# FSBL_MAIN=<file> and FSBL_USB=<file> run another copy of the loader's main.c
# or usb_descriptors.c, e.g. to show a test failing without a fix.
# FSBL_NO_UPLOADER leaves the uploader's tests out of a run of everything.
set -e
cd "$(dirname "$0")"
CC="${CC:-gcc}"
OUT="${TMPDIR:-/tmp}/fsbl_sim"
REPO=../../../..
EXTMEM=$REPO/FSBL/Middlewares/ST/STM32_ExtMem_Manager
MAIN="${FSBL_MAIN:-$REPO/FSBL/Core/Src/main.c}"
USB="${FSBL_USB:-$REPO/FSBL/Core/Src/usb_descriptors.c}"

# The real device, HAL and memory-manager headers assume 32-bit pointers:
# sound here, as every address the loader uses is mapped below 4 GB.
CFLAGS="-std=gnu11 -O1 -g -Wall -Wno-attributes -Wno-unused-function"
CFLAGS="$CFLAGS -Wno-pointer-to-int-cast -Wno-int-to-pointer-cast -Wno-format"
CFLAGS="$CFLAGS -DSTM32N657xx -DUSE_HAL_DRIVER -D__ARM_FEATURE_CMSE=3"
CFLAGS="$CFLAGS -Ifake -I. -I$REPO/FSBL/Core/Inc"
CFLAGS="$CFLAGS -I$REPO/Drivers/STM32N6xx_HAL_Driver/Inc"
CFLAGS="$CFLAGS -I$REPO/Drivers/CMSIS/Device/ST/STM32N6xx/Include"
CFLAGS="$CFLAGS -I$EXTMEM -I$EXTMEM/boot -I$EXTMEM/sal -I$EXTMEM/nor_sfdp"
CFLAGS="$CFLAGS -I$EXTMEM/psram -I$EXTMEM/sdcard -I$EXTMEM/user"

$CC $CFLAGS -Dmain=fsbl_main -c "$MAIN" -o "$OUT.main.o"
$CC $CFLAGS -c fsbl_sim.c -o "$OUT.sim.o"
$CC $CFLAGS -c fsbl_pty.c -o "$OUT.pty.o"
BOARD="$OUT.sim.o $OUT.pty.o $OUT.main.o"
$CC $CFLAGS test_fsbl.c $BOARD -o "$OUT"
$CC $CFLAGS fuzz_fsbl.c $BOARD -o "$OUT.fuzz"
$CC $CFLAGS fsbl_serve.c $BOARD -o "$OUT.serve"
# The loader's USB descriptors, against its own TinyUSB headers.
$CC -std=gnu11 -O1 -Wall -I$REPO/FSBL/src -I$REPO/FSBL/Core/Inc test_usb_descriptors.c "$USB" -o "$OUT.usb"

uploader() {
  if python3 -c 'import serial' 2>/dev/null; then
    python3 test_uploader.py "$OUT.serve" "$@"
  else
    echo "SKIP  the uploader against the loader: python3 with pyserial is not installed"
  fi
}

rc=0
case "${1:-everything}" in
  everything)
    "$OUT" || rc=1
    echo "SKIP  the loader driven at random: its rules are the next boot loader's (README.md); run.sh fuzz runs it"
    "$OUT.usb" || rc=1
    [ -n "$FSBL_NO_UPLOADER" ] || uploader || rc=1
    ;;
  fuzz)     shift; "$OUT.fuzz" "$@" || rc=1 ;;
  uploader) shift; uploader "$@" || rc=1 ;;
  usb)      "$OUT.usb" || rc=1 ;;
  *)        "$OUT" "$@" || rc=1 ;;
esac
exit $rc
