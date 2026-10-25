#!/bin/bash

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: BSD-3-Clause
# Licensed under the terms in the LICENSE file at the root of this
# repository.

# Compile-check every BSP translation unit against the STM32N6 HAL, with
# -Wall -Wextra as errors, and every public header from C++ (extern "C").
#
# Usage:  NEURON6_HAL_ROOT=<STM32CubeN6>/Drivers bash harness/compile.sh
# The tree must contain STM32N6xx_HAL_Driver/ and CMSIS/ (the STM32CubeN6
# package's Drivers folder does); everything else is in this repository.
# harness/ also carries stm32n6xx_hal_iwdg.h (stock ST, BSD-3-Clause) for HAL
# copies that omit that module.
set -e
cd "$(dirname "$0")/.."

HAL_ROOT="${NEURON6_HAL_ROOT:-${HAL_ROOT:-}}"
if [ -z "$HAL_ROOT" ]; then
  echo "set NEURON6_HAL_ROOT to a tree containing STM32N6xx_HAL_Driver/ and CMSIS/" >&2
  exit 2
fi
CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
# Compiled in full, not only parsed: some warnings come from later passes.
GATE_OBJ="$(mktemp)"
GATE_CONF="$(mktemp -d)"
trap 'rm -f "$GATE_OBJ"; rm -rf "$GATE_CONF"' EXIT

# VideoEncoder/inc must precede VideoEncoder/source/*: the vendor tree's stale
# source/jpeg/basetype.h redefines size_t (this repository drops that copy).
INC="-I. -Iharness -IComponents/ov5640 -IComponents/lsm6dsl -IComponents/mmc5603nj \
     -IComponents/lepton -IComponents/vd6g -IComponents/vd55g1 -IComponents/vl53l1cb \
     -IUtilities/usb \
     -IMiddlewares/tinyusb \
     -IComponents/venc/VideoEncoder/inc -IComponents/venc/VideoEncoder/source/jpeg \
     -IComponents/venc/VideoEncoder/source/common -IComponents/venc/EWL \
     -I$HAL_ROOT/STM32N6xx_HAL_Driver/Inc \
     -I$HAL_ROOT/CMSIS/Device/ST/STM32N6xx/Include \
     -I$HAL_ROOT/CMSIS/Core/Include"
# CFG_TUSB_OS is pinned here: TinyUSB finds the RTOS-flavoured tusb_config.h
# beside itself first. The bare-metal configuration is in Utilities/usb/.
FLAGS="-mcpu=cortex-m55 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
       -DSTM32N657xx -DUSE_HAL_DRIVER -DCFG_TUSB_OS=OPT_OS_NONE -Os"
WARN="-Wall -Wextra -Werror -Wno-unused-parameter"
WARN_VENDORED="-Wall -Wextra -Wno-unused-parameter"

# Per-example switches: its own TinyUSB configuration, or USB video with the
# BSP's. Either goes ahead of TinyUSB's own copy (they share an include guard).
example_flags() {
  local dir="${1%/Core/Src/*}"
  local conf="$dir/Core/Inc/neuron6_conf.h"
  if [ -f "$dir/Utilities/usb/tusb_config.h" ]; then
    echo "-include $dir/Utilities/usb/tusb_config.h"
  elif [ -f "$conf" ] && grep -Eq '^#define[[:space:]]+BSP_USB_UVC[[:space:]]+1' "$conf"; then
    echo "-DBSP_USB_UVC=1 -include Utilities/usb/tusb_config.h"
  fi
}

# Middleware stacks an example lists in *_build.json files. Their include
# paths come last, as -isystem, so "neuron6.h" stays the BSP's on a
# case-insensitive disk and their headers keep their own warning policy.
example_stack_flags() {
  local dir="${1%/Core/Src/*}"
  ls "$dir"/*_build.json >/dev/null 2>&1 || return 0
  stack_tool --flags "$(basename "$dir")"
}

# Python on Windows ends its lines with CR LF; the CRs would stick to names.
stack_tool() {
  local out
  out="$(python harness/examples/addstack.py "$@")" || return 1
  printf '%s\n' "$out" | tr -d '\r'
}

# An example written for the kernel says so unconditionally in its own
# configuration, and is compiled by the FreeRTOS pass alone.
example_is_rtos_only() {
  local conf="${1%/Src/*}/Inc/neuron6_conf.h"
  [ -f "$conf" ] && grep -Eq '^#define[[:space:]]+BSP_USE_FREERTOS[[:space:]]+1' "$conf"
}

# CubeMX's secure-state boilerplate needs its project's own headers; the
# example's CubeIDE build compiles it.
is_st_secure_template() {
  case "$1" in */secure_nsc.c|*/system_stm32n6xx_s.c) return 0;; esac
  return 1
}

# An example whose network's licence would travel into a product built from
# it (the AGPL YOLOv8 ones) is built only when named, so a default build of
# this repository produces nothing derived from it.
example_is_restricted() {
  case "$1" in
    Examples/*) [ -f "Examples/$(echo "${1#Examples/}" | cut -d/ -f1)/RESTRICTED-LICENCE" ] \
                  && [ "${NEURON6_BUILD_RESTRICTED:-0}" != "1" ];;
    *) false;;
  esac
}

# Both security states are built. The shipped firmware runs secure, and the
# isolation grants and secure DMA attributes exist only under -mcmse.
SECURE_FLAGS="-mcmse"

fail=0
for state in nonsecure secure; do
  case "$state" in
    secure) extra="$SECURE_FLAGS" ;;
    *)      extra="" ;;
  esac

  # find, not a fixed-depth glob, which would silently skip deeper files (the
  # encoder core nests five levels deep).
  for f in neuron6*.c $(find Components -name '*.c' | sort); do
    [ -f "$f" ] || continue

    case "$f" in
      # Included by vd6g.c; compiled alone they would duplicate its symbols.
      Components/vd6g/vd6g_patch.c|Components/vd6g/vd6g_vtpatch.c) continue;;
      Components/vd55g1/vd55g1_patch_cut_2.c) continue;;
    esac

    # Ohm Lab's code is held to the warning policy. Redistributed code shows
    # its warnings without failing, so it can stay unmodified against upstream.
    warn="$WARN"
    case "$f" in
      Components/ov5640/*|Components/vd6g/*|Components/vd55g1/*|Components/venc/*) warn="$WARN_VENDORED";;
    esac

    if $CC -c -o "$GATE_OBJ" -std=c11 $warn $FLAGS $extra $INC "$f"; then
      echo "PASS  [$state] $f"
    else
      echo "FAIL  [$state] $f"; fail=1
    fi
  done

  # Common/Src and the examples, against the harness configuration plus each
  # example's own switches.
  for f in Common/Src/*.c Examples/*/Core/Src/*.c; do
    [ -f "$f" ] || continue
    is_st_secure_template "$f" && continue
    if example_is_restricted "$f"; then
      echo "SKIP  [$state] $f (restricted licence; set NEURON6_BUILD_RESTRICTED=1)"
      continue
    fi
    if example_is_rtos_only "$f"; then
      echo "SKIP  [$state] $f (FreeRTOS only)"
      continue
    fi
    if ! stack="$(example_stack_flags "$f")"; then
      echo "FAIL  [$state] $f (its *_build.json)"; fail=1
      continue
    fi
    if $CC -c -o "$GATE_OBJ" -std=c11 $WARN $FLAGS $extra $(example_flags "$f") $INC $stack "$f"; then
      echo "PASS  [$state] $f"
    else
      echo "FAIL  [$state] $f"; fail=1
    fi
  done

  # C++ consumer: include every public header from C++.
  cxx_probe=$(mktemp --suffix=.cpp 2>/dev/null || echo /tmp/bsp_probe.cpp)
  {
    for h in neuron6*.h; do
      case "$h" in *_template*) continue;; esac
      echo "#include \"$h\""
    done
    echo "int main(void) { return 0; }"
  } > "$cxx_probe"
  if $CXX -c -o "$GATE_OBJ" -std=gnu++17 $WARN $FLAGS $extra $INC "$cxx_probe"; then
    echo "PASS  [$state] C++ header consumer"
  else
    echo "FAIL  [$state] C++ header consumer"; fail=1
  fi
  rm -f "$cxx_probe"
done

# FreeRTOS flavour: BSP_USE_FREERTOS set, secure state only. The kernel is held
# to the vendored warning policy, the glue to the strict one.
RTOS_INC="-IMiddlewares/FreeRTOS/Source/include \
          -IMiddlewares/FreeRTOS/Source/portable/GCC/ARM_CM55_NTZ/non_secure \
          -IMiddlewares/FreeRTOS/Source/CMSIS_RTOS_V2 \
          -IUtilities/freertos"
# Without the bare-metal CFG_TUSB_OS pin, tusb_config.h follows
# BSP_USE_FREERTOS, as in a project.
RTOS_BASE_FLAGS="${FLAGS/-DCFG_TUSB_OS=OPT_OS_NONE/}"
RTOS_FLAGS="-DBSP_USE_FREERTOS=1"
for f in neuron6*.c Utilities/freertos/stm32n6xx_hal_timebase_tim.c \
         Components/venc/EWL/ewl_impl.c Examples/*/Core/Src/*.c; do
  [ -f "$f" ] || continue
  is_st_secure_template "$f" && continue
  if example_is_restricted "$f"; then
    echo "SKIP  [freertos] $f (restricted licence; set NEURON6_BUILD_RESTRICTED=1)"
    continue
  fi
  warn="$WARN"
  case "$f" in Components/*) warn="$WARN_VENDORED";; esac
  ex=""
  stack=""
  case "$f" in Examples/*) ex="$(example_flags "$f")";; esac
  case "$f" in
    Examples/*)
      if ! stack="$(example_stack_flags "$f")"; then
        echo "FAIL  [freertos] $f (its *_build.json)"; fail=1
        continue
      fi;;
  esac
  if $CC -c -o "$GATE_OBJ" -std=c11 $warn $RTOS_BASE_FLAGS $SECURE_FLAGS $RTOS_FLAGS $ex $INC $RTOS_INC $stack "$f"; then
    echo "PASS  [freertos] $f"
  else
    echo "FAIL  [freertos] $f"; fail=1
  fi
done
# The stacks' code, built as the examples build it (under the kernel, with
# their switches); warnings shown, not failed on. A file built differently by
# two examples (Mbed TLS for BLE and for TLS) is built each way. Some stacks
# need -fpermissive on GCC 14; GCC 13 and older do not, and warn that it is
# C++ only, so it is dropped there.
permissive_c=1
echo 'int x;' | $CC -x c -fsyntax-only -fpermissive -Werror - 2>/dev/null || permissive_c=0
declare -A stack_seen stack_file_seen
for exdir in Examples/*/; do
  ex="$(basename "$exdir")"
  ls "$exdir"*_build.json >/dev/null 2>&1 || continue
  if [ -f "$exdir/RESTRICTED-LICENCE" ] && [ "${NEURON6_BUILD_RESTRICTED:-0}" != "1" ]; then
    echo "SKIP  [freertos] Examples/$ex (restricted licence; set NEURON6_BUILD_RESTRICTED=1)"
    continue
  fi
  if ! stack="$(stack_tool --flags "$ex")" || ! srcs="$(stack_tool --sources "$ex")"; then
    echo "FAIL  [freertos] Examples/$ex (its *_build.json)"; fail=1
    continue
  fi
  while IFS='|' read -r lang f; do
    [ -n "$f" ] || continue
    [ -z "${stack_seen[$f|$lang|$stack]}" ] || continue
    stack_seen[$f|$lang|$stack]=1
    as=""
    [ -z "${stack_file_seen[$f]}" ] || as=" (as $ex builds it)"
    stack_file_seen[$f]=1
    [ "$permissive_c" = 1 ] || lang="${lang//-fpermissive/}"
    if $CC -c -o "$GATE_OBJ" $lang $WARN_VENDORED $RTOS_BASE_FLAGS $SECURE_FLAGS $RTOS_FLAGS $INC $RTOS_INC $stack "$f"; then
      echo "PASS  [freertos] $f$as"
    else
      echo "FAIL  [freertos] $f$as"; fail=1
    fi
  done <<< "$srcs"
done
for f in $(find Middlewares/FreeRTOS/Source -name '*.c' | sort); do
  if $CC -c -o "$GATE_OBJ" -std=c11 $WARN_VENDORED $RTOS_BASE_FLAGS $SECURE_FLAGS $RTOS_FLAGS $INC $RTOS_INC "$f"; then
    echo "PASS  [freertos] $f"
  else
    echo "FAIL  [freertos] $f"; fail=1
  fi
done

# Every USE_HAL_<PPP>_REGISTER_CALLBACKS at 1 (the harness, like a shipped
# project, leaves them at 0), so the BSP_*_RegisterMspCallbacks code is
# compiled too. Secure state, bare metal.
sed -E 's/^(#define[[:space:]]+USE_HAL_[A-Z0-9_]+_REGISTER_CALLBACKS[[:space:]]+)0U/\11U/' \
    harness/stm32n6xx_hal_conf.h > "$GATE_CONF/stm32n6xx_hal_conf.h"
if grep -Eq '^#define[[:space:]]+USE_HAL_[A-Z0-9_]+_REGISTER_CALLBACKS[[:space:]]+0U' \
     "$GATE_CONF/stm32n6xx_hal_conf.h" ||
   ! grep -Eq '^#define[[:space:]]+USE_HAL_I2C_REGISTER_CALLBACKS[[:space:]]+1U' \
     "$GATE_CONF/stm32n6xx_hal_conf.h"; then
  echo "FAIL  [callbacks] harness/stm32n6xx_hal_conf.h no longer takes the switch"; fail=1
else
  for f in neuron6*.c; do
    if $CC -c -o "$GATE_OBJ" -std=c11 $WARN $FLAGS $SECURE_FLAGS -I"$GATE_CONF" $INC "$f"; then
      echo "PASS  [callbacks] $f"
    else
      echo "FAIL  [callbacks] $f"; fail=1
    fi
  done
fi

# The asset store's layout is a contract with the boot loader, which writes
# what the application stages: the two copies have to be the same file.
if cmp -s FSBL/Core/Inc/n6_ota_layout.h Middlewares/NeuroN6/N6OTA/n6_ota_layout.h; then
  echo "PASS  n6_ota_layout.h matches the FSBL's"
else
  echo "FAIL  Middlewares/NeuroN6/N6OTA/n6_ota_layout.h differs from FSBL/Core/Inc/n6_ota_layout.h"; fail=1
fi

exit $fail
