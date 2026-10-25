#!/bin/bash

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: BSD-3-Clause
# Licensed under the terms in the LICENSE file at the root of this
# repository.

# Build an example's CubeIDE projects from the repository, without the IDE's
# window: the same build as importing Examples/<Example>/STM32CubeIDE/Appli
# and .../FSBL into STM32CubeIDE and building Debug. Each project is imported
# into a throw-away workspace, its makefiles are generated from the project
# files, and it is built from clean.
#
#   bash harness/examples/cubebuild.sh Blink            # every project it has
#   bash harness/examples/cubebuild.sh Blink Appli      # one of them
#   bash harness/examples/cubebuild.sh --all            # every example
#
# Prints each image and its size, or the errors. The exit code is the
# number of projects that failed.
#
# Written for Git Bash on Windows. STM32CUBEIDE_DIR is the IDE's install
# directory (default C:/ST/STM32CubeIDE_2.2.0); STM32CUBEIDE_EXE, if set, is
# the headless executable itself.
#
# The headless builder prints "Build Finished. 0 errors" for a setup pass
# first, so success is no "Build Failed" and the image existing. After a
# failed build it can hang "Saving workspace", so it is stopped there.

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
IDE="${STM32CUBEIDE_DIR:-/c/ST/STM32CubeIDE_2.2.0}"
IDE="$(cygpath -u "$IDE" 2>/dev/null || echo "$IDE")"
EXE="${STM32CUBEIDE_EXE:-$IDE/STM32CubeIDE/stm32cubeidec.exe}"
winpath() { cygpath -w "$1" 2>/dev/null || echo "$1"; }

build_one() {   # <Example> <Appli|FSBL>
  local dir="$REPO/Examples/$1/STM32CubeIDE/$2"
  [ -f "$dir/.project" ] || { echo "no project: Examples/$1/STM32CubeIDE/$2"; return 1; }
  local proj ws log pid t img
  proj="$(grep -o '<name>[^<]*</name>' "$dir/.project" | head -1 | sed 's/<[^>]*>//g')"
  ws="$(mktemp -d)"
  log="$ws.log"
  rm -rf "$dir/Debug"
  MSYS_NO_PATHCONV=1 "$EXE" -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
    -data "$(winpath "$ws")" -import "$(winpath "$dir")" -build "$proj/Debug" > "$log" 2>&1 &
  pid=$!
  t=0
  while kill -0 $pid 2>/dev/null; do
    sleep 5; t=$((t + 5))
    if grep -q 'Saving workspace' "$log" 2>/dev/null; then
      for _ in $(seq 1 12); do kill -0 $pid 2>/dev/null || break; sleep 5; done
      break
    fi
    [ $t -lt 1500 ] || break
  done
  if kill -0 $pid 2>/dev/null; then
    local winpid
    winpid="$(cat /proc/$pid/winpid 2>/dev/null)"
    [ -n "$winpid" ] && taskkill //PID "$winpid" //T //F >/dev/null 2>&1
    kill $pid 2>/dev/null
  fi
  rm -rf "$ws"
  # Importing rewrites the project's IDE metadata (scanner cache, fingerprint,
  # sometimes .cproject and .project). Put the tracked copies back once the
  # workspace has gone, as the IDE writes some of them at shutdown.
  git -C "$REPO" checkout -- "Examples/$1/STM32CubeIDE/$2/.cproject" \
                             "Examples/$1/STM32CubeIDE/$2/.project" \
                             "Examples/$1/STM32CubeIDE/$2/.settings" 2>/dev/null
  img="$(ls "$dir/Debug/"*_Trusted.bin 2>/dev/null | head -1)"
  if grep -q 'Build Failed' "$log" || [ -z "$img" ]; then
    echo "FAIL  $1 $2 (log $log)"
    grep -E ' error: |error:|undefined reference|multiple definition|Error [0-9]' "$log" | sort -u | head -20
    return 1
  fi
  echo "OK    $1 $2: ${img#$REPO/} ($(stat -c %s "$img") B, $(grep -c 'warning:' "$log") warnings)"
  rm -f "$log"
}

[ -x "$EXE" ] || { echo "no headless STM32CubeIDE at $EXE; set STM32CUBEIDE_DIR"; exit 1; }
if [ "$1" = "--all" ]; then
  examples=()
  for d in "$REPO"/Examples/*/STM32CubeIDE; do
    ex="$(basename "$(dirname "$d")")"
    # An example under a licence that reaches the product is built only when
    # asked for by name, or with NEURON6_BUILD_RESTRICTED=1, as compile.sh does.
    if [ -f "$REPO/Examples/$ex/RESTRICTED-LICENCE" ] && [ "${NEURON6_BUILD_RESTRICTED:-0}" != "1" ]; then
      echo "SKIP  $ex (restricted licence; set NEURON6_BUILD_RESTRICTED=1)"
      continue
    fi
    examples+=("$ex")
  done
else
  examples=("${1:?usage: cubebuild.sh <Example> [project] | --all}")
fi

# The example's projects: the one named, or every one it has (Appli, FSBL,
# and BSPTest's Appli_FreeRTOS).
projects() {
  if [ -n "$2" ] && [ "$1" != "--all" ]; then echo "$2"; return; fi
  for d in "$REPO/Examples/$3/STM32CubeIDE"/*/; do
    [ -f "$d/.project" ] && basename "$d"
  done
}

failed=0
for ex in "${examples[@]}"; do
  for part in $(projects "$1" "$2" "$ex"); do
    build_one "$ex" "$part" || failed=$((failed + 1))
  done
done
# The IDE can still be writing its cache after the builder has returned.
sleep 5
for ex in "${examples[@]}"; do
  for part in $(projects "$1" "$2" "$ex"); do
    git -C "$REPO" checkout -- "Examples/$ex/STM32CubeIDE/$part/.cproject" \
                               "Examples/$ex/STM32CubeIDE/$part/.project" \
                               "Examples/$ex/STM32CubeIDE/$part/.settings" 2>/dev/null
  done
done
exit $failed
