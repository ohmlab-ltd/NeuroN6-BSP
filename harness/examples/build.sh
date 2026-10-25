#!/bin/bash

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: BSD-3-Clause
# Licensed under the terms in the LICENSE file at the root of this
# repository.

# Build a CubeIDE project from the command line.
#
# The generated makefiles pass -fcyclomatic-complexity, which only ST's own
# GNU build accepts, so its toolchain has to come ahead of any arm-none-eabi
# on PATH. Usage:  bash build.sh "NeuroN6 - BSPTest"
set -e

IDE="${STM32CUBEIDE_DIR:-/c/ST/STM32CubeIDE_2.2.0}"
IDE="$(cygpath -u "$IDE" 2>/dev/null || echo "$IDE")/STM32CubeIDE/plugins"
TOOLS="$IDE/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin"
MAKE="$IDE/com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.200.202604021615/tools/bin"
export PATH="$TOOLS:$MAKE:$PATH"

PROJ="${1:?project name}"
CUBE="${NEURON6_CUBE_DIR:?set NEURON6_CUBE_DIR to the directory holding the CubeIDE projects}"
DIR="$(cygpath -u "$CUBE" 2>/dev/null || echo "$CUBE")/$PROJ/Appli/Debug"
cd "$DIR"
make -j8 all
