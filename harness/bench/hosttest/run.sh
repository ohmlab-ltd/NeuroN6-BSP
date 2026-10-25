#!/bin/sh
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
#
# Build and run the host tests against this repository's copies of the code
# they test (their own comments give the original paths). Any C11
# compiler for the PC; on Windows, WSL's gcc:
#
#   sh harness/bench/hosttest/run.sh
#   wsl sh harness/bench/hosttest/run.sh
set -e
cd "$(dirname "$0")"
CC="${CC:-gcc}"
OUT="${TMPDIR:-/tmp}"
REPO=../../..

$CC -std=c11 -Wall -Ifake -I$REPO/Middlewares/NeuroN6/N6OTA test_asset.c -o "$OUT/test_asset"
$CC -std=c11 -Wall -I$REPO/Middlewares/NeuroN6/MayaW476/maya_w4_board test_sha256.c \
    $REPO/Middlewares/NeuroN6/MayaW476/maya_w4_board/maya_w4_sha256.c -o "$OUT/test_sha256"
$CC -std=c11 -Wall -I$REPO/Components/lepton test_vospi.c \
    $REPO/Components/lepton/lepton.c -o "$OUT/test_vospi"
$CC -std=c11 -Wall -Ifake_thermal -I$REPO -I$REPO/Components/lepton test_thermal_boot.c \
    fake_thermal/fake_thermal.c $REPO/neuron6_thermal.c $REPO/Components/lepton/lepton.c \
    -o "$OUT/test_thermal_boot"
$CC -std=c11 -Wall -Ifake_thermal -I$REPO -I$REPO/Components/lepton test_thermal_resync.c \
    fake_thermal/fake_thermal.c $REPO/neuron6_thermal.c $REPO/Components/lepton/lepton.c \
    -o "$OUT/test_thermal_resync"
$CC -std=c11 -Wall -Ifake -I$REPO/Middlewares/NeuroN6/PostProcess \
    -I$REPO/Middlewares/ST/lib_vision_models_pp/Inc test_iseg_meta.c \
    $REPO/Middlewares/NeuroN6/PostProcess/vnd_meta_yolov8_Iseg.c -o "$OUT/test_iseg_meta"
$CC -std=c11 -Wall -Wextra -Ifake -I$REPO/Middlewares/NeuroN6/PostProcess \
    -I$REPO/Middlewares/ST/lib_vision_models_pp/Inc test_meta_box.c \
    $REPO/Middlewares/NeuroN6/PostProcess/vnd_meta.c -lm -o "$OUT/test_meta_box"
$CC -std=c11 -Wall -Wextra -Ifake_thermal -I$REPO -I$REPO/Components/vl53l1cb test_ranging.c \
    $REPO/neuron6_ranging_sensor.c $REPO/Components/vl53l1cb/vl53l1cb.c -o "$OUT/test_ranging"
$CC -std=gnu11 -Wall -Ifake_thermal -I$REPO -I$REPO/Components/lsm6dsl -I$REPO/Components/mmc5603nj \
    test_motion.c $REPO/neuron6_motion_sensor.c $REPO/Components/lsm6dsl/lsm6dsl.c \
    $REPO/Components/mmc5603nj/mmc5603nj.c -o "$OUT/test_motion"
$CC -std=c11 -Wall -Wextra -Ifake_bt -I$REPO test_bt_txpower.c $REPO/neuron6_bt.c \
    -o "$OUT/test_bt_txpower"
"$OUT/test_asset"
"$OUT/test_sha256"
"$OUT/test_vospi"
"$OUT/test_thermal_boot"
"$OUT/test_thermal_resync"
"$OUT/test_iseg_meta"
"$OUT/test_ranging"
"$OUT/test_motion"
"$OUT/test_meta_box"
"$OUT/test_bt_txpower"
