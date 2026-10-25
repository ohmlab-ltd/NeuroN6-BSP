/**
 ******************************************************************************
 * @file    LCD_Overlay_MPE.h
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 * This software is licensed under the terms in
 * Middlewares/NeuroN6/PostProcess/LICENSE-MIT.txt.
 */

/* YOLOv8 multi-person pose overlay: draws boxes, keypoints, skeletons and
 * labels on the LTDC overlay layer after post-processing, e.g.
 * LCD_Overlay_MPE(OVL_RELOAD_CLEAR). */

#pragma once

/* Outside extern "C": the header sets its own linkage guards. */
#include "NeuroN6_IPostProcessor.h"

#ifdef __cplusplus
extern "C" {
#endif

void LCD_Overlay_MPE(OVL_Mode_t mode);

#ifdef __cplusplus
}
#endif
