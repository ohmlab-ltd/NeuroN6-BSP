/**
 ******************************************************************************
 * @file    LCD_Overlay_PD.h
 * @brief   MediaPipe palm detector overlay: palm boxes, the 7 landmarks and
 *          the hand skeleton on the LTDC overlay layer, e.g.
 *          LCD_Overlay_PD(OVL_RELOAD_CLEAR).
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 * This software is licensed under the terms in
 * Middlewares/NeuroN6/PostProcess/LICENSE-MIT.txt.
 ******************************************************************************
 */

#pragma once

/* Outside extern "C": the header sets its own linkage guards. */
#include "NeuroN6_IPostProcessor.h"

#ifdef __cplusplus
extern "C" {
#endif

void LCD_Overlay_PD(OVL_Mode_t mode);

#ifdef __cplusplus
}
#endif
