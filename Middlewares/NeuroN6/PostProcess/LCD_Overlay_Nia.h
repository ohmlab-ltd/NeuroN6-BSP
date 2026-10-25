/**
 ******************************************************************************
 * @file    LCD_Overlay_Nia.h
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

/* NIA Triad-416 (COCO-80) overlay: a box and a label per detection on the
 * LTDC overlay layer. Called from PostProcessor_NIA416.c's draw_lcd hook; the
 * application defines it, drawing nothing when it has no panel. */

#pragma once

/* Outside extern "C": the header sets its own linkage guards. */
#include "NeuroN6_IPostProcessor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward tag: no need to include nia_pp.h. */
struct nia_pp_out;
void LCD_Overlay_Nia_From(const struct nia_pp_out *out, OVL_Mode_t mode);

#ifdef __cplusplus
}
#endif
