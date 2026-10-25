/**
 ******************************************************************************
 * @file    PostProcessor_HandLM.h
 * @brief   IPostProcessor factory for the MediaPipe hand-landmark model, the
 *          cascade's second stage.
 *
 * Decode only: the application chooses the region the model runs on
 * (Camera_HandLandmarks_USB_FreeRTOS holds the palm-to-hand constants).
 * Returns a singleton, like the other factories.
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

#ifndef POSTPROCESSOR_HANDLM_H
#define POSTPROCESSOR_HANDLM_H

#include "NeuroN6_IPostProcessor.h"
#include "vnd_meta.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "app_postprocess_handlm_ui.h" /* handlm_result_t */

/* min_score: hand-presence gate (0.5 is MediaPipe's default). */
IPostProcessor *PostProcess_HandLandmarks(float min_score);

/* Same, with metadata in `slot` (e.g. when the palm detector owns MODEL_0). */
IPostProcessor *PostProcess_HandLandmarks_OnSlot(float min_score,
                                                 vnd_meta_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* POSTPROCESSOR_HANDLM_H */
