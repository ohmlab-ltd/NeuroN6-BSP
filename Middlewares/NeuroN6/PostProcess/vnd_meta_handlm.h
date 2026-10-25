/**
 * @file    vnd_meta_handlm.h
 * @brief   Hand-landmark metadata for USB streaming (Neuro Studio).
 *
 * Emits the same TLV types the pose post-processor uses (META_KEYPOINTS,
 * META_BINDINGS, META_INFERENCE_MS), so the host renders the 21-point hand
 * skeleton with no host-side changes.
 *
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

#ifndef VND_META_HANDLM_H
#define VND_META_HANDLM_H

#include <stdint.h>
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "vnd_meta.h"
#include "app_postprocess_handlm_ui.h"

void vnd_meta_build_handlm_on_slot(const handlm_result_t *res,
                                   uint32_t inference_ms, vnd_meta_slot_t slot);

void Display_NS_HandLandmarks(void);
void Display_NS_HandLandmarks_OnSlot(vnd_meta_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* VND_META_HANDLM_H */
