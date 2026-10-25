/**
 * @file    vnd_meta_pd.h
 * @brief   Palm-detector metadata building for USB streaming (Neuro Studio).
 *
 * Emits the SAME TLV types the pose (MPE) post-processor uses (META_BOXES,
 * META_KEYPOINTS, META_BINDINGS, META_INFERENCE_MS), so the host renders palm
 * boxes + landmark skeletons with no host-side changes.
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

#ifndef VND_META_PD_H
#define VND_META_PD_H

#include <stdint.h>
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "vnd_meta.h"
#include "app_postprocess_pd_ui.h"

/* Build the palm-detection TLVs for `slot` from an explicit result set. */
void vnd_meta_build_pd_on_slot(const pd_ui_detection_t *dets, uint32_t nb_dets,
                               uint32_t inference_ms, vnd_meta_slot_t slot);

/* Convenience: publish the module-global results (pd_ui_detections) with
 * the inference time from the application's timestamps (ts). */
void Display_NS_PalmDetector(void);
void Display_NS_PalmDetector_OnSlot(vnd_meta_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* VND_META_PD_H */
