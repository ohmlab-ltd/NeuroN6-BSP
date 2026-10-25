/**
 * @file    vnd_meta_yolov8_Iseg.h
 * @brief   Instance segmentation metadata building for USB streaming
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

#ifndef VND_META_YOLOV8_ISEG_H
#define VND_META_YOLOV8_ISEG_H

#include "app_postprocess_iseg_yolo_v8_ui.h"
#include "vnd_meta.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Build the ISEG TLV stream into the chosen slot. For single-model applications
 * the slot is VND_META_SLOT_PRIMARY (= MODEL_0). For sequential multi-model
 * applications pass a distinct slot per model so each emits independently and
 * the background task composes the freshest from each slot. */
void vnd_meta_build_iseg_on_slot(const iseg_pp_out_t *out, uint32_t inference_ms,
                                 const iseg_yolov8_pp_static_param_t *pp_params,
                                 vnd_meta_slot_t slot);

/* The same, on VND_META_SLOT_PRIMARY. */
void vnd_meta_build_iseg(const iseg_pp_out_t *out, uint32_t inference_ms,
                         const iseg_yolov8_pp_static_param_t *pp_params);

/**
 * @brief Wrapper: builds iseg metadata from global pp_output/pp_params
 */
void Display_NS_Yolo_V8_Iseg(void);
void Display_NS_Yolo_V8_Iseg_OnSlot(vnd_meta_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* VND_META_YOLOV8_ISEG_H */
