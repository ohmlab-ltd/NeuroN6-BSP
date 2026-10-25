/**
 * @file    vnd_meta_yolov8_MPE.h
 * @brief   Multi-person pose estimation metadata building for USB streaming
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

#ifndef VND_META_YOLOV8_MPE_H
#define VND_META_YOLOV8_MPE_H

#include "app_postprocess_mpe_yolo_v8_ui.h"
#include "vnd_meta.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Build the MPE TLV stream into the chosen slot. See vnd_meta_build_iseg_on_slot
 * for the rationale (multi-model applications use one slot per model). */
void vnd_meta_build_mpe_on_slot(const mpe_pp_out_t *out, uint32_t inference_ms,
                                const mpe_yolov8_pp_static_param_t *pp_params,
                                vnd_meta_slot_t slot);

/* The same, on VND_META_SLOT_PRIMARY. */
void vnd_meta_build_mpe(const mpe_pp_out_t *out, uint32_t inference_ms,
                        const mpe_yolov8_pp_static_param_t *pp_params);

/**
 * @brief Wrapper: builds MPE metadata from global mpe_pp_output/mpe_pp_params
 */
void Display_NS_Yolo_V8_MPE(void);
void Display_NS_Yolo_V8_MPE_OnSlot(vnd_meta_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* VND_META_YOLOV8_MPE_H */
