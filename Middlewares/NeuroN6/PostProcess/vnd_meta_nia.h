/**
 * @file    vnd_meta_nia.h
 * @brief   Detection metadata for the Nia v0.5 detector (mirrors
 *          vnd_meta_yolov8_Iseg.h). Emits META_INFERENCE_MS/META_BOXES
 *          (frame-global, MODEL_0 slot only, camera-normalised) and
 *          META_ISEG_CLASSSTR (per detection: COCO-80 name + obj_id).
 *
 * @attention
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 * This software is licensed under the terms in
 * Middlewares/NeuroN6/PostProcess/LICENSE-MIT.txt.
 */

#ifndef VND_META_NIA_H
#define VND_META_NIA_H

#include <stdint.h>

/* arm_math_types.h OUTSIDE/BEFORE extern "C" so its guard is set before
 * nia_pp.h re-includes it from within the extern "C" block. */
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "vnd_meta.h"
#include "nia_pp.h"

/* Build Nia detection TLVs into `slot`. inference_ms is the NPU time in ms. */
void vnd_meta_build_nia_on_slot(const nia_pp_out_t *out, uint32_t inference_ms,
                                const nia_pp_params_t *params,
                                vnd_meta_slot_t slot);

/* The same, on the primary slot. */
void vnd_meta_build_nia(const nia_pp_out_t *out, uint32_t inference_ms,
                        const nia_pp_params_t *params);

/* Build from the file-scope nia_pp_output/nia_pp_params using the NeuroN6
 * inference timestamps (ts[1]-ts[0]). Use these from the IPostProcessor. */
void Display_NS_Nia(void);
void Display_NS_Nia_OnSlot(vnd_meta_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* VND_META_NIA_H */
