/**
 ******************************************************************************
 * @file    PostProcessor_PD.h
 * @brief   IPostProcessor factory for the MediaPipe palm detector.
 *
 * Model: MediaPipe palm_detection_full, 192x192 uint8 input, two float32
 * outputs ([1,2016,1] scores, [1,2016,18] boxes+landmarks), as
 * Middlewares/NeuroN6/Models/PalmDet generates it. As in
 * Examples/Camera_PalmDetect_USB_FreeRTOS:
 *
 *   LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(palmdet);
 *   IPostProcessor *pp = PostProcess_PalmDetector(0.5f, 0.3f);
 *   pp->init(pp->ctx, &NN_Instance_palmdet);
 *   ...after each inference...
 *   pp->run(pp->ctx, &NN_Instance_palmdet);
 *   pp->emit_metadata(pp->ctx);
 *
 * Returns a singleton (the decode keeps file-scope state). Calling the
 * factory again updates the thresholds; init() applies them.
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

#ifndef POSTPROCESSOR_PD_H
#define POSTPROCESSOR_PD_H

#include "NeuroN6_IPostProcessor.h"
#include "vnd_meta.h"   /* vnd_meta_slot_t for the _OnSlot variant */

#ifdef __cplusplus
extern "C" {
#endif

/* conf: detection confidence threshold (e.g. 0.5); iou: NMS IoU threshold
 * (e.g. 0.3). Metadata goes to VND_META_SLOT_PRIMARY
 * (= VND_META_SLOT_MODEL_0). */
IPostProcessor *PostProcess_PalmDetector(float conf, float iou);

/* Same, with metadata in `slot` (multi-model applications). */
IPostProcessor *PostProcess_PalmDetector_OnSlot(float conf, float iou,
                                                vnd_meta_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* POSTPROCESSOR_PD_H */
