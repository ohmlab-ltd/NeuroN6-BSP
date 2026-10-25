/**
 ******************************************************************************
 * @file    PostProcessor_YOLOv8.h
 * @brief   IPostProcessor factories for the bundled YOLOv8 models, as used by
 *          Examples/Camera_YoloV8Seg_USB_FreeRTOS.
 *
 * Both return singletons (the decodes keep file-scope state). Calling a
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

#ifndef POSTPROCESSOR_YOLOV8_H
#define POSTPROCESSOR_YOLOV8_H

#include "NeuroN6_IPostProcessor.h"
#include "vnd_meta.h"   /* vnd_meta_slot_t for the _OnSlot variants */

#ifdef __cplusplus
extern "C" {
#endif

/* Instance segmentation. conf: detection confidence threshold (e.g. 0.4);
 * iou: NMS IoU threshold (e.g. 0.5). Metadata goes to VND_META_SLOT_PRIMARY
 * (= VND_META_SLOT_MODEL_0). */
IPostProcessor *PostProcess_YOLOv8_ISEG(float conf, float iou);

/* Same, with metadata in `slot`, for multi-model applications: the n6_capture
 * task merges the freshest TLVs of all slots into one USB frame. */
IPostProcessor *PostProcess_YOLOv8_ISEG_OnSlot(float conf, float iou,
                                               vnd_meta_slot_t slot);

/* Multi-person pose estimation. conf: detection confidence threshold
 * (e.g. 0.4); iou: NMS IoU threshold (e.g. 0.5). Metadata goes to
 * VND_META_SLOT_PRIMARY (= VND_META_SLOT_MODEL_0). */
IPostProcessor *PostProcess_YOLOv8_MPE(float conf, float iou);

/* Same, with metadata in `slot`. */
IPostProcessor *PostProcess_YOLOv8_MPE_OnSlot(float conf, float iou,
                                              vnd_meta_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* POSTPROCESSOR_YOLOV8_H */
