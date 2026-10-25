/**
 ******************************************************************************
 * @file    PostProcessor_NIA416.h
 * @brief   IPostProcessor factory for the NIA Triad-416 ship-distance detector.
 *
 * Needs an export calibrated for asymmetric [0,1] input (scale 1/255, zp
 * -128); a symmetric-input export reads the camera's uint8 feed as noise.
 * Returns a singleton; calling the factory again updates the thresholds.
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

#ifndef POSTPROCESSOR_NIA416_H
#define POSTPROCESSOR_NIA416_H

#include "NeuroN6_IPostProcessor.h"
#include "vnd_meta.h"   /* vnd_meta_slot_t for the _OnSlot variant */

#ifdef __cplusplus
extern "C" {
#endif

/* conf: confidence threshold (default 0.25); iou: class-aware NMS IoU
 * threshold (default 0.70). Metadata goes to VND_META_SLOT_PRIMARY. */
IPostProcessor *PostProcess_NIA416(float conf, float iou);

/* Same, with metadata in `slot` (multi-model applications). */
IPostProcessor *PostProcess_NIA416_OnSlot(float conf, float iou,
                                          vnd_meta_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* POSTPROCESSOR_NIA416_H */
