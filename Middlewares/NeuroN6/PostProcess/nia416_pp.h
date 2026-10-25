/**
 * @file    nia416_pp.h
 * @brief   Host (Cortex-M55) post-processing for the NIA Triad-416 ship-distance
 *          detector (nia_triad416_shipdist, INT8 ONNX, 416x416, COCO-80).
 *
 * Six INT8 outputs, dequantised with each one's scale and zero point. reg is
 * already LTRB pixels (ReLU + in-graph Conv1x1(diag(s,s,s,s)), not B-Reg: do
 * not scale again). Multi-label scoring, class-aware NMS at IoU 0.70, grids
 * 52/26/13 at strides 8/16/32; class logits are thresholded against
 * logit(conf), so expf() runs only on survivors. Boxes are NN-normalised
 * [0..1] centre+size in nia_det_t (nia_pp.h). The DCMIPP NN pipe cannot pad,
 * so applications use ASPECT_CROP_CENTER, not a letterbox.
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

#ifndef NIA416_PP_H
#define NIA416_PP_H

#include <stdint.h>
#include <stdbool.h>

/* arm_math_types.h OUTSIDE extern "C" so arm_mve.h keeps its C++ overloads. */
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "nia_pp.h"                /* nia_det_t / nia_pp_out_t / nia_pp_params_t */
#include "ll_aton_rt_user_api.h"

/* Tuning; override before including. */
#ifndef NIA416_PP_CONF_THRESHOLD
#define NIA416_PP_CONF_THRESHOLD  (0.25f)  /* embedded default; 0.001f only for COCO mAP */
#endif
#ifndef NIA416_PP_IOU_THRESHOLD
#define NIA416_PP_IOU_THRESHOLD   (0.70f)  /* product NMS setting */
#endif
#ifndef NIA416_PP_MAX_DET
#define NIA416_PP_MAX_DET         (100)    /* 100 is plenty for live use */
#endif
#ifndef NIA416_PP_NB_CLASSES
#define NIA416_PP_NB_CLASSES      (80)     /* COCO-80, contiguous order */
#endif
/* Pre-NMS candidate cap; decode simply stops once this is hit. */
#ifndef NIA416_PP_MAX_CAND
#define NIA416_PP_MAX_CAND        (512)
#endif
/* Model input is square 416x416; decode happens in that pixel space. */
#ifndef NIA416_PP_INPUT_DIM
#define NIA416_PP_INPUT_DIM       (416.0f)
#endif

/**
 * @brief  Decode the NPU outputs into detections (NN-normalised [0..1]).
 * @param  outs    from LL_ATON_Output_Buffers_Info(instance); tensors matched
 *                 by shape, not index, so a re-export that reorders them is safe.
 * @param  out     result; out->pOutBuff must hold >= params->max_det entries.
 * @param  params  thresholds + cap (iou_threshold is used here, unlike nia_pp).
 * @return 0 on success, negative on bad arguments.
 */
int32_t nia416_pp_process(const LL_Buffer_InfoTypeDef *outs,
                          nia_pp_out_t *out,
                          const nia_pp_params_t *params);

#ifdef __cplusplus
}
#endif

#endif /* NIA416_PP_H */
