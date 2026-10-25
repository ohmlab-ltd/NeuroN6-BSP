/**
 * @file    nia_pp.h
 * @brief   Host (Cortex-M55) post-processing for the Nia v0.5 flagship detector
 *          (nia_v4 / flagship_ft_ep27_int8_256, int8 / 256x256, COCO-80).
 *
 * Decodes 6 INT8 output tensors (split cls/reg head, 3 strides): dequantise
 * -> sigmoid -> B-Reg decode -> EQ-gate dedup -> threshold -> top-MAX_DET ->
 * normalise to [0..1]. B-Reg: dist = sigmoid(reg)*SCALES[level]*stride (NOT
 * exp/DFL). EQ-gate = 3x3 local-max on objectness, class-agnostic; no NMS.
 *
 * Output boxes are NN-normalised [0..1] centre+size, consumed via
 * nn_norm_to_cam_norm_* / nn_norm_to_lcd_*; no on-device letterbox.
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

#ifndef NIA_PP_H
#define NIA_PP_H

#include <stdint.h>
#include <stdbool.h>

/* arm_math_types.h outside extern "C" so arm_mve.h keeps its C++ overloads. */
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "ll_aton_rt_user_api.h"   /* LL_Buffer_InfoTypeDef, LL_ATON_Output_Buffers_Info */

/* Tuning; override before including. */
#ifndef NIA_PP_CONF_THRESHOLD
#define NIA_PP_CONF_THRESHOLD   (0.25f)   /* product default; 0.001f only for COCO mAP */
#endif
/* Unused: decode is NMS-free (the EQ-gate dedups). Kept for
 * PostProcess_NIA(conf, iou). */
#ifndef NIA_PP_IOU_THRESHOLD
#define NIA_PP_IOU_THRESHOLD    (0.60f)   /* unused (gate is NMS-free) */
#endif
#ifndef NIA_PP_MAX_DET
#define NIA_PP_MAX_DET          (100)     /* cap on kept detections */
#endif
#ifndef NIA_PP_NB_CLASSES
#define NIA_PP_NB_CLASSES       (80)      /* COCO-80 */
#endif
/* Candidate buffer: at most one per cell = 32^2+16^2+8^2 = 1344 (worst case, so nothing is dropped even at conf=0.001). */
#ifndef NIA_PP_MAX_CAND
#define NIA_PP_MAX_CAND         (1344)
#endif
/* Per-level objectness scratch sized to the largest grid (stride-8 = 32x32). */
#ifndef NIA_PP_MAX_GRID_CELLS
#define NIA_PP_MAX_GRID_CELLS   (32 * 32)
#endif
/* EQ-gate max-pool window (this flagship uses 3,3,3 - all levels). */
#ifndef NIA_PP_GATE_KERNEL
#define NIA_PP_GATE_KERNEL      (3)
#endif
/* Model input is square 256x256; decode happens in that pixel space. */
#ifndef NIA_PP_INPUT_DIM
#define NIA_PP_INPUT_DIM        (256.0f)
#endif

/* One detection. Coords are NN-normalised [0..1] centre+size, matching the iseg/mpe output convention. */
typedef struct {
  float32_t x_center;
  float32_t y_center;
  float32_t width;
  float32_t height;
  float32_t conf;
  int32_t   class_index;
} nia_det_t;

/* Tagged so headers that only need a pointer (e.g. LCD_Overlay_Nia.h) can forward-declare `struct nia_pp_out`. */
typedef struct nia_pp_out {
  nia_det_t *pOutBuff;   /* caller-provided array, >= max_det entries */
  int32_t    nb_detect;  /* number of valid detections after the gate */
} nia_pp_out_t;

typedef struct {
  float32_t conf_threshold;
  float32_t iou_threshold;   /* unused (NMS-free); kept for API compat */
  int32_t   max_det;
} nia_pp_params_t;

/**
 * @brief  Decode the NPU outputs into detections (NN-normalised [0..1]).
 * @param  outs    from LL_ATON_Output_Buffers_Info(instance); tensors matched by shape, not index.
 * @param  out     result; out->pOutBuff must hold >= params->max_det entries.
 * @param  params  thresholds + cap (iou_threshold ignored - gate is NMS-free).
 * @return 0 on success, negative on bad arguments.
 */
int32_t nia_pp_process(const LL_Buffer_InfoTypeDef *outs,
                       nia_pp_out_t *out,
                       const nia_pp_params_t *params);

#ifdef __cplusplus
}
#endif

#endif /* NIA_PP_H */
