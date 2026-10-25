/**
 ******************************************************************************
 * @file    app_postprocess_pd_ui.h
 * @brief   Palm-detector (MediaPipe "pd") post-processing.
 *
 * Model contract (MediaPipe palm_detection_full, 192x192):
 *   input   [1,192,192,3] uint8 channel-last  (= the DCMIPP NN-pipe feed)
 *   output  [1,2016, 1] float32  per-anchor score logits
 *   output  [1,2016,18] float32  per-anchor box (4) + 7 keypoints (14)
 * The outputs are told apart by size, so their order does not matter.
 * Results go to this module's own array, not ST's pd_pp_out_t (see the .c).
 *
 * Usage:
 *   app_postprocess_pd_ui_init(0.5f, 0.3f);
 *   ...
 *   app_postprocess_pd_ui_run(&NN_Instance_palmdet);
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

#ifndef APP_POSTPROCESS_PD_UI_H
#define APP_POSTPROCESS_PD_UI_H

#include <stdint.h>
#include <stdbool.h>

/* Outside extern "C", so arm_mve.h keeps its C++ overloads. */
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "pd_model_pp_if.h"
#include "ll_aton_rt_user_api.h"

/* Configuration; an example may override each before including this. */

/* NN input side (square); must match cfg.nn.width / cfg.nn.height. */
#ifndef AI_PD_PP_INPUT_SIZE
#define AI_PD_PP_INPUT_SIZE         (192)
#endif

/* Total anchors the model emits (24*24*2 + 12*12*6). */
#ifndef AI_PD_PP_TOTAL_BOXES
#define AI_PD_PP_TOTAL_BOXES        (2016)
#endif

/* Keypoints per palm (wrist + 4 knuckles + 2 thumb). */
#ifndef AI_PD_PP_NB_KEYPOINTS
#define AI_PD_PP_NB_KEYPOINTS       (7)
#endif

/* Pre-NMS candidate cap; ST's decoder stops scanning at it. One palm fires
 * 10-30 anchors above threshold. */
#ifndef AI_PD_PP_MAX_BOXES_LIMIT
#define AI_PD_PP_MAX_BOXES_LIMIT    (48)
#endif

/* Post-NMS detections reported to the example / host. */
#ifndef AI_PD_PP_MAX_DETECTIONS
#define AI_PD_PP_MAX_DETECTIONS     (8)
#endif

#ifndef AI_PD_PP_CONF_THRESHOLD
#define AI_PD_PP_CONF_THRESHOLD     (0.5f)
#endif

#ifndef AI_PD_PP_IOU_THRESHOLD
#define AI_PD_PP_IOU_THRESHOLD      (0.3f)
#endif

/* Output: NN-normalised [0..1] geometry in the NN input frame; map it with
 * nn_norm_to_cam_norm_* / nn_norm_to_lcd_*. */

typedef struct {
  float32_t x;
  float32_t y;
} pd_ui_point_t;

typedef struct {
  float32_t     prob;                              /* sigmoid'd score 0..1   */
  float32_t     x_center, y_center;                /* NN-normalised centre   */
  float32_t     width, height;                     /* NN-normalised extent   */
  pd_ui_point_t kps[AI_PD_PP_NB_KEYPOINTS];        /* NN-normalised landmarks*/
} pd_ui_detection_t;

/* Latest results, valid until the next app_postprocess_pd_ui_run(). */
extern pd_ui_detection_t          pd_ui_detections[AI_PD_PP_MAX_DETECTIONS];
extern uint32_t                   pd_ui_nb_detections;
extern pd_model_pp_static_param_t pd_pp_params;

/* Builds the anchor grid and the decode parameters. */
int32_t app_postprocess_pd_ui_init(float conf_threshold, float iou_threshold);
/* Decodes the outputs and runs NMS into pd_ui_detections. */
int32_t app_postprocess_pd_ui_run(NN_Instance_TypeDef *instance);

#ifdef __cplusplus
}
#endif

#endif /* APP_POSTPROCESS_PD_UI_H */
