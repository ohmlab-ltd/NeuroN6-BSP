/**
 ******************************************************************************
 * @file    app_postprocess_mpe_yolo_v8_ui.h
 * @brief   The MPE YOLOv8 post-processing wrapper.
 *          - init(conf_threshold, iou_threshold) sets up all parameters
 *          - run(instance) reads NN output buffers and runs post-processing
 ******************************************************************************
 * @attention
 *
 * Portions Copyright (c) STMicroelectronics. All rights reserved.
 *
 * SPDX-License-Identifier: SLA0044
 * This file follows STMicroelectronics' model-zoo post-processing and is
 * licensed under ST SLA0044, whose terms are in
 * Middlewares/NeuroN6/PostProcess/LICENSE-SLA0044.md. It is NOT covered by
 * this repository's BSD-3-Clause licence, and may be used only on, or with,
 * STMicroelectronics devices.
 ******************************************************************************
 */

#ifndef APP_POSTPROCESS_MPE_YOLO_V8_UI_H
#define APP_POSTPROCESS_MPE_YOLO_V8_UI_H

#include <stdint.h>
#include <stdbool.h>

/* Include arm_math_types.h OUTSIDE extern "C" so that arm_mve.h
 * (pulled in transitively) can use C++ overloads without conflict. */
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "mpe_yolov8_pp_if.h"
#include "ll_aton_rt_user_api.h"

/* -------------------------------------------------------------------------
 * MPE (Multi-Person / Pose Estimation) YOLOv8 post-processing configuration
 * All can be overridden by the application before including this header.
 * ------------------------------------------------------------------------- */

#ifndef AI_MPE_YOLOV8_PP_NB_CLASSES
#define AI_MPE_YOLOV8_PP_NB_CLASSES         (1)
#endif

#ifndef AI_MPE_YOLOV8_PP_TOTAL_BOXES
#define AI_MPE_YOLOV8_PP_TOTAL_BOXES        (1344)
#endif

#ifndef AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT
#define AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT    (10)
#endif

#ifndef AI_MPE_YOLOV8_PP_CONF_THRESHOLD
#define AI_MPE_YOLOV8_PP_CONF_THRESHOLD     (0.4f)
#endif

#ifndef AI_MPE_YOLOV8_PP_IOU_THRESHOLD
#define AI_MPE_YOLOV8_PP_IOU_THRESHOLD      (0.5f)
#endif

#ifndef AI_POSE_PP_POSE_KEYPOINTS_NB
#define AI_POSE_PP_POSE_KEYPOINTS_NB        (17)
#endif

/* -------------------------------------------------------------------------
 * Global outputs accessible from other modules and the application
 * ------------------------------------------------------------------------- */

extern mpe_pp_out_t                   mpe_pp_output;
extern mpe_yolov8_pp_static_param_t   mpe_pp_params;

int32_t app_postprocess_mpe_yolo_v8_ui_init(float conf_threshold, float iou_threshold);
int32_t app_postprocess_mpe_yolo_v8_ui_run(NN_Instance_TypeDef *instance);

#ifdef __cplusplus
}
#endif

#endif /* APP_POSTPROCESS_MPE_YOLO_V8_UI_H */
