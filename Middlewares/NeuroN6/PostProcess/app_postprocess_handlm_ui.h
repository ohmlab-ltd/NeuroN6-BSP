/**
 ******************************************************************************
 * @file    app_postprocess_handlm_ui.h
 * @brief   MediaPipe hand-landmark post-processing.
 *
 * Model contract (MediaPipe hand_landmark_full, 224x224):
 *   input   [1,224,224,3] uint8 channel-last, scale 1/255 zp 0
 *           (the DCMIPP NN-pipe format, no conversion)
 *   outputs [1,63] screen landmarks: 21 x (x, y, z), x/y in input pixels
 *           [1,63] world landmarks:  21 x (x, y, z) in metres, wrist-relative
 *           [1, 1] hand-presence score (sigmoid'd)
 *           [1, 1] handedness (0 = left .. 1 = right)
 *
 * Landmarks are published normalised [0..1] within the model input, so
 * nn_norm_to_cam_norm_* / nn_norm_to_lcd_* map them like any other
 * post-processor's.
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

#ifndef APP_POSTPROCESS_HANDLM_UI_H
#define APP_POSTPROCESS_HANDLM_UI_H

#include <stdint.h>
#include <stdbool.h>

/* Outside extern "C", so arm_mve.h keeps its C++ overloads. */
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "ll_aton_rt_user_api.h"

#ifndef AI_HANDLM_NB_LANDMARKS
#define AI_HANDLM_NB_LANDMARKS   (21)
#endif

/* Model input side (square), in pixels. */
#ifndef AI_HANDLM_INPUT_SIZE
#define AI_HANDLM_INPUT_SIZE     (224.0f)
#endif

/* Minimum hand-presence score for a valid result. */
#ifndef AI_HANDLM_MIN_SCORE
#define AI_HANDLM_MIN_SCORE      (0.5f)
#endif

typedef struct {
  float32_t x;      /* NN-normalised [0..1] within the model input */
  float32_t y;
  float32_t z;      /* relative depth, wrist = 0, negative = toward camera */
} handlm_point_t;

typedef struct {
  bool           valid;                              /* score >= min_score  */
  float32_t      score;                              /* hand presence 0..1  */
  float32_t      handedness;                         /* 0 = left, 1 = right */
  handlm_point_t kps[AI_HANDLM_NB_LANDMARKS];        /* 21 landmarks        */
} handlm_result_t;

/* Latest result, valid until the next app_postprocess_handlm_ui_run(). */
extern handlm_result_t handlm_output;
extern float32_t       handlm_min_score;

/* Sets the presence gate. */
int32_t app_postprocess_handlm_ui_init(float min_score);
/* Decodes the outputs into handlm_output; -1 without a landmark output. */
int32_t app_postprocess_handlm_ui_run(NN_Instance_TypeDef *instance);

#ifdef __cplusplus
}
#endif

#endif /* APP_POSTPROCESS_HANDLM_UI_H */
