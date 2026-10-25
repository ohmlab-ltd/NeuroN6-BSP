/**
 * @file    app_postprocess_nia_ui.h
 * @brief   Application-facing init/run wrapper for the Nia v0.5 detector
 *          post-processing. Mirrors app_postprocess_iseg_yolo_v8_ui.h.
 *
 * Holds the file-scope output (nia_pp_output) + params (nia_pp_params) that the
 * metadata builder (vnd_meta_nia) and the application read after run().
 *
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
 */

#ifndef APP_POSTPROCESS_NIA_UI_H
#define APP_POSTPROCESS_NIA_UI_H

#include <stdint.h>
#include <stdbool.h>

/* arm_math_types.h OUTSIDE extern "C" (and before it) so its include guard is set before nia_pp.h re-includes it within extern "C". */
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "nia_pp.h"
#include "ll_aton_rt_user_api.h"

/* Globals accessible from other modules and the application. */
extern nia_pp_out_t    nia_pp_output;
extern nia_pp_params_t nia_pp_params;

/**
 * @brief  Initialise Nia post-processing (thresholds + output buffer).
 *         Defaults to NIA_PP_MAX_DET kept detections.
 */
int32_t app_postprocess_nia_ui_init(float conf_threshold, float iou_threshold);

/**
 * @brief  Run post-processing on the latest NPU outputs for `instance`.
 *         Fills nia_pp_output. Returns 0 on success.
 */
int32_t app_postprocess_nia_ui_run(NN_Instance_TypeDef *instance);

#ifdef __cplusplus
}
#endif

#endif /* APP_POSTPROCESS_NIA_UI_H */
