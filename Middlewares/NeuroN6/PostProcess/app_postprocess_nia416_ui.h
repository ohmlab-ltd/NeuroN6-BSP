/**
 * @file    app_postprocess_nia416_ui.h
 * @brief   Init/run wrapper for the NIA Triad-416 ship-distance detector.
 *
 *   - app_postprocess_nia416_ui_init(conf, iou)  sets thresholds + buffers
 *   - app_postprocess_nia416_ui_run(instance)    decodes the NN outputs
 *
 * Results land in nia416_pp_output, using the same nia_pp_out_t / nia_det_t
 * types as the v0.5 flagship detector, so the NIA metadata builder and LCD
 * overlay consume them unchanged.
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

#ifndef APP_POSTPROCESS_NIA416_UI_H
#define APP_POSTPROCESS_NIA416_UI_H

#include <stdint.h>
#include <stdbool.h>

#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "nia416_pp.h"
#include "ll_aton_rt_user_api.h"

extern nia_pp_out_t    nia416_pp_output;
extern nia_pp_params_t nia416_pp_params;

int32_t app_postprocess_nia416_ui_init(float conf_threshold, float iou_threshold);
int32_t app_postprocess_nia416_ui_run(NN_Instance_TypeDef *instance);

#ifdef __cplusplus
}
#endif

#endif /* APP_POSTPROCESS_NIA416_UI_H */
