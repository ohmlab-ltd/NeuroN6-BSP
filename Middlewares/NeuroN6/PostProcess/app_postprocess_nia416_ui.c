/**
 * @file    app_postprocess_nia416_ui.c
 * @brief   Init/run wrapper for the NIA Triad-416 detector (see the header).
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

#include "app_postprocess_nia416_ui.h"

nia_pp_out_t    nia416_pp_output;
nia_pp_params_t nia416_pp_params;

/* Kept-detection storage: write-before-read CPU buffer -> external PSRAM, to spare the tight AXISRAM load-run region. */
static nia_det_t s_out_detections[NIA416_PP_MAX_DET] __attribute__((section(".ext_psram")));

int32_t app_postprocess_nia416_ui_init(float conf_threshold, float iou_threshold)
{
  nia416_pp_params.conf_threshold = conf_threshold;
  nia416_pp_params.iou_threshold  = iou_threshold;
  nia416_pp_params.max_det        = NIA416_PP_MAX_DET;
  nia416_pp_output.pOutBuff       = s_out_detections;
  nia416_pp_output.nb_detect      = 0;
  return 0;
}

int32_t app_postprocess_nia416_ui_run(NN_Instance_TypeDef *instance)
{
  const LL_Buffer_InfoTypeDef *nn_out_info = LL_ATON_Output_Buffers_Info(instance);
  nia416_pp_output.pOutBuff = s_out_detections;   /* re-assert (idempotent) */
  return nia416_pp_process(nn_out_info, &nia416_pp_output, &nia416_pp_params);
}
