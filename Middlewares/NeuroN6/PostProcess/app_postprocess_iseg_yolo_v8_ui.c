 /**
 ******************************************************************************
 * @file    app_postprocess_iseg_yolo_v8_ui.c
 * @author  GPM Application Team
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * SPDX-License-Identifier: SLA0044
 * This file follows STMicroelectronics' model-zoo post-processing and is
 * licensed under ST SLA0044, whose terms are in
 * Middlewares/NeuroN6/PostProcess/LICENSE-SLA0044.md. It is NOT covered by
 * this repository's BSD-3-Clause licence, and may be used only on, or with,
 * STMicroelectronics devices.
 ******************************************************************************
 */


#include "app_postprocess_iseg_yolo_v8_ui.h"
#include "tusb.h"
#include <assert.h>
#include <stdio.h>

/* Post-processing output and parameters - accessible to the application */
iseg_pp_out_t pp_output;
iseg_yolov8_pp_static_param_t pp_params;

/* Internal buffers. The three large mask/scratch buffers (~98 KB total) live in
 * external PSRAM (.ext_psram) instead of AXISRAM: the load-run RAM region is only
 * ~1.5 MB and hard-capped by the line-buffer band, so a WiFi + QSXGA + ISEG build
 * overflows it by ~48 KB. These are write-before-read scratch/output buffers
 * touched once per inference (CPU only, no DMA), so PSRAM latency is negligible and
 * the un-zeroed NOLOAD .ext_psram section is safe. The two small buffers (a handful
 * of KB) stay in AXISRAM. */
uint8_t _iseg_mask[AI_YOLOV8_SEG_PP_MASK_SIZE * AI_YOLOV8_SEG_PP_MASK_SIZE * AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT] __attribute__((section(".ext_psram")));
iseg_pp_outBuffer_t out_detections[AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT];
iseg_yolov8_pp_scratchBuffer_s8_t scratch_detections[AI_YOLOV8_SEG_PP_TOTAL_BOXES] __attribute__((section(".ext_psram")));
float32_t _out_buf_mask[AI_YOLOV8_SEG_PP_MASK_NB];
int8_t _out_buf_mask_s8[AI_YOLOV8_SEG_PP_MASK_NB * AI_YOLOV8_SEG_PP_TOTAL_BOXES] __attribute__((section(".ext_psram")));

/* Instance segmentation post-processing API functions */
int32_t app_postprocess_iseg_yolo_v8_ui_init(float conf_threshold, float iou_threshold)
{
  pp_params.nb_classes = AI_YOLOV8_SEG_PP_NB_CLASSES;
  pp_params.nb_total_boxes = AI_YOLOV8_SEG_PP_TOTAL_BOXES;
  pp_params.max_boxes_limit = AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT;
  pp_params.conf_threshold = conf_threshold;
  pp_params.iou_threshold = iou_threshold;
  pp_params.size_masks = AI_YOLOV8_SEG_PP_MASK_SIZE;
  pp_params.raw_output_scale = AI_YOLOV8_SEG_SCALE;
  pp_params.raw_output_zero_point = AI_YOLOV8_SEG_ZERO_POINT;
  pp_params.nb_masks = AI_YOLOV8_SEG_PP_MASK_NB;
  pp_params.mask_raw_output_zero_point = AI_YOLOV8_SEG_MASK_ZERO_POINT;
  pp_params.mask_raw_output_scale = AI_YOLOV8_SEG_MASK_SCALE;
  pp_params.pMask = _out_buf_mask;
  pp_params.pTmpBuff = scratch_detections;
  for (size_t i = 0; i < AI_YOLOV8_SEG_PP_TOTAL_BOXES; i++) {
    scratch_detections[i].pMask = &_out_buf_mask_s8[i * AI_YOLOV8_SEG_PP_MASK_NB];
  }
  for (size_t i = 0; i < AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT; i++) {
    out_detections[i].pMask = &_iseg_mask[i * AI_YOLOV8_SEG_PP_MASK_SIZE * AI_YOLOV8_SEG_PP_MASK_SIZE];
  }
  return iseg_yolov8_pp_reset(&pp_params);
}

int32_t app_postprocess_iseg_yolo_v8_ui_run(NN_Instance_TypeDef *instance)
{
  const LL_Buffer_InfoTypeDef *nn_out_info = LL_ATON_Output_Buffers_Info(instance);

  pp_output.pOutBuff = out_detections;
  pp_output.nb_detect = 0;

  /* Two outputs, each at least what the post-processing reads: int8 boxes,
   * classes and mask coefficients for every anchor, and the mask
   * prototypes. A different network is refused rather than read past. */
  if ((nn_out_info == NULL) || (nn_out_info[0].name == NULL) || (nn_out_info[1].name == NULL)) {
    return AI_ISEG_POSTPROCESS_ERROR_BAD_HW;
  }
  {
    const uint32_t want0 = (uint32_t)(4 + pp_params.nb_classes + pp_params.nb_masks) *
                           (uint32_t)pp_params.nb_total_boxes;
    const uint32_t want1 = (uint32_t)pp_params.nb_masks * (uint32_t)pp_params.size_masks *
                           (uint32_t)pp_params.size_masks;
    const uint8_t *b0 = LL_Buffer_addr_start(&nn_out_info[0]);
    const uint8_t *b1 = LL_Buffer_addr_start(&nn_out_info[1]);

    if ((b0 == NULL) || (b1 == NULL) ||
        ((uint32_t)(LL_Buffer_addr_end(&nn_out_info[0]) - b0) < want0) ||
        ((uint32_t)(LL_Buffer_addr_end(&nn_out_info[1]) - b1) < want1)) {
      return AI_ISEG_POSTPROCESS_ERROR_BAD_HW;
    }
  }

  iseg_yolov8_pp_in_centroid_t pp_input = {
    .pRaw_detections = (int8_t *)LL_Buffer_addr_start(&nn_out_info[0]),
    .pRaw_masks = (int8_t *)LL_Buffer_addr_start(&nn_out_info[1])
  };

  int32_t ret = iseg_yolov8_pp_process_int8(&pp_input, &pp_output, &pp_params);

  return ret;
}
