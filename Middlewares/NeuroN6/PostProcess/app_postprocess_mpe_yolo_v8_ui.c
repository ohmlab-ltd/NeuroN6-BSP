/**
 ******************************************************************************
 * @file    app_postprocess_mpe_yolo_v8_ui.c
 * @brief   The MPE YOLOv8 post-processing wrapper.
 *
 * Pattern is identical to app_postprocess_iseg_yolo_v8_ui.c:
 *   - app_postprocess_mpe_yolo_v8_ui_init(conf, iou)  sets up all params
 *   - app_postprocess_mpe_yolo_v8_ui_run(instance)     reads NN output
 *     buffers from the NN instance and runs post-processing
 *
 * Usage:
 *   app_postprocess_mpe_yolo_v8_ui_init(0.4f, 0.5f);
 *   ...
 *   app_postprocess_mpe_yolo_v8_ui_run(&NN_Instance_yolov8_mpe);
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

#include "app_postprocess_mpe_yolo_v8_ui.h"
#include <stddef.h>

/* --------------------------------------------------------------------------
 * Global outputs - accessible to the application via extern
 * -------------------------------------------------------------------------- */
mpe_pp_out_t                 mpe_pp_output;
mpe_yolov8_pp_static_param_t mpe_pp_params;

/* --------------------------------------------------------------------------
 * Internal buffers
 *
 * The float post-processing path (mpe_yolov8_pp_process) writes ALL pre-NMS
 * detections into pOutput->pOutBuff before running NMS/score filtering.
 * Therefore out_detections must be sized to the total number of raw anchor
 * boxes (AI_MPE_YOLOV8_PP_TOTAL_BOXES = 1344), not the final max_boxes_limit.
 * -------------------------------------------------------------------------- */

__attribute__((section(".ext_psram")))
static mpe_pp_outBuffer_t out_detections[AI_MPE_YOLOV8_PP_TOTAL_BOXES];

__attribute__((section(".ext_psram")))
static mpe_pp_keyPoints_t out_keyPoints[AI_MPE_YOLOV8_PP_TOTAL_BOXES
                                        * AI_POSE_PP_POSE_KEYPOINTS_NB];

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

/**
 * @brief  Initialise MPE post-processing buffers and parameters.
 *
 *         Mirrors app_postprocess_iseg_yolo_v8_ui_init(): thresholds are
 *         passed in rather than hardcoded, scratch/output buffers are linked,
 *         and mpe_yolov8_pp_reset() is called to prepare the library.
 *
 * @param  conf_threshold  Confidence threshold (e.g. 0.4)
 * @param  iou_threshold   NMS IoU threshold    (e.g. 0.5)
 * @return 0 on success, negative on error
 */
int32_t app_postprocess_mpe_yolo_v8_ui_init(float conf_threshold, float iou_threshold)
{
    mpe_pp_params.nb_classes      = AI_MPE_YOLOV8_PP_NB_CLASSES;
    mpe_pp_params.nb_total_boxes  = AI_MPE_YOLOV8_PP_TOTAL_BOXES;
    mpe_pp_params.max_boxes_limit = AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT;
    mpe_pp_params.conf_threshold  = conf_threshold;
    mpe_pp_params.iou_threshold   = iou_threshold;
    mpe_pp_params.nb_keypoints    = AI_POSE_PP_POSE_KEYPOINTS_NB;

    /* Link each output detection slot to its float keypoint array */
    for (int i = 0; i < AI_MPE_YOLOV8_PP_TOTAL_BOXES; i++)
    {
        out_detections[i].pKeyPoints =
            &out_keyPoints[i * AI_POSE_PP_POSE_KEYPOINTS_NB];
    }

    /* Float path does not use the int8 scratch buffer */
    mpe_pp_params.pScratchBuffer = NULL;

    return mpe_yolov8_pp_reset(&mpe_pp_params);
}

/**
 * @brief  Run MPE post-processing on the latest NN inference result.
 *
 *         Mirrors app_postprocess_iseg_yolo_v8_ui_run(): uses
 *         LL_ATON_Output_Buffers_Info() to locate the NN output buffer
 *         inside the instance, then calls the post-processing library.
 *
 * @param  instance  Pointer to the NN instance (e.g. &NN_Instance_yolov8_mpe)
 * @return 0 on success, negative on error
 */
int32_t app_postprocess_mpe_yolo_v8_ui_run(NN_Instance_TypeDef *instance)
{
    const LL_Buffer_InfoTypeDef *nn_out_info =
        LL_ATON_Output_Buffers_Info(instance);

    mpe_pp_output.pOutBuff = out_detections;
    mpe_pp_output.nb_detect = 0;

    /* One output, at least what the post-processing reads: for every
     * anchor, float32 box, class scores and three values a keypoint. A
     * different network is refused rather than read past. */
    if ((nn_out_info == NULL) || (nn_out_info[0].name == NULL)) {
        return AI_MPE_PP_ERROR;
    }
    {
        const uint32_t want = (uint32_t)(4 + mpe_pp_params.nb_classes +
                                         (int32_t)(3U * mpe_pp_params.nb_keypoints)) *
                              (uint32_t)mpe_pp_params.nb_total_boxes * (uint32_t)sizeof(float32_t);
        const uint8_t *b0 = LL_Buffer_addr_start(&nn_out_info[0]);

        if ((b0 == NULL) || ((uint32_t)(LL_Buffer_addr_end(&nn_out_info[0]) - b0) < want)) {
            return AI_MPE_PP_ERROR;
        }
    }

    mpe_yolov8_pp_in_centroid_t pp_input = {
        .pRaw_detections = (float32_t *)LL_Buffer_addr_start(&nn_out_info[0])
    };

    return mpe_yolov8_pp_process(&pp_input, &mpe_pp_output, &mpe_pp_params);
}
