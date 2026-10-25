/**
 * @file    vnd_meta_yolov8_MPE.c
 * @brief   Multi-person pose estimation metadata building for USB streaming
 *
 * Encodes the same data that LCD_Overlay_MPE.c draws on the LCD into
 * TLV metadata for the host application (NeuroStudio).
 *
 * TLV types emitted: META_INFERENCE_MS, META_KEYPOINTS, META_BINDINGS, META_BOXES
 *
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

#include "vnd_meta_yolov8_MPE.h"
#include "app_postprocess_mpe_yolo_v8_ui.h"
#include "NeuroN6.h"   /* nn_norm_to_cam_norm_x/y for AspectMode-aware coords */
#include "n6_host_contract.h"   /* N6_STATIC_ASSERT */

/* Inference timestamps set by the application (ts[0] pre-inference, ts[1] post). */
extern uint32_t ts[];

/* Body-part colour IDs (must match Neuro Studio's overlayColorFromId):
 * 0 head (green), 1 arms (blue), 2 trunk (magenta), 3 legs (orange). */

#define COLOR_HEAD  0
#define COLOR_ARMS  1
#define COLOR_TRUNK 2
#define COLOR_LEGS  3

/* COCO 17-keypoint skeleton: 19 bindings {kp_a, kp_b, colorId}
 * Matches LCD_Overlay_MPE.c s_bindings exactly. */
static const uint8_t s_bindings[][3] = {
  {15, 13, COLOR_LEGS},
  {13, 11, COLOR_LEGS},
  {16, 14, COLOR_LEGS},
  {14, 12, COLOR_LEGS},
  {11, 12, COLOR_TRUNK},
  { 5, 11, COLOR_TRUNK},
  { 6, 12, COLOR_TRUNK},
  { 5,  6, COLOR_ARMS},
  { 5,  7, COLOR_ARMS},
  { 6,  8, COLOR_ARMS},
  { 7,  9, COLOR_ARMS},
  { 8, 10, COLOR_ARMS},
  { 1,  2, COLOR_HEAD},
  { 0,  1, COLOR_HEAD},
  { 0,  2, COLOR_HEAD},
  { 1,  3, COLOR_HEAD},
  { 2,  4, COLOR_HEAD},
  { 3,  5, COLOR_HEAD},
  { 4,  6, COLOR_HEAD},
};

#define NUM_BINDINGS (sizeof(s_bindings) / sizeof(s_bindings[0]))

/* Box, keypoint and binding counts, and keypoint ids, are one byte each. */
N6_STATIC_ASSERT((AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT * AI_POSE_PP_POSE_KEYPOINTS_NB) <= 255,
                 "more keypoints a frame than a one-byte id can number");
N6_STATIC_ASSERT((AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT * NUM_BINDINGS) <= 255,
                 "more bindings a frame than a one-byte count holds");

/* Per-keypoint colour ID by body part (COCO keypoints): 0-4 HEAD, 5-10 ARMS,
 * 11-12 TRUNK, 13-16 LEGS. The shoulders are in ARMS colour, as on the LCD. */
static const uint8_t s_kp_color[17] = {
  COLOR_HEAD,  /* 0  nose          */
  COLOR_HEAD,  /* 1  left_eye      */
  COLOR_HEAD,  /* 2  right_eye     */
  COLOR_HEAD,  /* 3  left_ear      */
  COLOR_HEAD,  /* 4  right_ear     */
  COLOR_ARMS,  /* 5  left_shoulder */
  COLOR_ARMS,  /* 6  right_shoulder*/
  COLOR_ARMS,  /* 7  left_elbow    */
  COLOR_ARMS,  /* 8  right_elbow   */
  COLOR_ARMS,  /* 9  left_wrist    */
  COLOR_ARMS,  /* 10 right_wrist   */
  COLOR_TRUNK, /* 11 left_hip      */
  COLOR_TRUNK, /* 12 right_hip     */
  COLOR_LEGS,  /* 13 left_knee     */
  COLOR_LEGS,  /* 14 right_knee    */
  COLOR_LEGS,  /* 15 left_ankle    */
  COLOR_LEGS,  /* 16 right_ankle   */
};

/* Public API. */

void vnd_meta_build_mpe_on_slot(const mpe_pp_out_t *out, uint32_t inference_ms,
                                const mpe_yolov8_pp_static_param_t *pp_params,
                                vnd_meta_slot_t slot)
{
  vnd_meta_builder_t meta;
  vnd_meta_builder_begin(&meta, slot);

  /* META_INFERENCE_MS and META_BOXES are frame-global: primary slot only (see
   * vnd_meta_yolov8_Iseg.c). Keypoint ids count from 0 in every slot and the
   * host keeps one map, so a second model's skeletons overwrite the primary's. */
  const bool emit_frame_global = (slot == VND_META_SLOT_MODEL_0);

  /* 1. Inference time. */
  if (emit_frame_global) {
    uint16_t inf_ms_u16 = (inference_ms > 65535u) ? 65535u : (uint16_t)inference_ms;
    (void)vnd_meta_builder_put_tlv(&meta, META_INFERENCE_MS, &inf_ms_u16, sizeof(inf_ms_u16));
  }

  if (!out || out->nb_detect == 0)
  {
    vnd_meta_builder_finish(&meta);
    return;
  }

  uint32_t n = (uint32_t)out->nb_detect;
  if (n > (uint32_t)AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT)
    n = (uint32_t)AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT;

  const uint32_t NKP = AI_POSE_PP_POSE_KEYPOINTS_NB;   /* 17 */
  const uint32_t NBI = NUM_BINDINGS;                     /* 19 */

  /* 2. Boxes, keypoints and bindings per person. */

  /* Worst-case buffer sizes */
  uint8_t boxes[1 + AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT * 10];
  uint32_t bl = 0;
  boxes[bl++] = 0; /* count placeholder */

  uint8_t kp_buf[1 + AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT * 17 * 7];
  uint32_t kl = 0;
  kp_buf[kl++] = 0; /* count placeholder */

  uint8_t bi_buf[1 + AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT * 19 * 3];
  uint32_t bil = 0;
  bi_buf[bil++] = 0; /* count placeholder */

  uint8_t kept_persons = 0;
  uint16_t total_kps = 0;
  uint16_t total_binds = 0;

  for (uint32_t d = 0; d < n; d++)
  {
    const mpe_pp_outBuffer_t *det = &out->pOutBuff[d];
    if (!det || !det->pKeyPoints) continue;
    if (pp_params && det->conf < pp_params->conf_threshold) continue;

    const uint8_t id_base = (uint8_t)(kept_persons * NKP);

    /* Box: NN-norm -> camera-norm (identity under ASPECT_STRETCH, offset and
     * scale under ASPECT_CROP_*); vnd_meta_put_box re-derives centre and extent. */
    float32_t nn_l = det->x_center - 0.5f * det->width;
    float32_t nn_r = det->x_center + 0.5f * det->width;
    float32_t nn_t = det->y_center - 0.5f * det->height;
    float32_t nn_b = det->y_center + 0.5f * det->height;

    float32_t cam_l = nn_norm_to_cam_norm_x(nn_l);
    float32_t cam_r = nn_norm_to_cam_norm_x(nn_r);
    float32_t cam_t = nn_norm_to_cam_norm_y(nn_t);
    float32_t cam_b = nn_norm_to_cam_norm_y(nn_b);

    vnd_meta_put_box(&boxes[bl], cam_l, cam_t, cam_r, cam_b); bl += 8;
    boxes[bl++] = vnd_meta_conf_to_q8(det->conf);
    boxes[bl++] = (uint8_t)det->class_index;

    /* Keypoints (17 per person), transformed as the box. */
    for (uint32_t k = 0; k < NKP; k++)
    {
      const mpe_pp_keyPoints_t *kp = &det->pKeyPoints[k];

      /* vnd_meta_f01_to_u16 clamps each to the frame. */
      float32_t kx_cam = nn_norm_to_cam_norm_x(kp->x);
      float32_t ky_cam = nn_norm_to_cam_norm_y(kp->y);

      kp_buf[kl++] = id_base + (uint8_t)k;            /* id       */
      kp_buf[kl++] = s_kp_color[k];                    /* colorId  */
      vnd_meta_put_u16_le(&kp_buf[kl],
                          vnd_meta_f01_to_u16(kx_cam));
      kl += 2;                                          /* x u16le  */
      vnd_meta_put_u16_le(&kp_buf[kl],
                          vnd_meta_f01_to_u16(ky_cam));
      kl += 2;                                          /* y u16le  */
      kp_buf[kl++] = vnd_meta_conf_to_q8(kp->conf);   /* conf u8  */
    }
    total_kps += (uint16_t)NKP;

    /* Bindings (19 per person), referencing offset ids. */
    for (uint32_t b = 0; b < NBI; b++)
    {
      bi_buf[bil++] = id_base + s_bindings[b][0];  /* kp_a     */
      bi_buf[bil++] = id_base + s_bindings[b][1];  /* kp_b     */
      bi_buf[bil++] = s_bindings[b][2];             /* colorId  */
    }
    total_binds += (uint16_t)NBI;

    kept_persons++;
  }

  /* 3. Emit TLVs. */

  /* Keypoints */
  kp_buf[0] = (uint8_t)total_kps;
  (void)vnd_meta_builder_put_tlv(&meta, META_KEYPOINTS, kp_buf, (uint16_t)kl);

  /* Bindings */
  bi_buf[0] = (uint8_t)total_binds;
  (void)vnd_meta_builder_put_tlv(&meta, META_BINDINGS, bi_buf, (uint16_t)bil);

  /* Boxes: primary slot only. Beside an ISEG model on MODEL_0, MPE boxes
   * would take the ISEG masks (matched by position); the skeleton needs no
   * box. */
  if (emit_frame_global) {
    boxes[0] = kept_persons;
    (void)vnd_meta_builder_put_tlv(&meta, META_BOXES, boxes, (uint16_t)bl);
  }

  vnd_meta_builder_finish(&meta);
}

/* The same, on the primary slot. */
void vnd_meta_build_mpe(const mpe_pp_out_t *out, uint32_t inference_ms,
                        const mpe_yolov8_pp_static_param_t *pp_params)
{
  vnd_meta_build_mpe_on_slot(out, inference_ms, pp_params, VND_META_SLOT_PRIMARY);
}

void Display_NS_Yolo_V8_MPE(void)
{
  vnd_meta_build_mpe_on_slot(&mpe_pp_output, ts[1] - ts[0], &mpe_pp_params,
                             VND_META_SLOT_PRIMARY);
}

void Display_NS_Yolo_V8_MPE_OnSlot(vnd_meta_slot_t slot)
{
  vnd_meta_build_mpe_on_slot(&mpe_pp_output, ts[1] - ts[0], &mpe_pp_params, slot);
}
