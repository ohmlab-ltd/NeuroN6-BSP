/**
 * @file    vnd_meta_pd.c
 * @brief   Palm-detector metadata building for USB streaming.
 *
 * Mirrors vnd_meta_yolov8_MPE.c: the palm box goes out as META_BOXES and the
 * 7 landmarks as META_KEYPOINTS + META_BINDINGS, so Neuro Studio's existing
 * skeleton renderer draws hands with no host-side change.
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

#include "vnd_meta_pd.h"
#include "NeuroN6.h"   /* nn_norm_to_cam_norm_x/y for AspectMode-aware coords */
#include "n6_host_contract.h"   /* N6_STATIC_ASSERT */

/* Inference timestamps set by the application (ts[0] pre-inference, ts[1] post). */
extern uint32_t ts[];

/* Palette slots (must match Neuro Studio's overlayColorFromId): 0 green,
 * 1 blue, 2 magenta, 3 orange. */
#define COLOR_GREEN   0
#define COLOR_BLUE    1
#define COLOR_MAGENTA 2
#define COLOR_ORANGE  3

/* MediaPipe palm landmarks: 0 wrist, 1 index MCP, 2 middle MCP, 3 ring MCP,
 * 4 pinky MCP, 5 thumb CMC, 6 thumb MCP. 0 and 2 are fixed by MediaPipe's
 * hand_detection_to_roi; the other labels are best-effort, and the skeleton
 * reads correctly either way. */
static const uint8_t s_bindings[][3] = {
  { 0, 1, COLOR_BLUE    },   /* wrist -> index MCP   */
  { 0, 2, COLOR_BLUE    },   /* wrist -> middle MCP  */
  { 0, 3, COLOR_BLUE    },   /* wrist -> ring MCP    */
  { 0, 4, COLOR_BLUE    },   /* wrist -> pinky MCP   */
  { 1, 2, COLOR_GREEN   },   /* knuckle chain        */
  { 2, 3, COLOR_GREEN   },
  { 3, 4, COLOR_GREEN   },
  { 0, 5, COLOR_MAGENTA },   /* wrist -> thumb CMC   */
  { 5, 6, COLOR_MAGENTA },   /* thumb CMC -> MCP     */
};
#define NUM_BINDINGS (sizeof(s_bindings) / sizeof(s_bindings[0]))

/* Box, keypoint and binding counts, and keypoint ids, are one byte each. */
N6_STATIC_ASSERT((AI_PD_PP_MAX_DETECTIONS * AI_PD_PP_NB_KEYPOINTS) <= 255,
                 "more keypoints a frame than a one-byte id can number");
N6_STATIC_ASSERT((AI_PD_PP_MAX_DETECTIONS * NUM_BINDINGS) <= 255,
                 "more bindings a frame than a one-byte count holds");

static const uint8_t s_kp_color[AI_PD_PP_NB_KEYPOINTS] = {
  COLOR_ORANGE,   /* 0 wrist      */
  COLOR_BLUE,     /* 1 index MCP  */
  COLOR_BLUE,     /* 2 middle MCP */
  COLOR_BLUE,     /* 3 ring MCP   */
  COLOR_BLUE,     /* 4 pinky MCP  */
  COLOR_MAGENTA,  /* 5 thumb CMC  */
  COLOR_MAGENTA,  /* 6 thumb MCP  */
};

static inline float32_t clamp01(float32_t v)
{
  if (v < 0.f) return 0.f;
  if (v > 1.f) return 1.f;
  return v;
}

void vnd_meta_build_pd_on_slot(const pd_ui_detection_t *dets, uint32_t nb_dets,
                               uint32_t inference_ms, vnd_meta_slot_t slot)
{
  vnd_meta_builder_t meta;
  vnd_meta_builder_begin(&meta, slot);

  /* META_INFERENCE_MS and META_BOXES are frame-global: primary slot only
   * (see vnd_meta_yolov8_Iseg.c). */
  const bool emit_frame_global = (slot == VND_META_SLOT_MODEL_0);

  if (emit_frame_global) {
    uint16_t inf_ms_u16 = (inference_ms > 65535u) ? 65535u : (uint16_t)inference_ms;
    (void)vnd_meta_builder_put_tlv(&meta, META_INFERENCE_MS, &inf_ms_u16, sizeof(inf_ms_u16));
  }

  if (dets == NULL || nb_dets == 0u) {
    vnd_meta_builder_finish(&meta);
    return;
  }
  if (nb_dets > (uint32_t)AI_PD_PP_MAX_DETECTIONS) nb_dets = (uint32_t)AI_PD_PP_MAX_DETECTIONS;

  const uint32_t NKP = (uint32_t)AI_PD_PP_NB_KEYPOINTS;   /* 7 */
  const uint32_t NBI = (uint32_t)NUM_BINDINGS;            /* 9 */

  uint8_t  boxes[1 + AI_PD_PP_MAX_DETECTIONS * 10];
  uint32_t bl = 0u;
  boxes[bl++] = 0u;                                        /* count placeholder */

  uint8_t  kp_buf[1 + AI_PD_PP_MAX_DETECTIONS * AI_PD_PP_NB_KEYPOINTS * 7];
  uint32_t kl = 0u;
  kp_buf[kl++] = 0u;

  uint8_t  bi_buf[1 + AI_PD_PP_MAX_DETECTIONS * 9 * 3];
  uint32_t bil = 0u;
  bi_buf[bil++] = 0u;

  uint8_t  kept = 0u;
  uint16_t total_kps = 0u;
  uint16_t total_binds = 0u;

  for (uint32_t d = 0u; d < nb_dets; d++) {
    const pd_ui_detection_t *det = &dets[d];
    const uint8_t id_base = (uint8_t)(kept * NKP);

    /* Box: NN-norm -> camera-norm (identity under ASPECT_STRETCH; offset +
     * scale under ASPECT_CROP_*). The corners are transformed, and
     * vnd_meta_put_box re-derives the centre and extent from them. */
    const float32_t cam_l = nn_norm_to_cam_norm_x(det->x_center - 0.5f * det->width);
    const float32_t cam_r = nn_norm_to_cam_norm_x(det->x_center + 0.5f * det->width);
    const float32_t cam_t = nn_norm_to_cam_norm_y(det->y_center - 0.5f * det->height);
    const float32_t cam_b = nn_norm_to_cam_norm_y(det->y_center + 0.5f * det->height);

    vnd_meta_put_box(&boxes[bl], cam_l, cam_t, cam_r, cam_b); bl += 8;
    boxes[bl++] = vnd_meta_conf_to_q8(det->prob);
    boxes[bl++] = 0u;   /* single-class model: class_index 0 = "palm" */

    /* Landmarks: same NN->camera transform so the dots land on the right
     * spot of the streamed image. The detector gives no per-landmark score,
     * so each inherits the palm's confidence. */
    for (uint32_t k = 0u; k < NKP; k++) {
      const float32_t kx = clamp01(nn_norm_to_cam_norm_x(det->kps[k].x));
      const float32_t ky = clamp01(nn_norm_to_cam_norm_y(det->kps[k].y));

      kp_buf[kl++] = id_base + (uint8_t)k;                       /* id      */
      kp_buf[kl++] = s_kp_color[k];                              /* colorId */
      vnd_meta_put_u16_le(&kp_buf[kl], vnd_meta_f01_to_u16(kx)); kl += 2;
      vnd_meta_put_u16_le(&kp_buf[kl], vnd_meta_f01_to_u16(ky)); kl += 2;
      kp_buf[kl++] = vnd_meta_conf_to_q8(det->prob);             /* conf    */
    }
    total_kps += (uint16_t)NKP;

    for (uint32_t b = 0u; b < NBI; b++) {
      bi_buf[bil++] = id_base + s_bindings[b][0];
      bi_buf[bil++] = id_base + s_bindings[b][1];
      bi_buf[bil++] = s_bindings[b][2];
    }
    total_binds += (uint16_t)NBI;

    kept++;
  }

  kp_buf[0] = (uint8_t)total_kps;
  (void)vnd_meta_builder_put_tlv(&meta, META_KEYPOINTS, kp_buf, (uint16_t)kl);

  bi_buf[0] = (uint8_t)total_binds;
  (void)vnd_meta_builder_put_tlv(&meta, META_BINDINGS, bi_buf, (uint16_t)bil);

  if (emit_frame_global) {
    boxes[0] = kept;
    (void)vnd_meta_builder_put_tlv(&meta, META_BOXES, boxes, (uint16_t)bl);
  }

  vnd_meta_builder_finish(&meta);
}

void Display_NS_PalmDetector(void)
{
  vnd_meta_build_pd_on_slot(pd_ui_detections, pd_ui_nb_detections,
                            ts[1] - ts[0], VND_META_SLOT_PRIMARY);
}

void Display_NS_PalmDetector_OnSlot(vnd_meta_slot_t slot)
{
  vnd_meta_build_pd_on_slot(pd_ui_detections, pd_ui_nb_detections,
                            ts[1] - ts[0], slot);
}
