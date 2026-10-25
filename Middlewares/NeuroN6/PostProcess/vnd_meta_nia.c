/**
 * @file    vnd_meta_nia.c
 * @brief   Nia v0.5 detection metadata builder (see vnd_meta_nia.h). Plain
 *          detections only (no masks/keypoints); boxes map NN-normalised to
 *          camera-normalised [0..1] via nn_norm_to_cam_norm_* so Neuro Studio
 *          renders them correctly under any AspectMode.
 *
 * @attention
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 * This software is licensed under the terms in
 * Middlewares/NeuroN6/PostProcess/LICENSE-MIT.txt.
 */

#include "vnd_meta_nia.h"
#include "app_postprocess_nia_ui.h"   /* nia_pp_output, nia_pp_params */
#include "NeuroN6.h"                  /* nn_norm_to_cam_norm_x/y */
#include "n6_host_contract.h"         /* N6_STATIC_ASSERT */
#include <string.h>

/* The box count is one byte. */
N6_STATIC_ASSERT(NIA_PP_MAX_DET <= 255, "more detections a frame than a one-byte count holds");

/* Inference timestamps set by the application (ts[0] pre-inference, ts[1] post). */
extern uint32_t ts[];

/* COCO-80 class names. Reuses the shared, application-overridable classes_table
 * (defined weak in classes_table_default.c) so Nia and YOLOv8 share one table.
 * Declared locally to avoid pulling in the iseg post-processing header. */
#ifndef NB_CLASSES
#define NB_CLASSES (80)
#endif
extern const char *classes_table[NB_CLASSES];

static inline const char *nia_class_name(uint32_t cls)
{
  if (cls < (uint32_t)NB_CLASSES) return classes_table[cls];
  return "unknown";
}

/* Box list scratch (count byte + 10 bytes/box). File-scope (not on the loop
 * task stack) and in PSRAM to spare AXISRAM; the builder runs in the
 * application task alone, so no aliasing. */
__attribute__((section(".ext_psram")))
static uint8_t s_boxes[1 + (NIA_PP_MAX_DET * (2 * 4 + 2))];

void vnd_meta_build_nia_on_slot(const nia_pp_out_t *out, uint32_t inference_ms,
                                const nia_pp_params_t *params,
                                vnd_meta_slot_t slot)
{
  vnd_meta_builder_t meta;
  vnd_meta_builder_begin(&meta, slot);

  /* META_INFERENCE_MS and META_BOXES are frame-global: only the primary slot
   * emits them; other slots emit class strings only (see
   * vnd_meta_yolov8_Iseg.c). */
  const bool emit_frame_global = (slot == VND_META_SLOT_MODEL_0);

  if (emit_frame_global) {
    uint16_t inf_ms_u16 = (inference_ms > 65535u) ? 65535u : (uint16_t)inference_ms;
    (void)vnd_meta_builder_put_tlv(&meta, META_INFERENCE_MS, &inf_ms_u16, sizeof(inf_ms_u16));
  }

  if (!out || out->nb_detect <= 0) {
    vnd_meta_builder_finish(&meta);
    return;
  }

  uint32_t n = (uint32_t)out->nb_detect;
  if (n > (uint32_t)NIA_PP_MAX_DET) n = (uint32_t)NIA_PP_MAX_DET;

  uint32_t bl = 0;
  s_boxes[bl++] = 0;   /* count placeholder */
  uint8_t kept = 0;

  for (uint32_t i = 0; i < n; i++) {
    const nia_det_t *d = &out->pOutBuff[i];

    /* Defensive: nia_pp already thresholded, but honour params if present. */
    if (params && d->conf < params->conf_threshold) continue;

    /* NN-normalised [0..1] -> camera-normalised [0..1]: the two box corners
     * are transformed (the identity under ASPECT_STRETCH), and
     * vnd_meta_put_box recovers the centre and size from them. */
    float32_t nn_l = d->x_center - 0.5f * d->width;
    float32_t nn_r = d->x_center + 0.5f * d->width;
    float32_t nn_t = d->y_center - 0.5f * d->height;
    float32_t nn_b = d->y_center + 0.5f * d->height;

    float32_t cam_l = nn_norm_to_cam_norm_x(nn_l);
    float32_t cam_r = nn_norm_to_cam_norm_x(nn_r);
    float32_t cam_t = nn_norm_to_cam_norm_y(nn_t);
    float32_t cam_b = nn_norm_to_cam_norm_y(nn_b);

    uint8_t  confq = vnd_meta_conf_to_q8(d->conf);

    /* Per-detection class name. obj_id = positional index into META_BOXES. */
    const char *name = nia_class_name((uint32_t)d->class_index);
    uint16_t name_len = (uint16_t)strlen(name);
    if (name_len > 200) name_len = 200;

    {
      uint8_t tmp[1 + 200];
      tmp[0] = kept;
      memcpy(&tmp[1], name, name_len);
      if (!vnd_meta_builder_put_tlv(&meta, META_ISEG_CLASSSTR, tmp,
                                    (uint16_t)(1u + name_len))) {
        /* slot full - stop adding detections */
        break;
      }
    }

    vnd_meta_put_box(&s_boxes[bl], cam_l, cam_t, cam_r, cam_b); bl += 8;
    s_boxes[bl++] = confq;
    s_boxes[bl++] = (uint8_t)d->class_index;
    kept++;
  }

  s_boxes[0] = kept;
  if (emit_frame_global) {
    (void)vnd_meta_builder_put_tlv(&meta, META_BOXES, s_boxes, (uint16_t)bl);
  }
  vnd_meta_builder_finish(&meta);
}

void vnd_meta_build_nia(const nia_pp_out_t *out, uint32_t inference_ms,
                        const nia_pp_params_t *params)
{
  vnd_meta_build_nia_on_slot(out, inference_ms, params, VND_META_SLOT_PRIMARY);
}

void Display_NS_Nia(void)
{
  vnd_meta_build_nia_on_slot(&nia_pp_output, ts[1] - ts[0], &nia_pp_params,
                             VND_META_SLOT_PRIMARY);
}

void Display_NS_Nia_OnSlot(vnd_meta_slot_t slot)
{
  vnd_meta_build_nia_on_slot(&nia_pp_output, ts[1] - ts[0], &nia_pp_params, slot);
}
