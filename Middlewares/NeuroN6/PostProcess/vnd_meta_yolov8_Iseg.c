/**
 * @file    vnd_meta_yolov8_Iseg.c
 * @brief   Instance segmentation metadata building for USB streaming
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

#include "vnd_meta_yolov8_Iseg.h"
#include "app_postprocess_iseg_yolo_v8_ui.h"
#include "NeuroN6.h"   /* nn_norm_to_cam_norm_x/y for AspectMode-aware coords */
#include "n6_host_contract.h"   /* N6_STATIC_ASSERT */
#include <string.h>

/* The box count, and each object's id, are one byte. */
N6_STATIC_ASSERT(AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT <= 255, "more detections a frame than a one-byte count holds");

/* Inference timestamps set by the application (ts[0] pre-inference, ts[1] post). */
extern uint32_t ts[];

/* Iseg helpers. */

static inline const char *class_name_from_id(uint32_t cls)
{
  if (cls < NB_CLASSES)
    return classes_table[cls];
  return "unknown";
}

static uint16_t pack_mask_roi_1bpp_thresh(
    uint8_t *dst,
    const uint8_t *srcMxM,
    uint16_t M,
    uint16_t x0,
    uint16_t y0,
    uint16_t W,
    uint16_t H,
    uint8_t thr)
{
  const uint32_t nbits  = (uint32_t)W * (uint32_t)H;
  const uint32_t nbytes = (nbits + 7u) / 8u;
  memset(dst, 0, nbytes);

  uint32_t bit = 0;
  for (uint16_t y = 0; y < H; y++) {
    const uint32_t src_row = (uint32_t)(y0 + y) * (uint32_t)M;
    for (uint16_t x = 0; x < W; x++, bit++) {
      const uint32_t si = src_row + (uint32_t)(x0 + x);
      if (srcMxM[si] >= thr) {
        dst[bit >> 3] |= (uint8_t)(1u << (bit & 7u));
      }
    }
  }
  return (uint16_t)nbytes;
}

/* A box edge in mask cells, clamped to [0, M]: NaN, and anything past
 * either side of the mask, lands on its edge. */
static int mask_edge(float32_t v, uint8_t M)
{
  if (!(v > 0.0f)) return 0;
  if (v >= (float32_t)M) return (int)M;
  return (int)v;
}

static bool meta_put_iseg_mask_bbox_local(
    vnd_meta_builder_t *meta,
    uint8_t obj_id,
    const uint8_t *maskMxM,
    uint8_t M,
    float32_t xc_c, float32_t yc_c, float32_t w_c, float32_t h_c)
{
  if (!meta || !maskMxM || M == 0) return false;

  float32_t l = (xc_c - 0.5f * w_c) * (float32_t)M;
  float32_t r = (xc_c + 0.5f * w_c) * (float32_t)M;
  float32_t t = (yc_c - 0.5f * h_c) * (float32_t)M;
  float32_t b = (yc_c + 0.5f * h_c) * (float32_t)M;

  /* A box starting past the mask's edge would otherwise start the region
   * outside it, and the packer would read past the mask. */
  int x0 = mask_edge(l, M);
  int x1 = mask_edge(r, M);
  int y0 = mask_edge(t, M);
  int y1 = mask_edge(b, M);

  if (x1 <= x0) { x0 = (x0 > 0) ? (x0 - 1) : 0; x1 = x0 + 1; }
  if (y1 <= y0) { y0 = (y0 > 0) ? (y0 - 1) : 0; y1 = y0 + 1; }

  uint16_t outW = (uint16_t)(x1 - x0);
  uint16_t outH = (uint16_t)(y1 - y0);

  if (outW > 255) outW = 255;
  if (outH > 255) outH = 255;

  const uint16_t mask_bytes =
      (uint16_t)(((uint32_t)outW * (uint32_t)outH + 7u) / 8u);

  if (meta->len + 3u + 5u + mask_bytes > meta->capacity) return false;

  meta->buf[meta->len + 0] = META_ISEG_MASK;
  vnd_meta_put_u16_le(&meta->buf[meta->len + 1], (uint16_t)(5u + mask_bytes));
  meta->len += 3u;

  meta->buf[meta->len + 0] = obj_id;
  meta->buf[meta->len + 1] = (uint8_t)outW;
  meta->buf[meta->len + 2] = (uint8_t)outH;
  vnd_meta_put_u16_le(&meta->buf[meta->len + 3], mask_bytes);
  meta->len += 5u;

#ifndef ISEG_MASK_BIN_THR
#define ISEG_MASK_BIN_THR 1
#endif

  (void)pack_mask_roi_1bpp_thresh(
      &meta->buf[meta->len],
      maskMxM,
      (uint16_t)M,
      (uint16_t)x0,
      (uint16_t)y0,
      outW,
      outH,
      (uint8_t)ISEG_MASK_BIN_THR);

  meta->len += mask_bytes;
  return true;
}

/* Public API. */

void vnd_meta_build_iseg_on_slot(const iseg_pp_out_t *out, uint32_t inference_ms,
                                 const iseg_yolov8_pp_static_param_t *pp_params,
                                 vnd_meta_slot_t slot)
{
  vnd_meta_builder_t meta;
  vnd_meta_builder_begin(&meta, slot);

  /* META_INFERENCE_MS and META_BOXES are frame-global (one each a frame, a
   * box's obj_id its position): only the primary slot (MODEL_0) emits them.
   * Other slots emit META_ISEG_CLASSSTR and META_ISEG_MASK, whose obj_ids
   * count from 0 in every slot, so the host draws one model's results
   * correctly. */
  const bool emit_frame_global = (slot == VND_META_SLOT_MODEL_0);

  if (emit_frame_global) {
    uint16_t inf_ms_u16 = (inference_ms > 65535u) ? 65535u : (uint16_t)inference_ms;
    (void)vnd_meta_builder_put_tlv(&meta, META_INFERENCE_MS, &inf_ms_u16, sizeof(inf_ms_u16));
  }

  if (!out || out->nb_detect == 0)
  {
    vnd_meta_builder_finish(&meta);
    return;
  }

  const uint8_t M = (uint8_t)AI_YOLOV8_SEG_PP_MASK_SIZE;

  uint32_t n = out->nb_detect;

  if (n > AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT) n = AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT;

  uint8_t obj_id = 0;
  uint8_t boxes[1 + (AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT * (2*4 + 2))];
  uint32_t bl = 0;

  boxes[bl++] = 0;
  uint8_t kept = 0;
  for (uint32_t i = 0; i < n; i++)
  {
    const iseg_pp_outBuffer_t *d = &out->pOutBuff[i];

    if (pp_params && d->conf < pp_params->conf_threshold) continue;
    if (!d->pMask) continue;

    uint8_t this_id = obj_id;

    /* NN-normalised corners -> camera-normalised (nn_norm_to_cam_norm_*, the
     * identity under ASPECT_STRETCH); vnd_meta_put_box recovers centre and
     * size. Cut to the network's view first: the mask covers only that part,
     * and the host stretches it over the whole box. */
    float32_t nn_l = vnd_meta_clamp01(d->x_center - 0.5f * d->width);
    float32_t nn_r = vnd_meta_clamp01(d->x_center + 0.5f * d->width);
    float32_t nn_t = vnd_meta_clamp01(d->y_center - 0.5f * d->height);
    float32_t nn_b = vnd_meta_clamp01(d->y_center + 0.5f * d->height);

    float32_t cam_l = nn_norm_to_cam_norm_x(nn_l);
    float32_t cam_r = nn_norm_to_cam_norm_x(nn_r);
    float32_t cam_t = nn_norm_to_cam_norm_y(nn_t);
    float32_t cam_b = nn_norm_to_cam_norm_y(nn_b);

    uint8_t confq = vnd_meta_conf_to_q8(d->conf);

    const char *name = class_name_from_id((uint32_t)d->class_index);
    if (!name) name = "unknown";
    uint16_t name_len = (uint16_t)strlen(name);
    if (name_len > 200) name_len = 200;

    /* The mask, the name and room for the closing box list (ten bytes a box):
     * a box any of them does not fit is taken out whole, as the host cannot
     * draw a mask without its name or a frame without a box list. */
    const uint32_t mark = meta.len;
    const uint32_t list_room = emit_frame_global ? (3u + bl + 10u) : 0u;
    uint8_t tmp[1 + 200];

    tmp[0] = this_id;
    memcpy(&tmp[1], name, name_len);
    if (!meta_put_iseg_mask_bbox_local(
            &meta,
            this_id,
            (const uint8_t *)d->pMask,
            (uint8_t)M,
            d->x_center, d->y_center, d->width, d->height) ||
        !vnd_meta_builder_put_tlv(&meta, META_ISEG_CLASSSTR, tmp, (uint16_t)(1u + name_len)) ||
        ((meta.len + list_room) > meta.capacity))
    {
      meta.len = mark;
      continue;
    }

    vnd_meta_put_box(&boxes[bl], cam_l, cam_t, cam_r, cam_b); bl += 8;
    boxes[bl++] = confq;
    boxes[bl++] = (uint8_t)d->class_index;
    kept++;

    obj_id++;
  }
  boxes[0] = kept;
  if (emit_frame_global) {
    /* Primary slot only; see the top of this function. */
    (void)vnd_meta_builder_put_tlv(&meta, META_BOXES, boxes, (uint16_t)bl);
  }
  vnd_meta_builder_finish(&meta);
}

/* The same, on the primary slot. */
void vnd_meta_build_iseg(const iseg_pp_out_t *out, uint32_t inference_ms,
                         const iseg_yolov8_pp_static_param_t *pp_params)
{
  vnd_meta_build_iseg_on_slot(out, inference_ms, pp_params, VND_META_SLOT_PRIMARY);
}

void Display_NS_Yolo_V8_Iseg(void)
{
  vnd_meta_build_iseg_on_slot(&pp_output, ts[1] - ts[0], &pp_params,
                              VND_META_SLOT_PRIMARY);
}

void Display_NS_Yolo_V8_Iseg_OnSlot(vnd_meta_slot_t slot)
{
  vnd_meta_build_iseg_on_slot(&pp_output, ts[1] - ts[0], &pp_params, slot);
}
