/**
 * @file    nia416_pp.c
 * @brief   Decode for the NIA Triad-416 ship-distance detector (see nia416_pp.h).
 *
 * Threshold-before-sigmoid on the class logits, anchor-add + clip on the
 * survivors only, then class-aware NMS at IoU 0.70.
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

#include "nia416_pp.h"
#include <math.h>
#include <stdlib.h>

/* Helium (MVE) gate, 16 int8 lanes a compare; scalar fallback without MVE. */
#if defined(__ARM_FEATURE_MVE) && ((__ARM_FEATURE_MVE) & 1)
#  include <arm_mve.h>
#  define NIA416_PP_USE_MVE 1
#  pragma message("nia416_pp: Helium MVE gate ENABLED (16 int8 lanes/compare)")
#else
#  define NIA416_PP_USE_MVE 0
#  pragma message("nia416_pp: MVE unavailable - scalar gate (loop-order fix still active)")
#endif

/* Level geometry: strides 8/16/32 over a 416 px input -> 52/26/13 grids; 2704 + 676 + 169 = 3549 anchors total. */
static const int NIA416_STRIDES[3] = { 8, 16, 32 };
static const int NIA416_GRID[3]    = { 52, 26, 13 };

typedef struct {
  float   x1, y1, x2, y2;   /* pixel-space corners in the 416 frame */
  float   score;
  int16_t cls;
} nia416_cand_t;

/* Candidate scratch: write-before-read CPU buffer, kept out of the tight AXISRAM load-run region. */
static nia416_cand_t s_cand[NIA416_PP_MAX_CAND] __attribute__((section(".ext_psram")));
static uint8_t       s_suppressed[NIA416_PP_MAX_CAND] __attribute__((section(".ext_psram")));

static int nia416_cand_cmp_desc(const void *pa, const void *pb)
{
  const nia416_cand_t *a = (const nia416_cand_t *)pa;
  const nia416_cand_t *b = (const nia416_cand_t *)pb;
  if (a->score < b->score) return  1;
  if (a->score > b->score) return -1;
  return 0;
}

static inline float nia416_iou(const nia416_cand_t *a, const nia416_cand_t *b)
{
  const float aa = (a->x2 - a->x1) * (a->y2 - a->y1);
  const float ba = (b->x2 - b->x1) * (b->y2 - b->y1);
  if (aa <= 0.f || ba <= 0.f) return 0.f;

  const float ix1 = (a->x1 > b->x1) ? a->x1 : b->x1;
  const float iy1 = (a->y1 > b->y1) ? a->y1 : b->y1;
  const float ix2 = (a->x2 < b->x2) ? a->x2 : b->x2;
  const float iy2 = (a->y2 < b->y2) ? a->y2 : b->y2;
  const float iw  = (ix2 > ix1) ? (ix2 - ix1) : 0.f;
  const float ih  = (iy2 > iy1) ? (iy2 - iy1) : 0.f;
  const float inter = iw * ih;

  const float uni = aa + ba - inter;
  return (uni > 0.f) ? (inter / uni) : 0.f;
}

int32_t nia416_pp_process(const LL_Buffer_InfoTypeDef *outs,
                          nia_pp_out_t *out,
                          const nia_pp_params_t *params)
{
  if (!outs || !out || !out->pOutBuff || !params) return -1;

  out->nb_detect = 0;

  /* Map the six outputs to (cls[level], dist[level]) by shape, not index, so a
   * re-export that reorders them still works: NCHW {1,C,H,W}, C == NB_CLASSES
   * class logits, C == 4 LTRB distances; grid 52/26/13 -> level 0/1/2.
   * Outputs are INT8 (STAI_FORMAT_S8) with per-tensor scale and zero point
   * read from the runtime; read as float they are noise and overrun 4x. */
  const int8_t *cls_ptr[3] = { 0, 0, 0 };
  const int8_t *dst_ptr[3] = { 0, 0, 0 };
  float cls_scale[3] = { 0 }, dst_scale[3] = { 0 };
  int   cls_zp[3]    = { 0 }, dst_zp[3]    = { 0 };

  for (int i = 0; outs[i].name != NULL; i++) {
    const LL_Buffer_InfoTypeDef *b = &outs[i];
    if (!b->mem_shape || b->mem_ndims < 4) continue;
    const uint32_t C = b->mem_shape[1];
    const uint32_t G = b->mem_shape[2];
    const int lvl = (G == 52u) ? 0 : (G == 26u) ? 1 : (G == 13u) ? 2 : -1;
    if (lvl < 0) continue;

    const int8_t *p  = (const int8_t *)LL_Buffer_addr_start(b);
    const float   sc = b->scale  ? b->scale[0]       : 1.0f;
    const int     zp = b->offset ? (int)b->offset[0] : 0;

    if (C == (uint32_t)NIA416_PP_NB_CLASSES) {
      cls_ptr[lvl] = p; cls_scale[lvl] = sc; cls_zp[lvl] = zp;
    } else if (C == 4u) {
      dst_ptr[lvl] = p; dst_scale[lvl] = sc; dst_zp[lvl] = zp;
    }
  }

  /* Threshold the LOGITS directly: sigmoid is monotonic, so
   * sigmoid(z) > conf  <=>  z > logit(conf) = -log(1/conf - 1),
   * which keeps expf() off the full 3549x80 logit grid. */
  const float conf_thr = params->conf_threshold;
  float logit_thr;
  if (conf_thr <= 0.0f)      logit_thr = -1e30f;
  else if (conf_thr >= 1.0f) logit_thr =  1e30f;
  else                       logit_thr = -logf(1.0f / conf_thr - 1.0f);

  int ncand = 0;

  /* Decode: threshold, anchor-add, clip; survivors only. */
  for (int lvl = 0; lvl < 3; lvl++) {
    if (!cls_ptr[lvl] || !dst_ptr[lvl]) continue;   /* missing tensor -> skip level */

    const int     s   = NIA416_STRIDES[lvl];
    const int     G   = NIA416_GRID[lvl];
    const int     HW  = G * G;
    const int8_t *cls = cls_ptr[lvl];
    const int8_t *dst = dst_ptr[lvl];
    const float   cs  = cls_scale[lvl]; const int cz = cls_zp[lvl];
    const float   ds  = dst_scale[lvl]; const int dz = dst_zp[lvl];

    /* Gate in the INT8 domain. Dequant is monotonic for scale > 0, so
     *   sigmoid((q - zp) * sc) > conf   <=>   q > logit_thr / sc + zp
     * - a plain integer compare, no dequant/expf() needed on the full grid. */
    int q_thr;
    {
      const float t = (cs > 0.f) ? (logit_thr / cs + (float)cz) : 0.f;
      if      (t < -128.0f) q_thr = -129;   /* every int8 passes */
      else if (t >  127.0f) q_thr =  127;   /* none passes       */
      else                  q_thr = (int)floorf(t);
    }

    /* Emit one survivor. MULTI-LABEL: an anchor may fire on several classes, so
     * every class above threshold becomes its own candidate - not an argmax.
     * A macro, used as the last statement of both the MVE lane loop and the
     * scalar tail loop: a `continue` inside it ends its do-while, which, being
     * last, ends that loop's pass for the current candidate. */
    #define NIA416_EMIT(CC, CELL, QQ)                                              \
      do {                                                                         \
        const int _cell = (CELL);                                                  \
        const int _x = _cell % G;                                                  \
        const int _y = _cell / G;                                                  \
        /* dt* dequantise to LTRB distances in PIXELS (ReLU + the in-graph         \
         * Conv1x1(diag(s,s,s,s)) already did the stride multiply, so there is     \
         * no further stride scaling - only the int8 dequant). */                  \
        const float dl = ((float)dst[0 * HW + _cell] - (float)dz) * ds;            \
        const float dt = ((float)dst[1 * HW + _cell] - (float)dz) * ds;            \
        const float dr = ((float)dst[2 * HW + _cell] - (float)dz) * ds;            \
        const float db = ((float)dst[3 * HW + _cell] - (float)dz) * ds;            \
        const float cx = ((float)_x + 0.5f) * (float)s;                            \
        const float cy = ((float)_y + 0.5f) * (float)s;                            \
        /* Signs are load-bearing: left/top SUBTRACT, right/bottom ADD. */         \
        float x1 = cx - dl, y1 = cy - dt, x2 = cx + dr, y2 = cy + db;              \
        if (x1 < 0.f) x1 = 0.f;                                                    \
        if (y1 < 0.f) y1 = 0.f;                                                    \
        if (x2 > NIA416_PP_INPUT_DIM) x2 = NIA416_PP_INPUT_DIM;                    \
        if (y2 > NIA416_PP_INPUT_DIM) y2 = NIA416_PP_INPUT_DIM;                    \
        if (x2 <= x1 || y2 <= y1) continue;                                        \
        nia416_cand_t *cd = &s_cand[ncand++];                                      \
        cd->x1 = x1; cd->y1 = y1; cd->x2 = x2; cd->y2 = y2;                        \
        /* Dequantise the logit and sigmoid it - survivors only. */                \
        const float z = ((float)(QQ) - (float)cz) * cs;                            \
        cd->score = 1.0f / (1.0f + expf(-z));                                      \
        cd->cls   = (int16_t)(CC);                                                 \
      } while (0)

    /* CLASS-MAJOR sweep for cache locality (contiguous per class row). The
     * (cell,class) candidate set is unaffected; only emission order changes,
     * which does not matter since NMS sorts by score anyway. */
    for (int c = 0; c < NIA416_PP_NB_CLASSES; c++) {
      const int8_t *row = cls + (size_t)c * (size_t)HW;
      int cell = 0;

#if NIA416_PP_USE_MVE
      /* Helium: 16 int8 lanes per compare, one vldrb+vcmpgt per 16 cells. Guarded
       * to the int8-representable range; q_thr == -129 (all pass) or >= 127 (none
       * pass) fall through to the scalar path below instead. */
      if (q_thr >= -128 && q_thr <= 126) {
        const int8x16_t vthr = vdupq_n_s8((int8_t)q_thr);
        for (; cell + 16 <= HW; cell += 16) {
          const int8x16_t v = vld1q_s8(&row[cell]);
          mve_pred16_t p = vcmpgtq_s8(v, vthr);
          if (p == 0) continue;                      /* the overwhelming case */
          for (int k = 0; k < 16; k++) {
            if ((p & (mve_pred16_t)(1u << k)) == 0) continue;
            if (ncand >= NIA416_PP_MAX_CAND) break;
            NIA416_EMIT(c, cell + k, row[cell + k]);
          }
          if (ncand >= NIA416_PP_MAX_CAND) break;
        }
      }
#endif
      /* Scalar tail (and the whole row when MVE is unavailable/out of range). */
      for (; cell < HW; cell++) {
        const int q = (int)row[cell];
        if (q <= q_thr) continue;
        if (ncand >= NIA416_PP_MAX_CAND) break;
        NIA416_EMIT(c, cell, q);
      }
      if (ncand >= NIA416_PP_MAX_CAND) break;
    }
    #undef NIA416_EMIT
    if (ncand >= NIA416_PP_MAX_CAND) break;
  }

  if (ncand == 0) return 0;

  /* Class-aware NMS: one sort by score, skipping pairs of different classes,
   * equals NMS per class. */
  qsort(s_cand, (size_t)ncand, sizeof(s_cand[0]), nia416_cand_cmp_desc);

  for (int i = 0; i < ncand; i++) s_suppressed[i] = 0u;

  const float iou_thr = params->iou_threshold;
  const int   max_det = (params->max_det > 0) ? params->max_det : NIA416_PP_MAX_DET;
  int kept = 0;

  for (int i = 0; i < ncand && kept < max_det; i++) {
    if (s_suppressed[i]) continue;

    const nia416_cand_t *w = &s_cand[i];

    /* Publish, converting pixel corners -> NN-normalised centre+size (the convention nia_det_t and its consumers expect). */
    nia_det_t *d = &out->pOutBuff[kept++];
    d->x_center    = 0.5f * (w->x1 + w->x2) / NIA416_PP_INPUT_DIM;
    d->y_center    = 0.5f * (w->y1 + w->y2) / NIA416_PP_INPUT_DIM;
    d->width       = (w->x2 - w->x1) / NIA416_PP_INPUT_DIM;
    d->height      = (w->y2 - w->y1) / NIA416_PP_INPUT_DIM;
    d->conf        = w->score;
    d->class_index = (int32_t)w->cls;

    for (int j = i + 1; j < ncand; j++) {
      if (s_suppressed[j]) continue;
      if (s_cand[j].cls != w->cls) continue;          /* class-aware */
      if (nia416_iou(w, &s_cand[j]) > iou_thr) s_suppressed[j] = 1u;
    }
  }

  out->nb_detect = kept;
  return 0;
}
