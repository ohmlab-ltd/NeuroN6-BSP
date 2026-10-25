/**
 ******************************************************************************
 * @file    PostProcessor_HandLM.c
 * @brief   IPostProcessor factory for the hand-landmark model.
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

#include "PostProcessor_HandLM.h"

#include "app_postprocess_handlm_ui.h"
#include "vnd_meta_handlm.h"

typedef struct {
  float           min_score;
  vnd_meta_slot_t slot;
} handlm_state_t;

static handlm_state_t g_handlm_state = {
  .min_score = AI_HANDLM_MIN_SCORE,
  .slot      = VND_META_SLOT_PRIMARY
};

static int32_t handlm_init_cb(void *ctx, NN_Instance_TypeDef *model)
{
  (void)model;
  handlm_state_t *s = (handlm_state_t *)ctx;
  return app_postprocess_handlm_ui_init(s->min_score);
}

static int32_t handlm_run_cb(void *ctx, NN_Instance_TypeDef *model)
{
  (void)ctx;
  return app_postprocess_handlm_ui_run(model);
}

static void handlm_emit_metadata_cb(void *ctx)
{
  handlm_state_t *s = (handlm_state_t *)ctx;
  Display_NS_HandLandmarks_OnSlot(s->slot);
}

static IPostProcessor g_handlm_pp = {
  .init          = handlm_init_cb,
  .run           = handlm_run_cb,
  .draw_lcd      = NULL,      /* no panel overlay; landmarks go out as metadata */
  .emit_metadata = handlm_emit_metadata_cb,
  .ctx           = &g_handlm_state,
};

IPostProcessor *PostProcess_HandLandmarks(float min_score)
{
  return PostProcess_HandLandmarks_OnSlot(min_score, VND_META_SLOT_PRIMARY);
}

IPostProcessor *PostProcess_HandLandmarks_OnSlot(float min_score,
                                                 vnd_meta_slot_t slot)
{
  g_handlm_state.min_score = min_score;
  g_handlm_state.slot      = slot;
  return &g_handlm_pp;
}
