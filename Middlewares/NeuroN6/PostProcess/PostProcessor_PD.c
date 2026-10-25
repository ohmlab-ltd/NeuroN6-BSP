/**
 ******************************************************************************
 * @file    PostProcessor_PD.c
 * @brief   IPostProcessor singleton factory for the MediaPipe palm detector.
 *          The thresholds are kept in ctx; init() passes the latest on.
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

#include "PostProcessor_PD.h"

#include "app_postprocess_pd_ui.h"
#include "LCD_Overlay_PD.h"
#include "vnd_meta_pd.h"

typedef struct {
  float           conf;
  float           iou;
  vnd_meta_slot_t slot;
} pd_state_t;

static pd_state_t g_pd_state = {
  .conf = AI_PD_PP_CONF_THRESHOLD,
  .iou  = AI_PD_PP_IOU_THRESHOLD,
  .slot = VND_META_SLOT_PRIMARY
};

static int32_t pd_pp_init_cb(void *ctx, NN_Instance_TypeDef *model)
{
  (void)model;   /* wrapper operates on file-scope globals */
  pd_state_t *s = (pd_state_t *)ctx;
  return app_postprocess_pd_ui_init(s->conf, s->iou);
}

static int32_t pd_pp_run_cb(void *ctx, NN_Instance_TypeDef *model)
{
  (void)ctx;
  return app_postprocess_pd_ui_run(model);
}

static void pd_pp_draw_lcd_cb(void *ctx, OVL_Mode_t mode)
{
  (void)ctx;
  LCD_Overlay_PD(mode);
}

static void pd_pp_emit_metadata_cb(void *ctx)
{
  pd_state_t *s = (pd_state_t *)ctx;
  Display_NS_PalmDetector_OnSlot(s->slot);
}

static IPostProcessor g_pd_pp = {
  .init          = pd_pp_init_cb,
  .run           = pd_pp_run_cb,
  .draw_lcd      = pd_pp_draw_lcd_cb,
  .emit_metadata = pd_pp_emit_metadata_cb,
  .ctx           = &g_pd_state,
};

IPostProcessor *PostProcess_PalmDetector(float conf, float iou)
{
  return PostProcess_PalmDetector_OnSlot(conf, iou, VND_META_SLOT_PRIMARY);
}

IPostProcessor *PostProcess_PalmDetector_OnSlot(float conf, float iou,
                                                vnd_meta_slot_t slot)
{
  g_pd_state.conf = conf;
  g_pd_state.iou  = iou;
  g_pd_state.slot = slot;
  return &g_pd_pp;
}
