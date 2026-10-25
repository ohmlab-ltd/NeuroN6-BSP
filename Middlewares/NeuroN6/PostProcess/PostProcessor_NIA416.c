/**
 ******************************************************************************
 * @file    PostProcessor_NIA416.c
 * @brief   IPostProcessor singleton factory for the NIA Triad-416 detector.
 *          Decodes with nia416_pp; the NIA metadata builder and overlay take
 *          the same nia_pp_out_t.
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

#include "PostProcessor_NIA416.h"

#include "app_postprocess_nia416_ui.h"
#include "LCD_Overlay_Nia.h"
#include "vnd_meta_nia.h"

/* Inference timestamps set by the application (ts[0] start, ts[1] end). */
extern uint32_t ts[];

typedef struct {
  float           conf;
  float           iou;
  vnd_meta_slot_t slot;
} nia416_state_t;

static nia416_state_t g_nia416_state = {
  .conf = NIA416_PP_CONF_THRESHOLD,
  .iou  = NIA416_PP_IOU_THRESHOLD,
  .slot = VND_META_SLOT_PRIMARY
};

static int32_t nia416_pp_init_cb(void *ctx, NN_Instance_TypeDef *model)
{
  (void)model;   /* wrapper operates on file-scope globals */
  nia416_state_t *s = (nia416_state_t *)ctx;
  return app_postprocess_nia416_ui_init(s->conf, s->iou);
}

static int32_t nia416_pp_run_cb(void *ctx, NN_Instance_TypeDef *model)
{
  (void)ctx;
  return app_postprocess_nia416_ui_run(model);
}

static void nia416_pp_draw_lcd_cb(void *ctx, OVL_Mode_t mode)
{
  (void)ctx;
  LCD_Overlay_Nia_From(&nia416_pp_output, mode);
}

static void nia416_pp_emit_metadata_cb(void *ctx)
{
  nia416_state_t *s = (nia416_state_t *)ctx;
  vnd_meta_build_nia_on_slot(&nia416_pp_output, ts[1] - ts[0],
                             &nia416_pp_params, s->slot);
}

static IPostProcessor g_nia416_pp = {
  .init          = nia416_pp_init_cb,
  .run           = nia416_pp_run_cb,
  .draw_lcd      = nia416_pp_draw_lcd_cb,
  .emit_metadata = nia416_pp_emit_metadata_cb,
  .ctx           = &g_nia416_state,
};

IPostProcessor *PostProcess_NIA416(float conf, float iou)
{
  return PostProcess_NIA416_OnSlot(conf, iou, VND_META_SLOT_PRIMARY);
}

IPostProcessor *PostProcess_NIA416_OnSlot(float conf, float iou,
                                          vnd_meta_slot_t slot)
{
  g_nia416_state.conf = conf;
  g_nia416_state.iou  = iou;
  g_nia416_state.slot = slot;
  return &g_nia416_pp;
}
