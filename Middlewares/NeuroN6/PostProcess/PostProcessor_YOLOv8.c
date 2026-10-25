/**
 ******************************************************************************
 * @file    PostProcessor_YOLOv8.c
 * @brief   IPostProcessor singleton factories for the bundled YOLOv8 models.
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

#include "PostProcessor_YOLOv8.h"

#include "app_postprocess_iseg_yolo_v8_ui.h"
#include "app_postprocess_mpe_yolo_v8_ui.h"
#include "LCD_Overlay_Iseg.h"
#include "LCD_Overlay_MPE.h"
#include "vnd_meta_yolov8_Iseg.h"
#include "vnd_meta_yolov8_MPE.h"

/* YOLOv8 instance segmentation */

typedef struct {
  float           conf;
  float           iou;
  vnd_meta_slot_t slot;   /* where emit_metadata writes */
} yolov8_iseg_state_t;

static yolov8_iseg_state_t g_iseg_state = {
  .conf = 0.4f, .iou = 0.5f, .slot = VND_META_SLOT_PRIMARY
};

static int32_t iseg_pp_init(void *ctx, NN_Instance_TypeDef *model)
{
  (void)model;  /* bundled init operates on file-scope globals */
  yolov8_iseg_state_t *s = (yolov8_iseg_state_t *)ctx;
  return app_postprocess_iseg_yolo_v8_ui_init(s->conf, s->iou);
}

static int32_t iseg_pp_run(void *ctx, NN_Instance_TypeDef *model)
{
  (void)ctx;
  return app_postprocess_iseg_yolo_v8_ui_run(model);
}

static void iseg_pp_draw_lcd(void *ctx, OVL_Mode_t mode)
{
  (void)ctx;
  LCD_Overlay_Iseg(mode);
}

static void iseg_pp_emit_metadata(void *ctx)
{
  yolov8_iseg_state_t *s = (yolov8_iseg_state_t *)ctx;
  Display_NS_Yolo_V8_Iseg_OnSlot(s->slot);
}

static IPostProcessor g_iseg_pp = {
  .init           = iseg_pp_init,
  .run            = iseg_pp_run,
  .draw_lcd       = iseg_pp_draw_lcd,
  .emit_metadata  = iseg_pp_emit_metadata,
  .ctx            = &g_iseg_state,
};

IPostProcessor *PostProcess_YOLOv8_ISEG(float conf, float iou)
{
  return PostProcess_YOLOv8_ISEG_OnSlot(conf, iou, VND_META_SLOT_PRIMARY);
}

IPostProcessor *PostProcess_YOLOv8_ISEG_OnSlot(float conf, float iou,
                                               vnd_meta_slot_t slot)
{
  g_iseg_state.conf = conf;
  g_iseg_state.iou  = iou;
  g_iseg_state.slot = slot;
  return &g_iseg_pp;
}

/* YOLOv8 MPE (multi-person pose estimation) */

typedef struct {
  float           conf;
  float           iou;
  vnd_meta_slot_t slot;
} yolov8_mpe_state_t;

static yolov8_mpe_state_t g_mpe_state = {
  .conf = 0.4f, .iou = 0.5f, .slot = VND_META_SLOT_PRIMARY
};

static int32_t mpe_pp_init(void *ctx, NN_Instance_TypeDef *model)
{
  (void)model;
  yolov8_mpe_state_t *s = (yolov8_mpe_state_t *)ctx;
  return app_postprocess_mpe_yolo_v8_ui_init(s->conf, s->iou);
}

static int32_t mpe_pp_run(void *ctx, NN_Instance_TypeDef *model)
{
  (void)ctx;
  return app_postprocess_mpe_yolo_v8_ui_run(model);
}

static void mpe_pp_draw_lcd(void *ctx, OVL_Mode_t mode)
{
  (void)ctx;
  LCD_Overlay_MPE(mode);
}

static void mpe_pp_emit_metadata(void *ctx)
{
  yolov8_mpe_state_t *s = (yolov8_mpe_state_t *)ctx;
  Display_NS_Yolo_V8_MPE_OnSlot(s->slot);
}

static IPostProcessor g_mpe_pp = {
  .init           = mpe_pp_init,
  .run            = mpe_pp_run,
  .draw_lcd       = mpe_pp_draw_lcd,
  .emit_metadata  = mpe_pp_emit_metadata,
  .ctx            = &g_mpe_state,
};

IPostProcessor *PostProcess_YOLOv8_MPE(float conf, float iou)
{
  return PostProcess_YOLOv8_MPE_OnSlot(conf, iou, VND_META_SLOT_PRIMARY);
}

IPostProcessor *PostProcess_YOLOv8_MPE_OnSlot(float conf, float iou,
                                              vnd_meta_slot_t slot)
{
  g_mpe_state.conf = conf;
  g_mpe_state.iou  = iou;
  g_mpe_state.slot = slot;
  return &g_mpe_pp;
}
