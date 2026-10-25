/**
 ******************************************************************************
 * @file    NeuroN6_IPostProcessor.h
 * @brief   Post-processor interface: a model's post-processing as the
 *          application calls it. The camera examples bind one in
 *          vision_begin() and call it from vision_run(); a custom model fills
 *          in the same struct.
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

#ifndef NEURON6_IPOSTPROCESSOR_H
#define NEURON6_IPOSTPROCESSOR_H

#include <stdint.h>

#include "ll_aton_runtime.h"   /* NN_Instance_TypeDef */

/* Overlay draw policy: clear the layer first, commit to the panel after. */
typedef enum {
  OVL_RELOAD_CLEAR,         /* Clear buffer before drawing, commit after  */
  OVL_RELOAD,               /* Do not clear, commit after drawing         */
  OVL_CLEAR,                /* Clear buffer before drawing, no commit     */
  OVL_RELOAD_CLEAR_NONE     /* No clear, no commit                        */
} OVL_Mode_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Call order:
 *   1. init(ctx, model)      once, at start-up
 *   2. run(ctx, model)       per frame, after NPU inference
 *   3. draw_lcd(ctx, mode)   per frame, after run(), with a panel (optional)
 *   4. emit_metadata(ctx)    per frame, after draw_lcd() (optional) */

typedef struct IPostProcessor {
  /* Initialise per-instance state. Return 0 on success, negative on error. */
  int32_t (*init)(void *ctx, NN_Instance_TypeDef *model);

  /* Decode the latest NPU output into `ctx`, once per frame after
   * NPU_Run_Inference. Return 0 on success. */
  int32_t (*run)(void *ctx, NN_Instance_TypeDef *model);

  /* Draw on the overlay layer (the bundled ones call the application's
   * LCD_Overlay_* function). NULL for no LCD output. */
  void    (*draw_lcd)(void *ctx, OVL_Mode_t mode);

  /* Build USB TLV metadata with the vnd_meta_builder_* helpers. NULL for
   * none. */
  void    (*emit_metadata)(void *ctx);

  /* Opaque per-instance state, passed to every call. */
  void   *ctx;
} IPostProcessor;

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_IPOSTPROCESSOR_H */
