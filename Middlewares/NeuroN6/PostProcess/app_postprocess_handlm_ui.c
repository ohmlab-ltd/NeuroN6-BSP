/**
 * @file    app_postprocess_handlm_ui.c
 * @brief   MediaPipe hand-landmark post-processing (see the header).
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

#include "app_postprocess_handlm_ui.h"
#include <stddef.h>
#include <string.h>
#include <math.h>

handlm_result_t handlm_output;
float32_t       handlm_min_score = AI_HANDLM_MIN_SCORE;

int32_t app_postprocess_handlm_ui_init(float min_score)
{
  handlm_min_score = min_score;
  memset(&handlm_output, 0, sizeof(handlm_output));
  return 0;
}

int32_t app_postprocess_handlm_ui_run(NN_Instance_TypeDef *instance)
{
  const LL_Buffer_InfoTypeDef *out = LL_ATON_Output_Buffers_Info(instance);

  handlm_output.valid = false;
  if (out == NULL) return -1;

  /* The two [1,63] outputs (screen landmarks in input pixels, up to ~224;
   * world landmarks in wrist-relative metres, |v| < 1) are told apart by
   * magnitude; a lone one is taken as screen. The two [1,1] outputs are both
   * 0..1, so order decides: the .tflite lists handedness first, the presence
   * score second; swapped, one hand is never reported. The generated network
   * renames the buffers. Flip this if a re-export reorders the outputs. */
#ifndef AI_HANDLM_SCORE_IS_SECOND
#define AI_HANDLM_SCORE_IS_SECOND  (1)
#endif
  const float *lm63[2] = { NULL, NULL };
  int          n63     = 0;
  const float *one[2]  = { NULL, NULL };
  int          n1      = 0;

  for (int i = 0; out[i].name != NULL; i++) {
    const LL_Buffer_InfoTypeDef *b = &out[i];
    const uint8_t *s = LL_Buffer_addr_start(b);
    const uint8_t *e = LL_Buffer_addr_end(b);
    if (s == NULL || e == NULL) continue;

    const uint32_t nfloats = (uint32_t)(e - s) / sizeof(float);
    if (nfloats >= (uint32_t)(AI_HANDLM_NB_LANDMARKS * 3) && n63 < 2) {
      lm63[n63++] = (const float *)s;
    } else if (nfloats >= 1u && nfloats < 4u && n1 < 2) {
      one[n1++] = (const float *)s;
    }
  }

  if (n63 == 0) return -1;

  /* The .tflite lists world before screen; the larger magnitude is screen. */
  const float *screen = (n63 == 2) ? lm63[1] : lm63[0];   /* order default */
  if (n63 == 2) {
    float mag0 = 0.0f, mag1 = 0.0f;
    for (int i = 0; i < AI_HANDLM_NB_LANDMARKS * 3; i++) {
      const float a = fabsf(lm63[0][i]); if (a > mag0) mag0 = a;
      const float b = fabsf(lm63[1][i]); if (b > mag1) mag1 = b;
    }
    screen = (mag0 >= mag1) ? lm63[0] : lm63[1];
  }

  if (n1 >= 2 && one[0] && one[1]) {
#if AI_HANDLM_SCORE_IS_SECOND
    handlm_output.score      = one[1][0];
    handlm_output.handedness = one[0][0];
#else
    handlm_output.score      = one[0][0];
    handlm_output.handedness = one[1][0];
#endif
  } else if (n1 >= 1 && one[0]) {
    handlm_output.score      = one[0][0];   /* single scalar output: trust it */
    handlm_output.handedness = 0.0f;
  } else {
    handlm_output.score      = 1.0f;        /* no scalar output; don't gate */
    handlm_output.handedness = 0.0f;
  }

  /* Some exports give the score as a logit: squash it into [0,1]. */
  if (handlm_output.score < 0.0f || handlm_output.score > 1.0f) {
    handlm_output.score = 1.0f / (1.0f + expf(-handlm_output.score));
  }

  const float inv = 1.0f / AI_HANDLM_INPUT_SIZE;
  for (int k = 0; k < AI_HANDLM_NB_LANDMARKS; k++) {
    handlm_output.kps[k].x = screen[k * 3 + 0] * inv;   /* px -> [0..1] */
    handlm_output.kps[k].y = screen[k * 3 + 1] * inv;
    handlm_output.kps[k].z = screen[k * 3 + 2] * inv;
  }

  handlm_output.valid = (handlm_output.score >= handlm_min_score);
  return 0;
}
