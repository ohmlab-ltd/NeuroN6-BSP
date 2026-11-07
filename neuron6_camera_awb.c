/**
  * @file    neuron6_camera_awb.c
  * @brief   Auto white balance on a line through measured lights.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6_camera_awb.h"

#include <stddef.h>

/* The averages are whole numbers: at mid-grey one count is 2%. The measured light is
 * smoothed, a settled loop restarts only past AWB_START, and a moving one stops inside
 * AWB_STOP (both in positions). */
#define AWB_SMOOTH_PERCENT 25
#define AWB_START          (BSP_CAMERA_AWB_SEGMENT / 20U)
#define AWB_STOP           (BSP_CAMERA_AWB_SEGMENT / 100U)

/* A component this high is mostly clipped: the frame cannot be judged. */
#define AWB_MAX_AVERAGE   250U

static uint32_t Awb_Last(const BSP_CAMERA_AWB_Config_t *pConfig)
{
  return (pConfig->RefCount - 1U) * BSP_CAMERA_AWB_SEGMENT;
}

static int32_t Awb_Lerp(int32_t A, int32_t B, uint32_t U)
{
  const int32_t d = (B - A) * (int32_t)U;

  return A + ((d >= 0) ? ((d + (int32_t)(BSP_CAMERA_AWB_SEGMENT / 2U)) / (int32_t)BSP_CAMERA_AWB_SEGMENT)
                       : ((d - (int32_t)(BSP_CAMERA_AWB_SEGMENT / 2U)) / (int32_t)BSP_CAMERA_AWB_SEGMENT));
}

/* Gains and temperature at the state's position. */
static void Awb_Apply(const BSP_CAMERA_AWB_Config_t *pConfig, BSP_CAMERA_AWB_State_t *pState)
{
  uint32_t i = pState->Position / BSP_CAMERA_AWB_SEGMENT;
  uint32_t u;
  const BSP_CAMERA_AWB_Ref_t *a;
  const BSP_CAMERA_AWB_Ref_t *b;

  if (i > (pConfig->RefCount - 2U))
  {
    i = pConfig->RefCount - 2U;
  }
  u = pState->Position - (i * BSP_CAMERA_AWB_SEGMENT);
  a = &pConfig->pRefs[i];
  b = &pConfig->pRefs[i + 1U];

  pState->Kelvin = (uint32_t)Awb_Lerp((int32_t)a->Kelvin, (int32_t)b->Kelvin, u);
  pState->GainG  = pConfig->WhiteGain;
  pState->GainR  = ((uint32_t)Awb_Lerp((int32_t)a->GainR, (int32_t)b->GainR, u) * pConfig->WhiteGain + 500U) / 1000U;
  pState->GainB  = ((uint32_t)Awb_Lerp((int32_t)a->GainB, (int32_t)b->GainB, u) * pConfig->WhiteGain + 500U) / 1000U;
}

void BSP_CAMERA_AWB_Init(const BSP_CAMERA_AWB_Config_t *pConfig, BSP_CAMERA_AWB_State_t *pState,
                         uint32_t Position)
{
  if ((pConfig == NULL) || (pState == NULL) || (pConfig->pRefs == NULL) || (pConfig->RefCount < 2U) ||
      (pConfig->RefCount > BSP_CAMERA_AWB_MAX_REFS))
  {
    return;
  }
  pState->Position = (Position > Awb_Last(pConfig)) ? Awb_Last(pConfig) : Position;
  pState->Target   = pState->Position;
  pState->Moving   = 0U;
  Awb_Apply(pConfig, pState);
}

uint32_t BSP_CAMERA_AWB_Step(const BSP_CAMERA_AWB_Config_t *pConfig, BSP_CAMERA_AWB_State_t *pState,
                             uint32_t AvgR, uint32_t AvgG, uint32_t AvgB)
{
  int64_t  target_r;
  int64_t  target_b;
  int64_t  best = -1;
  uint32_t target = 0U;
  int32_t  delta;
  int32_t  limit;
  int32_t  step;

  if ((pConfig == NULL) || (pState == NULL) || (pConfig->pRefs == NULL) || (pConfig->RefCount < 2U) ||
      (pConfig->RefCount > BSP_CAMERA_AWB_MAX_REFS) || (pConfig->WhiteGain == 0U))
  {
    return 0U;
  }
  if ((AvgG < pConfig->MinGreen) || (AvgR == 0U) || (AvgB == 0U) ||
      (AvgR >= AWB_MAX_AVERAGE) || (AvgG >= AWB_MAX_AVERAGE) || (AvgB >= AWB_MAX_AVERAGE))
  {
    return 0U;
  }

  /* The gains that would make this frame's average grey, white gain taken out. */
  target_r = ((int64_t)pState->GainR * 1000 / pConfig->WhiteGain) * AvgG / AvgR;
  target_b = ((int64_t)pState->GainB * 1000 / pConfig->WhiteGain) * AvgG / AvgB;

  /* The nearest point on the line through the lights. */
  for (uint32_t i = 0U; (i + 1U) < pConfig->RefCount; i++)
  {
    const int64_t ar = pConfig->pRefs[i].GainR;
    const int64_t ab = pConfig->pRefs[i].GainB;
    const int64_t dr = (int64_t)pConfig->pRefs[i + 1U].GainR - ar;
    const int64_t db = (int64_t)pConfig->pRefs[i + 1U].GainB - ab;
    const int64_t len2 = (dr * dr) + (db * db);
    int64_t u = (len2 > 0) ? ((((target_r - ar) * dr) + ((target_b - ab) * db)) * (int64_t)BSP_CAMERA_AWB_SEGMENT / len2) : 0;
    int64_t er;
    int64_t eb;
    int64_t dist;

    if (u < 0) { u = 0; }
    if (u > (int64_t)BSP_CAMERA_AWB_SEGMENT) { u = (int64_t)BSP_CAMERA_AWB_SEGMENT; }
    er = (ar + (dr * u / (int64_t)BSP_CAMERA_AWB_SEGMENT)) - target_r;
    eb = (ab + (db * u / (int64_t)BSP_CAMERA_AWB_SEGMENT)) - target_b;
    dist = (er * er) + (eb * eb);
    if ((best < 0) || (dist < best))
    {
      best = dist;
      target = (i * BSP_CAMERA_AWB_SEGMENT) + (uint32_t)u;
    }
  }

  /* Smooth the measured light, then move towards it outside the hold band. */
  delta = (int32_t)target - (int32_t)pState->Target;
  pState->Target = (uint32_t)((int32_t)pState->Target + ((delta * AWB_SMOOTH_PERCENT) / 100));

  delta = (int32_t)pState->Target - (int32_t)pState->Position;
  limit = (int32_t)((pState->Moving != 0U) ? AWB_STOP : AWB_START);
  if ((delta < limit) && (delta > -limit))
  {
    pState->Moving = 0U;
    return 0U;
  }
  pState->Moving = 1U;
  step = (delta * (int32_t)pConfig->StepPercent) / 100;
  if (step == 0)
  {
    step = (delta > 0) ? 1 : -1;
  }
  pState->Position = (uint32_t)((int32_t)pState->Position + step);
  Awb_Apply(pConfig, pState);
  return 1U;
}

void BSP_CAMERA_AWB_ToShiftMultiplier(uint32_t Gain, uint8_t *pShift, uint8_t *pMultiplier)
{
  uint32_t shift = 0U;
  uint32_t mult;

  if ((pShift == NULL) || (pMultiplier == NULL))
  {
    return;
  }
  /* The multiplier holds 255/128 at most: double through the shift beyond that. */
  while (((Gain * 128U) > (255U * 1000U * (1U << shift))) && (shift < 7U))
  {
    shift++;
  }
  mult = ((Gain * 128U) + (500U * (1U << shift))) / (1000U * (1U << shift));
  *pShift      = (uint8_t)shift;
  *pMultiplier = (uint8_t)((mult > 255U) ? 255U : mult);
}
