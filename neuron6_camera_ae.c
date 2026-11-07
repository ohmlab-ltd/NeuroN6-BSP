/**
  * @file    neuron6_camera_ae.c
  * @brief   Auto-exposure for raw sensors with no loop of their own.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6_camera_ae.h"

#include <math.h>
#include <stddef.h>

/* A reading this close to full scale says only "too bright": step as if it read twice
 * the target. */
#define AE_SATURATED            250U

/* No change within a sixteenth of the target, so the picture does not hunt. */
#define AE_DEADBAND_SHIFT       4U

/* Share of the correction applied per step: the meter is linear, so most of it. */
#define AE_DAMPING              0.8f

/* Largest correction in one step, either way. */
#define AE_RATIO_MIN            0.25f
#define AE_RATIO_MAX            4.0f

uint32_t BSP_CAMERA_AE_Step(const BSP_CAMERA_AE_Config_t *pConfig, BSP_CAMERA_AE_State_t *pState,
                            uint32_t Average)
{
  float    ratio;
  float    total;
  float    gain;
  uint32_t exposure;
  uint32_t gain_mdb;
  uint32_t error;

  if ((pConfig == NULL) || (pState == NULL) || (pConfig->Target == 0U) ||
      (pConfig->ExposureMinUs == 0U) || (pConfig->ExposureMaxUs < pConfig->ExposureMinUs))
  {
    return 0U;
  }

  if (pState->Wait != 0U)
  {
    pState->Wait--;
    return 0U;
  }

  error = (Average > pConfig->Target) ? (Average - pConfig->Target) : (pConfig->Target - Average);
  if ((error << AE_DEADBAND_SHIFT) <= pConfig->Target)
  {
    return 0U;
  }

  if (Average >= AE_SATURATED)
  {
    ratio = 0.5f;
  }
  else
  {
    ratio = (float)pConfig->Target / (float)((Average != 0U) ? Average : 1U);
  }
  ratio = 1.0f + (AE_DAMPING * (ratio - 1.0f));
  if (ratio < AE_RATIO_MIN)
  {
    ratio = AE_RATIO_MIN;
  }
  if (ratio > AE_RATIO_MAX)
  {
    ratio = AE_RATIO_MAX;
  }

  /* The picture follows exposure x linear gain. */
  total = (float)pState->ExposureUs * powf(10.0f, (float)pState->GainMdb / 20000.0f) * ratio;

  /* Time first, up to the frame; whole flicker half-periods once it is that long. */
  exposure = (total > (float)pConfig->ExposureMaxUs) ? pConfig->ExposureMaxUs : (uint32_t)total;
  if (pConfig->FlickerHz != 0U)
  {
    const uint32_t half_period = 500000U / pConfig->FlickerHz;

    if (exposure >= half_period)
    {
      exposure = (exposure / half_period) * half_period;
    }
  }
  if (exposure < pConfig->ExposureMinUs)
  {
    exposure = pConfig->ExposureMinUs;
  }

  /* Then gain for the rest. */
  gain = total / (float)exposure;
  gain_mdb = (gain > 1.0f) ? (uint32_t)(20000.0f * log10f(gain)) : 0U;
  if (pConfig->GainStepMdb != 0U)
  {
    gain_mdb = ((gain_mdb + (pConfig->GainStepMdb / 2U)) / pConfig->GainStepMdb) * pConfig->GainStepMdb;
  }
  if (gain_mdb > pConfig->GainMaxMdb)
  {
    gain_mdb = pConfig->GainMaxMdb;
  }

  if ((exposure == pState->ExposureUs) && (gain_mdb == pState->GainMdb))
  {
    return 0U;
  }

  pState->ExposureUs = exposure;
  pState->GainMdb    = gain_mdb;
  pState->Wait       = pConfig->SettleFrames;

  return 1U;
}
