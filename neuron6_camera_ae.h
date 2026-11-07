/**
  * @file    neuron6_camera_ae.h
  * @brief   Auto-exposure for raw sensors with no loop of their own, metered from the
  *          camera pipe's statistics (BSP_CAMERA_GetIspStatistics).
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_CAMERA_AE_H
#define NEURON6_CAMERA_AE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exposure time first, then gain; one step per settled frame. Pure arithmetic: the caller
 * meters, and writes the result to its sensor. */

/** @brief The sensor's limits and the loop's target. */
typedef struct
{
  uint32_t Target;        /*!< Wanted average, 1 to 254, of a linear (pre-gamma) meter. */
  uint32_t ExposureMinUs; /*!< Shortest exposure, in microseconds.                      */
  uint32_t ExposureMaxUs; /*!< Longest: the frame period, less the sensor's margin.     */
  uint32_t GainMaxMdb;    /*!< Highest gain used, in thousandths of a dB.               */
  uint32_t GainStepMdb;   /*!< The sensor's gain step; 0 for continuous.                */
  uint32_t FlickerHz;     /*!< Mains frequency, 50 or 60, or 0: exposures longer than
                            *   half its period are whole half-periods.                 */
  uint32_t SettleFrames;  /*!< Frames between a change and the first frame it shows in. */
} BSP_CAMERA_AE_Config_t;

/** @brief The loop's state: what the sensor holds, and the frames still to wait. */
typedef struct
{
  uint32_t ExposureUs;    /*!< In force, in microseconds.                               */
  uint32_t GainMdb;       /*!< In force, in thousandths of a dB.                        */
  uint32_t Wait;          /*!< Frames still to skip before the next step.               */
} BSP_CAMERA_AE_State_t;

/**
  * @brief  One frame's step.
  * @param  pConfig The sensor's limits and the target.
  * @param  pState  Updated in place.
  * @param  Average The frame's meter reading, 0 to 255.
  * @retval 1 if ExposureUs or GainMdb changed (write them to the sensor), 0 if not.
  */
uint32_t BSP_CAMERA_AE_Step(const BSP_CAMERA_AE_Config_t *pConfig, BSP_CAMERA_AE_State_t *pState,
                            uint32_t Average);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_CAMERA_AE_H */
