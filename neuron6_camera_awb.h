/**
  * @file    neuron6_camera_awb.h
  * @brief   Auto white balance for raw sensors, metered from the camera pipe's red, green
  *          and blue statistics (BSP_CAMERA_GetIspStatisticsUnit).
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_CAMERA_AWB_H
#define NEURON6_CAMERA_AWB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* - Grey world, held to the line through measured lights (warm to cool): the scene's
 *   average is made grey only as far as a real light would explain it.
 * - Every gain carries a white gain, so a pixel clipped at the sensor reaches 255 on all
 *   three components (white, not tinted).
 * - Pure arithmetic: the caller meters after the gains and writes the result. */

/** @brief Lights a profile may hold. */
#define BSP_CAMERA_AWB_MAX_REFS   5U

/** @brief Positions along the line between two neighbouring lights. */
#define BSP_CAMERA_AWB_SEGMENT    1000U

/** @brief One measured light: the gains that make a grey card grey under it. */
typedef struct
{
  uint32_t Kelvin;   /*!< Its colour temperature, for reports.                     */
  uint32_t GainR;    /*!< Red gain, in thousandths (green is 1000).               */
  uint32_t GainB;    /*!< Blue gain, in thousandths.                              */
} BSP_CAMERA_AWB_Ref_t;

/** @brief The sensor's lights and the loop's settings. */
typedef struct
{
  const BSP_CAMERA_AWB_Ref_t *pRefs; /*!< Warm to cool.                                       */
  uint32_t RefCount;    /*!< 2 to BSP_CAMERA_AWB_MAX_REFS.                                    */
  uint32_t WhiteGain;   /*!< On every component, in thousandths: 255 over (255 - black level). */
  uint32_t StepPercent; /*!< Share of the way to the measured light moved per step, 1 to 100. */
  uint32_t MinGreen;    /*!< Green average below which the scene is too dark to judge.        */
} BSP_CAMERA_AWB_Config_t;

/** @brief The loop's state and the gains to write. */
typedef struct
{
  uint32_t Position;  /*!< Along the line: 0 at the first light, BSP_CAMERA_AWB_SEGMENT per
                        *   light after it.                                              */
  uint32_t Kelvin;    /*!< Colour temperature at Position.                                */
  uint32_t GainR;     /*!< Gains in force, in thousandths, white gain included.           */
  uint32_t GainG;
  uint32_t GainB;
  uint32_t Target;    /*!< The measured light, smoothed over frames.                      */
  uint32_t Moving;    /*!< 1 while moving towards Target.                                 */
} BSP_CAMERA_AWB_State_t;

/**
  * @brief  Start at a position (0 for the warmest light) and fill in its gains.
  */
void BSP_CAMERA_AWB_Init(const BSP_CAMERA_AWB_Config_t *pConfig, BSP_CAMERA_AWB_State_t *pState,
                         uint32_t Position);

/**
  * @brief  One step from a frame's averages, measured after the gains in force.
  * @param  AvgR, AvgG, AvgB The frame's component averages, 0 to 255.
  * @retval 1 if the gains changed (write them), 0 if not.
  */
uint32_t BSP_CAMERA_AWB_Step(const BSP_CAMERA_AWB_Config_t *pConfig, BSP_CAMERA_AWB_State_t *pState,
                             uint32_t AvgR, uint32_t AvgG, uint32_t AvgB);

/**
  * @brief  A gain in thousandths as the camera pipe's shift and multiplier
  *         (gain = Multiplier / 128 x 2^Shift), as DCMIPP_ExposureConfTypeDef holds them.
  */
void BSP_CAMERA_AWB_ToShiftMultiplier(uint32_t Gain, uint8_t *pShift, uint8_t *pMultiplier);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_CAMERA_AWB_H */
