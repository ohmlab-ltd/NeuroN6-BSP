/**
  * @file    neuron6_camera_vd66gy.h
  * @brief   VD66GY global-shutter image sensor as a Neuro N6 camera sensor.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_CAMERA_VD66GY_H
#define NEURON6_CAMERA_VD66GY_H

#include "neuron6_conf.h"
#include "neuron6_errno.h"
#include "neuron6_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Device address on the sensor bus, 8-bit form. */
#define BSP_CAMERA_VD66GY_I2C_ADDRESS   0x20U

/** @brief Expected model register value. */
#define BSP_CAMERA_VD66GY_ID            0x5603U

/** @brief Frequency of the oscillator the carrier feeds the sensor, in Hz. */
#ifndef BSP_CAMERA_VD66GY_EXT_CLOCK_HZ
#define BSP_CAMERA_VD66GY_EXT_CLOCK_HZ  12000000U
#endif

/**
  * @brief  BSP_CAMERA_Init modes for this sensor; the BSP_CAMERA_R* values do not apply.
  * @note   The 1120-pixel modes read a line in 9.5 us (not 7.7) so the encoder keeps up;
  *         it codes 1360 of the array's 1364 lines (BSP_VENC_GetBandGeometry).
  */
#define BSP_CAMERA_VD66GY_R320x240      0U
#define BSP_CAMERA_VD66GY_R640x480      1U
#define BSP_CAMERA_VD66GY_R480x640      2U
#define BSP_CAMERA_VD66GY_R1024x768     3U
#define BSP_CAMERA_VD66GY_R768x1024     4U
#define BSP_CAMERA_VD66GY_R720x1280     5U
#define BSP_CAMERA_VD66GY_R1024x1280    6U
#define BSP_CAMERA_VD66GY_R1120x720     7U /*!< Full width, cropped height. */
#define BSP_CAMERA_VD66GY_R1120x1364    8U /*!< The whole array.            */

/**
  * @brief  Register this sensor; call before BSP_CAMERA_Init and BSP_CAMERA_ConfigPipe.
  * @note   Set the lanes with BSP_CAMERA_SetCarrier before BSP_CAMERA_Init: STANDARD (the
  *         default) for the NeuroVision carrier, INVERTED for the thermal carrier.
  * @note   The driver starts the sensor only from power-up and no carrier has its reset
  *         line: if the board's reset does not reach it, Init fails until a power cycle.
  * @note   Build Components/vd6g/vd6g.c with NDEBUG: its reachable assertions hang.
  * @retval BSP_ERROR_BUSY if the camera is already up.
  */
int32_t BSP_CAMERA_VD66GY_Register(void);

/**
  * @brief  Frame rate for the next BSP_CAMERA_Init: 2 fps or more, 0 (default) for the
  *         mode's ceiling; above the ceiling runs at the ceiling (faster never streams).
  * @retval BSP_ERROR_BUSY if the sensor is already initialised.
  */
int32_t BSP_CAMERA_VD66GY_SetFrameRate(uint32_t Fps);

/**
  * @brief  The colour-filter order (BSP_CAMERA_BAYER_*) read by BSP_CAMERA_Init.
  * @note   A monochrome part reports RGGB, which passes its one channel through as grey.
  * @retval BSP_ERROR_NO_INIT before BSP_CAMERA_Init.
  */
int32_t BSP_CAMERA_VD66GY_GetBayerOrder(uint32_t *pBayerOrder);

/** @brief Auto-exposure target brightness, 0 to 100 (about 40 is mid-range); it steers
  *        the sensor's own exposure loop. */
int32_t BSP_CAMERA_VD66GY_SetBrightness(int32_t Level);

/** @brief The VD6G driver context, valid after BSP_CAMERA_VD66GY_Register, for the driver
  *        calls not wrapped here (manual exposure, gains, flicker, test pattern). */
void *BSP_CAMERA_VD66GY_GetContext(void);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_CAMERA_VD66GY_H */
