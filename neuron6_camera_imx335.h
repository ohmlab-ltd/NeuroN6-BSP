/**
  * @file    neuron6_camera_imx335.h
  * @brief   Sony IMX335 (ST's B-CAMS-IMX module) as a Neuro N6 camera sensor.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_CAMERA_IMX335_H
#define NEURON6_CAMERA_IMX335_H

#include "neuron6_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

/* - One mode: 2592 x 1944 RAW10, RGGB, two lanes at 1600 Mbps. Smaller pictures come
 *   from MAIN's crop and downsize.
 * - No exposure loop of its own: drive BSP_CAMERA_IMX335_AutoExposure from MAIN's
 *   statistics (BSP_CAMERA_SetIspStatistics), or set exposure and gain directly.
 * - Power: EN_MODULE switches the module's 1.2 V and its 24 MHz oscillator; XCLR is the
 *   sensor's reset. Define the pins the carrier routes in neuron6_conf.h:
 *     BSP_CAMERA_IMX335_EN_PORT, _EN_PIN, _EN_CLK_ENABLE()
 *     BSP_CAMERA_IMX335_XCLR_PORT, _XCLR_PIN, _XCLR_CLK_ENABLE()
 *   and BSP_CAMERA_IMX335_IO_ENABLE() if they sit on a VDDIO2..5 bank. A line left
 *   undefined is taken as tied on by the carrier. */

/** @brief Bus address, 8-bit form. */
#define BSP_CAMERA_IMX335_I2C_ADDRESS   0x34U

/** @brief What register 0x3912 reads from reset (0x33 once configured). A weak check, so
  *        ReadID also checks the line length and tests a register write. */
#define BSP_CAMERA_IMX335_ID            0x00U

/** @brief The one mode, for BSP_CAMERA_Init. */
#define BSP_CAMERA_IMX335_R2592x1944    0x3350U

/** @brief The mode's size in pixels. */
#define BSP_CAMERA_IMX335_WIDTH         2592U
#define BSP_CAMERA_IMX335_HEIGHT        1944U

/** @brief The sensor's black level in 8-bit units (50 of 1023 at 10 bits). */
#define BSP_CAMERA_IMX335_BLACK_LEVEL   12U

/** @brief Frames between an exposure or gain write and the first frame it shows in. */
#define BSP_CAMERA_IMX335_SETTLE_FRAMES 3U

/** @brief Gain range and step, in thousandths of a dB. */
#define BSP_CAMERA_IMX335_GAIN_MAX_MDB  72000U   /*!< 30 dB analogue, the rest digital. */
#define BSP_CAMERA_IMX335_AGAIN_MAX_MDB 30000U   /*!< Analogue part of the range.       */
#define BSP_CAMERA_IMX335_GAIN_STEP_MDB 300U

/**
  * @brief  Power the IMX335 up through EN and XCLR (where defined) and register it; call
  *         before BSP_CAMERA_Init.
  * @retval BSP_ERROR_BUSY if the sensor is already initialised.
  */
int32_t BSP_CAMERA_IMX335_Register(void);

/**
  * @brief  Frames a second (default 30) and line length in percent of ST's 30 fps line
  *         (100 to 400, default 100), before BSP_CAMERA_Init. Longer lines let the
  *         encoder keep up with a wide picture; Fps x LinePercent may not exceed 3000.
  * @retval BSP_ERROR_BUSY once the sensor is up.
  */
int32_t BSP_CAMERA_IMX335_SetTiming(uint32_t Fps, uint32_t LinePercent);

/** @brief Exposure in microseconds, clamped to the frame. */
int32_t BSP_CAMERA_IMX335_SetExposure(uint32_t Us);

/** @brief Gain in thousandths of a dB, 0 to BSP_CAMERA_IMX335_GAIN_MAX_MDB, rounded down to
  *        BSP_CAMERA_IMX335_GAIN_STEP_MDB. */
int32_t BSP_CAMERA_IMX335_SetGain(uint32_t MilliDb);

/** @brief Exposure in microseconds and gain in thousandths of a dB, as programmed. */
int32_t BSP_CAMERA_IMX335_GetExposure(uint32_t *pUs, uint32_t *pMilliDb);

/** @brief Longest exposure the frame allows, in microseconds. */
int32_t BSP_CAMERA_IMX335_GetExposureMax(uint32_t *pUs);

/** @brief -1 off, or ST's pattern 0 to 11 (10 and 11 are colour bars). */
int32_t BSP_CAMERA_IMX335_SetTestPattern(int32_t Mode);

/**
  * @brief  One auto-exposure step, once a frame, outside interrupt context (it writes the
  *         sensor over I2C).
  * @param  Average MAIN's luminance average for the frame (BSP_CAMERA_GetIspStatistics).
  * @param  Target  The average wanted, 1 to 254, before gamma.
  * @retval BSP_ERROR_NO_INIT before BSP_CAMERA_Init.
  */
int32_t BSP_CAMERA_IMX335_AutoExposure(uint32_t Average, uint32_t Target);

/** @brief Line period in nanoseconds, and frame length in lines, as programmed. */
int32_t BSP_CAMERA_IMX335_GetLineTiming(uint32_t *pLineNs, uint32_t *pFrameLines);

/** @brief The driver object, for calls not wrapped here. */
void *BSP_CAMERA_IMX335_GetContext(void);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_CAMERA_IMX335_H */
