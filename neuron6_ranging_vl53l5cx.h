/**
  * @file    neuron6_ranging_vl53l5cx.h
  * @brief   VL53L5CX multizone time-of-flight ranging (ST's B-CAMS-IMX module).
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_RANGING_VL53L5CX_H
#define NEURON6_RANGING_VL53L5CX_H

#include "neuron6_conf.h"
#include "neuron6_errno.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The VL53L5CX on ST's B-CAMS-IMX module, on the sensor bus (I2C1) through Ohm Lab's
 * B-CAMS-IMX adapter: 4 x 4 or 8 x 8 zones, up to 4 m. Built on ST's ultra lite
 * driver (Components/vl53l5cx), which loads the part's 84 KB firmware at each Init.
 * Optional lines, in neuron6_conf.h:
 *   BSP_RANGING_VL53L5CX_LPN_PORT, _LPN_PIN, _LPN_CLK_ENABLE(): LPn, driven high by Init.
 *   BSP_RANGING_VL53L5CX_INT_PORT, _INT_PIN, _INT_CLK_ENABLE(): INT, which pulses low
 *   about 100 us per result. Its EXTI line's pending bit holds the pulse (the line's
 *   NVIC interrupt stays off), so IsReady and GetResult skip the bus until a result
 *   lands. No header pin on the same EXTI line may then use interrupts. */

/** @brief 8-bit I2C address; the OV5640 carriers' VL53L1CB answers there too. */
#define BSP_RANGING_VL53L5CX_I2C_ADDRESS   0x52U

/** @brief BSP_RANGING_VL53L5CX_ReadID on a working part: device 0xF0, revision 0x02. */
#define BSP_RANGING_VL53L5CX_ID            0xF002U

/** @brief Zones: 4 x 4 at up to 60 results a second, 8 x 8 at up to 15. */
#define BSP_RANGING_VL53L5CX_ZONES_4X4     16U
#define BSP_RANGING_VL53L5CX_ZONES_8X8     64U

/** @brief Target status of a zone with no target. 5 and 9 are valid ranges. */
#define BSP_RANGING_VL53L5CX_NO_TARGET     255U

/** @brief One result. Zone z is row z / side and column z % side, side 4 or 8, in the
  *        order ST's driver gives them. */
typedef struct
{
  uint32_t Zones;                                     /*!< 16 or 64.                      */
  int32_t  TemperatureC;                              /*!< The part's die temperature.    */
  uint16_t Distance[BSP_RANGING_VL53L5CX_ZONES_8X8];  /*!< mm; 0 = no valid range.        */
  uint8_t  Status[BSP_RANGING_VL53L5CX_ZONES_8X8];    /*!< ST's target status (UM2884).   */
} BSP_RANGING_VL53L5CX_Result_t;

/**
  * @brief  Bring up the bus and LPn, check the part, load its firmware and default
  *         settings: about 3 s on the 295 kHz bus, calling BSP_RANGING_VL53L5CX_WaitHook
  *         throughout. Called again, it reloads the part.
  * @retval BSP_ERROR_UNKNOWN_COMPONENT if no VL53L5CX answers,
  *         BSP_ERROR_COMPONENT_FAILURE if its firmware did not start.
  */
int32_t BSP_RANGING_VL53L5CX_Init(void);

/** @brief Stop ranging and release the bus and the lines. */
int32_t BSP_RANGING_VL53L5CX_DeInit(void);

/** @brief Read the identity: BSP_RANGING_VL53L5CX_ID on a working part. */
int32_t BSP_RANGING_VL53L5CX_ReadID(uint32_t *pId);

/**
  * @brief  Range continuously.
  * @param  Zones   BSP_RANGING_VL53L5CX_ZONES_4X4 or _8X8.
  * @param  RateHz  Results a second: 1 to 60 at 16 zones, 1 to 15 at 64.
  * @retval BSP_ERROR_BUSY if already ranging.
  */
int32_t BSP_RANGING_VL53L5CX_Start(uint32_t Zones, uint32_t RateHz);

/** @brief Stop ranging; BSP_ERROR_NO_INIT if it was not running. */
int32_t BSP_RANGING_VL53L5CX_Stop(void);

/** @brief *pReady = 1 when a new result waits: INT if wired, else a 4-byte bus read.
  *        With INT, a result that landed during the last read can show here and then
  *        read as BSP_ERROR_BUSY. */
int32_t BSP_RANGING_VL53L5CX_IsReady(uint8_t *pReady);

/**
  * @brief  Read the newest result over the bus: 11 ms at 8 x 8 zones, 4.5 ms at 4 x 4.
  * @retval BSP_ERROR_BUSY if none is new, BSP_ERROR_COMPONENT_FAILURE if the read
  *         failed or the part overwrote it mid-read.
  */
int32_t BSP_RANGING_VL53L5CX_GetResult(BSP_RANGING_VL53L5CX_Result_t *pResult);

/** @brief Weak, empty; runs between bus transfers and while the driver waits. Service
  *        USB and the watchdog here: Init takes about 3 s. */
void BSP_RANGING_VL53L5CX_WaitHook(void);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_RANGING_VL53L5CX_H */
