/**
  * @file    neuron6_ranging_sensor.h
  * @brief   VL53L1CB time-of-flight ranging: one ranger, or a 16-zone depth grid.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_RANGING_SENSOR_H
#define NEURON6_RANGING_SENSOR_H

#include "neuron6_conf.h"
#include "neuron6_errno.h"
#include "vl53l1cb.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One VL53L1CB on the on-board sensor bus of the OV5640 carriers (plain and TFT-W);
 * the thermal carrier has none. Its interrupt pin is not routed: results are polled. */

/** @brief The only ranging-sensor instance on this board. */
#define RANGING_SENSOR_VL53L1CB_0           0U

#define RANGING_SENSOR_INSTANCES_NBR        1U

/** @brief VL53L1CB 8-bit I2C address. */
#define RANGING_SENSOR_I2C_ADDRESS          VL53L1CB_DEVICE_ADDRESS

/** @brief Value BSP_RANGING_SENSOR_ReadID returns on a working part. */
#define RANGING_SENSOR_ID                   VL53L1CB_ID

/** @brief Most targets the part reports from one region. */
#define RANGING_SENSOR_MAX_NB_TARGETS       1U

/** @brief Regions per measurement: the part ranges one at a time. */
#define RANGING_SENSOR_MAX_NB_ZONES         1U

/** @brief Time one measurement takes, in milliseconds. */
#define RANGING_SENSOR_TIMING_BUDGET_MS     VL53L1CB_TIMING_BUDGET_MS

/** @brief Modes: blocking ones wait in BSP_RANGING_SENSOR_GetDistance; async ones return
  *        at once, to poll with BSP_RANGING_SENSOR_IsMeasurementReady. One-shot modes
  *        stop the part once the measurement is read. */
#define RANGING_SENSOR_MODE_BLOCKING_CONTINUOUS 1U
#define RANGING_SENSOR_MODE_BLOCKING_ONESHOT    2U
#define RANGING_SENSOR_MODE_ASYNC_CONTINUOUS    3U
#define RANGING_SENSOR_MODE_ASYNC_ONESHOT       4U

/** @brief Swept depth-grid geometry. */
#define RANGING_SENSOR_GRID_ROWS            4U
#define RANGING_SENSOR_GRID_COLS            4U
#define RANGING_SENSOR_GRID_ZONES           (RANGING_SENSOR_GRID_ROWS * RANGING_SENSOR_GRID_COLS)

/** @brief Zone width and height in SPADs: the narrowest region the part accepts. */
#define RANGING_SENSOR_GRID_ZONE_SPADS      VL53L1CB_MIN_ROI

typedef struct
{
  uint32_t NumberOfZones;
  uint32_t MaxNumberOfTargetsPerZone;
  uint32_t CustomROI;                 /*!< 1 if the region can be narrowed.   */
  uint32_t ThresholdDetection;        /*!< 1 if threshold interrupts exist.   */
} RANGING_SENSOR_Capabilities_t;

/** @brief Region to range: X 0-15 from the left, Y 0-15 from the bottom (so the top edge
  *        has the larger Y); at least 4 x 4 SPADs. */
typedef struct
{
  uint8_t TopLeftX;
  uint8_t TopLeftY;
  uint8_t BotRightX;
  uint8_t BotRightY;
} RANGING_SENSOR_ROIConfig_t;

/** @brief What one region saw. */
typedef struct
{
  uint32_t NumberOfTargets;                         /*!< 1 for a good range, 0 for none. */
  uint32_t Distance[RANGING_SENSOR_MAX_NB_TARGETS]; /*!< mm, as the part reported it.    */
  uint32_t Status[RANGING_SENSOR_MAX_NB_TARGETS];   /*!< 0 good, else the range status.  */
} RANGING_SENSOR_TargetResult_t;

typedef struct
{
  uint32_t NumberOfZones;
  RANGING_SENSOR_TargetResult_t ZoneResult[RANGING_SENSOR_MAX_NB_ZONES];
} RANGING_SENSOR_Result_t;

typedef struct
{
  uint16_t Distance[RANGING_SENSOR_GRID_ROWS][RANGING_SENSOR_GRID_COLS];
                        /*!< mm, 0 = no good range; row 0 is the top, column 0 the left. */
  uint32_t SweepCount;  /*!< Completed sweeps since the grid was started.     */
  uint32_t ZoneIndex;   /*!< Zone the next step will range, 0 to 15.          */
} RANGING_SENSOR_Grid_t;

/** @brief Bring up the bus and the part (see VL53L1CB_Init, about 150 ms); the region is
  *        the whole array. Called again, it stops what is running and resets the part.
  * @retval BSP_ERROR_UNKNOWN_COMPONENT if nothing answered as a VL53L1CB,
  *         BSP_ERROR_COMPONENT_FAILURE if the part did not come up. */
int32_t BSP_RANGING_SENSOR_Init(uint32_t Instance);

/** @brief Stop the ranger and release its share of the sensor bus. */
int32_t BSP_RANGING_SENSOR_DeInit(uint32_t Instance);

/** @brief Read the identity (RANGING_SENSOR_ID on a working part). */
int32_t BSP_RANGING_SENSOR_ReadID(uint32_t Instance, uint32_t *pId);

int32_t BSP_RANGING_SENSOR_GetCapabilities(uint32_t Instance,
                                           RANGING_SENSOR_Capabilities_t *pCapabilities);

/** @brief Narrow the ranged region: a narrower field of view, but less light and range.
  * @retval BSP_ERROR_WRONG_PARAM outside the array or under 4 x 4 SPADs,
  *         BSP_ERROR_BUSY while ranging. */
int32_t BSP_RANGING_SENSOR_ConfigROI(uint32_t Instance, RANGING_SENSOR_ROIConfig_t *pConfig);

/** @brief Start ranging in a RANGING_SENSOR_MODE_* mode.
  * @retval BSP_ERROR_BUSY if ranging or a grid sweep is already running. */
int32_t BSP_RANGING_SENSOR_Start(uint32_t Instance, uint32_t Mode);

/** @brief Stop ranging; BSP_ERROR_NO_INIT if it was not running. */
int32_t BSP_RANGING_SENSOR_Stop(uint32_t Instance);

/** @brief *pReady = 1 when a measurement is waiting; BSP_ERROR_NO_INIT if not ranging. */
int32_t BSP_RANGING_SENSOR_IsMeasurementReady(uint32_t Instance, uint8_t *pReady);

/** @brief Collect a measurement. Blocking modes wait up to three timing budgets; async
  *        modes return at once. BSP_ERROR_BUSY if none was waiting.
  * @note  Only Status 0 is a distance. 4: return under VL53L1CB_MIN_SIGNAL_KCPS;
  *        6: sigma over VL53L1CB_SIGMA_LIMIT_MM; 7: phases disagree (a far target wraps). */
int32_t BSP_RANGING_SENSOR_GetDistance(uint32_t Instance, RANGING_SENSOR_Result_t *pResult);

/** @brief Start sweeping the grid: one 4 x 4 SPAD zone per timing budget or less (a weak
  *        return ends early), so up to about 1.6 s a sweep. The grid starts at zero. */
int32_t BSP_RANGING_SENSOR_StartGrid(uint32_t Instance);

/** @brief Advance the sweep; call once per pass of the application loop (never waits).
  * @note  A zone that fails reads 0 and the sweep moves on; the step then returns
  *        BSP_ERROR_COMPONENT_FAILURE. */
int32_t BSP_RANGING_SENSOR_StepGrid(uint32_t Instance);

/** @brief Copy the grid; it fills zone by zone, so it holds a mixture of ages. */
int32_t BSP_RANGING_SENSOR_GetGrid(uint32_t Instance, RANGING_SENSOR_Grid_t *pGrid);

/** @brief Stop the sweep; the part is left idle with the whole array as its region. */
int32_t BSP_RANGING_SENSOR_StopGrid(uint32_t Instance);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_RANGING_SENSOR_H */
