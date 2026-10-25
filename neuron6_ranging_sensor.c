/**
  * @file    neuron6_ranging_sensor.c
  * @brief   VL53L1CB time-of-flight ranging: one ranger, or a 16-zone depth grid.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6_ranging_sensor.h"
#include "neuron6.h"
#include "neuron6_bus.h"

#include <string.h>

/* Give up on a measurement after three timing budgets plus the part's start-up time. */
#define RANGING_SENSOR_WAIT_MS    ((3U * RANGING_SENSOR_TIMING_BUDGET_MS) + 50U)

static VL53L1CB_Object_t RangingSensor_Obj;

static uint8_t RangingSensor_Ready;

/* Set while a BSP_RANGING_SENSOR_Start session runs. */
static uint8_t RangingSensor_Ranging;

static uint32_t RangingSensor_Mode;

static uint8_t RangingSensor_GridOn;

static uint8_t RangingSensor_ZoneBusy;

/* Start of the current zone's measurement, ms. */
static uint32_t RangingSensor_ZoneStart;

static RANGING_SENSOR_Grid_t RangingSensor_Grid;

static void    RangingSensor_Delay(uint32_t Milliseconds);
static int32_t RangingSensor_SetRegion(uint8_t Left, uint8_t Bottom, uint8_t Width,
                                       uint8_t Height);
static int32_t RangingSensor_Check(uint32_t Instance);
static void    RangingSensor_NextZone(uint16_t DistanceMm);

static void RangingSensor_Delay(uint32_t Milliseconds)
{
  HAL_Delay(Milliseconds);
}

/* Range a region given by its bottom-left SPAD and size; the part takes its centre (for
 * an even size, the SPAD above and right of the middle). */
static int32_t RangingSensor_SetRegion(uint8_t Left, uint8_t Bottom, uint8_t Width,
                                       uint8_t Height)
{
  const uint8_t spad = VL53L1CB_SpadNumber((uint8_t)(Bottom + (Height / 2U)),
                                           (uint8_t)(Left + (Width / 2U)));

  return (VL53L1CB_SetROI(&RangingSensor_Obj, Width, Height, spad) == VL53L1CB_OK) ?
         BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
}

/* Checks common to every entry point after Init. */
static int32_t RangingSensor_Check(uint32_t Instance)
{
  if (Instance >= RANGING_SENSOR_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  return (RangingSensor_Ready != 0U) ? BSP_ERROR_NONE : BSP_ERROR_NO_INIT;
}

/* Store the zone's range (0 = none) and move to the next zone. */
static void RangingSensor_NextZone(uint16_t DistanceMm)
{
  const uint32_t zone = RangingSensor_Grid.ZoneIndex;

  RangingSensor_Grid.Distance[zone / RANGING_SENSOR_GRID_COLS][zone % RANGING_SENSOR_GRID_COLS] =
    DistanceMm;
  RangingSensor_ZoneBusy = 0U;
  RangingSensor_Grid.ZoneIndex = (zone + 1U) % RANGING_SENSOR_GRID_ZONES;
  if (RangingSensor_Grid.ZoneIndex == 0U)
  {
    RangingSensor_Grid.SweepCount++;
  }
}

int32_t BSP_RANGING_SENSOR_Init(uint32_t Instance)
{
  VL53L1CB_IO_t io;
  uint16_t      id = 0U;

  if (Instance >= RANGING_SENSOR_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  RangingSensor_Ranging = 0U;
  RangingSensor_GridOn  = 0U;

  /* Already up: the bus is held, so only the part is reset. */
  if (RangingSensor_Ready != 0U)
  {
    if (VL53L1CB_Init(&RangingSensor_Obj) != VL53L1CB_OK)
    {
      (void)VL53L1CB_DeInit(&RangingSensor_Obj);
      RangingSensor_Ready = 0U;
      return BSP_ERROR_COMPONENT_FAILURE;
    }
    return BSP_ERROR_NONE;
  }

  io.Init     = BSP_I2C1_Init;
  io.DeInit   = BSP_I2C1_DeInit;
  io.BusType  = VL53L1CB_I2C_BUS;
  io.Address  = RANGING_SENSOR_I2C_ADDRESS;
  io.WriteReg = BSP_I2C1_WriteReg16;
  io.ReadReg  = BSP_I2C1_ReadReg16;
  io.GetTick  = BSP_GetTick;
  io.Delay    = RangingSensor_Delay;
  if (VL53L1CB_RegisterBusIO(&RangingSensor_Obj, &io) != VL53L1CB_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  if ((VL53L1CB_ReadID(&RangingSensor_Obj, &id) != VL53L1CB_OK) || (id != VL53L1CB_ID))
  {
    (void)BSP_I2C1_DeInit();
    return BSP_ERROR_UNKNOWN_COMPONENT;
  }
  if (VL53L1CB_Init(&RangingSensor_Obj) != VL53L1CB_OK)
  {
    (void)BSP_I2C1_DeInit();
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  RangingSensor_Ready = 1U;
  return BSP_ERROR_NONE;
}

int32_t BSP_RANGING_SENSOR_DeInit(uint32_t Instance)
{
  const int32_t ret = RangingSensor_Check(Instance);

  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }
  (void)VL53L1CB_DeInit(&RangingSensor_Obj);
  RangingSensor_Ready   = 0U;
  RangingSensor_Ranging = 0U;
  RangingSensor_GridOn  = 0U;
  return BSP_ERROR_NONE;
}

int32_t BSP_RANGING_SENSOR_ReadID(uint32_t Instance, uint32_t *pId)
{
  uint16_t id  = 0U;
  int32_t  ret = RangingSensor_Check(Instance);

  if ((ret == BSP_ERROR_NONE) && (pId == NULL))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }
  if (VL53L1CB_ReadID(&RangingSensor_Obj, &id) != VL53L1CB_OK)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }
  *pId = id;
  return BSP_ERROR_NONE;
}

int32_t BSP_RANGING_SENSOR_GetCapabilities(uint32_t Instance,
                                           RANGING_SENSOR_Capabilities_t *pCapabilities)
{
  int32_t ret = RangingSensor_Check(Instance);

  if ((ret == BSP_ERROR_NONE) && (pCapabilities == NULL))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }
  pCapabilities->NumberOfZones             = RANGING_SENSOR_MAX_NB_ZONES;
  pCapabilities->MaxNumberOfTargetsPerZone = RANGING_SENSOR_MAX_NB_TARGETS;
  pCapabilities->CustomROI                 = 1U;
  pCapabilities->ThresholdDetection        = 0U;
  return BSP_ERROR_NONE;
}

int32_t BSP_RANGING_SENSOR_ConfigROI(uint32_t Instance, RANGING_SENSOR_ROIConfig_t *pConfig)
{
  int32_t ret = RangingSensor_Check(Instance);
  uint32_t width;
  uint32_t height;

  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }
  if ((pConfig == NULL) || (pConfig->BotRightX >= VL53L1CB_SPADS) ||
      (pConfig->TopLeftY >= VL53L1CB_SPADS) || (pConfig->TopLeftX > pConfig->BotRightX) ||
      (pConfig->BotRightY > pConfig->TopLeftY))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  width  = (uint32_t)pConfig->BotRightX - pConfig->TopLeftX + 1U;
  height = (uint32_t)pConfig->TopLeftY - pConfig->BotRightY + 1U;
  if ((width < VL53L1CB_MIN_ROI) || (height < VL53L1CB_MIN_ROI))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if ((RangingSensor_Ranging != 0U) || (RangingSensor_GridOn != 0U))
  {
    return BSP_ERROR_BUSY;
  }
  return RangingSensor_SetRegion(pConfig->TopLeftX, pConfig->BotRightY, (uint8_t)width,
                                 (uint8_t)height);
}

int32_t BSP_RANGING_SENSOR_Start(uint32_t Instance, uint32_t Mode)
{
  int32_t ret = RangingSensor_Check(Instance);

  if ((ret == BSP_ERROR_NONE) &&
      ((Mode < RANGING_SENSOR_MODE_BLOCKING_CONTINUOUS) || (Mode > RANGING_SENSOR_MODE_ASYNC_ONESHOT)))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }
  if ((RangingSensor_Ranging != 0U) || (RangingSensor_GridOn != 0U))
  {
    return BSP_ERROR_BUSY;
  }
  if (VL53L1CB_StartRanging(&RangingSensor_Obj) != VL53L1CB_OK)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }
  RangingSensor_Mode    = Mode;
  RangingSensor_Ranging = 1U;
  return BSP_ERROR_NONE;
}

int32_t BSP_RANGING_SENSOR_Stop(uint32_t Instance)
{
  const int32_t ret = RangingSensor_Check(Instance);

  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }
  if (RangingSensor_Ranging == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }
  RangingSensor_Ranging = 0U;
  return (VL53L1CB_StopRanging(&RangingSensor_Obj) == VL53L1CB_OK) ?
         BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
}

int32_t BSP_RANGING_SENSOR_IsMeasurementReady(uint32_t Instance, uint8_t *pReady)
{
  int32_t ret = RangingSensor_Check(Instance);

  if ((ret == BSP_ERROR_NONE) && (pReady == NULL))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }
  if (RangingSensor_Ranging == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }
  return (VL53L1CB_DataReady(&RangingSensor_Obj, pReady) == VL53L1CB_OK) ?
         BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
}

int32_t BSP_RANGING_SENSOR_GetDistance(uint32_t Instance, RANGING_SENSOR_Result_t *pResult)
{
  int32_t  ret = RangingSensor_Check(Instance);
  uint8_t  ready = 0U;
  uint8_t  status = 0U;
  uint16_t mm = 0U;
  uint32_t start;

  if ((ret == BSP_ERROR_NONE) && (pResult == NULL))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }
  if (RangingSensor_Ranging == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  start = (uint32_t)BSP_GetTick();
  for (;;)
  {
    if (VL53L1CB_DataReady(&RangingSensor_Obj, &ready) != VL53L1CB_OK)
    {
      return BSP_ERROR_COMPONENT_FAILURE;
    }
    if ((ready != 0U) ||
        (RangingSensor_Mode == RANGING_SENSOR_MODE_ASYNC_CONTINUOUS) ||
        (RangingSensor_Mode == RANGING_SENSOR_MODE_ASYNC_ONESHOT) ||
        (((uint32_t)BSP_GetTick() - start) >= RANGING_SENSOR_WAIT_MS))
    {
      break;
    }
    RangingSensor_Delay(1U);
  }
  if (ready == 0U)
  {
    return BSP_ERROR_BUSY;
  }

  if (VL53L1CB_ReadResult(&RangingSensor_Obj, &mm, &status) != VL53L1CB_OK)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }
  if ((RangingSensor_Mode == RANGING_SENSOR_MODE_BLOCKING_ONESHOT) ||
      (RangingSensor_Mode == RANGING_SENSOR_MODE_ASYNC_ONESHOT))
  {
    RangingSensor_Ranging = 0U;
    (void)VL53L1CB_StopRanging(&RangingSensor_Obj);
  }

  pResult->NumberOfZones                = 1U;
  pResult->ZoneResult[0].NumberOfTargets = (status == VL53L1CB_RANGE_VALID) ? 1U : 0U;
  pResult->ZoneResult[0].Distance[0]     = mm;
  pResult->ZoneResult[0].Status[0]       = (status == VL53L1CB_RANGE_VALID) ? 0U : status;
  return BSP_ERROR_NONE;
}

int32_t BSP_RANGING_SENSOR_StartGrid(uint32_t Instance)
{
  const int32_t ret = RangingSensor_Check(Instance);

  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }
  if ((RangingSensor_Ranging != 0U) || (RangingSensor_GridOn != 0U))
  {
    return BSP_ERROR_BUSY;
  }
  (void)memset(&RangingSensor_Grid, 0, sizeof(RangingSensor_Grid));
  RangingSensor_ZoneBusy = 0U;
  RangingSensor_GridOn   = 1U;
  return BSP_ERROR_NONE;
}

int32_t BSP_RANGING_SENSOR_StepGrid(uint32_t Instance)
{
  int32_t  ret = RangingSensor_Check(Instance);
  uint8_t  ready = 0U;
  uint8_t  status = 0U;
  uint16_t mm = 0U;

  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }
  if (RangingSensor_GridOn == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (RangingSensor_ZoneBusy == 0U)
  {
    /* Zone (r, c): detector rows 15 - 4r down to 12 - 4r, columns 4c to 4c + 3. */
    const uint32_t row = RangingSensor_Grid.ZoneIndex / RANGING_SENSOR_GRID_COLS;
    const uint32_t col = RangingSensor_Grid.ZoneIndex % RANGING_SENSOR_GRID_COLS;

    if ((RangingSensor_SetRegion((uint8_t)(RANGING_SENSOR_GRID_ZONE_SPADS * col),
                                 (uint8_t)(12U - (RANGING_SENSOR_GRID_ZONE_SPADS * row)),
                                 RANGING_SENSOR_GRID_ZONE_SPADS,
                                 RANGING_SENSOR_GRID_ZONE_SPADS) != BSP_ERROR_NONE) ||
        (VL53L1CB_StartRanging(&RangingSensor_Obj) != VL53L1CB_OK))
    {
      RangingSensor_NextZone(0U);
      return BSP_ERROR_COMPONENT_FAILURE;
    }
    RangingSensor_ZoneStart = (uint32_t)BSP_GetTick();
    RangingSensor_ZoneBusy  = 1U;
    return BSP_ERROR_NONE;
  }

  if (VL53L1CB_DataReady(&RangingSensor_Obj, &ready) != VL53L1CB_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else if (ready != 0U)
  {
    /* One result, then the part stops: the next zone needs a new region. */
    if (VL53L1CB_ReadResult(&RangingSensor_Obj, &mm, &status) != VL53L1CB_OK)
    {
      status = 0U;
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    (void)VL53L1CB_StopRanging(&RangingSensor_Obj);
    RangingSensor_NextZone((status == VL53L1CB_RANGE_VALID) ? mm : 0U);
    return ret;
  }
  else if (((uint32_t)BSP_GetTick() - RangingSensor_ZoneStart) < RANGING_SENSOR_WAIT_MS)
  {
    return BSP_ERROR_NONE;
  }
  else
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }

  /* No answer, or the zone took too long: it reads as no range and the sweep goes on. */
  (void)VL53L1CB_StopRanging(&RangingSensor_Obj);
  RangingSensor_NextZone(0U);
  return ret;
}

int32_t BSP_RANGING_SENSOR_GetGrid(uint32_t Instance, RANGING_SENSOR_Grid_t *pGrid)
{
  int32_t ret = RangingSensor_Check(Instance);

  if ((ret == BSP_ERROR_NONE) && (pGrid == NULL))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }
  if (RangingSensor_GridOn == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }
  *pGrid = RangingSensor_Grid;
  return BSP_ERROR_NONE;
}

int32_t BSP_RANGING_SENSOR_StopGrid(uint32_t Instance)
{
  const int32_t ret = RangingSensor_Check(Instance);

  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }
  if (RangingSensor_GridOn == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }
  RangingSensor_GridOn   = 0U;
  RangingSensor_ZoneBusy = 0U;
  (void)VL53L1CB_StopRanging(&RangingSensor_Obj);
  return RangingSensor_SetRegion(0U, 0U, VL53L1CB_SPADS, VL53L1CB_SPADS);
}
