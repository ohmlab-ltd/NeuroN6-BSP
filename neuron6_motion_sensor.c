/**
  * @file    neuron6_motion_sensor.c
  * @brief   LSM6DSL accelerometer/gyroscope and MMC5603NJ magnetometer on the Neuro N6.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6_motion_sensor.h"
#include "neuron6.h"
#include "neuron6_bus.h"

static LSM6DSL_Object_t   MotionSensor_Lsm6dsl;

static MMC5603NJ_Object_t MotionSensor_Mmc5603nj;

static uint8_t MotionSensor_Ready[MOTION_SENSOR_INSTANCES_NBR];

static const uint32_t MotionSensor_Supported[MOTION_SENSOR_INSTANCES_NBR] =
{
  MOTION_ACCELERO | MOTION_GYRO,
  MOTION_MAGNETO,
};

static void    MotionSensor_Delay(uint32_t Milliseconds);
static int32_t MotionSensor_ProbeLsm6dsl(void);
static int32_t MotionSensor_ProbeMmc5603nj(void);
static int32_t MotionSensor_CheckFunction(uint32_t Instance, uint32_t Function);
static int32_t MotionSensor_CheckOne(uint32_t Instance, uint32_t Function);

/* Delay for the component drivers: the magnetometer's reset needs a 20 ms wait. */
static void MotionSensor_Delay(uint32_t Milliseconds)
{
  HAL_Delay(Milliseconds);
}

/* Bind the LSM6DSL to the sensor bus, trying both strap addresses. */
static int32_t MotionSensor_ProbeLsm6dsl(void)
{
  LSM6DSL_IO_t io;

  io.Init     = BSP_I2C1_Init;
  io.DeInit   = BSP_I2C1_DeInit;
  io.BusType  = LSM6DSL_I2C_BUS;
  io.Address  = MOTION_SENSOR_LSM6DSL_I2C_ADDRESS;
  io.WriteReg = BSP_I2C1_WriteReg;
  io.ReadReg  = BSP_I2C1_ReadReg;
  io.GetTick  = BSP_GetTick;
  io.Delay    = MotionSensor_Delay;

  if (LSM6DSL_RegisterBusIO(&MotionSensor_Lsm6dsl, &io) == LSM6DSL_OK)
  {
    return BSP_ERROR_NONE;
  }

  io.Address = (MOTION_SENSOR_LSM6DSL_I2C_ADDRESS == LSM6DSL_I2C_ADDRESS_LOW)
               ? LSM6DSL_I2C_ADDRESS_HIGH
               : LSM6DSL_I2C_ADDRESS_LOW;

  if (LSM6DSL_RegisterBusIO(&MotionSensor_Lsm6dsl, &io) == LSM6DSL_OK)
  {
    return BSP_ERROR_NONE;
  }

  return BSP_ERROR_UNKNOWN_COMPONENT;
}

static int32_t MotionSensor_ProbeMmc5603nj(void)
{
  MMC5603NJ_IO_t io;

  io.Init     = BSP_I2C1_Init;
  io.DeInit   = BSP_I2C1_DeInit;
  io.BusType  = MMC5603NJ_I2C_BUS;
  io.Address  = MOTION_SENSOR_MMC5603NJ_I2C_ADDRESS;
  io.WriteReg = BSP_I2C1_WriteReg;
  io.ReadReg  = BSP_I2C1_ReadReg;
  io.GetTick  = BSP_GetTick;
  io.Delay    = MotionSensor_Delay;

  if (MMC5603NJ_RegisterBusIO(&MotionSensor_Mmc5603nj, &io) != MMC5603NJ_OK)
  {
    return BSP_ERROR_UNKNOWN_COMPONENT;
  }

  return BSP_ERROR_NONE;
}

/* Check that an instance exists, is up, and has Function (0 skips that check). */
static int32_t MotionSensor_CheckFunction(uint32_t Instance, uint32_t Function)
{
  if (Instance >= MOTION_SENSOR_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (MotionSensor_Ready[Instance] == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if ((Function != 0U) && ((MotionSensor_Supported[Instance] & Function) != Function))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  return BSP_ERROR_NONE;
}

/* As MotionSensor_CheckFunction, and Function must be a single function: otherwise 0
 * or two bits would reach another part's driver. */
static int32_t MotionSensor_CheckOne(uint32_t Instance, uint32_t Function)
{
  if ((Function != MOTION_GYRO) && (Function != MOTION_ACCELERO) && (Function != MOTION_MAGNETO))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  return MotionSensor_CheckFunction(Instance, Function);
}

int32_t BSP_MOTION_SENSOR_Init(uint32_t Instance, uint32_t Functions)
{
  int32_t status;
  uint8_t brought_up;

  if (Instance >= MOTION_SENSOR_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* A function the instance does not have is rejected, not dropped. */
  if ((Functions == 0U) ||
      ((MotionSensor_Supported[Instance] & Functions) != Functions))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* On failure, release an instance this call brought up, so the bus is not held. */
  brought_up = (MotionSensor_Ready[Instance] == 0U) ? 1U : 0U;

  if (MotionSensor_Ready[Instance] == 0U)
  {
    if (Instance == MOTION_SENSOR_LSM6DSL_0)
    {
      status = MotionSensor_ProbeLsm6dsl();

      if (status != BSP_ERROR_NONE)
      {
        return status;
      }

      /* From here a failure gives the bus back through the driver. */
      if (LSM6DSL_Init(&MotionSensor_Lsm6dsl) != LSM6DSL_OK)
      {
        (void)LSM6DSL_DeInit(&MotionSensor_Lsm6dsl);
        return BSP_ERROR_COMPONENT_FAILURE;
      }

      /* Narrow ranges and a moderate rate: the more precise defaults. */
      if ((LSM6DSL_ACC_SetFullScale(&MotionSensor_Lsm6dsl,
                                    MOTION_SENSOR_ACC_DEFAULT_FS) != LSM6DSL_OK) ||
          (LSM6DSL_GYRO_SetFullScale(&MotionSensor_Lsm6dsl,
                                     MOTION_SENSOR_GYRO_DEFAULT_FS) != LSM6DSL_OK))
      {
        (void)LSM6DSL_DeInit(&MotionSensor_Lsm6dsl);
        return BSP_ERROR_COMPONENT_FAILURE;
      }

      (void)LSM6DSL_ACC_SetOutputDataRate(&MotionSensor_Lsm6dsl,
                                          MOTION_SENSOR_ACC_DEFAULT_ODR);
      (void)LSM6DSL_GYRO_SetOutputDataRate(&MotionSensor_Lsm6dsl,
                                           MOTION_SENSOR_GYRO_DEFAULT_ODR);
    }
    else
    {
      status = MotionSensor_ProbeMmc5603nj();

      if (status != BSP_ERROR_NONE)
      {
        return status;
      }

      if (MMC5603NJ_Init(&MotionSensor_Mmc5603nj) != MMC5603NJ_OK)
      {
        (void)MMC5603NJ_DeInit(&MotionSensor_Mmc5603nj);
        return BSP_ERROR_COMPONENT_FAILURE;
      }
    }

    MotionSensor_Ready[Instance] = 1U;
  }

  status = BSP_ERROR_NONE;

  if ((Functions & MOTION_ACCELERO) != 0U)
  {
    status = BSP_MOTION_SENSOR_Enable(Instance, MOTION_ACCELERO);
  }

  if ((status == BSP_ERROR_NONE) && ((Functions & MOTION_GYRO) != 0U))
  {
    status = BSP_MOTION_SENSOR_Enable(Instance, MOTION_GYRO);
  }

  if ((status == BSP_ERROR_NONE) && ((Functions & MOTION_MAGNETO) != 0U))
  {
    status = BSP_MOTION_SENSOR_Enable(Instance, MOTION_MAGNETO);
  }

  if ((status != BSP_ERROR_NONE) && (brought_up != 0U))
  {
    (void)BSP_MOTION_SENSOR_DeInit(Instance);
  }

  return status;
}

int32_t BSP_MOTION_SENSOR_DeInit(uint32_t Instance)
{
  int32_t status = MotionSensor_CheckFunction(Instance, 0U);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (Instance == MOTION_SENSOR_LSM6DSL_0)
  {
    if (LSM6DSL_DeInit(&MotionSensor_Lsm6dsl) != LSM6DSL_OK)
    {
      return BSP_ERROR_COMPONENT_FAILURE;
    }
  }
  else
  {
    if (MMC5603NJ_DeInit(&MotionSensor_Mmc5603nj) != MMC5603NJ_OK)
    {
      return BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  MotionSensor_Ready[Instance] = 0U;

  return BSP_ERROR_NONE;
}

int32_t BSP_MOTION_SENSOR_GetCapabilities(uint32_t Instance,
                                          BSP_MOTION_SENSOR_Capabilities_t *pCapabilities)
{
  int32_t status = MotionSensor_CheckFunction(Instance, 0U);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pCapabilities == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Instance == MOTION_SENSOR_LSM6DSL_0)
  {
    LSM6DSL_Capabilities_t caps;

    if (LSM6DSL_GetCapabilities(&MotionSensor_Lsm6dsl, &caps) != LSM6DSL_OK)
    {
      return BSP_ERROR_COMPONENT_FAILURE;
    }

    pCapabilities->Acc        = caps.Acc;
    pCapabilities->Gyro       = caps.Gyro;
    pCapabilities->Magneto    = caps.Magneto;
    pCapabilities->LowPower   = caps.LowPower;
    pCapabilities->GyroMaxFS  = caps.GyroMaxFS;
    pCapabilities->AccMaxFS   = caps.AccMaxFS;
    pCapabilities->MagMaxFS   = caps.MagMaxFS;
    pCapabilities->GyroMaxOdr = caps.GyroMaxOdr;
    pCapabilities->AccMaxOdr  = caps.AccMaxOdr;
    pCapabilities->MagMaxOdr  = caps.MagMaxOdr;
  }
  else
  {
    MMC5603NJ_Capabilities_t caps;

    if (MMC5603NJ_GetCapabilities(&MotionSensor_Mmc5603nj, &caps) != MMC5603NJ_OK)
    {
      return BSP_ERROR_COMPONENT_FAILURE;
    }

    pCapabilities->Acc        = caps.Acc;
    pCapabilities->Gyro       = caps.Gyro;
    pCapabilities->Magneto    = caps.Magneto;
    pCapabilities->LowPower   = caps.LowPower;
    pCapabilities->GyroMaxFS  = caps.GyroMaxFS;
    pCapabilities->AccMaxFS   = caps.AccMaxFS;
    pCapabilities->MagMaxFS   = caps.MagMaxFS;
    pCapabilities->GyroMaxOdr = caps.GyroMaxOdr;
    pCapabilities->AccMaxOdr  = caps.AccMaxOdr;
    pCapabilities->MagMaxOdr  = caps.MagMaxOdr;
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_MOTION_SENSOR_ReadID(uint32_t Instance, uint8_t *pId)
{
  int32_t status = MotionSensor_CheckFunction(Instance, 0U);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pId == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Instance == MOTION_SENSOR_LSM6DSL_0)
  {
    if (LSM6DSL_ReadID(&MotionSensor_Lsm6dsl, pId) != LSM6DSL_OK)
    {
      return BSP_ERROR_COMPONENT_FAILURE;
    }
  }
  else
  {
    if (MMC5603NJ_ReadID(&MotionSensor_Mmc5603nj, pId) != MMC5603NJ_OK)
    {
      return BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_MOTION_SENSOR_Enable(uint32_t Instance, uint32_t Function)
{
  int32_t status = MotionSensor_CheckOne(Instance, Function);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  switch (Function)
  {
    case MOTION_ACCELERO:
      status = (LSM6DSL_ACC_Enable(&MotionSensor_Lsm6dsl) == LSM6DSL_OK)
               ? BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
      break;

    case MOTION_GYRO:
      status = (LSM6DSL_GYRO_Enable(&MotionSensor_Lsm6dsl) == LSM6DSL_OK)
               ? BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
      break;

    case MOTION_MAGNETO:
      status = (MMC5603NJ_MAG_Enable(&MotionSensor_Mmc5603nj) == MMC5603NJ_OK)
               ? BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
      break;

    default:
      status = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return status;
}

int32_t BSP_MOTION_SENSOR_Disable(uint32_t Instance, uint32_t Function)
{
  int32_t status = MotionSensor_CheckOne(Instance, Function);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  switch (Function)
  {
    case MOTION_ACCELERO:
      status = (LSM6DSL_ACC_Disable(&MotionSensor_Lsm6dsl) == LSM6DSL_OK)
               ? BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
      break;

    case MOTION_GYRO:
      status = (LSM6DSL_GYRO_Disable(&MotionSensor_Lsm6dsl) == LSM6DSL_OK)
               ? BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
      break;

    case MOTION_MAGNETO:
      status = (MMC5603NJ_MAG_Disable(&MotionSensor_Mmc5603nj) == MMC5603NJ_OK)
               ? BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
      break;

    default:
      status = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return status;
}

int32_t BSP_MOTION_SENSOR_GetAxes(uint32_t Instance, uint32_t Function,
                                  BSP_MOTION_SENSOR_Axes_t *pAxes)
{
  int32_t status = MotionSensor_CheckOne(Instance, Function);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pAxes == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Function == MOTION_MAGNETO)
  {
    MMC5603NJ_Axes_t axes;

    if (MMC5603NJ_MAG_GetAxes(&MotionSensor_Mmc5603nj, &axes) != MMC5603NJ_OK)
    {
      return BSP_ERROR_COMPONENT_FAILURE;
    }

    pAxes->x = axes.x;
    pAxes->y = axes.y;
    pAxes->z = axes.z;
  }
  else
  {
    LSM6DSL_Axes_t axes;
    int32_t component;

    component = (Function == MOTION_ACCELERO)
                ? LSM6DSL_ACC_GetAxes(&MotionSensor_Lsm6dsl, &axes)
                : LSM6DSL_GYRO_GetAxes(&MotionSensor_Lsm6dsl, &axes);

    if (component != LSM6DSL_OK)
    {
      return BSP_ERROR_COMPONENT_FAILURE;
    }

    pAxes->x = axes.x;
    pAxes->y = axes.y;
    pAxes->z = axes.z;
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_MOTION_SENSOR_GetAxesRaw(uint32_t Instance, uint32_t Function,
                                     BSP_MOTION_SENSOR_AxesRaw_t *pAxes)
{
  int32_t status = MotionSensor_CheckOne(Instance, Function);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pAxes == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Function == MOTION_MAGNETO)
  {
    MMC5603NJ_AxesRaw_t axes;

    if (MMC5603NJ_MAG_GetAxesRaw(&MotionSensor_Mmc5603nj, &axes) != MMC5603NJ_OK)
    {
      return BSP_ERROR_COMPONENT_FAILURE;
    }

    pAxes->x = axes.x;
    pAxes->y = axes.y;
    pAxes->z = axes.z;
  }
  else
  {
    LSM6DSL_AxesRaw_t axes;
    int32_t component;

    component = (Function == MOTION_ACCELERO)
                ? LSM6DSL_ACC_GetAxesRaw(&MotionSensor_Lsm6dsl, &axes)
                : LSM6DSL_GYRO_GetAxesRaw(&MotionSensor_Lsm6dsl, &axes);

    if (component != LSM6DSL_OK)
    {
      return BSP_ERROR_COMPONENT_FAILURE;
    }

    /* 16-bit counts; the type is wider for the magnetometer's 20 bits. */
    pAxes->x = (int32_t)axes.x;
    pAxes->y = (int32_t)axes.y;
    pAxes->z = (int32_t)axes.z;
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_MOTION_SENSOR_GetSensitivity(uint32_t Instance, uint32_t Function,
                                         float *pSensitivity)
{
  int32_t status = MotionSensor_CheckOne(Instance, Function);
  int32_t component;

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pSensitivity == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  switch (Function)
  {
    case MOTION_ACCELERO:
      component = LSM6DSL_ACC_GetSensitivity(&MotionSensor_Lsm6dsl, pSensitivity);
      break;

    case MOTION_GYRO:
      component = LSM6DSL_GYRO_GetSensitivity(&MotionSensor_Lsm6dsl, pSensitivity);
      break;

    default:
      component = MMC5603NJ_MAG_GetSensitivity(&MotionSensor_Mmc5603nj, pSensitivity);
      break;
  }

  return (component == 0) ? BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
}

int32_t BSP_MOTION_SENSOR_GetOutputDataRate(uint32_t Instance, uint32_t Function,
                                            float *pOdr)
{
  int32_t status = MotionSensor_CheckOne(Instance, Function);
  int32_t component;

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pOdr == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  switch (Function)
  {
    case MOTION_ACCELERO:
      component = LSM6DSL_ACC_GetOutputDataRate(&MotionSensor_Lsm6dsl, pOdr);
      break;

    case MOTION_GYRO:
      component = LSM6DSL_GYRO_GetOutputDataRate(&MotionSensor_Lsm6dsl, pOdr);
      break;

    default:
      component = MMC5603NJ_MAG_GetOutputDataRate(&MotionSensor_Mmc5603nj, pOdr);
      break;
  }

  return (component == 0) ? BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
}

int32_t BSP_MOTION_SENSOR_SetOutputDataRate(uint32_t Instance, uint32_t Function,
                                            float Odr)
{
  int32_t status = MotionSensor_CheckOne(Instance, Function);
  int32_t component;

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  switch (Function)
  {
    case MOTION_ACCELERO:
      component = LSM6DSL_ACC_SetOutputDataRate(&MotionSensor_Lsm6dsl, Odr);
      break;

    case MOTION_GYRO:
      component = LSM6DSL_GYRO_SetOutputDataRate(&MotionSensor_Lsm6dsl, Odr);
      break;

    default:
      component = MMC5603NJ_MAG_SetOutputDataRate(&MotionSensor_Mmc5603nj, Odr);
      break;
  }

  return (component == 0) ? BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
}

int32_t BSP_MOTION_SENSOR_GetFullScale(uint32_t Instance, uint32_t Function,
                                       int32_t *pFullScale)
{
  int32_t status = MotionSensor_CheckOne(Instance, Function);
  int32_t component;

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pFullScale == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  switch (Function)
  {
    case MOTION_ACCELERO:
      component = LSM6DSL_ACC_GetFullScale(&MotionSensor_Lsm6dsl, pFullScale);
      break;

    case MOTION_GYRO:
      component = LSM6DSL_GYRO_GetFullScale(&MotionSensor_Lsm6dsl, pFullScale);
      break;

    default:
      component = MMC5603NJ_MAG_GetFullScale(&MotionSensor_Mmc5603nj, pFullScale);
      break;
  }

  return (component == 0) ? BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
}

int32_t BSP_MOTION_SENSOR_SetFullScale(uint32_t Instance, uint32_t Function,
                                       int32_t FullScale)
{
  int32_t status = MotionSensor_CheckOne(Instance, Function);
  int32_t component;

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  switch (Function)
  {
    case MOTION_ACCELERO:
      component = LSM6DSL_ACC_SetFullScale(&MotionSensor_Lsm6dsl, FullScale);
      break;

    case MOTION_GYRO:
      component = LSM6DSL_GYRO_SetFullScale(&MotionSensor_Lsm6dsl, FullScale);
      break;

    default:
      component = MMC5603NJ_MAG_SetFullScale(&MotionSensor_Mmc5603nj, FullScale);
      break;
  }

  return (component == 0) ? BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
}

int32_t BSP_MOTION_SENSOR_GetTemperature(uint32_t Instance, float *pTemp)
{
  int32_t status = MotionSensor_CheckFunction(Instance, 0U);
  int32_t component;

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pTemp == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  component = (Instance == MOTION_SENSOR_LSM6DSL_0)
              ? LSM6DSL_GetTemperature(&MotionSensor_Lsm6dsl, pTemp)
              : MMC5603NJ_GetTemperature(&MotionSensor_Mmc5603nj, pTemp);

  return (component == 0) ? BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
}

int32_t BSP_MOTION_SENSOR_GetDataReady(uint32_t Instance, uint32_t *pStatus)
{
  int32_t status = MotionSensor_CheckFunction(Instance, 0U);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pStatus == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  *pStatus = 0U;

  if (Instance == MOTION_SENSOR_LSM6DSL_0)
  {
    uint8_t flags = 0U;

    if (LSM6DSL_GetStatus(&MotionSensor_Lsm6dsl, &flags) != LSM6DSL_OK)
    {
      return BSP_ERROR_COMPONENT_FAILURE;
    }

    if ((flags & LSM6DSL_STATUS_XLDA) != 0U)
    {
      *pStatus |= MOTION_ACCELERO;
    }

    if ((flags & LSM6DSL_STATUS_GDA) != 0U)
    {
      *pStatus |= MOTION_GYRO;
    }
  }
  else
  {
    uint8_t ready = 0U;

    if (MMC5603NJ_MAG_GetDataReady(&MotionSensor_Mmc5603nj, &ready) != MMC5603NJ_OK)
    {
      return BSP_ERROR_COMPONENT_FAILURE;
    }

    if (ready != 0U)
    {
      *pStatus |= MOTION_MAGNETO;
    }
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_MOTION_SENSOR_Degauss(uint32_t Instance)
{
  int32_t status = MotionSensor_CheckFunction(Instance, MOTION_MAGNETO);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  return (MMC5603NJ_MAG_Degauss(&MotionSensor_Mmc5603nj) == MMC5603NJ_OK)
         ? BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
}
