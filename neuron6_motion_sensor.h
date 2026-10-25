/**
  * @file    neuron6_motion_sensor.h
  * @brief   LSM6DSL accelerometer/gyroscope and MMC5603NJ magnetometer on the Neuro N6.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_MOTION_SENSOR_H
#define NEURON6_MOTION_SENSOR_H

#include "neuron6_conf.h"
#include "neuron6_errno.h"
#include "lsm6dsl.h"
#include "mmc5603nj.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ST-style motion-sensor API for the two parts on the on-board sensor bus. */

/** @brief Instance carrying the accelerometer and gyroscope. */
#define MOTION_SENSOR_LSM6DSL_0         0U

/** @brief Instance carrying the magnetometer. */
#define MOTION_SENSOR_MMC5603NJ_0       1U

#define MOTION_SENSOR_INSTANCES_NBR     2U

/** @brief Function bits; BSP_MOTION_SENSOR_Init takes a mask of them. */
#define MOTION_GYRO                     1U
#define MOTION_ACCELERO                 2U
#define MOTION_MAGNETO                  4U

/** @brief LSM6DSL 8-bit I2C address; the board strap selects the lower one. */
#define MOTION_SENSOR_LSM6DSL_I2C_ADDRESS   LSM6DSL_I2C_ADDRESS_LOW

/** @brief MMC5603NJ 8-bit I2C address (fixed). */
#define MOTION_SENSOR_MMC5603NJ_I2C_ADDRESS MMC5603NJ_I2C_ADDRESS

/** @brief Accelerometer output rate applied by BSP_MOTION_SENSOR_Init, in Hz. */
#ifndef MOTION_SENSOR_ACC_DEFAULT_ODR
#define MOTION_SENSOR_ACC_DEFAULT_ODR   104.0f
#endif

/** @brief Gyroscope output rate applied by BSP_MOTION_SENSOR_Init, in Hz. */
#ifndef MOTION_SENSOR_GYRO_DEFAULT_ODR
#define MOTION_SENSOR_GYRO_DEFAULT_ODR  104.0f
#endif

/** @brief Acceleration range applied by BSP_MOTION_SENSOR_Init, in g. */
#ifndef MOTION_SENSOR_ACC_DEFAULT_FS
#define MOTION_SENSOR_ACC_DEFAULT_FS    2
#endif

/** @brief Angular-rate range applied by BSP_MOTION_SENSOR_Init, in dps. */
#ifndef MOTION_SENSOR_GYRO_DEFAULT_FS
#define MOTION_SENSOR_GYRO_DEFAULT_FS   245
#endif

/** @brief Three-axis reading in mg, mdps or milligauss. */
typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} BSP_MOTION_SENSOR_Axes_t;

/** @brief Three-axis reading in counts. */
typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} BSP_MOTION_SENSOR_AxesRaw_t;

/** @brief Filled by BSP_MOTION_SENSOR_GetCapabilities. */
typedef struct
{
  uint8_t  Acc;
  uint8_t  Gyro;
  uint8_t  Magneto;
  uint8_t  LowPower;
  uint32_t GyroMaxFS;  /*!< dps.   */
  uint32_t AccMaxFS;   /*!< g.     */
  uint32_t MagMaxFS;   /*!< Gauss. */
  float    GyroMaxOdr; /*!< Hz.    */
  float    AccMaxOdr;  /*!< Hz.    */
  float    MagMaxOdr;  /*!< Hz.    */
} BSP_MOTION_SENSOR_Capabilities_t;

/** @brief Bring up an instance (bus, probe, reset, defaults) and enable a mask of
  *        MOTION_* functions; one the instance lacks is BSP_ERROR_WRONG_PARAM.
  * @note  The LSM6DSL is probed at both strap addresses.
  * @retval BSP_ERROR_UNKNOWN_COMPONENT if no part or bus answered,
  *         BSP_ERROR_COMPONENT_FAILURE if the part refused. A failure releases the
  *         instance this call brought up. */
int32_t BSP_MOTION_SENSOR_Init(uint32_t Instance, uint32_t Functions);

/** @brief Shut an instance down and release its share of the sensor bus. */
int32_t BSP_MOTION_SENSOR_DeInit(uint32_t Instance);

int32_t BSP_MOTION_SENSOR_GetCapabilities(uint32_t Instance,
                                          BSP_MOTION_SENSOR_Capabilities_t *pCapabilities);

int32_t BSP_MOTION_SENSOR_ReadID(uint32_t Instance, uint8_t *pId);

/** @brief Start one function: MOTION_ACCELERO, MOTION_GYRO or MOTION_MAGNETO. */
int32_t BSP_MOTION_SENSOR_Enable(uint32_t Instance, uint32_t Function);

/** @brief Stop one function. On the LSM6DSL this powers the sensor down; a running
  *        gyroscope draws several times what the accelerometer does. */
int32_t BSP_MOTION_SENSOR_Disable(uint32_t Instance, uint32_t Function);

/** @brief Read one function in mg, mdps or milligauss.
  * @note  On demand, a magnetometer read triggers a conversion and blocks a few ms. */
int32_t BSP_MOTION_SENSOR_GetAxes(uint32_t Instance, uint32_t Function,
                                  BSP_MOTION_SENSOR_Axes_t *pAxes);

/** @brief Read one function in counts; multiply by the sensitivity to convert. */
int32_t BSP_MOTION_SENSOR_GetAxesRaw(uint32_t Instance, uint32_t Function,
                                     BSP_MOTION_SENSOR_AxesRaw_t *pAxes);

/** @brief Units per count of one function, in the GetAxes unit. */
int32_t BSP_MOTION_SENSOR_GetSensitivity(uint32_t Instance, uint32_t Function,
                                         float *pSensitivity);

/** @brief Output rate in Hz; 0 if not measuring, or the magnetometer is on demand. */
int32_t BSP_MOTION_SENSOR_GetOutputDataRate(uint32_t Instance, uint32_t Function,
                                            float *pOdr);

/** @brief Set the output rate in Hz, rounded up to one the part offers; 0 stops the
  *        function. Read it back: the achieved rate is rarely the one requested. */
int32_t BSP_MOTION_SENSOR_SetOutputDataRate(uint32_t Instance, uint32_t Function,
                                            float Odr);

/** @brief Range: g for acceleration, dps for angular rate, gauss for field. */
int32_t BSP_MOTION_SENSOR_GetFullScale(uint32_t Instance, uint32_t Function,
                                       int32_t *pFullScale);

/** @brief Set the range, rounded up to one the part offers; wider coarsens readings. */
int32_t BSP_MOTION_SENSOR_SetFullScale(uint32_t Instance, uint32_t Function,
                                       int32_t FullScale);

/** @brief Die temperature in degrees C: the silicon, not the room.
  * @note  The LSM6DSL converts only while one of its sensors is enabled; a free-running
  *        MMC5603NJ pauses for about 50 ms. An extension to ST's motion-sensor API. */
int32_t BSP_MOTION_SENSOR_GetTemperature(uint32_t Instance, float *pTemp);

/** @brief Mask of MOTION_* functions with a fresh reading.
  * @note  The data-ready pins are not routed: this is the only way to pace a reader
  *        against the sensor. The LSM6DSL flags clear as the readings are read.
  * @note  The magnetometer is judged by the clock, not asked: a status read during its
  *        set/reset conversion zeros the outputs, and polling the outputs stops updates.
  *        On demand it is always ready. */
int32_t BSP_MOTION_SENSOR_GetDataReady(uint32_t Instance, uint32_t *pStatus);

/** @brief Clear the residual offset a strong field leaves in the magnetometer (enough
  *        to swamp the Earth's field). Instance: MOTION_SENSOR_MMC5603NJ_0 only.
  * @note  An extension: ST's API has no equivalent. */
int32_t BSP_MOTION_SENSOR_Degauss(uint32_t Instance);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_MOTION_SENSOR_H */
