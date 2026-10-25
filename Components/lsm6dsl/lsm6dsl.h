/**
  * @file    lsm6dsl.h
  * @brief   Component driver for the LSM6DSL six-axis inertial module.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  *
  * Register addresses, field layouts, scale factors and timings are taken
  * from the STMicroelectronics LSM6DSL datasheet (DocID027608). This is an
  * original implementation written against that document; it is not derived
  * from any other driver.
  */

#ifndef LSM6DSL_H
#define LSM6DSL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 8-bit I2C address with the address-select pin low or high. */
#define LSM6DSL_I2C_ADDRESS_LOW         0xD4U
#define LSM6DSL_I2C_ADDRESS_HIGH        0xD6U

/** @brief WHO_AM_I value. */
#define LSM6DSL_ID                      0x6AU

#define LSM6DSL_WHO_AM_I                0x0FU /*!< Identity.                    */
#define LSM6DSL_CTRL1_XL                0x10U /*!< Accelerometer rate and range. */
#define LSM6DSL_CTRL2_G                 0x11U /*!< Gyroscope rate and range.     */
#define LSM6DSL_CTRL3_C                 0x12U /*!< Common control.               */
#define LSM6DSL_STATUS_REG              0x1EU /*!< New-data flags.               */
#define LSM6DSL_OUT_TEMP_L              0x20U /*!< Temperature, two bytes.       */
#define LSM6DSL_OUTX_L_G                0x22U /*!< Angular rate, six bytes.      */
#define LSM6DSL_OUTX_L_XL               0x28U /*!< Acceleration, six bytes.      */

#define LSM6DSL_CTRL3_C_BOOT            0x80U /*!< Reload the trimming memory.  */
#define LSM6DSL_CTRL3_C_BDU             0x40U /*!< Block data update.           */
#define LSM6DSL_CTRL3_C_IF_INC          0x04U /*!< Register auto-increment.     */
#define LSM6DSL_CTRL3_C_SW_RESET        0x01U /*!< Restore the reset state.     */

#define LSM6DSL_STATUS_XLDA             0x01U /*!< Acceleration sample ready. */
#define LSM6DSL_STATUS_GDA              0x02U /*!< Angular-rate sample ready. */
#define LSM6DSL_STATUS_TDA              0x04U /*!< Temperature sample ready.  */

#define LSM6DSL_OK                      0
#define LSM6DSL_ERROR                   -1

/** @brief Bus type reported to the driver by the board layer. */
#define LSM6DSL_I2C_BUS                 0x00U

typedef int32_t (*LSM6DSL_Init_Func)(void);
typedef int32_t (*LSM6DSL_DeInit_Func)(void);
typedef int32_t (*LSM6DSL_GetTick_Func)(void);
typedef void    (*LSM6DSL_Delay_Func)(uint32_t);
typedef int32_t (*LSM6DSL_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*LSM6DSL_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

/** @brief Bus access supplied by the board. */
typedef struct
{
  LSM6DSL_Init_Func     Init;     /*!< Bring the bus up.                    */
  LSM6DSL_DeInit_Func   DeInit;   /*!< Release the bus.                     */
  uint32_t              BusType;  /*!< LSM6DSL_I2C_BUS.                     */
  uint8_t               Address;  /*!< Device address, 8-bit form.          */
  LSM6DSL_WriteReg_Func WriteReg;
  LSM6DSL_ReadReg_Func  ReadReg;
  LSM6DSL_GetTick_Func  GetTick;  /*!< Millisecond tick, for timeouts.      */
  LSM6DSL_Delay_Func    Delay;    /*!< Millisecond delay, for reset waits.  */
} LSM6DSL_IO_t;

/** @brief Three-axis sample in mg or mdps. */
typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} LSM6DSL_Axes_t;

/** @brief Three-axis sample in raw counts. */
typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} LSM6DSL_AxesRaw_t;

/** @brief Filled by LSM6DSL_GetCapabilities. */
typedef struct
{
  uint8_t Acc;
  uint8_t Gyro;
  uint8_t Magneto;
  uint8_t LowPower;
  uint32_t GyroMaxFS; /*!< dps. */
  uint32_t AccMaxFS;  /*!< g.   */
  uint32_t MagMaxFS;
  float    GyroMaxOdr;/*!< Hz.  */
  float    AccMaxOdr; /*!< Hz.  */
  float    MagMaxOdr;
} LSM6DSL_Capabilities_t;

/** @brief Driver state. One per fitted part; the board layer owns it. */
typedef struct
{
  LSM6DSL_IO_t IO;
  uint8_t      IsInitialized;  /*!< 1 once LSM6DSL_Init has succeeded.     */
  uint8_t      AccEnabled;
  uint8_t      GyroEnabled;
  float        AccOdr;         /*!< Rate to restore when re-enabled, Hz.   */
  float        GyroOdr;        /*!< Rate to restore when re-enabled, Hz.   */
} LSM6DSL_Object_t;

/** @brief Copy pIO into pObj and check WHO_AM_I. */
int32_t LSM6DSL_RegisterBusIO(LSM6DSL_Object_t *pObj, const LSM6DSL_IO_t *pIO);

/** @brief Reset, enable BDU and auto-increment; leaves both sensors powered down. */
int32_t LSM6DSL_Init(LSM6DSL_Object_t *pObj);

/** @brief Power both sensors down and release the bus. */
int32_t LSM6DSL_DeInit(LSM6DSL_Object_t *pObj);

/** @brief Read WHO_AM_I (LSM6DSL_ID on a working part). */
int32_t LSM6DSL_ReadID(LSM6DSL_Object_t *pObj, uint8_t *pId);

int32_t LSM6DSL_GetCapabilities(LSM6DSL_Object_t *pObj, LSM6DSL_Capabilities_t *pCapabilities);

/** @brief Start the accelerometer at its stored output rate. */
int32_t LSM6DSL_ACC_Enable(LSM6DSL_Object_t *pObj);

/** @brief Power the accelerometer down; its output rate is kept. */
int32_t LSM6DSL_ACC_Disable(LSM6DSL_Object_t *pObj);

/** @brief Acceleration in mg. */
int32_t LSM6DSL_ACC_GetAxes(LSM6DSL_Object_t *pObj, LSM6DSL_Axes_t *pAxes);

int32_t LSM6DSL_ACC_GetAxesRaw(LSM6DSL_Object_t *pObj, LSM6DSL_AxesRaw_t *pAxes);

/** @brief mg per count at the current range. */
int32_t LSM6DSL_ACC_GetSensitivity(LSM6DSL_Object_t *pObj, float *pSensitivity);

/** @brief Output rate in Hz, 0 when powered down. */
int32_t LSM6DSL_ACC_GetOutputDataRate(LSM6DSL_Object_t *pObj, float *pOdr);

/** @brief Odr in Hz, rounded up to a rate the part offers (clamped to the fastest).
  * @note  While disabled, the rate is stored and applied at the next enable. */
int32_t LSM6DSL_ACC_SetOutputDataRate(LSM6DSL_Object_t *pObj, float Odr);

/** @brief Range in g: 2, 4, 8 or 16. */
int32_t LSM6DSL_ACC_GetFullScale(LSM6DSL_Object_t *pObj, int32_t *pFullScale);

/** @brief FullScale in g, rounded up to 2, 4, 8 or 16. */
int32_t LSM6DSL_ACC_SetFullScale(LSM6DSL_Object_t *pObj, int32_t FullScale);

/** @brief Start the gyroscope at its stored output rate. */
int32_t LSM6DSL_GYRO_Enable(LSM6DSL_Object_t *pObj);

/** @brief Power the gyroscope down; its output rate is kept. */
int32_t LSM6DSL_GYRO_Disable(LSM6DSL_Object_t *pObj);

/** @brief Angular rate in mdps. */
int32_t LSM6DSL_GYRO_GetAxes(LSM6DSL_Object_t *pObj, LSM6DSL_Axes_t *pAxes);

int32_t LSM6DSL_GYRO_GetAxesRaw(LSM6DSL_Object_t *pObj, LSM6DSL_AxesRaw_t *pAxes);

/** @brief mdps per count at the current range. */
int32_t LSM6DSL_GYRO_GetSensitivity(LSM6DSL_Object_t *pObj, float *pSensitivity);

/** @brief Output rate in Hz, 0 when powered down. */
int32_t LSM6DSL_GYRO_GetOutputDataRate(LSM6DSL_Object_t *pObj, float *pOdr);

/** @brief As LSM6DSL_ACC_SetOutputDataRate, for the gyroscope. */
int32_t LSM6DSL_GYRO_SetOutputDataRate(LSM6DSL_Object_t *pObj, float Odr);

/** @brief Range in dps: 125, 245, 500, 1000 or 2000. */
int32_t LSM6DSL_GYRO_GetFullScale(LSM6DSL_Object_t *pObj, int32_t *pFullScale);

/** @brief FullScale in dps, rounded up to a range the part offers. */
int32_t LSM6DSL_GYRO_SetFullScale(LSM6DSL_Object_t *pObj, int32_t FullScale);

/** @brief Die temperature in degrees C (above ambient; for sensor compensation).
  * @note  Converts only while the accelerometer or gyroscope is enabled. */
int32_t LSM6DSL_GetTemperature(LSM6DSL_Object_t *pObj, float *pTemp);

/** @brief Read the LSM6DSL_STATUS_* new-data flags. */
int32_t LSM6DSL_GetStatus(LSM6DSL_Object_t *pObj, uint8_t *pStatus);

int32_t LSM6DSL_ReadReg(LSM6DSL_Object_t *pObj, uint8_t Reg, uint8_t *pData);

int32_t LSM6DSL_WriteReg(LSM6DSL_Object_t *pObj, uint8_t Reg, uint8_t Data);

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSL_H */
