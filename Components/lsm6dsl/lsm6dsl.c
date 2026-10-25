/**
  * @file    lsm6dsl.c
  * @brief   Component driver for the LSM6DSL six-axis inertial module.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  *
  * Register addresses, field layouts, scale factors and timings are taken
  * from the STMicroelectronics LSM6DSL datasheet (DocID027608).
  */

#include "lsm6dsl.h"

/* Rate (ODR) and range (FS) fields, shared by CTRL1_XL and CTRL2_G. */
#define LSM6DSL_ODR_SHIFT           4U
#define LSM6DSL_ODR_MASK            0xF0U
#define LSM6DSL_FS_SHIFT            2U
#define LSM6DSL_FS_MASK             0x0CU

/* The gyroscope's 125 dps range is a separate bit, not a range code. */
#define LSM6DSL_CTRL2_G_FS_125      0x02U

/* Software reset bound in ms; the datasheet quotes 50 us. */
#define LSM6DSL_RESET_TIMEOUT_MS    50U

/* Temperature output: 0 counts is 25 degrees C; 256 counts per degree. */
#define LSM6DSL_TEMP_ZERO_DEGC      25.0f
#define LSM6DSL_TEMP_LSB_PER_DEGC   256.0f

#define LSM6DSL_ODR_TABLE_SIZE      10U

/* Output rates in Hz for codes 1-10, both sensors. Code 0 is power-down; the
 * accelerometer's low-power 1.6 Hz is left out. */
static const float LSM6DSL_ODR_HZ[LSM6DSL_ODR_TABLE_SIZE] =
{
  12.5f, 26.0f, 52.0f, 104.0f, 208.0f, 416.0f, 833.0f, 1660.0f, 3330.0f, 6660.0f
};

/* Acceleration ranges in g by range code. Not ascending: code 01 is 16 g. */
static const int32_t LSM6DSL_ACC_FS_G[4] = { 2, 16, 4, 8 };

/* mg per count, by range code. */
static const float LSM6DSL_ACC_SENSITIVITY[4] = { 0.061f, 0.488f, 0.122f, 0.244f };

/* Gyroscope ranges in dps, by range code. */
static const int32_t LSM6DSL_GYRO_FS_DPS[4] = { 245, 500, 1000, 2000 };

/* mdps per count, by range code. */
static const float LSM6DSL_GYRO_SENSITIVITY[4] = { 8.75f, 17.50f, 35.0f, 70.0f };

/* The separate 125 dps gyroscope range and its mdps per count. */
#define LSM6DSL_GYRO_FS_125_DPS         125
#define LSM6DSL_GYRO_SENSITIVITY_125    4.375f

static int32_t LSM6DSL_Read(LSM6DSL_Object_t *pObj, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t LSM6DSL_Write(LSM6DSL_Object_t *pObj, uint8_t Reg, uint8_t Data);
static int32_t LSM6DSL_Modify(LSM6DSL_Object_t *pObj, uint8_t Reg, uint8_t Mask, uint8_t Value);
static uint8_t LSM6DSL_OdrToCode(float Odr, uint8_t LowestCode);
static int32_t LSM6DSL_ReadAxesRaw(LSM6DSL_Object_t *pObj, uint8_t Reg, LSM6DSL_AxesRaw_t *pAxes);

static int32_t LSM6DSL_Read(LSM6DSL_Object_t *pObj, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  if (pObj->IO.ReadReg(pObj->IO.Address, Reg, pData, Length) != 0)
  {
    return LSM6DSL_ERROR;
  }

  return LSM6DSL_OK;
}

static int32_t LSM6DSL_Write(LSM6DSL_Object_t *pObj, uint8_t Reg, uint8_t Data)
{
  uint8_t value = Data;

  if (pObj->IO.WriteReg(pObj->IO.Address, Reg, &value, 1U) != 0)
  {
    return LSM6DSL_ERROR;
  }

  return LSM6DSL_OK;
}

/* Replace the Mask bits of Reg with Value (already shifted). */
static int32_t LSM6DSL_Modify(LSM6DSL_Object_t *pObj, uint8_t Reg, uint8_t Mask, uint8_t Value)
{
  uint8_t current = 0U;

  if (LSM6DSL_Read(pObj, Reg, &current, 1U) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }

  current = (uint8_t)((current & (uint8_t)(~Mask)) | (Value & Mask));

  return LSM6DSL_Write(pObj, Reg, current);
}

/* Rate in Hz to code 0-10 (0 = power-down). Rounds up: a control loop degrades
 * if run slower than asked, while faster only costs current. */
static uint8_t LSM6DSL_OdrToCode(float Odr, uint8_t LowestCode)
{
  uint8_t code;

  if (Odr <= 0.0f)
  {
    return 0U;
  }

  for (code = LowestCode; code < LSM6DSL_ODR_TABLE_SIZE; code++)
  {
    if (Odr <= LSM6DSL_ODR_HZ[code - 1U])
    {
      return code;
    }
  }

  return LSM6DSL_ODR_TABLE_SIZE;
}

/* One 6-byte burst: with BDU and auto-increment, the three axes are one sample. */
static int32_t LSM6DSL_ReadAxesRaw(LSM6DSL_Object_t *pObj, uint8_t Reg, LSM6DSL_AxesRaw_t *pAxes)
{
  uint8_t buffer[6];

  if (LSM6DSL_Read(pObj, Reg, buffer, 6U) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }

  pAxes->x = (int16_t)(((uint16_t)buffer[1] << 8) | (uint16_t)buffer[0]);
  pAxes->y = (int16_t)(((uint16_t)buffer[3] << 8) | (uint16_t)buffer[2]);
  pAxes->z = (int16_t)(((uint16_t)buffer[5] << 8) | (uint16_t)buffer[4]);

  return LSM6DSL_OK;
}

int32_t LSM6DSL_RegisterBusIO(LSM6DSL_Object_t *pObj, const LSM6DSL_IO_t *pIO)
{
  uint8_t id = 0U;

  if ((pObj == NULL) || (pIO == NULL))
  {
    return LSM6DSL_ERROR;
  }

  pObj->IO = *pIO;

  if ((pObj->IO.ReadReg == NULL) || (pObj->IO.WriteReg == NULL))
  {
    return LSM6DSL_ERROR;
  }

  if (pObj->IO.Init != NULL)
  {
    if (pObj->IO.Init() != 0)
    {
      return LSM6DSL_ERROR;
    }
  }

  /* Not this part, or none: give the bus back before a retry at the other address. */
  if ((LSM6DSL_ReadID(pObj, &id) != LSM6DSL_OK) || (id != LSM6DSL_ID))
  {
    if (pObj->IO.DeInit != NULL)
    {
      (void)pObj->IO.DeInit();
    }

    return LSM6DSL_ERROR;
  }

  return LSM6DSL_OK;
}

int32_t LSM6DSL_Init(LSM6DSL_Object_t *pObj)
{
  uint8_t control = 0U;
  int32_t start;

  if (pObj == NULL)
  {
    return LSM6DSL_ERROR;
  }

  if (LSM6DSL_Write(pObj, LSM6DSL_CTRL3_C, LSM6DSL_CTRL3_C_SW_RESET) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }

  /* The reset bit clears itself when the part is ready. */
  start = (pObj->IO.GetTick != NULL) ? pObj->IO.GetTick() : 0;

  for (;;)
  {
    if (LSM6DSL_Read(pObj, LSM6DSL_CTRL3_C, &control, 1U) != LSM6DSL_OK)
    {
      return LSM6DSL_ERROR;
    }

    if ((control & LSM6DSL_CTRL3_C_SW_RESET) == 0U)
    {
      break;
    }

    if (pObj->IO.GetTick == NULL)
    {
      /* No time base: one read is all that can be justified. */
      return LSM6DSL_ERROR;
    }

    if (((uint32_t)pObj->IO.GetTick() - (uint32_t)start) > LSM6DSL_RESET_TIMEOUT_MS)
    {
      return LSM6DSL_ERROR;
    }
  }

  /* BDU and auto-increment: without both, a six-byte read can splice two samples. */
  if (LSM6DSL_Write(pObj, LSM6DSL_CTRL3_C,
                    LSM6DSL_CTRL3_C_BDU | LSM6DSL_CTRL3_C_IF_INC) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }

  /* Both sensors powered down; reset leaves each at its narrowest range. */
  pObj->AccEnabled  = 0U;
  pObj->GyroEnabled = 0U;
  pObj->AccOdr      = 104.0f;
  pObj->GyroOdr     = 104.0f;
  pObj->IsInitialized = 1U;

  return LSM6DSL_OK;
}

int32_t LSM6DSL_DeInit(LSM6DSL_Object_t *pObj)
{
  if (pObj == NULL)
  {
    return LSM6DSL_ERROR;
  }

  if (pObj->IsInitialized != 0U)
  {
    (void)LSM6DSL_ACC_Disable(pObj);
    (void)LSM6DSL_GYRO_Disable(pObj);
  }

  if (pObj->IO.DeInit != NULL)
  {
    (void)pObj->IO.DeInit();
  }

  pObj->IsInitialized = 0U;

  return LSM6DSL_OK;
}

int32_t LSM6DSL_ReadID(LSM6DSL_Object_t *pObj, uint8_t *pId)
{
  if ((pObj == NULL) || (pId == NULL))
  {
    return LSM6DSL_ERROR;
  }

  return LSM6DSL_Read(pObj, LSM6DSL_WHO_AM_I, pId, 1U);
}

int32_t LSM6DSL_GetCapabilities(LSM6DSL_Object_t *pObj, LSM6DSL_Capabilities_t *pCapabilities)
{
  (void)pObj;

  if (pCapabilities == NULL)
  {
    return LSM6DSL_ERROR;
  }

  pCapabilities->Acc        = 1U;
  pCapabilities->Gyro       = 1U;
  pCapabilities->Magneto    = 0U;
  pCapabilities->LowPower   = 0U;
  pCapabilities->AccMaxFS   = 16U;
  pCapabilities->GyroMaxFS  = 2000U;
  pCapabilities->MagMaxFS   = 0U;
  pCapabilities->AccMaxOdr  = 6660.0f;
  pCapabilities->GyroMaxOdr = 6660.0f;
  pCapabilities->MagMaxOdr  = 0.0f;

  return LSM6DSL_OK;
}

int32_t LSM6DSL_ACC_Enable(LSM6DSL_Object_t *pObj)
{
  uint8_t code;

  if (pObj == NULL)
  {
    return LSM6DSL_ERROR;
  }

  if (pObj->AccEnabled != 0U)
  {
    return LSM6DSL_OK;
  }

  code = LSM6DSL_OdrToCode(pObj->AccOdr, 1U);

  if (LSM6DSL_Modify(pObj, LSM6DSL_CTRL1_XL, LSM6DSL_ODR_MASK,
                     (uint8_t)(code << LSM6DSL_ODR_SHIFT)) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }

  pObj->AccEnabled = 1U;

  return LSM6DSL_OK;
}

int32_t LSM6DSL_ACC_Disable(LSM6DSL_Object_t *pObj)
{
  if (pObj == NULL)
  {
    return LSM6DSL_ERROR;
  }

  if (LSM6DSL_Modify(pObj, LSM6DSL_CTRL1_XL, LSM6DSL_ODR_MASK, 0U) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }

  pObj->AccEnabled = 0U;

  return LSM6DSL_OK;
}

int32_t LSM6DSL_ACC_GetAxes(LSM6DSL_Object_t *pObj, LSM6DSL_Axes_t *pAxes)
{
  LSM6DSL_AxesRaw_t raw;
  float sensitivity = 0.0f;

  if ((pObj == NULL) || (pAxes == NULL))
  {
    return LSM6DSL_ERROR;
  }

  if (LSM6DSL_ACC_GetAxesRaw(pObj, &raw) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }

  if (LSM6DSL_ACC_GetSensitivity(pObj, &sensitivity) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }

  pAxes->x = (int32_t)((float)raw.x * sensitivity);
  pAxes->y = (int32_t)((float)raw.y * sensitivity);
  pAxes->z = (int32_t)((float)raw.z * sensitivity);

  return LSM6DSL_OK;
}

int32_t LSM6DSL_ACC_GetAxesRaw(LSM6DSL_Object_t *pObj, LSM6DSL_AxesRaw_t *pAxes)
{
  if ((pObj == NULL) || (pAxes == NULL))
  {
    return LSM6DSL_ERROR;
  }

  return LSM6DSL_ReadAxesRaw(pObj, LSM6DSL_OUTX_L_XL, pAxes);
}

int32_t LSM6DSL_ACC_GetSensitivity(LSM6DSL_Object_t *pObj, float *pSensitivity)
{
  uint8_t control = 0U;

  if ((pObj == NULL) || (pSensitivity == NULL))
  {
    return LSM6DSL_ERROR;
  }

  if (LSM6DSL_Read(pObj, LSM6DSL_CTRL1_XL, &control, 1U) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }

  *pSensitivity = LSM6DSL_ACC_SENSITIVITY[(control & LSM6DSL_FS_MASK) >> LSM6DSL_FS_SHIFT];

  return LSM6DSL_OK;
}

int32_t LSM6DSL_ACC_GetOutputDataRate(LSM6DSL_Object_t *pObj, float *pOdr)
{
  uint8_t control = 0U;
  uint8_t code;

  if ((pObj == NULL) || (pOdr == NULL))
  {
    return LSM6DSL_ERROR;
  }

  if (LSM6DSL_Read(pObj, LSM6DSL_CTRL1_XL, &control, 1U) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }

  code = (uint8_t)((control & LSM6DSL_ODR_MASK) >> LSM6DSL_ODR_SHIFT);
  *pOdr = (code == 0U) ? 0.0f : LSM6DSL_ODR_HZ[code - 1U];

  return LSM6DSL_OK;
}

int32_t LSM6DSL_ACC_SetOutputDataRate(LSM6DSL_Object_t *pObj, float Odr)
{
  uint8_t code;

  if (pObj == NULL)
  {
    return LSM6DSL_ERROR;
  }

  code = LSM6DSL_OdrToCode(Odr, 1U);

  if (code == 0U)
  {
    return LSM6DSL_ACC_Disable(pObj);
  }

  pObj->AccOdr = LSM6DSL_ODR_HZ[code - 1U];

  /* Disabled: store the rate only, as writing it would power the sensor up. */
  if (pObj->AccEnabled == 0U)
  {
    return LSM6DSL_OK;
  }

  return LSM6DSL_Modify(pObj, LSM6DSL_CTRL1_XL, LSM6DSL_ODR_MASK,
                        (uint8_t)(code << LSM6DSL_ODR_SHIFT));
}

int32_t LSM6DSL_ACC_GetFullScale(LSM6DSL_Object_t *pObj, int32_t *pFullScale)
{
  uint8_t control = 0U;

  if ((pObj == NULL) || (pFullScale == NULL))
  {
    return LSM6DSL_ERROR;
  }

  if (LSM6DSL_Read(pObj, LSM6DSL_CTRL1_XL, &control, 1U) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }

  *pFullScale = LSM6DSL_ACC_FS_G[(control & LSM6DSL_FS_MASK) >> LSM6DSL_FS_SHIFT];

  return LSM6DSL_OK;
}

int32_t LSM6DSL_ACC_SetFullScale(LSM6DSL_Object_t *pObj, int32_t FullScale)
{
  uint8_t code;

  if (pObj == NULL)
  {
    return LSM6DSL_ERROR;
  }

  /* Round up, so the requested acceleration is never clipped. */
  if (FullScale <= 2)
  {
    code = 0U; /* 2 g  */
  }
  else if (FullScale <= 4)
  {
    code = 2U; /* 4 g  */
  }
  else if (FullScale <= 8)
  {
    code = 3U; /* 8 g  */
  }
  else
  {
    code = 1U; /* 16 g */
  }

  return LSM6DSL_Modify(pObj, LSM6DSL_CTRL1_XL, LSM6DSL_FS_MASK,
                        (uint8_t)(code << LSM6DSL_FS_SHIFT));
}

int32_t LSM6DSL_GYRO_Enable(LSM6DSL_Object_t *pObj)
{
  uint8_t code;

  if (pObj == NULL)
  {
    return LSM6DSL_ERROR;
  }

  if (pObj->GyroEnabled != 0U)
  {
    return LSM6DSL_OK;
  }

  code = LSM6DSL_OdrToCode(pObj->GyroOdr, 1U);

  if (LSM6DSL_Modify(pObj, LSM6DSL_CTRL2_G, LSM6DSL_ODR_MASK,
                     (uint8_t)(code << LSM6DSL_ODR_SHIFT)) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }

  pObj->GyroEnabled = 1U;

  return LSM6DSL_OK;
}

int32_t LSM6DSL_GYRO_Disable(LSM6DSL_Object_t *pObj)
{
  if (pObj == NULL)
  {
    return LSM6DSL_ERROR;
  }

  if (LSM6DSL_Modify(pObj, LSM6DSL_CTRL2_G, LSM6DSL_ODR_MASK, 0U) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }

  pObj->GyroEnabled = 0U;

  return LSM6DSL_OK;
}

int32_t LSM6DSL_GYRO_GetAxes(LSM6DSL_Object_t *pObj, LSM6DSL_Axes_t *pAxes)
{
  LSM6DSL_AxesRaw_t raw;
  float sensitivity = 0.0f;

  if ((pObj == NULL) || (pAxes == NULL))
  {
    return LSM6DSL_ERROR;
  }

  if (LSM6DSL_GYRO_GetAxesRaw(pObj, &raw) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }

  if (LSM6DSL_GYRO_GetSensitivity(pObj, &sensitivity) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }

  pAxes->x = (int32_t)((float)raw.x * sensitivity);
  pAxes->y = (int32_t)((float)raw.y * sensitivity);
  pAxes->z = (int32_t)((float)raw.z * sensitivity);

  return LSM6DSL_OK;
}

int32_t LSM6DSL_GYRO_GetAxesRaw(LSM6DSL_Object_t *pObj, LSM6DSL_AxesRaw_t *pAxes)
{
  if ((pObj == NULL) || (pAxes == NULL))
  {
    return LSM6DSL_ERROR;
  }

  return LSM6DSL_ReadAxesRaw(pObj, LSM6DSL_OUTX_L_G, pAxes);
}

int32_t LSM6DSL_GYRO_GetSensitivity(LSM6DSL_Object_t *pObj, float *pSensitivity)
{
  uint8_t control = 0U;

  if ((pObj == NULL) || (pSensitivity == NULL))
  {
    return LSM6DSL_ERROR;
  }

  if (LSM6DSL_Read(pObj, LSM6DSL_CTRL2_G, &control, 1U) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }

  /* The narrow range is a bit of its own that overrides the range field. */
  if ((control & LSM6DSL_CTRL2_G_FS_125) != 0U)
  {
    *pSensitivity = LSM6DSL_GYRO_SENSITIVITY_125;
  }
  else
  {
    *pSensitivity = LSM6DSL_GYRO_SENSITIVITY[(control & LSM6DSL_FS_MASK) >> LSM6DSL_FS_SHIFT];
  }

  return LSM6DSL_OK;
}

int32_t LSM6DSL_GYRO_GetOutputDataRate(LSM6DSL_Object_t *pObj, float *pOdr)
{
  uint8_t control = 0U;
  uint8_t code;

  if ((pObj == NULL) || (pOdr == NULL))
  {
    return LSM6DSL_ERROR;
  }

  if (LSM6DSL_Read(pObj, LSM6DSL_CTRL2_G, &control, 1U) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }

  code = (uint8_t)((control & LSM6DSL_ODR_MASK) >> LSM6DSL_ODR_SHIFT);
  *pOdr = (code == 0U) ? 0.0f : LSM6DSL_ODR_HZ[code - 1U];

  return LSM6DSL_OK;
}

int32_t LSM6DSL_GYRO_SetOutputDataRate(LSM6DSL_Object_t *pObj, float Odr)
{
  uint8_t code;

  if (pObj == NULL)
  {
    return LSM6DSL_ERROR;
  }

  code = LSM6DSL_OdrToCode(Odr, 1U);

  if (code == 0U)
  {
    return LSM6DSL_GYRO_Disable(pObj);
  }

  pObj->GyroOdr = LSM6DSL_ODR_HZ[code - 1U];

  if (pObj->GyroEnabled == 0U)
  {
    return LSM6DSL_OK;
  }

  return LSM6DSL_Modify(pObj, LSM6DSL_CTRL2_G, LSM6DSL_ODR_MASK,
                        (uint8_t)(code << LSM6DSL_ODR_SHIFT));
}

int32_t LSM6DSL_GYRO_GetFullScale(LSM6DSL_Object_t *pObj, int32_t *pFullScale)
{
  uint8_t control = 0U;

  if ((pObj == NULL) || (pFullScale == NULL))
  {
    return LSM6DSL_ERROR;
  }

  if (LSM6DSL_Read(pObj, LSM6DSL_CTRL2_G, &control, 1U) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }

  if ((control & LSM6DSL_CTRL2_G_FS_125) != 0U)
  {
    *pFullScale = LSM6DSL_GYRO_FS_125_DPS;
  }
  else
  {
    *pFullScale = LSM6DSL_GYRO_FS_DPS[(control & LSM6DSL_FS_MASK) >> LSM6DSL_FS_SHIFT];
  }

  return LSM6DSL_OK;
}

int32_t LSM6DSL_GYRO_SetFullScale(LSM6DSL_Object_t *pObj, int32_t FullScale)
{
  uint8_t value;

  if (pObj == NULL)
  {
    return LSM6DSL_ERROR;
  }

  if (FullScale <= LSM6DSL_GYRO_FS_125_DPS)
  {
    /* The narrow range needs its own bit set and the range field cleared. */
    value = LSM6DSL_CTRL2_G_FS_125;
  }
  else if (FullScale <= 245)
  {
    value = 0U;
  }
  else if (FullScale <= 500)
  {
    value = (uint8_t)(1U << LSM6DSL_FS_SHIFT);
  }
  else if (FullScale <= 1000)
  {
    value = (uint8_t)(2U << LSM6DSL_FS_SHIFT);
  }
  else
  {
    value = (uint8_t)(3U << LSM6DSL_FS_SHIFT);
  }

  return LSM6DSL_Modify(pObj, LSM6DSL_CTRL2_G,
                        LSM6DSL_FS_MASK | LSM6DSL_CTRL2_G_FS_125, value);
}

int32_t LSM6DSL_GetTemperature(LSM6DSL_Object_t *pObj, float *pTemp)
{
  uint8_t buffer[2];
  int16_t raw;

  if ((pObj == NULL) || (pTemp == NULL))
  {
    return LSM6DSL_ERROR;
  }

  if (LSM6DSL_Read(pObj, LSM6DSL_OUT_TEMP_L, buffer, 2U) != LSM6DSL_OK)
  {
    return LSM6DSL_ERROR;
  }

  raw = (int16_t)(((uint16_t)buffer[1] << 8) | (uint16_t)buffer[0]);

  *pTemp = LSM6DSL_TEMP_ZERO_DEGC + ((float)raw / LSM6DSL_TEMP_LSB_PER_DEGC);

  return LSM6DSL_OK;
}

int32_t LSM6DSL_GetStatus(LSM6DSL_Object_t *pObj, uint8_t *pStatus)
{
  if ((pObj == NULL) || (pStatus == NULL))
  {
    return LSM6DSL_ERROR;
  }

  return LSM6DSL_Read(pObj, LSM6DSL_STATUS_REG, pStatus, 1U);
}

int32_t LSM6DSL_ReadReg(LSM6DSL_Object_t *pObj, uint8_t Reg, uint8_t *pData)
{
  if ((pObj == NULL) || (pData == NULL))
  {
    return LSM6DSL_ERROR;
  }

  return LSM6DSL_Read(pObj, Reg, pData, 1U);
}

int32_t LSM6DSL_WriteReg(LSM6DSL_Object_t *pObj, uint8_t Reg, uint8_t Data)
{
  if (pObj == NULL)
  {
    return LSM6DSL_ERROR;
  }

  return LSM6DSL_Write(pObj, Reg, Data);
}
