/**
  * @file    vl53l1cb.c
  * @brief   VL53L1CB time-of-flight driver, using the ranging the part does itself.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  *
  * The register addresses and the start-up sequence are those of
  * STMicroelectronics' VL53L1X ultra lite driver (STSW-IMG009, BSD-3-Clause).
  * Its default configuration, the only content taken from it, is in
  * vl53l1cb_st_tables.h with ST's notice.
  */

#include "vl53l1cb.h"
#include "vl53l1cb_st_tables.h"

/* Registers: 16-bit addresses, multi-byte values most significant byte first. */
#define VL53L1CB_SOFT_RESET              0x0000U
#define VL53L1CB_VHV_LOOP_BOUND          0x0008U  /* VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND */
#define VL53L1CB_VHV_INIT                0x000BU  /* VHV_CONFIG__INIT */
#define VL53L1CB_GPIO_HV_MUX_CTRL        0x0030U
#define VL53L1CB_GPIO_TIO_HV_STATUS      0x0031U
#define VL53L1CB_SIGMA_THRESH            0x0064U  /* RANGE_CONFIG__SIGMA_THRESH, mm in 14.2 */
#define VL53L1CB_MIN_COUNT_RATE          0x0066U  /* RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 9.7 */
#define VL53L1CB_ROI_CENTRE_SPAD         0x007FU  /* ROI_CONFIG__USER_ROI_CENTRE_SPAD */
#define VL53L1CB_ROI_XY_SIZE             0x0080U  /* ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE */
#define VL53L1CB_INTERRUPT_CLEAR         0x0086U  /* SYSTEM__INTERRUPT_CLEAR */
#define VL53L1CB_MODE_START              0x0087U  /* SYSTEM__MODE_START */
#define VL53L1CB_RANGE_STATUS            0x0089U  /* RESULT__RANGE_STATUS */
#define VL53L1CB_RANGE_MM                0x0096U  /* RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 */
#define VL53L1CB_FIRMWARE_STATUS         0x00E5U  /* FIRMWARE__SYSTEM_STATUS */
#define VL53L1CB_MODEL_ID                0x010FU  /* IDENTIFICATION__MODEL_ID, then MODULE_TYPE */

#define VL53L1CB_MODE_STOP               0x00U
#define VL53L1CB_MODE_TIMED              0x40U

/* GPIO_HV_MUX_CTRL bit 4 set: the interrupt is active low. */
#define VL53L1CB_INT_ACTIVE_LOW          0x10U

/* How long the part may take: to boot after a reset, and to finish the first,
 * calibrating, measurement (three timing budgets). */
#define VL53L1CB_BOOT_TIMEOUT_MS         500U
#define VL53L1CB_FIRST_RANGE_TIMEOUT_MS  (3U * VL53L1CB_TIMING_BUDGET_MS)

static int32_t VL53L1CB_Read(VL53L1CB_Object_t *pObj, uint16_t Reg, uint8_t *pData,
                             uint16_t Length);
static int32_t VL53L1CB_Write(VL53L1CB_Object_t *pObj, uint16_t Reg, const uint8_t *pData,
                              uint16_t Length);
static int32_t VL53L1CB_Write8(VL53L1CB_Object_t *pObj, uint16_t Reg, uint8_t Value);
static int32_t VL53L1CB_Write16(VL53L1CB_Object_t *pObj, uint16_t Reg, uint16_t Value);
static void    VL53L1CB_Wait(VL53L1CB_Object_t *pObj, uint32_t Milliseconds);
static int32_t VL53L1CB_WaitReady(VL53L1CB_Object_t *pObj, uint32_t TimeoutMs);

static int32_t VL53L1CB_Read(VL53L1CB_Object_t *pObj, uint16_t Reg, uint8_t *pData,
                             uint16_t Length)
{
  if (pObj->IO.ReadReg(pObj->IO.Address, Reg, pData, Length) != 0)
  {
    return VL53L1CB_ERROR;
  }
  return VL53L1CB_OK;
}

static int32_t VL53L1CB_Write(VL53L1CB_Object_t *pObj, uint16_t Reg, const uint8_t *pData,
                              uint16_t Length)
{
  if (pObj->IO.WriteReg(pObj->IO.Address, Reg, (uint8_t *)pData, Length) != 0)
  {
    return VL53L1CB_ERROR;
  }
  return VL53L1CB_OK;
}

static int32_t VL53L1CB_Write8(VL53L1CB_Object_t *pObj, uint16_t Reg, uint8_t Value)
{
  return VL53L1CB_Write(pObj, Reg, &Value, 1U);
}

static int32_t VL53L1CB_Write16(VL53L1CB_Object_t *pObj, uint16_t Reg, uint16_t Value)
{
  const uint8_t data[2] = { (uint8_t)(Value >> 8), (uint8_t)(Value & 0xFFU) };

  return VL53L1CB_Write(pObj, Reg, data, 2U);
}

static void VL53L1CB_Wait(VL53L1CB_Object_t *pObj, uint32_t Milliseconds)
{
  if (pObj->IO.Delay != NULL)
  {
    pObj->IO.Delay(Milliseconds);
  }
}

/* 1 once a result is waiting, VL53L1CB_TIMEOUT if none came in time,
 * VL53L1CB_ERROR if the part stopped answering. */
static int32_t VL53L1CB_WaitReady(VL53L1CB_Object_t *pObj, uint32_t TimeoutMs)
{
  const uint32_t start = (uint32_t)pObj->IO.GetTick();
  uint8_t        ready = 0U;

  for (;;)
  {
    if (VL53L1CB_DataReady(pObj, &ready) != VL53L1CB_OK)
    {
      return VL53L1CB_ERROR;
    }
    if (ready != 0U)
    {
      return 1;
    }
    if (((uint32_t)pObj->IO.GetTick() - start) >= TimeoutMs)
    {
      return VL53L1CB_TIMEOUT;
    }
    VL53L1CB_Wait(pObj, 1U);
  }
}

int32_t VL53L1CB_RegisterBusIO(VL53L1CB_Object_t *pObj, const VL53L1CB_IO_t *pIO)
{
  if ((pObj == NULL) || (pIO == NULL) || (pIO->ReadReg == NULL) || (pIO->WriteReg == NULL) ||
      (pIO->GetTick == NULL))
  {
    return VL53L1CB_ERROR;
  }
  pObj->IO            = *pIO;
  pObj->IsInitialized = 0U;
  pObj->IsRanging     = 0U;
  if ((pObj->IO.Init != NULL) && (pObj->IO.Init() != 0))
  {
    return VL53L1CB_ERROR;
  }
  return VL53L1CB_OK;
}

int32_t VL53L1CB_ReadID(VL53L1CB_Object_t *pObj, uint16_t *pId)
{
  uint8_t id[2];

  if ((pObj == NULL) || (pId == NULL) ||
      (VL53L1CB_Read(pObj, VL53L1CB_MODEL_ID, id, 2U) != VL53L1CB_OK))
  {
    return VL53L1CB_ERROR;
  }
  *pId = (uint16_t)(((uint16_t)id[0] << 8) | id[1]);
  return VL53L1CB_OK;
}

int32_t VL53L1CB_Init(VL53L1CB_Object_t *pObj)
{
  uint16_t id = 0U;
  uint8_t  fw = 0U;
  uint32_t start;
  int32_t  ready;

  if ((pObj == NULL) || (VL53L1CB_ReadID(pObj, &id) != VL53L1CB_OK) || (id != VL53L1CB_ID))
  {
    return VL53L1CB_ERROR;
  }
  pObj->IsInitialized = 0U;
  pObj->IsRanging     = 0U;

  /* Software reset: held, released, then the firmware's boot flag. */
  if ((VL53L1CB_Write8(pObj, VL53L1CB_SOFT_RESET, 0x00U) != VL53L1CB_OK))
  {
    return VL53L1CB_ERROR;
  }
  VL53L1CB_Wait(pObj, 2U);
  if ((VL53L1CB_Write8(pObj, VL53L1CB_SOFT_RESET, 0x01U) != VL53L1CB_OK))
  {
    return VL53L1CB_ERROR;
  }
  VL53L1CB_Wait(pObj, 2U);
  start = (uint32_t)pObj->IO.GetTick();
  while ((VL53L1CB_Read(pObj, VL53L1CB_FIRMWARE_STATUS, &fw, 1U) != VL53L1CB_OK) ||
         ((fw & 0x01U) == 0U))
  {
    if (((uint32_t)pObj->IO.GetTick() - start) >= VL53L1CB_BOOT_TIMEOUT_MS)
    {
      return VL53L1CB_TIMEOUT;
    }
    VL53L1CB_Wait(pObj, 2U);
  }

  /* The default configuration, registers 0x2D to 0x87 in one write. */
  if (VL53L1CB_Write(pObj, VL53L1_ST_CONFIG_FIRST_REG, vl53l1_st_default_config,
                     (uint16_t)sizeof(vl53l1_st_default_config)) != VL53L1CB_OK)
  {
    return VL53L1CB_ERROR;
  }

  /* One timed measurement calibrates VHV; later ones start from that value
   * rather than searching again. */
  if ((VL53L1CB_Write8(pObj, VL53L1CB_INTERRUPT_CLEAR, 0x01U) != VL53L1CB_OK) ||
      (VL53L1CB_Write8(pObj, VL53L1CB_MODE_START, VL53L1CB_MODE_TIMED) != VL53L1CB_OK))
  {
    return VL53L1CB_ERROR;
  }
  ready = VL53L1CB_WaitReady(pObj, VL53L1CB_FIRST_RANGE_TIMEOUT_MS);
  if ((VL53L1CB_Write8(pObj, VL53L1CB_INTERRUPT_CLEAR, 0x01U) != VL53L1CB_OK) ||
      (VL53L1CB_Write8(pObj, VL53L1CB_MODE_START, VL53L1CB_MODE_STOP) != VL53L1CB_OK))
  {
    return VL53L1CB_ERROR;
  }
  if (ready != 1)
  {
    return (ready == VL53L1CB_TIMEOUT) ? VL53L1CB_TIMEOUT : VL53L1CB_ERROR;
  }
  if ((VL53L1CB_Write8(pObj, VL53L1CB_VHV_LOOP_BOUND, 0x09U) != VL53L1CB_OK) ||
      (VL53L1CB_Write8(pObj, VL53L1CB_VHV_INIT, 0x00U) != VL53L1CB_OK))
  {
    return VL53L1CB_ERROR;
  }

  /* The limits a 4 x 4 region ranges within, in place of the configuration's
   * 90 mm and 1.0 MCPS: sigma in 14.2 fixed point, the minimum in 9.7. */
  if ((VL53L1CB_Write16(pObj, VL53L1CB_SIGMA_THRESH,
                        (uint16_t)(VL53L1CB_SIGMA_LIMIT_MM * 4U)) != VL53L1CB_OK) ||
      (VL53L1CB_Write16(pObj, VL53L1CB_MIN_COUNT_RATE,
                        (uint16_t)((VL53L1CB_MIN_SIGNAL_KCPS * 128U) / 1000U)) != VL53L1CB_OK))
  {
    return VL53L1CB_ERROR;
  }

  pObj->IsInitialized = 1U;
  return VL53L1CB_OK;
}

int32_t VL53L1CB_DeInit(VL53L1CB_Object_t *pObj)
{
  if (pObj == NULL)
  {
    return VL53L1CB_ERROR;
  }
  if (pObj->IsInitialized != 0U)
  {
    (void)VL53L1CB_StopRanging(pObj);
  }
  pObj->IsInitialized = 0U;
  if (pObj->IO.DeInit != NULL)
  {
    (void)pObj->IO.DeInit();
  }
  return VL53L1CB_OK;
}

uint8_t VL53L1CB_SpadNumber(uint8_t Row, uint8_t Col)
{
  if ((Row >= VL53L1CB_SPADS) || (Col >= VL53L1CB_SPADS))
  {
    return 0U;
  }
  if (Row > 7U)
  {
    return (uint8_t)(128U + (8U * Col) + (15U - Row));
  }
  return (uint8_t)((8U * (15U - Col)) + Row);
}

int32_t VL53L1CB_SetROI(VL53L1CB_Object_t *pObj, uint8_t Width, uint8_t Height,
                        uint8_t CentreSpad)
{
  if ((pObj == NULL) || (Width < VL53L1CB_MIN_ROI) || (Width > VL53L1CB_SPADS) ||
      (Height < VL53L1CB_MIN_ROI) || (Height > VL53L1CB_SPADS))
  {
    return VL53L1CB_ERROR;
  }
  if ((VL53L1CB_Write8(pObj, VL53L1CB_ROI_CENTRE_SPAD, CentreSpad) != VL53L1CB_OK) ||
      (VL53L1CB_Write8(pObj, VL53L1CB_ROI_XY_SIZE,
                       (uint8_t)(((uint32_t)(Height - 1U) << 4) | (uint32_t)(Width - 1U))) !=
       VL53L1CB_OK))
  {
    return VL53L1CB_ERROR;
  }
  return VL53L1CB_OK;
}

int32_t VL53L1CB_StartRanging(VL53L1CB_Object_t *pObj)
{
  if ((pObj == NULL) ||
      (VL53L1CB_Write8(pObj, VL53L1CB_INTERRUPT_CLEAR, 0x01U) != VL53L1CB_OK) ||
      (VL53L1CB_Write8(pObj, VL53L1CB_MODE_START, VL53L1CB_MODE_TIMED) != VL53L1CB_OK))
  {
    return VL53L1CB_ERROR;
  }
  pObj->IsRanging = 1U;
  return VL53L1CB_OK;
}

int32_t VL53L1CB_StopRanging(VL53L1CB_Object_t *pObj)
{
  int32_t ret = VL53L1CB_OK;

  if (pObj == NULL)
  {
    return VL53L1CB_ERROR;
  }
  if ((VL53L1CB_Write8(pObj, VL53L1CB_INTERRUPT_CLEAR, 0x01U) != VL53L1CB_OK) ||
      (VL53L1CB_Write8(pObj, VL53L1CB_MODE_START, VL53L1CB_MODE_STOP) != VL53L1CB_OK))
  {
    ret = VL53L1CB_ERROR;
  }
  pObj->IsRanging = 0U;
  return ret;
}

int32_t VL53L1CB_DataReady(VL53L1CB_Object_t *pObj, uint8_t *pReady)
{
  uint8_t mux = 0U;
  uint8_t tio = 0U;
  uint8_t active;

  if ((pObj == NULL) || (pReady == NULL) ||
      (VL53L1CB_Read(pObj, VL53L1CB_GPIO_HV_MUX_CTRL, &mux, 1U) != VL53L1CB_OK) ||
      (VL53L1CB_Read(pObj, VL53L1CB_GPIO_TIO_HV_STATUS, &tio, 1U) != VL53L1CB_OK))
  {
    return VL53L1CB_ERROR;
  }
  active  = ((mux & VL53L1CB_INT_ACTIVE_LOW) != 0U) ? 0U : 1U;
  *pReady = ((tio & 0x01U) == active) ? 1U : 0U;
  return VL53L1CB_OK;
}

int32_t VL53L1CB_ReadResult(VL53L1CB_Object_t *pObj, uint16_t *pDistanceMm,
                            uint8_t *pRangeStatus)
{
  uint8_t status = 0U;
  uint8_t range[2];

  if ((pObj == NULL) || (pDistanceMm == NULL) || (pRangeStatus == NULL) ||
      (VL53L1CB_Read(pObj, VL53L1CB_RANGE_STATUS, &status, 1U) != VL53L1CB_OK) ||
      (VL53L1CB_Read(pObj, VL53L1CB_RANGE_MM, range, 2U) != VL53L1CB_OK) ||
      (VL53L1CB_Write8(pObj, VL53L1CB_INTERRUPT_CLEAR, 0x01U) != VL53L1CB_OK))
  {
    return VL53L1CB_ERROR;
  }
  *pDistanceMm  = (uint16_t)(((uint16_t)range[0] << 8) | range[1]);
  *pRangeStatus = (uint8_t)(status & 0x1FU);
  return VL53L1CB_OK;
}
