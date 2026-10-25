/**
  * @file    mmc5603nj.c
  * @brief   Component driver for the MMC5603NJ three-axis magnetometer.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  *
  * Register addresses, field layouts, scale factors and timings are taken
  * from the MEMSIC MMC5603NJ datasheet (Rev. B, 2022-01-17).
  */

#include "mmc5603nj.h"

/* Count at zero field: the output is 20 unsigned bits, centred on mid-scale. */
#define MMC5603NJ_NULL_FIELD_OFFSET     524288L

/* 16384 counts per gauss in 20-bit mode. */
#define MMC5603NJ_MGAUSS_PER_LSB        (1000.0f / 16384.0f)

/* Temperature output: 0.8 degrees C per count, -75 degrees C at 0 counts. */
#define MMC5603NJ_TEMP_DEGC_PER_LSB     0.8f
#define MMC5603NJ_TEMP_ZERO_DEGC        (-75.0f)

/* Measurement timeout, ms (the slowest bandwidth takes 6.6 ms); bounds a dead part. */
#define MMC5603NJ_MEASURE_TIMEOUT_MS    20U

/* Post-reset wait, ms: for 20 ms the part reloads its trim and does not answer. */
#define MMC5603NJ_RESET_DELAY_MS        25U

/* Wait after each degauss pulse, ms. */
#define MMC5603NJ_PULSE_DELAY_MS        1U

#define MMC5603NJ_OUTPUT_BYTES          9U

/* Settle time, ms, after a free-running stop and after the trigger then spent (the part
 * usually ignores the first trigger after a stop); covers one conversion each. */
#define MMC5603NJ_STOP_SETTLE_MS        20U

static int32_t MMC5603NJ_Read(MMC5603NJ_Object_t *pObj, uint8_t Reg, uint8_t *pData,
                              uint16_t Length);
static int32_t MMC5603NJ_Write(MMC5603NJ_Object_t *pObj, uint8_t Reg, uint8_t Data);
static void    MMC5603NJ_Wait(MMC5603NJ_Object_t *pObj, uint32_t Milliseconds);
static uint32_t MMC5603NJ_ConversionMs(const MMC5603NJ_Object_t *pObj);
static int32_t MMC5603NJ_WaitDone(MMC5603NJ_Object_t *pObj, uint8_t DoneMask);
static int32_t MMC5603NJ_StopFreeRunning(MMC5603NJ_Object_t *pObj);
static int32_t MMC5603NJ_Pause(MMC5603NJ_Object_t *pObj, uint8_t *pWasRunning);
static int32_t MMC5603NJ_Resume(MMC5603NJ_Object_t *pObj, uint8_t WasRunning);

static int32_t MMC5603NJ_Read(MMC5603NJ_Object_t *pObj, uint8_t Reg, uint8_t *pData,
                              uint16_t Length)
{
  if (pObj->IO.ReadReg(pObj->IO.Address, Reg, pData, Length) != 0)
  {
    return MMC5603NJ_ERROR;
  }

  return MMC5603NJ_OK;
}

static int32_t MMC5603NJ_Write(MMC5603NJ_Object_t *pObj, uint8_t Reg, uint8_t Data)
{
  uint8_t value = Data;

  if (pObj->IO.WriteReg(pObj->IO.Address, Reg, &value, 1U) != 0)
  {
    return MMC5603NJ_ERROR;
  }

  return MMC5603NJ_OK;
}

/* Waits only if the board supplied a Delay. */
static void MMC5603NJ_Wait(MMC5603NJ_Object_t *pObj, uint32_t Milliseconds)
{
  if (pObj->IO.Delay != NULL)
  {
    pObj->IO.Delay(Milliseconds);
  }
}

/* Conversion time in ms at the current bandwidth (6.6, 3.5, 2.0 or 1.2 ms), rounded up
 * because it decides when the bus may be touched. */
static uint32_t MMC5603NJ_ConversionMs(const MMC5603NJ_Object_t *pObj)
{
  switch (pObj->Bandwidth)
  {
    case MMC5603NJ_CTRL1_BW_1MS2: return 2U;
    case MMC5603NJ_CTRL1_BW_2MS0: return 2U;
    case MMC5603NJ_CTRL1_BW_3MS5: return 4U;
    default:                      return 7U;
  }
}

/* Poll a done flag until it sets or MMC5603NJ_MEASURE_TIMEOUT_MS runs out. */
static int32_t MMC5603NJ_WaitDone(MMC5603NJ_Object_t *pObj, uint8_t DoneMask)
{
  int32_t start;

  if (pObj->IO.GetTick == NULL)
  {
    /* No time base: wait out the slowest measurement and read once. */
    MMC5603NJ_Wait(pObj, MMC5603NJ_MEASURE_TIMEOUT_MS);
    return MMC5603NJ_OK;
  }

  start = pObj->IO.GetTick();

  for (;;)
  {
    uint8_t status = 0U;

    if (MMC5603NJ_Read(pObj, MMC5603NJ_STATUS1, &status, 1U) != MMC5603NJ_OK)
    {
      return MMC5603NJ_ERROR;
    }

    if ((status & DoneMask) != 0U)
    {
      return MMC5603NJ_OK;
    }

    if (((uint32_t)pObj->IO.GetTick() - (uint32_t)start) > MMC5603NJ_MEASURE_TIMEOUT_MS)
    {
      return MMC5603NJ_ERROR;
    }
  }
}

/* Stop free-running, then spend the trigger the part usually ignores after a stop, once
 * the conversion in flight has finished on a quiet bus. */
static int32_t MMC5603NJ_StopFreeRunning(MMC5603NJ_Object_t *pObj)
{
  if (MMC5603NJ_Write(pObj, MMC5603NJ_CTRL2, 0U) != MMC5603NJ_OK)
  {
    return MMC5603NJ_ERROR;
  }

  pObj->Running = 0U;
  MMC5603NJ_Wait(pObj, MMC5603NJ_STOP_SETTLE_MS);

  if (MMC5603NJ_Write(pObj, MMC5603NJ_CTRL0,
                      MMC5603NJ_CTRL0_AUTO_SR_EN | MMC5603NJ_CTRL0_TAKE_MEAS_M)
      != MMC5603NJ_OK)
  {
    return MMC5603NJ_ERROR;
  }

  MMC5603NJ_Wait(pObj, MMC5603NJ_STOP_SETTLE_MS);

  return MMC5603NJ_OK;
}

/* Stop free-running for a moment; *pWasRunning is for MMC5603NJ_Resume. */
static int32_t MMC5603NJ_Pause(MMC5603NJ_Object_t *pObj, uint8_t *pWasRunning)
{
  *pWasRunning = pObj->Running;

  if (pObj->Running == 0U)
  {
    return MMC5603NJ_OK;
  }

  return MMC5603NJ_StopFreeRunning(pObj);
}

/* Restart free-running if MMC5603NJ_Pause stopped it. */
static int32_t MMC5603NJ_Resume(MMC5603NJ_Object_t *pObj, uint8_t WasRunning)
{
  if (WasRunning == 0U)
  {
    return MMC5603NJ_OK;
  }

  return MMC5603NJ_MAG_SetOutputDataRate(pObj, (float)pObj->ContinuousHz);
}

int32_t MMC5603NJ_RegisterBusIO(MMC5603NJ_Object_t *pObj, const MMC5603NJ_IO_t *pIO)
{
  uint8_t id = 0U;

  if ((pObj == NULL) || (pIO == NULL))
  {
    return MMC5603NJ_ERROR;
  }

  pObj->IO = *pIO;

  if ((pObj->IO.ReadReg == NULL) || (pObj->IO.WriteReg == NULL))
  {
    return MMC5603NJ_ERROR;
  }

  if (pObj->IO.Init != NULL)
  {
    if (pObj->IO.Init() != 0)
    {
      return MMC5603NJ_ERROR;
    }
  }

  /* Not this part, or none: the bus goes back. */
  if ((MMC5603NJ_ReadID(pObj, &id) != MMC5603NJ_OK) || (id != MMC5603NJ_ID))
  {
    if (pObj->IO.DeInit != NULL)
    {
      (void)pObj->IO.DeInit();
    }

    return MMC5603NJ_ERROR;
  }

  return MMC5603NJ_OK;
}

int32_t MMC5603NJ_Init(MMC5603NJ_Object_t *pObj)
{
  if (pObj == NULL)
  {
    return MMC5603NJ_ERROR;
  }

  if (MMC5603NJ_Write(pObj, MMC5603NJ_CTRL1, MMC5603NJ_CTRL1_SW_RESET) != MMC5603NJ_OK)
  {
    return MMC5603NJ_ERROR;
  }

  MMC5603NJ_Wait(pObj, MMC5603NJ_RESET_DELAY_MS);

  /* The reset leaves the part measuring on demand, with nothing measured. */
  pObj->Running      = 0U;
  pObj->ContinuousHz = 0U;
  pObj->HaveTemp     = 0U;

  if (MMC5603NJ_MAG_Degauss(pObj) != MMC5603NJ_OK)
  {
    return MMC5603NJ_ERROR;
  }

  /* Slowest (least noisy) bandwidth, on demand; a caller that wants speed sets a rate. */
  if (MMC5603NJ_Write(pObj, MMC5603NJ_CTRL1, MMC5603NJ_CTRL1_BW_6MS6) != MMC5603NJ_OK)
  {
    return MMC5603NJ_ERROR;
  }

  pObj->Bandwidth     = MMC5603NJ_CTRL1_BW_6MS6;
  pObj->LastTempDegC  = 0.0f;
  pObj->IsEnabled     = 0U;
  pObj->IsInitialized = 1U;

  return MMC5603NJ_OK;
}

int32_t MMC5603NJ_DeInit(MMC5603NJ_Object_t *pObj)
{
  if (pObj == NULL)
  {
    return MMC5603NJ_ERROR;
  }

  if (pObj->IsInitialized != 0U)
  {
    (void)MMC5603NJ_MAG_Disable(pObj);
  }

  if (pObj->IO.DeInit != NULL)
  {
    (void)pObj->IO.DeInit();
  }

  pObj->IsInitialized = 0U;

  return MMC5603NJ_OK;
}

int32_t MMC5603NJ_ReadID(MMC5603NJ_Object_t *pObj, uint8_t *pId)
{
  if ((pObj == NULL) || (pId == NULL))
  {
    return MMC5603NJ_ERROR;
  }

  return MMC5603NJ_Read(pObj, MMC5603NJ_PRODUCT_ID, pId, 1U);
}

int32_t MMC5603NJ_GetCapabilities(MMC5603NJ_Object_t *pObj,
                                  MMC5603NJ_Capabilities_t *pCapabilities)
{
  (void)pObj;

  if (pCapabilities == NULL)
  {
    return MMC5603NJ_ERROR;
  }

  pCapabilities->Acc        = 0U;
  pCapabilities->Gyro       = 0U;
  pCapabilities->Magneto    = 1U;
  pCapabilities->LowPower   = 0U;
  pCapabilities->GyroMaxFS  = 0U;
  pCapabilities->AccMaxFS   = 0U;
  pCapabilities->MagMaxFS   = (uint32_t)MMC5603NJ_FULL_SCALE_GAUSS;
  pCapabilities->GyroMaxOdr = 0.0f;
  pCapabilities->AccMaxOdr  = 0.0f;
  pCapabilities->MagMaxOdr  = MMC5603NJ_MAX_ODR_HZ;

  return MMC5603NJ_OK;
}

int32_t MMC5603NJ_MAG_Enable(MMC5603NJ_Object_t *pObj)
{
  if (pObj == NULL)
  {
    return MMC5603NJ_ERROR;
  }

  pObj->IsEnabled = 1U;

  /* If a rate was chosen while the sensor was idle, start it now. */
  if (pObj->ContinuousHz != 0U)
  {
    return MMC5603NJ_MAG_SetOutputDataRate(pObj, (float)pObj->ContinuousHz);
  }

  return MMC5603NJ_OK;
}

int32_t MMC5603NJ_MAG_Disable(MMC5603NJ_Object_t *pObj)
{
  if (pObj == NULL)
  {
    return MMC5603NJ_ERROR;
  }

  /* Stop measuring but keep the rate for the next enable. */
  if (pObj->Running != 0U)
  {
    if (MMC5603NJ_StopFreeRunning(pObj) != MMC5603NJ_OK)
    {
      return MMC5603NJ_ERROR;
    }
  }
  else if (MMC5603NJ_Write(pObj, MMC5603NJ_CTRL2, 0U) != MMC5603NJ_OK)
  {
    return MMC5603NJ_ERROR;
  }

  pObj->IsEnabled = 0U;

  return MMC5603NJ_OK;
}

int32_t MMC5603NJ_MAG_GetAxes(MMC5603NJ_Object_t *pObj, MMC5603NJ_Axes_t *pAxes)
{
  MMC5603NJ_AxesRaw_t raw;

  if ((pObj == NULL) || (pAxes == NULL))
  {
    return MMC5603NJ_ERROR;
  }

  if (MMC5603NJ_MAG_GetAxesRaw(pObj, &raw) != MMC5603NJ_OK)
  {
    return MMC5603NJ_ERROR;
  }

  pAxes->x = (int32_t)((float)raw.x * MMC5603NJ_MGAUSS_PER_LSB);
  pAxes->y = (int32_t)((float)raw.y * MMC5603NJ_MGAUSS_PER_LSB);
  pAxes->z = (int32_t)((float)raw.z * MMC5603NJ_MGAUSS_PER_LSB);

  return MMC5603NJ_OK;
}

int32_t MMC5603NJ_MAG_GetAxesRaw(MMC5603NJ_Object_t *pObj, MMC5603NJ_AxesRaw_t *pAxes)
{
  uint8_t buffer[MMC5603NJ_OUTPUT_BYTES];

  if ((pObj == NULL) || (pAxes == NULL))
  {
    return MMC5603NJ_ERROR;
  }

  if (pObj->Running == 0U)
  {
    /* Not free-running (never started, or stopped by an error mid-pause): trigger a
     * measurement. AUTO_SR_EN is re-sent because CTRL0 is write-only. */
    if (MMC5603NJ_Write(pObj, MMC5603NJ_CTRL0,
                        MMC5603NJ_CTRL0_AUTO_SR_EN | MMC5603NJ_CTRL0_TAKE_MEAS_M)
        != MMC5603NJ_OK)
    {
      return MMC5603NJ_ERROR;
    }

    /* Keep the bus quiet until the conversion is over: a STATUS1 read during an
     * automatic set/reset measurement zeros all nine outputs, with no error.
     * Set/reset runs two conversions, so wait twice the conversion time. */
    MMC5603NJ_Wait(pObj, 2U * MMC5603NJ_ConversionMs(pObj));

    if (MMC5603NJ_WaitDone(pObj, MMC5603NJ_STATUS1_MEAS_M_DONE) != MMC5603NJ_OK)
    {
      return MMC5603NJ_ERROR;
    }
  }

  if (MMC5603NJ_Read(pObj, MMC5603NJ_XOUT0, buffer, MMC5603NJ_OUTPUT_BYTES) != MMC5603NJ_OK)
  {
    return MMC5603NJ_ERROR;
  }

  if (pObj->IO.GetTick != NULL)
  {
    pObj->SampleTick = (uint32_t)pObj->IO.GetTick();
  }

  /* Bytes 0-5: top 16 bits of each axis; bytes 6-8: the low 4 bits in the high nibble. */
  pAxes->x = (int32_t)(((uint32_t)buffer[0] << 12) |
                       ((uint32_t)buffer[1] << 4) |
                       ((uint32_t)buffer[6] >> 4)) - MMC5603NJ_NULL_FIELD_OFFSET;
  pAxes->y = (int32_t)(((uint32_t)buffer[2] << 12) |
                       ((uint32_t)buffer[3] << 4) |
                       ((uint32_t)buffer[7] >> 4)) - MMC5603NJ_NULL_FIELD_OFFSET;
  pAxes->z = (int32_t)(((uint32_t)buffer[4] << 12) |
                       ((uint32_t)buffer[5] << 4) |
                       ((uint32_t)buffer[8] >> 4)) - MMC5603NJ_NULL_FIELD_OFFSET;

  return MMC5603NJ_OK;
}

int32_t MMC5603NJ_MAG_GetDataReady(MMC5603NJ_Object_t *pObj, uint8_t *pReady)
{
  uint32_t period_ms;

  if ((pObj == NULL) || (pReady == NULL))
  {
    return MMC5603NJ_ERROR;
  }

  /* On demand the next read measures; without a clock, the caller paces. */
  if ((pObj->Running == 0U) || (pObj->ContinuousHz == 0U) || (pObj->IO.GetTick == NULL))
  {
    *pReady = 1U;
    return MMC5603NJ_OK;
  }

  /* Free-running: by the clock; polling the status or outputs costs samples. */
  period_ms = (1000U + (uint32_t)pObj->ContinuousHz - 1U) / (uint32_t)pObj->ContinuousHz;
  *pReady = (((uint32_t)pObj->IO.GetTick() - pObj->SampleTick) >= period_ms) ? 1U : 0U;

  return MMC5603NJ_OK;
}

int32_t MMC5603NJ_MAG_GetSensitivity(MMC5603NJ_Object_t *pObj, float *pSensitivity)
{
  (void)pObj;

  if (pSensitivity == NULL)
  {
    return MMC5603NJ_ERROR;
  }

  *pSensitivity = MMC5603NJ_MGAUSS_PER_LSB;

  return MMC5603NJ_OK;
}

int32_t MMC5603NJ_MAG_GetOutputDataRate(MMC5603NJ_Object_t *pObj, float *pOdr)
{
  if ((pObj == NULL) || (pOdr == NULL))
  {
    return MMC5603NJ_ERROR;
  }

  *pOdr = (float)pObj->ContinuousHz;

  return MMC5603NJ_OK;
}

int32_t MMC5603NJ_MAG_SetOutputDataRate(MMC5603NJ_Object_t *pObj, float Odr)
{
  uint16_t rate;
  uint8_t  bandwidth;

  if (pObj == NULL)
  {
    return MMC5603NJ_ERROR;
  }

  if (Odr <= 0.0f)
  {
    /* Back to on demand, at the slowest and quietest bandwidth. */
    if (pObj->Running != 0U)
    {
      if (MMC5603NJ_StopFreeRunning(pObj) != MMC5603NJ_OK)
      {
        return MMC5603NJ_ERROR;
      }
    }
    else if (MMC5603NJ_Write(pObj, MMC5603NJ_CTRL2, 0U) != MMC5603NJ_OK)
    {
      return MMC5603NJ_ERROR;
    }

    if (MMC5603NJ_Write(pObj, MMC5603NJ_CTRL1, MMC5603NJ_CTRL1_BW_6MS6) != MMC5603NJ_OK)
    {
      return MMC5603NJ_ERROR;
    }

    pObj->Bandwidth    = MMC5603NJ_CTRL1_BW_6MS6;
    pObj->ContinuousHz = 0U;

    return MMC5603NJ_OK;
  }

  /* Round up: a rate between two whole hertz runs at the higher. */
  if (Odr >= MMC5603NJ_MAX_ODR_HZ)
  {
    rate = 255U;
  }
  else
  {
    rate = (uint16_t)Odr;
    if ((float)rate < Odr)
    {
      rate++;
    }
  }

  if (rate == 0U)
  {
    rate = 1U;
  }

  /* Narrowest bandwidth that fits the period; a slower one silently misses the rate. */
  if (rate <= 75U)
  {
    bandwidth = MMC5603NJ_CTRL1_BW_6MS6;
  }
  else if (rate <= 150U)
  {
    bandwidth = MMC5603NJ_CTRL1_BW_3MS5;
  }
  else
  {
    bandwidth = MMC5603NJ_CTRL1_BW_2MS0;
  }

  if (MMC5603NJ_Write(pObj, MMC5603NJ_CTRL1, bandwidth) != MMC5603NJ_OK)
  {
    return MMC5603NJ_ERROR;
  }

  pObj->Bandwidth = bandwidth;

  if (MMC5603NJ_Write(pObj, MMC5603NJ_ODR, (uint8_t)rate) != MMC5603NJ_OK)
  {
    return MMC5603NJ_ERROR;
  }

  /* ODR is latched when CMM_FREQ_EN is written: after the rate, before the start. */
  if (MMC5603NJ_Write(pObj, MMC5603NJ_CTRL0,
                      MMC5603NJ_CTRL0_AUTO_SR_EN | MMC5603NJ_CTRL0_CMM_FREQ_EN)
      != MMC5603NJ_OK)
  {
    return MMC5603NJ_ERROR;
  }

  if (MMC5603NJ_Write(pObj, MMC5603NJ_CTRL2, MMC5603NJ_CTRL2_CMM_EN) != MMC5603NJ_OK)
  {
    return MMC5603NJ_ERROR;
  }

  pObj->Running      = 1U;
  pObj->ContinuousHz = rate;

  /* The first sample is a conversion away. */
  if (pObj->IO.GetTick != NULL)
  {
    pObj->SampleTick = (uint32_t)pObj->IO.GetTick();
  }

  return MMC5603NJ_OK;
}

int32_t MMC5603NJ_MAG_GetFullScale(MMC5603NJ_Object_t *pObj, int32_t *pFullScale)
{
  (void)pObj;

  if (pFullScale == NULL)
  {
    return MMC5603NJ_ERROR;
  }

  *pFullScale = MMC5603NJ_FULL_SCALE_GAUSS;

  return MMC5603NJ_OK;
}

int32_t MMC5603NJ_MAG_SetFullScale(MMC5603NJ_Object_t *pObj, int32_t FullScale)
{
  (void)pObj;

  return (FullScale <= MMC5603NJ_FULL_SCALE_GAUSS) ? MMC5603NJ_OK : MMC5603NJ_ERROR;
}

int32_t MMC5603NJ_MAG_Degauss(MMC5603NJ_Object_t *pObj)
{
  uint8_t was_running = 0U;

  if (pObj == NULL)
  {
    return MMC5603NJ_ERROR;
  }

  /* Pause so the pulses miss conversions; the restart rewrites the write-only CTRL0. */
  if (MMC5603NJ_Pause(pObj, &was_running) != MMC5603NJ_OK)
  {
    return MMC5603NJ_ERROR;
  }

  /* A failed write still restarts free-running, so the part is not left stopped. */
  if (MMC5603NJ_Write(pObj, MMC5603NJ_CTRL0, MMC5603NJ_CTRL0_DO_SET) != MMC5603NJ_OK)
  {
    (void)MMC5603NJ_Resume(pObj, was_running);
    return MMC5603NJ_ERROR;
  }

  MMC5603NJ_Wait(pObj, MMC5603NJ_PULSE_DELAY_MS);

  if (MMC5603NJ_Write(pObj, MMC5603NJ_CTRL0, MMC5603NJ_CTRL0_DO_RESET) != MMC5603NJ_OK)
  {
    (void)MMC5603NJ_Resume(pObj, was_running);
    return MMC5603NJ_ERROR;
  }

  MMC5603NJ_Wait(pObj, MMC5603NJ_PULSE_DELAY_MS);

  return MMC5603NJ_Resume(pObj, was_running);
}

int32_t MMC5603NJ_GetTemperature(MMC5603NJ_Object_t *pObj, float *pTemp)
{
  uint8_t raw = 0U;
  uint8_t was_running = 0U;
  int32_t measured;

  if ((pObj == NULL) || (pTemp == NULL))
  {
    return MMC5603NJ_ERROR;
  }

  /* Free-running ignores a temperature trigger, and STATUS1 must not be read during a
   * set/reset conversion: pause free-running for the reading. */
  if (MMC5603NJ_Pause(pObj, &was_running) != MMC5603NJ_OK)
  {
    return MMC5603NJ_ERROR;
  }

  if (MMC5603NJ_Write(pObj, MMC5603NJ_CTRL0,
                      MMC5603NJ_CTRL0_AUTO_SR_EN | MMC5603NJ_CTRL0_TAKE_MEAS_T)
      != MMC5603NJ_OK)
  {
    (void)MMC5603NJ_Resume(pObj, was_running);
    return MMC5603NJ_ERROR;
  }

  /* Quiet for a conversion's length, then the flag. */
  MMC5603NJ_Wait(pObj, MMC5603NJ_ConversionMs(pObj));
  measured = MMC5603NJ_WaitDone(pObj, MMC5603NJ_STATUS1_MEAS_T_DONE);

  if (measured == MMC5603NJ_OK)
  {
    if (MMC5603NJ_Read(pObj, MMC5603NJ_TOUT, &raw, 1U) != MMC5603NJ_OK)
    {
      (void)MMC5603NJ_Resume(pObj, was_running);
      return MMC5603NJ_ERROR;
    }

    pObj->LastTempDegC = ((float)raw * MMC5603NJ_TEMP_DEGC_PER_LSB) + MMC5603NJ_TEMP_ZERO_DEGC;
    pObj->HaveTemp     = 1U;
  }

  if (MMC5603NJ_Resume(pObj, was_running) != MMC5603NJ_OK)
  {
    return MMC5603NJ_ERROR;
  }

  /* A missed measurement is not a fault while an earlier reading stands. */
  if (pObj->HaveTemp == 0U)
  {
    return MMC5603NJ_ERROR;
  }

  *pTemp = pObj->LastTempDegC;

  return MMC5603NJ_OK;
}

int32_t MMC5603NJ_ReadReg(MMC5603NJ_Object_t *pObj, uint8_t Reg, uint8_t *pData)
{
  if ((pObj == NULL) || (pData == NULL))
  {
    return MMC5603NJ_ERROR;
  }

  return MMC5603NJ_Read(pObj, Reg, pData, 1U);
}

int32_t MMC5603NJ_WriteReg(MMC5603NJ_Object_t *pObj, uint8_t Reg, uint8_t Data)
{
  if (pObj == NULL)
  {
    return MMC5603NJ_ERROR;
  }

  return MMC5603NJ_Write(pObj, Reg, Data);
}
