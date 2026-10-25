/**
  * @file    neuron6_adc.c
  * @brief   Analogue input for the Neuro N6: six header pins and three internal channels.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6_adc.h"
#include "neuron6.h"     /* BSP_ISOLATION_Config: the converters need a stamp */

/* Poll budget for one conversion: only bounds a converter that has stopped responding. */
#define ADC_CONVERSION_TIMEOUT_MS   10U

/* 12-bit full scale, the width the factory VREFINT calibration was measured at. */
#define ADC_FULL_SCALE_12B          4096U

/* Plausible VREFINT calibration: an erased or unreadable word reads 0 or all ones. */
#define ADC_VREFINT_CAL_MIN         1000U
#define ADC_VREFINT_CAL_MAX         4000U

/* Nominal VDDA in mV (1.8 V, not 3.3 V), assumed when there is no usable calibration. */
#ifndef BSP_ADC_NOMINAL_VDDA_MV
#define BSP_ADC_NOMINAL_VDDA_MV     1800U
#endif

/* Index 0 is ADC1, 1 is ADC2. */
static ADC_HandleTypeDef hbsp_adc[2];

/* Set once the converter is configured and calibrated. */
static uint8_t ADC_Ready[2];

static uint8_t Adc_VddaMeasured;

static uint32_t ADC_InitCounter;

/* Width of pin reads; internal channels always use 12 bits. */
static uint32_t ADC_Resolution = BSP_ADC_RESOLUTION_12B;

static int32_t ADC_SelectKernelClock(void);
static int32_t ADC_Prepare(uint8_t Instance);
static int32_t ADC_Convert(uint8_t Instance, uint32_t Channel, uint32_t Resolution,
                           uint32_t *pValue);
static int32_t ADC_ReadInternal(uint8_t Instance, uint32_t Channel, uint32_t *pMillivolts);
static int32_t ADC_ReadPin(uint32_t Pin, uint32_t *pValue, uint32_t *pResolution);

/* Both converters share one kernel clock: HCLK / BSP_ADC_CLOCK_DIVIDER. */
static int32_t ADC_SelectKernelClock(void)
{
  RCC_PeriphCLKInitTypeDef periph_clk_init = {0};

  periph_clk_init.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  periph_clk_init.AdcClockSelection    = RCC_ADCCLKSOURCE_HCLK;
  periph_clk_init.AdcDivider           = BSP_ADC_CLOCK_DIVIDER;

  if (HAL_RCCEx_PeriphCLKConfig(&periph_clk_init) != HAL_OK)
  {
    return BSP_ERROR_CLOCK_FAILURE;
  }

  return BSP_ERROR_NONE;
}

/* Configure and calibrate converter Instance (1 or 2), once. */
static int32_t ADC_Prepare(uint8_t Instance)
{
  ADC_HandleTypeDef *hadc;
  uint32_t index;
  int32_t  status;

  if ((Instance != 1U) && (Instance != 2U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  index = (uint32_t)Instance - 1U;

  if (ADC_Ready[index] != 0U)
  {
    return BSP_ERROR_NONE;
  }

  status = ADC_SelectKernelClock();

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  /* Marks ADC12 secure: otherwise the analogue pads, which boot secure, are not routed
   * to it and every external channel reads an unconnected node. */
  (void)BSP_ISOLATION_Config();

  __HAL_RCC_ADC12_CLK_ENABLE();

  hadc = &hbsp_adc[index];
  hadc->Instance = (Instance == 1U) ? ADC1 : ADC2;

  hadc->Init.Resolution                = ADC_Resolution;
  hadc->Init.ScanConvMode              = ADC_SCAN_DISABLE;
  hadc->Init.EOCSelection              = ADC_EOC_SINGLE_CONV;
  hadc->Init.LowPowerAutoWait          = DISABLE;
  hadc->Init.ContinuousConvMode        = DISABLE;
  hadc->Init.NbrOfConversion           = 1U;
  hadc->Init.DiscontinuousConvMode     = DISABLE;
  hadc->Init.ExternalTrigConv          = ADC_SOFTWARE_START;
  hadc->Init.ExternalTrigConvEdge      = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc->Init.ConversionDataManagement  = ADC_CONVERSIONDATA_DR;
  hadc->Init.Overrun                   = ADC_OVR_DATA_OVERWRITTEN;
  hadc->Init.OversamplingMode          = DISABLE;

  if (HAL_ADC_Init(hadc) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* Offset calibration, required once per converter for rated accuracy. */
  if (HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED) != HAL_OK)
  {
    (void)HAL_ADC_DeInit(hadc);
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* Calibration leaves it enabled, and HAL_ADC_ConfigChannel refuses an enabled ADC. */
  (void)HAL_ADC_Stop(hadc);

  ADC_Ready[index] = 1U;

  return BSP_ERROR_NONE;
}

static int32_t ADC_Convert(uint8_t Instance, uint32_t Channel, uint32_t Resolution,
                           uint32_t *pValue)
{
  ADC_HandleTypeDef *hadc;
  ADC_ChannelConfTypeDef channel_config = {0};
  int32_t status;

  status = ADC_Prepare(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  hadc = &hbsp_adc[(uint32_t)Instance - 1U];

  /* A new width needs a re-init: the init structure owns that register. */
  if (hadc->Init.Resolution != Resolution)
  {
    hadc->Init.Resolution = Resolution;

    if (HAL_ADC_Init(hadc) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
  }

  channel_config.Channel      = Channel;
  channel_config.Rank         = ADC_REGULAR_RANK_1;
  channel_config.SamplingTime = BSP_ADC_SAMPLING_TIME;
  channel_config.SingleDiff   = ADC_SINGLE_ENDED;
  channel_config.OffsetNumber = ADC_OFFSET_NONE;

  /* The channel can only be selected with the converter stopped. */
  (void)HAL_ADC_Stop(hadc);

  if (HAL_ADC_ConfigChannel(hadc, &channel_config) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  if (HAL_ADC_Start(hadc) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  if (HAL_ADC_PollForConversion(hadc, ADC_CONVERSION_TIMEOUT_MS) != HAL_OK)
  {
    (void)HAL_ADC_Stop(hadc);
    return BSP_ERROR_BUSY;
  }

  *pValue = HAL_ADC_GetValue(hadc);

  (void)HAL_ADC_Stop(hadc);

  return BSP_ERROR_NONE;
}

/* Internal channel in mV, always at 12 bits whatever width pin reads use. */
static int32_t ADC_ReadInternal(uint8_t Instance, uint32_t Channel, uint32_t *pMillivolts)
{
  uint32_t raw = 0U;
  uint32_t vdda = 0U;
  int32_t  status;

  if (pMillivolts == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Not under the lock: BSP_ADC_GetVdda takes it, and the lock need not be recursive. */
  status = BSP_ADC_GetVdda(&vdda);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  BSP_ADC_Lock();
  status = ADC_Convert(Instance, Channel, BSP_ADC_RESOLUTION_12B, &raw);
  BSP_ADC_Unlock();

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  *pMillivolts = (vdda * raw) / ADC_FULL_SCALE_12B;

  return BSP_ERROR_NONE;
}

int32_t BSP_ADC_Init(void)
{
  int32_t status;

  BSP_ADC_Lock();

  if (ADC_InitCounter == 0U)
  {
    status = ADC_Prepare(1U);

    if (status == BSP_ERROR_NONE)
    {
      status = ADC_Prepare(2U);
    }
  }
  else
  {
    status = BSP_ERROR_NONE;
  }

  if (status == BSP_ERROR_NONE)
  {
    ADC_InitCounter++;
  }

  BSP_ADC_Unlock();

  return status;
}

int32_t BSP_ADC_DeInit(void)
{
  int32_t status = BSP_ERROR_NONE;

  BSP_ADC_Lock();

  if (ADC_InitCounter == 0U)
  {
    status = BSP_ERROR_NO_INIT;
  }
  else
  {
    ADC_InitCounter--;

    if (ADC_InitCounter == 0U)
    {
      uint32_t index;

      for (index = 0U; index < 2U; index++)
      {
        if (ADC_Ready[index] != 0U)
        {
          (void)HAL_ADC_DeInit(&hbsp_adc[index]);
          ADC_Ready[index] = 0U;
        }
      }

      /* Read back before gating the clock, or a posted write stalls the bus. */
      (void)READ_REG(ADC12_COMMON->CCR);
      __DSB();
      __HAL_RCC_ADC12_CLK_DISABLE();
    }
  }

  BSP_ADC_Unlock();

  return status;
}

int32_t BSP_ADC_SetResolution(uint32_t Resolution)
{
  if ((Resolution != BSP_ADC_RESOLUTION_6B) &&
      (Resolution != BSP_ADC_RESOLUTION_8B) &&
      (Resolution != BSP_ADC_RESOLUTION_10B) &&
      (Resolution != BSP_ADC_RESOLUTION_12B))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  ADC_Resolution = Resolution;

  return BSP_ERROR_NONE;
}

uint32_t BSP_ADC_GetResolution(void)
{
  return ADC_Resolution;
}

/* pResolution (or NULL) gets the width used, read under the conversion's lock. */
static int32_t ADC_ReadPin(uint32_t Pin, uint32_t *pValue, uint32_t *pResolution)
{
  BSP_IO_PinInfo_t info;
  GPIO_InitTypeDef gpio_init_structure = {0};
  uint32_t resolution;
  int32_t status;

  if (pValue == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  status = BSP_IO_GetPinInfo(Pin, &info);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((info.AdcInstance == 0U) || (info.AdcChannel >= 20U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Pulls off: a pull would load the source and drag the reading towards a rail. */
  gpio_init_structure.Pin  = info.Pin;
  gpio_init_structure.Mode = GPIO_MODE_ANALOG;
  gpio_init_structure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(info.Port, &gpio_init_structure);

  BSP_ADC_Lock();
  resolution = ADC_Resolution;
  /* The pin table holds the input number; the HAL wants the encoded channel. */
  status = ADC_Convert(info.AdcInstance,
                       __LL_ADC_DECIMAL_NB_TO_CHANNEL((uint32_t)info.AdcChannel),
                       resolution, pValue);
  BSP_ADC_Unlock();

  if (pResolution != NULL)
  {
    *pResolution = resolution;
  }

  return status;
}

int32_t BSP_ADC_ReadChannel(uint32_t Pin, uint32_t *pValue)
{
  return ADC_ReadPin(Pin, pValue, NULL);
}

int32_t BSP_ADC_ReadVoltage(uint32_t Pin, uint32_t *pMillivolts)
{
  uint32_t raw = 0U;
  uint32_t vdda = 0U;
  uint32_t full_scale;
  uint32_t resolution = BSP_ADC_RESOLUTION_12B;
  int32_t  status;

  if (pMillivolts == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Not under the lock: each step takes it. Scaled by the width actually converted at. */
  status = BSP_ADC_GetVdda(&vdda);

  if (status == BSP_ERROR_NONE)
  {
    status = ADC_ReadPin(Pin, &raw, &resolution);
  }

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  switch (resolution)
  {
    case BSP_ADC_RESOLUTION_6B:
      full_scale = 64U;
      break;

    case BSP_ADC_RESOLUTION_8B:
      full_scale = 256U;
      break;

    case BSP_ADC_RESOLUTION_10B:
      full_scale = 1024U;
      break;

    default:
      full_scale = ADC_FULL_SCALE_12B;
      break;
  }

  *pMillivolts = (vdda * raw) / full_scale;

  return BSP_ERROR_NONE;
}

int32_t BSP_ADC_GetVdda(uint32_t *pMillivolts)
{
  uint32_t calibration;
  uint32_t raw = 0U;
  int32_t  status;

  if (pMillivolts == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  calibration = (uint32_t)(*VREFINT_CAL_ADDR);

  if ((calibration < ADC_VREFINT_CAL_MIN) || (calibration > ADC_VREFINT_CAL_MAX))
  {
    /* No usable calibration: assume the nominal supply rather than fail every read. */
    *pMillivolts = BSP_ADC_NOMINAL_VDDA_MV;
    Adc_VddaMeasured = 0U;

    return BSP_ERROR_NONE;
  }

  BSP_ADC_Lock();
  /* The reference is only wired to the first converter on this part. */
  status = ADC_Convert(1U, ADC_CHANNEL_VREFINT, BSP_ADC_RESOLUTION_12B, &raw);
  BSP_ADC_Unlock();

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (raw == 0U)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* VREFINT is fixed: VDDA = VREFINT_CAL_VREF x calibration / reading now. */
  *pMillivolts = (VREFINT_CAL_VREF * calibration) / raw;
  Adc_VddaMeasured = 1U;

  return BSP_ERROR_NONE;
}

int32_t BSP_ADC_IsVddaMeasured(void)
{
  return (int32_t)Adc_VddaMeasured;
}

int32_t BSP_ADC_ReadVBat(uint32_t *pMillivolts)
{
  int32_t status = ADC_ReadInternal(2U, ADC_CHANNEL_VBAT, pMillivolts);

  if (status == BSP_ERROR_NONE)
  {
    *pMillivolts *= BSP_ADC_VBAT_DIVIDER;
  }

  return status;
}

int32_t BSP_ADC_ReadVddCore(uint32_t *pMillivolts)
{
  return ADC_ReadInternal(2U, ADC_CHANNEL_VDDCORE, pMillivolts);
}

__weak void BSP_ADC_Lock(void)
{
}

__weak void BSP_ADC_Unlock(void)
{
}
