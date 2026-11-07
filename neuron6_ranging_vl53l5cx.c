/**
  * @file    neuron6_ranging_vl53l5cx.c
  * @brief   VL53L5CX multizone time-of-flight ranging (ST's B-CAMS-IMX module).
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6_ranging_vl53l5cx.h"
#include "neuron6.h"
#include "neuron6_bus.h"
#include "vl53l5cx_api.h"

#include <string.h>

/* VL53L1CB model ID register and value: that part is not probed with writes. */
#define L5_L1_MODEL_ID_REG    0x010FU
#define L5_L1_MODEL_ID        0xEACCU

/* Wait after LPn goes high before the first transfer. */
#define L5_LPN_SETTLE_MS      2U

static VL53L5CX_Configuration L5_Dev;
static VL53L5CX_ResultsData   L5_Data;

static uint8_t  L5_BusUp;     /* this module holds a share of I2C1 */
static uint8_t  L5_Loaded;    /* firmware running */
static uint8_t  L5_Ranging;
static uint8_t  L5_Pending;   /* IsReady saw a result GetResult has not read */
static uint32_t L5_Zones;

static void     L5_PinsUp(void);
static void     L5_PinsDown(void);
static uint32_t L5_IntSeen(uint32_t Clear);
static int32_t  L5_Release(void);

__weak void BSP_RANGING_VL53L5CX_WaitHook(void)
{
}

static void L5_PinsUp(void)
{
#if (defined(BSP_RANGING_VL53L5CX_LPN_PORT) && defined(BSP_RANGING_VL53L5CX_LPN_PIN)) || \
    (defined(BSP_RANGING_VL53L5CX_INT_PORT) && defined(BSP_RANGING_VL53L5CX_INT_PIN))
  GPIO_InitTypeDef gpio = {0};

  gpio.Speed = GPIO_SPEED_FREQ_LOW;
#endif

#if defined(BSP_RANGING_VL53L5CX_LPN_PORT) && defined(BSP_RANGING_VL53L5CX_LPN_PIN)
  BSP_RANGING_VL53L5CX_LPN_CLK_ENABLE();
  HAL_GPIO_WritePin(BSP_RANGING_VL53L5CX_LPN_PORT, BSP_RANGING_VL53L5CX_LPN_PIN, GPIO_PIN_SET);
  gpio.Pin  = BSP_RANGING_VL53L5CX_LPN_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BSP_RANGING_VL53L5CX_LPN_PORT, &gpio);
#endif

#if defined(BSP_RANGING_VL53L5CX_INT_PORT) && defined(BSP_RANGING_VL53L5CX_INT_PIN)
  /* INT pulses low about 100 us per result. The line's falling-edge pending bit holds
   * it; the bit sets only with the line unmasked, and its NVIC interrupt stays off. */
  BSP_RANGING_VL53L5CX_INT_CLK_ENABLE();
  gpio.Pin  = BSP_RANGING_VL53L5CX_INT_PIN;
  gpio.Mode = GPIO_MODE_IT_FALLING;
  gpio.Pull = GPIO_PULLUP;      /* open drain */
  HAL_GPIO_Init(BSP_RANGING_VL53L5CX_INT_PORT, &gpio);
  __HAL_GPIO_EXTI_CLEAR_FALLING_IT(BSP_RANGING_VL53L5CX_INT_PIN);
#endif
}

/* 1 if INT has fallen since the last call that cleared it; always 1 without INT. */
static uint32_t L5_IntSeen(uint32_t Clear)
{
#if defined(BSP_RANGING_VL53L5CX_INT_PORT) && defined(BSP_RANGING_VL53L5CX_INT_PIN)
  const uint32_t seen = (__HAL_GPIO_EXTI_GET_FALLING_IT(BSP_RANGING_VL53L5CX_INT_PIN) != 0U) ? 1U : 0U;

  if ((seen != 0U) && (Clear != 0U))
  {
    __HAL_GPIO_EXTI_CLEAR_FALLING_IT(BSP_RANGING_VL53L5CX_INT_PIN);
  }
  return seen;
#else
  (void)Clear;
  return 1U;
#endif
}

/* Back to their reset state: the module's own pulls hold the lines. */
static void L5_PinsDown(void)
{
#if defined(BSP_RANGING_VL53L5CX_LPN_PORT) && defined(BSP_RANGING_VL53L5CX_LPN_PIN)
  HAL_GPIO_DeInit(BSP_RANGING_VL53L5CX_LPN_PORT, BSP_RANGING_VL53L5CX_LPN_PIN);
#endif
#if defined(BSP_RANGING_VL53L5CX_INT_PORT) && defined(BSP_RANGING_VL53L5CX_INT_PIN)
  HAL_GPIO_DeInit(BSP_RANGING_VL53L5CX_INT_PORT, BSP_RANGING_VL53L5CX_INT_PIN);
  __HAL_GPIO_EXTI_CLEAR_FALLING_IT(BSP_RANGING_VL53L5CX_INT_PIN);
#endif
}

static int32_t L5_Release(void)
{
  int32_t ret = BSP_ERROR_NONE;

  L5_Loaded  = 0U;
  L5_Ranging = 0U;
  L5_Pending = 0U;
  L5_PinsDown();
  if (L5_BusUp != 0U)
  {
    L5_BusUp = 0U;
    ret = BSP_I2C1_DeInit();
  }
  return ret;
}

int32_t BSP_RANGING_VL53L5CX_Init(void)
{
  uint8_t id[2] = {0U, 0U};
  uint8_t alive = 0U;

  if (L5_BusUp == 0U)
  {
    if (BSP_I2C1_Init() != BSP_ERROR_NONE)
    {
      return BSP_ERROR_BUS_FAILURE;
    }
    L5_BusUp = 1U;
  }
  else if (L5_Ranging != 0U)
  {
    (void)vl53l5cx_stop_ranging(&L5_Dev);
  }
  L5_Loaded  = 0U;
  L5_Ranging = 0U;
  L5_Pending = 0U;
  L5_PinsUp();

  (void)memset(&L5_Dev, 0, sizeof(L5_Dev));
  L5_Dev.platform.address = BSP_RANGING_VL53L5CX_I2C_ADDRESS;
  L5_Dev.platform.Write   = BSP_I2C1_WriteReg16;
  L5_Dev.platform.Read    = BSP_I2C1_ReadReg16;
  L5_Dev.platform.GetTick = HAL_GetTick;
  L5_Dev.platform.Wait    = BSP_RANGING_VL53L5CX_WaitHook;
  (void)WaitMs(&L5_Dev.platform, L5_LPN_SETTLE_MS);

  /* A VL53L1CB at the same address is recognised by a read alone. */
  if ((BSP_I2C1_ReadReg16(BSP_RANGING_VL53L5CX_I2C_ADDRESS, L5_L1_MODEL_ID_REG, id, 2U) != BSP_ERROR_NONE) ||
      ((((uint32_t)id[0] << 8) | id[1]) == L5_L1_MODEL_ID) ||
      (vl53l5cx_is_alive(&L5_Dev, &alive) != VL53L5CX_STATUS_OK) || (alive == 0U))
  {
    (void)L5_Release();
    return BSP_ERROR_UNKNOWN_COMPONENT;
  }

  if (vl53l5cx_init(&L5_Dev) != VL53L5CX_STATUS_OK)
  {
    (void)L5_Release();
    return BSP_ERROR_COMPONENT_FAILURE;
  }
  L5_Loaded = 1U;
  return BSP_ERROR_NONE;
}

int32_t BSP_RANGING_VL53L5CX_DeInit(void)
{
  if (L5_BusUp == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }
  if (L5_Ranging != 0U)
  {
    (void)vl53l5cx_stop_ranging(&L5_Dev);
  }
  return L5_Release();
}

int32_t BSP_RANGING_VL53L5CX_ReadID(uint32_t *pId)
{
  uint8_t status = VL53L5CX_STATUS_OK;
  uint8_t device = 0U;
  uint8_t revision = 0U;

  if (pId == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  *pId = 0U;
  if (L5_Loaded == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  /* Page 0 holds the identity; the driver works on page 2. */
  status |= WrByte(&L5_Dev.platform, 0x7FFFU, 0x00U);
  status |= RdByte(&L5_Dev.platform, 0x0000U, &device);
  status |= RdByte(&L5_Dev.platform, 0x0001U, &revision);
  status |= WrByte(&L5_Dev.platform, 0x7FFFU, 0x02U);
  if (status != VL53L5CX_STATUS_OK)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }
  *pId = ((uint32_t)device << 8) | revision;
  return BSP_ERROR_NONE;
}

int32_t BSP_RANGING_VL53L5CX_Start(uint32_t Zones, uint32_t RateHz)
{
  const uint32_t max_hz = (Zones == BSP_RANGING_VL53L5CX_ZONES_4X4) ? 60U : 15U;
  uint8_t        status = VL53L5CX_STATUS_OK;

  if (L5_Loaded == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }
  if (L5_Ranging != 0U)
  {
    return BSP_ERROR_BUSY;
  }
  if (((Zones != BSP_RANGING_VL53L5CX_ZONES_4X4) && (Zones != BSP_RANGING_VL53L5CX_ZONES_8X8)) ||
      (RateHz == 0U) || (RateHz > max_hz))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* The rate's limit depends on the zones, so they go first. */
  status |= vl53l5cx_set_resolution(&L5_Dev, (uint8_t)Zones);
  status |= vl53l5cx_set_ranging_mode(&L5_Dev, VL53L5CX_RANGING_MODE_CONTINUOUS);
  status |= vl53l5cx_set_ranging_frequency_hz(&L5_Dev, (uint8_t)RateHz);
  status |= vl53l5cx_start_ranging(&L5_Dev);
  if (status != VL53L5CX_STATUS_OK)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }
  L5_Zones   = Zones;
  L5_Pending = 0U;
  L5_Ranging = 1U;
  (void)L5_IntSeen(1U);
  return BSP_ERROR_NONE;
}

int32_t BSP_RANGING_VL53L5CX_Stop(void)
{
  if (L5_Ranging == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }
  L5_Ranging = 0U;
  L5_Pending = 0U;
  return (vl53l5cx_stop_ranging(&L5_Dev) == VL53L5CX_STATUS_OK) ? BSP_ERROR_NONE :
         BSP_ERROR_COMPONENT_FAILURE;
}

int32_t BSP_RANGING_VL53L5CX_IsReady(uint8_t *pReady)
{
  if (pReady == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  *pReady = 0U;
  if (L5_Ranging == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }
  if (L5_Pending != 0U)
  {
    *pReady = 1U;
    return BSP_ERROR_NONE;
  }

#if defined(BSP_RANGING_VL53L5CX_INT_PORT) && defined(BSP_RANGING_VL53L5CX_INT_PIN)
  *pReady = (uint8_t)L5_IntSeen(0U);
  return BSP_ERROR_NONE;
#else
  if (vl53l5cx_check_data_ready(&L5_Dev, pReady) != VL53L5CX_STATUS_OK)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }
  L5_Pending = *pReady;
  return BSP_ERROR_NONE;
#endif
}

int32_t BSP_RANGING_VL53L5CX_GetResult(BSP_RANGING_VL53L5CX_Result_t *pResult)
{
  uint8_t ready = L5_Pending;

  if (pResult == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (L5_Ranging == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  /* No INT fall since the last read: nothing new, and no bus access. Cleared before
   * the check, so a result that lands during this read leaves INT set for the next. */
  if ((ready == 0U) && (L5_IntSeen(1U) == 0U))
  {
    return BSP_ERROR_BUSY;
  }

  /* The part's stream count says whether the result is new. */
  if ((ready == 0U) && (vl53l5cx_check_data_ready(&L5_Dev, &ready) != VL53L5CX_STATUS_OK))
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }
  if (ready == 0U)
  {
    return BSP_ERROR_BUSY;
  }
  L5_Pending = 0U;

  if (vl53l5cx_get_ranging_data(&L5_Dev, &L5_Data) != VL53L5CX_STATUS_OK)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  (void)memset(pResult, 0, sizeof(*pResult));
  pResult->Zones        = L5_Zones;
  pResult->TemperatureC = L5_Data.silicon_temp_degc;
  for (uint32_t z = 0U; z < L5_Zones; z++)
  {
    const uint8_t status = L5_Data.target_status[z];

    pResult->Status[z]   = status;
    pResult->Distance[z] = ((status == 5U) || (status == 9U)) ? (uint16_t)L5_Data.distance_mm[z] : 0U;
  }
  return BSP_ERROR_NONE;
}
