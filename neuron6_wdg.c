/**
  * @file    neuron6_wdg.c
  * @brief   Independent watchdog (IWDG) support for the Neuro N6.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6_wdg.h"

#define WDG_RELOAD_MAX                  ((uint32_t)IWDG_RLR_RL)

#define WDG_TICKS_MAX                   (WDG_RELOAD_MAX + 1U)

static uint32_t WDG_GetLsiFreq(void);
static int32_t  WDG_Start(uint32_t Timeout_ms, uint32_t EarlyWarning_ms);

static IWDG_HandleTypeDef hbsp_iwdg;

/* Never clears: the IWDG cannot be stopped. */
static uint8_t wdg_started = 0U;

/* Finest to coarsest; WDG_PRESCALER_DIV matches entry for entry. */
static const uint32_t WDG_PRESCALER_CODE[] =
{
  IWDG_PRESCALER_4,
  IWDG_PRESCALER_8,
  IWDG_PRESCALER_16,
  IWDG_PRESCALER_32,
  IWDG_PRESCALER_64,
  IWDG_PRESCALER_128,
  IWDG_PRESCALER_256,
  IWDG_PRESCALER_512,
  IWDG_PRESCALER_1024,
};

static const uint32_t WDG_PRESCALER_DIV[] =
{
  4U, 8U, 16U, 32U, 64U, 128U, 256U, 512U, 1024U,
};

#define WDG_PRESCALER_COUNT  (sizeof(WDG_PRESCALER_DIV) / sizeof(WDG_PRESCALER_DIV[0]))

static uint32_t WDG_GetLsiFreq(void)
{
  return (uint32_t)LSI_VALUE;
}

/* EarlyWarning_ms: lead time before the reset, 0 = no interrupt. Uses the finest
 * prescaler whose reload fits. */
static int32_t WDG_Start(uint32_t Timeout_ms, uint32_t EarlyWarning_ms)
{
  uint32_t lsi_hz = WDG_GetLsiFreq();
  uint32_t index;
  uint32_t reload = 0U;
  uint32_t ewi = (uint32_t)IWDG_EWI_DISABLE;
  uint8_t  found = 0U;

  if ((Timeout_ms == 0U) || (lsi_hz == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  for (index = 0U; index < WDG_PRESCALER_COUNT; index++)
  {
    /* 64-bit: the numerator overflows 32 bits at the longest timeouts. */
    uint64_t ticks = ((uint64_t)Timeout_ms * (uint64_t)lsi_hz)
                     / (1000ULL * (uint64_t)WDG_PRESCALER_DIV[index]);

    if (ticks == 0ULL)
    {
      /* Shorter than one tick of the finest prescaler. */
      return BSP_ERROR_WRONG_PARAM;
    }

    if (ticks <= (uint64_t)WDG_TICKS_MAX)
    {
      reload = (uint32_t)ticks - 1U;
      found = 1U;
      break;
    }
  }

  if (found == 0U)
  {
    /* Longer than the counter can express even divided by 1024. */
    return BSP_ERROR_WRONG_PARAM;
  }

  if (EarlyWarning_ms != 0U)
  {
    uint64_t ewi_ticks = ((uint64_t)EarlyWarning_ms * (uint64_t)lsi_hz)
                         / (1000ULL * (uint64_t)WDG_PRESCALER_DIV[index]);

    /* The comparator matches the down-counter: ticks left when the interrupt fires. */
    if ((ewi_ticks == 0ULL) || (ewi_ticks >= (uint64_t)reload))
    {
      return BSP_ERROR_WRONG_PARAM;
    }

    ewi = (uint32_t)ewi_ticks;
  }

  hbsp_iwdg.Instance       = IWDG;
  hbsp_iwdg.Init.Prescaler = WDG_PRESCALER_CODE[index];
  hbsp_iwdg.Init.Reload    = reload;
  hbsp_iwdg.Init.Window    = IWDG_WINDOW_DISABLE;
  hbsp_iwdg.Init.EWI       = ewi;

  /* HAL_IWDG_Init starts the counter and turns the LSI on by itself. */
  if (HAL_IWDG_Init(&hbsp_iwdg) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

#if (USE_BSP_WDG_EWI == 1)
  /* HAL_IWDG_Init arms the warning in the IWDG only; the NVIC line is ours. */
  if (ewi != 0U)
  {
    HAL_NVIC_SetPriority(IWDG_IRQn, BSP_WDG_IT_PRIORITY, 0x00U);
    HAL_NVIC_EnableIRQ(IWDG_IRQn);
  }
  else
  {
    HAL_NVIC_DisableIRQ(IWDG_IRQn);
  }
#endif /* USE_BSP_WDG_EWI == 1 */

  wdg_started = 1U;

  return BSP_ERROR_NONE;
}

/**
  * @brief  Start the IWDG; Timeout_ms from BSP_WDG_TIMEOUT_MIN_MS to BSP_WDG_TIMEOUT_MAX_MS.
  * @note   It cannot be stopped until the next reset. It runs from the LSI, through
  *         Standby too, so a sleeping application must wake to refresh it.
  *         The timeout is quantised and carries the LSI's tolerance.
  * @retval BSP_ERROR_WRONG_PARAM if the timeout cannot be expressed.
  */
int32_t BSP_WDG_Init(uint32_t Timeout_ms)
{
  return WDG_Start(Timeout_ms, 0U);
}

/** @brief Reload the counter, at least once per timeout. BSP_ERROR_NO_INIT if not started. */
int32_t BSP_WDG_Refresh(void)
{
  if (wdg_started == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (HAL_IWDG_Refresh(&hbsp_iwdg) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return BSP_ERROR_NONE;
}

#if (USE_BSP_WDG_EWI == 1)
/**
  * @brief  As BSP_WDG_Init, plus BSP_WDG_EWI_Callback EarlyWarning_ms before the reset
  *         (non-zero and shorter than Timeout_ms).
  * @note   The application defines IWDG_IRQHandler and calls BSP_WDG_IRQHandler. The
  *         interrupt runs at BSP_WDG_IT_PRIORITY (0), so the callback must not use the
  *         RTOS: no HAL_GetTick under FreeRTOS, no allocation. BSP_COM_RebootToBootloader,
  *         BSP_CAMERA_HaltCapture and BSP_VENC_Quiesce are safe there.
  */
int32_t BSP_WDG_InitEWI(uint32_t Timeout_ms, uint32_t EarlyWarning_ms)
{
  if (EarlyWarning_ms == 0U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  return WDG_Start(Timeout_ms, EarlyWarning_ms);
}

/** @brief Call from the application's IWDG_IRQHandler: clears the flag, runs the callback. */
void BSP_WDG_IRQHandler(void)
{
  HAL_IWDG_IRQHandler(&hbsp_iwdg);
}

/** @brief Weak; override to record why the refresh stopped. */
__weak void BSP_WDG_EWI_Callback(void)
{
}

/* The BSP owns the IWDG, so it takes the HAL's weak callback. */
void HAL_IWDG_EarlyWakeupCallback(IWDG_HandleTypeDef *hiwdg)
{
  if ((hiwdg != NULL) && (hiwdg->Instance == IWDG))
  {
    BSP_WDG_EWI_Callback();
  }
}
#endif /* USE_BSP_WDG_EWI == 1 */
