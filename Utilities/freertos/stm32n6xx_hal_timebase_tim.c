/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32n6xx_hal_timebase_tim.c
  * @brief   HAL time base based on the hardware TIM.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32n6xx_hal.h"
#include "stm32n6xx_hal_tim.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef        htim16;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function configures the TIM16 as a time base source.
  *         The time source is configured  to have 1ms time base with a dedicated
  *         Tick interrupt priority.
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock = 0;
  uint32_t              uwPrescalerValue = 0;

  /*Configure the TIM16 IRQ priority */
  HAL_NVIC_SetPriority(TIM16_IRQn, TickPriority ,0);
  /* Enable the TIM16 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM16_IRQn);

  /* Enable TIM16 clock */
  __HAL_RCC_TIM16_CLK_ENABLE();

  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig);

  /* Compute TIM16 clock: the timer-group clock, which is not PCLK2 here */
  uwTimclock = HAL_RCCEx_GetTIMGFreq();

  /* Compute the prescaler value to have TIM16 counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

  /* Initialize TIM16 */
  htim16.Instance = TIM16;

  /* Initialize TIMx peripheral as follow:
   * Period = [(TIM16CLK/1000) - 1]. to have a (1/1000) s time base.
   * Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
   * ClockDivision = 0
   * Counter direction = Up
   */
  htim16.Init.Period = (1000000U / 1000U) - 1U;
  htim16.Init.Prescaler = uwPrescalerValue;
  htim16.Init.ClockDivision = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;

  if(HAL_TIM_Base_Init(&htim16) == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    return HAL_TIM_Base_Start_IT(&htim16);
  }

  /* Return function status */
  return HAL_ERROR;
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called when TIM16 interrupt took place, inside
  *         HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to
  *         increment a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM16) {
    HAL_IncTick();
  }
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM16 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Disable TIM16 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim16, TIM_IT_UPDATE);
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TIM16 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
  /* Enable TIM16 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim16, TIM_IT_UPDATE);
}

/* HAL_GetTick override. Before the scheduler starts it returns uwTick, which
 * TIM16 advances, so HAL_Delay works during initialisation. Once the scheduler
 * runs it follows the FreeRTOS tick, driven by SysTick at configTICK_RATE_HZ =
 * 1000, the same 1 ms unit HAL expects, so the HAL and the kernel keep one
 * time. This strong definition overrides the __weak HAL_GetTick in
 * stm32n6xx_hal.c.
 *
 * Ohm Lab: two changes. The kernel tick starts again from zero when the
 * scheduler starts, while uwTick has counted since reset, so a time taken
 * before the scheduler and compared after it would run backwards: the
 * kernel tick is offset to carry on from uwTick. And the kernel pends its
 * ticks while the scheduler is suspended, and this BSP suspends it for the
 * length of an I2C transaction (BSP_I2C_TransactionBegin), so inside one the
 * kernel tick stands still and a HAL timeout could never expire - a wedged
 * bus would hang the task instead of returning an error. The gap is bridged
 * from the cycle counter: while the kernel tick has not moved, the
 * milliseconds since it last did are added from DWT_CYCCNT. When the
 * scheduler resumes and the pended ticks land, the two agree to within a
 * millisecond. The BSP's test console checks both (`rtos`). */
#include "FreeRTOS.h"
#include "task.h"

uint32_t HAL_GetTick(void)
{
  static uint32_t seen_tick, seen_cyc, cyc_per_ms, base;
  uint32_t ticks, cyc, now, primask;

  /* taskSCHEDULER_NOT_STARTED until vTaskStartScheduler runs. */
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
    return uwTick;  /* pre-scheduler fallback */
  }

  /* The anchor is shared by every caller. A caller preempted between the
   * tick comparison and the subtraction, by one that moves the anchor on,
   * subtracts a newer count from its older one, and the wrapped difference
   * reads as seconds: a HAL timeout polling through that call expires at
   * once. So the anchor is read and moved with interrupts off. */
  primask = __get_PRIMASK();
  __disable_irq();

  if (xPortIsInsideInterrupt() == pdTRUE) {
    ticks = (uint32_t)xTaskGetTickCountFromISR();
  } else {
    ticks = (uint32_t)xTaskGetTickCount();
  }
  if (cyc_per_ms == 0U) {
    /* First call under the scheduler: start the cycle counter. The CPU clock
     * is read back, not assumed, as everything in this BSP does. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
    cyc_per_ms = HAL_RCC_GetCpuClockFreq() / 1000U;
    if (cyc_per_ms == 0U) { cyc_per_ms = 1U; }
    /* Anchor the extrapolation here. The counter may already be running from
     * before the scheduler, and measured from zero it would add that whole
     * time to every call until the kernel tick first moves, after which the
     * tick would appear to run backwards. */
    seen_tick = ticks;
    seen_cyc  = DWT->CYCCNT;
    now       = ticks;
    /* uwTick and the kernel tick have both counted milliseconds since the
     * scheduler started, so their difference is uwTick at that moment. */
    base      = uwTick - ticks;
  } else {
    cyc = DWT->CYCCNT;
    if (ticks != seen_tick) {
      seen_tick = ticks;
      seen_cyc  = cyc;
      now       = ticks;
    } else {
      now = ticks + (cyc - seen_cyc) / cyc_per_ms;
    }
  }

  __set_PRIMASK(primask);
  return base + now;
}

