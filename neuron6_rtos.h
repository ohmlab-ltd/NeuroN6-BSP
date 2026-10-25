/**
  * @file    neuron6_rtos.h
  * @brief   FreeRTOS integration for the Neuro N6 BSP.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  *
  * At BSP_USE_FREERTOS 1: the BSP lock mutexes, the newlib malloc lock, the kernel hooks,
  * a yielding HAL_Delay and the USB device task (at 0, nothing). The kernel owns SysTick,
  * so compile Utilities/freertos/stm32n6xx_hal_timebase_tim.c (TIM16 time base) and route
  * TIM16_IRQHandler to HAL_TIM_IRQHandler(&htim16).
  */

#ifndef NEURON6_RTOS_H
#define NEURON6_RTOS_H

#include "neuron6_conf.h"
#include "neuron6_errno.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(BSP_USE_FREERTOS) && (BSP_USE_FREERTOS == 1)

#include "FreeRTOS.h"
#include "task.h"

/** @brief Stack of the USB device task, in words (4 KB). */
#ifndef BSP_RTOS_USB_TASK_STACK_WORDS
#define BSP_RTOS_USB_TASK_STACK_WORDS   1024U
#endif

/** @brief USB device task priority: above application workers, below the ceiling. */
#ifndef BSP_RTOS_USB_TASK_PRIORITY
#define BSP_RTOS_USB_TASK_PRIORITY      (tskIDLE_PRIORITY + 3U)
#endif

/**
  * @brief  Start the USB device task (BSP_COM_Task, which keeps the console's 'b' and the
  *         1200-baud touch working); 0 selects the default stack or priority.
  * @retval BSP_ERROR_BUSY if it already exists, BSP_ERROR_NO_INIT if allocation failed.
  */
int32_t BSP_RTOS_StartUsbTask(uint16_t StackWords, uint32_t Priority);

/** @brief The USB device task's handle; NULL before BSP_RTOS_StartUsbTask. */
TaskHandle_t BSP_RTOS_GetUsbTask(void);

/**
  * @brief  Weak, resets by default: called on a stack overflow (pDetail = task name) or a
  *         failed configASSERT (pDetail = NULL).
  * @note   Runs in whatever context failed, so it must not print, lock or wait.
  */
void BSP_RTOS_FatalHook(const char *pWhat, const char *pDetail);

#endif /* BSP_USE_FREERTOS */

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_RTOS_H */
