/**
  * @file    neuron6_rtos.c
  * @brief   FreeRTOS integration for the Neuro N6 BSP.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6_rtos.h"

#if defined(BSP_USE_FREERTOS) && (BSP_USE_FREERTOS == 1)

#include <reent.h>

#include "neuron6.h"
#include "neuron6_com.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* One recursive mutex per BSP lock pair: static storage (no kernel heap), created on
 * first use with the scheduler suspended so two first callers cannot race. */
typedef struct
{
  SemaphoreHandle_t  Handle;
  StaticSemaphore_t  Storage;
} RTOS_Lock_t;

static RTOS_Lock_t s_lock_i2c1;
static RTOS_Lock_t s_lock_i2c2;
static RTOS_Lock_t s_lock_spi2;
static RTOS_Lock_t s_lock_adc;
static RTOS_Lock_t s_lock_wifi;

static TaskHandle_t s_usb_task;

/* The kernel's idle and timer task memory (configSUPPORT_STATIC_ALLOCATION is 1). */
static StaticTask_t s_idle_tcb;
static StackType_t  s_idle_stack[configMINIMAL_STACK_SIZE];

#if (configUSE_TIMERS == 1)
static StaticTask_t s_timer_tcb;
static StackType_t  s_timer_stack[configTIMER_TASK_STACK_DEPTH];
#endif

/* No-op in interrupts and before the scheduler starts (vTaskSuspendAll must not run). */
static void RTOS_LockTake(RTOS_Lock_t *pLock)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) { return; }
  if (xPortIsInsideInterrupt() != pdFALSE)                    { return; }

  if (pLock->Handle == NULL)
  {
    vTaskSuspendAll();
    if (pLock->Handle == NULL)
    {
      pLock->Handle = xSemaphoreCreateRecursiveMutexStatic(&pLock->Storage);
    }
    (void)xTaskResumeAll();
  }

  if (pLock->Handle != NULL)
  {
    (void)xSemaphoreTakeRecursive(pLock->Handle, portMAX_DELAY);
  }
}

/* Same no-op rules as RTOS_LockTake, so the two always balance. */
static void RTOS_LockGive(RTOS_Lock_t *pLock)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) { return; }
  if (xPortIsInsideInterrupt() != pdFALSE)                    { return; }

  if (pLock->Handle != NULL)
  {
    (void)xSemaphoreGiveRecursive(pLock->Handle);
  }
}

/* Sleeps until the USB interrupt reports an event; the 1 ms fallback serves paths with
 * no interrupt (the 1200-baud touch's port-close detection, a console 'b'). */
static void RTOS_UsbTask(void *pArg)
{
  (void)pArg;

  for (;;)
  {
    BSP_COM_TaskWait(1U);
  }
}

int32_t BSP_RTOS_StartUsbTask(uint16_t StackWords, uint32_t Priority)
{
  if (s_usb_task != NULL)
  {
    return BSP_ERROR_BUSY;
  }

  if (StackWords == 0U) { StackWords = (uint16_t)BSP_RTOS_USB_TASK_STACK_WORDS; }
  if (Priority == 0U)   { Priority   = (uint32_t)BSP_RTOS_USB_TASK_PRIORITY;    }

  if (xTaskCreate(RTOS_UsbTask, "bsp_usb", StackWords, NULL,
                  (UBaseType_t)Priority, &s_usb_task) != pdPASS)
  {
    s_usb_task = NULL;
    return BSP_ERROR_NO_INIT;
  }

  BSP_COM_SetPumpTask(s_usb_task);

  return BSP_ERROR_NONE;
}

TaskHandle_t BSP_RTOS_GetUsbTask(void)
{
  return s_usb_task;
}

__weak void BSP_RTOS_FatalHook(const char *pWhat, const char *pDetail)
{
  (void)pWhat;
  (void)pDetail;

  __disable_irq();
  __DSB();
  NVIC_SystemReset();
  for (;;) { }
}

/* The BSP's weak lock hooks, as recursive mutexes. */

void BSP_I2C1_Lock(void)   { RTOS_LockTake(&s_lock_i2c1); }
void BSP_I2C1_Unlock(void) { RTOS_LockGive(&s_lock_i2c1); }
void BSP_I2C2_Lock(void)   { RTOS_LockTake(&s_lock_i2c2); }
void BSP_I2C2_Unlock(void) { RTOS_LockGive(&s_lock_i2c2); }
void BSP_SPI2_Lock(void)   { RTOS_LockTake(&s_lock_spi2); }
void BSP_SPI2_Unlock(void) { RTOS_LockGive(&s_lock_spi2); }
void BSP_ADC_Lock(void)    { RTOS_LockTake(&s_lock_adc);  }
void BSP_ADC_Unlock(void)  { RTOS_LockGive(&s_lock_adc);  }
void BSP_WIFI_Lock(void)   { RTOS_LockTake(&s_lock_wifi); }
void BSP_WIFI_Unlock(void) { RTOS_LockGive(&s_lock_wifi); }

/* One I2C transaction without a task switch. */

void BSP_I2C_TransactionBegin(void)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) { return; }
  if (xPortIsInsideInterrupt() != pdFALSE)                    { return; }

  vTaskSuspendAll();
}

void BSP_I2C_TransactionEnd(void)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) { return; }
  if (xPortIsInsideInterrupt() != pdFALSE)                    { return; }

  (void)xTaskResumeAll();
}

/* newlib's malloc locks are weak no-ops; concurrent allocations corrupt the heap. */
void __malloc_lock(struct _reent *pReent)
{
  (void)pReent;
  configASSERT(xPortIsInsideInterrupt() == pdFALSE);
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    vTaskSuspendAll();
  }
}

void __malloc_unlock(struct _reent *pReent)
{
  (void)pReent;
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    (void)xTaskResumeAll();
  }
}

/* The HAL's HAL_Delay spins and starves lower-priority tasks; this one yields. */
void HAL_Delay(uint32_t Delay)
{
  if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) &&
      (xPortIsInsideInterrupt() == pdFALSE))
  {
    /* 64-bit: pdMS_TO_TICKS wraps past 71 minutes. One tick more, as the HAL's own
     * HAL_Delay waits, except on the longest delay, which it would wrap to none. */
    uint64_t ticks = ((uint64_t)Delay * (uint64_t)configTICK_RATE_HZ) / 1000U;

    if (ticks > (uint64_t)portMAX_DELAY)
    {
      ticks = (uint64_t)portMAX_DELAY;
    }
    if (ticks < (uint64_t)portMAX_DELAY)
    {
      ticks++;
    }
    vTaskDelay((TickType_t)ticks);
    return;
  }

  {
    uint32_t tickstart = HAL_GetTick();
    uint32_t wait      = Delay;

    if (wait < HAL_MAX_DELAY)
    {
      wait += (uint32_t)uwTickFreq;
    }
    while ((HAL_GetTick() - tickstart) < wait)
    {
      /* spin: no scheduler to hand the time to */
    }
  }
}

/* Kernel hooks */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  BSP_RTOS_FatalHook("stack overflow", pcTaskName);
}

/* Called by name from FreeRTOSConfig.h's configASSERT, in whatever context asserted. */
void n6_configassert_fail(void)
{
  BSP_RTOS_FatalHook("configASSERT", NULL);
}

/* Weak and empty, so the idle task spins. Before each __WFI() here, copy each RCC enable
 * register to its sleep-enable set register (RCC->MEMLPENSR = RCC->MEMENR, and so on for
 * BUS, MISC, AHB1-5, APB1-5), or Sleep stops nearly every peripheral and memory. */
__weak void vApplicationIdleHook(void)
{
}

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t  **ppxIdleTaskStackBuffer,
                                   uint32_t      *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer   = &s_idle_tcb;
  *ppxIdleTaskStackBuffer = s_idle_stack;
  *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

#if (configUSE_TIMERS == 1)
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t  **ppxTimerTaskStackBuffer,
                                    uint32_t      *pulTimerTaskStackSize)
{
  *ppxTimerTaskTCBBuffer   = &s_timer_tcb;
  *ppxTimerTaskStackBuffer = s_timer_stack;
  *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}
#endif

#else /* BSP_USE_FREERTOS */

/* Bare metal: nothing here; the typedef stops a pedantic compiler objecting. */
typedef int neuron6_rtos_not_configured_t;

#endif /* BSP_USE_FREERTOS */
