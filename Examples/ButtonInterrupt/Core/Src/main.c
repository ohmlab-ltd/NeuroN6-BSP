/*
  Button interrupt

  Counts presses of the on-board USER button with an interrupt, so the count
  is right even while loop() is busy. The button is on EXTI line 8; the BSP
  configures the line and its priority, the application only the callback.

  The circuit:
  - Uses the on-board USER button. No wiring needed.

  Send 'b' on the USB console at any time to return to the bootloader.

  Copyright (c) 2026 Ohm Lab Ltd.
  SPDX-License-Identifier: CC0-1.0
  Released under CC0 1.0 Universal: copy it into your own project
  freely, with or without attribution.
*/

#include "main.h"
#include "neuron6.h"
#include "neuron6_com.h"
#include "neuron6_wdg.h"

/* Provided by the scaffolding at the end of this file. */
static inline void     delay(uint32_t ms);
static inline uint32_t millis(void);
static void            setup(void);
static void            loop(void);

/* Formatted output to the USB console. Dropped, not blocked, with no host. */
#define print(...)   (void)BSP_COM_Printf(__VA_ARGS__)

/* Written by the interrupt, read by loop(), so it must be volatile. */
static volatile uint32_t pressCount = 0U;

/* The BSP calls this from the button's interrupt handler. */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    pressCount++;
  }
}

static void setup(void)
{
  (void)BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
}

static void loop(void)
{
  static uint32_t lastReported = 0U;
  uint32_t now = pressCount;

  if (now != lastReported)
  {
    lastReported = now;
    print("presses: %lu\r\n", (unsigned long)now);
  }
  delay(50U);
}

/* Board scaffolding, the same in every example. It keeps the board reachable
 * over USB: the watchdog is armed first, its early warning and its reset both
 * lead to the bootloader, and every wait services USB, refreshes the watchdog
 * and takes 'b' as a reboot into the bootloader. The fault handlers are in
 * stm32n6xx_it.c. */

#define WDG_TIMEOUT_MS   4000U   /* reset if delay() is not called for this long */
#define WDG_WARNING_MS    300U   /* the early warning fires this long before it  */

static void mpu_config(void);

/* One pass of the housekeeping: the USB stack, the watchdog, and 'b'. */
static void service(void)
{
  uint8_t ch;

  BSP_COM_Task();
  (void)BSP_WDG_Refresh();
  if ((BSP_COM_Read(&ch, 1U) == 1) && (ch == (uint8_t)'b'))
  {
    BSP_COM_RebootToBootloader();
  }
}

/* Wait, staying reachable. Inline, like millis(), so an example that uses
 * neither gets no unused-function warning. */
static inline void delay(uint32_t ms)
{
  uint32_t t0 = HAL_GetTick();

  do
  {
    service();
  } while ((HAL_GetTick() - t0) < ms);
}

/* Milliseconds since boot. */
static inline uint32_t millis(void)
{
  return HAL_GetTick();
}

/* The watchdog is about to reset the board: go to the bootloader instead. */
void BSP_WDG_EWI_Callback(void)
{
  BSP_COM_RebootToBootloader();
}

int main(void)
{
  SCB_DisableDCache();
  mpu_config();
  SCB_EnableICache();
  SCB_EnableDCache();
  HAL_Init();

  (void)BSP_LED_Init(LED_RED);

  /* Armed before anything that could fail. A watchdog reset goes to the
   * bootloader. */
  (void)BSP_WDG_InitEWI(WDG_TIMEOUT_MS, WDG_WARNING_MS);
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U)
  {
    __HAL_RCC_CLEAR_RESET_FLAGS();
    BSP_COM_RebootToBootloader();
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* USB last, immediately before the loop that services it. */
  if (BSP_COM_Init() != BSP_ERROR_NONE)
  {
    (void)BSP_LED_On(LED_RED);
    Error_Handler();
  }

  setup();
  for (;;)
  {
    service();
    loop();
  }
}

void Error_Handler(void)
{
  /* Interrupts off: the watchdog resets the board, and main sends that reset
   * to the bootloader. */
  __disable_irq();
  for (;;)
  {
  }
}

static void mpu_config(void)
{
  MPU_Region_InitTypeDef     default_config = {0};
  MPU_Attributes_InitTypeDef attr_config    = {0};
  uint32_t primask_bit = __get_PRIMASK();

  __disable_irq();
  HAL_MPU_Disable();

  attr_config.Attributes = INNER_OUTER(MPU_NOT_CACHEABLE);
  attr_config.Number     = MPU_ATTRIBUTES_NUMBER0;
  HAL_MPU_ConfigMemoryAttributes(&attr_config);

  default_config.Enable           = MPU_REGION_ENABLE;
  default_config.Number           = MPU_REGION_NUMBER0;
  default_config.BaseAddress      = __NON_CACHEABLE_SECTION_BEGIN;
  default_config.LimitAddress     = __NON_CACHEABLE_SECTION_END;
  default_config.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  default_config.AccessPermission = MPU_REGION_ALL_RW;
  default_config.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  default_config.AttributesIndex  = MPU_ATTRIBUTES_NUMBER0;
  HAL_MPU_ConfigRegion(&default_config);

  default_config.Number       = MPU_REGION_NUMBER1;
  default_config.BaseAddress  = 0x70000000;
  default_config.LimitAddress = 0x71FFFFFF;
  HAL_MPU_ConfigRegion(&default_config);

  extern uint8_t __axisram_nc_start[];
  extern uint8_t __axisram_nc_end[];
  if ((uint32_t)__axisram_nc_end > (uint32_t)__axisram_nc_start)
  {
    default_config.Number       = MPU_REGION_NUMBER2;
    default_config.BaseAddress  = (uint32_t)__axisram_nc_start;
    default_config.LimitAddress = (uint32_t)__axisram_nc_end - 1u;
    HAL_MPU_ConfigRegion(&default_config);
  }

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
  __set_PRIMASK(primask_bit);
}
