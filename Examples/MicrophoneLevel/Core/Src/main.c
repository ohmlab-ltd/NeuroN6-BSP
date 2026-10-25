/*
  Microphone level

  Records from the on-board microphone continuously and prints the sound
  level ten times a second as a bar, with the RMS and the peak of the most
  recent buffer. Talk, clap or tap the board and watch the bar move.

  The circuit:
  - Nothing to wire: the microphone is on the board.

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
#include "neuron6_audio.h"
#include <math.h>
#include <stdlib.h>

/* Provided by the scaffolding at the end of this file. */
static inline void     delay(uint32_t ms);
static inline uint32_t millis(void);
static void            setup(void);
static void            loop(void);

/* Formatted output to the USB console. Dropped, not blocked, with no host. */
#define print(...)   (void)BSP_COM_Printf(__VA_ARGS__)

#define SAMPLES  1024U                  /* the most the staging buffer holds */

static int16_t buffer[SAMPLES] __attribute__((aligned(32)));

/* Set by the BSP from interrupt context each time the buffer is full. */
static volatile uint32_t buffersDone = 0U;

void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)     { (void)Instance; }
void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance) { (void)Instance; buffersDone++; }
void BSP_AUDIO_IN_Error_CallBack(uint32_t Instance)            { (void)Instance; }

static int32_t  microphone;       /* how recording came up: BSP_ERROR_NONE if it did */
static uint32_t rate = 0U;        /* the sample rate it records at                   */
static uint8_t  told = 0U;        /* 1 once the console has been told it             */

static void setup(void)
{
  BSP_AUDIO_Init_t init = { BSP_AUDIO_IN_DEVICE_DIGITAL_MIC, BSP_AUDIO_FREQUENCY_NATIVE,
                            BSP_AUDIO_RESOLUTION_16B, 1U, 0U };

  microphone = BSP_AUDIO_IN_Init(0, &init);
  if (microphone == BSP_ERROR_NONE)
  {
    microphone = BSP_AUDIO_IN_GetSampleRate(0, &rate);
  }
  if (microphone == BSP_ERROR_NONE)
  {
    microphone = BSP_AUDIO_IN_Record(0, (uint8_t *)buffer, sizeof buffer);
  }
}

static void loop(void)
{
  double  sum  = 0.0;
  int32_t peak = 0;
  uint32_t rms, bars;
  char bar[41];

  if (microphone != BSP_ERROR_NONE)
  {
    print("microphone error %ld\r\n", (long)microphone);
    delay(1000U);
    return;
  }

  /* Once a terminal has the console open: earlier output reaches nobody. */
  if ((told == 0U) && (BSP_COM_IsConnected() != 0))
  {
    print("sample rate %lu Hz\r\n", (unsigned long)rate);
    told = 1U;
  }

  for (uint32_t i = 0U; i < SAMPLES; i++)
  {
    int32_t v = buffer[i];
    sum += (double)v * (double)v;
    if (abs(v) > peak) { peak = abs(v); }
  }
  rms  = (uint32_t)sqrt(sum / (double)SAMPLES);
  bars = (rms > 8000U) ? 40U : (rms / 200U);
  for (uint32_t i = 0U; i < 40U; i++) { bar[i] = (i < bars) ? '#' : ' '; }
  bar[40] = '\0';
  print("|%s| rms %5lu peak %5ld buffers %lu\r\n",
        bar, (unsigned long)rms, (long)peak, (unsigned long)buffersDone);
  delay(100U);
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
