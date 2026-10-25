/*
  Speaker tone

  Plays a tone of close to 1 kHz on the speaker for one second, then one
  second of silence, repeatedly. The looped buffer holds a whole number of
  sine cycles, so the tone is continuous and free of clicks.

  The circuit:
  - Nothing to wire: the speaker is on the board.

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

/* Provided by the scaffolding at the end of this file. */
static inline void     delay(uint32_t ms);
static inline uint32_t millis(void);
static void            setup(void);
static void            loop(void);

/* Formatted output to the USB console. Dropped, not blocked, with no host. */
#define print(...)   (void)BSP_COM_Printf(__VA_ARGS__)

#define FRAMES   2048U                  /* stereo frames in the buffer   */
#define CYCLES   42U                    /* whole sine cycles it holds    */

static int16_t buffer[FRAMES * 2U] __attribute__((aligned(32)));

static int32_t  speaker;          /* how playback came up: BSP_ERROR_NONE if it did */
static uint32_t rate = 0U;        /* the sample rate it plays at                    */
static uint8_t  told = 0U;        /* 1 once the console has been told it            */

/* The BSP calls these as the DMA works through the buffer; nothing to do. */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Instance)     { (void)Instance; }
void BSP_AUDIO_OUT_TransferComplete_CallBack(uint32_t Instance) { (void)Instance; }
void BSP_AUDIO_OUT_Error_CallBack(uint32_t Instance)            { (void)Instance; }

static void setup(void)
{
  BSP_AUDIO_Init_t init = { BSP_AUDIO_OUT_DEVICE_SPEAKER, BSP_AUDIO_FREQUENCY_NATIVE,
                            BSP_AUDIO_RESOLUTION_16B, 2U, 50U };

  for (uint32_t i = 0U; i < FRAMES; i++)
  {
    int16_t v = (int16_t)(6000.0f * sinf(6.2831853f * (float)CYCLES * (float)i / (float)FRAMES));
    buffer[2U * i]      = v;   /* left  */
    buffer[2U * i + 1U] = v;   /* right */
  }

  speaker = BSP_AUDIO_OUT_Init(0, &init);
  if (speaker == BSP_ERROR_NONE)
  {
    speaker = BSP_AUDIO_OUT_GetSampleRate(0, &rate);
  }
  if (speaker == BSP_ERROR_NONE)
  {
    speaker = BSP_AUDIO_OUT_Play(0, (uint8_t *)buffer, sizeof buffer);
  }
}

static void loop(void)
{
  if (speaker != BSP_ERROR_NONE)
  {
    print("speaker error %ld\r\n", (long)speaker);
    delay(1000U);
    return;
  }

  /* Once a terminal has the console open: earlier output reaches nobody. */
  if ((told == 0U) && (BSP_COM_IsConnected() != 0))
  {
    print("sample rate %lu Hz, tone %lu Hz\r\n",
          (unsigned long)rate, (unsigned long)((rate * CYCLES) / FRAMES));
    told = 1U;
  }

  (void)BSP_AUDIO_OUT_UnMute(0);
  print("tone\r\n");
  delay(1000U);
  (void)BSP_AUDIO_OUT_Mute(0);
  print("silence\r\n");
  delay(1000U);
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
