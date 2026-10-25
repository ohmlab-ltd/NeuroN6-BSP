/*
  Camera on the display

  Shows the camera on the 800 x 480 panel. The capture pipe writes RGB565
  straight into the frame buffer the display controller scans out, so nothing
  copies or converts between them.

  The circuit:
  - Needs the display carrier with its panel and camera module.

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
#include "neuron6_camera.h"
#include "neuron6_lcd.h"

/* Provided by the scaffolding at the end of this file. */
static inline void     delay(uint32_t ms);
static inline uint32_t millis(void);
static void            setup(void);
static void            loop(void);

/* Formatted output to the USB console. Dropped, not blocked, with no host. */
#define print(...)   (void)BSP_COM_Printf(__VA_ARGS__)

/* 800 x 480 pixels at 16 bits, in external PSRAM: the capture pipe writes
 * it and the display controller scans it out. */
static uint8_t frameBuffer[BSP_LCD_WIDTH * BSP_LCD_HEIGHT * 2U]
    __attribute__((section(".ext_psram"), aligned(32)));

static volatile uint32_t frames;
static uint32_t reported;

void BSP_CAMERA_FrameEventCallback(uint32_t Instance, uint32_t Pipe)
{
  if ((Instance == 0U) && (Pipe == BSP_CAMERA_PIPE_DUMP))
  {
    frames++;
  }
}

static int32_t startCamera(void)
{
  BSP_CAMERA_PipeConfig_t pipe = {0};
  int32_t status;

  /* The display carrier crosses the camera's data pair over. */
  status = BSP_CAMERA_SetCarrier(BSP_CAMERA_CARRIER_INVERTED);
  if (status == BSP_ERROR_NONE)
  {
    status = BSP_CAMERA_Init(0, BSP_CAMERA_R800x480, BSP_CAMERA_PF_RGB565);
  }
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  pipe.PixelFormat = BSP_CAMERA_PIPE_PF_RGB565;
  pipe.Width       = BSP_LCD_WIDTH;
  pipe.Height      = BSP_LCD_HEIGHT;
  status = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_DUMP, &pipe);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  return BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_DUMP, frameBuffer,
                          BSP_CAMERA_MODE_CONTINUOUS);
}

static void setup(void)
{
  BSP_LCD_LayerConfig_t layer = {0};

  (void)BSP_LCD_Init(0, BSP_LCD_WIDTH, BSP_LCD_HEIGHT);

  layer.X0 = 0U;
  layer.X1 = BSP_LCD_WIDTH;
  layer.Y0 = 0U;
  layer.Y1 = BSP_LCD_HEIGHT;
  layer.PixelFormat = BSP_LCD_PIXEL_FORMAT_RGB565;
  layer.Address = (uint32_t)frameBuffer;
  layer.Alpha = 255U;                  /* opaque: 0 would show only the black behind it */
  (void)BSP_LCD_ConfigLayer(0, BSP_LCD_LAYER_0, &layer);
  (void)BSP_LCD_SetActiveLayer(0, BSP_LCD_LAYER_0);
  (void)BSP_LCD_DisplayOn(0);

  print("camera %ld\r\n", (long)startCamera());
}

static void loop(void)
{
  uint32_t seen;

  delay(1000U);
  seen = frames;
  print("%lu frames a second\r\n", (unsigned long)(seen - reported));

  /* A bring-up can leave the link up with no frame arriving: restart it. */
  if (seen == reported)
  {
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_DUMP);
    (void)BSP_CAMERA_DeInit(0);
    print("no frame, restarted: %ld\r\n", (long)startCamera());
  }
  reported = seen;
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
