/*
  Read thermal

  Reads the FLIR Lepton thermal camera, about nine frames a second, and
  prints once a second the temperature at the centre of the picture and of
  its coldest and hottest pixels, in degrees Celsius. Each pixel of this
  camera is a temperature, in hundredths of a kelvin.

  The circuit:
  - A Neuro N6 on the thermal carrier, with the Lepton 3.1R fitted. The
    carrier mounts the camera upside down, so every frame is rotated 180
    degrees.
  - PD3, the camera's power line, is the BLE flow control on other
    carriers. Nothing here uses BLE.

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
#include "neuron6_thermal.h"

/* Provided by the scaffolding at the end of this file. */
static inline void     delay(uint32_t ms);
static inline uint32_t millis(void);
static void            setup(void);
static void            loop(void);

/* Formatted output to the USB console. Dropped, not blocked, with no host. */
#define print(...)   (void)BSP_COM_Printf(__VA_ARGS__)

/* Video, written only by DMA: whole packet pairs and cache lines, as large
 * as one transfer can be. Read it within half its length (13 ms at the
 * camera's highest clock) or the stream skips to the next frame. */
#define VIDEO_BYTES  (49U * 8U * BSP_THERMAL_PACKET_BYTES)   /* 64288 */

static uint8_t  video[VIDEO_BYTES] __attribute__((aligned(32)));
static uint16_t frame[BSP_THERMAL_FRAME_PIXELS];

/* While booting, the camera is left alone and the loop keeps servicing USB. */
enum { CAMERA_OFF, CAMERA_BOOTING, CAMERA_RUNNING };
static int      camera     = CAMERA_OFF;
static uint32_t since      = 0U;   /* when it got there              */
static uint8_t  tried      = 0U;   /* 1 once a start has failed      */
static uint32_t lastFrame  = 0U;   /* when the last frame arrived    */
static uint32_t lastPrint  = 0U;
static uint32_t lastHealth = 0U;
static uint32_t frames     = 0U;   /* frames since the last print    */

/* The latest frame's readings, taken on arrival: the next frame is assembled
 * in the same buffer. */
static uint16_t centre, coldest, hottest;
static uint32_t hottestAt;

/* The thermal module installs no interrupt vectors: these route its video
 * DMA (GPDMA1 channel 1) and its link-error interrupt. */
void GPDMA1_Channel1_IRQHandler(void) { BSP_THERMAL_DMA_IRQHandler(0); }
void SPI1_IRQHandler(void)            { BSP_THERMAL_SPI_IRQHandler(0); }

/* Hundredths of a kelvin as degrees Celsius, to two places: the default C
 * library has no floating-point printf. */
static void printCelsius(const char *label, uint16_t centikelvin)
{
  int32_t  c = (int32_t)centikelvin - 27315;
  uint32_t a = (uint32_t)((c < 0) ? -c : c);

  print("%s %s%lu.%02lu C", label, (c < 0) ? "-" : "",
        (unsigned long)(a / 100U), (unsigned long)(a % 100U));
}

/* Init the camera, report what it is, and start the video. The command
 * interface is shut once the video starts. */
static int32_t startCamera(void)
{
  uint16_t serial[4] = {0};
  uint16_t sensor = 0U;
  uint16_t housing = 0U;
  uint8_t  gain = 0U;
  uint8_t  haveSensor;
  int32_t  status;

  status = BSP_THERMAL_Init(0);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  /* A camera that stops answering holds each read up to two seconds: a
   * delay(1) after each keeps the watchdog and the console serviced. */
  if (BSP_THERMAL_GetSerialNumber(0, serial) == BSP_ERROR_NONE)
  {
    print("Lepton serial %04X%04X%04X%04X\r\n",
          serial[3], serial[2], serial[1], serial[0]);
  }
  delay(1U);
  haveSensor = (BSP_THERMAL_GetFpaTemperature(0, &sensor) == BSP_ERROR_NONE);
  delay(1U);
  if (haveSensor && (BSP_THERMAL_GetAuxTemperature(0, &housing) == BSP_ERROR_NONE))
  {
    printCelsius("sensor", sensor);
    printCelsius(", housing", housing);
    print("\r\n");
  }
  delay(1U);
  if (BSP_THERMAL_GetAgcEnabled(0, &gain) == BSP_ERROR_NONE)
  {
    print("automatic gain %s\r\n", (gain != 0U) ? "on: pixels are not temperatures" : "off");
  }
  delay(1U);

  (void)BSP_THERMAL_SetFlip180(0, 1U);
  return BSP_THERMAL_VideoStart(0, video, VIDEO_BYTES);
}

static void setup(void)
{
}

static void loop(void)
{
  uint32_t now = millis();
  int32_t  status;

  if (camera == CAMERA_OFF)
  {
    /* Power up once a terminal opens the console, or ten seconds after the
     * start, so its report is seen; then retry every ten seconds. */
    if ((tried == 0U) && (BSP_COM_IsConnected() == 0) && (now < 10000U))
    {
      return;
    }
    if ((tried != 0U) && ((now - since) < 10000U))
    {
      return;
    }
    print("starting the thermal camera...\r\n");
    (void)BSP_THERMAL_PowerOn(0);
    camera = CAMERA_BOOTING;
    since = millis();
    return;
  }

  if (camera == CAMERA_BOOTING)
  {
    /* It boots only with the sensor bus quiet. After this wait,
     * BSP_THERMAL_Init asks at once instead of waiting with USB unserviced. */
    if ((now - since) < BSP_THERMAL_BOOT_QUIET_MS)
    {
      return;
    }
    status = startCamera();
    if (status != BSP_ERROR_NONE)
    {
      print("thermal camera not started: %ld\r\n", (long)status);
      (void)BSP_THERMAL_DeInit(0);
      camera = CAMERA_OFF;
      tried = 1U;
      since = millis();
      return;
    }
    print("video started\r\n");
    camera = CAMERA_RUNNING;
    lastFrame = millis();
    lastPrint = lastFrame;
    lastHealth = lastFrame;
    return;
  }

  /* A short wait, so USB and the watchdog are serviced between calls. */
  status = BSP_THERMAL_GetFrame(0, frame, 50U);
  if (status == BSP_ERROR_NONE)
  {
    uint32_t low = 0U;
    uint32_t high = 0U;

    for (uint32_t i = 1U; i < BSP_THERMAL_FRAME_PIXELS; i++)
    {
      if (frame[i] < frame[low])  { low = i; }
      if (frame[i] > frame[high]) { high = i; }
    }
    centre    = frame[(BSP_THERMAL_HEIGHT / 2U) * BSP_THERMAL_WIDTH + (BSP_THERMAL_WIDTH / 2U)];
    coldest   = frame[low];
    hottest   = frame[high];
    hottestAt = high;
    frames++;
    lastFrame = millis();
  }

  /* Once a second; a second with no frames (a recalibration) prints nothing. */
  now = millis();
  if ((now - lastPrint) >= 1000U)
  {
    if (frames != 0U)
    {
      print("%lu frames/s  ", (unsigned long)frames);
      printCelsius("centre", centre);
      printCelsius("  coldest", coldest);
      printCelsius("  hottest", hottest);
      print(" at %lu,%lu\r\n", (unsigned long)(hottestAt % BSP_THERMAL_WIDTH),
            (unsigned long)(hottestAt / BSP_THERMAL_WIDTH));
    }
    frames = 0U;
    lastPrint = now;
  }

  /* The stream's health, every ten seconds. */
  if ((now - lastHealth) >= 10000U)
  {
    BSP_THERMAL_Health_t h = {0};

    (void)BSP_THERMAL_GetHealth(0, &h);
    print("link %lu Hz: frames %lu, segments %lu, resyncs %lu, restarts %lu, overruns %lu, errors %lu\r\n",
          (unsigned long)h.SckHz, (unsigned long)h.FramesParsed, (unsigned long)h.SegmentsParsed,
          (unsigned long)h.Resyncs, (unsigned long)h.Restarts, (unsigned long)h.Overruns,
          (unsigned long)h.SpiErrors);
    lastHealth = now;
  }

  /* No frame for twenty seconds despite the module's own restarts: the camera
   * has stopped, so power-cycle it. The next read restarts the video while
   * the camera boots. */
  if ((now - lastFrame) >= 20000U)
  {
    print("no frame for twenty seconds: power-cycling the camera\r\n");
    (void)BSP_THERMAL_PowerOn(0);
    lastFrame = millis();
  }
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
