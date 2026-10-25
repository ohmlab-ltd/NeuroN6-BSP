/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : The FLIR Lepton thermal camera to Neuro Studio over the
  *                   USB vendor interface: every frame, about nine a second,
  *                   with a temperature for each pixel.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  *
  * All code is inside USER CODE sections, so regenerating from the .ioc keeps
  * it.
  *
  * Each frame is a DNV0 vendor frame with no payload (flags 0) and one
  * META_THERMAL_FRAME record: width, height, format 0 (16-bit little-endian),
  * flag bit 0 (pixels in hundredths of a kelvin), then the 19200 pixels,
  * row-major. Until the first frame the record goes out every two seconds
  * with no pixels. Status lines go to the console and, as META_DEBUG_TEXT
  * records, to Neuro Studio's terminal.
  *
  * The watchdog runs from the first line of main; its early warning and a
  * watchdog reset both go to the bootloader. The console, 'b' included, is
  * serviced on every pass of the loop.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "neuron6.h"
#include "neuron6_com.h"
#include "neuron6_thermal.h"
#include "neuron6_wdg.h"

#include "tusb.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Sent in front of every frame. */
typedef struct __attribute__((packed))
{
  uint32_t magic;
  uint32_t frame_id;
  uint32_t meta_len;
  uint32_t payload_len;
  uint32_t flags;
} vnd_frame_hdr_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* The wire format: N6_FRAME_MAGIC in n6_host_contract.h, META_THERMAL_FRAME
 * and META_DEBUG_TEXT in vnd_meta.h. */
#define VND_MAGIC              0x30564E44U   /* 'DNV0' on the wire */
#define META_THERMAL            0x30U
#define META_TEXT               0x17U
#define THERMAL_FORMAT_RAW16    0U
#define THERMAL_FLAG_KELVIN     0x01U         /* the pixels are temperatures */
#define THERMAL_HEAD_BYTES      11U           /* type, length, then eight bytes */

/* A frame the host has taken none of for this long is replaced by a fresher
 * one. One partly sent is never dropped. */
#define VND_STALL_MS            500U

/* The DMA-only video buffer: whole packet pairs and cache lines, one
 * transfer's maximum. The loop must return within half of it: 13 ms at the
 * camera's highest clock. */
#define VIDEO_BYTES            (49U * 8U * BSP_THERMAL_PACKET_BYTES)   /* 64288 */

#define ANNOUNCE_MS             2000U   /* the empty record, until the first frame */
#define RETRY_MS                10000U  /* a camera that did not start, tried again */
/* No frame for this long despite the module's own stream restarts: the
 * camera has stopped, so power-cycle it. */
#define RESTART_AFTER_MS        20000U
#define REPORT_MS               1000U
#define HEALTH_MS               10000U

/* Room for the camera's start-up before the loop: up to three seconds when no
 * camera answers. */
#ifndef WDG_TIMEOUT_MS
#define WDG_TIMEOUT_MS          4000U
#endif
#define WDG_WARN_MS             300U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static uint8_t  Video[VIDEO_BYTES] __attribute__((aligned(32)));
static uint16_t Frame[BSP_THERMAL_FRAME_PIXELS];

/* The records in flight. Each is built only while the link is free, so
 * neither changes under the pump. */
static uint8_t  ThermalRecord[THERMAL_HEAD_BYTES + BSP_THERMAL_FRAME_BYTES] __attribute__((aligned(4)));
static uint8_t  TextRecord[3U + 128U];

/* The latest status line not yet sent to the host. Only the latest is kept. */
static char     TextLine[128];
static uint8_t  TextPending;

static vnd_frame_hdr_t VndHdr;
static const uint8_t  *VndMeta;
static uint32_t        VndOffset;
static uint32_t        VndTotal;
static uint8_t         VndActive;     /* a frame is in flight */
static uint32_t        VndProgress;   /* tick it last moved */
static uint32_t        VndFrameId;

/* Camera state and the tick it was entered. Booting: powered, and left alone
 * while the loop services USB. */
#define CAMERA_OFF             0U
#define CAMERA_BOOTING          1U
#define CAMERA_RUNNING          2U
static uint8_t  CameraState = CAMERA_OFF;
static uint32_t CameraSince;
static uint32_t LastFrame;
static uint32_t LastAnnounce;
static uint32_t FramesEver;

/* What the camera reported at start. The first start precedes the console,
 * so the loop says it later. */
static int32_t  StartStatus;
static uint16_t CameraSerial[4];
static uint16_t SensorKelvin;
static uint8_t  GainOn;
static uint8_t  StartToSay;
/* Centre pixel of the latest frame, taken on arrival: the next frame is
 * assembled in the same buffer. */
static uint16_t CentreKelvin;
static uint32_t StatFrames, StatSent, StatBusy, StatDropped;
static uint32_t ReportTick, HealthTick;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void Error_Handler(void);
/* USER CODE BEGIN PFP */
static void mpu_config(void);
static bool usb_ready(void);
static bool vnd_start(const uint8_t *pMeta, uint32_t MetaLen);
static void vnd_pump(void);
static void say(const char *pFormat, ...);
static void console(void);
static void thermal_step(void);
static void report_step(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  MPU: non-cacheable regions 0 .noncacheable, 1 the external memory
  *         window, 2 .axisram_nc (the thermal module's DMA descriptor).
  */
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

/* The thermal module installs no vectors: its video DMA (GPDMA1 channel 1)
 * and link error interrupts come here. */
void GPDMA1_Channel1_IRQHandler(void)
{
  BSP_THERMAL_DMA_IRQHandler(0);
}

void SPI1_IRQHandler(void)
{
  BSP_THERMAL_SPI_IRQHandler(0);
}

/* USB output */

static bool usb_ready(void)
{
  return tud_mounted() && tud_vendor_n_mounted(0);
}

/* No host commands: drain whatever arrives so the endpoint keeps taking it. */
void tud_vendor_rx_cb(uint8_t idx, const uint8_t *buffer, uint32_t bufsize)
{
  (void)buffer;
  (void)bufsize;
  tud_vendor_n_read_flush(idx);
}

/**
  * @brief  Put a metadata-only frame in flight.
  * @retval false if another is still going, or nobody is there.
  */
static bool vnd_start(const uint8_t *pMeta, uint32_t MetaLen)
{
  if ((VndActive != 0U) || !usb_ready())
  {
    return false;
  }
  VndHdr.magic       = VND_MAGIC;
  VndHdr.frame_id    = VndFrameId++;
  VndHdr.meta_len    = MetaLen;
  VndHdr.payload_len = 0U;
  VndHdr.flags       = 0U;                 /* no JPEG: the record is the frame */
  VndMeta     = pMeta;
  VndOffset   = 0U;
  VndTotal    = sizeof(VndHdr) + MetaLen;
  VndProgress = HAL_GetTick();
  VndActive   = 1U;
  return true;
}

/**
  * @brief  Push as much of the frame in flight as the vendor FIFO takes.
  */
static void vnd_pump(void)
{
  uint32_t before;

  if (VndActive == 0U)
  {
    return;
  }
  if (!usb_ready())
  {
    VndActive = 0U;                        /* nobody to send to */
    StatDropped++;
    return;
  }

  before = VndOffset;
  while (VndOffset < VndTotal)
  {
    uint32_t avail = tud_vendor_n_write_available(0);
    uint32_t chunk;
    uint32_t written;

    if (avail == 0U)
    {
      break;
    }
    if (VndOffset < sizeof(VndHdr))
    {
      chunk   = sizeof(VndHdr) - VndOffset;
      chunk   = (chunk < avail) ? chunk : avail;
      written = tud_vendor_n_write(0, ((const uint8_t *)&VndHdr) + VndOffset, chunk);
    }
    else
    {
      uint32_t off = VndOffset - sizeof(VndHdr);

      chunk   = VndTotal - VndOffset;
      chunk   = (chunk < avail) ? chunk : avail;
      written = tud_vendor_n_write(0, VndMeta + off, chunk);
    }
    if (written == 0U)
    {
      break;
    }
    VndOffset += written;
  }
  tud_vendor_n_write_flush(0);

  if (VndOffset >= VndTotal)
  {
    VndActive = 0U;
    return;
  }
  if (VndOffset != before)
  {
    VndProgress = HAL_GetTick();           /* it moved: the host is reading */
  }
  else if ((VndOffset == 0U) && ((HAL_GetTick() - VndProgress) >= VND_STALL_MS))
  {
    VndActive = 0U;                        /* nobody is reading: a fresher one */
    StatDropped++;
  }
  else
  {
    /* A partly sent frame waits rather than being dropped: a returning
     * reader would otherwise find the next frame's bytes inside this one's
     * length. */
  }
}

/* The thermal record's eight bytes of description, then the pixels if any. */
static uint32_t thermal_record(const uint16_t *pPixels)
{
  const uint16_t length = (uint16_t)(8U + ((pPixels != NULL) ? BSP_THERMAL_FRAME_BYTES : 0U));

  ThermalRecord[0]  = META_THERMAL;
  ThermalRecord[1]  = (uint8_t)length;
  ThermalRecord[2]  = (uint8_t)(length >> 8);
  ThermalRecord[3]  = (uint8_t)BSP_THERMAL_WIDTH;
  ThermalRecord[4]  = (uint8_t)(BSP_THERMAL_WIDTH >> 8);
  ThermalRecord[5]  = (uint8_t)BSP_THERMAL_HEIGHT;
  ThermalRecord[6]  = (uint8_t)(BSP_THERMAL_HEIGHT >> 8);
  ThermalRecord[7]  = THERMAL_FORMAT_RAW16;
  ThermalRecord[8]  = THERMAL_FLAG_KELVIN;
  ThermalRecord[9]  = 0U;
  ThermalRecord[10] = 0U;
  if (pPixels != NULL)
  {
    /* The processor is little-endian, which is the wire's order. */
    (void)memcpy(&ThermalRecord[THERMAL_HEAD_BYTES], pPixels, BSP_THERMAL_FRAME_BYTES);
  }
  return 3U + (uint32_t)length;
}

/**
  * @brief  A status line, to the console now and to the host's terminal when
  *         the link is next free.
  */
static void say(const char *pFormat, ...)
{
  va_list args;

  va_start(args, pFormat);
  (void)vsnprintf(TextLine, sizeof(TextLine), pFormat, args);
  va_end(args);
  (void)BSP_COM_Printf("%s\r\n", TextLine);
  TextPending = 1U;
}

/* Send the pending status line, if the link is free. */
static void text_step(void)
{
  uint32_t len;

  if ((TextPending == 0U) || (VndActive != 0U))
  {
    return;
  }
  len = (uint32_t)strlen(TextLine);
  TextRecord[0] = META_TEXT;
  TextRecord[1] = (uint8_t)len;
  TextRecord[2] = (uint8_t)(len >> 8);
  (void)memcpy(&TextRecord[3], TextLine, len);
  if (vnd_start(TextRecord, 3U + len) || !usb_ready())
  {
    TextPending = 0U;                      /* sent, or nobody to send it to */
  }
}

/* Hundredths of a kelvin as a signed whole and two places of a degree. */
static void celsius(uint16_t CentiKelvin, long *pWhole, unsigned long *pHundredths, const char **pSign)
{
  int32_t  c = (int32_t)CentiKelvin - 27315;
  uint32_t a = (uint32_t)((c < 0) ? -c : c);

  *pSign       = (c < 0) ? "-" : "";
  *pWhole      = (long)(a / 100U);
  *pHundredths = (unsigned long)(a % 100U);
}

/* The camera */

/**
  * @brief  Init the command interface (it powers the camera and waits out its
  *         boot unless the loop already has), read its identity, start the
  *         video. The command interface is shut once the video starts.
  */
static int32_t thermal_start(void)
{
  int32_t status;

  (void)memset(CameraSerial, 0, sizeof(CameraSerial));
  SensorKelvin = 0U;
  GainOn = 0U;

  (void)BSP_WDG_Refresh();
  status = BSP_THERMAL_Init(0);
  (void)BSP_WDG_Refresh();
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  /* A camera that stops answering holds each read up to two seconds. */
  (void)BSP_THERMAL_GetSerialNumber(0, CameraSerial);
  (void)BSP_WDG_Refresh();
  (void)BSP_THERMAL_GetFpaTemperature(0, &SensorKelvin);
  (void)BSP_WDG_Refresh();
  (void)BSP_THERMAL_GetAgcEnabled(0, &GainOn);
  (void)BSP_WDG_Refresh();

  /* The thermal carrier mounts the camera upside down. */
  (void)BSP_THERMAL_SetFlip180(0, 1U);
  return BSP_THERMAL_VideoStart(0, Video, VIDEO_BYTES);
}

/**
  * @brief  Start the camera and note the result for the console. Called once
  *         before USB is up, then from the loop once a camera it powered has
  *         had time to boot.
  */
static void thermal_up(void)
{
  StartStatus = thermal_start();
  StartToSay  = 1U;
  CameraSince = HAL_GetTick();
  if (StartStatus != BSP_ERROR_NONE)
  {
    (void)BSP_THERMAL_DeInit(0);
    CameraState = CAMERA_OFF;
    return;
  }
  CameraState = CAMERA_RUNNING;
  LastFrame   = CameraSince;
}

/* Say how the last start went, once the console is there to hear it. */
static void start_step(void)
{
  BSP_THERMAL_Health_t h = {0};
  const char *sign;
  long whole;
  unsigned long hundredths;

  if (StartToSay == 0U)
  {
    return;
  }
  StartToSay = 0U;
  if (StartStatus != BSP_ERROR_NONE)
  {
    say("[thermal] camera not started: %ld", (long)StartStatus);
    return;
  }
  (void)BSP_THERMAL_GetHealth(0, &h);
  celsius(SensorKelvin, &whole, &hundredths, &sign);
  say("[thermal] Lepton %04X%04X%04X%04X, sensor at %s%ld.%02lu C, video at %lu Hz%s",
      CameraSerial[3], CameraSerial[2], CameraSerial[1], CameraSerial[0],
      sign, whole, hundredths, (unsigned long)h.SckHz,
      (GainOn != 0U) ? "; automatic gain is on, so the pixels are not temperatures" : "");
}

/**
  * @brief  One pass: start the camera if it is not running, take a frame if
  *         one is ready, and send it if the link is free.
  */
static void thermal_step(void)
{
  const uint32_t now = HAL_GetTick();
  int32_t status;

  if (CameraState == CAMERA_OFF)
  {
    /* Not started: every RETRY_MS, power it and let it boot while the loop
     * services USB. */
    if ((now - CameraSince) < RETRY_MS)
    {
      return;
    }
    (void)BSP_THERMAL_PowerOn(0);
    CameraState = CAMERA_BOOTING;
    CameraSince = HAL_GetTick();
    return;
  }

  if (CameraState == CAMERA_BOOTING)
  {
    /* It boots only with nothing on the sensor bus, as here. */
    if ((now - CameraSince) >= BSP_THERMAL_BOOT_QUIET_MS)
    {
      thermal_up();
    }
    return;
  }

  /* Parse whatever has arrived; the loop returns long before the DMA is half
   * way round again. */
  status = BSP_THERMAL_GetFrame(0, Frame, 1U);
  if (status == BSP_ERROR_NONE)
  {
    CentreKelvin = Frame[(BSP_THERMAL_HEIGHT / 2U) * BSP_THERMAL_WIDTH + (BSP_THERMAL_WIDTH / 2U)];
    StatFrames++;
    FramesEver++;
    LastFrame = HAL_GetTick();
    if (VndActive == 0U)
    {
      if (vnd_start(ThermalRecord, thermal_record(Frame)))
      {
        StatSent++;
      }
    }
    else
    {
      StatBusy++;                          /* the last frame is still going out */
    }
    return;
  }

  if ((FramesEver == 0U) && ((now - LastAnnounce) >= ANNOUNCE_MS) && (VndActive == 0U))
  {
    if (vnd_start(ThermalRecord, thermal_record(NULL)))
    {
      LastAnnounce = now;
    }
  }

  if ((now - LastFrame) >= RESTART_AFTER_MS)
  {
    /* The next read starts the video again while the camera boots. */
    say("[thermal] no frame for %lu s: power-cycling the camera",
        (unsigned long)(RESTART_AFTER_MS / 1000U));
    (void)BSP_THERMAL_PowerOn(0);
    LastFrame = HAL_GetTick();
  }
}

/**
  * @brief  The once-a-second report, and the link's health every ten.
  */
static void report_step(void)
{
  const uint32_t now = HAL_GetTick();

  if ((now - ReportTick) >= REPORT_MS)
  {
    ReportTick = now;
    if (StatFrames != 0U)
    {
      const char *sign;
      long whole;
      unsigned long hundredths;

      celsius(CentreKelvin, &whole, &hundredths, &sign);
      (void)BSP_COM_Printf("[thermal] %lu frames, %lu sent, %lu while the link was busy, %lu dropped; centre %s%ld.%02lu C\r\n",
                           (unsigned long)StatFrames, (unsigned long)StatSent,
                           (unsigned long)StatBusy, (unsigned long)StatDropped,
                           sign, whole, hundredths);
    }
    StatFrames = 0U; StatSent = 0U; StatBusy = 0U; StatDropped = 0U;
  }

  if ((CameraState == CAMERA_RUNNING) && ((now - HealthTick) >= HEALTH_MS))
  {
    BSP_THERMAL_Health_t h = {0};

    HealthTick = now;
    (void)BSP_THERMAL_GetHealth(0, &h);
    say("[thermal] link %lu Hz: frames %lu, segments %lu, resyncs %lu, restarts %lu, overruns %lu, errors %lu",
        (unsigned long)h.SckHz, (unsigned long)h.FramesParsed, (unsigned long)h.SegmentsParsed,
        (unsigned long)h.Resyncs, (unsigned long)h.Restarts, (unsigned long)h.Overruns,
        (unsigned long)h.SpiErrors);
  }
}

/* Console */

/**
  * @brief  Service USB, the watchdog and the console.
  */
static void console(void)
{
  uint8_t ch;

  BSP_COM_Task();
  (void)BSP_WDG_Refresh();

  while (BSP_COM_Read(&ch, 1U) == 1)
  {
    if (ch == (uint8_t)'b')
    {
      BSP_COM_RebootToBootloader();        /* does not return */
    }
  }
}

/* Watchdog early warning, 300 ms before the reset: go to the bootloader. */
void BSP_WDG_EWI_Callback(void)
{
  BSP_COM_RebootToBootloader();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  SCB_DisableDCache();
  mpu_config();
  SCB_EnableICache();
  SCB_EnableDCache();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */
  /* No SystemClock_Config: the boot loader owns the clock tree and both
   * external memories; the BSP reads the frequencies back at run time. */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  (void)BSP_LED_Init(LED_RED);
  (void)BSP_LED_Init(LED_GREEN);

  /* The watchdog first, before anything that could wedge. */
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U)
  {
    __HAL_RCC_CLEAR_RESET_FLAGS();
    BSP_COM_RebootToBootloader();
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
  (void)BSP_WDG_InitEWI(WDG_TIMEOUT_MS, WDG_WARN_MS);

  /* The camera first (up to three seconds while it boots), USB last, then
   * service it. */
  thermal_up();

  if (BSP_COM_Init() != BSP_ERROR_NONE)
  {
    (void)BSP_LED_On(LED_RED);
    Error_Handler();
  }
  (void)BSP_LED_On(LED_GREEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    console();
    vnd_pump();
    thermal_step();
    start_step();
    text_step();
    report_step();
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Go to the bootloader. */
  BSP_COM_RebootToBootloader();
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  UNUSED(file);
  UNUSED(line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
