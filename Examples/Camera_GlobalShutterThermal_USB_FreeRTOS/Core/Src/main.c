/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : The thermal carrier to Neuro Studio over USB, under
  *                   FreeRTOS: the VD66GY global-shutter camera as JPEG, the
  *                   Lepton thermal camera with a temperature for every pixel,
  *                   and the board's motion and magnetic field readings.
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
  * Tasks:
  *   encode  (+4)  VD66GY VGA into one of two JPEG buffers while the other
  *                 goes out. Highest, so each re-arm runs in one piece; the
  *                 timer task (+2) wakes it when a frame is done.
  *   usb     (+3)  USB, the console and the vendor link: a ready thermal
  *                 frame, else the next JPEG with the latest motion and
  *                 magnetic readings as metadata. Only this task prints.
  *   thermal (+3)  woken at each half of the Lepton's DMA buffer; parses, and
  *                 fills the thermal record once the link is done with it.
  *                 Below the encoder, so never inside a re-arm.
  *   sensors (+1)  accelerometer, gyroscope and magnetometer at 10 Hz. Below
  *                 the timer task, so it never delays the encode task's wake.
  *
  * The Lepton finishes booting only with the sensor bus quiet, so start-up
  * runs before the scheduler in this order: motion sensors, thermal camera,
  * global-shutter camera and encoder, USB. After any later power-up of the
  * Lepton, the sensors task waits for it to boot.
  *
  * Each record is one DNV0 vendor frame:
  *   - JPEG: flags bit 0, the JPEG as payload, and as metadata META_IMU_FRAME
  *     (seven floats: acceleration in g, rotation in degrees a second,
  *     temperature in degrees Celsius) and META_MAG_FRAME (four floats: the
  *     field in microtesla, temperature);
  *   - thermal: flags 0, no payload, one META_THERMAL_FRAME record (width,
  *     height, format 0, flag bit 0 for hundredths of a kelvin, then the
  *     19200 pixels, row-major); until the first frame it goes out every two
  *     seconds with no pixels;
  *   - with no camera, the two sensor records alone, ten times a second.
  * Status lines go to the console and, as META_DEBUG_TEXT records, to Neuro
  * Studio's terminal.
  *
  * The watchdog runs from the first line of main; its early warning and a
  * watchdog reset both go to the bootloader, as does 'b' on the console.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "neuron6.h"
#include "neuron6_com.h"
#include "neuron6_camera.h"
#include "neuron6_camera_vd66gy.h"
#include "neuron6_venc.h"
#include "neuron6_thermal.h"
#include "neuron6_motion_sensor.h"
#include "neuron6_wdg.h"
#include "neuron6_rtos.h"

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

/* The motion and magnetic readings, in the units the host takes. */
typedef struct
{
  float   accel[3];    /* g */
  float   gyro[3];     /* degrees a second */
  float   imuTemp;     /* degrees Celsius */
  float   field[3];    /* microtesla */
  float   magTemp;     /* degrees Celsius */
  uint8_t valid;       /* SENSORS_MOTION, SENSORS_FIELD */
} readings_t;

/* The kind of frame in flight. */
typedef enum
{
  VND_JPEG,
  VND_THERMAL,
  VND_OTHER
} vnd_kind_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* The VD66GY: mode, rate, carrier, and its exposure target (0 to 100). Its
 * exposure loop meters the raw picture, before white balance and gamma
 * brighten it, so a middle target overexposes. The thermal carrier crosses
 * the two data lanes; a wrong setting gives no error, only no frames. */
#define CAM_RESOLUTION         BSP_CAMERA_VD66GY_R640x480
#define CAM_FRAME_RATE          60U
#define CAM_CARRIER             BSP_CAMERA_CARRIER_INVERTED
#define CAM_EXPOSURE_TARGET     25

/* The band the camera writes and the encoder reads, sized for VGA. */
#define CAM_BAND_BYTES          81920U

/* One finished JPEG. A VGA frame runs to a few tens of kilobytes. */
#define JPEG_BUF_BYTES          (256U * 1024U)

/* The wire format: N6_FRAME_MAGIC in n6_host_contract.h; META_IMU_FRAME,
 * META_MAG_FRAME, META_THERMAL_FRAME and META_DEBUG_TEXT in vnd_meta.h. */
#define VND_MAGIC              0x30564E44U   /* 'DNV0' on the wire */
#define VND_FLAG_JPEG           0x00000001U
#define META_IMU                0x1BU
#define META_MAG                0x1AU
#define META_THERMAL            0x30U
#define META_TEXT               0x17U
#define THERMAL_FORMAT_RAW16    0U
#define THERMAL_FLAG_KELVIN     0x01U         /* the pixels are temperatures */
#define THERMAL_HEAD_BYTES      11U           /* type, length, then eight bytes */

/* A frame the host has taken none of for this long is replaced by a fresher
 * one. One partly sent is never dropped. */
#define VND_STALL_MS            500U
/* JPEG frames go to USB through a slice of on-chip RAM this size. */
#define VND_SLICE_BYTES         8192U

/* Failed encodes, or torn frames, in a row before the camera and encoder
 * restart. */
#define STREAM_RESYNC_AFTER_FAILS  8U
#define STREAM_RESYNC_AFTER_TORN   2U

/* The Lepton's DMA-only video buffer: whole packet pairs and cache lines,
 * one transfer's maximum. Each half wakes the thermal task, at least 13 ms
 * apart at the camera's highest clock. */
#define THERMAL_VIDEO_BYTES    (49U * 8U * BSP_THERMAL_PACKET_BYTES)   /* 64288 */
#define THERMAL_WAKE_MS         100U    /* the thermal task's longest sleep */
#define THERMAL_ANNOUNCE_MS     2000U   /* the empty record, until the first frame */
/* No frame for this long despite the module's own stream restarts: the
 * camera has stopped, so power-cycle it. */
#define THERMAL_RESTART_MS     20000U
#define THERMAL_RETRY_MS        30000U  /* a camera that did not start, tried again */

#define SENSORS_PERIOD_MS       100U
#define SENSORS_TEMP_MS         10000U  /* the temperatures, read this often */
#define SENSORS_MOTION          0x01U
#define SENSORS_FIELD           0x02U

/* Status lines waiting for the host. */
#define TEXT_LINES              4U
#define TEXT_LINE_BYTES         160U

#define REPORT_MS               1000U
#define HEALTH_MS               10000U

/* Room for the start-up before the scheduler: the Lepton's (up to three
 * seconds) and the VD66GY's, which loads its firmware. */
#ifndef WDG_TIMEOUT_MS
#define WDG_TIMEOUT_MS          4000U
#endif
#define WDG_WARN_MS             300U

#define USB_TASK_STACK_WORDS        1024U
#define USB_TASK_PRIORITY           (tskIDLE_PRIORITY + 3U)
#define ENCODE_TASK_STACK_WORDS     2048U
#define ENCODE_TASK_PRIORITY        (tskIDLE_PRIORITY + 4U)
#define THERMAL_TASK_STACK_WORDS    1024U
#define THERMAL_TASK_PRIORITY       (tskIDLE_PRIORITY + 3U)
#define SENSORS_TASK_STACK_WORDS    1024U
/* Below the timer task (configTIMER_TASK_PRIORITY, 2), through which the
 * encoder's interrupt wakes the encode task each frame. */
#define SENSORS_TASK_PRIORITY      (tskIDLE_PRIORITY + 1U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* The global-shutter camera */
/* The band the camera writes and the encoder reads, wrapped. */
static uint8_t Band[CAM_BAND_BYTES]
  __attribute__((section(".axisram_nc"), aligned(32)));

/* Two finished frames: one being sent while the next is encoded. */
static uint8_t JpegBuf[2][JPEG_BUF_BYTES]
  __attribute__((section(".ext_psram"), aligned(32)));

/* Frames posted by the encoder, and retired (sent or dropped) by USB. Frame
 * n is in JpegBuf[n % 2]; each count has one writer. */
static volatile uint32_t EncSeq, TxSeq;
static uint32_t          PostedBytes[2];

static BSP_VENC_Band_t   StreamBand;
static uint8_t           LinebufArmed;
static uint8_t           ArmFollowsEncode;
static uint32_t          ConsecFail, ConsecTorn;
/* Line events since the camera's last frame event: zero between its frames. */
static volatile uint32_t FrameLines;
/* Set from the camera's frame start to its frame end. */
static volatile uint32_t FrameActive;
/* Frame events on the main pipe. */
static volatile uint32_t FrameEnds;
/* A wait for the camera to end its frame: running, start tick, FrameEnds at
 * the start, and the limit before the camera counts as stopped. */
static volatile uint8_t  GapWaiting;
static uint32_t          GapWaitSince;
static uint32_t          GapWaitFrames;
static uint32_t          GapWaitMs = 1000U;
static TaskHandle_t      EncodeTask;
static int32_t           CameraStatus = BSP_ERROR_NO_INIT;
static const char       *CameraStep = "not started";

/* The thermal camera */
static uint8_t  ThermalVideo[THERMAL_VIDEO_BYTES] __attribute__((aligned(32)));
/* Assembled a segment at a time, so whole only when a read returns one. */
static uint16_t ThermalFrame[BSP_THERMAL_FRAME_PIXELS];
/* The record: filled by the thermal task while FREE, sent by the USB task
 * while READY and SENDING. Each state change has one writer. */
static uint8_t  ThermalRecord[THERMAL_HEAD_BYTES + BSP_THERMAL_FRAME_BYTES] __attribute__((aligned(4)));
static uint32_t ThermalRecordLen;
#define RECORD_FREE     0U
#define RECORD_READY    1U
#define RECORD_SENDING  2U
static volatile uint8_t ThermalState = RECORD_FREE;
static uint8_t  ThermalAnnounce[THERMAL_HEAD_BYTES];
static SemaphoreHandle_t ThermalHalf;
/* What the camera reported at start: written before ThermalTries counts the
 * start, read by the USB task after. */
static volatile int32_t  ThermalStatus = BSP_ERROR_NO_INIT;
static uint16_t ThermalSerial[4];
static uint16_t ThermalSensorKelvin;
static uint8_t  ThermalGainOn;
/* Counted by the thermal task from the start, read by the USB task. */
static volatile uint32_t ThermalTries, ThermalPowerCycles;
static volatile uint32_t ThermalFrames, ThermalBusy;
/* Centre pixel of the latest frame, taken on arrival. */
static volatile uint16_t ThermalCentreKelvin;

/* The motion and magnetic readings */
/* Held by the sensors task while it reads, and by the thermal task while the
 * thermal camera boots, which needs the bus quiet. */
static SemaphoreHandle_t SensorBus;
static uint8_t    SensorsUp;          /* SENSORS_MOTION, SENSORS_FIELD */
static readings_t Readings;           /* the sensors task's, copied under a critical section */
/* The metadata a frame carries: the readings when it went out. */
static uint8_t    SensorMeta[(3U + 28U) + (3U + 16U)];
static uint32_t   LastSensorFrame;

/* The vendor link (USB task only) */
static vnd_frame_hdr_t VndHdr;
static const uint8_t  *VndMeta;
static uint32_t        VndMetaLen;
static const uint8_t  *VndPayload;
static uint32_t        VndPayloadLen;
static uint32_t        VndOffset;
static uint32_t        VndTotal;
static uint8_t         VndActive;     /* a frame is in flight */
static vnd_kind_t      VndKind;
static uint32_t        VndProgress;   /* tick the frame in flight last moved */
static uint32_t        VndFrameId;
static uint64_t        VndSlice[VND_SLICE_BYTES / 8U];
static uint32_t        LastAnnounce;

/* Status lines said and not yet sent to the host; the oldest goes when full. */
static char     TextLines[TEXT_LINES][TEXT_LINE_BYTES];
static uint32_t TextSaid, TextSent;
static uint8_t  TextRecord[3U + TEXT_LINE_BYTES];

/* Counted between reports. */
static uint32_t StatEncoded, StatFailed, StatTorn, StatResync, StatSent, StatDropped;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void Error_Handler(void);
/* USER CODE BEGIN PFP */
static void    mpu_config(void);
static void    say(const char *pFormat, ...) __attribute__((format(printf, 1, 2)));
static int32_t camera_start(void *pArg);
static int32_t camera_init(void);
static int32_t thermal_init(void);
static void    sensors_init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  MPU: non-cacheable regions 0 .noncacheable, 1 the external memory
  *         window (the encoded frames), 2 .axisram_nc (the encoder's band and
  *         the thermal module's DMA descriptor).
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

/* Status */

/**
  * @brief  A status line, to the console now and to the host's terminal when
  *         the link is next free. USB task only.
  */
static void say(const char *pFormat, ...)
{
  va_list args;
  char   *line;

  if ((TextSaid - TextSent) >= TEXT_LINES)
  {
    TextSent++;                            /* full: the oldest goes */
  }
  line = TextLines[TextSaid % TEXT_LINES];
  va_start(args, pFormat);
  (void)vsnprintf(line, TEXT_LINE_BYTES, pFormat, args);
  va_end(args);
  (void)BSP_COM_Printf("%s\r\n", line);
  TextSaid++;
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

/* Global-shutter camera */

/**
  * @brief  The pipe's processing stages: ST's daylight (D65) tuning for the
  *         VD66GY module (STEVAL-66GYMAI1).
  * @note   Black level out of 255, white-balance gains in 128ths, colour
  *         correction in 256ths with each row summing to 256, so grey stays
  *         grey.
  */
static int32_t camera_isp(void)
{
  DCMIPP_RawBayer2RGBConfTypeDef    demosaic = {0};
  DCMIPP_BlackLevelConfTypeDef      black    = {0};
  DCMIPP_ExposureConfTypeDef        gains    = {0};
  DCMIPP_ColorConversionConfTypeDef ccm      = {0};
  uint32_t                          bayer    = BSP_CAMERA_BAYER_RGGB;
  int32_t                           status;

  /* The colour-filter order is read out of the sensor as it starts. */
  status = BSP_CAMERA_VD66GY_GetBayerOrder(&bayer);
  if (status != BSP_ERROR_NONE) { return status; }

  demosaic.RawBayerType  = bayer;
  demosaic.PeakStrength  = DCMIPP_RAWBAYER_ALGO_STRENGTH_4;
  demosaic.VLineStrength = DCMIPP_RAWBAYER_ALGO_STRENGTH_8;
  demosaic.HLineStrength = DCMIPP_RAWBAYER_ALGO_STRENGTH_8;
  demosaic.EdgeStrength  = DCMIPP_RAWBAYER_ALGO_STRENGTH_16;
  status = BSP_CAMERA_SetIspDemosaicing(0, BSP_CAMERA_PIPE_MAIN, &demosaic);
  if (status != BSP_ERROR_NONE) { return status; }

  black.RedCompBlackLevel   = 16U;
  black.GreenCompBlackLevel = 16U;
  black.BlueCompBlackLevel  = 16U;
  status = BSP_CAMERA_SetIspBlackLevel(0, BSP_CAMERA_PIPE_MAIN, &black);
  if (status != BSP_ERROR_NONE) { return status; }

  gains.ShiftRed   = 0U; gains.MultiplierRed   = 219U;   /* 1.71 */
  gains.ShiftGreen = 0U; gains.MultiplierGreen = 128U;   /* 1.00 */
  gains.ShiftBlue  = 0U; gains.MultiplierBlue  = 193U;   /* 1.51 */
  status = BSP_CAMERA_SetIspGain(0, BSP_CAMERA_PIPE_MAIN, &gains);
  if (status != BSP_ERROR_NONE) { return status; }

  ccm.ClampOutputSamples = DISABLE;
  ccm.OutputSamplesType  = DCMIPP_CLAMP_RGB;
  ccm.RR = 362; ccm.RG =   29; ccm.RB = -135; ccm.RA = 0;
  ccm.GR = -88; ccm.GG =  474; ccm.GB = -130; ccm.GA = 0;
  ccm.BR = -23; ccm.BG = -102; ccm.BB =  381; ccm.BA = 0;
  status = BSP_CAMERA_SetIspColorConversion(0, BSP_CAMERA_PIPE_MAIN, &ccm);
  if (status != BSP_ERROR_NONE) { return status; }

  return BSP_CAMERA_SetIspGamma(0, BSP_CAMERA_PIPE_MAIN, 1U);
}

/**
  * @brief  Start the camera. Called by the encoder, which must be armed on
  *         venc_rdy before the first line event.
  */
static int32_t camera_start(void *pArg)
{
  (void)pArg;
  return (BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, Band,
                           BSP_CAMERA_MODE_CONTINUOUS) == BSP_ERROR_NONE) ? 0 : -1;
}

void BSP_CAMERA_LineEventCallback(uint32_t Instance, uint32_t Pipe)
{
  if ((Instance == 0U) && (Pipe == BSP_CAMERA_PIPE_MAIN))
  {
    FrameLines++;
    (void)BSP_VENC_SourceLineEvent(0);
  }
}

/* Anchors the row count to the frame, so a late-armed encode shows as a gap
 * and a torn frame. */
void BSP_CAMERA_FrameEventCallback(uint32_t Instance, uint32_t Pipe)
{
  if ((Instance == 0U) && (Pipe == BSP_CAMERA_PIPE_MAIN))
  {
    FrameLines = 0U;
    FrameActive = 0U;
    FrameEnds++;
    (void)BSP_VENC_SourceFrameEvent(0);
    if ((GapWaiting != 0U) && (EncodeTask != NULL))
    {
      /* The encode task waits for this: arm it at the start of the gap. */
      BaseType_t woken = pdFALSE;

      vTaskNotifyGiveFromISR(EncodeTask, &woken);
      portYIELD_FROM_ISR(woken);
    }
  }
}

/**
  * @brief  The camera has started a frame.
  */
void BSP_CAMERA_VsyncEventCallback(uint32_t Instance, uint32_t Pipe)
{
  if ((Instance == 0U) && (Pipe == BSP_CAMERA_PIPE_MAIN))
  {
    FrameActive = 1U;
  }
}

/**
  * @brief  Sensor, camera, pipe, processing, line buffer, prime, encoder.
  */
static int32_t camera_init(void)
{
  BSP_CAMERA_PipeConfig_t   pipe = {0};
  BSP_CAMERA_Capabilities_t cap  = {0};
  BSP_VENC_Config_t         venc = {0};
  BSP_VENC_Band_t           band;
  int32_t                   status;

  (void)BSP_WDG_Refresh();

  CameraStep = "sensor choice";
  status = BSP_CAMERA_VD66GY_Register();
  if (status == BSP_ERROR_NONE) { status = BSP_CAMERA_SetCarrier(CAM_CARRIER); }
  if (status == BSP_ERROR_NONE) { status = BSP_CAMERA_VD66GY_SetFrameRate(CAM_FRAME_RATE); }
  if (status != BSP_ERROR_NONE) { return status; }

  /* Identifies the sensor, loads its firmware and sets it up, idle. */
  CameraStep = "camera init";
  status = BSP_CAMERA_Init(0, CAM_RESOLUTION, BSP_CAMERA_PF_RAW8);
  if (status != BSP_ERROR_NONE) { return status; }
  (void)BSP_WDG_Refresh();

  CameraStep = "picture size";
  status = BSP_CAMERA_GetCapabilities(0, &cap);
  if (status != BSP_ERROR_NONE) { return status; }
  {
    const uint32_t fps = (CAM_FRAME_RATE < cap.MaxFrameRate) ? CAM_FRAME_RATE : cap.MaxFrameRate;

    GapWaitMs = (fps != 0U) ? (5000U / fps) : 1000U;
    if (GapWaitMs < 150U) { GapWaitMs = 150U; }
  }
  status = BSP_VENC_GetBandGeometry(cap.Width, cap.Height, &band);
  if (status != BSP_ERROR_NONE) { return status; }
  if (band.SizeBytes > sizeof(Band)) { return BSP_ERROR_WRONG_PARAM; }

  /* The pipe takes the sensor's whole frame; the encoder codes the top
   * band.CodedHeight lines of it. */
  CameraStep = "camera pipe";
  pipe.PixelFormat = BSP_CAMERA_PIPE_PF_YUV422;
  pipe.Width       = cap.Width;
  pipe.Height      = cap.Height;
  status = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pipe);
  if (status != BSP_ERROR_NONE) { return status; }
  (void)BSP_WDG_Refresh();

  /* After the pipe: configuring it resets its colour reconstruction. */
  CameraStep = "image processing";
  status = camera_isp();
  if (status != BSP_ERROR_NONE) { return status; }

  CameraStep = "line buffer";
  status = BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN,
                                       band.WrapLines, BSP_CAMERA_LINES_OFF);
  if (status != BSP_ERROR_NONE) { return status; }
  (void)BSP_WDG_Refresh();

  /* Prime, and leave it running into the wrapped band until the first encode
   * stops it and arms the pacing event. The sensor starts streaming here. */
  CameraStep = "camera start";
  status = BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, Band, BSP_CAMERA_MODE_CONTINUOUS);
  if (status != BSP_ERROR_NONE) { return status; }
  (void)BSP_WDG_Refresh();

  /* Set once the sensor streams. */
  CameraStep = "exposure target";
  status = BSP_CAMERA_VD66GY_SetBrightness(CAM_EXPOSURE_TARGET);
  if (status != BSP_ERROR_NONE) { return status; }

  StreamBand   = band;
  LinebufArmed = 0U;

  CameraStep = "encoder";
  venc.Band        = band;
  venc.pBand       = Band;
  venc.Quality     = BSP_VENC_QUALITY_DEFAULT;
  venc.SourceStart = camera_start;
  venc.pSourceArg  = NULL;
  status = BSP_VENC_Init(0, &venc);
  if (status == BSP_ERROR_NONE) { CameraStep = "none"; }
  return status;
}

/**
  * @brief  Stop the camera and reset the encoder; the next encode starts them
  *         as the first one did.
  */
static void stream_park(void)
{
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
  (void)BSP_VENC_Resync(0);
  LinebufArmed = 0U;
  ConsecFail   = 0U;
  ConsecTorn   = 0U;
  FrameLines   = 0U;        /* the stopped camera's frame will not end */
  FrameActive  = 0U;
  GapWaiting   = 0U;
}

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
  * @brief  One encode, while the host has the vendor interface up and a
  *         buffer is free.
  * @retval 1 if an encode ran, 0 if there was nothing to do.
  */
static int stream_encode_step(void)
{
  uint32_t bytes = 0U;
  uint32_t slot;
  uint8_t  follow_on;
  int32_t  enc;

  if (!usb_ready() || (CameraStatus != BSP_ERROR_NONE))
  {
    /* Stopped: park, so the next start is clean rather than mid-frame. */
    if (LinebufArmed)
    {
      stream_park();
    }
    return 0;
  }

  /* Both buffers are with USB. */
  if ((EncSeq - TxSeq) >= 2U)
  {
    GapWaiting = 0U;
    ArmFollowsEncode = 0U;
    return 0;
  }
  follow_on        = ArmFollowsEncode;
  ArmFollowsEncode = 0U;

  /* Just before the first encode, and the first after a park: stop the
   * free-running pipe (flushing the receiver) and arm the pacing event. */
  if (!LinebufArmed)
  {
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
    (void)BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN,
                                      StreamBand.WrapLines, StreamBand.EventLines);
    LinebufArmed = 1U;
  }
  else if ((FrameLines != 0U) || ((follow_on == 0U) && (FrameActive != 0U)))
  {
    /* Mid-frame: wait for it to end, and arm in the vertical blanking. */
    if (GapWaiting == 0U)
    {
      GapWaitSince  = HAL_GetTick();
      GapWaitFrames = FrameEnds;
      GapWaiting    = 1U;
    }
    if ((HAL_GetTick() - GapWaitSince) < GapWaitMs)
    {
      return 0;
    }
    GapWaiting = 0U;
    if (FrameEnds == GapWaitFrames)
    {
      /* No frame ended: the camera stopped mid-frame. Restart it. */
      stream_park();
      StatResync++;
      return 1;
    }
  }
  GapWaiting = 0U;

  slot = EncSeq & 1U;
  enc = BSP_VENC_EncodeBand(0, JpegBuf[slot], JPEG_BUF_BYTES, &bytes);
  if (enc != BSP_ERROR_NONE)
  {
    StatFailed++;
    if (++ConsecFail >= STREAM_RESYNC_AFTER_FAILS)
    {
      stream_park();
      StatResync++;
    }
    return 1;
  }
  ConsecFail = 0U;
  ArmFollowsEncode = 1U;

  /* A lapped frame is split between two frames: never sent. */
  if (BSP_VENC_LastFrameTorn(0) != 0U)
  {
    StatTorn++;
    if (++ConsecTorn >= STREAM_RESYNC_AFTER_TORN)
    {
      stream_park();
      StatResync++;
    }
    return 1;
  }
  ConsecTorn = 0U;

  StatEncoded++;
  PostedBytes[slot] = bytes;
  __DMB();                 /* the length lands before the post */
  EncSeq++;
  return 1;
}

static void encode_task(void *pArg)
{
  (void)pArg;

  for (;;)
  {
    if (stream_encode_step() == 0)
    {
      /* Up to a tick; the frame event wakes it early during a gap wait. */
      (void)ulTaskNotifyTake(pdTRUE, 1U);
    }
  }
}

/* Thermal camera */

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

/* Half the video buffer has filled: wake the thermal task. Interrupt context,
 * at BSP_THERMAL_IT_PRIORITY, where FreeRTOS calls are allowed. */
void BSP_THERMAL_HalfCpltCallback(uint32_t Instance)
{
  BaseType_t woken = pdFALSE;

  (void)Instance;
  if (ThermalHalf != NULL)
  {
    (void)xSemaphoreGiveFromISR(ThermalHalf, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

/* The thermal record's eight bytes of description, then the pixels if any. */
static uint32_t thermal_record(uint8_t *pRecord, const uint16_t *pPixels)
{
  const uint16_t length = (uint16_t)(8U + ((pPixels != NULL) ? BSP_THERMAL_FRAME_BYTES : 0U));

  pRecord[0]  = META_THERMAL;
  pRecord[1]  = (uint8_t)length;
  pRecord[2]  = (uint8_t)(length >> 8);
  pRecord[3]  = (uint8_t)BSP_THERMAL_WIDTH;
  pRecord[4]  = (uint8_t)(BSP_THERMAL_WIDTH >> 8);
  pRecord[5]  = (uint8_t)BSP_THERMAL_HEIGHT;
  pRecord[6]  = (uint8_t)(BSP_THERMAL_HEIGHT >> 8);
  pRecord[7]  = THERMAL_FORMAT_RAW16;
  pRecord[8]  = THERMAL_FLAG_KELVIN;
  pRecord[9]  = 0U;
  pRecord[10] = 0U;
  if (pPixels != NULL)
  {
    /* The processor is little-endian, which is the wire's order. */
    (void)memcpy(&pRecord[THERMAL_HEAD_BYTES], pPixels, BSP_THERMAL_FRAME_BYTES);
  }
  return 3U + (uint32_t)length;
}

/**
  * @brief  Power the camera, wait for its boot, read its identity, start the
  *         video. The command interface is shut once the video starts.
  * @note   Needs a quiet sensor bus: before the scheduler, or with SensorBus
  *         held.
  */
static int32_t thermal_init(void)
{
  int32_t status;

  (void)memset(ThermalSerial, 0, sizeof(ThermalSerial));
  ThermalSensorKelvin = 0U;
  ThermalGainOn = 0U;

  (void)BSP_WDG_Refresh();
  status = BSP_THERMAL_Init(0);
  (void)BSP_WDG_Refresh();
  if (status == BSP_ERROR_NONE)
  {
    /* A camera that stops answering holds each read up to two seconds. */
    (void)BSP_THERMAL_GetSerialNumber(0, ThermalSerial);
    (void)BSP_WDG_Refresh();
    (void)BSP_THERMAL_GetFpaTemperature(0, &ThermalSensorKelvin);
    (void)BSP_WDG_Refresh();
    (void)BSP_THERMAL_GetAgcEnabled(0, &ThermalGainOn);
    (void)BSP_WDG_Refresh();

    /* The thermal carrier mounts the camera upside down. */
    (void)BSP_THERMAL_SetFlip180(0, 1U);
    status = BSP_THERMAL_VideoStart(0, ThermalVideo, THERMAL_VIDEO_BYTES);
  }
  if (status != BSP_ERROR_NONE)
  {
    (void)BSP_THERMAL_DeInit(0);
  }
  ThermalStatus = status;
  __DMB();                 /* what it said lands before the count */
  ThermalTries++;
  return status;
}

/**
  * @brief  Woken by each half of the video buffer, or after THERMAL_WAKE_MS:
  *         parse, and put a finished frame in the record once the link is
  *         done with the last.
  */
static void thermal_task(void *pArg)
{
  uint32_t lastFrame  = HAL_GetTick();
  uint32_t lastTry    = HAL_GetTick();
  uint32_t quietSince = 0U;
  uint8_t  quiet      = 0U;     /* SensorBus held while the camera boots */

  (void)pArg;
  for (;;)
  {
    (void)xSemaphoreTake(ThermalHalf, pdMS_TO_TICKS(THERMAL_WAKE_MS));

    if ((quiet != 0U) && ((HAL_GetTick() - quietSince) >= BSP_THERMAL_BOOT_QUIET_MS))
    {
      (void)xSemaphoreGive(SensorBus);
      quiet = 0U;
    }

    if (ThermalStatus != BSP_ERROR_NONE)
    {
      /* Not started: retry rarely, as the motion sensors wait during a try
       * (three seconds when no camera answers). */
      if ((HAL_GetTick() - lastTry) >= THERMAL_RETRY_MS)
      {
        (void)xSemaphoreTake(SensorBus, portMAX_DELAY);
        (void)thermal_init();
        (void)xSemaphoreGive(SensorBus);
        lastTry   = HAL_GetTick();
        lastFrame = lastTry;
      }
      continue;
    }

    /* Parse whatever has arrived. Reads never touch the sensor bus, so they
     * go on while the camera boots. */
    if (BSP_THERMAL_GetFrame(0, ThermalFrame, 0U) == BSP_ERROR_NONE)
    {
      lastFrame = HAL_GetTick();
      ThermalFrames++;
      ThermalCentreKelvin = ThermalFrame[(BSP_THERMAL_HEIGHT / 2U) * BSP_THERMAL_WIDTH +
                                         (BSP_THERMAL_WIDTH / 2U)];
      if (ThermalState == RECORD_FREE)
      {
        ThermalRecordLen = thermal_record(ThermalRecord, ThermalFrame);
        __DMB();          /* the record is whole before the USB task sees it */
        ThermalState = RECORD_READY;
      }
      else
      {
        ThermalBusy++;    /* the last frame is still going out */
      }
    }
    else if ((quiet == 0U) && ((HAL_GetTick() - lastFrame) >= THERMAL_RESTART_MS))
    {
      /* Power-cycle the camera, with the motion sensors off the bus until it
       * boots. Capture restarts at the next read, during the boot, for a
       * sooner first frame. */
      (void)xSemaphoreTake(SensorBus, portMAX_DELAY);
      quiet = 1U;
      (void)BSP_THERMAL_PowerOn(0);
      quietSince = HAL_GetTick();
      lastFrame  = quietSince;
      ThermalPowerCycles++;
    }
    else
    {
      /* nothing finished yet */
    }
  }
}

/* Motion and magnetic */

static void sensors_init(void)
{
  if ((BSP_MOTION_SENSOR_Init(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO) == BSP_ERROR_NONE) &&
      (BSP_MOTION_SENSOR_Enable(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO) == BSP_ERROR_NONE) &&
      (BSP_MOTION_SENSOR_Enable(MOTION_SENSOR_LSM6DSL_0, MOTION_GYRO) == BSP_ERROR_NONE))
  {
    SensorsUp |= SENSORS_MOTION;
  }
  if ((BSP_MOTION_SENSOR_Init(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO) == BSP_ERROR_NONE) &&
      (BSP_MOTION_SENSOR_Enable(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO) == BSP_ERROR_NONE))
  {
    SensorsUp |= SENSORS_FIELD;
  }
}

/**
  * @brief  The readings, every SENSORS_PERIOD_MS. The temperatures only every
  *         SENSORS_TEMP_MS: the magnetometer measures its own when asked.
  */
static void sensors_task(void *pArg)
{
  readings_t r = {0};
  uint32_t   lastTemp = 0U;
  uint8_t    tempTaken = 0U;

  (void)pArg;
  for (;;)
  {
    BSP_MOTION_SENSOR_Axes_t a = {0};
    BSP_MOTION_SENSOR_Axes_t g = {0};
    BSP_MOTION_SENSOR_Axes_t m = {0};
    const uint8_t takeTemp = ((tempTaken == 0U) ||
                              ((HAL_GetTick() - lastTemp) >= SENSORS_TEMP_MS)) ? 1U : 0U;

    (void)xSemaphoreTake(SensorBus, portMAX_DELAY);
    r.valid = 0U;
    if (((SensorsUp & SENSORS_MOTION) != 0U) &&
        (BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO, &a) == BSP_ERROR_NONE) &&
        (BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_LSM6DSL_0, MOTION_GYRO, &g) == BSP_ERROR_NONE))
    {
      /* milli-g and milli-degrees a second */
      r.accel[0] = (float)a.x / 1000.0f;
      r.accel[1] = (float)a.y / 1000.0f;
      r.accel[2] = (float)a.z / 1000.0f;
      r.gyro[0]  = (float)g.x / 1000.0f;
      r.gyro[1]  = (float)g.y / 1000.0f;
      r.gyro[2]  = (float)g.z / 1000.0f;
      if (takeTemp != 0U)
      {
        (void)BSP_MOTION_SENSOR_GetTemperature(MOTION_SENSOR_LSM6DSL_0, &r.imuTemp);
      }
      r.valid |= SENSORS_MOTION;
    }
    if (((SensorsUp & SENSORS_FIELD) != 0U) &&
        (BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO, &m) == BSP_ERROR_NONE))
    {
      /* milligauss: ten to the microtesla */
      r.field[0] = (float)m.x / 10.0f;
      r.field[1] = (float)m.y / 10.0f;
      r.field[2] = (float)m.z / 10.0f;
      if (takeTemp != 0U)
      {
        (void)BSP_MOTION_SENSOR_GetTemperature(MOTION_SENSOR_MMC5603NJ_0, &r.magTemp);
      }
      r.valid |= SENSORS_FIELD;
    }
    (void)xSemaphoreGive(SensorBus);

    if (takeTemp != 0U)
    {
      lastTemp  = HAL_GetTick();
      tempTaken = 1U;
    }

    taskENTER_CRITICAL();
    Readings = r;
    taskEXIT_CRITICAL();

    vTaskDelay(pdMS_TO_TICKS(SENSORS_PERIOD_MS));
  }
}

/* The latest readings, copied out whole. */
static readings_t readings(void)
{
  readings_t r;

  taskENTER_CRITICAL();
  r = Readings;
  taskEXIT_CRITICAL();
  return r;
}

/* The metadata a frame carries: the readings as they are now. */
static uint32_t sensor_meta(void)
{
  const readings_t r = readings();
  uint32_t n = 0U;

  if ((r.valid & SENSORS_MOTION) != 0U)
  {
    const float v[7] = { r.accel[0], r.accel[1], r.accel[2],
                         r.gyro[0], r.gyro[1], r.gyro[2], r.imuTemp };

    SensorMeta[n++] = META_IMU;
    SensorMeta[n++] = (uint8_t)sizeof(v);
    SensorMeta[n++] = 0U;
    (void)memcpy(&SensorMeta[n], v, sizeof(v));
    n += sizeof(v);
  }
  if ((r.valid & SENSORS_FIELD) != 0U)
  {
    const float v[4] = { r.field[0], r.field[1], r.field[2], r.magTemp };

    SensorMeta[n++] = META_MAG;
    SensorMeta[n++] = (uint8_t)sizeof(v);
    SensorMeta[n++] = 0U;
    (void)memcpy(&SensorMeta[n], v, sizeof(v));
    n += sizeof(v);
  }
  return n;
}

/* USB output */

/* Put a frame in flight: the header, metadata from on-chip RAM, and a JPEG
 * from external RAM or nothing. */
static void vnd_start(vnd_kind_t Kind, const uint8_t *pMeta, uint32_t MetaLen,
                      const uint8_t *pPayload, uint32_t PayloadLen)
{
  VndHdr.magic       = VND_MAGIC;
  VndHdr.frame_id    = VndFrameId++;
  VndHdr.meta_len    = MetaLen;
  VndHdr.payload_len = PayloadLen;
  VndHdr.flags       = (Kind == VND_JPEG) ? VND_FLAG_JPEG : 0U;
  VndKind       = Kind;
  VndMeta       = pMeta;
  VndMetaLen    = MetaLen;
  VndPayload    = pPayload;
  VndPayloadLen = PayloadLen;
  VndOffset     = 0U;
  VndTotal      = sizeof(VndHdr) + MetaLen + PayloadLen;
  VndProgress   = HAL_GetTick();
  VndActive     = 1U;
}

/* The frame in flight is done with, sent or dropped. */
static void vnd_finish(bool Sent)
{
  VndActive = 0U;
  if (VndKind == VND_JPEG)
  {
    TxSeq++;
  }
  else if (VndKind == VND_THERMAL)
  {
    ThermalState = RECORD_FREE;
  }
  else
  {
    /* nothing held */
  }
  if (Sent) { StatSent++; } else { StatDropped++; }
}

/* Choose what goes next, if the link is free: a thermal frame, then a JPEG,
 * then a status line, then the empty thermal record or, with no camera, the
 * readings. */
static void vnd_next(void)
{
  const uint32_t now = HAL_GetTick();

  if (VndActive != 0U)
  {
    return;
  }
  if (ThermalState == RECORD_READY)
  {
    ThermalState = RECORD_SENDING;
    vnd_start(VND_THERMAL, ThermalRecord, ThermalRecordLen, NULL, 0U);
  }
  else if (TxSeq != EncSeq)
  {
    const uint32_t slot = TxSeq & 1U;

    __DMB();               /* the frame's length is read after its post */
    vnd_start(VND_JPEG, SensorMeta, sensor_meta(), JpegBuf[slot], PostedBytes[slot]);
  }
  else if (TextSent != TextSaid)
  {
    const char    *line = TextLines[TextSent % TEXT_LINES];
    const uint32_t len  = (uint32_t)strlen(line);

    TextRecord[0] = META_TEXT;
    TextRecord[1] = (uint8_t)len;
    TextRecord[2] = (uint8_t)(len >> 8);
    (void)memcpy(&TextRecord[3], line, len);
    TextSent++;
    vnd_start(VND_OTHER, TextRecord, 3U + len, NULL, 0U);
  }
  else if ((ThermalFrames == 0U) && ((now - LastAnnounce) >= THERMAL_ANNOUNCE_MS))
  {
    LastAnnounce = now;
    vnd_start(VND_OTHER, ThermalAnnounce, thermal_record(ThermalAnnounce, NULL), NULL, 0U);
  }
  else if ((CameraStatus != BSP_ERROR_NONE) && ((now - LastSensorFrame) >= SENSORS_PERIOD_MS))
  {
    const uint32_t len = sensor_meta();

    LastSensorFrame = now;
    if (len != 0U)
    {
      vnd_start(VND_OTHER, SensorMeta, len, NULL, 0U);
    }
  }
  else
  {
    /* nothing to send */
  }
}

/* Copy a double word per bus access: from uncached PSRAM the C library's
 * memcpy moves a byte per access. */
static void vnd_copy(uint64_t *pDst, const uint8_t *pSrc, uint32_t Bytes)
{
  uint8_t  *dst = (uint8_t *)pDst;
  uint32_t  i   = 0U;

  if (((uint32_t)pSrc & 7U) == 0U)
  {
    const uint64_t *src = (const uint64_t *)(const void *)pSrc;
    for (; (i + 8U) <= Bytes; i += 8U)
    {
      pDst[i / 8U] = src[i / 8U];
    }
  }
  for (; i < Bytes; i++)
  {
    dst[i] = pSrc[i];
  }
}

/**
  * @brief  Push as much of the frame in flight as the vendor FIFO takes.
  */
static void vnd_pump(void)
{
  uint32_t before;

  if (!usb_ready())
  {
    /* No host: drop what is in flight and what is waiting. */
    if (VndActive != 0U)
    {
      vnd_finish(false);
    }
    StatDropped += EncSeq - TxSeq;
    TxSeq = EncSeq;
    if (ThermalState == RECORD_READY)
    {
      ThermalState = RECORD_FREE;
    }
    return;
  }

  vnd_next();
  if (VndActive == 0U)
  {
    return;
  }

  /* A host that stops reading mid-frame does not disconnect. */
  before = VndOffset;
  while (VndOffset < VndTotal)
  {
    const uint32_t avail = tud_vendor_n_write_available(0);
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
    else if (VndOffset < (sizeof(VndHdr) + VndMetaLen))
    {
      const uint32_t off = VndOffset - sizeof(VndHdr);

      chunk   = VndMetaLen - off;
      chunk   = (chunk < avail) ? chunk : avail;
      written = tud_vendor_n_write(0, VndMeta + off, chunk);
    }
    else
    {
      const uint32_t off       = VndOffset - sizeof(VndHdr) - VndMetaLen;
      const uint32_t remaining = VndPayloadLen - off;

      chunk = (remaining < avail) ? remaining : avail;
      chunk = (chunk < VND_SLICE_BYTES) ? chunk : VND_SLICE_BYTES;
      if (chunk < remaining)
      {
        chunk &= ~7U;              /* whole double words until the tail */
      }
      if (chunk == 0U)
      {
        break;
      }
      vnd_copy(VndSlice, VndPayload + off, chunk);
      written = tud_vendor_n_write(0, VndSlice, chunk);
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
    vnd_finish(true);
  }
  else if (VndOffset != before)
  {
    VndProgress = HAL_GetTick();          /* it moved: the host is reading */
  }
  else if ((VndOffset == 0U) && ((HAL_GetTick() - VndProgress) >= VND_STALL_MS))
  {
    vnd_finish(false);                    /* nobody is reading: a fresher one */
  }
  else
  {
    /* A partly sent frame waits rather than being dropped: a returning
     * reader would otherwise find the next frame's bytes inside this one's
     * length. */
  }
}

/* Console */

static void console(void)
{
  uint8_t ch;

  (void)BSP_WDG_Refresh();
  while (BSP_COM_Read(&ch, 1U) == 1)
  {
    if (ch == (uint8_t)'b')
    {
      /* Never reset with the encoder in flight: stop the pipe and let the
       * encoder go idle first. */
      (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
      (void)BSP_VENC_Quiesce(0);
      BSP_COM_RebootToBootloader();        /* does not return */
    }
  }
}

/* Say how a thermal start went. */
static void say_thermal_start(void)
{
  BSP_THERMAL_Health_t h = {0};
  const char *sign;
  long whole;
  unsigned long hundredths;

  if (ThermalStatus != BSP_ERROR_NONE)
  {
    say("[thermal] camera not started: %ld; trying again every %lu s",
        (long)ThermalStatus, (unsigned long)(THERMAL_RETRY_MS / 1000U));
    return;
  }
  (void)BSP_THERMAL_GetHealth(0, &h);
  celsius(ThermalSensorKelvin, &whole, &hundredths, &sign);
  say("[thermal] Lepton %04X%04X%04X%04X, sensor at %s%ld.%02lu C, video at %lu Hz%s",
      ThermalSerial[3], ThermalSerial[2], ThermalSerial[1], ThermalSerial[0],
      sign, whole, hundredths, (unsigned long)h.SckHz,
      (ThermalGainOn != 0U) ? "; automatic gain is on, so the pixels are not temperatures" : "");
}

/* How the start went, once the console is there to hear it. */
static void say_start(void)
{
  if (CameraStatus == BSP_ERROR_NONE)
  {
    say("[camera] VD66GY %lux%lu at %lu frames a second, %s lanes",
        (unsigned long)StreamBand.CodedWidth, (unsigned long)StreamBand.CodedHeight,
        (unsigned long)CAM_FRAME_RATE,
        (CAM_CARRIER == BSP_CAMERA_CARRIER_INVERTED) ? "crossed" : "straight");
  }
  else
  {
    say("[camera] VD66GY not started at %s: %ld", CameraStep, (long)CameraStatus);
  }
  say_thermal_start();
  say("[sensors] motion %s, magnetic field %s",
      ((SensorsUp & SENSORS_MOTION) != 0U) ? "up" : "not found",
      ((SensorsUp & SENSORS_FIELD) != 0U) ? "up" : "not found");
}

/**
  * @brief  What the thermal task did, the once-a-second report, and the
  *         thermal link's health every HEALTH_MS.
  */
static void report_step(void)
{
  static uint32_t lastReport, lastHealth;
  static uint32_t camFramesThen, thermalThen, busyThen;
  static uint32_t triesSaid = 1U, cyclesSaid;
  const uint32_t  now = HAL_GetTick();

  if (ThermalTries != triesSaid)
  {
    /* A retry is reported only when it works. */
    triesSaid = ThermalTries;
    if (ThermalStatus == BSP_ERROR_NONE)
    {
      say_thermal_start();
    }
  }
  if (ThermalPowerCycles != cyclesSaid)
  {
    cyclesSaid = ThermalPowerCycles;
    say("[thermal] no frame for %lu s: power-cycled the camera",
        (unsigned long)(THERMAL_RESTART_MS / 1000U));
  }

  if ((now - lastReport) >= REPORT_MS)
  {
    const uint32_t   camFrames = FrameEnds;
    const uint32_t   thermal   = ThermalFrames;
    const uint32_t   busy      = ThermalBusy;
    const readings_t r         = readings();
    const uint16_t   centreK   = ThermalCentreKelvin;
    const char      *sign;
    long             whole;
    unsigned long    hundredths;
    char             centre[16];

    lastReport = now;
    if (centreK == 0U)
    {
      /* No frame yet: the camera reports no pixel at 0 K. */
      (void)snprintf(centre, sizeof(centre), "-");
    }
    else
    {
      celsius(centreK, &whole, &hundredths, &sign);
      (void)snprintf(centre, sizeof(centre), "%s%ld.%02luC", sign, whole, hundredths);
    }
    (void)BSP_COM_Printf("[n6] camera=%lu enc=%lu fail=%lu torn=%lu resync=%lu sent=%lu drop=%lu"
                         " thermal=%lu busy=%lu centre=%s\r\n",
                         (unsigned long)(camFrames - camFramesThen), (unsigned long)StatEncoded,
                         (unsigned long)StatFailed, (unsigned long)StatTorn,
                         (unsigned long)StatResync, (unsigned long)StatSent,
                         (unsigned long)StatDropped, (unsigned long)(thermal - thermalThen),
                         (unsigned long)(busy - busyThen), centre);
    (void)BSP_COM_Printf("[n6] accel=%ld,%ld,%ld mg field=%ld,%ld,%ld mG\r\n",
                         (long)(r.accel[0] * 1000.0f), (long)(r.accel[1] * 1000.0f),
                         (long)(r.accel[2] * 1000.0f), (long)(r.field[0] * 10.0f),
                         (long)(r.field[1] * 10.0f), (long)(r.field[2] * 10.0f));
    camFramesThen = camFrames;
    thermalThen   = thermal;
    busyThen      = busy;
    StatEncoded = 0U; StatFailed = 0U; StatTorn = 0U; StatResync = 0U;
    StatSent    = 0U; StatDropped = 0U;
  }

  if ((ThermalStatus == BSP_ERROR_NONE) && ((now - lastHealth) >= HEALTH_MS))
  {
    BSP_THERMAL_Health_t h = {0};

    lastHealth = now;
    (void)BSP_THERMAL_GetHealth(0, &h);
    say("[thermal] link %lu Hz: frames %lu, segments %lu, resyncs %lu, restarts %lu, overruns %lu, errors %lu",
        (unsigned long)h.SckHz, (unsigned long)h.FramesParsed, (unsigned long)h.SegmentsParsed,
        (unsigned long)h.Resyncs, (unsigned long)h.Restarts, (unsigned long)h.Overruns,
        (unsigned long)h.SpiErrors);
  }
}

static void usb_task(void *pArg)
{
  (void)pArg;

  BSP_COM_SetPumpTask(xTaskGetCurrentTaskHandle());
  say_start();
  for (;;)
  {
    BSP_COM_TaskWait(1U);
    vnd_pump();
    console();
    report_step();
  }
}

/* Watchdog early warning, 300 ms before the reset, on whatever context is
 * stuck: stop capture and the encoder, then the bootloader. */
void BSP_WDG_EWI_Callback(void)
{
  (void)BSP_CAMERA_HaltCapture(0);
  (void)BSP_VENC_Quiesce(0);
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

  /* The watchdog first; a reset it caused goes to the bootloader, not back
   * into the code that caused it. */
  HAL_PWR_EnableBkUpAccess();
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U)
  {
    __HAL_RCC_CLEAR_RESET_FLAGS();
    BSP_COM_RebootToBootloader();
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
  (void)BSP_WDG_InitEWI(WDG_TIMEOUT_MS, WDG_WARN_MS);

  /* Motion sensors, then the thermal camera (it boots only on a quiet bus),
   * then the global-shutter camera and encoder, then USB. No kernel object
   * exists until all of it has run. */
  sensors_init();
  (void)thermal_init();
  CameraStatus = camera_init();

  if (BSP_COM_Init() != BSP_ERROR_NONE)
  {
    (void)BSP_LED_On(LED_RED);
    Error_Handler();
  }
  if ((CameraStatus != BSP_ERROR_NONE) || (ThermalStatus != BSP_ERROR_NONE))
  {
    /* Part of it is missing; the console says which. */
    (void)BSP_LED_On(LED_RED);
  }
  (void)BSP_LED_On(LED_GREEN);

  ThermalHalf = xSemaphoreCreateBinary();
  SensorBus   = xSemaphoreCreateMutex();
  if ((ThermalHalf == NULL) || (SensorBus == NULL) ||
      (xTaskCreate(usb_task, "usb", USB_TASK_STACK_WORDS, NULL,
                   USB_TASK_PRIORITY, NULL) != pdPASS) ||
      (xTaskCreate(encode_task, "encode", ENCODE_TASK_STACK_WORDS, NULL,
                   ENCODE_TASK_PRIORITY, &EncodeTask) != pdPASS) ||
      (xTaskCreate(thermal_task, "thermal", THERMAL_TASK_STACK_WORDS, NULL,
                   THERMAL_TASK_PRIORITY, NULL) != pdPASS) ||
      (xTaskCreate(sensors_task, "sensors", SENSORS_TASK_STACK_WORDS, NULL,
                   SENSORS_TASK_PRIORITY, NULL) != pdPASS))
  {
    (void)BSP_LED_On(LED_RED);
    Error_Handler();
  }
  vTaskStartScheduler();
  Error_Handler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
