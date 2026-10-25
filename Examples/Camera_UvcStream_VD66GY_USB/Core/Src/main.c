/*
  Camera_UvcStream_VD66GY_USB

  Streams the VD66GY global-shutter camera as a standard USB video device:
  VGA, Motion-JPEG, sixty frames a second. Any webcam application shows the
  picture, with no driver. The camera pipe turns the sensor's raw data into
  colour with ST's daylight tuning for this module, so other light looks
  warmer or cooler.

  The circuit:
  - Neuro N6 board with the VD66GY camera on the thermal carrier. The
    carrier's thermal camera is not used.

  A USB console enumerates alongside the camera: send 'b' on it to return to
  the bootloader. Every five seconds it says how the camera came up.

  Copyright (c) 2026 Ohm Lab Ltd.
  SPDX-License-Identifier: CC0-1.0
  Released under CC0 1.0 Universal: copy it into your own project
  freely, with or without attribution.
*/

#include "main.h"

#include "neuron6.h"
#include "neuron6_bus.h"
#include "neuron6_com.h"
#include "neuron6_camera.h"
#include "neuron6_camera_vd66gy.h"
#include "neuron6_venc.h"
#include "neuron6_wdg.h"

#include "tusb.h"

#include <stdbool.h>

/* VGA, from the middle of the 1120 x 1364 array; up to 88 fps, held to the
 * rate the descriptors declare. Size and rate are set in neuron6_conf.h. */
#define UVC_RESOLUTION          BSP_CAMERA_VD66GY_R640x480

/* The descriptors' defaults, if neuron6_conf.h leaves them unset. */
#ifndef UVC_FRAME_WIDTH
#define UVC_FRAME_WIDTH         640
#endif
#ifndef UVC_FRAME_HEIGHT
#define UVC_FRAME_HEIGHT        480
#endif
#ifndef UVC_FRAME_RATE
#define UVC_FRAME_RATE          60
#endif

/* The thermal carrier crosses the two data lanes; the NeuroVision carrier
 * needs BSP_CAMERA_CARRIER_STANDARD. A wrong setting gives no error, only no
 * frames, which the console reports. */
#define UVC_CARRIER             BSP_CAMERA_CARRIER_INVERTED

/* Target of the sensor's exposure loop, 0 to 100. It meters the raw picture,
 * before white balance and gamma brighten it, so a middle target overexposes. */
#define UVC_EXPOSURE_TARGET     25

/* The band the camera writes and the encoder reads, sized for VGA. */
#define UVC_BAND_BYTES          81920U

/* One finished JPEG: a VGA frame runs to a few tens of kilobytes. */
#define JPEG_BUF_BYTES          (256U * 1024U)

/* Covers the sensor's start-up, which sends it 17 KB of firmware over I2C
 * outside the main loop. */
#define WDG_TIMEOUT_MS          4000U
#define WDG_WARN_MS             300U

/* If no host has enumerated the device by then, return to the bootloader so
 * the board can be reflashed. Applies until a host is seen; 0 disables it. */
#ifndef UVC_HOST_TIMEOUT_MS
#define UVC_HOST_TIMEOUT_MS     30000U
#endif

/* A run of failed encodes, or of torn frames, means the encoder and camera
 * have slipped: stop, reset the encoder and start again. */
#define UVC_RESYNC_AFTER_FAILS  8U
#define UVC_RESYNC_AFTER_TORN   2U

/* The sensor's state register: 1 ready to start, 2 standing by, 3 streaming. */
#define VD66GY_REG_SYSTEM_FSM   0x0028U
#define VD66GY_STATE_NO_ANSWER  0xFFU

/* The camera writes here, wrapped, and the encoder follows it. */
static uint8_t Band[UVC_BAND_BYTES]
  __attribute__((section(".axisram_nc"), aligned(32)));

/* Two finished frames: one on the wire while the next is encoded. */
static uint8_t JpegBuf[2][JPEG_BUF_BYTES]
  __attribute__((section(".ext_psram"), aligned(32)));

/* Frame N is encoded into JpegBuf[N & 1]. EncSeq counts frames posted, TxSeq
 * frames the host has taken: at most two are outstanding. */
static uint32_t         EncSeq;
static volatile uint32_t TxSeq;
static uint32_t         PostedBytes[2];
static volatile uint8_t FrameInFlight;

static BSP_VENC_Band_t  StreamBand;
static uint8_t          LinebufArmed;
static int32_t          InitStatus = BSP_ERROR_NONE;
static uint32_t         ConsecFail, ConsecTorn;
static uint32_t         StatEncoded, StatSent, StatFailed, StatTorn, StatResync;
static uint32_t         StatBytes, StatGap;

/* How the camera came up, for the console. */
static const char      *InitStep = "not started";
static uint8_t          StateBefore = VD66GY_STATE_NO_ANSWER;
static uint32_t         InitMs;

/* Counted by the camera pipe's callbacks. */
static volatile uint32_t FrameEvents;
static volatile uint32_t ErrorEvents;
static volatile uint32_t LastErrorCode;

/* Non-zero while the camera is inside a frame. */
static volatile uint32_t FrameActive;
/* A wait for the frame to end: flag, start time, FrameEvents at the start. */
static volatile uint8_t  GapWaiting;
static uint32_t          GapWaitSince;
static uint32_t          GapWaitFrames;
/* How long to wait for a frame to end before taking the camera for stopped:
 * five frame periods (a sensor may stretch its frames), at least 150 ms. */
static uint32_t          GapWaitMs = 1000U;

static void    mpu_config(void);
static void    service(void);
static void    uvc_pump(void);
static int32_t video_init(void);

void Error_Handler(void);

/* Formatted output to the USB console. Dropped, not blocked, with no host. */
#define print(...)   (void)BSP_COM_Printf(__VA_ARGS__)

/* Sensor */

/**
  * @brief  Ask the sensor what state it is in.
  * @retval Its state register, or VD66GY_STATE_NO_ANSWER.
  */
static uint8_t sensor_state(void)
{
  uint8_t state = VD66GY_STATE_NO_ANSWER;

  if (BSP_I2C1_ReadReg16(BSP_CAMERA_VD66GY_I2C_ADDRESS, VD66GY_REG_SYSTEM_FSM,
                         &state, 1U) != BSP_ERROR_NONE)
  {
    return VD66GY_STATE_NO_ANSWER;
  }
  return state;
}

static const char *state_name(uint8_t State)
{
  switch (State)
  {
    case 0x01U:                  return "ready to start";
    case 0x02U:                  return "standing by";
    case 0x03U:                  return "streaming";
    case VD66GY_STATE_NO_ANSWER: return "no answer";
    default:                     return "unknown";
  }
}

static const char *bayer_name(uint32_t Bayer)
{
  switch (Bayer)
  {
    case BSP_CAMERA_BAYER_GRBG: return "GRBG";
    case BSP_CAMERA_BAYER_GBRG: return "GBRG";
    case BSP_CAMERA_BAYER_BGGR: return "BGGR";
    default:                    return "RGGB or none";
  }
}

/**
  * @brief  The pipe's processing, from ST's daylight (D65) tuning for the
  *         VD66GY module (STEVAL-66GYMAI1).
  * @note   Black level out of 255, white-balance gains in 128ths, colour
  *         correction in 256ths with each row summing to 256 (grey stays grey).
  */
static int32_t isp_config(void)
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

/* Capture */

/**
  * @brief  Start the camera. The encoder calls this once per stream, after it
  *         has parked waiting for the first line event.
  */
static int32_t camera_start(void *pArg)
{
  (void)pArg;
  return (BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, Band,
                           BSP_CAMERA_MODE_CONTINUOUS) == BSP_ERROR_NONE) ? 0 : -1;
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
  * @brief  Each line event tells the encoder more of the band is written.
  */
void BSP_CAMERA_LineEventCallback(uint32_t Instance, uint32_t Pipe)
{
  if ((Instance == 0U) && (Pipe == BSP_CAMERA_PIPE_MAIN))
  {
    (void)BSP_VENC_SourceLineEvent(0);
  }
}

/**
  * @brief  Frame end: anchors the encoder's row count, so an encode armed late
  *         reads as a gap, not a clean frame.
  */
void BSP_CAMERA_FrameEventCallback(uint32_t Instance, uint32_t Pipe)
{
  if ((Instance == 0U) && (Pipe == BSP_CAMERA_PIPE_MAIN))
  {
    FrameActive = 0U;
    FrameEvents++;
    (void)BSP_VENC_SourceFrameEvent(0);
  }
}

/**
  * @brief  Count pipe errors: errors and no frames is a link that never
  *         synchronised.
  */
void BSP_CAMERA_ErrorCallback(uint32_t Instance, uint32_t Pipe)
{
  (void)Instance;
  (void)Pipe;
  ErrorEvents++;
  LastErrorCode = hcamera_dcmipp.ErrorCode;
}

/**
  * @brief  Keep USB moving while BSP_VENC_EncodeBand waits a whole frame for
  *         the camera; the previous frame goes out from here.
  */
void BSP_VENC_ReadPointerHook(uint32_t MbRow)
{
  (void)MbRow;
  service();
}

/**
  * @brief  Sensor, camera, pipe, processing, line buffer, prime, encoder.
  */
static int32_t video_init(void)
{
  BSP_CAMERA_PipeConfig_t   pipe = {0};
  BSP_CAMERA_Capabilities_t cap  = {0};
  BSP_VENC_Config_t         venc = {0};
  BSP_VENC_Band_t           band;
  uint32_t                  started;
  int32_t                   status;

  /* The watchdog runs through bring-up, which never reaches the main loop. */
  (void)BSP_WDG_Refresh();

  /* Own the sensor bus, to read the sensor's state even if the camera fails. */
  InitStep = "sensor bus";
  status = BSP_I2C1_Init();
  if (status != BSP_ERROR_NONE) { return status; }
  StateBefore = sensor_state();

  InitStep = "sensor choice";
  status = BSP_CAMERA_VD66GY_Register();
  if (status == BSP_ERROR_NONE) { status = BSP_CAMERA_SetCarrier(UVC_CARRIER); }
  if (status == BSP_ERROR_NONE) { status = BSP_CAMERA_VD66GY_SetFrameRate(UVC_FRAME_RATE); }
  if (status != BSP_ERROR_NONE) { return status; }

  /* Identifies the sensor, loads its firmware and sets it up, idle. */
  InitStep = "camera init";
  started  = HAL_GetTick();
  status   = BSP_CAMERA_Init(0, UVC_RESOLUTION, BSP_CAMERA_PF_RAW8);
  InitMs   = HAL_GetTick() - started;
  if (status != BSP_ERROR_NONE) { return status; }
  (void)BSP_WDG_Refresh();

  /* The sensor's size, less bottom lines the encoder cannot code, must match
   * what the USB descriptors declare. */
  InitStep = "picture size";
  status = BSP_CAMERA_GetCapabilities(0, &cap);
  if (status != BSP_ERROR_NONE) { return status; }
  {
    const uint32_t fps = ((uint32_t)UVC_FRAME_RATE < cap.MaxFrameRate)
                         ? (uint32_t)UVC_FRAME_RATE : cap.MaxFrameRate;

    GapWaitMs = (fps != 0U) ? (5000U / fps) : 1000U;
    if (GapWaitMs < 150U) { GapWaitMs = 150U; }
  }

  status = BSP_VENC_GetBandGeometry(cap.Width, cap.Height, &band);
  if (status != BSP_ERROR_NONE) { return status; }
  if (band.SizeBytes > sizeof(Band)) { return BSP_ERROR_WRONG_PARAM; }
  if ((band.CodedWidth != (uint32_t)UVC_FRAME_WIDTH) ||
      (band.CodedHeight != (uint32_t)UVC_FRAME_HEIGHT))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* The pipe takes the sensor's whole frame; the encoder codes the top
   * band.CodedHeight lines of it. */
  InitStep = "camera pipe";
  pipe.PixelFormat = BSP_CAMERA_PIPE_PF_YUV422;
  pipe.Width       = cap.Width;
  pipe.Height      = cap.Height;
  status = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pipe);
  if (status != BSP_ERROR_NONE) { return status; }
  (void)BSP_WDG_Refresh();

  /* After the pipe: configuring it resets its colour reconstruction. */
  InitStep = "image processing";
  status = isp_config();
  if (status != BSP_ERROR_NONE) { return status; }

  InitStep = "line buffer";
  status = BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN,
                                       band.WrapLines, BSP_CAMERA_LINES_OFF);
  if (status != BSP_ERROR_NONE) { return status; }
  (void)BSP_WDG_Refresh();

  /* Prime: start the pipe once and leave it running. The encoder's first arm
   * expects a pipe started and stopped before; the first capture stops it. */
  InitStep = "camera start";
  status = BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, Band, BSP_CAMERA_MODE_CONTINUOUS);
  if (status != BSP_ERROR_NONE) { return status; }
  (void)BSP_WDG_Refresh();

  /* Set once the sensor streams. */
  InitStep = "exposure target";
  status = BSP_CAMERA_VD66GY_SetBrightness(UVC_EXPOSURE_TARGET);
  if (status != BSP_ERROR_NONE) { return status; }

  StreamBand   = band;
  LinebufArmed = 0U;

  InitStep = "encoder";
  venc.Band        = band;
  venc.pBand       = Band;
  venc.Quality     = BSP_VENC_QUALITY_DEFAULT;
  venc.SourceStart = camera_start;
  venc.pSourceArg  = NULL;
  status = BSP_VENC_Init(0, &venc);
  if (status == BSP_ERROR_NONE)
  {
    InitStep = "none";
  }
  return status;
}

/**
  * @brief  Recovery: stop the pipe, reset the encoder, and let the
  *         next capture reconfigure and start again, encoder first.
  */
static void uvc_resync(void)
{
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
  (void)BSP_VENC_Resync(0);
  LinebufArmed = 0U;
  ConsecFail   = 0U;
  ConsecTorn   = 0U;
  FrameActive  = 0U;        /* the stopped camera's frame will not end */
  GapWaiting   = 0U;
  StatResync++;
}

/**
  * @brief  One pass of the capture loop: encode the next frame unless
  *         the host is two behind, and post it unless it is torn.
  */
static void uvc_capture_step(void)
{
  BSP_VENC_Statistics_t st;
  uint32_t              bytes = 0U;
  uint32_t              slot;

  if ((EncSeq - TxSeq) >= 2U)
  {
    GapWaiting = 0U;
    return;
  }

  /* First capture, or first after a resync: stop the free-running pipe (which
   * flushes the receiver) and arm the line event the encoder paces itself on;
   * the encode below restarts the pipe once the encoder is parked. */
  if (!LinebufArmed)
  {
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
    (void)BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN,
                                      StreamBand.WrapLines, StreamBand.EventLines);
    LinebufArmed = 1U;
  }
  else if (FrameActive != 0U)
  {
    /* Mid-frame: an encode armed now would code the rest of this frame and
     * the top of the next as one picture, so wait for the vertical blanking.
     * The frame counts from its start, not its first line event: the encoder
     * takes about half a millisecond to arm and could miss that event. */
    if (GapWaiting == 0U)
    {
      GapWaitSince  = HAL_GetTick();
      GapWaitFrames = FrameEvents;
      GapWaiting    = 1U;
    }
    if ((HAL_GetTick() - GapWaitSince) < GapWaitMs)
    {
      return;
    }
    GapWaiting = 0U;
    if (FrameEvents == GapWaitFrames)
    {
      /* No frame ended: the camera stopped mid-frame. Restart it. */
      uvc_resync();
      return;
    }
    /* Frames ended but every gap was missed: arm anyway, and the torn-frame
     * check drops what that codes. */
  }
  GapWaiting = 0U;

  slot = EncSeq & 1U;
  if (BSP_VENC_EncodeBand(0, JpegBuf[slot], JPEG_BUF_BYTES, &bytes) != BSP_ERROR_NONE)
  {
    StatFailed++;
    if (++ConsecFail >= UVC_RESYNC_AFTER_FAILS)
    {
      uvc_resync();
    }
    return;
  }
  ConsecFail = 0U;

  if (BSP_VENC_GetStatistics(0, &st) == BSP_ERROR_NONE)
  {
    if (st.LastGap > StatGap) { StatGap = st.LastGap; }
  }

  /* A frame the camera lapped is split: never send it. */
  if (BSP_VENC_LastFrameTorn(0) != 0U)
  {
    StatTorn++;
    if (++ConsecTorn >= UVC_RESYNC_AFTER_TORN)
    {
      uvc_resync();
    }
    return;
  }
  ConsecTorn = 0U;

  StatEncoded++;
  StatBytes         = bytes;
  PostedBytes[slot] = bytes;
  EncSeq++;
  uvc_pump();
}

/* USB */

/**
  * @brief  Put the oldest posted frame on the wire if the wire is free.
  */
static void uvc_pump(void)
{
  uint32_t slot;

  if (FrameInFlight || (EncSeq == TxSeq) || !tud_video_n_streaming(0, 0))
  {
    return;
  }

  slot = TxSeq & 1U;
  if (tud_video_n_frame_xfer(0, 0, JpegBuf[slot], (size_t)PostedBytes[slot]))
  {
    FrameInFlight = 1U;
  }
}

/**
  * @brief  The host has taken a whole frame; its buffer is free.
  */
void tud_video_frame_xfer_complete_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx)
{
  (void)ctl_idx;
  (void)stm_idx;
  FrameInFlight = 0U;
  if (TxSeq != EncSeq)
  {
    TxSeq++;
    StatSent++;
  }
}

/* Scaffolding */

/**
  * @brief  Say how the camera came up, and what it is doing now.
  * @param  Frames   Frames the pipe delivered in the last second.
  * @param  Streamed Whether a host has streamed yet; until then the pipe runs
  *                  on its own, so a second with no frame means a fault.
  * @note   Errors count from the start: the pipe reports a few as it is
  *         stopped and started.
  */
static void camera_report(uint32_t Frames, uint8_t Streamed)
{
  uint32_t bayer = BSP_CAMERA_BAYER_RGGB;
  uint8_t  now   = sensor_state();

  if (InitStatus != BSP_ERROR_NONE)
  {
    print("[cam] VD66GY failed at %s, status %ld, after %lu ms; the sensor was %s at start and is %s now\r\n",
          InitStep, (long)InitStatus, (unsigned long)InitMs,
          state_name(StateBefore), state_name(now));
    if ((StateBefore == 0x02U) || (StateBefore == 0x03U))
    {
      print("[cam] the sensor was already running when this program started, and its driver starts it only from power-up: switch the board off and on\r\n");
    }
    return;
  }

  (void)BSP_CAMERA_VD66GY_GetBayerOrder(&bayer);
  print("[cam] VD66GY %ux%u at %u fps, %s lanes: started in %lu ms from %s, now %s, filter %s, %lu frames in the last second, %lu camera errors since start (codes 0x%lx)\r\n",
        (unsigned)UVC_FRAME_WIDTH, (unsigned)UVC_FRAME_HEIGHT, (unsigned)UVC_FRAME_RATE,
        (UVC_CARRIER == BSP_CAMERA_CARRIER_INVERTED) ? "crossed" : "straight",
        (unsigned long)InitMs, state_name(StateBefore), state_name(now), bayer_name(bayer),
        (unsigned long)Frames, (unsigned long)ErrorEvents, (unsigned long)LastErrorCode);
  if ((Streamed == 0U) && (Frames == 0U))
  {
    print("[cam] no frame arrives: with the sensor streaming, the lane mapping (UVC_CARRIER) is the first suspect\r\n");
  }
}

/**
  * @brief  Service USB, the watchdog and the console. Called from the main
  *         loop and from inside the encode.
  */
static void service(void)
{
  uint8_t ch;

  BSP_COM_Task();
  (void)BSP_WDG_Refresh();
  uvc_pump();
  if (BSP_COM_Read(&ch, 1U) == 1)
  {
    if (ch == (uint8_t)'b')
    {
      /* Never reset with the encoder in flight: stop the pipe and quiesce
       * the encoder first. */
      (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
      (void)BSP_VENC_Quiesce(0);
      BSP_COM_RebootToBootloader();
    }
  }
}

/**
  * @brief  Runs 300 ms before the watchdog resets the chip, on whatever
  *         context is stuck. Points the reset at the bootloader.
  */
void BSP_WDG_EWI_Callback(void)
{
  (void)BSP_CAMERA_HaltCapture(0);
  (void)BSP_VENC_Quiesce(0);
  BSP_COM_RebootToBootloader();
}

/**
  * @brief  The external memories are not cacheable: the camera and the
  *         encoder write them behind its back.
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

int main(void)
{
  uint32_t report = 0U;
  uint32_t booted = 0U;
  uint32_t seconds = 0U;
  uint32_t frames_seen = 0U;
  uint8_t  seen_a_host = 0U;
  uint8_t  streaming = 0U;
  uint8_t  was_streaming = 0U;
  uint8_t  streamed = 0U;

  SCB_DisableDCache();
  mpu_config();
  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();

  /* No SystemClock_Config: the boot loader owns the clock tree and both
   * external memories; the BSP reads the frequencies back at run time. */

  (void)BSP_LED_Init(LED_RED);
  (void)BSP_LED_Init(LED_GREEN);

  /* A watchdog reset goes to the bootloader, not back into the code that
   * caused it; the watchdog is armed before anything that could wedge. */
  HAL_PWR_EnableBkUpAccess();
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U)
  {
    __HAL_RCC_CLEAR_RESET_FLAGS();
    BSP_COM_RebootToBootloader();
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
  (void)BSP_WDG_InitEWI(WDG_TIMEOUT_MS, WDG_WARN_MS);

  /* Camera and encoder first, USB last, then service it. */
  InitStatus = video_init();

  if (BSP_COM_Init() != BSP_ERROR_NONE)
  {
    (void)BSP_LED_On(LED_RED);
    Error_Handler();
  }

  if (InitStatus != BSP_ERROR_NONE)
  {
    /* No video, but a console and a way back to the bootloader. */
    (void)BSP_LED_On(LED_RED);
  }

  booted = HAL_GetTick();

  while (1)
  {
    service();

    if ((UVC_HOST_TIMEOUT_MS != 0U) && !seen_a_host)
    {
      if (tud_mounted())
      {
        seen_a_host = 1U;
      }
      else if ((HAL_GetTick() - booted) > UVC_HOST_TIMEOUT_MS)
      {
        BSP_COM_RebootToBootloader();
      }
    }

    streaming = tud_video_n_streaming(0, 0) ? 1U : 0U;
    if (streaming == 0U)
    {
      if (was_streaming != 0U)
      {
        /* Alternate setting 0: the host stopped watching. Drop what was
         * outstanding and stop the camera; the next start is encoder-first. */
        (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
        (void)BSP_VENC_Resync(0);
        LinebufArmed  = 0U;
        FrameActive   = 0U;
        GapWaiting    = 0U;
        FrameInFlight = 0U;
        TxSeq         = EncSeq;
        (void)BSP_LED_Off(LED_GREEN);
      }
    }
    else
    {
      streamed = 1U;
      (void)BSP_LED_On(LED_GREEN);
      if (InitStatus == BSP_ERROR_NONE)
      {
        uvc_capture_step();
      }
    }
    was_streaming = streaming;

    if ((HAL_GetTick() - report) >= 1000U)
    {
      uint32_t frames = FrameEvents - frames_seen;

      frames_seen = FrameEvents;
      report = HAL_GetTick();
      print("[uvc] enc=%lu sent=%lu torn=%lu fail=%lu resync=%lu gap=%lu/%lu last=%luB"
            " streaming=%u\r\n",
            (unsigned long)StatEncoded, (unsigned long)StatSent,
            (unsigned long)StatTorn, (unsigned long)StatFailed,
            (unsigned long)StatResync, (unsigned long)StatGap,
            (unsigned long)(StreamBand.Depth * 2U), (unsigned long)StatBytes,
            (unsigned)tud_video_n_streaming(0, 0));
      StatEncoded = 0U; StatSent = 0U; StatTorn = 0U; StatFailed = 0U;
      StatResync  = 0U; StatGap  = 0U;

      /* Every fifth second, and the first: the console may open late. */
      if ((seconds++ % 5U) == 0U)
      {
        camera_report(frames, streamed);
      }
    }
  }
}

/**
  * @brief  Nothing to spin for: leave the board where it can be reflashed.
  */
void Error_Handler(void)
{
  BSP_COM_RebootToBootloader();
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  UNUSED(file);
  UNUSED(line);
}
#endif /* USE_FULL_ASSERT */
