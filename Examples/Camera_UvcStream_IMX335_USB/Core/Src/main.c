/*
  Camera_UvcStream_IMX335_USB

  Streams ST's B-CAMS-IMX camera (Sony IMX335, 5 MP) as a standard USB video
  device: Motion-JPEG at the sensor's full 2592 x 1944, twenty-one frames a
  second, the most the encoder sustains at that size. Any webcam application
  shows the picture, with no driver. The camera pipe turns the sensor's raw
  data into colour, and meters it for the exposure and white-balance loops.

  The circuit:
  - Neuro N6 board with Ohm Lab's B-CAMS-IMX adapter, and ST's B-CAMS-IMX
    module on the adapter's flex connector.

  The module's VL53L5CX time-of-flight sensor ranges 8 x 8 zones alongside.

  A USB console enumerates alongside the camera: send 'b' on it to return to
  the bootloader. Every second it gives the nearest target the time-of-flight
  sensor sees; every five seconds, how the camera came up and the sensor's
  whole grid.

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
#include "neuron6_camera_imx335.h"
#include "neuron6_camera_awb.h"
#include "neuron6_ranging_vl53l5cx.h"
#include "neuron6_venc.h"
#include "neuron6_wdg.h"

#include "tusb.h"

#include <stdbool.h>
#include <stdio.h>

/* The sensor's one mode; the pipe reduces it to the size below. Size and
 * rate are set in neuron6_conf.h. */
#define UVC_RESOLUTION          BSP_CAMERA_IMX335_R2592x1944

/* The descriptors' defaults, if neuron6_conf.h leaves them unset. */
#ifndef UVC_FRAME_WIDTH
#define UVC_FRAME_WIDTH         2592
#endif
#ifndef UVC_FRAME_HEIGHT
#define UVC_FRAME_HEIGHT        1944
#endif
#ifndef UVC_FRAME_RATE
#define UVC_FRAME_RATE          21
#endif

/* How the adapter wires the two data lanes. A wrong setting gives no error,
 * only no frames, which the console reports; BSPTest's bcamslink says which. */
#ifndef UVC_CARRIER
#define UVC_CARRIER             BSP_CAMERA_CARRIER_STANDARD
#endif

/* Sensor line length in percent of ST's 30 fps line: longer lines let the
 * encoder keep up with a wide picture. 100 for every size up to 1920 wide;
 * frame rate x percent may not exceed 3000. */
#ifndef UVC_LINE_PERCENT
#define UVC_LINE_PERCENT        142U
#endif

/* Target of the exposure loop, 0 to 255, on the linear picture before gamma
 * brightens it: 50 lands mid-grey after gamma. */
#define UVC_EXPOSURE_TARGET     50U

/* JPEG quality, 0 to 9 (the BSP's default is 2). */
#define UVC_JPEG_QUALITY        5U

/* The band the camera writes and the encoder reads, sized for 2592 x 1944
 * (fits every smaller picture). */
#define UVC_BAND_BYTES          663552U

/* One finished JPEG: a 2592 x 1944 frame at quality 5 runs to a few hundred
 * kilobytes, megabytes on fine detail. */
#define JPEG_BUF_BYTES          (4U * 1024U * 1024U)

/* Finished frames held for the host: three keep the wire busy while the
 * encoder waits for the next frame to start. */
#define JPEG_BUF_COUNT          3U

#define WDG_TIMEOUT_MS          4000U
#define WDG_WARN_MS             300U

/* Time of flight: 8 x 8 zones, one result a second. A result is read inside an
 * encode while at least half the frame's rows are to come, so the 11 ms read
 * ends before the frame does. Reading stops USB for its length: about 1% of the
 * stream's time at one a second. */
#define UVC_TOF_ZONES           BSP_RANGING_VL53L5CX_ZONES_8X8
#define UVC_TOF_RATE_HZ         1U

/* If no host has enumerated the device by then, return to the bootloader so
 * the board can be reflashed. Applies until a host is seen; 0 disables it. */
#ifndef UVC_HOST_TIMEOUT_MS
#define UVC_HOST_TIMEOUT_MS     30000U
#endif

/* A run of failed encodes, or of torn frames, means the encoder and camera
 * have slipped: stop, reset the encoder and start again. */
#define UVC_RESYNC_AFTER_FAILS  8U
#define UVC_RESYNC_AFTER_TORN   2U

/* The sensor's mode register: 1 standing by, 0 streaming. */
#define IMX335_REG_MODE_SELECT  0x3000U
#define IMX335_STATE_NO_ANSWER  0xFFU

/* The camera writes here, wrapped, and the encoder follows it. */
static uint8_t Band[UVC_BAND_BYTES]
  __attribute__((section(".axisram_nc"), aligned(32)));

static uint8_t JpegBuf[JPEG_BUF_COUNT][JPEG_BUF_BYTES]
  __attribute__((section(".ext_psram"), aligned(32)));

/* Frame N is encoded into JpegBuf[N % JPEG_BUF_COUNT]. EncSeq counts frames
 * posted, TxSeq frames the host has taken: at most JPEG_BUF_COUNT are
 * outstanding. */
static uint32_t         EncSeq;
static volatile uint32_t TxSeq;
static uint32_t         PostedBytes[JPEG_BUF_COUNT];
static volatile uint8_t FrameInFlight;

static BSP_VENC_Band_t  StreamBand;
static uint8_t          LinebufArmed;
static int32_t          InitStatus = BSP_ERROR_NONE;
static uint32_t         ConsecFail, ConsecTorn;
static uint32_t         StatEncoded, StatSent, StatFailed, StatTorn, StatResync;
static uint32_t         StatBytes, StatGap;

/* How the camera came up, for the console. */
static const char      *InitStep = "not started";
static uint8_t          StateBefore = IMX335_STATE_NO_ANSWER;
static uint32_t         InitMs;

/* Counted by the camera pipe's callbacks. */
static volatile uint32_t FrameEvents;
static volatile uint32_t ErrorEvents;
static volatile uint32_t LastErrorCode;

/* The exposure loop's last frame and reading. */
static uint32_t          ExposureFrame;
static uint32_t          ExposureAverage;

/* White balance: the gains that make a grey card grey under ST's three measured
 * lights for the module, warm to cool, in thousandths of green. */
static const BSP_CAMERA_AWB_Ref_t AwbRefs[3] =
{
  { 2810U, 1240U, 2820U },   /* tungsten    */
  { 4015U, 1820U, 2120U },   /* fluorescent */
  { 6650U, 2440U, 1430U },   /* daylight    */
};

/* The white gain lifts a pixel clipped at the sensor to 255 on all three
 * components after the black level: white, not tinted. */
static const BSP_CAMERA_AWB_Config_t AwbConfig =
{
  .pRefs       = AwbRefs,
  .RefCount    = 3U,
  .WhiteGain   = (255000U + ((255U - BSP_CAMERA_IMX335_BLACK_LEVEL) / 2U)) / (255U - BSP_CAMERA_IMX335_BLACK_LEVEL),
  .StepPercent = 30U,
  .MinGreen    = 8U,
};

/* The loop's state. Statistics units: 1 luminance (exposure), 2 red, 3 green
 * and blue in turn; AwbSwitched is the frame unit 3 last changed on. */
static BSP_CAMERA_AWB_State_t AwbState;
static uint32_t          AwbFrame;
static uint32_t          AwbSwitched;
static uint32_t          AwbGreen;
static uint8_t           AwbStarted;
static uint8_t           AwbOnBlue;

/* Non-zero while the camera is inside a frame. */
static volatile uint32_t FrameActive;
/* A wait for the frame to end: flag, start time, FrameEvents at the start. */
static volatile uint8_t  GapWaiting;
static uint32_t          GapWaitSince;
static uint32_t          GapWaitFrames;
/* How long to wait for a frame to end before taking the camera for stopped:
 * five frame periods (a sensor may stretch its frames), at least 150 ms. */
static uint32_t          GapWaitMs = 1000U;

/* The time-of-flight sensor: how it came up, its last result, and the encoder
 * row before which a result may be read. */
static int32_t           TofStatus = BSP_ERROR_NO_INIT;
static BSP_RANGING_VL53L5CX_Result_t Tof;
static uint32_t          TofResults;
static uint32_t          TofRowLimit;

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
  * @retval Its mode register, or IMX335_STATE_NO_ANSWER.
  */
static uint8_t sensor_state(void)
{
  uint8_t state = IMX335_STATE_NO_ANSWER;

  if (BSP_I2C1_ReadReg16(BSP_CAMERA_IMX335_I2C_ADDRESS, IMX335_REG_MODE_SELECT,
                         &state, 1U) != BSP_ERROR_NONE)
  {
    return IMX335_STATE_NO_ANSWER;
  }
  return state;
}

static const char *state_name(uint8_t State)
{
  switch (State)
  {
    case 0x00U:                  return "streaming";
    case 0x01U:                  return "standing by";
    case IMX335_STATE_NO_ANSWER: return "no answer";
    default:                     return "unknown";
  }
}

/**
  * @brief  Write the white balance's gains to the pipe.
  */
static int32_t awb_apply(void)
{
  DCMIPP_ExposureConfTypeDef gains = {0};

  BSP_CAMERA_AWB_ToShiftMultiplier(AwbState.GainR, &gains.ShiftRed,   &gains.MultiplierRed);
  BSP_CAMERA_AWB_ToShiftMultiplier(AwbState.GainG, &gains.ShiftGreen, &gains.MultiplierGreen);
  BSP_CAMERA_AWB_ToShiftMultiplier(AwbState.GainB, &gains.ShiftBlue,  &gains.MultiplierBlue);
  return BSP_CAMERA_SetIspGain(0, BSP_CAMERA_PIPE_MAIN, &gains);
}

/**
  * @brief  One white-balance step every six frames. Unit 3 measures green,
  *         then blue, each read two whole frames after it switches; with unit
  *         2's red, the loop steps. Runs in the main loop.
  */
static void awb_step(void)
{
  const uint32_t frames = FrameEvents;
  uint32_t       value  = 0U;
  uint32_t       red    = 0U;

  if ((InitStatus != BSP_ERROR_NONE) || (frames == AwbFrame))
  {
    return;
  }
  AwbFrame = frames;
  if (((frames - AwbSwitched) < 3U) ||
      (BSP_CAMERA_GetIspStatisticsUnit(0, BSP_CAMERA_PIPE_MAIN, 3U, &value) != BSP_ERROR_NONE))
  {
    return;
  }

  if (AwbOnBlue == 0U)
  {
    AwbGreen = value;
  }
  else if ((BSP_CAMERA_GetIspStatisticsUnit(0, BSP_CAMERA_PIPE_MAIN, 2U, &red) == BSP_ERROR_NONE) &&
           (BSP_CAMERA_AWB_Step(&AwbConfig, &AwbState, red, AwbGreen, value) != 0U))
  {
    (void)awb_apply();
  }
  else
  {
    /* Settled, or no reading this time. */
  }

  AwbOnBlue = (AwbOnBlue == 0U) ? 1U : 0U;
  (void)BSP_CAMERA_SetIspStatisticsUnit(0, BSP_CAMERA_PIPE_MAIN, 3U,
                                        (AwbOnBlue != 0U) ? DCMIPP_STAT_EXT_SOURCE_POST_DEMOS_B
                                                          : DCMIPP_STAT_EXT_SOURCE_POST_DEMOS_G);
  AwbSwitched = frames;
}

/**
  * @brief  The pipe's processing: ST's colour reconstruction and black level
  *         for the B-CAMS-IMX module, automatic white balance (awb_step), no
  *         colour matrix, gamma.
  * @note   Black level out of 255.
  */
static int32_t isp_config(void)
{
  DCMIPP_RawBayer2RGBConfTypeDef    demosaic = {0};
  DCMIPP_BlackLevelConfTypeDef      black    = {0};
  int32_t                           status;

  demosaic.RawBayerType  = BSP_CAMERA_BAYER_RGGB;
  demosaic.PeakStrength  = DCMIPP_RAWBAYER_ALGO_STRENGTH_4;
  demosaic.VLineStrength = DCMIPP_RAWBAYER_ALGO_STRENGTH_8;
  demosaic.HLineStrength = DCMIPP_RAWBAYER_ALGO_STRENGTH_8;
  demosaic.EdgeStrength  = DCMIPP_RAWBAYER_ALGO_STRENGTH_16;
  status = BSP_CAMERA_SetIspDemosaicing(0, BSP_CAMERA_PIPE_MAIN, &demosaic);
  if (status != BSP_ERROR_NONE) { return status; }

  black.RedCompBlackLevel   = BSP_CAMERA_IMX335_BLACK_LEVEL;
  black.GreenCompBlackLevel = BSP_CAMERA_IMX335_BLACK_LEVEL;
  black.BlueCompBlackLevel  = BSP_CAMERA_IMX335_BLACK_LEVEL;
  status = BSP_CAMERA_SetIspBlackLevel(0, BSP_CAMERA_PIPE_MAIN, &black);
  if (status != BSP_ERROR_NONE) { return status; }

  /* White balance (awb_step), starting under fluorescent light. */
  if (AwbStarted == 0U)
  {
    BSP_CAMERA_AWB_Init(&AwbConfig, &AwbState, BSP_CAMERA_AWB_SEGMENT);
    AwbStarted = 1U;
  }
  status = awb_apply();
  if (status != BSP_ERROR_NONE) { return status; }

  /* No colour matrix: on this sensor it amplifies colour noise, which the JPEG
   * turns into purple and green speckle, more than it improves colour. */
  status = BSP_CAMERA_SetIspColorConversion(0, BSP_CAMERA_PIPE_MAIN, NULL);
  if (status != BSP_ERROR_NONE) { return status; }

  status = BSP_CAMERA_SetIspGamma(0, BSP_CAMERA_PIPE_MAIN, 1U);
  if (status != BSP_ERROR_NONE) { return status; }

  /* Statistics after the gains and colour reconstruction, before gamma:
   * luminance for exposure; red, then green and blue in turn, for white balance. */
  status = BSP_CAMERA_SetIspStatistics(0, BSP_CAMERA_PIPE_MAIN, DCMIPP_STAT_EXT_SOURCE_POST_DEMOS_L);
  if (status != BSP_ERROR_NONE) { return status; }
  status = BSP_CAMERA_SetIspStatisticsUnit(0, BSP_CAMERA_PIPE_MAIN, 2U, DCMIPP_STAT_EXT_SOURCE_POST_DEMOS_R);
  if (status != BSP_ERROR_NONE) { return status; }
  AwbOnBlue   = 0U;
  AwbSwitched = FrameEvents;
  return BSP_CAMERA_SetIspStatisticsUnit(0, BSP_CAMERA_PIPE_MAIN, 3U, DCMIPP_STAT_EXT_SOURCE_POST_DEMOS_G);
}

/**
  * @brief  The largest centred window of the picture's shape, so a 16:9 size
  *         is cropped, not squashed.
  */
static void picture_window(BSP_CAMERA_PipeConfig_t *pPipe)
{
  uint32_t w = BSP_CAMERA_IMX335_WIDTH;
  uint32_t h = BSP_CAMERA_IMX335_HEIGHT;

  if (((uint32_t)UVC_FRAME_WIDTH * h) > ((uint32_t)UVC_FRAME_HEIGHT * w))
  {
    h = ((w * (uint32_t)UVC_FRAME_HEIGHT) / (uint32_t)UVC_FRAME_WIDTH) & ~1U;
  }
  else
  {
    w = ((h * (uint32_t)UVC_FRAME_WIDTH) / (uint32_t)UVC_FRAME_HEIGHT) & ~1U;
  }

  if ((w != BSP_CAMERA_IMX335_WIDTH) || (h != BSP_CAMERA_IMX335_HEIGHT))
  {
    pPipe->CropEnable = 1U;
    pPipe->CropX      = ((BSP_CAMERA_IMX335_WIDTH - w) / 2U) & ~1U;
    pPipe->CropY      = ((BSP_CAMERA_IMX335_HEIGHT - h) / 2U) & ~1U;
    pPipe->CropWidth  = w;
    pPipe->CropHeight = h;
  }
}

/**
  * @brief  Feeds the exposure loop the pipe's luminance average, once a frame.
  *         Runs in the main loop: it writes the sensor over I2C.
  */
static void exposure_step(void)
{
  const uint32_t frames = FrameEvents;

  if ((InitStatus != BSP_ERROR_NONE) || (frames == ExposureFrame))
  {
    return;
  }
  ExposureFrame = frames;

  if (BSP_CAMERA_GetIspStatistics(0, BSP_CAMERA_PIPE_MAIN, &ExposureAverage) == BSP_ERROR_NONE)
  {
    (void)BSP_CAMERA_IMX335_AutoExposure(ExposureAverage, UVC_EXPOSURE_TARGET);
  }
}

/* Time of flight */

/**
  * @brief  Load the sensor's firmware (about 3 s, USB served throughout by
  *         BSP_RANGING_VL53L5CX_WaitHook) and start it ranging.
  */
static void tof_init(void)
{
  TofStatus = BSP_RANGING_VL53L5CX_Init();
  if (TofStatus == BSP_ERROR_NONE)
  {
    TofStatus = BSP_RANGING_VL53L5CX_Start(UVC_TOF_ZONES, UVC_TOF_RATE_HZ);
  }
}

/**
  * @brief  Take a result if one has landed. Until TOF_INT falls this is free:
  *         the driver does not touch the bus.
  */
static void tof_step(void)
{
  if ((TofStatus == BSP_ERROR_NONE) && (BSP_RANGING_VL53L5CX_GetResult(&Tof) == BSP_ERROR_NONE))
  {
    TofResults++;
  }
}

/**
  * @brief  The nearest zone in range and the four centre zones' mean; with Grid,
  *         every zone, a row a line (- for none in range), in the order ST's
  *         driver gives them.
  */
static void tof_report(uint8_t Grid)
{
  const uint32_t side = (Tof.Zones == BSP_RANGING_VL53L5CX_ZONES_8X8) ? 8U : 4U;
  const uint32_t mid  = (side / 2U) - 1U;      /* the centre zones: mid and mid + 1 */
  uint32_t nearest = 0U, at = 0U, valid = 0U, centre = 0U, centre_n = 0U;

  if (TofStatus != BSP_ERROR_NONE)
  {
    print("[tof] VL53L5CX not running (status %ld)\r\n", (long)TofStatus);
    return;
  }
  if (TofResults == 0U)
  {
    print("[tof] no result yet\r\n");
    return;
  }

  for (uint32_t z = 0U; z < Tof.Zones; z++)
  {
    const uint32_t d   = Tof.Distance[z];
    const uint32_t row = z / side;
    const uint32_t col = z % side;

    if (d == 0U) { continue; }
    valid++;
    if ((nearest == 0U) || (d < nearest)) { nearest = d; at = z; }
    if ((row >= mid) && (row <= (mid + 1U)) && (col >= mid) && (col <= (mid + 1U)))
    {
      centre += d;
      centre_n++;
    }
  }
  if (valid == 0U)
  {
    print("[tof] no zone in range: nothing within 4 m, or the view is blocked\r\n");
    return;
  }
  print("[tof] %lu of %lu zones in range; nearest %lu mm (row %lu, column %lu); centre %lu mm\r\n",
        (unsigned long)valid, (unsigned long)Tof.Zones, (unsigned long)nearest, (unsigned long)(at / side),
        (unsigned long)(at % side), (unsigned long)((centre_n != 0U) ? (centre / centre_n) : 0U));

  if (Grid != 0U)
  {
    for (uint32_t row = 0U; row < side; row++)
    {
      char line[64];
      int  n = snprintf(line, sizeof line, "[tof] row %lu:", (unsigned long)row);

      for (uint32_t col = 0U; (col < side) && (n > 0) && ((size_t)n < sizeof line); col++)
      {
        const uint32_t d = Tof.Distance[(row * side) + col];

        n += (d != 0U) ? snprintf(&line[n], sizeof line - (size_t)n, " %5lu", (unsigned long)d)
                       : snprintf(&line[n], sizeof line - (size_t)n, "     -");
      }
      print("%s\r\n", line);
    }
  }
}

/**
  * @brief  USB and the watchdog stay served while the time-of-flight driver waits.
  */
void BSP_RANGING_VL53L5CX_WaitHook(void)
{
  service();
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
  *         the camera; the previous frame goes out from here. Early in the
  *         frame there is time for a time-of-flight read before it ends.
  */
void BSP_VENC_ReadPointerHook(uint32_t MbRow)
{
  service();
  /* In the camera's frame: the read row is 0 again once the picture is done. */
  if ((FrameActive != 0U) && (MbRow < TofRowLimit))
  {
    tof_step();
  }
}

/**
  * @brief  Sensor, camera, pipe, processing, line buffer, prime, encoder.
  */
static int32_t video_init(void)
{
  BSP_CAMERA_PipeConfig_t   pipe = {0};
  BSP_VENC_Config_t         venc = {0};
  BSP_VENC_Band_t           band;
  uint32_t                  started;
  int32_t                   status;

  /* Bring-up runs before the main loop: refresh the watchdog as it goes. */
  (void)BSP_WDG_Refresh();

  /* Own the sensor bus, to read the sensor's state even if the camera fails. */
  InitStep = "sensor bus";
  status = BSP_I2C1_Init();
  if (status != BSP_ERROR_NONE) { return status; }

  /* Powers the module up through EN_MODULE and XCLR, as neuron6_conf.h wires them. */
  InitStep = "sensor choice";
  status = BSP_CAMERA_IMX335_Register();
  if (status == BSP_ERROR_NONE) { status = BSP_CAMERA_SetCarrier(UVC_CARRIER); }
  if (status == BSP_ERROR_NONE) { status = BSP_CAMERA_IMX335_SetTiming(UVC_FRAME_RATE, UVC_LINE_PERCENT); }
  StateBefore = sensor_state();
  if (status != BSP_ERROR_NONE) { return status; }

  /* Identifies the sensor and sets it up, standing by. */
  InitStep = "camera init";
  started  = HAL_GetTick();
  status   = BSP_CAMERA_Init(0, UVC_RESOLUTION, BSP_CAMERA_PF_RAW10);
  InitMs   = HAL_GetTick() - started;
  if (status != BSP_ERROR_NONE) { return status; }
  (void)BSP_WDG_Refresh();

  GapWaitMs = 5000U / (uint32_t)UVC_FRAME_RATE;
  if (GapWaitMs < 150U) { GapWaitMs = 150U; }

  /* The picture, less bottom lines the encoder cannot code, must match what
   * the USB descriptors declare. */
  InitStep = "picture size";
  status = BSP_VENC_GetBandGeometry(UVC_FRAME_WIDTH, UVC_FRAME_HEIGHT, &band);
  if (status != BSP_ERROR_NONE) { return status; }
  if (band.SizeBytes > sizeof(Band)) { return BSP_ERROR_WRONG_PARAM; }
  if ((band.CodedWidth != (uint32_t)UVC_FRAME_WIDTH) ||
      (band.CodedHeight != (uint32_t)UVC_FRAME_HEIGHT))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  TofRowLimit = (band.CodedHeight / 16U) / 2U;

  /* The pipe crops the sensor's frame to the picture's shape and reduces it. */
  InitStep = "camera pipe";
  pipe.PixelFormat = BSP_CAMERA_PIPE_PF_YUV422;
  pipe.Width       = UVC_FRAME_WIDTH;
  pipe.Height      = UVC_FRAME_HEIGHT;
  picture_window(&pipe);
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

  StreamBand   = band;
  LinebufArmed = 0U;

  InitStep = "encoder";
  venc.Band        = band;
  venc.pBand       = Band;
  venc.Quality     = UVC_JPEG_QUALITY;
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
  *         every buffer waits for the host, and post it unless it is torn.
  */
static void uvc_capture_step(void)
{
  BSP_VENC_Statistics_t st;
  uint32_t              bytes = 0U;
  uint32_t              slot;

  if ((EncSeq - TxSeq) >= JPEG_BUF_COUNT)
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

  slot = EncSeq % JPEG_BUF_COUNT;
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

  slot = TxSeq % JPEG_BUF_COUNT;
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
  uint32_t exposure_us = 0U;
  uint32_t gain_mdb    = 0U;
  uint8_t  now         = sensor_state();

  if (InitStatus != BSP_ERROR_NONE)
  {
    print("[cam] IMX335 failed at %s, status %ld, after %lu ms; the sensor was %s at start and is %s now\r\n",
          InitStep, (long)InitStatus, (unsigned long)InitMs,
          state_name(StateBefore), state_name(now));
    if (StateBefore == IMX335_STATE_NO_ANSWER)
    {
      print("[cam] nothing answers at 0x34: check the module's supply and clock (EN_MODULE) and its reset (XCLR)\r\n");
    }
    return;
  }

  (void)BSP_CAMERA_IMX335_GetExposure(&exposure_us, &gain_mdb);
  print("[cam] IMX335 %ux%u at %u fps, quality %u, %s lanes: started in %lu ms, now %s, %lu frames in the last second, %lu camera errors since start (codes 0x%lx)\r\n",
        (unsigned)UVC_FRAME_WIDTH, (unsigned)UVC_FRAME_HEIGHT, (unsigned)UVC_FRAME_RATE,
        (unsigned)UVC_JPEG_QUALITY, (UVC_CARRIER == BSP_CAMERA_CARRIER_INVERTED) ? "crossed" : "straight",
        (unsigned long)InitMs, state_name(now), (unsigned long)Frames,
        (unsigned long)ErrorEvents, (unsigned long)LastErrorCode);
  print("[cam] exposure %lu us, gain %lu.%lu dB, average %lu (target %u); white balance %lu K (red x%lu.%02lu, blue x%lu.%02lu)\r\n",
        (unsigned long)exposure_us, (unsigned long)(gain_mdb / 1000U), (unsigned long)((gain_mdb % 1000U) / 100U),
        (unsigned long)ExposureAverage, (unsigned)UVC_EXPOSURE_TARGET, (unsigned long)AwbState.Kelvin,
        (unsigned long)(AwbState.GainR / 1000U), (unsigned long)((AwbState.GainR % 1000U) / 10U),
        (unsigned long)(AwbState.GainB / 1000U), (unsigned long)((AwbState.GainB % 1000U) / 10U));
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
  * @brief  PSRAM, .noncacheable and .axisram_nc are not cacheable: the camera
  *         and the encoder write them by DMA.
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

  /* No SystemClock_Config: the bootloader owns the clock tree and both
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

  /* After the camera, which powers the module up. */
  tof_init();

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

    /* While streaming, results are read inside the encode (BSP_VENC_ReadPointerHook). */
    if ((streaming == 0U) || (InitStatus != BSP_ERROR_NONE))
    {
      tof_step();
    }

    /* Between encodes, and while the primed pipe runs before a host streams. */
    exposure_step();
    awb_step();

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
        tof_report(1U);
      }
      else
      {
        tof_report(0U);
      }
    }
  }
}

/**
  * @brief  Return to the bootloader, where the board can be reflashed.
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
