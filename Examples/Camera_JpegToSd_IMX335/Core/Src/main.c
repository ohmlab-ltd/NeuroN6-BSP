/*
  Camera_JpegToSd_IMX335

  Takes full-resolution pictures (2592 x 1944) with ST's B-CAMS-IMX camera
  (Sony IMX335) and saves each as a numbered JPEG on the adapter's SD card.
  The camera pipe turns the sensor's raw data into colour, with automatic
  exposure and white balance; the hardware encoder makes the JPEG; FatFs
  writes it (FAT32 or exFAT).

  Press the USER button, or send 'c' on the console, to take a picture.
  SD_INTERVAL_MS also takes one every so many milliseconds. With each picture
  goes the module's VL53L5CX time-of-flight grid, 8 x 8 distances in
  millimetres, as IMG_nnnnn.CSV.

  The circuit:
  - Neuro N6 board with Ohm Lab's B-CAMS-IMX adapter: ST's B-CAMS-IMX module
    on its flex connector, and an SD card in its slot.

  The encoder runs on every frame, so a picture is the frame being read out
  when it is asked for. The console reports each file: its name, size, how
  soon after the request it was ready, and the time to write it. Send 'b' to
  return to the bootloader.

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
#include "neuron6_sd.h"
#include "neuron6_venc.h"
#include "neuron6_wdg.h"

#include "ff.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PICTURE_WIDTH           BSP_CAMERA_IMX335_WIDTH
#define PICTURE_HEIGHT          BSP_CAMERA_IMX335_HEIGHT

/* 21 frames a second with lines 142% of ST's length: a macroblock row every
 * 0.33 ms, just slower than the 0.32 ms the encoder needs at full width. The
 * fastest full-size rate; fps x line percent may not exceed 3000. */
#ifndef SD_FRAME_RATE
#define SD_FRAME_RATE           21U
#endif
#ifndef SD_LINE_PERCENT
#define SD_LINE_PERCENT         142U
#endif

/* How the adapter wires the two data lanes; BSPTest's bcamslink says which. */
#ifndef SD_CARRIER
#define SD_CARRIER              BSP_CAMERA_CARRIER_STANDARD
#endif

/* A picture every so many milliseconds as well as on demand; 0 for on demand only. */
#ifndef SD_INTERVAL_MS
#define SD_INTERVAL_MS          0U
#endif

/* Exposure target before gamma (50 lands mid-grey after it), and JPEG quality 0 to 9. */
#define SD_EXPOSURE_TARGET      50U
#define SD_JPEG_QUALITY         5U

/* 128 lines of 2592 YUV422 pixels: the encoder's deepest band at this width. */
#define SD_BAND_BYTES           663552U

/* A 5 MP picture at quality 5 is a few hundred kilobytes; fine detail and
 * noise make it several times larger. The encoder writes on-chip RAM, which
 * the card's DMA reads directly; a picture too big for it goes to PSRAM, which
 * the card driver copies out at about 18 MB/s. The on-chip RAM left after this
 * buffer is the heap the encoder allocates from at start (about 130 KB). */
#define JPEG_FAST_BYTES         (832U * 1024U)
#define JPEG_BUF_BYTES          (6U * 1024U * 1024U)

/* The encoder's LastError for an output buffer too small for the picture. */
#define VENC_ERROR_OVERFLOW     (-6)

/* Frames a picture may take before giving up: a torn frame passes to the next. */
#define SD_ENCODE_TRIES         4U

/* Between attempts to find a card, or after a failed write. */
#define SD_RETRY_MS             2000U

#define WDG_TIMEOUT_MS          4000U
#define WDG_WARN_MS             300U

/* Time of flight: 8 x 8 zones, ten results a second; a picture takes the newest.
 * A result is read inside an encode while at least half the frame's rows are to
 * come, so the 11 ms read ends before the frame does. */
#define SD_TOF_ZONES            BSP_RANGING_VL53L5CX_ZONES_8X8
#define SD_TOF_RATE_HZ          10U

/* The camera writes here, wrapped, and the encoder follows it. */
static uint8_t Band[SD_BAND_BYTES]
  __attribute__((section(".axisram_nc"), aligned(32)));

static uint8_t JpegFast[JPEG_FAST_BYTES]
  __attribute__((section(".axisram_nc"), aligned(32)));

static uint8_t JpegBuf[JPEG_BUF_BYTES]
  __attribute__((section(".ext_psram"), aligned(32)));

/* The buffer the last encode wrote; PSRAM after an overflow, until a picture
 * is half the on-chip buffer or less. */
static uint8_t *JpegOut = JpegFast;
static uint8_t  UseLargeBuffer;

static FATFS   Fs;
static uint8_t Mounted;
static uint32_t NextNumber = 1U;
static uint32_t LastRetry;
static uint32_t LastPicture;

static BSP_VENC_Band_t  StreamBand;
static uint8_t          LinebufArmed;
static int32_t          InitStatus = BSP_ERROR_NONE;
static const char      *InitStep = "not started";

static volatile uint32_t FrameEvents;
static volatile uint32_t ErrorEvents;
static volatile uint32_t FrameActive;
static volatile uint8_t  CaptureRequested;
static uint32_t          RequestTick;
static uint32_t          RequestTries;
static uint32_t          TornFrames;
static uint32_t          EncodedFrames;

static uint32_t          ExposureFrame;
static uint32_t          ExposureAverage;
static uint32_t          GapWaitMs = 500U;

/* The time-of-flight sensor: how it came up, its last result, and the encoder
 * row before which a result may be read. */
static int32_t           TofStatus = BSP_ERROR_NO_INIT;
static BSP_RANGING_VL53L5CX_Result_t Tof;
static uint32_t          TofResults;
static uint32_t          TofRowLimit;

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

static void    mpu_config(void);
static void    service(void);
static int32_t video_init(void);

void Error_Handler(void);

#define print(...)   (void)BSP_COM_Printf(__VA_ARGS__)

/* Sensor */

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
  * @brief  The pipe's processing, as in the UVC example: ST's colour
  *         reconstruction and black level, automatic white balance, no colour
  *         matrix, gamma.
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

  /* Luminance for exposure; red, then green and blue in turn, for white balance. */
  status = BSP_CAMERA_SetIspStatistics(0, BSP_CAMERA_PIPE_MAIN, DCMIPP_STAT_EXT_SOURCE_POST_DEMOS_L);
  if (status != BSP_ERROR_NONE) { return status; }
  status = BSP_CAMERA_SetIspStatisticsUnit(0, BSP_CAMERA_PIPE_MAIN, 2U, DCMIPP_STAT_EXT_SOURCE_POST_DEMOS_R);
  if (status != BSP_ERROR_NONE) { return status; }
  AwbOnBlue   = 0U;
  AwbSwitched = FrameEvents;
  return BSP_CAMERA_SetIspStatisticsUnit(0, BSP_CAMERA_PIPE_MAIN, 3U, DCMIPP_STAT_EXT_SOURCE_POST_DEMOS_G);
}

/**
  * @brief  Feeds the exposure loop the pipe's luminance average, once a frame.
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
    (void)BSP_CAMERA_IMX335_AutoExposure(ExposureAverage, SD_EXPOSURE_TARGET);
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
    TofStatus = BSP_RANGING_VL53L5CX_Start(SD_TOF_ZONES, SD_TOF_RATE_HZ);
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
  * @brief  The nearest distance in range in the last result, 0 if none.
  */
static uint32_t tof_nearest(void)
{
  uint32_t nearest = 0U;

  for (uint32_t z = 0U; z < Tof.Zones; z++)
  {
    if ((Tof.Distance[z] != 0U) && ((nearest == 0U) || (Tof.Distance[z] < nearest)))
    {
      nearest = Tof.Distance[z];
    }
  }
  return nearest;
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
  * @brief  Start the camera once the encoder is parked on the first line event.
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
  * @brief  Frame end: anchors the encoder's row count.
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
  * @brief  Count pipe errors for the console.
  */
void BSP_CAMERA_ErrorCallback(uint32_t Instance, uint32_t Pipe)
{
  (void)Instance;
  (void)Pipe;
  ErrorEvents++;
}

/**
  * @brief  USB and the watchdog stay served through an encode. Early in the frame
  *         there is time for a time-of-flight read before it ends.
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
  * @brief  USB and the watchdog stay served through every card wait.
  */
void BSP_SD_WaitHook(void)
{
  service();
}

/**
  * @brief  Ask for a picture; the time is kept for the console.
  */
static void request_picture(void)
{
  if (CaptureRequested == 0U)
  {
    RequestTick      = HAL_GetTick();
    CaptureRequested = 1U;
  }
}

/**
  * @brief  A press of the USER button asks for a picture: polled, with the
  *         state held 30 ms before it counts.
  */
static void button_step(void)
{
  static int32_t  stable;
  static int32_t  last;
  static uint32_t since;
  const int32_t   now = BSP_PB_GetState(BUTTON_USER);

  if (now != last)
  {
    last  = now;
    since = HAL_GetTick();
  }
  else if ((now != stable) && ((HAL_GetTick() - since) >= 30U))
  {
    stable = now;
    if (stable == 1)
    {
      request_picture();
    }
  }
}

/**
  * @brief  Sensor, camera, pipe, processing, line buffer, prime, encoder; as
  *         in the UVC example, at full size.
  */
static int32_t video_init(void)
{
  BSP_CAMERA_PipeConfig_t pipe = {0};
  BSP_VENC_Config_t       venc = {0};
  BSP_VENC_Band_t         band;
  int32_t                 status;

  (void)BSP_WDG_Refresh();

  InitStep = "sensor bus";
  status = BSP_I2C1_Init();
  if (status != BSP_ERROR_NONE) { return status; }

  InitStep = "sensor choice";
  status = BSP_CAMERA_IMX335_Register();
  if (status == BSP_ERROR_NONE) { status = BSP_CAMERA_SetCarrier(SD_CARRIER); }
  if (status == BSP_ERROR_NONE) { status = BSP_CAMERA_IMX335_SetTiming(SD_FRAME_RATE, SD_LINE_PERCENT); }
  if (status != BSP_ERROR_NONE) { return status; }

  InitStep = "camera init";
  status = BSP_CAMERA_Init(0, BSP_CAMERA_IMX335_R2592x1944, BSP_CAMERA_PF_RAW10);
  if (status != BSP_ERROR_NONE) { return status; }
  (void)BSP_WDG_Refresh();

  GapWaitMs = 5000U / SD_FRAME_RATE;

  InitStep = "picture size";
  status = BSP_VENC_GetBandGeometry(PICTURE_WIDTH, PICTURE_HEIGHT, &band);
  if (status != BSP_ERROR_NONE) { return status; }
  if (band.SizeBytes > sizeof(Band)) { return BSP_ERROR_WRONG_PARAM; }
  TofRowLimit = (band.CodedHeight / 16U) / 2U;

  InitStep = "camera pipe";
  pipe.PixelFormat = BSP_CAMERA_PIPE_PF_YUV422;
  pipe.Width       = PICTURE_WIDTH;
  pipe.Height      = PICTURE_HEIGHT;
  status = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pipe);
  if (status != BSP_ERROR_NONE) { return status; }

  InitStep = "image processing";
  status = isp_config();
  if (status != BSP_ERROR_NONE) { return status; }

  /* The pipe runs from here on, so the exposure loop settles before a picture. */
  InitStep = "line buffer";
  status = BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN, band.WrapLines, BSP_CAMERA_LINES_OFF);
  if (status != BSP_ERROR_NONE) { return status; }

  InitStep = "camera start";
  status = BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, Band, BSP_CAMERA_MODE_CONTINUOUS);
  if (status != BSP_ERROR_NONE) { return status; }
  (void)BSP_WDG_Refresh();

  StreamBand   = band;
  LinebufArmed = 0U;

  InitStep = "encoder";
  venc.Band        = band;
  venc.pBand       = Band;
  venc.Quality     = SD_JPEG_QUALITY;
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
  * @brief  One frame into JpegOut: arms the line event the first time, and
  *         otherwise waits for the vertical blanking.
  * @retval The JPEG's size, or 0 for a torn or failed frame.
  */
static uint32_t encode_frame(void)
{
  uint32_t bytes = 0U;

  if (!LinebufArmed)
  {
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
    (void)BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN,
                                      StreamBand.WrapLines, StreamBand.EventLines);
    LinebufArmed = 1U;
  }
  else
  {
    /* An encode armed mid-frame would join two frames: wait for the gap. */
    const uint32_t since  = HAL_GetTick();
    const uint32_t frames = FrameEvents;

    while ((FrameActive != 0U) && ((HAL_GetTick() - since) < GapWaitMs))
    {
      service();
    }
    if ((FrameActive != 0U) && (FrameEvents == frames))
    {
      /* The camera stopped mid-frame: restart it, encoder first. */
      (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
      (void)BSP_VENC_Resync(0);
      LinebufArmed = 0U;
      FrameActive  = 0U;
      return 0U;
    }
  }

  JpegOut = (UseLargeBuffer != 0U) ? JpegBuf : JpegFast;
  if (BSP_VENC_EncodeBand(0, JpegOut, (UseLargeBuffer != 0U) ? JPEG_BUF_BYTES : JPEG_FAST_BYTES,
                          &bytes) != BSP_ERROR_NONE)
  {
    BSP_VENC_Statistics_t st = {0};

    (void)BSP_VENC_GetStatistics(0, &st);
    if ((UseLargeBuffer == 0U) && ((int32_t)st.LastError == VENC_ERROR_OVERFLOW))
    {
      UseLargeBuffer = 1U;
      print("[cam] picture over %u KB: encoding to PSRAM\r\n", (unsigned)(JPEG_FAST_BYTES / 1024U));
    }
    else
    {
      print("[cam] encode failed (encoder %ld)\r\n", (long)(int32_t)st.LastError);
    }
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
    (void)BSP_VENC_Resync(0);
    LinebufArmed = 0U;
    FrameActive  = 0U;
    return 0U;
  }

  if (BSP_VENC_LastFrameTorn(0) != 0U)
  {
    TornFrames++;
    return 0U;
  }

  /* Back on-chip once pictures are well under its size. */
  if ((UseLargeBuffer != 0U) && (bytes <= (JPEG_FAST_BYTES / 2U)))
  {
    UseLargeBuffer = 0U;
  }

  EncodedFrames++;
  return bytes;
}

/* Card */

/**
  * @brief  Mount the card and find the number after the highest IMG_nnnnn.JPG.
  */
static void card_mount(void)
{
  DIR     dir;
  FILINFO info;
  FRESULT fr;
  BSP_SD_CardInfo_t card;

  LastRetry = HAL_GetTick();

  fr = f_mount(&Fs, "", 1);
  if (fr != FR_OK)
  {
    (void)f_mount(NULL, "", 0);
    (void)BSP_SD_DeInit();
    print("[sd] %s (FatFs %d); trying again in %u s\r\n",
          (fr == FR_NOT_READY) ? "no card" : (fr == FR_NO_FILESYSTEM) ? "no FAT32 or exFAT volume on the card" : "card not mounted",
          (int)fr, (unsigned)(SD_RETRY_MS / 1000U));
    return;
  }

  NextNumber = 1U;
  if (f_opendir(&dir, "") == FR_OK)
  {
    while ((f_readdir(&dir, &info) == FR_OK) && (info.fname[0] != '\0'))
    {
      if ((strncmp(info.fname, "IMG_", 4U) == 0) && (strlen(info.fname) == 13U) &&
          (strcmp(&info.fname[9], ".JPG") == 0))
      {
        const uint32_t n = (uint32_t)strtoul(&info.fname[4], NULL, 10);

        if (n >= NextNumber) { NextNumber = n + 1U; }
      }
      service();
    }
    (void)f_closedir(&dir);
  }

  Mounted = 1U;
  (void)BSP_SD_GetCardInfo(&card);
  print("[sd] card mounted: %s, %lu KB clusters, %lu MB, bus %lu Hz; next picture IMG_%05lu.JPG\r\n",
        (Fs.fs_type == FS_EXFAT) ? "exFAT" : (Fs.fs_type == FS_FAT32) ? "FAT32" : "FAT12/16",
        (unsigned long)(Fs.csize / 2U), (unsigned long)(card.BlockCount / 2048U), (unsigned long)card.ClockHz,
        (unsigned long)NextNumber);
}

static void card_unmount(void)
{
  (void)f_mount(NULL, "", 0);
  (void)BSP_SD_DeInit();
  Mounted   = 0U;
  LastRetry = HAL_GetTick();
}

/**
  * @brief  Save the newest time-of-flight result as IMG_nnnnn.CSV: a line a row
  *         of zones, in millimetres, 0 where a zone has no valid range.
  */
static void save_depth(uint32_t Number)
{
  const uint32_t side = (Tof.Zones == BSP_RANGING_VL53L5CX_ZONES_8X8) ? 8U : 4U;
  char     name[16];
  char     text[400];
  FIL      file;
  FRESULT  fr;
  UINT     written = 0U;
  int      n = 0;

  if ((TofStatus != BSP_ERROR_NONE) || (TofResults == 0U))
  {
    return;   /* the five-second report says why */
  }

  (void)snprintf(name, sizeof name, "IMG_%05lu.CSV", (unsigned long)Number);
  for (uint32_t z = 0U; (z < Tof.Zones) && (n >= 0) && ((size_t)n < sizeof text); z++)
  {
    n += snprintf(&text[n], sizeof text - (size_t)n, "%u%s", (unsigned)Tof.Distance[z],
                  ((z % side) == (side - 1U)) ? "\r\n" : ",");
  }
  if ((n <= 0) || ((size_t)n >= sizeof text))
  {
    return;
  }

  fr = f_open(&file, name, FA_CREATE_ALWAYS | FA_WRITE);
  if (fr == FR_OK)
  {
    fr = f_write(&file, text, (UINT)n, &written);
    if (f_close(&file) != FR_OK) { fr = (fr == FR_OK) ? FR_DISK_ERR : fr; }
  }
  if ((fr != FR_OK) || (written != (UINT)n))
  {
    print("[sd] %s not saved (FatFs %d)\r\n", name, (int)fr);
    return;
  }
  print("[sd] %s %lu x %lu distances, nearest %lu mm\r\n", name, (unsigned long)side, (unsigned long)side,
        (unsigned long)tof_nearest());
}

/**
  * @brief  Save JpegOut as the next IMG_nnnnn.JPG. Every failure says what
  *         happened and leaves the loop to try again.
  * @param  bytes      The JPEG's size.
  * @param  ready_ms   From the request to the JPEG being ready.
  */
static void save_picture(uint32_t bytes, uint32_t ready_ms)
{
  char     name[16];
  FIL      file;
  FRESULT  fr;
  UINT     written = 0U;
  uint32_t t0;
  uint32_t write_ms;

  (void)snprintf(name, sizeof name, "IMG_%05lu.JPG", (unsigned long)NextNumber);
  (void)BSP_LED_On(LED_GREEN);
  t0 = HAL_GetTick();
  fr = f_open(&file, name, FA_CREATE_NEW | FA_WRITE);
  if (fr == FR_OK)
  {
    fr = f_write(&file, JpegOut, bytes, &written);
    if (f_close(&file) != FR_OK) { fr = (fr == FR_OK) ? FR_DISK_ERR : fr; }
  }
  write_ms = HAL_GetTick() - t0;
  (void)BSP_LED_Off(LED_GREEN);

  if ((fr == FR_OK) && (written == bytes))
  {
    print("[sd] %s %lu bytes, quality %u, ready %lu ms after the request, write %lu ms (%lu KB/s)\r\n",
          name, (unsigned long)bytes, (unsigned)SD_JPEG_QUALITY, (unsigned long)ready_ms, (unsigned long)write_ms,
          (unsigned long)((write_ms != 0U) ? (bytes / write_ms) : 0U));
    save_depth(NextNumber);
    NextNumber++;
    return;
  }

  (void)BSP_LED_On(LED_RED);
  if ((fr == FR_OK) || (fr == FR_DENIED))
  {
    /* A short write with no error is a full volume. */
    (void)f_unlink(name);
    print("[sd] card full: %s not saved (%lu of %lu bytes)\r\n", name, (unsigned long)written, (unsigned long)bytes);
  }
  else if (fr == FR_EXIST)
  {
    print("[sd] %s already exists: skipping the number\r\n", name);
    NextNumber++;
  }
  else
  {
    print("[sd] write failed (FatFs %d): %s not saved; remounting\r\n", (int)fr, name);
    card_unmount();
  }
  (void)BSP_LED_Off(LED_RED);
}

/**
  * @brief  The encoder runs on every frame while a card is mounted, so a
  *         picture is the frame being read out when it is asked for, not the
  *         next one. A torn frame passes the request to the next frame.
  */
static void picture_step(void)
{
  const uint32_t bytes = encode_frame();

  if (CaptureRequested == 0U)
  {
    return;
  }

  if (bytes == 0U)
  {
    if (++RequestTries >= SD_ENCODE_TRIES)
    {
      print("[cam] no picture after %u frames\r\n", (unsigned)SD_ENCODE_TRIES);
      RequestTries     = 0U;
      CaptureRequested = 0U;
    }
    return;
  }

  RequestTries     = 0U;
  CaptureRequested = 0U;
  save_picture(bytes, HAL_GetTick() - RequestTick);
}

/* Scaffolding */

/**
  * @brief  Service USB, the watchdog and the console. Called from the main
  *         loop, inside the encode and while the card is busy.
  */
static void service(void)
{
  uint8_t ch;

  BSP_COM_Task();
  (void)BSP_WDG_Refresh();
  button_step();
  if (BSP_COM_Read(&ch, 1U) == 1)
  {
    if (ch == (uint8_t)'b')
    {
      (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
      (void)BSP_VENC_Quiesce(0);
      BSP_COM_RebootToBootloader();
    }
    else if (ch == (uint8_t)'c')
    {
      request_picture();
    }
    else
    {
      /* Ignored. */
    }
  }
}

/**
  * @brief  Runs 300 ms before the watchdog resets the chip: points the reset
  *         at the bootloader.
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
  uint32_t seconds = 0U;
  uint32_t frames_seen = 0U;
  uint32_t encoded_seen = 0U;

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

  InitStatus = video_init();

  if (BSP_COM_Init() != BSP_ERROR_NONE)
  {
    (void)BSP_LED_On(LED_RED);
    Error_Handler();
  }

  if (InitStatus != BSP_ERROR_NONE)
  {
    (void)BSP_LED_On(LED_RED);
  }

  /* After the camera, which powers the module up. */
  tof_init();

  (void)BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
  card_mount();
  LastPicture = HAL_GetTick();

  while (1)
  {
    service();
    exposure_step();
    awb_step();

    if ((Mounted == 0U) && ((HAL_GetTick() - LastRetry) >= SD_RETRY_MS))
    {
      card_mount();
    }

#if (SD_INTERVAL_MS != 0U)
    if ((HAL_GetTick() - LastPicture) >= SD_INTERVAL_MS)
    {
      LastPicture = HAL_GetTick();
      request_picture();
    }
#endif

    /* While the encoder runs, results are read inside it (BSP_VENC_ReadPointerHook). */
    if ((InitStatus != BSP_ERROR_NONE) || (Mounted == 0U))
    {
      tof_step();
    }

    if (InitStatus == BSP_ERROR_NONE)
    {
      if (Mounted != 0U)
      {
        picture_step();
      }
      else if (CaptureRequested != 0U)
      {
        CaptureRequested = 0U;
        print("[sd] no card mounted: picture not taken\r\n");
      }
    }

    if ((HAL_GetTick() - report) >= 1000U)
    {
      const uint32_t frames  = FrameEvents - frames_seen;
      const uint32_t encoded = EncodedFrames - encoded_seen;

      frames_seen  = FrameEvents;
      encoded_seen = EncodedFrames;
      report = HAL_GetTick();

      /* Every fifth second, and the first: the console may open late. */
      if ((seconds++ % 5U) == 0U)
      {
        uint32_t exposure_us = 0U;
        uint32_t gain_mdb    = 0U;

        if (InitStatus != BSP_ERROR_NONE)
        {
          print("[cam] IMX335 failed at %s, status %ld\r\n", InitStep, (long)InitStatus);
        }
        else
        {
          (void)BSP_CAMERA_IMX335_GetExposure(&exposure_us, &gain_mdb);
          print("[cam] %ux%u, %lu frames and %lu encoded in the last second, %lu torn and %lu camera errors since start; exposure %lu us, gain %lu.%lu dB, average %lu; white balance %lu K; card %s\r\n",
                (unsigned)PICTURE_WIDTH, (unsigned)PICTURE_HEIGHT, (unsigned long)frames, (unsigned long)encoded,
                (unsigned long)TornFrames, (unsigned long)ErrorEvents,
                (unsigned long)exposure_us, (unsigned long)(gain_mdb / 1000U), (unsigned long)((gain_mdb % 1000U) / 100U),
                (unsigned long)ExposureAverage, (unsigned long)AwbState.Kelvin, (Mounted != 0U) ? "mounted" : "absent");
        }
        if (TofStatus != BSP_ERROR_NONE)
        {
          print("[tof] VL53L5CX not running (status %ld)\r\n", (long)TofStatus);
        }
        else
        {
          print("[tof] %lu results since start; nearest %lu mm\r\n", (unsigned long)TofResults,
                (unsigned long)tof_nearest());
        }
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
