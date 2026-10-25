/**
  * @file    neuron6_venc.c
  * @brief   Hardware JPEG encoding fed by the DCMIPP-to-VENC line-buffer handshake.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6_venc.h"
#include "neuron6.h"
#include "neuron6_camera.h"

#include <string.h>

#include "jpegencapi.h"
#include "ewl.h"
#include "stm32n6xx_ll_venc.h"
#include "stm32n6xx_ll_bus.h"

/* Registers the encoder's input band. ST's wrapper defines it but no header declares it;
 * the virt and bus addresses are the same on this device. */
extern void EWLSetInputLineBufferBase(u32 *virt, ptr_t bus, u32 size);

/* Depth-4 band budget: 1080p's, a 3840-byte pitch x 128 lines, the largest mode. */
#define VENC_BAND_MAX_BYTES             491520U

/* Lines in one macroblock row. */
#define VENC_MB_LINES                   16U

/* The 4:2:2 macroblock is 16 wide by 8 tall: coded heights are multiples of 8. */
#define VENC_HEIGHT_ALIGN               8U

#define VENC_RET_FRAME_READY            JPEGENC_FRAME_READY

/* ES0620 step 2: ms for the encoder to go idle after the camera stops (generous). */
#define VENC_IDLE_SETTLE_MS             20U

typedef struct
{
  uint8_t               Initialised;  /*!< 1 once Init has succeeded.       */
  uint8_t               Unconfigured; /*!< 1 after a failed resync: encodes fail. */
  uint8_t               FrameMode;    /*!< 1: memory source, no band.       */
  uint8_t               SourceArmed;  /*!< 1 once the camera started for this stream. */
  BSP_VENC_Band_t       Band;         /*!< Geometry in force.               */
  uint8_t              *pBand;        /*!< The band.                        */
  uint32_t              Quality;      /*!< Level in force.                  */
  int32_t             (*SourceStart)(void *pArg);
  void                 *pSourceArg;
  uint32_t              SourceMbRows; /*!< Rows the camera has produced this frame. */
  uint8_t               SourceAnchored; /*!< 1 once SourceMbRows resets at frame end. */
  uint8_t               LastTorn;     /*!< 1 if the last encode lapped.     */
  uint32_t              GapMax;       /*!< Largest camera-to-encoder gap this encode. */
  BSP_VENC_Statistics_t Stats;
} VENC_Ctx_t;

static VENC_Ctx_t   Venc_Ctx;
static JpegEncInst  Venc_Instance;
static JpegEncCfg   Venc_Cfg;

/* Isolation (BSP_ISOLATION_Config stamps the encoder as a bus master), then clocks. */
void BSP_VENC_EnablePeripheral(void)
{
  (void)BSP_ISOLATION_Config();

  LL_VENC_Init();
}

/* Configure the encoder from the stored context; used by Init and the erratum resync. */
static int32_t VENC_Configure(void)
{
  int32_t ret;

  BSP_VENC_EnablePeripheral();

  /* Erratum ES0620 steps 2 and 3 (idle wait, reset), on every configuration. */
  HAL_Delay(VENC_IDLE_SETTLE_MS);

  __DSB();
  __ISB();
  __HAL_RCC_VENC_FORCE_RESET();
  __DSB();
  __ISB();
  __HAL_RCC_VENC_RELEASE_RESET();
  __DSB();
  __ISB();

  memset(&Venc_Cfg, 0, sizeof(Venc_Cfg));

  Venc_Cfg.qLevel          = (i32)Venc_Ctx.Quality;
  Venc_Cfg.frameType       = JPEGENC_YUV422_INTERLEAVED_YUYV;
  Venc_Cfg.codingMode      = JPEGENC_422_MODE;
  Venc_Cfg.codingType      = JPEGENC_WHOLE_FRAME;
  Venc_Cfg.markerType      = JPEGENC_SINGLE_MARKER;
  Venc_Cfg.unitsType       = JPEGENC_DOTS_PER_INCH;
  Venc_Cfg.rotation        = JPEGENC_ROTATE_0;
  Venc_Cfg.xDensity        = 72;
  Venc_Cfg.yDensity        = 72;
  Venc_Cfg.xOffset         = 0;
  Venc_Cfg.yOffset         = 0;
  Venc_Cfg.restartInterval = 0;

  Venc_Cfg.inputWidth   = (i32)Venc_Ctx.Band.CodedWidth;
  Venc_Cfg.inputHeight  = (i32)Venc_Ctx.Band.CodedHeight;
  Venc_Cfg.codingWidth  = (i32)Venc_Ctx.Band.CodedWidth;
  Venc_Cfg.codingHeight = (i32)Venc_Ctx.Band.CodedHeight;

  /* Line-buffer path. Loop-back is the other half of the DCMIPP's address wrap: without
   * it the encoder reads past the end of the band after the wrap. */
  if (Venc_Ctx.FrameMode == 0U)
  {
    Venc_Cfg.inputLineBufEn         = 1;
    Venc_Cfg.inputLineBufLoopBackEn = 1;
    Venc_Cfg.inputLineBufHwModeEn   = 1;
    Venc_Cfg.inputLineBufDepth      = (u32)Venc_Ctx.Band.Depth;
  }

  if (Venc_Ctx.FrameMode == 0U)
  {
    EWLSetInputLineBufferBase((u32 *)Venc_Ctx.pBand, (ptr_t)Venc_Ctx.pBand,
                              Venc_Ctx.Band.SizeBytes);
  }

  /* A whole-frame instance is created and released before the line-buffer one. */
  if (Venc_Ctx.FrameMode == 0U)
  {
    JpegEncCfg  full = Venc_Cfg;
    JpegEncInst warm = NULL;

    full.inputLineBufEn         = 0;
    full.inputLineBufLoopBackEn = 0;
    full.inputLineBufHwModeEn   = 0;
    full.inputLineBufDepth      = 0;

    if (JpegEncInit(&full, &warm) == JPEGENC_OK)
    {
      (void)JpegEncSetPictureSize(warm, &full);
      JpegEncRelease(warm);
    }
  }

  /* A rejected configuration leaves the instance invalid: later encodes fail at once. */
  ret = JpegEncInit(&Venc_Cfg, &Venc_Instance);
  if (ret != JPEGENC_OK)
  {
    Venc_Instance = NULL;
    LL_VENC_DeInit();
    return BSP_ERROR_NO_INIT;
  }

  if (JpegEncSetPictureSize(Venc_Instance, &Venc_Cfg) != JPEGENC_OK)
  {
    JpegEncRelease(Venc_Instance);
    Venc_Instance = NULL;
    LL_VENC_DeInit();
    return BSP_ERROR_NO_INIT;
  }

  return BSP_ERROR_NONE;
}

/* Pass the handle itself: given its address, JpegEncRelease silently does nothing. */
static void VENC_Release(void)
{
  if (Venc_Instance != NULL)
  {
    JpegEncRelease(Venc_Instance);
    Venc_Instance = NULL;
  }
}

int32_t BSP_VENC_GetBandGeometry(uint32_t Width, uint32_t Height, BSP_VENC_Band_t *pBand)
{
  const BSP_CAMERA_Sensor_t *sensor;
  uint32_t pitch;
  uint32_t depth;
  uint32_t mb_width;
  uint32_t coded_height;

  if ((pBand == NULL) || (Width == 0U) || (Height == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Whole macroblocks only; flooring the width would shift the picture, so refuse it. */
  if ((Width % 16U) != 0U)
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  /* The vendor library checks only its own 8176 limit, not the hardware's. */
  if (Width > BSP_VENC_MAX_CODED_WIDTH)
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  coded_height = Height & ~(VENC_HEIGHT_ALIGN - 1U);
  if (coded_height == 0U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* YUV422: 2 bytes a pixel; the DCMIPP pitch must be a multiple of 16 bytes. */
  pitch    = ((Width * 2U) + 15U) & ~15U;
  mb_width = (Width + 15U) / 16U;

  /* Deepest the budget allows: depth is how far the camera may lead before it tears. */
  depth = ((pitch * (4U * VENC_MB_LINES * 2U)) <= VENC_BAND_MAX_BYTES) ? 4U : 2U;

  /* These two cases need the deeper band whatever the budget. */
  sensor = BSP_CAMERA_GetSensor();
  if (((mb_width & 1U) != 0U) ||
      ((sensor != NULL) && (sensor->SensorFormat == BSP_CAMERA_PF_RAW8) && (Height > 1024U)))
  {
    depth = 4U;
  }
  /* The 480-line modes trade slack for memory; depth 2 holds at their frame rates. */
  else if ((Height == 480U) && ((Width == 800U) || (Width == 640U)))
  {
    depth = 2U;
  }
  else
  {
    /* keep the budgeted depth */
  }

  pBand->Pitch       = pitch;
  pBand->Depth       = depth;
  pBand->SizeBytes   = pitch * (depth * VENC_MB_LINES * 2U);
  pBand->WrapLines   = depth * VENC_MB_LINES * 2U;
  pBand->EventLines  = depth * VENC_MB_LINES;
  pBand->CodedWidth  = Width;
  pBand->CodedHeight = coded_height;

  return BSP_ERROR_NONE;
}

int32_t BSP_VENC_Init(uint32_t Instance, const BSP_VENC_Config_t *pConfig)
{
  if ((Instance >= BSP_VENC_INSTANCES_NBR) || (pConfig == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Both NULL is the memory-source mode; one NULL is a mistake. */
  uint8_t frame_mode = ((pConfig->pBand == NULL) && (pConfig->SourceStart == NULL)) ? 1U : 0U;
  if ((frame_mode == 0U) && ((pConfig->pBand == NULL) || (pConfig->SourceStart == NULL)))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (pConfig->Quality > BSP_VENC_QUALITY_MAX)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (pConfig->Band.SizeBytes == 0U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* The DCMIPP silently refuses a misaligned band: no frames, no other symptom. */
  if ((frame_mode == 0U) && (((uint32_t)pConfig->pBand & (BSP_VENC_BUFFER_ALIGNMENT - 1U)) != 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Venc_Ctx.Initialised != 0U)
  {
    return BSP_ERROR_BUSY;
  }

  /* An instance BSP_VENC_Quiesce stopped but could not free. */
  VENC_Release();

  memset(&Venc_Ctx, 0, sizeof(Venc_Ctx));
  Venc_Ctx.FrameMode   = frame_mode;
  Venc_Ctx.Band        = pConfig->Band;
  Venc_Ctx.pBand       = pConfig->pBand;
  Venc_Ctx.Quality     = pConfig->Quality;
  Venc_Ctx.SourceStart = pConfig->SourceStart;
  Venc_Ctx.pSourceArg  = pConfig->pSourceArg;

  if (VENC_Configure() != BSP_ERROR_NONE)
  {
    return BSP_ERROR_NO_INIT;
  }

  Venc_Ctx.Initialised = 1U;

  return BSP_ERROR_NONE;
}

int32_t BSP_VENC_Quiesce(uint32_t Instance)
{
  volatile uint32_t spin;

  if (Instance >= BSP_VENC_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Venc_Ctx.Initialised == 0U)
  {
    return BSP_ERROR_NONE;
  }

  /* ES0620 step 2: let the encoder finish its buffers and stop its bus accesses. */
  for (spin = 0U; spin < ((SystemCoreClock / 1000U) * VENC_IDLE_SETTLE_MS) / 4U; spin++)
  {
  }

  /* Step 3: reset only now that it is idle. */
  __DSB();
  __ISB();
  __HAL_RCC_VENC_FORCE_RESET();
  __DSB();
  __ISB();
  __HAL_RCC_VENC_RELEASE_RESET();
  __DSB();
  __ISB();

  /* Only the clock goes: this runs from watchdog warnings and fault handlers, so the
   * stale instance is freed by the next BSP_VENC_Init or BSP_VENC_DeInit. */
  LL_VENC_DeInit();
  Venc_Ctx.Initialised = 0U;
  Venc_Ctx.SourceArmed = 0U;

  return BSP_ERROR_NONE;
}

int32_t BSP_VENC_DeInit(uint32_t Instance)
{
  if (Instance >= BSP_VENC_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Venc_Ctx.Initialised == 0U)
  {
    /* An instance BSP_VENC_Quiesce stopped but could not free. */
    VENC_Release();
    return BSP_ERROR_NO_INIT;
  }

  VENC_Release();
  LL_VENC_DeInit();

  Venc_Ctx.Initialised = 0U;
  Venc_Ctx.SourceArmed = 0U;

  return BSP_ERROR_NONE;
}

static int32_t VENC_Encode(const uint8_t *pSource, uint8_t *pOut, uint32_t OutBytes,
                           uint32_t *pJpegBytes);

int32_t BSP_VENC_EncodeBand(uint32_t Instance, uint8_t *pOut, uint32_t OutBytes,
                            uint32_t *pJpegBytes)
{
  if (Instance >= BSP_VENC_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if ((Venc_Ctx.Initialised != 0U) && (Venc_Ctx.FrameMode != 0U))
  {
    return BSP_ERROR_WRONG_PARAM;     /* memory-source mode: use EncodeFrame */
  }
  return VENC_Encode(Venc_Ctx.pBand, pOut, OutBytes, pJpegBytes);
}

int32_t BSP_VENC_EncodeFrame(uint32_t Instance, const uint8_t *pSource, uint8_t *pOut,
                             uint32_t OutBytes, uint32_t *pJpegBytes)
{
  if ((Instance >= BSP_VENC_INSTANCES_NBR) || (pSource == NULL) ||
      (((uint32_t)pSource & (BSP_VENC_BUFFER_ALIGNMENT - 1U)) != 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if ((Venc_Ctx.Initialised != 0U) && (Venc_Ctx.FrameMode == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;     /* band mode: use EncodeBand */
  }
  return VENC_Encode(pSource, pOut, OutBytes, pJpegBytes);
}

/* Both modes: pSource is the band, or the whole frame in memory-source mode. */
static int32_t VENC_Encode(const uint8_t *pSource, uint8_t *pOut, uint32_t OutBytes,
                           uint32_t *pJpegBytes)
{
  JpegEncIn  in;
  JpegEncOut out;
  int32_t    ret;

  if ((pOut == NULL) || (pJpegBytes == NULL) || (OutBytes == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (((uint32_t)pOut & (BSP_VENC_BUFFER_ALIGNMENT - 1U)) != 0U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Venc_Ctx.Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  /* No encoder instance until a resync configures one. */
  if (Venc_Ctx.Unconfigured != 0U)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  *pJpegBytes = 0U;

  memset(&in, 0, sizeof(in));
  memset(&out, 0, sizeof(out));

  in.pOutBuf     = (u8 *)pOut;
  in.busOutBuf   = (ptr_t)pOut;
  in.outBufSize  = OutBytes;
  in.frameHeader = 1;
  in.busLum      = (ptr_t)pSource;
  in.busCb       = 0;
  in.busCr       = 0;

  /* Zero: in hardware-handshake mode the line events advance the write count. */
  in.lineBufWrCnt = 0U;

  /* Reset per encode unless the application anchors the count at frame end. */
  if (Venc_Ctx.SourceAnchored == 0U)
  {
    Venc_Ctx.SourceMbRows = 0U;
  }
  Venc_Ctx.GapMax   = 0U;
  Venc_Ctx.LastTorn = 0U;

  /* No line-buffer callback: the hardware handshake paces the band. */
  ret = (int32_t)JpegEncEncode(Venc_Instance, &in, &out, NULL, NULL);

  /* A gap that reached the band's capacity means the camera lapped the encoder: torn. */
  Venc_Ctx.Stats.LastGap = Venc_Ctx.GapMax;
  if ((Venc_Ctx.SourceAnchored != 0U) &&
      (Venc_Ctx.GapMax >= (Venc_Ctx.Band.Depth * 2U)))
  {
    Venc_Ctx.LastTorn = 1U;
    Venc_Ctx.Stats.Torn++;
  }

  if ((ret == VENC_RET_FRAME_READY) || (ret == JPEGENC_RESTART_INTERVAL))
  {
    *pJpegBytes = (uint32_t)out.jfifSize;
    Venc_Ctx.Stats.Frames++;
    Venc_Ctx.Stats.LastBytes = *pJpegBytes;
    return BSP_ERROR_NONE;
  }

  Venc_Ctx.Stats.Failures++;
  Venc_Ctx.Stats.LastError = (uint32_t)ret;

  return BSP_ERROR_PERIPH_FAILURE;
}

int32_t BSP_VENC_Resync(uint32_t Instance)
{
  if (Instance >= BSP_VENC_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Venc_Ctx.Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  /* Erratum ES0620 steps 2 to 4; the caller has stopped the camera (step 1). */
  __DSB();
  __ISB();

  VENC_Release();

  /* VENC_Configure runs steps 2-4; if it fails, the next resync tries again. */
  if (VENC_Configure() != BSP_ERROR_NONE)
  {
    Venc_Ctx.Unconfigured = 1U;
    Venc_Ctx.SourceArmed  = 0U;
    return BSP_ERROR_PERIPH_FAILURE;
  }
  Venc_Ctx.Unconfigured = 0U;

  /* The camera is stopped, so the next encode has to start it again. */
  Venc_Ctx.SourceArmed = 0U;

  return BSP_ERROR_NONE;
}

int32_t BSP_VENC_GetStatistics(uint32_t Instance, BSP_VENC_Statistics_t *pStatistics)
{
  if ((Instance >= BSP_VENC_INSTANCES_NBR) || (pStatistics == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  *pStatistics = Venc_Ctx.Stats;

  return BSP_ERROR_NONE;
}

int32_t BSP_VENC_ResetStatistics(uint32_t Instance)
{
  if (Instance >= BSP_VENC_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  memset(&Venc_Ctx.Stats, 0, sizeof(Venc_Ctx.Stats));

  return BSP_ERROR_NONE;
}

int32_t BSP_VENC_SourceLineEvent(uint32_t Instance)
{
  uint32_t rows;

  if (Instance >= BSP_VENC_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  rows = (Venc_Ctx.Band.CodedHeight + VENC_MB_LINES - 1U) / VENC_MB_LINES;

  /* One event is Depth rows, but the count stops at the picture's last row as the
   * encoder's does; counted past it, a short tail would read as a lap. */
  Venc_Ctx.SourceMbRows += Venc_Ctx.Band.Depth;
  if (Venc_Ctx.SourceMbRows > rows)
  {
    Venc_Ctx.SourceMbRows = rows;
  }

  /* Tear probe: register 497 (0x7C4) has the encoder's read row in bits 17:9 and the rows
   * fed to it in 8:0; sampled here because the encoder's per-row interrupt is unreliable.
   * Read row 0 with every row fed is a finished picture, not a lap. */
  /* Not while unconfigured: the encoder's clock is off and a read stalls the bus. */
  if ((Venc_Ctx.SourceAnchored != 0U) && (Venc_Ctx.Initialised != 0U) &&
      (Venc_Ctx.Unconfigured == 0U))
  {
    const uint32_t reg  = LL_VENC_ReadRegister(497UL);
    const uint32_t fed  = reg & 0x1FFUL;
    const uint32_t rd   = (reg >> 9) & 0x1FFUL;
    const uint32_t wr   = Venc_Ctx.SourceMbRows;

    if ((rd != 0U) || (fed < rows))
    {
      const uint32_t gap = (wr > rd) ? (wr - rd) : (rd - wr);

      if (gap > Venc_Ctx.GapMax)
      {
        Venc_Ctx.GapMax = gap;
      }
    }
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_VENC_SourceFrameEvent(uint32_t Instance)
{
  if (Instance >= BSP_VENC_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  Venc_Ctx.SourceMbRows   = 0U;
  Venc_Ctx.SourceAnchored = 1U;

  return BSP_ERROR_NONE;
}

uint32_t BSP_VENC_LastFrameTorn(uint32_t Instance)
{
  if (Instance >= BSP_VENC_INSTANCES_NBR)
  {
    return 0U;
  }

  return (uint32_t)Venc_Ctx.LastTorn;
}

void BSP_VENC_SourceArmHook(void)
{
  if ((Venc_Ctx.Initialised == 0U) || (Venc_Ctx.SourceArmed != 0U) || (Venc_Ctx.FrameMode != 0U))
  {
    return;
  }

  Venc_Ctx.SourceArmed = 1U;

  /* Once per stream: the camera free-runs; a restart costs a vertical-sync re-acquire. */
  if (Venc_Ctx.SourceStart(Venc_Ctx.pSourceArg) != 0)
  {
    /* Retry on the next call rather than wait for a camera that never started. */
    Venc_Ctx.SourceArmed = 0U;
  }
}

__weak void BSP_VENC_WaitBeginHook(void)
{
  /* Nothing to do without a scheduler. See neuron6_venc.h. */
}

__weak void BSP_VENC_ReadPointerHook(uint32_t MbRow)
{
  uint32_t produced = Venc_Ctx.SourceMbRows;

  if (MbRow > Venc_Ctx.Stats.MaxReadRow)
  {
    Venc_Ctx.Stats.MaxReadRow = MbRow;
  }

  /* Read together, so this is a real lead, not two maxima that never coexisted. */
  if ((produced > MbRow) && ((produced - MbRow) > Venc_Ctx.Stats.MaxLead))
  {
    Venc_Ctx.Stats.MaxLead = produced - MbRow;
  }
}
