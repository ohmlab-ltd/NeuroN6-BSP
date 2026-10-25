/**
  * @file    neuron6_venc.h
  * @brief   Hardware JPEG encoding, fed by the camera over the DCMIPP-VENC line buffer.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_VENC_H
#define NEURON6_VENC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "neuron6_conf.h"
#include "neuron6_errno.h"

/* The STM32N6 carries a Verisilicon VC8000NanoE video encoder. */

/* One encoder: every Instance argument must be 0. */
#define BSP_VENC_INSTANCES_NBR          1U

/** @brief Encoder heap in bytes: vendor pool plus instance and working buffers. */
#define BSP_VENC_HEAP_BYTES             (72U * 1024U)

/** @brief Required alignment of every band and output buffer, in bytes. */
#define BSP_VENC_BUFFER_ALIGNMENT       16U

/** @brief Widest coded picture, in pixels (the sensor's widest mode). Capability register
  * 63 reads 1920, but YUV422, the only input used here, codes wider. */
#define BSP_VENC_MAX_CODED_WIDTH        2592U

/** @brief Quality 0 (coarsest) to 9: one of ten built-in tables, not a percentage. */
#define BSP_VENC_QUALITY_MIN            0U
#define BSP_VENC_QUALITY_DEFAULT        2U
#define BSP_VENC_QUALITY_MAX            9U

/** @brief Band geometry from BSP_VENC_GetBandGeometry(); the fields must agree with each
  * other or the stream tears. */
typedef struct
{
  uint32_t Pitch;      /*!< Line length: the width in YUV422 bytes, rounded up to 16. */
  uint32_t Depth;      /*!< Macroblock-row chunks the camera may run ahead: 1, 2 or 4. */
  uint32_t SizeBytes;  /*!< Band size. Allocate exactly this.                */
  uint32_t WrapLines;  /*!< BSP_CAMERA_ConfigLineBuffer WrapLines: Depth * 32. */
  uint32_t EventLines; /*!< BSP_CAMERA_ConfigLineBuffer EventLines: Depth * 16. */
  uint32_t CodedWidth; /*!< Width the encoder will code.                     */
  uint32_t CodedHeight;/*!< Frame height floored to the 8-line macroblock.   */
} BSP_VENC_Band_t;

typedef struct
{
  BSP_VENC_Band_t Band;        /*!< From BSP_VENC_GetBandGeometry.          */
  uint8_t        *pBand;       /*!< Band.SizeBytes, 16-byte aligned, non-cacheable, and in
                                *   on-chip RAM: from PSRAM every encode fails.      */
  uint32_t        Quality;     /*!< BSP_VENC_QUALITY_MIN..MAX.              */

  /** @brief Starts the camera, once per stream, from inside the first BSP_VENC_EncodeBand
    * with the encoder armed and waiting: the camera must not start first. */
  int32_t       (*SourceStart)(void *pArg);
  void           *pSourceArg;  /*!< Handed back to SourceStart.             */
} BSP_VENC_Config_t;

/** @brief Stream counters, from BSP_VENC_GetStatistics(). */
typedef struct
{
  uint32_t Frames;        /*!< Encodes that returned a frame.               */
  uint32_t Failures;      /*!< Encodes that did not.                        */
  uint32_t LastBytes;     /*!< Size of the last encoded frame.              */
  uint32_t LastError;     /*!< Vendor code of the last failure; -15, a hardware reset,
                            *   usually means the encoder and camera lost sync.  */
  uint32_t MaxReadRow;    /*!< Highest macroblock row the encoder reached; bare metal. */
  uint32_t MaxLead;       /*!< Camera's highest lead, in rows (vs Depth); bare metal. */
  uint32_t LastGap;       /*!< Largest camera-to-encoder gap in the last encode, in rows,
                            *   with BSP_VENC_SourceFrameEvent; 2 x Depth = torn. */
  uint32_t Torn;          /*!< Encodes whose LastGap reached that.          */
} BSP_VENC_Statistics_t;

/**
  * @brief  Clock the encoder and admit it to the isolation scheme.
  * @note   Call while configuring the camera pipe, before it starts; BSP_VENC_Init later.
  */
void BSP_VENC_EnablePeripheral(void);

/**
  * @brief  Band geometry for the camera's frame size in pixels.
  * @retval BSP_ERROR_FEATURE_NOT_SUPPORTED if Width is not a multiple of 16 or is over
  *         BSP_VENC_MAX_CODED_WIDTH.
  */
int32_t BSP_VENC_GetBandGeometry(uint32_t Width, uint32_t Height, BSP_VENC_Band_t *pBand);

/**
  * @brief  Bring the encoder up: YUV422, line-buffer mode, hardware handshake.
  * @param  pConfig With pBand and SourceStart both NULL, memory-source mode instead
  *         (BSP_VENC_EncodeFrame); Band still gives the coded size.
  * @retval BSP_ERROR_NO_INIT: the encoder refused (too little heap, or an uncodable size).
  */
int32_t BSP_VENC_Init(uint32_t Instance, const BSP_VENC_Config_t *pConfig);

/**
  * @brief  Shut the encoder down and free its heap, and anything BSP_VENC_Quiesce left.
  * @retval BSP_ERROR_NO_INIT if the encoder was not up.
  */
int32_t BSP_VENC_DeInit(uint32_t Instance);

/**
  * @brief  Stop the encoder safely before a reset; call after stopping the camera pipe.
  * @note   Safe in an interrupt or fault handler (no heap); the next BSP_VENC_Init or
  *         BSP_VENC_DeInit frees its memory. BSP_ERROR_NONE if there was nothing to do.
  */
int32_t BSP_VENC_Quiesce(uint32_t Instance);

/**
  * @brief  Encode one frame out of the band; blocks for about a frame period.
  * @note   The camera starts on the first call. A frame slower than EWL_TIMEOUT (300 ms,
  *         Components/venc/EWL/ewl_conf.h) fails with LastError -11: under about 3.5 fps,
  *         or 7 fps for the first frame after the camera starts.
  * @note   pOut: 16-byte aligned and not cached, as the encoder writes past the cache
  *         after the CPU's JFIF header.
  * @note   OutBytes must cover the worst case (noise): an overflow fails the whole frame
  *         (LastError -6). The examples use 512 KB.
  * @retval BSP_ERROR_PERIPH_FAILURE on failure; BSP_VENC_GetStatistics says why.
  */
int32_t BSP_VENC_EncodeBand(uint32_t Instance, uint8_t *pOut, uint32_t OutBytes,
                            uint32_t *pJpegBytes);

/**
  * @brief  Memory-source mode: encode a whole frame in memory (no camera, no band).
  * @note   pSource: 16-byte aligned, and not cached or cleaned (SCB_CleanDCache_by_Addr)
  *         after the CPU last wrote it. pOut and OutBytes as for BSP_VENC_EncodeBand.
  */
int32_t BSP_VENC_EncodeFrame(uint32_t Instance, const uint8_t *pSource, uint8_t *pOut,
                             uint32_t OutBytes, uint32_t *pJpegBytes);

/**
  * @brief  Recover from lost encoder-camera sync (erratum ES0620). Stop the camera
  *         first; the next encode restarts it.
  * @retval BSP_ERROR_PERIPH_FAILURE: not reconfigured; encodes fail until a resync works.
  */
int32_t BSP_VENC_Resync(uint32_t Instance);

int32_t BSP_VENC_GetStatistics(uint32_t Instance, BSP_VENC_Statistics_t *pStatistics);

int32_t BSP_VENC_ResetStatistics(uint32_t Instance);

/** @brief Call from BSP_CAMERA_LineEventCallback for the pipe feeding the band. */
int32_t BSP_VENC_SourceLineEvent(uint32_t Instance);

/**
  * @brief  Call from BSP_CAMERA_FrameEventCallback for the pipe feeding the band.
  * @note   Anchors the camera's row count to the frame, so a late arm shows in LastGap;
  *         without it the count resets at each encode.
  */
int32_t BSP_VENC_SourceFrameEvent(uint32_t Instance);

/** @brief 1 if the last encode lapped the band: the frame is torn, do not send it. */
uint32_t BSP_VENC_LastFrameTorn(uint32_t Instance);

/** @brief Internal: called with the encoder armed and waiting; runs SourceStart. */
void BSP_VENC_SourceArmHook(void);

/** @brief Weak, empty; called once per encode as the wait begins. An RTOS can drop the
  * encode's raised priority here so another task drains the previous frame. */
void BSP_VENC_WaitBeginHook(void);

/**
  * @brief  Weak; gets the encoder's read row (MbRow) while a bare-metal encode polls.
  * @note   The default feeds MaxReadRow and MaxLead; under FreeRTOS it is never called.
  */
void BSP_VENC_ReadPointerHook(uint32_t MbRow);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_VENC_H */
