/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : VGA 60 fps JPEG video from the OV5640 to Neuro Studio,
  *                   over the USB vendor interface, encoded by the hardware
  *                   video encoder with the processor outside the pixel path.
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
  * DCMIPP writes YUV422 slices into an on-chip band, the encoder turns them
  * into JPEG in external RAM, and each frame goes out of the vendor interface
  * as a DNV0 frame. The processor never touches a pixel.
  *
  * The watchdog runs from the first line of main; its early warning, 300 ms
  * before the reset, sends the reset to the bootloader. Backup registers keep
  * breadcrumbs across it: 4 the last stage, 5 encodes, 6 encoder poll-hook
  * calls, 7 the fault handlers. The console, 'b' included, is serviced from
  * the encoder's poll hook; 's' and 'x' start and stop the stream, which also
  * starts once the host has the vendor interface up.
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
#include "neuron6_venc.h"
#include "neuron6_wdg.h"
#if defined(BSP_USE_FREERTOS) && (BSP_USE_FREERTOS == 1)
#include "neuron6_rtos.h"
#endif

#include "tusb.h"

#include <stdbool.h>
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
/* The streamed resolution. To stream another, change this and
 * STREAM_BAND_BYTES together. */
#ifndef STREAM_RESOLUTION
#define STREAM_RESOLUTION       BSP_CAMERA_R640x480
#endif

#ifndef STREAM_BAND_BYTES
#define STREAM_BAND_BYTES       81920U
#endif

/* One finished frame. The worst case, the sensor's random test pattern at
 * QSXGA, codes to 1.24 MB at the default quality. */
#define JPEG_BUF_BYTES          (1536U * 1024U)
#define VND_MAGIC               0x30564E44U   /* 'DNV0' on the wire */
#define VND_FLAG_JPEG           0x00000001U
/* A frame the host has taken none of for this long is replaced by a fresher
 * one. One partly sent is never dropped. */
#define VND_STALL_MS            500U
/* On-chip slice the frame in flight goes to USB through. Bare metal, the pump
 * moves one slice a call, so the encoder's return is polled promptly. */
#define VND_SLICE_BYTES         8192U
#define VND_PUMP_ALL            0xFFFFFFFFU
/* Failed encodes, or torn frames, in a row before the camera and encoder
 * restart. */
#define STREAM_RESYNC_AFTER_FAILS  8U
#define STREAM_RESYNC_AFTER_TORN   2U
/* Overridable from neuron6_conf.h: a longer watchdog leaves a wedge in place
 * to inspect; STREAM_AUTOSTART 0 waits for 's'. */
#ifndef WDG_TIMEOUT_MS
#define WDG_TIMEOUT_MS          2000U
#endif
#define WDG_WARN_MS             300U
#ifndef STREAM_AUTOSTART
#define STREAM_AUTOSTART        1U
#endif

/* Breadcrumb stages, backup register 4. */
#define CRUMB_BOOT              0x10U
#define CRUMB_CAM_INIT          0x11U
#define CRUMB_PIPE              0x12U
#define CRUMB_LINEBUF           0x13U
#define CRUMB_PRIME_START       0x14U
#define CRUMB_PRIME_STOP        0x15U
#define CRUMB_VENC_INIT         0x16U
#define CRUMB_USB_INIT          0x17U
#define CRUMB_IDLE              0x20U
#define CRUMB_ENCODE            0x30U
#define CRUMB_SOURCE_START      0x31U
#define CRUMB_ENCODE_DONE       0x32U
#define CRUMB_PUMP              0x40U
#define CRUMB_ERROR_HANDLER     0xEEU
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BKP(i)                  ((&TAMP->BKP0R)[(i)])
#define CRUMB(stage)            do { BKP(4) = (stage); } while (0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* The camera writes it and the encoder reads it: slices of 16 macroblock
 * rows, wrapped. */
static uint8_t Band[STREAM_BAND_BYTES]
  __attribute__((section(".axisram_nc"), aligned(32)));

/* Two finished frames: one being sent while the next is encoded. */
static uint8_t JpegBuf[2][JPEG_BUF_BYTES]
  __attribute__((section(".ext_psram"), aligned(32)));

/* Frames posted by the encoder, and retired (sent or dropped) by the pump.
 * Frame n is in JpegBuf[n % 2]; the encoder waits only when USB is two
 * behind. */
static volatile uint32_t EncSeq, TxSeq;
static uint32_t          PostedBytes[2];

static vnd_frame_hdr_t VndHdr;
static const uint8_t  *VndPayload;
static uint32_t        VndPayloadLen;
static uint32_t        VndOffset;
static uint32_t        VndTotal;
static uint8_t         VndActive;     /* a frame is in flight (pump only) */
static uint32_t        VndProgress;   /* tick the frame in flight last moved */
static uint32_t        VndFrameId;
static uint64_t        VndSlice[VND_SLICE_BYTES / 8U];

static uint32_t StatEncoded, StatFailed, StatTorn, StatResync, StatGap;
static uint32_t StatSent, StatDropped;
static uint32_t ConsecFail, ConsecTorn;
static volatile uint32_t CamLines;
/* Line events since the camera's last frame event: zero between its frames. */
static volatile uint32_t FrameLines;
/* Set from the camera's frame start to its frame end. */
static volatile uint32_t FrameActive;
/* The next arm follows the last encode at once. Cleared when frames are held
 * back for the host, or an encode fails. */
static uint8_t           ArmFollowsEncode;
/* Main-pipe frame events: tell a stopped camera from gaps the loop missed. */
static volatile uint32_t FrameEnds;
/* A wait for the camera to end its frame: running, start tick, FrameEnds at
 * the start. */
static volatile uint8_t  GapWaiting;
static uint32_t          GapWaitSince;
static uint32_t          GapWaitFrames;
/* How long a frame may take to end before the camera counts as stopped: five
 * frame periods (night mode stretches frames to a quarter of the rate), at
 * least 150 ms, set from the camera's rate. */
static uint32_t          GapWaitMs = 1000U;
#if defined(BSP_USE_FREERTOS) && (BSP_USE_FREERTOS == 1)
/* The encode task, which the frame event wakes while it waits for a gap. */
static TaskHandle_t      EncodeTask;
#endif

/* Kept for the pipe set-up the main loop finishes at the first encode. */
static BSP_VENC_Band_t StreamBand;
static uint8_t         LinebufArmed;
static volatile uint8_t StreamEnabled = STREAM_AUTOSTART;     /* 'x' clears it, 's' sets it */

/* Tick of the last once-a-second report. */
static uint32_t ReportTick;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void Error_Handler(void);
/* USER CODE BEGIN PFP */
static void    mpu_config(void);
static bool    usb_ready(void);
static void    vnd_next(void);
static void    vnd_pump(uint32_t Budget);
static int32_t camera_start(void *pArg);
static int32_t stream_init(void);
static void    console(void);
static void    stream_report_step(uint32_t *pReport);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  MPU: non-cacheable regions 0 .noncacheable, 1 the external memory
  *         window (the encoded frames), 2 .axisram_nc (the encoder's band).
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

  const uint32_t start = 0x70000000;
  const uint32_t end   = 0x71FFFFFF;

  default_config.Number       = MPU_REGION_NUMBER1;
  default_config.BaseAddress  = start;
  default_config.LimitAddress = end;

  HAL_MPU_ConfigRegion(&default_config);

  extern uint8_t __axisram_nc_start[];
  extern uint8_t __axisram_nc_end[];
  if ((uint32_t)__axisram_nc_end > (uint32_t)__axisram_nc_start) {
    default_config.Number       = MPU_REGION_NUMBER2;
    default_config.BaseAddress  = (uint32_t)__axisram_nc_start;
    default_config.LimitAddress = (uint32_t)__axisram_nc_end - 1u;
    HAL_MPU_ConfigRegion(&default_config);
  }

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

  __set_PRIMASK(primask_bit);
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

/* Put the oldest posted frame in flight, if nothing is. */
static void vnd_next(void)
{
  uint32_t slot;

  if (VndActive || (TxSeq == EncSeq))
  {
    return;
  }
  __DMB();                 /* the frame's length is read after its post */
  slot = TxSeq & 1U;
  VndHdr.magic       = VND_MAGIC;
  VndHdr.frame_id    = VndFrameId++;
  VndHdr.meta_len    = 0U;
  VndHdr.payload_len = PostedBytes[slot];
  VndHdr.flags       = VND_FLAG_JPEG;
  VndPayload    = JpegBuf[slot];
  VndPayloadLen = PostedBytes[slot];
  VndOffset     = 0U;
  VndTotal      = sizeof(VndHdr) + VndPayloadLen;
  VndProgress   = HAL_GetTick();
  VndActive     = 1U;
}

/* Copy a double word per bus access: from uncached PSRAM the C library's
 * memcpy moves a byte per access (64 KB: 3.4 ms, against 0.6 ms). */
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

/* Send as much of the frame in flight as the vendor FIFO takes, up to Budget
 * bytes. */
static void vnd_pump(uint32_t Budget)
{
  uint32_t before;

  if (!usb_ready())
  {
    /* No host: drop the frame in flight and any posted one. */
    StatDropped += EncSeq - TxSeq;
    VndActive = 0U;
    TxSeq     = EncSeq;
    return;
  }

  vnd_next();
  if (!VndActive)
  {
    return;
  }

  /* A host that stops reading mid-frame does not disconnect. */
  before = VndOffset;

  while ((VndOffset < VndTotal) && ((VndOffset - before) < Budget))
  {
    uint32_t remaining, avail, chunk, written;

    avail = tud_vendor_n_write_available(0);
    if (avail == 0U)
    {
      break;
    }
    if (VndOffset < sizeof(VndHdr))
    {
      remaining = sizeof(VndHdr) - VndOffset;
      chunk     = (remaining < avail) ? remaining : avail;
      written   = tud_vendor_n_write(0, ((const uint8_t *)&VndHdr) + VndOffset, chunk);
    }
    else
    {
      uint32_t off = VndOffset - sizeof(VndHdr);
      remaining = VndPayloadLen - off;
      chunk     = (remaining < avail) ? remaining : avail;
      chunk     = (chunk < VND_SLICE_BYTES) ? chunk : VND_SLICE_BYTES;
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
    VndActive = 0U;
    TxSeq++;
    StatSent++;
    return;
  }

  if (VndOffset != before)
  {
    VndProgress = HAL_GetTick();          /* it moved: the host is reading */
  }
  else if ((VndOffset == 0U) && ((HAL_GetTick() - VndProgress) >= VND_STALL_MS))
  {
    VndActive = 0U;                       /* nobody is reading: a fresher one */
    TxSeq++;
    StatDropped++;
  }
  else
  {
    /* A partly sent frame waits rather than being dropped: a returning
     * reader would otherwise find the next frame's bytes inside this one's
     * length. */
  }
}

/* Pixel path */

/**
  * @brief  Start the camera. Called by the encoder, which must be armed on
  *         venc_rdy before the first line event.
  */
static int32_t camera_start(void *pArg)
{
  (void)pArg;
  CRUMB(CRUMB_SOURCE_START);
  return (BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, Band,
                           BSP_CAMERA_MODE_CONTINUOUS) == BSP_ERROR_NONE) ? 0 : -1;
}

/**
  * @brief  Called on every poll of BSP_VENC_EncodeBand: keeps USB, the
  *         watchdog and the console going. Bare metal, it also sends the
  *         previous frame, a slice a call, and the report; under FreeRTOS
  *         the USB task does.
  */
void BSP_VENC_ReadPointerHook(uint32_t MbRow)
{
  static uint32_t last_service = 0U;
  (void)MbRow;
  BKP(6)++;
#if !defined(BSP_USE_FREERTOS) || (BSP_USE_FREERTOS == 0)
  BSP_COM_Task();
  vnd_pump(VND_SLICE_BYTES);
#endif
  if ((HAL_GetTick() - last_service) >= 5U)
  {
    last_service = HAL_GetTick();
    console();
#if !defined(BSP_USE_FREERTOS) || (BSP_USE_FREERTOS == 0)
    stream_report_step(&ReportTick);
#endif
  }
}

void BSP_CAMERA_LineEventCallback(uint32_t Instance, uint32_t Pipe)
{
  if ((Instance == 0U) && (Pipe == BSP_CAMERA_PIPE_MAIN))
  {
    CamLines++;
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
#if defined(BSP_USE_FREERTOS) && (BSP_USE_FREERTOS == 1)
    if ((GapWaiting != 0U) && (EncodeTask != NULL))
    {
      /* The encode task waits for this: arm it at the start of the gap. */
      BaseType_t woken = pdFALSE;

      vTaskNotifyGiveFromISR(EncodeTask, &woken);
      portYIELD_FROM_ISR(woken);
    }
#endif
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

/* Watchdog early warning, 300 ms before the reset, in interrupt context: stop
 * capture and the encoder (bounded, no HAL timing), then the bootloader. */
void BSP_WDG_EWI_Callback(void)
{
  (void)BSP_CAMERA_HaltCapture(0);
  (void)BSP_VENC_Quiesce(0);
  BSP_COM_RebootToBootloader();
}

/**
  * @brief  Camera, pipe, line buffer, prime, encoder. Everything but USB.
  */
static int32_t stream_init(void)
{
  BSP_CAMERA_PipeConfig_t   pipe = {0};
  BSP_CAMERA_Capabilities_t cap  = {0};
  BSP_VENC_Config_t         venc = {0};
  BSP_VENC_Band_t           band;
  int32_t                   status;

  /* Bring-up never reaches the main loop: refresh the watchdog between
   * steps. */
  (void)BSP_WDG_Refresh();

  status = BSP_CAMERA_Init(0, STREAM_RESOLUTION, BSP_CAMERA_PF_RGB565);
  if (status != BSP_ERROR_NONE) { return status; }
  CRUMB(CRUMB_CAM_INIT);
  (void)BSP_WDG_Refresh();

  /* The picture size comes from the sensor. */
  status = BSP_CAMERA_GetCapabilities(0, &cap);
  if (status != BSP_ERROR_NONE) { return status; }
  GapWaitMs = (cap.MaxFrameRate != 0U) ? (5000U / cap.MaxFrameRate) : 1000U;
  if (GapWaitMs < 150U) { GapWaitMs = 150U; }

  status = BSP_VENC_GetBandGeometry(cap.Width, cap.Height, &band);
  if (status != BSP_ERROR_NONE) { return status; }
  /* An undersized band fails here rather than tearing later. */
  if (band.SizeBytes > sizeof(Band)) { return BSP_ERROR_WRONG_PARAM; }

  pipe.PixelFormat = BSP_CAMERA_PIPE_PF_YUV422;
  pipe.Width       = cap.Width;
  pipe.Height      = band.CodedHeight;
  status = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pipe);
  if (status != BSP_ERROR_NONE) { return status; }
  CRUMB(CRUMB_PIPE);
  (void)BSP_WDG_Refresh();

  /* Prime the line buffer before the pipe starts. */
  status = BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN,
                                       band.WrapLines, BSP_CAMERA_LINES_OFF);
  if (status != BSP_ERROR_NONE) { return status; }
  CRUMB(CRUMB_LINEBUF);
  (void)BSP_WDG_Refresh();

  /* Prime, and leave it running into the wrapped band. The first encode, in
   * the main loop, stops it, flushes the receiver and arms the pacing event
   * just before arming the encoder. */
  status = BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, Band, BSP_CAMERA_MODE_CONTINUOUS);
  if (status != BSP_ERROR_NONE) { return status; }
  CRUMB(CRUMB_PRIME_START);
  (void)BSP_WDG_Refresh();

  StreamBand   = band;
  LinebufArmed = 0U;

  venc.Band        = band;
  venc.pBand       = Band;
  venc.Quality     = BSP_VENC_QUALITY_DEFAULT;
  venc.SourceStart = camera_start;
  venc.pSourceArg  = NULL;
  status = BSP_VENC_Init(0, &venc);
  if (status == BSP_ERROR_NONE) { CRUMB(CRUMB_VENC_INIT); }
  return status;
}

/* Console */

/**
  * @brief  Service USB, the watchdog and the console; also called from inside
  *         the encode.
  */
static void console(void)
{
  uint8_t ch;

#if !defined(BSP_USE_FREERTOS) || (BSP_USE_FREERTOS == 0)
  BSP_COM_Task();
#endif
  (void)BSP_WDG_Refresh();

  while (BSP_COM_Read(&ch, 1U) == 1)
  {
    switch (ch)
    {
      case 'b':
        /* Never reset with the encoder in flight: stop the pipe and quiesce
         * the encoder first (ES0620 steps 1 to 3). */
        (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
        (void)BSP_VENC_Quiesce(0);
        BSP_COM_RebootToBootloader();                             /* does not return */
        break;
      case 's': StreamEnabled = 1U; BSP_COM_Printf("[n6] stream on\r\n"); break;
      case 'x': StreamEnabled = 0U; BSP_COM_Printf("[n6] stream off\r\n"); break;
      case '?':
      {
        BSP_VENC_Statistics_t st;
        (void)BSP_VENC_GetStatistics(0, &st);
        BSP_COM_Printf("[n6] stage=0x%02lx encodes=%lu hooks=%lu lines=%lu frames=%lu fail=%lu "
                       "lastB=%lu gap=%lu torn=%lu err=%ld mounted=%d vnd=%d\r\n",
                       (unsigned long)BKP(4), (unsigned long)BKP(5), (unsigned long)BKP(6),
                       (unsigned long)CamLines, (unsigned long)st.Frames, (unsigned long)st.Failures,
                       (unsigned long)st.LastBytes, (unsigned long)st.LastGap, (unsigned long)st.Torn,
                       (long)(int32_t)st.LastError, tud_mounted() ? 1 : 0, tud_vendor_n_mounted(0) ? 1 : 0);
        break;
      }
      default: break;
    }
  }
}

/* The loop */

/* Result of stream_init, kept for the encode gate and the report. */
static int32_t StreamInitStatus = BSP_ERROR_NO_INIT;

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

/**
  * @brief  One encode, while the host has the vendor interface up and a
  *         buffer is free.
  * @retval 1 if an encode ran, 0 if there was nothing to do.
  */
static int stream_encode_step(void)
{
  BSP_VENC_Statistics_t st;
  uint32_t              bytes = 0U;
  uint32_t              slot;
  uint8_t               follow_on;
  int32_t               enc;

  if (!StreamEnabled || !usb_ready() || (StreamInitStatus != BSP_ERROR_NONE))
  {
    /* Stopped: park, so the next start is clean rather than mid-frame. */
    if (LinebufArmed)
    {
      stream_park();
    }
    CRUMB(CRUMB_IDLE);
    return 0;
  }

  /* Both buffers are with USB. */
  if ((EncSeq - TxSeq) >= 2U)
  {
    GapWaiting = 0U;
    ArmFollowsEncode = 0U;
    CRUMB(CRUMB_IDLE);
    return 0;
  }
  follow_on        = ArmFollowsEncode;
  ArmFollowsEncode = 0U;

  /* Just before the first encode, and the first after a park: stop the
   * free-running pipe (flushing the receiver FIFOs) and arm the pacing
   * event. */
  if (!LinebufArmed)
  {
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
    CRUMB(CRUMB_PRIME_STOP);
    (void)BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN,
                                      StreamBand.WrapLines, StreamBand.EventLines);
    CRUMB(CRUMB_LINEBUF);
    LinebufArmed = 1U;
  }
  else if ((FrameLines != 0U) || ((follow_on == 0U) && (FrameActive != 0U)))
  {
    /* Mid-frame: arming now would code two frames as one, so wait for the
     * blanking. A follow-on arm waits only for arrived lines (waiting out the
     * frame start too would skip every other QSXGA frame); any other arm also
     * waits out frame start to first line event, or it runs a band behind. */
    if (GapWaiting == 0U)
    {
      GapWaitSince  = HAL_GetTick();
      GapWaitFrames = FrameEnds;
      GapWaiting    = 1U;
    }
    if ((HAL_GetTick() - GapWaitSince) < GapWaitMs)
    {
      CRUMB(CRUMB_IDLE);
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
    /* Frames ended but every gap was missed: arm anyway; the torn-frame
     * check drops the result. */
  }
  GapWaiting = 0U;

  slot = EncSeq & 1U;
  CRUMB(CRUMB_ENCODE);
  BKP(5)++;
  enc = BSP_VENC_EncodeBand(0, JpegBuf[slot], JPEG_BUF_BYTES, &bytes);
  CRUMB(CRUMB_ENCODE_DONE);
  if (enc != BSP_ERROR_NONE)
  {
    /* A run of failures means lost sync, which only a peripheral reset
     * clears. */
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

  if (BSP_VENC_GetStatistics(0, &st) == BSP_ERROR_NONE)
  {
    if (st.LastGap > StatGap) { StatGap = st.LastGap; }
  }

  /* A late-armed encode codes a picture split between two frames: never
   * sent, and a second in a row restarts the pair. */
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

/**
  * @brief  The once-a-second report, when a second has passed.
  */
static void stream_report_step(uint32_t *pReport)
{
  if ((HAL_GetTick() - *pReport) >= 1000U)
  {
    BSP_VENC_Statistics_t st;

    *pReport = HAL_GetTick();
    if (BSP_VENC_GetStatistics(0, &st) == BSP_ERROR_NONE)
    {
      BSP_COM_Printf("[n6] enc=%lu fail=%lu torn=%lu resync=%lu sent=%lu drop=%lu last=%luB "
                     "gap=%lu/%lu err=%ld lines=%lu init=%ld\r\n",
                     (unsigned long)StatEncoded, (unsigned long)StatFailed,
                     (unsigned long)StatTorn, (unsigned long)StatResync,
                     (unsigned long)StatSent, (unsigned long)StatDropped,
                     (unsigned long)st.LastBytes, (unsigned long)StatGap,
                     (unsigned long)(2U * StreamBand.Depth),
                     (long)(int32_t)st.LastError, (unsigned long)CamLines, (long)StreamInitStatus);
    }
    StatEncoded = 0U; StatFailed = 0U; StatTorn = 0U; StatResync = 0U; StatGap = 0U;
    StatSent = 0U; StatDropped = 0U;
    (void)BSP_VENC_ResetStatistics(0);
  }
}

#if defined(BSP_USE_FREERTOS) && (BSP_USE_FREERTOS == 1)
/* FreeRTOS: the same work as two tasks, encode (+4) above USB (+3). */
#define STREAM_USB_TASK_STACK_WORDS     1024U
#define STREAM_USB_TASK_PRIORITY        (tskIDLE_PRIORITY + 3U)
#define STREAM_ENCODE_TASK_STACK_WORDS  2048U
#define STREAM_ENCODE_TASK_PRIORITY     (tskIDLE_PRIORITY + 4U)

static void stream_usb_task(void *pArg)
{
  uint32_t report = 0U;
  (void)pArg;

  BSP_COM_SetPumpTask(xTaskGetCurrentTaskHandle());
  for (;;)
  {
    BSP_COM_TaskWait(1U);
    CRUMB(CRUMB_PUMP);
    vnd_pump(VND_PUMP_ALL);
    console();
    stream_report_step(&report);
  }
}

static void stream_encode_task(void *pArg)
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
#endif

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

    /* Breadcrumbs and the watchdog first, before anything that could wedge. */
  HAL_PWR_EnableBkUpAccess();
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U)
  {
    __HAL_RCC_CLEAR_RESET_FLAGS();
    BSP_COM_RebootToBootloader();
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
  BKP(5) = 0U;
  BKP(6) = 0U;
  CRUMB(CRUMB_BOOT);
  (void)BSP_WDG_InitEWI(WDG_TIMEOUT_MS, WDG_WARN_MS);

  /* Camera and encoder first, USB last, then service it. */
  StreamInitStatus = stream_init();

  if (BSP_COM_Init() != BSP_ERROR_NONE)
  {
    (void)BSP_LED_On(LED_RED);
    Error_Handler();
  }
  CRUMB(CRUMB_USB_INIT);

  if (StreamInitStatus != BSP_ERROR_NONE)
  {
    /* No video, but a console and a way back to the bootloader. */
    StreamEnabled = 0U;
  }
  (void)BSP_LED_On(LED_GREEN);

#if defined(BSP_USE_FREERTOS) && (BSP_USE_FREERTOS == 1)
    /* Two tasks and the scheduler; see stream_usb_task. */
  if ((xTaskCreate(stream_usb_task, "usb", STREAM_USB_TASK_STACK_WORDS, NULL,
                   STREAM_USB_TASK_PRIORITY, NULL) != pdPASS) ||
      (xTaskCreate(stream_encode_task, "encode", STREAM_ENCODE_TASK_STACK_WORDS, NULL,
                   STREAM_ENCODE_TASK_PRIORITY, &EncodeTask) != pdPASS))
  {
    (void)BSP_LED_On(LED_RED);
    Error_Handler();
  }
  vTaskStartScheduler();
  Error_Handler();
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    console();
    CRUMB(CRUMB_PUMP);
    vnd_pump(VND_SLICE_BYTES);
    stream_report_step(&ReportTick);

    /* Encodes run back to back, each frame sent from the poll hook during the
     * next encode: at 1080p the next band comes 3.3 ms after a frame ends,
     * the encode returns at 1.0 ms, and sending 38 KB first takes 2.2 ms. */
    while (stream_encode_step() != 0)
    {
    }
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
  /* Leave a crumb and go to the bootloader. */
  HAL_PWR_EnableBkUpAccess();
  CRUMB(CRUMB_ERROR_HANDLER);
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
