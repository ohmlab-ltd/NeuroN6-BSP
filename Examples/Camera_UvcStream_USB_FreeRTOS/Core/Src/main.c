/*
  Camera_UvcStream_USB_FreeRTOS

  Streams the camera as a standard USB video device under FreeRTOS: VGA,
  Motion-JPEG, sixty frames a second. Any webcam application shows the
  picture, with no driver. A capture task encodes; a USB task runs TinyUSB,
  in DMA mode, and sends the frames.

  The circuit:
  - Uses the on-board camera module on the 30-pin connector. No wiring needed.

  A USB console enumerates alongside the camera: send 'b' on it to return to
  the bootloader.

  Copyright (c) 2026 Ohm Lab Ltd.
  SPDX-License-Identifier: CC0-1.0
  Released under CC0 1.0 Universal: copy it into your own project
  freely, with or without attribution.
*/

#include "main.h"

#include "neuron6.h"
#include "neuron6_com.h"
#include "neuron6_camera.h"
#include "neuron6_venc.h"
#include "neuron6_wdg.h"
#include "neuron6_rtos.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "tusb.h"

#include <stdbool.h>

#if !defined(BSP_USE_FREERTOS) || (BSP_USE_FREERTOS != 1)
#error "This example runs under FreeRTOS: set BSP_USE_FREERTOS to 1 in neuron6_conf.h."
#endif

/* This example's Utilities/usb/usb_descriptors.c declares the same size and
 * rate. */
#define UVC_RESOLUTION          BSP_CAMERA_R640x480

/* The band the camera writes and the encoder reads, sized for VGA. */
#define UVC_BAND_BYTES          81920U

/* One finished JPEG: a VGA frame runs to a few tens of kilobytes. */
#define JPEG_BUF_BYTES          (256U * 1024U)

#define WDG_TIMEOUT_MS          2000U
#define WDG_WARN_MS             300U

/* USB is serviced this long before the scheduler starts, for enumeration. */
#define USB_BOOT_PUMP_MS        500U

/* If no host has enumerated the device by then, return to the bootloader so
 * the board can be reflashed. Applies until a host is seen; 0 disables it. */
#ifndef UVC_HOST_TIMEOUT_MS
#define UVC_HOST_TIMEOUT_MS     30000U
#endif

/* A run of failed encodes, or of torn frames, means the encoder and camera
 * have slipped: stop, reset the encoder and start again. */
#define UVC_RESYNC_AFTER_FAILS  8U
#define UVC_RESYNC_AFTER_TORN   2U

/* Slots outstanding this long, with the link idle and nothing queued, are
 * lost accounting and are reclaimed. */
#define USB_JAM_MS              1000U

/* Both tasks share a base priority; the capture task is raised one step
 * across a re-arm. */
#define USB_TASK_STACK_WORDS      (8192U / sizeof(StackType_t))
#define USB_TASK_PRIORITY         (tskIDLE_PRIORITY + 3U)
#define CAPTURE_TASK_STACK_WORDS  (8192U / sizeof(StackType_t))
#define CAPTURE_BASE_PRIORITY     (tskIDLE_PRIORITY + 3U)
#define CAPTURE_REARM_PRIORITY    (tskIDLE_PRIORITY + 4U)

/* A finished frame, as the capture task posts it to the USB task. */
typedef struct
{
  uint8_t  *jpeg_ptr;
  uint32_t  jpeg_len;
} frame_msg_t;

typedef enum
{
  CAPTURE_READY,
  CAPTURE_TORN,
  CAPTURE_FAILED,
  CAPTURE_WAIT                   /* not armed: the camera is mid-frame */
} capture_result_t;

/* How a wait for the camera to end its frame came out. */
typedef enum
{
  CAPTURE_GAP_OPEN,              /* between frames: arm now */
  CAPTURE_GAP_MISSED,            /* frames ended, every gap missed: arm anyway */
  CAPTURE_GAP_STOPPED            /* no frame ended: the camera has stopped */
} capture_gap_t;

/* The camera writes here, wrapped, and the encoder follows it. */
static uint8_t Band[UVC_BAND_BYTES]
  __attribute__((section(".axisram_nc"), aligned(32)));

/* Two finished frames: one on the wire while the next is encoded. */
static uint8_t JpegBuf[2][JPEG_BUF_BYTES]
  __attribute__((section(".ext_psram"), aligned(32)));

/* Frame N is encoded into JpegBuf[N & 1]. g_enc_seq counts frames posted,
 * g_tx_seq frames the USB task has finished: at most two are outstanding. */
static QueueHandle_t     g_video_usb_queue;
static volatile uint32_t g_enc_seq;
static volatile uint32_t g_tx_seq;

/* Set while the video class holds a frame. */
static volatile bool     uvc_tx_busy = false;

/* Owned by the capture task. */
static BSP_VENC_Band_t   StreamBand;
static uint8_t           LinebufArmed;
static uint32_t          ConsecFail;
/* Line events since the last frame event: zero between frames. */
static volatile uint32_t FrameLines;
/* Non-zero from a frame's start to its end; FrameLines counts only from its
 * first line event. */
static volatile uint32_t FrameActive;
/* The last encode coded a frame, so the next arm follows it straight away. */
static uint8_t           ArmFollowsEncode;
/* Frame ends, which tell a stopped camera from gaps the capture task missed. */
static volatile uint32_t FrameEnds;
/* Set while the capture task waits for a frame to end; the frame event then
 * wakes it, to arm at the start of the gap. */
static volatile uint8_t  CaptureWaiting;
static TaskHandle_t      CaptureTask;
/* Arm where the camera is: the wait saw frames end but caught no gap. */
static uint8_t           ArmAnyway;
/* How long a capture waits for a frame to end before taking the camera for
 * stopped: five frame periods (night mode stretches them to a quarter of the
 * rate), at least 150 ms. */
static uint32_t          CaptureGapWaitMs = 1000U;
static int32_t           InitStatus = BSP_ERROR_NONE;

/* Counters for the report, each written by one task and read by the other. */
static volatile uint32_t CapEncoded, CapTorn, CapFailed, CapResync, CapBytes, CapGap;
static volatile uint32_t UsbSent;

static void    mpu_config(void);
static int32_t video_init(void);

void Error_Handler(void);

/* Formatted output to the USB console. Dropped, not blocked, with no host. */
#define print(...)   (void)BSP_COM_Printf(__VA_ARGS__)

/* Capture */

/**
  * @brief  Start the camera. The encoder calls this once per stream, after it
  *         has parked waiting for the first line event.
  */
static int32_t camera_start(void *pArg)
{
  (void)pArg;
  return (BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, Band, BSP_CAMERA_MODE_CONTINUOUS)
          == BSP_ERROR_NONE) ? 0 : -1;
}

/**
  * @brief  Each line event tells the encoder more of the band is written.
  */
void BSP_CAMERA_LineEventCallback(uint32_t Instance, uint32_t Pipe)
{
  if ((Instance == 0U) && (Pipe == BSP_CAMERA_PIPE_MAIN))
  {
    FrameLines++;
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
    FrameLines = 0U;
    FrameActive = 0U;
    (void)BSP_VENC_SourceFrameEvent(0);
    FrameEnds++;
    if (CaptureWaiting != 0U)
    {
      BaseType_t woken = pdFALSE;

      CaptureWaiting = 0U;
      vTaskNotifyGiveFromISR(CaptureTask, &woken);
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
  * @brief  Camera, pipe, line buffer, prime, encoder.
  */
static int32_t video_init(void)
{
  BSP_CAMERA_PipeConfig_t   pipe = {0};
  BSP_CAMERA_Capabilities_t cap  = {0};
  BSP_VENC_Config_t         venc = {0};
  BSP_VENC_Band_t           band;
  int32_t                   status;

  (void)BSP_WDG_Refresh();

  status = BSP_CAMERA_Init(0, UVC_RESOLUTION, BSP_CAMERA_PF_RGB565);
  if (status != BSP_ERROR_NONE) { return status; }
  (void)BSP_WDG_Refresh();

  status = BSP_CAMERA_GetCapabilities(0, &cap);
  if (status != BSP_ERROR_NONE) { return status; }
  CaptureGapWaitMs = (cap.MaxFrameRate != 0U) ? (5000U / cap.MaxFrameRate) : 1000U;
  if (CaptureGapWaitMs < 150U) { CaptureGapWaitMs = 150U; }

  status = BSP_VENC_GetBandGeometry(cap.Width, cap.Height, &band);
  if (status != BSP_ERROR_NONE) { return status; }
  if (band.SizeBytes > sizeof(Band)) { return BSP_ERROR_WRONG_PARAM; }

  pipe.PixelFormat = BSP_CAMERA_PIPE_PF_YUV422;
  pipe.Width       = cap.Width;
  pipe.Height      = band.CodedHeight;
  status = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pipe);
  if (status != BSP_ERROR_NONE) { return status; }
  (void)BSP_WDG_Refresh();

  status = BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN,
                                       band.WrapLines, BSP_CAMERA_LINES_OFF);
  if (status != BSP_ERROR_NONE) { return status; }
  (void)BSP_WDG_Refresh();

  /* Prime: start the pipe once and leave it running. The encoder's first arm
   * expects a pipe started and stopped before; the first capture stops it. */
  status = BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, Band, BSP_CAMERA_MODE_CONTINUOUS);
  if (status != BSP_ERROR_NONE) { return status; }
  (void)BSP_WDG_Refresh();

  StreamBand   = band;
  LinebufArmed = 0U;

  venc.Band        = band;
  venc.pBand       = Band;
  venc.Quality     = BSP_VENC_QUALITY_DEFAULT;
  venc.SourceStart = camera_start;
  venc.pSourceArg  = NULL;
  return BSP_VENC_Init(0, &venc);
}

/**
  * @brief  Recovery: stop the pipe, reset the encoder, and let the
  *         next capture reconfigure and start again, encoder first.
  */
static void capture_resync(void)
{
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
  FrameLines   = 0U;        /* the stopped camera's frame will not end */
  FrameActive  = 0U;
  ArmAnyway    = 0U;
  (void)BSP_VENC_Resync(0);
  LinebufArmed = 0U;
  ConsecFail   = 0U;
  CapResync++;
}

/**
  * @brief  Encode one frame into JpegBuf[Slot], arming first if needed and
  *         resyncing after a run of failures.
  */
static capture_result_t capture_one(uint32_t Slot, uint32_t *pBytes)
{
  BSP_VENC_Statistics_t st;
  const uint8_t         follow_on = ArmFollowsEncode;

  ArmFollowsEncode = 0U;

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
  else if ((ArmAnyway == 0U) &&
           ((FrameLines != 0U) || ((follow_on == 0U) && (FrameActive != 0U))))
  {
    /* Mid-frame: an encode armed now would code the rest of this frame and
     * the top of the next as one picture, so wait for the vertical blanking.
     * A follow-on arm waits only for arrived lines (else QSXGA's short
     * blanking skips every other frame); any other arm also waits out the
     * frame's start, or it can miss the first line event. */
    return CAPTURE_WAIT;
  }
  ArmAnyway = 0U;

  /* Blocks for about a frame; the task sleeps while the hardware encodes. */
  if (BSP_VENC_EncodeBand(0, JpegBuf[Slot], JPEG_BUF_BYTES, pBytes) != BSP_ERROR_NONE)
  {
    if (++ConsecFail >= UVC_RESYNC_AFTER_FAILS)
    {
      capture_resync();
    }
    return CAPTURE_FAILED;
  }
  ConsecFail = 0U;
  ArmFollowsEncode = 1U;

  if (BSP_VENC_GetStatistics(0, &st) == BSP_ERROR_NONE)
  {
    CapGap = st.LastGap;
  }

  return (BSP_VENC_LastFrameTorn(0) != 0U) ? CAPTURE_TORN : CAPTURE_READY;
}

/**
  * @brief  Queue the frame for the USB task; advance g_enc_seq only if queued.
  */
static void post_encoded_frame(uint8_t *pBuf, uint32_t Bytes)
{
  frame_msg_t msg;

  msg.jpeg_ptr = pBuf;
  msg.jpeg_len = Bytes;
  if (xQueueSend(g_video_usb_queue, &msg, 0) == pdTRUE)
  {
    g_enc_seq++;
    CapEncoded++;
    CapBytes = Bytes;
  }
}

/**
  * @brief  Wait up to CaptureGapWaitMs for the camera to end its frame. The
  *         frame event wakes the task at its raised priority, so it arms at
  *         the start of the gap.
  * @retval See capture_gap_t.
  */
static capture_gap_t capture_wait_for_gap(void)
{
  const TickType_t start  = xTaskGetTickCount();
  const TickType_t limit  = pdMS_TO_TICKS(CaptureGapWaitMs);
  const uint32_t   frames = FrameEnds;

  for (;;)
  {
    const TickType_t waited = xTaskGetTickCount() - start;

    if (waited >= limit)
    {
      return (FrameEnds == frames) ? CAPTURE_GAP_STOPPED : CAPTURE_GAP_MISSED;
    }

    /* Clear a stale wake, raise the flag, then look again: a frame that ended
     * meanwhile is seen here, and one that ends later wakes the task. */
    (void)ulTaskNotifyTake(pdTRUE, 0U);
    CaptureWaiting = 1U;
    if ((FrameLines == 0U) && (FrameActive == 0U))
    {
      CaptureWaiting = 0U;
      return CAPTURE_GAP_OPEN;
    }
    (void)ulTaskNotifyTake(pdTRUE, limit - waited);
    CaptureWaiting = 0U;
    if ((FrameLines == 0U) && (FrameActive == 0U))
    {
      return CAPTURE_GAP_OPEN;
    }
  }
}

static void capture_task(void *pArg)
{
  uint32_t torn_run = 0U;

  (void)pArg;

  for (;;)
  {
    capture_result_t result;
    uint32_t         bytes = 0U;
    uint32_t         slot;
    bool             can_encode;

    if (InitStatus != BSP_ERROR_NONE)
    {
      vTaskDelay(pdMS_TO_TICKS(100U));
      continue;
    }

    can_encode = ((g_enc_seq - g_tx_seq) < 2U);
    if (!can_encode)
    {
      /* Two frames behind: yield at base priority so the consumer drains;
       * never wait at the raised one. */
      ArmFollowsEncode = 0U;
      vTaskPrioritySet(NULL, CAPTURE_BASE_PRIORITY);
      vTaskDelay(pdMS_TO_TICKS(1U));
      continue;
    }

    /* Raised across the re-arm so no time slice lands in the gap between
     * frames. */
    vTaskPrioritySet(NULL, CAPTURE_REARM_PRIORITY);
    slot   = g_enc_seq & 1U;
    result = capture_one(slot, &bytes);

    if (result == CAPTURE_WAIT)
    {
      switch (capture_wait_for_gap())
      {
        case CAPTURE_GAP_OPEN:
          break;
        case CAPTURE_GAP_MISSED:
          ArmAnyway = 1U;
          break;
        default:
          /* No frame ended: the camera stopped mid-frame. Restart it. */
          capture_resync();
          break;
      }
      continue;
    }

    if ((result == CAPTURE_READY) && (bytes > 0U) && (bytes <= JPEG_BUF_BYTES))
    {
      /* Held raised across the post and into the next re-arm. */
      vTaskPrioritySet(NULL, CAPTURE_REARM_PRIORITY);
      post_encoded_frame(JpegBuf[slot], bytes);
      torn_run = 0U;
    }
    else
    {
      /* Never post a torn frame. Drop the raised priority so the consumer is
       * not starved. */
      vTaskPrioritySet(NULL, CAPTURE_BASE_PRIORITY);
      if (result == CAPTURE_TORN)
      {
        CapTorn++;
        if (++torn_run >= UVC_RESYNC_AFTER_TORN)
        {
          capture_resync();
          torn_run = 0U;
        }
      }
      else
      {
        CapFailed++;
      }
    }
  }
}

/* USB */

/* Mounted, and the host has selected the alternate setting with the endpoint. */
static inline bool uvc_ready_to_tx(void)
{
  return tud_mounted() && tud_video_n_streaming(0, 0);
}

static inline void uvc_tx_reset(void)
{
  uvc_tx_busy = false;
}

void tud_mount_cb(void)
{
  uvc_tx_reset();
}

void tud_umount_cb(void)
{
  uvc_tx_reset();
}

/**
  * @brief  The video class has put the whole frame on the wire.
  */
void tud_video_frame_xfer_complete_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx)
{
  (void)ctl_idx;
  (void)stm_idx;
  uvc_tx_busy = false;
  UsbSent++;
}

/**
  * @brief  Hand a frame to the video class, which sends it from inside the
  *         stack's service call.
  */
static inline bool uvc_tx_start_frame(const uint8_t *pJpeg, uint32_t Bytes)
{
  if (uvc_tx_busy || !uvc_ready_to_tx())
  {
    return false;
  }
  if (!tud_video_n_frame_xfer(0, 0, (void *)(uintptr_t)pJpeg, (size_t)Bytes))
  {
    return false;
  }
  uvc_tx_busy = true;
  return true;
}

/**
  * @brief  Abandon a frame in flight on a link that has gone away.
  */
static inline void uvc_tx_pump(void)
{
  if (!uvc_tx_busy)
  {
    return;
  }
  if (!uvc_ready_to_tx())
  {
    uvc_tx_reset();
  }
}

/**
  * @brief  'b' on the console: return to the bootloader.
  */
static void console(void)
{
  uint8_t ch;

  while (BSP_COM_Read(&ch, 1U) == 1)
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
  * @brief  The USB task: the stack, the video pump, the console and the report.
  */
static void usb_task(void *pArg)
{
  bool     s_tx_counts    = false;
  uint32_t s_jam_since_ms = 0U;
  uint32_t booted         = HAL_GetTick();
  uint32_t report         = booted;
  uint8_t  seen_a_host    = 0U;
  bool     lit            = false;
  uint32_t enc0 = 0U, sent0 = 0U, torn0 = 0U, fail0 = 0U, resync0 = 0U;

  (void)pArg;

  /* The USB interrupt wakes this task when it queues an event. */
  BSP_COM_SetPumpTask(xTaskGetCurrentTaskHandle());

  for (;;)
  {
    frame_msg_t msg;

    (void)BSP_WDG_Refresh();
    tud_task_ext(0, false);   /* non-blocking drain; bare tud_task() blocks under OPT_OS_FREERTOS */
    uvc_tx_pump();

    /* Release the slot once its transfer is no longer active. State-based: a
     * bus reset clears the busy flag inside tud_task's callbacks, and an edge
     * sampled afterwards would leak the slot. */
    if (s_tx_counts && !uvc_tx_busy)
    {
      g_tx_seq++;
      s_tx_counts = false;
    }

    /* Outstanding slots with the link idle and nothing queued can only be
     * lost accounting: reclaim them, or the camera stays stopped. */
    {
      const bool jammed = !uvc_tx_busy && (g_enc_seq != g_tx_seq) &&
                          (uxQueueMessagesWaiting(g_video_usb_queue) == 0U);
      if (!jammed)
      {
        s_jam_since_ms = 0U;
      }
      else if (s_jam_since_ms == 0U)
      {
        s_jam_since_ms = HAL_GetTick() | 1U;
      }
      else if ((HAL_GetTick() - s_jam_since_ms) > USB_JAM_MS)
      {
        g_tx_seq       = g_enc_seq;
        s_tx_counts    = false;
        s_jam_since_ms = 0U;
      }
    }

    console();

    if (uvc_ready_to_tx() != lit)
    {
      lit = uvc_ready_to_tx();
      (void)(lit ? BSP_LED_On(LED_GREEN) : BSP_LED_Off(LED_GREEN));
    }

    if ((UVC_HOST_TIMEOUT_MS != 0U) && (seen_a_host == 0U))
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

    if ((HAL_GetTick() - report) >= 1000U)
    {
      uint32_t enc = CapEncoded, sent = UsbSent, torn = CapTorn;
      uint32_t fail = CapFailed, resync = CapResync;

      report = HAL_GetTick();
      print("[uvc] enc=%lu sent=%lu torn=%lu fail=%lu resync=%lu gap=%lu/%lu last=%luB"
            " streaming=%u\r\n",
            (unsigned long)(enc - enc0), (unsigned long)(sent - sent0),
            (unsigned long)(torn - torn0), (unsigned long)(fail - fail0),
            (unsigned long)(resync - resync0), (unsigned long)CapGap,
            (unsigned long)(StreamBand.Depth * 2U), (unsigned long)CapBytes,
            (unsigned)(lit ? 1U : 0U));
      enc0 = enc; sent0 = sent; torn0 = torn; fail0 = fail; resync0 = resync;
    }

    /* Take the next frame when the wire is free. It owns its slot from here,
     * accepted or not; the release above returns it. */
    if (!uvc_tx_busy && (xQueueReceive(g_video_usb_queue, &msg, 0) == pdTRUE))
    {
      (void)uvc_tx_start_frame(msg.jpeg_ptr, msg.jpeg_len);
      s_tx_counts = true;
    }

    if (uvc_tx_busy)
    {
      /* In flight: sleep until the USB interrupt reports progress, at most a
       * tick. */
      (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1U));
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(1U));   /* idle at 1 kHz */
    }
  }
}

/* Scaffolding */

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
  uint32_t start;

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

  /* USB first, then the boot-time enumeration pump. Blocking stack calls are
   * safe this early only because the OSAL clamps their waits to zero until
   * the scheduler runs. */
  if (BSP_COM_Init() != BSP_ERROR_NONE)
  {
    (void)BSP_LED_On(LED_RED);
    Error_Handler();
  }
  start = HAL_GetTick();
  while (!tud_mounted() && ((HAL_GetTick() - start) < USB_BOOT_PUMP_MS))
  {
    tud_task_ext(0, false);   /* non-blocking drain; bare tud_task() blocks under OPT_OS_FREERTOS */
    (void)BSP_WDG_Refresh();
    HAL_Delay(10U);
  }

  /* Then the camera and the encoder. */
  InitStatus = video_init();
  if (InitStatus != BSP_ERROR_NONE)
  {
    /* No video, but a console and a way back to the bootloader. */
    (void)BSP_LED_On(LED_RED);
  }

  /* A depth-2 frame queue, matching the ping-pong. */
  g_video_usb_queue = xQueueCreate(2U, sizeof(frame_msg_t));
  if ((g_video_usb_queue == NULL) ||
      (xTaskCreate(capture_task, "capture", CAPTURE_TASK_STACK_WORDS, NULL,
                   CAPTURE_BASE_PRIORITY, &CaptureTask) != pdPASS) ||
      (xTaskCreate(usb_task, "usb", USB_TASK_STACK_WORDS, NULL,
                   USB_TASK_PRIORITY, NULL) != pdPASS))
  {
    (void)BSP_LED_On(LED_RED);
    Error_Handler();
  }

  vTaskStartScheduler();
  Error_Handler();
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
