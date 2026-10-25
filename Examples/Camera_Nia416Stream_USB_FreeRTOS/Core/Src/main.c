/*
  Camera_Nia416Stream_USB_FreeRTOS

  Streams the camera to Neuro Studio over USB, XGA at thirty frames a second,
  with a labelled box around every object Ohm Lab's NIA Triad-416 detector
  finds among the 80 COCO classes. The detector runs on the Neural-ART NPU on
  a centred square of the picture; Neuro Studio still receives the whole
  frame, and the boxes are mapped back onto it.

  The circuit:
  - Uses the on-board camera module (OV5640) on the
    30-pin connector. No wiring needed.
  - The detector's weights must already be in external flash at 0x90300000,
    where harness/n6weights.py writes them. See README.md.

  Send 'b' on the USB console at any time to return to the bootloader.

  Copyright (c) 2026 Ohm Lab Ltd.
  SPDX-License-Identifier: CC0-1.0
  Released under CC0 1.0 Universal: copy it into your own project
  freely, with or without attribution.

  The NIA Triad-416 model this example embeds is not CC0: it is Ohm Lab's,
  under the Ohm Lab Model Licence
  (Middlewares/NeuroN6/Models/Nia416/LICENSE-OhmLab-Model.txt). A firmware
  image built from this example contains the model and may be run only on
  Ohm Lab hardware.
*/

#include "main.h"

#include "neuron6.h"
#include "neuron6_bus.h"
#include "neuron6_camera.h"
#include "neuron6_com.h"
#include "neuron6_npu.h"
#include "neuron6_rtos.h"
#include "neuron6_venc.h"
#include "neuron6_wdg.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "tusb.h"

#include "ll_aton_rt_user_api.h"
#include "npu_cache.h"

#include "PostProcessor_NIA416.h"
#include "app_postprocess_nia416_ui.h"
#include "LCD_Overlay_Nia.h"
#include "vnd_meta.h"
#include "n6_host_contract.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#if !defined(BSP_USE_FREERTOS) || (BSP_USE_FREERTOS != 1)
#error "This example runs under FreeRTOS: set BSP_USE_FREERTOS to 1 in neuron6_conf.h."
#endif

/* Tuning */

#define NIA_CONF                0.36f   /* the model's live default */
#define NIA_IOU                 0.70f   /* the model's NMS setting  */
#define HEARTBEAT_MS            1000U

/* XGA, at the camera mode's native 30 fps. */
#define CAM_RESOLUTION          BSP_CAMERA_R1024x768
#define CAM_FPS                 30U

/* The detector's input. */
#define NN_WIDTH                416U
#define NN_HEIGHT               416U

/* The band the camera writes and the encoder reads: XGA, four macroblock
 * rows deep. */
#define BAND_BYTES              (256U * 1024U)

/* One finished JPEG: a quarter of a byte a pixel, with a 512 KB floor. */
#define JPEG_BUF_BYTES          (512U * 1024U)

#define WDG_TIMEOUT_MS          2000U
#define WDG_WARN_MS             300U

/* How long USB is pumped to enumerate before the scheduler starts. */
#define USB_BOOT_PUMP_MS        500U

/* With no host by this time, go back to the bootloader; once a host has been
 * seen this never applies. 0 runs regardless. */
#ifndef NIA_HOST_TIMEOUT_MS
#define NIA_HOST_TIMEOUT_MS     30000U
#endif

/* A run of failed encodes, or torn frames in a row: camera and encoder have
 * slipped, so resync them. */
#define RESYNC_AFTER_FAILS      8U
#define RESYNC_AFTER_TORN       2U

/* How long each part may go without progress before the USB task stops
 * feeding the watchdog, whose early warning then takes the board to its
 * bootloader. */
#define SETUP_LIMIT_MS          60000U  /* setup() */
#define APP_STALL_MS            10000U  /* a pass of loop() */
#define NN_STALL_MS             10000U  /* an inference run to the end */

/* How long one pass waits for an NN snapshot. A late one is not cancelled
 * (stopping the pipe can fail and still deliver it later): the next pass
 * waits on for it. */
#define NN_SNAPSHOT_WAIT_MS     1000U

/* With no inference finished for this long, the boxes are taken off the
 * stream, so the host does not draw stale ones over live video. */
#define NN_STALE_MS             2000U

/* Outstanding frames with the link idle and nothing queued for this long can
 * only be lost accounting. */
#define USB_JAM_MS              1000U

/* The application task sits below capture and USB, so they run while the NPU
 * is busy; capture is raised one step across a re-arm. */
#define APP_TASK_STACK_WORDS      ((16U * 1024U) / sizeof(StackType_t))
#define APP_TASK_PRIORITY         (tskIDLE_PRIORITY + 2U)
#define USB_TASK_STACK_WORDS      (8192U / sizeof(StackType_t))
#define USB_TASK_PRIORITY         (tskIDLE_PRIORITY + 3U)
#define CAPTURE_TASK_STACK_WORDS  (8192U / sizeof(StackType_t))
#define CAPTURE_BASE_PRIORITY     (tskIDLE_PRIORITY + 3U)
#define CAPTURE_REARM_PRIORITY    (tskIDLE_PRIORITY + 4U)

/* The capture controls Neuro Studio shows: exposure, gain and white balance. */
#define CAM_CAPS                0x0FU
#define META_CAM_CAPS_TAG       0x64U
#define CAM_CAPS_ANNOUNCE_MS    1000U

/* 'CAM0' packet fields (NeuroN6_app.h): the auto states it asks for, and
 * which of its fields apply. */
#define N6_CAM0_FLAG_AUTO_EXPOSURE   0x01U
#define N6_CAM0_FLAG_AUTO_GAIN       0x02U
#define N6_CAM0_FLAG_AUTO_AWB        0x04U
#define N6_CAM0_APPLY_AUTO_EXPOSURE  0x01U
#define N6_CAM0_APPLY_AUTO_GAIN      0x02U
#define N6_CAM0_APPLY_AUTO_AWB       0x04U
#define N6_CAM0_APPLY_EXPOSURE_US    0x08U
#define N6_CAM0_APPLY_GAIN           0x10U

#define VND_MAGIC               0x30564E44U   /* 'DNV0' on the wire */

#define STROBE_DISABLED         0
#define STROBE_ENABLED          1

/* The camera's strobe LED. STROBE_ENABLED lights it for every exposure. */
#define CAM_STROBE              STROBE_DISABLED

/* COCO-80 class names, from the post-processing library. */
#define NB_CLASSES              80
extern const char *classes_table[NB_CLASSES];

/* Types */

typedef struct __attribute__((packed))
{
  uint32_t magic;
  uint32_t frame_id;
  uint32_t meta_len;
  uint32_t payload_len;
  uint32_t flags;
} vnd_frame_hdr_t;

/* A finished frame and its metadata, posted by the capture task to the USB
 * task. */
typedef struct
{
  uint8_t       *jpeg_ptr;
  uint32_t       jpeg_len;
  const uint8_t *meta_ptr;
  uint32_t       meta_len;
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

/* The model */

LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(nia416);

/* Read by the post-processing: the inference timestamps, and the crop window
 * that maps a detection onto the camera frame (non-zero at boot, so the
 * mapping never divides by zero). */
uint32_t ts[2]           = { 0 };
uint16_t g_nn_crop_x     = 0;
uint16_t g_nn_crop_y     = 0;
uint16_t g_nn_crop_w     = 800;
uint16_t g_nn_crop_h     = 480;
uint16_t g_camera_full_w = 800;
uint16_t g_camera_full_h = 480;

/* The NN pipe's snapshot handshake. */
static volatile int32_t cameraFrameReceived = 0;
static uint8_t          NnSnapshotArmed;   /* a snapshot is on its way into the input */

/* The ATON error-interrupt recovery state the runtime keeps. */
extern volatile uint32_t g_aton_err_irq_count, g_aton_err_irqs_last, g_aton_err_ec_irq,
                         g_aton_err_ec_bc, g_aton_err_no_owner, g_aton_err_abort, g_aton_err_burst;

/* Memory */

/* The camera writes here, wrapped, and the encoder follows it. */
static uint8_t Band[BAND_BYTES]
  __attribute__((section(".axisram_nc"), aligned(32)));

/* Two finished frames: one on the wire while the next is encoded. */
static uint8_t JpegBuf[2][JPEG_BUF_BYTES]
  __attribute__((section(".ext_psram"), aligned(32)));

/* State */

/* The vendor transmitter. */
static volatile bool     vnd_tx_active = false;
static vnd_frame_hdr_t   vnd_hdr;
static uint32_t          vnd_off;
static uint32_t          vnd_total;
static const uint8_t    *vnd_meta;
static uint32_t          vnd_meta_len;
static const uint8_t    *vnd_payload;
static uint32_t          vnd_payload_len;
static uint32_t          vnd_frame_id;

/* Ping-pong: frame N is encoded into JpegBuf[N & 1] while the host takes the
 * other. g_enc_seq advances when capture posts a frame, g_tx_seq when USB has
 * finished with one. */
static QueueHandle_t     g_video_usb_queue;
static volatile uint32_t g_enc_seq;
static volatile uint32_t g_tx_seq;

/* Owned by the capture task. */
static BSP_VENC_Band_t   StreamBand;
static uint8_t           LinebufArmed;
static uint32_t          ConsecFail;
/* Line events since the camera's last frame event: zero between its frames. */
static volatile uint32_t FrameLines;
/* Set from the camera's frame start to its frame end. */
static volatile uint32_t FrameActive;
/* The last encode coded a frame, so the next arm follows it straight away.
 * Cleared when frames are held back for the host, or an encode fails. */
static uint8_t           ArmFollowsEncode;
/* Main-pipe frame events: tell a stopped camera from gaps capture missed. */
static volatile uint32_t FrameEnds;
/* Set while the capture task waits for a frame to end; the frame event wakes it. */
static volatile uint8_t  CaptureWaiting;
static TaskHandle_t      CaptureTask;
/* Arm the next capture where the camera is: the wait saw frames end but
 * caught no gap between them. */
static uint8_t           ArmAnyway;
/* How long a capture waits for a frame to end before taking the camera for
 * stopped: five frame periods (night mode stretches frames to a quarter of
 * the rate), never under 150 ms; set when the camera reports its rate. */
static uint32_t          CaptureGapWaitMs = 1000U;

/* The pipeline as setup left it. */
static volatile uint8_t  CameraOk;
static IPostProcessor   *BoundPP;

/* The weights live in external flash, written once with harness/n6weights.py
 * and read in place by the NPU. Anything else written there gives nonsense
 * detections with no error, so vision_begin() first checks the region against
 * the CRC32 in nia416_model.c; on a mismatch the network never runs and the
 * console says how to put the weights back. */
extern const uint32_t    nia416_weights_address;
extern const uint32_t    nia416_weights_size;
extern const uint32_t    nia416_weights_crc32;
static uint8_t           NnWeightsOk;
static uint8_t           NnInputOk;       /* the NN frame fits the network's input */
static uint32_t          NnWeightsCrc;

/* Counters for the report, each written by one task and read by another. */
static volatile uint32_t CapEncoded, CapTorn, CapFailed, CapResync, CapBytes;
static volatile uint32_t UsbFrames, NnRuns, NnLastMs, NnSnapMisses;

/* The application task's progress, in HAL ticks, for the USB task, which
 * feeds the watchdog only while it goes on (app_progressing()). */
static volatile uint8_t  AppReady;        /* setup() has finished */
static volatile uint32_t AppLoopMs;       /* loop() went round, or setup() began */
static volatile uint32_t NnDoneMs;        /* an inference ran to the end */

static char              BootLine[96];

void Error_Handler(void);

static void    mpu_config(void);
static void    usb_task(void *pArg);

/* Formatted output to the USB console. Dropped, not blocked, with no host. */
#define print(...)   (void)BSP_COM_Printf(__VA_ARGS__)

/* The sensor */

/* OV5640 extras, over the BSP's sensor bus. */
#define OV5640_REG_STROBE_CTRL      0x3B00U   /* bit7 on, bit6 polarity, bits[1:0] mode */
#define OV5640_REG_PAD_OUT_EN00     0x3016U   /* bit1 = STROBE pad output enable */
#define OV5640_REG_CLOCK_ENABLE00   0x3004U   /* bit3 = STROBE peripheral clock */
#define OV5640_REG_PAD_SELECT00     0x301CU   /* bit1 = STROBE pad data-path select */
#define OV5640_REG_FREX_MODE        0x3B07U   /* bit1 = rolling-strobe enable */
#define OV5640_REG_AEC_MANUAL       0x3503U   /* bit0 manual gain, bit1 manual exposure */
#define OV5640_REG_AEC_EXP_19_16    0x3500U
#define OV5640_REG_AEC_EXP_HIGH     0x3501U
#define OV5640_REG_AEC_EXP_LOW      0x3502U
#define OV5640_REG_AEC_GAIN_9_8     0x350AU
#define OV5640_REG_AEC_GAIN_LOW     0x350BU
#define OV5640_REG_AWB_MANUAL       0x3406U   /* 0 = auto, 1 = manual */
#define OV5640_REG_TIMING_VTS_HIGH  0x380EU
#define OV5640_REG_TIMING_VTS_LOW   0x380FU
#define OV5640_AEC_MANUAL_GAIN      0x01U
#define OV5640_AEC_MANUAL_EXPOSURE  0x02U


static void ov5640_write(uint16_t Reg, uint8_t Value)
{
  (void)BSP_I2C1_WriteReg16(BSP_CAMERA_I2C_ADDRESS, Reg, &Value, 1U);
}

static uint8_t ov5640_read(uint16_t Reg)
{
  uint8_t value = 0U;

  (void)BSP_I2C1_ReadReg16(BSP_CAMERA_I2C_ADDRESS, Reg, &value, 1U);
  return value;
}

/**
  * @brief  Drive the OV5640's strobe output.
  */
static int ov5640_set_strobe(int Mode)
{
  if (Mode == STROBE_DISABLED)
  {
    ov5640_write(OV5640_REG_STROBE_CTRL, 0x00);
    return 0;
  }

  /* Clock the strobe block, select the normal pad path, enable the rolling
   * strobe, make the pad an output, then request it on. */
  ov5640_write(OV5640_REG_CLOCK_ENABLE00,
               (uint8_t)(ov5640_read(OV5640_REG_CLOCK_ENABLE00) | 0x08U));
  ov5640_write(OV5640_REG_PAD_SELECT00,
               (uint8_t)(ov5640_read(OV5640_REG_PAD_SELECT00) & (uint8_t)~0x02U));
  ov5640_write(OV5640_REG_FREX_MODE,
               (uint8_t)(ov5640_read(OV5640_REG_FREX_MODE) | 0x02U));
  ov5640_write(OV5640_REG_PAD_OUT_EN00,
               (uint8_t)(ov5640_read(OV5640_REG_PAD_OUT_EN00) | 0x02U));
  ov5640_write(OV5640_REG_STROBE_CTRL, 0x83);   /* on, LED3 mode */
  return 0;
}

/* OV5640 manual controls. */
static void ov5640_set_auto_exposure(bool Enable)
{
  uint8_t m = ov5640_read(OV5640_REG_AEC_MANUAL);

  if (Enable) { m &= (uint8_t)~OV5640_AEC_MANUAL_EXPOSURE; }
  else        { m |= (uint8_t)OV5640_AEC_MANUAL_EXPOSURE;  }
  ov5640_write(OV5640_REG_AEC_MANUAL, m);
}

static void ov5640_set_auto_gain(bool Enable)
{
  uint8_t m = ov5640_read(OV5640_REG_AEC_MANUAL);

  if (Enable) { m &= (uint8_t)~OV5640_AEC_MANUAL_GAIN; }
  else        { m |= (uint8_t)OV5640_AEC_MANUAL_GAIN;  }
  ov5640_write(OV5640_REG_AEC_MANUAL, m);
}

static void ov5640_set_auto_white_balance(bool Enable)
{
  ov5640_write(OV5640_REG_AWB_MANUAL, Enable ? 0x00U : 0x01U);
}

static void ov5640_set_exposure_us(uint32_t Us)
{
  const uint32_t vts = ((uint32_t)ov5640_read(OV5640_REG_TIMING_VTS_HIGH) << 8) |
                        (uint32_t)ov5640_read(OV5640_REG_TIMING_VTS_LOW);
  const uint32_t frame_us = 1000000U / CAM_FPS;
  const uint32_t max_lines = (vts > 4U) ? (vts - 4U) : 1U;
  uint32_t lines;
  uint32_t v;

  if (vts == 0U)
  {
    return;
  }

  lines = (uint32_t)(((uint64_t)Us * (uint64_t)vts) / (uint64_t)frame_us);
  if (lines > max_lines) { lines = max_lines; }
  if (lines < 1U)        { lines = 1U; }

  v = lines << 4;                    /* 1/16 line units */
  if (v > 0xFFFFFU) { v = 0xFFFFFU; }

  ov5640_set_auto_exposure(false);
  ov5640_write(OV5640_REG_AEC_EXP_19_16, (uint8_t)((v >> 16) & 0x0FU));
  ov5640_write(OV5640_REG_AEC_EXP_HIGH,  (uint8_t)((v >>  8) & 0xFFU));
  ov5640_write(OV5640_REG_AEC_EXP_LOW,   (uint8_t)( v        & 0xFFU));
}

static void ov5640_set_gain(float Gain)
{
  uint32_t v;

  if (Gain < 1.0f)  { Gain = 1.0f; }
  if (Gain > 63.9f) { Gain = 63.9f; }
  v = (uint32_t)((Gain * 16.0f) + 0.5f);

  ov5640_set_auto_gain(false);
  ov5640_write(OV5640_REG_AEC_GAIN_9_8, (uint8_t)((v >> 8) & 0x03U));
  ov5640_write(OV5640_REG_AEC_GAIN_LOW, (uint8_t)( v       & 0xFFU));
}

/**
  * @brief  Apply a CAM0 packet, in order: the auto flags first, so a
  *         packet that turns auto off and supplies a value keeps the value.
  */
static void cam0_apply(uint8_t Flags, uint32_t ExposureUs, uint16_t GainX256, uint8_t ApplyMask)
{
  if ((ApplyMask & N6_CAM0_APPLY_AUTO_EXPOSURE) != 0U)
  {
    ov5640_set_auto_exposure((Flags & N6_CAM0_FLAG_AUTO_EXPOSURE) != 0U);
  }
  if ((ApplyMask & N6_CAM0_APPLY_AUTO_GAIN) != 0U)
  {
    ov5640_set_auto_gain((Flags & N6_CAM0_FLAG_AUTO_GAIN) != 0U);
  }
  if ((ApplyMask & N6_CAM0_APPLY_AUTO_AWB) != 0U)
  {
    ov5640_set_auto_white_balance((Flags & N6_CAM0_FLAG_AUTO_AWB) != 0U);
  }
  if ((ApplyMask & N6_CAM0_APPLY_EXPOSURE_US) != 0U)
  {
    ov5640_set_exposure_us(ExposureUs);
  }
  if ((ApplyMask & N6_CAM0_APPLY_GAIN) != 0U)
  {
    ov5640_set_gain((float)GainX256 / 256.0f);
  }
}

/* Capture */

/**
  * @brief  Start the camera. The encoder calls this once per stream, after it
  *         has parked waiting for the camera's first line event.
  */
static int32_t camera_start(void *pArg)
{
  (void)pArg;
  return (BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, Band, BSP_CAMERA_MODE_CONTINUOUS)
          == BSP_ERROR_NONE) ? 0 : -1;
}

/**
  * @brief  The main pipe's line events pace the encoder; the auxiliary
  *         pipe's frame event completes an NN snapshot.
  */
void BSP_CAMERA_LineEventCallback(uint32_t Instance, uint32_t Pipe)
{
  if ((Instance == 0U) && (Pipe == BSP_CAMERA_PIPE_MAIN))
  {
    FrameLines++;
    (void)BSP_VENC_SourceLineEvent(0);
  }
}

void BSP_CAMERA_FrameEventCallback(uint32_t Instance, uint32_t Pipe)
{
  if (Instance != 0U)
  {
    return;
  }
  if (Pipe == BSP_CAMERA_PIPE_MAIN)
  {
    /* Anchor the producer counter to the camera's frame. */
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
  if (Pipe == BSP_CAMERA_PIPE_AUX)
  {
    cameraFrameReceived = 1;
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
  * @brief  One frame into JpegBuf[Slot], with the one-time arm in front of it
  *         and the failure self-heal behind it.
  */
static capture_result_t capture_one(uint32_t Slot, uint32_t *pBytes)
{
  const uint8_t follow_on = ArmFollowsEncode;

  ArmFollowsEncode = 0U;

  /* The first capture, and the first after a resync: stop the free-running
   * pipe and arm the line event the encoder paces itself on. The encode then
   * starts the pipe again, after the encoder is parked. */
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
    /* Mid-frame: arming now would code the rest of this frame and the top of
     * the next as one picture, so wait for the blanking. A follow-on arm
     * waits only for lines already arrived (also waiting out the frame start
     * skips every other frame at QSXGA); any other arm also avoids the frame
     * start, or it misses the first line event and runs a band behind. */
    return CAPTURE_WAIT;
  }
  ArmAnyway = 0U;

  if (BSP_VENC_EncodeBand(0, JpegBuf[Slot], JPEG_BUF_BYTES, pBytes) != BSP_ERROR_NONE)
  {
    if (++ConsecFail >= RESYNC_AFTER_FAILS)
    {
      capture_resync();
    }
    return CAPTURE_FAILED;
  }
  ConsecFail = 0U;
  ArmFollowsEncode = 1U;

  return (BSP_VENC_LastFrameTorn(0) != 0U) ? CAPTURE_TORN : CAPTURE_READY;
}

/**
  * @brief  Queue a frame with the metadata buffer of its parity; the ping-pong
  *         advances only if the queue took them.
  */
static void post_encoded_frame(uint8_t *pBuf, uint32_t Bytes)
{
  frame_msg_t msg = {0};

  msg.jpeg_ptr = pBuf;
  msg.jpeg_len = Bytes;
  (void)vnd_meta_get_pingpong(g_enc_seq & 1U, &msg.meta_ptr, &msg.meta_len);

  if (xQueueSend(g_video_usb_queue, &msg, 0) == pdTRUE)
  {
    g_enc_seq++;
    CapEncoded++;
    CapBytes = Bytes;
  }
}

/**
  * @brief  Wait, at most CaptureGapWaitMs, for the camera to end its frame;
  *         the frame event wakes the task so it arms at the start of the gap.
  * @retval CAPTURE_GAP_OPEN    between frames: arm now
  * @retval CAPTURE_GAP_MISSED  frames ended but every gap closed first: arm
  *                            anyway; the torn-frame check drops the result
  * @retval CAPTURE_GAP_STOPPED no frame ended: the camera has stopped
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
     * meanwhile is seen here, and a later one wakes the task. */
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

    if ((g_enc_seq - g_tx_seq) >= 2U)
    {
      /* Two frames behind: yield at base priority so USB drains; never wait
       * at the raised one. */
      ArmFollowsEncode = 0U;
      vTaskPrioritySet(NULL, CAPTURE_BASE_PRIORITY);
      vTaskDelay(pdMS_TO_TICKS(1U));
      continue;
    }

    /* Raised across the re-arm, so no time slice lands in the gap between
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
          /* No frame ended: the camera stopped mid-frame; restart it. */
          capture_resync();
          break;
      }
      continue;
    }

    if ((result == CAPTURE_READY) && (bytes > 0U) && (bytes <= JPEG_BUF_BYTES))
    {
      vTaskPrioritySet(NULL, CAPTURE_REARM_PRIORITY);
      post_encoded_frame(JpegBuf[slot], bytes);
      torn_run = 0U;
    }
    else
    {
      /* A torn frame is never posted: a split JPEG must not reach the host. */
      vTaskPrioritySet(NULL, CAPTURE_BASE_PRIORITY);
      if (result == CAPTURE_TORN)
      {
        CapTorn++;
        if (++torn_run >= RESYNC_AFTER_TORN)
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

/* NPU */

/**
  * @brief  Bring the NPU up: the BSP's part (isolation, RAM banks, NPU cache,
  *         interrupts), then the runtime's cache handle, then the runtime.
  */
static void npu_init(void)
{
  if (BSP_NPU_Init() != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  npu_cache_init();
  npu_cache_enable();
  LL_ATON_RT_RuntimeInit();
}

/**
  * @brief  Run one inference, recovering from an NPU error interrupt.
  * @retval 1 when the network ran to the end, 0 when an error interrupt cut it
  *         short and its outputs are not to be read
  */
static uint8_t run_inference(NN_Instance_TypeDef *pInstance)
{
  LL_ATON_RT_RetValues_t ret = LL_ATON_RT_DONE;

  LL_ATON_RT_Init_Network(pInstance);
  do
  {
    if (g_aton_err_abort)
    {
      break;
    }
    ret = LL_ATON_RT_RunEpochBlock(pInstance);

    if (ret == LL_ATON_RT_WFE)
    {
      /* Blocks this task until the epoch interrupt wakes it. */
      LL_ATON_OSAL_WFE();
    }
  } while (ret != LL_ATON_RT_DONE);

  if (g_aton_err_abort)
  {
    print("[NPU] ATON error IRQ recovered: total=%lu INTREG=0x%08lX EC_IRQ=0x%08lX "
          "EC_BC=0x%08lX no_owner=%lu - frame dropped, ATON runtime recycled\r\n",
          (unsigned long)g_aton_err_irq_count, (unsigned long)g_aton_err_irqs_last,
          (unsigned long)g_aton_err_ec_irq, (unsigned long)g_aton_err_ec_bc,
          (unsigned long)g_aton_err_no_owner);
    LL_ATON_RT_DeInit_Network(pInstance);
    LL_ATON_RT_RuntimeDeInit();
    g_aton_err_burst = 0U;
    g_aton_err_abort = 0U;
    LL_ATON_RT_RuntimeInit();
    return 0U;
  }

  LL_ATON_RT_Reset_Network(pInstance);
  LL_ATON_RT_DeInit_Network(pInstance);
  return 1U;
}

/**
  * @brief  Whether a Width x Height RGB888 snapshot fits the network's input,
  *         which it is written straight into (a smaller one is reported).
  * @retval 1 when the frame fits, 0 when it does not
  */
static uint8_t nn_input_check(NN_Instance_TypeDef *pInstance, uint32_t Width, uint32_t Height)
{
  const LL_Buffer_InfoTypeDef *in    = LL_ATON_Input_Buffers_Info(pInstance);
  const uint32_t               frame = Width * Height * 3U;
  const uint32_t               room  = LL_Buffer_len(&in[0]);

  if (frame > room)
  {
    print("[VISION] the NN frame, %lux%lu RGB888, is %lu bytes and the network's input %lu: "
          "the network is not run\r\n", (unsigned long)Width, (unsigned long)Height,
          (unsigned long)frame, (unsigned long)room);
    return 0U;
  }
  if (frame < room)
  {
    print("[VISION WARN] the NN frame, %lux%lu RGB888, fills %lu of the network's %lu input "
          "bytes\r\n", (unsigned long)Width, (unsigned long)Height, (unsigned long)frame,
          (unsigned long)room);
  }
  return 1U;
}

/**
  * @brief  Start one snapshot of the NN pipe straight into the model's input.
  * @retval 1 when the snapshot was started, 0 when the camera refused it
  */
static uint8_t nn_feed_start(NN_Instance_TypeDef *pInstance)
{
  const LL_Buffer_InfoTypeDef *nn_in_info = LL_ATON_Input_Buffers_Info(pInstance);
  uint8_t *nn_in = (uint8_t *)LL_Buffer_addr_start(&nn_in_info[0]);

  cameraFrameReceived = 0;
  if (BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_AUX, nn_in, BSP_CAMERA_MODE_SNAPSHOT) != BSP_ERROR_NONE)
  {
    /* Refused: counted, and tried again next pass. Refused for good, it
     * starves the network and the watchdog stops being fed. */
    NnSnapMisses++;
    vTaskDelay(pdMS_TO_TICKS(10U));
    return 0U;
  }
  NnSnapshotArmed = 1U;
  return 1U;
}

/**
  * @brief  Wait up to NN_SNAPSHOT_WAIT_MS for the snapshot, then run and time
  *         the network; a late snapshot stays armed for the next pass.
  * @retval 1 when the outputs are the network's, 0 when there are none
  */
static uint8_t npu_run_inference(NN_Instance_TypeDef *pInstance)
{
  const uint32_t t0 = HAL_GetTick();
  uint8_t        done;

  while (cameraFrameReceived == 0)
  {
    if ((HAL_GetTick() - t0) > NN_SNAPSHOT_WAIT_MS)
    {
      NnSnapMisses++;
      return 0U;
    }
    vTaskDelay(pdMS_TO_TICKS(1U));
  }
  cameraFrameReceived = 0;
  NnSnapshotArmed     = 0U;

  ts[0] = HAL_GetTick();
  done  = run_inference(pInstance);
  ts[1] = HAL_GetTick();
  NnLastMs = ts[1] - ts[0];   /* for the report, as ts[] may be mid-update */
  NnRuns++;
  if (done != 0U)
  {
    NnDoneMs = ts[1];
  }
  return done;
}

/**
  * @brief  Check the weights region's CRC32 (zlib's) against the network's.
  *         The USB task, at a higher priority, keeps the watchdog fed meanwhile.
  * @retval 1 when the weights are the network's, 0 when they are not
  */
static uint8_t nn_weights_check(void)
{
  static uint32_t table[256];
  const uint8_t  *p   = (const uint8_t *)nia416_weights_address;
  uint32_t        crc = 0xFFFFFFFFU;
  uint32_t        t0  = HAL_GetTick();

  for (uint32_t i = 0U; i < 256U; i++)
  {
    uint32_t c = i;
    for (uint32_t k = 0U; k < 8U; k++)
    {
      c = ((c & 1U) != 0U) ? (0xEDB88320U ^ (c >> 1)) : (c >> 1);
    }
    table[i] = c;
  }
  for (uint32_t i = 0U; i < nia416_weights_size; i++)
  {
    crc = table[(crc ^ p[i]) & 0xFFU] ^ (crc >> 8);
  }
  NnWeightsCrc = ~crc;

  if (NnWeightsCrc == nia416_weights_crc32)
  {
    print("[NN] weights at 0x%08lX: CRC32 0x%08lX over %lu bytes, the network's (%lu ms)\r\n",
          (unsigned long)nia416_weights_address, (unsigned long)NnWeightsCrc,
          (unsigned long)nia416_weights_size, (unsigned long)(HAL_GetTick() - t0));
    return 1U;
  }
  print("[NN] the weights at 0x%08lX are not this network's: CRC32 0x%08lX, where it was "
        "generated with 0x%08lX. The network will not run; the stream carries on without "
        "detections.\r\n", (unsigned long)nia416_weights_address, (unsigned long)NnWeightsCrc,
        (unsigned long)nia416_weights_crc32);
  print("[NN] To write them: python harness/n6weights.py "
        "Middlewares/NeuroN6/Models/Nia416/model.json --write\r\n");
  return 0U;
}

/* Vision */

/**
  * @brief  Resolve the crop for ASPECT_CROP_CENTER: the largest centred
  *         rectangle with the model's aspect. For XGA, 768 x 768 at (128, 0).
  */
static void resolve_crop_center(uint32_t CamW, uint32_t CamH,
                                uint32_t *pX, uint32_t *pY, uint32_t *pW, uint32_t *pH)
{
  uint32_t kw;
  uint32_t kh;

  /* Wider than the network's aspect: the height is the limit. */
  if ((CamW * NN_HEIGHT) > (CamH * NN_WIDTH))
  {
    kh = CamH;
    kw = (CamH * NN_WIDTH) / NN_HEIGHT;
  }
  else
  {
    kw = CamW;
    kh = (CamW * NN_HEIGHT) / NN_WIDTH;
  }
  *pX = (CamW - kw) / 2U;
  *pY = (CamH - kh) / 2U;
  *pW = kw;
  *pH = kh;
}

/**
  * @brief  Bring the camera, the encoder and the NPU up.
  */
static void vision_begin(IPostProcessor *pPP)
{
  BSP_CAMERA_Capabilities_t cap  = {0};
  BSP_CAMERA_PipeConfig_t   pipe = {0};
  BSP_VENC_Config_t         venc = {0};
  uint32_t crop_x = 0U, crop_y = 0U, crop_w = 0U, crop_h = 0U;
  int32_t  status;

  /* 1. The sensor: it comes up streaming, and is suspended while the strobe
   *    is set. */
  CameraOk = 0U;
  status = BSP_CAMERA_Init(0, CAM_RESOLUTION, BSP_CAMERA_PF_RGB565);
  if (status != BSP_ERROR_NONE)
  {
    print("[VISION] camera init failed (%ld). Inference skipped; console stays up.\r\n",
          (long)status);
  }
  else
  {
    CameraOk = 1U;
    (void)BSP_CAMERA_SuspendSensor(0);
    if (CAM_STROBE != STROBE_DISABLED)
    {
      (void)ov5640_set_strobe(CAM_STROBE);
    }
    (void)BSP_CAMERA_ResumeSensor(0);
  }

  if (CameraOk != 0U)
  {
    status = BSP_CAMERA_GetCapabilities(0, &cap);
    if (status == BSP_ERROR_NONE)
    {
      status = BSP_VENC_GetBandGeometry(cap.Width, cap.Height, &StreamBand);
      CaptureGapWaitMs = (cap.MaxFrameRate != 0U) ? (5000U / cap.MaxFrameRate) : 1000U;
      if (CaptureGapWaitMs < 150U) { CaptureGapWaitMs = 150U; }
    }
    if ((status == BSP_ERROR_NONE) && (StreamBand.SizeBytes > sizeof(Band)))
    {
      status = BSP_ERROR_WRONG_PARAM;
    }

    /* The camera frame, and the NN input's centred square within it. */
    g_camera_full_w = (uint16_t)cap.Width;
    g_camera_full_h = (uint16_t)cap.Height;
    resolve_crop_center(cap.Width, cap.Height, &crop_x, &crop_y, &crop_w, &crop_h);

    /* 2-5. The JPEG pipe, its line buffer, the NN pipe, and the prime. */
    if (status == BSP_ERROR_NONE)
    {
      pipe.PixelFormat = BSP_CAMERA_PIPE_PF_YUV422;
      pipe.Width       = cap.Width;
      pipe.Height      = StreamBand.CodedHeight;
      status = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pipe);
    }
    if (status == BSP_ERROR_NONE)
    {
      status = BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN,
                                           StreamBand.WrapLines, BSP_CAMERA_LINES_OFF);
    }
    if (status == BSP_ERROR_NONE)
    {
      memset(&pipe, 0, sizeof(pipe));
      pipe.PixelFormat = BSP_CAMERA_PIPE_PF_RGB888;
      pipe.Width       = NN_WIDTH;
      pipe.Height      = NN_HEIGHT;
      pipe.CropEnable  = 1U;
      pipe.CropX       = crop_x;
      pipe.CropY       = crop_y;
      pipe.CropWidth   = crop_w;
      pipe.CropHeight  = crop_h;
      status = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_AUX, &pipe);
    }
    if (status == BSP_ERROR_NONE)
    {
      g_nn_crop_x = (uint16_t)crop_x;
      g_nn_crop_y = (uint16_t)crop_y;
      g_nn_crop_w = (uint16_t)crop_w;
      g_nn_crop_h = (uint16_t)crop_h;
      /* Prime: run the JPEG pipe, wrapped into the band, until the first
       * capture stops it. */
      status = BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, Band, BSP_CAMERA_MODE_CONTINUOUS);
    }

    if (status == BSP_ERROR_NONE)
    {
      LinebufArmed     = 0U;
      venc.Band        = StreamBand;
      venc.pBand       = Band;
      venc.Quality     = BSP_VENC_QUALITY_DEFAULT;   /* quality level 2 */
      venc.SourceStart = camera_start;
      venc.pSourceArg  = NULL;
      status = BSP_VENC_Init(0, &venc);
    }
    if (status != BSP_ERROR_NONE)
    {
      print("[VISION] camera pipes failed (%ld). Inference skipped; console stays up.\r\n",
            (long)status);
      CameraOk = 0U;
    }
  }

  /* 6. The NPU, camera or not; the weights are checked first. */
  NnWeightsOk = nn_weights_check();
  npu_init();
  NnInputOk = nn_input_check(&NN_Instance_nia416, NN_WIDTH, NN_HEIGHT);

  /* 7. The post-processor. */
  BoundPP = pPP;
  if ((pPP != NULL) && (pPP->init != NULL))
  {
    (void)pPP->init(pPP->ctx, &NN_Instance_nia416);
  }

  /* 8. The capture task; the USB task has run since boot. */
  if (CameraOk != 0U)
  {
    if (xTaskCreate(capture_task, "capture", CAPTURE_TASK_STACK_WORDS, NULL,
                    CAPTURE_BASE_PRIORITY, &CaptureTask) != pdPASS)
    {
      Error_Handler();
    }
  }
}

/**
  * @brief  A pass with no inference: after NN_STALE_MS without one, the boxes
  *         are taken off the stream.
  */
static void nn_missed(void)
{
  if ((HAL_GetTick() - NnDoneMs) > NN_STALE_MS)
  {
    vnd_meta_invalidate_slot(VND_META_SLOT_PRIMARY);
  }
}

/**
  * @brief  Whether the network is meant to run.
  */
static bool nn_expected(void)
{
  return (BoundPP != NULL) && (CameraOk != 0U) && (NnWeightsOk != 0U) && (NnInputOk != 0U);
}

/**
  * @brief  Whether the application task is still making progress: setup()
  *         within SETUP_LIMIT_MS, then each pass of loop() within
  *         APP_STALL_MS and, while the network is meant to run, an inference
  *         within NN_STALL_MS. The stamps are read before the time.
  */
static bool app_progressing(void)
{
  const uint32_t loop_ms = AppLoopMs;
  const uint32_t nn_ms   = NnDoneMs;
  const uint32_t now     = HAL_GetTick();

  if (AppReady == 0U)
  {
    return (now - loop_ms) <= SETUP_LIMIT_MS;
  }
  return ((now - loop_ms) <= APP_STALL_MS) &&
         (!nn_expected() || ((now - nn_ms) <= NN_STALL_MS));
}

/* One snapshot, one inference, the post-processing, and the metadata the
 * next frame carries. */
static void vision_run(void)
{
  /* No camera, wrong weights, or a frame that does not fit the input: the
   * network never runs. */
  if ((BoundPP == NULL) || (CameraOk == 0U) || (NnWeightsOk == 0U) || (NnInputOk == 0U))
  {
    vTaskDelay(pdMS_TO_TICKS(100U));
    return;
  }

  if ((NnSnapshotArmed == 0U) && (nn_feed_start(&NN_Instance_nia416) == 0U))
  {
    nn_missed();
    return;
  }
  if (npu_run_inference(&NN_Instance_nia416) == 0U)
  {
    /* No snapshot yet, or cut short by an NPU error: some outputs share
     * memory with the input, so there is nothing to decode. */
    nn_missed();
    return;
  }

  if (BoundPP->run != NULL)
  {
    (void)BoundPP->run(BoundPP->ctx, &NN_Instance_nia416);
  }
  if (BoundPP->emit_metadata != NULL)
  {
    BoundPP->emit_metadata(BoundPP->ctx);
  }
}

/* Application */

static void setup(void)
{
  vision_begin(PostProcess_NIA416(NIA_CONF, NIA_IOU));

  print("=== n6_nia416_detector : OV5640 XGA 30fps -> USB, NIA Triad-416 COCO-80 ===\r\n");
}

static void loop(void)
{
  static uint32_t last_beat = 0U;
  uint32_t        now;

  /* One capture and inference pass; the post-processor decodes the six NPU
   * tensors and publishes the boxes as USB metadata. */
  vision_run();

  /* Heartbeat: detection count and the top detection. */
  now = HAL_GetTick();
  if ((now - last_beat) >= HEARTBEAT_MS)
  {
    last_beat = now;
    if (NnWeightsOk == 0U)
    {
      print("[hb] no inference: the weights at 0x%08lX are not this network's "
            "(CRC32 0x%08lX, want 0x%08lX)\r\n", (unsigned long)nia416_weights_address,
            (unsigned long)NnWeightsCrc, (unsigned long)nia416_weights_crc32);
    }
    else if (NnInputOk == 0U)
    {
      print("[hb] no inference: the NN frame does not fit the network's input (see the "
            "start-up message)\r\n");
    }
    else if (nia416_pp_output.nb_detect > 0)
    {
      const nia_det_t *d = &nia416_pp_output.pOutBuff[0];

      print("[hb] dets=%d  top=%s %.2f at x=%.2f y=%.2f\r\n",
            (int)nia416_pp_output.nb_detect,
            ((d->class_index >= 0) && (d->class_index < NB_CLASSES))
              ? classes_table[d->class_index] : "?",
            (double)d->conf, (double)d->x_center, (double)d->y_center);
    }
    else
    {
      print("[hb] dets=0\r\n");
    }
  }
}

static void app_task(void *pArg)
{
  (void)pArg;

  AppLoopMs = HAL_GetTick();
  setup();
  NnDoneMs  = HAL_GetTick();
  AppLoopMs = NnDoneMs;
  AppReady  = 1U;
  for (;;)
  {
    loop();
    AppLoopMs = HAL_GetTick();
  }
}

/* USB */

/* Mounted and not suspended. */
static inline bool usb_ready_to_tx(void)
{
  return tud_ready();
}

/**
  * @brief  Back to the bootloader, never with the encoder in flight: stop the
  *         pipes and let the encoder go idle first.
  */
static void leave_to_bootloader(void)
{
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
  (void)BSP_CAMERA_HaltCapture(0);
  (void)BSP_VENC_Quiesce(0);
  BSP_COM_RebootToBootloader();
}

static inline void vnd_tx_reset(void)
{
  vnd_tx_active   = false;
  vnd_off         = 0U;
  vnd_total       = 0U;
  vnd_meta        = NULL;
  vnd_meta_len    = 0U;
  vnd_payload     = NULL;
  vnd_payload_len = 0U;
}

void tud_mount_cb(void)
{
  vnd_tx_reset();
  vnd_frame_id = 0U;
  vnd_meta_invalidate();
  vnd_meta_debug_mark_replay();
}

void tud_umount_cb(void)
{
  vnd_tx_reset();
}

/**
  * @brief  Claim the transmitter: header, metadata and payload
  *         published together under one critical section.
  */
static bool vnd_tx_start_frame(const uint8_t *pMeta, uint32_t MetaLen,
                               const uint8_t *pPayload, uint32_t PayloadLen,
                               uint32_t Flags)
{
  if (vnd_tx_active)
  {
    return false;
  }
  taskENTER_CRITICAL();
  if (vnd_tx_active)
  {
    taskEXIT_CRITICAL();
    return false;
  }
  if ((PayloadLen > 0U) && (pPayload == NULL))
  {
    taskEXIT_CRITICAL();
    return false;
  }
  if ((MetaLen == 0U) && (PayloadLen == 0U))
  {
    taskEXIT_CRITICAL();
    return false;
  }
  if (MetaLen == 0U)
  {
    pMeta = NULL;
  }

  vnd_hdr.magic       = VND_MAGIC;
  vnd_hdr.frame_id    = vnd_frame_id++;
  vnd_hdr.meta_len    = MetaLen;
  vnd_hdr.payload_len = PayloadLen;
  vnd_hdr.flags       = Flags;

  vnd_meta        = pMeta;
  vnd_meta_len    = MetaLen;
  vnd_payload     = pPayload;
  vnd_payload_len = PayloadLen;

  vnd_off       = 0U;
  vnd_total     = sizeof(vnd_hdr) + vnd_meta_len + vnd_payload_len;
  vnd_tx_active = true;
  taskEXIT_CRITICAL();
  return true;
}

/**
  * @brief  Send the header, the metadata, then the payload, as fast as the
  *         endpoint takes them.
  */
static void vnd_tx_pump(void)
{
  uint32_t avail;
  bool     wrote_any = false;

  if (!vnd_tx_active)
  {
    return;
  }
  /* Not ready (suspended or not mounted): a frame not started is dropped; one
   * part way out waits, or a resuming reader would find the next frame's
   * bytes inside this one's length. A mount starts afresh (tud_mount_cb). */
  if (!usb_ready_to_tx())
  {
    if (vnd_off == 0U)
    {
      vnd_tx_reset();
    }
    return;
  }

  avail = tud_vendor_n_write_available(0);
  if (avail == 0U)
  {
    return;
  }

  while (avail != 0U)
  {
    const uint32_t remaining = vnd_total - vnd_off;
    const uint8_t *src;
    uint32_t       chunk_rem;
    uint32_t       n;
    uint32_t       w;

    if (remaining == 0U)
    {
      vnd_tx_active = false;
      (void)tud_vendor_n_write_flush(0);
      return;
    }

    if (vnd_off < sizeof(vnd_hdr))
    {
      src       = ((const uint8_t *)&vnd_hdr) + vnd_off;
      chunk_rem = sizeof(vnd_hdr) - vnd_off;
    }
    else if (vnd_off < (sizeof(vnd_hdr) + vnd_meta_len))
    {
      src       = vnd_meta + (vnd_off - sizeof(vnd_hdr));
      chunk_rem = (sizeof(vnd_hdr) + vnd_meta_len) - vnd_off;
    }
    else
    {
      const uint32_t poff = vnd_off - (sizeof(vnd_hdr) + vnd_meta_len);
      src       = vnd_payload + poff;
      chunk_rem = vnd_payload_len - poff;
    }

    n = (chunk_rem < avail) ? chunk_rem : avail;
    w = tud_vendor_n_write(0, src, n);
    if (w == 0U)
    {
      break;
    }

    wrote_any = true;
    vnd_off  += w;
    avail     = tud_vendor_n_write_available(0);
  }

  if (wrote_any)
  {
    (void)tud_vendor_n_write_flush(0);
  }
}

/**
  * @brief  Host commands, run in the USB task. Only the capture controls and
  *         the reboot act; the rest are taken and dropped.
  */
void tud_vendor_rx_cb(uint8_t idx, const uint8_t *buffer, uint32_t bufsize)
{
  uint8_t  cmd_buf[N6_HOST_CMD_MAX];
  uint32_t count;

  (void)idx;

  if (buffer == NULL)
  {
    count = tud_vendor_n_read(0, cmd_buf, sizeof(cmd_buf));
    /* One command a transfer: the rest is dropped, so the next command
     * starts clean and the FIFO keeps room. */
    tud_vendor_n_read_flush(0);
  }
  else
  {
    count = (bufsize < sizeof(cmd_buf)) ? bufsize : sizeof(cmd_buf);
    memcpy(cmd_buf, buffer, count);
  }

  if (count == 0U)
  {
    return;
  }

  if (count >= 4U)
  {
    const uint32_t magic = (uint32_t)cmd_buf[0] | ((uint32_t)cmd_buf[1] << 8) |
                           ((uint32_t)cmd_buf[2] << 16) | ((uint32_t)cmd_buf[3] << 24);

    if ((magic == N6_CMD_ISP0) && (count >= N6_CMD_ISP0_MIN_BYTES))
    {
      return;   /* the manual ISP is RAW8 only; the OV5640 is not */
    }
    if ((magic == N6_CMD_CAM0) && (count >= sizeof(n6_cmd_cam0_t)))
    {
      /* Applied here; it waits at most one sensor-bus transaction. */
      const uint8_t  flags       = cmd_buf[4];
      const uint32_t exposure_us = (uint32_t)cmd_buf[5]         |
                                   ((uint32_t)cmd_buf[6] << 8)  |
                                   ((uint32_t)cmd_buf[7] << 16) |
                                   ((uint32_t)cmd_buf[8] << 24);
      const uint16_t gain_x256   = (uint16_t)((uint16_t)cmd_buf[9] |
                                              ((uint16_t)cmd_buf[10] << 8));
      const uint8_t  apply_mask  = cmd_buf[11];

      if (CameraOk != 0U)
      {
        cam0_apply(flags, exposure_us, gain_x256, apply_mask);
      }
      return;
    }
    if (((magic == N6_CMD_TUN0) && (count >= sizeof(n6_cmd_tun0_t))) ||
        ((magic == N6_CMD_SUB0) && (count >= sizeof(n6_cmd_sub0_t))) ||
        ((magic == N6_CMD_AST0) && (count >= sizeof(n6_cmd_ast0_t))) ||
        ((magic == N6_CMD_AST1) && (count >= sizeof(n6_cmd_ast1_t))) ||
        ((magic == N6_CMD_AST2) && (count >= sizeof(n6_cmd_ast2_t))))
    {
      return;
    }
  }

  if ((count >= 10U) && (memcmp(cmd_buf, "REBOOT_DFU", 10U) == 0))
  {
    leave_to_bootloader();
  }
}

/**
  * @brief  'b' on the console returns to the bootloader.
  */
static void console(void)
{
  uint8_t ch;

  while (BSP_COM_Read(&ch, 1U) == 1)
  {
    if (ch == (uint8_t)'b')
    {
      leave_to_bootloader();
    }
  }
}

/**
  * @brief  The reset-cause line, for the console and Neuro Studio's
  *         terminal.
  */
static void boot_report_build(uint32_t Rsr)
{
  const char *cause = "unknown";

  if      ((Rsr & RCC_RSR_IWDGRSTF) != 0U) { cause = "WATCHDOG"; }
  else if ((Rsr & RCC_RSR_WWDGRSTF) != 0U) { cause = "window-watchdog"; }
  else if ((Rsr & RCC_RSR_LCKRSTF)  != 0U) { cause = "CPU LOCKUP (double fault)"; }
  else if ((Rsr & RCC_RSR_PORRSTF)  != 0U) { cause = "power-on"; }
  else if ((Rsr & RCC_RSR_BORRSTF)  != 0U) { cause = "brown-out (supply dipped!)"; }
  else if ((Rsr & RCC_RSR_SFTRSTF)  != 0U) { cause = "software reset (reflash/reboot)"; }
  else if ((Rsr & RCC_RSR_PINRSTF)  != 0U) { cause = "reset pin"; }

  (void)snprintf(BootLine, sizeof(BootLine), "[BOOT] reset=%s (wdg=%lus armed)",
                 cause, (unsigned long)(WDG_TIMEOUT_MS / 1000U));
}

/**
  * @brief  The USB task: the stack, the vendor stream, the console and the
  *         watchdog feed.
  */
static void usb_task(void *pArg)
{
  bool     s_tx_counts    = false;
  bool     s_cdc_was_up   = false;
  bool     s_stall_told   = false;
  uint8_t  s_boot_pub     = 0U;
  uint32_t s_tx0_ms       = 0U;
  uint32_t s_jam_since_ms = 0U;
  uint32_t s_caps_ms      = 0U;
  uint32_t booted         = HAL_GetTick();
  uint32_t report         = booted;
  uint8_t  seen_a_host    = 0U;
  uint32_t enc0 = 0U, sent0 = 0U, torn0 = 0U, fail0 = 0U, resync0 = 0U, nn0 = 0U;

  (void)pArg;

  /* The USB interrupt wakes this task when it queues an event. */
  BSP_COM_SetPumpTask(xTaskGetCurrentTaskHandle());

  for (;;)
  {
    frame_msg_t msg;

    /* Fed only while the application task makes progress, so the watchdog
     * catches one that is stuck. */
    if (app_progressing())
    {
      (void)BSP_WDG_Refresh();
    }
    else if (!s_stall_told)
    {
      print("[stall] the application task has stopped (%s): the watchdog takes "
            "the board to the bootloader\r\n", (AppReady == 0U) ? "setup" : "loop or network");
      s_stall_told = true;
    }
    tud_task_ext(0, false);   /* non-blocking drain; bare tud_task() blocks under OPT_OS_FREERTOS */
    vnd_tx_pump();

    /* The boot-cause line to the host terminal: published straight away, and
     * again 1, 6 and 16 s after frames start to flow. */
    if ((s_tx0_ms == 0U) && (g_tx_seq != 0U))
    {
      s_tx0_ms = HAL_GetTick() | 1U;
    }
    {
      const uint32_t tx_ms = (s_tx0_ms != 0U) ? (HAL_GetTick() - s_tx0_ms) : 0U;

      if (s_boot_pub == 0U)
      {
        (void)vnd_meta_publish_debug_text(BootLine);
        s_boot_pub++;
      }
      else if (((s_boot_pub == 1U) && (tx_ms > 1000U)) ||
               ((s_boot_pub == 2U) && (tx_ms > 6000U)) ||
               ((s_boot_pub == 3U) && (tx_ms > 16000U)))
      {
        char replay[sizeof(BootLine) + 12U];

        (void)snprintf(replay, sizeof(replay), "%s [replay]", BootLine);
        (void)vnd_meta_publish_debug_text(replay);
        s_boot_pub++;
      }
    }

    /* Release the ping-pong slot once its transfer is no longer active. */
    if (s_tx_counts && !vnd_tx_active)
    {
      g_tx_seq++;
      UsbFrames++;
      s_tx_counts = false;
    }

    /* Jam-breaker: see USB_JAM_MS. */
    {
      const bool jammed = !vnd_tx_active && (g_enc_seq != g_tx_seq) &&
                          (g_video_usb_queue != NULL) &&
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
        print("[JAMBREAK] reclaimed %lu leaked tx slot(s) (enc=%lu tx=%lu)\r\n",
              (unsigned long)(g_enc_seq - g_tx_seq),
              (unsigned long)g_enc_seq, (unsigned long)g_tx_seq);
        g_tx_seq       = g_enc_seq;
        s_tx_counts    = false;
        s_jam_since_ms = 0U;
      }
    }

    /* The capture-control capability announce, ~1 Hz. */
    if (!vnd_tx_active && ((HAL_GetTick() - s_caps_ms) >= CAM_CAPS_ANNOUNCE_MS))
    {
      static uint8_t caps_buf[4];

      s_caps_ms   = HAL_GetTick();
      caps_buf[0] = META_CAM_CAPS_TAG;
      caps_buf[1] = 1U;
      caps_buf[2] = 0U;
      caps_buf[3] = CAM_CAPS;
      (void)vnd_tx_start_frame(caps_buf, sizeof(caps_buf), NULL, 0U, 0U);
    }

    /* The boot-cause line on every console connect. */
    {
      const bool cdc_up = (BSP_COM_IsConnected() != 0);

      if (cdc_up && !s_cdc_was_up)
      {
        print("%s\r\n", BootLine);
      }
      s_cdc_was_up = cdc_up;
    }

    console();

    if ((NIA_HOST_TIMEOUT_MS != 0U) && (seen_a_host == 0U))
    {
      if (tud_mounted())
      {
        seen_a_host = 1U;
      }
      else if ((HAL_GetTick() - booted) > NIA_HOST_TIMEOUT_MS)
      {
        leave_to_bootloader();
      }
    }

    if ((HAL_GetTick() - report) >= 1000U)
    {
      const uint32_t enc = CapEncoded, sent = UsbFrames, torn = CapTorn;
      const uint32_t fail = CapFailed, resync = CapResync, nn = NnRuns;

      report = HAL_GetTick();
      print("[rate] enc=%lu tx=%lu torn=%lu fail=%lu resync=%lu nn=%lu npu=%lums jpg=%luB\r\n",
            (unsigned long)(enc - enc0), (unsigned long)(sent - sent0),
            (unsigned long)(torn - torn0), (unsigned long)(fail - fail0),
            (unsigned long)(resync - resync0), (unsigned long)(nn - nn0),
            (unsigned long)NnLastMs, (unsigned long)CapBytes);
      enc0 = enc; sent0 = sent; torn0 = torn; fail0 = fail; resync0 = resync; nn0 = nn;
    }

    /* Boot-log replay: one recorded line per idle step. */
    vnd_meta_debug_replay_step(vnd_tx_active ? 0 : 1);

    /* Take the next frame when the wire is free. A frame with no JPEG is
     * metadata only, sent with flags bit 0 clear, and owns no slot. */
    if (!vnd_tx_active && (g_video_usb_queue != NULL) &&
        (xQueueReceive(g_video_usb_queue, &msg, 0) == pdTRUE))
    {
      const uint32_t flags = (msg.jpeg_len > 0U) ? 1U : 0U;

      (void)vnd_tx_start_frame(msg.meta_ptr, msg.meta_len, msg.jpeg_ptr, msg.jpeg_len, flags);
      s_tx_counts = (msg.jpeg_len > 0U);
    }

    if (vnd_tx_active)
    {
      /* In flight: sleep until the USB interrupt reports progress, with one
       * tick as the safety net. */
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
  * @brief  The post-processor's LCD overlay hook: no panel here, so nothing
  *         to draw.
  */
void LCD_Overlay_Nia_From(const struct nia_pp_out *out, OVL_Mode_t mode)
{
  (void)out;
  (void)mode;
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
  uint32_t start;

  SCB_DisableDCache();
  mpu_config();
  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();

  /* No SystemClock_Config: the boot loader owns the clock tree and both
   * external memories; the BSP reads the frequencies back at run time. */

  /* The run LED: purple. */
  (void)BSP_LED_Init(LED_RED);
  (void)BSP_LED_Init(LED_GREEN);
  (void)BSP_LED_Init(LED_BLUE);
  (void)BSP_LED_On(LED_RED);
  (void)BSP_LED_On(LED_BLUE);

  /* The watchdog first, before anything that could wedge; a reset it caused
   * goes to the bootloader, not back into the code that caused it. */
  HAL_PWR_EnableBkUpAccess();
  boot_report_build(RCC->RSR);
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U)
  {
    __HAL_RCC_CLEAR_RESET_FLAGS();
    BSP_COM_RebootToBootloader();
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
  (void)BSP_WDG_InitEWI(WDG_TIMEOUT_MS, WDG_WARN_MS);

  /* Then USB: the stack, the metadata slots, the enumeration pump. Blocking
   * stack calls are safe this early only because the OSAL clamps their waits
   * to zero until the scheduler runs. */
  if (BSP_COM_Init() != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  vnd_meta_init(NULL, 0U);
  start = HAL_GetTick();
  while (!tud_mounted() && ((HAL_GetTick() - start) < USB_BOOT_PUMP_MS))
  {
    tud_task_ext(0, false);   /* non-blocking drain; bare tud_task() blocks under OPT_OS_FREERTOS */
    (void)BSP_WDG_Refresh();
    HAL_Delay(10U);
  }

  /* A depth-2 frame queue, matching the ping-pong. The USB task runs from the
   * start, so the console and the way back to the bootloader come first. */
  g_video_usb_queue = xQueueCreate(2U, sizeof(frame_msg_t));
  if ((g_video_usb_queue == NULL) ||
      (xTaskCreate(usb_task, "usb", USB_TASK_STACK_WORDS, NULL,
                   USB_TASK_PRIORITY, NULL) != pdPASS) ||
      (xTaskCreate(app_task, "app", APP_TASK_STACK_WORDS, NULL,
                   APP_TASK_PRIORITY, NULL) != pdPASS))
  {
    Error_Handler();
  }

  vTaskStartScheduler();

  Error_Handler();
  return 0;
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
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
