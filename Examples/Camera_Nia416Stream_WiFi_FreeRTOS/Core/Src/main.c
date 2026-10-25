/*
  Camera_Nia416Stream_WiFi_FreeRTOS

  Streams the camera to Neuro Studio over Wi-Fi, XGA, with a labelled box
  around every object Ohm Lab's NIA Triad-416 detector finds among the 80
  COCO classes. The board hosts its own access point, with its own password.
  The stream stays on USB until a Wi-Fi host connects, and returns to USB
  when it leaves. Neuro Studio finds the board by broadcast probe, as
  <name>.local over mDNS, or by the address it announces over USB. The
  radio is 2.4 GHz only, capped at 8 dBm, as EMC-tested.

  The circuit:
  - Uses the on-board camera module (OV5640) on the 30-pin connector, and
    the MAYA-W476 radio module. No wiring needed.
  - The detector's weights must already be in external flash at 0x90300000,
    and the radio's firmware at 0x91E00000. See README.md.

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
#include "neuron6_lcd.h"
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
#include "nia_lcd.h"
#include "vnd_meta.h"
#include "n6_host_contract.h"

#include "MayaW476.h"
#include "NeuroN6_FrameQueue.h"
#include "n6_asset.h"
#include "n6_asset_host.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#if !defined(BSP_USE_FREERTOS) || (BSP_USE_FREERTOS != 1)
#error "This example runs under FreeRTOS: set BSP_USE_FREERTOS to 1 in neuron6_conf.h."
#endif

/* Wi-Fi */

/* The board's own access point, with its own password; the stream runs on it
 * only. */
#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID            "NeuroN6-Cam"
#endif
#ifndef WIFI_AP_CHANNEL
#define WIFI_AP_CHANNEL         6U   /* 1..11, or 0 to choose; 2.4 GHz only */
#endif
#ifndef WIFI_AP_SECURITY
#define WIFI_AP_SECURITY        WIFI_SECURITY_WPA2   /* or WIFI_SECURITY_WPA3_SAE, _WPA2_WPA3 */
#endif

/* 1: an access point stored over USB (Neuro Studio's WiFi settings, or
 * harness/bench/n6bench.py wifiset) replaces the one above; a stored network
 * to join is not used. 0 ignores them. */
#ifndef WIFI_USE_STORED
#define WIFI_USE_STORED         1
#endif

/* How long to wait for the network, and Neuro Studio's port. */
#define WIFI_TIMEOUT_MS         10000U
#define WIFI_TCP_PORT           5000U

/* Tuning */

#define NIA_CONF                0.36f   /* the model's live default */
#define NIA_IOU                 0.70f   /* the model's NMS setting  */
#define HEARTBEAT_MS            1000U

/* NIA_LCD (neuron6_conf.h) also shows the picture on a TFT-W carrier's
 * 800 x 480 panel, which takes nothing larger than itself. */
#if NIA_LCD
/* WVGA, the panel's size, at its native 60 fps. */
#define CAM_RESOLUTION          BSP_CAMERA_R800x480
#define CAM_FPS                 60U
#else
/* XGA at its native 30 fps. */
#define CAM_RESOLUTION          BSP_CAMERA_R1024x768
#define CAM_FPS                 30U
#endif

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

/* How long USB is pumped at boot, before the scheduler starts. */
#define USB_BOOT_PUMP_MS        500U

/* Non-zero: go back to the bootloader if no host (USB or Wi-Fi) has connected
 * by then; never once one has. 0, the default, runs regardless. */
#ifndef NIA_HOST_TIMEOUT_MS
#define NIA_HOST_TIMEOUT_MS     0U
#endif

/* Resync the encoder and camera after this many failed encodes, or torn
 * frames in a row. */
#define RESYNC_AFTER_FAILS      8U
#define RESYNC_AFTER_TORN       2U

/* Progress deadlines: past one, the USB task stops feeding the watchdog and
 * its early warning takes the board to the bootloader. */
#define SETUP_LIMIT_MS          60000U  /* setup() */
#define APP_STALL_MS            10000U  /* a pass of loop() */
#define NN_STALL_MS             10000U  /* an inference run to the end */

/* How long a loop pass waits for an NN snapshot (30 frames at 30 fps). A late
 * snapshot is not cancelled, as a stopped pipe can still deliver it; the next
 * pass waits on. */
#define NN_SNAPSHOT_WAIT_MS     1000U

/* With no inference finished for this long, the boxes leave the stream. */
#define NN_STALE_MS             2000U

/* Frames outstanding with the link idle and nothing queued for this long are
 * lost accounting. */
#define USB_JAM_MS              1000U

/* The application task sits below capture and USB, so they run while the NPU
 * is busy; the capture task is raised one step across a re-arm. */
#define APP_TASK_STACK_WORDS      ((16U * 1024U) / sizeof(StackType_t))
#define APP_TASK_PRIORITY         (tskIDLE_PRIORITY + 2U)
#define USB_TASK_STACK_WORDS      (8192U / sizeof(StackType_t))
#define USB_TASK_PRIORITY         (tskIDLE_PRIORITY + 3U)
#define CAPTURE_TASK_STACK_WORDS  (8192U / sizeof(StackType_t))
#define CAPTURE_BASE_PRIORITY     (tskIDLE_PRIORITY + 3U)
#define CAPTURE_REARM_PRIORITY    (tskIDLE_PRIORITY + 4U)

/* The OV5640's capture controls Neuro Studio shows: exposure, gain and white
 * balance. */
#define CAM_CAPS                0x0FU
#define META_CAM_CAPS_TAG       0x64U
#define CAM_CAPS_ANNOUNCE_MS    1000U

/* 'CAM0' packet fields (NeuroN6_app.h): the auto states asked for, and which
 * fields apply. */
#define N6_CAM0_FLAG_AUTO_EXPOSURE   0x01U
#define N6_CAM0_FLAG_AUTO_GAIN       0x02U
#define N6_CAM0_FLAG_AUTO_AWB        0x04U
#define N6_CAM0_APPLY_AUTO_EXPOSURE  0x01U
#define N6_CAM0_APPLY_AUTO_GAIN      0x02U
#define N6_CAM0_APPLY_AUTO_AWB       0x04U
#define N6_CAM0_APPLY_EXPOSURE_US    0x08U
#define N6_CAM0_APPLY_GAIN           0x10U

#define VND_MAGIC               N6_FRAME_MAGIC   /* 'DNV0' on the wire */

#define STROBE_DISABLED         0
#define STROBE_ENABLED          1

/* The camera's strobe LED. STROBE_ENABLED lights it for every exposure. */
#define CAM_STROBE              STROBE_DISABLED

/* COCO-80 class names, from the post-processing library. */
#define NB_CLASSES              80
extern const char *classes_table[NB_CLASSES];

/* Types */

typedef n6_frame_hdr_t vnd_frame_hdr_t;   /* n6_host_contract.h */

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

/* Read by the post-processing: inference timestamps, and the crop window that
 * maps a detection onto the camera frame (non-zero at boot: no divide by 0). */
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
#if NIA_LCD
/* The panel's picture takes the band's on-chip RAM (nia_lcd.c), so the band
 * moves to AXISRAM6: 448 KB of NPU RAM from 0x34350000, unused by this
 * network and enabled by the isolation set-up with the other NPU banks. */
#define BAND_ADDRESS            0x34350000UL
static uint8_t *const Band = (uint8_t *)BAND_ADDRESS;
#else
static uint8_t Band[BAND_BYTES]
  __attribute__((section(".axisram_nc"), aligned(32)));
#endif

/* Two finished frames: one on the wire while the next is encoded. */
static uint8_t JpegBuf[2][JPEG_BUF_BYTES]
  __attribute__((section(".ext_psram"), aligned(32)));

/* State */

/* The vendor transmitter. */
static volatile bool     vnd_tx_active = false;
static vnd_frame_hdr_t   vnd_hdr;
static uint32_t          vnd_off;
static uint32_t          vnd_total;
/* Bytes still owed to a USB reader by a frame cut off when the stream moved
 * to a network host; paid in zeros before its next frame. */
static uint32_t          vnd_usb_owed;
static const uint8_t    *vnd_meta;
static uint32_t          vnd_meta_len;
static const uint8_t    *vnd_payload;
static uint32_t          vnd_payload_len;
static uint32_t          vnd_frame_id;
/* The SoftAP announce frame. It carries a passphrase, so the pump never sends
 * it to a network host (softap_announce_service). */
static uint8_t           SoftApAnnounceBuf[3U + 1U + 32U + 1U + 64U];

/* The frame queue (NeuroN6_FrameQueue.h): the capture task posts, the USB
 * task sends each frame to the attached host, USB or Wi-Fi. */
QueueHandle_t             g_n6_video_usb_queue  = NULL;
volatile n6_transport_t   g_n6_active_transport = TRANSPORT_USB;

/* The attached network host (NeuroN6_NetAttach), or NULL. Only the USB task
 * writes it; while it is set, the vendor stream goes to it instead of USB. */
static const n6_net_link_t *volatile s_net = NULL;
static TaskHandle_t       UsbTask;

/* Ping-pong: frame N is encoded into JpegBuf[N & 1] while the host takes the
 * other. g_enc_seq counts frames posted, g_tx_seq frames a consumer finished. */
static volatile uint32_t g_enc_seq;
static volatile uint32_t g_tx_seq;

/* Owned by the capture task. */
static BSP_VENC_Band_t   StreamBand;
static uint8_t           LinebufArmed;
static uint32_t          ConsecFail;
/* Line events since the camera's last frame event: zero between its frames. */
static volatile uint32_t FrameLines;
/* Set at the camera's frame start, cleared at its end; with FrameLines it
 * marks the whole frame. */
static volatile uint32_t FrameActive;
/* The last encode coded a frame, so the next arm follows it straight away.
 * Cleared when the frames are held back for the host, or an encode fails. */
static uint8_t           ArmFollowsEncode;
/* Frame events on the main pipe, which tell a camera that has stopped from
 * gaps the capture task missed. */
static volatile uint32_t FrameEnds;
#if NIA_LCD
/* Frames the dump pipe has put on the panel. */
static volatile uint32_t LcdFrames;
#endif
/* Set while the capture task waits for a frame to end: the frame event
 * then wakes it, so that it is armed at the start of the gap. */
static volatile uint8_t  CaptureWaiting;
static TaskHandle_t      CaptureTask;
/* The next capture is armed where the camera is: the wait saw frames end
 * but caught none of the gaps between them. */
static uint8_t           ArmAnyway;
/* How long a capture waits for a frame to end before taking the camera for
 * stopped: five frame periods, at least 150 ms, set from the camera's rate.
 * Five, as night mode stretches frames to a quarter of the rate. */
static uint32_t          CaptureGapWaitMs = 1000U;

/* The pipeline as setup left it. */
static volatile uint8_t  CameraOk;
static IPostProcessor   *BoundPP;

/* The weights are in external flash (harness/n6weights.py), read in place by
 * the NPU; anything else there gives nonsense detections with no error. So
 * vision_begin() checks the region against the CRC32 in nia416_model.c; on a
 * mismatch the network never runs, the stream carries on without detections,
 * and the console says how to put the weights back. */
extern const uint32_t    nia416_weights_address;
extern const uint32_t    nia416_weights_size;
extern const uint32_t    nia416_weights_crc32;
static uint8_t           NnWeightsOk;
static uint8_t           NnInputOk;       /* the NN frame fits the network's input */
static uint32_t          NnWeightsCrc;

/* Counters for the report, each written by one task and read by another. */
static volatile uint32_t CapEncoded, CapTorn, CapFailed, CapResync, CapBytes;
static volatile uint32_t SentFrames, NnRuns, NnLastMs, NnSnapMisses;

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

  /* Clock, pad path, rolling strobe and pad output before turning it on. */
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
  * @brief  Start the camera. The encoder calls this itself, once per stream,
  *         after it has parked waiting for the camera's first line event.
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
#if NIA_LCD
  if (Pipe == BSP_CAMERA_PIPE_DUMP)
  {
    LcdFrames++;
  }
#endif
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

  /* First capture, or first after a resync: stop the pipe and arm the line
   * event the encoder paces on; the encode restarts the pipe once parked. */
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
     * the top of the next as one picture, so it waits for the blanking. A
     * follow-on arm waits only for lines already arrived (also waiting out
     * the frame start would skip every other QSXGA frame); any other arm also
     * waits out frame start to first line event, or it runs a band behind. */
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
  * @brief  Release a ping-pong buffer. Every consumer calls it once for
  *         each frame it finishes, abandons, or drains on a reconnect.
  */
void n6_video_tx_complete(void)
{
  g_tx_seq++;
  SentFrames++;
}

/**
  * @brief  Queue the frame and its metadata (same-parity buffer) for the USB
  *         task; the ping-pong advances only if the queue took them.
  */
static void post_encoded_frame(uint8_t *pBuf, uint32_t Bytes)
{
  n6_frame_msg_t msg = {0};

  msg.jpeg_ptr = pBuf;
  msg.jpeg_len = Bytes;
  (void)vnd_meta_get_pingpong(g_enc_seq & 1U, &msg.meta_ptr, &msg.meta_len);

  if ((g_n6_video_usb_queue != NULL) && (xQueueSend(g_n6_video_usb_queue, &msg, 0) == pdTRUE))
  {
    g_enc_seq++;
    CapEncoded++;
    CapBytes = Bytes;
  }
}

/**
  * @brief  Wait, at most CaptureGapWaitMs, for the camera to end its frame;
  *         the frame event wakes the task so it arms at the gap's start.
  * @retval CAPTURE_GAP_OPEN    between frames: arm now
  * @retval CAPTURE_GAP_MISSED  frames ended but every gap was missed: arm
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

    /* Clear a stale wake, raise the flag, then look again, so no frame end
     * is lost between the two. */
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

    /* The ping-pong gate. */
    if ((g_enc_seq - g_tx_seq) >= 2U)
    {
      /* Consumer two frames behind: yield at base priority so it drains;
       * never wait at the raised one. */
      ArmFollowsEncode = 0U;
      vTaskPrioritySet(NULL, CAPTURE_BASE_PRIORITY);
      vTaskDelay(pdMS_TO_TICKS(1U));
      continue;
    }

    /* Raised across the re-arm so no time slice lands in the frame gap. */
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
          /* The camera stopped mid-frame: restart it. */
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
  * @brief  Bring the NPU up: the BSP part (isolation, RAM banks, NPU cache,
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
  * @retval 1 if it ran to the end, 0 if an error cut it short (outputs invalid)
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
      /* Blocks until the NPU's epoch interrupt (FreeRTOS OSAL). */
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
  * @brief  Check a Width x Height RGB888 snapshot fits the network's input
  *         (a smaller one is reported).
  * @retval 1 when it fits, 0 when it would overrun the input
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
  * @brief  Start one NN-pipe snapshot straight into the model's input.
  * @retval 1 when the snapshot was started, 0 when the camera refused it
  */
static uint8_t nn_feed_start(NN_Instance_TypeDef *pInstance)
{
  const LL_Buffer_InfoTypeDef *nn_in_info = LL_ATON_Input_Buffers_Info(pInstance);
  uint8_t *nn_in = (uint8_t *)LL_Buffer_addr_start(&nn_in_info[0]);

  cameraFrameReceived = 0;
  if (BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_AUX, nn_in, BSP_CAMERA_MODE_SNAPSHOT) != BSP_ERROR_NONE)
  {
    /* Refused: a missed snapshot, retried next pass. If refusals persist the
     * watchdog stops being fed (app_progressing()). */
    NnSnapMisses++;
    vTaskDelay(pdMS_TO_TICKS(10U));
    return 0U;
  }
  NnSnapshotArmed = 1U;
  return 1U;
}

/**
  * @brief  Wait for the snapshot (NN_SNAPSHOT_WAIT_MS), run the network, time it.
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
  NnLastMs = ts[1] - ts[0];   /* for the report: ts[] is mid-update whenever it runs */
  NnRuns++;
  if (done != 0U)
  {
    NnDoneMs = ts[1];
  }
  return done;
}

/**
  * @brief  Check the weights region's CRC32 (zlib's) against the network's:
  *         about a quarter of a second for the 8.5 MB.
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
static void vision_begin(IPostProcessor *pPP, int Transport)
{
  BSP_CAMERA_Capabilities_t cap  = {0};
  BSP_CAMERA_PipeConfig_t   pipe = {0};
  BSP_VENC_Config_t         venc = {0};
  uint32_t crop_x = 0U, crop_y = 0U, crop_w = 0U, crop_h = 0U;
  int32_t  status;

  /* The sensor, suspended while the strobe is set. */
  CameraOk = 0U;
#if NIA_LCD
  /* The TFT-W carrier crosses the camera's data lanes. */
  (void)BSP_CAMERA_SetCarrier(BSP_CAMERA_CARRIER_INVERTED);
#endif
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
    if ((status == BSP_ERROR_NONE) && (StreamBand.SizeBytes > BAND_BYTES))
    {
      status = BSP_ERROR_WRONG_PARAM;
    }

    /* The camera frame, and the NN input's centred square within it. */
    g_camera_full_w = (uint16_t)cap.Width;
    g_camera_full_h = (uint16_t)cap.Height;
    resolve_crop_center(cap.Width, cap.Height, &crop_x, &crop_y, &crop_w, &crop_h);

    /* The JPEG pipe, its line buffer, the NN pipe, and the prime. */
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
#if NIA_LCD
    /* The panel: the dump pipe writes RGB565 straight into its buffer. The
     * stream does not depend on it. */
    if (status == BSP_ERROR_NONE)
    {
      uint8_t *panel = NiaLcd_Init(cap.Width, cap.Height);
      int32_t  shown = BSP_ERROR_NO_INIT;

      if (panel != NULL)
      {
        memset(&pipe, 0, sizeof(pipe));
        pipe.PixelFormat = BSP_CAMERA_PIPE_PF_RGB565;
        pipe.Width       = cap.Width;
        pipe.Height      = cap.Height;
        shown = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_DUMP, &pipe);
        if (shown == BSP_ERROR_NONE)
        {
          shown = BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_DUMP, panel, BSP_CAMERA_MODE_CONTINUOUS);
        }
      }
      print("[LCD] panel %s, camera to it: %ld\r\n", (panel != NULL) ? "up" : "failed",
            (long)shown);
    }
#endif
    if (status == BSP_ERROR_NONE)
    {
      g_nn_crop_x = (uint16_t)crop_x;
      g_nn_crop_y = (uint16_t)crop_y;
      g_nn_crop_w = (uint16_t)crop_w;
      g_nn_crop_h = (uint16_t)crop_h;
      /* Prime: run the JPEG pipe into the band until the first capture. */
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

  /* The NPU, camera or not; the weights are checked first. */
  NnWeightsOk = nn_weights_check();
  npu_init();
  NnInputOk = nn_input_check(&NN_Instance_nia416, NN_WIDTH, NN_HEIGHT);

  /* The post-processor. */
  BoundPP = pPP;
  if ((pPP != NULL) && (pPP->init != NULL))
  {
    (void)pPP->init(pPP->ctx, &NN_Instance_nia416);
  }

  /* The producer; the USB consumer has run since boot. */
  if (CameraOk != 0U)
  {
    if (xTaskCreate(capture_task, "capture", CAPTURE_TASK_STACK_WORDS, NULL,
                    CAPTURE_BASE_PRIORITY, &CaptureTask) != pdPASS)
    {
      Error_Handler();
    }
  }

  /* A Wi-Fi transport starts the Wi-Fi library and its consumer; if that
   * fails, the video stays on USB. */
  g_n6_active_transport = (n6_transport_t)Transport;
  print("[VISION] transport dispatch: cfg.transport=%d\r\n", Transport);
  if (Transport == TRANSPORT_WIFI_SOFTAP)
  {
    const int rc = MayaW476_Start(Transport);

    if (rc != 0)
    {
      print("[VISION WARN] cfg.transport=%d requested but MayaW476 returned %d. "
            "Falling back to TRANSPORT_USB.\r\n", Transport, rc);
      g_n6_active_transport = TRANSPORT_USB;
    }
  }
}

/**
  * @brief  A pass with no inference to publish. Once none has finished for
  *         NN_STALE_MS, the boxes are taken off the stream.
  */
static void nn_missed(void)
{
  if ((HAL_GetTick() - NnDoneMs) > NN_STALE_MS)
  {
    vnd_meta_invalidate_slot(VND_META_SLOT_PRIMARY);
  }
}

/**
  * @brief  Whether the network is meant to run, so inferences are expected.
  */
static bool nn_expected(void)
{
  return (BoundPP != NULL) && (CameraOk != 0U) && (NnWeightsOk != 0U) && (NnInputOk != 0U);
}

/**
  * @brief  Whether the application task is progressing: setup() within
  *         SETUP_LIMIT_MS, then each loop() pass within APP_STALL_MS and,
  *         while the network should run, an inference within NN_STALL_MS.
  *         The stamps are read before the time.
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

/**
  * @brief  One snapshot, one inference, the post-processing, and the
  *         metadata the next frame carries.
  */
static void vision_run(void)
{
  /* No camera, wrong weights or an input that does not fit: the network
   * never runs, so wait rather than spin. */
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
    /* No snapshot yet, or an NPU error. Some outputs share memory with the
     * input and may hold camera pixels: nothing to decode or publish. */
    nn_missed();
    return;
  }

  if (BoundPP->run != NULL)
  {
    (void)BoundPP->run(BoundPP->ctx, &NN_Instance_nia416);
  }
#if NIA_LCD
  if (BoundPP->draw_lcd != NULL)
  {
    BoundPP->draw_lcd(BoundPP->ctx, OVL_RELOAD_CLEAR);
  }
#endif
  if (BoundPP->emit_metadata != NULL)
  {
    BoundPP->emit_metadata(BoundPP->ctx);
  }
}

/* Application */

/* The board's network and password, for every console that connects. The
 * password is random, made and stored on first start, which restarts the
 * board once. */
static char WifiLine[112];

static void setup(void)
{
  wl_status_t wifi_status;

  /* Optional: both have defaults. */
  MayaW476_SetTimeout(WIFI_TIMEOUT_MS);
  MayaW476_SetTcpPort(WIFI_TCP_PORT);

  /* The radio first: the slow step, it waits until the access point is up or
   * has given up. Its name and password are printed again whenever a console
   * connects. */
  const char *ssid = WIFI_AP_SSID;

#if (WIFI_USE_STORED == 1)
  if (MayaW476_StoredMode() == WL_STORED_STA)
  {
    print("[WiFi] the network stored on this board is one to join, and the stream "
          "runs on the board's own access point: it is not used\r\n");
  }
  if (MayaW476_StoredMode() == WL_STORED_AP)
  {
    ssid        = MayaW476_StoredSSID();
    wifi_status = MayaW476_BeginStored();
  }
  else
#endif
  {
    wifi_status = MayaW476_BeginAP(WIFI_AP_SSID, NULL, WIFI_AP_CHANNEL, WIFI_AP_SECURITY);
  }

  /* Refused settings leave the radio unstarted and its SDIO controller
   * unclocked; the stream stays on USB. */
  const bool refused = (wifi_status == WL_AP_FAILED) && (__HAL_RCC_SDMMC1_IS_CLK_ENABLED() == 0U);

  if (refused)
  {
    (void)snprintf(WifiLine, sizeof(WifiLine), "[WiFi] no access point: the stream stays on USB");
  }
  else
  {
    (void)snprintf(WifiLine, sizeof(WifiLine), "[WiFi] SoftAP \"%s\"  password: %s",
                   ssid, MayaW476_SoftApPassword());
  }
  print("%s\r\n", WifiLine);

  /* The network can be up a moment before it has an address to report,
   * so wait for a real one. */
  for (int i = 0; (i < 30) && (MayaW476_LocalIP() == 0U); i++)
  {
    vTaskDelay(pdMS_TO_TICKS(100U));
  }

  /* Diagnostic only. */
  print("[WiFi] status %d, address %s, port %u\r\n",
        (int)wifi_status, MayaW476_Ipv4String(), (unsigned)WIFI_TCP_PORT);

  /* The SDIO bus clock, read back: dividers 7 and 7 give 8.163 MHz, the clock
   * the radio passed radiated emissions with (neuron6_wifi.h). Read only while
   * clocked: reading an unclocked peripheral stalls the bus. */
  if (__HAL_RCC_SDMMC1_IS_CLK_ENABLED() != 0U)
  {
    const uint32_t kernel = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SDMMC1);
    const uint32_t clkdiv = (SDMMC1->CLKCR & SDMMC_CLKCR_CLKDIV_Msk) >> SDMMC_CLKCR_CLKDIV_Pos;

    print("[WiFi] SDIO clock: kernel %lu Hz (IC4 /%lu), bus divider %lu, bus %lu Hz\r\n",
          (unsigned long)kernel, (unsigned long)LL_RCC_IC4_GetDivider(),
          (unsigned long)clkdiv,
          (unsigned long)((clkdiv != 0U) ? (kernel / (2U * clkdiv)) : kernel));
  }

  /* Start the camera, the network stream and the detector. */
  vision_begin(PostProcess_NIA416(NIA_CONF, NIA_IOU),
               refused ? TRANSPORT_USB : TRANSPORT_WIFI_SOFTAP);

  print("=== n6_nia416_detector : OV5640 XGA -> Wi-Fi, NIA Triad-416 COCO-80 ===\r\n");
}

static void loop(void)
{
  static uint32_t last_beat = 0U;
  uint32_t        now;

  /* One capture and inference pass; the post-processor decodes the six NPU
   * tensors and publishes the boxes as stream metadata. */
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
  * @brief  Before any reset: stop the pipes and let the encoder go idle, so
  *         nothing is in flight when the board goes down.
  */
static void stop_capture(void)
{
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
  (void)BSP_CAMERA_HaltCapture(0);
  (void)BSP_VENC_Quiesce(0);
}

/**
  * @brief  Back to the bootloader, never with the encoder in flight.
  */
static void leave_to_bootloader(void)
{
  stop_capture();
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

/**
  * @brief  A new host reads the stream (USB mounted, or a network host
  *         attached or left): drop any half-sent frame (a USB reader is still
  *         paid: vnd_usb_owed), restart frame ids and metadata, and drop the
  *         queued frames, which can be minutes old, freeing their slots.
  */
static void host_session_start(void)
{
  n6_frame_msg_t stale;

  vnd_tx_reset();
  vnd_frame_id = 0U;
  while ((g_n6_video_usb_queue != NULL) &&
         (xQueueReceive(g_n6_video_usb_queue, &stale, 0) == pdTRUE))
  {
    if (stale.jpeg_len > 0U)
    {
      n6_video_tx_complete();
    }
  }
  vnd_meta_invalidate();
  vnd_meta_debug_mark_replay();
}

/* While a network host has the stream, USB coming and going must not touch
 * it. Both run in the USB task, the only writer of s_net. */
void tud_mount_cb(void)
{
  vnd_usb_owed = 0U;   /* a new reader was promised nothing */
  if (s_net == NULL)
  {
    host_session_start();
  }
}

void tud_umount_cb(void)
{
  vnd_usb_owed = 0U;
  if (s_net == NULL)
  {
    vnd_tx_reset();
  }
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
  * @brief  The unsent run of the current frame at vnd_off: the rest of the
  *         header, of the metadata or of the payload, whichever it is in.
  */
static uint32_t vnd_tx_chunk(const uint8_t **ppSrc)
{
  if (vnd_off < sizeof(vnd_hdr))
  {
    *ppSrc = ((const uint8_t *)&vnd_hdr) + vnd_off;
    return sizeof(vnd_hdr) - vnd_off;
  }
  if (vnd_off < (sizeof(vnd_hdr) + vnd_meta_len))
  {
    *ppSrc = vnd_meta + (vnd_off - sizeof(vnd_hdr));
    return (sizeof(vnd_hdr) + vnd_meta_len) - vnd_off;
  }
  *ppSrc = vnd_payload + (vnd_off - (sizeof(vnd_hdr) + vnd_meta_len));
  return vnd_total - vnd_off;
}

/* The network pump: the same frame walk, sent until the connection is full.
 * Header and metadata go as one write when they fit NET_HEAD_MAX. A host that
 * takes nothing for NET_STALL_MS is dropped: TCP would probe for ever. */
#define NET_HEAD_MAX   1024U
#define NET_STALL_MS   5000U

static uint32_t NetProgressMs;   /* last byte taken, or last idle pass */

static void net_detach(void);

static void vnd_tx_pump_net(const n6_net_link_t *pNet)
{
  static uint8_t head[NET_HEAD_MAX];
  const uint32_t head_len = sizeof(vnd_hdr) + vnd_meta_len;

  while (vnd_off < vnd_total)
  {
    const uint8_t *src;
    uint32_t       n;
    int32_t        w;

    if ((vnd_off < head_len) && (head_len <= sizeof(head)))
    {
      if (vnd_off == 0U)
      {
        memcpy(head, &vnd_hdr, sizeof(vnd_hdr));
        if (vnd_meta_len != 0U)
        {
          memcpy(&head[sizeof(vnd_hdr)], vnd_meta, vnd_meta_len);
        }
      }
      src = &head[vnd_off];
      n   = head_len - vnd_off;
    }
    else
    {
      n = vnd_tx_chunk(&src);
    }

    w = pNet->send(src, n);
    if (w < 0)
    {
      net_detach();
      return;
    }
    if (w == 0)
    {
      if ((HAL_GetTick() - NetProgressMs) > NET_STALL_MS)
      {
        print("[NET] host stopped reading - dropped\r\n");
        net_detach();
      }
      return;   /* connection full: the next pass carries on */
    }
    vnd_off      += ((uint32_t)w < n) ? (uint32_t)w : n;
    NetProgressMs = HAL_GetTick();
  }
  vnd_tx_active = false;
}

/**
  * @brief  The vendor pump: header, then metadata, then payload, as
  *         fast as the endpoint takes them, or the attached network host.
  */
static void vnd_tx_pump(void)
{
  const n6_net_link_t *const net = s_net;
  uint32_t avail;
  bool     wrote_any = false;

  if (!vnd_tx_active)
  {
    return;
  }
  if (net != NULL)
  {
    /* A host that attached part way through the SoftAP announce never
     * gets it. */
    if (vnd_meta == SoftApAnnounceBuf)
    {
      vnd_tx_reset();
      return;
    }
    vnd_tx_pump_net(net);
    return;
  }
  /* Not ready (suspended or unmounted): a frame not started is dropped; one
   * part way out waits, or a resumed reader would find the next frame inside
   * this one's length. A mount starts afresh (tud_mount_cb). */
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

  /* What the reader is owed, in zeros, so that this frame's header lands
   * where the reader looks for one. */
  while ((vnd_usb_owed != 0U) && (avail != 0U))
  {
    static const uint8_t zeros[64];
    const uint32_t       n = (vnd_usb_owed < sizeof(zeros)) ? vnd_usb_owed : sizeof(zeros);
    const uint32_t       w = tud_vendor_n_write(0, zeros, (n < avail) ? n : avail);

    if (w == 0U)
    {
      break;
    }
    vnd_usb_owed -= w;
    wrote_any     = true;
    avail         = tud_vendor_n_write_available(0);
  }
  if (vnd_usb_owed != 0U)
  {
    if (wrote_any)
    {
      (void)tud_vendor_n_write_flush(0);
    }
    return;
  }

  while (avail != 0U)
  {
    const uint8_t *src;
    uint32_t       chunk_rem;
    uint32_t       n;
    uint32_t       w;

    if (vnd_off == vnd_total)
    {
      vnd_tx_active = false;
      (void)tud_vendor_n_write_flush(0);
      return;
    }

    chunk_rem = vnd_tx_chunk(&src);
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
  * @brief  Route one host command (a USB bulk transfer or a framed network
  *         command), in the USB task. ISP, tunable and evidence commands are
  *         taken and dropped: this example has none.
  */
static void host_command(const uint8_t *pCmd, uint32_t Count, bool FromUsb)
{
  if (Count >= 4U)
  {
    const uint32_t magic = (uint32_t)pCmd[0] | ((uint32_t)pCmd[1] << 8) |
                           ((uint32_t)pCmd[2] << 16) | ((uint32_t)pCmd[3] << 24);

    if ((magic == N6_CMD_ISP0) && (Count >= N6_CMD_ISP0_MIN_BYTES))
    {
      return;   /* the manual ISP is RAW8 only; the OV5640 is not */
    }
    if ((magic == N6_CMD_CAM0) && (Count >= sizeof(n6_cmd_cam0_t)))
    {
      /* Applied here: the BSP locks the sensor bus per transaction, so this
       * waits one transaction at most. */
      const uint8_t  flags       = pCmd[4];
      const uint32_t exposure_us = (uint32_t)pCmd[5]         |
                                   ((uint32_t)pCmd[6] << 8)  |
                                   ((uint32_t)pCmd[7] << 16) |
                                   ((uint32_t)pCmd[8] << 24);
      const uint16_t gain_x256   = (uint16_t)((uint16_t)pCmd[9] |
                                              ((uint16_t)pCmd[10] << 8));
      const uint8_t  apply_mask  = pCmd[11];

      if (CameraOk != 0U)
      {
        cam0_apply(flags, exposure_us, gain_x256, apply_mask);
      }
      return;
    }
    if (((magic == N6_CMD_TUN0) && (Count >= sizeof(n6_cmd_tun0_t))) ||
        ((magic == N6_CMD_SUB0) && (Count >= sizeof(n6_cmd_sub0_t))))
    {
      return;
    }

    /* USB only from here: a network host gets the live controls only. A
     * staged asset or record resets the board, and the FSBL writes it to
     * flash on the way back up. */
    if (!FromUsb)
    {
      return;
    }
    switch (n6_asset_host_command(pCmd, Count))
    {
      case N6_ASSET_HOST_STAGED:
        stop_capture();
        n6_asset_apply();
        return;
      case N6_ASSET_HOST_TAKEN:
        return;
      default:
        break;
    }
  }

  if (FromUsb && (Count >= 10U) && (memcmp(pCmd, "REBOOT_DFU", 10U) == 0))
  {
    leave_to_bootloader();
  }
}

void tud_vendor_rx_cb(uint8_t idx, const uint8_t *buffer, uint32_t bufsize)
{
  uint8_t  cmd_buf[N6_HOST_CMD_MAX];
  uint32_t count;

  (void)idx;

  if (buffer == NULL)
  {
    count = tud_vendor_n_read(0, cmd_buf, sizeof(cmd_buf));
    /* One command a transfer: the rest is dropped, so the next starts clean
     * and the FIFO keeps room. */
    tud_vendor_n_read_flush(0);
  }
  else
  {
    count = (bufsize < sizeof(cmd_buf)) ? bufsize : sizeof(cmd_buf);
    memcpy(cmd_buf, buffer, count);
  }
  if (count != 0U)
  {
    host_command(cmd_buf, count, true);
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

/* The IP announce: on a network transport, a metadata-only frame at about
 * 1 Hz with one META_BOARD_IPV4 TLV, so the host can open the video socket:
 *   [3 B TLV header] type 0x20, length 8, little-endian
 *   [4 B IPv4]       the address, first octet first
 *   [4 B port]       u32 little-endian; 0 means the default port */
#define IP_ANNOUNCE_INTERVAL_MS   1000U
#define IP_ANNOUNCE_DEFAULT_PORT  5000U
#define META_BOARD_IPV4           0x20U

static uint8_t  IpAnnounceBuf[11];
static uint32_t LastIpAnnounceMs;

static bool ip_announce_should_emit(void)
{
  uint32_t now;

  if (g_n6_active_transport == TRANSPORT_USB)
  {
    return false;
  }
  if (vnd_tx_active)
  {
    return false;   /* don't race the normal frame path */
  }
  if (MayaW476_GetIPv4_BE() == 0U)
  {
    return false;   /* not connected yet */
  }
  now = HAL_GetTick();
  if ((now - LastIpAnnounceMs) < IP_ANNOUNCE_INTERVAL_MS)
  {
    return false;
  }
  LastIpAnnounceMs = now;
  return true;
}

/* The 8-byte META_BOARD_IPV4 value, written at pV. */
static void ip_announce_value(uint8_t *pV)
{
  const uint32_t ip   = MayaW476_GetIPv4_BE();
  uint16_t       port = MayaW476_GetIPv4_Port();

  if (port == 0U)
  {
    port = IP_ANNOUNCE_DEFAULT_PORT;
  }

  /* The copy puts the first octet first on this little-endian core. */
  memcpy(pV, &ip, 4U);
  pV[4] = (uint8_t)(port & 0xFFU);
  pV[5] = (uint8_t)((port >> 8) & 0xFFU);
  pV[6] = 0U;
  pV[7] = 0U;
}

static void ip_announce_emit(void)
{
  IpAnnounceBuf[0] = META_BOARD_IPV4;
  IpAnnounceBuf[1] = 8U;
  IpAnnounceBuf[2] = 0U;
  ip_announce_value(&IpAnnounceBuf[3]);
  (void)vnd_tx_start_frame(IpAnnounceBuf, sizeof(IpAnnounceBuf), NULL, 0U, 0U);
}

/* The SoftAP announce, USB only: about once a second, a metadata-only frame
 * with one META_BOARD_SOFTAP TLV so Neuro Studio can join the access point:
 *   [3 B TLV header] type 0x24, length, little-endian
 *   [1 B] the name's length, the name, [1 B] the passphrase's length, the
 *   passphrase
 * It never goes to a network host (vnd_tx_pump() drops it if one attaches). */
#define SOFTAP_ANNOUNCE_INTERVAL_MS  1000U

static uint32_t LastSoftApAnnounceMs;

static void softap_announce_service(void)
{
  uint32_t now;
  uint32_t len;

  if (vnd_tx_active || (s_net != NULL))
  {
    return;
  }
  now = HAL_GetTick();
  if ((now - LastSoftApAnnounceMs) < SOFTAP_ANNOUNCE_INTERVAL_MS)
  {
    return;
  }
  len = MayaW476_SoftApAnnounce(&SoftApAnnounceBuf[3], sizeof(SoftApAnnounceBuf) - 3U);
  if (len == 0U)
  {
    return;
  }
  SoftApAnnounceBuf[0] = N6_META_BOARD_SOFTAP;
  SoftApAnnounceBuf[1] = (uint8_t)(len & 0xFFU);
  SoftApAnnounceBuf[2] = (uint8_t)(len >> 8);
  if (vnd_tx_start_frame(SoftApAnnounceBuf, 3U + len, NULL, 0U, 0U))
  {
    LastSoftApAnnounceMs = now;
  }
}

/**
  * @brief  The same announce as one whole frame, the Wi-Fi library's answer
  *         to a discovery probe. 0 when Cap is too small or there is no
  *         address yet.
  */
uint32_t NeuroN6_IpAnnounceFrame(uint8_t *pOut, uint32_t Cap)
{
  const vnd_frame_hdr_t hdr = { VND_MAGIC, 0U, sizeof(IpAnnounceBuf), 0U, 0U };
  const uint32_t        len = sizeof(hdr) + sizeof(IpAnnounceBuf);

  if ((pOut == NULL) || (Cap < len) || (MayaW476_GetIPv4_BE() == 0U))
  {
    return 0U;
  }
  memcpy(pOut, &hdr, sizeof(hdr));
  pOut[sizeof(hdr)]      = META_BOARD_IPV4;
  pOut[sizeof(hdr) + 1U] = 8U;
  pOut[sizeof(hdr) + 2U] = 0U;
  ip_announce_value(&pOut[sizeof(hdr) + 3U]);
  return len;
}

/* Network host (Wi-Fi) */

/* A client the Wi-Fi library has handed over, not yet taken by the USB task. */
static const n6_net_link_t *volatile s_net_pending = NULL;

/* Framed commands from the network host, reassembled across reads. A whole
 * frame always fits, so a full buffer always holds one to consume. */
static uint8_t  NetRx[sizeof(vnd_frame_hdr_t) + N6_HOST_CMD_MAX];
static uint32_t NetRxLen;

/**
  * @brief  Hand a connected client to the USB task, which takes it within a
  *         millisecond. False, leaving the connection with the caller, when a
  *         host is already attached or the USB task is not running.
  */
bool NeuroN6_NetAttach(const n6_net_link_t *pLink)
{
  bool free_slot;

  if ((pLink == NULL) || (pLink->send == NULL) || (pLink->recv == NULL) ||
      (pLink->closed == NULL) || (UsbTask == NULL))
  {
    return false;
  }
  taskENTER_CRITICAL();
  free_slot = (s_net == NULL) && (s_net_pending == NULL);
  if (free_slot)
  {
    s_net_pending = pLink;
  }
  taskEXIT_CRITICAL();

  if (free_slot)
  {
    (void)xTaskNotifyGive(UsbTask);
  }
  return free_slot;
}

/**
  * @brief  Let go of the network host: the stream returns to USB as a new
  *         session, and the library is told it may close the connection.
  */
static void net_detach(void)
{
  const n6_net_link_t *const link = s_net;

  if (link == NULL)
  {
    return;
  }
  s_net    = NULL;
  NetRxLen = 0U;
  host_session_start();
  print("[NET] host detached - stream back on USB\r\n");
  link->closed();
}

/**
  * @brief  Once per USB task pass: take over a newly handed-over host, or
  *         apply the commands the current one has sent.
  */
static void net_service(void)
{
  const n6_net_link_t *link = s_net;
  uint32_t             space;
  int32_t              n;

  if (link == NULL)
  {
    link = s_net_pending;
    if (link == NULL)
    {
      return;
    }
    /* A frame part way out to the USB reader is cut off here: the rest of
     * what it promised is owed to that reader. */
    if (vnd_tx_active && (vnd_off != 0U) && (vnd_off < vnd_total))
    {
      vnd_usb_owed = vnd_total - vnd_off;
    }
    s_net_pending = NULL;
    s_net         = link;
    NetRxLen      = 0U;
    host_session_start();
    NetProgressMs = HAL_GetTick();
    print("[NET] host attached - stream moved to the network\r\n");
    return;
  }

  /* The stall clock only runs while a frame waits (vnd_tx_pump_net). */
  if (!vnd_tx_active)
  {
    NetProgressMs = HAL_GetTick();
  }

  space = sizeof(NetRx) - NetRxLen;
  n     = link->recv(&NetRx[NetRxLen], space);
  if (n < 0)
  {
    net_detach();
    return;
  }
  NetRxLen += ((uint32_t)n < space) ? (uint32_t)n : space;

  while (NetRxLen >= sizeof(vnd_frame_hdr_t))
  {
    vnd_frame_hdr_t h;
    uint32_t        total;

    memcpy(&h, NetRx, sizeof(h));
    if ((h.magic != VND_MAGIC) || (h.meta_len != 0U) || (h.payload_len > N6_HOST_CMD_MAX))
    {
      print("[NET] host sent something other than a command - dropped\r\n");
      net_detach();
      return;
    }
    total = sizeof(h) + h.payload_len;
    if (NetRxLen < total)
    {
      break;
    }
    if (h.payload_len != 0U)
    {
      host_command(&NetRx[sizeof(h)], h.payload_len, false);
    }
    NetRxLen -= total;
    memmove(NetRx, &NetRx[total], NetRxLen);
  }
}

/**
  * @brief  The USB task: feeds the watchdog, pumps the stream and the
  *         announces, serves the network host and the console.
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
  uint32_t net_report     = 0U;
  uint32_t enc0 = 0U, sent0 = 0U, torn0 = 0U, fail0 = 0U, resync0 = 0U, nn0 = 0U;

  (void)pArg;

  /* The USB interrupt wakes this task when it queues an event. */
  BSP_COM_SetPumpTask(xTaskGetCurrentTaskHandle());

  for (;;)
  {
    n6_frame_msg_t msg;

    /* Fed only while the application task progresses, so a task stuck in the
     * NPU's or the camera's wait is caught by the watchdog. */
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
    net_service();            /* network host: take over, or read its commands */

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

    /* Release the slot once its transfer is done (state-, not edge-based). */
    if (s_tx_counts && !vnd_tx_active)
    {
      n6_video_tx_complete();
      s_tx_counts = false;
    }

    /* Jam-breaker: slots outstanding with the link idle and nothing queued
     * are lost accounting. */
    {
      const bool jammed = !vnd_tx_active && (g_enc_seq != g_tx_seq) &&
                          (g_n6_video_usb_queue != NULL) &&
                          (uxQueueMessagesWaiting(g_n6_video_usb_queue) == 0U);
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

    /* The network address, for Neuro Studio: silent on USB video, and before
     * there is an address. */
    if (ip_announce_should_emit())
    {
      ip_announce_emit();
    }

    /* The access point's name and passphrase, USB only, ~1 Hz. */
    softap_announce_service();

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
        if (WifiLine[0] != '\0')
        {
          print("%s\r\n", WifiLine);
        }
      }
      s_cdc_was_up = cdc_up;
    }

    console();

    if ((NIA_HOST_TIMEOUT_MS != 0U) && (seen_a_host == 0U))
    {
      if (tud_mounted() || (s_net != NULL))
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
      const uint32_t enc = CapEncoded, sent = SentFrames, torn = CapTorn;
      const uint32_t fail = CapFailed, resync = CapResync, nn = NnRuns;

      report = HAL_GetTick();
      print("[rate] enc=%lu tx=%lu torn=%lu fail=%lu resync=%lu nn=%lu npu=%lums jpg=%luB\r\n",
            (unsigned long)(enc - enc0), (unsigned long)(sent - sent0),
            (unsigned long)(torn - torn0), (unsigned long)(fail - fail0),
            (unsigned long)(resync - resync0), (unsigned long)(nn - nn0),
            (unsigned long)NnLastMs, (unsigned long)CapBytes);
      enc0 = enc; sent0 = sent; torn0 = torn; fail0 = fail; resync0 = resync; nn0 = nn;
#if NIA_LCD
      /* The panel: frames shown, overlays drawn, stalled draws, and display
       * controller errors. */
      {
        static uint32_t   shown0;
        static uint32_t   draws0;
        const uint32_t    shown = LcdFrames;
        const uint32_t    draws = NiaLcd_Draws();
        BSP_LCD_Status_t  lcd   = {0};

        (void)BSP_LCD_GetStatus(0, &lcd);
        print("[lcd] camera=%lu boxes=%lu stalls=%lu underrun=%lu buserr=%lu\r\n",
              (unsigned long)(shown - shown0), (unsigned long)(draws - draws0),
              (unsigned long)NiaLcd_Stalls(), (unsigned long)lcd.Underrun,
              (unsigned long)lcd.TransferError);
        shown0 = shown;
        draws0 = draws;
      }
#endif

      /* The network, and the kernel heap the Wi-Fi stack allocates from. */
      if ((++net_report % 5U) == 0U)
      {
        print("[wifi] transport=%d status=%d ssid=%s ip=%s:%u heap=%luB min=%luB\r\n",
              (int)g_n6_active_transport, (int)MayaW476_Status(), MayaW476_SSID(),
              MayaW476_Ipv4String(), (unsigned)MayaW476_GetIPv4_Port(),
              (unsigned long)xPortGetFreeHeapSize(),
              (unsigned long)xPortGetMinimumEverFreeHeapSize());
      }
    }

    /* Boot-log replay: one recorded line per idle step. */
    vnd_meta_debug_replay_step(vnd_tx_active ? 0 : 1);

    /* Take the next frame when the wire is free. A frame with no JPEG is
     * metadata only, sent with flags bit 0 clear, and owns no slot. */
    if (!vnd_tx_active && (g_n6_video_usb_queue != NULL) &&
        (xQueueReceive(g_n6_video_usb_queue, &msg, 0) == pdTRUE))
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
  * @brief  The Wi-Fi library's console: the USB console, and nothing when
  *         no host is attached.
  */
void NeuroN6_DbgPrint(const char *msg)
{
  print("%s", msg);
}

/**
  * @brief  Replaces newlib's assert, whose abort() loses the message. In a
  *         task: report, then suspend it so the USB task keeps running. In an
  *         interrupt or before the scheduler: hold until the watchdog takes
  *         the board to the bootloader.
  */
void __assert_func(const char *file, int line, const char *func, const char *expr)
{
  if ((__get_IPSR() == 0U) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING))
  {
    print("\r\n[ASSERT] %s:%d in %s(): '%s'\r\n",
          (file != NULL) ? file : "?", line,
          (func != NULL) ? func : "?",
          (expr != NULL) ? expr : "?");
    vTaskSuspend(NULL);
  }
  for (;;)
  {
    __NOP();
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

  /* The watchdog first; a reset it caused goes to the bootloader, not back
   * into the code that caused it. */
  HAL_PWR_EnableBkUpAccess();
  boot_report_build(RCC->RSR);
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U)
  {
    __HAL_RCC_CLEAR_RESET_FLAGS();
    BSP_COM_RebootToBootloader();
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
  (void)BSP_WDG_InitEWI(WDG_TIMEOUT_MS, WDG_WARN_MS);

  /* Then USB: controller, stack, metadata slots, boot enumeration pump.
   * Blocking stack calls work this early only because the OSAL clamps their
   * waits to zero until the scheduler runs. */
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

  /* A depth-2 frame queue, matching the ping-pong. The USB task runs first,
   * so the console and the way back to the bootloader exist before the
   * application touches the radio or the camera. */
  g_n6_video_usb_queue = xQueueCreate(2U, sizeof(n6_frame_msg_t));
  if ((g_n6_video_usb_queue == NULL) ||
      (xTaskCreate(usb_task, "usb", USB_TASK_STACK_WORDS, NULL,
                   USB_TASK_PRIORITY, &UsbTask) != pdPASS) ||
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
