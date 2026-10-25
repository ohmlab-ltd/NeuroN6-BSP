/**
  * @file    neuron6_camera.h
  * @brief   Camera capture on the Neuro N6: a CSI-2 sensor into the DCMIPP's three pipes.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_CAMERA_H
#define NEURON6_CAMERA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "neuron6_conf.h"
#include "neuron6_errno.h"
#include "ov5640.h"

/* One sensor on the board's CSI-2 link (two data lanes, virtual channel 0). */

/** @brief One instance: a single link, with whichever sensor the carrier fits. */
#define BSP_CAMERA_INSTANCES_NBR        1U

/** @brief OV5640 bus address, 8-bit form. */
#define BSP_CAMERA_I2C_ADDRESS          0x78U

/** @brief OV5640 chip ID; the probe fails on any other. */
#define BSP_CAMERA_OV5640_ID            0x5640U

/** @brief Capture destination alignment, in bytes. With USE_BSP_CPU_CACHE_MAINTENANCE a
  * cacheable destination must also start on a 32-byte line and own its last line: whole
  * lines are cleaned and invalidated. */
#define BSP_CAMERA_BUFFER_ALIGNMENT     16U

/* Carrier lane mapping: the carrier, not the sensor or board, decides which wire carries
 * which data lane. */
/** @brief Data lanes straight through. */
#define BSP_CAMERA_CARRIER_STANDARD     0U
/** @brief Display carrier: the data pair is crossed. */
#define BSP_CAMERA_CARRIER_INVERTED     1U

/** @brief Carrier used until BSP_CAMERA_SetCarrier; override in neuron6_conf.h. */
#ifndef BSP_CAMERA_DEFAULT_CARRIER
#define BSP_CAMERA_DEFAULT_CARRIER      BSP_CAMERA_CARRIER_STANDARD
#endif

/* Pipes; the values are the DCMIPP pipe indices. */
#define BSP_CAMERA_PIPE_DUMP            DCMIPP_PIPE0  /*!< RGB565 full frame.   */
#define BSP_CAMERA_PIPE_MAIN            DCMIPP_PIPE1  /*!< YUV422 full frame.   */
#define BSP_CAMERA_PIPE_AUX             DCMIPP_PIPE2  /*!< RGB888, crop/resize. */
#define BSP_CAMERA_PIPES_NBR            3U
/** @brief Error-callback pipe for a fault that is not pipe specific. */
#define BSP_CAMERA_PIPE_NONE            0xFFFFFFFFU

#define BSP_CAMERA_MODE_CONTINUOUS      DCMIPP_MODE_CONTINUOUS /*!< Free-running. */

/** @brief Capture one frame and stop.
  * @warning The first frame after the sensor starts streaming is not a picture. */
#define BSP_CAMERA_MODE_SNAPSHOT        DCMIPP_MODE_SNAPSHOT

/* OV5640 resolutions supported on this board (the driver's own codes). */
#define BSP_CAMERA_R320x240             OV5640_R320x240   /*!< QVGA.  */
#define BSP_CAMERA_R640x480             OV5640_R640x480   /*!< VGA.   */
#define BSP_CAMERA_R800x480             OV5640_R800x480   /*!< WVGA.  */
/** @warning XGA (768 Mbps per lane) is the worst case of the silent bring-up failure (see
  * BSP_CAMERA_Init). XGA, 720p and 1080p need the D-PHY patch in docs/HAL-PATCHES.md. */
#define BSP_CAMERA_R1024x768            OV5640_R1024x768  /*!< XGA.   */
#define BSP_CAMERA_R1280x720            OV5640_R1280x720  /*!< 720p.  */
#define BSP_CAMERA_R1920x1080           OV5640_R1920x1080 /*!< 1080p. */
#define BSP_CAMERA_R2592x1944           OV5640_R2592x1944 /*!< QSXGA. */

/* Sensor output format; a sensor accepts only the one its descriptor names. */
#define BSP_CAMERA_PF_RGB565            OV5640_RGB565 /*!< Colour, ready to use. */
#define BSP_CAMERA_PF_RAW8              0x10U         /*!< Raw Bayer, needs demosaicing. */

/* CSI-2 data types; they follow from the sensor descriptor, not the caller. */
#define BSP_CAMERA_CSI_DT_RGB565        DCMIPP_DT_RGB565
#define BSP_CAMERA_CSI_DT_RAW8          DCMIPP_DT_RAW8

/* Pipe destination formats, each for its own pipe. The planar ones (MAIN) start through
 * their own entry points, not BSP_CAMERA_Start. */
#define BSP_CAMERA_PIPE_PF_RGB565       0x00U /*!< 16 bpp, DUMP only, colour sensor. */
#define BSP_CAMERA_PIPE_PF_YUV422       0x01U /*!< 16 bpp YUYV, MAIN only. */
#define BSP_CAMERA_PIPE_PF_RGB888       0x02U /*!< 24 bpp, AUX only.  */
#define BSP_CAMERA_PIPE_PF_RAW8         0x03U /*!< 8 bpp Bayer (raw sensor), DUMP only. */
#define BSP_CAMERA_PIPE_PF_YUV420_SP    0x04U /*!< 12 bpp NV12 (two planes), MAIN only. */
#define BSP_CAMERA_PIPE_PF_YUV420_FP    0x05U /*!< 12 bpp, three planes, MAIN only. */

#define BSP_CAMERA_MIRRORFLIP_NONE      OV5640_MIRROR_FLIP_NONE
#define BSP_CAMERA_MIRRORFLIP_FLIP      OV5640_FLIP
#define BSP_CAMERA_MIRRORFLIP_MIRROR    OV5640_MIRROR
#define BSP_CAMERA_MIRRORFLIP_BOTH      OV5640_MIRROR_FLIP

/* BSP_CAMERA_DEFAULT_MIRRORFLIP: the OV5640's start-up orientation. Undefined, it follows
 * the carrier (flip on the standard one, none on the display one); define it in
 * neuron6_conf.h for a module mounted otherwise. */

/* Picture-control ranges, enforced: the driver indexes a table with them unchecked. */
#define BSP_CAMERA_BRIGHTNESS_MIN       (-4)
#define BSP_CAMERA_BRIGHTNESS_MAX       4
#define BSP_CAMERA_SATURATION_MIN       (-4)
#define BSP_CAMERA_SATURATION_MAX       4
#define BSP_CAMERA_CONTRAST_MIN         (-4)
#define BSP_CAMERA_CONTRAST_MAX         4
#define BSP_CAMERA_HUE_DEGREE_MIN       (-6)
#define BSP_CAMERA_HUE_DEGREE_MAX       5

/* White-balance presets. */
#define BSP_CAMERA_LIGHT_AUTO           OV5640_LIGHT_AUTO
#define BSP_CAMERA_LIGHT_SUNNY          OV5640_LIGHT_SUNNY
#define BSP_CAMERA_LIGHT_OFFICE         OV5640_LIGHT_OFFICE
#define BSP_CAMERA_LIGHT_HOME           OV5640_LIGHT_HOME
#define BSP_CAMERA_LIGHT_CLOUDY         OV5640_LIGHT_CLOUDY

#define BSP_CAMERA_COLOR_EFFECT_NONE    OV5640_COLOR_EFFECT_NONE
#define BSP_CAMERA_COLOR_EFFECT_BLUE    OV5640_COLOR_EFFECT_BLUE
#define BSP_CAMERA_COLOR_EFFECT_RED     OV5640_COLOR_EFFECT_RED
#define BSP_CAMERA_COLOR_EFFECT_GREEN   OV5640_COLOR_EFFECT_GREEN
#define BSP_CAMERA_COLOR_EFFECT_BW      OV5640_COLOR_EFFECT_BW
#define BSP_CAMERA_COLOR_EFFECT_SEPIA   OV5640_COLOR_EFFECT_SEPIA
#define BSP_CAMERA_COLOR_EFFECT_NEGATIVE OV5640_COLOR_EFFECT_NEGATIVE

/* Sensor digital zoom: crops and scales back up, so the pipes need no change. */
#define BSP_CAMERA_ZOOM_x1              OV5640_ZOOM_x1
#define BSP_CAMERA_ZOOM_x2              OV5640_ZOOM_x2
#define BSP_CAMERA_ZOOM_x4              OV5640_ZOOM_x4
#define BSP_CAMERA_ZOOM_x8              OV5640_ZOOM_x8

/* Night mode: in low light the sensor may drop below the BSP_CAMERA_GetCapabilities rate. */
#define BSP_CAMERA_NIGHT_MODE_DISABLE   0U
#define BSP_CAMERA_NIGHT_MODE_ENABLE    1U

/** @brief BSP_CAMERA_ConfigLineBuffer value for no wrapping or no line event. */
#define BSP_CAMERA_LINES_OFF            0U

/** @brief BSP_CAMERA_ConfigLineBuffer counts: a power of two from 1 to this. */
#define BSP_CAMERA_LINES_MAX            128U

/* Colour-filter order, first row's pair then second row's. A wrong order does not fail: it
 * swaps colours, easily mistaken for a white-balance fault. */
#define BSP_CAMERA_BAYER_RGGB           DCMIPP_RAWBAYER_RGGB
#define BSP_CAMERA_BAYER_GRBG           DCMIPP_RAWBAYER_GRBG
#define BSP_CAMERA_BAYER_GBRG           DCMIPP_RAWBAYER_GBRG
#define BSP_CAMERA_BAYER_BGGR           DCMIPP_RAWBAYER_BGGR

/** @brief The camera's current setup and features; rates are what the board sustains, not
  * datasheet maxima. */
typedef struct
{
  uint32_t Resolution;        /*!< Active resolution, a BSP_CAMERA_R* value.        */
  uint32_t PixelFormat;       /*!< Active sensor format, a BSP_CAMERA_PF_* value.   */
  uint32_t Width;             /*!< Sensor output width in pixels.                   */
  uint32_t Height;            /*!< Sensor output height in pixels.                  */
  uint32_t MaxFrameRate;      /*!< Frame-rate ceiling at this resolution, in fps.   */
  uint32_t LaneRateMbps;      /*!< Per-lane CSI-2 rate that picks the D-PHY bucket. */
  uint32_t PhyBitrateBucket;  /*!< The DCMIPP_CSI_PHY_BT_* value in force.          */
  uint32_t NeedsHalDPhyPatch; /*!< 1 if the stock HAL mis-calibrates this bucket.   */
  uint32_t MirrorFlip;        /*!< 1 when mirror and flip control is available.     */
  uint32_t PictureControls;   /*!< 1 when the sensor has the picture controls.      */
  uint32_t ResolutionChange;  /*!< 1 when BSP_CAMERA_SetResolution works in place.  */
} BSP_CAMERA_Capabilities_t;

#if defined(USE_HAL_DCMIPP_REGISTER_CALLBACKS) && (USE_HAL_DCMIPP_REGISTER_CALLBACKS == 1U)
/** @brief MSP routines for BSP_CAMERA_RegisterMspCallbacks(); the HAL calls them from
  * HAL_DCMIPP_Init() and HAL_DCMIPP_DeInit(). */
typedef struct
{
  void (* pMspInitCb)(DCMIPP_HandleTypeDef *);   /*!< Clocks and vectors up.   */
  void (* pMspDeInitCb)(DCMIPP_HandleTypeDef *); /*!< Clocks and vectors down. */
} BSP_CAMERA_Cb_t;
#endif /* USE_HAL_DCMIPP_REGISTER_CALLBACKS */

/** @brief Pipe destination format and size (Width x Height as written to memory). */
typedef struct
{
  uint32_t PixelFormat; /*!< Destination format, a BSP_CAMERA_PIPE_PF_* value.  */
  uint32_t Width;       /*!< Destination width in pixels.                       */
  uint32_t Height;      /*!< Destination height in pixels.                      */
  uint32_t CropEnable;  /*!< Non-zero to apply the crop rectangle. AUX only.    */
  uint32_t CropX;       /*!< Crop origin, pixels from the left edge.            */
  uint32_t CropY;       /*!< Crop origin, pixels from the top edge.             */
  uint32_t CropWidth;   /*!< Crop width in pixels.                              */
  uint32_t CropHeight;  /*!< Crop height in pixels.                             */
} BSP_CAMERA_PipeConfig_t;

/** @brief One sensor mode; rates are what the board sustains, not datasheet figures. */
typedef struct
{
  uint32_t Resolution;   /*!< Value the caller passes to BSP_CAMERA_Init. */
  uint16_t Width;        /*!< Frame width the sensor produces in this mode.  */
  uint16_t Height;       /*!< Frame height the sensor produces in this mode. */
  uint16_t MaxFrameRate; /*!< Sustained ceiling in frames a second.          */
  uint16_t LaneRateMbps; /*!< Per-lane rate; 0 keeps the default bucket.     */
} BSP_CAMERA_SensorMode_t;

/** @brief Sensor driver operations, each returning 0 on success. SetZoom may also return
  * BSP_ERROR_FEATURE_NOT_SUPPORTED, which BSP_CAMERA_SetZoom passes on. */
typedef struct
{
  int32_t (*Init)(void *pObj, uint32_t Resolution, uint32_t PixelFormat);
  int32_t (*DeInit)(void *pObj);
  int32_t (*ReadID)(void *pObj, uint32_t *pId);
  int32_t (*Start)(void *pObj);
  int32_t (*Stop)(void *pObj);
  int32_t (*SetMirrorFlip)(void *pObj, uint32_t MirrorFlip);
  int32_t (*SetBrightness)(void *pObj, int32_t Level);
  int32_t (*SetSaturation)(void *pObj, int32_t Level);
  int32_t (*SetContrast)(void *pObj, int32_t Level);
  int32_t (*SetHueDegree)(void *pObj, int32_t Degree);
  int32_t (*SetLightMode)(void *pObj, uint32_t LightMode);
  int32_t (*SetColorEffect)(void *pObj, uint32_t Effect);
  int32_t (*SetZoom)(void *pObj, uint32_t Zoom);
  int32_t (*SetNightMode)(void *pObj, uint32_t Enable);
  int32_t (*SetResolution)(void *pObj, uint32_t Resolution);
} BSP_CAMERA_SensorDrv_t;

/** @brief Sensor descriptor. BSP_CAMERA_RegisterSensor keeps a reference, so it must
  * outlive the camera (static storage, not a local). */
typedef struct
{
  const char *Name;        /*!< For diagnostics. Never interpreted.          */
  void       *pObj;        /*!< The driver's own state, passed back to it.   */
  const BSP_CAMERA_SensorDrv_t *pDrv; /*!< Operations above.                 */
  uint32_t    Id;          /*!< ReadID must match this or the probe fails.   */
  uint32_t    CsiDataType; /*!< A BSP_CAMERA_CSI_DT_* value.                 */
  uint32_t    SensorFormat;/*!< The one BSP_CAMERA_PF_* value Init accepts.  */
  uint32_t    BayerOrder;  /*!< A BSP_CAMERA_BAYER_* value, raw sensors only: the
                             *   starting order, which a driver that detects its own
                             *   corrects with BSP_CAMERA_SetIspDemosaicing. */
  uint8_t     SharedPipes; /*!< 1 if AUX takes its data from MAIN, not the link. */
  const BSP_CAMERA_SensorMode_t *pModes; /*!< Modes offered.                 */
  uint32_t    ModeCount;   /*!< How many.                                    */
  uint8_t     DataLanes;   /*!< CSI-2 data lanes the sensor drives, 1 or 2. */
  uint16_t    I2cAddress;  /*!< 8-bit bus address BSP_CAMERA_Init checks for an
                             *   answer first; 0 leaves identification to ReadID. */
} BSP_CAMERA_Sensor_t;

/** @brief The DCMIPP handle, exported for diagnostics (pipe state, error codes). */
extern DCMIPP_HandleTypeDef hcamera_dcmipp;

/**
  * @brief  Select the fitted sensor before BSP_CAMERA_Init (default: the OV5640 of the
  *         standard carrier). The descriptor is held by reference, not copied.
  * @retval BSP_ERROR_BUSY if the camera is already up.
  */
int32_t BSP_CAMERA_RegisterSensor(const BSP_CAMERA_Sensor_t *pSensor);

/** @brief The sensor descriptor in force; never NULL. */
const BSP_CAMERA_Sensor_t *BSP_CAMERA_GetSensor(void);

/**
  * @brief  Select the carrier (BSP_CAMERA_CARRIER_*) before BSP_CAMERA_Init; only needed
  *         by an application that runs on both.
  * @retval BSP_ERROR_BUSY if the camera is already up.
  */
int32_t BSP_CAMERA_SetCarrier(uint32_t Carrier);

/** @brief The carrier in force (BSP_CAMERA_CARRIER_*). */
uint32_t BSP_CAMERA_GetCarrier(void);

/**
  * @brief  Bring the camera up at a BSP_CAMERA_R* resolution and the sensor's
  *         BSP_CAMERA_PF_* format; the sensor is left idle.
  * @warning Success does not guarantee frames: occasionally (more often when XGA follows
  *         another mode) the link stays silent with no error. If a started pipe gets no
  *         frame within a few frame periods (the examples allow five, at least 150 ms),
  *         call BSP_CAMERA_DeInit and BSP_CAMERA_Init again.
  */
int32_t BSP_CAMERA_Init(uint32_t Instance, uint32_t Resolution, uint32_t PixelFormat);
int32_t BSP_CAMERA_DeInit(uint32_t Instance);

#if defined(USE_HAL_DCMIPP_REGISTER_CALLBACKS) && (USE_HAL_DCMIPP_REGISTER_CALLBACKS == 1U)
/** @brief Restore this module's MSP routines; BSP_CAMERA_Init() installs them itself the
  * first time. */
int32_t BSP_CAMERA_RegisterDefaultMspCallbacks(uint32_t Instance);

/** @brief Replace the MSP routines (neither may be NULL; called with &hcamera_dcmipp).
  * Register before the first BSP_CAMERA_Init(), which runs them. */
int32_t BSP_CAMERA_RegisterMspCallbacks(uint32_t Instance, BSP_CAMERA_Cb_t *Callbacks);
#endif /* USE_HAL_DCMIPP_REGISTER_CALLBACKS */

int32_t BSP_CAMERA_ConfigPipe(uint32_t Instance, uint32_t Pipe,
                              const BSP_CAMERA_PipeConfig_t *pConfig);

int32_t BSP_CAMERA_Start(uint32_t Instance, uint32_t Pipe, uint8_t *pBuf, uint32_t Mode);

/** @brief Capture into two buffers (pBuf1 != pBuf0) the DCMIPP swaps at every frame; not
  * for planar formats, which have their own double-buffered starts. */
int32_t BSP_CAMERA_DoubleBufferStart(uint32_t Instance, uint32_t Pipe,
                                     uint8_t *pBuf0, uint8_t *pBuf1, uint32_t Mode);

/** @brief Start MAIN in NV12: pBufY is Width x Height bytes, pBufUV (interleaved chroma)
  * half that. */
int32_t BSP_CAMERA_SemiPlanarStart(uint32_t Instance, uint32_t Pipe,
                                   uint8_t *pBufY, uint8_t *pBufUV, uint32_t Mode);

/** @brief Start MAIN in three-plane YUV420: pBufY is Width x Height bytes, pBufU and pBufV
  * a quarter of that each. */
int32_t BSP_CAMERA_FullPlanarStart(uint32_t Instance, uint32_t Pipe,
                                   uint8_t *pBufY, uint8_t *pBufU, uint8_t *pBufV,
                                   uint32_t Mode);

/** @brief BSP_CAMERA_SemiPlanarStart with two sets of planes, alternated. */
int32_t BSP_CAMERA_SemiPlanarDoubleBufferStart(uint32_t Instance, uint32_t Pipe,
                                               uint8_t *pBufY0, uint8_t *pBufUV0,
                                               uint8_t *pBufY1, uint8_t *pBufUV1,
                                               uint32_t Mode);

/** @brief BSP_CAMERA_FullPlanarStart with two sets of planes, alternated. */
int32_t BSP_CAMERA_FullPlanarDoubleBufferStart(uint32_t Instance, uint32_t Pipe,
                                               uint8_t *pBufY0, uint8_t *pBufU0, uint8_t *pBufV0,
                                               uint8_t *pBufY1, uint8_t *pBufU1, uint8_t *pBufV1,
                                               uint32_t Mode);

int32_t BSP_CAMERA_Stop(uint32_t Instance, uint32_t Pipe);
int32_t BSP_CAMERA_Suspend(uint32_t Instance, uint32_t Pipe);
int32_t BSP_CAMERA_Resume(uint32_t Instance, uint32_t Pipe);

int32_t BSP_CAMERA_SuspendSensor(uint32_t Instance);
int32_t BSP_CAMERA_ResumeSensor(uint32_t Instance);

/** @brief Stop all capture without waiting, from any context (e.g. a watchdog warning or
  * fault handler before a reset). BSP_ERROR_NONE if the camera was never up. */
int32_t BSP_CAMERA_HaltCapture(uint32_t Instance);

int32_t BSP_CAMERA_GetCapabilities(uint32_t Instance, BSP_CAMERA_Capabilities_t *pCapabilities);
int32_t BSP_CAMERA_GetFrameSize(uint32_t Instance, uint32_t Pipe, uint32_t *pSizeBytes);

int32_t BSP_CAMERA_SetMirrorFlip(uint32_t Instance, uint32_t MirrorFlip);
int32_t BSP_CAMERA_GetMirrorFlip(uint32_t Instance, uint32_t *pMirrorFlip);

/* Sensor picture controls: they act on the sensor, so every pipe sees them at once. */
int32_t BSP_CAMERA_SetBrightness(uint32_t Instance, int32_t Level);
int32_t BSP_CAMERA_GetBrightness(uint32_t Instance, int32_t *pLevel);
int32_t BSP_CAMERA_SetSaturation(uint32_t Instance, int32_t Level);
int32_t BSP_CAMERA_GetSaturation(uint32_t Instance, int32_t *pLevel);
int32_t BSP_CAMERA_SetContrast(uint32_t Instance, int32_t Level);
int32_t BSP_CAMERA_GetContrast(uint32_t Instance, int32_t *pLevel);
int32_t BSP_CAMERA_SetHueDegree(uint32_t Instance, int32_t Degree);
int32_t BSP_CAMERA_GetHueDegree(uint32_t Instance, int32_t *pDegree);
int32_t BSP_CAMERA_SetLightMode(uint32_t Instance, uint32_t LightMode);
int32_t BSP_CAMERA_GetLightMode(uint32_t Instance, uint32_t *pLightMode);
int32_t BSP_CAMERA_SetColorEffect(uint32_t Instance, uint32_t Effect);
int32_t BSP_CAMERA_GetColorEffect(uint32_t Instance, uint32_t *pEffect);
int32_t BSP_CAMERA_SetZoom(uint32_t Instance, uint32_t Zoom);
int32_t BSP_CAMERA_GetZoom(uint32_t Instance, uint32_t *pZoom);
int32_t BSP_CAMERA_EnableNightMode(uint32_t Instance);
int32_t BSP_CAMERA_DisableNightMode(uint32_t Instance);
int32_t BSP_CAMERA_GetNightMode(uint32_t Instance, uint32_t *pEnabled);

/** @brief Change resolution (a constant of the fitted sensor) without a re-init; every
  * pipe must then be reconfigured. BSP_ERROR_BUSY while a pipe runs or is suspended. */
int32_t BSP_CAMERA_SetResolution(uint32_t Instance, uint32_t Resolution);
int32_t BSP_CAMERA_GetResolution(uint32_t Instance, uint32_t *pResolution);

/** @brief The sensor's BSP_CAMERA_PF_* format, fixed by its descriptor (no Set). */
int32_t BSP_CAMERA_GetPixelFormat(uint32_t Instance, uint32_t *pPixelFormat);

/**
  * @brief  Make a pipe write a circular band of WrapLines lines, and/or raise an event
  *         every EventLines lines: each a power of two up to BSP_CAMERA_LINES_MAX, or
  *         BSP_CAMERA_LINES_OFF. Wrapping on MAIN or AUX only; events on any pipe.
  * @note   Call before starting the pipe (BSP_ERROR_BUSY if it is running). A band is
  *         never cleaned or invalidated, as the encoder reads it while the camera writes
  *         it, so it must be in non-cacheable memory.
  * @retval BSP_ERROR_FEATURE_NOT_SUPPORTED for wrapping on DUMP.
  */
int32_t BSP_CAMERA_ConfigLineBuffer(uint32_t Instance, uint32_t Pipe,
                                    uint32_t WrapLines, uint32_t EventLines);

/** @brief Demosaicing (raw to colour) on MAIN; NULL turns it off. Safe while running:
  * double-buffered, it applies from the next frame. BSP_ERROR_FEATURE_NOT_SUPPORTED on a
  * pipe without the stage. */
int32_t BSP_CAMERA_SetIspDemosaicing(uint32_t Instance, uint32_t Pipe,
                                     const DCMIPP_RawBayer2RGBConfTypeDef *pConfig);

/** @brief Black-level subtraction (the dark-current pedestal) on MAIN; NULL turns it off.
  * As BSP_CAMERA_SetIspDemosaicing. */
int32_t BSP_CAMERA_SetIspBlackLevel(uint32_t Instance, uint32_t Pipe,
                                    const DCMIPP_BlackLevelConfTypeDef *pConfig);

/** @brief Per-channel gain on MAIN: exposure overall, white balance by ratio. NULL turns
  * it off. As BSP_CAMERA_SetIspDemosaicing. */
int32_t BSP_CAMERA_SetIspGain(uint32_t Instance, uint32_t Pipe,
                              const DCMIPP_ExposureConfTypeDef *pConfig);

/** @brief 3x3 colour-conversion matrix with offsets, MAIN only (not the fixed YUV422
  * conversion BSP_CAMERA_ConfigPipe sets); NULL turns it off. As
  * BSP_CAMERA_SetIspDemosaicing. */
int32_t BSP_CAMERA_SetIspColorConversion(uint32_t Instance, uint32_t Pipe,
                                         const DCMIPP_ColorConversionConfTypeDef *pConfig);

/** @brief Fixed-curve gamma on or off, on MAIN or AUX (the one ISP stage AUX has). */
int32_t BSP_CAMERA_SetIspGamma(uint32_t Instance, uint32_t Pipe, uint32_t Enable);

/** @brief Every EventLines lines (BSP_CAMERA_ConfigLineBuffer); interrupt context. */
void BSP_CAMERA_LineEventCallback(uint32_t Instance, uint32_t Pipe);

/** @brief A frame is complete; interrupt context. With USE_BSP_CPU_CACHE_MAINTENANCE its
  * planes are already invalidated, except a wrapping pipe's band. */
void BSP_CAMERA_FrameEventCallback(uint32_t Instance, uint32_t Pipe);

/** @brief A frame has started; interrupt context. Weak, empty. */
void BSP_CAMERA_VsyncEventCallback(uint32_t Instance, uint32_t Pipe);

/** @brief A capture error, in interrupt context; Pipe is BSP_CAMERA_PIPE_NONE for a fault
  * not tied to one pipe. hcamera_dcmipp.ErrorCode has the HAL's classification. */
void BSP_CAMERA_ErrorCallback(uint32_t Instance, uint32_t Pipe);

/** @brief Call from DCMIPP_IRQHandler. */
void BSP_CAMERA_DCMIPP_IRQHandler(uint32_t Instance);

/** @brief Call from CSI_IRQHandler. */
void BSP_CAMERA_CSI_IRQHandler(uint32_t Instance);

/** @brief Alias of BSP_CAMERA_DCMIPP_IRQHandler, for code written for other ST BSPs. It
  * does not cover the CSI vector: install both handlers. */
void BSP_CAMERA_IRQHandler(uint32_t Instance);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_CAMERA_H */
