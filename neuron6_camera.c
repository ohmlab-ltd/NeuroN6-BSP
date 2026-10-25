/**
  * @file    neuron6_camera.c
  * @brief   Camera capture on the Neuro N6: OV5640 over MIPI CSI-2 into the DCMIPP.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6_camera.h"
#include "neuron6.h"
#include "neuron6_bus.h"
#include "neuron6_venc.h"

/* The sensor is on I2C1, reached only through the bus module. */

/* SYSTEM_CTRL0, for the soft reset. */
#define CAMERA_REG_SYSTEM_CTRL0         0x3008U
/* Reset asserted: held in reset, powered down. */
#define CAMERA_SYSCTRL_RESET            0x82U
/* Reset released: configured, not streaming. */
#define CAMERA_SYSCTRL_IDLE             0x42U
#define CAMERA_RESET_ASSERT_DELAY_MS    50U
#define CAMERA_RESET_RELEASE_DELAY_MS   10U
#define CAMERA_PROBE_TRIALS             3U
/* Longest wait for a stopping pipe to go idle, as the HAL allows its own stop. */
#define CAMERA_PIPE_STOP_TIMEOUT_MS     1000U

/* MIPI_CTRL00 lane count. The driver's set-up writes 0x45 (two lanes), so the one-lane
 * value for the small modes is written after it. */
#define CAMERA_REG_MIPI_CTRL00          0x300EU
#define CAMERA_MIPI_TWO_LANES           0x45U
#define CAMERA_MIPI_ONE_LANE            0x24U
#define CAMERA_MIPI_TWO_LANE_FROM       OV5640_R640x480

/* DCMIPP kernel clock: PLL1 via IC17. CSI (IC18) is the D-PHY configuration clock and
 * must be near 27 MHz; CAMERA_ClockConfig re-derives its divider if it misses. */
#define CAMERA_IC17_DIV_DCMIPP          2U
#define CAMERA_IC18_DIV_CSI             30U
#define CAMERA_CSI_KERNEL_TARGET_HZ     27000000U
#define CAMERA_CSI_KERNEL_MIN_HZ        24000000U
#define CAMERA_CSI_KERNEL_MAX_HZ        30000000U
#define CAMERA_IC_DIV_MAX               256U

/* Lowest D-PHY bucket in the range the stock HAL mis-calibrates. */
#define CAMERA_DPHY_PATCH_THRESHOLD     DCMIPP_CSI_PHY_BT_800

#define CAMERA_DOWNSIZE_RATIO_MAX       65535U
#define CAMERA_DOWNSIZE_DIVFACTOR_MIN   0x80U
#define CAMERA_DOWNSIZE_DIVFACTOR_MAX   0x3FFU
#define CAMERA_DOWNSIZE_SIZE_MAX        0xFFFU
#define CAMERA_DECIMATION_MAX           8U

typedef enum
{
  CAMERA_PIPE_UNCONFIGURED = 0, /*!< No geometry programmed yet.        */
  CAMERA_PIPE_STOPPED,          /*!< Configured, not capturing.         */
  CAMERA_PIPE_RUNNING,          /*!< Capturing.                         */
  CAMERA_PIPE_SUSPENDED         /*!< Capture request withdrawn, armed.  */
} CAMERA_PipeState_t;

/* Three planes, double-buffered. */
#define CAMERA_REGIONS_MAX              6U

/* A destination plane, as the frame event invalidates it. */
typedef struct
{
  uint8_t           *pAddr;      /*!< First byte of the plane.              */
  uint32_t           Bytes;      /*!< Bytes the pipe writes there a frame.  */
} CAMERA_Region_t;

typedef struct
{
  CAMERA_PipeState_t State;      /*!< Life-cycle position.                  */
  uint32_t           PixelFormat;/*!< BSP_CAMERA_PIPE_PF_* in force.        */
  uint32_t           Width;      /*!< Destination width in pixels.          */
  uint32_t           Height;     /*!< Destination height in pixels.         */
  uint32_t           FrameBytes; /*!< Bytes the pipe writes per frame.      */
  uint8_t           *pBuf;       /*!< Destination handed to Start.          */
  CAMERA_Region_t    Region[CAMERA_REGIONS_MAX]; /*!< Planes of the last start. */
  uint32_t           Regions;    /*!< How many of Region are in use.        */
  uint32_t         LineWrapped;  /*!< Non-zero while writing a wrapped band. */
  uint32_t           LineEventOn;  /*!< Non-zero while line events are asked for. */
  uint32_t           LineEventCode;/*!< The HAL's code for their spacing.   */
} CAMERA_PipeCtx_t;

typedef struct
{
  uint32_t         Initialised;  /*!< Non-zero once Init has succeeded.     */
  uint32_t         Resolution;   /*!< BSP_CAMERA_R* in force.               */
  uint32_t         PixelFormat;  /*!< BSP_CAMERA_PF_* in force.             */
  uint32_t         Width;        /*!< Sensor output width in pixels.        */
  uint32_t         Height;       /*!< Sensor output height in pixels.       */
  uint32_t         MaxFrameRate; /*!< Ceiling for Resolution, fps.          */
  uint32_t         LaneRateMbps; /*!< Rate fed to the bucket ladder.        */
  uint32_t         PhyBucket;    /*!< DCMIPP_CSI_PHY_BT_* selected.         */
  uint32_t         MirrorFlip;   /*!< BSP_CAMERA_MIRRORFLIP_* in force.     */
  int32_t          Brightness;   /*!< Last level accepted by the sensor.    */
  int32_t          Saturation;   /*!< Last level accepted by the sensor.    */
  int32_t          Contrast;     /*!< Last level accepted by the sensor.    */
  int32_t          HueDegree;    /*!< Last degree accepted by the sensor.   */
  uint32_t         LightMode;    /*!< BSP_CAMERA_LIGHT_* in force.          */
  uint32_t         ColorEffect;  /*!< BSP_CAMERA_COLOR_EFFECT_* in force.   */
  uint32_t         Zoom;         /*!< BSP_CAMERA_ZOOM_* in force.           */
  uint32_t         NightMode;    /*!< BSP_CAMERA_NIGHT_MODE_* in force.     */
  uint32_t         SensorOn;     /*!< Non-zero while the sensor streams.    */
  CAMERA_PipeCtx_t Pipe[BSP_CAMERA_PIPES_NBR];
} CAMERA_Ctx_t;

DCMIPP_HandleTypeDef hcamera_dcmipp;

static OV5640_Object_t Camera_CompObj;
static CAMERA_Ctx_t    Camera_Ctx[BSP_CAMERA_INSTANCES_NBR];

/* Kernel-clock result from the MSP init, which cannot return one; BSP_CAMERA_Init checks
 * it after HAL_DCMIPP_Init. */
static int32_t Camera_ClockStatus = BSP_ERROR_NONE;

/* MAIN's demosaic settings as last programmed, kept so a later mirror or flip can fix the
 * filter order without undoing the application's other settings. */
static DCMIPP_RawBayer2RGBConfTypeDef Camera_Demosaic;
static uint8_t                        Camera_DemosaicOn;

#if defined(USE_HAL_DCMIPP_REGISTER_CALLBACKS) && (USE_HAL_DCMIPP_REGISTER_CALLBACKS == 1U)
/* Set once MSP routines are on the handle, so Init never puts this module's over an
 * application's. Survives DeInit, as the HAL's registration does. */
static uint8_t Camera_MspCbValid;
#endif /* USE_HAL_DCMIPP_REGISTER_CALLBACKS */

/* Read once, as the receiver is configured. */
static uint32_t Camera_Carrier = BSP_CAMERA_DEFAULT_CARRIER;

/* The OV5640's start-up orientation; see BSP_CAMERA_DEFAULT_MIRRORFLIP. */
static uint32_t CAMERA_Ov5640MirrorFlip(void)
{
#if defined(BSP_CAMERA_DEFAULT_MIRRORFLIP)
  return BSP_CAMERA_DEFAULT_MIRRORFLIP;
#else
  return (Camera_Carrier == BSP_CAMERA_CARRIER_STANDARD) ? BSP_CAMERA_MIRRORFLIP_FLIP
                                                         : BSP_CAMERA_MIRRORFLIP_NONE;
#endif
}

/* OV5640 modes validated on this board; 15 fps is the sensor's 5 MP maximum. */
static const BSP_CAMERA_SensorMode_t Camera_Ov5640Modes[] =
{
  { BSP_CAMERA_R320x240,    320U,  240U, 60U,   0U },
  { BSP_CAMERA_R640x480,    640U,  480U, 60U,   0U },
  { BSP_CAMERA_R800x480,    800U,  480U, 60U,   0U },
  { BSP_CAMERA_R1024x768,  1024U,  768U, 30U, 768U },
  { BSP_CAMERA_R1280x720,  1280U,  720U, 60U, 592U },
  { BSP_CAMERA_R1920x1080, 1920U, 1080U, 30U, 592U },
  { BSP_CAMERA_R2592x1944, 2592U, 1944U, 15U,   0U },
};

#define CAMERA_OV5640_MODES_NBR \
  (sizeof(Camera_Ov5640Modes) / sizeof(Camera_Ov5640Modes[0]))

static int32_t Camera_Ov5640_Init(void *pObj, uint32_t Resolution, uint32_t PixelFormat);
static int32_t Camera_Ov5640_DeInit(void *pObj);
static int32_t Camera_Ov5640_ReadID(void *pObj, uint32_t *pId);
static int32_t Camera_Ov5640_Start(void *pObj);
static int32_t Camera_Ov5640_Stop(void *pObj);
static int32_t Camera_Ov5640_SetMirrorFlip(void *pObj, uint32_t MirrorFlip);
static int32_t Camera_Ov5640_SetBrightness(void *pObj, int32_t Level);
static int32_t Camera_Ov5640_SetSaturation(void *pObj, int32_t Level);
static int32_t Camera_Ov5640_SetContrast(void *pObj, int32_t Level);
static int32_t Camera_Ov5640_SetHueDegree(void *pObj, int32_t Degree);
static int32_t Camera_Ov5640_SetLightMode(void *pObj, uint32_t LightMode);
static int32_t Camera_Ov5640_SetColorEffect(void *pObj, uint32_t Effect);
static int32_t Camera_Ov5640_SetZoom(void *pObj, uint32_t Zoom);
static int32_t Camera_Ov5640_SetNightMode(void *pObj, uint32_t Enable);
static int32_t Camera_Ov5640_SetResolution(void *pObj, uint32_t Resolution);

static const BSP_CAMERA_SensorDrv_t Camera_Ov5640Drv =
{
  Camera_Ov5640_Init,
  Camera_Ov5640_DeInit,
  Camera_Ov5640_ReadID,
  Camera_Ov5640_Start,
  Camera_Ov5640_Stop,
  Camera_Ov5640_SetMirrorFlip,
  Camera_Ov5640_SetBrightness,
  Camera_Ov5640_SetSaturation,
  Camera_Ov5640_SetContrast,
  Camera_Ov5640_SetHueDegree,
  Camera_Ov5640_SetLightMode,
  Camera_Ov5640_SetColorEffect,
  Camera_Ov5640_SetZoom,
  Camera_Ov5640_SetNightMode,
  Camera_Ov5640_SetResolution,
};

static const BSP_CAMERA_Sensor_t Camera_Ov5640Sensor =
{
  "OV5640",
  NULL,                          /* The object is a file-scope singleton. */
  &Camera_Ov5640Drv,
  BSP_CAMERA_OV5640_ID,
  BSP_CAMERA_CSI_DT_RGB565,
  BSP_CAMERA_PF_RGB565,
  BSP_CAMERA_BAYER_RGGB,         /* Unused: this sensor sends colour. */
  0U,                            /* Every pipe can take the link directly. */
  Camera_Ov5640Modes,
  CAMERA_OV5640_MODES_NBR,
  2U,                            /* Two CSI-2 data lanes. */
  (uint16_t)BSP_CAMERA_I2C_ADDRESS,
};

/* The OV5640 (standard carrier) until BSP_CAMERA_RegisterSensor says otherwise. */
static const BSP_CAMERA_Sensor_t *Camera_Sensor = &Camera_Ov5640Sensor;

#define CAMERA_LINE_CODES_NBR   8U

/* Both tables are indexed by log2 of the line count (1 to 128 lines). */
static const uint32_t Camera_WrapCode[CAMERA_LINE_CODES_NBR] =
{
  DCMIPP_WRAP_ADDRESS_1_LINE,   DCMIPP_WRAP_ADDRESS_2_LINES,
  DCMIPP_WRAP_ADDRESS_4_LINES,  DCMIPP_WRAP_ADDRESS_8_LINES,
  DCMIPP_WRAP_ADDRESS_16_LINES, DCMIPP_WRAP_ADDRESS_32_LINES,
  DCMIPP_WRAP_ADDRESS_64_LINES, DCMIPP_WRAP_ADDRESS_128_LINES,
};

static const uint32_t Camera_LineEventCode[CAMERA_LINE_CODES_NBR] =
{
  DCMIPP_MULTILINE_1_LINE,   DCMIPP_MULTILINE_2_LINES,
  DCMIPP_MULTILINE_4_LINES,  DCMIPP_MULTILINE_8_LINES,
  DCMIPP_MULTILINE_16_LINES, DCMIPP_MULTILINE_32_LINES,
  DCMIPP_MULTILINE_64_LINES, DCMIPP_MULTILINE_128_LINES,
};

static int32_t  CAMERA_IO_Init(void);
static int32_t  CAMERA_IO_DeInit(void);
static int32_t  CAMERA_IO_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
static int32_t  CAMERA_IO_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
static int32_t  CAMERA_IO_GetTick(void);

static const BSP_CAMERA_SensorMode_t *CAMERA_FindMode(uint32_t Resolution);
static uint32_t CAMERA_PhyBucket(uint32_t LaneRateMbps);
static void     CAMERA_RIFConfig(void);
static void     CAMERA_ClockConfig(void);
static int32_t  CAMERA_CsiConfig(uint32_t PhyBucket);
static int32_t  CAMERA_ProbeSensor(uint32_t Resolution, uint32_t PixelFormat);
static void     CAMERA_ClearStaleErrorState(void);
static uint8_t  CAMERA_HalPipeHeld(uint32_t Pipe);
static int32_t  CAMERA_LineCountIndex(uint32_t Lines);
static int32_t  CAMERA_CheckIspPipe(uint32_t Instance, uint32_t Pipe, uint32_t AuxAllowed);
static void     CAMERA_PipeAssertDataType(uint32_t Pipe, uint32_t DataType);
static int32_t  CAMERA_PipeCsiSource(uint32_t Pipe, uint32_t DataType);
static int32_t  CAMERA_PipeCoreConfig(uint32_t Pipe, uint32_t Pitch, uint32_t PackerFormat);
static int32_t  CAMERA_ConfigDump(const BSP_CAMERA_PipeConfig_t *pConfig, CAMERA_PipeCtx_t *pCtx);
static int32_t  CAMERA_ConfigMain(const BSP_CAMERA_PipeConfig_t *pConfig, CAMERA_PipeCtx_t *pCtx);
static int32_t  CAMERA_ConfigAux(const BSP_CAMERA_PipeConfig_t *pConfig, CAMERA_PipeCtx_t *pCtx);
static void     CAMERA_InvalidateFrame(const CAMERA_PipeCtx_t *pCtx);
static uint32_t CAMERA_PlaneBytes(const CAMERA_PipeCtx_t *pCtx, uint32_t Plane);
static void     CAMERA_SetRegions(CAMERA_PipeCtx_t *pCtx, uint8_t *const *ppPlane,
                                  uint32_t Planes, uint32_t Buffers);
static void     CAMERA_PrepareStart(uint32_t Pipe);
static int32_t  CAMERA_StopPipeOnly(uint32_t Pipe);
static int32_t  CAMERA_FinishStart(CAMERA_Ctx_t *ctx, uint32_t Pipe);
static uint32_t CAMERA_AnyPipeRunning(const CAMERA_Ctx_t *pCtx);

/* Sensor bus: SCCB, i.e. I2C with 16-bit register addresses. */

/* The bus is already up. */
static int32_t CAMERA_IO_Init(void)
{
  return OV5640_OK;
}

/* No-op: I2C1 is shared with other on-board devices. */
static int32_t CAMERA_IO_DeInit(void)
{
  return OV5640_OK;
}

static int32_t CAMERA_IO_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  return (BSP_I2C1_WriteReg16(DevAddr, Reg, pData, Length) == BSP_ERROR_NONE) ? OV5640_OK : OV5640_ERROR;
}

static int32_t CAMERA_IO_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  return (BSP_I2C1_ReadReg16(DevAddr, Reg, pData, Length) == BSP_ERROR_NONE) ? OV5640_OK : OV5640_ERROR;
}

static int32_t CAMERA_IO_GetTick(void)
{
  return (int32_t)HAL_GetTick();
}

/* Mode and D-PHY selection */

static const BSP_CAMERA_SensorMode_t *CAMERA_FindMode(uint32_t Resolution)
{
  uint32_t i;

  for (i = 0U; i < Camera_Sensor->ModeCount; i++)
  {
    if (Camera_Sensor->pModes[i].Resolution == Resolution)
    {
      return &Camera_Sensor->pModes[i];
    }
  }

  return NULL;
}

/* Rounds up: the bucket must be at or above the lane rate (0 = default bucket). */
static uint32_t CAMERA_PhyBucket(uint32_t LaneRateMbps)
{
  uint32_t bucket;

  if (LaneRateMbps == 0U)
  {
    bucket = DCMIPP_CSI_PHY_BT_500;
  }
  else if (LaneRateMbps <= 400U)
  {
    bucket = DCMIPP_CSI_PHY_BT_400;
  }
  else if (LaneRateMbps <= 500U)
  {
    bucket = DCMIPP_CSI_PHY_BT_500;
  }
  else if (LaneRateMbps <= 800U)
  {
    bucket = DCMIPP_CSI_PHY_BT_800;
  }
  else if (LaneRateMbps <= 850U)
  {
    bucket = DCMIPP_CSI_PHY_BT_850;
  }
  else if (LaneRateMbps <= 900U)
  {
    bucket = DCMIPP_CSI_PHY_BT_900;
  }
  else if (LaneRateMbps <= 1000U)
  {
    bucket = DCMIPP_CSI_PHY_BT_1000;
  }
  else if (LaneRateMbps <= 1200U)
  {
    bucket = DCMIPP_CSI_PHY_BT_1200;
  }
  else if (LaneRateMbps <= 1350U)
  {
    bucket = DCMIPP_CSI_PHY_BT_1350;
  }
  else
  {
    bucket = DCMIPP_CSI_PHY_BT_1500;
  }

  return bucket;
}

/* Platform bring-up */

/* Grant DCMIPP and CSI their RIF rights: the N6 silently drops transactions from an
 * unconfigured bus master (the buffer just never fills). */
static void CAMERA_RIFConfig(void)
{
  RIMC_MasterConfig_t master = {0};

  /* Board-wide isolation first. */
  (void)BSP_ISOLATION_Config();

  /* Via LL: the HAL's RIFSC clock macro exists only in secure builds. */
  LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_RIFSC);

  master.MasterCID = RIF_CID_1;
  master.SecPriv   = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;

  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_DCMIPP, &master);

  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_DCMIPP,
                                        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_CSI,
                                        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
}

/* Both kernel clocks come from PLL1, which is the FSBL's and must not be touched here. */
static void CAMERA_ClockConfig(void)
{
  RCC_PeriphCLKInitTypeDef periph = {0};
  uint32_t csi_hz;
  uint32_t pll1_hz;
  uint32_t divider;

  Camera_ClockStatus = BSP_ERROR_NONE;

  periph.PeriphClockSelection                    = RCC_PERIPHCLK_DCMIPP;
  periph.DcmippClockSelection                    = RCC_DCMIPPCLKSOURCE_IC17;
  periph.ICSelection[RCC_IC17].ClockSelection    = RCC_ICCLKSOURCE_PLL1;
  periph.ICSelection[RCC_IC17].ClockDivider      = CAMERA_IC17_DIV_DCMIPP;
  if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK)
  {
    Camera_ClockStatus = BSP_ERROR_CLOCK_FAILURE;
    return;
  }

  periph.PeriphClockSelection                 = RCC_PERIPHCLK_CSI;
  periph.ICSelection[RCC_IC18].ClockSelection = RCC_ICCLKSOURCE_PLL1;
  periph.ICSelection[RCC_IC18].ClockDivider   = CAMERA_IC18_DIV_CSI;
  if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK)
  {
    Camera_ClockStatus = BSP_ERROR_CLOCK_FAILURE;
    return;
  }

  csi_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_CSI);
  if (csi_hz == 0U)
  {
    Camera_ClockStatus = BSP_ERROR_CLOCK_FAILURE;
    return;
  }

  if ((csi_hz < CAMERA_CSI_KERNEL_MIN_HZ) || (csi_hz > CAMERA_CSI_KERNEL_MAX_HZ))
  {
    /* This FSBL's PLL1 rate does not suit the default divider: re-derive it. */
    pll1_hz = csi_hz * CAMERA_IC18_DIV_CSI;
    divider = (pll1_hz + (CAMERA_CSI_KERNEL_TARGET_HZ / 2U)) / CAMERA_CSI_KERNEL_TARGET_HZ;

    if (divider < 1U)
    {
      divider = 1U;
    }
    if (divider > CAMERA_IC_DIV_MAX)
    {
      divider = CAMERA_IC_DIV_MAX;
    }

    periph.PeriphClockSelection                 = RCC_PERIPHCLK_CSI;
    periph.ICSelection[RCC_IC18].ClockSelection = RCC_ICCLKSOURCE_PLL1;
    periph.ICSelection[RCC_IC18].ClockDivider   = divider;
    if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK)
    {
      Camera_ClockStatus = BSP_ERROR_CLOCK_FAILURE;
      return;
    }

    csi_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_CSI);
    if ((csi_hz < CAMERA_CSI_KERNEL_MIN_HZ) || (csi_hz > CAMERA_CSI_KERNEL_MAX_HZ))
    {
      /* No integer divider of this PLL1 reaches the D-PHY configuration clock window. */
      Camera_ClockStatus = BSP_ERROR_FEATURE_NOT_SUPPORTED;
    }
  }
}

static int32_t CAMERA_CsiConfig(uint32_t PhyBucket)
{
  DCMIPP_CSI_ConfTypeDef csi = {0};

  csi.NumberOfLanes   = (Camera_Sensor->DataLanes == 1U)
                        ? DCMIPP_CSI_ONE_DATA_LANE
                        : DCMIPP_CSI_TWO_DATA_LANES;
  csi.DataLaneMapping = (Camera_Carrier == BSP_CAMERA_CARRIER_INVERTED)
                        ? DCMIPP_CSI_INVERTED_DATA_LANES
                        : DCMIPP_CSI_PHYSICAL_DATA_LANES;
  csi.PHYBitrate      = PhyBucket;

  if (HAL_DCMIPP_CSI_SetConfig(&hcamera_dcmipp, &csi) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* The D-PHY is not waited for: see BSP_CAMERA_Init for a bring-up with no frames. */

  if (HAL_DCMIPP_CSI_SetVCConfig(&hcamera_dcmipp, DCMIPP_VIRTUAL_CHANNEL0,
                                 DCMIPP_CSI_DT_BPP8) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return BSP_ERROR_NONE;
}

/* No camera reset or power-down line: RESETB and PWDN are tied off on the carrier. */
static int32_t CAMERA_ProbeSensor(uint32_t Resolution, uint32_t PixelFormat)
{
  uint32_t id = 0U;

  /* Identify first, so a missing or wrong part fails before anything is configured. */
  if (Camera_Sensor->pDrv->ReadID(Camera_Sensor->pObj, &id) != 0)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  if (id != Camera_Sensor->Id)
  {
    return BSP_ERROR_UNKNOWN_COMPONENT;
  }

  if (Camera_Sensor->pDrv->Init(Camera_Sensor->pObj, Resolution, PixelFormat) != 0)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  return BSP_ERROR_NONE;
}

/* OV5640 adapter (the standard carrier's sensor) */

/* Bind the driver and soft-reset the sensor. The reset is required: a sensor left
 * streaming by a warm reset identifies correctly, then misbehaves once configured. */
static int32_t Camera_Ov5640_Bind(void)
{
  OV5640_IO_t io;
  uint8_t     value;

  io.Init     = CAMERA_IO_Init;
  io.DeInit   = CAMERA_IO_DeInit;
  io.Address  = (uint16_t)BSP_CAMERA_I2C_ADDRESS;
  io.WriteReg = CAMERA_IO_WriteReg;
  io.ReadReg  = CAMERA_IO_ReadReg;
  io.GetTick  = CAMERA_IO_GetTick;

  Camera_CompObj.Mode             = SERIAL_MODE;
  Camera_CompObj.VirtualChannelID = DCMIPP_VIRTUAL_CHANNEL0;

  if (OV5640_RegisterBusIO(&Camera_CompObj, &io) != OV5640_OK)
  {
    return -1;
  }

  value = CAMERA_SYSCTRL_RESET;
  if (CAMERA_IO_WriteReg((uint16_t)BSP_CAMERA_I2C_ADDRESS, CAMERA_REG_SYSTEM_CTRL0,
                         &value, 1U) != OV5640_OK)
  {
    return -1;
  }
  HAL_Delay(CAMERA_RESET_ASSERT_DELAY_MS);

  value = CAMERA_SYSCTRL_IDLE;
  if (CAMERA_IO_WriteReg((uint16_t)BSP_CAMERA_I2C_ADDRESS, CAMERA_REG_SYSTEM_CTRL0,
                         &value, 1U) != OV5640_OK)
  {
    return -1;
  }
  HAL_Delay(CAMERA_RESET_RELEASE_DELAY_MS);

  return 0;
}

/* Also binds the bus and soft-resets the sensor. */
static int32_t Camera_Ov5640_ReadID(void *pObj, uint32_t *pId)
{
  UNUSED(pObj);

  if (Camera_Ov5640_Bind() != 0)
  {
    return -1;
  }

  return (OV5640_ReadID(&Camera_CompObj, pId) == OV5640_OK) ? 0 : -1;
}

static int32_t Camera_Ov5640_Init(void *pObj, uint32_t Resolution, uint32_t PixelFormat)
{
  uint8_t lanes;

  UNUSED(pObj);

  if (OV5640_Init(&Camera_CompObj, Resolution, PixelFormat) != OV5640_OK)
  {
    return -1;
  }
  /* After the mode program, which writes the two-lane value itself. */
  lanes = (Resolution >= CAMERA_MIPI_TWO_LANE_FROM) ? CAMERA_MIPI_TWO_LANES
                                                    : CAMERA_MIPI_ONE_LANE;
  if ((CAMERA_IO_WriteReg((uint16_t)BSP_CAMERA_I2C_ADDRESS, CAMERA_REG_MIPI_CTRL00,
                          &lanes, 1U) != OV5640_OK) ||
      (OV5640_MirrorFlipConfig(&Camera_CompObj, CAMERA_Ov5640MirrorFlip()) != OV5640_OK) ||
      (OV5640_Stop(&Camera_CompObj) != OV5640_OK))
  {
    /* Else the driver, counting itself initialised, would skip programming the sensor the
     * retry has just reset. */
    (void)OV5640_DeInit(&Camera_CompObj);
    return -1;
  }

  /* Left idle: the receiver comes up before the sensor may send (see BSP_CAMERA_Init). */
  return 0;
}

static int32_t Camera_Ov5640_DeInit(void *pObj)
{
  UNUSED(pObj);

  return (OV5640_DeInit(&Camera_CompObj) == OV5640_OK) ? 0 : -1;
}

static int32_t Camera_Ov5640_Start(void *pObj)
{
  UNUSED(pObj);

  return (OV5640_Start(&Camera_CompObj) == OV5640_OK) ? 0 : -1;
}

static int32_t Camera_Ov5640_Stop(void *pObj)
{
  UNUSED(pObj);

  return (OV5640_Stop(&Camera_CompObj) == OV5640_OK) ? 0 : -1;
}

static int32_t Camera_Ov5640_SetMirrorFlip(void *pObj, uint32_t MirrorFlip)
{
  UNUSED(pObj);

  return (OV5640_MirrorFlipConfig(&Camera_CompObj, MirrorFlip) == OV5640_OK) ? 0 : -1;
}

/* The driver indexes a nine-entry table with Level + 4, unchecked: callers of these four
 * must range-check first. */
static int32_t Camera_Ov5640_SetBrightness(void *pObj, int32_t Level)
{
  UNUSED(pObj);

  return (OV5640_SetBrightness(&Camera_CompObj, Level) == OV5640_OK) ? 0 : -1;
}

static int32_t Camera_Ov5640_SetSaturation(void *pObj, int32_t Level)
{
  UNUSED(pObj);

  return (OV5640_SetSaturation(&Camera_CompObj, Level) == OV5640_OK) ? 0 : -1;
}

static int32_t Camera_Ov5640_SetContrast(void *pObj, int32_t Level)
{
  UNUSED(pObj);

  return (OV5640_SetContrast(&Camera_CompObj, Level) == OV5640_OK) ? 0 : -1;
}

/* Degree is in 30-degree steps. */
static int32_t Camera_Ov5640_SetHueDegree(void *pObj, int32_t Degree)
{
  UNUSED(pObj);

  return (OV5640_SetHueDegree(&Camera_CompObj, Degree) == OV5640_OK) ? 0 : -1;
}

static int32_t Camera_Ov5640_SetLightMode(void *pObj, uint32_t LightMode)
{
  UNUSED(pObj);

  return (OV5640_SetLightMode(&Camera_CompObj, LightMode) == OV5640_OK) ? 0 : -1;
}

static int32_t Camera_Ov5640_SetColorEffect(void *pObj, uint32_t Effect)
{
  UNUSED(pObj);

  return (OV5640_SetColorEffect(&Camera_CompObj, Effect) == OV5640_OK) ? 0 : -1;
}

/* The zoom uses the sensor's scaler, which 720p and above do not run: only x1 there. */
static int32_t Camera_Ov5640_SetZoom(void *pObj, uint32_t Zoom)
{
  uint32_t res;

  UNUSED(pObj);

  if (OV5640_GetResolution(&Camera_CompObj, &res) != OV5640_OK)
  {
    return -1;
  }

  if ((Zoom != OV5640_ZOOM_x1) &&
      ((res == OV5640_R1280x720) || (res == OV5640_R1920x1080) || (res == OV5640_R2592x1944)))
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  return (OV5640_ZoomConfig(&Camera_CompObj, Zoom) == OV5640_OK) ? 0 : -1;
}

static int32_t Camera_Ov5640_SetNightMode(void *pObj, uint32_t Enable)
{
  const uint32_t cmd = (Enable == BSP_CAMERA_NIGHT_MODE_ENABLE) ? ENABLE : DISABLE;

  UNUSED(pObj);

  return (OV5640_NightModeConfig(&Camera_CompObj, cmd) == OV5640_OK) ? 0 : -1;
}

/* BSP_CAMERA_R* are the driver's own resolution constants. */
static int32_t Camera_Ov5640_SetResolution(void *pObj, uint32_t Resolution)
{
  UNUSED(pObj);

  return (OV5640_SetResolution(&Camera_CompObj, Resolution) == OV5640_OK) ? 0 : -1;
}

/* Pipe configuration helpers */

/* A CSI error interrupt (a marginal link at high resolution) leaves the HAL in error. */
static void CAMERA_ClearStaleErrorState(void)
{
  if (hcamera_dcmipp.State == HAL_DCMIPP_STATE_ERROR)
  {
    hcamera_dcmipp.State     = HAL_DCMIPP_STATE_READY;
    hcamera_dcmipp.ErrorCode = HAL_DCMIPP_ERROR_NONE;
  }
}

/* 1 if the HAL would refuse to start the pipe. */
static uint8_t CAMERA_HalPipeHeld(uint32_t Pipe)
{
  return ((hcamera_dcmipp.PipeState[Pipe] == HAL_DCMIPP_PIPE_STATE_BUSY) ||
          (hcamera_dcmipp.PipeState[Pipe] == HAL_DCMIPP_PIPE_STATE_ERROR)) ? 1U : 0U;
}

/* The line registers take a 3-bit exponent: 1 to 128 lines, powers of two; else -1. */
static int32_t CAMERA_LineCountIndex(uint32_t Lines)
{
  uint32_t index;

  if ((Lines == 0U) || (Lines > BSP_CAMERA_LINES_MAX))
  {
    return -1;
  }

  if ((Lines & (Lines - 1U)) != 0U)
  {
    return -1;
  }

  for (index = 0U; index < CAMERA_LINE_CODES_NBR; index++)
  {
    if ((1UL << index) == Lines)
    {
      return (int32_t)index;
    }
  }

  return -1;
}

/* The ISP is on MAIN; AuxAllowed for the one stage AUX also has. */
static int32_t CAMERA_CheckIspPipe(uint32_t Instance, uint32_t Pipe, uint32_t AuxAllowed)
{
  if ((Instance >= BSP_CAMERA_INSTANCES_NBR) || (Pipe >= BSP_CAMERA_PIPES_NBR))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Camera_Ctx[Instance].Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (Pipe == BSP_CAMERA_PIPE_MAIN)
  {
    return BSP_ERROR_NONE;
  }

  if ((Pipe == BSP_CAMERA_PIPE_AUX) && (AuxAllowed != 0U))
  {
    return BSP_ERROR_NONE;
  }

  return BSP_ERROR_FEATURE_NOT_SUPPORTED;
}

/* Write the data-type filter directly, whatever the HAL left. It survives a stop, and a
 * start does not rewrite it (see BSP_CAMERA_Stop). */
static void CAMERA_PipeAssertDataType(uint32_t Pipe, uint32_t DataType)
{
  if (Pipe == BSP_CAMERA_PIPE_DUMP)
  {
    MODIFY_REG(DCMIPP->P0FSCR,
               DCMIPP_P0FSCR_DTMODE | DCMIPP_P0FSCR_DTIDA | DCMIPP_P0FSCR_DTIDB,
               DCMIPP_DTMODE_DTIDA |
               (DataType << DCMIPP_P0FSCR_DTIDA_Pos) |
               (DataType << DCMIPP_P0FSCR_DTIDB_Pos));
  }
  else if (Pipe == BSP_CAMERA_PIPE_MAIN)
  {
    MODIFY_REG(DCMIPP->P1FSCR,
               DCMIPP_P1FSCR_DTMODE | DCMIPP_P1FSCR_DTIDA | DCMIPP_P1FSCR_DTIDB,
               DCMIPP_DTMODE_DTIDA |
               (DataType << DCMIPP_P1FSCR_DTIDA_Pos) |
               (DataType << DCMIPP_P1FSCR_DTIDB_Pos));
  }
  else
  {
    /* Pipe 2 has no data-type mode field and no second data type. */
    MODIFY_REG(DCMIPP->P2FSCR, DCMIPP_P2FSCR_DTIDA,
               (DataType << DCMIPP_P2FSCR_DTIDA_Pos));
  }
}

static int32_t CAMERA_PipeCsiSource(uint32_t Pipe, uint32_t DataType)
{
  DCMIPP_CSI_PIPE_ConfTypeDef csi = {0};

  csi.DataTypeMode = DCMIPP_DTMODE_DTIDA;
  csi.DataTypeIDA  = DataType;
  csi.DataTypeIDB  = DataType;

  if (HAL_DCMIPP_CSI_PIPE_SetConfig(&hcamera_dcmipp, Pipe, &csi) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  CAMERA_PipeAssertDataType(Pipe, DataType);

  SET_BIT(DCMIPP->CMCR, DCMIPP_CMCR_INSEL);

  return BSP_ERROR_NONE;
}

/* HAL_DCMIPP_PIPE_SetConfig takes a pipe only in reset or error state, so later calls use
 * the single setters. Pitch and PackerFormat are ignored for pipe 0. */
static int32_t CAMERA_PipeCoreConfig(uint32_t Pipe, uint32_t Pitch, uint32_t PackerFormat)
{
  DCMIPP_PipeConfTypeDef cfg = {0};
  HAL_DCMIPP_PipeStateTypeDef pipe_state = hcamera_dcmipp.PipeState[Pipe];

  cfg.FrameRate         = DCMIPP_FRAME_RATE_ALL;
  cfg.PixelPipePitch    = Pitch;
  cfg.PixelPackerFormat = PackerFormat;

  if ((pipe_state == HAL_DCMIPP_PIPE_STATE_RESET) ||
      (pipe_state == HAL_DCMIPP_PIPE_STATE_ERROR))
  {
    if (HAL_DCMIPP_PIPE_SetConfig(&hcamera_dcmipp, Pipe, &cfg) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
    return BSP_ERROR_NONE;
  }

  if (HAL_DCMIPP_PIPE_SetFrameRate(&hcamera_dcmipp, Pipe, DCMIPP_FRAME_RATE_ALL) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  if (Pipe != BSP_CAMERA_PIPE_DUMP)
  {
    if (HAL_DCMIPP_PIPE_SetPixelPackerFormat(&hcamera_dcmipp, Pipe, PackerFormat) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
    if (HAL_DCMIPP_PIPE_SetPitch(&hcamera_dcmipp, Pipe, Pitch) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }

    /* The chroma pitch has no HAL setter (only the first configuration writes it): the
     * luma pitch for semi-planar, half that for each full-planar chroma plane. */
    if (Pipe == BSP_CAMERA_PIPE_MAIN)
    {
      if (PackerFormat == DCMIPP_PIXEL_PACKER_FORMAT_YUV420_2)
      {
        MODIFY_REG(DCMIPP->P1PPM1PR, DCMIPP_P1PPM1PR_PITCH, Pitch << DCMIPP_P1PPM1PR_PITCH_Pos);
      }
      else if (PackerFormat == DCMIPP_PIXEL_PACKER_FORMAT_YUV420_3)
      {
        MODIFY_REG(DCMIPP->P1PPM1PR, DCMIPP_P1PPM1PR_PITCH, (Pitch / 2U) << DCMIPP_P1PPM1PR_PITCH_Pos);
      }
      else
      {
        /* Packed: no chroma plane. */
      }
    }
  }

  return BSP_ERROR_NONE;
}

/* DUMP (pipe 0) has no processing stages: no downsizer, decimator or colour conversion. */
static int32_t CAMERA_ConfigDump(const BSP_CAMERA_PipeConfig_t *pConfig, CAMERA_PipeCtx_t *pCtx)
{
  const uint32_t width  = Camera_Ctx[0].Width;
  const uint32_t height = Camera_Ctx[0].Height;
  int32_t        status;

  uint32_t bytes_per_pixel;
  uint32_t packer;

  /* The link is copied untouched, so the sensor decides the format. */
  if (Camera_Sensor->SensorFormat == BSP_CAMERA_PF_RAW8)
  {
    if (pConfig->PixelFormat != BSP_CAMERA_PIPE_PF_RAW8)
    {
      return BSP_ERROR_FEATURE_NOT_SUPPORTED;
    }

    bytes_per_pixel = 1U;
    packer          = DCMIPP_PIXEL_PACKER_FORMAT_MONO_Y8_G8_1;
  }
  else
  {
    if (pConfig->PixelFormat != BSP_CAMERA_PIPE_PF_RGB565)
    {
      return BSP_ERROR_FEATURE_NOT_SUPPORTED;
    }

    bytes_per_pixel = 2U;
    packer          = DCMIPP_PIXEL_PACKER_FORMAT_RGB565_1;
  }

  if (((pConfig->Width != 0U) && (pConfig->Width != width)) ||
      ((pConfig->Height != 0U) && (pConfig->Height != height)))
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  if (pConfig->CropEnable != 0U)
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  status = CAMERA_PipeCsiSource(BSP_CAMERA_PIPE_DUMP, Camera_Sensor->CsiDataType);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  /* Clear any crop a previous configuration left behind. */
  (void)HAL_DCMIPP_PIPE_DisableCrop(&hcamera_dcmipp, BSP_CAMERA_PIPE_DUMP);

  status = CAMERA_PipeCoreConfig(BSP_CAMERA_PIPE_DUMP, width * bytes_per_pixel, packer);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  pCtx->PixelFormat = pConfig->PixelFormat;
  pCtx->Width       = width;
  pCtx->Height      = height;
  pCtx->FrameBytes  = width * height * bytes_per_pixel;

  return BSP_ERROR_NONE;
}

/* MAIN: full-frame YUV from its colour conversion; the packer sets the memory layout. */
static int32_t CAMERA_ConfigMain(const BSP_CAMERA_PipeConfig_t *pConfig, CAMERA_PipeCtx_t *pCtx)
{
  const uint32_t width  = Camera_Ctx[0].Width;
  const uint32_t height = Camera_Ctx[0].Height;
  DCMIPP_ColorConversionConfTypeDef rgb_to_yuv = {0};
  uint32_t packer;
  uint32_t pitch;
  uint32_t frame_bytes;
  int32_t status;

  /* Pitch is the luma line in bytes: 2 per pixel packed, 1 planar. */
  switch (pConfig->PixelFormat)
  {
    case BSP_CAMERA_PIPE_PF_YUV422:
      packer      = DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1;
      pitch       = width * 2U;
      frame_bytes = width * height * 2U;
      break;

    case BSP_CAMERA_PIPE_PF_YUV420_SP:
      packer      = DCMIPP_PIXEL_PACKER_FORMAT_YUV420_2;
      pitch       = width;
      frame_bytes = (width * height * 3U) / 2U;
      break;

    case BSP_CAMERA_PIPE_PF_YUV420_FP:
      packer      = DCMIPP_PIXEL_PACKER_FORMAT_YUV420_3;
      pitch       = width;
      /* Each chroma plane is half the luma pitch wide and half its rows. */
      frame_bytes = (width * height * 3U) / 2U;
      break;

    default:
      return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  /* Planar chroma is subsampled by two each way, so width and height must be even. */
  if ((pConfig->PixelFormat != BSP_CAMERA_PIPE_PF_YUV422) &&
      (((width & 1U) != 0U) || ((height & 1U) != 0U)))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (((pConfig->Width != 0U) && (pConfig->Width != width)) ||
      ((pConfig->Height != 0U) && (pConfig->Height != height)))
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  if (pConfig->CropEnable != 0U)
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  if ((pitch % BSP_CAMERA_BUFFER_ALIGNMENT) != 0U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Full planar: the half-width chroma pitch must be aligned too. */
  if ((pConfig->PixelFormat == BSP_CAMERA_PIPE_PF_YUV420_FP) &&
      (((pitch / 2U) % BSP_CAMERA_BUFFER_ALIGNMENT) != 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  status = CAMERA_PipeCsiSource(BSP_CAMERA_PIPE_MAIN, Camera_Sensor->CsiDataType);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  /* A raw sensor is demosaiced first, in the descriptor's filter order;
   * BSP_CAMERA_SetIspDemosaicing corrects it once the driver knows its own. */
  if (Camera_Sensor->SensorFormat == BSP_CAMERA_PF_RAW8)
  {
    DCMIPP_RawBayer2RGBConfTypeDef demosaic = {0};

    demosaic.RawBayerType  = Camera_Sensor->BayerOrder;
    demosaic.VLineStrength = DCMIPP_RAWBAYER_ALGO_STRENGTH_8;
    demosaic.HLineStrength = DCMIPP_RAWBAYER_ALGO_STRENGTH_8;
    demosaic.PeakStrength  = DCMIPP_RAWBAYER_ALGO_STRENGTH_8;
    demosaic.EdgeStrength  = DCMIPP_RAWBAYER_ALGO_STRENGTH_8;

    if (HAL_DCMIPP_PIPE_SetISPRawBayer2RGBConfig(&hcamera_dcmipp, BSP_CAMERA_PIPE_MAIN,
                                                 &demosaic) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }

    if (HAL_DCMIPP_PIPE_EnableISPRawBayer2RGB(&hcamera_dcmipp, BSP_CAMERA_PIPE_MAIN) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }

    Camera_Demosaic   = demosaic;
    Camera_DemosaicOn = 1U;
  }
  else
  {
    (void)HAL_DCMIPP_PIPE_DisableISPRawBayer2RGB(&hcamera_dcmipp, BSP_CAMERA_PIPE_MAIN);
    Camera_DemosaicOn = 0U;
  }

  /* Clear geometry stages a previous configuration may have enabled. */
  (void)HAL_DCMIPP_PIPE_DisableCrop(&hcamera_dcmipp, BSP_CAMERA_PIPE_MAIN);
  (void)HAL_DCMIPP_PIPE_DisableDecimation(&hcamera_dcmipp, BSP_CAMERA_PIPE_MAIN);
  (void)HAL_DCMIPP_PIPE_DisableDownsize(&hcamera_dcmipp, BSP_CAMERA_PIPE_MAIN);

  status = CAMERA_PipeCoreConfig(BSP_CAMERA_PIPE_MAIN, pitch, packer);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  rgb_to_yuv.ClampOutputSamples = DISABLE;
  rgb_to_yuv.OutputSamplesType  = DCMIPP_CLAMP_YUV;
  rgb_to_yuv.RR = 131;  rgb_to_yuv.RG = -110; rgb_to_yuv.RB = -21; rgb_to_yuv.RA = 128;
  rgb_to_yuv.GR = 77;   rgb_to_yuv.GG = 150;  rgb_to_yuv.GB = 29;  rgb_to_yuv.GA = 0;
  rgb_to_yuv.BR = -44;  rgb_to_yuv.BG = -87;  rgb_to_yuv.BB = 131; rgb_to_yuv.BA = 128;

  if (HAL_DCMIPP_PIPE_SetYUVConversionConfig(&hcamera_dcmipp, BSP_CAMERA_PIPE_MAIN,
                                             &rgb_to_yuv) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  if (HAL_DCMIPP_PIPE_EnableYUVConversion(&hcamera_dcmipp, BSP_CAMERA_PIPE_MAIN) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  pCtx->PixelFormat = pConfig->PixelFormat;
  pCtx->Width       = width;
  pCtx->Height      = height;
  pCtx->FrameBytes  = frame_bytes;

  return BSP_ERROR_NONE;
}

/* AUX: RGB888; crop, then decimation, then downsize. */
static int32_t CAMERA_ConfigAux(const BSP_CAMERA_PipeConfig_t *pConfig, CAMERA_PipeCtx_t *pCtx)
{
  const uint32_t sensor_w = Camera_Ctx[0].Width;
  const uint32_t sensor_h = Camera_Ctx[0].Height;
  uint32_t out_w  = (pConfig->Width  != 0U) ? pConfig->Width  : sensor_w;
  uint32_t out_h  = (pConfig->Height != 0U) ? pConfig->Height : sensor_h;
  uint32_t in_w;
  uint32_t in_h;
  uint32_t pitch;
  uint32_t factor;
  DCMIPP_DownsizeTypeDef downsize = {0};
  int32_t  status;

  if (pConfig->PixelFormat != BSP_CAMERA_PIPE_PF_RGB888)
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  if ((out_w == 0U) || (out_h == 0U) ||
      (out_w > CAMERA_DOWNSIZE_SIZE_MAX) || (out_h > CAMERA_DOWNSIZE_SIZE_MAX))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (pConfig->CropEnable != 0U)
  {
    /* Compared without adding, so a large offset cannot wrap the sum. */
    if ((pConfig->CropWidth == 0U) || (pConfig->CropHeight == 0U) ||
        (pConfig->CropX > sensor_w) || (pConfig->CropWidth > (sensor_w - pConfig->CropX)) ||
        (pConfig->CropY > sensor_h) || (pConfig->CropHeight > (sensor_h - pConfig->CropY)))
    {
      return BSP_ERROR_WRONG_PARAM;
    }
    in_w = pConfig->CropWidth;
    in_h = pConfig->CropHeight;
  }
  else
  {
    in_w = sensor_w;
    in_h = sensor_h;
  }

  /* The scaler reduces only. */
  if ((out_w > in_w) || (out_h > in_h))
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  pitch = out_w * 3U;
  if ((pitch % BSP_CAMERA_BUFFER_ALIGNMENT) != 0U)
  {
    /* The pitch register only holds multiples of 16 bytes. */
    return BSP_ERROR_WRONG_PARAM;
  }

  /* PIPEDIFF (shared or own input) can change only with both pipes disabled, so it is
   * written only when it must change. */
  if (Camera_Sensor->SharedPipes != 0U)
  {
    /* Take the data from MAIN, not the link. */
    if (((DCMIPP->P1FSCR & DCMIPP_P1FSCR_PIPEDIFF) != 0U) &&
        (HAL_DCMIPP_PIPE_CSI_EnableShare(&hcamera_dcmipp, BSP_CAMERA_PIPE_AUX) != HAL_OK))
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
  }
  else
  {
    /* Own input and data-type filter, so AUX captures with MAIN unconfigured. With MAIN
     * running the bit cannot change, and MAIN's input is the same stream. */
    if ((DCMIPP->P1FSCR & DCMIPP_P1FSCR_PIPEDIFF) == 0U)
    {
      (void)HAL_DCMIPP_PIPE_CSI_DisableShare(&hcamera_dcmipp, BSP_CAMERA_PIPE_AUX);
    }

    status = CAMERA_PipeCsiSource(BSP_CAMERA_PIPE_AUX, Camera_Sensor->CsiDataType);
    if (status != BSP_ERROR_NONE)
    {
      return status;
    }
  }

  status = CAMERA_PipeCoreConfig(BSP_CAMERA_PIPE_AUX, pitch,
                                 DCMIPP_PIXEL_PACKER_FORMAT_RGB888_YUV444_1);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  (void)HAL_DCMIPP_PIPE_EnableRedBlueSwap(&hcamera_dcmipp, BSP_CAMERA_PIPE_AUX);
  if ((DCMIPP->P2PPCR & DCMIPP_P2PPCR_SWAPRB) == 0U)
  {
    SET_BIT(DCMIPP->P2PPCR, DCMIPP_P2PPCR_SWAPRB);
  }

  if (pConfig->CropEnable != 0U)
  {
    DCMIPP_CropConfTypeDef crop = {0};

    crop.HStart   = pConfig->CropX;
    crop.VStart   = pConfig->CropY;
    crop.HSize    = pConfig->CropWidth;
    crop.VSize    = pConfig->CropHeight;
    crop.PipeArea = DCMIPP_POSITIVE_AREA;

    if (HAL_DCMIPP_PIPE_SetCropConfig(&hcamera_dcmipp, BSP_CAMERA_PIPE_AUX, &crop) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
    if (HAL_DCMIPP_PIPE_EnableCrop(&hcamera_dcmipp, BSP_CAMERA_PIPE_AUX) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
  }
  else
  {
    (void)HAL_DCMIPP_PIPE_DisableCrop(&hcamera_dcmipp, BSP_CAMERA_PIPE_AUX);
  }

  /* Smallest power-of-two decimation, after the crop, that brings both axes within the
   * downsizer's ratio. */
  factor = 1U;
  while ((((in_w / factor) * 8192U) / out_w > CAMERA_DOWNSIZE_RATIO_MAX) ||
         (((in_h / factor) * 8192U) / out_h > CAMERA_DOWNSIZE_RATIO_MAX))
  {
    factor <<= 1;
    if (factor >= CAMERA_DECIMATION_MAX)
    {
      break;
    }
  }

  if (factor > 1U)
  {
    DCMIPP_DecimationConfTypeDef dec = {0};

    dec.HRatio = (factor >= 8U) ? DCMIPP_HDEC_1_OUT_8 :
                 (factor == 4U) ? DCMIPP_HDEC_1_OUT_4 : DCMIPP_HDEC_1_OUT_2;
    dec.VRatio = (factor >= 8U) ? DCMIPP_VDEC_1_OUT_8 :
                 (factor == 4U) ? DCMIPP_VDEC_1_OUT_4 : DCMIPP_VDEC_1_OUT_2;

    if (HAL_DCMIPP_PIPE_SetDecimationConfig(&hcamera_dcmipp, BSP_CAMERA_PIPE_AUX,
                                            &dec) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
    if (HAL_DCMIPP_PIPE_EnableDecimation(&hcamera_dcmipp, BSP_CAMERA_PIPE_AUX) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }

    in_w /= factor;
    in_h /= factor;
  }
  else
  {
    (void)HAL_DCMIPP_PIPE_DisableDecimation(&hcamera_dcmipp, BSP_CAMERA_PIPE_AUX);
  }

  downsize.HRatio     = (in_w * 8192U) / out_w;
  downsize.VRatio     = (in_h * 8192U) / out_h;
  downsize.HSize      = out_w;
  downsize.VSize      = out_h;
  downsize.HDivFactor = (out_w * 1023U) / in_w;
  downsize.VDivFactor = (out_h * 1023U) / in_h;

  if ((downsize.HRatio > CAMERA_DOWNSIZE_RATIO_MAX) ||
      (downsize.VRatio > CAMERA_DOWNSIZE_RATIO_MAX) ||
      (downsize.HDivFactor < CAMERA_DOWNSIZE_DIVFACTOR_MIN) ||
      (downsize.VDivFactor < CAMERA_DOWNSIZE_DIVFACTOR_MIN) ||
      (downsize.HDivFactor > CAMERA_DOWNSIZE_DIVFACTOR_MAX) ||
      (downsize.VDivFactor > CAMERA_DOWNSIZE_DIVFACTOR_MAX))
  {
    /* Beyond the scaler even after the deepest decimation: lower the sensor resolution or
     * raise the target size. */
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  if (HAL_DCMIPP_PIPE_SetDownsizeConfig(&hcamera_dcmipp, BSP_CAMERA_PIPE_AUX,
                                        &downsize) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  if (HAL_DCMIPP_PIPE_EnableDownsize(&hcamera_dcmipp, BSP_CAMERA_PIPE_AUX) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  pCtx->PixelFormat = BSP_CAMERA_PIPE_PF_RGB888;
  pCtx->Width       = out_w;
  pCtx->Height      = out_h;
  pCtx->FrameBytes  = out_w * out_h * 3U;

  return BSP_ERROR_NONE;
}

/* Invalidate every plane of the start's buffers: the finished frame, and the next
 * destination, where no stale line may survive either. */
static void CAMERA_InvalidateFrame(const CAMERA_PipeCtx_t *pCtx)
{
#if defined(USE_BSP_CPU_CACHE_MAINTENANCE) && (USE_BSP_CPU_CACHE_MAINTENANCE == 1U)
  uint32_t i;

  /* Never invalidate a wrapped band: non-cacheable, and the heap sits right behind it. */
  if (pCtx->LineWrapped != 0U)
  {
    return;
  }

  for (i = 0U; i < pCtx->Regions; i++)
  {
    SCB_InvalidateDCache_by_Addr(pCtx->Region[i].pAddr, (int32_t)pCtx->Region[i].Bytes);
  }
#else
  (void)pCtx;
#endif
}

/* Plane 0: luma or packed; 1: UV or U; 2: V. */
static uint32_t CAMERA_PlaneBytes(const CAMERA_PipeCtx_t *pCtx, uint32_t Plane)
{
  if (pCtx->PixelFormat == BSP_CAMERA_PIPE_PF_YUV420_SP)
  {
    return (Plane == 0U) ? (pCtx->Width * pCtx->Height) : (pCtx->Width * (pCtx->Height / 2U));
  }
  if (pCtx->PixelFormat == BSP_CAMERA_PIPE_PF_YUV420_FP)
  {
    return (Plane == 0U) ? (pCtx->Width * pCtx->Height)
                         : ((pCtx->Width / 2U) * (pCtx->Height / 2U));
  }
  return pCtx->FrameBytes;
}

/* ppPlane: each buffer's planes in turn, luma or packed first. Each is cleaned and
 * invalidated before the start, so no dirty CPU line is written back over a frame; the
 * planes then belong to the pipe until it stops. */
static void CAMERA_SetRegions(CAMERA_PipeCtx_t *pCtx, uint8_t *const *ppPlane,
                              uint32_t Planes, uint32_t Buffers)
{
  uint32_t i;

  pCtx->Regions = 0U;
  for (i = 0U; (i < (Planes * Buffers)) && (i < CAMERA_REGIONS_MAX); i++)
  {
    pCtx->Region[i].pAddr = ppPlane[i];
    pCtx->Region[i].Bytes = CAMERA_PlaneBytes(pCtx, i % Planes);
    pCtx->Regions++;
#if defined(USE_BSP_CPU_CACHE_MAINTENANCE) && (USE_BSP_CPU_CACHE_MAINTENANCE == 1U)
    if (pCtx->LineWrapped == 0U)
    {
      SCB_CleanInvalidateDCache_by_Addr(pCtx->Region[i].pAddr, (int32_t)pCtx->Region[i].Bytes);
    }
#endif
  }
}

/* The HAL's start ORs in the capture mode (a continuous start after a snapshot would take
 * one frame) and enables interrupts over stale flags: clear both. */
static void CAMERA_PrepareStart(uint32_t Pipe)
{
  if (Pipe == BSP_CAMERA_PIPE_DUMP)
  {
    CLEAR_BIT(DCMIPP->P0FCTCR, DCMIPP_P0FCTCR_CPTMODE);
    __HAL_DCMIPP_CLEAR_FLAG(&hcamera_dcmipp, DCMIPP_FLAG_PIPE0_FRAME | DCMIPP_FLAG_PIPE0_VSYNC |
                                             DCMIPP_FLAG_PIPE0_LINE | DCMIPP_FLAG_PIPE0_LIMIT |
                                             DCMIPP_FLAG_PIPE0_OVR);
  }
  else if (Pipe == BSP_CAMERA_PIPE_MAIN)
  {
    CLEAR_BIT(DCMIPP->P1FCTCR, DCMIPP_P1FCTCR_CPTMODE);
    __HAL_DCMIPP_CLEAR_FLAG(&hcamera_dcmipp, DCMIPP_FLAG_PIPE1_FRAME | DCMIPP_FLAG_PIPE1_VSYNC |
                                             DCMIPP_FLAG_PIPE1_LINE | DCMIPP_FLAG_PIPE1_OVR);
  }
  else
  {
    CLEAR_BIT(DCMIPP->P2FCTCR, DCMIPP_P2FCTCR_CPTMODE);
    __HAL_DCMIPP_CLEAR_FLAG(&hcamera_dcmipp, DCMIPP_FLAG_PIPE2_FRAME | DCMIPP_FLAG_PIPE2_VSYNC |
                                             DCMIPP_FLAG_PIPE2_LINE | DCMIPP_FLAG_PIPE2_OVR);
  }
}

/* HAL_DCMIPP_CSI_PIPE_Stop also stops the shared virtual channel: this does only its pipe
 * steps, for use while another pipe runs. */
static int32_t CAMERA_StopPipeOnly(uint32_t Pipe)
{
  __IO uint32_t *fctcr;
  __IO uint32_t *ppcr;
  __IO uint32_t *fscr;
  uint32_t       cptreq;
  uint32_t       cptact;
  uint32_t       dbm;
  uint32_t       pipen;
  uint32_t       its;
  uint32_t       start;

  if (Pipe == BSP_CAMERA_PIPE_DUMP)
  {
    fctcr  = &DCMIPP->P0FCTCR; cptreq = DCMIPP_P0FCTCR_CPTREQ; cptact = DCMIPP_CMSR1_P0CPTACT;
    ppcr   = &DCMIPP->P0PPCR;  dbm    = DCMIPP_P0PPCR_DBM;
    fscr   = &DCMIPP->P0FSCR;  pipen  = DCMIPP_P0FSCR_PIPEN;
    its    = DCMIPP_IT_PIPE0_FRAME | DCMIPP_IT_PIPE0_VSYNC | DCMIPP_IT_PIPE0_LINE |
             DCMIPP_IT_PIPE0_LIMIT | DCMIPP_IT_PIPE0_OVR;
  }
  else if (Pipe == BSP_CAMERA_PIPE_MAIN)
  {
    fctcr  = &DCMIPP->P1FCTCR; cptreq = DCMIPP_P1FCTCR_CPTREQ; cptact = DCMIPP_CMSR1_P1CPTACT;
    ppcr   = &DCMIPP->P1PPCR;  dbm    = DCMIPP_P1PPCR_DBM;
    fscr   = &DCMIPP->P1FSCR;  pipen  = DCMIPP_P1FSCR_PIPEN;
    its    = DCMIPP_IT_PIPE1_FRAME | DCMIPP_IT_PIPE1_VSYNC | DCMIPP_IT_PIPE1_LINE |
             DCMIPP_IT_PIPE1_OVR;
  }
  else
  {
    fctcr  = &DCMIPP->P2FCTCR; cptreq = DCMIPP_P2FCTCR_CPTREQ; cptact = DCMIPP_CMSR1_P2CPTACT;
    ppcr   = &DCMIPP->P2PPCR;  dbm    = DCMIPP_P2PPCR_DBM;
    fscr   = &DCMIPP->P2FSCR;  pipen  = DCMIPP_P2FSCR_PIPEN;
    its    = DCMIPP_IT_PIPE2_FRAME | DCMIPP_IT_PIPE2_VSYNC | DCMIPP_IT_PIPE2_LINE |
             DCMIPP_IT_PIPE2_OVR;
  }

  CLEAR_BIT(*fctcr, cptreq);

  start = HAL_GetTick();
  while ((DCMIPP->CMSR1 & cptact) != 0U)
  {
    if ((HAL_GetTick() - start) > CAMERA_PIPE_STOP_TIMEOUT_MS)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
  }

  CLEAR_BIT(*ppcr, dbm);
  CLEAR_BIT(*fscr, pipen);
  __HAL_DCMIPP_DISABLE_IT(&hcamera_dcmipp, its);
  hcamera_dcmipp.PipeState[Pipe] = HAL_DCMIPP_PIPE_STATE_READY;

  return BSP_ERROR_NONE;
}

/* 1 if a pipe is running or suspended. */
static uint32_t CAMERA_AnyPipeRunning(const CAMERA_Ctx_t *pCtx)
{
  uint32_t pipe;

  for (pipe = 0U; pipe < BSP_CAMERA_PIPES_NBR; pipe++)
  {
    if ((pCtx->Pipe[pipe].State == CAMERA_PIPE_RUNNING) ||
        (pCtx->Pipe[pipe].State == CAMERA_PIPE_SUSPENDED))
    {
      return 1U;
    }
  }

  return 0U;
}

/* Public API */

int32_t BSP_CAMERA_RegisterSensor(const BSP_CAMERA_Sensor_t *pSensor)
{
  if ((pSensor == NULL) || (pSensor->pDrv == NULL) ||
      (pSensor->pModes == NULL) || (pSensor->ModeCount == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* These are called unconditionally: refuse a gap here, not a NULL call at bring-up. */
  if ((pSensor->pDrv->Init == NULL) || (pSensor->pDrv->DeInit == NULL) ||
      (pSensor->pDrv->ReadID == NULL) || (pSensor->pDrv->Start == NULL) ||
      (pSensor->pDrv->Stop == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if ((pSensor->CsiDataType != BSP_CAMERA_CSI_DT_RGB565) &&
      (pSensor->CsiDataType != BSP_CAMERA_CSI_DT_RAW8))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (pSensor->DataLanes > 2U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Refused once up: the link type, modes and pipe topology are in hardware by then. */
  if (Camera_Ctx[0].Initialised != 0U)
  {
    return BSP_ERROR_BUSY;
  }

  Camera_Sensor = pSensor;

  return BSP_ERROR_NONE;
}

const BSP_CAMERA_Sensor_t *BSP_CAMERA_GetSensor(void)
{
  return Camera_Sensor;
}

int32_t BSP_CAMERA_SetCarrier(uint32_t Carrier)
{
  if ((Carrier != BSP_CAMERA_CARRIER_STANDARD) &&
      (Carrier != BSP_CAMERA_CARRIER_INVERTED))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Refused once the camera is up rather than accepted and ignored. */
  if (Camera_Ctx[0].Initialised != 0U)
  {
    return BSP_ERROR_BUSY;
  }

  Camera_Carrier = Carrier;

  return BSP_ERROR_NONE;
}

uint32_t BSP_CAMERA_GetCarrier(void)
{
  return Camera_Carrier;
}

int32_t BSP_CAMERA_Init(uint32_t Instance, uint32_t Resolution, uint32_t PixelFormat)
{
  const BSP_CAMERA_SensorMode_t *mode;
  CAMERA_Ctx_t        *ctx;
  uint32_t             pipe;
  int32_t              status;

  if (Instance >= BSP_CAMERA_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  ctx = &Camera_Ctx[Instance];

  if (ctx->Initialised != 0U)
  {
    return BSP_ERROR_BUSY;
  }

  if (PixelFormat != Camera_Sensor->SensorFormat)
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  mode = CAMERA_FindMode(Resolution);
  if (mode == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  for (pipe = 0U; pipe < BSP_CAMERA_PIPES_NBR; pipe++)
  {
    ctx->Pipe[pipe].State       = CAMERA_PIPE_UNCONFIGURED;
    ctx->Pipe[pipe].PixelFormat = 0U;
    ctx->Pipe[pipe].Width       = 0U;
    ctx->Pipe[pipe].Height      = 0U;
    ctx->Pipe[pipe].FrameBytes  = 0U;
    ctx->Pipe[pipe].pBuf        = NULL;
    ctx->Pipe[pipe].Regions     = 0U;
    ctx->Pipe[pipe].LineWrapped = 0U;   /* a fresh peripheral does not wrap */
    ctx->Pipe[pipe].LineEventOn = 0U;
  }
  ctx->SensorOn = 0U;

  /* Isolation first: until granted, the DCMIPP's writes are dropped silently. */
  CAMERA_RIFConfig();

  if (BSP_I2C1_Init() != BSP_ERROR_NONE)
  {
    return BSP_ERROR_BUS_FAILURE;
  }

  /* Probe the registered sensor's own address, not the OV5640's. */
  if ((Camera_Sensor->I2cAddress != 0U) &&
      (BSP_I2C1_IsReady(Camera_Sensor->I2cAddress, CAMERA_PROBE_TRIALS) != BSP_ERROR_NONE))
  {
    (void)BSP_I2C1_DeInit();
    return BSP_ERROR_UNKNOWN_COMPONENT;
  }

  /* Receiver first: calibrate the D-PHY before the sensor's continuous clock starts. */
  hcamera_dcmipp.Instance = DCMIPP;

#if defined(USE_HAL_DCMIPP_REGISTER_CALLBACKS) && (USE_HAL_DCMIPP_REGISTER_CALLBACKS == 1U)
  /* Nothing registered yet: install this module's MSP routines. */
  if (Camera_MspCbValid == 0U)
  {
    if (BSP_CAMERA_RegisterDefaultMspCallbacks(Instance) != BSP_ERROR_NONE)
    {
      (void)BSP_I2C1_DeInit();
      return BSP_ERROR_MSP_FAILURE;
    }
  }
#endif /* USE_HAL_DCMIPP_REGISTER_CALLBACKS */

  if (HAL_DCMIPP_Init(&hcamera_dcmipp) != HAL_OK)
  {
    (void)BSP_I2C1_DeInit();
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* The kernel-clock result recorded by the MSP init. */
  if (Camera_ClockStatus != BSP_ERROR_NONE)
  {
    (void)HAL_DCMIPP_DeInit(&hcamera_dcmipp);
    (void)BSP_I2C1_DeInit();
    return Camera_ClockStatus;
  }

  ctx->PhyBucket = CAMERA_PhyBucket(mode->LaneRateMbps);

  status = CAMERA_CsiConfig(ctx->PhyBucket);
  if (status == BSP_ERROR_NONE)
  {
    status = CAMERA_ProbeSensor(Resolution, PixelFormat);
  }
  if (status != BSP_ERROR_NONE)
  {
    (void)HAL_DCMIPP_DeInit(&hcamera_dcmipp);
    (void)BSP_I2C1_DeInit();
    return status;
  }

  ctx->Resolution   = Resolution;
  ctx->PixelFormat  = PixelFormat;
  ctx->Width        = mode->Width;
  ctx->Height       = mode->Height;
  ctx->MaxFrameRate = mode->MaxFrameRate;
  ctx->LaneRateMbps = mode->LaneRateMbps;
  ctx->MirrorFlip   = (Camera_Sensor == &Camera_Ov5640Sensor) ? CAMERA_Ov5640MirrorFlip()
                                                             : BSP_CAMERA_MIRRORFLIP_NONE;
  /* The sensor's own state after its initialisation. */
  ctx->Brightness   = 0;
  ctx->Saturation   = 0;
  ctx->Contrast     = 0;
  ctx->HueDegree    = 0;
  ctx->LightMode    = BSP_CAMERA_LIGHT_AUTO;
  ctx->ColorEffect  = BSP_CAMERA_COLOR_EFFECT_NONE;
  ctx->Zoom         = BSP_CAMERA_ZOOM_x1;
  ctx->NightMode    = BSP_CAMERA_NIGHT_MODE_DISABLE;
  ctx->Initialised  = 1U;

  return BSP_ERROR_NONE;
}

/** @brief Shut the camera down and release the bus. */
int32_t BSP_CAMERA_DeInit(uint32_t Instance)
{
  CAMERA_Ctx_t *ctx;
  uint32_t      pipe;
  int32_t       status = BSP_ERROR_NONE;

  if (Instance >= BSP_CAMERA_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  ctx = &Camera_Ctx[Instance];

  if (ctx->Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  for (pipe = 0U; pipe < BSP_CAMERA_PIPES_NBR; pipe++)
  {
    if ((ctx->Pipe[pipe].State == CAMERA_PIPE_RUNNING) ||
        (ctx->Pipe[pipe].State == CAMERA_PIPE_SUSPENDED))
    {
      (void)HAL_DCMIPP_CSI_PIPE_Stop(&hcamera_dcmipp, pipe, DCMIPP_VIRTUAL_CHANNEL0);
    }
    ctx->Pipe[pipe].State       = CAMERA_PIPE_UNCONFIGURED;
    ctx->Pipe[pipe].pBuf        = NULL;
    ctx->Pipe[pipe].FrameBytes  = 0U;
    ctx->Pipe[pipe].Regions     = 0U;
    ctx->Pipe[pipe].LineWrapped = 0U;
    ctx->Pipe[pipe].LineEventOn = 0U;
  }
  ctx->SensorOn = 0U;

  (void)Camera_Sensor->pDrv->Stop(Camera_Sensor->pObj);
  (void)Camera_Sensor->pDrv->DeInit(Camera_Sensor->pObj);

  if (HAL_DCMIPP_DeInit(&hcamera_dcmipp) != HAL_OK)
  {
    status = BSP_ERROR_PERIPH_FAILURE;
  }

  if (BSP_I2C1_DeInit() != BSP_ERROR_NONE)
  {
    status = BSP_ERROR_BUS_FAILURE;
  }

  ctx->Initialised  = 0U;
  Camera_DemosaicOn = 0U;

  return status;
}

/** @brief Set a pipe's destination format and geometry; BSP_ERROR_BUSY if it is running
  * or suspended. */
int32_t BSP_CAMERA_ConfigPipe(uint32_t Instance, uint32_t Pipe,
                              const BSP_CAMERA_PipeConfig_t *pConfig)
{
  CAMERA_Ctx_t *ctx;
  int32_t       status;

  if ((Instance >= BSP_CAMERA_INSTANCES_NBR) || (Pipe >= BSP_CAMERA_PIPES_NBR) ||
      (pConfig == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  ctx = &Camera_Ctx[Instance];

  if (ctx->Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if ((ctx->Pipe[Pipe].State == CAMERA_PIPE_RUNNING) ||
      (ctx->Pipe[Pipe].State == CAMERA_PIPE_SUSPENDED))
  {
    return BSP_ERROR_BUSY;
  }

  CAMERA_ClearStaleErrorState();

  switch (Pipe)
  {
    case BSP_CAMERA_PIPE_DUMP:
      status = CAMERA_ConfigDump(pConfig, &ctx->Pipe[Pipe]);
      break;

    case BSP_CAMERA_PIPE_MAIN:
      status = CAMERA_ConfigMain(pConfig, &ctx->Pipe[Pipe]);

      /* Clock the encoder now, so both ends of the line-buffer handshake are powered
       * before the first line event. */
      if (status == BSP_ERROR_NONE)
      {
        BSP_VENC_EnablePeripheral();
      }
      break;

    default:
      status = CAMERA_ConfigAux(pConfig, &ctx->Pipe[Pipe]);
      break;
  }

  if (status == BSP_ERROR_NONE)
  {
    ctx->Pipe[Pipe].State = CAMERA_PIPE_STOPPED;
  }

  return status;
}

/* Checks shared by every start entry point. */
static int32_t CAMERA_StartReady(uint32_t Instance, uint32_t Pipe, uint32_t Mode,
                                 CAMERA_Ctx_t **ppCtx)
{
  CAMERA_Ctx_t *ctx;

  if ((Instance >= BSP_CAMERA_INSTANCES_NBR) || (Pipe >= BSP_CAMERA_PIPES_NBR))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if ((Mode != BSP_CAMERA_MODE_CONTINUOUS) && (Mode != BSP_CAMERA_MODE_SNAPSHOT))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  ctx = &Camera_Ctx[Instance];

  if (ctx->Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (ctx->Pipe[Pipe].State == CAMERA_PIPE_UNCONFIGURED)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (ctx->Pipe[Pipe].State != CAMERA_PIPE_STOPPED)
  {
    return BSP_ERROR_BUSY;
  }

  /* Stopped here but held by the HAL: a snapshot's frame end and an overrun handled
   * together leave it in error; the HAL refuses starts until its own stop has run. */
  if (CAMERA_HalPipeHeld(Pipe) != 0U)
  {
    uint32_t others = 0U;
    uint32_t p;

    for (p = 0U; p < BSP_CAMERA_PIPES_NBR; p++)
    {
      if ((p != Pipe) && ((ctx->Pipe[p].State == CAMERA_PIPE_RUNNING) ||
                          (ctx->Pipe[p].State == CAMERA_PIPE_SUSPENDED)))
      {
        others = 1U;
      }
    }

    /* The virtual channel is shared, so it stops only with the last pipe. */
    if (others != 0U)
    {
      if (CAMERA_StopPipeOnly(Pipe) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_PERIPH_FAILURE;
      }
    }
    else if (HAL_DCMIPP_CSI_PIPE_Stop(&hcamera_dcmipp, Pipe, DCMIPP_VIRTUAL_CHANNEL0) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {
      /* Stopped with the virtual channel; the start below restarts both. */
    }
  }

  *ppCtx = ctx;

  return BSP_ERROR_NONE;
}

static uint32_t CAMERA_StartBufferOk(const uint8_t *pBuf)
{
  return ((pBuf != NULL) && (((uint32_t)pBuf & (BSP_CAMERA_BUFFER_ALIGNMENT - 1U)) == 0U))
         ? 1U : 0U;
}

/* One sensor stream feeds every pipe: the first pipe to start starts the sensor. */
static int32_t CAMERA_StartSensor(CAMERA_Ctx_t *ctx, uint32_t Pipe)
{
  if (ctx->SensorOn != 0U)
  {
    return BSP_ERROR_NONE;
  }

  if (Camera_Sensor->pDrv->Start(Camera_Sensor->pObj) != 0)
  {
    /* BSP_CAMERA_Stop keeps the virtual channel for other pipes, which can be running
     * here after BSP_CAMERA_SuspendSensor. */
    (void)BSP_CAMERA_Stop(0U, Pipe);
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  ctx->SensorOn = 1U;

  return BSP_ERROR_NONE;
}

/* Every stop turns the line interrupt off: re-enable requested line events, then start
 * the sensor if no pipe has. */
static int32_t CAMERA_FinishStart(CAMERA_Ctx_t *ctx, uint32_t Pipe)
{
  if (ctx->Pipe[Pipe].LineEventOn != 0U)
  {
    if (HAL_DCMIPP_PIPE_EnableLineEvent(&hcamera_dcmipp, Pipe, ctx->Pipe[Pipe].LineEventCode) != HAL_OK)
    {
      (void)BSP_CAMERA_Stop(0U, Pipe);
      return BSP_ERROR_PERIPH_FAILURE;
    }
  }

  return CAMERA_StartSensor(ctx, Pipe);
}

/** @brief Start capturing into pBuf: 16-byte aligned, at least BSP_CAMERA_GetFrameSize
  * bytes. Not for planar formats. */
int32_t BSP_CAMERA_Start(uint32_t Instance, uint32_t Pipe, uint8_t *pBuf, uint32_t Mode)
{
  CAMERA_Ctx_t *ctx;
  int32_t status = CAMERA_StartReady(Instance, Pipe, Mode, &ctx);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (CAMERA_StartBufferOk(pBuf) == 0U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* A planar pipe needs plane addresses this entry point never supplies. */
  if ((ctx->Pipe[Pipe].PixelFormat == BSP_CAMERA_PIPE_PF_YUV420_SP) ||
      (ctx->Pipe[Pipe].PixelFormat == BSP_CAMERA_PIPE_PF_YUV420_FP))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Re-assert serial input: an error interrupt since configuration can have cost it. */
  SET_BIT(DCMIPP->CMCR, DCMIPP_CMCR_INSEL);
  CAMERA_PrepareStart(Pipe);

  ctx->Pipe[Pipe].pBuf = pBuf;
  CAMERA_SetRegions(&ctx->Pipe[Pipe], &pBuf, 1U, 1U);

  ctx->Pipe[Pipe].State = CAMERA_PIPE_RUNNING;

  if (HAL_DCMIPP_CSI_PIPE_Start(&hcamera_dcmipp, Pipe, DCMIPP_VIRTUAL_CHANNEL0,
                                (uint32_t)pBuf, Mode) != HAL_OK)
  {
    ctx->Pipe[Pipe].State = CAMERA_PIPE_STOPPED;
    ctx->Pipe[Pipe].pBuf  = NULL;
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return CAMERA_FinishStart(ctx, Pipe);
}

int32_t BSP_CAMERA_DoubleBufferStart(uint32_t Instance, uint32_t Pipe,
                                     uint8_t *pBuf0, uint8_t *pBuf1, uint32_t Mode)
{
  CAMERA_Ctx_t *ctx;
  int32_t status = CAMERA_StartReady(Instance, Pipe, Mode, &ctx);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((CAMERA_StartBufferOk(pBuf0) == 0U) || (CAMERA_StartBufferOk(pBuf1) == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (pBuf0 == pBuf1)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* As for BSP_CAMERA_Start: planar pipes have their own double-buffered starts. */
  if ((ctx->Pipe[Pipe].PixelFormat == BSP_CAMERA_PIPE_PF_YUV420_SP) ||
      (ctx->Pipe[Pipe].PixelFormat == BSP_CAMERA_PIPE_PF_YUV420_FP))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  SET_BIT(DCMIPP->CMCR, DCMIPP_CMCR_INSEL);
  CAMERA_PrepareStart(Pipe);

  {
    uint8_t *const planes[2] = { pBuf0, pBuf1 };

    /* pBuf: the frame to read once a frame event has fired. */
    ctx->Pipe[Pipe].pBuf = pBuf0;
    CAMERA_SetRegions(&ctx->Pipe[Pipe], planes, 1U, 2U);
  }
  ctx->Pipe[Pipe].State = CAMERA_PIPE_RUNNING;

  if (HAL_DCMIPP_CSI_PIPE_DoubleBufferStart(&hcamera_dcmipp, Pipe, DCMIPP_VIRTUAL_CHANNEL0,
                                            (uint32_t)pBuf0, (uint32_t)pBuf1, Mode) != HAL_OK)
  {
    ctx->Pipe[Pipe].State = CAMERA_PIPE_STOPPED;
    ctx->Pipe[Pipe].pBuf  = NULL;
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return CAMERA_FinishStart(ctx, Pipe);
}

/* The planar packer exists on MAIN (pipe 1) only. */
static int32_t CAMERA_PlanarStartAllowed(const CAMERA_Ctx_t *ctx, uint32_t Pipe, uint32_t Format)
{
  if (Pipe != BSP_CAMERA_PIPE_MAIN)
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  if (ctx->Pipe[Pipe].PixelFormat != Format)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_SemiPlanarStart(uint32_t Instance, uint32_t Pipe,
                                   uint8_t *pBufY, uint8_t *pBufUV, uint32_t Mode)
{
  DCMIPP_SemiPlanarDstAddressTypeDef addr;
  CAMERA_Ctx_t *ctx;
  int32_t status = CAMERA_StartReady(Instance, Pipe, Mode, &ctx);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  status = CAMERA_PlanarStartAllowed(ctx, Pipe, BSP_CAMERA_PIPE_PF_YUV420_SP);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((CAMERA_StartBufferOk(pBufY) == 0U) || (CAMERA_StartBufferOk(pBufUV) == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  SET_BIT(DCMIPP->CMCR, DCMIPP_CMCR_INSEL);
  CAMERA_PrepareStart(Pipe);

  addr.YAddress  = (uint32_t)pBufY;
  addr.UVAddress = (uint32_t)pBufUV;

  {
    uint8_t *const planes[2] = { pBufY, pBufUV };

    ctx->Pipe[Pipe].pBuf = pBufY;
    CAMERA_SetRegions(&ctx->Pipe[Pipe], planes, 2U, 1U);
  }
  ctx->Pipe[Pipe].State = CAMERA_PIPE_RUNNING;

  if (HAL_DCMIPP_CSI_PIPE_SemiPlanarStart(&hcamera_dcmipp, Pipe, DCMIPP_VIRTUAL_CHANNEL0,
                                          &addr, Mode) != HAL_OK)
  {
    ctx->Pipe[Pipe].State = CAMERA_PIPE_STOPPED;
    ctx->Pipe[Pipe].pBuf  = NULL;
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return CAMERA_FinishStart(ctx, Pipe);
}

int32_t BSP_CAMERA_FullPlanarStart(uint32_t Instance, uint32_t Pipe,
                                   uint8_t *pBufY, uint8_t *pBufU, uint8_t *pBufV,
                                   uint32_t Mode)
{
  DCMIPP_FullPlanarDstAddressTypeDef addr;
  CAMERA_Ctx_t *ctx;
  int32_t status = CAMERA_StartReady(Instance, Pipe, Mode, &ctx);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  status = CAMERA_PlanarStartAllowed(ctx, Pipe, BSP_CAMERA_PIPE_PF_YUV420_FP);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((CAMERA_StartBufferOk(pBufY) == 0U) || (CAMERA_StartBufferOk(pBufU) == 0U) ||
      (CAMERA_StartBufferOk(pBufV) == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  SET_BIT(DCMIPP->CMCR, DCMIPP_CMCR_INSEL);
  CAMERA_PrepareStart(Pipe);

  addr.YAddress = (uint32_t)pBufY;
  addr.UAddress = (uint32_t)pBufU;
  addr.VAddress = (uint32_t)pBufV;

  {
    uint8_t *const planes[3] = { pBufY, pBufU, pBufV };

    ctx->Pipe[Pipe].pBuf = pBufY;
    CAMERA_SetRegions(&ctx->Pipe[Pipe], planes, 3U, 1U);
  }
  ctx->Pipe[Pipe].State = CAMERA_PIPE_RUNNING;

  if (HAL_DCMIPP_CSI_PIPE_FullPlanarStart(&hcamera_dcmipp, Pipe, DCMIPP_VIRTUAL_CHANNEL0,
                                          &addr, Mode) != HAL_OK)
  {
    ctx->Pipe[Pipe].State = CAMERA_PIPE_STOPPED;
    ctx->Pipe[Pipe].pBuf  = NULL;
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return CAMERA_FinishStart(ctx, Pipe);
}

int32_t BSP_CAMERA_SemiPlanarDoubleBufferStart(uint32_t Instance, uint32_t Pipe,
                                               uint8_t *pBufY0, uint8_t *pBufUV0,
                                               uint8_t *pBufY1, uint8_t *pBufUV1,
                                               uint32_t Mode)
{
  DCMIPP_SemiPlanarDstAddressTypeDef addr0;
  DCMIPP_SemiPlanarDstAddressTypeDef addr1;
  CAMERA_Ctx_t *ctx;
  int32_t status = CAMERA_StartReady(Instance, Pipe, Mode, &ctx);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  status = CAMERA_PlanarStartAllowed(ctx, Pipe, BSP_CAMERA_PIPE_PF_YUV420_SP);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((CAMERA_StartBufferOk(pBufY0) == 0U) || (CAMERA_StartBufferOk(pBufUV0) == 0U) ||
      (CAMERA_StartBufferOk(pBufY1) == 0U) || (CAMERA_StartBufferOk(pBufUV1) == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if ((pBufY0 == pBufY1) || (pBufUV0 == pBufUV1))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  SET_BIT(DCMIPP->CMCR, DCMIPP_CMCR_INSEL);
  CAMERA_PrepareStart(Pipe);

  addr0.YAddress  = (uint32_t)pBufY0;
  addr0.UVAddress = (uint32_t)pBufUV0;
  addr1.YAddress  = (uint32_t)pBufY1;
  addr1.UVAddress = (uint32_t)pBufUV1;

  {
    uint8_t *const planes[4] = { pBufY0, pBufUV0, pBufY1, pBufUV1 };

    ctx->Pipe[Pipe].pBuf = pBufY0;
    CAMERA_SetRegions(&ctx->Pipe[Pipe], planes, 2U, 2U);
  }
  ctx->Pipe[Pipe].State = CAMERA_PIPE_RUNNING;

  if (HAL_DCMIPP_CSI_PIPE_SemiPlanarDoubleBufferStart(&hcamera_dcmipp, Pipe,
                                                      DCMIPP_VIRTUAL_CHANNEL0,
                                                      &addr0, &addr1, Mode) != HAL_OK)
  {
    ctx->Pipe[Pipe].State = CAMERA_PIPE_STOPPED;
    ctx->Pipe[Pipe].pBuf  = NULL;
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return CAMERA_FinishStart(ctx, Pipe);
}

int32_t BSP_CAMERA_FullPlanarDoubleBufferStart(uint32_t Instance, uint32_t Pipe,
                                               uint8_t *pBufY0, uint8_t *pBufU0, uint8_t *pBufV0,
                                               uint8_t *pBufY1, uint8_t *pBufU1, uint8_t *pBufV1,
                                               uint32_t Mode)
{
  DCMIPP_FullPlanarDstAddressTypeDef addr0;
  DCMIPP_FullPlanarDstAddressTypeDef addr1;
  CAMERA_Ctx_t *ctx;
  int32_t status = CAMERA_StartReady(Instance, Pipe, Mode, &ctx);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  status = CAMERA_PlanarStartAllowed(ctx, Pipe, BSP_CAMERA_PIPE_PF_YUV420_FP);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((CAMERA_StartBufferOk(pBufY0) == 0U) || (CAMERA_StartBufferOk(pBufU0) == 0U) ||
      (CAMERA_StartBufferOk(pBufV0) == 0U) || (CAMERA_StartBufferOk(pBufY1) == 0U) ||
      (CAMERA_StartBufferOk(pBufU1) == 0U) || (CAMERA_StartBufferOk(pBufV1) == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if ((pBufY0 == pBufY1) || (pBufU0 == pBufU1) || (pBufV0 == pBufV1))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  SET_BIT(DCMIPP->CMCR, DCMIPP_CMCR_INSEL);
  CAMERA_PrepareStart(Pipe);

  addr0.YAddress = (uint32_t)pBufY0;
  addr0.UAddress = (uint32_t)pBufU0;
  addr0.VAddress = (uint32_t)pBufV0;
  addr1.YAddress = (uint32_t)pBufY1;
  addr1.UAddress = (uint32_t)pBufU1;
  addr1.VAddress = (uint32_t)pBufV1;

  {
    uint8_t *const planes[6] = { pBufY0, pBufU0, pBufV0, pBufY1, pBufU1, pBufV1 };

    ctx->Pipe[Pipe].pBuf = pBufY0;
    CAMERA_SetRegions(&ctx->Pipe[Pipe], planes, 3U, 2U);
  }
  ctx->Pipe[Pipe].State = CAMERA_PIPE_RUNNING;

  if (HAL_DCMIPP_CSI_PIPE_FullPlanarDoubleBufferStart(&hcamera_dcmipp, Pipe,
                                                      DCMIPP_VIRTUAL_CHANNEL0,
                                                      &addr0, &addr1, Mode) != HAL_OK)
  {
    ctx->Pipe[Pipe].State = CAMERA_PIPE_STOPPED;
    ctx->Pipe[Pipe].pBuf  = NULL;
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return CAMERA_FinishStart(ctx, Pipe);
}

/**
  * @brief  Stop a pipe. The sensor keeps streaming, so the receiver keeps its lock
  *         (BSP_CAMERA_SuspendSensor quietens it).
  * @retval BSP_ERROR_PERIPH_FAILURE if the pipe did not stop in time and may still be
  *         writing: call again, and reuse the buffer only after BSP_ERROR_NONE.
  */
int32_t BSP_CAMERA_Stop(uint32_t Instance, uint32_t Pipe)
{
  CAMERA_Ctx_t *ctx;
  int32_t       status = BSP_ERROR_NONE;

  if ((Instance >= BSP_CAMERA_INSTANCES_NBR) || (Pipe >= BSP_CAMERA_PIPES_NBR))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  ctx = &Camera_Ctx[Instance];

  if (ctx->Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (ctx->Pipe[Pipe].State == CAMERA_PIPE_UNCONFIGURED)
  {
    return BSP_ERROR_NO_INIT;
  }

  /* The HAL refuses starts until its own stop has run: stop any pipe it still holds. */
  if (CAMERA_HalPipeHeld(Pipe) ||
      (ctx->Pipe[Pipe].State == CAMERA_PIPE_RUNNING) ||
      (ctx->Pipe[Pipe].State == CAMERA_PIPE_SUSPENDED))
  {
    uint32_t others = 0U;
    uint32_t p;

    for (p = 0U; p < BSP_CAMERA_PIPES_NBR; p++)
    {
      if ((p != Pipe) && ((ctx->Pipe[p].State == CAMERA_PIPE_RUNNING) ||
                          (ctx->Pipe[p].State == CAMERA_PIPE_SUSPENDED)))
      {
        others = 1U;
      }
    }

    /* The virtual channel is shared, so it stops only with the last pipe. */
    if (others != 0U)
    {
      status = CAMERA_StopPipeOnly(Pipe);
      ctx->Pipe[Pipe].State   = CAMERA_PIPE_STOPPED;
      ctx->Pipe[Pipe].pBuf    = NULL;
      ctx->Pipe[Pipe].Regions = 0U;
      return status;
    }

    if (HAL_DCMIPP_CSI_PIPE_Stop(&hcamera_dcmipp, Pipe, DCMIPP_VIRTUAL_CHANNEL0) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
    ctx->Pipe[Pipe].State = CAMERA_PIPE_STOPPED;

    /* Clear the receiver's W1C flags, so the next start has no stale error. */
    CSI->FCR0 = 0xFFFFFFFFU;
    CSI->FCR1 = 0xFFFFFFFFU;
    __DSB();
    __ISB();

    /* Leave the data-type filter: it survives the stop, and rewriting it before the
     * encoder is armed prevents the arm. */
  }

  ctx->Pipe[Pipe].pBuf    = NULL;
  ctx->Pipe[Pipe].Regions = 0U;

  return status;
}

/** @brief Stop the sensor transmitting; the camera stays configured. */
int32_t BSP_CAMERA_SuspendSensor(uint32_t Instance)
{
  CAMERA_Ctx_t *ctx;

  if (Instance >= BSP_CAMERA_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  ctx = &Camera_Ctx[Instance];

  if (ctx->Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (ctx->SensorOn == 0U)
  {
    return BSP_ERROR_NONE;
  }

  if (Camera_Sensor->pDrv->Stop(Camera_Sensor->pObj) != 0)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  ctx->SensorOn = 0U;

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_HaltCapture(uint32_t Instance)
{
  CAMERA_Ctx_t *ctx;
  uint32_t      pipe;

  if (Instance >= BSP_CAMERA_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  ctx = &Camera_Ctx[Instance];

  if (ctx->Initialised == 0U)
  {
    return BSP_ERROR_NONE;
  }

  CLEAR_BIT(DCMIPP->P0FCTCR, DCMIPP_P0FCTCR_CPTREQ);
  CLEAR_BIT(DCMIPP->P1FCTCR, DCMIPP_P1FCTCR_CPTREQ);
  CLEAR_BIT(DCMIPP->P2FCTCR, DCMIPP_P2FCTCR_CPTREQ);
  __DSB();

  for (pipe = 0U; pipe < BSP_CAMERA_PIPES_NBR; pipe++)
  {
    if ((ctx->Pipe[pipe].State == CAMERA_PIPE_RUNNING) ||
        (ctx->Pipe[pipe].State == CAMERA_PIPE_SUSPENDED))
    {
      ctx->Pipe[pipe].State = CAMERA_PIPE_STOPPED;
    }
  }

  return BSP_ERROR_NONE;
}

/** @brief Restart the sensor after BSP_CAMERA_SuspendSensor. */
int32_t BSP_CAMERA_ResumeSensor(uint32_t Instance)
{
  CAMERA_Ctx_t *ctx;

  if (Instance >= BSP_CAMERA_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  ctx = &Camera_Ctx[Instance];

  if (ctx->Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (ctx->SensorOn != 0U)
  {
    return BSP_ERROR_NONE;
  }

  if (Camera_Sensor->pDrv->Start(Camera_Sensor->pObj) != 0)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  ctx->SensorOn = 1U;

  return BSP_ERROR_NONE;
}

/** @brief Withdraw a pipe's capture request, keeping its configuration and destination;
  * the current frame finishes. */
int32_t BSP_CAMERA_Suspend(uint32_t Instance, uint32_t Pipe)
{
  CAMERA_Ctx_t *ctx;

  if ((Instance >= BSP_CAMERA_INSTANCES_NBR) || (Pipe >= BSP_CAMERA_PIPES_NBR))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  ctx = &Camera_Ctx[Instance];

  if (ctx->Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (ctx->Pipe[Pipe].State != CAMERA_PIPE_RUNNING)
  {
    return BSP_ERROR_BUSY;
  }

  if (HAL_DCMIPP_PIPE_Suspend(&hcamera_dcmipp, Pipe) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  ctx->Pipe[Pipe].State = CAMERA_PIPE_SUSPENDED;

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_Resume(uint32_t Instance, uint32_t Pipe)
{
  CAMERA_Ctx_t *ctx;

  if ((Instance >= BSP_CAMERA_INSTANCES_NBR) || (Pipe >= BSP_CAMERA_PIPES_NBR))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  ctx = &Camera_Ctx[Instance];

  if (ctx->Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (ctx->Pipe[Pipe].State != CAMERA_PIPE_SUSPENDED)
  {
    return BSP_ERROR_BUSY;
  }

  if (HAL_DCMIPP_PIPE_Resume(&hcamera_dcmipp, Pipe) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  ctx->Pipe[Pipe].State = CAMERA_PIPE_RUNNING;

  return BSP_ERROR_NONE;
}

/** @brief Report the active configuration and its limits. */
int32_t BSP_CAMERA_GetCapabilities(uint32_t Instance, BSP_CAMERA_Capabilities_t *pCapabilities)
{
  const CAMERA_Ctx_t *ctx;

  if ((Instance >= BSP_CAMERA_INSTANCES_NBR) || (pCapabilities == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  ctx = &Camera_Ctx[Instance];

  if (ctx->Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  pCapabilities->Resolution        = ctx->Resolution;
  pCapabilities->PixelFormat       = ctx->PixelFormat;
  pCapabilities->Width             = ctx->Width;
  pCapabilities->Height            = ctx->Height;
  pCapabilities->MaxFrameRate      = ctx->MaxFrameRate;
  pCapabilities->LaneRateMbps      = ctx->LaneRateMbps;
  pCapabilities->PhyBitrateBucket  = ctx->PhyBucket;
  pCapabilities->NeedsHalDPhyPatch = (ctx->PhyBucket >= CAMERA_DPHY_PATCH_THRESHOLD) ? 1U : 0U;
  pCapabilities->MirrorFlip        = (Camera_Sensor->pDrv->SetMirrorFlip != NULL) ? 1U : 0U;
  pCapabilities->PictureControls   = (Camera_Sensor->pDrv->SetBrightness != NULL) ? 1U : 0U;
  pCapabilities->ResolutionChange  = (Camera_Sensor->pDrv->SetResolution != NULL) ? 1U : 0U;

  return BSP_ERROR_NONE;
}

/** @brief Bytes per frame of a configured pipe: the minimum BSP_CAMERA_Start buffer. */
int32_t BSP_CAMERA_GetFrameSize(uint32_t Instance, uint32_t Pipe, uint32_t *pSizeBytes)
{
  const CAMERA_Ctx_t *ctx;

  if ((Instance >= BSP_CAMERA_INSTANCES_NBR) || (Pipe >= BSP_CAMERA_PIPES_NBR) ||
      (pSizeBytes == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  ctx = &Camera_Ctx[Instance];

  if (ctx->Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (ctx->Pipe[Pipe].State == CAMERA_PIPE_UNCONFIGURED)
  {
    return BSP_ERROR_NO_INIT;
  }

  *pSizeBytes = ctx->Pipe[Pipe].FrameBytes;

  return BSP_ERROR_NONE;
}

/** @brief Set mirror and flip; on a raw sensor the demosaic order follows. */
int32_t BSP_CAMERA_SetMirrorFlip(uint32_t Instance, uint32_t MirrorFlip)
{
  CAMERA_Ctx_t *ctx;

  if (Instance >= BSP_CAMERA_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if ((MirrorFlip != BSP_CAMERA_MIRRORFLIP_NONE) &&
      (MirrorFlip != BSP_CAMERA_MIRRORFLIP_FLIP) &&
      (MirrorFlip != BSP_CAMERA_MIRRORFLIP_MIRROR) &&
      (MirrorFlip != BSP_CAMERA_MIRRORFLIP_BOTH))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  ctx = &Camera_Ctx[Instance];

  if (ctx->Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (Camera_Sensor->pDrv->SetMirrorFlip == NULL)
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  if (Camera_Sensor->pDrv->SetMirrorFlip(Camera_Sensor->pObj, MirrorFlip) != 0)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  ctx->MirrorFlip = MirrorFlip;

  /* Mirroring changes a raw sensor's filter order: follow it, other settings kept. */
  if ((Camera_Sensor->SensorFormat == BSP_CAMERA_PF_RAW8) && (Camera_DemosaicOn != 0U) &&
      (Camera_Demosaic.RawBayerType != Camera_Sensor->BayerOrder))
  {
    Camera_Demosaic.RawBayerType = Camera_Sensor->BayerOrder;
    if (HAL_DCMIPP_PIPE_SetISPRawBayer2RGBConfig(&hcamera_dcmipp, BSP_CAMERA_PIPE_MAIN,
                                                 &Camera_Demosaic) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_GetMirrorFlip(uint32_t Instance, uint32_t *pMirrorFlip)
{
  const CAMERA_Ctx_t *ctx;

  if ((Instance >= BSP_CAMERA_INSTANCES_NBR) || (pMirrorFlip == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  ctx = &Camera_Ctx[Instance];

  if (ctx->Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  *pMirrorFlip = ctx->MirrorFlip;

  return BSP_ERROR_NONE;
}

/* Sensor picture controls: check, apply, then cache what the sensor accepted. */

/* Available is a flag, not the driver slot: C does not define converting a function
 * pointer to void *. */
static int32_t CAMERA_SensorControlReady(uint32_t Instance, uint32_t Available,
                                         CAMERA_Ctx_t **ppCtx)
{
  CAMERA_Ctx_t *ctx;

  if (Instance >= BSP_CAMERA_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  ctx = &Camera_Ctx[Instance];

  if (ctx->Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (Available == 0U)
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  *ppCtx = ctx;

  return BSP_ERROR_NONE;
}

static int32_t CAMERA_SensorReadReady(uint32_t Instance, const void *pOut,
                                      CAMERA_Ctx_t **ppCtx)
{
  CAMERA_Ctx_t *ctx;

  if ((Instance >= BSP_CAMERA_INSTANCES_NBR) || (pOut == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  ctx = &Camera_Ctx[Instance];

  if (ctx->Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  *ppCtx = ctx;

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_SetBrightness(uint32_t Instance, int32_t Level)
{
  CAMERA_Ctx_t *ctx;
  int32_t status;

  status = CAMERA_SensorControlReady(Instance,
                                     (Camera_Sensor->pDrv->SetBrightness != NULL) ? 1U : 0U,
                                     &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((Level < BSP_CAMERA_BRIGHTNESS_MIN) || (Level > BSP_CAMERA_BRIGHTNESS_MAX))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Negated: the driver's sign is reversed on this board; the cache keeps the caller's. */
  if (Camera_Sensor->pDrv->SetBrightness(Camera_Sensor->pObj, -Level) != 0)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  ctx->Brightness = Level;

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_GetBrightness(uint32_t Instance, int32_t *pLevel)
{
  CAMERA_Ctx_t *ctx;
  int32_t status = CAMERA_SensorReadReady(Instance, pLevel, &ctx);

  if (status == BSP_ERROR_NONE)
  {
    *pLevel = ctx->Brightness;
  }

  return status;
}

int32_t BSP_CAMERA_SetSaturation(uint32_t Instance, int32_t Level)
{
  CAMERA_Ctx_t *ctx;
  int32_t status;

  status = CAMERA_SensorControlReady(Instance,
                                     (Camera_Sensor->pDrv->SetSaturation != NULL) ? 1U : 0U,
                                     &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((Level < BSP_CAMERA_SATURATION_MIN) || (Level > BSP_CAMERA_SATURATION_MAX))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Camera_Sensor->pDrv->SetSaturation(Camera_Sensor->pObj, Level) != 0)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  ctx->Saturation = Level;

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_GetSaturation(uint32_t Instance, int32_t *pLevel)
{
  CAMERA_Ctx_t *ctx;
  int32_t status = CAMERA_SensorReadReady(Instance, pLevel, &ctx);

  if (status == BSP_ERROR_NONE)
  {
    *pLevel = ctx->Saturation;
  }

  return status;
}

int32_t BSP_CAMERA_SetContrast(uint32_t Instance, int32_t Level)
{
  CAMERA_Ctx_t *ctx;
  int32_t status;

  status = CAMERA_SensorControlReady(Instance,
                                     (Camera_Sensor->pDrv->SetContrast != NULL) ? 1U : 0U,
                                     &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((Level < BSP_CAMERA_CONTRAST_MIN) || (Level > BSP_CAMERA_CONTRAST_MAX))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Camera_Sensor->pDrv->SetContrast(Camera_Sensor->pObj, Level) != 0)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  ctx->Contrast = Level;

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_GetContrast(uint32_t Instance, int32_t *pLevel)
{
  CAMERA_Ctx_t *ctx;
  int32_t status = CAMERA_SensorReadReady(Instance, pLevel, &ctx);

  if (status == BSP_ERROR_NONE)
  {
    *pLevel = ctx->Contrast;
  }

  return status;
}

/** @brief Rotate the hue, in 30-degree steps. */
int32_t BSP_CAMERA_SetHueDegree(uint32_t Instance, int32_t Degree)
{
  CAMERA_Ctx_t *ctx;
  int32_t status;

  status = CAMERA_SensorControlReady(Instance,
                                     (Camera_Sensor->pDrv->SetHueDegree != NULL) ? 1U : 0U,
                                     &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((Degree < BSP_CAMERA_HUE_DEGREE_MIN) || (Degree > BSP_CAMERA_HUE_DEGREE_MAX))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Camera_Sensor->pDrv->SetHueDegree(Camera_Sensor->pObj, Degree) != 0)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  ctx->HueDegree = Degree;

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_GetHueDegree(uint32_t Instance, int32_t *pDegree)
{
  CAMERA_Ctx_t *ctx;
  int32_t status = CAMERA_SensorReadReady(Instance, pDegree, &ctx);

  if (status == BSP_ERROR_NONE)
  {
    *pDegree = ctx->HueDegree;
  }

  return status;
}

/** @brief Select a white-balance preset (BSP_CAMERA_LIGHT_*). */
int32_t BSP_CAMERA_SetLightMode(uint32_t Instance, uint32_t LightMode)
{
  CAMERA_Ctx_t *ctx;
  int32_t status;

  status = CAMERA_SensorControlReady(Instance,
                                     (Camera_Sensor->pDrv->SetLightMode != NULL) ? 1U : 0U,
                                     &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((LightMode != BSP_CAMERA_LIGHT_AUTO) && (LightMode != BSP_CAMERA_LIGHT_SUNNY) &&
      (LightMode != BSP_CAMERA_LIGHT_OFFICE) && (LightMode != BSP_CAMERA_LIGHT_HOME) &&
      (LightMode != BSP_CAMERA_LIGHT_CLOUDY))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Camera_Sensor->pDrv->SetLightMode(Camera_Sensor->pObj, LightMode) != 0)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  ctx->LightMode = LightMode;

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_GetLightMode(uint32_t Instance, uint32_t *pLightMode)
{
  CAMERA_Ctx_t *ctx;
  int32_t status = CAMERA_SensorReadReady(Instance, pLightMode, &ctx);

  if (status == BSP_ERROR_NONE)
  {
    *pLightMode = ctx->LightMode;
  }

  return status;
}

int32_t BSP_CAMERA_SetColorEffect(uint32_t Instance, uint32_t Effect)
{
  CAMERA_Ctx_t *ctx;
  int32_t status;

  status = CAMERA_SensorControlReady(Instance,
                                     (Camera_Sensor->pDrv->SetColorEffect != NULL) ? 1U : 0U,
                                     &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((Effect != BSP_CAMERA_COLOR_EFFECT_NONE) && (Effect != BSP_CAMERA_COLOR_EFFECT_BLUE) &&
      (Effect != BSP_CAMERA_COLOR_EFFECT_RED) && (Effect != BSP_CAMERA_COLOR_EFFECT_GREEN) &&
      (Effect != BSP_CAMERA_COLOR_EFFECT_BW) && (Effect != BSP_CAMERA_COLOR_EFFECT_SEPIA) &&
      (Effect != BSP_CAMERA_COLOR_EFFECT_NEGATIVE))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Camera_Sensor->pDrv->SetColorEffect(Camera_Sensor->pObj, Effect) != 0)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  ctx->ColorEffect = Effect;

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_GetColorEffect(uint32_t Instance, uint32_t *pEffect)
{
  CAMERA_Ctx_t *ctx;
  int32_t status = CAMERA_SensorReadReady(Instance, pEffect, &ctx);

  if (status == BSP_ERROR_NONE)
  {
    *pEffect = ctx->ColorEffect;
  }

  return status;
}

/**
  * @brief  Digital zoom in the sensor; the frame size on the link does not change.
  * @note   On the OV5640 only x1 is accepted from 720p up; a zoom set in a smaller mode
  *         returns with it.
  * @retval BSP_ERROR_FEATURE_NOT_SUPPORTED if the sensor cannot give that zoom now.
  */
int32_t BSP_CAMERA_SetZoom(uint32_t Instance, uint32_t Zoom)
{
  CAMERA_Ctx_t *ctx;
  int32_t status;

  status = CAMERA_SensorControlReady(Instance,
                                     (Camera_Sensor->pDrv->SetZoom != NULL) ? 1U : 0U,
                                     &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if ((Zoom != BSP_CAMERA_ZOOM_x1) && (Zoom != BSP_CAMERA_ZOOM_x2) &&
      (Zoom != BSP_CAMERA_ZOOM_x4) && (Zoom != BSP_CAMERA_ZOOM_x8))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  status = Camera_Sensor->pDrv->SetZoom(Camera_Sensor->pObj, Zoom);
  if (status == BSP_ERROR_FEATURE_NOT_SUPPORTED)
  {
    return status;
  }
  if (status != 0)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  ctx->Zoom = Zoom;

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_GetZoom(uint32_t Instance, uint32_t *pZoom)
{
  CAMERA_Ctx_t *ctx;
  int32_t status = CAMERA_SensorReadReady(Instance, pZoom, &ctx);

  if (status == BSP_ERROR_NONE)
  {
    *pZoom = ctx->Zoom;
  }

  return status;
}

static int32_t CAMERA_SetNightMode(uint32_t Instance, uint32_t Enable)
{
  CAMERA_Ctx_t *ctx;
  int32_t status;

  status = CAMERA_SensorControlReady(Instance,
                                     (Camera_Sensor->pDrv->SetNightMode != NULL) ? 1U : 0U,
                                     &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (Camera_Sensor->pDrv->SetNightMode(Camera_Sensor->pObj, Enable) != 0)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  ctx->NightMode = Enable;

  return BSP_ERROR_NONE;
}

/** @brief Let the sensor lower its frame rate in low light, for longer exposures. */
int32_t BSP_CAMERA_EnableNightMode(uint32_t Instance)
{
  return CAMERA_SetNightMode(Instance, BSP_CAMERA_NIGHT_MODE_ENABLE);
}

/** @brief Hold the sensor at its configured frame rate. */
int32_t BSP_CAMERA_DisableNightMode(uint32_t Instance)
{
  return CAMERA_SetNightMode(Instance, BSP_CAMERA_NIGHT_MODE_DISABLE);
}

int32_t BSP_CAMERA_GetNightMode(uint32_t Instance, uint32_t *pEnabled)
{
  CAMERA_Ctx_t *ctx;
  int32_t status = CAMERA_SensorReadReady(Instance, pEnabled, &ctx);

  if (status == BSP_ERROR_NONE)
  {
    *pEnabled = ctx->NightMode;
  }

  return status;
}

/* Re-selects the D-PHY bucket if the new mode needs another. */
int32_t BSP_CAMERA_SetResolution(uint32_t Instance, uint32_t Resolution)
{
  const BSP_CAMERA_SensorMode_t *mode;
  CAMERA_Ctx_t *ctx;
  uint32_t      bucket;
  uint32_t      pipe;
  int32_t       status;

  status = CAMERA_SensorControlReady(Instance,
                                     (Camera_Sensor->pDrv->SetResolution != NULL) ? 1U : 0U,
                                     &ctx);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (CAMERA_AnyPipeRunning(ctx) != 0U)
  {
    return BSP_ERROR_BUSY;
  }

  mode = CAMERA_FindMode(Resolution);
  if (mode == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Resolution == ctx->Resolution)
  {
    return BSP_ERROR_NONE;
  }

  CAMERA_ClearStaleErrorState();

  bucket = CAMERA_PhyBucket(mode->LaneRateMbps);
  if (bucket != ctx->PhyBucket)
  {
    status = CAMERA_CsiConfig(bucket);
    if (status != BSP_ERROR_NONE)
    {
      return status;
    }
    ctx->PhyBucket = bucket;
  }

  if (Camera_Sensor->pDrv->SetResolution(Camera_Sensor->pObj, Resolution) != 0)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  ctx->Resolution   = Resolution;
  ctx->Width        = mode->Width;
  ctx->Height       = mode->Height;
  ctx->MaxFrameRate = mode->MaxFrameRate;
  ctx->LaneRateMbps = mode->LaneRateMbps;

  for (pipe = 0U; pipe < BSP_CAMERA_PIPES_NBR; pipe++)
  {
    ctx->Pipe[pipe].State       = CAMERA_PIPE_UNCONFIGURED;
    ctx->Pipe[pipe].PixelFormat = 0U;
    ctx->Pipe[pipe].Width       = 0U;
    ctx->Pipe[pipe].Height      = 0U;
    ctx->Pipe[pipe].FrameBytes  = 0U;
    ctx->Pipe[pipe].pBuf        = NULL;
    ctx->Pipe[pipe].Regions     = 0U;
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_GetResolution(uint32_t Instance, uint32_t *pResolution)
{
  CAMERA_Ctx_t *ctx;
  int32_t status = CAMERA_SensorReadReady(Instance, pResolution, &ctx);

  if (status == BSP_ERROR_NONE)
  {
    *pResolution = ctx->Resolution;
  }

  return status;
}

int32_t BSP_CAMERA_GetPixelFormat(uint32_t Instance, uint32_t *pPixelFormat)
{
  CAMERA_Ctx_t *ctx;
  int32_t status = CAMERA_SensorReadReady(Instance, pPixelFormat, &ctx);

  if (status == BSP_ERROR_NONE)
  {
    *pPixelFormat = ctx->PixelFormat;
  }

  return status;
}

int32_t BSP_CAMERA_ConfigLineBuffer(uint32_t Instance, uint32_t Pipe,
                                    uint32_t WrapLines, uint32_t EventLines)
{
  CAMERA_Ctx_t *ctx;
  int32_t wrap_index = 0;
  int32_t event_index = 0;

  if ((Instance >= BSP_CAMERA_INSTANCES_NBR) || (Pipe >= BSP_CAMERA_PIPES_NBR))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  ctx = &Camera_Ctx[Instance];

  if (ctx->Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  /* Not while running: unwrapping a pipe started on a band would overrun that buffer. */
  if ((ctx->Pipe[Pipe].State == CAMERA_PIPE_RUNNING) ||
      (ctx->Pipe[Pipe].State == CAMERA_PIPE_SUSPENDED))
  {
    return BSP_ERROR_BUSY;
  }

  if (WrapLines != BSP_CAMERA_LINES_OFF)
  {
    /* Only the two processing pipes carry the wrap counter. */
    if (Pipe == BSP_CAMERA_PIPE_DUMP)
    {
      return BSP_ERROR_FEATURE_NOT_SUPPORTED;
    }

    wrap_index = CAMERA_LineCountIndex(WrapLines);

    if (wrap_index < 0)
    {
      return BSP_ERROR_WRONG_PARAM;
    }
  }

  if (EventLines != BSP_CAMERA_LINES_OFF)
  {
    event_index = CAMERA_LineCountIndex(EventLines);

    if (event_index < 0)
    {
      return BSP_ERROR_WRONG_PARAM;
    }
  }

  CAMERA_ClearStaleErrorState();

  if (WrapLines != BSP_CAMERA_LINES_OFF)
  {
    /* Depth before enable, so a stale depth is never live over the new buffer. */
    if (HAL_DCMIPP_PIPE_SetLineWrappingConfig(&hcamera_dcmipp, Pipe,
                                              Camera_WrapCode[wrap_index]) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }

    if (HAL_DCMIPP_PIPE_EnableLineWrapping(&hcamera_dcmipp, Pipe) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
  }
  else if (Pipe != BSP_CAMERA_PIPE_DUMP)
  {
    if (HAL_DCMIPP_PIPE_DisableLineWrapping(&hcamera_dcmipp, Pipe) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
  }
  else
  {
    /* The dump pipe never wraps, so there is nothing to switch off. */
  }

  ctx->Pipe[Pipe].LineWrapped = (WrapLines != BSP_CAMERA_LINES_OFF) ? 1U : 0U;

  if (EventLines != BSP_CAMERA_LINES_OFF)
  {
    if (HAL_DCMIPP_PIPE_EnableLineEvent(&hcamera_dcmipp, Pipe,
                                        Camera_LineEventCode[event_index]) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
    ctx->Pipe[Pipe].LineEventOn   = 1U;
    ctx->Pipe[Pipe].LineEventCode = Camera_LineEventCode[event_index];
  }
  else
  {
    if (HAL_DCMIPP_PIPE_DisableLineEvent(&hcamera_dcmipp, Pipe) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
    ctx->Pipe[Pipe].LineEventOn = 0U;
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_SetIspDemosaicing(uint32_t Instance, uint32_t Pipe,
                                     const DCMIPP_RawBayer2RGBConfTypeDef *pConfig)
{
  int32_t status = CAMERA_CheckIspPipe(Instance, Pipe, 0U);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pConfig == NULL)
  {
    Camera_DemosaicOn = 0U;
    return (HAL_DCMIPP_PIPE_DisableISPRawBayer2RGB(&hcamera_dcmipp, Pipe) == HAL_OK)
           ? BSP_ERROR_NONE : BSP_ERROR_PERIPH_FAILURE;
  }

  if (HAL_DCMIPP_PIPE_SetISPRawBayer2RGBConfig(&hcamera_dcmipp, Pipe, pConfig) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  if (HAL_DCMIPP_PIPE_EnableISPRawBayer2RGB(&hcamera_dcmipp, Pipe) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  Camera_Demosaic   = *pConfig;
  Camera_DemosaicOn = 1U;
  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_SetIspBlackLevel(uint32_t Instance, uint32_t Pipe,
                                    const DCMIPP_BlackLevelConfTypeDef *pConfig)
{
  int32_t status = CAMERA_CheckIspPipe(Instance, Pipe, 0U);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pConfig == NULL)
  {
    return (HAL_DCMIPP_PIPE_DisableISPBlackLevelCalibration(&hcamera_dcmipp, Pipe) == HAL_OK)
           ? BSP_ERROR_NONE : BSP_ERROR_PERIPH_FAILURE;
  }

  if (HAL_DCMIPP_PIPE_SetISPBlackLevelCalibrationConfig(&hcamera_dcmipp, Pipe, pConfig) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return (HAL_DCMIPP_PIPE_EnableISPBlackLevelCalibration(&hcamera_dcmipp, Pipe) == HAL_OK)
         ? BSP_ERROR_NONE : BSP_ERROR_PERIPH_FAILURE;
}

int32_t BSP_CAMERA_SetIspGain(uint32_t Instance, uint32_t Pipe,
                              const DCMIPP_ExposureConfTypeDef *pConfig)
{
  int32_t status = CAMERA_CheckIspPipe(Instance, Pipe, 0U);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pConfig == NULL)
  {
    return (HAL_DCMIPP_PIPE_DisableISPExposure(&hcamera_dcmipp, Pipe) == HAL_OK)
           ? BSP_ERROR_NONE : BSP_ERROR_PERIPH_FAILURE;
  }

  if (HAL_DCMIPP_PIPE_SetISPExposureConfig(&hcamera_dcmipp, Pipe, pConfig) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return (HAL_DCMIPP_PIPE_EnableISPExposure(&hcamera_dcmipp, Pipe) == HAL_OK)
         ? BSP_ERROR_NONE : BSP_ERROR_PERIPH_FAILURE;
}

int32_t BSP_CAMERA_SetIspColorConversion(uint32_t Instance, uint32_t Pipe,
                                         const DCMIPP_ColorConversionConfTypeDef *pConfig)
{
  int32_t status = CAMERA_CheckIspPipe(Instance, Pipe, 0U);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pConfig == NULL)
  {
    return (HAL_DCMIPP_PIPE_DisableISPColorConversion(&hcamera_dcmipp, Pipe) == HAL_OK)
           ? BSP_ERROR_NONE : BSP_ERROR_PERIPH_FAILURE;
  }

  if (HAL_DCMIPP_PIPE_SetISPColorConversionConfig(&hcamera_dcmipp, Pipe, pConfig) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return (HAL_DCMIPP_PIPE_EnableISPColorConversion(&hcamera_dcmipp, Pipe) == HAL_OK)
         ? BSP_ERROR_NONE : BSP_ERROR_PERIPH_FAILURE;
}

int32_t BSP_CAMERA_SetIspGamma(uint32_t Instance, uint32_t Pipe, uint32_t Enable)
{
  int32_t status = CAMERA_CheckIspPipe(Instance, Pipe, 1U);
  HAL_StatusTypeDef hal_status;

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  hal_status = (Enable != 0U)
               ? HAL_DCMIPP_PIPE_EnableGammaConversion(&hcamera_dcmipp, Pipe)
               : HAL_DCMIPP_PIPE_DisableGammaConversion(&hcamera_dcmipp, Pipe);

  return (hal_status == HAL_OK) ? BSP_ERROR_NONE : BSP_ERROR_PERIPH_FAILURE;
}

/* Weak application callbacks */

__weak void BSP_CAMERA_LineEventCallback(uint32_t Instance, uint32_t Pipe)
{
  (void)Instance;
  (void)Pipe;
}

__weak void BSP_CAMERA_FrameEventCallback(uint32_t Instance, uint32_t Pipe)
{
  (void)Instance;
  (void)Pipe;
}

__weak void BSP_CAMERA_VsyncEventCallback(uint32_t Instance, uint32_t Pipe)
{
  (void)Instance;
  (void)Pipe;
}

__weak void BSP_CAMERA_ErrorCallback(uint32_t Instance, uint32_t Pipe)
{
  (void)Instance;
  (void)Pipe;
}

/* HAL callbacks and interrupt entry points. The HAL callbacks are strong: an application
 * that also defines them fails to link. */

void HAL_DCMIPP_PIPE_FrameEventCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t Pipe)
{
  if ((hdcmipp != &hcamera_dcmipp) || (Pipe >= BSP_CAMERA_PIPES_NBR))
  {
    return;
  }

  /* A snapshot ends here: the HAL has already returned the pipe to ready. */
  if ((hcamera_dcmipp.PipeState[Pipe] == HAL_DCMIPP_PIPE_STATE_READY) &&
      (Camera_Ctx[0].Pipe[Pipe].State == CAMERA_PIPE_RUNNING))
  {
    Camera_Ctx[0].Pipe[Pipe].State = CAMERA_PIPE_STOPPED;
  }

  CAMERA_InvalidateFrame(&Camera_Ctx[0].Pipe[Pipe]);

  BSP_CAMERA_FrameEventCallback(0U, Pipe);
}

void HAL_DCMIPP_PIPE_VsyncEventCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t Pipe)
{
  if ((hdcmipp != &hcamera_dcmipp) || (Pipe >= BSP_CAMERA_PIPES_NBR))
  {
    return;
  }

  BSP_CAMERA_VsyncEventCallback(0U, Pipe);
}

void HAL_DCMIPP_PIPE_LineEventCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t Pipe)
{
  if ((hdcmipp != &hcamera_dcmipp) || (Pipe >= BSP_CAMERA_PIPES_NBR))
  {
    return;
  }

  BSP_CAMERA_LineEventCallback(0U, Pipe);
}

void HAL_DCMIPP_PIPE_ErrorCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t Pipe)
{
  if ((hdcmipp != &hcamera_dcmipp) || (Pipe >= BSP_CAMERA_PIPES_NBR))
  {
    return;
  }

  BSP_CAMERA_ErrorCallback(0U, Pipe);
}

void HAL_DCMIPP_ErrorCallback(DCMIPP_HandleTypeDef *hdcmipp)
{
  if (hdcmipp != &hcamera_dcmipp)
  {
    return;
  }

  BSP_CAMERA_ErrorCallback(0U, BSP_CAMERA_PIPE_NONE);
}

/* A lane error is upstream of the pipes: reported against none. */
void HAL_DCMIPP_CSI_LineErrorCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t DataLane)
{
  (void)DataLane;

  if (hdcmipp != &hcamera_dcmipp)
  {
    return;
  }

  BSP_CAMERA_ErrorCallback(0U, BSP_CAMERA_PIPE_NONE);
}

void BSP_CAMERA_DCMIPP_IRQHandler(uint32_t Instance)
{
  if (Instance < BSP_CAMERA_INSTANCES_NBR)
  {
    HAL_DCMIPP_IRQHandler(&hcamera_dcmipp);
  }
}

void BSP_CAMERA_CSI_IRQHandler(uint32_t Instance)
{
  if (Instance < BSP_CAMERA_INSTANCES_NBR)
  {
    HAL_DCMIPP_CSI_IRQHandler(&hcamera_dcmipp);
  }
}

void BSP_CAMERA_IRQHandler(uint32_t Instance)
{
  BSP_CAMERA_DCMIPP_IRQHandler(Instance);
}

/* MSP, called from HAL_DCMIPP_Init and HAL_DCMIPP_DeInit */

void HAL_DCMIPP_MspInit(DCMIPP_HandleTypeDef *hdcmipp)
{
  if (hdcmipp->Instance != DCMIPP)
  {
    return;
  }

  __HAL_RCC_DCMIPP_CLK_ENABLE();
  __HAL_RCC_DCMIPP_FORCE_RESET();
  __HAL_RCC_DCMIPP_RELEASE_RESET();

  __HAL_RCC_CSI_CLK_ENABLE();
  __HAL_RCC_CSI_FORCE_RESET();
  __HAL_RCC_CSI_RELEASE_RESET();

  CAMERA_ClockConfig();

  HAL_NVIC_SetPriority(DCMIPP_IRQn, BSP_CAMERA_IT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DCMIPP_IRQn);

  HAL_NVIC_SetPriority(CSI_IRQn, BSP_CAMERA_IT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(CSI_IRQn);
}

void HAL_DCMIPP_MspDeInit(DCMIPP_HandleTypeDef *hdcmipp)
{
  if (hdcmipp->Instance != DCMIPP)
  {
    return;
  }

  HAL_NVIC_DisableIRQ(DCMIPP_IRQn);
  HAL_NVIC_DisableIRQ(CSI_IRQn);

  /* Read back before reset and clock gating: an APB5 write overtaken by the gate never
   * completes, and stalls the next access on that bus. */
  (void)READ_REG(CSI->CR);
  (void)READ_REG(hdcmipp->Instance->CMCR);
  __DSB();

  __HAL_RCC_CSI_FORCE_RESET();
  __HAL_RCC_CSI_RELEASE_RESET();
  __HAL_RCC_CSI_CLK_DISABLE();

  __HAL_RCC_DCMIPP_FORCE_RESET();
  __HAL_RCC_DCMIPP_RELEASE_RESET();
  __HAL_RCC_DCMIPP_CLK_DISABLE();
}

#if defined(USE_HAL_DCMIPP_REGISTER_CALLBACKS) && (USE_HAL_DCMIPP_REGISTER_CALLBACKS == 1U)
/* Registered callbacks: Init installs the MSP routines above on the handle;
 * BSP_CAMERA_RegisterMspCallbacks replaces them. */

static int32_t CAMERA_RegisterMsp(pDCMIPP_CallbackTypeDef pInit,
                                  pDCMIPP_CallbackTypeDef pDeInit)
{
  if ((pInit == NULL) || (pDeInit == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (HAL_DCMIPP_RegisterCallback(&hcamera_dcmipp, HAL_DCMIPP_MSPINIT_CB_ID, pInit) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  if (HAL_DCMIPP_RegisterCallback(&hcamera_dcmipp, HAL_DCMIPP_MSPDEINIT_CB_ID, pDeInit) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  Camera_MspCbValid = 1U;

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_RegisterDefaultMspCallbacks(uint32_t Instance)
{
  if (Instance >= BSP_CAMERA_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  return CAMERA_RegisterMsp(HAL_DCMIPP_MspInit, HAL_DCMIPP_MspDeInit);
}

int32_t BSP_CAMERA_RegisterMspCallbacks(uint32_t Instance, BSP_CAMERA_Cb_t *Callbacks)
{
  if ((Instance >= BSP_CAMERA_INSTANCES_NBR) || (Callbacks == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  return CAMERA_RegisterMsp(Callbacks->pMspInitCb, Callbacks->pMspDeInitCb);
}
#endif /* USE_HAL_DCMIPP_REGISTER_CALLBACKS */
