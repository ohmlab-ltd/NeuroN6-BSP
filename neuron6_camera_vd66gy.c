/**
  * @file    neuron6_camera_vd66gy.c
  * @brief   VD66GY global-shutter image sensor as a Neuro N6 camera sensor.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include <stdint.h>
#include <string.h>

#include "neuron6_camera_vd66gy.h"
#include "neuron6.h"
#include "neuron6_bus.h"
#include "vd6g.h"

#define VD66GY_REG_MODEL_ID         0x0000U

/* The link runs at 804 Mbps; reporting 800 picks the receiver's nearer rate bucket. */
#define VD66GY_LANE_RATE_MBPS       800U

/* 9.5 us lines (pixel clock 160.8 MHz) for modes wider than VD66GY_WIDE_WIDTH: at the
 * sensor's own 7.7 us, a 1120-pixel macroblock row comes every 123 us, faster than the
 * encoder's line-buffer path codes one (about 135 us). Global shutter: same picture. */
#define VD66GY_WIDE_LINE_LENGTH     1528

#define VD66GY_WIDE_WIDTH           1024U

#define VD66GY_MODES_NBR            9U

static VD6G_Ctx_t Vd66gy_Ctx;

static uint8_t Vd66gy_Initialised;

/* The frame length is a 16-bit line count, so 1 fps (130,097 lines) wraps: floor 2. */
#define VD66GY_MIN_FPS                  2U

/* From BSP_CAMERA_VD66GY_SetFrameRate; 0 = each mode's ceiling. */
static uint32_t Vd66gy_FrameRate;

/* In BSP_CAMERA_VD66GY_R* order; rate ceilings are this board's, not the datasheet's. */
static const BSP_CAMERA_SensorMode_t Vd66gy_Modes[VD66GY_MODES_NBR] =
{
  { BSP_CAMERA_VD66GY_R320x240,   320U,  240U,  88U, VD66GY_LANE_RATE_MBPS },
  { BSP_CAMERA_VD66GY_R640x480,   640U,  480U,  88U, VD66GY_LANE_RATE_MBPS },
  { BSP_CAMERA_VD66GY_R480x640,   480U,  640U,  88U, VD66GY_LANE_RATE_MBPS },
  { BSP_CAMERA_VD66GY_R1024x768, 1024U,  768U,  60U, VD66GY_LANE_RATE_MBPS },
  { BSP_CAMERA_VD66GY_R768x1024,  768U, 1024U,  60U, VD66GY_LANE_RATE_MBPS },
  { BSP_CAMERA_VD66GY_R720x1280,  720U, 1280U,  60U, VD66GY_LANE_RATE_MBPS },
  { BSP_CAMERA_VD66GY_R1024x1280,1024U, 1280U,  45U, VD66GY_LANE_RATE_MBPS },
  { BSP_CAMERA_VD66GY_R1120x720, 1120U,  720U,  60U, VD66GY_LANE_RATE_MBPS },
  { BSP_CAMERA_VD66GY_R1120x1364,1120U, 1364U,  45U, VD66GY_LANE_RATE_MBPS },
};

/* The driver's resolution codes, in the same order. */
static const VD6G_Res_t Vd66gy_DriverRes[VD66GY_MODES_NBR] =
{
  VD6G_RES_QVGA_320_240,
  VD6G_RES_VGA_640_480,
  VD6G_RES_VGA_PORTRAIT_480_640,
  VD6G_RES_XGA_1024_768,
  VD6G_RES_XGA_PORTRAIT_768_1024,
  VD6G_RES_720P_PORTRAIT_720_1280,
  VD6G_RES_SXGA_PORTRAIT_1024_1280,
  VD6G_RES_PORTRAIT_1120_720,
  VD6G_RES_FULL_1120_1364,
};

/* Not const: the colour-filter order is written in once the sensor is initialised. */
static BSP_CAMERA_Sensor_t Vd66gy_Sensor;

static void    Vd66gy_Shutdown(VD6G_Ctx_t *pCtx, int Value);
static int     Vd66gy_Read8(VD6G_Ctx_t *pCtx, uint16_t Address, uint8_t *pValue);
static int     Vd66gy_Read16(VD6G_Ctx_t *pCtx, uint16_t Address, uint16_t *pValue);
static int     Vd66gy_Read32(VD6G_Ctx_t *pCtx, uint16_t Address, uint32_t *pValue);
static int     Vd66gy_Write8(VD6G_Ctx_t *pCtx, uint16_t Address, uint8_t Value);
static int     Vd66gy_Write16(VD6G_Ctx_t *pCtx, uint16_t Address, uint16_t Value);
static int     Vd66gy_Write32(VD6G_Ctx_t *pCtx, uint16_t Address, uint32_t Value);
static int     Vd66gy_WriteArray(VD6G_Ctx_t *pCtx, uint16_t Address, uint8_t *pData, int Length);
static void    Vd66gy_Delay(VD6G_Ctx_t *pCtx, uint32_t Milliseconds);
static void    Vd66gy_Log(VD6G_Ctx_t *pCtx, int Level, const char *pFormat, va_list Args);
static void    Vd66gy_BindCallbacks(void);
static uint32_t Vd66gy_MapBayer(VD6G_BayerType_t Bayer);
static const BSP_CAMERA_SensorMode_t *Vd66gy_FindMode(uint32_t Resolution);

static int32_t Vd66gy_DrvInit(void *pObj, uint32_t Resolution, uint32_t PixelFormat);
static int32_t Vd66gy_DrvDeInit(void *pObj);
static int32_t Vd66gy_DrvReadID(void *pObj, uint32_t *pId);
static int32_t Vd66gy_DrvStart(void *pObj);
static int32_t Vd66gy_DrvStop(void *pObj);
static int32_t Vd66gy_DrvSetMirrorFlip(void *pObj, uint32_t MirrorFlip);

/* Picture controls are NULL: the sensor sends raw data, so they belong to the DCMIPP. */
static const BSP_CAMERA_SensorDrv_t Vd66gy_Drv =
{
  Vd66gy_DrvInit,
  Vd66gy_DrvDeInit,
  Vd66gy_DrvReadID,
  Vd66gy_DrvStart,
  Vd66gy_DrvStop,
  Vd66gy_DrvSetMirrorFlip,
  NULL,   /* SetBrightness  - see above; use BSP_CAMERA_VD66GY_SetBrightness */
  NULL,   /* SetSaturation  */
  NULL,   /* SetContrast    */
  NULL,   /* SetHueDegree   */
  NULL,   /* SetLightMode   */
  NULL,   /* SetColorEffect */
  NULL,   /* SetZoom        */
  NULL,   /* SetNightMode   */
  NULL,   /* SetResolution  - re-initialise to change mode */
};

/* No-op: the carrier holds the shutdown line, and the board has no pin for it. */
static void Vd66gy_Shutdown(VD6G_Ctx_t *pCtx, int Value)
{
  UNUSED(pCtx);
  UNUSED(Value);
}

static int Vd66gy_Read8(VD6G_Ctx_t *pCtx, uint16_t Address, uint8_t *pValue)
{
  UNUSED(pCtx);

  return (BSP_I2C1_ReadReg16(BSP_CAMERA_VD66GY_I2C_ADDRESS, Address,
                             pValue, 1U) == BSP_ERROR_NONE) ? 0 : -1;
}

/* Multi-byte values are least significant byte first, unlike the register address. */
static int Vd66gy_Read16(VD6G_Ctx_t *pCtx, uint16_t Address, uint16_t *pValue)
{
  uint8_t buffer[2];

  UNUSED(pCtx);

  if (BSP_I2C1_ReadReg16(BSP_CAMERA_VD66GY_I2C_ADDRESS, Address,
                         buffer, 2U) != BSP_ERROR_NONE)
  {
    return -1;
  }

  *pValue = (uint16_t)((uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8));

  return 0;
}

static int Vd66gy_Read32(VD6G_Ctx_t *pCtx, uint16_t Address, uint32_t *pValue)
{
  uint8_t buffer[4];

  UNUSED(pCtx);

  if (BSP_I2C1_ReadReg16(BSP_CAMERA_VD66GY_I2C_ADDRESS, Address,
                         buffer, 4U) != BSP_ERROR_NONE)
  {
    return -1;
  }

  *pValue = (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) |
            ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24);

  return 0;
}

static int Vd66gy_Write8(VD6G_Ctx_t *pCtx, uint16_t Address, uint8_t Value)
{
  uint8_t value = Value;

  UNUSED(pCtx);

  return (BSP_I2C1_WriteReg16(BSP_CAMERA_VD66GY_I2C_ADDRESS, Address,
                              &value, 1U) == BSP_ERROR_NONE) ? 0 : -1;
}

static int Vd66gy_Write16(VD6G_Ctx_t *pCtx, uint16_t Address, uint16_t Value)
{
  uint8_t buffer[2];

  UNUSED(pCtx);

  buffer[0] = (uint8_t)(Value & 0xFFU);
  buffer[1] = (uint8_t)(Value >> 8);

  return (BSP_I2C1_WriteReg16(BSP_CAMERA_VD66GY_I2C_ADDRESS, Address,
                              buffer, 2U) == BSP_ERROR_NONE) ? 0 : -1;
}

static int Vd66gy_Write32(VD6G_Ctx_t *pCtx, uint16_t Address, uint32_t Value)
{
  uint8_t buffer[4];

  UNUSED(pCtx);

  buffer[0] = (uint8_t)(Value & 0xFFU);
  buffer[1] = (uint8_t)((Value >> 8) & 0xFFU);
  buffer[2] = (uint8_t)((Value >> 16) & 0xFFU);
  buffer[3] = (uint8_t)((Value >> 24) & 0xFFU);

  return (BSP_I2C1_WriteReg16(BSP_CAMERA_VD66GY_I2C_ADDRESS, Address,
                              buffer, 4U) == BSP_ERROR_NONE) ? 0 : -1;
}

/* The driver loads its firmware patch (tens of KB) here; never split it into bytes. */
static int Vd66gy_WriteArray(VD6G_Ctx_t *pCtx, uint16_t Address, uint8_t *pData, int Length)
{
  UNUSED(pCtx);

  /* The bus layer counts bytes in 16 bits. */
  if ((Length <= 0) || (Length > (int)UINT16_MAX))
  {
    return -1;
  }

  return (BSP_I2C1_WriteReg16(BSP_CAMERA_VD66GY_I2C_ADDRESS, Address, pData,
                              (uint16_t)Length) == BSP_ERROR_NONE) ? 0 : -1;
}

static void Vd66gy_Delay(VD6G_Ctx_t *pCtx, uint32_t Milliseconds)
{
  UNUSED(pCtx);

  HAL_Delay(Milliseconds);
}

/* Driver diagnostics are discarded: the console is neuron6_com's, not required here. */
static void Vd66gy_Log(VD6G_Ctx_t *pCtx, int Level, const char *pFormat, va_list Args)
{
  UNUSED(pCtx);
  UNUSED(Level);
  UNUSED(pFormat);
  UNUSED(Args);
}

/* Point the driver at this board's bus and timing. */
static void Vd66gy_BindCallbacks(void)
{
  Vd66gy_Ctx.shutdown_pin = Vd66gy_Shutdown;
  Vd66gy_Ctx.read8        = Vd66gy_Read8;
  Vd66gy_Ctx.read16       = Vd66gy_Read16;
  Vd66gy_Ctx.read32       = Vd66gy_Read32;
  Vd66gy_Ctx.write8       = Vd66gy_Write8;
  Vd66gy_Ctx.write16      = Vd66gy_Write16;
  Vd66gy_Ctx.write32      = Vd66gy_Write32;
  Vd66gy_Ctx.write_array  = Vd66gy_WriteArray;
  Vd66gy_Ctx.delay        = Vd66gy_Delay;
  Vd66gy_Ctx.log          = Vd66gy_Log;
}

/* Monochrome (no order) maps to RGGB, so its one channel passes through as grey. */
static uint32_t Vd66gy_MapBayer(VD6G_BayerType_t Bayer)
{
  switch (Bayer)
  {
    case VD6G_BAYER_GRBG:
      return BSP_CAMERA_BAYER_GRBG;

    case VD6G_BAYER_GBRG:
      return BSP_CAMERA_BAYER_GBRG;

    case VD6G_BAYER_BGGR:
      return BSP_CAMERA_BAYER_BGGR;

    default:
      return BSP_CAMERA_BAYER_RGGB;
  }
}

static const BSP_CAMERA_SensorMode_t *Vd66gy_FindMode(uint32_t Resolution)
{
  uint32_t index;

  for (index = 0U; index < VD66GY_MODES_NBR; index++)
  {
    if (Vd66gy_Modes[index].Resolution == Resolution)
    {
      return &Vd66gy_Modes[index];
    }
  }

  return NULL;
}

/* Configure a mode (RAW8 only) and leave the sensor idle. */
static int32_t Vd66gy_DrvInit(void *pObj, uint32_t Resolution, uint32_t PixelFormat)
{
  const BSP_CAMERA_SensorMode_t *mode = Vd66gy_FindMode(Resolution);
  VD6G_Config_t config;
  uint32_t index;

  UNUSED(pObj);

  if ((mode == NULL) || (PixelFormat != BSP_CAMERA_PF_RAW8))
  {
    return -1;
  }

  (void)memset(&config, 0, sizeof(config));

  config.ext_clock_freq_in_hz = (int)BSP_CAMERA_VD66GY_EXT_CLOCK_HZ;
  config.resolution           = Vd66gy_DriverRes[(uint32_t)(mode - Vd66gy_Modes)];

  /* Capped at the ceiling: a faster rate is accepted, but the sensor never streams. */
  config.frame_rate = ((Vd66gy_FrameRate != 0U) && (Vd66gy_FrameRate < mode->MaxFrameRate))
                      ? (int)Vd66gy_FrameRate : (int)mode->MaxFrameRate;
  config.flip_mirror_mode     = VD6G_MIRROR_FLIP_NONE;
  config.line_len             = (mode->Width > VD66GY_WIDE_WIDTH)
                                ? VD66GY_WIDE_LINE_LENGTH : 0;  /* 0: the sensor's own. */
  config.patgen               = VD6G_PATGEN_DISABLE;
  config.flicker              = VD6G_FLICKER_FREE_50HZ;
  config.pixel_depth          = VD6G_COLOR_DEPTH_RAW8;

  /* The lane swap is done on the sensor, not in the receiver. */
  config.out_itf.datalane_nb                    = 2;
  config.out_itf.clock_lane_swap_enable         = 0;
  config.out_itf.data_lane0_swap_enable         = 0;
  config.out_itf.data_lane1_swap_enable         = 0;
  config.out_itf.data_lanes_mapping_swap_enable = 1;

  /* The sensor's own pins are left as inputs. */
  for (index = 0U; index < (uint32_t)VD6G_GPIO_NB; index++)
  {
    config.gpio_ctrl[index] = (uint8_t)VD6G_GPIO_GPIO_IN;
  }

  /* The sensor's own exposure loop, running from the first frame. */
  config.exposure_mode = VD6G_EXPOSURE_AUTO;

  if (VD6G_Init(&Vd66gy_Ctx, &config) != 0)
  {
    return -1;
  }

  /* The driver reads the colour-filter order out of the part during init. */
  Vd66gy_Sensor.BayerOrder = Vd66gy_MapBayer(Vd66gy_Ctx.bayer);

  Vd66gy_Initialised = 1U;

  return 0;
}

static int32_t Vd66gy_DrvDeInit(void *pObj)
{
  UNUSED(pObj);

  Vd66gy_Initialised = 0U;

  return (VD6G_DeInit(&Vd66gy_Ctx) == 0) ? 0 : -1;
}

/* The driver has no identification call: read the model register directly. */
static int32_t Vd66gy_DrvReadID(void *pObj, uint32_t *pId)
{
  uint16_t model = 0U;

  UNUSED(pObj);

  Vd66gy_BindCallbacks();

  if (Vd66gy_Read16(&Vd66gy_Ctx, VD66GY_REG_MODEL_ID, &model) != 0)
  {
    return -1;
  }

  *pId = (uint32_t)model;

  return 0;
}

static int32_t Vd66gy_DrvStart(void *pObj)
{
  UNUSED(pObj);

  return (VD6G_Start(&Vd66gy_Ctx) == 0) ? 0 : -1;
}

static int32_t Vd66gy_DrvStop(void *pObj)
{
  UNUSED(pObj);

  return (VD6G_Stop(&Vd66gy_Ctx) == 0) ? 0 : -1;
}

/* The module's and driver's orientation values match, so they pass straight through. The
 * Bayer order changes with orientation and the driver works it out only at boot. */
static int32_t Vd66gy_DrvSetMirrorFlip(void *pObj, uint32_t MirrorFlip)
{
  static const VD6G_BayerType_t order[] =
  {
    VD6G_BAYER_GRBG,   /* VD6G_MIRROR_FLIP_NONE */
    VD6G_BAYER_BGGR,   /* VD6G_FLIP             */
    VD6G_BAYER_RGGB,   /* VD6G_MIRROR           */
    VD6G_BAYER_GBRG,   /* VD6G_MIRROR_FLIP      */
  };

  UNUSED(pObj);

  if (MirrorFlip > (uint32_t)VD6G_MIRROR_FLIP)
  {
    return -1;
  }

  if (VD6G_SetFlipMirrorMode(&Vd66gy_Ctx, (VD6G_MirrorFlip_t)MirrorFlip) != 0)
  {
    return -1;
  }

  /* A monochrome part has no filter, before or after. */
  if (Vd66gy_Ctx.bayer != VD6G_BAYER_NONE)
  {
    Vd66gy_Ctx.bayer = order[MirrorFlip];
    Vd66gy_Sensor.BayerOrder = Vd66gy_MapBayer(Vd66gy_Ctx.bayer);
  }

  return 0;
}

int32_t BSP_CAMERA_VD66GY_Register(void)
{
  Vd66gy_BindCallbacks();

  Vd66gy_Sensor.Name         = "VD66GY";
  Vd66gy_Sensor.pObj         = NULL;   /* The context is a singleton here. */
  Vd66gy_Sensor.pDrv         = &Vd66gy_Drv;
  Vd66gy_Sensor.Id           = BSP_CAMERA_VD66GY_ID;
  Vd66gy_Sensor.CsiDataType  = BSP_CAMERA_CSI_DT_RAW8;
  Vd66gy_Sensor.SensorFormat = BSP_CAMERA_PF_RAW8;

  /* A placeholder until init reads the real order; left alone while the sensor is up,
   * as this is the live descriptor (and the registration below is refused then). */
  if (Vd66gy_Initialised == 0U)
  {
    Vd66gy_Sensor.BayerOrder = BSP_CAMERA_BAYER_RGGB;
  }

  /* The auxiliary pipe feeds from the main pipe: only it reconstructs colour. */
  Vd66gy_Sensor.SharedPipes  = 1U;

  Vd66gy_Sensor.pModes       = Vd66gy_Modes;
  Vd66gy_Sensor.ModeCount    = VD66GY_MODES_NBR;
  Vd66gy_Sensor.DataLanes    = 2U;
  Vd66gy_Sensor.I2cAddress   = (uint16_t)BSP_CAMERA_VD66GY_I2C_ADDRESS;

  return BSP_CAMERA_RegisterSensor(&Vd66gy_Sensor);
}

int32_t BSP_CAMERA_VD66GY_SetFrameRate(uint32_t Fps)
{
  if ((Fps != 0U) && (Fps < VD66GY_MIN_FPS))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Vd66gy_Initialised != 0U)
  {
    return BSP_ERROR_BUSY;
  }

  Vd66gy_FrameRate = Fps;

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_VD66GY_GetBayerOrder(uint32_t *pBayerOrder)
{
  if (pBayerOrder == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Vd66gy_Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  *pBayerOrder = Vd66gy_Sensor.BayerOrder;

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_VD66GY_SetBrightness(int32_t Level)
{
  if ((Level < VD6G_MIN_BRIGHTNESS) || (Level > VD6G_MAX_BRIGHTNESS))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Vd66gy_Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  return (VD6G_SetBrightnessLevel(&Vd66gy_Ctx, (int)Level) == 0)
         ? BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
}

void *BSP_CAMERA_VD66GY_GetContext(void)
{
  return &Vd66gy_Ctx;
}
