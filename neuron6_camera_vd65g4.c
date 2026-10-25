/**
  * @file    neuron6_camera_vd65g4.c
  * @brief   VD65G4 global-shutter image sensor as a Neuro N6 camera sensor.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include <stdint.h>
#include <string.h>
#include "neuron6_camera_vd65g4.h"
#include "neuron6.h"
#include "neuron6_bus.h"
#include "vd55g1.h"

#define VD65G4_REG_MODEL_ID         0x0000U

#define VD65G4_MODES_NBR            4U

#define VD65G4_MAX_FPS              60U

static VD55G1_Ctx_t Vd65g4_Ctx;

static uint8_t Vd65g4_Initialised;

/* Vd65g4_DriverRes matches this table entry for entry. */
static const BSP_CAMERA_SensorMode_t Vd65g4_Modes[VD65G4_MODES_NBR] =
{
  { BSP_CAMERA_VD65G4_R320x240, 320U, 240U, VD65G4_MAX_FPS, BSP_CAMERA_VD65G4_LANE_RATE_MBPS },
  { BSP_CAMERA_VD65G4_R640x480, 640U, 480U, VD65G4_MAX_FPS, BSP_CAMERA_VD65G4_LANE_RATE_MBPS },
  { BSP_CAMERA_VD65G4_R800x600, 800U, 600U, VD65G4_MAX_FPS, BSP_CAMERA_VD65G4_LANE_RATE_MBPS },
  { BSP_CAMERA_VD65G4_R804x704, 804U, 704U, VD65G4_MAX_FPS, BSP_CAMERA_VD65G4_LANE_RATE_MBPS },
};

static const VD55G1_Res_t Vd65g4_DriverRes[VD65G4_MODES_NBR] =
{
  VD55G1_RES_QVGA_320_240,
  VD55G1_RES_VGA_640_480,
  VD55G1_RES_SXGA_800_600,
  VD55G1_RES_FULL_804_704,
};

/* BayerOrder is read from the part at initialisation. */
static BSP_CAMERA_Sensor_t Vd65g4_Sensor;

static void    Vd65g4_Shutdown(VD55G1_Ctx_t *pCtx, int Value);
static int     Vd65g4_Read8(VD55G1_Ctx_t *pCtx, uint16_t Address, uint8_t *pValue);
static int     Vd65g4_Read16(VD55G1_Ctx_t *pCtx, uint16_t Address, uint16_t *pValue);
static int     Vd65g4_Read32(VD55G1_Ctx_t *pCtx, uint16_t Address, uint32_t *pValue);
static int     Vd65g4_Write8(VD55G1_Ctx_t *pCtx, uint16_t Address, uint8_t Value);
static int     Vd65g4_Write16(VD55G1_Ctx_t *pCtx, uint16_t Address, uint16_t Value);
static int     Vd65g4_Write32(VD55G1_Ctx_t *pCtx, uint16_t Address, uint32_t Value);
static int     Vd65g4_WriteArray(VD55G1_Ctx_t *pCtx, uint16_t Address, uint8_t *pData, int Length);
static void    Vd65g4_Delay(VD55G1_Ctx_t *pCtx, uint32_t Milliseconds);
static void    Vd65g4_Log(VD55G1_Ctx_t *pCtx, int Level, const char *pFormat, va_list Args);
static void    Vd65g4_BindCallbacks(void);
static uint32_t Vd65g4_MapBayer(VD55G1_Bayer_t Bayer);
static const BSP_CAMERA_SensorMode_t *Vd65g4_FindMode(uint32_t Resolution);
static int32_t Vd65g4_DrvInit(void *pObj, uint32_t Resolution, uint32_t PixelFormat);
static int32_t Vd65g4_DrvDeInit(void *pObj);
static int32_t Vd65g4_DrvReadID(void *pObj, uint32_t *pId);
static int32_t Vd65g4_DrvStart(void *pObj);
static int32_t Vd65g4_DrvStop(void *pObj);
static int32_t Vd65g4_DrvSetMirrorFlip(void *pObj, uint32_t MirrorFlip);

/* Picture controls are null: the output is raw, so they belong to the DCMIPP ISP
 * (BSP_CAMERA_SetIsp*). */
static const BSP_CAMERA_SensorDrv_t Vd65g4_Drv =
{
  Vd65g4_DrvInit,
  Vd65g4_DrvDeInit,
  Vd65g4_DrvReadID,
  Vd65g4_DrvStart,
  Vd65g4_DrvStop,
  Vd65g4_DrvSetMirrorFlip,
  NULL,   /* SetBrightness  */
  NULL,   /* SetSaturation  */
  NULL,   /* SetContrast    */
  NULL,   /* SetHueDegree   */
  NULL,   /* SetLightMode   */
  NULL,   /* SetColorEffect */
  NULL,   /* SetZoom        */
  NULL,   /* SetNightMode   */
  NULL,   /* SetResolution  */
};

/* The module holds the shutdown line itself; requests are ignored. */
static void Vd65g4_Shutdown(VD55G1_Ctx_t *pCtx, int Value)
{
  UNUSED(pCtx);
  UNUSED(Value);
}

static int Vd65g4_Read8(VD55G1_Ctx_t *pCtx, uint16_t Address, uint8_t *pValue)
{
  UNUSED(pCtx);
  return (BSP_I2C1_ReadReg16(BSP_CAMERA_VD65G4_I2C_ADDRESS, Address,
                             pValue, 1U) == BSP_ERROR_NONE) ? 0 : -1;
}

/* Multi-byte registers are least significant byte first. */
static int Vd65g4_Read16(VD55G1_Ctx_t *pCtx, uint16_t Address, uint16_t *pValue)
{
  uint8_t buffer[2];

  UNUSED(pCtx);

  if (BSP_I2C1_ReadReg16(BSP_CAMERA_VD65G4_I2C_ADDRESS, Address,
                         buffer, 2U) != BSP_ERROR_NONE)
  {
    return -1;
  }

  *pValue = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
  return 0;
}

static int Vd65g4_Read32(VD55G1_Ctx_t *pCtx, uint16_t Address, uint32_t *pValue)
{
  uint8_t buffer[4];

  UNUSED(pCtx);

  if (BSP_I2C1_ReadReg16(BSP_CAMERA_VD65G4_I2C_ADDRESS, Address,
                         buffer, 4U) != BSP_ERROR_NONE)
  {
    return -1;
  }

  *pValue = (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) |
            ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24);
  return 0;
}

static int Vd65g4_Write8(VD55G1_Ctx_t *pCtx, uint16_t Address, uint8_t Value)
{
  uint8_t value = Value;

  UNUSED(pCtx);
  return (BSP_I2C1_WriteReg16(BSP_CAMERA_VD65G4_I2C_ADDRESS, Address,
                              &value, 1U) == BSP_ERROR_NONE) ? 0 : -1;
}

static int Vd65g4_Write16(VD55G1_Ctx_t *pCtx, uint16_t Address, uint16_t Value)
{
  uint8_t buffer[2];

  UNUSED(pCtx);
  buffer[0] = (uint8_t)(Value & 0xFFU);
  buffer[1] = (uint8_t)(Value >> 8);
  return (BSP_I2C1_WriteReg16(BSP_CAMERA_VD65G4_I2C_ADDRESS, Address,
                              buffer, 2U) == BSP_ERROR_NONE) ? 0 : -1;
}

static int Vd65g4_Write32(VD55G1_Ctx_t *pCtx, uint16_t Address, uint32_t Value)
{
  uint8_t buffer[4];

  UNUSED(pCtx);
  buffer[0] = (uint8_t)(Value & 0xFFU);
  buffer[1] = (uint8_t)((Value >> 8) & 0xFFU);
  buffer[2] = (uint8_t)((Value >> 16) & 0xFFU);
  buffer[3] = (uint8_t)((Value >> 24) & 0xFFU);
  return (BSP_I2C1_WriteReg16(BSP_CAMERA_VD65G4_I2C_ADDRESS, Address,
                              buffer, 4U) == BSP_ERROR_NONE) ? 0 : -1;
}

static int Vd65g4_WriteArray(VD55G1_Ctx_t *pCtx, uint16_t Address, uint8_t *pData, int Length)
{
  UNUSED(pCtx);

  if ((Length <= 0) || (Length > (int)UINT16_MAX))
  {
    return -1;
  }
  return (BSP_I2C1_WriteReg16(BSP_CAMERA_VD65G4_I2C_ADDRESS, Address, pData,
                              (uint16_t)Length) == BSP_ERROR_NONE) ? 0 : -1;
}

static void Vd65g4_Delay(VD55G1_Ctx_t *pCtx, uint32_t Milliseconds)
{
  UNUSED(pCtx);
  HAL_Delay(Milliseconds);
}

static void Vd65g4_Log(VD55G1_Ctx_t *pCtx, int Level, const char *pFormat, va_list Args)
{
  UNUSED(pCtx);
  UNUSED(Level);
  UNUSED(pFormat);
  UNUSED(Args);
}

static void Vd65g4_BindCallbacks(void)
{
  Vd65g4_Ctx.shutdown_pin = Vd65g4_Shutdown;
  Vd65g4_Ctx.read8        = Vd65g4_Read8;
  Vd65g4_Ctx.read16       = Vd65g4_Read16;
  Vd65g4_Ctx.read32       = Vd65g4_Read32;
  Vd65g4_Ctx.write8       = Vd65g4_Write8;
  Vd65g4_Ctx.write16      = Vd65g4_Write16;
  Vd65g4_Ctx.write32      = Vd65g4_Write32;
  Vd65g4_Ctx.write_array  = Vd65g4_WriteArray;
  Vd65g4_Ctx.delay        = Vd65g4_Delay;
  Vd65g4_Ctx.log          = Vd65g4_Log;
}

/* A monochrome part reports no order: described as RGGB. */
static uint32_t Vd65g4_MapBayer(VD55G1_Bayer_t Bayer)
{
  switch (Bayer)
  {
    case VD55G1_BAYER_GRBG:
      return BSP_CAMERA_BAYER_GRBG;
    case VD55G1_BAYER_GBRG:
      return BSP_CAMERA_BAYER_GBRG;
    case VD55G1_BAYER_BGGR:
      return BSP_CAMERA_BAYER_BGGR;
    default:
      return BSP_CAMERA_BAYER_RGGB;
  }
}

static const BSP_CAMERA_SensorMode_t *Vd65g4_FindMode(uint32_t Resolution)
{
  uint32_t index;

  for (index = 0U; index < VD65G4_MODES_NBR; index++)
  {
    if (Vd65g4_Modes[index].Resolution == Resolution)
    {
      return &Vd65g4_Modes[index];
    }
  }
  return NULL;
}

/* Configure a mode (RAW8 only) and leave the sensor idle. */
static int32_t Vd65g4_DrvInit(void *pObj, uint32_t Resolution, uint32_t PixelFormat)
{
  const BSP_CAMERA_SensorMode_t *mode = Vd65g4_FindMode(Resolution);
  VD55G1_Config_t config;
  uint32_t index;

  UNUSED(pObj);

  if ((mode == NULL) || (PixelFormat != BSP_CAMERA_PF_RAW8))
  {
    return -1;
  }

  (void)memset(&config, 0, sizeof(config));
  config.ext_clock_freq_in_hz         = (int)BSP_CAMERA_VD65G4_EXT_CLOCK_HZ;
  config.resolution                   = Vd65g4_DriverRes[(uint32_t)(mode - Vd65g4_Modes)];
  config.frame_rate                   = (int)mode->MaxFrameRate;
  config.flip_mirror_mode             = VD55G1_MIRROR_FLIP_NONE;
  config.patgen                       = VD55G1_PATGEN_DISABLE;
  config.flicker                      = VD55G1_FLICKER_FREE_NONE;
  config.pixel_depth                  = 8U;
  config.out_itf.data_rate_in_mps     = (int)(BSP_CAMERA_VD65G4_LANE_RATE_MBPS * 1000000U);
  config.out_itf.clock_lane_swap_enable = (int)BSP_CAMERA_VD65G4_CLOCK_LANE_SWAP;
  config.out_itf.data_lane_swap_enable  = (int)BSP_CAMERA_VD65G4_DATA_LANE_SWAP;
  config.awu.is_enable                = 0;

  /* The sensor's GPIOs are not wired: all inputs. */
  for (index = 0U; index < (uint32_t)VD55G1_GPIO_NB; index++)
  {
    config.gpio_ctrl[index] = (uint8_t)VD55G1_GPIO_GPIO_IN;
  }

  if (VD55G1_Init(&Vd65g4_Ctx, &config) != 0)
  {
    return -1;
  }

  /* VD55G1_Init reads the filter order from the part. */
  Vd65g4_Sensor.BayerOrder = Vd65g4_MapBayer(Vd65g4_Ctx.bayer);
  Vd65g4_Initialised = 1U;
  return 0;
}

static int32_t Vd65g4_DrvDeInit(void *pObj)
{
  UNUSED(pObj);
  Vd65g4_Initialised = 0U;
  return (VD55G1_DeInit(&Vd65g4_Ctx) == 0) ? 0 : -1;
}

static int32_t Vd65g4_DrvReadID(void *pObj, uint32_t *pId)
{
  uint32_t model = 0U;

  UNUSED(pObj);
  Vd65g4_BindCallbacks();

  if (Vd65g4_Read32(&Vd65g4_Ctx, VD65G4_REG_MODEL_ID, &model) != 0)
  {
    return -1;
  }
  *pId = model;
  return 0;
}

static int32_t Vd65g4_DrvStart(void *pObj)
{
  UNUSED(pObj);
  return (VD55G1_Start(&Vd65g4_Ctx) == 0) ? 0 : -1;
}

static int32_t Vd65g4_DrvStop(void *pObj)
{
  UNUSED(pObj);
  return (VD55G1_Stop(&Vd65g4_Ctx) == 0) ? 0 : -1;
}

/* BSP_CAMERA_MIRRORFLIP_* codes are the driver's; the flip changes the filter order. */
static int32_t Vd65g4_DrvSetMirrorFlip(void *pObj, uint32_t MirrorFlip)
{
  UNUSED(pObj);

  if (MirrorFlip > (uint32_t)VD55G1_MIRROR_FLIP)
  {
    return -1;
  }
  if (VD55G1_SetFlipMirrorMode(&Vd65g4_Ctx, (VD55G1_MirrorFlip_t)MirrorFlip) != 0)
  {
    return -1;
  }
  Vd65g4_Sensor.BayerOrder = Vd65g4_MapBayer(Vd65g4_Ctx.bayer);
  return 0;
}

int32_t BSP_CAMERA_VD65G4_Register(void)
{
  Vd65g4_BindCallbacks();

  Vd65g4_Sensor.Name         = "VD65G4";
  Vd65g4_Sensor.pObj         = NULL;
  Vd65g4_Sensor.pDrv         = &Vd65g4_Drv;
  Vd65g4_Sensor.Id           = BSP_CAMERA_VD65G4_ID;
  Vd65g4_Sensor.CsiDataType  = BSP_CAMERA_CSI_DT_RAW8;
  Vd65g4_Sensor.SensorFormat = BSP_CAMERA_PF_RAW8;
  /* Keep the live order while the sensor is up; the registration is refused then anyway. */
  if (Vd65g4_Initialised == 0U)
  {
    Vd65g4_Sensor.BayerOrder = BSP_CAMERA_BAYER_RGGB;
  }
  Vd65g4_Sensor.SharedPipes  = 1U;
  Vd65g4_Sensor.pModes       = Vd65g4_Modes;
  Vd65g4_Sensor.ModeCount    = VD65G4_MODES_NBR;
  Vd65g4_Sensor.DataLanes    = 1U;
  Vd65g4_Sensor.I2cAddress   = (uint16_t)BSP_CAMERA_VD65G4_I2C_ADDRESS;

  return BSP_CAMERA_RegisterSensor(&Vd65g4_Sensor);
}

int32_t BSP_CAMERA_VD65G4_GetBayerOrder(uint32_t *pBayerOrder)
{
  if (pBayerOrder == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (Vd65g4_Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }
  *pBayerOrder = Vd65g4_Sensor.BayerOrder;
  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_VD65G4_SetBrightness(int32_t Level)
{
  if ((Level < VD55G1_MIN_BRIGHTNESS) || (Level > VD55G1_MAX_BRIGHTNESS))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (Vd65g4_Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }
  return (VD55G1_SetBrightnessLevel(&Vd65g4_Ctx, (int)Level) == 0)
         ? BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
}

void *BSP_CAMERA_VD65G4_GetContext(void)
{
  return &Vd65g4_Ctx;
}
