/**
  * @file    neuron6_camera_imx335.c
  * @brief   Sony IMX335 (ST's B-CAMS-IMX module) as a Neuro N6 camera sensor.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include <stdint.h>

#include "neuron6_camera_imx335.h"
#include "neuron6_camera_ae.h"
#include "neuron6.h"
#include "neuron6_bus.h"
#include "imx335.h"

#define IMX335_LANE_RATE_MBPS       1600U
#define IMX335_MAX_FPS              30U

/* Line length (HMAX) counts a 74.25 MHz clock; ST's 30 fps timing is HMAX 550, VMAX 4500. */
#define IMX335_REG_HMAX_LSB         0x3034U
#define IMX335_HMAX_CLOCK_HZ        74250000U
#define IMX335_HMAX_DEFAULT         550U

/* Line length in percent of IMX335_HMAX_DEFAULT. */
#define IMX335_LINE_PERCENT_MIN     100U
#define IMX335_LINE_PERCENT_MAX     400U

/* Exposure is VMAX - SHR0 lines; SHR0 may not fall below 9. */
#define IMX335_SHR0_MIN             9U

/* Power sequence: off long enough to drain the 1.2 V rail, then supply and clock settled
 * before XCLR is released (the datasheet asks 500 ns). */
#define IMX335_POWER_OFF_MS         10U
#define IMX335_SUPPLY_SETTLE_MS     5U
#define IMX335_RESET_RELEASE_MS     2U

#define IMX335_INITIAL_EXPOSURE_US  10000U

/* Auto-exposure limits: 36 dB keeps noise usable; 50 Hz mains. */
#ifndef BSP_CAMERA_IMX335_AE_GAIN_MAX_MDB
#define BSP_CAMERA_IMX335_AE_GAIN_MAX_MDB  36000U
#endif
#ifndef BSP_CAMERA_IMX335_AE_FLICKER_HZ
#define BSP_CAMERA_IMX335_AE_FLICKER_HZ    50U
#endif

static IMX335_Object_t Imx335_Obj;

static uint8_t  Imx335_Initialised;
static uint32_t Imx335_Fps = IMX335_MAX_FPS;
static uint32_t Imx335_LinePercent = IMX335_LINE_PERCENT_MIN;

/* Line timing as programmed. */
static uint32_t Imx335_Hmax;
static uint32_t Imx335_Vmax;
static uint32_t Imx335_LineNs;

static BSP_CAMERA_AE_State_t Imx335_Ae;

/* The rate ceiling is ST's; any encoder limit is the application's. */
static const BSP_CAMERA_SensorMode_t Imx335_Modes[1] =
{
  { BSP_CAMERA_IMX335_R2592x1944, BSP_CAMERA_IMX335_WIDTH, BSP_CAMERA_IMX335_HEIGHT,
    IMX335_MAX_FPS, IMX335_LANE_RATE_MBPS },
};

static BSP_CAMERA_Sensor_t Imx335_Sensor;

static int32_t Imx335_IoInit(void);
static int32_t Imx335_IoDeInit(void);
static int32_t Imx335_IoWrite(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
static int32_t Imx335_IoRead(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
static int32_t Imx335_IoTick(void);
static int32_t Imx335_Bind(void);
static void    Imx335_PowerUp(void);
static int32_t Imx335_WriteReg(uint16_t Reg, uint32_t Value, uint16_t Bytes);
static int32_t Imx335_ReadReg(uint16_t Reg, uint32_t *pValue, uint16_t Bytes);
static int32_t Imx335_WriteExposure(uint32_t Us, uint32_t MilliDb);

static int32_t Imx335_DrvInit(void *pObj, uint32_t Resolution, uint32_t PixelFormat);
static int32_t Imx335_DrvDeInit(void *pObj);
static int32_t Imx335_DrvReadID(void *pObj, uint32_t *pId);
static int32_t Imx335_DrvStart(void *pObj);
static int32_t Imx335_DrvStop(void *pObj);
static int32_t Imx335_DrvSetMirrorFlip(void *pObj, uint32_t MirrorFlip);

/* Picture controls are NULL: the sensor sends raw data, so they belong to the DCMIPP. */
static const BSP_CAMERA_SensorDrv_t Imx335_Drv =
{
  Imx335_DrvInit,
  Imx335_DrvDeInit,
  Imx335_DrvReadID,
  Imx335_DrvStart,
  Imx335_DrvStop,
  Imx335_DrvSetMirrorFlip,
  NULL,   /* SetBrightness  */
  NULL,   /* SetSaturation  */
  NULL,   /* SetContrast    */
  NULL,   /* SetHueDegree   */
  NULL,   /* SetLightMode   */
  NULL,   /* SetColorEffect */
  NULL,   /* SetZoom        */
  NULL,   /* SetNightMode   */
  NULL,   /* SetResolution  - one mode */
};

/* The bus is the board's, already up. */
static int32_t Imx335_IoInit(void)
{
  return IMX335_OK;
}

static int32_t Imx335_IoDeInit(void)
{
  return IMX335_OK;
}

static int32_t Imx335_IoWrite(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  return (BSP_I2C1_WriteReg16(DevAddr, Reg, pData, Length) == BSP_ERROR_NONE) ? IMX335_OK : IMX335_ERROR;
}

static int32_t Imx335_IoRead(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  return (BSP_I2C1_ReadReg16(DevAddr, Reg, pData, Length) == BSP_ERROR_NONE) ? IMX335_OK : IMX335_ERROR;
}

static int32_t Imx335_IoTick(void)
{
  return (int32_t)HAL_GetTick();
}

static int32_t Imx335_Bind(void)
{
  IMX335_IO_t io;

  io.Init     = Imx335_IoInit;
  io.DeInit   = Imx335_IoDeInit;
  io.Address  = (uint16_t)BSP_CAMERA_IMX335_I2C_ADDRESS;
  io.WriteReg = Imx335_IoWrite;
  io.ReadReg  = Imx335_IoRead;
  io.GetTick  = Imx335_IoTick;

  return (IMX335_RegisterBusIO(&Imx335_Obj, &io) == IMX335_OK) ? 0 : -1;
}

/* EN off then on, with XCLR held low until supply and clock are up, so a board reset
 * restarts the sensor from power-up. Each level is written before the pin drives. */
static void Imx335_PowerUp(void)
{
#if (defined(BSP_CAMERA_IMX335_EN_PORT) && defined(BSP_CAMERA_IMX335_EN_PIN)) || \
    (defined(BSP_CAMERA_IMX335_XCLR_PORT) && defined(BSP_CAMERA_IMX335_XCLR_PIN))
  GPIO_InitTypeDef gpio_init_structure = {0};

  gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull  = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;
#endif

#if defined(BSP_CAMERA_IMX335_IO_ENABLE)
  BSP_CAMERA_IMX335_IO_ENABLE();
#endif

#if defined(BSP_CAMERA_IMX335_XCLR_PORT) && defined(BSP_CAMERA_IMX335_XCLR_PIN)
  BSP_CAMERA_IMX335_XCLR_CLK_ENABLE();
  HAL_GPIO_WritePin(BSP_CAMERA_IMX335_XCLR_PORT, BSP_CAMERA_IMX335_XCLR_PIN, GPIO_PIN_RESET);
  gpio_init_structure.Pin = BSP_CAMERA_IMX335_XCLR_PIN;
  HAL_GPIO_Init(BSP_CAMERA_IMX335_XCLR_PORT, &gpio_init_structure);
#endif

#if defined(BSP_CAMERA_IMX335_EN_PORT) && defined(BSP_CAMERA_IMX335_EN_PIN)
  BSP_CAMERA_IMX335_EN_CLK_ENABLE();
  HAL_GPIO_WritePin(BSP_CAMERA_IMX335_EN_PORT, BSP_CAMERA_IMX335_EN_PIN, GPIO_PIN_RESET);
  gpio_init_structure.Pin = BSP_CAMERA_IMX335_EN_PIN;
  HAL_GPIO_Init(BSP_CAMERA_IMX335_EN_PORT, &gpio_init_structure);
  HAL_Delay(IMX335_POWER_OFF_MS);
  HAL_GPIO_WritePin(BSP_CAMERA_IMX335_EN_PORT, BSP_CAMERA_IMX335_EN_PIN, GPIO_PIN_SET);
#endif

  HAL_Delay(IMX335_SUPPLY_SETTLE_MS);

#if defined(BSP_CAMERA_IMX335_XCLR_PORT) && defined(BSP_CAMERA_IMX335_XCLR_PIN)
  HAL_GPIO_WritePin(BSP_CAMERA_IMX335_XCLR_PORT, BSP_CAMERA_IMX335_XCLR_PIN, GPIO_PIN_SET);
#endif

  HAL_Delay(IMX335_RESET_RELEASE_MS);
}

/* Multi-byte values are least significant byte first, at rising addresses. */
static int32_t Imx335_WriteReg(uint16_t Reg, uint32_t Value, uint16_t Bytes)
{
  uint8_t  buffer[4];
  uint16_t i;

  for (i = 0U; i < Bytes; i++)
  {
    buffer[i] = (uint8_t)((Value >> (8U * i)) & 0xFFU);
  }

  return Imx335_IoWrite((uint16_t)BSP_CAMERA_IMX335_I2C_ADDRESS, Reg, buffer, Bytes);
}

static int32_t Imx335_ReadReg(uint16_t Reg, uint32_t *pValue, uint16_t Bytes)
{
  uint8_t  buffer[4] = {0};
  uint16_t i;

  if (Imx335_IoRead((uint16_t)BSP_CAMERA_IMX335_I2C_ADDRESS, Reg, buffer, Bytes) != IMX335_OK)
  {
    return IMX335_ERROR;
  }

  *pValue = 0U;
  for (i = 0U; i < Bytes; i++)
  {
    *pValue |= (uint32_t)buffer[i] << (8U * i);
  }

  return IMX335_OK;
}

/* ST's exposure call assumes its 30 fps line; this one uses the line as programmed and
 * writes the gain with it. */
static int32_t Imx335_WriteExposure(uint32_t Us, uint32_t MilliDb)
{
  uint32_t lines = (uint32_t)(((uint64_t)Us * 1000U) / Imx335_LineNs);
  int32_t  status;

  if (lines < 1U)
  {
    lines = 1U;
  }
  if (lines > (Imx335_Vmax - IMX335_SHR0_MIN))
  {
    lines = Imx335_Vmax - IMX335_SHR0_MIN;
  }

  /* Both under one hold, so they take effect in the same frame. */
  status = Imx335_WriteReg(IMX335_REG_HOLD, 1U, 1U);
  if (status == IMX335_OK)
  {
    status = Imx335_WriteReg(IMX335_REG_SHUTTER, Imx335_Vmax - lines, 3U);
  }
  if (status == IMX335_OK)
  {
    status = Imx335_WriteReg(IMX335_REG_GAIN, MilliDb / BSP_CAMERA_IMX335_GAIN_STEP_MDB, 2U);
  }
  if (Imx335_WriteReg(IMX335_REG_HOLD, 0U, 1U) != IMX335_OK)
  {
    status = IMX335_ERROR;
  }

  if (status == IMX335_OK)
  {
    Imx335_Ae.ExposureUs = (uint32_t)(((uint64_t)lines * Imx335_LineNs) / 1000U);
    Imx335_Ae.GainMdb    = (MilliDb / BSP_CAMERA_IMX335_GAIN_STEP_MDB) * BSP_CAMERA_IMX335_GAIN_STEP_MDB;
  }

  return status;
}

/* Configure the one mode (RAW10 only) and leave the sensor in standby. */
static int32_t Imx335_DrvInit(void *pObj, uint32_t Resolution, uint32_t PixelFormat)
{
  UNUSED(pObj);

  if ((Resolution != BSP_CAMERA_IMX335_R2592x1944) || (PixelFormat != BSP_CAMERA_PF_RAW10))
  {
    return -1;
  }

  Imx335_Obj.IsInitialized = 0U;

  if ((IMX335_SetFrequency(&Imx335_Obj, IMX335_INCK_24MHZ) != IMX335_OK) ||
      (IMX335_MirrorFlipConfig(&Imx335_Obj, IMX335_MIRROR_FLIP_NONE) != IMX335_OK) ||
      (IMX335_Init(&Imx335_Obj, IMX335_R2592_1944, IMX335_RAW_RGGB10) != IMX335_OK))
  {
    return -1;
  }

  /* Derived from ST's line, never read back: a sensor that missed the reset keeps the
   * previous value, and scaling it again would compound. */
  Imx335_Hmax   = (IMX335_HMAX_DEFAULT * Imx335_LinePercent) / 100U;
  Imx335_Vmax   = IMX335_HMAX_CLOCK_HZ / (Imx335_Fps * Imx335_Hmax);
  Imx335_LineNs = (uint32_t)((((uint64_t)Imx335_Hmax * 1000000000U) + (IMX335_HMAX_CLOCK_HZ / 2U)) /
                             IMX335_HMAX_CLOCK_HZ);

  if ((Imx335_WriteReg(IMX335_REG_HMAX_LSB, Imx335_Hmax, 2U) != IMX335_OK) ||
      (Imx335_WriteReg(IMX335_REG_VMAX, Imx335_Vmax, 3U) != IMX335_OK) ||
      (Imx335_WriteExposure(IMX335_INITIAL_EXPOSURE_US, 0U) != IMX335_OK))
  {
    return -1;
  }

  Imx335_Ae.Wait = 0U;

  Imx335_Initialised = 1U;

  return 0;
}

static int32_t Imx335_DrvDeInit(void *pObj)
{
  UNUSED(pObj);

  Imx335_Initialised = 0U;

  return (IMX335_DeInit(&Imx335_Obj) == IMX335_OK) ? 0 : -1;
}

/* 0x3912 reads 0x00 from reset and 0x33 once configured, and a dead bus also reads 0x00.
 * Accept 0x00, or a line length this driver programs (550 to 2200) on a sensor that kept
 * its configuration; then confirm with a register written, read back and restored. */
static int32_t Imx335_DrvReadID(void *pObj, uint32_t *pId)
{
  uint32_t id = 0xFFU;
  uint32_t hmax = 0U;
  uint32_t saved = 0U;
  uint32_t back = 0U;

  UNUSED(pObj);

  if ((Imx335_Bind() != 0) || (IMX335_ReadID(&Imx335_Obj, &id) != IMX335_OK) ||
      (Imx335_ReadReg(IMX335_REG_HMAX_LSB, &hmax, 2U) != IMX335_OK))
  {
    return -1;
  }

  if ((id != BSP_CAMERA_IMX335_ID) &&
      ((hmax < IMX335_HMAX_DEFAULT) ||
       (hmax > ((IMX335_HMAX_DEFAULT * IMX335_LINE_PERCENT_MAX) / 100U))))
  {
    *pId = id;
    return 0;
  }
  id = BSP_CAMERA_IMX335_ID;

  /* VMAX's low byte: rewritten at every init, so the test leaves no trace. */
  if ((Imx335_ReadReg(IMX335_REG_VMAX, &saved, 1U) != IMX335_OK) ||
      (Imx335_WriteReg(IMX335_REG_VMAX, saved ^ 0x5AU, 1U) != IMX335_OK) ||
      (Imx335_ReadReg(IMX335_REG_VMAX, &back, 1U) != IMX335_OK) ||
      (Imx335_WriteReg(IMX335_REG_VMAX, saved, 1U) != IMX335_OK) ||
      (back != (saved ^ 0x5AU)))
  {
    return -1;
  }

  *pId = id;

  return 0;
}

static int32_t Imx335_DrvStart(void *pObj)
{
  UNUSED(pObj);

  return (IMX335_Start(&Imx335_Obj) == IMX335_OK) ? 0 : -1;
}

static int32_t Imx335_DrvStop(void *pObj)
{
  UNUSED(pObj);

  return (Imx335_WriteReg(IMX335_REG_MODE_SELECT, IMX335_MODE_STANDBY, 1U) == IMX335_OK) ? 0 : -1;
}

/* The BSP's and the driver's orientation values match, so they pass straight through.
 * ST's tables move the read-out window with the flip, so the filter stays RGGB. */
static int32_t Imx335_DrvSetMirrorFlip(void *pObj, uint32_t MirrorFlip)
{
  UNUSED(pObj);

  if (MirrorFlip > IMX335_MIRROR_FLIP)
  {
    return -1;
  }

  return (IMX335_MirrorFlipConfig(&Imx335_Obj, MirrorFlip) == IMX335_OK) ? 0 : -1;
}

int32_t BSP_CAMERA_IMX335_Register(void)
{
  /* Registration power-cycles the sensor: refused while it is initialised. */
  if (Imx335_Initialised != 0U)
  {
    return BSP_ERROR_BUSY;
  }

  Imx335_PowerUp();

  if (Imx335_Bind() != 0)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  Imx335_Sensor.Name         = "IMX335";
  Imx335_Sensor.pObj         = NULL;   /* The object is a singleton here. */
  Imx335_Sensor.pDrv         = &Imx335_Drv;
  Imx335_Sensor.Id           = BSP_CAMERA_IMX335_ID;
  Imx335_Sensor.CsiDataType  = BSP_CAMERA_CSI_DT_RAW10;
  Imx335_Sensor.SensorFormat = BSP_CAMERA_PF_RAW10;
  Imx335_Sensor.BayerOrder   = BSP_CAMERA_BAYER_RGGB;

  /* Only MAIN demosaics, so AUX takes its input from MAIN. */
  Imx335_Sensor.SharedPipes  = 1U;

  Imx335_Sensor.pModes       = Imx335_Modes;
  Imx335_Sensor.ModeCount    = 1U;
  Imx335_Sensor.DataLanes    = 2U;
  Imx335_Sensor.I2cAddress   = (uint16_t)BSP_CAMERA_IMX335_I2C_ADDRESS;

  return BSP_CAMERA_RegisterSensor(&Imx335_Sensor);
}

int32_t BSP_CAMERA_IMX335_SetTiming(uint32_t Fps, uint32_t LinePercent)
{
  if ((Fps == 0U) || (Fps > IMX335_MAX_FPS) ||
      (LinePercent < IMX335_LINE_PERCENT_MIN) || (LinePercent > IMX335_LINE_PERCENT_MAX) ||
      ((Fps * LinePercent) > (IMX335_MAX_FPS * 100U)))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Imx335_Initialised != 0U)
  {
    return BSP_ERROR_BUSY;
  }

  Imx335_Fps         = Fps;
  Imx335_LinePercent = LinePercent;

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_IMX335_SetExposure(uint32_t Us)
{
  if (Imx335_Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  return (Imx335_WriteExposure(Us, Imx335_Ae.GainMdb) == IMX335_OK) ? BSP_ERROR_NONE : BSP_ERROR_COMPONENT_FAILURE;
}

int32_t BSP_CAMERA_IMX335_SetGain(uint32_t MilliDb)
{
  if (MilliDb > BSP_CAMERA_IMX335_GAIN_MAX_MDB)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Imx335_Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  /* Rounded down to the sensor's 0.3 dB step. */
  return (Imx335_WriteExposure(Imx335_Ae.ExposureUs, MilliDb) == IMX335_OK) ? BSP_ERROR_NONE
                                                                            : BSP_ERROR_COMPONENT_FAILURE;
}

int32_t BSP_CAMERA_IMX335_GetExposure(uint32_t *pUs, uint32_t *pMilliDb)
{
  if ((pUs == NULL) || (pMilliDb == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Imx335_Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  *pUs      = Imx335_Ae.ExposureUs;
  *pMilliDb = Imx335_Ae.GainMdb;

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_IMX335_GetExposureMax(uint32_t *pUs)
{
  if (pUs == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Imx335_Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  *pUs = (uint32_t)(((uint64_t)(Imx335_Vmax - IMX335_SHR0_MIN) * Imx335_LineNs) / 1000U);

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_IMX335_SetTestPattern(int32_t Mode)
{
  if ((Mode < -1) || (Mode > 11))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Imx335_Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  return (IMX335_SetTestPattern(&Imx335_Obj, Mode) == IMX335_OK) ? BSP_ERROR_NONE
                                                                 : BSP_ERROR_COMPONENT_FAILURE;
}

int32_t BSP_CAMERA_IMX335_AutoExposure(uint32_t Average, uint32_t Target)
{
  BSP_CAMERA_AE_Config_t config;
  uint32_t exposure_max = 0U;

  if ((Target == 0U) || (Target > 254U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (BSP_CAMERA_IMX335_GetExposureMax(&exposure_max) != BSP_ERROR_NONE)
  {
    return BSP_ERROR_NO_INIT;
  }

  config.Target        = Target;
  config.ExposureMinUs = (Imx335_LineNs + 999U) / 1000U;
  config.ExposureMaxUs = exposure_max;
  config.GainMaxMdb    = BSP_CAMERA_IMX335_AE_GAIN_MAX_MDB;
  config.GainStepMdb   = BSP_CAMERA_IMX335_GAIN_STEP_MDB;
  config.FlickerHz     = BSP_CAMERA_IMX335_AE_FLICKER_HZ;
  config.SettleFrames  = BSP_CAMERA_IMX335_SETTLE_FRAMES;

  if (BSP_CAMERA_AE_Step(&config, &Imx335_Ae, Average) == 0U)
  {
    return BSP_ERROR_NONE;
  }

  if (Imx335_WriteExposure(Imx335_Ae.ExposureUs, Imx335_Ae.GainMdb) != IMX335_OK)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_CAMERA_IMX335_GetLineTiming(uint32_t *pLineNs, uint32_t *pFrameLines)
{
  if ((pLineNs == NULL) || (pFrameLines == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Imx335_Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  *pLineNs     = Imx335_LineNs;
  *pFrameLines = Imx335_Vmax;

  return BSP_ERROR_NONE;
}

void *BSP_CAMERA_IMX335_GetContext(void)
{
  return &Imx335_Obj;
}
