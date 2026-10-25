/**
  * @file    mmc5603nj.h
  * @brief   Component driver for the MMC5603NJ three-axis magnetometer.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  *
  * Register addresses, field layouts, scale factors and timings are taken
  * from the MEMSIC MMC5603NJ datasheet (Rev. B, 2022-01-17). This is an
  * original implementation written against that document; it is not derived
  * from any other driver.
  */

#ifndef MMC5603NJ_H
#define MMC5603NJ_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The control registers are write-only: every write carries the whole register value.
 * On demand (the default) a read triggers a conversion and waits up to about 7 ms;
 * above about 100 Hz use free-running, where a read returns the latest sample. */

/** @brief Fixed I2C address, 8-bit form. */
#define MMC5603NJ_I2C_ADDRESS           0x60U

/** @brief PRODUCT_ID value. */
#define MMC5603NJ_ID                    0x10U

#define MMC5603NJ_XOUT0                 0x00U /*!< Field output, nine bytes.  */
#define MMC5603NJ_TOUT                  0x09U /*!< Temperature output.        */
#define MMC5603NJ_STATUS1               0x18U /*!< Measurement-done flags.    */
#define MMC5603NJ_ODR                   0x1AU /*!< Free-running rate, 1..255. */
#define MMC5603NJ_CTRL0                 0x1BU /*!< Triggers and options.      */
#define MMC5603NJ_CTRL1                 0x1CU /*!< Bandwidth and reset.       */
#define MMC5603NJ_CTRL2                 0x1DU /*!< Free-running enable.       */
#define MMC5603NJ_PRODUCT_ID            0x39U /*!< Product identity.          */

/* STATUS1 flags clear when the matching output register is read. */
#define MMC5603NJ_STATUS1_MEAS_M_DONE   0x40U /*!< Field measurement ready.       */
#define MMC5603NJ_STATUS1_MEAS_T_DONE   0x80U /*!< Temperature measurement ready. */

#define MMC5603NJ_CTRL0_TAKE_MEAS_M     0x01U /*!< Start one field measurement.   */
#define MMC5603NJ_CTRL0_TAKE_MEAS_T     0x02U /*!< Start one temperature reading. */
#define MMC5603NJ_CTRL0_DO_SET          0x08U /*!< Magnetise pulse.               */
#define MMC5603NJ_CTRL0_DO_RESET        0x10U /*!< Opposite-polarity pulse.       */
#define MMC5603NJ_CTRL0_AUTO_SR_EN      0x20U /*!< Degauss around each measurement. */
#define MMC5603NJ_CTRL0_CMM_FREQ_EN     0x80U /*!< Latch the free-running rate.   */

/* Bandwidth caps the free-running rate: 6.6 ms 75 Hz, 3.5 ms 150 Hz, 2.0 ms 255 Hz. */
#define MMC5603NJ_CTRL1_BW_6MS6         0x00U
#define MMC5603NJ_CTRL1_BW_3MS5         0x01U
#define MMC5603NJ_CTRL1_BW_2MS0         0x02U
#define MMC5603NJ_CTRL1_BW_1MS2         0x03U
#define MMC5603NJ_CTRL1_SW_RESET        0x80U /*!< Full reset and reload.  */

#define MMC5603NJ_CTRL2_CMM_EN          0x10U /*!< Start free-running measurement. */

#define MMC5603NJ_OK                    0
#define MMC5603NJ_ERROR                 -1

/** @brief Bus type reported to the driver by the board layer. */
#define MMC5603NJ_I2C_BUS               0x00U

/** @brief Widest field the part can report, in gauss. */
#define MMC5603NJ_FULL_SCALE_GAUSS      30

/** @brief Fastest free-running rate the part offers, in Hz. */
#define MMC5603NJ_MAX_ODR_HZ            255.0f

typedef int32_t (*MMC5603NJ_Init_Func)(void);
typedef int32_t (*MMC5603NJ_DeInit_Func)(void);
typedef int32_t (*MMC5603NJ_GetTick_Func)(void);
typedef void    (*MMC5603NJ_Delay_Func)(uint32_t);
typedef int32_t (*MMC5603NJ_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*MMC5603NJ_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

/** @brief Bus access supplied by the board. */
typedef struct
{
  MMC5603NJ_Init_Func     Init;     /*!< Bring the bus up.                   */
  MMC5603NJ_DeInit_Func   DeInit;   /*!< Release the bus.                    */
  uint32_t                BusType;  /*!< MMC5603NJ_I2C_BUS.                  */
  uint8_t                 Address;  /*!< Device address, 8-bit form.         */
  MMC5603NJ_WriteReg_Func WriteReg;
  MMC5603NJ_ReadReg_Func  ReadReg;
  MMC5603NJ_GetTick_Func  GetTick;  /*!< Millisecond tick, for the waits.    */
  MMC5603NJ_Delay_Func    Delay;    /*!< Millisecond delay, for reset waits. */
} MMC5603NJ_IO_t;

/** @brief Three-axis sample in milligauss. */
typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} MMC5603NJ_Axes_t;

/** @brief Three-axis sample in counts, centred on zero field. */
typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} MMC5603NJ_AxesRaw_t;

/** @brief Filled by MMC5603NJ_GetCapabilities. */
typedef struct
{
  uint8_t  Acc;
  uint8_t  Gyro;
  uint8_t  Magneto;
  uint8_t  LowPower;
  uint32_t GyroMaxFS;
  uint32_t AccMaxFS;
  uint32_t MagMaxFS;   /*!< Gauss. */
  float    GyroMaxOdr;
  float    AccMaxOdr;
  float    MagMaxOdr;  /*!< Hz.    */
} MMC5603NJ_Capabilities_t;

/** @brief Driver state. One per fitted part; the board layer owns it. */
typedef struct
{
  MMC5603NJ_IO_t IO;
  uint8_t        IsInitialized; /*!< 1 once MMC5603NJ_Init has succeeded.    */
  uint8_t        IsEnabled;     /*!< 1 while the magnetometer is in use.     */
  uint16_t       ContinuousHz;  /*!< Free-running rate, 0 in on-demand mode. */
  uint8_t        Bandwidth;     /*!< CTRL1 bandwidth selection in force.     */
  uint8_t        Running;       /*!< 1 while the part is free-running.       */
  uint8_t        HaveTemp;      /*!< 1 once LastTempDegC holds a reading.    */
  uint32_t       SampleTick;    /*!< Last sample read, or the start, in ms.  */
  float          LastTempDegC;  /*!< Last good temperature reading.          */
} MMC5603NJ_Object_t;

/** @brief Copy pIO into pObj and check PRODUCT_ID. */
int32_t MMC5603NJ_RegisterBusIO(MMC5603NJ_Object_t *pObj, const MMC5603NJ_IO_t *pIO);

/** @brief Reset the part (no answer for about 20 ms), degauss it, measure on demand. */
int32_t MMC5603NJ_Init(MMC5603NJ_Object_t *pObj);

/** @brief Return the part to on-demand measurement and release the bus. */
int32_t MMC5603NJ_DeInit(MMC5603NJ_Object_t *pObj);

/** @brief Read PRODUCT_ID (MMC5603NJ_ID on a working part). */
int32_t MMC5603NJ_ReadID(MMC5603NJ_Object_t *pObj, uint8_t *pId);

int32_t MMC5603NJ_GetCapabilities(MMC5603NJ_Object_t *pObj,
                                  MMC5603NJ_Capabilities_t *pCapabilities);

/** @brief Mark in use and restart the free-running rate, if one is set.
  * @note  The part has no power-down state: on demand it draws nothing between readings. */
int32_t MMC5603NJ_MAG_Enable(MMC5603NJ_Object_t *pObj);

/** @brief Stop free-running (the rate is kept) and mark the magnetometer idle. */
int32_t MMC5603NJ_MAG_Disable(MMC5603NJ_Object_t *pObj);

/** @brief Field in milligauss. On demand it blocks for up to about 7 ms. */
int32_t MMC5603NJ_MAG_GetAxes(MMC5603NJ_Object_t *pObj, MMC5603NJ_Axes_t *pAxes);

/** @brief Field in counts, with 0 at zero field. */
int32_t MMC5603NJ_MAG_GetAxesRaw(MMC5603NJ_Object_t *pObj, MMC5603NJ_AxesRaw_t *pAxes);

/** @brief *pReady = 1 when a fresh sample can be read, judged by the clock, not the part.
  * @note  Keep the bus quiet while it measures: a STATUS1 read during a set/reset
  *        conversion zeros all nine outputs, and polling the outputs stops updates.
  * @note  Free-running: ready one period after the last read (or the start). On demand,
  *        or with no GetTick: always 1. */
int32_t MMC5603NJ_MAG_GetDataReady(MMC5603NJ_Object_t *pObj, uint8_t *pReady);

/** @brief Milligauss per count (fixed: the part has one range). */
int32_t MMC5603NJ_MAG_GetSensitivity(MMC5603NJ_Object_t *pObj, float *pSensitivity);

/** @brief Free-running rate in Hz, 0 in on-demand mode. */
int32_t MMC5603NJ_MAG_GetOutputDataRate(MMC5603NJ_Object_t *pObj, float *pOdr);

/** @brief Free-running at Odr Hz (1-255, rounded up, clamped at 255), or 0 for on demand.
  * @note  The bandwidth is widened as needed: the part silently misses a rate it cannot
  *        sustain.
  * @note  Returning to on demand takes about 40 ms: the part usually ignores the first
  *        trigger after free-running stops, so one is spent here. */
int32_t MMC5603NJ_MAG_SetOutputDataRate(MMC5603NJ_Object_t *pObj, float Odr);

/** @brief Field range in gauss. */
int32_t MMC5603NJ_MAG_GetFullScale(MMC5603NJ_Object_t *pObj, int32_t *pFullScale);

/** @brief The range is fixed: MMC5603NJ_OK at or below it, MMC5603NJ_ERROR above. */
int32_t MMC5603NJ_MAG_SetFullScale(MMC5603NJ_Object_t *pObj, int32_t FullScale);

/** @brief One set/reset pulse pair: clears the offset a strong field leaves behind.
  * @note  Free-running measurement is paused for the pulses and restarted. */
int32_t MMC5603NJ_MAG_Degauss(MMC5603NJ_Object_t *pObj);

/** @brief Die temperature in degrees C, 0.8 degree resolution.
  * @note  Measured on demand only: free-running pauses for about 50 ms.
  * @retval MMC5603NJ_ERROR if the bus failed, or if the part did not measure and there
  *         is no earlier reading to return. */
int32_t MMC5603NJ_GetTemperature(MMC5603NJ_Object_t *pObj, float *pTemp);

int32_t MMC5603NJ_ReadReg(MMC5603NJ_Object_t *pObj, uint8_t Reg, uint8_t *pData);

int32_t MMC5603NJ_WriteReg(MMC5603NJ_Object_t *pObj, uint8_t Reg, uint8_t Data);

#ifdef __cplusplus
}
#endif

#endif /* MMC5603NJ_H */
