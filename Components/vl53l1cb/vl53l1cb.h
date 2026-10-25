/**
  * @file    vl53l1cb.h
  * @brief   VL53L1CB time-of-flight driver, using the ranging the part does itself.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  *
  * The register addresses and the start-up sequence are those of
  * STMicroelectronics' VL53L1X ultra lite driver (STSW-IMG009, BSD-3-Clause).
  * Its default configuration, the only content taken from it, is in
  * vl53l1cb_st_tables.h with ST's notice.
  */

#ifndef VL53L1CB_H
#define VL53L1CB_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* A 16 x 16 SPAD array, ranged one region at a time by the part itself (default
 * configuration: long distance mode, 100 ms budget). The driver replaces two of its
 * limits so that a 4 x 4 region ranges too. */

#define VL53L1CB_OK                 0
#define VL53L1CB_ERROR              (-1)
#define VL53L1CB_TIMEOUT            (-2)

/** @brief Bus type for VL53L1CB_IO_t.BusType. */
#define VL53L1CB_I2C_BUS            0U

/** @brief Device address, 8-bit form of 0x29. */
#define VL53L1CB_DEVICE_ADDRESS     0x52U

/** @brief Identity: model 0xEA, module type 0xCC. */
#define VL53L1CB_ID                 0xEACCU

/** @brief SPADs on each side of the detector array. */
#define VL53L1CB_SPADS              16U

/** @brief Narrowest region the part ranges, in SPADs on a side. */
#define VL53L1CB_MIN_ROI            4U

/** @brief SPAD at the centre of the whole array, for a 16 x 16 region. */
#define VL53L1CB_CENTRE_SPAD        199U

/** @brief Timing budget of the default configuration, in milliseconds. */
#define VL53L1CB_TIMING_BUDGET_MS   100U

/** @brief Weakest return ranged, in kcps, in place of the configuration's 1.0 MCPS:
  *        a 4 x 4 region sees only 16 of the 256 SPADs. */
#define VL53L1CB_MIN_SIGNAL_KCPS    125U

/** @brief Least certain range reported, sigma in mm (the configuration's is 90 mm);
  *        it refuses the uncertain ranges that weak returns give. */
#define VL53L1CB_SIGMA_LIMIT_MM     15U

/** @brief Range status of a good range (RESULT__RANGE_STATUS, bits 4..0). */
#define VL53L1CB_RANGE_VALID        9U

/** @brief Range status when the return was under VL53L1CB_MIN_SIGNAL_KCPS. */
#define VL53L1CB_RANGE_SIGNAL_FAIL  4U

/** @brief Range status when sigma was over VL53L1CB_SIGMA_LIMIT_MM. */
#define VL53L1CB_RANGE_SIGMA_FAIL   6U

typedef int32_t  (*VL53L1CB_Init_Func)(void);
typedef int32_t  (*VL53L1CB_DeInit_Func)(void);
typedef int32_t  (*VL53L1CB_GetTick_Func)(void);
typedef void     (*VL53L1CB_Delay_Func)(uint32_t);
typedef int32_t  (*VL53L1CB_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t  (*VL53L1CB_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

/** @brief Bus access supplied by the board. */
typedef struct
{
  VL53L1CB_Init_Func     Init;     /*!< Bring the bus up.                          */
  VL53L1CB_DeInit_Func   DeInit;   /*!< Release the bus.                           */
  uint32_t               BusType;  /*!< VL53L1CB_I2C_BUS.                          */
  uint8_t                Address;  /*!< Device address, 8-bit form.                */
  VL53L1CB_WriteReg_Func WriteReg; /*!< Register write, 16-bit register address.   */
  VL53L1CB_ReadReg_Func  ReadReg;  /*!< Register read, 16-bit register address.    */
  VL53L1CB_GetTick_Func  GetTick;  /*!< Millisecond tick, for the waits.           */
  VL53L1CB_Delay_Func    Delay;    /*!< Millisecond delay.                         */
} VL53L1CB_IO_t;

/** @brief Driver state. One per fitted part; the board layer owns it. */
typedef struct
{
  VL53L1CB_IO_t IO;
  uint8_t       IsInitialized; /*!< 1 once VL53L1CB_Init has succeeded.        */
  uint8_t       IsRanging;     /*!< 1 between StartRanging and StopRanging.    */
} VL53L1CB_Object_t;

/** @brief Copy pIO into pObj and bring the bus up. */
int32_t VL53L1CB_RegisterBusIO(VL53L1CB_Object_t *pObj, const VL53L1CB_IO_t *pIO);

/** @brief Reset, load the default configuration, calibrate the VHV bias with one
  *        measurement, then apply VL53L1CB_MIN_SIGNAL_KCPS and VL53L1CB_SIGMA_LIMIT_MM.
  * @note  The reset clears another driver's setup (histogram ranging stops the part
  *        reporting ranges of its own). Takes about 150 ms.
  * @retval VL53L1CB_ERROR if the part is not a VL53L1CB or a register access failed,
  *         VL53L1CB_TIMEOUT if it did not boot or finish its first measurement. */
int32_t VL53L1CB_Init(VL53L1CB_Object_t *pObj);

/** @brief Stop ranging and release the bus. */
int32_t VL53L1CB_DeInit(VL53L1CB_Object_t *pObj);

/** @brief Read the identity (VL53L1CB_ID on a working part). */
int32_t VL53L1CB_ReadID(VL53L1CB_Object_t *pObj, uint16_t *pId);

/** @brief SPAD number of Row (0-15, from the bottom) and Col (0-15, from the left);
  *        0 outside the array.
  * @note  SPADs are numbered in halves: the top eight rows 128-255, column by column
  *        from the left; the bottom eight 127 down to 0. */
uint8_t VL53L1CB_SpadNumber(uint8_t Row, uint8_t Col);

/** @brief Range a Width x Height region (VL53L1CB_MIN_ROI to 16) centred on CentreSpad.
  * @note  For an even size the centre is the SPAD above and right of the middle, as
  *        VL53L1CB_CENTRE_SPAD is for the whole array. */
int32_t VL53L1CB_SetROI(VL53L1CB_Object_t *pObj, uint8_t Width, uint8_t Height,
                        uint8_t CentreSpad);

/** @brief Start timed ranging, one measurement per timing budget.
  * @note  The part's single-shot start gives no valid range: for one measurement,
  *        start, read the first result, then call VL53L1CB_StopRanging. */
int32_t VL53L1CB_StartRanging(VL53L1CB_Object_t *pObj);

/** @brief Stop ranging and clear any waiting result. */
int32_t VL53L1CB_StopRanging(VL53L1CB_Object_t *pObj);

/** @brief *pReady = 1 when a result is waiting, from the interrupt status (pin unused). */
int32_t VL53L1CB_DataReady(VL53L1CB_Object_t *pObj, uint8_t *pReady);

/** @brief Read and clear the waiting result: distance in mm and range status
  *        (RESULT__RANGE_STATUS bits 4..0); only VL53L1CB_RANGE_VALID is a good range. */
int32_t VL53L1CB_ReadResult(VL53L1CB_Object_t *pObj, uint16_t *pDistanceMm,
                            uint8_t *pRangeStatus);

#ifdef __cplusplus
}
#endif

#endif /* VL53L1CB_H */
