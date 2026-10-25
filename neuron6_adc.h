/**
  * @file    neuron6_adc.h
  * @brief   Analogue input for the Neuro N6: six header pins and three internal channels.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_ADC_H
#define NEURON6_ADC_H

#include "neuron6_conf.h"
#include "neuron6_errno.h"
#include "neuron6_io.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A1, A2, A3 and A5 are on ADC1, A0 and A4 on ADC2. Full scale is 1.8 V: the reference
 * is VDDA_1V8 and the pads are not rated above it. */

/** @brief HCLK divider for both ADCs: 16 keeps them in spec (400 MHz gives 25 MHz). */
#ifndef BSP_ADC_CLOCK_DIVIDER
#define BSP_ADC_CLOCK_DIVIDER           16U
#endif

/** @brief Sampling time for every channel (deliberately generous). */
#ifndef BSP_ADC_SAMPLING_TIME
#define BSP_ADC_SAMPLING_TIME           ADC_SAMPLETIME_246CYCLES_5
#endif

/** @brief Conversion widths, programmed into the converter: narrower is faster. */
#define BSP_ADC_RESOLUTION_6B           ADC_RESOLUTION_6B  /*!< 0..63    */
#define BSP_ADC_RESOLUTION_8B           ADC_RESOLUTION_8B  /*!< 0..255   */
#define BSP_ADC_RESOLUTION_10B          ADC_RESOLUTION_10B /*!< 0..1023  */
#define BSP_ADC_RESOLUTION_12B          ADC_RESOLUTION_12B /*!< 0..4095  */

/** @brief The package divides VBAT by this before the converter, as it can exceed VDDA. */
#define BSP_ADC_VBAT_DIVIDER            4U

/**
  * @brief  Bring up and calibrate both converters; reference counted.
  * @note   Optional: every read brings up the converter it needs.
  */
int32_t BSP_ADC_Init(void);

/** @brief Release one user; the last one shuts the converters down. */
int32_t BSP_ADC_DeInit(void);

/**
  * @brief  Set the width (BSP_ADC_RESOLUTION_*) of later pin reads, on both converters.
  * @note   Internal channels always convert at 12 bits, the factory calibration's width.
  */
int32_t BSP_ADC_SetResolution(uint32_t Resolution);

uint32_t BSP_ADC_GetResolution(void);

/**
  * @brief  Raw reading of BSP_IO_PIN_A0..A5, right aligned at the width in force.
  * @note   Leaves the pad analogue (pulls off); reconfigure it through neuron6_io to
  *         read it digitally again.
  * @warning 1.8 V full scale, not 3.3 V: the pads are not rated above 1.8 V.
  */
int32_t BSP_ADC_ReadChannel(uint32_t Pin, uint32_t *pValue);

/** @brief BSP_IO_PIN_A0..A5 in mV (1.8 V full scale), scaled by BSP_ADC_GetVdda(). */
int32_t BSP_ADC_ReadVoltage(uint32_t Pin, uint32_t *pMillivolts);

/**
  * @brief  Analogue supply in millivolts, from VREFINT and its factory calibration.
  * @note   Without a usable calibration (the word reads zero until OTP word 110 is
  *         reloaded through BSEC) it returns BSP_ADC_NOMINAL_VDDA_MV (1800) instead;
  *         BSP_ADC_IsVddaMeasured() says which.
  */
int32_t BSP_ADC_GetVdda(uint32_t *pMillivolts);

/** @brief 1 if the last BSP_ADC_GetVdda() measured the supply, 0 if assumed or none yet. */
int32_t BSP_ADC_IsVddaMeasured(void);

/** @brief Battery rail in millivolts, the package's divider undone. */
int32_t BSP_ADC_ReadVBat(uint32_t *pMillivolts);

/** @brief Core supply rail in millivolts. */
int32_t BSP_ADC_ReadVddCore(uint32_t *pMillivolts);

/**
  * @brief  Claim the converters: two tasks converting at once can swap results.
  * @note   Weak and empty; a recursive mutex under FreeRTOS (neuron6_rtos.c). The
  *         module never nests it, so a replacement need not be recursive.
  */
void BSP_ADC_Lock(void);

void BSP_ADC_Unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_ADC_H */
