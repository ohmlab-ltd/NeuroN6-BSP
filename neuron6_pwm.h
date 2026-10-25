/**
  * @file    neuron6_pwm.h
  * @brief   PWM on the Neuro N6's timer pins, and a tone generator on any pin.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_PWM_H
#define NEURON6_PWM_H

#include "neuron6_conf.h"
#include "neuron6_errno.h"
#include "neuron6_io.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Timer outputs reach five pins: D5, A4 and the three LED colours (see BSP_IO_HasPwm). */

/** @brief Tone timer: TIM3 is free (the PWM uses TIM1/2/15/17, the HAL tick TIM16). */
#define BSP_TONE_TIMER                  TIM3
#define BSP_TONE_TIMER_IRQn             TIM3_IRQn

/** @brief Tone interrupt priority; it fires twice per tone cycle, so it is a busy one. */
#ifndef BSP_TONE_IT_PRIORITY
#define BSP_TONE_IT_PRIORITY            8U
#endif

/** @brief Default modulation frequency in Hz, applied until changed. */
#ifndef BSP_PWM_DEFAULT_FREQUENCY
#define BSP_PWM_DEFAULT_FREQUENCY       1000U
#endif

/** @brief Accepted modulation frequency range, in Hz. */
#define BSP_PWM_MIN_FREQUENCY           1U
#define BSP_PWM_MAX_FREQUENCY           1000000U

/** @brief Default duty resolution in bits (8: duty 0..255). */
#ifndef BSP_PWM_DEFAULT_RESOLUTION
#define BSP_PWM_DEFAULT_RESOLUTION      8U
#endif

/** @brief Accepted duty resolution range, in bits. */
#define BSP_PWM_MIN_RESOLUTION          4U
#define BSP_PWM_MAX_RESOLUTION          16U

/**
  * @brief  Start the cycle counter the timer clocks are measured against.
  * @retval BSP_ERROR_FEATURE_NOT_SUPPORTED if it does not run; the module is unusable.
  */
int32_t BSP_PWM_Init(void);

/** @brief Stop every output and the tone, and release the timers. */
int32_t BSP_PWM_DeInit(void);

/** @brief Modulation frequency in Hz, applied by each timer at its next BSP_PWM_Start. */
int32_t BSP_PWM_SetFrequency(uint32_t Frequency);

uint32_t BSP_PWM_GetFrequency(void);

/**
  * @brief  Bits of the BSP_PWM_Start duty argument (8: 0..255).
  * @note   Only the argument's range: the timer period sets the real step, so 16 bits at
  *         a high frequency is not a finer output.
  */
int32_t BSP_PWM_SetResolution(uint32_t Bits);

uint32_t BSP_PWM_GetResolution(void);

/**
  * @brief  Drive a BSP_IO_HasPwm pin; DutyCycle 0..2^resolution - 1, clamped.
  * @note   Duty is high time, before any board inversion. A timer's first start
  *         measures its clock (about 50 us).
  */
int32_t BSP_PWM_Start(uint32_t Pin, uint32_t DutyCycle);

/**
  * @brief  Stop a pin; it rests as an output, low (high, so off, for the active-low LED).
  * @retval BSP_ERROR_NO_INIT if the pin was not being driven.
  */
int32_t BSP_PWM_Stop(uint32_t Pin);

/** @brief The pin's timer clock in Hz, measured on first use (about 50 us). */
int32_t BSP_PWM_GetTimerClockFreq(uint32_t Pin, uint32_t *pHz);

/**
  * @brief  Play Frequency Hz on any routed pin for Duration_ms (0: until stopped).
  * @note   One tone at a time: BSP_ERROR_BUSY for another pin while one plays.
  * @note   The application's TIM3_IRQHandler must call BSP_TONE_IRQHandler.
  */
int32_t BSP_TONE_Start(uint32_t Pin, uint32_t Frequency, uint32_t Duration_ms);

/** @brief Stop the tone; its pin rests low (high for the active-low LED). */
int32_t BSP_TONE_Stop(void);

int32_t BSP_TONE_IsActive(void);

/** @brief Call from the application's TIM3_IRQHandler. */
void BSP_TONE_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_PWM_H */
