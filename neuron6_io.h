/**
  * @file    neuron6_io.h
  * @brief   Neuro N6 header pins: pin table, digital I/O, EXTI, pulse and shift helpers.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_IO_H
#define NEURON6_IO_H

#include "neuron6_conf.h"
#include "neuron6_errno.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The board's Nano-style header: one table row per silkscreen pin (port, mask, ADC and
 * timer routes), shared with neuron6_adc and neuron6_pwm. */

/** @brief Number of rows in the pin table, routed and unrouted alike. */
#define BSP_IO_PIN_NBR                  29U

/** @brief Value that is never a valid pin number. */
#define BSP_IO_PIN_NONE                 0xFFFFFFFFU

#define BSP_IO_PIN_D0                   0U   /*!< PD0,  header RX. */
#define BSP_IO_PIN_D1                   1U   /*!< PD1,  header TX. */
#define BSP_IO_PIN_D5                   5U   /*!< PE9,  TIM1_CH1. */
#define BSP_IO_PIN_D6                   6U   /*!< PF5. */
#define BSP_IO_PIN_D9                   9U   /*!< PD12. */
#define BSP_IO_PIN_D10                  10U  /*!< PF10, the chip select by convention. */
#define BSP_IO_PIN_D11                  11U  /*!< PF15. */
#define BSP_IO_PIN_D12                  12U  /*!< PA3. */
#define BSP_IO_PIN_D13                  13U  /*!< PG14. */
#define BSP_IO_PIN_A0                   14U  /*!< PF13, ADC2_INP2. */
#define BSP_IO_PIN_A1                   15U  /*!< PF11, ADC1_INP2. */
#define BSP_IO_PIN_A2                   16U  /*!< PF12, ADC1_INP6. */
#define BSP_IO_PIN_A3                   17U  /*!< PF3,  ADC1_INP16. */
#define BSP_IO_PIN_A4                   18U  /*!< PF14, ADC2_INP6, TIM2_CH2. */
#define BSP_IO_PIN_A5                   19U  /*!< PG15, ADC1_INP7. */
#define BSP_IO_PIN_SDA                  20U  /*!< PD15, header I2C2_SDA. */
#define BSP_IO_PIN_SCL                  21U  /*!< PD14, header I2C2_SCL. */
#define BSP_IO_PIN_MOSI                 22U  /*!< PD7,  header SPI2_MOSI. */
#define BSP_IO_PIN_MISO                 23U  /*!< PD11, header SPI2_MISO. */
#define BSP_IO_PIN_SCK                  24U  /*!< PF2,  header SPI2_SCK. */
#define BSP_IO_PIN_LED_RED              25U  /*!< PE11, TIM1_CH2.  Also BSP_LED_* LED1. */
#define BSP_IO_PIN_LED_GREEN            26U  /*!< PD6,  TIM15_CH2. Also BSP_LED_* LED2. */
#define BSP_IO_PIN_LED_BLUE             27U  /*!< PC1,  TIM17_CH1. Also BSP_LED_* LED3. */
#define BSP_IO_PIN_BUTTON               28U  /*!< PG8.  Also BSP_PB_* BUTTON_USER. */

#define BSP_IO_MODE_INPUT               0U /*!< High impedance, no pull.      */
#define BSP_IO_MODE_INPUT_PU            1U /*!< Input, internal pull-up.      */
#define BSP_IO_MODE_INPUT_PD            2U /*!< Input, internal pull-down.    */
#define BSP_IO_MODE_OUTPUT_PP           3U /*!< Push-pull output.             */
#define BSP_IO_MODE_OUTPUT_OD           4U /*!< Open-drain output, no pull.   */
#define BSP_IO_MODE_ANALOG              5U /*!< Analogue, pulls disconnected. */

#define BSP_IO_PIN_RESET                0U /*!< Low.  */
#define BSP_IO_PIN_SET                  1U /*!< High. */

#define BSP_IO_TRIGGER_RISING           0U
#define BSP_IO_TRIGGER_FALLING          1U
#define BSP_IO_TRIGGER_BOTH             2U

#define BSP_IO_MSB_FIRST                0U
#define BSP_IO_LSB_FIRST                1U

/** @brief Longest BSP_IO_ReadPulse timeout in us (its cycle counter wraps at 2^32). */
#define BSP_IO_PULSE_MAX_TIMEOUT_US     5000000U

/** @brief Header EXTI priority, below camera, audio and USB: a slow callback must never
  * delay the capture path. */
#ifndef BSP_IO_IT_PRIORITY
#define BSP_IO_IT_PRIORITY              8U
#endif

/** @brief One header pin's routes, from BSP_IO_GetPinInfo(). */
typedef struct
{
  GPIO_TypeDef *Port;         /*!< GPIO port, NULL if the pin is not routed.   */
  uint16_t      Pin;          /*!< GPIO_PIN_x mask.                            */
  uint8_t       Line;         /*!< Pin index 0..15, which is its EXTI line.    */
  uint8_t       AdcInstance;  /*!< 1 for ADC1, 2 for ADC2, 0 if not analogue.  */
  uint8_t       AdcChannel;   /*!< Channel (INPx) index, valid with the above. */
  TIM_TypeDef  *PwmTimer;     /*!< Timer behind the pin, NULL if none.         */
  uint8_t       PwmChannel;   /*!< Timer channel 1..4, valid with the above.   */
  uint8_t       PwmAlternate; /*!< GPIO alternate function for that timer.     */
} BSP_IO_PinInfo_t;

/** @brief Header-pin interrupt callback; it gets the pin number. */
typedef void (*BSP_IO_Callback_t)(uint32_t Pin);

/** @brief Start the cycle counter for BSP_IO_ReadPulse; optional for digital I/O. */
int32_t BSP_IO_Init(void);

/** @brief Detach interrupts from, and reset, every pin this module configured. */
int32_t BSP_IO_DeInit(void);

/** @brief Set a pin's BSP_IO_MODE_*; an output keeps the level its data register held. */
int32_t BSP_IO_ConfigPin(uint32_t Pin, uint32_t Mode);

/** @brief Drive a pin with one atomic BSRR store, so safe from an interrupt. */
int32_t BSP_IO_WritePin(uint32_t Pin, uint32_t PinState);

/** @brief BSP_IO_PIN_SET or BSP_IO_PIN_RESET, or a negative error. */
int32_t BSP_IO_ReadPin(uint32_t Pin);

/** @brief Invert a pin: reads ODR, then one BSRR store. */
int32_t BSP_IO_TogglePin(uint32_t Pin);

/** @brief Copy a pin's table row; BSP_ERROR_WRONG_PARAM for an unrouted pin. */
int32_t BSP_IO_GetPinInfo(uint32_t Pin, BSP_IO_PinInfo_t *pInfo);

/** @brief 1 if neuron6_pwm can drive the pin from a timer, else 0; negative on error. */
int32_t BSP_IO_HasPwm(uint32_t Pin);

/** @brief 1 if neuron6_adc can sample the pin, else 0; negative on error. */
int32_t BSP_IO_HasAdc(uint32_t Pin);

/**
  * @brief  Call Callback(Pin) from interrupt context on each Trigger edge of the pin.
  * @note   The pin becomes an input and keeps its pull; earlier latched edges are dropped.
  * @note   One EXTI line per pin index across all ports: BSP_ERROR_BUSY if another pin or
  *         owner holds it (line 7 is the radio's PB7 wake once neuron6_wifi arms it).
  * @note   The application's EXTIn_IRQHandler calls BSP_IO_IRQHandler(n), n = the pin's
  *         Line. EXTI8_IRQHandler calls BSP_PB_IRQHandler(BUTTON_USER) first: pin 28 is
  *         the user button, so use it here or through BSP_PB_Init, not both.
  */
int32_t BSP_IO_AttachInterrupt(uint32_t Pin, uint32_t Trigger, BSP_IO_Callback_t Callback);

/**
  * @brief  Release the pin's EXTI line, leaving a plain input with its pull.
  * @retval BSP_ERROR_NO_INIT if no interrupt was attached.
  */
int32_t BSP_IO_DetachInterrupt(uint32_t Pin);

/** @brief Service EXTI Line (0..15): clear both edge flags, run the callback if any. */
void BSP_IO_IRQHandler(uint32_t Line);

/**
  * @brief  Width in us of one State pulse on an input pin, busy-waiting.
  * @param  Timeout_us Covers the pulse in progress, the wait and the pulse; clamped to
  *         BSP_IO_PULSE_MAX_TIMEOUT_US.
  * @note   Interrupts stay enabled, so one taken during the pulse lengthens the reading.
  * @retval BSP_ERROR_BUSY if no whole pulse fitted in the budget.
  */
int32_t BSP_IO_ReadPulse(uint32_t Pin, uint32_t State, uint32_t Timeout_us, uint32_t *pWidth_us);

/**
  * @brief  Clock a byte out; DataPin and ClockPin already outputs, the clock low.
  * @note   No delays: the edges can be too fast for some discrete logic.
  */
int32_t BSP_IO_ShiftOut(uint32_t DataPin, uint32_t ClockPin, uint32_t BitOrder, uint8_t Value);

/** @brief Clock a byte in; DataPin already an input, ClockPin an output and low. */
int32_t BSP_IO_ShiftIn(uint32_t DataPin, uint32_t ClockPin, uint32_t BitOrder, uint8_t *pValue);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_IO_H */
