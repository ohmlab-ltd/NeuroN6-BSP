/**
  * @file    neuron6_io.c
  * @brief   Neuro N6 header pins: pin table, digital I/O, EXTI, pulse and shift helpers.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6_io.h"

typedef struct
{
  BSP_IO_Callback_t Callback; /*!< NULL when the line is free.  */
  uint32_t          Pin;      /*!< Pin number that owns the line. */
} IO_ExtiSlot_t;

/* Row for a header position this board does not route. */
#define IO_PIN_NOT_ROUTED   { NULL, 0U, 0U, 0U, 0U, NULL, 0U, 0U }

/* Ports and masks follow the carrier schematic. */
static const BSP_IO_PinInfo_t IO_PIN_TABLE[BSP_IO_PIN_NBR] =
{
  /*  0 D0    PD0  */ { GPIOD, GPIO_PIN_0,  0U,  0U, 0U,  NULL,  0U, 0U },
  /*  1 D1    PD1  */ { GPIOD, GPIO_PIN_1,  1U,  0U, 0U,  NULL,  0U, 0U },
  /*  2       ---  */ IO_PIN_NOT_ROUTED,
  /*  3       ---  */ IO_PIN_NOT_ROUTED,
  /*  4       ---  */ IO_PIN_NOT_ROUTED,
  /*  5 D5    PE9  */ { GPIOE, GPIO_PIN_9,  9U,  0U, 0U,  TIM1,  1U, GPIO_AF1_TIM1  },
  /*  6 D6    PF5  */ { GPIOF, GPIO_PIN_5,  5U,  0U, 0U,  NULL,  0U, 0U },
  /*  7       ---  */ IO_PIN_NOT_ROUTED,
  /*  8       ---  */ IO_PIN_NOT_ROUTED,
  /*  9 D9    PD12 */ { GPIOD, GPIO_PIN_12, 12U, 0U, 0U,  NULL,  0U, 0U },
  /* 10 D10   PF10 */ { GPIOF, GPIO_PIN_10, 10U, 0U, 0U,  NULL,  0U, 0U },
  /* 11 D11   PF15 */ { GPIOF, GPIO_PIN_15, 15U, 0U, 0U,  NULL,  0U, 0U },
  /* 12 D12   PA3  */ { GPIOA, GPIO_PIN_3,  3U,  0U, 0U,  NULL,  0U, 0U },
  /* 13 D13   PG14 */ { GPIOG, GPIO_PIN_14, 14U, 0U, 0U,  NULL,  0U, 0U },
  /* 14 A0    PF13 */ { GPIOF, GPIO_PIN_13, 13U, 2U, 2U,  NULL,  0U, 0U },
  /* 15 A1    PF11 */ { GPIOF, GPIO_PIN_11, 11U, 1U, 2U,  NULL,  0U, 0U },
  /* 16 A2    PF12 */ { GPIOF, GPIO_PIN_12, 12U, 1U, 6U,  NULL,  0U, 0U },
  /* 17 A3    PF3  */ { GPIOF, GPIO_PIN_3,  3U,  1U, 16U, NULL,  0U, 0U },
  /* 18 A4    PF14 */ { GPIOF, GPIO_PIN_14, 14U, 2U, 6U,  TIM2,  2U, GPIO_AF1_TIM2  },
  /* 19 A5    PG15 */ { GPIOG, GPIO_PIN_15, 15U, 1U, 7U,  NULL,  0U, 0U },
  /* 20 SDA   PD15 */ { GPIOD, GPIO_PIN_15, 15U, 0U, 0U,  NULL,  0U, 0U },
  /* 21 SCL   PD14 */ { GPIOD, GPIO_PIN_14, 14U, 0U, 0U,  NULL,  0U, 0U },
  /* 22 MOSI  PD7  */ { GPIOD, GPIO_PIN_7,  7U,  0U, 0U,  NULL,  0U, 0U },
  /* 23 MISO  PD11 */ { GPIOD, GPIO_PIN_11, 11U, 0U, 0U,  NULL,  0U, 0U },
  /* 24 SCK   PF2  */ { GPIOF, GPIO_PIN_2,  2U,  0U, 0U,  NULL,  0U, 0U },
  /* 25 LEDR  PE11 */ { GPIOE, GPIO_PIN_11, 11U, 0U, 0U,  TIM1,  2U, GPIO_AF1_TIM1  },
  /* 26 LEDG  PD6  */ { GPIOD, GPIO_PIN_6,  6U,  0U, 0U,  TIM15, 2U, GPIO_AF4_TIM15 },
  /* 27 LEDB  PC1  */ { GPIOC, GPIO_PIN_1,  1U,  0U, 0U,  TIM17, 1U, GPIO_AF1_TIM17 },
  /* 28 USER  PG8  */ { GPIOG, GPIO_PIN_8,  8U,  0U, 0U,  NULL,  0U, 0U },
};

/* One callback slot per EXTI line; the line is the pin index. */
static volatile IO_ExtiSlot_t IO_ExtiSlots[16];

static const IRQn_Type IO_EXTI_IRQN[16] =
{
  EXTI0_IRQn,  EXTI1_IRQn,  EXTI2_IRQn,  EXTI3_IRQn,
  EXTI4_IRQn,  EXTI5_IRQn,  EXTI6_IRQn,  EXTI7_IRQn,
  EXTI8_IRQn,  EXTI9_IRQn,  EXTI10_IRQn, EXTI11_IRQn,
  EXTI12_IRQn, EXTI13_IRQn, EXTI14_IRQn, EXTI15_IRQn,
};

/* One bit per pin configured here, for BSP_IO_DeInit. */
static uint32_t IO_ConfiguredPins;

/* 1 once the cycle counter has been seen to run. */
static uint8_t IO_CycleCounterReady;

static const BSP_IO_PinInfo_t *IO_GetPin(uint32_t Pin);
static void                    IO_EnablePortClock(const GPIO_TypeDef *Port);
static uint32_t                IO_CurrentPull(const BSP_IO_PinInfo_t *pPin);
static int32_t                 IO_EnableCycleCounter(void);

/* Table row, or NULL if out of range or not routed. */
static const BSP_IO_PinInfo_t *IO_GetPin(uint32_t Pin)
{
  const BSP_IO_PinInfo_t *entry;

  if (Pin >= BSP_IO_PIN_NBR)
  {
    return NULL;
  }

  entry = &IO_PIN_TABLE[Pin];

  return (entry->Port == NULL) ? NULL : entry;
}

/* The enable macros write a set-only register: idempotent and race-free. */
static void IO_EnablePortClock(const GPIO_TypeDef *Port)
{
  if (Port == GPIOA)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
  }
  else if (Port == GPIOB)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
  }
  else if (Port == GPIOC)
  {
    __HAL_RCC_GPIOC_CLK_ENABLE();
  }
  else if (Port == GPIOD)
  {
    __HAL_RCC_GPIOD_CLK_ENABLE();
  }
  else if (Port == GPIOE)
  {
    __HAL_RCC_GPIOE_CLK_ENABLE();
  }
  else if (Port == GPIOF)
  {
    __HAL_RCC_GPIOF_CLK_ENABLE();
  }
  else if (Port == GPIOG)
  {
    __HAL_RCC_GPIOG_CLK_ENABLE();
  }
  else if (Port == GPIOH)
  {
    __HAL_RCC_GPIOH_CLK_ENABLE();
  }
  else
  {
    /* No other port carries a header pin. */
  }
}

/* The pin's current pull, kept when it is re-initialised for another purpose. */
static uint32_t IO_CurrentPull(const BSP_IO_PinInfo_t *pPin)
{
  uint32_t pupdr = (pPin->Port->PUPDR >> (2U * (uint32_t)pPin->Line)) & 0x3U;

  if (pupdr == 1U)
  {
    return GPIO_PULLUP;
  }

  if (pupdr == 2U)
  {
    return GPIO_PULLDOWN;
  }

  return GPIO_NOPULL;
}

/* Start the DWT cycle counter, which reset and the boot loader leave off. */
static int32_t IO_EnableCycleCounter(void)
{
  uint32_t first;
  uint32_t second;

  if (IO_CycleCounterReady != 0U)
  {
    return BSP_ERROR_NONE;
  }

  /* Never zeroed: the FreeRTOS HAL_GetTick extrapolates from the running count. */
  DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  first  = DWT->CYCCNT;
  second = DWT->CYCCNT;

  if (first == second)
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  IO_CycleCounterReady = 1U;

  return BSP_ERROR_NONE;
}

int32_t BSP_IO_Init(void)
{
  (void)IO_EnableCycleCounter();

  return BSP_ERROR_NONE;
}

int32_t BSP_IO_DeInit(void)
{
  uint32_t pin;

  for (pin = 0U; pin < BSP_IO_PIN_NBR; pin++)
  {
    const BSP_IO_PinInfo_t *entry = IO_GetPin(pin);

    if ((entry != NULL) && ((IO_ConfiguredPins & (1UL << pin)) != 0U))
    {
      (void)BSP_IO_DetachInterrupt(pin);
      HAL_GPIO_DeInit(entry->Port, entry->Pin);
    }
  }

  IO_ConfiguredPins = 0U;

  return BSP_ERROR_NONE;
}

int32_t BSP_IO_ConfigPin(uint32_t Pin, uint32_t Mode)
{
  const BSP_IO_PinInfo_t *entry = IO_GetPin(Pin);
  GPIO_InitTypeDef gpio_init_structure = {0};

  if (entry == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  switch (Mode)
  {
    case BSP_IO_MODE_INPUT:
      gpio_init_structure.Mode = GPIO_MODE_INPUT;
      gpio_init_structure.Pull = GPIO_NOPULL;
      break;

    case BSP_IO_MODE_INPUT_PU:
      gpio_init_structure.Mode = GPIO_MODE_INPUT;
      gpio_init_structure.Pull = GPIO_PULLUP;
      break;

    case BSP_IO_MODE_INPUT_PD:
      gpio_init_structure.Mode = GPIO_MODE_INPUT;
      gpio_init_structure.Pull = GPIO_PULLDOWN;
      break;

    case BSP_IO_MODE_OUTPUT_PP:
      gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
      gpio_init_structure.Pull = GPIO_NOPULL;
      break;

    case BSP_IO_MODE_OUTPUT_OD:
      gpio_init_structure.Mode = GPIO_MODE_OUTPUT_OD;
      gpio_init_structure.Pull = GPIO_NOPULL;
      break;

    case BSP_IO_MODE_ANALOG:
      gpio_init_structure.Mode = GPIO_MODE_ANALOG;
      gpio_init_structure.Pull = GPIO_NOPULL;
      break;

    default:
      return BSP_ERROR_WRONG_PARAM;
  }

  IO_EnablePortClock(entry->Port);

  /* Low slew rate: the header runs alongside the camera lanes. SPI2 and I2C2, which need
   * fast edges, configure their own pads. */
  gpio_init_structure.Pin   = entry->Pin;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(entry->Port, &gpio_init_structure);

  IO_ConfiguredPins |= (1UL << Pin);

  return BSP_ERROR_NONE;
}

int32_t BSP_IO_WritePin(uint32_t Pin, uint32_t PinState)
{
  const BSP_IO_PinInfo_t *entry = IO_GetPin(Pin);

  if (entry == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* One atomic BSRR store: the low half sets, the high half resets. */
  entry->Port->BSRR = (PinState != BSP_IO_PIN_RESET)
                      ? (uint32_t)entry->Pin
                      : ((uint32_t)entry->Pin << 16);

  return BSP_ERROR_NONE;
}

int32_t BSP_IO_ReadPin(uint32_t Pin)
{
  const BSP_IO_PinInfo_t *entry = IO_GetPin(Pin);

  if (entry == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  return ((entry->Port->IDR & (uint32_t)entry->Pin) != 0U)
         ? (int32_t)BSP_IO_PIN_SET
         : (int32_t)BSP_IO_PIN_RESET;
}

int32_t BSP_IO_TogglePin(uint32_t Pin)
{
  const BSP_IO_PinInfo_t *entry = IO_GetPin(Pin);
  uint32_t odr;

  if (entry == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  odr = entry->Port->ODR;
  entry->Port->BSRR = ((odr & (uint32_t)entry->Pin) << 16) |
                      (~odr & (uint32_t)entry->Pin);

  return BSP_ERROR_NONE;
}

int32_t BSP_IO_GetPinInfo(uint32_t Pin, BSP_IO_PinInfo_t *pInfo)
{
  const BSP_IO_PinInfo_t *entry = IO_GetPin(Pin);

  if ((entry == NULL) || (pInfo == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  *pInfo = *entry;

  return BSP_ERROR_NONE;
}

int32_t BSP_IO_HasPwm(uint32_t Pin)
{
  const BSP_IO_PinInfo_t *entry = IO_GetPin(Pin);

  if (entry == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  return (entry->PwmTimer != NULL) ? 1 : 0;
}

int32_t BSP_IO_HasAdc(uint32_t Pin)
{
  const BSP_IO_PinInfo_t *entry = IO_GetPin(Pin);

  if (entry == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  return (entry->AdcInstance != 0U) ? 1 : 0;
}

int32_t BSP_IO_AttachInterrupt(uint32_t Pin, uint32_t Trigger, BSP_IO_Callback_t Callback)
{
  const BSP_IO_PinInfo_t *entry = IO_GetPin(Pin);
  GPIO_InitTypeDef gpio_init_structure = {0};
  uint32_t line;

  if ((entry == NULL) || (Callback == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  switch (Trigger)
  {
    case BSP_IO_TRIGGER_RISING:
      gpio_init_structure.Mode = GPIO_MODE_IT_RISING;
      break;

    case BSP_IO_TRIGGER_FALLING:
      gpio_init_structure.Mode = GPIO_MODE_IT_FALLING;
      break;

    case BSP_IO_TRIGGER_BOTH:
      gpio_init_structure.Mode = GPIO_MODE_IT_RISING_FALLING;
      break;

    default:
      return BSP_ERROR_WRONG_PARAM;
  }

  line = (uint32_t)entry->Line;

  /* One line per pin index: line 8 is PA8 or PG8 or another port's, never two at once. */
  if ((IO_ExtiSlots[line].Callback != NULL) && (IO_ExtiSlots[line].Pin != Pin))
  {
    return BSP_ERROR_BUSY;
  }

  /* Likewise a line armed outside this module for another port. */
  if ((IO_ExtiSlots[line].Callback == NULL) && ((EXTI->IMR1 & (1UL << line)) != 0U) &&
      (((EXTI->EXTICR[line >> 2U] >> ((line & 0x03U) * EXTI_EXTICR1_EXTI1_Pos)) & 0x0FUL) !=
       GPIO_GET_INDEX(entry->Port)))
  {
    return BSP_ERROR_BUSY;
  }

  IO_EnablePortClock(entry->Port);

  gpio_init_structure.Pin   = entry->Pin;
  gpio_init_structure.Pull  = IO_CurrentPull(entry);
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;

  /* Publish the slot before the line can fire. */
  IO_ExtiSlots[line].Pin      = Pin;
  IO_ExtiSlots[line].Callback = Callback;

  HAL_GPIO_Init(entry->Port, &gpio_init_structure);

  /* Drop an edge latched before this attachment. */
  EXTI->RPR1 = 1UL << line;
  EXTI->FPR1 = 1UL << line;
  HAL_NVIC_ClearPendingIRQ(IO_EXTI_IRQN[line]);

  IO_ConfiguredPins |= (1UL << Pin);

  HAL_NVIC_SetPriority(IO_EXTI_IRQN[line], BSP_IO_IT_PRIORITY, 0x00U);
  HAL_NVIC_EnableIRQ(IO_EXTI_IRQN[line]);

  return BSP_ERROR_NONE;
}

int32_t BSP_IO_DetachInterrupt(uint32_t Pin)
{
  const BSP_IO_PinInfo_t *entry = IO_GetPin(Pin);
  GPIO_InitTypeDef gpio_init_structure = {0};
  uint32_t line;
  uint32_t mask;

  if (entry == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  line = (uint32_t)entry->Line;

  if ((IO_ExtiSlots[line].Callback == NULL) || (IO_ExtiSlots[line].Pin != Pin))
  {
    return BSP_ERROR_NO_INIT;
  }

  HAL_NVIC_DisableIRQ(IO_EXTI_IRQN[line]);
  IO_ExtiSlots[line].Callback = NULL;

  /* Back to a plain input, keeping the pull. */
  gpio_init_structure.Pin   = entry->Pin;
  gpio_init_structure.Mode  = GPIO_MODE_INPUT;
  gpio_init_structure.Pull  = IO_CurrentPull(entry);
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(entry->Port, &gpio_init_structure);

  /* HAL_GPIO_Init leaves the EXTI alone for an input: mask it and drop latched edges. */
  mask = 1UL << line;
  EXTI->IMR1  &= ~mask;
  EXTI->EMR1  &= ~mask;
  EXTI->RTSR1 &= ~mask;
  EXTI->FTSR1 &= ~mask;
  EXTI->RPR1   = mask;
  EXTI->FPR1   = mask;
  HAL_NVIC_ClearPendingIRQ(IO_EXTI_IRQN[line]);

  return BSP_ERROR_NONE;
}

void BSP_IO_IRQHandler(uint32_t Line)
{
  uint32_t mask;
  BSP_IO_Callback_t callback;

  if (Line >= 16U)
  {
    return;
  }

  mask = 1UL << Line;

  if (((EXTI->RPR1 & mask) | (EXTI->FPR1 & mask)) == 0U)
  {
    return;
  }

  /* Write-one-to-clear, harmless on a clear bit: this clears whichever edge fired. */
  EXTI->RPR1 = mask;
  EXTI->FPR1 = mask;

  callback = IO_ExtiSlots[Line].Callback;

  if (callback != NULL)
  {
    callback(IO_ExtiSlots[Line].Pin);
  }
}

int32_t BSP_IO_ReadPulse(uint32_t Pin, uint32_t State, uint32_t Timeout_us, uint32_t *pWidth_us)
{
  const BSP_IO_PinInfo_t *entry = IO_GetPin(Pin);
  uint32_t cycles_per_us;
  uint32_t timeout_cycles;
  uint32_t wanted;
  uint32_t start;
  uint32_t pulse_start;
  int32_t  status;

  if ((entry == NULL) || (pWidth_us == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  status = IO_EnableCycleCounter();

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (Timeout_us > BSP_IO_PULSE_MAX_TIMEOUT_US)
  {
    Timeout_us = BSP_IO_PULSE_MAX_TIMEOUT_US;
  }

  /* Read the CPU clock rather than assume it: the boot loader owns the clock tree. */
  cycles_per_us = HAL_RCC_GetCpuClockFreq() / 1000000U;

  if (cycles_per_us == 0U)
  {
    return BSP_ERROR_CLOCK_FAILURE;
  }

  timeout_cycles = Timeout_us * cycles_per_us;
  wanted = (State != BSP_IO_PIN_RESET) ? (uint32_t)entry->Pin : 0U;
  start = DWT->CYCCNT;

  /* Let any pulse already in progress finish, so the reading is a whole one. */
  while ((entry->Port->IDR & (uint32_t)entry->Pin) == wanted)
  {
    if ((DWT->CYCCNT - start) > timeout_cycles)
    {
      return BSP_ERROR_BUSY;
    }
  }

  /* Wait for the leading edge. */
  while ((entry->Port->IDR & (uint32_t)entry->Pin) != wanted)
  {
    if ((DWT->CYCCNT - start) > timeout_cycles)
    {
      return BSP_ERROR_BUSY;
    }
  }

  pulse_start = DWT->CYCCNT;

  /* Wait for the trailing edge. */
  while ((entry->Port->IDR & (uint32_t)entry->Pin) == wanted)
  {
    if ((DWT->CYCCNT - start) > timeout_cycles)
    {
      return BSP_ERROR_BUSY;
    }
  }

  *pWidth_us = (DWT->CYCCNT - pulse_start) / cycles_per_us;

  return BSP_ERROR_NONE;
}

int32_t BSP_IO_ShiftOut(uint32_t DataPin, uint32_t ClockPin, uint32_t BitOrder, uint8_t Value)
{
  uint32_t bit;

  if ((IO_GetPin(DataPin) == NULL) || (IO_GetPin(ClockPin) == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  for (bit = 0U; bit < 8U; bit++)
  {
    uint32_t shift = (BitOrder == BSP_IO_LSB_FIRST) ? bit : (7U - bit);
    uint32_t level = ((Value & (1U << shift)) != 0U) ? BSP_IO_PIN_SET : BSP_IO_PIN_RESET;

    (void)BSP_IO_WritePin(DataPin, level);
    (void)BSP_IO_WritePin(ClockPin, BSP_IO_PIN_SET);
    (void)BSP_IO_WritePin(ClockPin, BSP_IO_PIN_RESET);
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_IO_ShiftIn(uint32_t DataPin, uint32_t ClockPin, uint32_t BitOrder, uint8_t *pValue)
{
  uint32_t bit;
  uint8_t  value = 0U;

  if ((IO_GetPin(DataPin) == NULL) || (IO_GetPin(ClockPin) == NULL) || (pValue == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  for (bit = 0U; bit < 8U; bit++)
  {
    (void)BSP_IO_WritePin(ClockPin, BSP_IO_PIN_SET);

    if (BSP_IO_ReadPin(DataPin) == (int32_t)BSP_IO_PIN_SET)
    {
      value |= (uint8_t)(1U << ((BitOrder == BSP_IO_LSB_FIRST) ? bit : (7U - bit)));
    }

    (void)BSP_IO_WritePin(ClockPin, BSP_IO_PIN_RESET);
  }

  *pValue = value;

  return BSP_ERROR_NONE;
}
