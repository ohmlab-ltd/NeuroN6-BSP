/**
  * @file    neuron6_pwm.c
  * @brief   PWM on the Neuro N6's timer pins, and a tone generator on any pin.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6_pwm.h"

/* Timers this module drives: TIM1, TIM2, TIM15, TIM17. */
#define PWM_TIMER_NBR               4U

/* Timer-clock measurement window in us: short, as three of the four timers are 16-bit. */
#define PWM_MEASURE_WINDOW_US       50U

typedef struct
{
  TIM_HandleTypeDef Handle;      /*!< HAL handle, valid once Configured.     */
  uint32_t          ClockHz;     /*!< Measured kernel clock, 0 if unmeasured. */
  uint32_t          Frequency;   /*!< Frequency the period was built for.     */
  uint32_t          Duty[5];     /*!< Duty each channel 1..4 was started at.  */
  uint32_t          Scale[5];    /*!< Full scale that duty was given against. */
  uint16_t          ActiveMask;  /*!< Bit per timer channel currently driven. */
  uint8_t           Configured;  /*!< 1 once the time base is programmed.     */
} PWM_TimerState_t;

/* In TIM1, TIM2, TIM15, TIM17 order. */
static PWM_TimerState_t PWM_Timers[PWM_TIMER_NBR];

/* One bit per pin number being driven. */
static uint32_t PWM_ActivePins;

static uint32_t PWM_Frequency = BSP_PWM_DEFAULT_FREQUENCY;

static uint32_t PWM_Resolution = BSP_PWM_DEFAULT_RESOLUTION;

/* Measured tone-timer clock, 0 until measured. */
static uint32_t PWM_ToneClockHz;

/* BSP_IO_PIN_NONE when idle. */
static volatile uint32_t PWM_TonePin = BSP_IO_PIN_NONE;

/* Half-cycles left; 0 plays until stopped. */
static volatile uint32_t PWM_ToneHalfCyclesLeft;

/* 1 once the cycle counter has been seen to run. */
static uint8_t PWM_CycleCounterReady;

/* Timers this module has clocked, to measure, drive or play: TIM1, TIM2, TIM3, TIM15 and
 * TIM17 as bits 0 to 4. DeInit releases them. */
static uint8_t PWM_ClockedMask;

static int32_t           PWM_EnableCycleCounter(void);
static void              PWM_EnableTimerClock(const TIM_TypeDef *Timer);
static void              PWM_DisableTimerClock(const TIM_TypeDef *Timer);
static uint8_t           PWM_ClockBit(const TIM_TypeDef *Timer);
static void              PWM_RoutePad(const BSP_IO_PinInfo_t *pInfo);
static PWM_TimerState_t *PWM_StateFor(const TIM_TypeDef *Timer);
static uint32_t          PWM_MeasureTimerClock(TIM_TypeDef *Timer);
static int32_t           PWM_TimerClockOf(TIM_TypeDef *Timer, uint32_t *pHz);
static uint32_t          PWM_HalChannel(uint8_t Channel);
static uint32_t          PWM_IdleLevel(uint32_t Pin);
static uint32_t          PWM_Compare(const PWM_TimerState_t *pState, uint32_t Duty,
                                     uint32_t Scale);

/* Start the DWT cycle counter, the reference for timer-clock measurement. */
static int32_t PWM_EnableCycleCounter(void)
{
  uint32_t first;
  uint32_t second;

  if (PWM_CycleCounterReady != 0U)
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

  PWM_CycleCounterReady = 1U;

  return BSP_ERROR_NONE;
}

/* The timer's PWM_ClockedMask bit, or 0. */
static uint8_t PWM_ClockBit(const TIM_TypeDef *Timer)
{
  if (Timer == TIM1)  { return 0x01U; }
  if (Timer == TIM2)  { return 0x02U; }
  if (Timer == TIM3)  { return 0x04U; }
  if (Timer == TIM15) { return 0x08U; }
  if (Timer == TIM17) { return 0x10U; }
  return 0U;
}

static void PWM_EnableTimerClock(const TIM_TypeDef *Timer)
{
  PWM_ClockedMask |= PWM_ClockBit(Timer);

  if (Timer == TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
  }
  else if (Timer == TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
  }
  else if (Timer == TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
  else if (Timer == TIM15)
  {
    __HAL_RCC_TIM15_CLK_ENABLE();
  }
  else if (Timer == TIM17)
  {
    __HAL_RCC_TIM17_CLK_ENABLE();
  }
  else
  {
    /* No other timer reaches a pin this module drives. */
  }
}

/* Read back before gating the clock, or a posted write stalls the bus. */
static void PWM_DisableTimerClock(const TIM_TypeDef *Timer)
{
  if ((PWM_ClockedMask & PWM_ClockBit(Timer)) == 0U)
  {
    return;
  }
  (void)READ_REG(Timer->CR1);
  __DSB();
  PWM_ClockedMask &= (uint8_t)~PWM_ClockBit(Timer);

  if (Timer == TIM1)
  {
    __HAL_RCC_TIM1_CLK_DISABLE();
  }
  else if (Timer == TIM2)
  {
    __HAL_RCC_TIM2_CLK_DISABLE();
  }
  else if (Timer == TIM3)
  {
    __HAL_RCC_TIM3_CLK_DISABLE();
  }
  else if (Timer == TIM15)
  {
    __HAL_RCC_TIM15_CLK_DISABLE();
  }
  else if (Timer == TIM17)
  {
    __HAL_RCC_TIM17_CLK_DISABLE();
  }
  else
  {
    /* Nothing to do. */
  }
}

/* State of a modulator timer, or NULL for any other timer. */
static PWM_TimerState_t *PWM_StateFor(const TIM_TypeDef *Timer)
{
  if (Timer == TIM1)
  {
    return &PWM_Timers[0];
  }

  if (Timer == TIM2)
  {
    return &PWM_Timers[1];
  }

  if (Timer == TIM15)
  {
    return &PWM_Timers[2];
  }

  if (Timer == TIM17)
  {
    return &PWM_Timers[3];
  }

  return NULL;
}

/* Count the timer, which must be driving nothing, against the cycle counter for a fixed
 * window, then restore its CR1. Returns Hz, or 0. */
static uint32_t PWM_MeasureTimerClock(TIM_TypeDef *Timer)
{
  uint32_t saved_cr1;
  uint32_t cycles_per_us;
  uint32_t window_cycles;
  uint32_t start;
  uint32_t first_count;
  uint32_t elapsed_cycles;
  uint32_t ticks;
  uint32_t primask;

  cycles_per_us = HAL_RCC_GetCpuClockFreq() / 1000000U;

  if (cycles_per_us == 0U)
  {
    return 0U;
  }

  PWM_EnableTimerClock(Timer);

  saved_cr1  = Timer->CR1;
  Timer->CR1 = 0U;
  Timer->PSC = 0U;
  Timer->ARR = 0xFFFFFFFFU;  /* A sixteen-bit timer keeps the low half. */
  Timer->EGR = TIM_EGR_UG;   /* Latch the prescaler. */
  Timer->CNT = 0U;
  Timer->CR1 = TIM_CR1_CEN;

  window_cycles = PWM_MEASURE_WINDOW_US * cycles_per_us;

  /* Interrupts off: one between the reads would skew a result that is kept. */
  primask = __get_PRIMASK();
  __disable_irq();

  start = DWT->CYCCNT;
  first_count = Timer->CNT;

  while ((DWT->CYCCNT - start) < window_cycles)
  {
    /* Busy wait: the window is fifty microseconds. */
  }

  elapsed_cycles = DWT->CYCCNT - start;
  /* A 16-bit mask suits both counter widths; the window is too short to wrap. */
  ticks = (Timer->CNT - first_count) & 0xFFFFU;

  __set_PRIMASK(primask);

  Timer->CR1 = saved_cr1;

  if ((elapsed_cycles == 0U) || (ticks == 0U))
  {
    return 0U;
  }

  return (uint32_t)(((uint64_t)ticks * (uint64_t)HAL_RCC_GetCpuClockFreq()) /
                    (uint64_t)elapsed_cycles);
}

/* A timer's kernel clock, measured on first use. */
static int32_t PWM_TimerClockOf(TIM_TypeDef *Timer, uint32_t *pHz)
{
  PWM_TimerState_t *state;
  uint32_t *cache;
  int32_t   status;

  status = PWM_EnableCycleCounter();

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  state = PWM_StateFor(Timer);

  if (state != NULL)
  {
    cache = &state->ClockHz;
  }
  else if (Timer == BSP_TONE_TIMER)
  {
    cache = &PWM_ToneClockHz;
  }
  else
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (*cache == 0U)
  {
    *cache = PWM_MeasureTimerClock(Timer);
  }

  if (*cache == 0U)
  {
    return BSP_ERROR_CLOCK_FAILURE;
  }

  *pHz = *cache;

  return BSP_ERROR_NONE;
}

static void PWM_RoutePad(const BSP_IO_PinInfo_t *pInfo)
{
  GPIO_InitTypeDef gpio_init_structure = {0};

  gpio_init_structure.Pin       = pInfo->Pin;
  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull      = GPIO_NOPULL;
  gpio_init_structure.Speed     = GPIO_SPEED_FREQ_LOW;
  gpio_init_structure.Alternate = pInfo->PwmAlternate;
  HAL_GPIO_Init(pInfo->Port, &gpio_init_structure);
}

/* HAL encoding of channel 1..4; TIM_CHANNEL_1 if out of range. */
static uint32_t PWM_HalChannel(uint8_t Channel)
{
  switch (Channel)
  {
    case 2U:
      return TIM_CHANNEL_2;

    case 3U:
      return TIM_CHANNEL_3;

    case 4U:
      return TIM_CHANNEL_4;

    default:
      return TIM_CHANNEL_1;
  }
}

/* Rest level: high (off) for the active-low RGB LED, low for everything else. */
static uint32_t PWM_IdleLevel(uint32_t Pin)
{
  return ((Pin == BSP_IO_PIN_LED_RED) || (Pin == BSP_IO_PIN_LED_GREEN) ||
          (Pin == BSP_IO_PIN_LED_BLUE)) ? BSP_IO_PIN_SET : BSP_IO_PIN_RESET;
}

/* Compare value for Duty out of Scale over the timer's period. */
static uint32_t PWM_Compare(const PWM_TimerState_t *pState, uint32_t Duty, uint32_t Scale)
{
  return (uint32_t)(((uint64_t)Duty * ((uint64_t)pState->Handle.Init.Period + 1U)) /
                    (uint64_t)Scale);
}

int32_t BSP_PWM_Init(void)
{
  return PWM_EnableCycleCounter();
}

int32_t BSP_PWM_DeInit(void)
{
  uint32_t pin;
  uint32_t index;

  (void)BSP_TONE_Stop();

  for (pin = 0U; pin < BSP_IO_PIN_NBR; pin++)
  {
    if ((PWM_ActivePins & (1UL << pin)) != 0U)
    {
      (void)BSP_PWM_Stop(pin);
    }
  }

  for (index = 0U; index < PWM_TIMER_NBR; index++)
  {
    if (PWM_Timers[index].Configured != 0U)
    {
      (void)HAL_TIM_PWM_DeInit(&PWM_Timers[index].Handle);
      PWM_Timers[index].Configured = 0U;
      PWM_Timers[index].Frequency  = 0U;
    }
  }

  PWM_DisableTimerClock(TIM1);
  PWM_DisableTimerClock(TIM2);
  PWM_DisableTimerClock(TIM3);
  PWM_DisableTimerClock(TIM15);
  PWM_DisableTimerClock(TIM17);

  return BSP_ERROR_NONE;
}

int32_t BSP_PWM_SetFrequency(uint32_t Frequency)
{
  if ((Frequency < BSP_PWM_MIN_FREQUENCY) || (Frequency > BSP_PWM_MAX_FREQUENCY))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Each timer rebuilds its time base at its next start; its measured clock is kept. */
  PWM_Frequency = Frequency;

  return BSP_ERROR_NONE;
}

uint32_t BSP_PWM_GetFrequency(void)
{
  return PWM_Frequency;
}

int32_t BSP_PWM_SetResolution(uint32_t Bits)
{
  if ((Bits < BSP_PWM_MIN_RESOLUTION) || (Bits > BSP_PWM_MAX_RESOLUTION))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  PWM_Resolution = Bits;

  return BSP_ERROR_NONE;
}

uint32_t BSP_PWM_GetResolution(void)
{
  return PWM_Resolution;
}

int32_t BSP_PWM_Start(uint32_t Pin, uint32_t DutyCycle)
{
  BSP_IO_PinInfo_t info;
  PWM_TimerState_t *state;
  TIM_OC_InitTypeDef oc_init_structure = {0};
  uint32_t clock_hz = 0U;
  uint32_t full_scale;
  uint32_t channel;
  uint32_t compare;
  int32_t  status;

  status = BSP_IO_GetPinInfo(Pin, &info);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (info.PwmTimer == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  state = PWM_StateFor(info.PwmTimer);

  if (state == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  status = PWM_TimerClockOf(info.PwmTimer, &clock_hz);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  full_scale = 1UL << PWM_Resolution;

  if (DutyCycle > (full_scale - 1U))
  {
    DutyCycle = full_scale - 1U;
  }

  if ((state->Configured == 0U) || (state->Frequency != PWM_Frequency))
  {
    uint32_t total_ticks = clock_hz / PWM_Frequency;
    uint32_t prescaler = 0U;
    uint32_t other;

    if (total_ticks == 0U)
    {
      return BSP_ERROR_WRONG_PARAM;
    }

    /* Fit the period in 16 bits, as three of the four timers count 16 bits. */
    while ((total_ticks / (prescaler + 1U)) > 0xFFFFU)
    {
      prescaler++;
    }

    PWM_EnableTimerClock(info.PwmTimer);

    state->Handle.Instance               = info.PwmTimer;
    state->Handle.Init.Prescaler         = prescaler;
    state->Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    state->Handle.Init.Period            = (total_ticks / (prescaler + 1U)) - 1U;
    state->Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    state->Handle.Init.RepetitionCounter = 0U;
    state->Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    /* Running channels keep their duty over the new period: their preloaded compares take
     * effect with it, at the update the initialisation generates. */
    for (other = 1U; other <= 4U; other++)
    {
      if ((state->ActiveMask & (1U << other)) != 0U)
      {
        __HAL_TIM_SET_COMPARE(&state->Handle, PWM_HalChannel((uint8_t)other),
                              (other == info.PwmChannel)
                              ? PWM_Compare(state, DutyCycle, full_scale)
                              : PWM_Compare(state, state->Duty[other], state->Scale[other]));
      }
    }

    if (HAL_TIM_PWM_Init(&state->Handle) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }

    state->Configured = 1U;
    state->Frequency  = PWM_Frequency;
  }

  channel = PWM_HalChannel(info.PwmChannel);
  compare = PWM_Compare(state, DutyCycle, full_scale);
  state->Duty[info.PwmChannel]  = DutyCycle;
  state->Scale[info.PwmChannel] = full_scale;

  /* Running: only a new compare (the HAL refuses to start a busy channel). The pad goes
   * back to the timer, as a tone, pin mode or conversion may have taken it. */
  if ((state->ActiveMask & (1U << info.PwmChannel)) != 0U)
  {
    __HAL_TIM_SET_COMPARE(&state->Handle, channel, compare);
    PWM_RoutePad(&info);
    return BSP_ERROR_NONE;
  }

  oc_init_structure.OCMode       = TIM_OCMODE_PWM1;
  oc_init_structure.Pulse        = compare;
  oc_init_structure.OCPolarity   = TIM_OCPOLARITY_HIGH;
  oc_init_structure.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  oc_init_structure.OCFastMode   = TIM_OCFAST_DISABLE;
  oc_init_structure.OCIdleState  = TIM_OCIDLESTATE_RESET;
  oc_init_structure.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(&state->Handle, &oc_init_structure, channel) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* Route the pad only once the compare is set, so no burst at the previous duty. */
  PWM_RoutePad(&info);

  if (HAL_TIM_PWM_Start(&state->Handle, channel) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  state->ActiveMask |= (uint16_t)(1U << info.PwmChannel);
  PWM_ActivePins |= (1UL << Pin);

  return BSP_ERROR_NONE;
}

int32_t BSP_PWM_Stop(uint32_t Pin)
{
  BSP_IO_PinInfo_t info;
  PWM_TimerState_t *state;
  int32_t status;

  status = BSP_IO_GetPinInfo(Pin, &info);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (info.PwmTimer == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if ((PWM_ActivePins & (1UL << Pin)) == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  state = PWM_StateFor(info.PwmTimer);

  if (state != NULL)
  {
    (void)HAL_TIM_PWM_Stop(&state->Handle, PWM_HalChannel(info.PwmChannel));
    state->ActiveMask &= (uint16_t)(~(1U << info.PwmChannel));
  }

  PWM_ActivePins &= ~(1UL << Pin);

  (void)BSP_IO_WritePin(Pin, PWM_IdleLevel(Pin));

  return BSP_IO_ConfigPin(Pin, BSP_IO_MODE_OUTPUT_PP);
}

int32_t BSP_PWM_GetTimerClockFreq(uint32_t Pin, uint32_t *pHz)
{
  BSP_IO_PinInfo_t info;
  int32_t status;

  if (pHz == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  status = BSP_IO_GetPinInfo(Pin, &info);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (info.PwmTimer == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  return PWM_TimerClockOf(info.PwmTimer, pHz);
}

int32_t BSP_TONE_Start(uint32_t Pin, uint32_t Frequency, uint32_t Duration_ms)
{
  BSP_IO_PinInfo_t info;
  uint32_t clock_hz = 0U;
  uint32_t half_period_ticks;
  uint32_t prescaler = 0U;
  int32_t  status;

  if (Frequency == 0U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  status = BSP_IO_GetPinInfo(Pin, &info);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  /* One generator, one pin: refuse rather than move the tone. */
  if ((PWM_TonePin != BSP_IO_PIN_NONE) && (PWM_TonePin != Pin))
  {
    return BSP_ERROR_BUSY;
  }

  status = PWM_TimerClockOf(BSP_TONE_TIMER, &clock_hz);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  half_period_ticks = clock_hz / (2U * Frequency);

  if (half_period_ticks == 0U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  while ((half_period_ticks / (prescaler + 1U)) > 0xFFFFU)
  {
    prescaler++;
  }

  status = BSP_IO_ConfigPin(Pin, BSP_IO_MODE_OUTPUT_PP);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  PWM_EnableTimerClock(BSP_TONE_TIMER);

  /* Stop first, so the last interrupt of a replaced tone cannot end this one. */
  BSP_TONE_TIMER->CR1  = 0U;
  BSP_TONE_TIMER->DIER = 0U;
  HAL_NVIC_ClearPendingIRQ(BSP_TONE_TIMER_IRQn);

  PWM_TonePin = Pin;
  /* Rounded up: a very short tone plays one half-cycle, as 0 would mean until stopped. */
  PWM_ToneHalfCyclesLeft = (Duration_ms != 0U)
                           ? (uint32_t)((((uint64_t)Duration_ms * Frequency * 2U) + 999U) / 1000U)
                           : 0U;

  /* Direct registers: only the time base and update interrupt are used, so the tone
   * is not tied to the five pins with a compare output. */
  BSP_TONE_TIMER->CR1  = 0U;
  BSP_TONE_TIMER->PSC  = prescaler;
  BSP_TONE_TIMER->ARR  = (half_period_ticks / (prescaler + 1U)) - 1U;
  BSP_TONE_TIMER->CNT  = 0U;
  BSP_TONE_TIMER->EGR  = TIM_EGR_UG;
  BSP_TONE_TIMER->SR   = 0U;
  BSP_TONE_TIMER->DIER = TIM_DIER_UIE;

  HAL_NVIC_SetPriority(BSP_TONE_TIMER_IRQn, BSP_TONE_IT_PRIORITY, 0x00U);
  HAL_NVIC_EnableIRQ(BSP_TONE_TIMER_IRQn);

  BSP_TONE_TIMER->CR1 = TIM_CR1_CEN;

  return BSP_ERROR_NONE;
}

int32_t BSP_TONE_Stop(void)
{
  uint32_t pin;

  /* A timer that was never clocked has nothing to stop. */
  if ((PWM_ClockedMask & PWM_ClockBit(BSP_TONE_TIMER)) != 0U)
  {
    BSP_TONE_TIMER->CR1  = 0U;
    BSP_TONE_TIMER->DIER = 0U;
  }
  HAL_NVIC_DisableIRQ(BSP_TONE_TIMER_IRQn);

  pin = PWM_TonePin;
  PWM_TonePin = BSP_IO_PIN_NONE;

  if (pin != BSP_IO_PIN_NONE)
  {
    (void)BSP_IO_WritePin(pin, PWM_IdleLevel(pin));
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_TONE_IsActive(void)
{
  return (PWM_TonePin != BSP_IO_PIN_NONE) ? 1 : 0;
}

void BSP_TONE_IRQHandler(void)
{
  uint32_t pin;
  uint32_t left;

  BSP_TONE_TIMER->SR = 0U;

  pin = PWM_TonePin;

  if (pin == BSP_IO_PIN_NONE)
  {
    return;
  }

  (void)BSP_IO_TogglePin(pin);

  left = PWM_ToneHalfCyclesLeft;

  if (left != 0U)
  {
    left--;

    if (left == 0U)
    {
      BSP_TONE_TIMER->CR1  = 0U;
      BSP_TONE_TIMER->DIER = 0U;
      PWM_TonePin = BSP_IO_PIN_NONE;
      (void)BSP_IO_WritePin(pin, PWM_IdleLevel(pin));
    }

    PWM_ToneHalfCyclesLeft = left;
  }
}
