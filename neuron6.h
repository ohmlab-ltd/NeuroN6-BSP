/**
  * @file    neuron6.h
  * @brief   Neuro N6 board core: BSP identity, isolation, LEDs and the user button.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_H
#define NEURON6_H

#include "neuron6_conf.h"
#include "neuron6_errno.h"

/* 1 = the application owns RIF isolation and BSP_ISOLATION_Config() does nothing. */
#ifndef BSP_ISOLATION_APPLICATION_OWNED
#define BSP_ISOLATION_APPLICATION_OWNED 0U
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @brief On-board LEDs: one RGB package, three independently driven cathodes. */
typedef enum
{
  LED1 = 0U,
  LED_RED   = LED1,
  LED2 = 1U,
  LED_GREEN = LED2,
  LED3 = 2U,
  LED_BLUE  = LED3,
  LEDn
} Led_TypeDef;

typedef enum
{
  BUTTON_USER = 0U,
  BUTTONn
} Button_TypeDef;

typedef enum
{
  BUTTON_MODE_GPIO = 0U,
  BUTTON_MODE_EXTI = 1U
} ButtonMode_TypeDef;

#define NEURON6_BSP_VERSION_MAIN   (0x00UL) /*!< [31:24] main version */
#define NEURON6_BSP_VERSION_SUB1   (0x01UL) /*!< [23:16] sub1 version */
#define NEURON6_BSP_VERSION_SUB2   (0x00UL) /*!< [15:8]  sub2 version */
#define NEURON6_BSP_VERSION_RC     (0x00UL) /*!< [7:0]   release candidate */
#define NEURON6_BSP_VERSION        ((NEURON6_BSP_VERSION_MAIN << 24)\
                                     |(NEURON6_BSP_VERSION_SUB1 << 16)\
                                     |(NEURON6_BSP_VERSION_SUB2 << 8 )\
                                     |(NEURON6_BSP_VERSION_RC))

#define NEURON6_BSP_BOARD_NAME     "NEURON6"
#define NEURON6_BSP_BOARD_ID       "OHM-N6"

/* Common anode at 3V3: each colour's GPIO sinks its cathode, so low = lit. */
#define LED1_PIN                         GPIO_PIN_11
#define LED1_GPIO_PORT                   GPIOE
#define LED1_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOE_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOE_CLK_DISABLE()

#define LED2_PIN                         GPIO_PIN_6
#define LED2_GPIO_PORT                   GPIOD
#define LED2_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOD_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOD_CLK_DISABLE()

#define LED3_PIN                         GPIO_PIN_1
#define LED3_GPIO_PORT                   GPIOC
#define LED3_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE()

/* Values reported by BSP_LED_GetState. */
#define LED_DARK                         0U
#define LED_LIT                          1U

/* Values reported by BSP_PB_GetState. */
#define BUTTON_RELEASED                  0U
#define BUTTON_PRESSED                   1U

/* External 10k pulldown: the pin reads LOW at rest and HIGH while the button is held. */
#define BUTTON_USER_PIN                  GPIO_PIN_8
#define BUTTON_USER_GPIO_PORT            GPIOG
#define BUTTON_USER_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOG_CLK_ENABLE()
#define BUTTON_USER_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOG_CLK_DISABLE()
#define BUTTON_USER_EXTI_IRQn            EXTI8_IRQn
#define BUTTON_USER_EXTI_LINE            EXTI_LINE_8

int32_t        BSP_GetVersion  (void);
const uint8_t *BSP_GetBoardName(void);
const uint8_t *BSP_GetBoardID  (void);
int32_t        BSP_GetTick     (void);

int32_t        BSP_ISOLATION_Config(void);

int32_t BSP_LED_Init    (Led_TypeDef Led);
int32_t BSP_LED_DeInit  (Led_TypeDef Led);
int32_t BSP_LED_On      (Led_TypeDef Led);
int32_t BSP_LED_Off     (Led_TypeDef Led);
int32_t BSP_LED_Toggle  (Led_TypeDef Led);
int32_t BSP_LED_GetState(Led_TypeDef Led);

int32_t BSP_PB_Init      (Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
int32_t BSP_PB_DeInit    (Button_TypeDef Button);
int32_t BSP_PB_GetState  (Button_TypeDef Button);
void    BSP_PB_IRQHandler(Button_TypeDef Button);
void    BSP_PB_Callback  (Button_TypeDef Button);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_H */
