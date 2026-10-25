/* Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 *
 * The part of the STM32N6 HAL that neuron6_bt.c uses, enough for it to
 * compile and run on a PC. The handle types carry the fields the module sets;
 * the functions are defined by the test, which records every byte the
 * module puts on the link. Nothing here models the UART itself. */
#ifndef FAKE_STM32N6XX_HAL_H
#define FAKE_STM32N6XX_HAL_H

#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;

#define HAL_MAX_DELAY  0xFFFFFFFFU
#define UNUSED(X)      (void)(X)
#define __weak         __attribute__((weak))

/* GPIO. */
typedef struct { uint32_t id; } GPIO_TypeDef;
extern GPIO_TypeDef fake_gpio[8];
#define GPIOC (&fake_gpio[2])
#define GPIOD (&fake_gpio[3])

typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef struct
{
  uint32_t Pin;
  uint32_t Mode;
  uint32_t Pull;
  uint32_t Speed;
  uint32_t Alternate;
} GPIO_InitTypeDef;

#define GPIO_PIN_3              0x0008U
#define GPIO_PIN_4              0x0010U
#define GPIO_PIN_6              0x0040U
#define GPIO_PIN_7              0x0080U
#define GPIO_MODE_AF_PP         2U
#define GPIO_MODE_OUTPUT_PP     1U
#define GPIO_NOPULL             0U
#define GPIO_SPEED_FREQ_LOW     0U
#define GPIO_AF7_USART6         7U

void HAL_GPIO_Init(GPIO_TypeDef *Port, GPIO_InitTypeDef *Init);
void HAL_GPIO_DeInit(GPIO_TypeDef *Port, uint32_t Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *Port, uint16_t Pin, GPIO_PinState State);

/* Clocks and interrupts. */
typedef int IRQn_Type;
#define USART6_IRQn  71

typedef struct { uint32_t ClockSelection; uint32_t ClockDivider; } RCC_ICInitTypeDef;
typedef struct
{
  uint64_t          PeriphClockSelection;
  uint32_t          Usart6ClockSelection;
  RCC_ICInitTypeDef ICSelection[20];
} RCC_PeriphCLKInitTypeDef;

#define RCC_PERIPHCLK_USART6     0x1U
#define RCC_USART6CLKSOURCE_IC9  0x2U
#define RCC_IC9                  8
#define RCC_ICCLKSOURCE_PLL1     0U

HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *Init);
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t Pre, uint32_t Sub);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);

#define __HAL_RCC_USART6_CLK_ENABLE()  do { } while (0)
#define __HAL_RCC_USART6_CLK_DISABLE() do { } while (0)
#define __HAL_RCC_GPIOC_CLK_ENABLE()   do { } while (0)
#define __HAL_RCC_GPIOD_CLK_ENABLE()   do { } while (0)

static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __set_PRIMASK(uint32_t v) { (void)v; }
static inline void __disable_irq(void) { }
static inline void __DSB(void) { }
#define READ_REG(REG)  ((REG))

/* UART. */
typedef struct { volatile uint32_t CR1; } USART_TypeDef;
extern USART_TypeDef fake_usart6;
#define USART6 (&fake_usart6)

typedef struct
{
  uint32_t BaudRate;
  uint32_t WordLength;
  uint32_t StopBits;
  uint32_t Parity;
  uint32_t Mode;
  uint32_t HwFlowCtl;
  uint32_t OverSampling;
  uint32_t OneBitSampling;
  uint32_t ClockPrescaler;
} UART_InitTypeDef;

typedef struct { uint32_t AdvFeatureInit; } UART_AdvFeatureInitTypeDef;

typedef struct __UART_HandleTypeDef
{
  USART_TypeDef             *Instance;
  UART_InitTypeDef           Init;
  UART_AdvFeatureInitTypeDef AdvancedInit;
  uint32_t                   ErrorCode;
} UART_HandleTypeDef;

#define UART_WORDLENGTH_8B           0U
#define UART_STOPBITS_1              0U
#define UART_PARITY_NONE             0U
#define UART_MODE_TX_RX              0xCU
#define UART_HWCONTROL_NONE          0U
#define UART_HWCONTROL_CTS           0x200U
#define UART_OVERSAMPLING_16         0U
#define UART_ONE_BIT_SAMPLE_DISABLE  0U
#define UART_PRESCALER_DIV1          0U
#define UART_ADVFEATURE_NO_INIT      0U
#define UART_TXFIFO_THRESHOLD_1_8    0U
#define UART_RXFIFO_THRESHOLD_1_8    0U
#define UART_TXDATA_FLUSH_REQUEST    0x10U

#define __HAL_UART_SEND_REQ(h, req)  ((void)(h), (void)(req))

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData,
                                    uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UARTEx_SetTxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold);
HAL_StatusTypeDef HAL_UARTEx_SetRxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold);
HAL_StatusTypeDef HAL_UARTEx_DisableFifoMode(UART_HandleTypeDef *huart);
void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);

/* Time. */
uint32_t HAL_GetTick(void);
void     HAL_Delay(uint32_t Delay);

/* SDIO: only the handle type, which neuron6_wifi.h declares. */
typedef struct { uint32_t id; } SDIO_HandleTypeDef;

#endif /* FAKE_STM32N6XX_HAL_H */
