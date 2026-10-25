/* Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 *
 * The part of the STM32N6 HAL that neuron6_thermal.c uses, enough for it to
 * compile and run on a PC. Types carry the fields the module sets; the
 * functions are defined by the test, which records what reaches the pins and
 * the bus. Nothing here models the peripherals themselves. */
#ifndef FAKE_STM32N6XX_HAL_H
#define FAKE_STM32N6XX_HAL_H

#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { DISABLE = 0, ENABLE = 1 } FunctionalState;

#define UNUSED(X)  (void)(X)
#define __weak     __attribute__((weak))

/* I2C: only the handle type, which neuron6_bus.h declares. */
typedef struct { uint32_t id; } I2C_HandleTypeDef;

/* GPIO. */
typedef struct { uint32_t id; } GPIO_TypeDef;
extern GPIO_TypeDef fake_gpio[8];
#define GPIOA (&fake_gpio[0])
#define GPIOB (&fake_gpio[1])
#define GPIOC (&fake_gpio[2])
#define GPIOD (&fake_gpio[3])
#define GPIOE (&fake_gpio[4])
#define GPIOF (&fake_gpio[5])
#define GPIOG (&fake_gpio[6])
#define GPIOH (&fake_gpio[7])

typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef struct
{
  uint32_t Pin;
  uint32_t Mode;
  uint32_t Pull;
  uint32_t Speed;
  uint32_t Alternate;
} GPIO_InitTypeDef;

#define GPIO_PIN_0   0x0001U
#define GPIO_PIN_1   0x0002U
#define GPIO_PIN_2   0x0004U
#define GPIO_PIN_3   0x0008U
#define GPIO_PIN_4   0x0010U
#define GPIO_PIN_5   0x0020U
#define GPIO_PIN_6   0x0040U
#define GPIO_PIN_7   0x0080U
#define GPIO_PIN_8   0x0100U
#define GPIO_PIN_9   0x0200U
#define GPIO_PIN_10  0x0400U
#define GPIO_PIN_11  0x0800U
#define GPIO_PIN_12  0x1000U
#define GPIO_PIN_13  0x2000U
#define GPIO_PIN_14  0x4000U
#define GPIO_PIN_15  0x8000U

#define GPIO_MODE_OUTPUT_PP        1U
#define GPIO_MODE_AF_PP            2U
#define GPIO_NOPULL                0U
#define GPIO_SPEED_FREQ_LOW        0U
#define GPIO_SPEED_FREQ_HIGH       2U
#define GPIO_SPEED_FREQ_VERY_HIGH  3U
#define GPIO_AF5_SPI1              5U

void          HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void          HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

/* Clocks. */
#define __HAL_RCC_GPIOA_CLK_ENABLE()   do { } while (0)
#define __HAL_RCC_GPIOB_CLK_ENABLE()   do { } while (0)
#define __HAL_RCC_GPIOC_CLK_ENABLE()   do { } while (0)
#define __HAL_RCC_GPIOD_CLK_ENABLE()   do { } while (0)
#define __HAL_RCC_GPIOE_CLK_ENABLE()   do { } while (0)
#define __HAL_RCC_GPIOF_CLK_ENABLE()   do { } while (0)
#define __HAL_RCC_GPIOG_CLK_ENABLE()   do { } while (0)
#define __HAL_RCC_GPIOH_CLK_ENABLE()   do { } while (0)
#define __HAL_RCC_SPI1_CLK_ENABLE()    do { } while (0)
#define __HAL_RCC_SPI1_CLK_DISABLE()   do { } while (0)
#define __HAL_RCC_GPDMA1_CLK_ENABLE()  do { } while (0)
#define __HAL_RCC_RIFSC_CLK_ENABLE()   do { } while (0)

typedef struct
{
  uint32_t PeriphClockSelection;
  uint32_t Spi1ClockSelection;
} RCC_PeriphCLKInitTypeDef;
#define RCC_PERIPHCLK_SPI1          1U
#define RCC_SPI1CLKSOURCE_PCLK2     1U
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *PeriphClkInit);
uint32_t          HAL_RCC_GetPCLK2Freq(void);

/* Interrupts. */
typedef enum { GPDMA1_Channel1_IRQn = 1, SPI1_IRQn = 2, EXTI8_IRQn = 3 } IRQn_Type;
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);

/* DMA. */
typedef struct { uint32_t id; } DMA_Channel_TypeDef;
extern DMA_Channel_TypeDef fake_dma_channel;
#define GPDMA1_Channel1 (&fake_dma_channel)

typedef struct { uint32_t LinkRegisters[8]; uint32_t NodeInfo; } DMA_NodeTypeDef;
typedef struct { DMA_NodeTypeDef *Head; uint32_t NodeNumber; uint32_t Type; } DMA_QListTypeDef;

typedef struct
{
  uint32_t Request;
  uint32_t BlkHWRequest;
  uint32_t Direction;
  uint32_t SrcInc;
  uint32_t DestInc;
  uint32_t SrcDataWidth;
  uint32_t DestDataWidth;
  uint32_t SrcBurstLength;
  uint32_t DestBurstLength;
  uint32_t Priority;
  uint32_t TransferEventMode;
  uint32_t TransferAllocatedPort;
} DMA_InitTypeDef;

typedef struct
{
  uint32_t NodeType;
  DMA_InitTypeDef Init;
  struct { uint32_t DataExchange; uint32_t DataAlignment; } DataHandlingConfig;
  struct { uint32_t TriggerPolarity; } TriggerConfig;
  uint32_t SrcSecure;
  uint32_t DestSecure;
} DMA_NodeConfTypeDef;

typedef struct
{
  uint32_t Priority;
  uint32_t LinkStepMode;
  uint32_t LinkAllocatedPort;
  uint32_t TransferEventMode;
  uint32_t LinkedListMode;
} DMA_InitLinkedListTypeDef;

typedef struct __DMA_HandleTypeDef
{
  DMA_Channel_TypeDef      *Instance;
  DMA_InitLinkedListTypeDef InitLinkedList;
  void                     *Parent;
} DMA_HandleTypeDef;

#define DMA_GPDMA_LINEAR_NODE             1U
#define GPDMA1_REQUEST_SPI1_RX            1U
#define DMA_BREQ_SINGLE_BURST             0U
#define DMA_PERIPH_TO_MEMORY              0U
#define DMA_SINC_FIXED                    0U
#define DMA_DINC_INCREMENTED              1U
#define DMA_SRC_DATAWIDTH_BYTE            0U
#define DMA_DEST_DATAWIDTH_BYTE           0U
#define DMA_HIGH_PRIORITY                 3U
#define DMA_TCEM_BLOCK_TRANSFER           0U
#define DMA_SRC_ALLOCATED_PORT0           0U
#define DMA_DEST_ALLOCATED_PORT1          1U
#define DMA_EXCHANGE_NONE                 0U
#define DMA_DATA_RIGHTALIGN_ZEROPADDED    0U
#define DMA_TRIG_POLARITY_MASKED          0U
#define DMA_LSM_FULL_EXECUTION            0U
#define DMA_LINK_ALLOCATED_PORT1          1U
#define DMA_LINKEDLIST_CIRCULAR           1U

HAL_StatusTypeDef HAL_DMAEx_List_BuildNode(DMA_NodeConfTypeDef *pNodeConfig, DMA_NodeTypeDef *pNode);
HAL_StatusTypeDef HAL_DMAEx_List_InsertNode_Tail(DMA_QListTypeDef *pQList, DMA_NodeTypeDef *pNewNode);
HAL_StatusTypeDef HAL_DMAEx_List_SetCircularMode(DMA_QListTypeDef *pQList);
HAL_StatusTypeDef HAL_DMAEx_List_Init(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMAEx_List_LinkQ(DMA_HandleTypeDef *hdma, DMA_QListTypeDef *pQList);
void              HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);

#define __HAL_LINKDMA(HANDLE, PPP_DMA_FIELD, DMA_HANDLE) \
  do { (HANDLE)->PPP_DMA_FIELD = &(DMA_HANDLE); (DMA_HANDLE).Parent = (HANDLE); } while (0)

/* SPI. */
typedef struct { uint32_t id; volatile uint32_t CR1; } SPI_TypeDef;
#ifndef READ_REG
#define READ_REG(REG) ((REG))
#endif
#ifndef __DSB
#define __DSB() do { } while (0)
#endif
extern SPI_TypeDef fake_spi1;
#define SPI1 (&fake_spi1)

typedef struct
{
  uint32_t Mode;
  uint32_t Direction;
  uint32_t DataSize;
  uint32_t CLKPolarity;
  uint32_t CLKPhase;
  uint32_t NSS;
  uint32_t BaudRatePrescaler;
  uint32_t FirstBit;
  uint32_t TIMode;
  uint32_t CRCCalculation;
  uint32_t CRCPolynomial;
  uint32_t CRCLength;
  uint32_t NSSPMode;
  uint32_t NSSPolarity;
  uint32_t FifoThreshold;
  uint32_t TxCRCInitializationPattern;
  uint32_t RxCRCInitializationPattern;
  uint32_t MasterSSIdleness;
  uint32_t MasterInterDataIdleness;
  uint32_t MasterReceiverAutoSusp;
  uint32_t MasterKeepIOState;
  uint32_t IOSwap;
  uint32_t ReadyMasterManagement;
  uint32_t ReadyPolarity;
} SPI_InitTypeDef;

typedef struct __SPI_HandleTypeDef
{
  SPI_TypeDef       *Instance;
  SPI_InitTypeDef    Init;
  DMA_HandleTypeDef *hdmarx;
  uint32_t           ErrorCode;
} SPI_HandleTypeDef;

#define SPI_MODE_MASTER                          1U
#define SPI_DIRECTION_2LINES_RXONLY              2U
#define SPI_DATASIZE_8BIT                        7U
#define SPI_POLARITY_HIGH                        1U
#define SPI_PHASE_2EDGE                          1U
#define SPI_NSS_SOFT                             1U
#define SPI_BAUDRATEPRESCALER_2                  0U
#define SPI_BAUDRATEPRESCALER_4                  1U
#define SPI_BAUDRATEPRESCALER_8                  2U
#define SPI_BAUDRATEPRESCALER_16                 3U
#define SPI_BAUDRATEPRESCALER_32                 4U
#define SPI_BAUDRATEPRESCALER_64                 5U
#define SPI_BAUDRATEPRESCALER_128                6U
#define SPI_BAUDRATEPRESCALER_256                7U
#define SPI_FIRSTBIT_MSB                         0U
#define SPI_TIMODE_DISABLE                       0U
#define SPI_CRCCALCULATION_DISABLE               0U
#define SPI_CRC_LENGTH_8BIT                      7U
#define SPI_NSS_PULSE_DISABLE                    0U
#define SPI_NSS_POLARITY_LOW                     0U
#define SPI_FIFO_THRESHOLD_01DATA                0U
#define SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN  0U
#define SPI_MASTER_SS_IDLENESS_00CYCLE           0U
#define SPI_MASTER_INTERDATA_IDLENESS_00CYCLE    0U
#define SPI_MASTER_RX_AUTOSUSP_DISABLE           0U
#define SPI_MASTER_KEEP_IO_STATE_ENABLE          1U
#define SPI_IO_SWAP_DISABLE                      0U
#define SPI_RDY_MASTER_MANAGEMENT_INTERNALLY     0U
#define SPI_RDY_POLARITY_HIGH                    0U

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *hspi);
void              HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi);
void              HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void              HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void              HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);

/* Time. */
void HAL_Delay(uint32_t Delay);

#endif /* FAKE_STM32N6XX_HAL_H */
