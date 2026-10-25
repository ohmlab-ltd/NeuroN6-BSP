/**
  * @file    neuron6_thermal.c
  * @brief   FLIR Lepton 3.1R thermal camera: command interface on I2C1, video on SPI1.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include <string.h>

#include "neuron6_thermal.h"
#include "neuron6.h"
#include "neuron6_bus.h"

/* Size unit: whole packets in each half, whole cache lines (1312 bytes, 41 lines). */
#define THERMAL_BUFFER_GRANULARITY  (8U * LEPTON_PACKET_BYTES)

/* The transfer length field is 16 bits. */
#define THERMAL_BUFFER_MAX_BYTES    65535U

/* Cache-line alignment: the buffer is invalidated. */
#define THERMAL_BUFFER_ALIGNMENT    32U

#if ((THERMAL_BUFFER_GRANULARITY % THERMAL_BUFFER_ALIGNMENT) != 0U) || \
    ((THERMAL_BUFFER_GRANULARITY % (2U * LEPTON_PACKET_BYTES)) != 0U)
#error "THERMAL_BUFFER_GRANULARITY must be whole packet pairs and whole cache lines"
#endif

#define THERMAL_PRESCALER_NBR       8U

/* A half that is 9/10 well-formed packets is progress even with no image: the camera
 * can send only fillers for seconds (settling, FFC, radiometry set-up). */
#define THERMAL_PROGRESS_NUMERATOR  9U
#define THERMAL_PROGRESS_DENOMINATOR 10U

static LEPTON_Object_t Thermal_Lepton;
static LEPTON_VoSPI_t Thermal_Parser;
static SPI_HandleTypeDef Thermal_Spi;
static DMA_HandleTypeDef Thermal_Dma;

/* The DMA descriptor must be non-cacheable: see BSP_THERMAL_DMA_NODE_SECTION. */
__attribute__((section(BSP_THERMAL_DMA_NODE_SECTION), aligned(32)))
static DMA_NodeTypeDef Thermal_DmaNode;
static DMA_QListTypeDef Thermal_DmaQueue;

static uint8_t Thermal_Ready;                     /* Command interface up */
static uint8_t Thermal_PoweredUp;                 /* Powered and out of reset */
static uint32_t Thermal_ReleaseTick;              /* Tick of the last reset release */
static uint8_t Thermal_VideoConfigured;
static volatile uint8_t Thermal_Streaming;        /* A transfer is running */
static uint8_t Thermal_VideoSession;              /* Commands refused while set */
static uint8_t Thermal_Flip180;
static uint8_t *Thermal_Buffer;                   /* The caller's circular buffer */
static uint32_t Thermal_BufferBytes;
static uint32_t Thermal_PacketsPerHalf;
static volatile uint32_t Thermal_HalvesProduced;  /* Written in interrupt context */
static uint32_t Thermal_HalvesConsumed;
static uint32_t Thermal_Overruns;
static uint32_t Thermal_Restarts;
static volatile uint32_t Thermal_SpiErrors;
static volatile uint32_t Thermal_LastSpiError;
static uint32_t Thermal_ProgressTick;             /* Tick of the last progress */
static uint32_t Thermal_BadHalves;                /* Mostly malformed halves in a row */
static uint32_t Thermal_SckHz;
static uint32_t Thermal_KernelHz;

static int32_t Thermal_Check(uint32_t Instance);
static int32_t Thermal_CheckCci(uint32_t Instance);
static int32_t Thermal_MapResult(int32_t Result);
static void    Thermal_Delay(uint32_t Milliseconds);
static void    Thermal_SelectLow(void);
static void    Thermal_SelectHigh(void);
static void    Thermal_ConfigureControlPins(void);
static int32_t Thermal_ConfigureSpi(void);
static int32_t Thermal_ConfigureDma(void);
static void    Thermal_CaptureStop(void);
static int32_t Thermal_CaptureStart(void);
static void    Thermal_InvalidateHalf(const uint8_t *pHalf, uint32_t Bytes);
static void    Thermal_PinsOff(void);
#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
static int32_t Thermal_SpiRegisterCallbacks(void);
#endif

static int32_t Thermal_Check(uint32_t Instance)
{
  if (Instance >= BSP_THERMAL_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Thermal_Ready == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  return BSP_ERROR_NONE;
}

/* Commands are refused for the whole video session, not just while streaming. */
static int32_t Thermal_CheckCci(uint32_t Instance)
{
  int32_t status = Thermal_Check(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (Thermal_VideoSession != 0U)
  {
    return BSP_ERROR_BUSY;
  }

  return BSP_ERROR_NONE;
}

static int32_t Thermal_MapResult(int32_t Result)
{
  if (Result == 0)
  {
    return BSP_ERROR_NONE;
  }

  if (Result == LEPTON_WRONG_PARAM)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* A timeout, a bus failure or the camera's refusal; BSP_ERROR_BUSY means video. */
  return BSP_ERROR_COMPONENT_FAILURE;
}

static void Thermal_Delay(uint32_t Milliseconds)
{
  HAL_Delay(Milliseconds);
}

static void Thermal_SelectLow(void)
{
  HAL_GPIO_WritePin(BSP_THERMAL_CS_GPIO_PORT, BSP_THERMAL_CS_PIN, GPIO_PIN_RESET);
}

static void Thermal_SelectHigh(void)
{
  HAL_GPIO_WritePin(BSP_THERMAL_CS_GPIO_PORT, BSP_THERMAL_CS_PIN, GPIO_PIN_SET);
}

/* Park the control pins; levels are written before outputs are enabled (no glitch). */
static void Thermal_ConfigureControlPins(void)
{
  GPIO_InitTypeDef gpio_init_structure = {0};

  BSP_THERMAL_PWRDWN_CLK_ENABLE();
  BSP_THERMAL_RESET_CLK_ENABLE();
  BSP_THERMAL_CS_CLK_ENABLE();

  /* Select high, power-down and reset low: off, with a released select at power-up. */
  HAL_GPIO_WritePin(BSP_THERMAL_CS_GPIO_PORT, BSP_THERMAL_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BSP_THERMAL_PWRDWN_GPIO_PORT, BSP_THERMAL_PWRDWN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BSP_THERMAL_RESET_GPIO_PORT, BSP_THERMAL_RESET_PIN, GPIO_PIN_RESET);

  gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull  = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;

  gpio_init_structure.Pin = BSP_THERMAL_PWRDWN_PIN;
  HAL_GPIO_Init(BSP_THERMAL_PWRDWN_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin = BSP_THERMAL_RESET_PIN;
  HAL_GPIO_Init(BSP_THERMAL_RESET_GPIO_PORT, &gpio_init_structure);

  /* A faster select: its edge relative to the clock sets byte alignment. */
  gpio_init_structure.Pin   = BSP_THERMAL_CS_PIN;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BSP_THERMAL_CS_GPIO_PORT, &gpio_init_structure);
}

static int32_t Thermal_ConfigureSpi(void)
{
  static const uint32_t prescaler_value[THERMAL_PRESCALER_NBR] =
  {
    SPI_BAUDRATEPRESCALER_2,   SPI_BAUDRATEPRESCALER_4,
    SPI_BAUDRATEPRESCALER_8,   SPI_BAUDRATEPRESCALER_16,
    SPI_BAUDRATEPRESCALER_32,  SPI_BAUDRATEPRESCALER_64,
    SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_256,
  };

  RCC_PeriphCLKInitTypeDef periph_clk_init = {0};
  GPIO_InitTypeDef gpio_init_structure = {0};
  uint32_t kernel_hz;
  uint32_t divider = 2U;
  uint32_t index = 0U;

  /* Kernel clock = PCLK2: every other source taps the PLL the boot loader dedicates
   * to external memory, which must not be disturbed. */
  periph_clk_init.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
  periph_clk_init.Spi1ClockSelection   = RCC_SPI1CLKSOURCE_PCLK2;
  (void)HAL_RCCEx_PeriphCLKConfig(&periph_clk_init);

  BSP_THERMAL_SPI_CLK_ENABLE();

  kernel_hz = HAL_RCC_GetPCLK2Freq();

  if (kernel_hz == 0U)
  {
    return BSP_ERROR_CLOCK_FAILURE;
  }

  Thermal_KernelHz = kernel_hz;

  /* Fastest rate at or below what the camera accepts. */
  while ((index < (THERMAL_PRESCALER_NBR - 1U)) &&
         ((kernel_hz / divider) > BSP_THERMAL_SCK_MAX_HZ))
  {
    divider <<= 1;
    index++;
  }

  Thermal_SckHz = kernel_hz / divider;

  BSP_THERMAL_SCK_CLK_ENABLE();
  BSP_THERMAL_MISO_CLK_ENABLE();
  BSP_THERMAL_MOSI_CLK_ENABLE();

  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull      = GPIO_NOPULL;
  gpio_init_structure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init_structure.Alternate = BSP_THERMAL_SPI_AF;

  gpio_init_structure.Pin = BSP_THERMAL_SCK_PIN;
  HAL_GPIO_Init(BSP_THERMAL_SCK_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin = BSP_THERMAL_MISO_PIN;
  HAL_GPIO_Init(BSP_THERMAL_MISO_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin = BSP_THERMAL_MOSI_PIN;
  HAL_GPIO_Init(BSP_THERMAL_MOSI_GPIO_PORT, &gpio_init_structure);

  /* Receive-only master: an endless receive makes the SPI generate the clock itself. */
  Thermal_Spi.Instance                        = BSP_THERMAL_SPI;
  Thermal_Spi.Init.Mode                       = SPI_MODE_MASTER;
  Thermal_Spi.Init.Direction                  = SPI_DIRECTION_2LINES_RXONLY;
  Thermal_Spi.Init.DataSize                   = SPI_DATASIZE_8BIT;
  Thermal_Spi.Init.CLKPolarity                = SPI_POLARITY_HIGH;
  Thermal_Spi.Init.CLKPhase                   = SPI_PHASE_2EDGE;
  Thermal_Spi.Init.NSS                        = SPI_NSS_SOFT;
  Thermal_Spi.Init.BaudRatePrescaler          = prescaler_value[index];
  Thermal_Spi.Init.FirstBit                   = SPI_FIRSTBIT_MSB;
  Thermal_Spi.Init.TIMode                     = SPI_TIMODE_DISABLE;
  Thermal_Spi.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;
  Thermal_Spi.Init.CRCPolynomial              = 7U;
  Thermal_Spi.Init.CRCLength                  = SPI_CRC_LENGTH_8BIT;
  Thermal_Spi.Init.NSSPMode                   = SPI_NSS_PULSE_DISABLE;
  Thermal_Spi.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;
  Thermal_Spi.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;
  Thermal_Spi.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  Thermal_Spi.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  Thermal_Spi.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;
  Thermal_Spi.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  Thermal_Spi.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  Thermal_Spi.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  Thermal_Spi.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;
  Thermal_Spi.Init.ReadyMasterManagement      = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  Thermal_Spi.Init.ReadyPolarity              = SPI_RDY_POLARITY_HIGH;

  if (HAL_SPI_Init(&Thermal_Spi) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
  /* HAL_SPI_Init() from reset restores the default callbacks: register after each. */
  return Thermal_SpiRegisterCallbacks();
#else
  return BSP_ERROR_NONE;
#endif
}

/* One circular descriptor over the whole buffer, linked to the SPI's receive. */
static int32_t Thermal_ConfigureDma(void)
{
  DMA_NodeConfTypeDef node_config = {0};

  __HAL_RCC_GPDMA1_CLK_ENABLE();

  (void)memset(&Thermal_DmaNode, 0, sizeof(Thermal_DmaNode));
  (void)memset(&Thermal_DmaQueue, 0, sizeof(Thermal_DmaQueue));

  Thermal_Dma.Instance = BSP_THERMAL_DMA_CH(BSP_THERMAL_DMA_CHANNEL);

  node_config.NodeType                         = DMA_GPDMA_LINEAR_NODE;
  node_config.Init.Request                     = GPDMA1_REQUEST_SPI1_RX;
  node_config.Init.BlkHWRequest                = DMA_BREQ_SINGLE_BURST;
  node_config.Init.Direction                   = DMA_PERIPH_TO_MEMORY;
  node_config.Init.SrcInc                      = DMA_SINC_FIXED;
  node_config.Init.DestInc                     = DMA_DINC_INCREMENTED;
  node_config.Init.SrcDataWidth                = DMA_SRC_DATAWIDTH_BYTE;
  node_config.Init.DestDataWidth               = DMA_DEST_DATAWIDTH_BYTE;
  node_config.Init.SrcBurstLength              = 1U;
  node_config.Init.DestBurstLength             = 1U;
  node_config.Init.Priority                    = DMA_HIGH_PRIORITY;
  node_config.Init.TransferEventMode           = DMA_TCEM_BLOCK_TRANSFER;
  node_config.Init.TransferAllocatedPort       = DMA_SRC_ALLOCATED_PORT0 |
                                                 DMA_DEST_ALLOCATED_PORT1;
  node_config.DataHandlingConfig.DataExchange  = DMA_EXCHANGE_NONE;
  node_config.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
  node_config.TriggerConfig.TriggerPolarity    = DMA_TRIG_POLARITY_MASKED;
#if defined(CPU_IN_SECURE_STATE)
  node_config.SrcSecure                        = DMA_CHANNEL_SRC_SEC;
  node_config.DestSecure                       = DMA_CHANNEL_DEST_SEC;
#endif

  if (HAL_DMAEx_List_BuildNode(&node_config, &Thermal_DmaNode) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  if (HAL_DMAEx_List_InsertNode_Tail(&Thermal_DmaQueue, &Thermal_DmaNode) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  if (HAL_DMAEx_List_SetCircularMode(&Thermal_DmaQueue) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  Thermal_Dma.InitLinkedList.Priority          = DMA_HIGH_PRIORITY;
  Thermal_Dma.InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
  Thermal_Dma.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT1;
  Thermal_Dma.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  Thermal_Dma.InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;

  if (HAL_DMAEx_List_Init(&Thermal_Dma) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* Isolation grants: secure-state builds only. */
#if defined(CPU_IN_SECURE_STATE)
  __HAL_RCC_RIFSC_CLK_ENABLE();

  if (HAL_DMA_ConfigChannelAttributes(&Thermal_Dma,
                                      DMA_CHANNEL_SEC | DMA_CHANNEL_PRIV |
                                      DMA_CHANNEL_SRC_SEC | DMA_CHANNEL_DEST_SEC) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* The secure channel reads the video peripheral as a slave. */
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_SPI1,
                                        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
#endif

  if (HAL_DMAEx_List_LinkQ(&Thermal_Dma, &Thermal_DmaQueue) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  __HAL_LINKDMA(&Thermal_Spi, hdmarx, Thermal_Dma);

  HAL_NVIC_SetPriority(BSP_THERMAL_DMA_IRQn, BSP_THERMAL_IT_PRIORITY, 0x00U);
  HAL_NVIC_EnableIRQ(BSP_THERMAL_DMA_IRQn);
  HAL_NVIC_SetPriority(BSP_THERMAL_SPI_IRQn, BSP_THERMAL_IT_PRIORITY, 0x00U);
  HAL_NVIC_EnableIRQ(BSP_THERMAL_SPI_IRQn);

  return BSP_ERROR_NONE;
}

static void Thermal_CaptureStop(void)
{
  if (Thermal_Streaming != 0U)
  {
    (void)HAL_SPI_Abort(&Thermal_Spi);
    Thermal_Streaming = 0U;
  }

  Thermal_SelectHigh();
}

/* Resynchronise and start the endless transfer: after the idle time the camera restarts
 * its framer, so the first byte clocked starts a packet. */
static int32_t Thermal_CaptureStart(void)
{
  Thermal_CaptureStop();
  Thermal_Delay(BSP_THERMAL_RESYNC_IDLE_MS);

  Thermal_HalvesProduced = 0U;
  Thermal_HalvesConsumed = 0U;
  Thermal_BadHalves = 0U;
  Thermal_SpiErrors = 0U;
  Thermal_LastSpiError = 0U;
  (void)LEPTON_VoSPI_Reset(&Thermal_Parser);

  /* Hold the sensor bus so no command transaction is in flight as the clock starts. */
  BSP_I2C1_Lock();

  Thermal_SelectLow();

  if (HAL_SPI_Receive_DMA(&Thermal_Spi, Thermal_Buffer,
                          (uint16_t)Thermal_BufferBytes) != HAL_OK)
  {
    Thermal_SelectHigh();
    BSP_I2C1_Unlock();
    return BSP_ERROR_PERIPH_FAILURE;
  }

  Thermal_Streaming = 1U;

  BSP_I2C1_Unlock();

  Thermal_Restarts++;
  Thermal_ProgressTick = (uint32_t)BSP_GetTick();

  return BSP_ERROR_NONE;
}

/* Safe: the CPU never writes the buffer, and it ends on a cache line of its own. */
static void Thermal_InvalidateHalf(const uint8_t *pHalf, uint32_t Bytes)
{
#if defined(USE_BSP_CPU_CACHE_MAINTENANCE) && (USE_BSP_CPU_CACHE_MAINTENANCE == 1U)
  SCB_InvalidateDCache_by_Addr((void *)(uintptr_t)pHalf, (int32_t)Bytes);
#else
  (void)pHalf;
  (void)Bytes;
#endif
}

int32_t BSP_THERMAL_Init(uint32_t Instance)
{
  LEPTON_IO_t io;
  int32_t status;
  uint32_t since_ms;
  uint32_t asking_ms;

  if (Instance >= BSP_THERMAL_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Thermal_Ready != 0U)
  {
    return BSP_ERROR_NONE;
  }

  io.Init     = BSP_I2C1_Init;
  io.DeInit   = BSP_I2C1_DeInit;
  io.Address  = BSP_THERMAL_I2C_ADDRESS;
  io.WriteReg = BSP_I2C1_WriteReg16;
  io.ReadReg  = BSP_I2C1_ReadReg16;
  io.IsReady  = BSP_I2C1_IsReady;
  io.GetTick  = BSP_GetTick;
  io.Delay    = Thermal_Delay;

  if (LEPTON_RegisterBusIO(&Thermal_Lepton, &io) != LEPTON_OK)
  {
    return BSP_ERROR_BUS_FAILURE;
  }

  (void)LEPTON_VoSPI_Init(&Thermal_Parser);

  /* Left to boot if the application powered it up; otherwise power it up now. */
  if (Thermal_PoweredUp == 0U)
  {
    status = BSP_THERMAL_PowerOn(Instance);

    if (status != BSP_ERROR_NONE)
    {
      return status;
    }
  }

  /* Asked too soon after its reset release, the camera never boots: wait out the quiet
   * time, then ask until the boot timeout (at least once). */
  since_ms = (uint32_t)BSP_GetTick() - Thermal_ReleaseTick;
  if (since_ms < BSP_THERMAL_BOOT_QUIET_MS)
  {
    Thermal_Delay(BSP_THERMAL_BOOT_QUIET_MS - since_ms);
    since_ms = (uint32_t)BSP_GetTick() - Thermal_ReleaseTick;
  }
  asking_ms = (BSP_THERMAL_BOOT_TIMEOUT_MS > since_ms)
              ? (BSP_THERMAL_BOOT_TIMEOUT_MS - since_ms) : 0U;

  /* The bus is brought up only now, as that can drive it. A camera that does not boot
   * is powered down; the driver has already released the bus. */
  status = LEPTON_Init(&Thermal_Lepton, asking_ms);
  if (status != LEPTON_OK)
  {
    (void)BSP_THERMAL_PowerOff(Instance);
    return (status == LEPTON_ERROR) ? BSP_ERROR_BUS_FAILURE : BSP_ERROR_COMPONENT_FAILURE;
  }

  Thermal_Ready = 1U;

  return BSP_ERROR_NONE;
}

int32_t BSP_THERMAL_DeInit(uint32_t Instance)
{
  int32_t status = Thermal_Check(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (Thermal_VideoSession != 0U)
  {
    (void)BSP_THERMAL_VideoStop(Instance);
  }

  Thermal_CaptureStop();
  Thermal_PinsOff();

  (void)LEPTON_DeInit(&Thermal_Lepton);

  Thermal_Ready = 0U;

  return BSP_ERROR_NONE;
}

int32_t BSP_THERMAL_PowerOn(uint32_t Instance)
{
  if (Instance >= BSP_THERMAL_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Park the pins and stop any transfer: the camera samples the select at power-up. */
  Thermal_ConfigureControlPins();
  Thermal_CaptureStop();

  /* A full power cycle, power returning with the reset still held. */
  HAL_GPIO_WritePin(BSP_THERMAL_PWRDWN_GPIO_PORT, BSP_THERMAL_PWRDWN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BSP_THERMAL_RESET_GPIO_PORT, BSP_THERMAL_RESET_PIN, GPIO_PIN_RESET);
  Thermal_Delay(BSP_THERMAL_POWER_CYCLE_MS);

  HAL_GPIO_WritePin(BSP_THERMAL_PWRDWN_GPIO_PORT, BSP_THERMAL_PWRDWN_PIN, GPIO_PIN_SET);
  Thermal_Delay(BSP_THERMAL_RESET_PULSE_MS);

  HAL_GPIO_WritePin(BSP_THERMAL_RESET_GPIO_PORT, BSP_THERMAL_RESET_PIN, GPIO_PIN_SET);
  Thermal_ReleaseTick = (uint32_t)BSP_GetTick();
  Thermal_PoweredUp = 1U;

  return BSP_ERROR_NONE;
}

int32_t BSP_THERMAL_PowerOff(uint32_t Instance)
{
  if (Instance >= BSP_THERMAL_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Off means no video and a fresh boot, so undo BSP_THERMAL_Init(). */
  if (Thermal_Ready != 0U)
  {
    return BSP_THERMAL_DeInit(Instance);
  }

  Thermal_CaptureStop();
  Thermal_PinsOff();

  return BSP_ERROR_NONE;
}

static void Thermal_PinsOff(void)
{
  HAL_GPIO_WritePin(BSP_THERMAL_RESET_GPIO_PORT, BSP_THERMAL_RESET_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BSP_THERMAL_PWRDWN_GPIO_PORT, BSP_THERMAL_PWRDWN_PIN, GPIO_PIN_RESET);
  Thermal_SelectHigh();
  Thermal_PoweredUp = 0U;
}

int32_t BSP_THERMAL_GetControlPins(uint32_t Instance, uint8_t *pPowerHigh, uint8_t *pResetHigh)
{
  if ((Instance >= BSP_THERMAL_INSTANCES_NBR) || (pPowerHigh == NULL) || (pResetHigh == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  *pPowerHigh = (HAL_GPIO_ReadPin(BSP_THERMAL_PWRDWN_GPIO_PORT,
                                  BSP_THERMAL_PWRDWN_PIN) == GPIO_PIN_SET) ? 1U : 0U;
  *pResetHigh = (HAL_GPIO_ReadPin(BSP_THERMAL_RESET_GPIO_PORT,
                                  BSP_THERMAL_RESET_PIN) == GPIO_PIN_SET) ? 1U : 0U;

  return BSP_ERROR_NONE;
}

int32_t BSP_THERMAL_IsPresent(uint32_t Instance)
{
  int32_t status = Thermal_CheckCci(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (LEPTON_IsPresent(&Thermal_Lepton) != LEPTON_OK)
  {
    return BSP_ERROR_UNKNOWN_COMPONENT;
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_THERMAL_Ping(uint32_t Instance)
{
  int32_t status = Thermal_CheckCci(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  return Thermal_MapResult(LEPTON_Ping(&Thermal_Lepton));
}

int32_t BSP_THERMAL_GetSerialNumber(uint32_t Instance, uint16_t *pSerial)
{
  int32_t status = Thermal_CheckCci(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pSerial == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  return Thermal_MapResult(LEPTON_GetSerialNumber(&Thermal_Lepton, pSerial));
}

int32_t BSP_THERMAL_GetUptime(uint32_t Instance, uint32_t *pUptime_ms)
{
  int32_t status = Thermal_CheckCci(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pUptime_ms == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  return Thermal_MapResult(LEPTON_GetUptime(&Thermal_Lepton, pUptime_ms));
}

int32_t BSP_THERMAL_GetFpaTemperature(uint32_t Instance, uint16_t *pKelvin100)
{
  int32_t status = Thermal_CheckCci(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pKelvin100 == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  return Thermal_MapResult(LEPTON_GetFpaTemperature(&Thermal_Lepton, pKelvin100));
}

int32_t BSP_THERMAL_GetAuxTemperature(uint32_t Instance, uint16_t *pKelvin100)
{
  int32_t status = Thermal_CheckCci(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pKelvin100 == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  return Thermal_MapResult(LEPTON_GetAuxTemperature(&Thermal_Lepton, pKelvin100));
}

int32_t BSP_THERMAL_GetAgcEnabled(uint32_t Instance, uint8_t *pEnabled)
{
  int32_t status = Thermal_CheckCci(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pEnabled == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  return Thermal_MapResult(LEPTON_GetAgcEnabled(&Thermal_Lepton, pEnabled));
}

int32_t BSP_THERMAL_SetAgcEnabled(uint32_t Instance, uint8_t Enabled)
{
  int32_t status = Thermal_CheckCci(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  return Thermal_MapResult(LEPTON_SetAgcEnabled(&Thermal_Lepton, Enabled));
}

int32_t BSP_THERMAL_RunFfc(uint32_t Instance)
{
  int32_t status = Thermal_CheckCci(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  return Thermal_MapResult(LEPTON_RunFfc(&Thermal_Lepton));
}

int32_t BSP_THERMAL_ReadRegister(uint32_t Instance, uint16_t Reg, uint16_t *pValue)
{
  int32_t status = Thermal_CheckCci(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pValue == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  return Thermal_MapResult(LEPTON_ReadRegister(&Thermal_Lepton, Reg, pValue));
}

int32_t BSP_THERMAL_WriteRegister(uint32_t Instance, uint16_t Reg, uint16_t Value)
{
  int32_t status = Thermal_CheckCci(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  return Thermal_MapResult(LEPTON_WriteRegister(&Thermal_Lepton, Reg, Value));
}

int32_t BSP_THERMAL_GetAttribute(uint32_t Instance, uint16_t Command,
                                 uint16_t *pWords, uint16_t LengthWords)
{
  int32_t status = Thermal_CheckCci(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  return Thermal_MapResult(LEPTON_GetAttribute(&Thermal_Lepton, Command, pWords, LengthWords));
}

int32_t BSP_THERMAL_SetAttribute(uint32_t Instance, uint16_t Command,
                                 const uint16_t *pWords, uint16_t LengthWords)
{
  int32_t status = Thermal_CheckCci(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  return Thermal_MapResult(LEPTON_SetAttribute(&Thermal_Lepton, Command, pWords, LengthWords));
}

int32_t BSP_THERMAL_RunCommand(uint32_t Instance, uint16_t Command)
{
  int32_t status = Thermal_CheckCci(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  return Thermal_MapResult(LEPTON_RunCommand(&Thermal_Lepton, Command));
}

int32_t BSP_THERMAL_GetLastResult(uint32_t Instance)
{
  if (Instance >= BSP_THERMAL_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  return (int32_t)Thermal_Lepton.LastResult;
}

int32_t BSP_THERMAL_VideoStart(uint32_t Instance, uint8_t *pBuffer, uint32_t Size)
{
  int32_t status = Thermal_Check(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (Thermal_VideoSession != 0U)
  {
    return BSP_ERROR_BUSY;
  }

  if (pBuffer == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Whole packets in each half (for the parser), and an end on a cache line so that
   * invalidating the second half spares whatever follows the buffer. */
  if ((Size == 0U) || (Size > THERMAL_BUFFER_MAX_BYTES) ||
      ((Size % THERMAL_BUFFER_GRANULARITY) != 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if ((((uintptr_t)pBuffer) % THERMAL_BUFFER_ALIGNMENT) != 0U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  Thermal_Buffer = pBuffer;
  Thermal_BufferBytes = Size;
  Thermal_PacketsPerHalf = (Size / 2U) / LEPTON_PACKET_BYTES;

  if (Thermal_VideoConfigured == 0U)
  {
    status = Thermal_ConfigureSpi();

    if (status != BSP_ERROR_NONE)
    {
      return status;
    }

    status = Thermal_ConfigureDma();

    if (status != BSP_ERROR_NONE)
    {
      (void)HAL_SPI_DeInit(&Thermal_Spi);
      return status;
    }

    Thermal_VideoConfigured = 1U;
  }

  /* Commands are shut from now, not from the first frame. */
  Thermal_VideoSession = 1U;
  Thermal_Overruns = 0U;
  Thermal_Restarts = 0U;
  Thermal_ProgressTick = (uint32_t)BSP_GetTick();

  return BSP_ERROR_NONE;
}

int32_t BSP_THERMAL_VideoStop(uint32_t Instance)
{
  int32_t status = Thermal_Check(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (Thermal_VideoSession == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  Thermal_CaptureStop();

  if (Thermal_VideoConfigured != 0U)
  {
    (void)HAL_SPI_DeInit(&Thermal_Spi);
    HAL_NVIC_DisableIRQ(BSP_THERMAL_DMA_IRQn);
    HAL_NVIC_DisableIRQ(BSP_THERMAL_SPI_IRQn);
    /* Read back before gating the clock, so the last write lands. */
    (void)READ_REG(BSP_THERMAL_SPI->CR1);
    __DSB();
    BSP_THERMAL_SPI_CLK_DISABLE();
    Thermal_VideoConfigured = 0U;
  }

  Thermal_VideoSession = 0U;
  Thermal_Buffer = NULL;
  Thermal_BufferBytes = 0U;

  return BSP_ERROR_NONE;
}

int32_t BSP_THERMAL_VideoResync(uint32_t Instance)
{
  int32_t status = Thermal_Check(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (Thermal_VideoSession == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  return Thermal_CaptureStart();
}

int32_t BSP_THERMAL_GetFrame(uint32_t Instance, uint16_t *pFrame, uint32_t Timeout_ms)
{
  uint32_t start;
  int32_t  status = Thermal_Check(Instance);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (Thermal_VideoSession == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (pFrame == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Thermal_Streaming == 0U)
  {
    status = Thermal_CaptureStart();

    if (status != BSP_ERROR_NONE)
    {
      return status;
    }
  }

  start = (uint32_t)BSP_GetTick();

  for (;;)
  {
    uint32_t produced = Thermal_HalvesProduced;

    while (Thermal_HalvesConsumed != produced)
    {
      const uint32_t half_bytes = Thermal_BufferBytes / 2U;
      const uint8_t *half;
      uint32_t recognised = 0U;
      int32_t  parsed;

      /* Two halves behind: the half due next has been overwritten. */
      if ((produced - Thermal_HalvesConsumed) > 1U)
      {
        Thermal_Overruns++;
        Thermal_HalvesConsumed = produced - 1U;
        (void)LEPTON_VoSPI_Reset(&Thermal_Parser);
      }

      half = &Thermal_Buffer[(Thermal_HalvesConsumed % 2U) * half_bytes];

      Thermal_InvalidateHalf(half, half_bytes);

      parsed = LEPTON_VoSPI_Parse(&Thermal_Parser, half, Thermal_PacketsPerHalf,
                                  pFrame, &recognised);

      /* Again after the parse: a delayed parse may have read a half being rewritten. */
      if (((uint32_t)Thermal_HalvesProduced - Thermal_HalvesConsumed) > 1U)
      {
        Thermal_Overruns++;
        (void)LEPTON_VoSPI_Reset(&Thermal_Parser);
        Thermal_HalvesConsumed++;
        continue;
      }

      Thermal_HalvesConsumed++;

      /* Progress, image or not: see THERMAL_PROGRESS_NUMERATOR. */
      if (recognised >= ((Thermal_PacketsPerHalf * THERMAL_PROGRESS_NUMERATOR) /
                         THERMAL_PROGRESS_DENOMINATOR))
      {
        Thermal_ProgressTick = (uint32_t)BSP_GetTick();
      }

      /* Mostly malformed: out of step (see BSP_THERMAL_RESYNC_BAD_HALVES). */
      Thermal_BadHalves = (recognised < (Thermal_PacketsPerHalf / 2U))
                          ? (Thermal_BadHalves + 1U) : 0U;

      if (parsed == 1)
      {
        if (Thermal_Flip180 != 0U)
        {
          (void)LEPTON_VoSPI_Rotate180(pFrame);
        }

        return BSP_ERROR_NONE;
      }
    }

    /* An SPI error, a stall or a run of malformed halves needs a restart. Judged after
     * the backlog is parsed, so a slow caller's absence is not taken for a stall. */
    if ((Thermal_SpiErrors != 0U) ||
        (((uint32_t)BSP_GetTick() - Thermal_ProgressTick) > BSP_THERMAL_STALL_TIMEOUT_MS) ||
        (Thermal_BadHalves >= BSP_THERMAL_RESYNC_BAD_HALVES))
    {
#if (BSP_THERMAL_AUTO_RESYNC == 1U)
      status = Thermal_CaptureStart();

      if (status != BSP_ERROR_NONE)
      {
        return status;
      }
#else
      return BSP_ERROR_PERIPH_FAILURE;
#endif
    }

    /* Unsigned elapsed time: any budget up to HAL_MAX_DELAY is waited in full. */
    if (((uint32_t)BSP_GetTick() - start) >= Timeout_ms)
    {
      return BSP_ERROR_BUSY;
    }
  }
}

int32_t BSP_THERMAL_SetFlip180(uint32_t Instance, uint8_t Enable)
{
  if (Instance >= BSP_THERMAL_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  Thermal_Flip180 = (Enable != 0U) ? 1U : 0U;

  return BSP_ERROR_NONE;
}

int32_t BSP_THERMAL_GetHealth(uint32_t Instance, BSP_THERMAL_Health_t *pHealth)
{
  if ((Instance >= BSP_THERMAL_INSTANCES_NBR) || (pHealth == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  pHealth->HalvesProduced = Thermal_HalvesProduced;
  pHealth->HalvesConsumed = Thermal_HalvesConsumed;
  pHealth->PacketsSeen    = Thermal_Parser.PacketsSeen;
  pHealth->FillersSeen    = Thermal_Parser.FillersSeen;
  pHealth->SegmentsParsed = Thermal_Parser.SegmentsParsed;
  pHealth->FramesParsed   = Thermal_Parser.FramesParsed;
  pHealth->Resyncs        = Thermal_Parser.Resyncs;
  pHealth->Restarts       = Thermal_Restarts;
  pHealth->Overruns       = Thermal_Overruns;
  pHealth->SpiErrors      = Thermal_SpiErrors;
  pHealth->LastSpiError   = Thermal_LastSpiError;
  pHealth->LastHeader     = Thermal_Parser.LastHeader;
  pHealth->SckHz          = Thermal_SckHz;
  pHealth->KernelHz       = Thermal_KernelHz;

  return BSP_ERROR_NONE;
}

void BSP_THERMAL_DMA_IRQHandler(uint32_t Instance)
{
  UNUSED(Instance);

  HAL_DMA_IRQHandler(&Thermal_Dma);
}

void BSP_THERMAL_SPI_IRQHandler(uint32_t Instance)
{
  UNUSED(Instance);

  HAL_SPI_IRQHandler(&Thermal_Spi);
}

__weak void BSP_THERMAL_HalfCpltCallback(uint32_t Instance)
{
  UNUSED(Instance);
}

__weak void BSP_THERMAL_ErrorCallback(uint32_t Instance)
{
  UNUSED(Instance);
}

/* SPI1 callbacks, registered or overriding the HAL's weak ones. Each checks the
 * instance, so an application using another SPI through the HAL is unaffected. */

/* The half count is odd after the first half and even after the second, telling
 * GetFrame which half to read. If the HAL reports only the later of two events, the
 * count steps by two to keep that parity, and the lost half shows as an overrun. */
static void Thermal_SpiRxHalf(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == BSP_THERMAL_SPI)
  {
    Thermal_HalvesProduced += ((Thermal_HalvesProduced & 1U) == 0U) ? 1U : 2U;
    BSP_THERMAL_HalfCpltCallback(0U);
  }
}

/* The transfer is endless: this is a wrap, and the DMA carries on into the first half. */
static void Thermal_SpiRxFull(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == BSP_THERMAL_SPI)
  {
    Thermal_HalvesProduced += ((Thermal_HalvesProduced & 1U) != 0U) ? 1U : 2U;
    BSP_THERMAL_HalfCpltCallback(0U);
  }
}

static void Thermal_SpiError(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == BSP_THERMAL_SPI)
  {
    Thermal_SpiErrors++;
    Thermal_LastSpiError = hspi->ErrorCode;
    BSP_THERMAL_ErrorCallback(0U);
  }
}

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)

static int32_t Thermal_SpiRegisterCallbacks(void)
{
  if ((HAL_SPI_RegisterCallback(&Thermal_Spi, HAL_SPI_RX_HALF_COMPLETE_CB_ID, Thermal_SpiRxHalf) != HAL_OK) ||
      (HAL_SPI_RegisterCallback(&Thermal_Spi, HAL_SPI_RX_COMPLETE_CB_ID, Thermal_SpiRxFull) != HAL_OK) ||
      (HAL_SPI_RegisterCallback(&Thermal_Spi, HAL_SPI_ERROR_CB_ID, Thermal_SpiError) != HAL_OK))
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return BSP_ERROR_NONE;
}

#else

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
  Thermal_SpiRxHalf(hspi);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  Thermal_SpiRxFull(hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  Thermal_SpiError(hspi);
}

#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
