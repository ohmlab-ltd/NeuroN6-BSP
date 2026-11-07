/**
  * @file    neuron6_sd.c
  * @brief   SD card on SDMMC1, the bus the MAYA-W476 radio also uses.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6_sd.h"
#include "neuron6.h"
#include "neuron6_wifi.h"

#include <string.h>

#if defined(HAL_SD_MODULE_ENABLED)

#define SD_CACHE_LINE               32U

/* Largest transfer in one command, so a stalled card times out in bounded time. */
#define SD_MAX_BLOCKS_PER_COMMAND   128U

/* AXISRAM1/2, through its secure and non-secure aliases: the IDMA reaches it directly;
 * other memory goes through the bounce buffer. */
#define SD_DIRECT_S_START           0x34000000UL
#define SD_DIRECT_NS_START          0x24000000UL
#define SD_DIRECT_SIZE              0x00200000UL

/* Fastest bus clock before a card is switched to high speed. */
#define SD_DEFAULT_SPEED_HZ         25000000U

/* Allocation unit in blocks, indexed by the card status AU_SIZE code. */
static const uint32_t Sd_AuBlocks[16] =
{
  0U, 32U, 64U, 128U, 256U, 512U, 1024U, 2048U,
  4096U, 8192U, 16384U, 24576U, 32768U, 49152U, 65536U, 131072U,
};

static SD_HandleTypeDef hbsp_sd;

static uint8_t Sd_Ready;

static volatile uint8_t Sd_Done;
static volatile uint8_t Sd_Failed;

static uint8_t Sd_Bounce[BSP_SD_BOUNCE_BLOCKS * BSP_SD_BLOCK_SIZE] __attribute__((aligned(SD_CACHE_LINE)));

static uint8_t Sd_RadioPowered(void);
static void    Sd_HoldRadioOff(void);
static int32_t Sd_ConfigureHost(void);
static uint32_t Sd_DefaultSpeedDiv(void);
static void    Sd_StopHostClock(void);
static uint8_t Sd_IsDirect(const uint8_t *pData, uint32_t Bytes);
static void    Sd_Poll(void);
static int32_t Sd_WaitTransferState(void);
static int32_t Sd_Transfer(uint8_t *pData, uint32_t Block, uint32_t Count, uint8_t Write);
static int32_t Sd_Fail(void);

__weak void BSP_SD_WaitHook(void)
{
}

/* 1 if PDn is driven high (the radio is on). Read from the port registers, so only the
 * radio's header is needed; an unclocked port would stall the read. */
static uint8_t Sd_RadioPowered(void)
{
  const uint32_t pin = (uint32_t)__builtin_ctz(BSP_WIFI_PDN_PIN);

  if (__HAL_RCC_GPIOB_IS_CLK_ENABLED() == 0U)
  {
    return 0U;
  }

  return ((((BSP_WIFI_PDN_GPIO_PORT->MODER >> (2U * pin)) & 3U) == 1U) &&
          ((BSP_WIFI_PDN_GPIO_PORT->ODR & BSP_WIFI_PDN_PIN) != 0U)) ? 1U : 0U;
}

/* PDn low: a powered-down radio leaves the bus to the card. */
static void Sd_HoldRadioOff(void)
{
  GPIO_InitTypeDef gpio_init_structure = {0};

  BSP_WIFI_PDN_CLK_ENABLE();
  HAL_GPIO_WritePin(BSP_WIFI_PDN_GPIO_PORT, BSP_WIFI_PDN_PIN, GPIO_PIN_RESET);

  gpio_init_structure.Pin   = BSP_WIFI_PDN_PIN;
  gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull  = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BSP_WIFI_PDN_GPIO_PORT, &gpio_init_structure);
}

/* Kernel clock, reset and pins. The radio sets its own kernel clock when it starts. */
static int32_t Sd_ConfigureHost(void)
{
  RCC_PeriphCLKInitTypeDef periph_clk_init = {0};
  GPIO_InitTypeDef gpio_init_structure = {0};

  periph_clk_init.PeriphClockSelection = RCC_PERIPHCLK_SDMMC1;
  periph_clk_init.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_IC4;
  periph_clk_init.ICSelection[RCC_IC4].ClockSelection = RCC_ICCLKSOURCE_PLL1;
  periph_clk_init.ICSelection[RCC_IC4].ClockDivider   = BSP_SD_KERNEL_DIV;

  if (HAL_RCCEx_PeriphCLKConfig(&periph_clk_init) != HAL_OK)
  {
    return BSP_ERROR_CLOCK_FAILURE;
  }

  /* Reset, whatever state the radio or earlier firmware left. */
  BSP_WIFI_SDIO_CLK_ENABLE();
  __HAL_RCC_SDMMC1_FORCE_RESET();
  __HAL_RCC_SDMMC1_RELEASE_RESET();

  BSP_WIFI_SDIO_DATA_CLK_ENABLE();
  BSP_WIFI_SDIO_CMD_CLK_ENABLE();

  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
  gpio_init_structure.Speed     = GPIO_SPEED_FREQ_HIGH;
  gpio_init_structure.Alternate = BSP_WIFI_SDIO_AF;

  gpio_init_structure.Pull = BSP_SD_DATA_PULL;
  gpio_init_structure.Pin  = BSP_WIFI_SDIO_D0_D3_PIN;
  HAL_GPIO_Init(BSP_WIFI_SDIO_DATA_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin = BSP_WIFI_SDIO_CMD_PIN;
  HAL_GPIO_Init(BSP_WIFI_SDIO_CMD_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Pin  = BSP_WIFI_SDIO_CK_PIN;
  HAL_GPIO_Init(BSP_WIFI_SDIO_DATA_GPIO_PORT, &gpio_init_structure);

  return BSP_ERROR_NONE;
}

/* Smallest transfer divider that keeps the bus at or under 25 MHz. */
static uint32_t Sd_DefaultSpeedDiv(void)
{
  const uint32_t kernel_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SDMMC1);

  return (kernel_hz + (2U * SD_DEFAULT_SPEED_HZ) - 1U) / (2U * SD_DEFAULT_SPEED_HZ);
}

/* Read back first, so the last write completes before the clock stops. */
static void Sd_StopHostClock(void)
{
  if (BSP_WIFI_SDIO_IS_CLK_ENABLED() != 0U)
  {
    (void)READ_REG(SDMMC1->POWER);
    __DSB();
    BSP_WIFI_SDIO_CLK_DISABLE();
  }
}

/* 1 if the buffer is whole cache lines in AXISRAM1/2, so the IDMA can use it directly. */
static uint8_t Sd_IsDirect(const uint8_t *pData, uint32_t Bytes)
{
  const uint32_t start = (uint32_t)pData;
  const uint32_t base  = ((start & 0xF0000000UL) == 0x20000000UL) ? SD_DIRECT_NS_START
                                                                  : SD_DIRECT_S_START;

  if (((start | Bytes) & (SD_CACHE_LINE - 1U)) != 0U)
  {
    return 0U;
  }

  return ((start >= base) && ((start - base) < SD_DIRECT_SIZE) &&
          (Bytes <= (SD_DIRECT_SIZE - (start - base)))) ? 1U : 0U;
}

/* The HAL's interrupt handler, polled: the SDMMC interrupt line stays off. */
static void Sd_Poll(void)
{
  if ((hbsp_sd.Instance->STA & hbsp_sd.Instance->MASK) != 0U)
  {
    HAL_SD_IRQHandler(&hbsp_sd);
  }
}

static int32_t Sd_WaitTransferState(void)
{
  const uint32_t start = HAL_GetTick();

  while (HAL_SD_GetCardState(&hbsp_sd) != HAL_SD_CARD_TRANSFER)
  {
    if ((HAL_GetTick() - start) >= BSP_SD_TRANSFER_TIMEOUT_MS)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
    BSP_SD_WaitHook();
  }

  return BSP_ERROR_NONE;
}

/* One command; pData is direct-capable or the bounce buffer. */
static int32_t Sd_Transfer(uint8_t *pData, uint32_t Block, uint32_t Count, uint8_t Write)
{
  const uint32_t bytes = Count * BSP_SD_BLOCK_SIZE;
  uint32_t start;
  HAL_StatusTypeDef status;

  Sd_Done   = 0U;
  Sd_Failed = 0U;

  /* The IDMA bypasses the D-cache: clean a source; clean and invalidate a destination
   * so no dirty line is evicted over it mid-transfer. */
  if (Write != 0U)
  {
    SCB_CleanDCache_by_Addr((volatile void *)pData, (int32_t)bytes);
    status = HAL_SD_WriteBlocks_DMA(&hbsp_sd, pData, Block, Count);
  }
  else
  {
    SCB_CleanInvalidateDCache_by_Addr((volatile void *)pData, (int32_t)bytes);
    status = HAL_SD_ReadBlocks_DMA(&hbsp_sd, pData, Block, Count);
  }

  if (status != HAL_OK)
  {
    return Sd_Fail();
  }

  /* The hardware data timeout is minutes at this clock: time out here. */
  start = HAL_GetTick();
  while ((Sd_Done == 0U) && (Sd_Failed == 0U))
  {
    Sd_Poll();
    if ((HAL_GetTick() - start) >= BSP_SD_TRANSFER_TIMEOUT_MS)
    {
      return Sd_Fail();
    }
    BSP_SD_WaitHook();
  }

  /* The HAL never unmasks the IDMA's transfer error: a refused access still ends the
   * transfer, with nothing moved. */
  if ((Sd_Failed != 0U) || ((hbsp_sd.Instance->STA & SDMMC_STA_IDMATE) != 0U))
  {
    return Sd_Fail();
  }

  if (Write == 0U)
  {
    SCB_InvalidateDCache_by_Addr((volatile void *)pData, (int32_t)bytes);
  }

  /* A write is complete only once the card has finished programming it. */
  if (Sd_WaitTransferState() != BSP_ERROR_NONE)
  {
    return Sd_Fail();
  }

  return BSP_ERROR_NONE;
}

/* After an error the card's and the HAL's states are unknown: stop the data path, reset
 * the peripheral and require BSP_SD_Init again. Bounded, unlike HAL_SD_Abort. */
static int32_t Sd_Fail(void)
{
  __HAL_SD_DISABLE_IT(&hbsp_sd, SDMMC_IT_DATAEND | SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT |
                                SDMMC_IT_TXUNDERR | SDMMC_IT_RXOVERR | SDMMC_IT_IDMABTC);
  hbsp_sd.Instance->IDMACTRL = SDMMC_DISABLE_IDMA;
  hbsp_sd.Instance->DCTRL    = 0U;
  __SDMMC_CMDTRANS_DISABLE(hbsp_sd.Instance);

  /* Read back so the writes complete before the reset and the clock gate. */
  (void)READ_REG(hbsp_sd.Instance->DCTRL);
  __DSB();
  __HAL_RCC_SDMMC1_FORCE_RESET();
  __HAL_RCC_SDMMC1_RELEASE_RESET();
  Sd_StopHostClock();

  hbsp_sd.State = HAL_SD_STATE_RESET;
  Sd_Ready      = 0U;

  return BSP_ERROR_PERIPH_FAILURE;
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
  UNUSED(hsd);
  Sd_Done = 1U;
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
  UNUSED(hsd);
  Sd_Done = 1U;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
  UNUSED(hsd);
  Sd_Failed = 1U;
}

int32_t BSP_SD_IsDetected(void)
{
#if defined(BSP_SD_DETECT_PORT) && defined(BSP_SD_DETECT_PIN)
  GPIO_InitTypeDef gpio_init_structure = {0};

  BSP_SD_DETECT_CLK_ENABLE();
  gpio_init_structure.Pin   = BSP_SD_DETECT_PIN;
  gpio_init_structure.Mode  = GPIO_MODE_INPUT;
  gpio_init_structure.Pull  = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BSP_SD_DETECT_PORT, &gpio_init_structure);

  return (HAL_GPIO_ReadPin(BSP_SD_DETECT_PORT, BSP_SD_DETECT_PIN) == (GPIO_PinState)BSP_SD_DETECT_ACTIVE)
         ? 1 : 0;
#else
  return 1;
#endif
}

int32_t BSP_SD_Init(void)
{
  int32_t  status;
  uint32_t divider;

  if (Sd_Ready != 0U)
  {
    return BSP_ERROR_NONE;
  }

  if (Sd_RadioPowered() != 0U)
  {
    return BSP_ERROR_BUSY;
  }

  if (BSP_SD_IsDetected() == 0)
  {
    return BSP_ERROR_UNKNOWN_COMPONENT;
  }

  Sd_HoldRadioOff();

  /* Polled: an interrupt the radio left enabled would hand the card's flags to its handler. */
  HAL_NVIC_DisableIRQ(BSP_WIFI_SDIO_IRQn);
  HAL_NVIC_ClearPendingIRQ(BSP_WIFI_SDIO_IRQn);

  status = Sd_ConfigureHost();
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  (void)memset(&hbsp_sd, 0, sizeof(hbsp_sd));
  hbsp_sd.Instance                 = SDMMC1;
  hbsp_sd.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
  hbsp_sd.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hbsp_sd.Init.BusWide             = SDMMC_BUS_WIDE_4B;
  hbsp_sd.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hbsp_sd.Init.ClockDiv            = Sd_DefaultSpeedDiv();
#if (USE_SD_TRANSCEIVER != 0U)
  hbsp_sd.Init.TranceiverPresent   = SDMMC_TRANSCEIVER_NOT_PRESENT;
#endif

  /* Identifies the card at 400 kHz, then sets the bus width at default speed. */
  if (HAL_SD_Init(&hbsp_sd) != HAL_OK)
  {
    /* Kept for BSP_SD_GetHandle: the DeInit clears it. */
    const uint32_t error = hbsp_sd.ErrorCode;

    (void)HAL_SD_DeInit(&hbsp_sd);
    Sd_StopHostClock();
    hbsp_sd.ErrorCode = error;
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  /* Above 25 MHz the card must switch to high speed first (CMD6); one that cannot stays
   * at default speed. */
  divider = BSP_SD_TRANSFER_DIV;
  if (divider < Sd_DefaultSpeedDiv())
  {
    if (HAL_SD_ConfigSpeedBusOperation(&hbsp_sd, SDMMC_SPEED_MODE_HIGH) != HAL_OK)
    {
      hbsp_sd.ErrorCode = HAL_SD_ERROR_NONE;
      divider = Sd_DefaultSpeedDiv();
    }
  }
  MODIFY_REG(hbsp_sd.Instance->CLKCR, SDMMC_CLKCR_CLKDIV, divider);

  Sd_Ready = 1U;

  return BSP_ERROR_NONE;
}

int32_t BSP_SD_DeInit(void)
{
  if (Sd_Ready == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  (void)HAL_SD_DeInit(&hbsp_sd);
  Sd_StopHostClock();
  Sd_Ready = 0U;

  return BSP_ERROR_NONE;
}

int32_t BSP_SD_GetCardInfo(BSP_SD_CardInfo_t *pInfo)
{
  HAL_SD_CardCIDTypeDef cid = {0};
  HAL_SD_CardStatusTypeDef card_status = {0};
  uint32_t kernel_hz;
  uint32_t divider;
  uint32_t index;

  if (pInfo == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Sd_Ready == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  (void)memset(pInfo, 0, sizeof(*pInfo));

  pInfo->CardType    = hbsp_sd.SdCard.CardType;
  pInfo->CardVersion = hbsp_sd.SdCard.CardVersion;
  pInfo->BlockCount  = hbsp_sd.SdCard.LogBlockNbr;
  for (index = 0U; index < 4U; index++)
  {
    pInfo->Cid[index] = hbsp_sd.CID[index];
    pInfo->Csd[index] = hbsp_sd.CSD[index];
  }

  if (HAL_SD_GetCardCID(&hbsp_sd, &cid) == HAL_OK)
  {
    pInfo->ManufacturerId = cid.ManufacturerID;
    pInfo->Product[0] = (char)((cid.ProdName1 >> 24) & 0xFFU);
    pInfo->Product[1] = (char)((cid.ProdName1 >> 16) & 0xFFU);
    pInfo->Product[2] = (char)((cid.ProdName1 >> 8) & 0xFFU);
    pInfo->Product[3] = (char)(cid.ProdName1 & 0xFFU);
    pInfo->Product[4] = (char)cid.ProdName2;
    pInfo->Serial     = cid.ProdSN;
  }

  pInfo->EraseBlocks = 1U;
  if ((HAL_SD_GetCardStatus(&hbsp_sd, &card_status) == HAL_OK) &&
      (Sd_AuBlocks[card_status.AllocationUnitSize & 0x0FU] != 0U))
  {
    pInfo->EraseBlocks = Sd_AuBlocks[card_status.AllocationUnitSize & 0x0FU];
  }

  /* The clock as programmed: kernel / (2 x CLKDIV), or the kernel itself at 0. */
  kernel_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SDMMC1);
  divider   = hbsp_sd.Instance->CLKCR & SDMMC_CLKCR_CLKDIV;
  pInfo->ClockHz = (divider == 0U) ? kernel_hz : (kernel_hz / (2U * divider));

  return BSP_ERROR_NONE;
}

int32_t BSP_SD_ReadBlocks(uint8_t *pData, uint32_t Block, uint32_t Count)
{
  uint32_t done = 0U;
  int32_t  status = BSP_ERROR_NONE;

  if ((pData == NULL) || (Count == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Sd_Ready == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (Sd_IsDirect(pData, Count * BSP_SD_BLOCK_SIZE) != 0U)
  {
    while ((done < Count) && (status == BSP_ERROR_NONE))
    {
      const uint32_t n = ((Count - done) < SD_MAX_BLOCKS_PER_COMMAND)
                         ? (Count - done) : SD_MAX_BLOCKS_PER_COMMAND;
      status = Sd_Transfer(&pData[done * BSP_SD_BLOCK_SIZE], Block + done, n, 0U);
      done += n;
    }
    return status;
  }

  while ((done < Count) && (status == BSP_ERROR_NONE))
  {
    const uint32_t n = ((Count - done) < BSP_SD_BOUNCE_BLOCKS) ? (Count - done) : BSP_SD_BOUNCE_BLOCKS;
    status = Sd_Transfer(Sd_Bounce, Block + done, n, 0U);
    if (status == BSP_ERROR_NONE)
    {
      (void)memcpy(&pData[done * BSP_SD_BLOCK_SIZE], Sd_Bounce, n * BSP_SD_BLOCK_SIZE);
    }
    done += n;
  }

  return status;
}

int32_t BSP_SD_WriteBlocks(const uint8_t *pData, uint32_t Block, uint32_t Count)
{
  uint32_t done = 0U;
  int32_t  status = BSP_ERROR_NONE;

  if ((pData == NULL) || (Count == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Sd_Ready == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (Sd_IsDirect(pData, Count * BSP_SD_BLOCK_SIZE) != 0U)
  {
    while ((done < Count) && (status == BSP_ERROR_NONE))
    {
      const uint32_t n = ((Count - done) < SD_MAX_BLOCKS_PER_COMMAND)
                         ? (Count - done) : SD_MAX_BLOCKS_PER_COMMAND;
      /* The HAL takes a non-const pointer; the IDMA only reads it. */
      status = Sd_Transfer((uint8_t *)(uintptr_t)&pData[done * BSP_SD_BLOCK_SIZE], Block + done, n, 1U);
      done += n;
    }
    return status;
  }

  while ((done < Count) && (status == BSP_ERROR_NONE))
  {
    const uint32_t n = ((Count - done) < BSP_SD_BOUNCE_BLOCKS) ? (Count - done) : BSP_SD_BOUNCE_BLOCKS;
    (void)memcpy(Sd_Bounce, &pData[done * BSP_SD_BLOCK_SIZE], n * BSP_SD_BLOCK_SIZE);
    status = Sd_Transfer(Sd_Bounce, Block + done, n, 1U);
    done += n;
  }

  return status;
}

SD_HandleTypeDef *BSP_SD_GetHandle(void)
{
  return &hbsp_sd;
}

#endif /* HAL_SD_MODULE_ENABLED */
