/**
  * @file    neuron6_wifi.c
  * @brief   MAYA-W476 module on the Neuro N6: power control and the Wi-Fi SDIO transport.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6_wifi.h"
#include "neuron6.h"

/* OCR window offered during identification: bit 20 is 3.2 to 3.3 V. */
#define WIFI_OCR_VDD_32_33          (1UL << 20U)

/* Set in the card's reply once its supplies have settled. */
#define WIFI_OCR_READY              (1UL << 31U)

#define WIFI_COMMAND_TIMEOUT_MS     1000U

#define WIFI_OCR_POLL_ATTEMPTS      1000U

#define WIFI_RCA_POLL_ATTEMPTS      1000U

#define WIFI_TRANSFER_TIMEOUT_MS    1000U

#define WIFI_POWER_SETTLE_MS        20U

#define WIFI_CLOCK_SETTLE_MS        5U

#define WIFI_FLASH_ERASED_WORD      0xFFFFFFFFUL

static SDIO_HandleTypeDef hbsp_sdio;

static uint32_t Wifi_PowerUsers;

static uint8_t Wifi_PinsReady;

static uint8_t Wifi_CardReady;

static void    Wifi_ConfigureControlPins(void);
static int32_t Wifi_ConfigureHost(void);
static int32_t Wifi_WaitCommand(uint8_t ExpectResponse);
static int32_t Wifi_SendOpCond(uint32_t Argument, uint32_t *pResponse);
static int32_t Wifi_Enumerate(void);
static int32_t Wifi_SetTransferClock(void);
static int32_t Wifi_MapStatus(HAL_StatusTypeDef Status);
static void    Wifi_FunctionCallback(SDIO_HandleTypeDef *hsdio, uint32_t Function);
static void    Wifi_StopHostClock(void);
static void    Wifi_HoldCardInterrupt(void);

/* Stop the host clock after HAL_SDIO_DeInit, unless an application MspDeInit already did.
 * Read back first, so the last write lands before the clock stops. */
static void Wifi_StopHostClock(void)
{
  if (BSP_WIFI_SDIO_IS_CLK_ENABLED() != 0U)
  {
    (void)READ_REG(hbsp_sdio.Instance->POWER);
    __DSB();
    BSP_WIFI_SDIO_CLK_DISABLE();
  }
}

/* Mask the card interrupt for a polled transfer: the HAL's handler would take its DATAEND
 * and the poll would time out. A card interrupt stays pending until the re-arm after. */
static void Wifi_HoldCardInterrupt(void)
{
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  __HAL_SDIO_DISABLE_IT(&hbsp_sdio, SDMMC_IT_SDIOIT);
  __set_PRIMASK(primask);
}

/* Park the control pins; each level is written before the pin becomes an output. */
static void Wifi_ConfigureControlPins(void)
{
  GPIO_InitTypeDef gpio_init_structure = {0};

  if (Wifi_PinsReady != 0U)
  {
    return;
  }

  BSP_WIFI_PDN_CLK_ENABLE();
  BSP_WIFI_WAKE_IN_CLK_ENABLE();
  BSP_WIFI_WAKE_OUT_CLK_ENABLE();
  BSP_WIFI_NB_WAKE_IN_CLK_ENABLE();
  BSP_WIFI_NB_WAKE_OUT_CLK_ENABLE();

  HAL_GPIO_WritePin(BSP_WIFI_PDN_GPIO_PORT, BSP_WIFI_PDN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BSP_WIFI_NB_WAKE_IN_GPIO_PORT, BSP_WIFI_NB_WAKE_IN_PIN, GPIO_PIN_RESET);

  gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull  = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;

  gpio_init_structure.Pin = BSP_WIFI_PDN_PIN;
  HAL_GPIO_Init(BSP_WIFI_PDN_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin = BSP_WIFI_NB_WAKE_IN_PIN;
  HAL_GPIO_Init(BSP_WIFI_NB_WAKE_IN_GPIO_PORT, &gpio_init_structure);

  /* The module drives both wake-out lines, so they are read, never driven. */
  gpio_init_structure.Mode = GPIO_MODE_INPUT;
  gpio_init_structure.Pull = GPIO_NOPULL;

  gpio_init_structure.Pin = BSP_WIFI_WAKE_OUT_PIN;
  HAL_GPIO_Init(BSP_WIFI_WAKE_OUT_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin = BSP_WIFI_NB_WAKE_OUT_PIN;
  HAL_GPIO_Init(BSP_WIFI_NB_WAKE_OUT_GPIO_PORT, &gpio_init_structure);

  /* High, so the radio stays awake while the stack runs. */
  HAL_GPIO_WritePin(BSP_WIFI_WAKE_IN_GPIO_PORT, BSP_WIFI_WAKE_IN_PIN, GPIO_PIN_SET);
  gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;
  gpio_init_structure.Pin   = BSP_WIFI_WAKE_IN_PIN;
  HAL_GPIO_Init(BSP_WIFI_WAKE_IN_GPIO_PORT, &gpio_init_structure);

  Wifi_PinsReady = 1U;
}

/* Select the SDIO kernel clock and configure the pins. */
static int32_t Wifi_ConfigureHost(void)
{
  RCC_PeriphCLKInitTypeDef periph_clk_init = {0};
  GPIO_InitTypeDef gpio_init_structure = {0};

  periph_clk_init.PeriphClockSelection = RCC_PERIPHCLK_SDMMC1;
  periph_clk_init.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_IC4;
  periph_clk_init.ICSelection[RCC_IC4].ClockSelection = RCC_ICCLKSOURCE_PLL1;
  periph_clk_init.ICSelection[RCC_IC4].ClockDivider   = BSP_WIFI_SDIO_KERNEL_DIV;

  if (HAL_RCCEx_PeriphCLKConfig(&periph_clk_init) != HAL_OK)
  {
    return BSP_ERROR_CLOCK_FAILURE;
  }

  BSP_WIFI_SDIO_CLK_ENABLE();
  BSP_WIFI_SDIO_DATA_CLK_ENABLE();
  BSP_WIFI_SDIO_CMD_CLK_ENABLE();

  /* High, not very high, speed on every line: the EMC-tested setting (softer edges). */
  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull      = GPIO_NOPULL;
  gpio_init_structure.Speed     = GPIO_SPEED_FREQ_HIGH;
  gpio_init_structure.Alternate = BSP_WIFI_SDIO_AF;

  gpio_init_structure.Pin = BSP_WIFI_SDIO_D0_D3_PIN | BSP_WIFI_SDIO_CK_PIN;
  HAL_GPIO_Init(BSP_WIFI_SDIO_DATA_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin = BSP_WIFI_SDIO_CMD_PIN;
  HAL_GPIO_Init(BSP_WIFI_SDIO_CMD_GPIO_PORT, &gpio_init_structure);

  return BSP_ERROR_NONE;
}

/* Wait for a command to be sent or answered: 1 answered, 0 no answer, -1 CRC mismatch. */
static int32_t Wifi_WaitCommand(uint8_t ExpectResponse)
{
  uint32_t start = (uint32_t)BSP_GetTick();

  do
  {
    uint32_t status = BSP_WIFI_SDIO->STA;

    if (ExpectResponse != 0U)
    {
      if ((status & SDMMC_STA_CMDREND) != 0U)
      {
        return 1;
      }

      if ((status & SDMMC_STA_CCRCFAIL) != 0U)
      {
        /* The identification (CMD5) reply carries no CRC, so this always flags it. */
        return -1;
      }

      if ((status & SDMMC_STA_CTIMEOUT) != 0U)
      {
        return 0;
      }
    }
    else if ((status & SDMMC_STA_CMDSENT) != 0U)
    {
      return 1;
    }
    else
    {
      /* Keep waiting. */
    }
  } while (((uint32_t)BSP_GetTick() - start) < WIFI_COMMAND_TIMEOUT_MS);

  return 0;
}

/* CMD5: offer Argument's voltage window (0 to probe); 1 on an answer, 0 if none. */
static int32_t Wifi_SendOpCond(uint32_t Argument, uint32_t *pResponse)
{
  int32_t result;

  BSP_WIFI_SDIO->ICR = 0xFFFFFFFFU;
  BSP_WIFI_SDIO->ARG = Argument;
  BSP_WIFI_SDIO->CMD = (5UL << SDMMC_CMD_CMDINDEX_Pos) |
                       SDMMC_CMD_WAITRESP_0 |
                       SDMMC_CMD_CPSMEN;

  result = Wifi_WaitCommand(1U);

  if (result == 0)
  {
    return 0;
  }

  *pResponse = BSP_WIFI_SDIO->RESP1;
  BSP_WIFI_SDIO->ICR = 0xFFFFFFFFU;

  return 1;
}

/* Identify and select the card by hand: the driver's own identification opens by asking
 * the card to move its bus to a lower voltage. */
static int32_t Wifi_Enumerate(void)
{
  uint32_t response = 0U;
  uint32_t attempts;
  uint16_t address = 1U;
  uint32_t error;

  /* Slow bus (BSP_WIFI_SDIO_INIT_DIV) and power on. */
  BSP_WIFI_SDIO->CLKCR = (BSP_WIFI_SDIO_INIT_DIV << SDMMC_CLKCR_CLKDIV_Pos);
  BSP_WIFI_SDIO->POWER = SDMMC_POWER_PWRCTRL;
  HAL_Delay(WIFI_POWER_SETTLE_MS);

  /* Put the card into its idle state. This command has no reply. */
  BSP_WIFI_SDIO->ICR = 0xFFFFFFFFU;
  BSP_WIFI_SDIO->ARG = 0U;
  BSP_WIFI_SDIO->CMD = (0UL << SDMMC_CMD_CMDINDEX_Pos) | SDMMC_CMD_CPSMEN;
  (void)Wifi_WaitCommand(0U);
  BSP_WIFI_SDIO->ICR = 0xFFFFFFFFU;

  /* Probe: does anything answer, and does it have at least one function. */
  if (Wifi_SendOpCond(0U, &response) != 1)
  {
    return BSP_ERROR_UNKNOWN_COMPONENT;
  }

  if (((response >> 28) & 0x07U) == 0U)
  {
    return BSP_ERROR_UNKNOWN_COMPONENT;
  }

  /* Offer the supply window and wait for the card to say its rails are up. */
  for (attempts = 0U; attempts < WIFI_OCR_POLL_ATTEMPTS; attempts++)
  {
    if (Wifi_SendOpCond(WIFI_OCR_VDD_32_33, &response) != 1)
    {
      return BSP_ERROR_UNKNOWN_COMPONENT;
    }

    if ((response & WIFI_OCR_READY) != 0U)
    {
      break;
    }

    HAL_Delay(1U);
  }

  if ((response & WIFI_OCR_READY) == 0U)
  {
    return BSP_ERROR_UNKNOWN_COMPONENT;
  }

  /* Ask for an address; the card may refuse for a while after going idle, so retry. */
  for (attempts = 0U; attempts < WIFI_RCA_POLL_ATTEMPTS; attempts++)
  {
    error = SDMMC_CmdSetRelAdd(BSP_WIFI_SDIO, &address);

    if (error == HAL_SDIO_ERROR_NONE)
    {
      break;
    }

    if (error != SDMMC_ERROR_ILLEGAL_CMD)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }

    HAL_Delay(1U);
  }

  if (error != HAL_SDIO_ERROR_NONE)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* Select it. From here the card accepts data transactions. */
  if (SDMMC_CmdSelDesel(BSP_WIFI_SDIO, ((uint32_t)address) << 16U) != HAL_SDIO_ERROR_NONE)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* The driver was bypassed: set its handle to the state its identification leaves. */
  hbsp_sdio.State     = HAL_SDIO_STATE_READY;
  hbsp_sdio.ErrorCode = HAL_SDIO_ERROR_NONE;
  hbsp_sdio.Context   = SDIO_CONTEXT_NONE;

  if (HAL_SDIO_SetDataBusWidth(&hbsp_sdio, HAL_SDIO_4_WIRES_MODE) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return Wifi_SetTransferClock();
}

/* Switch to the transfer clock, keeping the bus width that shares CLKCR. */
static int32_t Wifi_SetTransferClock(void)
{
  uint32_t clock_control = BSP_WIFI_SDIO->CLKCR;

  clock_control &= ~SDMMC_CLKCR_CLKDIV_Msk;
  clock_control |= (BSP_WIFI_SDIO_TRANSFER_DIV << SDMMC_CLKCR_CLKDIV_Pos);

  /* Hardware flow control: the clock stretches when the peripheral's buffer runs dry. */
  clock_control |= SDMMC_CLKCR_HWFC_EN;

  BSP_WIFI_SDIO->CLKCR = clock_control;
  HAL_Delay(WIFI_CLOCK_SETTLE_MS);

  return BSP_ERROR_NONE;
}

static int32_t Wifi_MapStatus(HAL_StatusTypeDef Status)
{
  switch (Status)
  {
    case HAL_OK:
      return BSP_ERROR_NONE;

    case HAL_TIMEOUT:
      return BSP_ERROR_BUSY;

    default:
      return BSP_ERROR_BUS_TRANSACTION_FAILURE;
  }
}

/* Masked here: the card deasserts its interrupt only after a transaction, and none can be
 * made from an interrupt. */
static void Wifi_FunctionCallback(SDIO_HandleTypeDef *hsdio, uint32_t Function)
{
  __HAL_SDIO_DISABLE_IT(hsdio, SDMMC_IT_SDIOIT);
  __HAL_SDIO_CLEAR_FLAG(hsdio, SDMMC_FLAG_SDIOIT);

  BSP_WIFI_CardInterruptCallback((uint8_t)Function);
}

int32_t BSP_WIFI_ModulePowerOn(void)
{
  BSP_WIFI_Lock();

  if (Wifi_PowerUsers == 0U)
  {
    Wifi_ConfigureControlPins();

    /* A full power cycle every time, not just on the first bring-up. */
    HAL_GPIO_WritePin(BSP_WIFI_PDN_GPIO_PORT, BSP_WIFI_PDN_PIN, GPIO_PIN_RESET);
    HAL_Delay(BSP_WIFI_POWER_DOWN_MS);
    HAL_GPIO_WritePin(BSP_WIFI_PDN_GPIO_PORT, BSP_WIFI_PDN_PIN, GPIO_PIN_SET);
    HAL_Delay(BSP_WIFI_POWER_UP_MS);
  }

  Wifi_PowerUsers++;

  BSP_WIFI_Unlock();

  return BSP_ERROR_NONE;
}

int32_t BSP_WIFI_ModuleClaim(void)
{
  BSP_WIFI_Lock();
  Wifi_PowerUsers++;
  BSP_WIFI_Unlock();

  return BSP_ERROR_NONE;
}

int32_t BSP_WIFI_ModulePowerOff(void)
{
  int32_t status = BSP_ERROR_NONE;

  BSP_WIFI_Lock();

  if (Wifi_PowerUsers == 0U)
  {
    status = BSP_ERROR_NO_INIT;
  }
  else
  {
    Wifi_PowerUsers--;

    if (Wifi_PowerUsers == 0U)
    {
      HAL_GPIO_WritePin(BSP_WIFI_PDN_GPIO_PORT, BSP_WIFI_PDN_PIN, GPIO_PIN_RESET);
    }
  }

  BSP_WIFI_Unlock();

  return status;
}

int32_t BSP_WIFI_ModuleIsPowered(void)
{
  return (Wifi_PowerUsers != 0U) ? 1 : 0;
}

int32_t BSP_WIFI_SetWake(uint8_t Awake)
{
  HAL_GPIO_WritePin(BSP_WIFI_WAKE_IN_GPIO_PORT, BSP_WIFI_WAKE_IN_PIN,
                    (Awake != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  return BSP_ERROR_NONE;
}

uint8_t BSP_WIFI_GetWakeOut(void)
{
  /* Make it an input first: the Wi-Fi stack claims the module without this module's pin
   * set-up, and a pad in its reset state reads 0. */
  if (((BSP_WIFI_WAKE_OUT_GPIO_PORT->MODER >> (2U * POSITION_VAL(BSP_WIFI_WAKE_OUT_PIN))) & 3U) != 0U)
  {
    GPIO_InitTypeDef gpio_init_structure = {0};

    BSP_WIFI_WAKE_OUT_CLK_ENABLE();
    gpio_init_structure.Pin   = BSP_WIFI_WAKE_OUT_PIN;
    gpio_init_structure.Mode  = GPIO_MODE_INPUT;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BSP_WIFI_WAKE_OUT_GPIO_PORT, &gpio_init_structure);
  }

  return (HAL_GPIO_ReadPin(BSP_WIFI_WAKE_OUT_GPIO_PORT,
                           BSP_WIFI_WAKE_OUT_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

int32_t BSP_WIFI_GetFirmware(const uint8_t **ppFirmware, uint32_t *pSize)
{
  if ((ppFirmware == NULL) || (pSize == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  *ppFirmware = (const uint8_t *)BSP_WIFI_FIRMWARE_BASE;
  *pSize      = BSP_WIFI_FIRMWARE_REGION_SIZE;

  return BSP_ERROR_NONE;
}

int32_t BSP_WIFI_IsFirmwarePresent(void)
{
  const volatile uint32_t *image = (const volatile uint32_t *)BSP_WIFI_FIRMWARE_BASE;
  uint32_t first = image[0];

  /* Erased flash reads all ones; unmapped or unpowered, zero. */
  if ((first == WIFI_FLASH_ERASED_WORD) || (first == 0U))
  {
    return 0;
  }

  return 1;
}

int32_t BSP_WIFI_Init(void)
{
  int32_t status;

  BSP_WIFI_Lock();

  if (Wifi_CardReady != 0U)
  {
    BSP_WIFI_Unlock();
    return BSP_ERROR_NONE;
  }

  BSP_WIFI_Unlock();

  (void)BSP_WIFI_ModulePowerOn();

  BSP_WIFI_Lock();

  hbsp_sdio.Instance = BSP_WIFI_SDIO;
  /* The HAL never resets its count of enabled function interrupts: after DeInit and Init
   * it would count two, seek a second callback and never clear the card interrupt. */
  hbsp_sdio.IOInterruptNbr = 0U;
  hbsp_sdio.IOFunctionMask = 0U;

  status = Wifi_ConfigureHost();

  if (status == BSP_ERROR_NONE)
  {
    status = Wifi_Enumerate();
  }

  if (status == BSP_ERROR_NONE)
  {
    if (HAL_SDIO_EnableIOFunction(&hbsp_sdio, HAL_SDIO_FUNCTION_1) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
  }

  if (status == BSP_ERROR_NONE)
  {
    if (HAL_SDIO_EnableIOFunctionInterrupt(&hbsp_sdio, HAL_SDIO_FUNCTION_1) != HAL_OK)
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
  }

  if (status == BSP_ERROR_NONE)
  {
    /* Registering the callback and arming the card interrupt are card-side actions. */
    (void)HAL_SDIO_RegisterIOFunctionCallback(&hbsp_sdio, HAL_SDIO_FUNCTION_1,
                                              Wifi_FunctionCallback);

    HAL_NVIC_SetPriority(BSP_WIFI_SDIO_IRQn, BSP_WIFI_IT_PRIORITY, 0x00U);
    HAL_NVIC_EnableIRQ(BSP_WIFI_SDIO_IRQn);

    Wifi_CardReady = 1U;
  }
  else if (BSP_WIFI_SDIO_IS_CLK_ENABLED() != 0U)
  {
    /* Undo the host as BSP_WIFI_DeInit does, before the module powers off. */
    HAL_NVIC_DisableIRQ(BSP_WIFI_SDIO_IRQn);
    (void)HAL_SDIO_DeInit(&hbsp_sdio);
    Wifi_StopHostClock();
  }

  BSP_WIFI_Unlock();

  if (status != BSP_ERROR_NONE)
  {
    (void)BSP_WIFI_ModulePowerOff();
  }

  return status;
}

int32_t BSP_WIFI_DeInit(void)
{
  BSP_WIFI_Lock();

  if (Wifi_CardReady == 0U)
  {
    BSP_WIFI_Unlock();
    return BSP_ERROR_NO_INIT;
  }

  HAL_NVIC_DisableIRQ(BSP_WIFI_SDIO_IRQn);
  (void)HAL_SDIO_DeInit(&hbsp_sdio);
  Wifi_StopHostClock();

  Wifi_CardReady = 0U;

  BSP_WIFI_Unlock();

  return BSP_WIFI_ModulePowerOff();
}

int32_t BSP_WIFI_GetCccrRevision(uint8_t *pRevision)
{
  return BSP_WIFI_ReadDirect(0U, BSP_WIFI_CCCR_REVISION, pRevision);
}

int32_t BSP_WIFI_ReadDirect(uint8_t Function, uint32_t Address, uint8_t *pValue)
{
  HAL_SDIO_DirectCmd_TypeDef command;
  int32_t status;

  if (pValue == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Wifi_CardReady == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  command.Reg_Addr       = Address;
  command.ReadAfterWrite = HAL_SDIO_WRITE_ONLY;   /* Not used by a read. */
  command.IOFunctionNbr  = Function;

  BSP_WIFI_Lock();
  Wifi_HoldCardInterrupt();
  status = Wifi_MapStatus(HAL_SDIO_ReadDirect(&hbsp_sdio, &command, pValue));
  /* Re-arm after every transaction (only a transaction updates the card's pending state),
   * under the lock so no other transfer is being set up meanwhile. */
  (void)BSP_WIFI_RearmCardInterrupt();
  BSP_WIFI_Unlock();

  return status;
}

int32_t BSP_WIFI_WriteDirect(uint8_t Function, uint32_t Address, uint8_t Value)
{
  HAL_SDIO_DirectCmd_TypeDef command;
  int32_t status;

  if (Wifi_CardReady == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  command.Reg_Addr       = Address;
  command.ReadAfterWrite = HAL_SDIO_READ_AFTER_WRITE;
  command.IOFunctionNbr  = Function;

  BSP_WIFI_Lock();
  Wifi_HoldCardInterrupt();
  status = Wifi_MapStatus(HAL_SDIO_WriteDirect(&hbsp_sdio, &command, Value));
  (void)BSP_WIFI_RearmCardInterrupt();
  BSP_WIFI_Unlock();

  return status;
}

/* Whether one CMD53 can carry it: up to 512 bytes, or up to 511 blocks of a power of two
 * up to 2048. Anything else would leave the card part way through a transfer. */
static uint8_t Wifi_TransferFits(uint32_t BlockCount, uint32_t BlockSize)
{
  if ((BlockCount == 0U) || (BlockSize == 0U))
  {
    return 0U;
  }
  if (BlockCount == 1U)
  {
    return (BlockSize <= 512U) ? 1U : 0U;
  }
  return ((BlockCount <= 511U) && (BlockSize <= 2048U) &&
          ((BlockSize & (BlockSize - 1U)) == 0U)) ? 1U : 0U;
}

int32_t BSP_WIFI_ReadExtended(uint8_t Function, uint32_t Address, uint8_t *pData,
                              uint32_t BlockCount, uint32_t BlockSize, uint32_t Flags)
{
  HAL_SDIO_ExtendedCmd_TypeDef command;
  int32_t status;

  if ((pData == NULL) || (Wifi_TransferFits(BlockCount, BlockSize) == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Wifi_CardReady == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  command.Reg_Addr      = Address;
  command.OpCode        = (Flags == BSP_WIFI_XFER_INCREMENT_ADDRESS)
                          ? HAL_SDIO_OP_CODE_AUTO_INC : HAL_SDIO_OP_CODE_NO_INC;
  command.Block_Mode    = (BlockCount > 1U) ? HAL_SDIO_MODE_BLOCK : HAL_SDIO_MODE_BYTE;
  command.IOFunctionNbr = Function;

  BSP_WIFI_Lock();
  /* The driver reads the block size from the handle: set it under the lock. */
  hbsp_sdio.block_size = (uint16_t)BlockSize;
  Wifi_HoldCardInterrupt();
  status = Wifi_MapStatus(HAL_SDIO_ReadExtended(&hbsp_sdio, &command, pData,
                                                BlockCount * BlockSize,
                                                WIFI_TRANSFER_TIMEOUT_MS));
  (void)BSP_WIFI_RearmCardInterrupt();
  BSP_WIFI_Unlock();

  return status;
}

int32_t BSP_WIFI_WriteExtended(uint8_t Function, uint32_t Address, const uint8_t *pData,
                               uint32_t BlockCount, uint32_t BlockSize, uint32_t Flags)
{
  HAL_SDIO_ExtendedCmd_TypeDef command;
  int32_t status;

  if ((pData == NULL) || (Wifi_TransferFits(BlockCount, BlockSize) == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (Wifi_CardReady == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  command.Reg_Addr      = Address;
  command.OpCode        = (Flags == BSP_WIFI_XFER_INCREMENT_ADDRESS)
                          ? HAL_SDIO_OP_CODE_AUTO_INC : HAL_SDIO_OP_CODE_NO_INC;
  command.Block_Mode    = (BlockCount > 1U) ? HAL_SDIO_MODE_BLOCK : HAL_SDIO_MODE_BYTE;
  command.IOFunctionNbr = Function;

  BSP_WIFI_Lock();
  hbsp_sdio.block_size = (uint16_t)BlockSize;
  Wifi_HoldCardInterrupt();
  /* The HAL takes a non-const pointer but only reads it. */
  status = Wifi_MapStatus(HAL_SDIO_WriteExtended(&hbsp_sdio, &command,
                                                 (uint8_t *)(uintptr_t)pData,
                                                 BlockCount * BlockSize,
                                                 WIFI_TRANSFER_TIMEOUT_MS));
  (void)BSP_WIFI_RearmCardInterrupt();
  BSP_WIFI_Unlock();

  return status;
}

int32_t BSP_WIFI_SetBlockSize(uint8_t Function, uint16_t BlockSize)
{
  int32_t status;

  if (Wifi_CardReady == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  BSP_WIFI_Lock();
  status = Wifi_MapStatus(HAL_SDIO_SetBlockSize(&hbsp_sdio, Function, BlockSize));
  BSP_WIFI_Unlock();

  return status;
}

int32_t BSP_WIFI_EnableFunction(uint8_t Function)
{
  int32_t status;

  if (Wifi_CardReady == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  BSP_WIFI_Lock();
  status = Wifi_MapStatus(HAL_SDIO_EnableIOFunction(&hbsp_sdio, (uint32_t)Function));
  BSP_WIFI_Unlock();

  return status;
}

int32_t BSP_WIFI_EnableFunctionInterrupt(uint8_t Function)
{
  int32_t status;

  if (Wifi_CardReady == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  BSP_WIFI_Lock();
  status = Wifi_MapStatus(HAL_SDIO_EnableIOFunctionInterrupt(&hbsp_sdio, (uint32_t)Function));
  BSP_WIFI_Unlock();

  return status;
}

int32_t BSP_WIFI_DisableFunctionInterrupt(uint8_t Function)
{
  int32_t status;

  if (Wifi_CardReady == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  BSP_WIFI_Lock();
  status = Wifi_MapStatus(HAL_SDIO_DisableIOFunctionInterrupt(&hbsp_sdio, (uint32_t)Function));
  BSP_WIFI_Unlock();

  return status;
}

int32_t BSP_WIFI_RearmCardInterrupt(void)
{
  if (Wifi_CardReady == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  /* Interrupts off: the card interrupt also writes the mask. */
  {
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __HAL_SDIO_ENABLE_IT(&hbsp_sdio, SDMMC_IT_SDIOIT);
    __set_PRIMASK(primask);
  }

  return BSP_ERROR_NONE;
}

SDIO_HandleTypeDef *BSP_WIFI_GetHandle(void)
{
  return &hbsp_sdio;
}

void BSP_WIFI_IRQHandler(void)
{
  HAL_SDIO_IRQHandler(&hbsp_sdio);
}

void BSP_WIFI_WakeIRQHandler(void)
{
  const uint32_t mask = BSP_WIFI_WAKE_OUT_PIN;

  if (((EXTI->RPR1 & mask) | (EXTI->FPR1 & mask)) == 0U)
  {
    return;
  }

  EXTI->RPR1 = mask;
  EXTI->FPR1 = mask;

  BSP_WIFI_WakeCallback();
}

__weak void BSP_WIFI_CardInterruptCallback(uint8_t Function)
{
  UNUSED(Function);
}

__weak void BSP_WIFI_WakeCallback(void)
{
}

__weak void BSP_WIFI_Lock(void)
{
}

__weak void BSP_WIFI_Unlock(void)
{
}
