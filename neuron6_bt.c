/**
  * @file    neuron6_bt.c
  * @brief   BLE transport on the Neuro N6: HCI to the MAYA-W476 module over USART6.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6_bt.h"
#include "neuron6.h"
#include "neuron6_wifi.h"

#include <string.h>

#define BT_RING_MASK                (BSP_BT_RX_BUFFER_SIZE - 1U)

/* Header bytes after the type byte, with a one- or two-byte length field. */
#define BT_HEADER_SHORT             3U
#define BT_HEADER_LONG              4U

#define BT_ISO_LENGTH_MASK          0x3FFFU

#define BT_INFO_TIMEOUT_MS          1000U

/* RTS hysteresis, in free ring bytes; the margins cover interrupt latency. */
#define BT_RTS_STOP_FREE            32U
#define BT_RTS_GO_FREE              128U

/* A partial packet followed by this much silence has lost a byte. */
#define BT_RX_GAP_MS                20U

#if (BSP_BT_RX_BUFFER_SIZE < (2U * BT_RTS_GO_FREE))
#error "BSP_BT_RX_BUFFER_SIZE must leave room for the RTS margins"
#endif
#if ((BSP_BT_RX_BUFFER_SIZE & (BSP_BT_RX_BUFFER_SIZE - 1U)) != 0U)
#error "BSP_BT_RX_BUFFER_SIZE must be a power of two: the ring wraps by masking"
#endif
/* Highest BLE transmit power a command may set, in dBm; Wi-Fi has the same cap. */
#define BT_TX_POWER_CAP_DBM         8

/* NXP LE transmit power command: level, front-end loss flag, limit (signed dBm). */
#define BT_OP_NXP_LE_TX_POWER       0xFC87U

/* LE Transmitter Test [v4]; its last parameter is the power level. */
#define BT_OP_LE_TX_TEST_V4         0x207BU

/* Transmitter Test power value that asks for the controller's minimum. */
#define BT_TX_POWER_MIN             0x7EU

#define BT_OGF_VENDOR               0x3FU

#if (BSP_BT_MAX_PACKET_SIZE < (4U + 255U))
#error "BSP_BT_MAX_PACKET_SIZE must hold a command: four bytes of header and 255 of parameters"
#endif

/* HCI times are in 0.625 ms units (ms * 8 / 5); the limits below are in those units. */
#define BT_SCAN_UNIT_NUM            8U
#define BT_SCAN_UNIT_DEN            5U
#define BT_SCAN_MIN_UNITS           4U
#define BT_ADV_MIN_UNITS            0x20U
#define BT_TIME_MAX_UNITS           0x4000U

static UART_HandleTypeDef hbsp_bt_uart;

/* Filled by the receive interrupt, drained by the reader. */
static uint8_t Bt_RxRing[BSP_BT_RX_BUFFER_SIZE];

/* 1 while RTS holds the controller off. */
static volatile uint8_t Bt_RtsStopped;

static volatile uint32_t Bt_RxHead;

static volatile uint32_t Bt_RxTail;

/* When the last byte arrived (BSP_GetTick). */
static volatile uint32_t Bt_RxLastMs;

/* Target of the one-byte receive the interrupt re-arms. */
static uint8_t Bt_RxByte;

static uint8_t Bt_Ready;

static uint32_t Bt_BaudRate;

static volatile BSP_BT_Health_t Bt_Health;

/* Type of the packet being sent, possibly across calls; 0 between packets. */
static uint8_t Bt_TxType;

/* A data packet's type byte and header, as far as they have come. */
static uint8_t Bt_TxHead[1U + BT_HEADER_LONG];

static uint32_t Bt_TxHeadHave;

/* Its payload bytes still to come. */
static uint32_t Bt_TxLeft;

/* A command is collected and sent only once whole. */
static uint8_t Bt_TxCmd[BSP_BT_MAX_PACKET_SIZE];

static uint32_t Bt_TxCmdHave;

static int32_t  Bt_ConfigureUart(uint32_t BaudRate);
static uint32_t Bt_RingCount(void);
static uint8_t  Bt_RingPeek(uint32_t Offset);
static void     Bt_RingDrop(uint32_t Count);
static void     Bt_RtsCheck(void);
static void     Bt_RtsRelease(void);
static uint32_t Bt_PacketLength(const uint8_t *pHeader, uint8_t Type);
static uint32_t Bt_HeaderLength(uint8_t Type);
static int32_t  Bt_TxPolicy(uint8_t *pCmd, uint32_t Length);
static int32_t  Bt_TxRaw(const uint8_t *pData, uint32_t Length, uint32_t Timeout_ms);
static void     Bt_RxComplete(UART_HandleTypeDef *huart);
static void     Bt_Error(UART_HandleTypeDef *huart);

/* Configure the UART at BaudRate (bit/s) and arm reception. */
static int32_t Bt_ConfigureUart(uint32_t BaudRate)
{
  hbsp_bt_uart.Instance                    = BSP_BT_UART;
  hbsp_bt_uart.Init.BaudRate               = BaudRate;
  hbsp_bt_uart.Init.WordLength             = UART_WORDLENGTH_8B;
  hbsp_bt_uart.Init.StopBits               = UART_STOPBITS_1;
  hbsp_bt_uart.Init.Parity                 = UART_PARITY_NONE;
  hbsp_bt_uart.Init.Mode                   = UART_MODE_TX_RX;
#if (USE_BSP_BT_FLOW_CONTROL == 1U)
  /* CTS only: RTS is driven by this module (Bt_RtsCheck). */
  hbsp_bt_uart.Init.HwFlowCtl              = UART_HWCONTROL_CTS;
#else
  hbsp_bt_uart.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
#endif
  hbsp_bt_uart.Init.OverSampling           = UART_OVERSAMPLING_16;
  hbsp_bt_uart.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  hbsp_bt_uart.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
  hbsp_bt_uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&hbsp_bt_uart) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1U)
  /* Registered after the initialisation, which puts the defaults back. */
  if ((HAL_UART_RegisterCallback(&hbsp_bt_uart, HAL_UART_RX_COMPLETE_CB_ID, Bt_RxComplete) != HAL_OK) ||
      (HAL_UART_RegisterCallback(&hbsp_bt_uart, HAL_UART_ERROR_CB_ID, Bt_Error) != HAL_OK))
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
#endif

  /* FIFO off: with it on, a packet's last bytes can wait there without an interrupt. */
  (void)HAL_UARTEx_SetTxFifoThreshold(&hbsp_bt_uart, UART_TXFIFO_THRESHOLD_1_8);
  (void)HAL_UARTEx_SetRxFifoThreshold(&hbsp_bt_uart, UART_RXFIFO_THRESHOLD_1_8);
  (void)HAL_UARTEx_DisableFifoMode(&hbsp_bt_uart);

  Bt_BaudRate = BaudRate;
  Bt_Health.BaudRate = BaudRate;

  if (HAL_UART_Receive_IT(&hbsp_bt_uart, &Bt_RxByte, 1U) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return BSP_ERROR_NONE;
}

static uint32_t Bt_RingCount(void)
{
  return (Bt_RxHead - Bt_RxTail) & BT_RING_MASK;
}

/* Offset counts from the oldest waiting byte; nothing is consumed. */
static uint8_t Bt_RingPeek(uint32_t Offset)
{
  return Bt_RxRing[(Bt_RxTail + Offset) & BT_RING_MASK];
}

static void Bt_RingDrop(uint32_t Count)
{
  Bt_RxTail = (Bt_RxTail + Count) & BT_RING_MASK;
  Bt_RtsRelease();
}

/* From the receive interrupt: hold the controller off before the ring fills. */
static void Bt_RtsCheck(void)
{
#if (USE_BSP_BT_FLOW_CONTROL == 1U)
  if ((Bt_RtsStopped == 0U) &&
      (((BSP_BT_RX_BUFFER_SIZE - 1U) - Bt_RingCount()) < BT_RTS_STOP_FREE))
  {
    Bt_RtsStopped = 1U;
    HAL_GPIO_WritePin(BSP_BT_FLOW_GPIO_PORT, BSP_BT_RTS_PIN, GPIO_PIN_SET);
  }
#endif
}

/* Let the controller send again once a reader has made room. */
static void Bt_RtsRelease(void)
{
#if (USE_BSP_BT_FLOW_CONTROL == 1U)
  uint32_t primask;

  if (Bt_RtsStopped == 0U)
  {
    return;
  }

  /* Interrupts off, so a stop the receive interrupt decides meanwhile is not undone. */
  primask = __get_PRIMASK();
  __disable_irq();
  if ((Bt_RtsStopped != 0U) &&
      (((BSP_BT_RX_BUFFER_SIZE - 1U) - Bt_RingCount()) >= BT_RTS_GO_FREE))
  {
    Bt_RtsStopped = 0U;
    HAL_GPIO_WritePin(BSP_BT_FLOW_GPIO_PORT, BSP_BT_RTS_PIN, GPIO_PIN_RESET);
  }
  __set_PRIMASK(primask);
#endif
}

/* Header bytes after the type byte; 0 for a type that cannot start a packet. */
static uint32_t Bt_HeaderLength(uint8_t Type)
{
  switch (Type)
  {
    case BSP_BT_PACKET_EVENT:
      /* Code and a one-byte length. */
      return 2U;

    case BSP_BT_PACKET_COMMAND:
    case BSP_BT_PACKET_SCO:
      /* Two-byte field and a one-byte length. */
      return BT_HEADER_SHORT;

    case BSP_BT_PACKET_ACL:
    case BSP_BT_PACKET_ISO:
      /* Two-byte field and a two-byte length. */
      return BT_HEADER_LONG;

    default:
      return 0U;
  }
}

/* Total length, type byte and header included. */
static uint32_t Bt_PacketLength(const uint8_t *pHeader, uint8_t Type)
{
  switch (Type)
  {
    case BSP_BT_PACKET_EVENT:
      return 3U + (uint32_t)pHeader[2];

    case BSP_BT_PACKET_COMMAND:
    case BSP_BT_PACKET_SCO:
      return 4U + (uint32_t)pHeader[3];

    case BSP_BT_PACKET_ACL:
      return 5U + (uint32_t)pHeader[3] + ((uint32_t)pHeader[4] << 8);

    case BSP_BT_PACKET_ISO:
      /* The top two bits are reserved; unmasked, the transport would wait forever. */
      return 5U + (((uint32_t)pHeader[3] + ((uint32_t)pHeader[4] << 8)) & BT_ISO_LENGTH_MASK);

    default:
      return 0U;
  }
}

/* Applied in place to a whole command; 0 = send, -1 = refuse.
 * BLE power <= BT_TX_POWER_CAP_DBM: 0xFC87 and 0x207B capped, 0xFC87's loss flag cleared.
 * Any other vendor command is refused: those can rewrite the module's calibration. */
static int32_t Bt_TxPolicy(uint8_t *pCmd, uint32_t Length)
{
  const uint16_t opcode = (uint16_t)((uint32_t)pCmd[1] | ((uint32_t)pCmd[2] << 8));
  uint8_t       *pParam = &pCmd[4];
  const uint32_t count  = Length - 4U;
  const int8_t   cap    = (int8_t)BT_TX_POWER_CAP_DBM;

  if (opcode == BT_OP_NXP_LE_TX_POWER)
  {
    if ((count >= 1U) && ((int8_t)pParam[0] > cap))
    {
      pParam[0] = (uint8_t)cap;
    }
    if (count >= 2U)
    {
      pParam[1] = 0U;
    }
    if ((count >= 3U) && ((int8_t)pParam[2] > cap))
    {
      pParam[2] = (uint8_t)cap;
    }
    return 0;
  }
  if ((opcode >> 10) == BT_OGF_VENDOR)
  {
    return -1;
  }
  if ((opcode == BT_OP_LE_TX_TEST_V4) && (count >= 8U) &&
      (pParam[count - 1U] != BT_TX_POWER_MIN) && ((int8_t)pParam[count - 1U] > cap))
  {
    pParam[count - 1U] = (uint8_t)cap;
  }
  return 0;
}

/* Put bytes on the link as they are. */
static int32_t Bt_TxRaw(const uint8_t *pData, uint32_t Length, uint32_t Timeout_ms)
{
  HAL_StatusTypeDef status;

  if (Length == 0U)
  {
    return BSP_ERROR_NONE;
  }

  /* The HAL takes a non-const pointer but only reads it. */
  status = HAL_UART_Transmit(&hbsp_bt_uart, (uint8_t *)(uintptr_t)pData,
                             (uint16_t)Length, Timeout_ms);

  if (status == HAL_TIMEOUT)
  {
    /* CTS held off: flush the byte the transmitter took, or it would go out ahead of
     * the next packet, and H4 cannot resynchronise. */
    __HAL_UART_SEND_REQ(&hbsp_bt_uart, UART_TXDATA_FLUSH_REQUEST);
    return BSP_ERROR_BUSY;
  }

  if (status != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  Bt_Health.BytesSent += Length;

  return BSP_ERROR_NONE;
}

int32_t BSP_BT_Init(void)
{
  RCC_PeriphCLKInitTypeDef periph_clk_init = {0};
  GPIO_InitTypeDef gpio_init_structure = {0};
  int32_t status;

  if (Bt_Ready != 0U)
  {
    return BSP_ERROR_NONE;
  }

  /* Reference counted with Wi-Fi, so neither radio power-cycles the other. */
  (void)BSP_WIFI_ModulePowerOn();

  periph_clk_init.PeriphClockSelection = RCC_PERIPHCLK_USART6;
  periph_clk_init.Usart6ClockSelection = RCC_USART6CLKSOURCE_IC9;
  periph_clk_init.ICSelection[RCC_IC9].ClockSelection = RCC_ICCLKSOURCE_PLL1;
  periph_clk_init.ICSelection[RCC_IC9].ClockDivider   = BSP_BT_KERNEL_DIV;

  if (HAL_RCCEx_PeriphCLKConfig(&periph_clk_init) != HAL_OK)
  {
    (void)BSP_WIFI_ModulePowerOff();
    return BSP_ERROR_CLOCK_FAILURE;
  }

  BSP_BT_UART_CLK_ENABLE();
  BSP_BT_DATA_CLK_ENABLE();

  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull      = GPIO_NOPULL;
  gpio_init_structure.Speed     = GPIO_SPEED_FREQ_LOW;
  gpio_init_structure.Alternate = BSP_BT_UART_AF;

  gpio_init_structure.Pin = BSP_BT_TX_PIN | BSP_BT_RX_PIN;
  HAL_GPIO_Init(BSP_BT_DATA_GPIO_PORT, &gpio_init_structure);

#if (USE_BSP_BT_FLOW_CONTROL == 1U)
  /* CTS (PD3) is also the thermal camera's power line. RTS is a GPIO, asserted low. */
  BSP_BT_FLOW_CLK_ENABLE();
  gpio_init_structure.Pin = BSP_BT_CTS_PIN;
  HAL_GPIO_Init(BSP_BT_FLOW_GPIO_PORT, &gpio_init_structure);

  HAL_GPIO_WritePin(BSP_BT_FLOW_GPIO_PORT, BSP_BT_RTS_PIN, GPIO_PIN_RESET);
  gpio_init_structure.Pin  = BSP_BT_RTS_PIN;
  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(BSP_BT_FLOW_GPIO_PORT, &gpio_init_structure);
#endif

  Bt_RxHead = 0U;
  Bt_RxTail = 0U;
  Bt_RtsStopped = 0U;
  Bt_TxType = 0U;

  HAL_NVIC_SetPriority(BSP_BT_UART_IRQn, BSP_BT_IT_PRIORITY, 0x00U);
  HAL_NVIC_EnableIRQ(BSP_BT_UART_IRQn);

  status = Bt_ConfigureUart(BSP_BT_DEFAULT_BAUDRATE);

  if (status != BSP_ERROR_NONE)
  {
    /* Undo the above, as BSP_BT_DeInit does. */
    HAL_NVIC_DisableIRQ(BSP_BT_UART_IRQn);
    (void)HAL_UART_DeInit(&hbsp_bt_uart);
    HAL_GPIO_DeInit(BSP_BT_DATA_GPIO_PORT, BSP_BT_TX_PIN | BSP_BT_RX_PIN);
#if (USE_BSP_BT_FLOW_CONTROL == 1U)
    HAL_GPIO_DeInit(BSP_BT_FLOW_GPIO_PORT, BSP_BT_CTS_PIN | BSP_BT_RTS_PIN);
#endif
    /* Read back so the last write lands before the clock stops. */
    (void)READ_REG(BSP_BT_UART->CR1);
    __DSB();
    BSP_BT_UART_CLK_DISABLE();
    (void)BSP_WIFI_ModulePowerOff();
    return status;
  }

  Bt_Ready = 1U;

  return BSP_ERROR_NONE;
}

int32_t BSP_BT_DeInit(void)
{
  if (Bt_Ready == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  HAL_NVIC_DisableIRQ(BSP_BT_UART_IRQn);
  (void)HAL_UART_DeInit(&hbsp_bt_uart);

  HAL_GPIO_DeInit(BSP_BT_DATA_GPIO_PORT, BSP_BT_TX_PIN | BSP_BT_RX_PIN);
#if (USE_BSP_BT_FLOW_CONTROL == 1U)
  HAL_GPIO_DeInit(BSP_BT_FLOW_GPIO_PORT, BSP_BT_CTS_PIN | BSP_BT_RTS_PIN);
#endif

  /* Read back so the last write lands before the clock stops. */
  (void)READ_REG(BSP_BT_UART->CR1);
  __DSB();
  BSP_BT_UART_CLK_DISABLE();

  Bt_Ready = 0U;
  Bt_BaudRate = 0U;

  return BSP_WIFI_ModulePowerOff();
}

int32_t BSP_BT_SetBaudRate(uint32_t BaudRate)
{
  const uint32_t old = Bt_BaudRate;
  int32_t        status;

  if (Bt_Ready == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if (BaudRate == 0U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

#if (USE_BSP_BT_FLOW_CONTROL == 1U)
  /* Hold the controller off until reception is re-armed at the new speed. */
  Bt_RtsStopped = 1U;
  HAL_GPIO_WritePin(BSP_BT_FLOW_GPIO_PORT, BSP_BT_RTS_PIN, GPIO_PIN_SET);
#endif
  (void)HAL_UART_AbortReceive(&hbsp_bt_uart);
  (void)HAL_UART_DeInit(&hbsp_bt_uart);

  /* Bytes received at the old speed cannot be trusted. */
  Bt_RxHead = 0U;
  Bt_RxTail = 0U;

  status = Bt_ConfigureUart(BaudRate);
  if (status != BSP_ERROR_NONE)
  {
    /* A speed the port cannot take leaves it at the old one, not down. */
    (void)HAL_UART_DeInit(&hbsp_bt_uart);
    (void)Bt_ConfigureUart(old);
  }

  Bt_RtsStopped = 1U;
  Bt_RtsRelease();

  return status;
}

uint32_t BSP_BT_GetBaudRate(void)
{
  return Bt_BaudRate;
}

int32_t BSP_BT_Send(const uint8_t *pPacket, uint32_t Length, uint32_t Timeout_ms)
{
  uint32_t i      = 0U;
  int32_t  status = BSP_ERROR_NONE;

  if (Bt_Ready == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if ((pPacket == NULL) || (Length == 0U) || (Length > 0xFFFFU))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  while ((i < Length) && (status == BSP_ERROR_NONE))
  {
    if (Bt_TxType == 0U)
    {
      /* A packet starts here; the host never sends events. */
      const uint8_t type = pPacket[i];

      if ((Bt_HeaderLength(type) == 0U) || (type == BSP_BT_PACKET_EVENT))
      {
        return BSP_ERROR_WRONG_PARAM;
      }
      Bt_TxType     = type;
      Bt_TxHeadHave = 0U;
      Bt_TxCmdHave  = 0U;
    }

    if (Bt_TxType == BSP_BT_PACKET_COMMAND)
    {
      /* Collected until whole, then held to Bt_TxPolicy before any byte goes out. */
      const uint32_t whole = (Bt_TxCmdHave >= 4U) ?
                             Bt_PacketLength(Bt_TxCmd, BSP_BT_PACKET_COMMAND) : 4U;
      uint32_t       take  = whole - Bt_TxCmdHave;

      if (take > (Length - i))
      {
        take = Length - i;
      }
      (void)memcpy(&Bt_TxCmd[Bt_TxCmdHave], &pPacket[i], take);
      Bt_TxCmdHave += take;
      i            += take;
      if ((Bt_TxCmdHave >= 4U) &&
          (Bt_TxCmdHave == Bt_PacketLength(Bt_TxCmd, BSP_BT_PACKET_COMMAND)))
      {
        Bt_TxType = 0U;
        status = (Bt_TxPolicy(Bt_TxCmd, Bt_TxCmdHave) == 0) ?
                 Bt_TxRaw(Bt_TxCmd, Bt_TxCmdHave, Timeout_ms) : BSP_ERROR_FEATURE_NOT_SUPPORTED;
      }
    }
    else
    {
      /* Data goes out as it comes; its header says where it ends. */
      const uint32_t start = i;

      while ((i < Length) && (Bt_TxType != 0U))
      {
        const uint32_t head = 1U + Bt_HeaderLength(Bt_TxType);

        if (Bt_TxHeadHave < head)
        {
          Bt_TxHead[Bt_TxHeadHave] = pPacket[i];
          Bt_TxHeadHave++;
          i++;
          if (Bt_TxHeadHave == head)
          {
            Bt_TxLeft = Bt_PacketLength(Bt_TxHead, Bt_TxType) - head;
            if (Bt_TxLeft == 0U)
            {
              Bt_TxType = 0U;
            }
          }
        }
        else
        {
          uint32_t take = Length - i;

          if (take > Bt_TxLeft)
          {
            take = Bt_TxLeft;
          }
          i         += take;
          Bt_TxLeft -= take;
          if (Bt_TxLeft == 0U)
          {
            Bt_TxType = 0U;
          }
        }
      }
      status = Bt_TxRaw(&pPacket[start], i - start, Timeout_ms);
    }
  }

  if ((status != BSP_ERROR_NONE) && (status != BSP_ERROR_FEATURE_NOT_SUPPORTED))
  {
    /* Unknown how much went out: the next byte starts a new packet. */
    Bt_TxType = 0U;
  }
  return status;
}

int32_t BSP_BT_Receive(uint8_t *pPacket, uint32_t Size, uint32_t *pLength,
                       uint32_t Timeout_ms)
{
  uint32_t start;

  if (Bt_Ready == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  if ((pPacket == NULL) || (pLength == NULL) || (Size < BSP_BT_MAX_PACKET_SIZE))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Elapsed time, not a deadline, which would wrap for HAL_MAX_DELAY. */
  start = (uint32_t)BSP_GetTick();

  for (;;)
  {
    uint32_t available = Bt_RingCount();

    if (available >= 1U)
    {
      uint8_t  type = Bt_RingPeek(0U);
      uint32_t header = Bt_HeaderLength(type);

      /* Only events and ACL data come from the controller: drop anything else. */
      if ((header == 0U) ||
          ((type != BSP_BT_PACKET_EVENT) && (type != BSP_BT_PACKET_ACL)))
      {
        Bt_RingDrop(1U);
        Bt_Health.FramingErrors++;
        continue;
      }

      if (available >= (header + 1U))
      {
        uint8_t  head[BT_HEADER_LONG + 1U];
        uint32_t index;
        uint32_t total;

        for (index = 0U; index <= header; index++)
        {
          head[index] = Bt_RingPeek(index);
        }

        total = Bt_PacketLength(head, type);

        if ((total == 0U) || (total > BSP_BT_MAX_PACKET_SIZE))
        {
          /* An impossible length: not a header. Drop a byte and resynchronise. */
          Bt_RingDrop(1U);
          Bt_Health.FramingErrors++;
          continue;
        }

        if (available >= total)
        {
          for (index = 0U; index < total; index++)
          {
            pPacket[index] = Bt_RingPeek(index);
          }

          Bt_RingDrop(total);
          Bt_Health.PacketsReceived++;
          *pLength = total;

          return BSP_ERROR_NONE;
        }
      }

      /* A partial packet, then silence: a byte was lost. Drop the fragment, or the next
       * packet would be read as its rest (flow control never leaves such a gap). */
      if (((uint32_t)BSP_GetTick() - Bt_RxLastMs) > BT_RX_GAP_MS)
      {
        Bt_RingDrop(available);
        Bt_Health.FramingErrors++;
        continue;
      }
    }

    if (((uint32_t)BSP_GetTick() - start) >= Timeout_ms)
    {
      return BSP_ERROR_BUSY;
    }
#if defined(BSP_USE_FREERTOS) && (BSP_USE_FREERTOS == 1)
    /* Yield a tick rather than spin; the interrupt keeps filling the ring. */
    HAL_Delay(0U);
#endif
  }
}

int32_t BSP_BT_Command(uint16_t Opcode, const uint8_t *pParam, uint8_t Length,
                       uint8_t *pReturn, uint32_t Size, uint32_t Timeout_ms)
{
  uint8_t  packet[BSP_BT_MAX_PACKET_SIZE];
  uint32_t length   = 0U;
  uint32_t start    = (uint32_t)BSP_GetTick();
  int32_t  status;

  if ((pParam == NULL) && (Length != 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  packet[0] = BSP_BT_PACKET_COMMAND;
  packet[1] = (uint8_t)(Opcode & 0xFFU);
  packet[2] = (uint8_t)(Opcode >> 8);
  packet[3] = Length;
  if (Length != 0U)
  {
    (void)memcpy(&packet[4], pParam, Length);
  }
  status = BSP_BT_Send(packet, 4U + Length, Timeout_ms);

  /* Event 0x0E: length, count, opcode, status, return parameters. Others are dropped. */
  while (status == BSP_ERROR_NONE)
  {
    const uint32_t elapsed = (uint32_t)BSP_GetTick() - start;

    status = BSP_BT_Receive(packet, sizeof(packet), &length,
                            (elapsed < Timeout_ms) ? (Timeout_ms - elapsed) : 0U);
    if ((status == BSP_ERROR_NONE) && (length >= 7U) && (packet[0] == BSP_BT_PACKET_EVENT) &&
        (packet[1] == 0x0EU) && (packet[4] == (uint8_t)(Opcode & 0xFFU)) &&
        (packet[5] == (uint8_t)(Opcode >> 8)))
    {
      if (pReturn != NULL)
      {
        (void)memcpy(pReturn, &packet[7], ((length - 7U) < Size) ? (length - 7U) : Size);
      }
      return (packet[6] == 0x00U) ? BSP_ERROR_NONE : BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return status;
}

uint32_t BSP_BT_GetAvailable(void)
{
  return Bt_RingCount();
}

int32_t BSP_BT_Flush(void)
{
  if (Bt_Ready == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }

  Bt_RxTail = Bt_RxHead;
  Bt_RtsRelease();

  return BSP_ERROR_NONE;
}

int32_t BSP_BT_GetHealth(BSP_BT_Health_t *pHealth)
{
  if (pHealth == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  pHealth->BytesReceived   = Bt_Health.BytesReceived;
  pHealth->BytesSent       = Bt_Health.BytesSent;
  pHealth->PacketsReceived = Bt_Health.PacketsReceived;
  pHealth->Overflows       = Bt_Health.Overflows;
  pHealth->FramingErrors   = Bt_Health.FramingErrors;
  pHealth->UartErrors      = Bt_Health.UartErrors;
  pHealth->BaudRate        = Bt_Health.BaudRate;

  return BSP_ERROR_NONE;
}

static uint16_t Bt_Le16(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

int32_t BSP_BT_GetInfo(BSP_BT_Info_t *pInfo)
{
  uint8_t buf[8];
  int32_t status;

  if (pInfo == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  (void)memset(pInfo, 0, sizeof(BSP_BT_Info_t));

  /* HCI_Read_BD_ADDR. */
  status = BSP_BT_Command(0x1009U, NULL, 0U, buf, 6U, BT_INFO_TIMEOUT_MS);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }
  (void)memcpy(pInfo->BdAddr, buf, 6U);

  /* HCI_Read_Local_Version_Information. */
  status = BSP_BT_Command(0x1001U, NULL, 0U, buf, 8U, BT_INFO_TIMEOUT_MS);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }
  pInfo->HciVersion   = buf[0];
  pInfo->HciRevision  = Bt_Le16(&buf[1]);
  pInfo->LmpVersion   = buf[3];
  pInfo->Manufacturer = Bt_Le16(&buf[4]);
  pInfo->LmpSubversion = Bt_Le16(&buf[6]);

  /* HCI_LE_Read_Buffer_Size. */
  status = BSP_BT_Command(0x2002U, NULL, 0U, buf, 3U, BT_INFO_TIMEOUT_MS);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }
  pInfo->LeAclDataLength = Bt_Le16(&buf[0]);
  pInfo->LeAclPackets    = buf[2];

  /* HCI_LE_Read_Local_Supported_Features. */
  status = BSP_BT_Command(0x2003U, NULL, 0U, pInfo->LeFeatures, 8U,
                          BT_INFO_TIMEOUT_MS);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  /* Optional: a controller may refuse these, leaving the fields 0. */
  if (BSP_BT_Command(0x203AU, NULL, 0U, buf, 2U, BT_INFO_TIMEOUT_MS) == BSP_ERROR_NONE)
  {
    pInfo->MaxAdvDataLength = Bt_Le16(&buf[0]);
  }
  if (BSP_BT_Command(0x203BU, NULL, 0U, buf, 1U, BT_INFO_TIMEOUT_MS) == BSP_ERROR_NONE)
  {
    pInfo->AdvSets = buf[0];
  }
  if (BSP_BT_Command(0x204BU, NULL, 0U, buf, 2U, BT_INFO_TIMEOUT_MS) == BSP_ERROR_NONE)
  {
    pInfo->TxPowerMin = (int8_t)buf[0];
    pInfo->TxPowerMax = (int8_t)buf[1];
  }

  return BSP_ERROR_NONE;
}

/* Advertising report event, kept whole and handed out one report per call. */
static uint8_t  Bt_ScanEvent[BSP_BT_MAX_PACKET_SIZE];
static uint32_t Bt_ScanOffset;
static uint32_t Bt_ScanLength;
static uint8_t  Bt_ScanRemaining;

/* After HCI_Reset LE meta events are masked, so nothing would be reported: set the
 * specification's default event mask plus bit 61 (LE meta event). */
static int32_t Bt_UnmaskEvents(void)
{
  static const uint8_t k_events[8]    = { 0xFFU, 0xFFU, 0xFFU, 0xFFU,
                                          0xFFU, 0x1FU, 0x00U, 0x20U };
  static const uint8_t k_le_events[8] = { 0x1FU, 0x00U, 0x00U, 0x00U,
                                          0x00U, 0x00U, 0x00U, 0x00U };
  int32_t status = BSP_BT_Command(0x0C01U, k_events, 8U, NULL, 0U,
                                  BT_INFO_TIMEOUT_MS);

  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  /* The LE event mask's default too: an earlier host may have narrowed it. */
  return BSP_BT_Command(0x2001U, k_le_events, 8U, NULL, 0U, BT_INFO_TIMEOUT_MS);
}

/* ms to 0.625 ms units, rounded; 0 outside MinUnits..BT_TIME_MAX_UNITS. */
static uint16_t Bt_TimeUnits(uint32_t Ms, uint32_t MinUnits)
{
  /* 64-bit, so a long time cannot wrap. */
  const uint64_t units = (((uint64_t)Ms * BT_SCAN_UNIT_NUM) + (BT_SCAN_UNIT_DEN / 2U)) /
                         BT_SCAN_UNIT_DEN;

  if ((units < MinUnits) || (units > BT_TIME_MAX_UNITS))
  {
    return 0U;
  }
  return (uint16_t)units;
}

/* Next report from Bt_ScanEvent; BSP_ERROR_BUSY when none is left. */
static int32_t Bt_ScanTake(BSP_BT_Advertisement_t *pAdv)
{
  const uint8_t *p;
  uint32_t       length;

  if (Bt_ScanRemaining == 0U)
  {
    return BSP_ERROR_BUSY;
  }

  /* A report is 10 bytes plus payload: event type, address type, address, data length,
   * data, RSSI. Checked against the received length, in case the count disagrees. */
  if ((Bt_ScanOffset + 10U) > Bt_ScanLength)
  {
    Bt_ScanRemaining = 0U;
    return BSP_ERROR_BUSY;
  }
  p = &Bt_ScanEvent[Bt_ScanOffset];
  if ((Bt_ScanOffset + 10U + (uint32_t)p[8]) > Bt_ScanLength)
  {
    Bt_ScanRemaining = 0U;
    return BSP_ERROR_BUSY;
  }
  Bt_ScanRemaining--;

  length = p[8];
  if (length > BSP_BT_ADV_DATA_MAX)
  {
    length = BSP_BT_ADV_DATA_MAX;
  }

  pAdv->EventType   = p[0];
  pAdv->AddressType = p[1];
  (void)memcpy(pAdv->Address, &p[2], 6U);
  pAdv->DataLength = (uint8_t)length;
  (void)memset(pAdv->Data, 0, BSP_BT_ADV_DATA_MAX);
  (void)memcpy(pAdv->Data, &p[9], length);
  pAdv->Rssi = (int8_t)p[9U + p[8]];

  Bt_ScanOffset += 10U + (uint32_t)p[8];

  return BSP_ERROR_NONE;
}

int32_t BSP_BT_ScanStart(uint8_t Active, uint32_t IntervalMs, uint32_t WindowMs)
{
  uint8_t  param[7];
  uint16_t interval = Bt_TimeUnits(IntervalMs, BT_SCAN_MIN_UNITS);
  uint16_t window   = Bt_TimeUnits(WindowMs, BT_SCAN_MIN_UNITS);
  int32_t  status;

  if ((interval == 0U) || (window == 0U) || (window > interval))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Drop reports left from an earlier scan. */
  Bt_ScanRemaining = 0U;
  Bt_ScanOffset    = 0U;
  Bt_ScanLength    = 0U;

  status = Bt_UnmaskEvents();
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  param[0] = (Active != 0U) ? 1U : 0U;   /* active or passive          */
  param[1] = (uint8_t)(interval & 0xFFU);
  param[2] = (uint8_t)(interval >> 8);
  param[3] = (uint8_t)(window & 0xFFU);
  param[4] = (uint8_t)(window >> 8);
  param[5] = 0U;                         /* this controller's public address */
  param[6] = 0U;                         /* report every advertiser          */

  /* HCI_LE_Set_Scan_Parameters is taken only while scanning is off, so stop it first
   * (harmless if idle); then HCI_LE_Set_Scan_Enable. */
  (void)BSP_BT_ScanStop();
  status = BSP_BT_Command(0x200BU, param, 7U, NULL, 0U, BT_INFO_TIMEOUT_MS);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  param[0] = 1U;                         /* scanning on                      */
  param[1] = 0U;                         /* report every advertisement, not once */

  return BSP_BT_Command(0x200CU, param, 2U, NULL, 0U, BT_INFO_TIMEOUT_MS);
}

int32_t BSP_BT_ScanStop(void)
{
  uint8_t param[2];

  param[0] = 0U;
  param[1] = 0U;

  return BSP_BT_Command(0x200CU, param, 2U, NULL, 0U, BT_INFO_TIMEOUT_MS);
}

int32_t BSP_BT_AdvertiseStart(uint8_t Connectable, uint32_t IntervalMs,
                              const uint8_t *pData, uint8_t Length)
{
  uint8_t        param[32];
  const uint16_t interval = Bt_TimeUnits(IntervalMs, BT_ADV_MIN_UNITS);
  int32_t        status;

  if ((interval == 0U) || (Length > BSP_BT_ADV_DATA_MAX))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  status = Bt_UnmaskEvents();
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  (void)memset(param, 0, sizeof(param));
  param[0]  = (uint8_t)(interval & 0xFFU);   /* shortest interval          */
  param[1]  = (uint8_t)(interval >> 8);
  param[2]  = param[0];                      /* and the longest, the same  */
  param[3]  = param[1];
  /* Connectable undirected, or advertising with no invitation. */
  param[4]  = (Connectable != 0U) ? 0x00U : 0x03U;
  param[5]  = 0U;                            /* this controller's public address */
  /* param[6..12] stay 0: no directed peer. */
  param[13] = 0x07U;                         /* all three advertising channels */
  param[14] = 0U;                            /* scans and connections from anyone */

  /* HCI_LE_Set_Advertising_Parameters is taken only while advertising is off. */
  (void)BSP_BT_AdvertiseStop();
  status = BSP_BT_Command(0x2006U, param, 15U, NULL, 0U, BT_INFO_TIMEOUT_MS);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  if (pData != NULL)
  {
    /* HCI_LE_Set_Advertising_Data is always 32 bytes: length, payload, padding. */
    (void)memset(param, 0, sizeof(param));
    param[0] = Length;
    if (Length > 0U)
    {
      (void)memcpy(&param[1], pData, Length);
    }
    status = BSP_BT_Command(0x2008U, param, 32U, NULL, 0U, BT_INFO_TIMEOUT_MS);
    if (status != BSP_ERROR_NONE)
    {
      return status;
    }
  }

  param[0] = 1U;

  return BSP_BT_Command(0x200AU, param, 1U, NULL, 0U, BT_INFO_TIMEOUT_MS);
}

int32_t BSP_BT_AdvertiseStop(void)
{
  uint8_t param = 0U;

  return BSP_BT_Command(0x200AU, &param, 1U, NULL, 0U, BT_INFO_TIMEOUT_MS);
}

int32_t BSP_BT_ScanNext(BSP_BT_Advertisement_t *pAdv, uint32_t Timeout_ms)
{
  uint32_t start = HAL_GetTick();

  if (pAdv == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Reports left in the current event come first. */
  if (Bt_ScanTake(pAdv) == BSP_ERROR_NONE)
  {
    return BSP_ERROR_NONE;
  }

  for (;;)
  {
    uint32_t elapsed = HAL_GetTick() - start;
    uint32_t length  = 0U;
    int32_t  status;

    if (elapsed >= Timeout_ms)
    {
      return BSP_ERROR_BUSY;
    }
    status = BSP_BT_Receive(Bt_ScanEvent, sizeof(Bt_ScanEvent), &length,
                            Timeout_ms - elapsed);
    if (status != BSP_ERROR_NONE)
    {
      return status;
    }

    /* Event packet, LE Meta (0x3E), Advertising Report (0x02), at least one report. */
    if ((length >= 6U) && (Bt_ScanEvent[0] == 0x04U) && (Bt_ScanEvent[1] == 0x3EU) &&
        (Bt_ScanEvent[3] == 0x02U) && (Bt_ScanEvent[4] > 0U))
    {
      Bt_ScanRemaining = Bt_ScanEvent[4];
      Bt_ScanOffset    = 5U;
      Bt_ScanLength    = length;
      if (Bt_ScanTake(pAdv) == BSP_ERROR_NONE)
      {
        return BSP_ERROR_NONE;
      }
    }
  }
}

void BSP_BT_IRQHandler(void)
{
  HAL_UART_IRQHandler(&hbsp_bt_uart);
}

__weak void BSP_BT_RxCallback(void)
{
}

__weak void BSP_BT_ErrorCallback(uint32_t ErrorCode)
{
  UNUSED(ErrorCode);
}

/* UART callbacks. Without USE_HAL_UART_REGISTER_CALLBACKS this module defines the HAL's
 * UART callbacks for USART6; an application with other UARTs should set it to 1. */

/* One byte received; reception is re-armed here. */
static void Bt_RxComplete(UART_HandleTypeDef *huart)
{
  uint32_t next = (Bt_RxHead + 1U) & BT_RING_MASK;

  Bt_RxLastMs = (uint32_t)BSP_GetTick();

  if (next != Bt_RxTail)
  {
    Bt_RxRing[Bt_RxHead] = Bt_RxByte;
    Bt_RxHead = next;
    Bt_Health.BytesReceived++;
  }
  else
  {
    /* Ring full (only without flow control): the byte, and its packet, are lost. */
    Bt_Health.Overflows++;
  }

  Bt_RtsCheck();

  BSP_BT_RxCallback();

  (void)HAL_UART_Receive_IT(huart, &Bt_RxByte, 1U);
}

/* An error aborts reception: re-arm, or the link goes quiet for good. */
static void Bt_Error(UART_HandleTypeDef *huart)
{
  Bt_Health.UartErrors++;

  BSP_BT_ErrorCallback(huart->ErrorCode);

  (void)HAL_UART_Receive_IT(huart, &Bt_RxByte, 1U);
}

#if !defined(USE_HAL_UART_REGISTER_CALLBACKS) || (USE_HAL_UART_REGISTER_CALLBACKS == 0U)

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == BSP_BT_UART)
  {
    Bt_RxComplete(huart);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == BSP_BT_UART)
  {
    Bt_Error(huart);
  }
}

#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
