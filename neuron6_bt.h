/**
  * @file    neuron6_bt.h
  * @brief   BLE transport on the Neuro N6: HCI to the MAYA-W476 module over USART6.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_BT_H
#define NEURON6_BT_H

#include "neuron6_conf.h"
#include "neuron6_errno.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_BT_UART                     USART6
#define BSP_BT_UART_CLK_ENABLE()        __HAL_RCC_USART6_CLK_ENABLE()
#define BSP_BT_UART_CLK_DISABLE()       __HAL_RCC_USART6_CLK_DISABLE()
#define BSP_BT_UART_IRQn                USART6_IRQn

#define BSP_BT_TX_PIN                   GPIO_PIN_6
#define BSP_BT_RX_PIN                   GPIO_PIN_7
#define BSP_BT_DATA_GPIO_PORT           GPIOC
#define BSP_BT_DATA_CLK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE()

/** @brief PD3; also the thermal camera's power line. */
#define BSP_BT_CTS_PIN                  GPIO_PIN_3
#define BSP_BT_RTS_PIN                  GPIO_PIN_4
#define BSP_BT_FLOW_GPIO_PORT           GPIOD
#define BSP_BT_FLOW_CLK_ENABLE()        __HAL_RCC_GPIOD_CLK_ENABLE()

#define BSP_BT_UART_AF                  GPIO_AF7_USART6

/**
  * @brief  Hardware flow control, 1 by default: a host stack wakes a sleeping controller
  *         by releasing RTS. A controller that is not running (no firmware) then holds
  *         the link shut, and BSP_BT_Send returns BSP_ERROR_BUSY.
  * @note   RTS is a GPIO driven here; the USART's own RTS lets the controller overrun.
  */
#ifndef USE_BSP_BT_FLOW_CONTROL
#define USE_BSP_BT_FLOW_CONTROL         1U
#endif

/** @brief PLL1 divider for the USART6 kernel clock; 16 gives about 50 MHz. */
#ifndef BSP_BT_KERNEL_DIV
#define BSP_BT_KERNEL_DIV               16U
#endif

/** @brief Baud rate out of reset, fixed by the controller until its firmware runs. */
#ifndef BSP_BT_DEFAULT_BAUDRATE
#define BSP_BT_DEFAULT_BAUDRATE         115200U
#endif

/** @brief Receive ring size in bytes; must be a power of two. */
#ifndef BSP_BT_RX_BUFFER_SIZE
#define BSP_BT_RX_BUFFER_SIZE           1024U
#endif

/** @brief Longest received packet in bytes; 260 fits any event or full-length LE data. */
#ifndef BSP_BT_MAX_PACKET_SIZE
#define BSP_BT_MAX_PACKET_SIZE          260U
#endif

/** @brief Link interrupt priority, just below the module's SDIO interrupt. */
#ifndef BSP_BT_IT_PRIORITY
#define BSP_BT_IT_PRIORITY              6U
#endif

/** @brief H4 packet types: the first byte of every packet, passed on as part of it. */
#define BSP_BT_PACKET_COMMAND           0x01U /*!< Host to controller.        */
#define BSP_BT_PACKET_ACL               0x02U /*!< Data, either direction.    */
#define BSP_BT_PACKET_SCO               0x03U /*!< Voice, either direction.   */
#define BSP_BT_PACKET_EVENT             0x04U /*!< Controller to host.        */
#define BSP_BT_PACKET_ISO               0x05U /*!< Isochronous, either way.   */

/** @brief Link health counters. */
typedef struct
{
  uint32_t BytesReceived;
  uint32_t BytesSent;
  uint32_t PacketsReceived;
  uint32_t Overflows;       /*!< Bytes dropped because the ring was full. */
  uint32_t FramingErrors;   /*!< Bytes discarded hunting for a packet start. */
  uint32_t UartErrors;
  uint32_t BaudRate;        /*!< bit/s */
} BSP_BT_Health_t;

/** @brief The controller's identity and capabilities, from its HCI answers. */
typedef struct
{
  uint8_t  BdAddr[6];        /*!< Public address, least significant byte first. */
  uint8_t  HciVersion;       /*!< Core spec version: 9 is 5.0, 10 is 5.1, and so on. */
  uint16_t HciRevision;
  uint8_t  LmpVersion;
  uint16_t LmpSubversion;
  uint16_t Manufacturer;     /*!< Company identifier. */
  uint16_t LeAclDataLength;  /*!< Bytes in one of the controller's LE buffers. */
  uint8_t  LeAclPackets;     /*!< How many of those buffers it has. */
  uint8_t  LeFeatures[8];    /*!< LE feature bits, in specification order. */
  uint16_t MaxAdvDataLength; /*!< Longest advertising payload; 0 if not reported. */
  uint8_t  AdvSets;          /*!< Concurrent advertising sets; 0 if not reported. */
  int8_t   TxPowerMin;       /*!< dBm; 0 if not reported. */
  int8_t   TxPowerMax;       /*!< dBm; 0 if not reported. */
} BSP_BT_Info_t;

/** @brief Longest advertising payload, in bytes. */
#define BSP_BT_ADV_DATA_MAX             31U

/** @brief One advertising report. */
typedef struct
{
  uint8_t Address[6];  /*!< Least significant byte first. */
  uint8_t AddressType; /*!< 0 public, 1 random, 2 and 3 resolvable. */
  uint8_t EventType;   /*!< 0 connectable undirected, 1 connectable directed, 2 scannable
                         *   undirected, 3 non-connectable undirected, 4 scan response. */
  int8_t  Rssi;        /*!< dBm; 127 if not reported. */
  uint8_t DataLength;  /*!< Valid bytes in Data. */
  uint8_t Data[BSP_BT_ADV_DATA_MAX];
} BSP_BT_Advertisement_t;

/**
  * @brief  Power the module (shared with Wi-Fi) and open the link at the default speed.
  * @note   Does not load the module firmware, one image for Wi-Fi and BLE: call
  *         MayaW476_RadioStart() first (FreeRTOS only; no network needed). Without it
  *         every BSP_BT_Send times out.
  * @note   Send HCI_Reset (0x0C03) before any other command; until then the controller
  *         answers nothing.
  * @retval BSP_ERROR_CLOCK_FAILURE if the kernel clock could not be selected.
  */
int32_t BSP_BT_Init(void);

/** @brief Close the link and release the module; BSP_ERROR_NO_INIT if not open. */
int32_t BSP_BT_DeInit(void);

/**
  * @brief  Change the link speed (bit/s); buffered bytes are discarded.
  * @note   Tell the controller first and wait for its answer at the old speed. Speeds
  *         above the default need flow control.
  */
int32_t BSP_BT_SetBaudRate(uint32_t BaudRate);

/** @brief Current link speed in bit/s; 0 if the link is not open. */
uint32_t BSP_BT_GetBaudRate(void);

/**
  * @brief  Send one packet (type byte first, counted in Length); blocks up to Timeout_ms.
  * @note   A packet may span consecutive calls with nothing else sent between: data goes
  *         out as it comes (and may exceed BSP_BT_MAX_PACKET_SIZE), a command once whole.
  * @note   No command can set BLE transmit power above 8 dBm (0xFC87, 0x207B capped).
  * @note   Any other vendor command (OGF 0x3F) is refused, and no byte of it is sent.
  * @retval BSP_ERROR_FEATURE_NOT_SUPPORTED for a refused command, BSP_ERROR_BUSY on
  *         timeout, BSP_ERROR_WRONG_PARAM for an event or unknown packet type.
  */
int32_t BSP_BT_Send(const uint8_t *pPacket, uint32_t Length, uint32_t Timeout_ms);

/**
  * @brief  Take one whole packet (type byte first); Size >= BSP_BT_MAX_PACKET_SIZE.
  * @note   Yields under FreeRTOS while waiting; stray bytes are discarded and counted.
  * @retval BSP_ERROR_BUSY if no whole packet arrived within Timeout_ms.
  */
int32_t BSP_BT_Receive(uint8_t *pPacket, uint32_t Size, uint32_t *pLength,
                       uint32_t Timeout_ms);

/**
  * @brief  Send an HCI command and wait for its Command Complete; pReturn (or NULL) gets
  *         the return parameters after the status byte.
  * @note   Drops other packets meanwhile; sent through BSP_BT_Send and its power policy.
  * @retval BSP_ERROR_PERIPH_FAILURE for a non-zero status, BSP_ERROR_BUSY if no answer.
  */
int32_t BSP_BT_Command(uint16_t Opcode, const uint8_t *pParam, uint8_t Length,
                       uint8_t *pReturn, uint32_t Size, uint32_t Timeout_ms);

/** @brief Received bytes waiting (bytes, not whole packets). */
uint32_t BSP_BT_GetAvailable(void);

/** @brief Discard all buffered received bytes, e.g. after a controller reset. */
int32_t BSP_BT_Flush(void);

/** @brief Copy the link health counters. */
int32_t BSP_BT_GetHealth(BSP_BT_Health_t *pHealth);

/**
  * @brief  Query the controller's address, versions and LE capabilities (seven commands).
  * @note   Fields the controller will not report stay 0. Do not call while a host stack
  *         owns the link.
  */
int32_t BSP_BT_GetInfo(BSP_BT_Info_t *pInfo);

/**
  * @brief  Start scanning; Active 1 also requests scan responses, so it transmits.
  * @note   IntervalMs 3 to 10240, WindowMs 3 to IntervalMs (equal is continuous), both
  *         rounded to 0.625 ms.
  */
int32_t BSP_BT_ScanStart(uint8_t Active, uint32_t IntervalMs, uint32_t WindowMs);

/**
  * @brief  Take the next advertising report; BSP_ERROR_BUSY if none within Timeout_ms.
  * @note   Discards other packets, so nothing else may be waiting on the link.
  */
int32_t BSP_BT_ScanNext(BSP_BT_Advertisement_t *pAdv, uint32_t Timeout_ms);

/** @brief Stop scanning; queued reports stay until taken or the next BSP_BT_ScanStart. */
int32_t BSP_BT_ScanStop(void);

/**
  * @brief  Start advertising every IntervalMs, 20 to 10240 (rounded to 0.625 ms).
  * @param  Connectable 1 only with a host stack behind this transport (e.g. the NimBLE
  *         port in Middlewares/NeuroN6/BleHost): this module answers no connection.
  * @param  pData AD structures (length, type, data), up to BSP_BT_ADV_DATA_MAX bytes;
  *         NULL keeps the last payload.
  */
int32_t BSP_BT_AdvertiseStart(uint8_t Connectable, uint32_t IntervalMs,
                              const uint8_t *pData, uint8_t Length);

int32_t BSP_BT_AdvertiseStop(void);

/** @brief Call from the application's USART6_IRQHandler. */
void BSP_BT_IRQHandler(void);

/** @brief Weak; runs in interrupt context once per received byte: keep it short. */
void BSP_BT_RxCallback(void);

/** @brief Weak; runs in interrupt context with the UART's error code. */
void BSP_BT_ErrorCallback(uint32_t ErrorCode);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_BT_H */
