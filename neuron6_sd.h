/**
  * @file    neuron6_sd.h
  * @brief   SD card on SDMMC1, the bus the MAYA-W476 radio also uses.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_SD_H
#define NEURON6_SD_H

#include "neuron6_conf.h"
#include "neuron6_errno.h"

#ifdef __cplusplus
extern "C" {
#endif

/* - The card shares SDMMC1 with the radio: BSP_SD_Init holds the radio powered down, and
 *   refuses while it is powered (BSP_WIFI_DeInit and power it off first).
 * - Transfers run on the SDMMC's internal DMA and are polled, so no interrupt handler is
 *   needed. BSP_SD_WaitHook runs while a transfer waits.
 * - The bus signals at 3.3 V, 4 bits wide, at 33.3 MHz with the card in high-speed mode;
 *   a card without it runs at 22.2 MHz.
 * - After any transfer error the module resets: calls return BSP_ERROR_NO_INIT until
 *   BSP_SD_Init brings the card up again. */

/** @brief Bytes in a block; every transfer is whole blocks. */
#define BSP_SD_BLOCK_SIZE               512U

/** @brief Kernel clock divider from PLL1 (800 MHz); 6 gives 133.3 MHz. */
#ifndef BSP_SD_KERNEL_DIV
#define BSP_SD_KERNEL_DIV               6U
#endif

/** @brief   Transfer clock divider (kernel / (2 x this)); 2 gives 33.3 MHz. Above 25 MHz
  *          the card is switched to high-speed mode, or the clock held to 25 MHz. */
#ifndef BSP_SD_TRANSFER_DIV
#define BSP_SD_TRANSFER_DIV             2U
#endif

/** @brief Internal pull-ups on CMD and D0-D3 (the specification wants 10 to 90 kohm);
  *        GPIO_NOPULL when the carrier fits its own. */
#ifndef BSP_SD_DATA_PULL
#define BSP_SD_DATA_PULL                GPIO_PULLUP
#endif

/* Card detect: define BSP_SD_DETECT_PORT, _PIN, _CLK_ENABLE() and _ACTIVE (the pin level
 * with a card in) in neuron6_conf.h if the carrier has one. Without it a card is assumed. */

/** @brief Timeout in ms for one transfer (up to 64 KB), and again for the card to finish
  *        programming it. */
#ifndef BSP_SD_TRANSFER_TIMEOUT_MS
#define BSP_SD_TRANSFER_TIMEOUT_MS      2000U
#endif

/** @brief Blocks in the bounce buffer for transfers that cannot use the caller's memory. */
#ifndef BSP_SD_BOUNCE_BLOCKS
#define BSP_SD_BOUNCE_BLOCKS            32U
#endif

/** @brief The card as identified. */
typedef struct
{
  uint32_t CardType;       /*!< CARD_SDSC or CARD_SDHC_SDXC (the HAL's).          */
  uint32_t CardVersion;    /*!< CARD_V1_X or CARD_V2_X.                           */
  uint32_t BlockCount;     /*!< Capacity in BSP_SD_BLOCK_SIZE blocks.             */
  uint32_t EraseBlocks;    /*!< Erase unit in blocks (card status), 1 if unknown. */
  uint32_t ClockHz;        /*!< Bus clock in use.                                 */
  uint32_t Cid[4];         /*!< Card identification register, as read.            */
  uint32_t Csd[4];         /*!< Card-specific data register, as read.             */
  uint8_t  ManufacturerId; /*!< From the CID.                                     */
  char     Product[6];     /*!< From the CID, NUL-terminated.                     */
  uint32_t Serial;         /*!< From the CID.                                     */
} BSP_SD_CardInfo_t;

/**
  * @brief  Hold the radio powered down, configure SDMMC1 and bring the card up.
  * @retval BSP_ERROR_BUSY if the radio is powered; BSP_ERROR_UNKNOWN_COMPONENT if no card
  *         is detected; BSP_ERROR_COMPONENT_FAILURE if the card does not answer.
  */
int32_t BSP_SD_Init(void);

/** @brief Release the card and stop SDMMC1's clock; the radio may then be powered. */
int32_t BSP_SD_DeInit(void);

/** @brief 1 with a card in (or no detect pin), 0 without. */
int32_t BSP_SD_IsDetected(void);

/** @brief The card's identity, capacity, erase unit and bus clock. */
int32_t BSP_SD_GetCardInfo(BSP_SD_CardInfo_t *pInfo);

/**
  * @brief  Read Count blocks from Block.
  * @note   Any buffer: one 32-byte aligned, a whole number of cache lines and in AXISRAM1/2
  *         is read into directly; any other goes through the bounce buffer.
  */
int32_t BSP_SD_ReadBlocks(uint8_t *pData, uint32_t Block, uint32_t Count);

/** @brief Write Count blocks at Block; returns once the card has finished programming. Any
  *        buffer, as BSP_SD_ReadBlocks. */
int32_t BSP_SD_WriteBlocks(const uint8_t *pData, uint32_t Block, uint32_t Count);

/** @brief Weak, empty; called while a transfer waits: feed the watchdog, service USB. */
void BSP_SD_WaitHook(void);

#if defined(HAL_SD_MODULE_ENABLED)
/** @brief The HAL handle, for diagnostics; valid between Init and DeInit. */
SD_HandleTypeDef *BSP_SD_GetHandle(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_SD_H */
