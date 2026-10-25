/**
  * @file    neuron6_wifi.h
  * @brief   MAYA-W476 module on the Neuro N6: power control and the Wi-Fi SDIO transport.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_WIFI_H
#define NEURON6_WIFI_H

#include "neuron6_conf.h"
#include "neuron6_errno.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Module control pins */

/** @brief PDn, active low: the module's one power-down line for both radios. */
#define BSP_WIFI_PDN_PIN                    GPIO_PIN_5
#define BSP_WIFI_PDN_GPIO_PORT              GPIOB
#define BSP_WIFI_PDN_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()

/** @brief The module's WL_WAKE_IN (host to module); names follow the module's view. */
#define BSP_WIFI_WAKE_IN_PIN                GPIO_PIN_0
#define BSP_WIFI_WAKE_IN_GPIO_PORT          GPIOG
#define BSP_WIFI_WAKE_IN_CLK_ENABLE()       __HAL_RCC_GPIOG_CLK_ENABLE()

/** @brief The module's WL_WAKE_OUT (module to host). */
#define BSP_WIFI_WAKE_OUT_PIN               GPIO_PIN_7
#define BSP_WIFI_WAKE_OUT_GPIO_PORT         GPIOB
#define BSP_WIFI_WAKE_OUT_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define BSP_WIFI_WAKE_OUT_EXTI_IRQn         EXTI7_IRQn
#define BSP_WIFI_WAKE_OUT_EXTI_LINE         EXTI_LINE_7

/** @brief Narrowband (the 802.15.4 radio, not BLE) wake, host to module. */
#define BSP_WIFI_NB_WAKE_IN_PIN             GPIO_PIN_5
#define BSP_WIFI_NB_WAKE_IN_GPIO_PORT       GPIOE
#define BSP_WIFI_NB_WAKE_IN_CLK_ENABLE()    __HAL_RCC_GPIOE_CLK_ENABLE()

/** @brief Narrowband wake, module to host (an input). */
#define BSP_WIFI_NB_WAKE_OUT_PIN            GPIO_PIN_10
#define BSP_WIFI_NB_WAKE_OUT_GPIO_PORT      GPIOE
#define BSP_WIFI_NB_WAKE_OUT_CLK_ENABLE()   __HAL_RCC_GPIOE_CLK_ENABLE()

/* SDIO host */

#define BSP_WIFI_SDIO                       SDMMC1
#define BSP_WIFI_SDIO_CLK_ENABLE()          __HAL_RCC_SDMMC1_CLK_ENABLE()
#define BSP_WIFI_SDIO_CLK_DISABLE()         __HAL_RCC_SDMMC1_CLK_DISABLE()
#define BSP_WIFI_SDIO_IS_CLK_ENABLED()      __HAL_RCC_SDMMC1_IS_CLK_ENABLED()
#define BSP_WIFI_SDIO_IRQn                  SDMMC1_IRQn

#define BSP_WIFI_SDIO_D0_D3_PIN             (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11)
#define BSP_WIFI_SDIO_CK_PIN                GPIO_PIN_12
#define BSP_WIFI_SDIO_DATA_GPIO_PORT        GPIOC
#define BSP_WIFI_SDIO_DATA_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()

#define BSP_WIFI_SDIO_CMD_PIN               GPIO_PIN_2
#define BSP_WIFI_SDIO_CMD_GPIO_PORT         GPIOH
#define BSP_WIFI_SDIO_CMD_CLK_ENABLE()      __HAL_RCC_GPIOH_CLK_ENABLE()

#define BSP_WIFI_SDIO_AF                    GPIO_AF10_SDMMC1

/**
  * @brief  PLL1 divider for the SDIO kernel clock; 7 gives 114.3 MHz from 800 MHz.
  * @note   EMC: this and BSP_WIFI_SDIO_TRANSFER_DIV must stay odd, or bus harmonics land
  *         on 200 MHz. Emissions passed at 7 and 7; retest them after any change.
  */
#ifndef BSP_WIFI_SDIO_KERNEL_DIV
#define BSP_WIFI_SDIO_KERNEL_DIV            7U
#endif

/** @brief Divider during enumeration: 0x80 gives 446 kHz (specification: 400 kHz). */
#ifndef BSP_WIFI_SDIO_INIT_DIV
#define BSP_WIFI_SDIO_INIT_DIV              0x80U
#endif

/**
  * @brief  Transfer clock divider: 7 gives 8.163 MHz, about 2.4 MB/s of Wi-Fi (EMC: see
  *         BSP_WIFI_SDIO_KERNEL_DIV).
  * @note   The clock is never gated: the card answers on a data line that is sampled only
  *         while the clock runs.
  */
#ifndef BSP_WIFI_SDIO_TRANSFER_DIV
#define BSP_WIFI_SDIO_TRANSFER_DIV          0x07U
#endif

/** @brief SDIO interrupt priority; override in neuron6_conf.h. */
#ifndef BSP_WIFI_IT_PRIORITY
#define BSP_WIFI_IT_PRIORITY                5U
#endif

/** @brief PDn low time in a power cycle, ms (specification: 1 us; see below). */
#ifndef BSP_WIFI_POWER_DOWN_MS
#define BSP_WIFI_POWER_DOWN_MS              500U
#endif

/** @brief Wait after power-up before addressing the card, ms (specification: about 20); a
  *        download that starts before the supplies and secure boot settle fails. */
#ifndef BSP_WIFI_POWER_UP_MS
#define BSP_WIFI_POWER_UP_MS                1000U
#endif

/** @brief The card's Wi-Fi function number. */
#define BSP_WIFI_FUNCTION                   1U

/** @brief CCCR revision register address (function 0). */
#define BSP_WIFI_CCCR_REVISION              0x00U

/** @brief Flags: one card address for every byte (the module's data port is a queue). */
#define BSP_WIFI_XFER_FIXED_ADDRESS         0U

/** @brief Flags: advance the card address by one per byte. */
#define BSP_WIFI_XFER_INCREMENT_ADDRESS     1U

/* Firmware image */

/** @brief Radio firmware in memory-mapped flash (boot contract), read in place. */
#ifndef BSP_WIFI_FIRMWARE_BASE
#define BSP_WIFI_FIRMWARE_BASE              0x91E00000U
#endif

/** @brief Reserved region size; the image's own length is known only to the driver. */
#ifndef BSP_WIFI_FIRMWARE_REGION_SIZE
#define BSP_WIFI_FIRMWARE_REGION_SIZE       0x00200000U
#endif

/** @brief Power the module up, reference counted (BLE shares it). */
int32_t BSP_WIFI_ModulePowerOn(void);

/** @brief Release one user (BSP_ERROR_NO_INIT if none); the last one powers the module
  *        down, which loses the downloaded firmware. */
int32_t BSP_WIFI_ModulePowerOff(void);

/**
  * @brief  Count a user that powered the module itself (the Wi-Fi stack); touches no pin.
  * @note   Without it, the next BSP_WIFI_ModulePowerOn (BSP_BT_Init's) would cycle PDn
  *         and erase the loaded firmware. Release with BSP_WIFI_ModulePowerOff.
  */
int32_t BSP_WIFI_ModuleClaim(void);

/** @brief 1 if the module is powered, 0 if not. */
int32_t BSP_WIFI_ModuleIsPowered(void);

/** @brief Drive WL_WAKE_IN: 1 holds the radio awake, 0 lets it sleep. */
int32_t BSP_WIFI_SetWake(uint8_t Awake);

/**
  * @brief  Read WL_WAKE_OUT (1 high, 0 low); BSP_WIFI_WakeCallback reports its edges.
  * @note   Not a working wake source: the shipped driver cannot enable host sleep on the
  *         IW610, so the module never drives it; with no pull, its level means nothing.
  * @note   The pad is made an input on the first read.
  */
uint8_t BSP_WIFI_GetWakeOut(void);

/** @brief Firmware image address (memory-mapped flash) and reserved region size. */
int32_t BSP_WIFI_GetFirmware(const uint8_t **ppFirmware, uint32_t *pSize);

/** @brief 1 if the region's first word looks programmed; a partial image passes too. */
int32_t BSP_WIFI_IsFirmwarePresent(void);

/**
  * @brief  Power the module, start the SDIO host and enumerate the card: on return it is
  *         selected, 4-bit, at the transfer clock, Wi-Fi function and interrupt on.
  * @retval BSP_ERROR_UNKNOWN_COMPONENT if no card answered.
  */
int32_t BSP_WIFI_Init(void);

/** @brief Stop the SDIO host and release the module; BSP_ERROR_NO_INIT if not up. */
int32_t BSP_WIFI_DeInit(void);

/** @brief Read the CCCR revision: the cheapest check that enumeration worked. */
int32_t BSP_WIFI_GetCccrRevision(uint8_t *pRevision);

/** @brief Read one byte from a card register; Function 0 is the common registers. */
int32_t BSP_WIFI_ReadDirect(uint8_t Function, uint32_t Address, uint8_t *pValue);

/** @brief Write one byte to a card register; Function 0 is the common registers. */
int32_t BSP_WIFI_WriteDirect(uint8_t Function, uint32_t Address, uint8_t Value);

/**
  * @brief  Read a run of bytes, blocking and polled: BlockCount 1 for up to 512 bytes,
  *         or up to 511 blocks of a power of two up to 2048. Flags: BSP_WIFI_XFER_*.
  * @retval BSP_ERROR_WRONG_PARAM for a transfer one command cannot carry.
  */
int32_t BSP_WIFI_ReadExtended(uint8_t Function, uint32_t Address, uint8_t *pData,
                              uint32_t BlockCount, uint32_t BlockSize, uint32_t Flags);

/** @brief Write a run of bytes; arguments and limits as BSP_WIFI_ReadExtended. */
int32_t BSP_WIFI_WriteExtended(uint8_t Function, uint32_t Address, const uint8_t *pData,
                               uint32_t BlockCount, uint32_t BlockSize, uint32_t Flags);

/** @brief Set one card function's block size, in bytes. */
int32_t BSP_WIFI_SetBlockSize(uint8_t Function, uint16_t BlockSize);

int32_t BSP_WIFI_EnableFunction(uint8_t Function);

int32_t BSP_WIFI_EnableFunctionInterrupt(uint8_t Function);

int32_t BSP_WIFI_DisableFunctionInterrupt(uint8_t Function);

/**
  * @brief  Re-enable the card interrupt once a transaction has cleared its pending state.
  * @note   Never from the interrupt itself.
  */
int32_t BSP_WIFI_RearmCardInterrupt(void);

/** @brief SDIO handle for HAL calls not wrapped here; valid between Init and DeInit. */
SDIO_HandleTypeDef *BSP_WIFI_GetHandle(void);

/** @brief Call from the application's SDMMC1_IRQHandler. */
void BSP_WIFI_IRQHandler(void);

/** @brief Call from the application's EXTI7_IRQHandler, if it uses that line. */
void BSP_WIFI_WakeIRQHandler(void);

/** @brief Weak; runs in interrupt context with the card interrupt still masked. */
void BSP_WIFI_CardInterruptCallback(uint8_t Function);

/** @brief Weak; a WL_WAKE_OUT edge, in interrupt context. */
void BSP_WIFI_WakeCallback(void);

/** @brief Claim the SDIO link; weak and empty, override as for neuron6_bus.h's locks. */
void BSP_WIFI_Lock(void);

void BSP_WIFI_Unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_WIFI_H */
