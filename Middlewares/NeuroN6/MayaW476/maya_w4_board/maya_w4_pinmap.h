/**
  ******************************************************************************
  * @file    maya_w4_pinmap.h
  * @brief   STM32N6 pins for the u-blox MAYA-W476 module.
  *
  * Names are module-side: WL_WAKE_IN is a module input (the host drives it),
  * WL_WAKE_OUT a module output. NB is 802.15.4/Thread, not BLE. PDn is the one
  * active-low power-down line (no WL_REG_ON/BT_REG_ON); BT wake/sleep is
  * in-band over the UART HCI link.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  *
  * SPDX-License-Identifier: MIT
  * This software is licensed under the terms in
  * Middlewares/NeuroN6/MayaW476/LICENSE-MIT.txt.
  */

#ifndef MAYA_W4_PINMAP_H
#define MAYA_W4_PINMAP_H

#include "main.h"               /* CubeMX-generated GPIO_Port / GPIO_Pin defines */
#include "stm32n6xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Neuro N6 carrier wiring; another carrier overrides it in main.h or with -D. */
#ifndef PDn_pin_MAYA_Pin
#define PDn_pin_MAYA_Pin                GPIO_PIN_5      /* PB5 - PDn (power-down, active low) */
#define PDn_pin_MAYA_GPIO_Port          GPIOB
#endif
#ifndef WL_WAKE_IN_MAYA_Pin
#define WL_WAKE_IN_MAYA_Pin             GPIO_PIN_0      /* PG0 - host->module WLAN wake (OUTPUT) */
#define WL_WAKE_IN_MAYA_GPIO_Port       GPIOG
#endif
#ifndef WL_WAKE_OUT_MAYA_Pin
#define WL_WAKE_OUT_MAYA_Pin            GPIO_PIN_7      /* PB7 - module->host WLAN wake (INPUT) */
#define WL_WAKE_OUT_MAYA_GPIO_Port      GPIOB
#endif
#ifndef NB_WAKE_IN_MAYA_Pin
#define NB_WAKE_IN_MAYA_Pin             GPIO_PIN_5      /* PE5 - 802.15.4 wake-in (reserved, unused for WiFi) */
#define NB_WAKE_IN_MAYA_GPIO_Port       GPIOE
#endif
#ifndef NB_WAKE_OUT_MAYA_Pin
#define NB_WAKE_OUT_MAYA_Pin            GPIO_PIN_10     /* PE10 - 802.15.4 wake-out (reserved, unused for WiFi) */
#define NB_WAKE_OUT_MAYA_GPIO_Port      GPIOE
#endif

/* Power and reset. */

/** PDn: power-down, active low, output push-pull. Low is a hard reset (all
 *  state lost), not a sleep. */
#define MAYA_PDn_PORT                   PDn_pin_MAYA_GPIO_Port
#define MAYA_PDn_PIN                    PDn_pin_MAYA_Pin

/* Wi-Fi wake pair (IW610 WLAN block). */

/** WL_WAKE_IN: host-to-module wake, output push-pull; high wakes Wi-Fi from
 *  power-save. */
#define MAYA_WL_WAKE_IN_PORT            WL_WAKE_IN_MAYA_GPIO_Port
#define MAYA_WL_WAKE_IN_PIN             WL_WAKE_IN_MAYA_Pin

/** WL_WAKE_OUT: module-to-host wake, input; unused, and left as found. */
#define MAYA_WL_WAKE_OUT_PORT           WL_WAKE_OUT_MAYA_GPIO_Port
#define MAYA_WL_WAKE_OUT_PIN            WL_WAKE_OUT_MAYA_Pin

/* Narrowband (802.15.4 / Thread) wake pair - reserved, unused for Wi-Fi+BLE. */

/** NB_WAKE_IN - host to module 802.15.4 wake. STM32 OUTPUT push-pull. */
#define MAYA_NB_WAKE_IN_PORT            NB_WAKE_IN_MAYA_GPIO_Port
#define MAYA_NB_WAKE_IN_PIN             NB_WAKE_IN_MAYA_Pin

/** NB_WAKE_OUT - module to host 802.15.4 wake. STM32 INPUT, EXTI. */
#define MAYA_NB_WAKE_OUT_PORT           NB_WAKE_OUT_MAYA_GPIO_Port
#define MAYA_NB_WAKE_OUT_PIN            NB_WAKE_OUT_MAYA_Pin

/* Data-path peripheral handles. */

/** SDMMC1 in SDIO mode - Wi-Fi data path over CMD52/CMD53. */
extern SDIO_HandleTypeDef hsdio1;
#define MAYA_WIFI_SDIO_HANDLE           (&hsdio1)

/* USART6 (BLE HCI H4) is private to neuron6_bt: use BSP_BT_Send. */

/* Power-up / power-down sequencing helpers. */

/** Drive PDn low to fully power down the module. */
static inline void MAYA_W4_AssertPowerDown(void)
{
    HAL_GPIO_WritePin(MAYA_PDn_PORT, MAYA_PDn_PIN, GPIO_PIN_RESET);
}

/** Release PDn (drive high) to bring the module out of power-down. */
static inline void MAYA_W4_ReleasePowerDown(void)
{
    HAL_GPIO_WritePin(MAYA_PDn_PORT, MAYA_PDn_PIN, GPIO_PIN_SET);
}

/** Wake the Wi-Fi block (pulse WL_WAKE_IN high). */
static inline void MAYA_W4_WakeWifi(void)
{
    HAL_GPIO_WritePin(MAYA_WL_WAKE_IN_PORT, MAYA_WL_WAKE_IN_PIN, GPIO_PIN_SET);
}

/** Allow the Wi-Fi block to re-enter PS (drop WL_WAKE_IN low). */
static inline void MAYA_W4_AllowWifiSleep(void)
{
    HAL_GPIO_WritePin(MAYA_WL_WAKE_IN_PORT, MAYA_WL_WAKE_IN_PIN, GPIO_PIN_RESET);
}

#ifdef __cplusplus
}
#endif

#endif /* MAYA_W4_PINMAP_H */
