/**
  ******************************************************************************
  * @file    ble_host.h
  * @brief   BLE host (Apache NimBLE) over the MAYA-W476's HCI link.
  *
  *          Applications use NimBLE's own API (host/ble_hs.h). Order:
  *            1. BleHost_Init(): radio firmware, HCI link, controller reset,
  *               NimBLE init (bonds kept in RAM).
  *            2. Set ble_hs_cfg and register GATT services.
  *            3. BleHost_Start(): NimBLE then calls ble_hs_cfg.sync_cb.
  *
  *          FreeRTOS only. Needs the MAYA-W476 Wi-Fi library, which loads the
  *          radio firmware. See Examples/BleBeacon_FreeRTOS.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  *
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  *
  ******************************************************************************
  */

#ifndef BLE_HOST_H
#define BLE_HOST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief HCI link counters. */
typedef struct
{
  uint32_t CommandsSent;    /*!< HCI commands sent to the controller.     */
  uint32_t AclSent;         /*!< Data packets sent to the controller.     */
  uint32_t EventsReceived;  /*!< Events delivered to the host.            */
  uint32_t AclReceived;     /*!< Data packets delivered to the host.      */
  uint32_t EventsDropped;   /*!< Advertising reports dropped (no buffer). */
  uint32_t AclDropped;      /*!< Data packets dropped (no buffer).        */
  uint32_t SendFailures;    /*!< Packets the link did not take in time.   */
  uint32_t Unrecognised;    /*!< Packets of a type the host does not use. */
} BleHost_Stats_t;

/**
  * @brief  Load the radio firmware, raise the HCI link, reset the controller
  *         and initialise NimBLE.
  * @note   Call once, from a task, before registering services.
  * @retval BSP_ERROR_NONE; BSP_ERROR_COMPONENT_FAILURE if the radio firmware
  *         did not load; BSP_ERROR_BUSY if the controller did not answer its
  *         reset;
  *         otherwise BSP_BT_Init()'s error.
  */
int32_t BleHost_Init(void);

/**
  * @brief  Start the host task (NimBLE's event queue).
  * @note   Call after BleHost_Init() and after registering services.
  *         ble_hs_cfg.sync_cb then runs on the host task.
  * @retval BSP_ERROR_NONE; BSP_ERROR_NO_INIT before BleHost_Init();
  *         BSP_ERROR_BUSY if already started; BSP_ERROR_PERIPH_FAILURE if the
  *         task could not be created.
  */
int32_t BleHost_Start(void);

/** @brief Copy the HCI link counters to pStats. */
void BleHost_GetStats(BleHost_Stats_t *pStats);

#ifdef __cplusplus
}
#endif

#endif /* BLE_HOST_H */
