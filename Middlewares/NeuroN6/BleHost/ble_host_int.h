/**
  ******************************************************************************
  * @file    ble_host_int.h
  * @brief   What ble_host.c and ble_hci_transport.c share.
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

#ifndef BLE_HOST_INT_H
#define BLE_HOST_INT_H

#include "ble_host.h"

/** @brief The HCI link counters, written by the transport. */
extern volatile BleHost_Stats_t BleHost_Counters;

#endif /* BLE_HOST_INT_H */
