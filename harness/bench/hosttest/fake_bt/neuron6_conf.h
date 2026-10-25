/* Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 *
 * The board configuration the BLE host test builds neuron6_bt.c with:
 * bare metal, flow control on, the fake HAL. */
#ifndef NEURON6_CONF_H
#define NEURON6_CONF_H

#include "stm32n6xx_hal.h"

#define BSP_USE_FREERTOS          0
#define USE_BSP_BT_FLOW_CONTROL   1U

#endif /* NEURON6_CONF_H */
