/**
  * @file    platform.h
  * @brief   Porting layer of ST's VL53L5CX ultra lite driver: bus, waits, byte order.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef VL53L5CX_PLATFORM_H
#define VL53L5CX_PLATFORM_H

#include <stdint.h>
#include <string.h>

/* Filled by neuron6_ranging_vl53l5cx.c. Wait runs between bus transfers and
 * while the driver waits. */
typedef struct
{
  uint16_t address;                                  /* 8-bit I2C address */
  int32_t  (*Write)(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
  int32_t  (*Read)(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
  uint32_t (*GetTick)(void);
  void     (*Wait)(void);
} VL53L5CX_Platform;

/* One target per zone; distance, target status and target count only. A result
 * is then 320 bytes at 8 x 8 zones and 128 at 4 x 4. */
#define VL53L5CX_NB_TARGET_PER_ZONE   1U
#define VL53L5CX_DISABLE_AMBIENT_PER_SPAD
#define VL53L5CX_DISABLE_NB_SPADS_ENABLED
#define VL53L5CX_DISABLE_SIGNAL_PER_SPAD
#define VL53L5CX_DISABLE_RANGE_SIGMA_MM
#define VL53L5CX_DISABLE_REFLECTANCE_PERCENT
#define VL53L5CX_DISABLE_MOTION_INDICATOR

uint8_t RdByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value);
uint8_t WrByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value);
uint8_t RdMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values,
                uint32_t size);
uint8_t WrMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values,
                uint32_t size);
void    SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs);

#endif /* VL53L5CX_PLATFORM_H */
