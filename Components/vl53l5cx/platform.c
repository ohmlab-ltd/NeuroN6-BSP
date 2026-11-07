/**
  * @file    platform.c
  * @brief   Porting layer of ST's VL53L5CX ultra lite driver: bus, waits, byte order.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "platform.h"

/* Longest single transfer. The part takes a long one in pieces at rising register
 * addresses; one 32 KB firmware bank would outlast the bus's 1 s transfer timeout. */
#define VL53L5CX_CHUNK_BYTES   256U

uint8_t RdByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value)
{
  return RdMulti(p_platform, RegisterAdress, p_value, 1U);
}

uint8_t WrByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value)
{
  return WrMulti(p_platform, RegisterAdress, &value, 1U);
}

uint8_t RdMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values,
                uint32_t size)
{
  uint32_t done = 0U;

  while (done < size)
  {
    const uint32_t n = ((size - done) > VL53L5CX_CHUNK_BYTES) ? VL53L5CX_CHUNK_BYTES : (size - done);

    if (p_platform->Read(p_platform->address, (uint16_t)(RegisterAdress + done),
                         &p_values[done], (uint16_t)n) != 0)
    {
      return 1U;
    }
    done += n;
    if ((done < size) && (p_platform->Wait != NULL))
    {
      p_platform->Wait();
    }
  }
  return 0U;
}

uint8_t WrMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values,
                uint32_t size)
{
  uint32_t done = 0U;

  while (done < size)
  {
    const uint32_t n = ((size - done) > VL53L5CX_CHUNK_BYTES) ? VL53L5CX_CHUNK_BYTES : (size - done);

    if (p_platform->Write(p_platform->address, (uint16_t)(RegisterAdress + done),
                          &p_values[done], (uint16_t)n) != 0)
    {
      return 1U;
    }
    done += n;
    if ((done < size) && (p_platform->Wait != NULL))
    {
      p_platform->Wait();
    }
  }
  return 0U;
}

/* The part sends 32-bit words most significant byte first. */
void SwapBuffer(uint8_t *buffer, uint16_t size)
{
  for (uint32_t i = 0U; (i + 3U) < size; i += 4U)
  {
    const uint8_t b0 = buffer[i];
    const uint8_t b1 = buffer[i + 1U];

    buffer[i]      = buffer[i + 3U];
    buffer[i + 1U] = buffer[i + 2U];
    buffer[i + 2U] = b1;
    buffer[i + 3U] = b0;
  }
}

/* At least TimeMs, as HAL_Delay. */
uint8_t WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs)
{
  const uint32_t start = p_platform->GetTick();

  while ((p_platform->GetTick() - start) <= TimeMs)
  {
    if (p_platform->Wait != NULL)
    {
      p_platform->Wait();
    }
  }
  return 0U;
}
