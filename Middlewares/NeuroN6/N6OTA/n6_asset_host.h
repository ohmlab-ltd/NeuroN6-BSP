/**
 ******************************************************************************
 * @file    n6_asset_host.h
 * @brief   The asset store's host commands, AST0 / ASR0 / AST1 / AST2
 *          (n6_host_contract.h), as the USB task takes them.
 *
 * Pass each command received over the USB cable; the network never carries
 * these. Once a commit has staged the data, stop work in flight and call
 * n6_asset_apply(), which resets the board so the FSBL writes it.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 * This software is licensed under the terms in
 * Middlewares/NeuroN6/N6OTA/LICENSE-MIT.txt.
 */

#ifndef N6_ASSET_HOST_H
#define N6_ASSET_HOST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  N6_ASSET_HOST_NONE = 0,   /* not an asset command */
  N6_ASSET_HOST_TAKEN,      /* an asset command, applied */
  N6_ASSET_HOST_STAGED      /* a commit that verified and staged: call n6_asset_apply() */
} n6_asset_host_t;

/* Take one host command. AST0 starts an asset and ASR0 a record (either
 * discards a partial transfer); AST1 is a chunk at an explicit offset (a lost
 * one fails the commit's CRC); AST2 checks the CRC and stages what arrived. */
n6_asset_host_t n6_asset_host_command(const uint8_t *cmd, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* N6_ASSET_HOST_H */
