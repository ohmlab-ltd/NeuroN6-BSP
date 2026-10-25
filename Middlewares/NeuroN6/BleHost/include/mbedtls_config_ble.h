/**
  ******************************************************************************
  * @file    mbedtls_config_ble.h
  * @brief   Mbed TLS for NimBLE's pairing (nimble/host/src/ble_sm_alg.c) only:
  *          AES, AES-CMAC and ECDH on P-256.
  *          Select with MBEDTLS_CONFIG_FILE="mbedtls_config_ble.h".
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

#ifndef MBEDTLS_CONFIG_BLE_H
#define MBEDTLS_CONFIG_BLE_H

#include <stddef.h>

/* AES-128: confirm values and key derivation. Tables in flash, not RAM. */
#define MBEDTLS_AES_C
#define MBEDTLS_AES_ROM_TABLES

/* AES-CMAC: LE Secure Connections' f4, f5 and f6. */
#define MBEDTLS_CIPHER_C
#define MBEDTLS_CMAC_C

/* P-256 key pair and shared secret (LE Secure Connections). */
#define MBEDTLS_BIGNUM_C
#define MBEDTLS_ECP_C
#define MBEDTLS_ECP_DP_SECP256R1_ENABLED
#define MBEDTLS_ECP_NIST_OPTIM
#define MBEDTLS_ECDH_C

/* Allocations on the FreeRTOS heap (ble_host.c). */
#define MBEDTLS_PLATFORM_C
#define MBEDTLS_PLATFORM_MEMORY
#define MBEDTLS_PLATFORM_CALLOC_MACRO   BleHost_Calloc
#define MBEDTLS_PLATFORM_FREE_MACRO     BleHost_Free

void *BleHost_Calloc(size_t Count, size_t Size);
void  BleHost_Free(void *p);

#endif /* MBEDTLS_CONFIG_BLE_H */
