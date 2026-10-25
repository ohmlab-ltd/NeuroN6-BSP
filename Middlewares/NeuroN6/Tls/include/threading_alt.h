/**
  ******************************************************************************
  * @file    threading_alt.h
  * @brief   Mbed TLS's FreeRTOS mutex type (MBEDTLS_THREADING_ALT). Mbed TLS
  *          includes this file by name; n6_tls.c implements the functions.
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

#ifndef THREADING_ALT_H
#define THREADING_ALT_H

#include "FreeRTOS.h"
#include "semphr.h"

/* Static storage: creating a mutex allocates nothing and cannot fail. */
typedef struct
{
  StaticSemaphore_t Storage;
  SemaphoreHandle_t Handle;   /* NULL until initialised, and after freeing */
} mbedtls_threading_mutex_t;

#endif /* THREADING_ALT_H */
