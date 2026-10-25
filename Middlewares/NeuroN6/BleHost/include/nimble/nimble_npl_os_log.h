/**
  ******************************************************************************
  * @file    nimble_npl_os_log.h
  * @brief   NimBLE's OS log header (included by nimble/nimble_npl_log.h).
  *
  *          Each generated function (e.g. BLE_HS_LOG_DEBUG()) prints through
  *          printf to the USB console only at or above its module's level in
  *          syscfg/syscfg.h (BLE_HS_LOG_LVL, BLE_EATT_LOG_LVL). The host logs
  *          every packet at DEBUG; printing that would stall it.
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

#ifndef NIMBLE_NPL_OS_LOG_H
#define NIMBLE_NPL_OS_LOG_H

#include <stdarg.h>
#include <stdio.h>

#include "syscfg/syscfg.h"

/* Mynewt's level numbers, as BLE_HS_LOG_LVL uses them. */
#define BLE_NPL_LOG_LEVEL_DEBUG     0
#define BLE_NPL_LOG_LEVEL_INFO      1
#define BLE_NPL_LOG_LEVEL_WARN      2
#define BLE_NPL_LOG_LEVEL_ERROR     3
#define BLE_NPL_LOG_LEVEL_CRITICAL  4

/* MYNEWT_VAL_<module>_LVL, e.g. MYNEWT_VAL_BLE_HS_LOG_LVL for BLE_HS_LOG. */
#define BLE_NPL_LOG_MODULE_LEVEL                                                \
    _BLE_NPL_LOG_CAT(MYNEWT_VAL_, _BLE_NPL_LOG_CAT(BLE_NPL_LOG_MODULE, _LVL))

/* One function per module and level; lvl is pasted, not expanded, into
 * BLE_NPL_LOG_LEVEL_<lvl>. */
#define BLE_NPL_LOG_IMPL(lvl)                                                   \
    static inline void                                                          \
    _BLE_NPL_LOG_CAT(BLE_NPL_LOG_MODULE, _BLE_NPL_LOG_CAT(_, lvl))(             \
        const char *fmt, ...)                                                   \
    {                                                                           \
        if (BLE_NPL_LOG_LEVEL_ ## lvl >= BLE_NPL_LOG_MODULE_LEVEL) {            \
            va_list args;                                                       \
            va_start(args, fmt);                                                \
            (void)vprintf(fmt, args);                                           \
            va_end(args);                                                       \
        }                                                                       \
    }

#endif /* NIMBLE_NPL_OS_LOG_H */
