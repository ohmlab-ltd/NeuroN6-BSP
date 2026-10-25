/**
  * @file    neuron6_wdg.h
  * @brief   Independent watchdog (IWDG) support for the Neuro N6.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_WDG_H
#define NEURON6_WDG_H

#include "neuron6_conf.h"
#include "neuron6_errno.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Early-wakeup interrupt support: off unless neuron6_conf.h enables it. */
#if !defined(USE_BSP_WDG_EWI)
#define USE_BSP_WDG_EWI                 0U
#endif /* USE_BSP_WDG_EWI */

/* Early-wakeup priority: highest by default, to preempt whatever stopped the refresh. */
#if !defined(BSP_WDG_IT_PRIORITY)
#define BSP_WDG_IT_PRIORITY             0U
#endif /* BSP_WDG_IT_PRIORITY */

/** @brief Timeout range: up to 4096 ticks of the LSI divided by 4 to 1024. */
#define BSP_WDG_TIMEOUT_MIN_MS          1UL
#define BSP_WDG_TIMEOUT_MAX_MS          ((1024UL * 4096UL * 1000UL) / LSI_VALUE)

int32_t BSP_WDG_Init   (uint32_t Timeout_ms);
int32_t BSP_WDG_Refresh(void);

#if (USE_BSP_WDG_EWI == 1)
int32_t BSP_WDG_InitEWI    (uint32_t Timeout_ms, uint32_t EarlyWarning_ms);
void    BSP_WDG_IRQHandler (void);
void    BSP_WDG_EWI_Callback(void);
#endif /* USE_BSP_WDG_EWI == 1 */

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_WDG_H */
