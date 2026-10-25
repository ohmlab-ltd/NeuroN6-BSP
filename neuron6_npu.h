/**
  * @file    neuron6_npu.h
  * @brief   Execution environment for the Neural-ART NPU on the Neuro N6.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_NPU_H
#define NEURON6_NPU_H

#include <stddef.h>
#include <stdint.h>

#include "neuron6_conf.h"
#include "neuron6_errno.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief NPU RAM window base (AXISRAM3-6); the application's RAM ends here
  * (see docs/BOOT-CONTRACT.md). */
#define NEURON6_NPU_RAM_BASE        0x34200000U

/** @brief Size of the NPU RAM window in kilobytes. */
#define NEURON6_NPU_RAM_SIZE_KB     1792U

/** @brief Preemption priority of the four NPU interrupts; override in neuron6_conf.h. */
#ifndef BSP_NPU_IT_PRIORITY
#define BSP_NPU_IT_PRIORITY         5U
#endif

int32_t BSP_NPU_Init(void);
int32_t BSP_NPU_DeInit(void);
int32_t BSP_NPU_GetRamInfo(uint32_t *pBase, uint32_t *pSizeKB);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_NPU_H */
