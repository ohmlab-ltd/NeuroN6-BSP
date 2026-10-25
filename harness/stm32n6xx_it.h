/**
  ******************************************************************************
  * @file    stm32n6xx_it.h
  * @brief   The CubeIDE projects' interrupt handler declarations, for the
  *          compile harness. A project compiles against its own.
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

#ifndef __STM32N6xx_IT_H
#define __STM32N6xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SecureFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

/* Where a fault leaves the board (stm32n6xx_it.c): the bootloader unless the
 * application defines it. */
void BSP_FaultCallback(uint32_t Breadcrumb);

#ifdef __cplusplus
}
#endif

#endif /* __STM32N6xx_IT_H */
