
  @verbatim
  ******************************************************************************
  * @file    ohmlab_readme.txt
  * @author  Ohm Lab Ltd
  * @brief   This file lists the main modifications done by Ohm Lab Ltd on the
  *          FreeRTOS Kernel for the Neuro N6.
  ******************************************************************************
  * Copyright (c) 2025-2026 Ohm Lab Ltd
  * Copyright (C) 2021 Amazon.com, Inc. or its affiliates (base component)
  *
  * SPDX-License-Identifier: MIT
  *
  * The base component is the FreeRTOS Kernel, distributed under the MIT
  * licence; see LICENSE.md in this directory and the per-file SPDX headers
  * throughout Source/. The modifications described below are made available
  * under the same MIT terms.
  *
  * Contact: team@ohmlab.co.uk
  * Repository: https://github.com/ohmlab-ltd/Neuro-N6-BSP
  ******************************************************************************
  @endverbatim

  Base package
  ============
  FreeRTOS Kernel V10.6.2 (tskKERNEL_VERSION_NUMBER in Source/include/task.h),
  as redistributed by STMicroelectronics in the STM32Cube MCU Package for
  STM32N6 (Middlewares/Third_Party/FreeRTOS), including the CMSIS-RTOS v2
  wrapper under Source/CMSIS_RTOS_V2/.

  Only the single file listed below is modified. All other files in this tree
  are unmodified upstream sources. The kernel configuration lives outside this
  tree in Utilities/freertos/FreeRTOSConfig.h and is Ohm Lab Ltd's own file.


### Neuro N6 local patches ###
=========================
  Both changes below are in
    \Source\portable\GCC\ARM_CM55_NTZ\non_secure\port.c

  They exist because this core deliberately runs USB enumeration and CDC
  bring-up BEFORE vTaskStartScheduler(), which the stock port does not
  anticipate. Both are initialiser changes only; xPortStartScheduler()
  overwrites each value before the first task runs, so post-scheduler
  behaviour is identical to stock.

  + Initialise ulCriticalNesting to 0 instead of the 0xaaaaaaaa sentinel
     (around line 488)

    With the stock "unstarted" sentinel, the first
    taskENTER_CRITICAL()/taskEXIT_CRITICAL() pair executed before the
    scheduler starts raises BASEPRI to configMAX_SYSCALL and never releases
    it, because the nesting count can never reach zero. On this core that
    wedge masked both the TIM16 HAL tick and the USB interrupt for the rest
    of boot, the moment TinyUSB's FreeRTOS OSAL took its first critical
    section inside tusb_init(). Diagnosed on hardware with LED breadcrumbs:
    boot froze inside HAL_Delay immediately after tusb_init.

    xPortStartScheduler() still resets the nesting count to zero before the
    first task runs, so this only affects the pre-scheduler window.

  + Initialise ulMaxPRIGROUPValue to portPRIORITY_GROUP_MASK instead of 0
     (around line 519)

    The real bound is computed inside xPortStartScheduler(); before that runs
    the value is uncalibrated. The stock 0 made
    vPortValidateInterruptPriority()'s PRIGROUP assert fail for any
    ...FromISR call issued pre-scheduler. This core's USB stack enumerates
    during the pre-scheduler boot pump with real USB interrupts posting
    events via xQueueSendToBackFromISR; under OPT_OS_NONE that path used no
    FreeRTOS primitives, so the hang appeared only once the FreeRTOS OSAL was
    enabled. Found on hardware.

    Post-scheduler validation is unchanged: the computed value overwrites
    this initialiser.

