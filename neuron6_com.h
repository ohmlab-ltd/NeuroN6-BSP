/**
  * @file    neuron6_com.h
  * @brief   Console over USB CDC on the USB-C port; the Neuro N6 has no debug UART.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_COM_H
#define NEURON6_COM_H

#include "neuron6_conf.h"
#include "neuron6_errno.h"

#ifdef __cplusplus
extern "C" {
#endif

/* stdio is retargeted here unless BSP_COM_NO_PRINTF_RETARGET is defined; a stdin read
 * blocks until a line arrives and refreshes no watchdog. The application supplies
 * TinyUSB, tusb_config.h and USB descriptors (reference copies in Utilities/usb/). */

/** @brief CDC interface the console uses. */
#define BSP_COM_ITF            0U

/** @brief Open the port at this baud, then close it, to reboot into the bootloader. */
#define BSP_COM_DFU_TOUCH_BAUD 1200U

#if defined(USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
/** @brief USB MSP (clocks and vector) routines, for BSP_COM_RegisterMspCallbacks(). */
typedef struct
{
  void (* pMspInitCb)(PCD_HandleTypeDef *);   /*!< Clocks and vector up.   */
  void (* pMspDeInitCb)(PCD_HandleTypeDef *); /*!< Clocks and vector down. */
} BSP_COM_Cb_t;

/** @brief Restore the BSP's USB MSP routines; BSP_COM_Init() sets them if none are. */
int32_t BSP_COM_RegisterDefaultMspCallbacks(void);

/** @brief Replace the USB MSP routines (neither NULL); call before BSP_COM_Init(). */
int32_t BSP_COM_RegisterMspCallbacks(BSP_COM_Cb_t *Callbacks);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

/**
  * @brief  Bring up the USB device stack and the CDC console.
  * @retval BSP_ERROR_BUSY after BSP_COM_DeInit(): the stack cannot restart until a reset.
  */
int32_t BSP_COM_Init(void);

/**
  * @brief  Shut the console down and release the USB peripheral; final until a reset.
  * @note   Under FreeRTOS a task already in the stack gets up to 50 ms to finish first.
  */
int32_t BSP_COM_DeInit(void);

/**
  * @brief  Service the USB stack; call regularly from the main loop. Non-blocking.
  * @note   Under FreeRTOS one task owns it: once BSP_RTOS_StartUsbTask has created it,
  *         nothing else should call this.
  */
void BSP_COM_Task(void);

/**
  * @brief  Wait up to Timeout_ms for a USB event, then service the stack.
  * @note   Bare metal: the same as BSP_COM_Task, without the wait.
  */
void BSP_COM_TaskWait(uint32_t Timeout_ms);

/**
  * @brief  Name the task that services the stack: a TaskHandle_t, or NULL for none.
  * @note   The USB interrupt wakes it; BSP_COM_Flush in another task waits for it.
  */
void BSP_COM_SetPumpTask(void *pTask);

/** @brief 1 when a host has the port open, else 0. */
int32_t BSP_COM_IsConnected(void);

/**
  * @brief  Queue bytes to send; never blocks.
  * @retval Bytes taken (no more than the free FIFO space or 65535; 0 with no host), or
  *         BSP_ERROR_WRONG_PARAM.
  */
int32_t BSP_COM_Write(const uint8_t *pData, uint32_t Length);

/** @brief Read received bytes (65535 at most per call); never blocks. Returns the count. */
int32_t BSP_COM_Read(uint8_t *pData, uint32_t Length);

/** @brief printf to the console; output past 255 characters is cut. */
int32_t BSP_COM_Printf(const char *Format, ...);

/**
  * @brief  Block until everything written has left the endpoint, or Timeout_ms passes.
  * @note   Pumps the stack meanwhile; use before a deliberate reset.
  */
void BSP_COM_Flush(uint32_t Timeout_ms);

/**
  * @brief  Reset into the boot loader's DFU mode.
  * @note   Cancels a firmware update or settings staged with N6OTA and not yet applied.
  */
void BSP_COM_RebootToBootloader(void);

/** @brief Call from the application's USB1_OTG_HS_IRQHandler(). */
void BSP_COM_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_COM_H */
