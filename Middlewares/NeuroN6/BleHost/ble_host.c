/**
  ******************************************************************************
  * @file    ble_host.c
  * @brief   NimBLE on the Neuro N6: controller bring-up, host init, host task.
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

#include "ble_host_int.h"

#include "neuron6_bt.h"

#include "MayaW476.h"

#include "nimble/nimble_port.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>

/* NimBLE's RAM bond store; no header declares it. */
extern void ble_store_config_init(void);

/* Sized for LE Secure Connections' P-256 key exchange, run on this task. */
#define BLE_HOST_TASK_STACK_BYTES  (6U * 1024U)
#define BLE_HOST_TASK_PRIORITY     (tskIDLE_PRIORITY + 2U)

/* The controller answers nothing until reset, and the first HCI_Reset after
 * link-up can go unanswered. */
#define HCI_RESET_OPCODE           0x0C03U
#define HCI_RESET_ATTEMPTS         3U
#define HCI_RESET_TIMEOUT_MS       2000U

/* The link must be quiet this long before NimBLE takes it. */
#define HCI_QUIET_MS               100U

static uint8_t      Initialised;
static TaskHandle_t HostTask;
static uint8_t      Stray[BSP_BT_MAX_PACKET_SIZE];

/* Runs NimBLE's default event queue; never returns. */
static void ble_host_task(void *pArg)
{
  (void)pArg;
  nimble_port_run();
}

int32_t BleHost_Init(void)
{
  int32_t  status;
  uint32_t attempt;

  if (Initialised != 0U)
  {
    return BSP_ERROR_NONE;
  }

  /* The BLE controller runs only once the Wi-Fi library has loaded the
   * IW610's combined Wi-Fi/BLE firmware. */
  if (MayaW476_RadioStart() != 0)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  status = BSP_BT_Init();
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  status = BSP_ERROR_BUSY;
  for (attempt = 0U; (attempt < HCI_RESET_ATTEMPTS) && (status != BSP_ERROR_NONE); attempt++)
  {
    status = BSP_BT_Command(HCI_RESET_OPCODE, NULL, 0U, NULL, 0U, HCI_RESET_TIMEOUT_MS);
  }
  if (status != BSP_ERROR_NONE)
  {
    return BSP_ERROR_BUSY;
  }

  /* A late answer to a timed-out reset would shift every NimBLE command onto
   * the previous one's answer: drain the link until it is quiet. */
  {
    uint32_t length;

    while (BSP_BT_Receive(Stray, sizeof(Stray), &length, HCI_QUIET_MS) == BSP_ERROR_NONE)
    {
    }
  }
  (void)BSP_BT_Flush();

  /* Only NimBLE talks to the controller from here; nimble_port_init()
   * starts the transport's receive task. */
  nimble_port_init();
  ble_store_config_init();

  Initialised = 1U;
  return BSP_ERROR_NONE;
}

int32_t BleHost_Start(void)
{
  if (Initialised == 0U)
  {
    return BSP_ERROR_NO_INIT;
  }
  if (HostTask != NULL)
  {
    return BSP_ERROR_BUSY;
  }
  if (xTaskCreate(ble_host_task, "ble_host", BLE_HOST_TASK_STACK_BYTES / sizeof(StackType_t),
                  NULL, BLE_HOST_TASK_PRIORITY, &HostTask) != pdPASS)
  {
    HostTask = NULL;
    return BSP_ERROR_PERIPH_FAILURE;
  }
  return BSP_ERROR_NONE;
}

void BleHost_GetStats(BleHost_Stats_t *pStats)
{
  if (pStats != NULL)
  {
    pStats->CommandsSent   = BleHost_Counters.CommandsSent;
    pStats->AclSent        = BleHost_Counters.AclSent;
    pStats->EventsReceived = BleHost_Counters.EventsReceived;
    pStats->AclReceived    = BleHost_Counters.AclReceived;
    pStats->EventsDropped  = BleHost_Counters.EventsDropped;
    pStats->AclDropped     = BleHost_Counters.AclDropped;
    pStats->SendFailures   = BleHost_Counters.SendFailures;
    pStats->Unrecognised   = BleHost_Counters.Unrecognised;
  }
}

/* mbedTLS allocator (security manager bignums) on the FreeRTOS heap; named
 * in mbedtls_config_ble.h. */
void *BleHost_Calloc(size_t Count, size_t Size)
{
  void *p;

  if ((Count != 0U) && (Size > (SIZE_MAX / Count)))
  {
    return NULL;
  }
  p = pvPortMalloc(Count * Size);
  if (p != NULL)
  {
    (void)memset(p, 0, Count * Size);
  }
  return p;
}

void BleHost_Free(void *p)
{
  vPortFree(p);
}
