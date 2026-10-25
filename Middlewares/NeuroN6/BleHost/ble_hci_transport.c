/**
  ******************************************************************************
  * @file    ble_hci_transport.c
  * @brief   NimBLE's custom LL transport (BLE_TRANSPORT_LL custom) over
  *          neuron6_bt.
  *
  *          Out: type byte then packet, sent in the caller's task. ACL data
  *          goes straight from NimBLE's mbufs, so it may be longer than
  *          BSP_BT_MAX_PACKET_SIZE.
  *          In: a task copies each whole packet into NimBLE's transport pools;
  *          BSP_BT_RxCallback wakes it on every received byte.
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

#include "host/ble_hs.h"
#include "nimble/hci_common.h"
#include "nimble/transport.h"
#include "os/os_mbuf.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <string.h>

/* Longest flow-control hold-off before a send fails. */
#define HCI_SEND_TIMEOUT_MS      1000U

/* Above the host task, so the link's ring drains while the host works. */
#define HCI_RX_TASK_STACK_BYTES  2048U
#define HCI_RX_TASK_PRIORITY     (tskIDLE_PRIORITY + 3U)

volatile BleHost_Stats_t BleHost_Counters;

static TaskHandle_t      HciRxTask;
static SemaphoreHandle_t HciTxLock;
static uint8_t           HciRxPacket[BSP_BT_MAX_PACKET_SIZE];

/* Send the type byte, then the body (pBody, or the mbuf chain om when pBody
 * is NULL). The lock keeps a packet's pieces together on the link. */
static int hci_send(uint8_t Type, const uint8_t *pBody, uint32_t Length, const struct os_mbuf *om)
{
  const struct os_mbuf *m;
  int32_t               status;

  (void)xSemaphoreTake(HciTxLock, portMAX_DELAY);
  status = BSP_BT_Send(&Type, 1U, HCI_SEND_TIMEOUT_MS);
  if (pBody != NULL)
  {
    if (status == BSP_ERROR_NONE)
    {
      status = BSP_BT_Send(pBody, Length, HCI_SEND_TIMEOUT_MS);
    }
  }
  else
  {
    for (m = om; (m != NULL) && (status == BSP_ERROR_NONE); m = SLIST_NEXT(m, om_next))
    {
      if (m->om_len != 0U)
      {
        status = BSP_BT_Send(m->om_data, m->om_len, HCI_SEND_TIMEOUT_MS);
      }
    }
  }
  (void)xSemaphoreGive(HciTxLock);

  if (status != BSP_ERROR_NONE)
  {
    BleHost_Counters.SendFailures++;
    return BLE_HS_ECONTROLLER;
  }
  return 0;
}

/* Host command: opcode, parameter length, parameters. */
int ble_transport_to_ll_cmd_impl(void *buf)
{
  const uint8_t *cmd = (const uint8_t *)buf;
  int rc;

  rc = hci_send(BSP_BT_PACKET_COMMAND, cmd, 3U + (uint32_t)cmd[2], NULL);
  ble_transport_free(buf);
  if (rc == 0)
  {
    BleHost_Counters.CommandsSent++;
  }
  return rc;
}

/* Host data packet: handle and flags, length, data. */
int ble_transport_to_ll_acl_impl(struct os_mbuf *om)
{
  int rc;

  rc = hci_send(BSP_BT_PACKET_ACL, NULL, 0U, om);
  os_mbuf_free_chain(om);
  if (rc == 0)
  {
    BleHost_Counters.AclSent++;
  }
  return rc;
}

/* Isochronous data: not configured on this board. */
int ble_transport_to_ll_iso_impl(struct os_mbuf *om)
{
  os_mbuf_free_chain(om);
  return BLE_HS_ENOTSUP;
}

/* Advertising reports may be dropped when buffers run short. */
static int hci_event_is_discardable(const uint8_t *pPacket, uint32_t Length)
{
  if ((Length < 4U) || (pPacket[1] != BLE_HCI_EVCODE_LE_META))
  {
    return 0;
  }
  switch (pPacket[3])
  {
    case BLE_HCI_LE_SUBEV_ADV_RPT:
    case BLE_HCI_LE_SUBEV_DIRECT_ADV_RPT:
    case BLE_HCI_LE_SUBEV_EXT_ADV_RPT:
    case BLE_HCI_LE_SUBEV_PERIODIC_ADV_RPT:
      return 1;
    default:
      return 0;
  }
}

/* Hand one received packet (type byte first) to the host. */
static void hci_deliver(const uint8_t *pPacket, uint32_t Length)
{
  if (pPacket[0] == BSP_BT_PACKET_EVENT)
  {
    int      discardable = hci_event_is_discardable(pPacket, Length);
    uint8_t *ev = (uint8_t *)ble_transport_alloc_evt(discardable);

    /* Only advertising reports may be dropped. Any other event waits for a
     * buffer: losing one (completed packets, disconnection) desynchronises
     * host and controller for good. Flow control holds the controller off. */
    while ((ev == NULL) && (discardable == 0))
    {
      vTaskDelay(1);
      ev = (uint8_t *)ble_transport_alloc_evt(0);
    }
    if (ev == NULL)
    {
      BleHost_Counters.EventsDropped++;
      return;
    }
    (void)memcpy(ev, &pPacket[1], Length - 1U);
    BleHost_Counters.EventsReceived++;
    (void)ble_transport_to_hs_evt(ev);
  }
  else if (pPacket[0] == BSP_BT_PACKET_ACL)
  {
    struct os_mbuf *om = ble_transport_alloc_acl_from_ll();

    /* Data waits for a buffer too; the host frees them as it works. */
    while (om == NULL)
    {
      vTaskDelay(1);
      om = ble_transport_alloc_acl_from_ll();
    }
    if ((om == NULL) || (os_mbuf_append(om, &pPacket[1], (uint16_t)(Length - 1U)) != 0))
    {
      if (om != NULL)
      {
        os_mbuf_free_chain(om);
      }
      BleHost_Counters.AclDropped++;
      return;
    }
    BleHost_Counters.AclReceived++;
    (void)ble_transport_to_hs_acl(om);
  }
  else
  {
    BleHost_Counters.Unrecognised++;
  }
}

/* Take whole packets from the link as they complete. */
static void hci_rx_task(void *pArg)
{
  uint32_t length;

  (void)pArg;
  for (;;)
  {
    if (BSP_BT_Receive(HciRxPacket, sizeof(HciRxPacket), &length, 0U) == BSP_ERROR_NONE)
    {
      hci_deliver(HciRxPacket, length);
      continue;
    }
    /* Wait for the next byte (notifications latch). The 10 ms timeout lets
     * BSP_BT_Receive drop a packet cut off part-way. */
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10U));
  }
}

/* A byte arrived: wake the receive task. Overrides neuron6_bt's weak
 * definition; interrupt context. */
void BSP_BT_RxCallback(void)
{
  BaseType_t woken = pdFALSE;

  if (HciRxTask != NULL)
  {
    vTaskNotifyGiveFromISR(HciRxTask, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

/* Called by nimble_port_init(), after BleHost_Init() has reset the
 * controller. */
void ble_transport_ll_init(void)
{
  HciTxLock = xSemaphoreCreateMutex();
  configASSERT(HciTxLock != NULL);
  (void)xTaskCreate(hci_rx_task, "ble_hci", HCI_RX_TASK_STACK_BYTES / sizeof(StackType_t),
                    NULL, HCI_RX_TASK_PRIORITY, &HciRxTask);
  configASSERT(HciRxTask != NULL);
}
