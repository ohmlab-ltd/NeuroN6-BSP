/*
  BlePeripheral_FreeRTOS

  A BLE peripheral on the Apache NimBLE host that a phone can connect to,
  pair with and control. It advertises as "Neuro-N6" and serves Device
  Information, a simulated Battery level, the Neuro N6 service (LED, Button,
  Acceleration) and the Nordic UART Service (RX echoed on TX). Writes need an
  encrypted link (LE Secure Connections), so the first one pairs. Bonds are
  kept in RAM: after a reset, remove the board from the phone's paired
  devices. README.md has the GATT layout.

  The circuit:
  - Uses the MAYA-W476 radio module, the RGB LED, the USER button and the
    LSM6DSL accelerometer. No wiring needed.
  - The radio's firmware must already be in external flash at 0x91E00000.

  Send 'b' on the USB console at any time to return to the bootloader.

  Copyright (c) 2026 Ohm Lab Ltd.
  SPDX-License-Identifier: CC0-1.0
  Released under CC0 1.0 Universal: copy it into your own project
  freely, with or without attribution.
*/

#include "main.h"

#include "neuron6.h"
#include "neuron6_bt.h"
#include "neuron6_com.h"
#include "neuron6_motion_sensor.h"
#include "neuron6_wdg.h"

#include "FreeRTOS.h"
#include "task.h"

#include "tusb.h"

#include "ble_host.h"

#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/bas/ble_svc_bas.h"
#include "services/dis/ble_svc_dis.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#if !defined(BSP_USE_FREERTOS) || (BSP_USE_FREERTOS != 1)
#error "This example runs under FreeRTOS: set BSP_USE_FREERTOS to 1 in neuron6_conf.h."
#endif

#define DEVICE_NAME               "Neuro-N6"

/* 0: Just Works pairing. 1: a passkey on the USB console, typed on the phone,
 * which also rules out a man in the middle. */
#define PAIRING_WITH_PASSKEY      0

#define LOOP_MS                   20U     /* the button is sampled this often */
#define ACCEL_EVERY               5U      /* passes: acceleration at 10 Hz    */
#define BATTERY_EVERY             500U    /* passes: the level every 10 s     */

#define WDG_TIMEOUT_MS            2000U
#define WDG_WARN_MS               300U
#define USB_BOOT_PUMP_MS          500U

#define APP_TASK_STACK_WORDS      ((8U * 1024U) / sizeof(StackType_t))
#define APP_TASK_PRIORITY         (tskIDLE_PRIORITY + 2U)
#define USB_TASK_STACK_WORDS      (4096U / sizeof(StackType_t))
#define USB_TASK_PRIORITY         (tskIDLE_PRIORITY + 3U)

#define MAX_LINKS                 MYNEWT_VAL(BLE_MAX_CONNECTIONS)

/* The longest write one ATT packet carries at the host's largest MTU. */
#define NUS_MAX_WRITE             (MYNEWT_VAL(BLE_ATT_PREFERRED_MTU) - 3U)

#define print(...)   (void)BSP_COM_Printf(__VA_ARGS__)

static void mpu_config(void);

/* State */

/* The Neuro N6 service. UUIDs generated for this example; a product
 * generates its own. */
static const ble_uuid128_t N6ServiceUuid =
  BLE_UUID128_INIT(0xDF, 0x19, 0x33, 0x54, 0x6B, 0xAF, 0xFA, 0x80,
                   0x11, 0x47, 0xEF, 0x8A, 0x01, 0x00, 0xC7, 0xE7);
static const ble_uuid128_t N6LedUuid =
  BLE_UUID128_INIT(0xDF, 0x19, 0x33, 0x54, 0x6B, 0xAF, 0xFA, 0x80,
                   0x11, 0x47, 0xEF, 0x8A, 0x02, 0x00, 0xC7, 0xE7);
static const ble_uuid128_t N6ButtonUuid =
  BLE_UUID128_INIT(0xDF, 0x19, 0x33, 0x54, 0x6B, 0xAF, 0xFA, 0x80,
                   0x11, 0x47, 0xEF, 0x8A, 0x03, 0x00, 0xC7, 0xE7);
static const ble_uuid128_t N6AccelUuid =
  BLE_UUID128_INIT(0xDF, 0x19, 0x33, 0x54, 0x6B, 0xAF, 0xFA, 0x80,
                   0x11, 0x47, 0xEF, 0x8A, 0x04, 0x00, 0xC7, 0xE7);

/* The Nordic UART Service, 6E400001-B5A3-F393-E0A9-E50E24DCCA9E, which UART
 * terminal apps look for. */
static const ble_uuid128_t NusServiceUuid =
  BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                   0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);
static const ble_uuid128_t NusRxUuid =
  BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                   0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);
static const ble_uuid128_t NusTxUuid =
  BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                   0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

/* Characteristic value handles, filled in by NimBLE at registration. */
static uint16_t LedHandle;
static uint16_t ButtonHandle;
static uint16_t AccelHandle;
static uint16_t NusTxHandle;

/* One entry per connection. Only the host task touches these. */
typedef struct
{
  uint16_t Handle;
  bool     InUse;
  bool     NusTxOn;   /* the peer turned on TX notifications */
} Link_t;

static Link_t   Links[MAX_LINKS];
static uint8_t  OwnAddrType;

/* Written by the application task, read by the host task. */
static volatile bool    Ready;
static volatile uint8_t LedBits;
static volatile uint8_t ButtonPressed;
static int16_t          Accel[3];
static bool             AccelOk;

static char SerialNumber[25];
static char FirmwareRevision[16];

static void advertise(void);

/* The board */

static void set_led(uint8_t Bits)
{
  (void)(((Bits & 1U) != 0U) ? BSP_LED_On(LED_RED)   : BSP_LED_Off(LED_RED));
  (void)(((Bits & 2U) != 0U) ? BSP_LED_On(LED_GREEN) : BSP_LED_Off(LED_GREEN));
  (void)(((Bits & 4U) != 0U) ? BSP_LED_On(LED_BLUE)  : BSP_LED_Off(LED_BLUE));
  LedBits = Bits;
}

static void read_accel(void)
{
  BSP_MOTION_SENSOR_Axes_t a = {0};

  if (AccelOk && (BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO, &a) == BSP_ERROR_NONE))
  {
    taskENTER_CRITICAL();
    Accel[0] = (int16_t)a.x;
    Accel[1] = (int16_t)a.y;
    Accel[2] = (int16_t)a.z;
    taskEXIT_CRITICAL();
  }
}

/* GATT services */

static int reply(struct ble_gatt_access_ctxt *pCtxt, const void *pData, uint16_t Length)
{
  return (os_mbuf_append(pCtxt->om, pData, Length) == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int led_access(uint16_t Conn, uint16_t Attr, struct ble_gatt_access_ctxt *pCtxt, void *pArg)
{
  uint8_t  bits;
  uint16_t length;

  (void)Conn;
  (void)Attr;
  (void)pArg;

  if (pCtxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
  {
    bits = LedBits;
    return reply(pCtxt, &bits, 1U);
  }
  if (OS_MBUF_PKTLEN(pCtxt->om) != 1U)
  {
    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
  }
  (void)ble_hs_mbuf_to_flat(pCtxt->om, &bits, 1U, &length);
  if (bits > 7U)
  {
    return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
  }
  set_led(bits);
  print("[N6] LED %c%c%c\r\n", ((bits & 1U) != 0U) ? 'R' : '-',
        ((bits & 2U) != 0U) ? 'G' : '-', ((bits & 4U) != 0U) ? 'B' : '-');
  return 0;
}

static int button_access(uint16_t Conn, uint16_t Attr, struct ble_gatt_access_ctxt *pCtxt, void *pArg)
{
  uint8_t pressed = ButtonPressed;

  (void)Conn;
  (void)Attr;
  (void)pArg;
  return reply(pCtxt, &pressed, 1U);
}

static int accel_access(uint16_t Conn, uint16_t Attr, struct ble_gatt_access_ctxt *pCtxt, void *pArg)
{
  uint8_t  value[6];
  uint32_t i;

  (void)Conn;
  (void)Attr;
  (void)pArg;

  taskENTER_CRITICAL();
  for (i = 0U; i < 3U; i++)
  {
    value[2U * i]      = (uint8_t)((uint16_t)Accel[i]);
    value[(2U * i) + 1U] = (uint8_t)((uint16_t)Accel[i] >> 8);
  }
  taskEXIT_CRITICAL();
  return reply(pCtxt, value, sizeof(value));
}

/**
  * @brief  A characteristic's User Description (the name a phone shows): pArg.
  */
static int name_access(uint16_t Conn, uint16_t Attr, struct ble_gatt_access_ctxt *pCtxt, void *pArg)
{
  (void)Conn;
  (void)Attr;
  return reply(pCtxt, pArg, (uint16_t)strlen((const char *)pArg));
}

static Link_t *find_link(uint16_t Conn)
{
  uint32_t i;

  for (i = 0U; i < MAX_LINKS; i++)
  {
    if (Links[i].InUse && (Links[i].Handle == Conn))
    {
      return &Links[i];
    }
  }
  return NULL;
}

/**
  * @brief  NUS RX: print what arrived and send it back on TX.
  */
static int nus_rx_access(uint16_t Conn, uint16_t Attr, struct ble_gatt_access_ctxt *pCtxt, void *pArg)
{
  uint8_t   data[NUS_MAX_WRITE];
  char      text[NUS_MAX_WRITE + 1U];
  uint16_t  length;
  Link_t   *link = find_link(Conn);
  uint32_t  i;

  (void)Attr;
  (void)pArg;

  if (ble_hs_mbuf_to_flat(pCtxt->om, data, sizeof(data), &length) != 0)
  {
    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
  }

  for (i = 0U; i < length; i++)
  {
    text[i] = ((data[i] >= 0x20U) && (data[i] < 0x7FU)) ? (char)data[i] : '.';
  }
  text[length] = '\0';
  print("[NUS] %u bytes from link %u:\r\n", length, Conn);
  print("%s\r\n", text);

  if ((link != NULL) && link->NusTxOn)
  {
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, length);

    if (om != NULL)
    {
      (void)ble_gatts_notify_custom(Conn, NusTxHandle, om);
    }
  }
  return 0;
}

static int nus_tx_access(uint16_t Conn, uint16_t Attr, struct ble_gatt_access_ctxt *pCtxt, void *pArg)
{
  (void)Conn;
  (void)Attr;
  (void)pCtxt;
  (void)pArg;
  return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

/* Writing the LED or NUS RX needs an encrypted link (authenticated with a
 * passkey), so an unpaired phone is made to pair first. */
#if PAIRING_WITH_PASSKEY
#define WRITE_SECURITY   BLE_GATT_CHR_F_WRITE_AUTHEN
#else
#define WRITE_SECURITY   BLE_GATT_CHR_F_WRITE_ENC
#endif

#define USER_DESCRIPTION(Name)                                                   \
  (struct ble_gatt_dsc_def[]) {                                                  \
    { .uuid = BLE_UUID16_DECLARE(0x2901), .att_flags = BLE_ATT_F_READ,           \
      .access_cb = name_access, .arg = (void *)(Name) },                         \
    { 0 }                                                                        \
  }

static const struct ble_gatt_svc_def Services[] =
{
  {
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = &N6ServiceUuid.u,
    .characteristics = (struct ble_gatt_chr_def[])
    {
      {
        .uuid        = &N6LedUuid.u,
        .access_cb   = led_access,
        .flags       = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | WRITE_SECURITY,
        .val_handle  = &LedHandle,
        .descriptors = USER_DESCRIPTION("LED"),
      },
      {
        .uuid        = &N6ButtonUuid.u,
        .access_cb   = button_access,
        .flags       = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle  = &ButtonHandle,
        .descriptors = USER_DESCRIPTION("Button"),
      },
      {
        .uuid        = &N6AccelUuid.u,
        .access_cb   = accel_access,
        .flags       = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle  = &AccelHandle,
        .descriptors = USER_DESCRIPTION("Acceleration"),
      },
      { 0 }
    },
  },
  {
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = &NusServiceUuid.u,
    .characteristics = (struct ble_gatt_chr_def[])
    {
      {
        .uuid      = &NusRxUuid.u,
        .access_cb = nus_rx_access,
        .flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP | WRITE_SECURITY,
      },
      {
        .uuid       = &NusTxUuid.u,
        .access_cb  = nus_tx_access,
        .flags      = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &NusTxHandle,
      },
      { 0 }
    },
  },
  { 0 }
};

/* GAP */

static uint32_t links_in_use(void)
{
  uint32_t i;
  uint32_t n = 0U;

  for (i = 0U; i < MAX_LINKS; i++)
  {
    n += Links[i].InUse ? 1U : 0U;
  }
  return n;
}

static void print_addr(const char *pWhat, const ble_addr_t *pAddr)
{
  print("%s %02X:%02X:%02X:%02X:%02X:%02X", pWhat, pAddr->val[5], pAddr->val[4],
        pAddr->val[3], pAddr->val[2], pAddr->val[1], pAddr->val[0]);
}

static int gap_event(struct ble_gap_event *pEvent, void *pArg)
{
  struct ble_gap_conn_desc desc;
  Link_t                  *link;
  uint32_t                 i;

  (void)pArg;

  switch (pEvent->type)
  {
    case BLE_GAP_EVENT_CONNECT:
      if (pEvent->connect.status != 0)
      {
        print("[BLE] connection failed (%d)\r\n", pEvent->connect.status);
        advertise();
        return 0;
      }
      for (i = 0U; (i < MAX_LINKS) && Links[i].InUse; i++)
      {
      }
      if (i < MAX_LINKS)
      {
        Links[i].Handle  = pEvent->connect.conn_handle;
        Links[i].InUse   = true;
        Links[i].NusTxOn = false;
      }
      if (ble_gap_conn_find(pEvent->connect.conn_handle, &desc) == 0)
      {
        print_addr("[BLE] connected to", &desc.peer_id_addr);
        print(", link %u\r\n", pEvent->connect.conn_handle);
      }
#if !MYNEWT_VAL(BLE_EXT_ADV)
      advertise();   /* for the next phone, while there is room */
#endif
      return 0;

    case BLE_GAP_EVENT_DISCONNECT:
      link = find_link(pEvent->disconnect.conn.conn_handle);
      if (link != NULL)
      {
        link->InUse = false;
      }
      print("[BLE] link %u closed (reason 0x%03X)\r\n", pEvent->disconnect.conn.conn_handle,
            (unsigned int)pEvent->disconnect.reason);
      advertise();
      return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
      /* With extended advertising a connection also ends the advertising,
       * reported here after BLE_GAP_EVENT_CONNECT. Restart only now: any
       * sooner would be taken for the set that just ended. */
      advertise();
      return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
      if ((pEvent->enc_change.status == 0) &&
          (ble_gap_conn_find(pEvent->enc_change.conn_handle, &desc) == 0))
      {
        print("[BLE] link %u encrypted%s%s\r\n", pEvent->enc_change.conn_handle,
              desc.sec_state.authenticated ? ", authenticated" : "",
              desc.sec_state.bonded ? ", bonded" : "");
      }
      else
      {
        print("[BLE] link %u: encryption failed (%d)\r\n", pEvent->enc_change.conn_handle,
              pEvent->enc_change.status);
      }
      return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
      /* One side lost the bond (the board does at a reset): forget it and
       * pair again. */
      if (ble_gap_conn_find(pEvent->repeat_pairing.conn_handle, &desc) == 0)
      {
        (void)ble_store_util_delete_peer(&desc.peer_id_addr);
      }
      return BLE_GAP_REPEAT_PAIRING_RETRY;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
      if (pEvent->passkey.params.action == BLE_SM_IOACT_DISP)
      {
        struct ble_sm_io io = {0};
        uint32_t         r  = 0U;

        (void)ble_hs_hci_rand(&r, sizeof(r));
        io.action  = BLE_SM_IOACT_DISP;
        io.passkey = r % 1000000U;
        print("[BLE] passkey %06lu: type it on the phone\r\n", (unsigned long)io.passkey);
        (void)ble_sm_inject_io(pEvent->passkey.conn_handle, &io);
      }
      return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
      if (pEvent->subscribe.attr_handle == NusTxHandle)
      {
        link = find_link(pEvent->subscribe.conn_handle);
        if (link != NULL)
        {
          link->NusTxOn = (pEvent->subscribe.cur_notify != 0U);
        }
      }
      print("[BLE] link %u: notifications %s for handle %u\r\n", pEvent->subscribe.conn_handle,
            (pEvent->subscribe.cur_notify != 0U) ? "on" : "off", pEvent->subscribe.attr_handle);
      return 0;

    case BLE_GAP_EVENT_MTU:
      print("[BLE] link %u: MTU %u\r\n", pEvent->mtu.conn_handle, pEvent->mtu.value);
      return 0;

    default:
      return 0;
  }
}

/**
  * @brief  Advertise, connectable, while there is room for another link.
  */
static void advertise(void)
{
  static const ble_uuid16_t more[2] = { BLE_UUID16_INIT(0x180A), BLE_UUID16_INIT(0x180F) };
  struct ble_gap_adv_params params = {0};
  struct ble_hs_adv_fields  adv    = {0};
  struct ble_hs_adv_fields  rsp    = {0};
  int                       rc;

  if (ble_gap_adv_active() || (links_in_use() >= MAX_LINKS))
  {
    return;
  }

  /* Advertisement: flags, name and the UART service (31 bytes). */
  adv.flags                = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  adv.name                 = (const uint8_t *)DEVICE_NAME;
  adv.name_len             = (uint8_t)(sizeof(DEVICE_NAME) - 1U);
  adv.name_is_complete     = 1U;
  adv.uuids128             = &NusServiceUuid;
  adv.num_uuids128         = 1U;
  adv.uuids128_is_complete = 0U;

  /* Scan response: device information, battery and the Neuro N6 service. */
  rsp.uuids16              = more;
  rsp.num_uuids16          = 2U;
  rsp.uuids16_is_complete  = 0U;
  rsp.uuids128             = &N6ServiceUuid;
  rsp.num_uuids128         = 1U;
  rsp.uuids128_is_complete = 0U;

  params.conn_mode = BLE_GAP_CONN_MODE_UND;
  params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  params.itvl_min  = BLE_GAP_ADV_ITVL_MS(100);
  params.itvl_max  = BLE_GAP_ADV_ITVL_MS(150);

  rc = ble_gap_adv_set_fields(&adv);
  if (rc == 0)
  {
    rc = ble_gap_adv_rsp_set_fields(&rsp);
  }
  if (rc == 0)
  {
    rc = ble_gap_adv_start(OwnAddrType, NULL, BLE_HS_FOREVER, &params, gap_event, NULL);
  }
  if (rc != 0)
  {
    print("[BLE] advertising failed (%d)\r\n", rc);
  }
}

/**
  * @brief  The host and the controller are in step: start advertising.
  * @note   Host task. Called again after every host reset.
  */
static void on_sync(void)
{
  uint8_t addr[6] = {0};

  if ((ble_hs_util_ensure_addr(0) != 0) || (ble_hs_id_infer_auto(0, &OwnAddrType) != 0))
  {
    print("[BLE] no usable address\r\n");
    return;
  }
  (void)ble_hs_id_copy_addr(OwnAddrType, addr, NULL);
  print("[BLE] '%s' advertising from %02X:%02X:%02X:%02X:%02X:%02X\r\n", DEVICE_NAME,
        addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
  Ready = true;
  advertise();
}

static void on_reset(int Reason)
{
  Ready = false;
  print("[BLE] host reset (%d)\r\n", Reason);
}

/* The application */

static void setup(void)
{
  int32_t status;
  int32_t v = BSP_GetVersion();

  (void)BSP_LED_Init(LED_RED);
  (void)BSP_LED_Init(LED_GREEN);
  (void)BSP_LED_Init(LED_BLUE);
  (void)BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
  AccelOk = (BSP_MOTION_SENSOR_Init(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO) == BSP_ERROR_NONE) &&
            (BSP_MOTION_SENSOR_Enable(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO) == BSP_ERROR_NONE);
  if (!AccelOk)
  {
    print("[N6] no accelerometer: Acceleration reads zero\r\n");
  }

  (void)snprintf(SerialNumber, sizeof(SerialNumber), "%08lX%08lX%08lX",
                 (unsigned long)HAL_GetUIDw2(), (unsigned long)HAL_GetUIDw1(),
                 (unsigned long)HAL_GetUIDw0());
  (void)snprintf(FirmwareRevision, sizeof(FirmwareRevision), "BSP %lu.%lu.%lu",
                 (unsigned long)(((uint32_t)v >> 24) & 0xFFU),
                 (unsigned long)(((uint32_t)v >> 16) & 0xFFU),
                 (unsigned long)(((uint32_t)v >> 8) & 0xFFU));

  /* The radio's firmware, the HCI link and NimBLE. */
  status = BleHost_Init();
  if (status != BSP_ERROR_NONE)
  {
    print("[BLE] no BLE controller (%ld) - is the radio firmware in flash?\r\n", (long)status);
    return;
  }

  ble_hs_cfg.sync_cb         = on_sync;
  ble_hs_cfg.reset_cb        = on_reset;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

  /* LE Secure Connections and bonding; the identity keys let the board
   * recognise a phone that uses a private address. */
  ble_hs_cfg.sm_sc             = 1U;
  ble_hs_cfg.sm_bonding        = 1U;
#if PAIRING_WITH_PASSKEY
  ble_hs_cfg.sm_io_cap         = BLE_HS_IO_DISPLAY_ONLY;
  ble_hs_cfg.sm_mitm           = 1U;
#else
  ble_hs_cfg.sm_io_cap         = BLE_HS_IO_NO_INPUT_OUTPUT;
  ble_hs_cfg.sm_mitm           = 0U;
#endif
  ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

  ble_svc_gap_init();
  ble_svc_gatt_init();
  ble_svc_dis_init();
  ble_svc_bas_init();
  (void)ble_svc_gap_device_name_set(DEVICE_NAME);
  (void)ble_svc_dis_serial_number_set(SerialNumber);
  (void)ble_svc_dis_firmware_revision_set(FirmwareRevision);
  (void)ble_svc_bas_battery_level_set(100U);

  if ((ble_gatts_count_cfg(Services) != 0) || (ble_gatts_add_svcs(Services) != 0))
  {
    print("[BLE] the services did not register\r\n");
    return;
  }

  if (BleHost_Start() != BSP_ERROR_NONE)
  {
    print("[BLE] the host did not start\r\n");
  }
}

static void loop(void)
{
  static uint32_t pass;
  static uint8_t  battery = 100U;
  uint8_t         pressed;

  vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
  pass++;

  pressed = (BSP_PB_GetState(BUTTON_USER) == (int32_t)BUTTON_PRESSED) ? 1U : 0U;
  if (pressed != ButtonPressed)
  {
    ButtonPressed = pressed;
    print("[N6] button %s\r\n", (pressed != 0U) ? "pressed" : "released");
    if (Ready)
    {
      ble_gatts_chr_updated(ButtonHandle);
    }
  }

  if ((pass % ACCEL_EVERY) == 0U)
  {
    read_accel();
    if (Ready)
    {
      ble_gatts_chr_updated(AccelHandle);
    }
  }

  if ((pass % BATTERY_EVERY) == 0U)
  {
    battery = (battery > 0U) ? (uint8_t)(battery - 1U) : 100U;
    (void)ble_svc_bas_battery_level_set(battery);
  }
}

static void app_task(void *pArg)
{
  (void)pArg;

  setup();
  for (;;)
  {
    loop();
  }
}

/* USB */

/**
  * @brief  Keeps the console and the way back to the bootloader alive.
  */
static void usb_task(void *pArg)
{
  uint8_t ch;

  (void)pArg;

  BSP_COM_SetPumpTask(xTaskGetCurrentTaskHandle());
  for (;;)
  {
    (void)BSP_WDG_Refresh();
    tud_task_ext(0, false);   /* non-blocking drain; bare tud_task() blocks under OPT_OS_FREERTOS */
    while (BSP_COM_Read(&ch, 1U) == 1)
    {
      if (ch == (uint8_t)'b')
      {
        BSP_COM_RebootToBootloader();
      }
    }
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1U));
  }
}

/* Scaffolding */

/**
  * @brief  The BLE UART's interrupt.
  */
void USART6_IRQHandler(void)
{
  BSP_BT_IRQHandler();
}

/**
  * @brief  The radio library's console: the USB console (dropped with no host).
  */
void NeuroN6_DbgPrint(const char *msg)
{
  print("%s", msg);
}

/**
  * @brief  Runs 300 ms before the watchdog reset, in whatever context is
  *         stuck: reboot into the bootloader instead.
  */
void BSP_WDG_EWI_Callback(void)
{
  BSP_COM_RebootToBootloader();
}

/**
  * @brief  The external memories are not cacheable.
  */
static void mpu_config(void)
{
  MPU_Region_InitTypeDef     default_config = {0};
  MPU_Attributes_InitTypeDef attr_config    = {0};
  uint32_t primask_bit = __get_PRIMASK();
  __disable_irq();

  HAL_MPU_Disable();

  attr_config.Attributes = INNER_OUTER(MPU_NOT_CACHEABLE);
  attr_config.Number     = MPU_ATTRIBUTES_NUMBER0;
  HAL_MPU_ConfigMemoryAttributes(&attr_config);

  default_config.Enable           = MPU_REGION_ENABLE;
  default_config.Number           = MPU_REGION_NUMBER0;
  default_config.BaseAddress      = __NON_CACHEABLE_SECTION_BEGIN;
  default_config.LimitAddress     = __NON_CACHEABLE_SECTION_END;
  default_config.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  default_config.AccessPermission = MPU_REGION_ALL_RW;
  default_config.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  default_config.AttributesIndex  = MPU_ATTRIBUTES_NUMBER0;
  HAL_MPU_ConfigRegion(&default_config);

  default_config.Number       = MPU_REGION_NUMBER1;
  default_config.BaseAddress  = 0x70000000;
  default_config.LimitAddress = 0x71FFFFFF;
  HAL_MPU_ConfigRegion(&default_config);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
  __set_PRIMASK(primask_bit);
}

int main(void)
{
  uint32_t start;

  SCB_DisableDCache();
  mpu_config();
  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();

  /* No SystemClock_Config: the boot loader owns the clock tree and both
   * external memories; the BSP reads the frequencies back at run time. */

  /* Watchdog first; a watchdog reset goes to the bootloader, not back into
   * the code that caused it. */
  HAL_PWR_EnableBkUpAccess();
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U)
  {
    __HAL_RCC_CLEAR_RESET_FLAGS();
    BSP_COM_RebootToBootloader();
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
  (void)BSP_WDG_InitEWI(WDG_TIMEOUT_MS, WDG_WARN_MS);

  /* USB first, then the boot-time enumeration pump. */
  if (BSP_COM_Init() != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  start = HAL_GetTick();
  while (!tud_mounted() && ((HAL_GetTick() - start) < USB_BOOT_PUMP_MS))
  {
    tud_task_ext(0, false);
    (void)BSP_WDG_Refresh();
    HAL_Delay(10U);
  }

  if ((xTaskCreate(usb_task, "usb", USB_TASK_STACK_WORDS, NULL,
                   USB_TASK_PRIORITY, NULL) != pdPASS) ||
      (xTaskCreate(app_task, "app", APP_TASK_STACK_WORDS, NULL,
                   APP_TASK_PRIORITY, NULL) != pdPASS))
  {
    Error_Handler();
  }

  vTaskStartScheduler();

  Error_Handler();
  return 0;
}

/**
  * @brief  Reboot into the bootloader, where the board can be reflashed.
  */
void Error_Handler(void)
{
  BSP_COM_RebootToBootloader();
}
