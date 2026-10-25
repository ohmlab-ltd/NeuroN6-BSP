/*
  BleBeacon_FreeRTOS

  Advertises a non-connectable BLE beacon named "Neuro-N6", with a counter in
  its manufacturer data that goes up once a second. A phone scanner such as
  nRF Connect shows the name and the changing bytes.

  No Wi-Fi network is needed: MayaW476_RadioStart() powers the radio and loads
  its firmware (one image for Wi-Fi and BLE); neuron6_bt carries HCI over its
  UART.

  The circuit:
  - Uses the MAYA-W476 radio module. No wiring needed.
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
#include "neuron6_wdg.h"

#include "FreeRTOS.h"
#include "task.h"

#include "tusb.h"

#include "MayaW476.h"

#include <stdbool.h>
#include <string.h>

#if !defined(BSP_USE_FREERTOS) || (BSP_USE_FREERTOS != 1)
#error "This example runs under FreeRTOS: set BSP_USE_FREERTOS to 1 in neuron6_conf.h."
#endif

#define BEACON_NAME               "Neuro-N6"

#define WDG_TIMEOUT_MS            2000U
#define WDG_WARN_MS               300U
#define USB_BOOT_PUMP_MS          500U

#define APP_TASK_STACK_WORDS      ((8U * 1024U) / sizeof(StackType_t))
#define APP_TASK_PRIORITY         (tskIDLE_PRIORITY + 2U)
#define USB_TASK_STACK_WORDS      (4096U / sizeof(StackType_t))
#define USB_TASK_PRIORITY         (tskIDLE_PRIORITY + 3U)

/* HCI commands (Core Specification, Vol 4 Part E). */
#define HCI_RESET                 0x0C03U
#define HCI_READ_BD_ADDR          0x1009U
#define HCI_LE_SET_ADV_PARAMS     0x2006U
#define HCI_LE_SET_ADV_DATA       0x2008U
#define HCI_LE_SET_ADV_ENABLE     0x200AU

#define print(...)   (void)BSP_COM_Printf(__VA_ARGS__)

static void mpu_config(void);

/* 0xFFFF is the company ID set aside for testing. */
static uint8_t Beacon[6] = { 0xFFU, 0xFFU, 'N', '6', 0U, 0U };

/**
  * @brief  One HCI command, completed with status 0 within two seconds.
  */
static bool hci(uint16_t Opcode, const uint8_t *pParam, uint8_t Len, uint8_t *pRet, uint32_t RetSize)
{
  return BSP_BT_Command(Opcode, pParam, Len, pRet, RetSize, 2000U) == BSP_ERROR_NONE;
}

/**
  * @brief  The advertising data: flags, the name, and the manufacturer data.
  */
static bool set_adv_data(void)
{
  uint8_t d[32] = {0};
  uint8_t o     = 1U;

  d[o++] = 2U;      /* Flags: LE General Discoverable, no BR/EDR */
  d[o++] = 0x01U;
  d[o++] = 0x06U;
  d[o++] = (uint8_t)(1U + (sizeof(BEACON_NAME) - 1U));
  d[o++] = 0x09U;   /* Complete Local Name */
  memcpy(&d[o], BEACON_NAME, sizeof(BEACON_NAME) - 1U);
  o     = (uint8_t)(o + (sizeof(BEACON_NAME) - 1U));
  d[o++] = (uint8_t)(1U + sizeof(Beacon));
  d[o++] = 0xFFU;   /* Manufacturer Specific Data */
  memcpy(&d[o], Beacon, sizeof(Beacon));
  o     = (uint8_t)(o + sizeof(Beacon));
  d[0]  = (uint8_t)(o - 1U);   /* significant length */
  return hci(HCI_LE_SET_ADV_DATA, d, sizeof(d), NULL, 0U);
}

static void setup(void)
{
  /* Non-connectable (ADV_NONCONN_IND) on all three channels, every 100 ms. */
  static const uint8_t params[15] = { 0xA0U, 0x00U, 0xA0U, 0x00U, 0x03U, 0x00U, 0x00U,
                                      0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x07U, 0x00U };
  const uint8_t on = 1U;
  uint8_t       a[6] = {0};

  if ((MayaW476_RadioStart() != 0) || (BSP_BT_Init() != BSP_ERROR_NONE) ||
      !hci(HCI_RESET, NULL, 0U, NULL, 0U))
  {
    print("[BLE] no BLE controller - is the radio firmware in flash?\r\n");
    return;
  }
  (void)hci(HCI_READ_BD_ADDR, NULL, 0U, a, sizeof(a));
  if (!hci(HCI_LE_SET_ADV_PARAMS, params, sizeof(params), NULL, 0U) || !set_adv_data() ||
      !hci(HCI_LE_SET_ADV_ENABLE, &on, 1U, NULL, 0U))
  {
    print("[BLE] the controller refused the advertising settings\r\n");
    return;
  }
  /* The controller reports its address least significant byte first. */
  print("[BLE] beacon '%s' from %02X:%02X:%02X:%02X:%02X:%02X\r\n", BEACON_NAME,
        a[5], a[4], a[3], a[2], a[1], a[0]);
}

static void loop(void)
{
  static uint16_t count;

  vTaskDelay(pdMS_TO_TICKS(1000U));
  count++;
  Beacon[4] = (uint8_t)(count >> 8);
  Beacon[5] = (uint8_t)count;
  (void)set_adv_data();   /* the next advertisement carries it */
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
