/**
  * @file    neuron6_com.c
  * @brief   Console over USB CDC for the Neuro N6.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6_com.h"
#include "neuron6.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "tusb.h"
#include "device/usbd_pvt.h"   /* usbd_edpt_busy() */

/* Found at reset, this makes the first stage boot loader stop in DFU mode. */
#define COM_FORCE_DFU_MAGIC   0xF4CDF4u
#define COM_BKP_DFU_INDEX     2U
#define COM_BKP_SPARE_INDEX   1U

/* Backup registers arming a staged update and staged settings for the next reset:
 * N6_OTA_TRIGGER_BKPR and N6_ASSET_TRIGGER_BKPR in N6OTA/n6_ota_layout.h. */
#define COM_BKP_OTA_TRIGGER_INDEX    20U
#define COM_BKP_ASSET_TRIGGER_INDEX  16U

/* AHB5 reset mask ST's own STM32N6 bring-up applies around the USB PHY. */
#define COM_AHB5_USB_RESET    0x00800000U

static PCD_HandleTypeDef s_hpcd;
static volatile uint8_t  s_initialised;
static volatile uint8_t  s_dfu_touch_armed;   /* set when a host selects 1200 baud */
static volatile uint32_t s_inside;            /* callers inside the device stack */
static uint8_t           s_tx_ep;             /* the console's bulk IN endpoint, once found */

/* Longest BSP_COM_DeInit waits for callers to leave the stack. */
#define COM_DEINIT_WAIT_MS    50U

/* Most bytes per write or read: the device stack counts them in 16 bits. */
#define COM_MAX_TRANSFER      0xFFFFU

/* Under FreeRTOS one task services the stack; the USB interrupt wakes it. */
#if (CFG_TUSB_OS == OPT_OS_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"
static TaskHandle_t      s_pump_task;
#endif

/* USB peripheral bring-up. Order sensitive: keep the clock selection, the AHB5 resets
 * around the PHY and the PHY register programming in the sequence shown. */

/* Between HAL releases ST swapped which USB clock-source macro family (RCC_USBOTGHS1*,
 * RCC_USBPHY1*) names the PHY reference and which the kernel clock. Wanted either way:
 * kernel clock from HSE/2, reference straight from the oscillator. */
#if defined(RCC_USBPHY1CLKSOURCE_HSE_DIV2)
/* Newer HAL. */
#define COM_PHY1_KERNEL_SOURCE      RCC_USBPHY1CLKSOURCE_HSE_DIV2
#define COM_OTGHS1_REF_SOURCE       RCC_USBOTGHS1CLKSOURCE_HSE_DIRECT
#else
/* Older HAL revisions. */
#define COM_PHY1_KERNEL_SOURCE      RCC_USBOTGHS1CLKSOURCE_HSE_DIV2
#define COM_OTGHS1_REF_SOURCE       RCC_USBPHY1REFCLKSOURCE_HSE_DIRECT
#endif

void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
  RCC_PeriphCLKInitTypeDef clk = {0};

  if (hpcd->Instance != USB1_OTG_HS) { return; }

  /* Both clock sources must be selected before the peripheral leaves reset. */
  clk.PeriphClockSelection    = RCC_PERIPHCLK_USBOTGHS1;
  clk.UsbOtgHs1ClockSelection = COM_OTGHS1_REF_SOURCE;
  (void)HAL_RCCEx_PeriphCLKConfig(&clk);

  clk.PeriphClockSelection  = RCC_PERIPHCLK_USBPHY1;
  clk.UsbPhy1ClockSelection = COM_PHY1_KERNEL_SOURCE;
  (void)HAL_RCCEx_PeriphCLKConfig(&clk);

  /* Hold controller and PHY in reset while the divider is selected. */
  LL_AHB5_GRP1_ForceReset(COM_AHB5_USB_RESET);
  __HAL_RCC_USB1_OTG_HS_FORCE_RESET();
  __HAL_RCC_USB1_OTG_HS_PHY_FORCE_RESET();
  LL_RCC_HSE_SelectHSEDiv2AsDiv2Clock();
  LL_AHB5_GRP1_ReleaseReset(COM_AHB5_USB_RESET);

  __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();

  /* The PHY control registers need a few clock cycles before they accept writes. */
  for (volatile uint32_t i = 0U; i < 10U; i++) { __NOP(); }

  /* FSEL = 24 MHz reference, high-speed enabled, UTMI level 2, PHY on. */
  USB1_HS_PHYC->USBPHYC_CR &= ~(0x7U << 4);
  USB1_HS_PHYC->USBPHYC_CR |= (0x1U << 16) | (0x2U << 4) | (0x1U << 2) | 0x1U;

  __HAL_RCC_USB1_OTG_HS_PHY_RELEASE_RESET();
  for (volatile uint32_t i = 0U; i < 10U; i++) { __NOP(); }
  __HAL_RCC_USB1_OTG_HS_RELEASE_RESET();

  __HAL_RCC_USB1_OTG_HS_PHY_CLK_ENABLE();

  HAL_PWREx_EnableVddUSB();

  HAL_NVIC_SetPriority(USB1_OTG_HS_IRQn, BSP_COM_IT_PRIORITY, 0U);
  HAL_NVIC_EnableIRQ(USB1_OTG_HS_IRQn);
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
  if (hpcd->Instance != USB1_OTG_HS) { return; }

  HAL_NVIC_DisableIRQ(USB1_OTG_HS_IRQn);
  /* Read back before gating the clock, or a posted write stalls the bus. */
  (void)READ_REG(hpcd->Instance->GUSBCFG);
  __DSB();
  __HAL_RCC_USB1_OTG_HS_CLK_DISABLE();
  __HAL_RCC_USB1_OTG_HS_PHY_CLK_DISABLE();
}

#if defined(USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
/* Registered-callback HAL: it calls the handle's routines, not the weak ones above. */

/* Set once MSP routines are on the handle; like the HAL registration, survives DeInit. */
static uint8_t s_msp_cb_valid;

static int32_t COM_RegisterMsp(pPCD_CallbackTypeDef pInit, pPCD_CallbackTypeDef pDeInit)
{
  if ((pInit == NULL) || (pDeInit == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (HAL_PCD_RegisterCallback(&s_hpcd, HAL_PCD_MSPINIT_CB_ID, pInit) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  if (HAL_PCD_RegisterCallback(&s_hpcd, HAL_PCD_MSPDEINIT_CB_ID, pDeInit) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  s_msp_cb_valid = 1U;

  return BSP_ERROR_NONE;
}

int32_t BSP_COM_RegisterDefaultMspCallbacks(void)
{
  return COM_RegisterMsp(HAL_PCD_MspInit, HAL_PCD_MspDeInit);
}

int32_t BSP_COM_RegisterMspCallbacks(BSP_COM_Cb_t *Callbacks)
{
  if (Callbacks == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  return COM_RegisterMsp(Callbacks->pMspInitCb, Callbacks->pMspDeInitCb);
}
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

/* Public API */

int32_t BSP_COM_Init(void)
{
#if (BSP_ISOLATION_APPLICATION_OWNED == 0U)
  RIMC_MasterConfig_t master = {0};
#endif /* BSP_ISOLATION_APPLICATION_OWNED */

  if (s_initialised != 0U) { return BSP_ERROR_NONE; }

  /* TinyUSB's DWC2 port has no deinit, so the stack cannot restart after BSP_COM_DeInit. */
  if (tud_inited()) { return BSP_ERROR_BUSY; }

  /* Isolation first: the controller's DMA reaches RAM through the memory firewalls. */
  (void)BSP_ISOLATION_Config();

  /* With buffer DMA the controller is a bus master writing straight into RAM. An
   * application that owns the isolation policy admits it itself. */
#if (BSP_ISOLATION_APPLICATION_OWNED == 0U)
#if defined(CPU_IN_SECURE_STATE)
  __HAL_RCC_RIFSC_CLK_ENABLE();
#endif
  master.MasterCID  = RIF_CID_1;
  master.SecPriv    = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_OTG1, &master);
#endif /* BSP_ISOLATION_APPLICATION_OWNED */

  s_hpcd.Instance                     = USB1_OTG_HS;
  s_hpcd.Init.dev_endpoints           = 9U;
  s_hpcd.Init.speed                   = PCD_SPEED_HIGH;
  s_hpcd.Init.phy_itface              = USB_OTG_HS_EMBEDDED_PHY;
  s_hpcd.Init.Sof_enable              = DISABLE;
  s_hpcd.Init.low_power_enable        = DISABLE;
  s_hpcd.Init.lpm_enable              = DISABLE;
  s_hpcd.Init.use_dedicated_ep1       = DISABLE;
  s_hpcd.Init.vbus_sensing_enable     = DISABLE;
  s_hpcd.Init.dma_enable              = ENABLE;

#if defined(USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
  /* The default routines, unless the application registered its own first. */
  if ((s_msp_cb_valid == 0U) && (BSP_COM_RegisterDefaultMspCallbacks() != BSP_ERROR_NONE))
  {
    return BSP_ERROR_MSP_FAILURE;
  }
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

  if (HAL_PCD_Init(&s_hpcd) != HAL_OK) { return BSP_ERROR_PERIPH_FAILURE; }

  /* From here TinyUSB drives the controller; the HAL PCD only brought it up. */
  if (!tud_init(BOARD_TUD_RHPORT)) { return BSP_ERROR_PERIPH_FAILURE; }

  s_initialised = 1U;
  return BSP_ERROR_NONE;
}

/* A caller enters the device stack only while the console is up. COM_Enter counts it
 * in before reading the flag, so BSP_COM_DeInit (clear flag, wait for the count) sees
 * it or is seen by it. On 1 the caller goes in and must call COM_Leave. */
static void COM_Leave(void)
{
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  s_inside--;
  __set_PRIMASK(primask);
}

static uint8_t COM_Enter(void)
{
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  s_inside++;
  __set_PRIMASK(primask);
  if (s_initialised == 0U)
  {
    COM_Leave();
    return 0U;
  }
  return 1U;
}

int32_t BSP_COM_DeInit(void)
{
  const uint32_t start = HAL_GetTick();

  if (s_initialised == 0U) { return BSP_ERROR_NONE; }

  /* Let callers already in the stack finish before the clocks go: a write to the
   * controller with its clock off stalls the bus. */
  s_initialised = 0U;
  __DSB();
  while ((s_inside != 0U) && ((HAL_GetTick() - start) < COM_DEINIT_WAIT_MS))
  {
#if (CFG_TUSB_OS == OPT_OS_FREERTOS)
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
      vTaskDelay(1U);
    }
#endif
  }

  (void)HAL_PCD_DeInit(&s_hpcd);
  return BSP_ERROR_NONE;
}

void BSP_COM_Task(void)
{
  if (COM_Enter() == 0U) { return; }

#if (CFG_TUSB_OS == OPT_OS_FREERTOS)
  /* Non-blocking: under FreeRTOS a bare tud_task() waits on the event queue. */
  tud_task_ext(0U, false);
#else
  /* With no OS, tud_task() drains what is pending and returns. */
  tud_task();
#endif

  /* 1200 baud selected, then the port closed: the host is asking for the bootloader. */
  if ((s_dfu_touch_armed != 0U) && !tud_cdc_n_connected(BSP_COM_ITF))
  {
    BSP_COM_RebootToBootloader();
  }

  COM_Leave();
}

void BSP_COM_TaskWait(uint32_t Timeout_ms)
{
#if (CFG_TUSB_OS == OPT_OS_FREERTOS)
  /* Sleep until the interrupt signals an event, or the timeout passes. */
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
  {
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(Timeout_ms));
  }
#else
  (void)Timeout_ms;
#endif
  BSP_COM_Task();
}

void BSP_COM_SetPumpTask(void *pTask)
{
#if (CFG_TUSB_OS == OPT_OS_FREERTOS)
  s_pump_task = (TaskHandle_t)pTask;
#else
  (void)pTask;
#endif
}

int32_t BSP_COM_IsConnected(void)
{
  return (s_initialised != 0U) && tud_cdc_n_connected(BSP_COM_ITF) ? 1 : 0;
}

int32_t BSP_COM_Write(const uint8_t *pData, uint32_t Length)
{
  uint32_t n = 0U;

  if (pData == NULL) { return BSP_ERROR_WRONG_PARAM; }
  if (COM_Enter() == 0U) { return 0; }

  if (tud_cdc_n_connected(BSP_COM_ITF))
  {
    n = tud_cdc_n_write(BSP_COM_ITF, pData, (Length > COM_MAX_TRANSFER) ? COM_MAX_TRANSFER : Length);
    (void)tud_cdc_n_write_flush(BSP_COM_ITF);
  }

  COM_Leave();
  return (int32_t)n;
}

int32_t BSP_COM_Read(uint8_t *pData, uint32_t Length)
{
  uint32_t n = 0U;

  if (pData == NULL) { return BSP_ERROR_WRONG_PARAM; }
  if (COM_Enter() == 0U) { return 0; }

  /* Not gated on DTR as writes are: bytes a host sent just before closing still count. */
  if (tud_cdc_n_available(BSP_COM_ITF))
  {
    n = tud_cdc_n_read(BSP_COM_ITF, pData, (Length > COM_MAX_TRANSFER) ? COM_MAX_TRANSFER : Length);
  }

  COM_Leave();
  return (int32_t)n;
}

int32_t BSP_COM_Printf(const char *Format, ...)
{
  char    buf[256];
  va_list ap;
  int     n;

  if (Format == NULL) { return BSP_ERROR_WRONG_PARAM; }

  va_start(ap, Format);
  n = vsnprintf(buf, sizeof buf, Format, ap);
  va_end(ap);

  if (n < 0) { return BSP_ERROR_UNKNOWN_FAILURE; }
  if ((size_t)n >= sizeof buf) { n = (int)(sizeof buf) - 1; }

  return BSP_COM_Write((const uint8_t *)buf, (uint32_t)n);
}

/* The bulk IN endpoint of the BSP_COM_ITF-th CDC data interface, or 0 if none. */
static uint8_t COM_TxEndpoint(void)
{
  const uint8_t *p;
  const uint8_t *end;
  uint8_t        data_itf = 0U;
  uint8_t        console = 0U;

  if (s_tx_ep != 0U) { return s_tx_ep; }

  p = tud_descriptor_configuration_cb(0U);
  if (p == NULL) { return 0U; }
  end = p + tu_le16toh(((const tusb_desc_configuration_t *)(const void *)p)->wTotalLength);

  for (; (p < end) && (tu_desc_len(p) != 0U); p = tu_desc_next(p))
  {
    if (tu_desc_type(p) == TUSB_DESC_INTERFACE)
    {
      const tusb_desc_interface_t *itf = (const tusb_desc_interface_t *)(const void *)p;

      console = 0U;
      if (itf->bInterfaceClass == (uint8_t)TUSB_CLASS_CDC_DATA)
      {
        console = (data_itf == (uint8_t)BSP_COM_ITF) ? 1U : 0U;
        data_itf++;
      }
    }
    else if ((console != 0U) && (tu_desc_type(p) == TUSB_DESC_ENDPOINT))
    {
      const uint8_t address = ((const tusb_desc_endpoint_t *)(const void *)p)->bEndpointAddress;

      if (tu_edpt_dir(address) == TUSB_DIR_IN)
      {
        s_tx_ep = address;
        break;
      }
    }
    else
    {
      /* Neither: nothing to note. */
    }
  }
  return s_tx_ep;
}

void BSP_COM_Flush(uint32_t Timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) < Timeout_ms)
  {
    /* The stack sends by itself only full packets or printf lines: push out the rest. */
    if (COM_Enter() != 0U)
    {
      (void)tud_cdc_n_write_flush(BSP_COM_ITF);
      COM_Leave();
    }
#if (CFG_TUSB_OS == OPT_OS_FREERTOS)
    /* Another task owns the pump: the stack must not be pumped from two tasks. */
    if ((s_pump_task != NULL) &&
        (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) &&
        (xTaskGetCurrentTaskHandle() != s_pump_task))
    {
      vTaskDelay(1U);
    }
    else
#endif
    {
      BSP_COM_Task();
    }
    if (BSP_COM_IsConnected() == 0) { break; }
    /* An empty FIFO is not enough: the endpoint may not have sent the last bytes. */
    if ((tud_cdc_n_write_available(BSP_COM_ITF) >= CFG_TUD_CDC_TX_BUFSIZE) &&
        ((COM_TxEndpoint() == 0U) || !usbd_edpt_busy(BOARD_TUD_RHPORT, COM_TxEndpoint())))
    {
      break;
    }
  }
}

void BSP_COM_RebootToBootloader(void)
{
  volatile uint32_t *bkp = &TAMP->BKP0R;

  HAL_PWR_EnableBkUpAccess();
  /* Cancel any staged update or settings: the upload replaces the application. */
  bkp[COM_BKP_OTA_TRIGGER_INDEX]   = 0U;
  bkp[COM_BKP_ASSET_TRIGGER_INDEX] = 0U;
  bkp[COM_BKP_DFU_INDEX]   = COM_FORCE_DFU_MAGIC;
  bkp[COM_BKP_SPARE_INDEX] = 0U;
  __DSB();
  __ISB();
  NVIC_SystemReset();
}

void BSP_COM_IRQHandler(void)
{
  tud_int_handler(BOARD_TUD_RHPORT);

#if (CFG_TUSB_OS == OPT_OS_FREERTOS)
  /* Wake the pump task only when the interrupt queued an event. */
  if ((s_pump_task != NULL) && tud_task_event_ready() &&
      (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING))
  {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_pump_task, &woken);
    portYIELD_FROM_ISR(woken);
  }
#endif
}

/* TinyUSB callbacks */

/* A host requests the bootloader by opening the port at 1200 baud and closing it:
 * the baud selection arms the reboot, the DTR drop fires it. */
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *coding)
{
  if ((itf == BSP_COM_ITF) && (coding != NULL))
  {
    s_dfu_touch_armed = (coding->bit_rate == BSP_COM_DFU_TOUCH_BAUD) ? 1U : 0U;
  }
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void)rts;

  if ((itf == BSP_COM_ITF) && (s_dfu_touch_armed != 0U) && !dtr)
  {
    BSP_COM_RebootToBootloader();
  }
}

/* newlib stdio */

/* stdout and stderr: syscalls.c writes through the weak __io_putchar(). */
#if !defined(BSP_COM_NO_PRINTF_RETARGET)
int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;

  /* Counted in like BSP_COM_Write, so BSP_COM_DeInit waits for it before gating clocks. */
  if (COM_Enter() != 0U)
  {
    if (BSP_COM_IsConnected() != 0)
    {
      (void)tud_cdc_n_write(BSP_COM_ITF, &c, 1U);
      if (c == (uint8_t)'\n') { (void)tud_cdc_n_write_flush(BSP_COM_ITF); }
    }
    COM_Leave();
  }
  return ch;
}

/* stdin: waits for the next character. Bare metal pumps the stack meanwhile; under
 * FreeRTOS the USB task fills the FIFO and the caller sleeps a tick at a time. */
int __io_getchar(void)
{
  uint8_t c = 0U;

  while (BSP_COM_Read(&c, 1U) != 1)
  {
#if defined(BSP_USE_FREERTOS) && (BSP_USE_FREERTOS == 1)
    HAL_Delay(1U);
#else
    BSP_COM_Task();
#endif
  }
  return (int)c;
}
#endif
