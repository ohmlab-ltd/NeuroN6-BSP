/**
 ******************************************************************************
 * @file    maya_w4_hw.c
 * @brief   MAYA-W476 (NXP IW610) bring-up: SDMMC1/SDIO, control GPIOs and
 *          firmware download.
 *
 * The firmware is read in place from memory-mapped XSPI flash at 0x91E00000,
 * where harness/n6weights.py writes it (from
 * Middlewares/NeuroN6/MayaW476/firmware/firmware.json). If it is missing the
 * region reads 0xFF and wlan_init() fails.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 * This software is licensed under the terms in
 * Middlewares/NeuroN6/MayaW476/LICENSE-MIT.txt.
 */

#include "stm32n6xx_hal.h"
#include "maya_w4_pinmap.h"      /* hsdio1 extern + MAYA_* pin macros + power helpers */
#include "wifi_bt_config.h"      /* MAYA_W4_FirstLightSDIO, maya_w4_status_t */
#include "wlan.h"                /* wlan_init, WM_SUCCESS */

#include <stdint.h>
#include <stdio.h>

/* Defined by the application. */
extern void NeuroN6_DbgPrint(const char *msg);

/* IW610 firmware: a pointer into memory-mapped XSPI flash, and its length. */
extern const unsigned char *wlan_fw_bin;
extern const unsigned int    wlan_fw_bin_len;

/* SDIO handle for the whole wireless port (declared in maya_w4_pinmap.h). */
SDIO_HandleTypeDef hsdio1;

/* Required: on the weak default handler the SDIO DMA-completion semaphore is
 * never given and wlan_init()'s CMD53 firmware download hangs. Defined here so
 * USB-only applications keep the weak handler. */
extern void STM32N6_SDIO_IRQHandler(void);   /* sdio_port/stm32n6_sdio_host.c */

void SDMMC1_IRQHandler(void)
{
  STM32N6_SDIO_IRQHandler();
}

/* SDMMC1 kernel clock (IC4) and pins: PC8-PC11 D0-D3, PC12 CK, PH2 CMD.
 * Overrides the HAL's weak HAL_SDIO_MspInit. */
void HAL_SDIO_MspInit(SDIO_HandleTypeDef *hsdio)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  if (hsdio->Instance == SDMMC1)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC1;
    PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_IC4;
    PeriphClkInitStruct.ICSelection[RCC_IC4].ClockSelection = RCC_ICCLKSOURCE_PLL1;
    /* EMC: do not change without a new radiated-emissions test. This divider
     * and the CLKDIV in stm32n6_sdio_host.c must both be odd, so the total
     * division of the 800 MHz PLL1 is not divisible by 4 and no SDMMC_CK
     * harmonic lands on 200 MHz. IC4 = 7 with CLKDIV 7: N = 98, an 8.163 MHz
     * pin clock, the configuration EMC-tested at 8 dBm. */
    PeriphClkInitStruct.ICSelection[RCC_IC4].ClockDivider   = 7;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      /* Non-fatal: the SDIO bring-up fails and MayaW476_Start() falls back to
       * USB. */
      NeuroN6_DbgPrint("[MayaW476] WARN: SDMMC1 clock config (IC4<-PLL1/7) "
                       "FAILED - WiFi will fall back to USB.\r\n");
    }

    __HAL_RCC_SDMMC1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* D0-D3 (PC8/9/10/11) + CK (PC12), AF10. */
    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    /* EMC: HIGH, not VERY_HIGH, keeps the edges soft (PC8-12 and PH2). */
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_SDMMC1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* CMD (PH2) - same settings. */
    GPIO_InitStruct.Pin       = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
  }
}

void HAL_SDIO_MspDeInit(SDIO_HandleTypeDef *hsdio)
{
  if (hsdio->Instance == SDMMC1)
  {
    /* Read back before gating the clock: a write the gate overtakes never
     * completes and stalls the next bus access. */
    (void)READ_REG(hsdio->Instance->POWER);
    __DSB();
    __HAL_RCC_SDMMC1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOH, GPIO_PIN_2);
  }
}

/* No HAL_SDIO_Init(): the IW610 rejects its CMD11 3.3 V to 1.8 V switch.
 * sdio_drv_init (via MAYA_W4_FirstLightSDIO) programs CLKCR/POWER itself and
 * enumerates manually (CMD0/CMD5/CMD3/CMD7) after MAYA_W4_PowerCycle. */

/* MAYA control GPIOs (PDn, WL/NB wake lines). */
static void maya_w4_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* Levels before the pins become outputs: NB_WAKE_IN low (802.15.4 wake,
   * unused), PDn high (module released), WL_WAKE_IN low. */
  HAL_GPIO_WritePin(NB_WAKE_IN_MAYA_GPIO_Port,      NB_WAKE_IN_MAYA_Pin,      GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PDn_pin_MAYA_GPIO_Port,         PDn_pin_MAYA_Pin,         GPIO_PIN_SET);
  HAL_GPIO_WritePin(WL_WAKE_IN_MAYA_GPIO_Port,      WL_WAKE_IN_MAYA_Pin,      GPIO_PIN_RESET);

  /* NB_WAKE_OUT (PE10): module to host. */
  GPIO_InitStruct.Pin  = NB_WAKE_OUT_MAYA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NB_WAKE_OUT_MAYA_GPIO_Port, &GPIO_InitStruct);

  /* NB_WAKE_IN (PE5): host to module. */
  GPIO_InitStruct.Pin   = NB_WAKE_IN_MAYA_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NB_WAKE_IN_MAYA_GPIO_Port, &GPIO_InitStruct);

  /* PDn (PB5). */
  GPIO_InitStruct.Pin   = PDn_pin_MAYA_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PDn_pin_MAYA_GPIO_Port, &GPIO_InitStruct);

  /* WL_WAKE_IN (PG0): host to module. WL_WAKE_OUT (PB7) is unused. */
  GPIO_InitStruct.Pin   = WL_WAKE_IN_MAYA_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WL_WAKE_IN_MAYA_GPIO_Port, &GPIO_InitStruct);
}

/* GPIO, SDIO, then firmware download. 0 once the IW610 firmware runs,
 * negative per failing stage. Task context only (wifi_nxp delays and
 * semaphores). */
int maya_w4_sdio_bringup(void)
{
  char dbg[160];

  NeuroN6_DbgPrint("[MayaW476] stage 1/3: MAYA control GPIO init (PDn etc.)...\r\n");

  /* Before FirstLightSDIO, which pulses PDn. The SDMMC1 pins and clock come
   * later, in HAL_SDIO_MspInit. */
  maya_w4_gpio_init();

  NeuroN6_DbgPrint("[MayaW476] stage 2/3: SDIO bring-up (PDn power-cycle + "
                   "sdio_drv_init manual CMD0/CMD5/CMD3/CMD7 + CCCR CMD52)...\r\n");

  /* PDn pulse to POR, manual enumeration, then a CCCR CMD52 read. */
  uint8_t cccr_rev = 0U;
  maya_w4_status_t st = MAYA_W4_FirstLightSDIO(&cccr_rev);
  if (st != MAYA_W4_OK)
  {
    snprintf(dbg, sizeof(dbg),
             "[MayaW476] SDIO bring-up FAILED (status=%d) - staying on USB. "
             "Check module wiring + PDn/SDMMC1 pins.\r\n", (int)st);
    NeuroN6_DbgPrint(dbg);
    return -11;
  }
  snprintf(dbg, sizeof(dbg),
           "[MayaW476] SDIO up. CCCR SDIO revision = 0x%02X\r\n",
           (unsigned)cccr_rev);
  NeuroN6_DbgPrint(dbg);

  NeuroN6_DbgPrint("[MayaW476] stage 3/3: wlan_init (IW610 firmware download "
                   "from XSPI @0x91E00000 over CMD53)...\r\n");

  {
    const volatile uint8_t *fw = (const volatile uint8_t *)wlan_fw_bin;
    uint32_t first = ((uint32_t)fw[0]) |
                     ((uint32_t)fw[1] << 8) |
                     ((uint32_t)fw[2] << 16) |
                     ((uint32_t)fw[3] << 24);
    uint32_t last = ((uint32_t)fw[wlan_fw_bin_len - 4U]) |
                    ((uint32_t)fw[wlan_fw_bin_len - 3U] << 8) |
                    ((uint32_t)fw[wlan_fw_bin_len - 2U] << 16) |
                    ((uint32_t)fw[wlan_fw_bin_len - 1U] << 24);
    snprintf(dbg, sizeof(dbg),
             "[MayaW476] FW blob addr=%p len=%u first=%08lX last=%08lX\r\n",
             (const void *)wlan_fw_bin,
             (unsigned)wlan_fw_bin_len,
             (unsigned long)first,
             (unsigned long)last);
    NeuroN6_DbgPrint(dbg);

    if ((first == 0xFFFFFFFFUL) || (first == 0x00000000UL) ||
        (last == 0xFFFFFFFFUL)) {
      NeuroN6_DbgPrint("[MayaW476] FW blob looks blank or truncated; skipping wlan_init.\r\n");
      return -12;
    }
  }

  /* Download over CMD53, then wait for the chip to report the firmware running. */
  int wlan_rc = wlan_init(wlan_fw_bin, wlan_fw_bin_len);
  if (wlan_rc != WM_SUCCESS)
  {
    snprintf(dbg, sizeof(dbg),
             "[MayaW476] wlan_init FAILED (rc=%d). Is the IW610 firmware "
             "flashed to XSPI 0x91E00000?\r\n", wlan_rc);
    NeuroN6_DbgPrint(dbg);
    return -12;
  }

  NeuroN6_DbgPrint("[MayaW476] wlan_init OK - IW610 firmware running.\r\n");
  return 0;
}
