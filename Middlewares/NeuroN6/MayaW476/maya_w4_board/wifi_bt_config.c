/**
  ******************************************************************************
  * @file    wifi_bt_config.c
  * @brief   MAYA-W476 board bring-up: PDn power sequencing and the first SDIO
  *          probe.
  *
  * One active-low PDn replaces WL_REG_ON and BT_REG_ON; the module rail is
  * always on, there is no card-detect, and the 32 kHz clock is on-module.
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

#include "wifi_bt_config.h"
#include "maya_w4_pinmap.h"
#include "stm32n6_sdio_host.h"

#include "cmsis_os2.h"          /* osDelay */
#include "neuron6_wifi.h"       /* the module's user count */


/* Power sequencing. PDn powers Wi-Fi and BLE together, so this stack claims a
   BSP module user: uncounted, BSP_BT_Init would cycle PDn and erase the
   firmware loaded here. */

static uint8_t s_counted;

void MAYA_W4_PowerCycle(void)
{
    /* Always a full PDn cycle: a failed firmware download leaves CMD52
       working but the download stuck. The dwells far exceed the datasheet
       minima (>= 1 us low, ~20 ms after release) so clocks and PLL settle
       before the first CMD53. */
    MAYA_W4_AssertPowerDown();
    osDelay(500U);
    MAYA_W4_ReleasePowerDown();
    osDelay(1000U);

    if (s_counted == 0U) {
        s_counted = 1U;
        (void)BSP_WIFI_ModuleClaim();
    }
}

void MAYA_W4_PowerOff(void)
{
    if (s_counted != 0U) {
        /* PDn goes low only once BLE has let go of the module too. */
        s_counted = 0U;
        (void)BSP_WIFI_ModulePowerOff();
    } else {
        MAYA_W4_AssertPowerDown();
    }
}


/* First SDIO probe. */

#define CCCR_SDIO_REVISION_OFFSET   0x00U   /* CCCR/FBR section 6.9 */

maya_w4_status_t MAYA_W4_FirstLightSDIO(uint8_t *out_sdio_revision)
{
    STM32N6_SDIO_BindHandle(MAYA_WIFI_SDIO_HANDLE);

    MAYA_W4_PowerCycle();

    /* The driver's own sdio_drv_* path: init, then CMD52 on CCCR 0. */
    extern int sdio_drv_init(void (*cd_int)(int));
    extern int sdio_drv_creg_read(int addr, int fn, uint32_t *resp);

    if (sdio_drv_init(NULL) != 0 /* WM_SUCCESS */) {
        return MAYA_W4_ERR_SDIO_INIT;
    }

    uint32_t resp = 0U;
    if (sdio_drv_creg_read(CCCR_SDIO_REVISION_OFFSET, 0 /* fn0 */, &resp) != 1) {
        return MAYA_W4_ERR_SDIO_CMD52;
    }

    if (out_sdio_revision != NULL) {
        *out_sdio_revision = (uint8_t)(resp & 0xFFU);
    }
    return MAYA_W4_OK;
}
