/**
  ******************************************************************************
  * @file    wifi_bt_config.h
  * @brief   MAYA-W476 board bring-up helpers.
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

#ifndef WIFI_BT_CONFIG_H
#define WIFI_BT_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Bring-up result codes. */
typedef enum {
    MAYA_W4_OK            = 0,
    MAYA_W4_ERR_NO_HSDIO  = -1,  /* SDIO handle never bound */
    MAYA_W4_ERR_SDIO_INIT = -2,  /* enumeration failed (CMD0/CMD5 timed out) */
    MAYA_W4_ERR_SDIO_CMD52 = -3, /* CCCR read failed */
} maya_w4_status_t;

/** Power-cycle the module through PDn and let it settle. Safe to repeat; hold
 *  off SDIO traffic until it returns. */
void MAYA_W4_PowerCycle(void);

/** Release Wi-Fi's hold on the module. PDn goes low (all state lost) once BLE
 *  has released it too. */
void MAYA_W4_PowerOff(void);

/**
 * First SDIO probe: power-cycle, sdio_drv_init() (CMD0/CMD5/...), then a
 * CMD52 read of CCCR 0x00 into *out_sdio_revision if non-NULL (the IW610
 * reports 0x04, SDIO 3.0).
 * @retval MAYA_W4_ERR_SDIO_INIT  check pinmux, clocks or PDn rise time.
 * @retval MAYA_W4_ERR_SDIO_CMD52 enumerated, but bus signalling or level
 *         does not match.
 */
maya_w4_status_t MAYA_W4_FirstLightSDIO(uint8_t *out_sdio_revision);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_BT_CONFIG_H */
