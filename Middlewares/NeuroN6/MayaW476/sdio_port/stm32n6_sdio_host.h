/**
  ******************************************************************************
  * @file    stm32n6_sdio_host.h
  * @brief   STM32N6 SDIO host port for NXP wifi_nxp / IW610 (MAYA-W476):
  *          wifi_nxp's sdio_drv_* on STM32 HAL_SDIO instead of NXP's
  *          fsl_sdio/fsl_usdhc.
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

#ifndef STM32N6_SDIO_HOST_H
#define STM32N6_SDIO_HOST_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32n6xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Bind the shim to SDMMC1's SDIO handle; call before sdio_drv_init(). */
void STM32N6_SDIO_BindHandle(SDIO_HandleTypeDef *hsdio);

/** @brief Forward the SDMMC IRQ to the HAL. Required: without it the
 *         function-1 callback and DMA completions never happen. */
void STM32N6_SDIO_IRQHandler(void);

/** @brief After wlan_init() fails: read the IW610's firmware-download
 *         registers into the sdio_fwprobe_* globals (CMD52 only; needs an
 *         enumerated bus). */
void STM32N6_SDIO_FwProbe(void);

/** @brief After STM32N6_SDIO_FwProbe(): hard PDn cycle, then re-read into
 *         sdio_fwprobe_post_reset_*. evt5c back at 0x0d: the chip is healthy
 *         and was rejecting the image; still 0x04: wiring, straps or silicon. */
void STM32N6_SDIO_PostResetProbe(void);

/* sdio_drv_* is declared in wifi_nxp's mlan_sdio_api.h. */

#ifdef __cplusplus
}
#endif

#endif /* STM32N6_SDIO_HOST_H */
