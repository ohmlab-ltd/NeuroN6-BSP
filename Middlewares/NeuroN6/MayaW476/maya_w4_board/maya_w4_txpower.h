/**
 ******************************************************************************
 * @file    maya_w4_txpower.h
 * @brief   Regulatory TX-power cap for the MAYA-W476 (IW610), 2.4 GHz.
 *
 * REGULATORY: EMC-tested at 8 dBm conducted (~9.6 dBm EIRP with the 1.56 dBi
 * antenna; EN 300 328 and FCC 15.247 allow 20 dBm EIRP). Applied at every
 * bring-up with no API to raise it (MAYA-W4 System Integration Manual 7.1.2,
 * FCC KDB 594280).
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

#ifndef MAYA_W4_TXPOWER_H
#define MAYA_W4_TXPOWER_H

#ifdef __cplusplus
extern "C" {
#endif

/** Conducted TX power cap, dBm. Not a tuning knob: changing it voids the EMC
 *  tests. */
#define MAYA_W4_TX_POWER_DBM   8u

/**
 * Apply the 2.4 GHz TRPC limit table (all channels and modulation groups) and
 * the station ceiling, then log the limit the firmware reports.
 * @note Call after wlan_init/wlan_start: wlan_init applies the driver's own
 *       regional table over any earlier call. Failures are logged, not fatal.
 */
void maya_w4_apply_tx_power_cap(void);

#ifdef __cplusplus
}
#endif

#endif /* MAYA_W4_TXPOWER_H */
