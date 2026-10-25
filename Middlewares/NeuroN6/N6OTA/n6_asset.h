/**
 ******************************************************************************
 * @file    n6_asset.h
 * @brief   Small, persistent, host-authored data (the asset store).
 *
 * Staged into PSRAM and programmed by the FSBL, like a firmware update.
 * Reads retry against unreliable mapped-flash reads; always handle a false
 * return from n6_asset_read().
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 * This software is licensed under the terms in
 * Middlewares/NeuroN6/N6OTA/LICENSE-MIT.txt.
 */

#ifndef N6_ASSET_H
#define N6_ASSET_H

#include <stdint.h>
#include <stdbool.h>
#include "n6_ota_layout.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Record types. n6_asset_read()/n6_asset_stage() use the user record; the
 * others share the store through n6_asset_record_*(), and writing one never
 * disturbs another. Must match N6_ASSET_REC_* in n6_host_contract.h. */
#ifndef N6_ASSET_REC_USER
#define N6_ASSET_REC_USER   1u
#define N6_ASSET_REC_WIFI   2u   /* MayaW476: the WiFi settings a host stored */
#define N6_ASSET_REC_AP_PASSWORD 3u   /* MayaW476: the board's own SoftAP password */
#endif

/* Read the user asset; false if none is valid. Of the two copies the highest
 * version passing its CRC wins, so an interrupted write never reads as good. */
bool n6_asset_read(void *out, uint32_t max, uint32_t *out_len,
                   uint32_t *out_version);

/* Whether a valid asset exists, with its version and length:
 * n6_asset_read() refuses a too-small buffer rather than truncating. */
bool n6_asset_present(uint32_t *out_version, uint32_t *out_len);

/* Highest version a store takes, so one past it can still be numbered. */
#define N6_ASSET_VERSION_MAX  0xFFFFFFFEu

/* Stage `len` bytes for the FSBL to program at the next boot. Version 0 means
 * "newer than stored"; a version not above the stored one is raised. Other
 * records, and anything staged since the last reset, are kept (two stages
 * before a reset apply together) and count against the 4 KB.
 * Returns false, staging nothing and keeping an earlier stage, if the payload
 * does not fit, the version exceeds N6_ASSET_VERSION_MAX or the stored one is
 * already at it, or a firmware image staged for the same reset is longer than
 * N6_OTA_STAGE_CLEAR_OF_ASSET bytes (it overlaps this stage). Also false, with
 * nothing armed, if the staged bytes do not read back. */
bool n6_asset_stage(const void *data, uint32_t len, uint32_t version);

/* Read record `type` (N6_ASSET_REC_*) into `out`. False if it is absent or
 * does not fit; out == NULL only reports its length. */
bool n6_asset_record_read(uint8_t type, void *out, uint32_t max, uint32_t *out_len);

/* Replace record `type` with `len` bytes, or remove it with len 0, keeping
 * every other record, and stage the store as n6_asset_stage() does. */
bool n6_asset_record_stage(uint8_t type, const void *data, uint32_t len);

/* Reset into the FSBL to program the stage. Clears the double-press key
 * (N6_BOOT_PRESS_KEY) first, so the reset is not taken for a second press of
 * the reset button. Does not return. */
void n6_asset_apply(void);

/* True while a stage waits for the FSBL. */
bool n6_asset_pending(void);

/* What the FSBL did with the last staged asset (N6_ASSET_RESULT_*). Zero when
 * it has not run since the backup domain was last cleared. */
uint32_t n6_asset_last_result(void);
const char *n6_asset_result_str(uint32_t code);

#ifdef __cplusplus
}
#endif

#endif /* N6_ASSET_H */
