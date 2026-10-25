/**
 ******************************************************************************
 * @file    n6_asset_host.c
 * @brief   The asset transfer commands, for an application's host-command
 *          router to call.
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

#include "n6_asset_host.h"
#include "n6_asset.h"
#include "n6_host_contract.h"

#include <stdbool.h>
#include <string.h>

/* The transfer under way. */
static uint8_t  s_buf[N6_ASSET_MAX_BYTES];
static uint32_t s_len, s_crc, s_ver, s_got;
static bool     s_active;
static uint8_t  s_record;   /* 0: the asset (AST0); else the record type (ASR0) */

n6_asset_host_t n6_asset_host_command(const uint8_t *cmd, uint32_t len)
{
  uint32_t magic;

  if ((cmd == NULL) || (len < 4u)) return N6_ASSET_HOST_NONE;
  memcpy(&magic, cmd, 4u);

  if ((magic == N6_CMD_AST0) && (len >= sizeof(n6_cmd_ast0_t))) {
    n6_cmd_ast0_t c0;
    memcpy(&c0, cmd, sizeof(c0));
    s_active = (c0.total_len > 0u) && (c0.total_len <= sizeof(s_buf));
    s_len = c0.total_len; s_crc = c0.crc32; s_ver = c0.version;
    s_got = 0u;
    s_record = 0u;
    return N6_ASSET_HOST_TAKEN;
  }
  /* A zero-length record removes it: the one transfer that may carry no
   * data. */
  if ((magic == N6_CMD_ASR0) && (len >= sizeof(n6_cmd_asr0_t))) {
    n6_cmd_asr0_t r0;
    memcpy(&r0, cmd, sizeof(r0));
    s_active = (r0.type != 0u) && (r0.total_len <= sizeof(s_buf));
    s_len = r0.total_len; s_crc = r0.crc32; s_ver = 0u;
    s_got = 0u;
    s_record = r0.type;
    return N6_ASSET_HOST_TAKEN;
  }
  if ((magic == N6_CMD_AST1) && (len >= sizeof(n6_cmd_ast1_t))) {
    n6_cmd_ast1_t c1;
    memcpy(&c1, cmd, sizeof(c1));
    const uint32_t avail = len - sizeof(n6_cmd_ast1_t);
    const uint32_t n = (c1.len < avail) ? c1.len : avail;
    if (s_active && (n > 0u) && (((uint32_t)c1.offset + n) <= s_len)) {
      memcpy(&s_buf[c1.offset], cmd + sizeof(n6_cmd_ast1_t), n);
      s_got += n;
    }
    return N6_ASSET_HOST_TAKEN;
  }
  if ((magic == N6_CMD_AST2) && (len >= sizeof(n6_cmd_ast2_t))) {
    const bool staged = s_active && (s_got >= s_len) &&
                        (n6_host_crc32(0u, s_buf, s_len) == s_crc) &&
                        ((s_record != 0u) ? n6_asset_record_stage(s_record, s_buf, s_len)
                                          : n6_asset_stage(s_buf, s_len, s_ver));
    s_active = false;
    return staged ? N6_ASSET_HOST_STAGED : N6_ASSET_HOST_TAKEN;
  }
  return N6_ASSET_HOST_NONE;
}
