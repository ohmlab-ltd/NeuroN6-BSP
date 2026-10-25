/**
 ******************************************************************************
 * @file    maya_w4_sha256.h
 * @brief   SHA-256 and HMAC-SHA256 (FIPS 180-4, RFC 2104), for the link key.
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

#ifndef MAYA_W4_SHA256_H
#define MAYA_W4_SHA256_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t h[8];
  uint64_t bytes;       /* message length so far */
  uint8_t  block[64];
  uint32_t fill;        /* bytes waiting in block */
} maya_w4_sha256_t;

void maya_w4_sha256_init(maya_w4_sha256_t *c);
void maya_w4_sha256_update(maya_w4_sha256_t *c, const void *data, size_t len);
void maya_w4_sha256_final(maya_w4_sha256_t *c, uint8_t out[32]);

/* HMAC-SHA256 of the two parts m1 then m2, either of which may be empty. */
void maya_w4_hmac_sha256(const uint8_t *key, size_t key_len,
                         const void *m1, size_t n1, const void *m2, size_t n2,
                         uint8_t out[32]);

/* Zero n bytes that held a secret; unlike memset, never optimised away. */
void maya_w4_wipe(void *p, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* MAYA_W4_SHA256_H */
