/**
 ******************************************************************************
 * @file    n6_asset.c
 * @brief   Persistent host-authored data (the asset store). See n6_asset.h.
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

#include "n6_asset.h"
#include <string.h>
#include "stm32n6xx.h"
#include "FreeRTOS.h"
#include "task.h"

/* The store holds one payload. The user asset alone is stored plain, byte for
 * byte, so plain-asset readers still read it. Otherwise it is a container:
 *
 *   'N6RC'  { type:u8  reserved:u8  len:u16 LE  data[len] } ...
 *
 * filling the payload exactly; the user asset is record type 1. A payload
 * starting 'N6RC' that does not parse exactly is a plain asset; staging never
 * writes one (such an asset is always wrapped). */
#define REC_MAGIC      0x4352364Eu   /* 'N6RC' little-endian */
#define REC_HDR_BYTES  4u

/* The whole store, read or rebuilt. Shared by every task: each use suspends
 * the scheduler (interrupts still run), for under a millisecond. */
static uint8_t s_work[N6_ASSET_MAX_BYTES];

static void hold(void)
{
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) vTaskSuspendAll();
}

static void release(void)
{
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) (void)xTaskResumeAll();
}

static void bkp_write(uint32_t index, uint32_t value)
{
  /* A write to a write-protected backup domain is silently dropped. */
  PWR->DBPCR |= PWR_DBPCR_DBP;
  (&TAMP->BKP0R)[index] = value;
  __DSB();
}

static uint32_t bkp_read(uint32_t index)
{
  return (&TAMP->BKP0R)[index];
}

static const uint8_t *copy_base(uint32_t copy)
{
  return (const uint8_t *)(N6_OTA_FLASH_MMAP_BASE + N6_ASSET_OFF(copy));
}

/* Invalidate before each try, or a bad read would come back from the cache to
 * every retry. Nothing writes through this window, so no line is dirty. */
static void uncache_copy(uint32_t copy)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  SCB_InvalidateDCache_by_Addr((volatile void *)copy_base(copy), (int32_t)N6_ASSET_SECTOR);
#else
  (void)copy;
#endif
}

/* Read and verify one copy, retrying on mismatch: mapped reads do not always
 * return the same bytes, so one bad CRC or magic does not prove the copy bad.
 * Only erased flash is final. */
static bool read_copy(uint32_t copy, void *out, uint32_t max,
                      uint32_t *out_len, uint32_t *out_version)
{
  for (uint32_t attempt = 0u; attempt < N6_ASSET_READ_TRIES; attempt++) {
    n6_asset_hdr_t h;
    uncache_copy(copy);
    memcpy(&h, copy_base(copy), sizeof(h));

    if (h.magic == 0xFFFFFFFFu) return false;      /* erased: never written */
    if (h.magic != N6_ASSET_MAGIC) continue;
    if (h.hdr_crc != n6_ota_crc32(0u, (const uint8_t *)&h, 16u)) continue;
    if (h.len == 0u || h.len > N6_ASSET_MAX_BYTES) continue;

    const uint8_t *payload = copy_base(copy) + N6_ASSET_HDR_BYTES;
    if (n6_ota_crc32(0u, payload, h.len) != h.crc32) continue;

    if (out != NULL) {
      if (h.len > max) return false;               /* caller's buffer too small */
      memcpy(out, payload, h.len);
      /* Verify what landed in the caller's buffer, not what was read in transit. */
      if (n6_ota_crc32(0u, (const uint8_t *)out, h.len) != h.crc32) continue;
    }
    if (out_len)     *out_len = h.len;
    if (out_version) *out_version = h.version;
    return true;
  }
  return false;
}

/* The newest copy that verifies, whole, into s_work. False when nothing valid
 * is stored. The caller holds the scheduler. */
static bool load(uint32_t *len, uint32_t *version)
{
  uint32_t best_ver = 0u;
  int best = -1;

  /* Highest version that verifies: an interrupted write can leave a newer copy broken. */
  for (uint32_t c = 0u; c < N6_ASSET_COPIES; c++) {
    uint32_t v = 0u, l = 0u;
    if (!read_copy(c, NULL, 0u, &l, &v)) continue;
    if (best < 0 || v > best_ver) { best = (int)c; best_ver = v; }
  }
  return best >= 0 && read_copy((uint32_t)best, s_work, sizeof(s_work), len, version);
}

/* The highest version a copy's header claims, where the header verifies even
 * if its payload does not. */
static uint32_t header_version(void)
{
  uint32_t best = 0u;
  for (uint32_t c = 0u; c < N6_ASSET_COPIES; c++) {
    n6_asset_hdr_t h;
    memcpy(&h, copy_base(c), sizeof(h));
    if (h.magic == N6_ASSET_MAGIC && h.version > best &&
        h.hdr_crc == n6_ota_crc32(0u, (const uint8_t *)&h, 16u)) best = h.version;
  }
  return best;
}

static uint32_t rec_len(const uint8_t *r)
{
  return (uint32_t)r[2] | ((uint32_t)r[3] << 8);
}

static bool is_container(const uint8_t *p, uint32_t len)
{
  uint32_t magic;
  if (len < 4u) return false;
  memcpy(&magic, p, 4u);
  if (magic != REC_MAGIC) return false;
  for (uint32_t pos = 4u; pos < len; pos += REC_HDR_BYTES + rec_len(p + pos)) {
    if (len - pos < REC_HDR_BYTES || p[pos] == 0u) return false;
    if (rec_len(p + pos) > len - pos - REC_HDR_BYTES) return false;
  }
  return true;
}

/* Record `type` in a payload, or NULL. A plain payload is the user asset. */
static const uint8_t *find_record(const uint8_t *p, uint32_t len, uint8_t type,
                                  uint32_t *out_len)
{
  if (!is_container(p, len)) {
    if (type != N6_ASSET_REC_USER) return NULL;
    *out_len = len;
    return p;
  }
  for (uint32_t pos = 4u; pos < len; pos += REC_HDR_BYTES + rec_len(p + pos)) {
    if (p[pos] == type) { *out_len = rec_len(p + pos); return p + pos + REC_HDR_BYTES; }
  }
  return NULL;
}

/* Copy a record out of the store. out == NULL only reports its length. */
static bool read_record(uint8_t type, void *out, uint32_t max,
                        uint32_t *out_len, uint32_t *out_version)
{
  bool ok = false;
  uint32_t len = 0u, ver = 0u, n = 0u;

  hold();
  if (load(&len, &ver)) {
    const uint8_t *r = find_record(s_work, len, type, &n);
    if (r != NULL && (out == NULL || n <= max)) {   /* refuse, never truncate */
      if (out != NULL) memcpy(out, r, n);
      if (out_len)     *out_len = n;
      if (out_version) *out_version = ver;
      ok = true;
    }
  }
  release();
  return ok;
}

bool n6_asset_read(void *out, uint32_t max, uint32_t *out_len,
                   uint32_t *out_version)
{
  return read_record(N6_ASSET_REC_USER, out, max, out_len, out_version);
}

bool n6_asset_present(uint32_t *out_version, uint32_t *out_len)
{
  return read_record(N6_ASSET_REC_USER, NULL, 0u, out_len, out_version);
}

bool n6_asset_record_read(uint8_t type, void *out, uint32_t max, uint32_t *out_len)
{
  return type != 0u && read_record(type, out, max, out_len, NULL);
}

/* Load the armed, intact stage into s_work, so a second stage before the reset
 * builds on the first instead of losing it. The caller holds the scheduler. */
static bool load_staged(uint32_t *len, uint32_t *version)
{
  const n6_asset_hdr_t *h = (const n6_asset_hdr_t *)N6_ASSET_PSRAM_BASE;
  const uint8_t *payload = (const uint8_t *)N6_ASSET_PSRAM_BASE + N6_ASSET_HDR_BYTES;

  if (bkp_read(N6_ASSET_TRIGGER_BKPR) != N6_ASSET_TRIGGER_MAGIC) return false;
  if (h->magic != N6_ASSET_PSRAM_MAGIC) return false;
  if (h->hdr_crc != n6_ota_crc32(0u, (const uint8_t *)h, 16u)) return false;
  if (h->len == 0u || h->len > N6_ASSET_MAX_BYTES) return false;
  memcpy(s_work, payload, h->len);
  if (n6_ota_crc32(0u, s_work, h->len) != h->crc32) return false;
  *len = h->len;
  *version = h->version;
  return true;
}

/* A staged firmware image longer than N6_OTA_STAGE_CLEAR_OF_ASSET reaches up
 * to 32 bytes into the asset stage; staging now would damage it, and the FSBL
 * would refuse it by its CRC. */
static bool image_stage_in_the_way(void)
{
  const n6_ota_psram_hdr_t *img = (const n6_ota_psram_hdr_t *)N6_OTA_PSRAM_BASE;

  if (bkp_read(N6_OTA_TRIGGER_BKPR) != N6_OTA_TRIGGER_MAGIC) return false;
  if (img->magic != N6_OTA_PSRAM_HDR_MAGIC) return false;
  if (img->hdr_crc != n6_ota_crc32(0u, (const uint8_t *)img, 16u)) return false;
  return img->size > N6_OTA_STAGE_CLEAR_OF_ASSET;
}

/* Replace record `type` (len 0 removes it), keep the others, and stage the
 * store. Built in RAM; PSRAM gets only a finished payload, read back before
 * anything is armed. A refusal leaves any earlier stage as it was. */
static bool stage_record(uint8_t type, const void *data, uint32_t len, uint32_t version)
{
  uint8_t *base = (uint8_t *)N6_ASSET_PSRAM_BASE;
  n6_asset_hdr_t *h = (n6_asset_hdr_t *)base;
  uint8_t *w = s_work;
  uint32_t cur = 0u, cur_ver = 0u, crc = 0u;
  const uint32_t magic = REC_MAGIC;
  bool ok = false;

  if (type == 0u || (len > 0u && data == NULL) || len > N6_ASSET_MAX_BYTES) return false;
  if (version > N6_ASSET_VERSION_MAX) return false;
  if (image_stage_in_the_way()) return false;

  hold();

  /* Start from the pending stage, else the newest flash copy (tried twice: a
   * bad mapped read would silently drop the other records). With nothing
   * valid, still number past any version a broken copy claims, or this write
   * would not be read. */
  if (!load_staged(&cur, &cur_ver) && !load(&cur, &cur_ver) && !load(&cur, &cur_ver)) {
    cur = 0u;
    cur_ver = header_version();
  }

  /* Reads take the highest version, so a write at or below the stored one
   * (0 included) goes one past it. None goes past N6_ASSET_VERSION_MAX; a
   * store already there takes no more. */
  if (version <= cur_ver) {
    if (cur_ver >= N6_ASSET_VERSION_MAX) goto out;
    version = cur_ver + 1u;
  }

  /* Work as a container. A plain asset is the user record: replacing the user
   * record drops it, anything else wraps it. */
  if (cur > 0u && !is_container(w, cur)) {
    if (type == N6_ASSET_REC_USER) {
      cur = 0u;
    } else if (cur + 2u * REC_HDR_BYTES > N6_ASSET_MAX_BYTES) {
      goto out;                           /* no room beside it */
    } else {
      memmove(w + 2u * REC_HDR_BYTES, w, cur);
      w[4] = N6_ASSET_REC_USER;
      w[5] = 0u;
      w[6] = (uint8_t)cur;
      w[7] = (uint8_t)(cur >> 8);
      cur += 2u * REC_HDR_BYTES;
    }
  }
  if (cur == 0u) cur = 4u;
  memcpy(w, &magic, 4u);

  /* Drop the old record of this type. */
  for (uint32_t pos = 4u; pos < cur; ) {
    const uint32_t sz = REC_HDR_BYTES + rec_len(w + pos);
    if (w[pos] == type) {
      memmove(w + pos, w + pos + sz, cur - pos - sz);
      cur -= sz;
    } else {
      pos += sz;
    }
  }

  /* Add the new one. The user asset alone is stored plain (all 4 KB usable),
   * unless its bytes would pass for a container. */
  if (len > 0u && type == N6_ASSET_REC_USER && cur == 4u &&
      !is_container((const uint8_t *)data, len)) {
    memcpy(w, data, len);
    cur = len;
  } else {
    if (len > 0u) {
      if (cur + REC_HDR_BYTES + len > N6_ASSET_MAX_BYTES) goto out;
      w[cur]      = type;
      w[cur + 1u] = 0u;
      w[cur + 2u] = (uint8_t)len;
      w[cur + 3u] = (uint8_t)(len >> 8);
      memcpy(w + cur + REC_HDR_BYTES, data, len);
      cur += REC_HDR_BYTES + len;
    }
    /* A removal can leave the user asset alone: plain again, on the same terms. */
    if (cur > 2u * REC_HDR_BYTES && w[4] == N6_ASSET_REC_USER &&
        cur == 2u * REC_HDR_BYTES + rec_len(w + 4) &&
        !is_container(w + 2u * REC_HDR_BYTES, cur - 2u * REC_HDR_BYTES)) {
      cur -= 2u * REC_HDR_BYTES;
      memmove(w, w + 2u * REC_HDR_BYTES, cur);
    }
  }

  /* Nothing armed while the stage is replaced. */
  bkp_write(N6_ASSET_TRIGGER_BKPR, 0u);
  h->magic = 0u;
  __DSB();

  memcpy(base + N6_ASSET_HDR_BYTES, w, cur);
  __DSB();

  /* Read back before committing: refuse a corrupted stage rather than program it. */
  crc = n6_ota_crc32(0u, w, cur);
  if (n6_ota_crc32(0u, base + N6_ASSET_HDR_BYTES, cur) != crc) goto out;

  h->version = version;
  h->len     = cur;
  h->crc32   = crc;
  __DSB();
  h->magic   = N6_ASSET_PSRAM_MAGIC;      /* magic last: complete or absent */
  h->hdr_crc = n6_ota_crc32(0u, (const uint8_t *)h, 16u);
  __DSB();

  bkp_write(N6_ASSET_RESULT_BKPR, 0u);
  bkp_write(N6_ASSET_TRIGGER_BKPR, N6_ASSET_TRIGGER_MAGIC);
  ok = true;
out:
  release();
  return ok;
}

bool n6_asset_stage(const void *data, uint32_t len, uint32_t version)
{
  if (data == NULL || len == 0u) return false;
  return stage_record(N6_ASSET_REC_USER, data, len, version);
}

bool n6_asset_record_stage(uint8_t type, const void *data, uint32_t len)
{
  return stage_record(type, data, len, 0u);
}

void n6_asset_apply(void)
{
  /* Not to be taken for a second press of the reset button: see
   * N6_BOOT_PRESS_KEY. */
  if (bkp_read(N6_BOOT_PRESS_BKPR) == N6_BOOT_PRESS_KEY) bkp_write(N6_BOOT_PRESS_BKPR, 0u);
  __DSB();
  NVIC_SystemReset();
}

bool n6_asset_pending(void)
{
  return bkp_read(N6_ASSET_TRIGGER_BKPR) == N6_ASSET_TRIGGER_MAGIC;
}

uint32_t n6_asset_last_result(void)
{
  return bkp_read(N6_ASSET_RESULT_BKPR);
}

const char *n6_asset_result_str(uint32_t code)
{
  switch (code) {
    case 0u:                         return "no attempt recorded";
    case N6_ASSET_RESULT_OK:         return "written";
    case N6_ASSET_RESULT_ERR_HDR:    return "staged header rejected";
    case N6_ASSET_RESULT_ERR_CRC:    return "staged payload failed its CRC";
    case N6_ASSET_RESULT_ERR_SIZE:   return "staged size rejected";
    case N6_ASSET_RESULT_ERR_VERIFY: return "what landed in flash was wrong";
    default:                         return "unknown code";
  }
}
