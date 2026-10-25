/**
 ******************************************************************************
 * @file    n6_ota_layout.h
 * @brief   OTA flash layout and control block: the contract between the app
 *          (N6OTA library) and the FSBL.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * This repository keeps two copies, byte for byte: the boot loader's, in
 * FSBL/Core/Inc, and the application library's, in Middlewares/NeuroN6/N6OTA.
 * harness/compile.sh checks that they match.
 *
 * All addresses are offsets from the flash base: the FSBL's DFU protocol
 * uses raw offsets (0x00100000), the app sees the same byte memory-mapped
 * at 0x90100000.
 *
 * LAYOUT (32 MB octal NOR)
 * ------------------------
 *   0x00000000  FSBL                    1 MB     (USB DFU only, never OTA'd)
 *   0x00100000  App slot A            1.5 MB
 *   0x00280000  control block copy 0   4 KB
 *   0x00281000  control block copy 1   4 KB
 *   0x00282000  asset store, 2 copies  4 KB each, then reserved to 0x00300000
 *   0x00300000  NN weights pool     25.5 MB      base is immovable (baked
 *                                                into generated model code)
 *   0x01C80000  App slot B            1.5 MB
 *   0x01E00000  MAYA-W476 IW610 fw      2 MB     (matches MAYA_FW_ADDR in
 *                                                neuron6_model_gen.py)
 *
 * With no valid control block the FSBL boots slot A.
 ******************************************************************************
 */

#ifndef N6_OTA_LAYOUT_H
#define N6_OTA_LAYOUT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Flash geometry (offsets from flash base) --------------------------- */

#define N6_OTA_SLOT_A_OFF        0x00100000u
#define N6_OTA_SLOT_B_OFF        0x01C80000u
#define N6_OTA_SLOT_SIZE         0x00180000u   /* 1.5 MB each               */

/* Bytes the FSBL copies from a slot to AXISRAM at 0x34000000; the FSBL runs
 * from 0x34180000, so no more fits. */
#define N6_OTA_MAX_LOADABLE      0x00180000u

#define N6_OTA_CB_OFF(copy)      (0x00280000u + ((copy) ? 0x1000u : 0u))
#define N6_OTA_CB_SECTOR         0x1000u       /* one 4 KB sector per copy  */

/* Memory-mapped view (what the app reads through XSPI1). */
#define N6_OTA_FLASH_MMAP_BASE   0x90000000u

/* ---- PSRAM staging area ----------------------------------------------------- */

/* The app stages an update here, sets N6_OTA_TRIGGER_MAGIC and resets; the FSBL
 * checks it and copies it into the inactive slot. */

/* The top 2 MB of the 32 MB PSRAM, kept clear of everything an allocator
 * can hand out: this repository's linker script ends EXTMEM_PSRAM
 * at 0x71BFFFFF, and a network's PSRAM pool ends at 0x71DFFFFF. */
#define N6_OTA_PSRAM_BASE        0x71E00000u
#define N6_OTA_PSRAM_HDR_MAGIC   0x4750364Eu   /* 'N6PG' little-endian      */
#define N6_OTA_PSRAM_IMAGE_OFF   32u           /* image starts here, aligned */

/* Backup register carrying the trigger. BKP1/BKP2 are the DFU magics and
 * BKP24..BKP31 the crash record; nothing else uses 20. */
#define N6_OTA_TRIGGER_BKPR      20u
#define N6_OTA_TRIGGER_MAGIC     0x504F364Eu   /* 'N6OP' little-endian      */

/* Outcome of the last apply, kept until the next one, for the app to read. */
#define N6_OTA_APPLY_BKPR        22u

#define N6_OTA_APPLY_OK             0x4F4B0000u  /* applied and pending      */
#define N6_OTA_APPLY_STARTED        0x53540000u  /* trigger seen; got no further */
#define N6_OTA_APPLY_ERR_PSRAM_MAP  0x45500000u  /* could not map PSRAM      */
#define N6_OTA_APPLY_ERR_HDR_MAGIC  0x45480000u  /* no staged header there   */
#define N6_OTA_APPLY_ERR_HDR_CRC    0x45430000u  /* header did not check out */
#define N6_OTA_APPLY_ERR_SIZE       0x455A0000u  /* size zero or too large   */
#define N6_OTA_APPLY_ERR_IMAGE_CRC  0x45490000u  /* staged image is not intact */
#define N6_OTA_APPLY_ERR_FLASH_CRC  0x45460000u  /* what landed in flash is wrong */

/* Set by the app once its image proves healthy, and on every boot; the FSBL
 * then commits the pending slot. A power cut clears it, costing one try. */
#define N6_OTA_CONFIRM_BKPR      21u
#define N6_OTA_CONFIRM_MAGIC     0x4B4F364Eu   /* 'N6OK' little-endian      */

/* The double-press window. Before the bootloader starts an application it
 * writes the RTC time of that boot to register 1 and this key to register 2;
 * a reset in the same RTC second, the key still there, is taken for a second
 * press of the reset button, and the board stays in DFU. An application that
 * resets the board itself, to apply a stage or to recover, clears the key
 * first, and leaves any other value of the register alone. */
#define N6_BOOT_PRESS_BKPR       2u
#define N6_BOOT_PRESS_KEY        0x00FE4ADEu

/* Written at N6_OTA_PSRAM_BASE. 20 bytes; the image follows at
 * N6_OTA_PSRAM_IMAGE_OFF. */
typedef struct {
  uint32_t magic;                 /* N6_OTA_PSRAM_HDR_MAGIC                 */
  uint32_t size;                  /* image bytes                            */
  uint32_t version;               /* recorded in the control block          */
  uint32_t crc32;                 /* zlib CRC32 over `size` image bytes     */
  uint32_t hdr_crc;               /* zlib CRC32 over the preceding 16 bytes */
} n6_ota_psram_hdr_t;

/* ---- Control block ------------------------------------------------------ */

#define N6_OTA_CB_MAGIC          0x4E364F54u   /* 'N6OT' little-endian      */

/* Per-slot state. EMPTY slots are never booted via the control block;
 * VALID means written and CRC-checked but not yet proven to run;
 * CONFIRMED means an app booted from it called n6_ota_confirm(). */
#define N6_OTA_SLOT_EMPTY        0x00000000u
#define N6_OTA_SLOT_VALID        0x56414C44u   /* 'VALD' */
#define N6_OTA_SLOT_CONFIRMED    0x434F4E46u   /* 'CONF' */

#define N6_OTA_NO_PENDING        0xFFu

/* Boots a pending image gets to call n6_ota_confirm() before the FSBL reverts
 * to the other slot. The application's watchdog resets an image that hangs. */
#define N6_OTA_DEFAULT_TRIES     3u

/* 64 bytes, little-endian on both sides; `crc` is the zlib CRC32 of the first
 * 60 bytes. */
typedef struct {
  uint32_t magic;                /* N6_OTA_CB_MAGIC                          */
  uint32_t seq;                  /* monotonic; higher valid copy wins        */
  uint8_t  active;               /* 0 = slot A, 1 = slot B                   */
  uint8_t  pending;              /* slot index, or N6_OTA_NO_PENDING         */
  uint8_t  tries_left;           /* decremented by the FSBL per pending boot */
  uint8_t  reserved;
  struct {
    uint32_t crc32;              /* over `size` bytes from the slot start    */
    uint32_t size;               /* image bytes (signed image, as flashed)   */
    uint32_t version;            /* app-declared, monotonic per product      */
    uint32_t state;              /* EMPTY / VALID / CONFIRMED                */
  } slot[2];
  uint8_t  pad[16];              /* headroom for future fields; zero-filled  */
  uint32_t crc;                  /* CRC32 of the preceding 60 bytes          */
} n6_ota_cb_t;

/* The struct must be exactly 64 bytes on both compilers; a size drift means
 * the two sides are reading different layouts. */
typedef char n6_ota_cb_size_check[(sizeof(n6_ota_cb_t) == 64) ? 1 : -1];

/* ---- Shared helpers (header-only, used by both sides) ------------------- */

static inline uint32_t n6_ota_slot_off(uint8_t slot)
{
  return slot ? N6_OTA_SLOT_B_OFF : N6_OTA_SLOT_A_OFF;
}

/* The slot (0 or 1) containing this flash offset, or -1 for neither. The
 * FSBL's DFU hook uses it to decide whether an upload was an app image. */
static inline int n6_ota_slot_of_off(uint32_t off)
{
  if (off >= N6_OTA_SLOT_A_OFF && off < N6_OTA_SLOT_A_OFF + N6_OTA_SLOT_SIZE) return 0;
  if (off >= N6_OTA_SLOT_B_OFF && off < N6_OTA_SLOT_B_OFF + N6_OTA_SLOT_SIZE) return 1;
  return -1;
}

/* zlib-compatible CRC32 (reflected, poly 0xEDB88320) with a 64-byte nibble
 * table, shared by the FSBL and the app. */
static inline uint32_t n6_ota_crc32(uint32_t crc, const uint8_t *p, uint32_t len)
{
  static const uint32_t tab[16] = {
    0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu,
    0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
    0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
    0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu,
  };
  crc = ~crc;
  while (len--) {
    crc ^= *p++;
    crc = (crc >> 4) ^ tab[crc & 0x0Fu];
    crc = (crc >> 4) ^ tab[crc & 0x0Fu];
  }
  return ~crc;
}

/* CRC of a control block's own contents (the first 60 bytes). */
static inline uint32_t n6_ota_cb_crc(const n6_ota_cb_t *cb)
{
  return n6_ota_crc32(0u, (const uint8_t *)cb, sizeof(*cb) - 4u);
}

static inline int n6_ota_cb_valid(const n6_ota_cb_t *cb)
{
  return cb->magic == N6_OTA_CB_MAGIC && cb->crc == n6_ota_cb_crc(cb);
}


/* ---- Asset store ------------------------------------------------------------- */

/* Small host-authored data kept across power cycles: two copies of one 4 KB
 * sector; the FSBL writes the copy not holding the highest valid version. */

#define N6_ASSET_OFF(copy)       (0x00282000u + ((copy) ? 0x1000u : 0u))
#define N6_ASSET_SECTOR          0x1000u
#define N6_ASSET_COPIES          2u

/* Header sits at the front of the sector, payload immediately after. */
#define N6_ASSET_MAGIC           0x54534136u   /* '6AST' little-endian */
#define N6_ASSET_HDR_BYTES       20u
#define N6_ASSET_MAX_BYTES       (N6_ASSET_SECTOR - N6_ASSET_HDR_BYTES)

/* How many times to re-read before giving up on a copy. */
#define N6_ASSET_READ_TRIES      3u

typedef struct __attribute__((packed)) {
  uint32_t magic;      /* N6_ASSET_MAGIC                       */
  uint32_t version;    /* monotonic; highest valid copy wins   */
  uint32_t len;        /* payload bytes following this header  */
  uint32_t crc32;      /* over the payload                     */
  uint32_t hdr_crc;    /* over the first 16 bytes              */
} n6_asset_hdr_t;

/* PSRAM staging for an asset. A staged update at most
 * N6_OTA_STAGE_CLEAR_OF_ASSET bytes long (0x17FFE0) ends at or below this
 * address; a longer one reaches into the asset's stage, so the two cannot
 * both be staged for one reset. */
#define N6_ASSET_PSRAM_BASE      0x71F80000u
#define N6_ASSET_PSRAM_MAGIC     0x47534136u   /* '6ASG' */
#define N6_OTA_STAGE_CLEAR_OF_ASSET \
  (N6_ASSET_PSRAM_BASE - N6_OTA_PSRAM_BASE - N6_OTA_PSRAM_IMAGE_OFF)

#define N6_ASSET_TRIGGER_BKPR    16u
#define N6_ASSET_TRIGGER_MAGIC   0x57534136u   /* '6ASW' */
#define N6_ASSET_RESULT_BKPR     15u

#define N6_ASSET_RESULT_OK          1u
#define N6_ASSET_RESULT_ERR_HDR     2u
#define N6_ASSET_RESULT_ERR_CRC     3u
#define N6_ASSET_RESULT_ERR_SIZE    4u
#define N6_ASSET_RESULT_ERR_VERIFY  5u

#ifdef __cplusplus
}
#endif

#endif /* N6_OTA_LAYOUT_H */
