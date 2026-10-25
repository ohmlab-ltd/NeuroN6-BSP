/**
 ******************************************************************************
 * @file    n6_host_contract.h
 * @brief   The contract between a Neuro N6 board and an attached host.
 ******************************************************************************
 *
 * Must stay in sync with the host tool. Uplink: TLV on the vendor bulk-IN
 * metadata stream (vnd_meta.h). Downlink: magic-prefixed binary on the vendor
 * bulk-OUT endpoint, or the same bytes behind a frame header on a network link
 * (below). Delivery per class:
 *
 *   CAPABILITY  D->H  Latest-wins, ~1 Hz, idempotent, safe to miss.
 *   RESULT      D->H  Per-frame, on the decision path - never dropped.
 *   EVIDENCE    D->H  Large, droppable, host-gated. Never on the decision path.
 *   PARAMETER   H->D  Live knob, bounded by N6_HOST_CMD_MAX, not persisted.
 *   ASSET       H->D  Large, persisted across a power cycle.
 *
 * The evidence tag range is normative: a transport drops evidence by tag alone.
 * Tags below it are classified by value in n6_host_tag_class(); never renumber
 * them, released hosts depend on them.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 * This software is licensed under the terms in
 * Middlewares/NeuroN6/PostProcess/LICENSE-MIT.txt.
 */

#ifndef N6_HOST_CONTRACT_H
#define N6_HOST_CONTRACT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Compile-time assertion that works from C99, C11 and C++. */
#if defined(__cplusplus)
#  define N6_STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#elif defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
#  define N6_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#else
#  define N6_STATIC_ASSERT(cond, msg) /* unavailable on this dialect */
#endif

/* Classes */

typedef enum
{
  N6_HOST_CLASS_CAPABILITY = 0,
  N6_HOST_CLASS_RESULT     = 1,
  N6_HOST_CLASS_EVIDENCE   = 2,
  N6_HOST_CLASS_PARAMETER  = 3,   /* downlink only */
  N6_HOST_CLASS_ASSET      = 4    /* downlink only */
} n6_host_class_t;

/* Uplink: the evidence range */

/* An EVENT: the device decided something happened and keeps evidence of it.
 * RESULT class, never dropped. Same value as META_EVENT in vnd_meta.h, which
 * has the layout; vnd_meta.c asserts they agree. */
#define N6_HOST_TAG_EVENT           0x67u

/* Every new EVIDENCE tag goes in this one-nibble range, so a transport can
 * drop it by mask without parsing. */
#define N6_HOST_EVIDENCE_TAG_FIRST  0x70u
#define N6_HOST_EVIDENCE_TAG_LAST   0x7Fu

#define N6_HOST_TAG_IS_EVIDENCE(tag) \
  ((uint8_t)(tag) >= N6_HOST_EVIDENCE_TAG_FIRST && \
   (uint8_t)(tag) <= N6_HOST_EVIDENCE_TAG_LAST)

/* Classify an uplink TLV tag. An unknown tag is a RESULT, so it is delivered,
 * never dropped. */
static inline n6_host_class_t n6_host_tag_class(uint8_t tag)
{
  if (N6_HOST_TAG_IS_EVIDENCE(tag)) return N6_HOST_CLASS_EVIDENCE;

  switch (tag) {
    /* What this build supports, republished at ~1 Hz. */
    case 0x20u:   /* META_BOARD_IPV4        */
    case 0x21u:   /* META_BOARD_NAME        */
    case 0x22u:   /* META_BOARD_LINK        */
    case 0x23u:   /* META_BOARD_CHALLENGE   */
    case 0x40u:   /* META_VD66_ISP_ENABLED  */
    case 0x53u:   /* META_AUDIO_LABELS      */
    case 0x60u:   /* META_VISION_ID         */
    case 0x64u:   /* META_CAM_CAPS          */
    case 0x65u:   /* META_TUNE_DESC         */
      return N6_HOST_CLASS_CAPABILITY;

    /* Diagnostics for a human, outside the range (cannot be renumbered). */
    case 0x17u:   /* META_DEBUG_TEXT        */
    case 0x52u:   /* META_AUDIO_MEL         */
      return N6_HOST_CLASS_EVIDENCE;

    default:
      return N6_HOST_CLASS_RESULT;
  }
}

/* zlib-compatible CRC32 (reflected, poly 0xEDB88320; 64-byte nibble table).
 * Must match n6_ota_crc32 (n6_ota_layout.h) and the FSBL's copy: every layer
 * on the way to flash checks the host's CRC of an asset. */
static inline uint32_t n6_host_crc32(uint32_t crc, const uint8_t *p, uint32_t len)
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

/* Downlink: commands */

/* Largest host->device command (the router's buffer in tud_vendor_rx_cb);
 * anything bigger is an ASSET. */
#define N6_HOST_CMD_MAX  64u

/* Little-endian four-character magic: the literals read as on the wire. */
#define N6_HOST_MAGIC(a, b, c, d)                                    \
  ((uint32_t)(uint8_t)(a)        | ((uint32_t)(uint8_t)(b) << 8) |   \
   ((uint32_t)(uint8_t)(c) << 16) | ((uint32_t)(uint8_t)(d) << 24))

#define N6_CMD_ISP0  N6_HOST_MAGIC('I', 'S', 'P', '0')  /* live ISP tuning     */
#define N6_CMD_CAM0  N6_HOST_MAGIC('C', 'A', 'M', '0')  /* capture controls    */
#define N6_CMD_TUN0  N6_HOST_MAGIC('T', 'U', 'N', '0')  /* one tunable         */
#define N6_CMD_SUB0  N6_HOST_MAGIC('S', 'U', 'B', '0')  /* evidence subscribe  */
#define N6_CMD_AST0  N6_HOST_MAGIC('A', 'S', 'T', '0')  /* asset: begin        */
#define N6_CMD_AST1  N6_HOST_MAGIC('A', 'S', 'T', '1')  /* asset: chunk        */
#define N6_CMD_AST2  N6_HOST_MAGIC('A', 'S', 'T', '2')  /* asset: commit+reset */

/* 'ISP0' is variable length, parsed by the VD66GY ISP path; only its minimum
 * is fixed here. */
#define N6_CMD_ISP0_MIN_BYTES  13u

typedef struct __attribute__((packed))
{
  uint32_t magic;        /* N6_CMD_CAM0 */
  uint8_t  flags;        /* bit0 auto exposure, bit1 auto gain, bit2 auto AWB */
  uint32_t exposure_us;
  uint16_t gain_x256;    /* 256 = 1.0x */
  uint8_t  apply_mask;   /* which fields of this packet are meaningful */
} n6_cmd_cam0_t;

/* Evidence subscription: evidence is sent only while subscribed. The host
 * renews about once a second (TTL below); mask 0 unsubscribes at once. */
typedef struct __attribute__((packed))
{
  uint32_t magic;        /* N6_CMD_SUB0 */
  uint32_t mask;         /* N6_EVIDENCE_* bits; 0 unsubscribes */
} n6_cmd_sub0_t;

/* Evidence classes a host can ask for. */
#define N6_EVIDENCE_CROP  (1u << 0)   /* per-result crop thumbnails */

/* Subscription lifetime without renewal. */
#define N6_HOST_SUB_TTL_MS  3000u

/* ASSET transfer in chunks: AST0 begins (discarding any partial transfer),
 * AST1 carries a chunk at an explicit offset, AST2 checks the CRC and commits.
 * Committing resets the board: the FSBL writes the flash, not the application. */
typedef struct __attribute__((packed))
{
  uint32_t magic;        /* N6_CMD_AST0 */
  uint32_t total_len;
  uint32_t crc32;        /* over the whole payload */
  uint32_t version;      /* 0: one past the stored. At most 0xFFFFFFFE; a commit
                            that cannot be numbered above the store is refused */
} n6_cmd_ast0_t;

#define N6_ASSET_CHUNK_MAX  48u

typedef struct __attribute__((packed))
{
  uint32_t magic;        /* N6_CMD_AST1 */
  uint16_t offset;
  uint8_t  len;          /* <= N6_ASSET_CHUNK_MAX */
  /* uint8_t data[len] follows */
} n6_cmd_ast1_t;

typedef struct __attribute__((packed))
{
  uint32_t magic;        /* N6_CMD_AST2 */
} n6_cmd_ast2_t;

/* 'ASR0' begins a record transfer (same AST1 chunks, AST2 commit): it replaces
 * one typed record beside the asset, keeping the rest; length 0 removes it.
 * A library owns its record type. USB only. */
#define N6_CMD_ASR0  N6_HOST_MAGIC('A', 'S', 'R', '0')  /* record: begin       */

typedef struct __attribute__((packed))
{
  uint32_t magic;        /* N6_CMD_ASR0 */
  uint32_t total_len;    /* 0 removes the record */
  uint32_t crc32;        /* over the whole record */
  uint8_t  type;         /* N6_ASSET_REC_* */
  uint8_t  reserved[3];
} n6_cmd_asr0_t;

/* Record types. 1 is the asset AST0 writes (calibration, a table ...). */
#define N6_ASSET_REC_USER   1u
#define N6_ASSET_REC_WIFI   2u   /* MayaW476: the WiFi settings a host stored */
#define N6_ASSET_REC_AP_PASSWORD 3u   /* MayaW476: the board's own SoftAP password */

/* The Wi-Fi record: TLVs { tag:u8 len:u8 value[len] }, read by
 * MayaW476_BeginStored(). Unknown tags are skipped. */
#define N6_WIFI_TLV_MODE      1u   /* u8: 1 join a network, 2 host one        */
#define N6_WIFI_TLV_SSID      2u   /* 1..32 bytes                              */
#define N6_WIFI_TLV_KEY       3u   /* passphrase 8..63 bytes, or 64 hex digits
                                        under WPA2; absent: joining is refused,
                                        hosting uses the board's own password */
#define N6_WIFI_TLV_SECURITY  4u   /* u8 WifiSecurity, WPA2 or WPA3; absent: WPA2 */
#define N6_WIFI_TLV_CHANNEL   5u   /* u8, hosting only; absent: 6              */
#define N6_WIFI_TLV_HOSTNAME  6u   /* 1..32 bytes: DHCP and mDNS name          */
#define N6_WIFI_TLV_LINK_KEY  7u   /* 8..64 bytes: a host must prove it has it */

typedef struct __attribute__((packed))
{
  uint32_t magic;        /* N6_CMD_TUN0 */
  uint8_t  id;           /* index into the tune registry */
  uint8_t  type;         /* 0 int, 1 float (scaled x1000), 2 bool */
  int32_t  value;
} n6_cmd_tun0_t;

/* The network link carries the same stream as USB, to one host at a time:
 * while a client is connected, every uplink frame goes to it instead of USB.
 * Uplink frame: the header below, the metadata TLVs, then the payload.
 *
 * Downlink over TCP: each command in the same header, meta_len 0, payload_len
 * the command's length (at most N6_HOST_CMD_MAX; 0 does nothing), then the
 * command bytes as on USB; frame_id and flags are ignored. Any other magic, a
 * non-zero meta_len or a longer payload closes the connection.
 *
 * Network hosts get the PARAMETER commands only (ISP0, CAM0, TUN0, SUB0); the
 * ASSET commands and REBOOT_DFU need the USB cable.
 *
 * Discovery: a datagram of at least a header's length starting with
 * N6_FRAME_MAGIC, to UDP port N6_NET_PORT (or the board's TCP port if the
 * application changed it), usually broadcast. The board answers the sender
 * with one frame: META_BOARD_IPV4 (0x20, IPv4 and TCP port), META_BOARD_NAME
 * (0x21, also its mDNS <name>.local) and, if it wants a link key,
 * META_BOARD_LINK (0x22, one byte, bit 0 set). A reply may carry
 * META_BOARD_IPV4 alone. */
#define N6_META_BOARD_NAME       0x21u   /* UTF-8, up to 32 bytes, no terminator */
#define N6_META_BOARD_LINK       0x22u   /* u8 flags */
#define N6_BOARD_LINK_KEYED      0x01u   /* a link key must be proved first */

/* USB only, never on a network link: the access point the board hosts, about
 * once a second while it is up. { ssid_len:u8 ssid pass_len:u8 passphrase } */
#define N6_META_BOARD_SOFTAP     0x24u

/* Optional link key: a host must prove it before it gets the stream or may
 * send a command. Its first frame's only TLV is META_BOARD_CHALLENGE (0x23,
 * N6_LINK_CHALLENGE_BYTES fresh bytes); the board then waits up to
 * N6_LINK_ANSWER_MS for one command, framed like any other:
 *
 *   'KEY0'  HMAC-SHA256(key, "N6LINK1" || challenge)        4 + 32 bytes
 *
 * A wrong or late answer closes the connection. The key is the given text's
 * bytes, 8 to 64. It keeps other hosts off the board; it does not encrypt the
 * stream. */
#define N6_META_BOARD_CHALLENGE  0x23u
#define N6_LINK_CHALLENGE_BYTES  16u
#define N6_LINK_ANSWER_MS        10000u
#define N6_LINK_CONTEXT          "N6LINK1"   /* HMAC'd ahead of the challenge */
#define N6_CMD_KEY0  N6_HOST_MAGIC('K', 'E', 'Y', '0')

/* The first four bytes of every vendor frame, "DNV0" on the wire. */
#define N6_FRAME_MAGIC  N6_HOST_MAGIC('D', 'N', 'V', '0')

typedef struct __attribute__((packed))
{
  uint32_t magic;        /* N6_FRAME_MAGIC */
  uint32_t frame_id;
  uint32_t meta_len;     /* TLV bytes after the header */
  uint32_t payload_len;  /* bytes after the TLVs */
  uint32_t flags;        /* uplink bit0: the payload is a JPEG */
} n6_frame_hdr_t;

/* Default TCP port of the stream, and the UDP port probes go to. */
#define N6_NET_PORT  5000u

/* Pin the wire sizes: released hosts send these bytes, and the router checks
 * incoming lengths against sizeof. */
N6_STATIC_ASSERT(sizeof(n6_cmd_cam0_t) == 12u, "CAM0 wire format changed");
N6_STATIC_ASSERT(sizeof(n6_cmd_tun0_t) == 10u, "TUN0 wire format changed");
N6_STATIC_ASSERT(sizeof(n6_cmd_sub0_t) == 8u,  "SUB0 wire format changed");
N6_STATIC_ASSERT(sizeof(n6_cmd_ast0_t) == 16u, "AST0 wire format changed");
N6_STATIC_ASSERT(sizeof(n6_cmd_ast1_t) == 7u,  "AST1 wire format changed");
N6_STATIC_ASSERT(sizeof(n6_cmd_asr0_t) == 16u, "ASR0 wire format changed");
N6_STATIC_ASSERT(sizeof(n6_cmd_ast1_t) + N6_ASSET_CHUNK_MAX <= N6_HOST_CMD_MAX,
                 "an AST1 chunk does not fit the command buffer");
N6_STATIC_ASSERT(sizeof(n6_frame_hdr_t) == 20u, "frame header wire format changed");
N6_STATIC_ASSERT(N6_FRAME_MAGIC == 0x30564E44u, "frame magic changed");

N6_STATIC_ASSERT(sizeof(n6_cmd_cam0_t) <= N6_HOST_CMD_MAX,
                 "CAM0 does not fit the command buffer");
N6_STATIC_ASSERT(sizeof(n6_cmd_tun0_t) <= N6_HOST_CMD_MAX,
                 "TUN0 does not fit the command buffer");
N6_STATIC_ASSERT(sizeof(n6_cmd_sub0_t) <= N6_HOST_CMD_MAX,
                 "SUB0 does not fit the command buffer");
N6_STATIC_ASSERT(N6_CMD_ISP0_MIN_BYTES <= N6_HOST_CMD_MAX,
                 "ISP0 does not fit the command buffer");

/* The evidence range must not collide with an allocated tag. */
N6_STATIC_ASSERT(N6_HOST_EVIDENCE_TAG_FIRST > N6_HOST_TAG_EVENT,
                 "evidence range overlaps an allocated tag");
N6_STATIC_ASSERT(N6_HOST_EVIDENCE_TAG_LAST >= N6_HOST_EVIDENCE_TAG_FIRST,
                 "evidence range is inverted");

#ifdef __cplusplus
}
#endif

#endif /* N6_HOST_CONTRACT_H */
