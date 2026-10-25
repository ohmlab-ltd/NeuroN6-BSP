/**
 * @file    vnd_meta.c
 * @brief   Common vendor metadata TLV infrastructure for USB streaming
 *
 * @attention
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 * This software is licensed under the terms in
 * Middlewares/NeuroN6/PostProcess/LICENSE-MIT.txt.
 */

#include "vnd_meta.h"
#include <string.h>
#include "stm32n6xx.h"          /* __get_PRIMASK / __disable_irq for the Terminal log */
#include "n6_host_contract.h"   /* N6_HOST_TAG_EVENT, N6_STATIC_ASSERT */

/* N6Events defines the event tag and slot index itself; these keep the two
 * copies equal. */
N6_STATIC_ASSERT(META_EVENT == N6_HOST_TAG_EVENT, "META_EVENT must equal N6_HOST_TAG_EVENT");
N6_STATIC_ASSERT(VND_META_SLOT_EVENT == 18, "VND_META_SLOT_EVENT is mirrored as N6_EVENTS_META_SLOT (18) in N6Events");

/* Metadata storage. A 256 KiB model slot fits a worst-case YOLOv8 ISEG result
 * (10 boxes x 64x64 packed masks + class strings). To grow, add a slot
 * (VND_META_SLOT_COUNT and a table entry) or raise META_MODEL_SLOT_BYTES. */

#define META_MODEL_SLOT_BYTES   (256u * 1024u)
/* The ToF slot: a distance grid (META_TOF_FRAME) and its range status
 * (META_TOF_STATUS), each with its 4-byte grid header, for up to 8 x 8 zones. */
#define META_TOF_MAX_ZONES      64u
#define META_TOF_MAX_BYTES      ((3u + 4u + (META_TOF_MAX_ZONES * 4u)) + \
                                 (3u + 4u + META_TOF_MAX_ZONES))
/* On-board sensor slots. IMU/MAG are tiny (internal SRAM); the thermal frame is
 * ~38 KB so its slot lives in PSRAM. Each = TLV-header(3) + value bytes. */
#define META_IMU_MAX_BYTES      (3u + (7u * sizeof(float32_t)))   /* 7xf32 = 31    */
#define META_MAG_MAX_BYTES      (3u + (4u * sizeof(float32_t)))   /* 4xf32 = 19    */
#define META_THERMAL_MAX_BYTES  (3u + 8u + (160u * 120u * 2u))    /* hdr + 160x120 u16 = 38411 */
#define VND_DEBUG_LINE_MAX      160u    /* longest Terminal line; longer text is cut */
#define VND_DEBUG_LOG_STORE     1280u   /* Terminal window: u8 len + u32 tick + text per line */
#define META_DEBUG_MAX_BYTES    ((3u + VND_DEBUG_LINE_MAX) + (3u + 6u + VND_DEBUG_LOG_STORE))

/* CV result slots (QR, barcode, blob, AprilTag; crops go to the evidence
 * slot), one size for all: 8 results x (8 B box + 2 B crop dims + up to
 * ~512 B text) is just over 4 KiB. vnd_meta_publish_tlv() refuses rather than
 * truncates, so an overflow is a missing panel, not corrupt data. */
#define META_VISION_SLOT_BYTES  (8u * 1024u)

/* Evidence: one slot for the one CV application running: 8 crops x 64x64
 * RGB565 = 64 KiB, touched only while a host is subscribed. */
#define META_EVIDENCE_SLOT_BYTES (68u * 1024u)

/* One 17-byte event TLV plus its 3-byte header, with headroom. */
#define META_EVENT_MAX_BYTES     64u
#define META_VISION_ID_MAX_BYTES (3u + 8u)

/* Audio slots. The mel diagnostic (one 64x96 = 6144 B model input plus a
 * header, with room for bigger spectrograms) goes to PSRAM. */
#define META_AUDIO_CLASS_MAX_BYTES  (3u + 6u + (64u * 2u))     /* <= 64 classes */
#define META_AUDIO_LABELS_MAX_BYTES (3u + 1u + (64u * 33u))
#define META_AUDIO_MEL_MAX_BYTES    (3u + 4u + (128u * 128u))  /* 16 KB payload */

/* TX snapshot must fit every slot concatenated end-to-end. */
#define META_TX_MAX_BYTES       ((4u * META_MODEL_SLOT_BYTES) + META_TOF_MAX_BYTES + \
                                 META_IMU_MAX_BYTES + META_MAG_MAX_BYTES + \
                                 META_THERMAL_MAX_BYTES + META_DEBUG_MAX_BYTES + \
                                 (4u * META_VISION_SLOT_BYTES) + META_EVIDENCE_SLOT_BYTES + \
                                 META_VISION_ID_MAX_BYTES + META_AUDIO_CLASS_MAX_BYTES + \
                                 META_AUDIO_LABELS_MAX_BYTES + META_AUDIO_MEL_MAX_BYTES + \
                                 META_EVENT_MAX_BYTES)

typedef struct
{
  uint8_t *buf[2];        /* per-slot ping-pong: producer reads buf[pub], inference writes buf[!pub] */
  uint32_t capacity;
  uint32_t len[2];
  volatile uint32_t pub;  /* published (front) index - RELEASE-stored by finish, ACQUIRE-loaded by compose */
} vnd_meta_slot_state_t;

/* Double-buffered slots: inference builds the back buffer while the producer
 * reads the front. */
__attribute__((aligned(32), section(".ext_psram")))
static uint8_t g_model0_meta_storage[2][META_MODEL_SLOT_BYTES];

__attribute__((aligned(32), section(".ext_psram")))
static uint8_t g_model1_meta_storage[2][META_MODEL_SLOT_BYTES];

__attribute__((aligned(32), section(".ext_psram")))
static uint8_t g_model2_meta_storage[2][META_MODEL_SLOT_BYTES];

__attribute__((aligned(32), section(".ext_psram")))
static uint8_t g_model3_meta_storage[2][META_MODEL_SLOT_BYTES];

static uint8_t g_tof_meta_storage[2][META_TOF_MAX_BYTES];

/* On-board sensor slots (see vnd_meta.h). IMU/MAG tiny -> internal SRAM; the
 * ~38 KB thermal frame -> PSRAM, like the model / TX buffers. */
static uint8_t g_imu_meta_storage[2][META_IMU_MAX_BYTES];
static uint8_t g_mag_meta_storage[2][META_MAG_MAX_BYTES];

__attribute__((aligned(32), section(".ext_psram")))
static uint8_t g_debug_meta_storage[2][META_DEBUG_MAX_BYTES];

__attribute__((aligned(32), section(".ext_psram")))
static uint8_t g_thermal_meta_storage[2][META_THERMAL_MAX_BYTES];

/* CV result slots; the 1-byte kind announce stays in internal SRAM. */
__attribute__((aligned(32), section(".ext_psram")))
static uint8_t g_qr_meta_storage[2][META_VISION_SLOT_BYTES];

__attribute__((aligned(32), section(".ext_psram")))
static uint8_t g_barcode_meta_storage[2][META_VISION_SLOT_BYTES];

__attribute__((aligned(32), section(".ext_psram")))
static uint8_t g_blob_meta_storage[2][META_VISION_SLOT_BYTES];

__attribute__((aligned(32), section(".ext_psram")))
static uint8_t g_apriltag_meta_storage[2][META_VISION_SLOT_BYTES];

static uint8_t g_visionid_meta_storage[2][META_VISION_ID_MAX_BYTES];

/* Evidence storage (all crops), PSRAM. */
__attribute__((aligned(32), section(".ext_psram")))
static uint8_t g_evidence_meta_storage[2][META_EVIDENCE_SLOT_BYTES];

static uint8_t g_event_meta_storage[2][META_EVENT_MAX_BYTES];

/* Audio slots - see the budgets above. */
static uint8_t g_audioclass_meta_storage[2][META_AUDIO_CLASS_MAX_BYTES];
static uint8_t g_audiolabels_meta_storage[2][META_AUDIO_LABELS_MAX_BYTES];

__attribute__((aligned(32), section(".ext_psram")))
static uint8_t g_audiomel_meta_storage[2][META_AUDIO_MEL_MAX_BYTES];

__attribute__((aligned(32), section(".ext_psram")))
static uint8_t g_tx_meta_storage[META_TX_MAX_BYTES];

static uint32_t g_tx_meta_len = 0;

/* Per-frame snapshots, paired with the JPEG ping-pong, so a consumer 1-2
 * frames behind (Wi-Fi) never reads frame N+1's metadata with frame N's
 * image. vnd_meta_get uses the single buffer above. */
__attribute__((aligned(32), section(".ext_psram")))
static uint8_t g_tx_meta_storage_pp[2][META_TX_MAX_BYTES];
static uint32_t g_tx_meta_len_pp[2] = { 0u, 0u };

/* Bumped by every vnd_meta_builder_finish; read by the camera-less meta
 * pump (via vnd_meta_pub_seq) so it emits a frame only when a slot changed. */
static volatile uint32_t g_meta_pub_seq = 0u;
/* Bumped only when the thermal slot finishes. The thermal standalone pump
 * tracks this to send flags=0 meta-only frames even during camera streaming, so
 * Neuro Studio receives META_THERMAL_FRAME in the standalone format it renders. */
static volatile uint32_t g_thermal_pub_seq = 0u;

static vnd_meta_slot_state_t g_meta_slots[VND_META_SLOT_COUNT] = {
  [VND_META_SLOT_MODEL_0] = { { g_model0_meta_storage[0], g_model0_meta_storage[1] }, META_MODEL_SLOT_BYTES, { 0u, 0u }, 0u },
  [VND_META_SLOT_MODEL_1] = { { g_model1_meta_storage[0], g_model1_meta_storage[1] }, META_MODEL_SLOT_BYTES, { 0u, 0u }, 0u },
  [VND_META_SLOT_MODEL_2] = { { g_model2_meta_storage[0], g_model2_meta_storage[1] }, META_MODEL_SLOT_BYTES, { 0u, 0u }, 0u },
  [VND_META_SLOT_MODEL_3] = { { g_model3_meta_storage[0], g_model3_meta_storage[1] }, META_MODEL_SLOT_BYTES, { 0u, 0u }, 0u },
  [VND_META_SLOT_TOF]     = { { g_tof_meta_storage[0],    g_tof_meta_storage[1] },    META_TOF_MAX_BYTES,    { 0u, 0u }, 0u },
  [VND_META_SLOT_IMU]     = { { g_imu_meta_storage[0],     g_imu_meta_storage[1] },     META_IMU_MAX_BYTES,     { 0u, 0u }, 0u },
  [VND_META_SLOT_MAG]     = { { g_mag_meta_storage[0],     g_mag_meta_storage[1] },     META_MAG_MAX_BYTES,     { 0u, 0u }, 0u },
  [VND_META_SLOT_THERMAL] = { { g_thermal_meta_storage[0], g_thermal_meta_storage[1] }, META_THERMAL_MAX_BYTES, { 0u, 0u }, 0u },
  [VND_META_SLOT_DEBUG]   = { { g_debug_meta_storage[0],   g_debug_meta_storage[1] },   META_DEBUG_MAX_BYTES,   { 0u, 0u }, 0u },
  [VND_META_SLOT_QR]        = { { g_qr_meta_storage[0],       g_qr_meta_storage[1] },       META_VISION_SLOT_BYTES,   { 0u, 0u }, 0u },
  [VND_META_SLOT_BARCODE]   = { { g_barcode_meta_storage[0],  g_barcode_meta_storage[1] },  META_VISION_SLOT_BYTES,   { 0u, 0u }, 0u },
  [VND_META_SLOT_BLOB]      = { { g_blob_meta_storage[0],     g_blob_meta_storage[1] },     META_VISION_SLOT_BYTES,   { 0u, 0u }, 0u },
  [VND_META_SLOT_VISION_ID] = { { g_visionid_meta_storage[0], g_visionid_meta_storage[1] }, META_VISION_ID_MAX_BYTES, { 0u, 0u }, 0u },
  [VND_META_SLOT_APRILTAG]  = { { g_apriltag_meta_storage[0],  g_apriltag_meta_storage[1] },  META_VISION_SLOT_BYTES,   { 0u, 0u }, 0u },
  [VND_META_SLOT_EVIDENCE]  = { { g_evidence_meta_storage[0],  g_evidence_meta_storage[1] },  META_EVIDENCE_SLOT_BYTES, { 0u, 0u }, 0u },
  [VND_META_SLOT_EVENT]     = { { g_event_meta_storage[0],     g_event_meta_storage[1] },     META_EVENT_MAX_BYTES,     { 0u, 0u }, 0u },
  [VND_META_SLOT_AUDIO_CLASS]  = { { g_audioclass_meta_storage[0],  g_audioclass_meta_storage[1] },  META_AUDIO_CLASS_MAX_BYTES,  { 0u, 0u }, 0u },
  [VND_META_SLOT_AUDIO_MEL]    = { { g_audiomel_meta_storage[0],    g_audiomel_meta_storage[1] },    META_AUDIO_MEL_MAX_BYTES,    { 0u, 0u }, 0u },
  [VND_META_SLOT_AUDIO_LABELS] = { { g_audiolabels_meta_storage[0], g_audiolabels_meta_storage[1] }, META_AUDIO_LABELS_MAX_BYTES, { 0u, 0u }, 0u },
};

static bool vnd_meta_slot_valid(vnd_meta_slot_t slot)
{
  /* Unsigned, so a negative value is out of range too. */
  return (uint32_t)slot < (uint32_t)VND_META_SLOT_COUNT;
}

static void vnd_meta_slot_clear(vnd_meta_slot_state_t *slot)
{
  if (!slot) return;
  /* Mark both buffers empty. Called only from invalidate (init/reset), never concurrently with a send. */
  slot->len[0] = 0u;
  slot->len[1] = 0u;
}

static bool vnd_meta_snapshot_append(const vnd_meta_slot_state_t *slot)
{
  if (!slot) return true;
  /* ACQUIRE pairs with finish()'s RELEASE: buf[p] is complete, and the last
   * detection stays published until the next one is finished. */
  const uint32_t p = __atomic_load_n(&slot->pub, __ATOMIC_ACQUIRE) & 1u;
  const uint32_t len = slot->len[p];
  if ((len == 0u) || !slot->buf[p]) return true;
  if (g_tx_meta_len + len > sizeof(g_tx_meta_storage)) return false;

  memcpy(&g_tx_meta_storage[g_tx_meta_len], slot->buf[p], len);
  g_tx_meta_len += len;
  return true;
}

/* TLV helpers. */

void vnd_meta_put_u16_le(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

void vnd_meta_put_u32_le(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

void vnd_meta_builder_begin(vnd_meta_builder_t *builder, vnd_meta_slot_t slot)
{
  if (!builder) return;

  builder->buf = NULL;
  builder->capacity = 0u;
  builder->len = 0u;
  builder->slot = slot;

  if (!vnd_meta_slot_valid(slot)) return;

  vnd_meta_slot_state_t *state = &g_meta_slots[slot];
  /* Build into the back buffer; the front stays valid until finish() swaps. */
  const uint32_t back = 1u - (state->pub & 1u);

  builder->buf = state->buf[back];
  builder->capacity = state->capacity;
}

void vnd_meta_builder_finish(const vnd_meta_builder_t *builder)
{
  if (!builder || !vnd_meta_slot_valid(builder->slot)) return;

  vnd_meta_slot_state_t *state = &g_meta_slots[builder->slot];
  /* Record the back buffer's length, then RELEASE the index swap, so an
   * ACQUIRE reader sees a matching buffer and length. An empty build clears
   * stale detections. */
  const uint32_t back = 1u - (state->pub & 1u);
  state->len[back] = builder->len;
  __atomic_store_n(&state->pub, back, __ATOMIC_RELEASE);
  /* A plain increment: the pump only needs to see that something changed. */
  g_meta_pub_seq++;
  if ((uint32_t)builder->slot == (uint32_t)VND_META_SLOT_THERMAL) {
    g_thermal_pub_seq++;
  }
}

bool vnd_meta_builder_append_bytes(vnd_meta_builder_t *builder, const uint8_t *src, uint32_t len)
{
  if (!builder) return false;
  if (!src || (len == 0u)) return true;
  if (!builder->buf) return false;
  if (builder->len + len > builder->capacity) return false;

  memcpy(&builder->buf[builder->len], src, len);
  builder->len += len;
  return true;
}

bool vnd_meta_builder_put_tlv(vnd_meta_builder_t *builder, uint8_t type, const void *value, uint16_t len)
{
  if (!builder || !builder->buf) return false;
  if (builder->len + 3u + (uint32_t)len > builder->capacity) return false;

  builder->buf[builder->len + 0] = type;
  vnd_meta_put_u16_le(&builder->buf[builder->len + 1], len);
  builder->len += 3u;

  if (len && value)
  {
    memcpy(&builder->buf[builder->len], value, len);
    builder->len += (uint32_t)len;
  }

  return true;
}

float32_t vnd_meta_clamp01(float32_t v)
{
  if (!(v > 0.f)) return 0.f;
  if (v > 1.f) return 1.f;
  return v;
}

uint16_t vnd_meta_f01_to_u16(float32_t v)
{
  return (uint16_t)(vnd_meta_clamp01(v) * 65535.f + 0.5f);
}

void vnd_meta_put_box(uint8_t *p, float32_t l, float32_t t, float32_t r, float32_t b)
{
  /* Clamp the edges, not the centre and size, so a box partly off the frame
   * is not drawn over a region where nothing was found. */
  l = vnd_meta_clamp01(l);
  t = vnd_meta_clamp01(t);
  r = vnd_meta_clamp01(r);
  b = vnd_meta_clamp01(b);
  if (r < l) r = l;
  if (b < t) b = t;

  vnd_meta_put_u16_le(&p[0], vnd_meta_f01_to_u16(0.5f * (l + r)));
  vnd_meta_put_u16_le(&p[2], vnd_meta_f01_to_u16(0.5f * (t + b)));
  vnd_meta_put_u16_le(&p[4], vnd_meta_f01_to_u16(r - l));
  vnd_meta_put_u16_le(&p[6], vnd_meta_f01_to_u16(b - t));
}

uint8_t vnd_meta_conf_to_q8(float32_t c)
{
  int v = (int)(c * 255.f + 0.5f);
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  return (uint8_t)v;
}

/* Shared API. */

void vnd_meta_init(uint8_t *meta_buf, uint32_t max_bytes)
{
  (void)meta_buf;
  (void)max_bytes;
  /* The cycle counter log_boot_byte reads, started if nothing has. */
  DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
  vnd_meta_invalidate();
}

void vnd_meta_reset(void)
{
  vnd_meta_invalidate();
}

bool vnd_meta_get(const uint8_t **out_meta, uint32_t *out_len)
{
  if (!out_meta || !out_len) return false;

  *out_meta = NULL;
  *out_len = 0u;
  g_tx_meta_len = 0u;

  if (!vnd_meta_is_ready()) return false;

  /* Append each slot's TLVs in id order, except thermal: the ~38 KB Lepton
   * frame goes only in its own meta-only frames (the thermal pump). */
  for (uint32_t i = 0u; i < (uint32_t)VND_META_SLOT_COUNT; i++)
  {
    if (i == (uint32_t)VND_META_SLOT_THERMAL) continue;
    if (!vnd_meta_snapshot_append(&g_meta_slots[i])) return false;
  }

  if (g_tx_meta_len == 0u) return false;

  *out_meta = g_tx_meta_storage;
  *out_len = g_tx_meta_len;
  return true;
}

/* vnd_meta_get for the per-frame producer: composes into
 * g_tx_meta_storage_pp[idx & 1], idx being the frame parity (g_enc_seq & 1),
 * so the snapshot pairs with mjpeg_buf[seq & 1] until the consumer reads it. */
bool vnd_meta_get_pingpong(uint32_t idx, const uint8_t **out_meta, uint32_t *out_len)
{
  if (!out_meta || !out_len) return false;
  idx &= 1u;

  *out_meta = NULL;
  *out_len = 0u;
  g_tx_meta_len_pp[idx] = 0u;

  if (!vnd_meta_is_ready()) return false;

  for (uint32_t i = 0u; i < (uint32_t)VND_META_SLOT_COUNT; i++)
  {
    /* Thermal never rides JPEG frames (see vnd_meta_get) - the standalone
     * pump sends it separately so this per-frame metadata stays small. */
    if (i == (uint32_t)VND_META_SLOT_THERMAL) continue;
    const vnd_meta_slot_state_t *slot = &g_meta_slots[i];
    /* ACQUIRE the published index, read its complete buffer (see vnd_meta_snapshot_append). */
    const uint32_t p = __atomic_load_n(&slot->pub, __ATOMIC_ACQUIRE) & 1u;
    const uint32_t len = slot->len[p];
    if ((len == 0u) || !slot->buf[p]) continue;
    if (g_tx_meta_len_pp[idx] + len > META_TX_MAX_BYTES) return false;
    memcpy(&g_tx_meta_storage_pp[idx][g_tx_meta_len_pp[idx]], slot->buf[p], len);
    g_tx_meta_len_pp[idx] += len;
  }

  if (g_tx_meta_len_pp[idx] == 0u) return false;

  *out_meta = g_tx_meta_storage_pp[idx];
  *out_len = g_tx_meta_len_pp[idx];
  return true;
}

bool vnd_meta_is_ready(void)
{
  for (uint32_t i = 0u; i < (uint32_t)VND_META_SLOT_COUNT; i++)
  {
    const uint32_t p = __atomic_load_n(&g_meta_slots[i].pub, __ATOMIC_ACQUIRE) & 1u;
    if (g_meta_slots[i].len[p] != 0u) return true;
  }
  return false;
}

uint32_t vnd_meta_pub_seq(void)
{
  return g_meta_pub_seq;
}

void vnd_meta_invalidate(void)
{
  for (uint32_t i = 0u; i < (uint32_t)VND_META_SLOT_COUNT; i++)
  {
    vnd_meta_slot_clear(&g_meta_slots[i]);
  }

  g_tx_meta_len = 0u;
  g_tx_meta_len_pp[0] = 0u;
  g_tx_meta_len_pp[1] = 0u;
}

void vnd_meta_invalidate_slot(vnd_meta_slot_t slot)
{
  if (!vnd_meta_slot_valid(slot)) return;
  vnd_meta_slot_clear(&g_meta_slots[slot]);
}

uint32_t vnd_meta_thermal_pub_seq(void)
{
  return g_thermal_pub_seq;
}

bool vnd_meta_get_slot(vnd_meta_slot_t slot, const uint8_t **ptr, uint32_t *len)
{
  if (!ptr || !len) return false;
  *ptr = NULL;
  *len = 0u;
  if (!vnd_meta_slot_valid(slot)) return false;
  const vnd_meta_slot_state_t *s = &g_meta_slots[slot];
  const uint32_t p = __atomic_load_n(&s->pub, __ATOMIC_ACQUIRE) & 1u;
  const uint32_t l = s->len[p];
  if (l == 0u || !s->buf[p]) return false;
  *ptr = s->buf[p];
  *len = l;
  return true;
}

/* Debug text and boot-log replay (see vnd_meta.h). */

#define VND_BOOTLOG_BYTES    2048u   /* ~25 typical lines; earliest lines win */
#define VND_BOOTLOG_STEP_MS  100u    /* one replayed line per idle step, its own vendor frame */

static char              s_bootlog[VND_BOOTLOG_BYTES];   /* NUL-terminated lines, back-to-back */
static volatile uint32_t s_bootlog_w = 0u;        /* write head (bytes recorded)     */
static volatile uint8_t  s_bootlog_frozen = 0u;   /* 1 = recording stopped forever   */
static volatile uint8_t  s_bootlog_applock = 0u;  /* append guard - a contended append drops
                                                      the record (live publish still happens) */
static uint8_t           s_bootlog_replay = 0u;   /* replay armed (usb task only)    */
static uint8_t           s_bootlog_banner = 0u;   /* banner sent for this replay     */
static uint32_t          s_bootlog_r = 0u;        /* replay cursor                   */
static uint32_t          s_bootlog_ms = 0u;       /* last replay-step tick           */

extern uint32_t HAL_GetTick(void);                /* 1 ms tick, no HAL header needed */

/* Numbered Terminal log (META_DEBUG_LOG): each line gets a number and rides
 * every frame for VND_DEBUG_LOG_TTL_MS, so the host prints each once and
 * reports gaps; the latest line also goes out as META_DEBUG_TEXT. Any task or
 * ISR may add a line; only the USB task (vnd_meta_debug_replay_step) writes
 * the slot. The window is shared under a short interrupt-masked section, in
 * on-chip RAM to keep it in the microseconds. */
#define VND_DEBUG_LOG_TTL_MS  2000u   /* a line rides every frame for this long */

static uint8_t  s_log[VND_DEBUG_LOG_STORE];
static uint32_t s_log_used  = 0u;     /* bytes of s_log in use                        */
static uint32_t s_log_count = 0u;     /* lines in s_log                               */
static uint32_t s_log_next  = 1u;     /* number the next line gets; lines start at 1  */
static char     s_log_last[VND_DEBUG_LINE_MAX];
static uint16_t s_log_last_len = 0u;  /* latest line, sent as META_DEBUG_TEXT         */
static uint8_t  s_log_dirty = 0u;     /* the slot no longer matches the window        */

/* USB task only: the record being built and a copy of the latest line. */
static uint8_t  s_log_rec[6u + VND_DEBUG_LOG_STORE];
static char     s_log_rec_last[VND_DEBUG_LINE_MAX];
static uint8_t  s_log_boot;           /* the record's last byte, once set             */
static uint8_t  s_log_boot_set = 0u;

/* A per-boot number: the cycle counter at the first record, scrambled. It
 * differs from boot to boot because start-up waits on clocks the processor
 * does not keep (oscillators, sensors, the host). USB task only. */
static uint8_t log_boot_byte(void)
{
  if (!s_log_boot_set) {
    s_log_boot = (uint8_t)((DWT->CYCCNT * 0x9E3779B1u) >> 24);
    s_log_boot_set = 1u;
  }
  return s_log_boot;
}

static inline uint32_t log_lock(void)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static inline void log_unlock(uint32_t primask)
{
  __set_PRIMASK(primask);
}

static uint32_t log_get_u32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Drop the oldest `lines` lines, `bytes` long in total. Caller holds the lock. */
static void log_drop_front(uint32_t bytes, uint32_t lines)
{
  if (bytes >= s_log_used) {
    s_log_used  = 0u;
    s_log_count = 0u;
    return;
  }
  memmove(s_log, &s_log[bytes], s_log_used - bytes);
  s_log_used  -= bytes;
  s_log_count -= lines;
}

/* Number one line (len <= VND_DEBUG_LINE_MAX) and add it to the window. Any
 * context. When the window is full the oldest lines go first; their numbers
 * simply never reach the host, which reports the gap. */
static void log_append(const char *text, uint16_t len)
{
  const uint32_t need = 5u + (uint32_t)len;
  const uint32_t now  = HAL_GetTick();
  const uint32_t primask = log_lock();

  uint32_t off = 0u, lines = 0u;
  while (off < s_log_used && (s_log_used - off) + need > sizeof(s_log)) {
    off += 5u + (uint32_t)s_log[off];
    lines++;
  }
  if (off != 0u) log_drop_front(off, lines);

  s_log[s_log_used] = (uint8_t)len;
  vnd_meta_put_u32_le(&s_log[s_log_used + 1u], now);
  memcpy(&s_log[s_log_used + 5u], text, len);
  s_log_used += need;
  s_log_count++;
  s_log_next++;

  memcpy(s_log_last, text, len);
  s_log_last_len = len;
  s_log_dirty = 1u;

  log_unlock(primask);
}

/* Bring the DEBUG slot in line with the window, after a new line or once a line
 * has ridden frames for VND_DEBUG_LOG_TTL_MS. USB task only. */
static void log_service(void)
{
  const uint32_t now = HAL_GetTick();
  const uint32_t primask = log_lock();

  uint32_t off = 0u, lines = 0u;
  while (off < s_log_used &&
         (now - log_get_u32(&s_log[off + 1u])) >= VND_DEBUG_LOG_TTL_MS) {
    off += 5u + (uint32_t)s_log[off];
    lines++;
  }
  if (off != 0u) {
    log_drop_front(off, lines);
    s_log_dirty = 1u;
  }
  if (!s_log_dirty) {
    log_unlock(primask);
    return;
  }
  s_log_dirty = 0u;

  /* META_DEBUG_LOG value: u32 number of the first line, u8 line count, per
   * line u8 length and the text, then the boot byte. */
  vnd_meta_put_u32_le(&s_log_rec[0], s_log_next - s_log_count);
  s_log_rec[4] = (uint8_t)s_log_count;
  uint32_t rec_len = 5u;
  for (uint32_t p = 0u; p < s_log_used; ) {
    const uint32_t n = (uint32_t)s_log[p];
    s_log_rec[rec_len++] = (uint8_t)n;
    memcpy(&s_log_rec[rec_len], &s_log[p + 5u], n);
    rec_len += n;
    p += 5u + n;
  }
  s_log_rec[rec_len++] = log_boot_byte();
  const uint16_t last_len = s_log_last_len;
  memcpy(s_log_rec_last, s_log_last, last_len);

  log_unlock(primask);

  /* Nothing to say (a fresh host session before any line) publishes an empty
   * slot, exactly as an invalidated slot looks. */
  vnd_meta_builder_t b;
  vnd_meta_builder_begin(&b, VND_META_SLOT_DEBUG);
  if (last_len != 0u) {
    (void)vnd_meta_builder_put_tlv(&b, (uint8_t)META_DEBUG_TEXT, s_log_rec_last, last_len);
  }
  if (last_len != 0u || s_log_rec[4] != 0u) {
    (void)vnd_meta_builder_put_tlv(&b, (uint8_t)META_DEBUG_LOG, s_log_rec, (uint16_t)rec_len);
  }
  vnd_meta_builder_finish(&b);
}

bool vnd_meta_publish_tlv(vnd_meta_slot_t slot, uint8_t type,
                          const void *value, uint16_t len)
{
  if (!value && (len != 0u)) return false;

  vnd_meta_builder_t b;
  vnd_meta_builder_begin(&b, slot);
  /* An invalid slot or over-capacity value returns without finishing: the
   * last good result stays published. */
  if (!vnd_meta_builder_put_tlv(&b, type, value, len)) return false;
  vnd_meta_builder_finish(&b);
  return true;
}

bool vnd_meta_publish_debug_text(const char *text)
{
  if (!text || !text[0]) return false;

  uint16_t len = 0u;
  while (text[len] != '\0' && len < (uint16_t)VND_DEBUG_LINE_MAX) {
    len++;
  }
  if (len == 0u) return false;

  /* Record into the boot log until it freezes (full, or first catch-up done).
   * The NUL is written before the write head advances (release), so a replay
   * step that reads the head (acquire) always finds terminated lines. */
  if (!s_bootlog_frozen &&
      !__atomic_test_and_set(&s_bootlog_applock, __ATOMIC_ACQUIRE)) {
    const uint32_t w = s_bootlog_w;
    if ((w + (uint32_t)len + 1u) <= VND_BOOTLOG_BYTES) {
      memcpy(&s_bootlog[w], text, len);
      s_bootlog[w + len] = '\0';
      __atomic_store_n(&s_bootlog_w, w + (uint32_t)len + 1u, __ATOMIC_RELEASE);
    } else {
      s_bootlog_frozen = 1u;   /* full - keep the earliest lines (the boot story) */
    }
    __atomic_clear(&s_bootlog_applock, __ATOMIC_RELEASE);
  }

  log_append(text, len);
  return true;
}

void vnd_meta_debug_mark_replay(void)
{
  /* The slots were just cleared for a new host session. Start its Terminal
   * from an empty window, without the previous session's last line (the
   * boot-log replay below supplies the history). Numbering carries on, so a
   * number is never reused within a boot. */
  const uint32_t primask = log_lock();
  s_log_used     = 0u;
  s_log_count    = 0u;
  s_log_last_len = 0u;
  s_log_dirty    = 1u;
  log_unlock(primask);

  s_bootlog_r      = 0u;
  s_bootlog_banner = 0u;
  s_bootlog_ms     = HAL_GetTick();
  s_bootlog_replay = (__atomic_load_n(&s_bootlog_w, __ATOMIC_ACQUIRE) != 0u) ? 1u : 0u;
}

void vnd_meta_debug_replay_step(int link_idle)
{
  /* Every pass: keep the DEBUG slot in step with the Terminal window. */
  log_service();

  if (!s_bootlog_replay || !link_idle) return;
  const uint32_t now = HAL_GetTick();
  if ((now - s_bootlog_ms) < VND_BOOTLOG_STEP_MS) return;
  s_bootlog_ms = now;

  if (!s_bootlog_banner) {
    s_bootlog_banner = 1u;
    log_append("[bootlog] --- pre-connect boot log ---\r\n",
               (uint16_t)strlen("[bootlog] --- pre-connect boot log ---\r\n"));
    return;
  }

  const uint32_t w = __atomic_load_n(&s_bootlog_w, __ATOMIC_ACQUIRE);
  if (s_bootlog_r >= w) {
    /* Caught up: the host has the full pre-attach history. Freeze recording -
     * the host sees lines live from here; a later remount replays this same
     * frozen history. */
    s_bootlog_replay = 0u;
    s_bootlog_frozen = 1u;
    log_append("[bootlog] --- end of boot log, live from here ---\r\n",
               (uint16_t)strlen("[bootlog] --- end of boot log, live from here ---\r\n"));
    return;
  }

  const char *line = &s_bootlog[s_bootlog_r];
  const uint16_t len = (uint16_t)strlen(line);
  if (len != 0u) {
    log_append(line, (len > VND_DEBUG_LINE_MAX) ? (uint16_t)VND_DEBUG_LINE_MAX : len);
  }
  s_bootlog_r += (uint32_t)len + 1u;
}
