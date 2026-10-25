/**
 * @file    vnd_meta.h
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

#ifndef VND_META_H
#define VND_META_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* float32_t is CMSIS-DSP's plain float. arm_math.h is not included: its
 * arm_mve.h C++ overloads conflict with extern "C" linkage. */
#ifndef _ARM_MATH_TYPES_H_
typedef float float32_t;
#endif

/* TLV Type Definitions */
enum {
  META_INFERENCE_MS      = 1,
  META_BOXES             = 2,
  META_KEYPOINTS         = 3,
  META_BINDINGS          = 4,
  META_ISEG_CLASSSTR     = 10,
  META_ISEG_MASK         = 11,
  META_TOF_FRAME_LEGACY  = 0x14,  /* Fixed 4x4 grid, 16 x u32 mm. Read by old hosts
                                     only; send META_TOF_FRAME. */
  META_TOF_FRAME         = 0x15,  /* ToF distance grid: u8 rows, cols, roi_size,
                                     roi_step, then a u32 distance in mm per zone,
                                     row-major, 0 = no reading. */
  META_TOF_STATUS        = 0x16,  /* The grid's range status: the same 4-byte
                                     header, then a u8 per zone, 0 = valid.
                                     Same shape as the distance grid in the
                                     slot; Neuro Studio drops another shape. */
  META_DEBUG_TEXT        = 0x17,  /* Terminal text, printable ASCII, no
                                     terminator: the latest line only.
                                     META_DEBUG_LOG (0x71) carries every recent
                                     line, numbered. */
  META_MAG_FRAME         = 0x1A,  /* MMC5603NJ magnetometer: 4 x float32 LE (x, y,
                                     z field, temperature degC). Meta-only, no JPEG. */
  META_IMU_FRAME         = 0x1B,  /* LSM6DSL IMU: 7 x float32 LE (accel xyz g, gyro
                                     xyz dps, temperature degC). Meta-only vendor frame. */
  META_THERMAL_FRAME     = 0x30,  /* FLIR Lepton. u16 width, height; u8 format
                                     (0=RAW16 LE), flags (bit0 radiometric/TLinear ->
                                     centikelvin); u16 reserved; then width*height
                                     uint16 LE pixels, row-major (160x120 = 38408 B).
                                     Standalone camera-less (flags=0, no JPEG), else
                                     rides JPEG frames; older hosts skip by length. */
  META_BOARD_IPV4        = 0x20,  /* IP announce, ~1 Hz while the active transport
                                     is Wi-Fi, so Neuro Studio knows where to
                                     TCP-connect. Value = 4 B IPv4 (dotted-quad,
                                     MSB-first) + 4 B TCP port (uint32 LE) = 8 B. */
  META_BOARD_SOFTAP      = 0x24,  /* u8 ssid_len, ssid, u8 pass_len, passphrase
                                     of the board's access point. USB only, at
                                     about 1 Hz while it is up. */
  META_VD66_ISP_ENABLED  = 0x40,  /* Manual-ISP-enabled announce, ~1 Hz, when an
                                     application sets cfg.isp_enabled = VD66_isp_tuning_manual.
                                     Value = 1 B mode (1 = VD66 manual); gates Neuro
                                     Studio's ISP tuning panel. */
  META_AUDIO_PCM         = 0x50,  /* PDM mic (MDF) live audio chunk. NOT latest-wins
                                     - a continuous, lossless, ordered stream, sent
                                     as its own meta-only frames. Self-describing:
                                       u32 sample_rate (Hz, LE)
                                       u8  channels (1 = mono)
                                       u8  bits (16)
                                       u16 sample_count (LE)
                                       int16 PCM[sample_count] (signed LE)
                                     Host plays whatever sample_rate is reported;
                                     older hosts skip it by length. */

  /* Classical-CV "virtual sensor" frames (0x60-0x63): CPU-vision results (no
   * NN), one latest-wins slot per algorithm; unknown types are skipped by
   * length. On-video boxes still go to META_BOXES. Boxes: full-frame
   * normalised uint16 LE (vnd_meta_f01_to_u16), as META_BOXES. "crop": u8
   * crop_w, crop_h, then crop_w*crop_h uint16 LE RGB565 pixels (0,0 = no
   * thumbnail). */
  META_AUDIO_CLASS       = 0x51,  /* Audio classifier (AED) result, latest-wins:
                                       u8  count            (classes reported)
                                       u8  top              (argmax, 0xFF = none
                                                             above the threshold)
                                       u16 level            (peak |sample| 0..32767)
                                       u16 threshold_x1000  (what the device used)
                                       per class: u16 score_x1000
                                     Scores are x1000. Class names are not
                                     sent: see META_AUDIO_LABELS. */
  META_AUDIO_LABELS      = 0x53,  /* Class names for META_AUDIO_CLASS, ~1 Hz.
                                     Value: u8 count, then per class: u8 len,
                                     chars. The host caches them and gates the
                                     audio panel on them. */
  META_AUDIO_MEL         = 0x52,  /* Log-mel spectrogram fed to the NN, uint8
                                     exactly as the model's input buffer holds it:
                                       u8  n_mel
                                       u8  n_frames
                                       u8  layout   (0 = frame-major, 1 = mel-major)
                                       u8  reserved
                                       n_mel * n_frames bytes
                                     Diagnostic only: what the network is shown. */
  META_VISION_ID         = 0x60,  /* Which CV application is running, so Neuro Studio
                                     selects a panel. Value = 1 B bitmask: bit0=QR,
                                     bit1=barcode, bit2=blob, bit3=AprilTag.
                                     Re-announced at ~0.5 Hz (latest-wins). */
  META_QR_FRAME          = 0x61,  /* u8 count, then per result:
                                       u16 cx, cy, w, h   (normalized box)
                                       u8  version        (QR symbol version)
                                       u8  ecc_level      (0..3 = M,L,H,Q)
                                       u16 text_len, text_len bytes of payload
                                       crop
                                     count = 0 clears the host panel. */
  META_BARCODE_FRAME     = 0x62,  /* u8 count, then per result:
                                       u16 cx, cy, w, h   (normalized box)
                                       u8  symbology      (barcode1d_sym_t)
                                       u16 text_len, text_len bytes of digits
                                       crop
                                     count = 0 clears the host panel. */
  META_BLOB_FRAME        = 0x63,  /* u8 count (0 or 1), then if count = 1:
                                       u16 cx, cy, w, h   (normalized box)
                                       u16 area           (normalized fraction of
                                                           the analysis window)
                                       u16 hue            (degrees, 0..359)
                                       u16 pixels         (clamped cluster size)
                                       crop
                                     count = 0 means "no blob tracked". */
  META_APRILTAG          = 0x66,  /* u8 count, then per tag:
                                       u16 cx, cy, w, h   (normalized box)
                                       u16 id
                                       u8  family         (0 = tag36h11)
                                       u8  hamming        (bit errors corrected)
                                       u16 margin         (normalized 0..65535)
                                       i16 angle_cdeg     (rotation, centi-degrees)
                                       u8  pose_valid     (1 = the six below
                                                           are meaningful)
                                       i16 tx,ty,tz       (MILLIMETRES)
                                       i16 rx,ry,rz       (Euler XYZ, centi-deg)
                                       crop
                                     pose_valid = 0 means no stored calibration
                                     (tag still detected/reported); count = 0
                                     clears the host panel. */

  META_CAM_CAPS          = 0x64,  /* Manual capture controls the active sensor
                                     implements, so the host shows a camera panel
                                     only when there is something to drive.
                                     Value = 1 B bitmask: bit0 exposure, bit1
                                     gain, bit2 white balance, bit3 autofocus.
                                     ~1 Hz, only when non-zero (the OV5640
                                     declares all four, VD66GY none). Driven by
                                     the host's 'CAM0' packet. */

  /* Evidence (0x70..0x7F): why the device decided something; not on the
   * decision path. Best-effort: may be dropped under backpressure so a result
   * is never delayed (the class is decidable from the tag alone). Produced
   * only while a host is subscribed (N6_CMD_SUB0), except META_DEBUG_LOG,
   * which always is. See n6_host_contract.h. */

  META_EVENT             = 0x67,  /* An event fired by N6Events: something the
                                     device decided happened. 17 B:
                                       u32 id           monotonic per boot
                                       u32 t_ms         boot-relative ms
                                       u8  sources      bit0 motion, bit1 NN,
                                                        bit2 audio, bit3 thermal,
                                                        bit4 ToF, bit5 GPIO,
                                                        bit6 timer, bit7 host
                                       u8  state        0 idle, 1 post-roll,
                                                        2 writing, 3 refractory
                                       u16 score        application-defined, 0..65535
                                       u16 pre_frames   clip frames before t
                                       u16 post_frames  clip frames after t
                                       u8  sinks_ok     bit i = sink i completed
                                     Published to its slot, and announced camera-less
                                     at 1 Hz for a few seconds so a board with no
                                     video still delivers it. Latest-wins with a
                                     monotonic id, so a host sees a gap, not a miss.
                                     Must equal N6_HOST_TAG_EVENT (asserted). */

  META_EVIDENCE_CROP     = 0x70,  /* Crop thumbnails for the results in this
                                   * same frame's result TLV.
                                   *   u8  count
                                   *   per crop:
                                   *     u8  result_index  (into the result TLV)
                                   *     u8  crop_w
                                   *     u8  crop_h
                                   *     crop_w*crop_h x u16 LE RGB565
                                   * A result with no crop simply has no entry;
                                   * indices need not be contiguous. */

  META_DEBUG_LOG         = 0x71,  /* The Terminal lines of the last ~2 s,
                                   * numbered and repeated in every frame: the
                                   * host prints each number once, and a gap
                                   * means lost lines.
                                   *   u32 number of the first line (from 1 each
                                   *       boot; with count 0, the next line's)
                                   *   u8  count
                                   *   per line: u8 len, len bytes of ASCII text
                                   *             (one whole line, no terminator,
                                   *             at most 160 bytes)
                                   *   u8  boot: differs from boot to boot, to
                                   *       tell a new boot's lines from the last
                                   *       one's (absent from older boards). */

};

/* Metadata slots, one per source of TLV bytes; vnd_meta_get() composes the
 * ready slots in slot-id order into one frame. VND_META_SLOT_PRIMARY is an
 * alias for MODEL_0. */
typedef enum
{
  VND_META_SLOT_MODEL_0 = 0,
  VND_META_SLOT_MODEL_1 = 1,
  VND_META_SLOT_MODEL_2 = 2,
  VND_META_SLOT_MODEL_3 = 3,
  VND_META_SLOT_TOF     = 4,   /* META_TOF_FRAME and META_TOF_STATUS, up to 8 x 8 zones */
  /* On-board sensors: each writes its own slot (one writer), ridden on every
   * camera frame or, camera-less, via the meta pump (vnd_meta_pub_seq). */
  VND_META_SLOT_IMU     = 5,   /* LSM6DSL - META_IMU_FRAME     (0x1B)        */
  VND_META_SLOT_MAG     = 6,   /* MMC5603NJ - META_MAG_FRAME     (0x1A)        */
  VND_META_SLOT_THERMAL = 7,   /* FLIR Lepton - META_THERMAL_FRAME (0x30)       */
  VND_META_SLOT_DEBUG   = 8,   /* Terminal - META_DEBUG_TEXT (0x17), META_DEBUG_LOG (0x71) */
  /* Classical-CV virtual sensors: separate slots, own storage budget. IDs are
   * appended - never renumber the slots above; vnd_meta.c's storage table is
   * positional. */
  VND_META_SLOT_QR        = 9,  /* META_QR_FRAME      (0x61) */
  VND_META_SLOT_BARCODE   = 10, /* META_BARCODE_FRAME (0x62) */
  VND_META_SLOT_BLOB      = 11, /* META_BLOB_FRAME    (0x63) */
  VND_META_SLOT_VISION_ID = 12, /* META_VISION_ID     (0x60) */
  VND_META_SLOT_APRILTAG  = 13, /* META_APRILTAG      (0x66) */
  /* Audio: separate from the CV slots so a classifier and a camera model can
   * both be live, without sharing budget with crop thumbnails. */
  VND_META_SLOT_AUDIO_CLASS  = 14, /* META_AUDIO_CLASS  (0x51) */
  VND_META_SLOT_AUDIO_MEL    = 15, /* META_AUDIO_MEL    (0x52) */
  VND_META_SLOT_AUDIO_LABELS = 16, /* META_AUDIO_LABELS (0x53) */
  /* Evidence: one slot for all of it (only one CV application runs at a time), so
   * the four result slots above shrink from crop-sized to result-sized. */
  VND_META_SLOT_EVIDENCE     = 17, /* META_EVIDENCE_CROP (0x70) */

  /* N6Events. Mirrored as N6_EVENTS_META_SLOT in that library, which does not
   * include this header; vnd_meta.c asserts the mirror. */
  VND_META_SLOT_EVENT        = 18, /* META_EVENT (0x67) */
  VND_META_SLOT_COUNT,

  /* Alias for slot 0. */
  VND_META_SLOT_PRIMARY = VND_META_SLOT_MODEL_0
} vnd_meta_slot_t;

typedef struct
{
  uint8_t *buf;
  uint32_t capacity;
  uint32_t len;
  vnd_meta_slot_t slot;
} vnd_meta_builder_t;

/* Shared API. */

void vnd_meta_init(uint8_t *meta_buf, uint32_t max_bytes);
void vnd_meta_reset(void);
bool vnd_meta_get(const uint8_t **out_meta, uint32_t *out_len);

/* Double-buffered snapshot for the per-frame video producer: idx (the frame
 * parity, g_enc_seq & 1) keeps two in-flight frames' metadata apart, as the
 * JPEG ping-pong does. vnd_meta_get is for synchronous readers. */
bool vnd_meta_get_pingpong(uint32_t idx, const uint8_t **out_meta, uint32_t *out_len);
bool vnd_meta_is_ready(void);
void vnd_meta_invalidate(void);

/* Bumped by every vnd_meta_builder_finish; the camera-less meta pump sends a
 * frame only when it changes. Weak-stubbed to 0 when PostProcess is not
 * linked. */
uint32_t vnd_meta_pub_seq(void);

/* Per-thermal monotonic counter - bumped only when the thermal slot finishes,
 * so the thermal standalone pump fires exclusively on new Lepton
 * frames. Weak-stubbed to 0 when PostProcess is not linked. */
uint32_t vnd_meta_thermal_pub_seq(void);

/* Return just one slot's current front-buffer pointer and length without
 * composing all slots (used by the thermal standalone pump). Returns false if
 * the slot has no data. The pointer is valid until the next
 * vnd_meta_builder_finish on the same slot. Weak-stubbed. */
bool vnd_meta_get_slot(vnd_meta_slot_t slot, const uint8_t **ptr, uint32_t *len);

/* Publish one ASCII line (at most 160 characters; longer is cut) to Neuro
 * Studio's Terminal. Each line rides every frame for about 2 s, so none is
 * lost between frames; the window holds about 1 KB, and a longer burst loses
 * its oldest lines (Neuro Studio reports how many). Safe from any task. */
bool vnd_meta_publish_debug_text(const char *text);

/* Publish one TLV as a slot's whole content (for several TLVs use
 * builder_begin/put_tlv/finish). Latest-wins: a slot keeps its last value
 * until overwritten or invalidated, so publish an empty value (e.g. count =
 * 0) to clear a stale result. Returns false, leaving the slot as it was, if
 * the slot is out of range or the value exceeds its capacity. `value` may be
 * NULL only when `len` is 0. */
bool vnd_meta_publish_tlv(vnd_meta_slot_t slot, uint8_t type,
                          const void *value, uint16_t len);

/* Boot-log replay: every Terminal line is also recorded in a 2 KB boot log
 * (earliest lines win) and replayed to a host that attaches after start-up
 * (the slots are cleared on USB mount). The USB task paces it. */
/* Call on USB mount (tud_mount_cb) to rearm the replay from line 0. */
void vnd_meta_debug_mark_replay(void);
/* Call once per USB task pass; link_idle = 1 when no vendor TX is in flight.
 * Keeps the DEBUG slot in step with the Terminal window (the USB task is its
 * only writer) and replays one recorded line per >= 100 ms idle step; the
 * first completed catch-up freezes the log. */
void vnd_meta_debug_replay_step(int link_idle);

/* Clear a single slot (ready flag and length to 0), e.g. to stop the host
 * seeing stale TLVs from a less-frequently-run model while others keep
 * refreshing. Out-of-range slots are silently ignored. */
void vnd_meta_invalidate_slot(vnd_meta_slot_t slot);

/* Shared TLV helpers for the model-specific builders. */

void vnd_meta_put_u16_le(uint8_t *p, uint16_t v);
void vnd_meta_put_u32_le(uint8_t *p, uint32_t v);
void vnd_meta_builder_begin(vnd_meta_builder_t *builder, vnd_meta_slot_t slot);
void vnd_meta_builder_finish(const vnd_meta_builder_t *builder);
bool vnd_meta_builder_append_bytes(vnd_meta_builder_t *builder, const uint8_t *src, uint32_t len);
bool vnd_meta_builder_put_tlv(vnd_meta_builder_t *builder, uint8_t type, const void *value, uint16_t len);
uint16_t vnd_meta_f01_to_u16(float32_t v);
uint8_t  vnd_meta_conf_to_q8(float32_t c);
/* v clamped to 0..1, with NaN taken as 0. */
float32_t vnd_meta_clamp01(float32_t v);
/* One box as META_BOXES carries it, from its frame-normalised edges: 8 bytes
 * at p, the centre x and y, the width and the height, u16 LE each. Each edge
 * is clamped to the frame first, so a box part way out of the frame is sent
 * as the part inside it. */
void     vnd_meta_put_box(uint8_t *p, float32_t l, float32_t t, float32_t r, float32_t b);

#ifdef __cplusplus
}
#endif

#endif /* VND_META_H */
