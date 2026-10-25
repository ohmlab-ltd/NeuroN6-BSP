/*
 * The YOLOv8 segmentation metadata (vnd_meta_build_iseg_on_slot,
 * Middlewares/NeuroN6/PostProcess/vnd_meta_yolov8_Iseg.c), built on the PC
 * into slots of every size. Each detection has a 64 x 64-bit mask and a
 * class name. At any size the build must hang together (as many masks as
 * names, for the same objects, and one box list counting them); with room,
 * every detection is in it. A box past the frame's edge, or NaN, reads no
 * byte outside its mask (each mask ends at an unreadable page), and a box
 * part way out is sent as the part its mask covers, to within a mask cell.
 *
 *   gcc -std=c11 -Ifake -I<PostProcess> -I<ST vision pp Inc> test_iseg_meta.c <PostProcess>/vnd_meta_yolov8_Iseg.c
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#define _DEFAULT_SOURCE
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include "vnd_meta_yolov8_Iseg.h"

/* What the builder needs from the rest of the board's software. */
uint32_t ts[2];
uint16_t g_nn_crop_x = 0, g_nn_crop_y = 0, g_nn_crop_w = 640, g_nn_crop_h = 480;
uint16_t g_camera_full_w = 640, g_camera_full_h = 480;
const char *classes_table[NB_CLASSES];
iseg_pp_out_t pp_output;
iseg_yolov8_pp_static_param_t pp_params;

/* The builder, as vnd_meta.c has it, over one buffer of a chosen size. */
static uint8_t  slot_buf[16384];
static uint32_t slot_cap;
static uint32_t built_len;

void vnd_meta_put_u16_le(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

void vnd_meta_builder_begin(vnd_meta_builder_t *b, vnd_meta_slot_t slot)
{
  b->buf = slot_buf;
  b->capacity = slot_cap;
  b->len = 0u;
  b->slot = slot;
}

void vnd_meta_builder_finish(const vnd_meta_builder_t *b)
{
  built_len = b->len;
}

bool vnd_meta_builder_put_tlv(vnd_meta_builder_t *b, uint8_t type, const void *value, uint16_t len)
{
  if (!b || !b->buf) return false;
  if (b->len + 3u + (uint32_t)len > b->capacity) return false;
  b->buf[b->len] = type;
  vnd_meta_put_u16_le(&b->buf[b->len + 1], len);
  b->len += 3u;
  if (len && value)
  {
    memcpy(&b->buf[b->len], value, len);
    b->len += len;
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
  return (uint8_t)((v < 0) ? 0 : ((v > 255) ? 255 : v));
}

/* What a build holds, or why it does not hang together. */
static const char *judge(int *pKept)
{
  int masks = 0, names = 0, lists = 0, counted = -1;
  uint8_t mask_ids[32] = {0}, name_ids[32] = {0};

  for (uint32_t o = 0; (o + 3u) <= built_len; )
  {
    const uint8_t  type = slot_buf[o];
    const uint16_t len  = (uint16_t)(slot_buf[o + 1] | (slot_buf[o + 2] << 8));
    const uint8_t *v    = &slot_buf[o + 3];

    if ((o + 3u + len) > built_len) return "a record runs past the end";
    if (type == META_ISEG_MASK)     { if (masks < 32) { mask_ids[masks] = v[0]; } masks++; }
    if (type == META_ISEG_CLASSSTR) { if (names < 32) { name_ids[names] = v[0]; } names++; }
    if (type == META_BOXES)
    {
      lists++;
      counted = v[0];
      if (len != (1u + (10u * v[0]))) return "the box list's length disagrees with its count";
    }
    o += 3u + len;
  }
  *pKept = masks;
  if (masks != names) return "a mask was sent without its name";
  if (memcmp(mask_ids, name_ids, sizeof(mask_ids)) != 0) return "masks and names name different objects";
  if (lists != 1) return "no box list";
  if (counted != masks) return "the box list counts other objects than the masks";
  return NULL;
}

int main(void)
{
  static uint8_t masks[AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT][AI_YOLOV8_SEG_PP_MASK_SIZE * AI_YOLOV8_SEG_PP_MASK_SIZE];
  iseg_pp_outBuffer_t det[AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT];
  iseg_pp_out_t out;
  int failed = 0, checks = 0, kept = 0;
  const char *why;

  for (int i = 0; i < NB_CLASSES; i++) classes_table[i] = "a class with a long name";
  memset(masks, 0xFF, sizeof(masks));
  for (int i = 0; i < AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT; i++)
  {
    det[i].x_center = 0.5f;
    det[i].y_center = 0.5f;
    det[i].width = 1.0f;
    det[i].height = 1.0f;
    det[i].conf = 0.9f;
    det[i].class_index = i;
    det[i].pMask = masks[i];
  }
  out.pOutBuff = det;
  out.nb_detect = AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT;

  /* Nine bytes is the smallest frame there is: the inference time and an
   * empty box list. */
  for (uint32_t cap = 9u; cap <= 8000u; cap++)
  {
    slot_cap = cap;
    vnd_meta_build_iseg_on_slot(&out, 7u, NULL, VND_META_SLOT_MODEL_0);
    why = judge(&kept);
    checks++;
    if (why != NULL)
    {
      if (failed < 5) printf("FAIL  a slot of %u bytes: %s\n", (unsigned)cap, why);
      failed++;
    }
  }

  slot_cap = sizeof(slot_buf);
  vnd_meta_build_iseg_on_slot(&out, 7u, NULL, VND_META_SLOT_MODEL_0);
  why = judge(&kept);
  checks++;
  if ((why != NULL) || (kept != AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT))
  {
    printf("FAIL  with room for everything: %d of %d detections, %s\n", kept,
           AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT, (why != NULL) ? why : "the rest dropped");
    failed++;
  }

  /* A box past the frame's edge: its region must stay inside the mask. The
   * mask fills the page before a guard page, so a read past it is fatal. */
  {
    static const float centres[] = { 0.95f, 1.10f, 1.50f, 2.04f, -0.60f, NAN };
    const size_t page = (size_t)sysconf(_SC_PAGESIZE);
    const size_t mask_bytes = AI_YOLOV8_SEG_PP_MASK_SIZE * AI_YOLOV8_SEG_PP_MASK_SIZE;
    uint8_t *pg = mmap(NULL, 2 * page, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);

    if ((pg == MAP_FAILED) || (mask_bytes > page) || (mprotect(pg + page, page, PROT_NONE) != 0))
    {
      printf("FAIL  could not set up a guard page\n");
      failed++;
    }
    else
    {
      for (size_t i = 0; i < sizeof(centres) / sizeof(centres[0]); i++)
      {
        iseg_pp_outBuffer_t one = det[0];
        iseg_pp_out_t lone = { &one, 1 };
        pid_t pid;
        int st = 0;

        one.x_center = centres[i];
        one.y_center = centres[i];
        one.width = 0.1f;
        one.height = 0.1f;
        one.pMask = pg + page - mask_bytes;
        memset(one.pMask, 1, mask_bytes);
        pid = fork();
        if (pid == 0)
        {
          slot_cap = sizeof(slot_buf);
          vnd_meta_build_iseg_on_slot(&lone, 7u, NULL, VND_META_SLOT_MODEL_0);
          _exit(judge(&kept) != NULL);
        }
        waitpid(pid, &st, 0);
        checks++;
        if (!WIFEXITED(st) || (WEXITSTATUS(st) != 0))
        {
          printf("FAIL  a box centred at %.2f %s\n", (double)centres[i],
                 WIFSIGNALED(st) ? "read past its mask" : "built a frame that does not hang together");
          failed++;
        }
      }
    }
  }

  /* A box part way out of the frame, 0.4 wide: the box sent is as wide as
   * the region its mask covers, to within a mask cell. */
  {
    static const float centres[] = { 0.10f, 0.50f, 0.95f, 1.05f };

    for (size_t i = 0; i < sizeof(centres) / sizeof(centres[0]); i++)
    {
      iseg_pp_outBuffer_t one = det[0];
      iseg_pp_out_t lone = { &one, 1 };
      float box_w = -1.f, mask_w = -2.f;

      one.x_center = centres[i];
      one.y_center = 0.5f;
      one.width = 0.4f;
      one.height = 0.2f;
      slot_cap = sizeof(slot_buf);
      vnd_meta_build_iseg_on_slot(&lone, 7u, NULL, VND_META_SLOT_MODEL_0);
      for (uint32_t o = 0; (o + 3u) <= built_len; )
      {
        const uint16_t len = (uint16_t)(slot_buf[o + 1] | (slot_buf[o + 2] << 8));
        const uint8_t *v = &slot_buf[o + 3];

        if ((slot_buf[o] == META_BOXES) && (len >= 11u)) box_w = (float)(v[5] | (v[6] << 8)) / 65535.f;
        if ((slot_buf[o] == META_ISEG_MASK) && (len >= 5u)) mask_w = (float)v[1] / (float)AI_YOLOV8_SEG_PP_MASK_SIZE;
        o += 3u + len;
      }
      checks++;
      if (fabsf(box_w - mask_w) > (1.f / (float)AI_YOLOV8_SEG_PP_MASK_SIZE))
      {
        printf("FAIL  a box centred at %.2f is %.3f wide over a mask %.3f wide\n",
               (double)centres[i], (double)box_w, (double)mask_w);
        failed++;
      }
    }
  }

  printf("%d checks, %d failed (the segmentation metadata)\n", checks, failed);
  return (failed != 0) ? 1 : 0;
}
