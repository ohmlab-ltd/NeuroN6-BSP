/*
 * The box packing every detector's metadata goes through, vnd_meta_put_box
 * in Middlewares/NeuroN6/PostProcess/vnd_meta.c, built on the PC. A box is
 * sent as the part of it inside the frame: its edges are clamped, not its
 * centre and its size, so a box part way out keeps the edges it has in the
 * frame, one wholly out has no size, and NaN reads as 0. A slot number past
 * the table, or below it, is refused.
 *
 *   gcc -std=c11 -Ifake -I<PostProcess> -I<ST vision pp Inc> test_meta_box.c <PostProcess>/vnd_meta.c -lm
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#include <math.h>
#include <stdio.h>

#include "vnd_meta.h"
#include "stm32n6xx.h"   /* the stand-in: the cycle counter vnd_meta.c reads */

/* What vnd_meta.c needs from the rest of the board's software. */
uint32_t HAL_GetTick(void) { return 0u; }
DWT_Type g_dwt;
DCB_Type g_dcb;

static int failed, checks;

static void check(int ok, const char *what)
{
  checks++;
  if (!ok)
  {
    failed++;
    printf("FAIL %s\n", what);
  }
}

/* The box as the host draws it: its edges, in frame units. */
static void drawn(const uint8_t *p, float e[4])
{
  float v[4];

  for (int i = 0; i < 4; i++)
  {
    v[i] = (float)(p[2 * i] | (p[2 * i + 1] << 8)) / 65535.f;
  }
  e[0] = v[0] - 0.5f * v[2];
  e[1] = v[1] - 0.5f * v[3];
  e[2] = v[0] + 0.5f * v[2];
  e[3] = v[1] + 0.5f * v[3];
}

static void check_box(const char *what, float l, float t, float r, float b,
                      float wl, float wt, float wr, float wb)
{
  const float want[4] = { wl, wt, wr, wb };
  uint8_t p[8];
  float   e[4];
  int     ok = 1;

  vnd_meta_put_box(p, l, t, r, b);
  drawn(p, e);
  for (int i = 0; i < 4; i++)
  {
    /* One step of the u16 either way. */
    if (fabsf(e[i] - want[i]) > (1.5f / 65535.f)) ok = 0;
  }
  if (!ok)
  {
    printf("     %s: drawn %.4f %.4f %.4f %.4f, want %.4f %.4f %.4f %.4f\n",
           what, e[0], e[1], e[2], e[3], wl, wt, wr, wb);
  }
  check(ok, what);
}

int main(void)
{
  const uint8_t *ptr;
  uint32_t       len;
  uint8_t        p[8];

  check_box("inside the frame", 0.2f, 0.3f, 0.6f, 0.7f, 0.2f, 0.3f, 0.6f, 0.7f);
  check_box("the whole frame", 0.f, 0.f, 1.f, 1.f, 0.f, 0.f, 1.f, 1.f);
  /* Past the right edge: the edges are clamped, so it is drawn from 0.9. */
  check_box("out on the right", 0.9f, 0.4f, 1.3f, 0.6f, 0.9f, 0.4f, 1.f, 0.6f);
  check_box("out on the left", -0.5f, 0.2f, 0.1f, 0.4f, 0.f, 0.2f, 0.1f, 0.4f);
  check_box("out at the top and bottom", 0.3f, -0.2f, 0.5f, 1.4f, 0.3f, 0.f, 0.5f, 1.f);
  check_box("wholly out", 1.1f, 0.2f, 1.3f, 0.4f, 1.f, 0.2f, 1.f, 0.4f);
  check_box("inverted", 0.6f, 0.6f, 0.4f, 0.4f, 0.6f, 0.6f, 0.6f, 0.6f);
  check_box("NaN", NAN, NAN, NAN, NAN, 0.f, 0.f, 0.f, 0.f);

  vnd_meta_put_box(p, 0.f, 0.f, 1.f, 1.f);
  check((p[0] | (p[1] << 8)) == 32768 && (p[4] | (p[5] << 8)) == 65535,
        "the whole frame is centre 32768, width 65535");

  check(vnd_meta_f01_to_u16(NAN) == 0u, "NaN packs as 0");
  check(vnd_meta_f01_to_u16(-1.f) == 0u, "below 0 packs as 0");
  check(vnd_meta_f01_to_u16(2.f) == 65535u, "above 1 packs as 65535");
  check(vnd_meta_clamp01(NAN) == 0.f, "NaN clamps to 0");
  check(vnd_meta_clamp01(0.25f) == 0.25f, "0.25 is left alone");

  vnd_meta_init(NULL, 0u);
  check(!vnd_meta_get_slot(VND_META_SLOT_COUNT, &ptr, &len), "a slot past the table is refused");
  check(!vnd_meta_get_slot((vnd_meta_slot_t)-1, &ptr, &len), "a slot below the table is refused");

  /* The ToF slot takes an 8 x 8 distance grid and its status, the records
   * the host reads; the fixed 4 x 4 record is for old hosts only. */
  {
    static uint8_t grid[4u + (64u * 4u)], status[4u + 64u];
    vnd_meta_builder_t b;
    int ok;

    grid[0] = status[0] = 8u;
    grid[1] = status[1] = 8u;
    vnd_meta_builder_begin(&b, VND_META_SLOT_TOF);
    ok  = vnd_meta_builder_put_tlv(&b, META_TOF_FRAME, grid, (uint16_t)sizeof(grid));
    ok &= vnd_meta_builder_put_tlv(&b, META_TOF_STATUS, status, (uint16_t)sizeof(status));
    vnd_meta_builder_finish(&b);
    check(ok && vnd_meta_get_slot(VND_META_SLOT_TOF, &ptr, &len) &&
          (len == (3u + sizeof(grid) + 3u + sizeof(status))) && (ptr[0] == META_TOF_FRAME),
          "an 8 x 8 ToF grid and its status fit the ToF slot");
  }

  /* The Terminal log's record ends with the boot byte: the cycle counter,
   * scrambled, when the first record is built, and the same for the rest of
   * the boot however the counter moves on. vnd_meta_init starts the counter. */
  {
    const uint8_t want = (uint8_t)((12345u * 0x9E3779B1u) >> 24);
    const uint8_t *rec[2] = { NULL, NULL };
    uint16_t       rec_len[2] = { 0u, 0u };

    check(((g_dcb.DEMCR & DCB_DEMCR_TRCENA_Msk) != 0u) &&
          ((g_dwt.CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u),
          "vnd_meta_init starts the cycle counter");
    for (int i = 0; i < 2; i++)
    {
      g_dwt.CYCCNT = (i == 0) ? 12345u : 999u;
      (void)vnd_meta_publish_debug_text((i == 0) ? "one" : "two");
      vnd_meta_debug_replay_step(0);
      if (vnd_meta_get_slot(VND_META_SLOT_DEBUG, &ptr, &len))
      {
        for (uint32_t o = 0u; (o + 3u) <= len; )
        {
          const uint16_t l = (uint16_t)(ptr[o + 1] | (ptr[o + 2] << 8));

          if (ptr[o] == META_DEBUG_LOG) { rec[i] = &ptr[o + 3u]; rec_len[i] = l; }
          o += 3u + l;
        }
      }
    }
    /* u32 first, u8 count, then u8 len and the text per line, then the byte. */
    check((rec[0] != NULL) && (rec_len[0] == (4u + 1u + 4u + 1u)) && (rec[0][4] == 1u) &&
          (rec[0][rec_len[0] - 1u] == want),
          "the Terminal log record ends with the boot byte");
    check((rec[1] != NULL) && (rec_len[1] == (4u + 1u + 4u + 4u + 1u)) && (rec[1][4] == 2u) &&
          (rec[1][rec_len[1] - 1u] == want),
          "the boot byte stays the same for the boot");
  }

  printf("test_meta_box: %d checks, %d failed\n", checks, failed);
  return failed ? 1 : 0;
}
