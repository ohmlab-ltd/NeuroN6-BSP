/* Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 *
 * Host test of the auto white balance step (neuron6_camera_awb.c) against a simulated
 * grey scene under a chosen light: the meter reads the raw averages times the gains in
 * force, clipped at 255. Checks that the loop reaches each reference light from either
 * side and then holds, follows a change of light, stops between lights where the light
 * lies, stays on the line for a light off it, holds in the dark and on clipped frames,
 * and converts gains to shift and multiplier.
 *
 *   sh harness/bench/hosttest/run.sh
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "neuron6_camera_awb.h"

static int failures;

#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL "); printf(__VA_ARGS__); printf("\n"); failures++; } } while (0)

/* ST's IMX335 references: tungsten, fluorescent, daylight. */
static const BSP_CAMERA_AWB_Ref_t refs[3] =
{
  { 2810U, 1240U, 2820U },
  { 4015U, 1820U, 2120U },
  { 6650U, 2440U, 1430U },
};

static BSP_CAMERA_AWB_Config_t config(void)
{
  BSP_CAMERA_AWB_Config_t c;
  c.pRefs       = refs;
  c.RefCount    = 3U;
  c.WhiteGain   = 1049U;
  c.StepPercent = 30U;
  c.MinGreen    = 8U;
  return c;
}

/* A grey scene of brightness S under a light whose neutral gains are (lr, lb). */
static void meter(const BSP_CAMERA_AWB_State_t *s, double S, double lr, double lb,
                  uint32_t *r, uint32_t *g, uint32_t *b)
{
  double vr = S * 1000.0 / lr * s->GainR / 1000.0;
  double vg = S * s->GainG / 1000.0;
  double vb = S * 1000.0 / lb * s->GainB / 1000.0;
  *r = (uint32_t)fmin(255.0, vr + 0.5);
  *g = (uint32_t)fmin(255.0, vg + 0.5);
  *b = (uint32_t)fmin(255.0, vb + 0.5);
}

/* Run until the loop holds; returns the steps taken, or -1 if it never settles. */
static int settle(const BSP_CAMERA_AWB_Config_t *c, BSP_CAMERA_AWB_State_t *s, double S, double lr, double lb)
{
  int quiet = 0;

  for (int n = 0; n < 200; n++)
  {
    uint32_t r, g, b;
    meter(s, S, lr, lb, &r, &g, &b);
    if (BSP_CAMERA_AWB_Step(c, s, r, g, b) == 0U) { if (++quiet >= 5) { return n; } }
    else { quiet = 0; }
  }
  return -1;
}

int main(void)
{
  const BSP_CAMERA_AWB_Config_t c = config();
  BSP_CAMERA_AWB_State_t s;
  int n;

  /* Each reference light, from the far end of the line. */
  for (int k = 0; k < 3; k++)
  {
    const uint32_t start = (k == 0) ? 2000U : 0U;
    BSP_CAMERA_AWB_Init(&c, &s, start);
    n = settle(&c, &s, 60.0, refs[k].GainR, refs[k].GainB);
    CHECK(n >= 0, "light %d: never settles", k);
    CHECK(abs((int)s.Position - k * 1000) <= 25, "light %d: position %u, want %d", k, s.Position, k * 1000);
    CHECK(abs((int)s.GainR - (int)(refs[k].GainR * 1049U / 1000U)) <= 30, "light %d: red gain %u", k, s.GainR);
    CHECK(abs((int)s.GainB - (int)(refs[k].GainB * 1049U / 1000U)) <= 30, "light %d: blue gain %u", k, s.GainB);
    CHECK(abs((int)s.Kelvin - (int)refs[k].Kelvin) <= 80, "light %d: %u K", k, s.Kelvin);
    CHECK(s.GainG == 1049U, "light %d: green gain %u, want the white gain", k, s.GainG);
    /* Once settled, no further changes on the same scene. */
    for (int m = 0; m < 20; m++)
    {
      uint32_t r, g, b;
      meter(&s, 60.0, refs[k].GainR, refs[k].GainB, &r, &g, &b);
      CHECK(BSP_CAMERA_AWB_Step(&c, &s, r, g, b) == 0U, "light %d: hunts after settling", k);
    }
  }

  /* Settled under fluorescent light, then the light changes to daylight: it follows. */
  BSP_CAMERA_AWB_Init(&c, &s, 1000U);
  (void)settle(&c, &s, 50.0, refs[1].GainR, refs[1].GainB);
  n = settle(&c, &s, 50.0, refs[2].GainR, refs[2].GainB);
  CHECK((n >= 0) && (abs((int)s.Position - 2000) <= 25), "light change: position %u", s.Position);

  /* Halfway between fluorescent and daylight. */
  BSP_CAMERA_AWB_Init(&c, &s, 0U);
  n = settle(&c, &s, 60.0, (1820.0 + 2440.0) / 2.0, (2120.0 + 1430.0) / 2.0);
  CHECK((n >= 0) && (abs((int)s.Position - 1500) <= 30), "between lights: position %u", s.Position);

  /* Bluer than daylight: stops at the end of the line. */
  BSP_CAMERA_AWB_Init(&c, &s, 1000U);
  n = settle(&c, &s, 60.0, 3000.0, 1100.0);
  CHECK((n >= 0) && (s.Position >= 1980U), "beyond daylight: position %u", s.Position);

  /* Off the line (a green cast): held to the nearest point on it, not chased. */
  BSP_CAMERA_AWB_Init(&c, &s, 1000U);
  n = settle(&c, &s, 60.0, 2400.0, 2600.0);
  CHECK((n >= 0) && (s.Position <= 2000U), "off the line: position %u", s.Position);

  /* Dark and clipped frames are not judged. */
  BSP_CAMERA_AWB_Init(&c, &s, 1000U);
  CHECK(BSP_CAMERA_AWB_Step(&c, &s, 2U, 3U, 9U) == 0U, "moved on a dark frame");
  CHECK(BSP_CAMERA_AWB_Step(&c, &s, 255U, 200U, 120U) == 0U, "moved on a clipped frame");
  CHECK(s.Position == 1000U, "position changed on frames it should ignore: %u", s.Position);

  /* Gains to the pipe's shift and multiplier, within half a percent. */
  {
    const uint32_t gains[] = { 500U, 1000U, 1049U, 1301U, 1820U, 1992U, 2000U, 2440U, 2960U, 3900U };
    for (unsigned k = 0; k < sizeof gains / sizeof gains[0]; k++)
    {
      uint8_t shift = 9U, mult = 0U;
      BSP_CAMERA_AWB_ToShiftMultiplier(gains[k], &shift, &mult);
      const double got = mult / 128.0 * (double)(1U << shift) * 1000.0;
      CHECK(fabs(got - gains[k]) / gains[k] < 0.005, "gain %u: shift %u multiplier %u = %.0f",
            gains[k], shift, mult, got);
    }
  }

  printf("%s: test_camera_awb\n", failures ? "FAIL" : "PASS");
  return failures ? 1 : 0;
}
