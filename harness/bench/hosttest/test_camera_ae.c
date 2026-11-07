/* Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 *
 * Host test of the auto-exposure step (neuron6_camera_ae.c) against a simulated
 * sensor: the meter reads scene x exposure x linear gain, clipped at 255, and a
 * change shows only after the settle frames, as on the IMX335. Checks that the
 * loop converges from both sides without hunting, holds its limits, keeps long
 * exposures to whole mains half-periods, uses time before gain, recovers from a
 * saturated frame, and stops at its ceiling in the dark.
 *
 *   sh harness/bench/hosttest/run.sh
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "neuron6_camera_ae.h"

static int failures;

#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL "); printf(__VA_ARGS__); printf("\n"); failures++; } } while (0)

/* 30 fps on the IMX335: 33.3 ms of exposure, 7.4 us lines, 0.3 dB steps. */
static BSP_CAMERA_AE_Config_t config(void)
{
  BSP_CAMERA_AE_Config_t c;
  c.Target        = 50U;
  c.ExposureMinUs = 8U;
  c.ExposureMaxUs = 33266U;
  c.GainMaxMdb    = 36000U;
  c.GainStepMdb   = 300U;
  c.FlickerHz     = 50U;
  c.SettleFrames  = 3U;
  return c;
}

static uint32_t meter(double scene, uint32_t exposure_us, uint32_t gain_mdb)
{
  double v = scene * (double)exposure_us * pow(10.0, (double)gain_mdb / 20000.0);
  return (v >= 255.0) ? 255U : (uint32_t)v;
}

/* Run Frames frames on Scene (meter counts per microsecond at 0 dB); returns the last
 * average and the number of writes. */
static uint32_t run(const BSP_CAMERA_AE_Config_t *c, BSP_CAMERA_AE_State_t *st, double scene,
                    unsigned frames, unsigned *writes, uint32_t *last_avg)
{
  uint32_t applied_exp = st->ExposureUs, applied_gain = st->GainMdb;
  uint32_t pend_exp = applied_exp, pend_gain = applied_gain;
  unsigned pend_at = 0U, frame;
  uint32_t avg = 0U;

  *writes = 0U;
  for (frame = 0U; frame < frames; frame++)
  {
    if ((pend_at != 0U) && (frame >= pend_at)) { applied_exp = pend_exp; applied_gain = pend_gain; pend_at = 0U; }
    avg = meter(scene, applied_exp, applied_gain);
    if (BSP_CAMERA_AE_Step(c, st, avg) != 0U)
    {
      (*writes)++;
      pend_exp = st->ExposureUs; pend_gain = st->GainMdb; pend_at = frame + c->SettleFrames;
    }
  }
  *last_avg = avg;
  return avg;
}

static int within_deadband(uint32_t avg, uint32_t target)
{
  uint32_t e = (avg > target) ? avg - target : target - avg;
  return (e * 16U) <= target + 16U;   /* a step of quantisation either side */
}

int main(void)
{
  BSP_CAMERA_AE_Config_t c = config();
  BSP_CAMERA_AE_State_t st;
  unsigned writes, writes2;
  uint32_t avg;

  /* Bright scene from a long start: exposure falls, gain stays 0. */
  st.ExposureUs = 10000U; st.GainMdb = 0U; st.Wait = 0U;
  run(&c, &st, 0.05, 80U, &writes, &avg);
  CHECK(within_deadband(avg, c.Target), "bright: settled at %u, target %u", avg, c.Target);
  CHECK(st.GainMdb == 0U, "bright: gain %u, want 0", st.GainMdb);
  CHECK(writes <= 12U, "bright: %u writes to settle", writes);
  run(&c, &st, 0.05, 60U, &writes2, &avg);
  CHECK(writes2 == 0U, "bright: still changing after settling (%u writes)", writes2);

  /* Dim scene: exposure to whole 10 ms half-periods, then gain. */
  st.ExposureUs = 1000U; st.GainMdb = 0U; st.Wait = 0U;
  run(&c, &st, 0.0005, 120U, &writes, &avg);
  CHECK(within_deadband(avg, c.Target), "dim: settled at %u, target %u", avg, c.Target);
  CHECK((st.ExposureUs % 10000U) == 0U, "dim: exposure %u not a whole 10 ms", st.ExposureUs);
  CHECK(st.GainMdb > 0U, "dim: no gain used");
  CHECK((st.GainMdb % 300U) == 0U, "dim: gain %u off the 0.3 dB grid", st.GainMdb);
  run(&c, &st, 0.0005, 60U, &writes2, &avg);
  CHECK(writes2 == 0U, "dim: hunting after settling (%u writes)", writes2);

  /* Saturated: recovers from a frame that reads full scale. */
  st.ExposureUs = 33266U; st.GainMdb = 20000U; st.Wait = 0U;
  run(&c, &st, 0.5, 120U, &writes, &avg);
  CHECK(within_deadband(avg, c.Target), "saturated: settled at %u", avg);
  CHECK(st.ExposureUs >= c.ExposureMinUs, "saturated: exposure %u under the minimum", st.ExposureUs);

  /* Dark: stops at the longest exposure and the gain ceiling, and stays there. */
  st.ExposureUs = 5000U; st.GainMdb = 0U; st.Wait = 0U;
  run(&c, &st, 0.000001, 120U, &writes, &avg);
  CHECK(st.GainMdb == c.GainMaxMdb, "dark: gain %u, want the ceiling %u", st.GainMdb, c.GainMaxMdb);
  CHECK(st.ExposureUs <= c.ExposureMaxUs, "dark: exposure %u over the frame", st.ExposureUs);
  CHECK(st.ExposureUs >= 30000U, "dark: exposure %u, want the longest whole half-period", st.ExposureUs);
  run(&c, &st, 0.000001, 30U, &writes2, &avg);
  CHECK(writes2 == 0U, "dark: still writing at its limits (%u)", writes2);

  /* A noisy meter (+/-3 counts) on a settled scene does not make it hunt. */
  {
    uint32_t seed = 12345U, changes = 0U;
    unsigned frame;
    st.ExposureUs = 1000U; st.GainMdb = 0U; st.Wait = 0U;
    run(&c, &st, 0.05, 80U, &writes, &avg);
    for (frame = 0U; frame < 300U; frame++)
    {
      int32_t noisy;
      seed = (seed * 1103515245U) + 12345U;
      noisy = (int32_t)meter(0.05, st.ExposureUs, st.GainMdb) + (int32_t)((seed >> 16) % 7U) - 3;
      changes += BSP_CAMERA_AE_Step(&c, &st, (noisy < 0) ? 0U : (uint32_t)noisy);
    }
    CHECK(changes <= 3U, "noise: %u changes in 300 settled frames", changes);
  }

  /* No flicker setting: a long exposure is not quantised. */
  c.FlickerHz = 0U;
  st.ExposureUs = 1000U; st.GainMdb = 0U; st.Wait = 0U;
  run(&c, &st, 0.0023, 120U, &writes, &avg);
  CHECK(within_deadband(avg, c.Target), "no flicker: settled at %u", avg);
  CHECK(st.GainMdb == 0U, "no flicker: gain %u used before time ran out", st.GainMdb);

  /* Bad configuration and a zero reading are safe. */
  CHECK(BSP_CAMERA_AE_Step(NULL, &st, 10U) == 0U, "NULL config changed something");
  c = config(); c.Target = 0U;
  CHECK(BSP_CAMERA_AE_Step(&c, &st, 10U) == 0U, "zero target changed something");
  c = config(); st.ExposureUs = 1000U; st.GainMdb = 0U; st.Wait = 0U;
  CHECK(BSP_CAMERA_AE_Step(&c, &st, 0U) == 1U, "a black frame should lengthen the exposure");
  CHECK(st.ExposureUs > 1000U, "a black frame shortened the exposure (%u)", st.ExposureUs);

  if (failures != 0) { printf("test_camera_ae: %d failed\n", failures); return 1; }
  printf("test_camera_ae: all passed\n");
  return 0;
}
