/* Host test of the motion sensors (neuron6_motion_sensor.c, with the LSM6DSL
 * and MMC5603NJ drivers) against a simulated I2C bus carrying an LSM6DSL at
 * 0xD4 and an MMC5603NJ at 0x60. From this folder, with a POSIX gcc:
 *
 *   gcc -std=gnu11 -Wall -Ifake_thermal -I../../.. -I../../../Components/lsm6dsl \
 *       -I../../../Components/mmc5603nj test_motion.c ../../../neuron6_motion_sensor.c \
 *       ../../../Components/lsm6dsl/lsm6dsl.c ../../../Components/mmc5603nj/mmc5603nj.c \
 *       -o /tmp/test_motion
 *
 * Checks that a call acting on one function refuses 0 or two functions
 * rather than taking them to another part's driver, that the gyroscope runs
 * at 12.5 Hz when asked, and that one failed write while the magnetometer is
 * paused does not leave its readings frozen.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#include "neuron6_motion_sensor.h"

#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>

/* Clock. */
static uint32_t now;
static void mmc_update(void);
void HAL_Delay(uint32_t d) { now += d; mmc_update(); }
int32_t BSP_GetTick(void) { now += 1U; mmc_update(); return (int32_t)now; }

/* LSM6DSL. */
static int     lsm_present = 1;
static uint8_t lsm[256];
static void lsm_reset(void) { memset(lsm, 0, sizeof lsm); lsm[0x0F] = 0x6A; lsm[0x12] = 0x04; }

/* MMC5603NJ. */
static int      mmc_present = 1;
static uint8_t  mmc[256];
static int      mmc_running;       /* CTRL2 CMM_EN */
static uint32_t mmc_odr;
static uint32_t mmc_next;          /* next free-running sample time */
static uint32_t mmc_sample;        /* sample counter, encoded into X */
static int      mmc_triggers_m;    /* TAKE_MEAS_M writes seen */
static int      fail_write_reg = -1, fail_write_val = -1;

static void mmc_new_sample(void)
{
  uint32_t x = 524288U + 1000U * (++mmc_sample);
  mmc[0] = (uint8_t)(x >> 12); mmc[1] = (uint8_t)(x >> 4); mmc[6] = (uint8_t)((x & 0xFU) << 4);
  mmc[2] = 0x80; mmc[3] = 0; mmc[7] = 0; mmc[4] = 0x80; mmc[5] = 0; mmc[8] = 0;
  mmc[0x18] |= 0x40;
}

static void mmc_update(void)
{
  if (mmc_running && mmc_odr) {
    while ((int32_t)(now - mmc_next) >= 0) { mmc_new_sample(); mmc_next += 1000U / mmc_odr; }
  }
}

static void mmc_reset(void)
{
  memset(mmc, 0, sizeof mmc); mmc[0x39] = 0x10; mmc_running = 0; mmc_odr = 0;
}

/* Bus. */
int32_t BSP_I2C1_Init(void)   { return 0; }
int32_t BSP_I2C1_DeInit(void) { return 0; }

int32_t BSP_I2C1_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  if (DevAddr == 0xD4 && lsm_present) {
    for (uint16_t i = 0; i < Length; i++) {
      uint8_t r = (uint8_t)(Reg + i);
      if (r == 0x12 && (pData[i] & 0x01)) { lsm_reset(); continue; }
      lsm[r] = pData[i];
    }
    return 0;
  }
  if (DevAddr == 0x60 && mmc_present) {
    uint8_t v = pData[0];
    if ((int)Reg == fail_write_reg && (int)v == fail_write_val) { fail_write_reg = -1; return -1; }
    switch (Reg) {
      case 0x1B:
        if (v & 0x01) { mmc_triggers_m++; if (!mmc_running) mmc_new_sample(); }
        if (v & 0x02) { if (!mmc_running) { mmc[0x09] = 125; mmc[0x18] |= 0x80; } }
        break;
      case 0x1C: if (v & 0x80) mmc_reset(); break;
      case 0x1D:
        if (v & 0x10) { mmc_running = 1; mmc_next = now + 1000U / (mmc_odr ? mmc_odr : 1U); }
        else mmc_running = 0;
        break;
      case 0x1A: mmc_odr = v; break;
      default: mmc[Reg] = v; break;
    }
    return 0;
  }
  return -1;
}

int32_t BSP_I2C1_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  if (DevAddr == 0xD4 && lsm_present) {
    for (uint16_t i = 0; i < Length; i++) pData[i] = lsm[(uint8_t)(Reg + i)];
    return 0;
  }
  if (DevAddr == 0x60 && mmc_present) {
    for (uint16_t i = 0; i < Length; i++) pData[i] = mmc[(uint8_t)(Reg + i)];
    if (Reg == 0x00) mmc[0x18] &= (uint8_t)~0x40;
    if (Reg == 0x09) mmc[0x18] &= (uint8_t)~0x80;
    return 0;
  }
  return -1;
}

/* Tests. */
static int checks, fails;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("FAIL line %d: ", __LINE__); \
                           printf(__VA_ARGS__); printf("\n"); } } while (0)

/* Runs f in a child; the signal it died on, or 0. */
static int crashes(void (*f)(void))
{
  pid_t pid = fork();
  if (pid == 0) { f(); _exit(0); }
  int st = 0;
  waitpid(pid, &st, 0);
  return WIFSIGNALED(st) ? WTERMSIG(st) : 0;
}

static void setodr_fn0(void) { (void)BSP_MOTION_SENSOR_SetOutputDataRate(MOTION_SENSOR_LSM6DSL_0, 0U, 52.0f); }
static void setodr_fn3(void)
{
  (void)BSP_MOTION_SENSOR_SetOutputDataRate(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO, 52.0f);
}
static void getaxes_mag_fn0(void)
{
  BSP_MOTION_SENSOR_Axes_t a;
  (void)BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_MMC5603NJ_0, 0U, &a);
}

int main(void)
{
  float s = 0.0f, odr = 0.0f, t = 0.0f;
  int32_t r;
  BSP_MOTION_SENSOR_Axes_t a;

  /* 1. Only the magnetometer up: function 0 does not reach the inertial
   *    part's driver, whose object was never bound. */
  lsm_present = 0; mmc_present = 1; mmc_reset();
  r = BSP_MOTION_SENSOR_Init(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO);
  CHECK(r == BSP_ERROR_NONE, "Init(MMC5603NJ, MAG) = %ld", (long)r);
  CHECK(crashes(getaxes_mag_fn0) == 0, "GetAxes(MMC5603NJ, 0) died on a signal");
  r = BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_MMC5603NJ_0, 0U, &a);
  CHECK(r == BSP_ERROR_WRONG_PARAM, "GetAxes(MMC5603NJ, 0) = %ld", (long)r);
  (void)BSP_MOTION_SENSOR_DeInit(MOTION_SENSOR_MMC5603NJ_0);

  /* 2. Only the inertial part up: 0 or two functions are refused. */
  lsm_present = 1; lsm_reset(); mmc_present = 0;
  r = BSP_MOTION_SENSOR_Init(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO);
  CHECK(r == BSP_ERROR_NONE, "Init(LSM6DSL, ACC|GYRO) = %ld", (long)r);
  r = BSP_MOTION_SENSOR_GetSensitivity(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO, &s);
  CHECK(r == BSP_ERROR_WRONG_PARAM, "GetSensitivity(LSM6DSL, ACC|GYRO) = %ld", (long)r);
  {
    const uint8_t ctrl1 = lsm[0x10];
    r = BSP_MOTION_SENSOR_SetFullScale(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO, 16);
    CHECK(r == BSP_ERROR_WRONG_PARAM && lsm[0x10] == ctrl1, "SetFullScale(LSM6DSL, ACC|GYRO) = %ld", (long)r);
  }
  CHECK(crashes(setodr_fn0) == 0, "SetOutputDataRate(LSM6DSL, 0) died on a signal");
  CHECK(crashes(setodr_fn3) == 0, "SetOutputDataRate(LSM6DSL, ACC|GYRO) died on a signal");
  r = BSP_MOTION_SENSOR_SetOutputDataRate(MOTION_SENSOR_LSM6DSL_0, 0U, 52.0f);
  CHECK(r == BSP_ERROR_WRONG_PARAM, "SetOutputDataRate(LSM6DSL, 0) = %ld", (long)r);
  r = BSP_MOTION_SENSOR_GetSensitivity(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO, &s);
  CHECK(r == BSP_ERROR_NONE && s > 0.06f && s < 0.062f, "GetSensitivity(LSM6DSL, ACC) = %ld, %.4f", (long)r, s);

  /* 3. The gyroscope's slowest rate, 12.5 Hz, is its code 1. */
  r = BSP_MOTION_SENSOR_SetOutputDataRate(MOTION_SENSOR_LSM6DSL_0, MOTION_GYRO, 12.5f);
  (void)BSP_MOTION_SENSOR_GetOutputDataRate(MOTION_SENSOR_LSM6DSL_0, MOTION_GYRO, &odr);
  CHECK(r == BSP_ERROR_NONE && odr == 12.5f && (lsm[0x11] >> 4) == 1U,
        "gyroscope at 12.5 Hz: r=%ld, reports %.1f Hz, code %u", (long)r, odr, (unsigned)(lsm[0x11] >> 4));
  (void)BSP_MOTION_SENSOR_DeInit(MOTION_SENSOR_LSM6DSL_0);

  /* 4. The magnetometer free-running at 50 Hz: one write not acknowledged
   *    inside a temperature reading, which pauses it, does not leave every
   *    later reading the same. */
  mmc_present = 1; mmc_reset();
  r = BSP_MOTION_SENSOR_Init(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO);
  CHECK(r == BSP_ERROR_NONE, "Init(MMC5603NJ, MAG) = %ld", (long)r);
  r = BSP_MOTION_SENSOR_SetOutputDataRate(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO, 50.0f);
  CHECK(r == BSP_ERROR_NONE, "SetOutputDataRate(MMC5603NJ, 50) = %ld", (long)r);
  HAL_Delay(25); (void)BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO, &a);
  {
    const int32_t x1 = a.x;
    HAL_Delay(25); (void)BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO, &a);
    CHECK(a.x != x1, "free-running samples move: %ld -> %ld", (long)x1, (long)a.x);
  }
  fail_write_reg = 0x1B; fail_write_val = 0x22;   /* AUTO_SR_EN | TAKE_MEAS_T, not acknowledged */
  r = BSP_MOTION_SENSOR_GetTemperature(MOTION_SENSOR_MMC5603NJ_0, &t);
  CHECK(r == BSP_ERROR_COMPONENT_FAILURE, "GetTemperature with one NACK = %ld", (long)r);
  {
    int moved = 0, ok = 0;
    int32_t first = 0;
    for (int i = 0; i < 50; i++) {
      HAL_Delay(25);
      if (BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO, &a) == BSP_ERROR_NONE) ok++;
      if (i == 0) first = a.x; else if (a.x != first) moved++;
    }
    CHECK(ok == 50 && moved > 0, "after it, 50 reads: %d OK, %d differ from the first", ok, moved);
  }
  (void)BSP_MOTION_SENSOR_DeInit(MOTION_SENSOR_MMC5603NJ_0);

  printf("test_motion: %d checks, %d failed\n", checks, fails);
  return fails != 0;
}
