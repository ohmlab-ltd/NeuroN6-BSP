/* Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 *
 * Host test of how neuron6_thermal.c (repository root) powers the thermal
 * camera and waits for it to boot, built with the real Lepton driver
 * (Components/lepton/lepton.c) against the fake board in fake_thermal/, whose
 * camera never boots if anything is on its bus early in the boot.
 *
 * Checked: BSP_THERMAL_Init on its own, and after the application's own
 * BSP_THERMAL_PowerOn at several distances; nothing on the bus too early; no
 * second power-up when the application powered the camera; one after a
 * power-down; a camera that never answers; the tick wrapping through the
 * wait, and the reset released at tick 0. */
#include <stdio.h>
#include <string.h>

#include "neuron6_thermal.h"
#include "neuron6.h"
#include "neuron6_bus.h"
#include "fake_thermal.h"

static int failures;
static int checks;

#define CHECK(cond, ...)                                   \
  do {                                                     \
    checks++;                                              \
    if (!(cond)) {                                         \
      failures++;                                          \
      printf("FAIL %s:%d: ", __FILE__, __LINE__);          \
      printf(__VA_ARGS__);                                 \
      printf("\n");                                        \
    }                                                      \
  } while (0)

/* The cases. */

/* Back to a board just reset: camera off and the module down. */
static void fresh(uint32_t start)
{
  (void)BSP_THERMAL_DeInit(0);
  (void)BSP_THERMAL_PowerOff(0);
  fake_reset(start);
}

/* Init on its own: one power-up, nothing on the bus for the quiet period,
 * a booted camera, and the call back soon after the quiet period ends. */
static void init_alone(uint32_t start)
{
  int32_t status;

  fresh(start);
  status = BSP_THERMAL_Init(0);
  CHECK(status == BSP_ERROR_NONE, "start %lu: Init returned %ld", (unsigned long)start, (long)status);
  CHECK(power_ups == 1, "start %lu: %d power-ups", (unsigned long)start, power_ups);
  CHECK(transactions > 0, "start %lu: the camera was never asked", (unsigned long)start);
  CHECK((first_transaction_at - released_at) >= BSP_THERMAL_BOOT_QUIET_MS,
        "start %lu: first asked %lu ms after the release", (unsigned long)start,
        (unsigned long)(first_transaction_at - released_at));
  CHECK(!disturbed, "start %lu: the boot was disturbed", (unsigned long)start);
  CHECK((now - released_at) < (BSP_THERMAL_BOOT_QUIET_MS + 100U),
        "start %lu: Init returned %lu ms after the release", (unsigned long)start,
        (unsigned long)(now - released_at));
}

/* The application powers the camera, waits Gap ms itself, then calls Init:
 * no second power-up, nothing asked before the quiet period from the
 * application's power-up, and only the rest of the quiet period waited. */
static void init_after_power_on(uint32_t start, uint32_t gap)
{
  uint32_t called_at;
  int32_t  status;

  fresh(start);
  CHECK(BSP_THERMAL_PowerOn(0) == BSP_ERROR_NONE, "PowerOn");
  now += gap;
  called_at = now;
  status = BSP_THERMAL_Init(0);
  CHECK(status == BSP_ERROR_NONE, "gap %lu: Init returned %ld", (unsigned long)gap, (long)status);
  CHECK(power_ups == 1, "gap %lu: %d power-ups", (unsigned long)gap, power_ups);
  CHECK((first_transaction_at - released_at) >= BSP_THERMAL_BOOT_QUIET_MS,
        "gap %lu: first asked %lu ms after the release", (unsigned long)gap,
        (unsigned long)(first_transaction_at - released_at));
  CHECK(!disturbed, "gap %lu: the boot was disturbed", (unsigned long)gap);
  if (gap >= BSP_THERMAL_BOOT_QUIET_MS)
  {
    CHECK((now - called_at) < 50U, "gap %lu: Init blocked %lu ms", (unsigned long)gap,
          (unsigned long)(now - called_at));
    CHECK(transactions <= 2, "gap %lu: asked %d times", (unsigned long)gap, transactions);
  }
  else
  {
    CHECK((now - released_at) < (BSP_THERMAL_BOOT_QUIET_MS + 100U),
          "gap %lu: Init returned %lu ms after the release", (unsigned long)gap,
          (unsigned long)(now - released_at));
  }
}

/* Powered up and then down again: Init powers it up itself. */
static void init_after_power_off(void)
{
  fresh(1000U);
  (void)BSP_THERMAL_PowerOn(0);
  now += 5000U;
  (void)BSP_THERMAL_PowerOff(0);
  now += 5000U;
  CHECK(BSP_THERMAL_Init(0) == BSP_ERROR_NONE, "Init after a power-down");
  CHECK(power_ups == 2, "after a power-down: %d power-ups", power_ups);
  CHECK(!disturbed, "after a power-down: the boot was disturbed");
}

/* Nobody there: Init gives up at the boot timeout, asks at least once, and
 * leaves the camera powered down. */
static void init_absent(uint32_t gap)
{
  int32_t status;

  fresh(1000U);
  present = 0;
  if (gap != 0U)
  {
    (void)BSP_THERMAL_PowerOn(0);
    now += gap;
  }
  status = BSP_THERMAL_Init(0);
  CHECK(status == BSP_ERROR_COMPONENT_FAILURE, "absent, gap %lu: Init returned %ld",
        (unsigned long)gap, (long)status);
  CHECK(transactions >= 1, "absent, gap %lu: never asked", (unsigned long)gap);
  CHECK(!camera_on, "absent, gap %lu: left powered", (unsigned long)gap);
  if (gap < BSP_THERMAL_BOOT_TIMEOUT_MS)
  {
    CHECK((now - released_at) >= BSP_THERMAL_BOOT_TIMEOUT_MS,
          "absent, gap %lu: gave up %lu ms after the release", (unsigned long)gap,
          (unsigned long)(now - released_at));
    CHECK((now - released_at) < (BSP_THERMAL_BOOT_TIMEOUT_MS + 100U),
          "absent, gap %lu: gave up %lu ms after the release", (unsigned long)gap,
          (unsigned long)(now - released_at));
  }
}

int main(void)
{
  /* Starting points: an ordinary one, the tick wrapping during the quiet
   * period, and the reset released at tick 0 (the power cycle takes
   * BSP_THERMAL_POWER_CYCLE_MS + BSP_THERMAL_RESET_PULSE_MS). */
  const uint32_t release_at_zero = 0U - (BSP_THERMAL_POWER_CYCLE_MS + BSP_THERMAL_RESET_PULSE_MS);
  const uint32_t starts[] = { 0U, 1U, 1000U, 0xFFFFFFFFU - 1000U, release_at_zero, 0x7FFFFF00U };
  const uint32_t gaps[] = { 0U, 1U, 500U, 1499U, 1999U, 2000U, 2001U, 2500U, 3000U, 60000U };

  for (size_t i = 0U; i < (sizeof(starts) / sizeof(starts[0])); i++)
  {
    init_alone(starts[i]);
    for (size_t j = 0U; j < (sizeof(gaps) / sizeof(gaps[0])); j++)
    {
      init_after_power_on(starts[i], gaps[j]);
    }
  }
  fresh(release_at_zero);
  init_alone(release_at_zero);
  CHECK(released_at == 0U, "the release fell at %lu, not 0", (unsigned long)released_at);

  init_after_power_off();
  init_absent(0U);
  init_absent(2500U);
  init_absent(60000U);

  printf("thermal boot: %d checks, %d failed\n", checks, failures);
  return (failures == 0) ? 0 : 1;
}
