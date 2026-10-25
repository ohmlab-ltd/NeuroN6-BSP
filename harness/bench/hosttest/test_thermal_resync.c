/* Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 *
 * Host test of when neuron6_thermal.c (repository root) restarts its video
 * capture, and how long it leaves the link idle first, built with the real
 * Lepton driver (Components/lepton/lepton.c) against the fake board in
 * fake_thermal/. Each buffer half is written here and announced through the
 * module's own DMA callbacks, as the engine would announce it.
 *
 * The camera streams whole discard packets while it has no video. When its
 * video starts, after the boot and after its first flat-field correction,
 * the stream read can turn to malformed packets, which only a
 * resynchronisation recovers: the chip select high and no clock, for long
 * enough that the camera resets its interface.
 *
 * Checked: every capture start follows at least BSP_THERMAL_RESYNC_IDLE_MS,
 * and IDLE_ENOUGH_MS, with the chip select high; discard
 * packets alone, for seconds, never restart capture; a stream gone
 * malformed restarts it after BSP_THERMAL_RESYNC_BAD_HALVES halves, long
 * before the stall timeout, and not after one fewer; a good half starts the
 * count again; the boundary at half the packets well formed; a stream too
 * poor to count as progress but not poor enough to count as lost, restarted
 * at the stall timeout; a line held low, restarted at the count rather than
 * the stall timeout; an interrupt too late for both halves' events, after
 * which the halves read are still the ones completed; a budget over 2^31 ms
 * waited out rather than ended at once; and a camera powered off after Init,
 * which the next Init powers up again. */
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

#define VIDEO_BYTES  (49U * 8U * BSP_THERMAL_PACKET_BYTES)        /* 64288 */
#define HALF_BYTES   (VIDEO_BYTES / 2U)
#define PER_HALF     (HALF_BYTES / BSP_THERMAL_PACKET_BYTES)      /* 196   */
/* A half at 12.5 MHz, the clock the fake board's bus clock gives the link. */
#define HALF_MS      21U
/* The idle that resets the camera's interface. */
#define IDLE_ENOUGH_MS  300U

static uint8_t  video[VIDEO_BYTES] __attribute__((aligned(32)));
static uint16_t frame[BSP_THERMAL_FRAME_PIXELS];
static SPI_HandleTypeDef link = { .Instance = SPI1 };

static uint32_t produced;   /* halves announced since the transfer started */
static int      started;    /* the transfer they belong to                */

/* The next half: Good of its packets discard packets, the rest malformed,
 * then announced and read. A transfer started since the last half begins
 * at the start of the buffer. */
static void half(uint32_t Good)
{
  uint8_t *base;

  if (captures != started)
  {
    started = captures;
    produced = 0U;
  }
  base = &capture_buffer[(produced % 2U) * HALF_BYTES];
  for (uint32_t i = 0U; i < PER_HALF; i++)
  {
    uint8_t *packet = &base[i * BSP_THERMAL_PACKET_BYTES];

    /* Pixel bytes where a header should be: no packet the camera makes. */
    (void)memset(packet, 0x71, BSP_THERMAL_PACKET_BYTES);
    if (i < Good)
    {
      packet[0] = 0x0FU;                 /* a discard packet */
      packet[1] = 0x00U;
    }
  }
  produced++;
  now += HALF_MS;
  if ((produced % 2U) == 1U)
  {
    HAL_SPI_RxHalfCpltCallback(&link);
  }
  else
  {
    HAL_SPI_RxCpltCallback(&link);
  }
  (void)BSP_THERMAL_GetFrame(0, frame, 0U);
}

/* The next half, every byte zero, as a line held low reads: packet 0 over
 * and over. */
static void half_zero(void)
{
  if (captures != started)
  {
    started = captures;
    produced = 0U;
  }
  (void)memset(&capture_buffer[(produced % 2U) * HALF_BYTES], 0, HALF_BYTES);
  produced++;
  now += HALF_MS;
  if ((produced % 2U) == 1U)
  {
    HAL_SPI_RxHalfCpltCallback(&link);
  }
  else
  {
    HAL_SPI_RxCpltCallback(&link);
  }
  (void)BSP_THERMAL_GetFrame(0, frame, 0U);
}

/* A camera booted, its video started, and the first read, which starts the
 * transfer. */
static void start_video(void)
{
  (void)BSP_THERMAL_DeInit(0);
  (void)BSP_THERMAL_PowerOff(0);
  fake_reset(1000U);
  CHECK(BSP_THERMAL_Init(0) == BSP_ERROR_NONE, "Init");
  CHECK(BSP_THERMAL_VideoStart(0, video, VIDEO_BYTES) == BSP_ERROR_NONE, "VideoStart");
  (void)BSP_THERMAL_GetFrame(0, frame, 0U);
  CHECK(captures == 1, "the first read started %d transfers", captures);
  CHECK(capture_buffer == video, "the transfer is not into the buffer given");
  CHECK(capture_bytes == VIDEO_BYTES, "the transfer is %u bytes", (unsigned)capture_bytes);
  started = captures;
  produced = 0U;
}

static void check_idle(const char *where)
{
  CHECK(idle_before_capture >= BSP_THERMAL_RESYNC_IDLE_MS,
        "%s: the select was high %lu ms before the start", where,
        (unsigned long)idle_before_capture);
  CHECK(idle_before_capture >= IDLE_ENOUGH_MS,
        "%s: the select was high %lu ms before the start, under %u", where,
        (unsigned long)idle_before_capture, IDLE_ENOUGH_MS);
}

/* Discard packets alone for Ms milliseconds: the camera booting, or its
 * flat-field correction. Never a restart. */
static void discards(uint32_t ms)
{
  start_video();
  check_idle("first start");
  for (uint32_t t = 0U; t < ms; t += HALF_MS)
  {
    half(PER_HALF);
  }
  CHECK(captures == 1, "%lu ms of discard packets: %d restarts", (unsigned long)ms, captures - 1);
}

/* Bad halves of Good well-formed packets each, Count of them, then the
 * restarts expected. */
static void bad_run(uint32_t good, uint32_t count, int restarts)
{
  start_video();
  for (uint32_t n = 0U; n < count; n++)
  {
    half(good);
  }
  CHECK((captures - 1) == restarts, "%lu halves of %lu good packets: %d restarts, %d expected",
        (unsigned long)count, (unsigned long)good, captures - 1, restarts);
  if (restarts != 0)
  {
    check_idle("restart");
  }
}

/* Bad halves, broken by one good half before they reach the count. */
static void interrupted(void)
{
  start_video();
  for (uint32_t n = 0U; n < (BSP_THERMAL_RESYNC_BAD_HALVES - 1U); n++)
  {
    half(0U);
  }
  half(PER_HALF);
  for (uint32_t n = 0U; n < (BSP_THERMAL_RESYNC_BAD_HALVES - 1U); n++)
  {
    half(0U);
  }
  CHECK(captures == 1, "bad halves broken by a good one: %d restarts", captures - 1);
  half(0U);
  CHECK(captures == 2, "the count after a good half: %d restarts", captures - 1);
  if (captures == 2)
  {
    check_idle("restart after a good half");
  }
}

/* Six in ten packets well formed: not progress, which wants nine in ten, and
 * not lost, which wants under half. Restarted only at the stall timeout. */
static void middling(void)
{
  const uint32_t good = (PER_HALF * 6U) / 10U;
  uint32_t       elapsed = 0U;

  start_video();
  while ((captures == 1) && (elapsed < (BSP_THERMAL_STALL_TIMEOUT_MS + 500U)))
  {
    half(good);
    elapsed += HALF_MS;
  }
  CHECK(captures == 2, "a middling stream was not restarted in %lu ms", (unsigned long)elapsed);
  CHECK(elapsed > BSP_THERMAL_STALL_TIMEOUT_MS, "a middling stream restarted after %lu ms",
        (unsigned long)elapsed);
  CHECK(elapsed <= (BSP_THERMAL_STALL_TIMEOUT_MS + (2U * HALF_MS)),
        "a middling stream restarted after %lu ms", (unsigned long)elapsed);
  if (captures == 2)
  {
    check_idle("restart at the stall timeout");
  }
}

/* A line held low: bad halves, restarted at the count, long before the
 * stall timeout. */
static void held_low(void)
{
  start_video();
  for (uint32_t n = 0U; n < BSP_THERMAL_RESYNC_BAD_HALVES; n++)
  {
    half_zero();
  }
  CHECK(captures == 2, "%lu halves of a line held low: %d restarts, 1 expected",
        (unsigned long)BSP_THERMAL_RESYNC_BAD_HALVES, captures - 1);
}

/* Both halves complete before the interrupt is taken, so the HAL reports
 * only the second. Afterwards each completed half holds discard packets and
 * the half being written what no camera sends: read from the right half,
 * the stream is healthy and never restarted. */
static void late_interrupt(void)
{
  start_video();
  (void)memset(capture_buffer, 0x71, VIDEO_BYTES);
  for (uint32_t i = 0U; i < (2U * PER_HALF); i++)
  {
    capture_buffer[i * BSP_THERMAL_PACKET_BYTES]      = 0x0FU;
    capture_buffer[(i * BSP_THERMAL_PACKET_BYTES) + 1U] = 0x00U;
  }
  produced = 2U;
  now += 2U * HALF_MS;
  HAL_SPI_RxCpltCallback(&link);
  (void)BSP_THERMAL_GetFrame(0, frame, 0U);

  for (uint32_t n = 0U; n < (4U * BSP_THERMAL_RESYNC_BAD_HALVES); n++)
  {
    const uint32_t done = produced % 2U;
    uint8_t *ready = &capture_buffer[done * HALF_BYTES];
    uint8_t *other = &capture_buffer[(1U - done) * HALF_BYTES];

    (void)memset(other, 0x71, HALF_BYTES);
    (void)memset(ready, 0x71, HALF_BYTES);
    for (uint32_t i = 0U; i < PER_HALF; i++)
    {
      ready[i * BSP_THERMAL_PACKET_BYTES]      = 0x0FU;
      ready[(i * BSP_THERMAL_PACKET_BYTES) + 1U] = 0x00U;
    }
    produced++;
    now += HALF_MS;
    if (done == 0U)
    {
      HAL_SPI_RxHalfCpltCallback(&link);
    }
    else
    {
      HAL_SPI_RxCpltCallback(&link);
    }
    (void)BSP_THERMAL_GetFrame(0, frame, 0U);
  }
  CHECK(captures == 1, "after a late interrupt the halves read were not the ones completed: "
        "%d restarts", captures - 1);
}

/* A budget over 2^31 ms is waited out, not ended at the first pass. */
static void long_budget(void)
{
  const uint32_t budget = 0x80000005U;
  uint32_t t0;
  int32_t  status;

  start_video();
  tick_slow = 10U;                  /* the first pass sees milliseconds go by */
  tick_step = 0x08000000U;          /* then each reading of the clock, 37 hours */
  t0 = now;
  status = BSP_THERMAL_GetFrame(0, frame, budget);
  tick_slow = 0U;
  tick_step = 0U;
  CHECK(status == BSP_ERROR_BUSY, "GetFrame with no frame returned %ld", (long)status);
  CHECK((now - t0) >= budget, "a budget of 0x%08lX ms ended after 0x%08lX",
        (unsigned long)budget, (unsigned long)(now - t0));
}

/* A camera powered off after Init is down: the next Init powers it up and
 * waits for its boot, and video started before the power-off is over. */
static void power_off_after_init(void)
{
  int ups;

  start_video();
  ups = power_ups;
  CHECK(BSP_THERMAL_PowerOff(0) == BSP_ERROR_NONE, "PowerOff");
  CHECK(BSP_THERMAL_GetFrame(0, frame, 0U) == BSP_ERROR_NO_INIT,
        "video still running after the power-off");
  CHECK(!camera_on, "the camera still on after the power-off");
  CHECK(BSP_THERMAL_Init(0) == BSP_ERROR_NONE, "Init after the power-off");
  CHECK(power_ups == (ups + 1), "Init after the power-off powered the camera up %d times",
        power_ups - ups);
}

int main(void)
{
  discards(3000U);

  /* Malformed from the start: restarted at the count, and not before. */
  bad_run(0U, BSP_THERMAL_RESYNC_BAD_HALVES - 1U, 0);
  bad_run(0U, BSP_THERMAL_RESYNC_BAD_HALVES, 1);
  CHECK((BSP_THERMAL_RESYNC_BAD_HALVES * HALF_MS) < BSP_THERMAL_STALL_TIMEOUT_MS,
        "the count takes %lu ms, no sooner than the stall timeout",
        (unsigned long)(BSP_THERMAL_RESYNC_BAD_HALVES * HALF_MS));

  /* The boundary: half the packets well formed is not lost, one fewer is. */
  bad_run(PER_HALF / 2U, BSP_THERMAL_RESYNC_BAD_HALVES, 0);
  bad_run((PER_HALF / 2U) - 1U, BSP_THERMAL_RESYNC_BAD_HALVES, 1);

  /* Still malformed after a restart: restarted again at the count. */
  bad_run(0U, 2U * BSP_THERMAL_RESYNC_BAD_HALVES, 2);

  interrupted();
  middling();
  held_low();
  late_interrupt();
  long_budget();
  power_off_after_init();

  printf("thermal resync: %d checks, %d failed\n", checks, failures);
  return (failures == 0) ? 0 : 1;
}
