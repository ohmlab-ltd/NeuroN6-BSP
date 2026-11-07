/* Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 *
 * Host test of the VL53L5CX driver's porting layer (Components/vl53l5cx/platform.c)
 * against a simulated sensor bus: long transfers go in chunks at rising register
 * addresses with the wait hook between them, a failed transfer stops the rest,
 * 32-bit words are byte-swapped, and waits last at least as long as asked while
 * calling the hook.
 *
 *   sh harness/bench/hosttest/run.sh
 */
#include <stdio.h>
#include <string.h>

#include "platform.h"

static uint8_t  mem[0x10000];           /* the part's register space */
static uint32_t now;                    /* milliseconds */
static int      transfers, waits, fail_at = -1;
static uint16_t first_reg[512];
static uint16_t lengths[512];
static int      checks, failures;

#define CHECK(cond, ...) do { checks++; if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); \
                                            printf(__VA_ARGS__); printf("\n"); } } while (0)

static int32_t bus_write(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  (void)DevAddr;
  if (transfers == fail_at) { transfers++; return -1; }
  if (transfers < 512) { first_reg[transfers] = Reg; lengths[transfers] = Length; }
  transfers++;
  memcpy(&mem[Reg], pData, Length);
  return 0;
}

static int32_t bus_read(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  (void)DevAddr;
  if (transfers == fail_at) { transfers++; return -1; }
  if (transfers < 512) { first_reg[transfers] = Reg; lengths[transfers] = Length; }
  transfers++;
  memcpy(pData, &mem[Reg], Length);
  return 0;
}

static uint32_t tick(void) { return now; }

static uint32_t tick_self(void) { return now++; }    /* time passes as it is read */

static void wait_hook(void) { waits++; now++; }       /* each call lets 1 ms pass */

static VL53L5CX_Platform platform = { 0x52U, bus_write, bus_read, tick, wait_hook };

static void reset(void) { transfers = 0; waits = 0; fail_at = -1; }

int main(void)
{
  static uint8_t data[0x8000], back[0x8000];

  for (uint32_t i = 0U; i < sizeof data; i++) { data[i] = (uint8_t)((i * 7U) ^ (i >> 8)); }

  /* A 32 KB firmware bank: 128 chunks of 256 bytes, the hook between them only. */
  reset();
  CHECK(WrMulti(&platform, 0x0000U, data, 0x8000U) == 0U, "WrMulti 32 KB failed");
  CHECK(transfers == 128, "%d transfers for 32 KB, 128 expected", transfers);
  CHECK(waits == 127, "%d waits for 128 chunks, 127 expected", waits);
  for (int i = 0; i < 128; i++)
  {
    CHECK((first_reg[i] == (uint16_t)(i * 256)) && (lengths[i] == 256U), "chunk %d at 0x%04x, %u bytes",
          i, (unsigned)first_reg[i], (unsigned)lengths[i]);
  }
  CHECK(memcmp(mem, data, 0x8000U) == 0, "the bank did not land intact");

  /* The 972-byte default configuration ends at the command register (0x2FFF): the
   * last chunk, and so the last byte written, is the trigger. */
  reset();
  CHECK(WrMulti(&platform, 0x2C34U, data, 972U) == 0U, "WrMulti 972 failed");
  CHECK(transfers == 4, "%d transfers for 972 bytes, 4 expected", transfers);
  CHECK((first_reg[3] == 0x2F34U) && (lengths[3] == 204U), "last chunk at 0x%04x, %u bytes",
        (unsigned)first_reg[3], (unsigned)lengths[3]);
  CHECK((uint32_t)first_reg[3] + lengths[3] == 0x3000U, "the configuration does not end at 0x2FFF");

  /* An 8 x 8 result (320 bytes) reads in two pieces; single bytes in one, no wait. */
  reset();
  memset(back, 0, sizeof back);
  CHECK(RdMulti(&platform, 0x0000U, back, 320U) == 0U, "RdMulti 320 failed");
  CHECK((transfers == 2) && (lengths[0] == 256U) && (lengths[1] == 64U) && (first_reg[1] == 256U),
        "320-byte read: %d transfers, %u then %u bytes at 0x%04x", transfers, (unsigned)lengths[0],
        (unsigned)lengths[1], (unsigned)first_reg[1]);
  CHECK(waits == 1, "%d waits for two chunks, 1 expected", waits);
  CHECK(memcmp(back, mem, 320U) == 0, "the result did not read back intact");
  reset();
  {
    uint8_t b = 0U;
    CHECK((WrByte(&platform, 0x7FFFU, 0x02U) == 0U) && (RdByte(&platform, 0x7FFFU, &b) == 0U) && (b == 0x02U),
          "byte write and read back");
    CHECK((transfers == 2) && (waits == 0), "%d transfers and %d waits for two bytes", transfers, waits);
  }

  /* A failed chunk fails the call and nothing after it is sent. */
  reset();
  fail_at = 3;
  CHECK(WrMulti(&platform, 0x0000U, data, 0x8000U) != 0U, "a failed chunk was not reported");
  CHECK(transfers == 4, "%d transfers after a failure at the fourth", transfers);
  reset();
  fail_at = 0;
  CHECK(RdMulti(&platform, 0x0000U, back, 4U) != 0U, "a failed read was not reported");

  /* 32-bit words arrive most significant byte first. */
  {
    uint8_t w[8] = { 0x01, 0x02, 0x03, 0x04, 0xA1, 0xB2, 0xC3, 0xD4 };
    const uint8_t want[8] = { 0x04, 0x03, 0x02, 0x01, 0xD4, 0xC3, 0xB2, 0xA1 };
    SwapBuffer(w, 8U);
    CHECK(memcmp(w, want, 8U) == 0, "SwapBuffer");
  }

  /* A wait lasts at least TimeMs and serves the hook; with no hook it still ends. */
  reset();
  now = 1000U;
  CHECK(WaitMs(&platform, 10U) == 0U, "WaitMs");
  CHECK((now >= 1010U) && (now <= 1011U), "WaitMs(10) took %u ms", (unsigned)(now - 1000U));
  CHECK(waits >= 10, "%d hook calls in a 10 ms wait", waits);
  {
    VL53L5CX_Platform quiet = { 0x52U, bus_write, bus_read, tick_self, NULL };

    now = 0U;
    CHECK(WaitMs(&quiet, 5U) == 0U, "WaitMs without a hook");
    CHECK(now >= 6U, "WaitMs(5) without a hook ended after %u ms", (unsigned)now);
    reset();
    CHECK(WrMulti(&quiet, 0x0000U, data, 600U) == 0U, "WrMulti without a hook");
    CHECK(transfers == 3, "%d transfers for 600 bytes without a hook, 3 expected", transfers);
  }

  printf("test_vl53l5cx_platform: %d checks, %d failed\n", checks, failures);
  return (failures == 0) ? 0 : 1;
}
