/* Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 *
 * Host test of the VL53L1CB component (Components/vl53l1cb) and the ranging
 * module (neuron6_ranging_sensor.c) against a simulated part on the sensor
 * bus. The simulation keeps the registers the driver uses: the identity, the
 * boot flag after a software reset, timed ranging that produces a result a
 * timing budget after it starts or after its interrupt is cleared, the
 * interrupt status against the polarity in GPIO_HV_MUX_CTRL, and a result per
 * region of interest. Each region reads 1000 + its centre SPAD in millimetres,
 * so a stored distance says which region produced it. Its return scales with
 * its area and is a signal fail under the minimum count rate.
 *
 *   sh harness/bench/hosttest/run.sh
 */
#include <stdio.h>
#include <string.h>

#include "stm32n6xx_hal.h"
#include "neuron6_ranging_sensor.h"
#include "vl53l1cb_st_tables.h"

/* The simulated part. */

static uint32_t now;                /* milliseconds */
static uint8_t  reg[0x200];
static int      present = 1;        /* answers on the bus */
static uint16_t model_id = 0xEACC;
static int      boots = 1;          /* sets its boot flag after a reset */
static int      ranges = 1;         /* produces results while ranging */
static uint32_t boot_at;            /* the boot flag is set from here */
static int      ranging;            /* SYSTEM__MODE_START is 0x40 */
static uint32_t result_at;          /* the next result is ready from here */
static int      result_ready;
static int      bad_centre = -1;    /* a region centred here reads signal fail */
static int      dead_centre = -1;   /* a region centred here never finishes */
static int      fail_reg = -1;      /* a write to this register is not acknowledged */
static unsigned bus_inits, bus_deinits;

/* What was written, in order: register, length, first byte. */
#define LOG_MAX 4096
static struct { uint16_t reg, len; uint8_t v; } wlog[LOG_MAX];
static int nlog;
static uint8_t config_seen[91];
static int config_writes;

/* The return from a target at 0.75 m: 5.6 MCPS over the whole array, scaled by
 * the region's area, so 1.4 MCPS for 8 x 8 and 0.34 for 4 x 4. Under the
 * minimum in RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS (9.7) it is too weak. */
static int weak(void)
{
  const uint32_t cells = (uint32_t)((reg[0x80] & 0x0FU) + 1U) * (uint32_t)((reg[0x80] >> 4) + 1U);
  const uint32_t rate  = ((5600U * 128U) / 1000U) * cells / 256U;
  return rate < (((uint32_t)reg[0x66] << 8) | reg[0x67]);
}

static void sim_update(void)
{
  if (ranging && !result_ready && (now >= result_at) && ranges &&
      ((int)reg[0x7F] != dead_centre))
  {
    result_ready = 1;
    const uint16_t mm = (uint16_t)(1000U + reg[0x7F]);
    reg[0x89] = (((int)reg[0x7F] == bad_centre) || weak()) ? 0xE4 : 0xE9;   /* upper bits set: masked */
    reg[0x96] = (uint8_t)(mm >> 8);
    reg[0x97] = (uint8_t)mm;
  }
  if ((reg[0xE5] & 1U) == 0U && boots && (now >= boot_at))
  {
    reg[0xE5] |= 1U;
  }
  const uint8_t active = (reg[0x30] & 0x10U) ? 0U : 1U;
  reg[0x31] = (uint8_t)((reg[0x31] & ~1U) | (result_ready ? active : (uint8_t)(active ^ 1U)));
}

static void sim_reset(void)
{
  memset(reg, 0, sizeof(reg));
  reg[0x10F] = (uint8_t)(model_id >> 8);
  reg[0x110] = (uint8_t)model_id;
  reg[0x30] = 0x01;     /* active high */
  reg[0xE5] = 0x01;     /* booted */
  ranging = 0;
  result_ready = 0;
  nlog = 0;
  config_writes = 0;
}

static void sim_write(uint16_t r, const uint8_t *p, uint16_t n)
{
  if (nlog < LOG_MAX) { wlog[nlog].reg = r; wlog[nlog].len = n; wlog[nlog].v = p[0]; nlog++; }
  if (r == 0x002D && n == 91) { memcpy(config_seen, p, 91); config_writes++; }
  for (uint16_t i = 0; i < n; i++)
  {
    const uint16_t a = (uint16_t)(r + i);
    if (a < sizeof(reg)) reg[a] = p[i];
  }
  if (r == 0x0000 && n == 1)
  {
    if (p[0] == 0x00) { reg[0xE5] = 0; ranging = 0; result_ready = 0; }
    else { boot_at = now + 5U; }
  }
  if (r == 0x0086 && p[0] == 0x01)
  {
    result_ready = 0;
    if (ranging) result_at = now + VL53L1CB_TIMING_BUDGET_MS;
  }
  if (r == 0x0087)
  {
    if (p[0] == 0x40) { ranging = 1; result_ready = 0; result_at = now + VL53L1CB_TIMING_BUDGET_MS; }
    else { ranging = 0; result_ready = 0; }
  }
  sim_update();
}

/* What the module calls. */

void HAL_Delay(uint32_t Delay) { now += Delay; sim_update(); }
int32_t BSP_GetTick(void) { return (int32_t)now; }
int32_t BSP_I2C1_Init(void) { bus_inits++; return 0; }
int32_t BSP_I2C1_DeInit(void) { bus_deinits++; return 0; }

int32_t BSP_I2C1_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  if (!present || DevAddr != 0x52U || (int)Reg == fail_reg) return BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
  sim_write(Reg, pData, Length);
  return BSP_ERROR_NONE;
}

int32_t BSP_I2C1_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  if (!present || DevAddr != 0x52U) return BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
  sim_update();
  for (uint16_t i = 0; i < Length; i++)
  {
    const uint16_t a = (uint16_t)(Reg + i);
    pData[i] = (a < sizeof(reg)) ? reg[a] : 0U;
  }
  return BSP_ERROR_NONE;
}

/* Checks. */

static int checks, failures;
#define CHECK(c) do { checks++; if (!(c)) { failures++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

static int logged(uint16_t r, uint8_t v)          /* index of the first write of v to r */
{
  for (int i = 0; i < nlog; i++) if (wlog[i].reg == r && wlog[i].len == 1 && wlog[i].v == v) return i;
  return -1;
}

static void fresh(void)
{
  present = 1; model_id = 0xEACC; boots = 1; ranges = 1; bad_centre = -1; dead_centre = -1;
  fail_reg = -1;
  sim_reset();
  if (BSP_RANGING_SENSOR_DeInit(RANGING_SENSOR_VL53L1CB_0) == BSP_ERROR_NONE) { }
  bus_inits = bus_deinits = 0;
  nlog = 0;
}

/* The SPAD numbering built a second way: the part's map, column by column,
 * the top half counting up from 128 and the bottom half down from 127. */
static void spad_map(uint8_t map[16][16])
{
  for (int col = 0; col < 16; col++)
  {
    for (int i = 0; i < 8; i++) map[15 - i][col] = (uint8_t)(128 + 8 * col + i);
    for (int i = 0; i < 8; i++) map[7 - i][col] = (uint8_t)(127 - 8 * col - i);
  }
}

int main(void)
{
  VL53L1CB_Object_t obj;
  VL53L1CB_IO_t io = { BSP_I2C1_Init, BSP_I2C1_DeInit, VL53L1CB_I2C_BUS, 0x52U,
                       BSP_I2C1_WriteReg16, BSP_I2C1_ReadReg16, BSP_GetTick, HAL_Delay };
  RANGING_SENSOR_Result_t res;
  RANGING_SENSOR_ROIConfig_t roi;
  RANGING_SENSOR_Grid_t grid;
  uint32_t id = 0;
  uint8_t ready = 0;

  /* The component. */

  /* SPAD numbering against the part's map, every position. */
  {
    uint8_t map[16][16];
    int same = 1;
    spad_map(map);
    for (int r = 0; r < 16; r++) for (int c = 0; c < 16; c++) if (VL53L1CB_SpadNumber((uint8_t)r, (uint8_t)c) != map[r][c]) same = 0;
    CHECK(same);
    CHECK(VL53L1CB_SpadNumber(8, 8) == VL53L1CB_CENTRE_SPAD);
    CHECK(VL53L1CB_SpadNumber(16, 0) == 0 && VL53L1CB_SpadNumber(0, 16) == 0);
  }

  /* Init on a working part: the order of what it writes. */
  fresh();
  CHECK(VL53L1CB_RegisterBusIO(&obj, &io) == VL53L1CB_OK && bus_inits == 1);
  CHECK(VL53L1CB_Init(&obj) == VL53L1CB_OK && obj.IsInitialized == 1);
  {
    const int r0 = logged(0x0000, 0x00), r1 = logged(0x0000, 0x01);
    const int start = logged(0x0087, 0x40), stop = logged(0x0087, 0x00);
    const int vhv1 = logged(0x0008, 0x09), vhv2 = logged(0x000B, 0x00);
    int cfg = -1;
    for (int i = 0; i < nlog; i++) if (wlog[i].reg == 0x002D && wlog[i].len == 91) cfg = i;
    CHECK(r0 >= 0 && r1 > r0);
    CHECK(cfg > r1 && config_writes == 1 && memcmp(config_seen, vl53l1_st_default_config, 91) == 0);
    CHECK(start > cfg && stop > start && vhv1 > stop && vhv2 > vhv1);
    CHECK(reg[0x87] == 0x00);             /* left stopped */
    int sigma = -1, rate = -1;
    for (int i = 0; i < nlog; i++)
    {
      if (wlog[i].reg == 0x0064 && wlog[i].len == 2) sigma = i;
      if (wlog[i].reg == 0x0066 && wlog[i].len == 2) rate = i;
    }
    CHECK(sigma > vhv2 && rate > vhv2);   /* the limits last, over the configuration's */
    CHECK(reg[0x64] == 0x00 && reg[0x65] == 0x3C && reg[0x66] == 0x00 && reg[0x67] == 0x10);   /* 15 mm, 0.125 MCPS */
  }

  /* A limit the part does not take: Init fails rather than range with the
   * configuration's. */
  fresh(); fail_reg = 0x0064;
  (void)VL53L1CB_RegisterBusIO(&obj, &io);
  CHECK(VL53L1CB_Init(&obj) == VL53L1CB_ERROR && obj.IsInitialized == 0);
  fresh(); fail_reg = 0x0066;
  (void)VL53L1CB_RegisterBusIO(&obj, &io);
  CHECK(VL53L1CB_Init(&obj) == VL53L1CB_ERROR && obj.IsInitialized == 0);

  /* A part that is not a VL53L1CB, one that never boots, one that never ranges. */
  fresh(); model_id = 0xEAAA; sim_reset();
  CHECK(VL53L1CB_RegisterBusIO(&obj, &io) == VL53L1CB_OK);
  CHECK(VL53L1CB_Init(&obj) == VL53L1CB_ERROR && nlog == 0);
  fresh(); boots = 0;
  (void)VL53L1CB_RegisterBusIO(&obj, &io);
  CHECK(VL53L1CB_Init(&obj) == VL53L1CB_TIMEOUT && config_writes == 0);
  fresh(); ranges = 0;
  (void)VL53L1CB_RegisterBusIO(&obj, &io);
  CHECK(VL53L1CB_Init(&obj) == VL53L1CB_TIMEOUT && reg[0x87] == 0x00 && obj.IsInitialized == 0);

  /* The region registers, and the sizes the part refuses. */
  fresh();
  (void)VL53L1CB_RegisterBusIO(&obj, &io);
  CHECK(VL53L1CB_Init(&obj) == VL53L1CB_OK);
  CHECK(VL53L1CB_SetROI(&obj, 4, 4, 145) == VL53L1CB_OK && reg[0x7F] == 145 && reg[0x80] == 0x33);
  CHECK(VL53L1CB_SetROI(&obj, 16, 16, 199) == VL53L1CB_OK && reg[0x80] == 0xFF);
  CHECK(VL53L1CB_SetROI(&obj, 6, 4, 100) == VL53L1CB_OK && reg[0x80] == 0x35);   /* height in the high nibble */
  CHECK(VL53L1CB_SetROI(&obj, 3, 4, 100) == VL53L1CB_ERROR && VL53L1CB_SetROI(&obj, 4, 17, 100) == VL53L1CB_ERROR);

  /* A 4 x 4 region ranges within the minimum Init sets, not within the
   * configuration's; 8 x 8 and the whole array range within either. */
  {
    static const struct { uint8_t side, centre, min; uint8_t want; } cases[] = {
      { 4, 145, 0x10, VL53L1CB_RANGE_VALID }, { 4, 145, 0x80, VL53L1CB_RANGE_SIGNAL_FAIL },
      { 8, 199, 0x80, VL53L1CB_RANGE_VALID }, { 16, 199, 0x80, VL53L1CB_RANGE_VALID } };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
    {
      uint16_t mm = 0; uint8_t st = 0;
      reg[0x66] = 0x00; reg[0x67] = cases[i].min;
      CHECK(VL53L1CB_SetROI(&obj, cases[i].side, cases[i].side, cases[i].centre) == VL53L1CB_OK &&
            VL53L1CB_StartRanging(&obj) == VL53L1CB_OK);
      now += VL53L1CB_TIMING_BUDGET_MS;
      CHECK(VL53L1CB_ReadResult(&obj, &mm, &st) == VL53L1CB_OK && st == cases[i].want);
      (void)VL53L1CB_StopRanging(&obj);
    }
    reg[0x67] = 0x10;
  }

  /* Data ready against both polarities. */
  {
    reg[0x30] = 0x01; reg[0x31] = 0x01; result_ready = 1;
    CHECK(VL53L1CB_DataReady(&obj, &ready) == VL53L1CB_OK && ready == 1);
    result_ready = 0;
    CHECK(VL53L1CB_DataReady(&obj, &ready) == VL53L1CB_OK && ready == 0);
    reg[0x30] = 0x11; result_ready = 1;     /* active low */
    CHECK(VL53L1CB_DataReady(&obj, &ready) == VL53L1CB_OK && ready == 1 && (reg[0x31] & 1U) == 0U);
    result_ready = 0;
    CHECK(VL53L1CB_DataReady(&obj, &ready) == VL53L1CB_OK && ready == 0);
    reg[0x30] = 0x01;
  }

  /* A result: the status masked to five bits, the distance most significant byte first. */
  {
    uint16_t mm = 0; uint8_t st = 0;
    reg[0x89] = 0xE9; reg[0x96] = 0x01; reg[0x97] = 0xF4; nlog = 0;
    CHECK(VL53L1CB_ReadResult(&obj, &mm, &st) == VL53L1CB_OK && mm == 500 && st == VL53L1CB_RANGE_VALID);
    CHECK(logged(0x0086, 0x01) >= 0);
  }

  /* The ranging module. */

  fresh(); present = 0;
  CHECK(BSP_RANGING_SENSOR_Init(RANGING_SENSOR_VL53L1CB_0) == BSP_ERROR_UNKNOWN_COMPONENT);
  CHECK(bus_inits == 1 && bus_deinits == 1);             /* the bus given back */
  fresh(); model_id = 0x1234; sim_reset();
  CHECK(BSP_RANGING_SENSOR_Init(RANGING_SENSOR_VL53L1CB_0) == BSP_ERROR_UNKNOWN_COMPONENT);
  fresh(); ranges = 0;
  CHECK(BSP_RANGING_SENSOR_Init(RANGING_SENSOR_VL53L1CB_0) == BSP_ERROR_COMPONENT_FAILURE);
  CHECK(bus_inits == bus_deinits);
  CHECK(BSP_RANGING_SENSOR_Init(1U) == BSP_ERROR_WRONG_PARAM);
  CHECK(BSP_RANGING_SENSOR_ReadID(RANGING_SENSOR_VL53L1CB_0, &id) == BSP_ERROR_NO_INIT);

  fresh();
  CHECK(BSP_RANGING_SENSOR_Init(RANGING_SENSOR_VL53L1CB_0) == BSP_ERROR_NONE);
  CHECK(BSP_RANGING_SENSOR_ReadID(RANGING_SENSOR_VL53L1CB_0, &id) == BSP_ERROR_NONE && id == RANGING_SENSOR_ID);
  CHECK(reg[0x7F] == 199 && reg[0x80] == 0xFF);           /* the configuration's region: the whole array */

  /* Blocking one-shot: waits about one budget, then the part is stopped. */
  {
    const uint32_t t0 = now;
    CHECK(BSP_RANGING_SENSOR_Start(RANGING_SENSOR_VL53L1CB_0, RANGING_SENSOR_MODE_BLOCKING_ONESHOT) == BSP_ERROR_NONE);
    CHECK(BSP_RANGING_SENSOR_GetDistance(RANGING_SENSOR_VL53L1CB_0, &res) == BSP_ERROR_NONE);
    CHECK(res.NumberOfZones == 1 && res.ZoneResult[0].NumberOfTargets == 1 &&
          res.ZoneResult[0].Status[0] == 0 && res.ZoneResult[0].Distance[0] == 1199);
    CHECK((now - t0) >= VL53L1CB_TIMING_BUDGET_MS && (now - t0) <= VL53L1CB_TIMING_BUDGET_MS + 5U);
    CHECK(reg[0x87] == 0x00);
    CHECK(BSP_RANGING_SENSOR_Stop(RANGING_SENSOR_VL53L1CB_0) == BSP_ERROR_NO_INIT);
  }

  /* Asynchronous continuous: busy until a result, then one a budget. */
  CHECK(BSP_RANGING_SENSOR_Start(RANGING_SENSOR_VL53L1CB_0, RANGING_SENSOR_MODE_ASYNC_CONTINUOUS) == BSP_ERROR_NONE);
  CHECK(BSP_RANGING_SENSOR_Start(RANGING_SENSOR_VL53L1CB_0, RANGING_SENSOR_MODE_ASYNC_CONTINUOUS) == BSP_ERROR_BUSY);
  CHECK(BSP_RANGING_SENSOR_IsMeasurementReady(RANGING_SENSOR_VL53L1CB_0, &ready) == BSP_ERROR_NONE && ready == 0);
  CHECK(BSP_RANGING_SENSOR_GetDistance(RANGING_SENSOR_VL53L1CB_0, &res) == BSP_ERROR_BUSY);
  now += VL53L1CB_TIMING_BUDGET_MS;
  CHECK(BSP_RANGING_SENSOR_IsMeasurementReady(RANGING_SENSOR_VL53L1CB_0, &ready) == BSP_ERROR_NONE && ready == 1);
  CHECK(BSP_RANGING_SENSOR_GetDistance(RANGING_SENSOR_VL53L1CB_0, &res) == BSP_ERROR_NONE && res.ZoneResult[0].Distance[0] == 1199);
  CHECK(BSP_RANGING_SENSOR_GetDistance(RANGING_SENSOR_VL53L1CB_0, &res) == BSP_ERROR_BUSY);   /* cleared */
  now += VL53L1CB_TIMING_BUDGET_MS;
  CHECK(BSP_RANGING_SENSOR_GetDistance(RANGING_SENSOR_VL53L1CB_0, &res) == BSP_ERROR_NONE && reg[0x87] == 0x40);

  /* The region cannot move while ranging; a bad range is no target. */
  roi.TopLeftX = 0; roi.TopLeftY = 15; roi.BotRightX = 3; roi.BotRightY = 12;
  CHECK(BSP_RANGING_SENSOR_ConfigROI(RANGING_SENSOR_VL53L1CB_0, &roi) == BSP_ERROR_BUSY);
  CHECK(BSP_RANGING_SENSOR_Stop(RANGING_SENSOR_VL53L1CB_0) == BSP_ERROR_NONE && reg[0x87] == 0x00);
  CHECK(BSP_RANGING_SENSOR_ConfigROI(RANGING_SENSOR_VL53L1CB_0, &roi) == BSP_ERROR_NONE && reg[0x7F] == 145 && reg[0x80] == 0x33);
  bad_centre = 145;
  CHECK(BSP_RANGING_SENSOR_Start(RANGING_SENSOR_VL53L1CB_0, RANGING_SENSOR_MODE_BLOCKING_CONTINUOUS) == BSP_ERROR_NONE);
  CHECK(BSP_RANGING_SENSOR_GetDistance(RANGING_SENSOR_VL53L1CB_0, &res) == BSP_ERROR_NONE &&
        res.ZoneResult[0].NumberOfTargets == 0 && res.ZoneResult[0].Status[0] == VL53L1CB_RANGE_SIGNAL_FAIL);
  CHECK(BSP_RANGING_SENSOR_Stop(RANGING_SENSOR_VL53L1CB_0) == BSP_ERROR_NONE);
  bad_centre = -1;
  roi.BotRightX = 2;                                         /* three wide */
  CHECK(BSP_RANGING_SENSOR_ConfigROI(RANGING_SENSOR_VL53L1CB_0, &roi) == BSP_ERROR_WRONG_PARAM);
  roi.TopLeftX = 5; roi.BotRightX = 4;                       /* inside out */
  CHECK(BSP_RANGING_SENSOR_ConfigROI(RANGING_SENSOR_VL53L1CB_0, &roi) == BSP_ERROR_WRONG_PARAM);
  roi.TopLeftX = 0; roi.TopLeftY = 15; roi.BotRightX = 15; roi.BotRightY = 0;
  CHECK(BSP_RANGING_SENSOR_ConfigROI(RANGING_SENSOR_VL53L1CB_0, &roi) == BSP_ERROR_NONE && reg[0x7F] == 199 && reg[0x80] == 0xFF);

  /* The grid: zone order and regions, a weak zone and a zone that never
   * finishes stored as 0, the sweep carrying on, and the whole array put
   * back at the end. */
  {
    static const uint8_t centres[16] = { 145, 177, 209, 241, 149, 181, 213, 245,
                                         110, 78, 46, 14, 106, 74, 42, 10 };
    int order_ok = 1, steps = 0, failures_seen = 0;
    bad_centre = 213;                                        /* zone (1, 2) */
    dead_centre = 46;                                        /* zone (2, 2) */
    CHECK(BSP_RANGING_SENSOR_StartGrid(RANGING_SENSOR_VL53L1CB_0) == BSP_ERROR_NONE);
    CHECK(BSP_RANGING_SENSOR_Start(RANGING_SENSOR_VL53L1CB_0, RANGING_SENSOR_MODE_ASYNC_CONTINUOUS) == BSP_ERROR_BUSY);
    CHECK(BSP_RANGING_SENSOR_GetGrid(RANGING_SENSOR_VL53L1CB_0, &grid) == BSP_ERROR_NONE && grid.SweepCount == 0);
    uint32_t zone_seen = 99;
    while (steps < 10000)
    {
      if (BSP_RANGING_SENSOR_GetGrid(RANGING_SENSOR_VL53L1CB_0, &grid) != BSP_ERROR_NONE) break;
      if (grid.SweepCount == 1) break;
      const uint32_t zone = grid.ZoneIndex;
      if (BSP_RANGING_SENSOR_StepGrid(RANGING_SENSOR_VL53L1CB_0) != BSP_ERROR_NONE) failures_seen++;
      if (zone != zone_seen && ranging) { if (reg[0x7F] != centres[zone] || reg[0x80] != 0x33) order_ok = 0; zone_seen = zone; }
      now += 10;
      steps++;
    }
    CHECK(order_ok && grid.SweepCount == 1 && grid.ZoneIndex == 0);
    CHECK(failures_seen == 1);                               /* the zone that never finished */
    for (int z = 0; z < 16; z++)
    {
      const uint16_t want = (centres[z] == 213 || centres[z] == 46) ? 0 : (uint16_t)(1000 + centres[z]);
      CHECK(grid.Distance[z / 4][z % 4] == want);
    }
    CHECK(steps < 16 * 40);                                  /* no zone waited past its time */
    CHECK(BSP_RANGING_SENSOR_StopGrid(RANGING_SENSOR_VL53L1CB_0) == BSP_ERROR_NONE);
    CHECK(reg[0x87] == 0x00 && reg[0x7F] == 199 && reg[0x80] == 0xFF);
    CHECK(BSP_RANGING_SENSOR_StepGrid(RANGING_SENSOR_VL53L1CB_0) == BSP_ERROR_NO_INIT);
    bad_centre = dead_centre = -1;
  }

  /* Init again while a sweep runs: the part reset, the bus not taken twice. */
  {
    const unsigned inits = bus_inits;
    CHECK(BSP_RANGING_SENSOR_StartGrid(RANGING_SENSOR_VL53L1CB_0) == BSP_ERROR_NONE);
    CHECK(BSP_RANGING_SENSOR_StepGrid(RANGING_SENSOR_VL53L1CB_0) == BSP_ERROR_NONE && ranging);
    nlog = 0;
    CHECK(BSP_RANGING_SENSOR_Init(RANGING_SENSOR_VL53L1CB_0) == BSP_ERROR_NONE);
    CHECK(bus_inits == inits && logged(0x0000, 0x00) >= 0 && reg[0x87] == 0x00);
    CHECK(BSP_RANGING_SENSOR_GetGrid(RANGING_SENSOR_VL53L1CB_0, &grid) == BSP_ERROR_NO_INIT);
    CHECK(BSP_RANGING_SENSOR_DeInit(RANGING_SENSOR_VL53L1CB_0) == BSP_ERROR_NONE && bus_deinits >= 1);
    CHECK(BSP_RANGING_SENSOR_DeInit(RANGING_SENSOR_VL53L1CB_0) == BSP_ERROR_NO_INIT);
  }

  printf("test_ranging: %d checks, %d failed\n", checks, failures);
  return failures ? 1 : 0;
}
