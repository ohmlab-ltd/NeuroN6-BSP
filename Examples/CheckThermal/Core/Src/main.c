/*
  Check thermal

  Checks the FLIR Lepton thermal camera a step at a time and says where it
  stops: power and reset lines, its answer on the sensor bus, boot, status
  register, a command, what it is, then its video. It times the first frame,
  draws a frame in characters every five seconds, and, if the camera does
  not answer, lists what the sensor bus does answer at.

  The circuit:
  - A Neuro N6 on the thermal carrier, with the Lepton 3.1R fitted.
  - PD3, the camera's power line, is the BLE flow control on other
    carriers. Nothing here uses BLE.

  Send 'b' on the USB console at any time to return to the bootloader.

  Copyright (c) 2026 Ohm Lab Ltd.
  SPDX-License-Identifier: CC0-1.0
  Released under CC0 1.0 Universal: copy it into your own project
  freely, with or without attribution.
*/

#include "main.h"
#include "neuron6.h"
#include "neuron6_com.h"
#include "neuron6_wdg.h"
#include "neuron6_thermal.h"
#include "neuron6_bus.h"

/* Provided by the scaffolding at the end of this file. */
static inline void     delay(uint32_t ms);
static inline uint32_t millis(void);
static void            setup(void);
static void            loop(void);

/* Formatted output to the USB console. Dropped, not blocked, with no host. */
#define print(...)   (void)BSP_COM_Printf(__VA_ARGS__)

/* Video, written only by DMA, sized as in ReadThermal. */
#define VIDEO_BYTES  (49U * 8U * BSP_THERMAL_PACKET_BYTES)   /* 64288 */

static uint8_t  video[VIDEO_BYTES] __attribute__((aligned(32)));
static uint16_t frame[BSP_THERMAL_FRAME_PIXELS];

static uint8_t  checked     = 0U;  /* 1 once a check has run          */
static uint8_t  streaming   = 0U;  /* 1 once the video has started    */
static uint8_t  cycled      = 0U;  /* 1 once step 8 has run           */
static uint32_t lastCheck   = 0U;  /* when the last check began       */
static uint32_t videoAt     = 0U;  /* when the video (re)started      */
static const char *startedBy = "the video started";
static uint32_t lastFrame   = 0U;
static uint32_t lastPrint   = 0U;
static uint32_t lastPicture = 0U;
static uint32_t frames      = 0U;  /* since the last print            */
static uint32_t total       = 0U;  /* since the video started         */

/* The latest frame's figures, taken on arrival: the next frame is assembled
 * in the same buffer. */
static uint16_t coldest, hottest, centre, mean;
static char     picture[20][41];   /* 40 x 20 characters */

/* The thermal module installs no interrupt vectors: these route its video
 * DMA (GPDMA1 channel 1) and its link-error interrupt. */
void GPDMA1_Channel1_IRQHandler(void) { BSP_THERMAL_DMA_IRQHandler(0); }
void SPI1_IRQHandler(void)            { BSP_THERMAL_SPI_IRQHandler(0); }

/* Hundredths of a kelvin as degrees Celsius, to two places: the default C
 * library has no floating-point printf. */
static void printCelsius(const char *label, uint16_t centikelvin)
{
  int32_t  c = (int32_t)centikelvin - 27315;
  uint32_t a = (uint32_t)((c < 0) ? -c : c);

  print("%s %s%lu.%02lu C", label, (c < 0) ? "-" : "",
        (unsigned long)(a / 100U), (unsigned long)(a % 100U));
}

/* Everything on the sensor bus that acknowledges. An empty address is
 * refused in well under a millisecond; a probe that takes the whole bus
 * timeout means the bus itself is stuck, so the scan stops there. */
static void scanBus(void)
{
  static const struct { uint16_t address; const char *name; } known[] = {
    { 0x10U, "VD66GY or VD65G4" }, { 0x2AU, "Lepton" }, { 0x30U, "MMC5603NJ" },
    { 0x3CU, "OV5640" }, { 0x6AU, "LSM6DSL" }, { 0x6BU, "LSM6DSL" } };
  uint32_t found = 0U;

  print("    the sensor bus answers at:");
  for (uint16_t a = 0x08U; a <= 0x77U; a++)
  {
    const uint32_t t0 = millis();

    if (BSP_I2C1_IsReady((uint16_t)(a << 1), 1U) == BSP_ERROR_NONE)
    {
      const char *name = "";

      for (size_t i = 0U; i < (sizeof(known) / sizeof(known[0])); i++)
      {
        if (known[i].address == a) { name = known[i].name; }
      }
      print(" 0x%02X%s%s", (unsigned)a, (name[0] != '\0') ? " " : "", name);
      found++;
    }
    else if ((millis() - t0) >= 100U)
    {
      break;
    }
    delay(1U);
  }
  print("%s\r\n", (found == 0U) ? " nothing: the bus itself, its pull-ups or its supply" : "");
}

/* The command interface a step at a time, then the video.
 * Returns 1 if the video started. */
static int check(void)
{
  uint8_t  powerHigh = 0U, resetHigh = 0U, gain = 0U;
  uint16_t status = 0U, sensor = 0U, housing = 0U;
  uint16_t serial[4] = {0U, 0U, 0U, 0U};
  uint32_t uptime = 0U;
  BSP_THERMAL_Health_t h = {0};
  int32_t  r;

  print("\r\n=== thermal camera check ===\r\n");

  /* 1. Power up: both lines read high once powered and out of reset; a line
   * that reads low was never driven. */
  (void)BSP_THERMAL_PowerOn(0);
  (void)BSP_THERMAL_GetControlPins(0, &powerHigh, &resetHigh);
  print("[1] power and reset ...... %s (power-down PD3 %s, reset PD9 %s)\r\n",
        ((powerHigh != 0U) && (resetHigh != 0U)) ? "OK" : "FAIL",
        (powerHigh != 0U) ? "high" : "low", (resetHigh != 0U) ? "high" : "low");

  /* 2. It boots only with the sensor bus quiet, so wait that long (USB still
   * serviced), then probe its address twice, which does it no harm. */
  delay(BSP_THERMAL_BOOT_QUIET_MS);
  (void)BSP_I2C1_Init();
  r = BSP_I2C1_IsReady(BSP_THERMAL_I2C_ADDRESS, 2U);
  delay(1U);
  print("[2] answers at 0x%02X ..... %s\r\n", (unsigned)(BSP_THERMAL_I2C_ADDRESS >> 1),
        (r == BSP_ERROR_NONE) ? "OK" : "FAIL (no acknowledge)");
  if (r != BSP_ERROR_NONE)
  {
    scanBus();
    (void)BSP_I2C1_DeInit();
    (void)BSP_THERMAL_PowerOff(0);
    return 0;
  }
  (void)BSP_I2C1_DeInit();

  /* 3. Boot: BSP_THERMAL_Init asks at once whether it has finished, and keeps
   * asking until BSP_THERMAL_BOOT_TIMEOUT_MS after the reset. */
  r = BSP_THERMAL_Init(0);
  print("[3] boot ................. %s\r\n",
        (r == BSP_ERROR_NONE) ? "OK" : "FAIL (it never reported a finished boot)");
  if (r != BSP_ERROR_NONE)
  {
    return 0;
  }

  /* 4. Status: busy, boot mode, boot finished, and the last command's result
   * (a signed byte). A busy camera holds each command up to two seconds, so a
   * delay(1) after each keeps the watchdog fed. */
  if (BSP_THERMAL_ReadRegister(0, LEPTON_REG_STATUS, &status) == BSP_ERROR_NONE)
  {
    print("[4] status ............... 0x%04X: busy %u, boot mode %u, booted %u, result %d\r\n",
          (unsigned)status, (unsigned)(status & LEPTON_STATUS_BUSY),
          (unsigned)((status & LEPTON_STATUS_BOOT_MODE) != 0U),
          (unsigned)((status & LEPTON_STATUS_BOOT_STATUS) != 0U),
          (int)(int8_t)(status >> 8));
  }
  else
  {
    print("[4] status ............... FAIL (no read)\r\n");
  }
  delay(1U);

  /* 5. A command. */
  r = BSP_THERMAL_Ping(0);
  print("[5] ping ................. %s (result %ld)\r\n",
        (r == BSP_ERROR_NONE) ? "OK" : "FAIL", (long)BSP_THERMAL_GetLastResult(0));
  delay(1U);

  /* 6. What it is, and how it is set up. */
  if (BSP_THERMAL_GetSerialNumber(0, serial) == BSP_ERROR_NONE)
  {
    print("[6] serial %04X%04X%04X%04X", serial[3], serial[2], serial[1], serial[0]);
  }
  else
  {
    print("[6] serial unknown");
  }
  delay(1U);
  if (BSP_THERMAL_GetUptime(0, &uptime) == BSP_ERROR_NONE)
  {
    print(", up %lu ms", (unsigned long)uptime);
  }
  delay(1U);
  if (BSP_THERMAL_GetFpaTemperature(0, &sensor) == BSP_ERROR_NONE)
  {
    printCelsius(", sensor", sensor);
  }
  delay(1U);
  if (BSP_THERMAL_GetAuxTemperature(0, &housing) == BSP_ERROR_NONE)
  {
    printCelsius(", housing", housing);
  }
  delay(1U);
  if (BSP_THERMAL_GetAgcEnabled(0, &gain) == BSP_ERROR_NONE)
  {
    print(", automatic gain %s", (gain != 0U) ? "on: the pixels are not temperatures" : "off");
  }
  print("\r\n");
  delay(1U);

  /* 7. The video. The carrier mounts the camera upside down. */
  (void)BSP_THERMAL_SetFlip180(0, 1U);
  r = BSP_THERMAL_VideoStart(0, video, VIDEO_BYTES);
  (void)BSP_THERMAL_GetHealth(0, &h);
  print("[7] video ................ %s (link %lu Hz)\r\n",
        (r == BSP_ERROR_NONE) ? "started" : "FAIL", (unsigned long)h.SckHz);
  if (r != BSP_ERROR_NONE)
  {
    (void)BSP_THERMAL_DeInit(0);
    return 0;
  }
  return 1;
}

/* The frame's figures, and the frame in 40 x 20 characters, coldest to
 * hottest. */
static void takeFrame(void)
{
  static const char ramp[] = " .:-=+*#%@";
  uint32_t sum = 0U;
  uint32_t span;

  coldest = frame[0];
  hottest = frame[0];
  for (uint32_t i = 0U; i < BSP_THERMAL_FRAME_PIXELS; i++)
  {
    if (frame[i] < coldest) { coldest = frame[i]; }
    if (frame[i] > hottest) { hottest = frame[i]; }
    sum += frame[i];
  }
  mean   = (uint16_t)(sum / BSP_THERMAL_FRAME_PIXELS);
  centre = frame[(BSP_THERMAL_HEIGHT / 2U) * BSP_THERMAL_WIDTH + (BSP_THERMAL_WIDTH / 2U)];
  span   = (hottest > coldest) ? (uint32_t)(hottest - coldest) : 1U;

  for (uint32_t y = 0U; y < 20U; y++)
  {
    for (uint32_t x = 0U; x < 40U; x++)
    {
      const uint16_t v = frame[(y * (BSP_THERMAL_HEIGHT / 20U)) * BSP_THERMAL_WIDTH +
                               (x * (BSP_THERMAL_WIDTH / 40U))];

      picture[y][x] = ramp[((uint32_t)(v - coldest) * 9U) / span];
    }
    picture[y][40] = '\0';
  }
}

static void setup(void)
{
}

static void loop(void)
{
  uint32_t now = millis();

  if (streaming == 0U)
  {
    /* Check once a terminal opens the console, or ten seconds after the
     * start, so the first steps are seen; then every ten seconds until the
     * video runs. */
    if ((checked == 0U) && (BSP_COM_IsConnected() == 0) && (now < 10000U))
    {
      return;
    }
    if ((checked != 0U) && ((now - lastCheck) < 10000U))
    {
      return;
    }
    checked = 1U;
    lastCheck = now;
    if (check() == 0)
    {
      print("=== the check stopped there; again in ten seconds ===\r\n");
      return;
    }
    streaming = 1U;
    videoAt = millis();
    lastFrame = videoAt;
    lastPrint = videoAt;
    lastPicture = videoAt;
    return;
  }

  /* A short wait, so USB and the watchdog are serviced between calls. */
  if (BSP_THERMAL_GetFrame(0, frame, 50U) == BSP_ERROR_NONE)
  {
    takeFrame();
    if (total == 0U)
    {
      print("first frame %lu ms after %s\r\n",
            (unsigned long)(millis() - videoAt), startedBy);
    }
    total++;
    frames++;
    lastFrame = millis();
  }

  now = millis();
  if ((now - lastPrint) >= 1000U)
  {
    if (frames != 0U)
    {
      print("%lu frames/s ", (unsigned long)frames);
      printCelsius(" coldest", coldest);
      printCelsius("  mean", mean);
      printCelsius("  centre", centre);
      printCelsius("  hottest", hottest);
      print("\r\n");
    }
    frames = 0U;
    lastPrint = now;
  }

  /* Every five seconds, the latest frame drawn, and the link's health. */
  if ((now - lastPicture) >= 5000U)
  {
    BSP_THERMAL_Health_t h = {0};

    if (total != 0U)
    {
      for (uint32_t y = 0U; y < 20U; y++)
      {
        print("|%s|\r\n", picture[y]);
      }
    }
    (void)BSP_THERMAL_GetHealth(0, &h);
    print("link %lu Hz: halves %lu/%lu, packets %lu, fillers %lu, segments %lu, frames %lu,"
          " resyncs %lu, restarts %lu, overruns %lu, errors %lu\r\n",
          (unsigned long)h.SckHz, (unsigned long)h.HalvesConsumed,
          (unsigned long)h.HalvesProduced, (unsigned long)h.PacketsSeen,
          (unsigned long)h.FillersSeen, (unsigned long)h.SegmentsParsed,
          (unsigned long)h.FramesParsed, (unsigned long)h.Resyncs,
          (unsigned long)h.Restarts, (unsigned long)h.Overruns, (unsigned long)h.SpiErrors);
    lastPicture = now;
  }

  /* 8. Once, a minute in: the power cycle below, so the first frame is timed
   * again with the video restarting while the camera boots. */
  if ((cycled == 0U) && (total != 0U) && ((now - videoAt) >= 60000U))
  {
    print("[8] power cycle mid-stream: the video restarts while the camera boots\r\n");
    cycled = 1U;
    (void)BSP_THERMAL_PowerOn(0);
    total = 0U;
    startedBy = "the power cycle";
    videoAt = millis();
    lastFrame = videoAt;
    return;               /* now is older than lastFrame from here on */
  }

  /* No frame for twenty seconds, despite the module's restarts: power-cycle
   * the camera. The next read restarts the video while it boots. */
  if ((now - lastFrame) >= 20000U)
  {
    print("no frame for twenty seconds: power-cycling the camera\r\n");
    (void)BSP_THERMAL_PowerOn(0);
    total = 0U;
    startedBy = "the power cycle";
    videoAt = millis();
    lastFrame = videoAt;
  }
}

/* Board scaffolding, the same in every example. It keeps the board reachable
 * over USB: the watchdog is armed first, its early warning and its reset both
 * lead to the bootloader, and every wait services USB, refreshes the watchdog
 * and takes 'b' as a reboot into the bootloader. The fault handlers are in
 * stm32n6xx_it.c. */

#define WDG_TIMEOUT_MS   4000U   /* reset if delay() is not called for this long */
#define WDG_WARNING_MS    300U   /* the early warning fires this long before it  */

static void mpu_config(void);

/* One pass of the housekeeping: the USB stack, the watchdog, and 'b'. */
static void service(void)
{
  uint8_t ch;

  BSP_COM_Task();
  (void)BSP_WDG_Refresh();
  if ((BSP_COM_Read(&ch, 1U) == 1) && (ch == (uint8_t)'b'))
  {
    BSP_COM_RebootToBootloader();
  }
}

/* Wait, staying reachable. Inline, like millis(), so an example that uses
 * neither gets no unused-function warning. */
static inline void delay(uint32_t ms)
{
  uint32_t t0 = HAL_GetTick();

  do
  {
    service();
  } while ((HAL_GetTick() - t0) < ms);
}

/* Milliseconds since boot. */
static inline uint32_t millis(void)
{
  return HAL_GetTick();
}

/* The watchdog is about to reset the board: go to the bootloader instead. */
void BSP_WDG_EWI_Callback(void)
{
  BSP_COM_RebootToBootloader();
}

int main(void)
{
  SCB_DisableDCache();
  mpu_config();
  SCB_EnableICache();
  SCB_EnableDCache();
  HAL_Init();

  (void)BSP_LED_Init(LED_RED);

  /* Armed before anything that could fail. A watchdog reset goes to the
   * bootloader. */
  (void)BSP_WDG_InitEWI(WDG_TIMEOUT_MS, WDG_WARNING_MS);
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U)
  {
    __HAL_RCC_CLEAR_RESET_FLAGS();
    BSP_COM_RebootToBootloader();
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* USB last, immediately before the loop that services it. */
  if (BSP_COM_Init() != BSP_ERROR_NONE)
  {
    (void)BSP_LED_On(LED_RED);
    Error_Handler();
  }

  setup();
  for (;;)
  {
    service();
    loop();
  }
}

void Error_Handler(void)
{
  /* Interrupts off: the watchdog resets the board, and main sends that reset
   * to the bootloader. */
  __disable_irq();
  for (;;)
  {
  }
}

static void mpu_config(void)
{
  MPU_Region_InitTypeDef     default_config = {0};
  MPU_Attributes_InitTypeDef attr_config    = {0};
  uint32_t primask_bit = __get_PRIMASK();

  __disable_irq();
  HAL_MPU_Disable();

  attr_config.Attributes = INNER_OUTER(MPU_NOT_CACHEABLE);
  attr_config.Number     = MPU_ATTRIBUTES_NUMBER0;
  HAL_MPU_ConfigMemoryAttributes(&attr_config);

  default_config.Enable           = MPU_REGION_ENABLE;
  default_config.Number           = MPU_REGION_NUMBER0;
  default_config.BaseAddress      = __NON_CACHEABLE_SECTION_BEGIN;
  default_config.LimitAddress     = __NON_CACHEABLE_SECTION_END;
  default_config.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  default_config.AccessPermission = MPU_REGION_ALL_RW;
  default_config.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  default_config.AttributesIndex  = MPU_ATTRIBUTES_NUMBER0;
  HAL_MPU_ConfigRegion(&default_config);

  default_config.Number       = MPU_REGION_NUMBER1;
  default_config.BaseAddress  = 0x70000000;
  default_config.LimitAddress = 0x71FFFFFF;
  HAL_MPU_ConfigRegion(&default_config);

  extern uint8_t __axisram_nc_start[];
  extern uint8_t __axisram_nc_end[];
  if ((uint32_t)__axisram_nc_end > (uint32_t)__axisram_nc_start)
  {
    default_config.Number       = MPU_REGION_NUMBER2;
    default_config.BaseAddress  = (uint32_t)__axisram_nc_start;
    default_config.LimitAddress = (uint32_t)__axisram_nc_end - 1u;
    HAL_MPU_ConfigRegion(&default_config);
  }

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
  __set_PRIMASK(primask_bit);
}
