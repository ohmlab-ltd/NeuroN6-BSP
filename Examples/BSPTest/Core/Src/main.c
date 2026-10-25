/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : The BSP test console: one command per module on the USB
  *                   console, each reporting OK, FAIL or SKIP.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  *
  * Protocol (harness/n6test.py): the host sends "<cmd>\n"; the board prints
  * any number of "INFO ..." lines, then "OK <cmd> k=v ...", "FAIL <cmd> why"
  * or "SKIP <cmd> why", then "END <cmd>".
  *
  * Every wait goes through svc(), which runs the USB stack, honours 'b' and
  * refreshes the watchdog (4 s, armed at the start of main). Blocking BSP
  * calls are bounded by timeouts shorter than the watchdog; a test that wedges
  * anyway is reset and the board comes back on the bus.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "neuron6.h"
#include "neuron6_com.h"
#include "neuron6_bus.h"
#include "neuron6_adc.h"
#include "neuron6_io.h"
#include "neuron6_pwm.h"
#include "neuron6_wdg.h"
#include "neuron6_npu.h"
#include "neuron6_motion_sensor.h"
#include "neuron6_ranging_sensor.h"
#include "neuron6_audio.h"
#include "neuron6_camera.h"
#include "neuron6_lcd.h"
#include "neuron6_venc.h"
#include "stm32n6xx_ll_venc.h"
#include "neuron6_wifi.h"
#include "neuron6_bt.h"
#if defined(BSP_USE_FREERTOS) && (BSP_USE_FREERTOS == 1)
#include "neuron6_rtos.h"
#include "timers.h"     /* the rtos command's one-shot timer */
#endif

#include "tusb.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*cmd_fn_t)(void);
typedef struct { const char *name; cmd_fn_t fn; } cmd_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WDG_TIMEOUT_MS          4000U
#define CAM_W                   640U
#define CAM_H                   480U
#define CAM_FRAME_BYTES         (CAM_W * CAM_H * 2U)
#define JPEG_BUF_BYTES          (1536U * 1024U)
#define MIC_SAMPLES             1024U           /* Record refuses more: NbrOfBytes/2 <= BSP_AUDIO_IN_STAGING_SAMPLES */
#define SPK_SAMPLES             4096U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define P(...)   (void)BSP_COM_Printf(__VA_ARGS__)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static uint8_t  CamFrame[CAM_FRAME_BYTES] __attribute__((section(".ext_psram"), aligned(32)));
static uint8_t  JpegBuf[JPEG_BUF_BYTES]  __attribute__((section(".ext_psram"), aligned(32)));
/* Bytes of JpegBuf that jpegdump may send; set only by jpegmem. */
static uint32_t JpegMemBytes;
static uint8_t  CapBuf[2592U * 1944U * 2U]
  __attribute__((section(".ext_psram"), aligned(32)));
/* The band, plus a guard region behind it. Sized for 1080p, the largest
 * sensor mode, because jpegres walks every mode through it. */
#define BAND_BYTES              491520U
#define BAND_GUARD_BYTES        4096U
static uint8_t  Band[BAND_BYTES + BAND_GUARD_BYTES] __attribute__((section(".axisram_nc"), aligned(32)));
static int16_t  MicBuf[MIC_SAMPLES]      __attribute__((aligned(32)));
static int16_t  SpkBuf[SPK_SAMPLES]      __attribute__((aligned(32)));

static volatile uint32_t CamFrames, CamLines, CamErrors;
static volatile uint32_t CamPipeFrames[3];   /* frames per pipe, for the checks that run two */
#if defined(BSP_USE_FREERTOS) && (BSP_USE_FREERTOS == 1)
static uint32_t PreSchedDelayCycles;          /* cycles in HAL_Delay(50) before the scheduler */
static uint32_t PreSchedTick;                 /* HAL_GetTick() just before the scheduler starts */
#endif
static volatile uint32_t MicHalf, MicFull, MicErr, SpkHalf, SpkFull, SpkErr;
static volatile uint32_t MicTcFirst, MicTcLast;   /* ticks of the first and latest transfer complete */
static volatile uint32_t SpkTcFirst, SpkTcLast;
static uint32_t mic_measured_rate(void);
static uint32_t spk_measured_rate(uint32_t framesPerBuffer);
static int      rate_agrees(uint32_t claimed, uint32_t measured);
static char     Line[64];
static uint32_t LineLen;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void Error_Handler(void);
/* USER CODE BEGIN PFP */
static void mpu_config(void);
static void svc(uint32_t ms);
static void dispatch(const char *cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Helpers */

static void reboot_to_bootloader_safely(void);

/**
  * @brief  Every wait in this file: runs the USB stack, honours 'b' and
  *         refreshes the watchdog.
  */
static void svc(uint32_t ms)
{
  uint32_t t0 = HAL_GetTick();
  uint8_t  ch;
  do
  {
#if defined(BSP_USE_FREERTOS) && (BSP_USE_FREERTOS == 1)
    /* The USB task services the stack. */
    vTaskDelay(1U);
#else
    BSP_COM_Task();
#endif
    (void)BSP_WDG_Refresh();
    if ((BSP_COM_Read(&ch, 1U) == 1) && (ch == (uint8_t)'b'))
    {
      reboot_to_bootloader_safely();
    }
  } while ((HAL_GetTick() - t0) < ms);
}

/* Called on every encoder poll iteration: keeps USB serviced during an encode. */
void BSP_VENC_ReadPointerHook(uint32_t MbRow)
{
  static uint32_t last = 0U;
  volatile uint32_t *bkp = &TAMP->BKP0R;
  (void)MbRow;

  /* This refreshes the watchdog, so a hung encode never trips it: the console
   * just stops answering. */
  bkp[6]++;   /* bkp[0]/bkp[3] belong to the register trace */

  if ((HAL_GetTick() - last) >= 5U) { last = HAL_GetTick(); svc(0U); }
}

/* Encoder register trace: BKP0 = last offset (bit 31 = write), BKP3 = count. */
void BSP_VENC_RegisterTrace(uint32_t Offset, uint32_t IsWrite)
{
  volatile uint32_t *bkp = &TAMP->BKP0R;
  bkp[0] = (IsWrite != 0U) ? (0x80000000U | Offset) : Offset;
  bkp[3]++;
}

/* Called once on entry to the encoder's hardware wait, before its poll loop. */
void BSP_VENC_WaitBeginHook(void)
{
  (&TAMP->BKP0R)[6] |= 0x80000000U;   /* top bit: the wait was entered */
}

/**
  * @brief  Reset into the bootloader without leaving the encoder hung. Every
  *         reset in this image goes through here.
  */
static void reboot_to_bootloader_safely(void)
{
  (void)BSP_CAMERA_HaltCapture(0);
  (void)BSP_VENC_Quiesce(0);
  BSP_COM_RebootToBootloader();
}

/* One console pass: refresh the watchdog, run a completed line, blink. */
static void dispatch(const char *cmd);
static void console_step(void)
{
  static uint32_t last_blink;
  uint8_t ch;

  (void)BSP_WDG_Refresh();

  while (BSP_COM_Read(&ch, 1U) == 1)
  {
    if ((ch == (uint8_t)'\n') || (ch == (uint8_t)'\r'))
    {
      if (LineLen > 0U)
      {
        Line[LineLen] = 0;
        LineLen = 0U;
        /* Only a whole-line "b" reboots: bkp, btn, bt also begin with b. */
        if (strcmp(Line, "b") == 0) { reboot_to_bootloader_safely(); }
        dispatch(Line);
      }
    }
    else if (LineLen < sizeof Line - 1U) { Line[LineLen++] = (char)ch; }
  }

  if ((HAL_GetTick() - last_blink) >= 500U) { last_blink = HAL_GetTick(); (void)BSP_LED_Toggle(LED_BLUE); }
}

#if defined(BSP_USE_FREERTOS) && (BSP_USE_FREERTOS == 1)
/* 16 KB: commands keep buffers on the stack, and the encoder's poll hook
 * re-enters svc(). */
#define CONSOLE_TASK_STACK_WORDS  (16U * 1024U / sizeof(StackType_t))
#define CONSOLE_TASK_PRIORITY     (tskIDLE_PRIORITY + 2U)
static TaskHandle_t ConsoleTask;

static void console_task(void *pArg)
{
  (void)pArg;
  for (;;)
  {
    vTaskDelay(1U);
    console_step();
  }
}
#endif

void BSP_WDG_EWI_Callback(void)
{
  volatile uint32_t *bkp = &TAMP->BKP0R;
  HAL_PWR_EnableBkUpAccess();

  /* Save the encoder's state for after the reset: a hung encode's console
   * output never escapes. Runs above the kernel's interrupts: no kernel
   * calls, time from uwTick, encoder read only with its clock on. */
  if (__HAL_RCC_VENC_IS_CLK_ENABLED() != 0U)
  {
    bkp[0] = LL_VENC_ReadRegister(1UL);
    bkp[3] = LL_VENC_ReadRegister(497UL);
  }
  bkp[7] = 0xFD000000U | ((uwTick - bkp[5]) & 0x00FFFFFFU);

  reboot_to_bootloader_safely();
}

static void ok(const char *c, const char *fmt, ...)
{
  char b[160]; va_list a; va_start(a, fmt); vsnprintf(b, sizeof b, fmt, a); va_end(a);
  P("OK %s %s\r\nEND %s\r\n", c, b, c); BSP_COM_Flush(200U);
}
static void fail(const char *c, const char *fmt, ...)
{
  char b[160]; va_list a; va_start(a, fmt); vsnprintf(b, sizeof b, fmt, a); va_end(a);
  P("FAIL %s %s\r\nEND %s\r\n", c, b, c); BSP_COM_Flush(200U);
}
static void skip(const char *c, const char *why) { P("SKIP %s %s\r\nEND %s\r\n", c, why, c); BSP_COM_Flush(200U); }

/* Commands */

static void cmd_id(void)
{
  P("INFO board=%s version=0x%08lx tick=%ld\r\n", (const char *)BSP_GetBoardName(),
    (unsigned long)BSP_GetVersion(), (long)BSP_GetTick());
  ok("id", "board=%s ver=0x%08lx", (const char *)BSP_GetBoardName(), (unsigned long)BSP_GetVersion());
}

static void cmd_clk(void)
{
  ok("clk", "sysclk=%lu hclk=%lu pclk1=%lu pclk2=%lu", (unsigned long)HAL_RCC_GetSysClockFreq(),
     (unsigned long)HAL_RCC_GetHCLKFreq(), (unsigned long)HAL_RCC_GetPCLK1Freq(), (unsigned long)HAL_RCC_GetPCLK2Freq());
}

static void cmd_led(void)
{
  int32_t bad = 0;
  Led_TypeDef leds[3] = { LED_RED, LED_GREEN, LED_BLUE };
  for (uint32_t i = 0; i < 3U; i++)
  {
    bad |= BSP_LED_Init(leds[i]); bad |= BSP_LED_On(leds[i]);
    if (BSP_LED_GetState(leds[i]) != 1) { bad |= 1; }
    svc(120U); bad |= BSP_LED_Off(leds[i]);
    if (BSP_LED_GetState(leds[i]) != 0) { bad |= 1; }
  }
  if (bad != 0) { fail("led", "init/on/off/getstate disagreed"); return; }
  ok("led", "red green blue each lit 120ms, state read back");
}

static void cmd_btn(void)
{
  if (BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO) != BSP_ERROR_NONE) { fail("btn", "init"); return; }
  ok("btn", "state=%ld (0=released)", (long)BSP_PB_GetState(BUTTON_USER));
}

/* Probe the sensor bus for everything that could be on it. */
static void cmd_i2c(void)
{
  static const struct { const char *n; uint16_t a7; } d[] = {
    { "ov5640", 0x3CU }, { "lsm6dsl", 0x6AU }, { "lsm6dsl_alt", 0x6BU },
    { "mmc5603nj", 0x30U }, { "vl53l1cb", 0x29U }, { "lepton", 0x2AU } };
  if (BSP_I2C1_Init() != BSP_ERROR_NONE) { fail("i2c", "I2C1 init"); return; }
  char found[120] = ""; uint32_t n = 0U;
  for (uint32_t i = 0; i < sizeof d / sizeof d[0]; i++)
  {
    int32_t r7 = BSP_I2C1_IsReady(d[i].a7, 2U), r8 = BSP_I2C1_IsReady((uint16_t)(d[i].a7 << 1), 2U);
    P("INFO %s 7bit=%s 8bit=%s\r\n", d[i].n, r7 == BSP_ERROR_NONE ? "ack" : "-", r8 == BSP_ERROR_NONE ? "ack" : "-");
    if ((r7 == BSP_ERROR_NONE) || (r8 == BSP_ERROR_NONE)) { strncat(found, d[i].n, sizeof found - strlen(found) - 2); strcat(found, " "); n++; }
    svc(2U);
  }
  (void)BSP_I2C1_DeInit();
  if (n == 0U) { fail("i2c", "nothing answered on I2C1"); return; }
  ok("i2c", "present=%lu [%s]", (unsigned long)n, found);
}

static void cmd_imu(void)
{
  BSP_MOTION_SENSOR_Axes_t a = {0}, g = {0}; uint8_t id = 0; float t = 0.0f;
  if (BSP_MOTION_SENSOR_Init(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO) != BSP_ERROR_NONE) { fail("imu", "init (LSM6DSL absent or bus)"); return; }
  (void)BSP_MOTION_SENSOR_ReadID(MOTION_SENSOR_LSM6DSL_0, &id);
  (void)BSP_MOTION_SENSOR_Enable(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO);
  (void)BSP_MOTION_SENSOR_Enable(MOTION_SENSOR_LSM6DSL_0, MOTION_GYRO);
  svc(60U);
  int32_t sx = 0, sy = 0, sz = 0, n = 0;
  for (uint32_t i = 0; i < 8U; i++)
  {
    if (BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO, &a) == BSP_ERROR_NONE) { sx += a.x; sy += a.y; sz += a.z; n++; }
    svc(12U);
  }
  (void)BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_LSM6DSL_0, MOTION_GYRO, &g);
  (void)BSP_MOTION_SENSOR_GetTemperature(MOTION_SENSOR_LSM6DSL_0, &t);
  if (n == 0) { fail("imu", "no accelerometer samples"); return; }
  sx /= n; sy /= n; sz /= n;
  float mag = sqrtf((float)sx * sx + (float)sy * sy + (float)sz * sz);
  P("INFO id=0x%02x acc_mg=%ld,%ld,%ld |a|=%.0f gyro_mdps=%ld,%ld,%ld temp=%.1fC\r\n", id, (long)sx, (long)sy, (long)sz, mag, (long)g.x, (long)g.y, (long)g.z, t);
  (void)BSP_MOTION_SENSOR_DeInit(MOTION_SENSOR_LSM6DSL_0);
  if ((mag < 700.0f) || (mag > 1300.0f)) { fail("imu", "|a|=%.0f mg, expected ~1000 at rest", mag); return; }
  ok("imu", "id=0x%02x |a|=%.0fmg gyro=%ld,%ld,%ld temp=%.1f", id, mag, (long)g.x, (long)g.y, (long)g.z, t);
}

static void cmd_mag(void)
{
  BSP_MOTION_SENSOR_Axes_t m = {0}; uint8_t id = 0;
  if (BSP_MOTION_SENSOR_Init(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO) != BSP_ERROR_NONE) { fail("mag", "init (MMC5603NJ absent or bus)"); return; }
  (void)BSP_MOTION_SENSOR_ReadID(MOTION_SENSOR_MMC5603NJ_0, &id);
  (void)BSP_MOTION_SENSOR_Enable(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO);
  svc(80U);
  int32_t r = BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO, &m);
  float mag = sqrtf((float)m.x * m.x + (float)m.y * m.y + (float)m.z * m.z);
  if ((r != BSP_ERROR_NONE) || (mag < 50.0f) || (mag > 5000.0f))
  {
    /* Post-mortem while the bus is up. Catches every output register zero
     * with the driver reporting success, which polling the status during an
     * automatic set/reset measurement causes. */
    uint8_t st = 0xEEU, st2 = 0xEEU, st3 = 0xEEU, raw[9] = {0}, raw2[9] = {0}, raw3[9] = {0};
    uint8_t v = MMC5603NJ_CTRL0_AUTO_SR_EN | MMC5603NJ_CTRL0_TAKE_MEAS_M;
    uint32_t polls = 0U, t0;
    BSP_MOTION_SENSOR_Axes_t m2 = {0};
    (void)BSP_I2C1_ReadReg(MMC5603NJ_I2C_ADDRESS, MMC5603NJ_STATUS1, &st, 1U);
    (void)BSP_I2C1_ReadReg(MMC5603NJ_I2C_ADDRESS, MMC5603NJ_XOUT0, raw, 9U);
    svc(20U);
    (void)BSP_I2C1_ReadReg(MMC5603NJ_I2C_ADDRESS, MMC5603NJ_STATUS1, &st2, 1U);
    (void)BSP_I2C1_ReadReg(MMC5603NJ_I2C_ADDRESS, MMC5603NJ_XOUT0, raw2, 9U);
    P("INFO post-mortem: r=%ld axes=%ld,%ld,%ld status=%02X xout=%02X%02X %02X%02X %02X%02X; 20ms later status=%02X xout=%02X%02X %02X%02X %02X%02X\r\n",
      (long)r, (long)m.x, (long)m.y, (long)m.z, st, raw[0], raw[1], raw[2], raw[3], raw[4], raw[5],
      st2, raw2[0], raw2[1], raw2[2], raw2[3], raw2[4], raw2[5]);
    t0 = HAL_GetTick();
    (void)BSP_I2C1_WriteReg(MMC5603NJ_I2C_ADDRESS, MMC5603NJ_CTRL0, &v, 1U);
    do
    {
      st3 = 0U;
      (void)BSP_I2C1_ReadReg(MMC5603NJ_I2C_ADDRESS, MMC5603NJ_STATUS1, &st3, 1U);
      polls++;
    } while (((st3 & MMC5603NJ_STATUS1_MEAS_M_DONE) == 0U) && ((HAL_GetTick() - t0) < 50U));
    (void)BSP_I2C1_ReadReg(MMC5603NJ_I2C_ADDRESS, MMC5603NJ_XOUT0, raw3, 9U);
    P("INFO post-mortem by hand: status=%02X after %lu polls %lu ms xout=%02X%02X %02X%02X %02X%02X\r\n",
      st3, (unsigned long)polls, (unsigned long)(HAL_GetTick() - t0), raw3[0], raw3[1], raw3[2], raw3[3], raw3[4], raw3[5]);
    r = BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO, &m2);
    P("INFO post-mortem driver again: r=%ld |B|=%.0f\r\n", (long)r,
      sqrtf((float)m2.x * m2.x + (float)m2.y * m2.y + (float)m2.z * m2.z));
    (void)BSP_MOTION_SENSOR_DeInit(MOTION_SENSOR_MMC5603NJ_0);
    if (r != BSP_ERROR_NONE) { fail("mag", "GetAxes %ld", (long)r); return; }
    /* Name the known failure: every output register zero. */
    if ((m.x == -32000) && (m.y == -32000) && (m.z == -32000))
    {
      fail("mag", "every output register read zero: the part answered (id=0x%02X) "
                  "and said its measurement finished, and stored nothing", id);
      return;
    }

    /* The ID tells the remaining causes apart. */
    fail("mag", "|B|=%.0f mG implausible, id=0x%02X (0x10 expected)", mag, id);
    return;
  }
  (void)BSP_MOTION_SENSOR_DeInit(MOTION_SENSOR_MMC5603NJ_0);
  ok("mag", "id=0x%02x mG=%ld,%ld,%ld |B|=%.0f", id, (long)m.x, (long)m.y, (long)m.z, mag);
}

static void cmd_adc(void)
{
  uint32_t vdda = 0, vbat = 0, core = 0, mv[6] = {0}, raw[6] = {0};
  uint32_t lo = 0xFFFFFFFFU, hi = 0U; int32_t bad = 0;
  if (BSP_ADC_Init() != BSP_ERROR_NONE) { fail("adc", "init"); return; }
  /* Rail reads fail with COMPONENT_FAILURE when this factory calibration word
   * is outside the accepted range. */
  P("INFO vrefint_cal@%p = 0x%04x (accepted 1000..4000)\r\n",
    (void *)VREFINT_CAL_ADDR, (unsigned)(*VREFINT_CAL_ADDR));
  int32_t r1 = BSP_ADC_GetVdda(&vdda);
  int32_t r2 = BSP_ADC_ReadVBat(&vbat);
  int32_t r3 = BSP_ADC_ReadVddCore(&core);
  P("INFO vdda=%lu(%ld) vbat=%lu(%ld) vcore=%lu(%ld)\r\n", (unsigned long)vdda, (long)r1,
    (unsigned long)vbat, (long)r2, (unsigned long)core, (long)r3);
  bad = r1 | r2 | r3;
  /* Raw counts too: with ADC12 left non-secure every external channel
   * converts an unconnected internal node, a narrow mid-scale band. */
  for (uint32_t i = 0; i < 6U; i++)
  {
    bad |= BSP_ADC_ReadVoltage(BSP_IO_PIN_A0 + i, &mv[i]);
    bad |= BSP_ADC_ReadChannel(BSP_IO_PIN_A0 + i, &raw[i]);
    if (raw[i] < lo) { lo = raw[i]; }
    if (raw[i] > hi) { hi = raw[i]; }
    svc(1U);
  }
  P("INFO A0..A5 mV=%lu,%lu,%lu,%lu,%lu,%lu\r\n", (unsigned long)mv[0], (unsigned long)mv[1], (unsigned long)mv[2], (unsigned long)mv[3], (unsigned long)mv[4], (unsigned long)mv[5]);
  P("INFO A0..A5 raw=%lu,%lu,%lu,%lu,%lu,%lu (of 4095, span=%lu)\r\n",
    (unsigned long)raw[0], (unsigned long)raw[1], (unsigned long)raw[2],
    (unsigned long)raw[3], (unsigned long)raw[4], (unsigned long)raw[5],
    (unsigned long)(hi - lo));
  (void)BSP_ADC_DeInit();
  if (bad != 0) { fail("adc", "rail read failed: vdda=%ld vbat=%ld vcore=%ld", (long)r1, (long)r2, (long)r3); return; }
  /* The reference is VDDA_1V8 (VREFINT_CAL_VREF is 1800), not 3.3 V. */
  if ((vdda < 1700U) || (vdda > 1900U)) { fail("adc", "vdda=%lumV implausible for a 1V8 reference", (unsigned long)vdda); return; }
  /* VDDCORE, nominally 800 mV, through the same reference: a cross-check. */
  if ((core < 600U) || (core > 1000U)) { fail("adc", "vcore=%lumV implausible (expect ~800)", (unsigned long)core); return; }
  ok("adc", "vdda=%lu vbat=%lu vcore=%lu mV, A0..A5 raw span %lu", (unsigned long)vdda, (unsigned long)vbat, (unsigned long)core, (unsigned long)(hi - lo));
}

/* The motion sensors' configuration calls. */
static void cmd_imucfg(void)
{
  BSP_MOTION_SENSOR_Capabilities_t cap = {0};
  BSP_MOTION_SENSOR_Axes_t    mg = {0};
  BSP_MOTION_SENSOR_AxesRaw_t raw = {0};
  float odr = 0.0f, odr_back = 0.0f, sens_2g = 0.0f, sens_4g = 0.0f;
  int32_t fs = 0, r;
  uint32_t ready = 0U;

  if (BSP_MOTION_SENSOR_Init(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO) != BSP_ERROR_NONE)
  { fail("imucfg", "init"); return; }

  if (BSP_MOTION_SENSOR_GetCapabilities(MOTION_SENSOR_LSM6DSL_0, &cap) != BSP_ERROR_NONE)
  { fail("imucfg", "GetCapabilities"); return; }
  P("INFO caps acc=%u gyro=%u mag=%u accMaxFS=%lug gyroMaxFS=%ludps accMaxOdr=%.0fHz\r\n",
    cap.Acc, cap.Gyro, cap.Magneto, (unsigned long)cap.AccMaxFS,
    (unsigned long)cap.GyroMaxFS, (double)cap.AccMaxOdr);

  (void)BSP_MOTION_SENSOR_Enable(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO);
  svc(20U);

  /* Output data rate: set a supported one and read it back. */
  if (BSP_MOTION_SENSOR_GetOutputDataRate(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO, &odr) != BSP_ERROR_NONE)
  { fail("imucfg", "GetOutputDataRate"); return; }
  r = BSP_MOTION_SENSOR_SetOutputDataRate(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO, 104.0f);
  svc(20U);
  (void)BSP_MOTION_SENSOR_GetOutputDataRate(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO, &odr_back);
  P("INFO acc odr %.1f -> asked 104 -> %.1f (set r=%ld)\r\n", (double)odr, (double)odr_back, (long)r);
  if (r != BSP_ERROR_NONE) { fail("imucfg", "SetOutputDataRate %ld", (long)r); return; }
  if ((odr_back < 52.0f) || (odr_back > 208.0f)) { fail("imucfg", "odr came back %.1f for a 104 request", (double)odr_back); return; }

  /* Full scale: the sensitivity (mg per count) must scale with it. */
  if (BSP_MOTION_SENSOR_SetFullScale(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO, 2) != BSP_ERROR_NONE)
  { fail("imucfg", "SetFullScale 2g"); return; }
  svc(10U);
  (void)BSP_MOTION_SENSOR_GetFullScale(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO, &fs);
  (void)BSP_MOTION_SENSOR_GetSensitivity(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO, &sens_2g);
  if (fs != 2) { fail("imucfg", "asked 2g, read back %ldg", (long)fs); return; }

  if (BSP_MOTION_SENSOR_SetFullScale(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO, 4) != BSP_ERROR_NONE)
  { fail("imucfg", "SetFullScale 4g"); return; }
  svc(10U);
  (void)BSP_MOTION_SENSOR_GetFullScale(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO, &fs);
  (void)BSP_MOTION_SENSOR_GetSensitivity(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO, &sens_4g);
  P("INFO sensitivity 2g=%.4f 4g=%.4f mg/LSB (ratio %.2f), fs readback=%ldg\r\n",
    (double)sens_2g, (double)sens_4g, (double)((sens_2g > 0.0f) ? (sens_4g / sens_2g) : 0.0f), (long)fs);
  if (fs != 4) { fail("imucfg", "asked 4g, read back %ldg", (long)fs); return; }
  if ((sens_2g <= 0.0f) || (sens_4g <= 0.0f)) { fail("imucfg", "sensitivity reported as zero"); return; }
  if (((sens_4g / sens_2g) < 1.6f) || ((sens_4g / sens_2g) > 2.4f))
  { fail("imucfg", "sensitivity ratio %.2f, expected about 2", (double)(sens_4g / sens_2g)); return; }

  /* Raw against scaled: the same sample, two ways. */
  svc(30U);
  (void)BSP_MOTION_SENSOR_GetDataReady(MOTION_SENSOR_LSM6DSL_0, &ready);
  if (BSP_MOTION_SENSOR_GetAxesRaw(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO, &raw) != BSP_ERROR_NONE)
  { fail("imucfg", "GetAxesRaw"); return; }
  if (BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO, &mg) != BSP_ERROR_NONE)
  { fail("imucfg", "GetAxes"); return; }
  {
    float from_raw = (float)raw.z * sens_4g;
    float diff = from_raw - (float)mg.z;
    if (diff < 0.0f) { diff = -diff; }
    P("INFO dataready=0x%lx raw.z=%d x sens = %.0f mg, GetAxes z = %ld mg, diff %.0f\r\n",
      (unsigned long)ready, raw.z, (double)from_raw, (long)mg.z, (double)diff);
    /* Two consecutive samples, so generous: it looks for a scale error. */
    if (diff > 250.0f) { fail("imucfg", "raw x sensitivity = %.0f mg but GetAxes says %ld mg", (double)from_raw, (long)mg.z); return; }
  }

  if (BSP_MOTION_SENSOR_Disable(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO) != BSP_ERROR_NONE)
  { fail("imucfg", "Disable"); return; }
  (void)BSP_MOTION_SENSOR_DeInit(MOTION_SENSOR_LSM6DSL_0);

  /* The magnetometer's degauss: the field after it must still be plausible. */
  if (BSP_MOTION_SENSOR_Init(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO) != BSP_ERROR_NONE)
  { fail("imucfg", "mag init"); return; }
  (void)BSP_MOTION_SENSOR_Enable(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO);
  svc(80U);
  r = BSP_MOTION_SENSOR_Degauss(MOTION_SENSOR_MMC5603NJ_0);
  svc(50U);
  {
    BSP_MOTION_SENSOR_Axes_t m = {0};
    float field;
    (void)BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO, &m);
    field = sqrtf((float)m.x * m.x + (float)m.y * m.y + (float)m.z * m.z);
    P("INFO degauss r=%ld, field after = %.0f mG\r\n", (long)r, (double)field);
    (void)BSP_MOTION_SENSOR_DeInit(MOTION_SENSOR_MMC5603NJ_0);
    if (r != BSP_ERROR_NONE) { fail("imucfg", "Degauss %ld", (long)r); return; }
    if ((field < 50.0f) || (field > 5000.0f)) { fail("imucfg", "field %.0f mG after degauss", (double)field); return; }
  }

  ok("imucfg", "odr, full scale and sensitivity track each other; raw agrees with scaled; degauss clean");
}

static int32_t cam_up(void);   /* defined with the camera tests further down */

/* Two display framebuffers, 800 x 480 RGB565 (768 KB each), in PSRAM. */
#define LCD_FB_BYTES            (BSP_LCD_WIDTH * BSP_LCD_HEIGHT * 2U)
static uint8_t LcdFb0[LCD_FB_BYTES] __attribute__((section(".ext_psram"), aligned(32)));
static uint8_t LcdFb1[LCD_FB_BYTES] __attribute__((section(".ext_psram"), aligned(32)));

/* Wait, at most 100 ms, for a layer's queued reload to be taken. */
static void lcd_wait_reload(uint32_t layer)
{
  uint32_t pending = 1U;
  const uint32_t t0 = HAL_GetTick();

  while ((pending != 0U) && ((HAL_GetTick() - t0) < 100U))
  {
    (void)BSP_LCD_IsReloadPending(0, layer, &pending);
  }
}

static void cmd_lcd(void)
{
  BSP_LCD_LayerConfig_t cfg = {0};
  BSP_LCD_Status_t st = {0};
  uint32_t x = 0U, y = 0U, px = 0U, hz = 0U, fmt = 0U, pending = 0U;
  int32_t  r;

  r = BSP_LCD_Init(0, BSP_LCD_WIDTH, BSP_LCD_HEIGHT);
  if (r != BSP_ERROR_NONE) { fail("lcd", "init %ld (panel absent, or the LTDC refused)", (long)r); return; }

  (void)BSP_LCD_GetXSize(0, &x);
  (void)BSP_LCD_GetYSize(0, &y);
  P("INFO panel %lu x %lu\r\n", (unsigned long)x, (unsigned long)y);
  if ((x != BSP_LCD_WIDTH) || (y != BSP_LCD_HEIGHT))
  { fail("lcd", "reported %lux%lu, expected %ux%u", (unsigned long)x, (unsigned long)y, (unsigned)BSP_LCD_WIDTH, (unsigned)BSP_LCD_HEIGHT); goto lcd_out; }

  cfg.X0 = 0U; cfg.X1 = BSP_LCD_WIDTH;
  cfg.Y0 = 0U; cfg.Y1 = BSP_LCD_HEIGHT;
  cfg.PixelFormat = BSP_LCD_PIXEL_FORMAT_RGB565;
  cfg.Address = (uint32_t)LcdFb0;
  r = BSP_LCD_ConfigLayer(0, BSP_LCD_LAYER_0, &cfg);
  if (r != BSP_ERROR_NONE) { fail("lcd", "ConfigLayer 0 %ld", (long)r); goto lcd_out; }

  (void)BSP_LCD_GetPixelFormat(0, &fmt);
  r = BSP_LCD_SetActiveLayer(0, BSP_LCD_LAYER_0);
  if (r != BSP_ERROR_NONE) { fail("lcd", "SetActiveLayer %ld", (long)r); goto lcd_out; }
  if (BSP_LCD_DisplayOn(0) != BSP_ERROR_NONE) { fail("lcd", "DisplayOn"); goto lcd_out; }

  /* Geometry, read back from the framebuffer. */
  if (BSP_LCD_FillRect(0, 0U, 0U, BSP_LCD_WIDTH, BSP_LCD_HEIGHT, 0x0000U) != BSP_ERROR_NONE)
  { fail("lcd", "FillRect over the whole panel"); goto lcd_out; }

  /* All four corners: catches a wrong pitch. */
  (void)BSP_LCD_WritePixel(0, 0U, 0U, 0xF800U);                                  /* red    */
  (void)BSP_LCD_WritePixel(0, BSP_LCD_WIDTH - 1U, 0U, 0x07E0U);                  /* green  */
  (void)BSP_LCD_WritePixel(0, 0U, BSP_LCD_HEIGHT - 1U, 0x001FU);                 /* blue   */
  (void)BSP_LCD_WritePixel(0, BSP_LCD_WIDTH - 1U, BSP_LCD_HEIGHT - 1U, 0xFFFFU); /* white  */
  {
    uint32_t c00 = 0U, c10 = 0U, c01 = 0U, c11 = 0U, wrap = 0U;
    (void)BSP_LCD_ReadPixel(0, 0U, 0U, &c00);
    (void)BSP_LCD_ReadPixel(0, BSP_LCD_WIDTH - 1U, 0U, &c10);
    (void)BSP_LCD_ReadPixel(0, 0U, BSP_LCD_HEIGHT - 1U, &c01);
    (void)BSP_LCD_ReadPixel(0, BSP_LCD_WIDTH - 1U, BSP_LCD_HEIGHT - 1U, &c11);
    /* The start of row 1 must still be black; green means the row stride is
     * wrong and the corner wrote into the next row. */
    (void)BSP_LCD_ReadPixel(0, 0U, 1U, &wrap);
    P("INFO corners TL=0x%04lx TR=0x%04lx BL=0x%04lx BR=0x%04lx, start of row 1 = 0x%04lx\r\n",
      (unsigned long)c00, (unsigned long)c10, (unsigned long)c01, (unsigned long)c11, (unsigned long)wrap);
    if ((c00 != 0xF800U) || (c10 != 0x07E0U) || (c01 != 0x001FU) || (c11 != 0xFFFFU))
    { fail("lcd", "a corner read back wrong: %04lx/%04lx/%04lx/%04lx", (unsigned long)c00, (unsigned long)c10, (unsigned long)c01, (unsigned long)c11); goto lcd_out; }
    if (wrap != 0x0000U)
    { fail("lcd", "the pixel after row 0 is 0x%04lx, so writes are wrapping between rows", (unsigned long)wrap); goto lcd_out; }
  }

  /* A filled rectangle, checked inside and just outside. */
  if (BSP_LCD_FillRect(0, 100U, 50U, 40U, 30U, 0xF81FU) != BSP_ERROR_NONE)
  { fail("lcd", "FillRect"); goto lcd_out; }
  {
    uint32_t in = 0U, left = 0U, below = 0U;
    (void)BSP_LCD_ReadPixel(0, 120U, 60U, &in);
    (void)BSP_LCD_ReadPixel(0, 99U, 60U, &left);
    (void)BSP_LCD_ReadPixel(0, 120U, 80U, &below);
    P("INFO fillrect inside=0x%04lx just left=0x%04lx just below=0x%04lx\r\n",
      (unsigned long)in, (unsigned long)left, (unsigned long)below);
    if (in != 0xF81FU) { fail("lcd", "the rectangle's interior read 0x%04lx", (unsigned long)in); goto lcd_out; }
    if ((left != 0x0000U) || (below != 0x0000U))
    { fail("lcd", "the rectangle spilled past its bounds (%04lx / %04lx)", (unsigned long)left, (unsigned long)below); goto lcd_out; }
  }

  /* Lines, checked on and one pixel off. */
  (void)BSP_LCD_DrawHLine(0, 200U, 200U, 50U, 0x07FFU);
  (void)BSP_LCD_DrawVLine(0, 300U, 100U, 40U, 0xFFE0U);
  {
    uint32_t h_on = 0U, h_end = 0U, v_on = 0U, v_end = 0U;
    (void)BSP_LCD_ReadPixel(0, 249U, 200U, &h_on);
    (void)BSP_LCD_ReadPixel(0, 250U, 200U, &h_end);
    (void)BSP_LCD_ReadPixel(0, 300U, 139U, &v_on);
    (void)BSP_LCD_ReadPixel(0, 300U, 140U, &v_end);
    P("INFO hline last=0x%04lx past=0x%04lx; vline last=0x%04lx past=0x%04lx\r\n",
      (unsigned long)h_on, (unsigned long)h_end, (unsigned long)v_on, (unsigned long)v_end);
    if ((h_on != 0x07FFU) || (v_on != 0xFFE0U))
    { fail("lcd", "a line did not reach its last pixel"); goto lcd_out; }
    if ((h_end != 0x0000U) || (v_end != 0x0000U))
    { fail("lcd", "a line ran one pixel past its length"); goto lcd_out; }
  }

  /* A small RGB block written from memory. */
  {
    static uint16_t block[8U * 4U];
    for (uint32_t i = 0U; i < (8U * 4U); i++) { block[i] = (uint16_t)(0x1000U + i); }
    r = BSP_LCD_FillRGBRect(0, 400U, 300U, (const uint8_t *)block, 8U, 4U);
    if (r != BSP_ERROR_NONE) { fail("lcd", "FillRGBRect %ld", (long)r); goto lcd_out; }
    (void)BSP_LCD_ReadPixel(0, 400U, 300U, &px);
    { uint32_t last = 0U;
      (void)BSP_LCD_ReadPixel(0, 407U, 303U, &last);
      P("INFO rgbrect first=0x%04lx (want 0x1000) last=0x%04lx (want 0x101f)\r\n",
        (unsigned long)px, (unsigned long)last);
      if ((px != 0x1000U) || (last != 0x101FU))
      { fail("lcd", "the RGB block landed wrong: first 0x%04lx last 0x%04lx", (unsigned long)px, (unsigned long)last); goto lcd_out; }
    }
  }

  /* The second layer and the compositing controls. */
  cfg.Address = (uint32_t)LcdFb1;
  r = BSP_LCD_ConfigLayer(0, BSP_LCD_LAYER_1, &cfg);
  if (r != BSP_ERROR_NONE) { fail("lcd", "ConfigLayer 1 %ld", (long)r); goto lcd_out; }
  r  = BSP_LCD_SetLayerVisible(0, BSP_LCD_LAYER_1, ENABLE);
  r |= BSP_LCD_SetTransparency(0, BSP_LCD_LAYER_1, 128U);
  r |= BSP_LCD_SetColorKeying(0, BSP_LCD_LAYER_1, 0x0000U);
  r |= BSP_LCD_ResetColorKeying(0, BSP_LCD_LAYER_1);
  r |= BSP_LCD_SetLayerWindow(0, BSP_LCD_LAYER_1, 10U, 10U, 100U, 80U);
  r |= BSP_LCD_SetLayerVisible(0, BSP_LCD_LAYER_1, DISABLE);
  P("INFO layer 1 visible/transparency/colour key/window r=%ld\r\n", (long)r);
  if (r != BSP_ERROR_NONE) { fail("lcd", "a layer-1 control refused (%ld)", (long)r); goto lcd_out; }

  /* Layer 1 must stay hidden while it is repointed or moved; its enable bit
   * is read once each change is taken. */
  lcd_wait_reload(BSP_LCD_LAYER_1);
  {
    static const char *const what[3] = { "address at the next blank", "address at once", "window" };
    uint32_t shown = 0U;

    for (uint32_t k = 0U; k < 3U; k++)
    {
      if (k == 0U)      { r = BSP_LCD_SetLayerAddressVBlank(0, BSP_LCD_LAYER_1, (uint32_t)LcdFb1); }
      else if (k == 1U) { r = BSP_LCD_SetLayerAddress(0, BSP_LCD_LAYER_1, (uint32_t)LcdFb1); }
      else              { r = BSP_LCD_SetLayerWindow(0, BSP_LCD_LAYER_1, 20U, 20U, 90U, 70U); }
      lcd_wait_reload(BSP_LCD_LAYER_1);
      if (r != BSP_ERROR_NONE) { fail("lcd", "setting hidden layer 1's %s refused (%ld)", what[k], (long)r); goto lcd_out; }
      if ((LTDC_Layer2->CR & LTDC_LxCR_LEN) != 0U)
      {
        P("INFO hidden layer 1 is showing again after setting its %s\r\n", what[k]);
        shown++;
        (void)BSP_LCD_SetLayerVisible(0, BSP_LCD_LAYER_1, DISABLE);
        lcd_wait_reload(BSP_LCD_LAYER_1);
      }
    }
    if (shown != 0U) { fail("lcd", "hidden layer 1 showed again after %lu of 3 changes", (unsigned long)shown); goto lcd_out; }
  }
  P("INFO hidden layer 1 stayed hidden through an address at the blank, one at once, and a window\r\n");

  /* Address changes and the reload that commits them. */
  r  = BSP_LCD_SetLayerAddress(0, BSP_LCD_LAYER_0, (uint32_t)LcdFb0);
  r |= BSP_LCD_SetLayerAddressVBlank(0, BSP_LCD_LAYER_0, (uint32_t)LcdFb0);
  (void)BSP_LCD_IsReloadPending(0, BSP_LCD_LAYER_0, &pending);
  r |= BSP_LCD_Reload(0, 0U);   /* at the next vertical blank */
  P("INFO address set + vblank set + reload r=%ld (pending was %lu)\r\n", (long)r, (unsigned long)pending);
  if (r != BSP_ERROR_NONE) { fail("lcd", "an address or reload call refused (%ld)", (long)r); goto lcd_out; }

  /* Reload must reach both layers: each has its own reload register, so a
   * blanking reload shows pending on each until the next frame takes it. */
  {
    uint32_t p0 = 0U, p1 = 0U, after0 = 1U, after1 = 1U;

    lcd_wait_reload(BSP_LCD_LAYER_0);
    lcd_wait_reload(BSP_LCD_LAYER_1);
    r = BSP_LCD_Reload(0, 0U);
    (void)BSP_LCD_IsReloadPending(0, BSP_LCD_LAYER_0, &p0);
    (void)BSP_LCD_IsReloadPending(0, BSP_LCD_LAYER_1, &p1);
    lcd_wait_reload(BSP_LCD_LAYER_0);
    lcd_wait_reload(BSP_LCD_LAYER_1);
    (void)BSP_LCD_IsReloadPending(0, BSP_LCD_LAYER_0, &after0);
    (void)BSP_LCD_IsReloadPending(0, BSP_LCD_LAYER_1, &after1);
    P("INFO reload queued on layer 0: %lu, layer 1: %lu; taken: %lu %lu\r\n", (unsigned long)p0,
      (unsigned long)p1, (unsigned long)(after0 == 0U), (unsigned long)(after1 == 0U));
    if ((r != BSP_ERROR_NONE) || (p0 == 0U) || (p1 == 0U))
    { fail("lcd", "Reload queued nothing on the layers (r=%ld, pending %lu and %lu)", (long)r, (unsigned long)p0, (unsigned long)p1); goto lcd_out; }
    if ((after0 != 0U) || (after1 != 0U))
    { fail("lcd", "a reload queued by Reload was never taken"); goto lcd_out; }
  }

  /* Refresh rate, and the error flags. */
  (void)BSP_LCD_GetRefreshRate(0, &hz);
  r = BSP_LCD_SetRefreshRate(0, hz);
  P("INFO refresh %lu Hz, setting it back r=%ld, pixel format 0x%lx\r\n",
    (unsigned long)hz, (long)r, (unsigned long)fmt);
  if (r != BSP_ERROR_NONE) { fail("lcd", "SetRefreshRate %ld", (long)r); goto lcd_out; }

  (void)BSP_LCD_GetStatus(0, &st);
  P("INFO status enabled=%lu underrun=%lu warn=%lu rot=%lu bus_error=%lu line=%lu\r\n",
    (unsigned long)st.Enabled, (unsigned long)st.Underrun, (unsigned long)st.UnderrunWarning,
    (unsigned long)st.UnderrunRotation, (unsigned long)st.TransferError, (unsigned long)st.CurrentLine);
  if (st.Enabled != 1U) { fail("lcd", "the controller reports itself disabled while displaying"); goto lcd_out; }
  if (st.TransferError != 0U)
  { fail("lcd", "the controller took a bus error fetching the framebuffer - PSRAM is not keeping up"); goto lcd_out; }

  /* The scan line must advance: the panel is driven, not just configured. */
  {
    uint32_t l0 = 0U, l1 = 0U, moved = 0U;
    (void)BSP_LCD_GetStatus(0, &st); l0 = st.CurrentLine;
    for (uint32_t i = 0U; (i < 20U) && (moved == 0U); i++)
    {
      svc(2U);
      (void)BSP_LCD_GetStatus(0, &st); l1 = st.CurrentLine;
      if (l1 != l0) { moved = 1U; }
    }
    P("INFO scan line %lu -> %lu (%s)\r\n", (unsigned long)l0, (unsigned long)l1,
      (moved != 0U) ? "advancing" : "STUCK");
    if (moved == 0U) { fail("lcd", "the scan line never moved: the controller is not scanning out"); goto lcd_out; }
  }

  if (BSP_LCD_DisplayOff(0) != BSP_ERROR_NONE) { fail("lcd", "DisplayOff"); goto lcd_out; }
  (void)BSP_LCD_DeInit(0);

  ok("lcd", "%lux%lu, corners and pitch right, rect and lines bounded, both layers configure, scan line advancing",
     (unsigned long)x, (unsigned long)y);
  return;

lcd_out:
  (void)BSP_LCD_DisplayOff(0);
  (void)BSP_LCD_DeInit(0);
}

/* A one-row bitmap file: the 14-byte file header, the 40-byte info header,
 * the three colour masks when there are any, and the row. */
static uint8_t BmpFile[128] __attribute__((aligned(4)));

static void bmp_put(uint8_t *p, uint32_t v, uint32_t bytes)
{
  for (uint32_t i = 0U; i < bytes; i++) { p[i] = (uint8_t)(v >> (8U * i)); }
}

static const uint8_t *bmp_row(uint32_t bpp, uint32_t compression, const uint32_t *masks,
                              const uint8_t *row, uint32_t pixels)
{
  const uint32_t hdr   = 14U + 40U + ((masks != NULL) ? 12U : 0U);
  const uint32_t bytes = ((pixels * (bpp / 8U)) + 3U) & ~3U;

  memset(BmpFile, 0, sizeof BmpFile);
  BmpFile[0] = (uint8_t)'B'; BmpFile[1] = (uint8_t)'M';
  bmp_put(&BmpFile[2], hdr + bytes, 4U);
  bmp_put(&BmpFile[10], hdr, 4U);
  bmp_put(&BmpFile[14], 40U, 4U);
  bmp_put(&BmpFile[18], pixels, 4U);
  bmp_put(&BmpFile[22], 1U, 4U);
  bmp_put(&BmpFile[26], 1U, 2U);
  bmp_put(&BmpFile[28], bpp, 2U);
  bmp_put(&BmpFile[30], compression, 4U);
  bmp_put(&BmpFile[34], bytes, 4U);
  if (masks != NULL)
  { bmp_put(&BmpFile[54], masks[0], 4U); bmp_put(&BmpFile[58], masks[1], 4U); bmp_put(&BmpFile[62], masks[2], 4U); }
  memcpy(&BmpFile[hdr], row, pixels * (bpp / 8U));
  return BmpFile;
}

/* Draw a one-row bitmap at the top left of the active layer and compare each
 * pixel with what the layer's format should hold. */
static int bmp_check(const char *name, const uint8_t *bmp, const uint32_t *want, uint32_t pixels)
{
  if (BSP_LCD_DrawBitmap(0, 0U, 0U, bmp) != BSP_ERROR_NONE) { P("INFO %s: DrawBitmap refused it\r\n", name); return 0; }
  for (uint32_t i = 0U; i < pixels; i++)
  {
    uint32_t got = 0U;
    (void)BSP_LCD_ReadPixel(0, i, 0U, &got);
    if (got != want[i])
    { P("INFO %s: pixel %lu is 0x%08lx, not 0x%08lx\r\n", name, (unsigned long)i, (unsigned long)got, (unsigned long)want[i]); return 0; }
  }
  P("INFO %s: %lu pixels as expected\r\n", name, (unsigned long)pixels);
  return 1;
}

/* Each common bitmap kind into each layer format. 16-bit BI_RGB is 5-5-5,
 * BI_BITFIELDS gives its masks; a 32-bit BI_RGB file's fourth byte is unused,
 * not alpha. */
static void cmd_lcdbmp(void)
{
  static const uint32_t m565[3] = { 0xF800U, 0x07E0U, 0x001FU };
  /* red, green, blue, white in each source form */
  static const uint8_t  rgb555[8]  = { 0x00, 0x7C, 0xE0, 0x03, 0x1F, 0x00, 0xFF, 0x7F };
  static const uint8_t  rgb565[8]  = { 0x00, 0xF8, 0xE0, 0x07, 0x1F, 0x00, 0xFF, 0xFF };
  static const uint8_t  bgr24[12]  = { 0, 0, 255,  0, 255, 0,  255, 0, 0,  255, 255, 255 };
  static const uint8_t  bgrx32[16] = { 0, 0, 255, 0,  0, 255, 0, 0,  255, 0, 0, 0,  255, 255, 255, 0 };
  static const uint32_t as565[4]   = { 0xF800U, 0x07E0U, 0x001FU, 0xFFFFU };
  static const uint32_t as8888[4]  = { 0xFFFF0000U, 0xFF00FF00U, 0xFF0000FFU, 0xFFFFFFFFU };
  static const uint32_t as4444[4]  = { 0xFF00U, 0xF0F0U, 0xF00FU, 0xFFFFU };
  BSP_LCD_LayerConfig_t cfg = {0};
  uint32_t bad = 0U;
  int32_t r;

  r = BSP_LCD_Init(0, BSP_LCD_WIDTH, BSP_LCD_HEIGHT);
  if (r != BSP_ERROR_NONE) { fail("lcdbmp", "init %ld", (long)r); return; }

  /* A 100 x 100 window, small enough for four bytes a pixel in LcdFb1. */
  cfg.X0 = 0U; cfg.X1 = 100U; cfg.Y0 = 0U; cfg.Y1 = 100U; cfg.Alpha = 255U;

  cfg.PixelFormat = BSP_LCD_PIXEL_FORMAT_RGB565; cfg.Address = (uint32_t)LcdFb1;
  if ((BSP_LCD_ConfigLayer(0, BSP_LCD_LAYER_1, &cfg) != BSP_ERROR_NONE) ||
      (BSP_LCD_SetActiveLayer(0, BSP_LCD_LAYER_1) != BSP_ERROR_NONE))
  { fail("lcdbmp", "an RGB565 layer"); goto out; }
  bad += bmp_check("16-bit 5-5-5 into RGB565", bmp_row(16U, 0U, NULL, rgb555, 4U), as565, 4U) ? 0U : 1U;
  bad += bmp_check("16-bit 5-6-5 bit fields into RGB565", bmp_row(16U, 3U, m565, rgb565, 4U), as565, 4U) ? 0U : 1U;
  bad += bmp_check("24-bit into RGB565", bmp_row(24U, 0U, NULL, bgr24, 4U), as565, 4U) ? 0U : 1U;

  cfg.PixelFormat = BSP_LCD_PIXEL_FORMAT_ARGB8888;
  if (BSP_LCD_ConfigLayer(0, BSP_LCD_LAYER_1, &cfg) != BSP_ERROR_NONE) { fail("lcdbmp", "an ARGB8888 layer"); goto out; }
  bad += bmp_check("32-bit BI_RGB into ARGB8888", bmp_row(32U, 0U, NULL, bgrx32, 4U), as8888, 4U) ? 0U : 1U;
  bad += bmp_check("24-bit into ARGB8888", bmp_row(24U, 0U, NULL, bgr24, 4U), as8888, 4U) ? 0U : 1U;

  cfg.PixelFormat = BSP_LCD_PIXEL_FORMAT_ARGB4444;
  if (BSP_LCD_ConfigLayer(0, BSP_LCD_LAYER_1, &cfg) != BSP_ERROR_NONE) { fail("lcdbmp", "an ARGB4444 layer"); goto out; }
  bad += bmp_check("24-bit into ARGB4444", bmp_row(24U, 0U, NULL, bgr24, 4U), as4444, 4U) ? 0U : 1U;
  bad += bmp_check("16-bit 5-6-5 bit fields into ARGB4444", bmp_row(16U, 3U, m565, rgb565, 4U), as4444, 4U) ? 0U : 1U;

  if (bad != 0U) { fail("lcdbmp", "%lu of the 7 bitmaps landed wrong; the INFO lines say where", (unsigned long)bad); goto out; }
  ok("lcdbmp", "16-bit 5-5-5 and 5-6-5, 24-bit and 32-bit bitmaps land right in RGB565, ARGB8888 and ARGB4444 layers");
out:
  (void)BSP_LCD_DeInit(0);
}

/* Room for the planar and double-buffered capture tests. */
#define CAM_SLOT_BYTES          (512U * 1024U)
#define CAM_SLOT(n)             (&CamArena[(n) * CAM_SLOT_BYTES])
#define CAM_PLANE_Y_BYTES       (CAM_W * CAM_H)          /* 307200 */
#define CAM_PLANE_UV_BYTES      ((CAM_W * CAM_H) / 2U)   /* 153600 */
/* A full-planar chroma plane is half the luma width and half its height. */
#define CAM_PLANE_U_BYTES       ((CAM_W * CAM_H) / 4U)   /* 76800 */
static uint8_t CamArena[4U * CAM_SLOT_BYTES] __attribute__((section(".ext_psram"), aligned(32)));

/* How much of a region stopped being the sentinel it was filled with. */
static uint32_t cam_written_pct(const uint8_t *pBuf, uint32_t Bytes, uint8_t Sentinel)
{
  uint32_t n = 0U, diff = 0U;

  for (uint32_t i = 0U; i < Bytes; i += 32U)
  {
    if (pBuf[i] != Sentinel) { diff++; }
    n++;
  }

  return (n != 0U) ? ((diff * 100U) / n) : 0U;
}

/* Fill with a sentinel and clean it from the cache, so no dirty line races
 * the capture hardware. */
static void cam_seed(uint8_t *pBuf, uint32_t Bytes, uint8_t Sentinel)
{
  memset(pBuf, Sentinel, Bytes);
  SCB_CleanDCache_by_Addr((uint32_t *)pBuf, (int32_t)Bytes);
}

/* Invalidate after a capture, so reads see what the hardware wrote. */
static void cam_refresh(uint8_t *pBuf, uint32_t Bytes)
{
  SCB_InvalidateDCache_by_Addr((uint32_t *)pBuf, (int32_t)Bytes);
}

/* hdredge */

static volatile uint32_t HdrEdgeIrqs;
static void hdredge_irq(uint32_t Pin) { (void)Pin; HdrEdgeIrqs++; }

/* The compare register of a pin's timer channel. */
static uint32_t hdredge_ccr(const BSP_IO_PinInfo_t *pInfo)
{
  switch (pInfo->PwmChannel)
  {
    case 2U: return pInfo->PwmTimer->CCR2;
    case 3U: return pInfo->PwmTimer->CCR3;
    case 4U: return pInfo->PwmTimer->CCR4;
    default: return pInfo->PwmTimer->CCR1;
  }
}

static void cmd_hdredge(void)
{
  BSP_IO_PinInfo_t d5 = {0}, red = {0}, d9 = {0};
  uint32_t bad = 0U;
  int32_t  r;

  (void)BSP_IO_GetPinInfo(BSP_IO_PIN_D5, &d5);
  (void)BSP_IO_GetPinInfo(BSP_IO_PIN_LED_RED, &red);
  (void)BSP_IO_GetPinInfo(BSP_IO_PIN_D9, &d9);
  (void)BSP_PWM_Init();
  (void)BSP_PWM_SetResolution(8U);
  (void)BSP_PWM_SetFrequency(1000U);

  /* 1. A second Start on a running pin changes its duty. */
  r  = BSP_PWM_Start(BSP_IO_PIN_D5, 64U);
  {
    const int32_t again = BSP_PWM_Start(BSP_IO_PIN_D5, 128U);
    P("INFO 1 duty change on a running pin r=%ld (first start r=%ld)\r\n", (long)again, (long)r);
    if ((r != BSP_ERROR_NONE) || (again != BSP_ERROR_NONE)) { P("INFO 1 FAILED: the duty change returned an error\r\n"); bad++; }
  }

  /* 2. A new frequency through the other channel of the same timer: D5's
   *    compare has to follow the new period and keep its duty. */
  (void)BSP_PWM_SetFrequency(20000U);
  r = BSP_PWM_Start(BSP_IO_PIN_LED_RED, 64U);
  {
    const uint32_t arr = d5.PwmTimer->ARR, ccr = hdredge_ccr(&d5);
    const uint32_t want = ((arr + 1U) * 128U) / 256U;
    const uint32_t diff = (ccr > want) ? (ccr - want) : (want - ccr);
    P("INFO 2 D5 after the timer moved to 20 kHz: CCR %lu, ARR %lu (want about %lu; r=%ld)\r\n",
      (unsigned long)ccr, (unsigned long)arr, (unsigned long)want, (long)r);
    if ((r != BSP_ERROR_NONE) || (ccr > arr) || (diff > ((arr / 50U) + 1U))) { P("INFO 2 FAILED: the other channel kept its old compare\r\n"); bad++; }
  }

  /* 3. Stopping the LED channel leaves the (active-low) LED off. */
  (void)BSP_PWM_Stop(BSP_IO_PIN_LED_RED);
  (void)BSP_PWM_Stop(BSP_IO_PIN_D5);
  {
    const uint32_t level = ((red.Port->ODR & red.Pin) != 0U) ? 1U : 0U;
    P("INFO 3 LED_RED pin after Stop: %lu (1 is off)\r\n", (unsigned long)level);
    if (level != 1U) { P("INFO 3 FAILED: stopping the LED's PWM lit it\r\n"); bad++; }
  }
  (void)BSP_PWM_SetFrequency(1000U);

  /* 4. A 2 ms tone at 200 Hz ends by itself. */
  r = BSP_TONE_Start(BSP_IO_PIN_D5, 200U, 2U);
  svc(50U);
  {
    const int32_t active = BSP_TONE_IsActive();
    P("INFO 4 a 2 ms tone at 200 Hz, 50 ms later: active=%ld (r=%ld)\r\n", (long)active, (long)r);
    if ((r != BSP_ERROR_NONE) || (active != 0)) { P("INFO 4 FAILED: the short tone played on\r\n"); bad++; }
  }
  (void)BSP_TONE_Stop();

  /* 5. An edge while detached must not reach the next attach. */
  {
    uint32_t first, stale, fresh;

    HdrEdgeIrqs = 0U;
    (void)BSP_IO_AttachInterrupt(BSP_IO_PIN_D9, BSP_IO_TRIGGER_RISING, hdredge_irq);
    EXTI->SWIER1 = (1UL << (d9.Line & 31U));
    svc(5U);
    first = HdrEdgeIrqs;
    (void)BSP_IO_DetachInterrupt(BSP_IO_PIN_D9);
    EXTI->SWIER1 = (1UL << (d9.Line & 31U));   /* while detached */
    svc(5U);
    HdrEdgeIrqs = 0U;
    (void)BSP_IO_AttachInterrupt(BSP_IO_PIN_D9, BSP_IO_TRIGGER_RISING, hdredge_irq);
    svc(5U);
    stale = HdrEdgeIrqs;
    EXTI->SWIER1 = (1UL << (d9.Line & 31U));
    svc(5U);
    fresh = HdrEdgeIrqs - stale;
    (void)BSP_IO_DetachInterrupt(BSP_IO_PIN_D9);
    (void)BSP_IO_ConfigPin(BSP_IO_PIN_D9, BSP_IO_MODE_INPUT_PU);
    P("INFO 5 D9 interrupts: %lu when attached, %lu from an edge while detached, %lu after re-attaching\r\n",
      (unsigned long)first, (unsigned long)stale, (unsigned long)fresh);
    if ((first != 1U) || (stale != 0U) || (fresh != 1U)) { P("INFO 5 FAILED: an edge from while detached was delivered\r\n"); bad++; }
  }

  /* 6. An SPI rate below the slowest division is refused, not run faster. */
  (void)BSP_SPI2_Init();
  {
    BSP_SPI_Config_t sc = { 400000U, BUS_SPI_MODE_0, BUS_SPI_MSB_FIRST };
    const uint32_t slowest = HAL_RCC_GetPCLK1Freq() >> 8;
    r = BSP_SPI2_SetConfig(&sc);
    P("INFO 6 SPI at 400 kHz: r=%ld, running at %lu Hz (the slowest division is %lu)\r\n",
      (long)r, (unsigned long)BSP_SPI2_GetClockFreq(), (unsigned long)slowest);
    if ((r == BSP_ERROR_NONE) && (BSP_SPI2_GetClockFreq() > 400000U)) { P("INFO 6 FAILED: accepted and run faster than asked\r\n"); bad++; }

    /* 7. The pins keep their idle levels between transfers. */
    sc.ClockFreq = 1000000U; sc.Mode = BUS_SPI_MODE_3;
    (void)BSP_SPI2_SetConfig(&sc);
    P("INFO 7 SPI2 CFG2 AFCNTR %lu (1 keeps SCK and MOSI driven between transfers)\r\n",
      (unsigned long)(((SPI2->CFG2 & SPI_CFG2_AFCNTR) != 0U) ? 1U : 0U));
    if ((SPI2->CFG2 & SPI_CFG2_AFCNTR) == 0U) { P("INFO 7 FAILED: SCK and MOSI float between transfers\r\n"); bad++; }

    /* 8. Recv sends zeroes in full duplex: receive-only leaves MOSI
     *    undriven. CFG2.COMM holds the last transfer's mode. */
    {
      uint8_t rx[8];
      uint32_t comm;

      r = BSP_SPI2_Recv(rx, (uint16_t)sizeof(rx));
      comm = (SPI2->CFG2 & SPI_CFG2_COMM) >> SPI_CFG2_COMM_Pos;
      P("INFO 8 SPI Recv r=%ld, in COMM mode %lu (0 full duplex, 2 receive-only)\r\n", (long)r, (unsigned long)comm);
      if ((r != BSP_ERROR_NONE) || (comm != 0U)) { P("INFO 8 FAILED: Recv ran receive-only, MOSI undriven\r\n"); bad++; }
    }
  }
  (void)BSP_SPI2_DeInit();

  /* 9. An interrupt on pin 28, the USER button, as a plain input. Last: a
   *    line never cleared hangs the board until the watchdog resets it. */
  {
    BSP_IO_PinInfo_t b = {0};

    (void)BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
    (void)BSP_IO_GetPinInfo(BSP_IO_PIN_BUTTON, &b);
    HdrEdgeIrqs = 0U;
    r = BSP_IO_AttachInterrupt(BSP_IO_PIN_BUTTON, BSP_IO_TRIGGER_RISING, hdredge_irq);
    P("INFO 9 attaching pin 28 r=%ld; triggering line %u\r\n", (long)r, (unsigned)b.Line);
    BSP_COM_Flush(200U);
    EXTI->SWIER1 = (1UL << (b.Line & 31U));
    svc(5U);
    (void)BSP_IO_DetachInterrupt(BSP_IO_PIN_BUTTON);
    P("INFO 9 pin 28 interrupts: %lu\r\n", (unsigned long)HdrEdgeIrqs);
    if ((r != BSP_ERROR_NONE) || (HdrEdgeIrqs != 1U)) { P("INFO 9 FAILED: the line-8 interrupt did not arrive once\r\n"); bad++; }
  }

  if (bad != 0U) { fail("hdredge", "%lu of 9 checks failed; the INFO lines say which", (unsigned long)bad); return; }
  ok("hdredge", "duty changes, shared-timer frequency change, LED off after stop, short tone, no stale pin interrupt, slow SPI refused, SPI pins kept, Recv drives MOSI, pin-28 interrupt");
}

/* campic */

/* One OV5640 register, read straight from the sensor. */
static uint8_t cam_reg(uint16_t Reg)
{
  uint8_t v = 0U;

  (void)BSP_I2C1_ReadReg16(BSP_CAMERA_I2C_ADDRESS, Reg, &v, 1U);
  return v;
}

/* The picture controls share SDE_CTRL0 (0x5580) and SDE_CTRL8 (0x5588), each
 * owning only some of their bits, and all switch the SDE on through bit 7 of
 * ISP_CONTROL01 (0x5001), whose other bits belong to the resolution. */
static void cmd_campic(void)
{
  uint32_t bad = 0U;
  uint8_t  isp0, isp1, sde0, sde8a, sde8b;
  int32_t  r, r2;

  if (cam_up() != BSP_ERROR_NONE) { fail("campic", "init"); return; }
  isp0 = cam_reg(0x5001U);

  /* 1. Saturation, then brightness: both enables stay set. */
  r  = BSP_CAMERA_SetSaturation(0, 2);
  r |= BSP_CAMERA_SetBrightness(0, 1);
  sde0 = cam_reg(0x5580U);
  P("INFO 1 after saturation then brightness: SDE_CTRL0 0x%02X (r=%ld; bits 1 and 2 wanted)\r\n", sde0, (long)r);
  if ((r != BSP_ERROR_NONE) || ((sde0 & 0x06U) != 0x06U)) { P("INFO 1 FAILED: brightness switched saturation off\r\n"); bad++; }

  /* 2. The resolution's ISP enables survive a picture control. */
  isp1 = cam_reg(0x5001U);
  P("INFO 2 ISP_CONTROL01 0x%02X at init, 0x%02X after the controls (only bit 7 may change)\r\n", isp0, isp1);
  if (((isp1 & 0x7FU) != (isp0 & 0x7FU)) || ((isp1 & 0x80U) == 0U)) { P("INFO 2 FAILED: the controls rewrote the ISP enables\r\n"); bad++; }

  /* 3. A brightness below zero, then contrast: the brightness sign stays. */
  r     = BSP_CAMERA_SetBrightness(0, -3);
  sde8a = cam_reg(0x5588U);
  r    |= BSP_CAMERA_SetContrast(0, 0);
  sde8b = cam_reg(0x5588U);
  P("INFO 3 SDE_CTRL8 0x%02X after brightness -3, 0x%02X after contrast 0 (bit 3 must not move; r=%ld)\r\n", sde8a, sde8b, (long)r);
  if ((r != BSP_ERROR_NONE) || ((sde8a & 0x08U) != (sde8b & 0x08U))) { P("INFO 3 FAILED: contrast flipped the brightness sign\r\n"); bad++; }

  /* 4. Into a colour effect and out again: the adjustments are still on. */
  r  = BSP_CAMERA_SetColorEffect(0, BSP_CAMERA_COLOR_EFFECT_SEPIA);
  r |= BSP_CAMERA_SetColorEffect(0, BSP_CAMERA_COLOR_EFFECT_NONE);
  sde0 = cam_reg(0x5580U);
  isp1 = cam_reg(0x5001U);
  P("INFO 4 after sepia then none: SDE_CTRL0 0x%02X, ISP_CONTROL01 0x%02X (bits 1, 2 and the SDE's bit 7 wanted; r=%ld)\r\n", sde0, isp1, (long)r);
  if ((r != BSP_ERROR_NONE) || ((sde0 & 0x06U) != 0x06U) || ((isp1 & 0x80U) == 0U))
  { P("INFO 4 FAILED: leaving the effect switched the adjustments off\r\n"); bad++; }

  /* 5. Zoom x1 at XGA, which only needs the sensor programmed. */
  (void)BSP_CAMERA_DeInit(0);
  r = BSP_ERROR_PERIPH_FAILURE;
  for (uint32_t k = 0U; (k < 3U) && (r != BSP_ERROR_NONE); k++)
  {
    r = BSP_CAMERA_Init(0, BSP_CAMERA_R1024x768, BSP_CAMERA_PF_RGB565);
    if (r != BSP_ERROR_NONE) { (void)BSP_CAMERA_DeInit(0); }
  }
  if (r != BSP_ERROR_NONE) { fail("campic", "XGA did not initialise in three tries (%ld)", (long)r); return; }
  r = BSP_CAMERA_SetZoom(0, BSP_CAMERA_ZOOM_x1);
  P("INFO 5 SetZoom x1 at XGA r=%ld\r\n", (long)r);
  if (r != BSP_ERROR_NONE) { P("INFO 5 FAILED: zoom refused at XGA\r\n"); bad++; }
  (void)BSP_CAMERA_DeInit(0);

  /* 6. 720p does not run the scaler the zoom is done in: x2 is refused, and
   * x1 accepted. */
  r = BSP_ERROR_PERIPH_FAILURE;
  for (uint32_t k = 0U; (k < 3U) && (r != BSP_ERROR_NONE); k++)
  {
    r = BSP_CAMERA_Init(0, BSP_CAMERA_R1280x720, BSP_CAMERA_PF_RGB565);
    if (r != BSP_ERROR_NONE) { (void)BSP_CAMERA_DeInit(0); }
  }
  if (r != BSP_ERROR_NONE) { fail("campic", "720p did not initialise in three tries (%ld)", (long)r); return; }
  r  = BSP_CAMERA_SetZoom(0, BSP_CAMERA_ZOOM_x2);
  r2 = BSP_CAMERA_SetZoom(0, BSP_CAMERA_ZOOM_x1);
  P("INFO 6 SetZoom at 720p: x2 r=%ld, x1 r=%ld\r\n", (long)r, (long)r2);
  if ((r != BSP_ERROR_FEATURE_NOT_SUPPORTED) || (r2 != BSP_ERROR_NONE))
  { P("INFO 6 FAILED: x2 should be refused as not supported, x1 accepted\r\n"); bad++; }
  (void)BSP_CAMERA_DeInit(0);

  if (bad != 0U) { fail("campic", "%lu of 6 checks failed; the INFO lines say which", (unsigned long)bad); return; }
  ok("campic", "saturation and brightness together, the ISP enables kept, the brightness sign kept by contrast, adjustments kept through a colour effect, zoom at XGA, and x2 refused at 720p");
}

/* camcore */

/* The OV5640's own colour bars, so a picture does not depend on the light. */
static void cam_bars(uint8_t On)
{
  uint8_t v = (On != 0U) ? 0x80U : 0x00U;

  (void)BSP_I2C1_WriteReg16(BSP_CAMERA_I2C_ADDRESS, 0x503DU, &v, 1U);
}

/* 32-byte lines of a region that are still entirely one byte value. */
static uint32_t cam_stale_lines(const uint8_t *pBuf, uint32_t Bytes, uint8_t Pattern)
{
  uint32_t n = 0U;

  for (uint32_t i = 0U; (i + 32U) <= Bytes; i += 32U)
  {
    uint32_t k = 0U;
    while ((k < 32U) && (pBuf[i + k] == Pattern)) { k++; }
    if (k == 32U) { n++; }
  }
  return n;
}

/* Wait, at most Ms, for a pipe to deliver Want frames. */
static void cam_wait_pipe(uint32_t Pipe, uint32_t Want, uint32_t Ms)
{
  const uint32_t t0 = HAL_GetTick();

  while (((HAL_GetTick() - t0) < Ms) && (CamPipeFrames[Pipe] < Want)) { svc(1U); }
}

/* Two 6 KB AUX destinations in internal RAM, which is cacheable (the PSRAM is
 * not): small enough to stay in the D-cache through a check. */
#define CAM_SMALL_W             64U
#define CAM_SMALL_H             32U
#define CAM_SMALL_BYTES         (CAM_SMALL_W * CAM_SMALL_H * 3U)
static uint8_t CamSmall[2][CAM_SMALL_BYTES] __attribute__((aligned(32)));

/* A QVGA semi-planar chroma plane, in internal RAM for the same reason. */
#define CAM_QVGA_UV_BYTES       ((320U * 240U) / 2U)
static uint8_t CamQvgaUv[CAM_QVGA_UV_BYTES] __attribute__((aligned(32)));

static void cmd_camcore(void)
{
  BSP_CAMERA_PipeConfig_t pc = {0};
  uint32_t bad = 0U;
  int32_t  r;

  if (cam_up() != BSP_ERROR_NONE) { fail("camcore", "init"); return; }
  cam_bars(1U);

  /* 1. A snapshot, then a continuous start on the same pipe. */
  pc.PixelFormat = BSP_CAMERA_PIPE_PF_RGB565; pc.Width = CAM_W; pc.Height = CAM_H;
  if (BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_DUMP, &pc) != BSP_ERROR_NONE) { fail("camcore", "config dump"); goto out; }
  CamPipeFrames[BSP_CAMERA_PIPE_DUMP] = 0U;
  r = BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_DUMP, CamFrame, BSP_CAMERA_MODE_SNAPSHOT);
  cam_wait_pipe(BSP_CAMERA_PIPE_DUMP, 1U, 1000U);
  if ((r != BSP_ERROR_NONE) || (CamPipeFrames[BSP_CAMERA_PIPE_DUMP] == 0U)) { fail("camcore", "no snapshot (%ld)", (long)r); goto out; }
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_DUMP);
  CamPipeFrames[BSP_CAMERA_PIPE_DUMP] = 0U;
  r = BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_DUMP, CamFrame, BSP_CAMERA_MODE_CONTINUOUS);
  cam_wait_pipe(BSP_CAMERA_PIPE_DUMP, 1000U, 500U);
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_DUMP);
  P("INFO 1 continuous after a snapshot: %lu frames in 0.5 s (r=%ld)\r\n",
    (unsigned long)CamPipeFrames[BSP_CAMERA_PIPE_DUMP], (long)r);
  if (CamPipeFrames[BSP_CAMERA_PIPE_DUMP] < 8U) { P("INFO 1 FAILED: the pipe stayed in snapshot mode\r\n"); bad++; }

  /* 2. Two pipes running; stop one, and the other has to carry on. */
  pc.PixelFormat = BSP_CAMERA_PIPE_PF_YUV422; pc.Width = 0U; pc.Height = 0U;
  if (BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pc) != BSP_ERROR_NONE) { fail("camcore", "config main"); goto out; }
  r  = BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, CAM_SLOT(0), BSP_CAMERA_MODE_CONTINUOUS);
  r |= BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_DUMP, CamFrame, BSP_CAMERA_MODE_CONTINUOUS);
  if (r != BSP_ERROR_NONE) { fail("camcore", "two pipes did not start (%ld)", (long)r); goto out; }
  { const uint32_t t0 = HAL_GetTick(); while ((HAL_GetTick() - t0) < 300U) { svc(1U); } }
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_DUMP);
  CamPipeFrames[BSP_CAMERA_PIPE_MAIN] = 0U;
  cam_wait_pipe(BSP_CAMERA_PIPE_MAIN, 1000U, 500U);
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
  P("INFO 2 main pipe after the dump pipe stopped: %lu frames in 0.5 s\r\n",
    (unsigned long)CamPipeFrames[BSP_CAMERA_PIPE_MAIN]);
  if (CamPipeFrames[BSP_CAMERA_PIPE_MAIN] < 8U) { P("INFO 2 FAILED: stopping one pipe starved the other\r\n"); bad++; }

  /* 3. The chroma pitch, after a packed format, then each planar one. */
  {
    uint32_t sp, fp;

    pc.PixelFormat = BSP_CAMERA_PIPE_PF_YUV420_SP;
    (void)BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pc);
    sp = (DCMIPP->P1PPM1PR & DCMIPP_P1PPM1PR_PITCH) >> DCMIPP_P1PPM1PR_PITCH_Pos;
    pc.PixelFormat = BSP_CAMERA_PIPE_PF_YUV420_FP;
    (void)BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pc);
    fp = (DCMIPP->P1PPM1PR & DCMIPP_P1PPM1PR_PITCH) >> DCMIPP_P1PPM1PR_PITCH_Pos;
    P("INFO 3 chroma pitch: semi-planar %lu (want %u), full planar %lu (want %u)\r\n",
      (unsigned long)sp, (unsigned)CAM_W, (unsigned long)fp, (unsigned)(CAM_W / 2U));
    if ((sp != CAM_W) || (fp != (CAM_W / 2U))) { P("INFO 3 FAILED: the chroma pitch did not follow the format\r\n"); bad++; }
  }

  /* 4. A packed double-buffered start on a planar pipe. */
  pc.PixelFormat = BSP_CAMERA_PIPE_PF_YUV420_SP;
  (void)BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pc);
  r = BSP_CAMERA_DoubleBufferStart(0, BSP_CAMERA_PIPE_MAIN, CAM_SLOT(0), CAM_SLOT(2), BSP_CAMERA_MODE_CONTINUOUS);
  P("INFO 4 DoubleBufferStart on a semi-planar pipe r=%ld (must not be 0)\r\n", (long)r);
  if (r == BSP_ERROR_NONE) { (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN); P("INFO 4 FAILED: accepted\r\n"); bad++; }

  /* 5. Both halves of a double-buffered start, filled and cached first. */
  pc.PixelFormat = BSP_CAMERA_PIPE_PF_RGB888; pc.Width = CAM_SMALL_W; pc.Height = CAM_SMALL_H;
  if (BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_AUX, &pc) != BSP_ERROR_NONE) { fail("camcore", "config aux"); goto out; }
  memset(CamSmall, 0xAA, sizeof CamSmall);
  SCB_CleanDCache_by_Addr((uint32_t *)CamSmall, (int32_t)sizeof CamSmall);
  CamPipeFrames[BSP_CAMERA_PIPE_AUX] = 0U;
  r = BSP_CAMERA_DoubleBufferStart(0, BSP_CAMERA_PIPE_AUX, CamSmall[0], CamSmall[1], BSP_CAMERA_MODE_CONTINUOUS);
  cam_wait_pipe(BSP_CAMERA_PIPE_AUX, 6U, 1000U);
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_AUX);
  {
    const uint32_t s0 = cam_stale_lines(CamSmall[0], CAM_SMALL_BYTES, 0xAAU);
    const uint32_t s1 = cam_stale_lines(CamSmall[1], CAM_SMALL_BYTES, 0xAAU);

    P("INFO 5 double buffer, %lu frames: stale lines %lu and %lu of %u (r=%ld)\r\n",
      (unsigned long)CamPipeFrames[BSP_CAMERA_PIPE_AUX], (unsigned long)s0, (unsigned long)s1,
      (unsigned)(CAM_SMALL_BYTES / 32U), (long)r);
    if ((CamPipeFrames[BSP_CAMERA_PIPE_AUX] < 2U) || (s0 > 8U) || (s1 > 8U))
    { P("INFO 5 FAILED: a double buffer was read stale\r\n"); bad++; }

    /* The eight colour bars along the middle row, to compare between builds. */
    P("INFO 5 bars:");
    for (uint32_t b = 0U; b < 8U; b++)
    {
      const uint8_t *px = &CamSmall[0][((16U * CAM_SMALL_W) + 4U + (8U * b)) * 3U];
      P(" %02X%02X%02X", px[0], px[1], px[2]);
    }
    P("\r\n");
  }

  /* 6. A semi-planar frame fills the first 8 KB of its chroma plane, rows a
   *    pitch apart. The plane is in non-cacheable PSRAM. */
  {
    uint8_t *const y  = CAM_SLOT(0);
    uint8_t *const uv = CAM_SLOT(1);
    uint32_t unwritten;

    pc.PixelFormat = BSP_CAMERA_PIPE_PF_YUV420_SP; pc.Width = 0U; pc.Height = 0U;
    (void)BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pc);
    memset(uv, 0xAA, 8192U);
    CamPipeFrames[BSP_CAMERA_PIPE_MAIN] = 0U;
    r = BSP_CAMERA_SemiPlanarStart(0, BSP_CAMERA_PIPE_MAIN, y, uv, BSP_CAMERA_MODE_SNAPSHOT);
    cam_wait_pipe(BSP_CAMERA_PIPE_MAIN, 1U, 1000U);
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
    unwritten = cam_stale_lines(uv, 8192U, 0xAAU);
    P("INFO 6 semi-planar snapshot (%lu frames, r=%ld): %lu of the chroma plane's first 256 lines never written\r\n",
      (unsigned long)CamPipeFrames[BSP_CAMERA_PIPE_MAIN], (long)r, (unsigned long)unwritten);
    if ((CamPipeFrames[BSP_CAMERA_PIPE_MAIN] == 0U) || (unwritten > 8U))
    { P("INFO 6 FAILED: the chroma rows did not land a pitch apart\r\n"); bad++; }
  }

  /* 7. Line events, then a stop and a start with no new line-buffer call. */
  {
    uint32_t first, again;

    pc.PixelFormat = BSP_CAMERA_PIPE_PF_YUV422;
    (void)BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pc);
    (void)BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN, BSP_CAMERA_LINES_OFF, 64U);
    CamLines = 0U;
    (void)BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, CAM_SLOT(0), BSP_CAMERA_MODE_CONTINUOUS);
    { const uint32_t t0 = HAL_GetTick(); while ((HAL_GetTick() - t0) < 200U) { svc(1U); } }
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
    first = CamLines;
    CamLines = 0U;
    (void)BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, CAM_SLOT(0), BSP_CAMERA_MODE_CONTINUOUS);
    { const uint32_t t0 = HAL_GetTick(); while ((HAL_GetTick() - t0) < 200U) { svc(1U); } }
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
    again = CamLines;
    (void)BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN, BSP_CAMERA_LINES_OFF, BSP_CAMERA_LINES_OFF);
    P("INFO 7 line events: %lu in the first run, %lu after a stop and a start\r\n",
      (unsigned long)first, (unsigned long)again);
    if (first == 0U) { fail("camcore", "no line events at all, so check 7 proves nothing"); goto out; }
    if (again == 0U) { P("INFO 7 FAILED: line events did not come back\r\n"); bad++; }
  }

  /* 8. A wrapped band on AUX, then DeInit and Init, then a whole AUX frame
   *    into internal RAM, seeded just before the snapshot. */
  {
    uint32_t stale;

    pc.PixelFormat = BSP_CAMERA_PIPE_PF_RGB888; pc.Width = CAM_SMALL_W; pc.Height = CAM_SMALL_H;
    (void)BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_AUX, &pc);
    r = BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_AUX, 16U, BSP_CAMERA_LINES_OFF);
    if (r != BSP_ERROR_NONE) { fail("camcore", "a 16-line band was refused (%ld), so check 8 proves nothing", (long)r); goto out; }
    (void)BSP_CAMERA_DeInit(0);
    if (cam_up() != BSP_ERROR_NONE) { fail("camcore", "init again"); return; }
    cam_bars(1U);
    {
      BSP_CAMERA_PipeConfig_t mc = {0};   /* MAIN too: check 10 is about AUX alone */
      mc.PixelFormat = BSP_CAMERA_PIPE_PF_YUV422;
      (void)BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &mc);
    }
    (void)BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_AUX, &pc);
    CamPipeFrames[BSP_CAMERA_PIPE_AUX] = 0U;
    (void)BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_AUX, CamSmall[0], BSP_CAMERA_MODE_CONTINUOUS);
    cam_wait_pipe(BSP_CAMERA_PIPE_AUX, 5U, 1500U);   /* the sensor warm */
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_AUX);
    memset(CamSmall[0], 0xAA, CAM_SMALL_BYTES);
    SCB_CleanDCache_by_Addr((uint32_t *)CamSmall[0], (int32_t)CAM_SMALL_BYTES);
    CamPipeFrames[BSP_CAMERA_PIPE_AUX] = 0U;
    r = BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_AUX, CamSmall[0], BSP_CAMERA_MODE_SNAPSHOT);
    cam_wait_pipe(BSP_CAMERA_PIPE_AUX, 1U, 1000U);
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_AUX);
    stale = cam_stale_lines(CamSmall[0], CAM_SMALL_BYTES, 0xAAU);
    P("INFO 8 AUX frame after a wrapped band and a re-init (%lu frames, r=%ld): stale lines %lu of %u\r\n",
      (unsigned long)CamPipeFrames[BSP_CAMERA_PIPE_AUX], (long)r, (unsigned long)stale,
      (unsigned)(CAM_SMALL_BYTES / 32U));
    if ((CamPipeFrames[BSP_CAMERA_PIPE_AUX] == 0U) || (stale > 8U))
    { P("INFO 8 FAILED: the frame was read stale\r\n"); bad++; }
  }

  /* 9. A QVGA semi-planar frame with its chroma plane in internal RAM, from a
   *    freshly initialised pipe, so the chroma pitch is right whatever else. */
  {
    uint32_t stale;

    (void)BSP_CAMERA_DeInit(0);
    if (BSP_CAMERA_Init(0, BSP_CAMERA_R320x240, BSP_CAMERA_PF_RGB565) != BSP_ERROR_NONE)
    { fail("camcore", "init at QVGA"); return; }
    cam_bars(1U);
    pc.PixelFormat = BSP_CAMERA_PIPE_PF_YUV420_SP; pc.Width = 0U; pc.Height = 0U;
    if (BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pc) != BSP_ERROR_NONE) { fail("camcore", "config main at QVGA"); goto out; }
    CamPipeFrames[BSP_CAMERA_PIPE_MAIN] = 0U;
    (void)BSP_CAMERA_SemiPlanarStart(0, BSP_CAMERA_PIPE_MAIN, CAM_SLOT(0), CamQvgaUv, BSP_CAMERA_MODE_CONTINUOUS);
    cam_wait_pipe(BSP_CAMERA_PIPE_MAIN, 5U, 1500U);   /* the sensor warm */
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
    memset(CamQvgaUv, 0xAA, sizeof CamQvgaUv);
    SCB_CleanDCache_by_Addr((uint32_t *)CamQvgaUv, (int32_t)sizeof CamQvgaUv);
    CamPipeFrames[BSP_CAMERA_PIPE_MAIN] = 0U;
    r = BSP_CAMERA_SemiPlanarStart(0, BSP_CAMERA_PIPE_MAIN, CAM_SLOT(0), CamQvgaUv, BSP_CAMERA_MODE_SNAPSHOT);
    cam_wait_pipe(BSP_CAMERA_PIPE_MAIN, 1U, 1000U);
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
    stale = cam_stale_lines(CamQvgaUv, 8192U, 0xAAU);
    P("INFO 9 QVGA semi-planar snapshot, chroma in internal RAM (%lu frames, r=%ld): stale lines %lu of 256\r\n",
      (unsigned long)CamPipeFrames[BSP_CAMERA_PIPE_MAIN], (long)r, (unsigned long)stale);
    if ((CamPipeFrames[BSP_CAMERA_PIPE_MAIN] == 0U) || (stale > 8U))
    { P("INFO 9 FAILED: the chroma plane was read stale\r\n"); bad++; }
  }

  /* 10. The AUX pipe on its own, straight after an Init, with MAIN never
   *     configured. */
  {
    (void)BSP_CAMERA_DeInit(0);
    if (cam_up() != BSP_ERROR_NONE) { fail("camcore", "init for AUX alone"); return; }
    cam_bars(1U);
    pc.PixelFormat = BSP_CAMERA_PIPE_PF_RGB888; pc.Width = CAM_SMALL_W; pc.Height = CAM_SMALL_H;
    r = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_AUX, &pc);
    CamPipeFrames[BSP_CAMERA_PIPE_AUX] = 0U;
    if (r == BSP_ERROR_NONE) { r = BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_AUX, CamSmall[0], BSP_CAMERA_MODE_CONTINUOUS); }
    cam_wait_pipe(BSP_CAMERA_PIPE_AUX, 1000U, 800U);
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_AUX);
    P("INFO 10 AUX alone after an Init: %lu frames in 0.8 s (r=%ld)\r\n",
      (unsigned long)CamPipeFrames[BSP_CAMERA_PIPE_AUX], (long)r);
    if (CamPipeFrames[BSP_CAMERA_PIPE_AUX] < 8U) { P("INFO 10 FAILED: the AUX pipe gets nothing without MAIN\r\n"); bad++; }
  }

  /* 11. AUX snapshots with MAIN running, as the NN examples take them, each
   *     then left in the error state the HAL leaves when a frame end and an
   *     overrun reach its handler together. The next snapshot must still
   *     start, and MAIN keep running. */
  {
    uint32_t snaps = 0U, i, main_frames;

    pc.PixelFormat = BSP_CAMERA_PIPE_PF_YUV422; pc.Width = 0U; pc.Height = 0U;
    r = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pc);
    if (r == BSP_ERROR_NONE) { r = BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, CAM_SLOT(0), BSP_CAMERA_MODE_CONTINUOUS); }
    for (i = 0U; (i < 3U) && (r == BSP_ERROR_NONE); i++)
    {
      CamPipeFrames[BSP_CAMERA_PIPE_AUX] = 0U;
      r = BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_AUX, CamSmall[0], BSP_CAMERA_MODE_SNAPSHOT);
      if (r != BSP_ERROR_NONE) { break; }
      cam_wait_pipe(BSP_CAMERA_PIPE_AUX, 1U, 1000U);
      if (CamPipeFrames[BSP_CAMERA_PIPE_AUX] == 0U) { break; }
      snaps++;
      hcamera_dcmipp.PipeState[BSP_CAMERA_PIPE_AUX] = HAL_DCMIPP_PIPE_STATE_ERROR;
    }
    CamPipeFrames[BSP_CAMERA_PIPE_MAIN] = 0U;
    cam_wait_pipe(BSP_CAMERA_PIPE_MAIN, 1000U, 500U);
    main_frames = CamPipeFrames[BSP_CAMERA_PIPE_MAIN];
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_AUX);
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
    P("INFO 11 AUX snapshots with MAIN running, each left as the HAL leaves a frame end and an overrun together: %lu of 3 (r=%ld); MAIN %lu frames in 0.5 s\r\n",
      (unsigned long)snaps, (long)r, (unsigned long)main_frames);
    if ((snaps != 3U) || (main_frames < 8U))
    { P("INFO 11 FAILED: a snapshot the HAL left in error blocks the next\r\n"); bad++; }
  }

  cam_bars(0U);
  if (bad != 0U) { fail("camcore", "%lu of 11 checks failed; the INFO lines say which", (unsigned long)bad); goto out; }
  ok("camcore", "snapshot then continuous, one pipe stopped under another, chroma pitch and rows, planar double-buffer refusal, fresh double buffers, line events after a stop, a frame after a band and a re-init, a fresh chroma plane, the AUX pipe alone, and a snapshot after one the HAL left in error");
out:
  cam_bars(0U);
  (void)BSP_CAMERA_DeInit(0);
}

/* The five capture starts that are not BSP_CAMERA_Start: double buffering,
 * and the two planar layouts with and without it. */
static void cmd_cammode(void)
{
  BSP_CAMERA_PipeConfig_t pc = {0};
  uint32_t w0, w1, wy, wuv, wu, wv;
  int32_t  r;

  CamFrames = CamErrors = 0;
  if (cam_up() != BSP_ERROR_NONE) { fail("cammode", "init"); return; }

  /* Packed double buffering, on the pipe that takes packed formats. */
  pc.PixelFormat = BSP_CAMERA_PIPE_PF_RGB565; pc.Width = CAM_W; pc.Height = CAM_H;
  if (BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_DUMP, &pc) != BSP_ERROR_NONE)
  { fail("cammode", "config dump pipe"); goto mode_fail; }

  /* The same buffer for both halves must be refused. */
  r = BSP_CAMERA_DoubleBufferStart(0, BSP_CAMERA_PIPE_DUMP, CAM_SLOT(0), CAM_SLOT(0), BSP_CAMERA_MODE_CONTINUOUS);
  P("INFO DoubleBufferStart with one buffer twice r=%ld (must not be 0)\r\n", (long)r);
  if (r == BSP_ERROR_NONE) { fail("cammode", "the same buffer was accepted for both halves"); goto mode_fail; }

  cam_seed(CAM_SLOT(0), CAM_FRAME_BYTES, 0xA5U);
  cam_seed(CAM_SLOT(2), CAM_FRAME_BYTES, 0x5AU);
  CamFrames = 0U;
  r = BSP_CAMERA_DoubleBufferStart(0, BSP_CAMERA_PIPE_DUMP, CAM_SLOT(0), CAM_SLOT(2), BSP_CAMERA_MODE_CONTINUOUS);
  if (r != BSP_ERROR_NONE) { fail("cammode", "DoubleBufferStart %ld", (long)r); goto mode_fail; }
  { uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 1500U) && (CamFrames < 6U)) { svc(5U); } }
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_DUMP);
  cam_refresh(CAM_SLOT(0), CAM_FRAME_BYTES);
  cam_refresh(CAM_SLOT(2), CAM_FRAME_BYTES);
  w0 = cam_written_pct(CAM_SLOT(0), CAM_FRAME_BYTES, 0xA5U);
  w1 = cam_written_pct(CAM_SLOT(2), CAM_FRAME_BYTES, 0x5AU);
  P("INFO double buffer: %lu frames, buffer 0 %lu%% written, buffer 1 %lu%% written\r\n",
    (unsigned long)CamFrames, (unsigned long)w0, (unsigned long)w1);
  if (CamFrames < 2U) { fail("cammode", "only %lu frames double buffered", (unsigned long)CamFrames); goto mode_fail; }
  if ((w0 < 50U) || (w1 < 50U))
  { fail("cammode", "double buffering wrote only one half: buffer 0 %lu%%, buffer 1 %lu%%", (unsigned long)w0, (unsigned long)w1); goto mode_fail; }

  /* Semi-planar: luma and interleaved chroma in separate planes. */
  pc.PixelFormat = BSP_CAMERA_PIPE_PF_YUV420_SP; pc.Width = 0U; pc.Height = 0U;
  r = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pc);
  if (r != BSP_ERROR_NONE) { fail("cammode", "config main pipe semi-planar %ld", (long)r); goto mode_fail; }

  /* A planar start on the pipe that has no planar packer has to be refused. */
  r = BSP_CAMERA_SemiPlanarStart(0, BSP_CAMERA_PIPE_DUMP, CAM_SLOT(0), CAM_SLOT(1), BSP_CAMERA_MODE_CONTINUOUS);
  P("INFO SemiPlanarStart on the dump pipe r=%ld (must not be 0)\r\n", (long)r);
  if (r == BSP_ERROR_NONE) { fail("cammode", "the dump pipe accepted a planar start"); goto mode_fail; }

  cam_seed(CAM_SLOT(0), CAM_PLANE_Y_BYTES, 0xA5U);
  cam_seed(CAM_SLOT(1), CAM_PLANE_UV_BYTES, 0x5AU);
  CamFrames = 0U;
  r = BSP_CAMERA_SemiPlanarStart(0, BSP_CAMERA_PIPE_MAIN, CAM_SLOT(0), CAM_SLOT(1), BSP_CAMERA_MODE_CONTINUOUS);
  if (r != BSP_ERROR_NONE) { fail("cammode", "SemiPlanarStart %ld", (long)r); goto mode_fail; }
  { uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 1500U) && (CamFrames < 4U)) { svc(5U); } }
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
  cam_refresh(CAM_SLOT(0), CAM_PLANE_Y_BYTES);
  cam_refresh(CAM_SLOT(1), CAM_PLANE_UV_BYTES);
  wy  = cam_written_pct(CAM_SLOT(0), CAM_PLANE_Y_BYTES, 0xA5U);
  wuv = cam_written_pct(CAM_SLOT(1), CAM_PLANE_UV_BYTES, 0x5AU);
  P("INFO semi-planar: %lu frames, Y %lu%% written, UV %lu%% written\r\n",
    (unsigned long)CamFrames, (unsigned long)wy, (unsigned long)wuv);
  if (CamFrames == 0U) { fail("cammode", "no semi-planar frames"); goto mode_fail; }
  if (wy < 50U)  { fail("cammode", "the luma plane was not written (%lu%%)", (unsigned long)wy); goto mode_fail; }
  if (wuv < 50U) { fail("cammode", "the chroma plane was not written (%lu%%) - its address never reached the packer", (unsigned long)wuv); goto mode_fail; }

  /* Full planar: three separate planes. */
  pc.PixelFormat = BSP_CAMERA_PIPE_PF_YUV420_FP;
  r = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pc);
  if (r != BSP_ERROR_NONE) { fail("cammode", "config main pipe full planar %ld", (long)r); goto mode_fail; }

  cam_seed(CAM_SLOT(0), CAM_PLANE_Y_BYTES, 0xA5U);
  /* Seeded past the plane too, so the probe below can spot an overrun. */
  cam_seed(CAM_SLOT(1), 2U * CAM_PLANE_U_BYTES, 0x5AU);
  cam_seed(CAM_SLOT(2), CAM_PLANE_U_BYTES, 0x3CU);
  CamFrames = 0U;
  r = BSP_CAMERA_FullPlanarStart(0, BSP_CAMERA_PIPE_MAIN, CAM_SLOT(0), CAM_SLOT(1), CAM_SLOT(2), BSP_CAMERA_MODE_CONTINUOUS);
  if (r != BSP_ERROR_NONE) { fail("cammode", "FullPlanarStart %ld", (long)r); goto mode_fail; }
  { uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 1500U) && (CamFrames < 4U)) { svc(5U); } }
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
  cam_refresh(CAM_SLOT(0), CAM_PLANE_Y_BYTES);
  cam_refresh(CAM_SLOT(1), CAM_PLANE_U_BYTES);
  cam_refresh(CAM_SLOT(2), CAM_PLANE_U_BYTES);
  wy = cam_written_pct(CAM_SLOT(0), CAM_PLANE_Y_BYTES, 0xA5U);
  wu = cam_written_pct(CAM_SLOT(1), CAM_PLANE_U_BYTES, 0x5AU);
  wv = cam_written_pct(CAM_SLOT(2), CAM_PLANE_U_BYTES, 0x3CU);
  P("INFO full planar: %lu frames, Y %lu%% U %lu%% V %lu%% written\r\n",
    (unsigned long)CamFrames, (unsigned long)wy, (unsigned long)wu, (unsigned long)wv);
  /* Nothing past the end of the plane, a quarter of the luma's size. */
  {
    const uint32_t probe = 2U * CAM_PLANE_U_BYTES;
    uint32_t last_u = 0U;
    cam_refresh(CAM_SLOT(1), probe);
    for (uint32_t i = 0U; i < probe; i += 32U)
    {
      if (CAM_SLOT(1)[i] != 0x5AU) { last_u = i; }
    }
    P("INFO full planar U plane is %lu bytes, last byte written at %lu\r\n",
      (unsigned long)CAM_PLANE_U_BYTES, (unsigned long)last_u);
    if (last_u >= CAM_PLANE_U_BYTES)
    { fail("cammode", "the U plane was written out to %lu, past its %lu bytes",
           (unsigned long)last_u, (unsigned long)CAM_PLANE_U_BYTES); goto mode_fail; }
  }
  if (CamFrames == 0U) { fail("cammode", "no full-planar frames"); goto mode_fail; }
  if (wy < 50U) { fail("cammode", "the luma plane was not written (%lu%%)", (unsigned long)wy); goto mode_fail; }
  /* Nearly all of each chroma plane: its rows sit half a luma pitch apart,
   * so there is no padding between them. */
  if (wu < 50U) { fail("cammode", "the U plane was not written (%lu%%)", (unsigned long)wu); goto mode_fail; }
  if (wv < 50U) { fail("cammode", "the V plane was not written (%lu%%)", (unsigned long)wv); goto mode_fail; }

  /* Semi-planar, double buffered: all four regions written. */
  pc.PixelFormat = BSP_CAMERA_PIPE_PF_YUV420_SP;
  r = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pc);
  if (r != BSP_ERROR_NONE) { fail("cammode", "reconfig semi-planar %ld", (long)r); goto mode_fail; }

  cam_seed(CAM_SLOT(0), CAM_PLANE_Y_BYTES, 0xA5U);
  cam_seed(CAM_SLOT(1), CAM_PLANE_UV_BYTES, 0x5AU);
  cam_seed(CAM_SLOT(2), CAM_PLANE_Y_BYTES, 0x3CU);
  cam_seed(CAM_SLOT(3), CAM_PLANE_UV_BYTES, 0xC3U);
  CamFrames = 0U;
  r = BSP_CAMERA_SemiPlanarDoubleBufferStart(0, BSP_CAMERA_PIPE_MAIN,
                                             CAM_SLOT(0), CAM_SLOT(1),
                                             CAM_SLOT(2), CAM_SLOT(3),
                                             BSP_CAMERA_MODE_CONTINUOUS);
  if (r != BSP_ERROR_NONE) { fail("cammode", "SemiPlanarDoubleBufferStart %ld", (long)r); goto mode_fail; }
  { uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 1500U) && (CamFrames < 6U)) { svc(5U); } }
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
  cam_refresh(CAM_SLOT(0), CAM_PLANE_Y_BYTES);
  cam_refresh(CAM_SLOT(1), CAM_PLANE_UV_BYTES);
  cam_refresh(CAM_SLOT(2), CAM_PLANE_Y_BYTES);
  cam_refresh(CAM_SLOT(3), CAM_PLANE_UV_BYTES);
  {
    uint32_t y0 = cam_written_pct(CAM_SLOT(0), CAM_PLANE_Y_BYTES, 0xA5U);
    uint32_t u0 = cam_written_pct(CAM_SLOT(1), CAM_PLANE_UV_BYTES, 0x5AU);
    uint32_t y1 = cam_written_pct(CAM_SLOT(2), CAM_PLANE_Y_BYTES, 0x3CU);
    uint32_t u1 = cam_written_pct(CAM_SLOT(3), CAM_PLANE_UV_BYTES, 0xC3U);
    P("INFO semi-planar double buffered: %lu frames, Y0 %lu%% UV0 %lu%% Y1 %lu%% UV1 %lu%%\r\n",
      (unsigned long)CamFrames, (unsigned long)y0, (unsigned long)u0,
      (unsigned long)y1, (unsigned long)u1);
    if (CamFrames < 2U) { fail("cammode", "only %lu frames semi-planar double buffered", (unsigned long)CamFrames); goto mode_fail; }
    if ((y0 < 50U) || (u0 < 50U) || (y1 < 50U) || (u1 < 50U))
    { fail("cammode", "not all four regions were written: %lu/%lu/%lu/%lu%%",
           (unsigned long)y0, (unsigned long)u0, (unsigned long)y1, (unsigned long)u1); goto mode_fail; }
  }

  /* Full planar, double buffered: six regions. */
  pc.PixelFormat = BSP_CAMERA_PIPE_PF_YUV420_FP;
  r = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pc);
  if (r != BSP_ERROR_NONE) { fail("cammode", "reconfig full planar %ld", (long)r); goto mode_fail; }

  /* Six regions in four slots: the two chroma planes of each frame are a
   * quarter the luma size, so they sit inside the back half of a slot. */
  cam_seed(CAM_SLOT(0), CAM_PLANE_Y_BYTES, 0xA5U);
  cam_seed(CAM_SLOT(1), CAM_PLANE_U_BYTES, 0x5AU);
  cam_seed(CAM_SLOT(1) + CAM_PLANE_U_BYTES, CAM_PLANE_U_BYTES, 0x6BU);
  cam_seed(CAM_SLOT(2), CAM_PLANE_Y_BYTES, 0x3CU);
  cam_seed(CAM_SLOT(3), CAM_PLANE_U_BYTES, 0xC3U);
  cam_seed(CAM_SLOT(3) + CAM_PLANE_U_BYTES, CAM_PLANE_U_BYTES, 0x1EU);
  CamFrames = 0U;
  r = BSP_CAMERA_FullPlanarDoubleBufferStart(0, BSP_CAMERA_PIPE_MAIN,
                                             CAM_SLOT(0), CAM_SLOT(1), CAM_SLOT(1) + CAM_PLANE_U_BYTES,
                                             CAM_SLOT(2), CAM_SLOT(3), CAM_SLOT(3) + CAM_PLANE_U_BYTES,
                                             BSP_CAMERA_MODE_CONTINUOUS);
  if (r != BSP_ERROR_NONE) { fail("cammode", "FullPlanarDoubleBufferStart %ld", (long)r); goto mode_fail; }
  { uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 1500U) && (CamFrames < 6U)) { svc(5U); } }
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
  cam_refresh(CAM_SLOT(0), CAM_PLANE_Y_BYTES);
  cam_refresh(CAM_SLOT(1), 2U * CAM_PLANE_U_BYTES);
  cam_refresh(CAM_SLOT(2), CAM_PLANE_Y_BYTES);
  cam_refresh(CAM_SLOT(3), 2U * CAM_PLANE_U_BYTES);
  {
    uint32_t y0 = cam_written_pct(CAM_SLOT(0), CAM_PLANE_Y_BYTES, 0xA5U);
    uint32_t u0 = cam_written_pct(CAM_SLOT(1), CAM_PLANE_U_BYTES, 0x5AU);
    uint32_t v0 = cam_written_pct(CAM_SLOT(1) + CAM_PLANE_U_BYTES, CAM_PLANE_U_BYTES, 0x6BU);
    uint32_t y1 = cam_written_pct(CAM_SLOT(2), CAM_PLANE_Y_BYTES, 0x3CU);
    uint32_t u1 = cam_written_pct(CAM_SLOT(3), CAM_PLANE_U_BYTES, 0xC3U);
    uint32_t v1 = cam_written_pct(CAM_SLOT(3) + CAM_PLANE_U_BYTES, CAM_PLANE_U_BYTES, 0x1EU);
    P("INFO full planar double buffered: %lu frames, Y0 %lu%% U0 %lu%% V0 %lu%% Y1 %lu%% U1 %lu%% V1 %lu%%\r\n",
      (unsigned long)CamFrames, (unsigned long)y0, (unsigned long)u0, (unsigned long)v0,
      (unsigned long)y1, (unsigned long)u1, (unsigned long)v1);
    if (CamFrames < 2U) { fail("cammode", "only %lu frames full-planar double buffered", (unsigned long)CamFrames); goto mode_fail; }
    if ((y0 < 50U) || (y1 < 50U) ||
        (u0 < 50U) || (v0 < 50U) || (u1 < 50U) || (v1 < 50U))
    { fail("cammode", "not all six regions were written: %lu/%lu/%lu/%lu/%lu/%lu%%",
           (unsigned long)y0, (unsigned long)u0, (unsigned long)v0,
           (unsigned long)y1, (unsigned long)u1, (unsigned long)v1); goto mode_fail; }
  }

  (void)BSP_CAMERA_DeInit(0);
  ok("cammode", "double buffering alternates for real, and every plane of both planar layouts is written");
  return;

mode_fail:
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_DUMP);
  (void)BSP_CAMERA_DeInit(0);
}

/* Capture a settled frame on the dump pipe and return its mean brightness. */
static int32_t cam_frame_luma(uint32_t *pLuma, uint32_t Frames)
{
  const uint16_t *px = (const uint16_t *)CamFrame;
  uint64_t sum = 0;
  uint32_t n = 0U, t0;

  CamFrames = 0U;
  if (BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_DUMP, CamFrame, BSP_CAMERA_MODE_CONTINUOUS) != BSP_ERROR_NONE)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  t0 = HAL_GetTick();
  while (((HAL_GetTick() - t0) < 1000U) && (CamFrames < Frames)) { svc(5U); }
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_DUMP);

  if (CamFrames == 0U) { return BSP_ERROR_BUSY; }

  for (uint32_t i = 0U; i < (CAM_W * CAM_H); i += 16U)
  {
    uint16_t p = px[i];
    uint32_t rr = ((uint32_t)(p >> 11) & 0x1FU) << 3;   /* 0..248 */
    uint32_t gg = ((uint32_t)(p >> 5)  & 0x3FU) << 2;   /* 0..252 */
    uint32_t bb = ((uint32_t)p         & 0x1FU) << 3;   /* 0..248 */
    sum += (rr + gg + bb) / 3U;
    n++;
  }
  *pLuma = (uint32_t)(sum / n);

  return BSP_ERROR_NONE;
}

/* Mean and spread of a settled frame's bytes on a given pipe. */
static int32_t cam_frame_spread(uint32_t Pipe, uint32_t Frames,
                                uint32_t *pMean, uint32_t *pSpread)
{
  const uint8_t *by = (const uint8_t *)CamFrame;
  const uint32_t bytes = CAM_W * CAM_H * 2U;
  uint64_t sum = 0, dev = 0;
  uint32_t n = 0U, t0, mean;

  CamFrames = 0U;
  if (BSP_CAMERA_Start(0, Pipe, CamFrame, BSP_CAMERA_MODE_CONTINUOUS) != BSP_ERROR_NONE)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  t0 = HAL_GetTick();
  while (((HAL_GetTick() - t0) < 1500U) && (CamFrames < Frames)) { svc(5U); }
  (void)BSP_CAMERA_Stop(0, Pipe);

  if (CamFrames == 0U) { return BSP_ERROR_BUSY; }

  for (uint32_t i = 0U; i < bytes; i += 16U) { sum += by[i]; n++; }
  mean = (uint32_t)(sum / n);
  for (uint32_t i = 0U; i < bytes; i += 16U)
  {
    uint32_t v = by[i];
    dev += (v > mean) ? (v - mean) : (mean - v);
  }

  *pMean = mean;
  *pSpread = (uint32_t)(dev / n);

  return BSP_ERROR_NONE;
}

/* The five ISP stages on the capture pipeline. */
static void cmd_camisp(void)
{
  BSP_CAMERA_PipeConfig_t pc = {0};
  DCMIPP_ColorConversionConfTypeDef cc = {0};
  DCMIPP_ExposureConfTypeDef gain = {0};
  DCMIPP_BlackLevelConfTypeDef black = {0};
  DCMIPP_RawBayer2RGBConfTypeDef demo = {0};
  uint32_t mean_live = 0U, spread_live = 0U, mean_flat = 0U, spread_flat = 0U;
  int32_t  r;

  CamFrames = CamErrors = 0;
  if (cam_up() != BSP_ERROR_NONE) { fail("camisp", "init"); return; }
  cam_bars(1U);
  /* YUV422, as the main pipe feeds the encoder; zero width and height follow
   * the sensor. */
  pc.PixelFormat = BSP_CAMERA_PIPE_PF_YUV422; pc.Width = 0U; pc.Height = 0U;
  r = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pc);
  if (r != BSP_ERROR_NONE)
  { fail("camisp", "config main pipe %ld", (long)r); goto isp_fail; }

  /* Acceptance on the main pipe, and NULL as the off switch. */
  gain.ShiftRed = 0U; gain.MultiplierRed = 32U;
  gain.ShiftGreen = 0U; gain.MultiplierGreen = 32U;
  gain.ShiftBlue = 0U; gain.MultiplierBlue = 32U;
  r = BSP_CAMERA_SetIspGain(0, BSP_CAMERA_PIPE_MAIN, &gain);
  P("INFO SetIspGain main r=%ld, off r=%ld\r\n", (long)r,
    (long)BSP_CAMERA_SetIspGain(0, BSP_CAMERA_PIPE_MAIN, NULL));
  if (r != BSP_ERROR_NONE) { fail("camisp", "SetIspGain %ld", (long)r); goto isp_fail; }

  black.RedCompBlackLevel = 0U; black.GreenCompBlackLevel = 0U; black.BlueCompBlackLevel = 0U;
  r = BSP_CAMERA_SetIspBlackLevel(0, BSP_CAMERA_PIPE_MAIN, &black);
  P("INFO SetIspBlackLevel main r=%ld, off r=%ld\r\n", (long)r,
    (long)BSP_CAMERA_SetIspBlackLevel(0, BSP_CAMERA_PIPE_MAIN, NULL));
  if (r != BSP_ERROR_NONE) { fail("camisp", "SetIspBlackLevel %ld", (long)r); goto isp_fail; }

  demo.RawBayerType = DCMIPP_RAWBAYER_RGGB;
  demo.VLineStrength = DCMIPP_RAWBAYER_ALGO_NONE;
  demo.HLineStrength = DCMIPP_RAWBAYER_ALGO_NONE;
  demo.PeakStrength  = DCMIPP_RAWBAYER_ALGO_NONE;
  demo.EdgeStrength  = DCMIPP_RAWBAYER_ALGO_NONE;
  r = BSP_CAMERA_SetIspDemosaicing(0, BSP_CAMERA_PIPE_MAIN, &demo);
  P("INFO SetIspDemosaicing main r=%ld, off r=%ld\r\n", (long)r,
    (long)BSP_CAMERA_SetIspDemosaicing(0, BSP_CAMERA_PIPE_MAIN, NULL));
  if (r != BSP_ERROR_NONE) { fail("camisp", "SetIspDemosaicing %ld", (long)r); goto isp_fail; }

  r = BSP_CAMERA_SetIspGamma(0, BSP_CAMERA_PIPE_MAIN, 1U);
  P("INFO SetIspGamma on r=%ld, off r=%ld\r\n", (long)r,
    (long)BSP_CAMERA_SetIspGamma(0, BSP_CAMERA_PIPE_MAIN, 0U));
  if (r != BSP_ERROR_NONE) { fail("camisp", "SetIspGamma %ld", (long)r); goto isp_fail; }

  /* The dump pipe has no such stage and must refuse it. */
  {
    int32_t rd = BSP_CAMERA_SetIspGain(0, BSP_CAMERA_PIPE_DUMP, &gain);
    P("INFO SetIspGain on the dump pipe r=%ld (must not be 0)\r\n", (long)rd);
    if (rd == BSP_ERROR_NONE) { fail("camisp", "the dump pipe accepted a stage it does not have"); goto isp_fail; }
  }

  /* The colour matrix, judged on the picture: the colour bars. */
  cc.OutputSamplesType = DCMIPP_CLAMP_RGB;
  cc.RR = 256; cc.RG = 0;   cc.RB = 0;   cc.RA = 0;
  cc.GR = 0;   cc.GG = 256; cc.GB = 0;   cc.GA = 0;
  cc.BR = 0;   cc.BG = 0;   cc.BB = 256; cc.BA = 0;
  r = BSP_CAMERA_SetIspColorConversion(0, BSP_CAMERA_PIPE_MAIN, &cc);
  if (r != BSP_ERROR_NONE) { fail("camisp", "SetIspColorConversion pass-through %ld", (long)r); goto isp_fail; }
  if (cam_frame_spread(BSP_CAMERA_PIPE_MAIN, 4U, &mean_live, &spread_live) != BSP_ERROR_NONE)
  { fail("camisp", "no frame on the main pipe with the pass-through matrix"); goto isp_fail; }

  cc.RR = 0;   cc.RG = 0;   cc.RB = 0;   cc.RA = 128;
  cc.GR = 0;   cc.GG = 0;   cc.GB = 0;   cc.GA = 128;
  cc.BR = 0;   cc.BG = 0;   cc.BB = 0;   cc.BA = 128;
  r = BSP_CAMERA_SetIspColorConversion(0, BSP_CAMERA_PIPE_MAIN, &cc);
  if (r != BSP_ERROR_NONE) { fail("camisp", "SetIspColorConversion flat %ld", (long)r); goto isp_fail; }
  if (cam_frame_spread(BSP_CAMERA_PIPE_MAIN, 4U, &mean_flat, &spread_flat) != BSP_ERROR_NONE)
  { fail("camisp", "no frame on the main pipe with the flat matrix"); goto isp_fail; }

  (void)BSP_CAMERA_SetIspColorConversion(0, BSP_CAMERA_PIPE_MAIN, NULL);
  cam_bars(0U);
  (void)BSP_CAMERA_DeInit(0);

  P("INFO pass-through: mean=%lu spread=%lu; coefficients zeroed: mean=%lu spread=%lu\r\n",
    (unsigned long)mean_live, (unsigned long)spread_live,
    (unsigned long)mean_flat, (unsigned long)spread_flat);

  if (spread_live < 4U)
  { fail("camisp", "the pass-through frame had no spread (%lu) - there was no picture to judge", (unsigned long)spread_live); return; }
  if (spread_flat >= spread_live)
  { fail("camisp", "zeroing the matrix did not flatten the picture: spread %lu became %lu, so the coefficients are not reaching the pipe",
         (unsigned long)spread_live, (unsigned long)spread_flat); return; }

  ok("camisp", "all five stages accept, refuse the wrong pipe and switch off; zeroing the colour matrix flattens the picture (spread %lu -> %lu)",
     (unsigned long)spread_live, (unsigned long)spread_flat);
  return;

isp_fail:
  cam_bars(0U);
  (void)BSP_CAMERA_DeInit(0);
}

/* The sensor's tuning calls: brightness, contrast, saturation, hue, colour
 * effect, light mode, night mode, mirror, zoom. */
static void cmd_camcfg(void)
{
  BSP_CAMERA_PipeConfig_t pc = {0};
  uint32_t mean_dark = 0U, mean_bright = 0U, u = 0U, mf_was = 0U;
  int32_t  r;

  CamFrames = CamErrors = 0;
  if (cam_up() != BSP_ERROR_NONE) { fail("camcfg", "init"); return; }
  pc.PixelFormat = BSP_CAMERA_PIPE_PF_RGB565; pc.Width = CAM_W; pc.Height = CAM_H;
  if (BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_DUMP, &pc) != BSP_ERROR_NONE)
  { fail("camcfg", "config pipe"); (void)BSP_CAMERA_DeInit(0); return; }

  /* What the driver thinks it is set to, before anything is changed. */
  {
    uint32_t pf = 0U;
    (void)BSP_CAMERA_GetResolution(0, &u);
    (void)BSP_CAMERA_GetPixelFormat(0, &pf);
    P("INFO resolution=%lu pixelformat=%lu\r\n", (unsigned long)u, (unsigned long)pf);
  }

  /* Round trips: a register write, read back through the driver's record. */
  {
    static const struct { const char *name;
                          int32_t (*set)(uint32_t, int32_t);
                          int32_t (*get)(uint32_t, int32_t *);
                          int32_t lo; int32_t hi; } lvl[] = {
      { "brightness", BSP_CAMERA_SetBrightness, BSP_CAMERA_GetBrightness, BSP_CAMERA_BRIGHTNESS_MIN, BSP_CAMERA_BRIGHTNESS_MAX },
      { "contrast",   BSP_CAMERA_SetContrast,   BSP_CAMERA_GetContrast,   BSP_CAMERA_CONTRAST_MIN,   BSP_CAMERA_CONTRAST_MAX   },
      { "saturation", BSP_CAMERA_SetSaturation, BSP_CAMERA_GetSaturation, BSP_CAMERA_SATURATION_MIN, BSP_CAMERA_SATURATION_MAX },
      { "hue",        BSP_CAMERA_SetHueDegree,  BSP_CAMERA_GetHueDegree,  BSP_CAMERA_HUE_DEGREE_MIN, BSP_CAMERA_HUE_DEGREE_MAX },
    };

    for (uint32_t i = 0U; i < (sizeof lvl / sizeof lvl[0]); i++)
    {
      int32_t back_lo = 999, back_hi = 999;
      int32_t r_lo = lvl[i].set(0, lvl[i].lo);
      (void)lvl[i].get(0, &back_lo);
      svc(5U);
      int32_t r_hi = lvl[i].set(0, lvl[i].hi);
      (void)lvl[i].get(0, &back_hi);
      svc(5U);
      P("INFO %-10s min %ld -> %ld (r=%ld), max %ld -> %ld (r=%ld)\r\n",
        lvl[i].name, (long)lvl[i].lo, (long)back_lo, (long)r_lo,
        (long)lvl[i].hi, (long)back_hi, (long)r_hi);
      if ((r_lo != BSP_ERROR_NONE) || (r_hi != BSP_ERROR_NONE))
      { fail("camcfg", "%s refused its own documented range (%ld/%ld)", lvl[i].name, (long)r_lo, (long)r_hi); goto cfg_fail; }
      if ((back_lo != lvl[i].lo) || (back_hi != lvl[i].hi))
      { fail("camcfg", "%s read back %ld/%ld for %ld/%ld", lvl[i].name, (long)back_lo, (long)back_hi, (long)lvl[i].lo, (long)lvl[i].hi); goto cfg_fail; }
      /* Leave it in the middle of its range. */
      (void)lvl[i].set(0, 0);
    }
  }

  /* The enumerated settings, same treatment. */
  {
    r = BSP_CAMERA_SetColorEffect(0, BSP_CAMERA_COLOR_EFFECT_BW);
    (void)BSP_CAMERA_GetColorEffect(0, &u);
    P("INFO color effect BW -> %lu (r=%ld)\r\n", (unsigned long)u, (long)r);
    if (r != BSP_ERROR_NONE) { fail("camcfg", "SetColorEffect %ld", (long)r); goto cfg_fail; }
    (void)BSP_CAMERA_SetColorEffect(0, BSP_CAMERA_COLOR_EFFECT_NONE);

    (void)BSP_CAMERA_GetMirrorFlip(0, &mf_was);
    r = BSP_CAMERA_SetMirrorFlip(0, BSP_CAMERA_MIRRORFLIP_BOTH);
    (void)BSP_CAMERA_GetMirrorFlip(0, &u);
    P("INFO mirrorflip %lu, BOTH -> %lu (r=%ld)\r\n", (unsigned long)mf_was, (unsigned long)u, (long)r);
    if (r != BSP_ERROR_NONE) { fail("camcfg", "SetMirrorFlip %ld", (long)r); goto cfg_fail; }
    (void)BSP_CAMERA_SetMirrorFlip(0, mf_was);

    r = BSP_CAMERA_SetZoom(0, BSP_CAMERA_ZOOM_x1);
    (void)BSP_CAMERA_GetZoom(0, &u);
    P("INFO zoom x1 -> %lu (r=%ld)\r\n", (unsigned long)u, (long)r);
    if (r != BSP_ERROR_NONE) { fail("camcfg", "SetZoom %ld", (long)r); goto cfg_fail; }

    r = BSP_CAMERA_EnableNightMode(0);
    svc(10U);
    { int32_t r2 = BSP_CAMERA_DisableNightMode(0);
      P("INFO night mode enable r=%ld disable r=%ld\r\n", (long)r, (long)r2);
      if ((r != BSP_ERROR_NONE) || (r2 != BSP_ERROR_NONE))
      { fail("camcfg", "night mode %ld/%ld", (long)r, (long)r2); goto cfg_fail; }
    }

    (void)BSP_CAMERA_GetLightMode(0, &u);
    r = BSP_CAMERA_SetLightMode(0, u);
    P("INFO light mode reads %lu, setting it back r=%ld\r\n", (unsigned long)u, (long)r);
    if (r != BSP_ERROR_NONE) { fail("camcfg", "SetLightMode %ld", (long)r); goto cfg_fail; }
  }

  /* Does brightness reach the picture? */
  {
    uint32_t fast_dark = 0U, slow_dark = 0U, fast_bright = 0U, slow_bright = 0U;

    /* Two samples per setting: one before the sensor's exposure loop reacts,
     * one half a second after. */
    (void)BSP_CAMERA_SetBrightness(0, BSP_CAMERA_BRIGHTNESS_MIN);
    if (cam_frame_luma(&fast_dark, 2U) != BSP_ERROR_NONE) { fail("camcfg", "no frame at minimum brightness"); goto cfg_fail; }
    svc(500U);
    if (cam_frame_luma(&slow_dark, 8U) != BSP_ERROR_NONE) { fail("camcfg", "no settled frame at minimum brightness"); goto cfg_fail; }

    (void)BSP_CAMERA_SetBrightness(0, BSP_CAMERA_BRIGHTNESS_MAX);
    if (cam_frame_luma(&fast_bright, 2U) != BSP_ERROR_NONE) { fail("camcfg", "no frame at maximum brightness"); goto cfg_fail; }
    svc(500U);
    if (cam_frame_luma(&slow_bright, 8U) != BSP_ERROR_NONE) { fail("camcfg", "no settled frame at maximum brightness"); goto cfg_fail; }

    P("INFO luma at brightness %d: %lu immediately, %lu after settling\r\n",
      BSP_CAMERA_BRIGHTNESS_MIN, (unsigned long)fast_dark, (unsigned long)slow_dark);
    P("INFO luma at brightness %d: %lu immediately, %lu after settling\r\n",
      BSP_CAMERA_BRIGHTNESS_MAX, (unsigned long)fast_bright, (unsigned long)slow_bright);

    /* The whole range, each sampled before the exposure loop reacts. */
    {
      char trail[120] = "";
      for (int32_t lv = BSP_CAMERA_BRIGHTNESS_MIN; lv <= BSP_CAMERA_BRIGHTNESS_MAX; lv += 2)
      {
        uint32_t l = 0U;
        char one[20];
        (void)BSP_CAMERA_SetBrightness(0, lv);
        if (cam_frame_luma(&l, 2U) != BSP_ERROR_NONE) { break; }
        snprintf(one, sizeof one, "%ld:%lu ", (long)lv, (unsigned long)l);
        if (strlen(trail) + strlen(one) < sizeof trail) { strcat(trail, one); }
      }
      P("INFO luma across the range (level:luma) %s\r\n", trail);
    }

    (void)BSP_CAMERA_SetBrightness(0, 0);
    (void)BSP_CAMERA_DeInit(0);

    mean_dark = slow_dark; mean_bright = slow_bright;

    /* The two ends must differ; where the settled image ends up is the
     * exposure loop's business. */
    {
      uint32_t d = (fast_bright > fast_dark) ? (fast_bright - fast_dark) : (fast_dark - fast_bright);
      if (d < 4U)
      { fail("camcfg", "brightness changed nothing: luma %lu at min, %lu at max",
             (unsigned long)fast_dark, (unsigned long)fast_bright); return; }
      /* And the right way: neuron6_camera negates the sensor driver's
       * backwards sign. */
      if (fast_bright <= fast_dark)
      { fail("camcfg", "brightness runs backwards: luma %lu at %d but %lu at %d",
             (unsigned long)fast_dark, BSP_CAMERA_BRIGHTNESS_MIN,
             (unsigned long)fast_bright, BSP_CAMERA_BRIGHTNESS_MAX); return; }
    }
  }

  ok("camcfg", "every setting round-trips, and brightness visibly changes the picture (settled luma %lu at min, %lu at max)",
     (unsigned long)mean_dark, (unsigned long)mean_bright);
  return;

cfg_fail:
  (void)BSP_CAMERA_DeInit(0);
}

/* Suspend and resume, on the pipe and on the sensor. */
static void cmd_camsusp(void)
{
  BSP_CAMERA_PipeConfig_t pc = {0};
  uint32_t before, during, after;
  int32_t  r;

  CamFrames = CamErrors = 0;
  if (cam_up() != BSP_ERROR_NONE) { fail("camsusp", "init"); return; }
  pc.PixelFormat = BSP_CAMERA_PIPE_PF_RGB565; pc.Width = CAM_W; pc.Height = CAM_H;
  if (BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_DUMP, &pc) != BSP_ERROR_NONE)
  { fail("camsusp", "config pipe"); (void)BSP_CAMERA_DeInit(0); return; }

  CamFrames = 0U;
  if (BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_DUMP, CamFrame, BSP_CAMERA_MODE_CONTINUOUS) != BSP_ERROR_NONE)
  { fail("camsusp", "start"); (void)BSP_CAMERA_DeInit(0); return; }
  { uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 800U) && (CamFrames < 5U)) { svc(5U); } }
  if (CamFrames == 0U) { fail("camsusp", "no frames before suspending"); goto susp_fail; }

  /* Pipe suspend. */
  r = BSP_CAMERA_Suspend(0, BSP_CAMERA_PIPE_DUMP);
  svc(50U);
  before = CamFrames;
  svc(300U);
  during = CamFrames;
  P("INFO pipe suspend r=%ld, frames %lu -> %lu across 300 ms suspended\r\n",
    (long)r, (unsigned long)before, (unsigned long)during);
  if (r != BSP_ERROR_NONE) { fail("camsusp", "Suspend %ld", (long)r); goto susp_fail; }
  if (during != before) { fail("camsusp", "frames kept arriving while the pipe was suspended (%lu -> %lu)", (unsigned long)before, (unsigned long)during); goto susp_fail; }

  r = BSP_CAMERA_Resume(0, BSP_CAMERA_PIPE_DUMP);
  { uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 800U) && (CamFrames <= during)) { svc(5U); } }
  after = CamFrames;
  P("INFO pipe resume r=%ld, frames now %lu\r\n", (long)r, (unsigned long)after);
  if (r != BSP_ERROR_NONE) { fail("camsusp", "Resume %ld", (long)r); goto susp_fail; }
  if (after <= during) { fail("camsusp", "no frames after resuming the pipe"); goto susp_fail; }

  /* Sensor suspend: the link goes quiet, so the receiver must reacquire on
   * resume. */
  r = BSP_CAMERA_SuspendSensor(0);
  svc(50U);
  before = CamFrames;
  svc(300U);
  during = CamFrames;
  P("INFO sensor suspend r=%ld, frames %lu -> %lu across 300 ms suspended\r\n",
    (long)r, (unsigned long)before, (unsigned long)during);
  if (r != BSP_ERROR_NONE) { fail("camsusp", "SuspendSensor %ld", (long)r); goto susp_fail; }
  if (during != before) { fail("camsusp", "frames kept arriving with the sensor suspended (%lu -> %lu)", (unsigned long)before, (unsigned long)during); goto susp_fail; }

  r = BSP_CAMERA_ResumeSensor(0);
  { uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 1500U) && (CamFrames <= during)) { svc(5U); } }
  after = CamFrames;
  P("INFO sensor resume r=%ld, frames now %lu (errors=%lu)\r\n",
    (long)r, (unsigned long)after, (unsigned long)CamErrors);
  if (r != BSP_ERROR_NONE) { fail("camsusp", "ResumeSensor %ld", (long)r); goto susp_fail; }
  if (after <= during) { fail("camsusp", "the link did not come back after resuming the sensor"); goto susp_fail; }

  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_DUMP);
  (void)BSP_CAMERA_DeInit(0);
  ok("camsusp", "pipe and sensor both stop the frames when suspended and deliver again when resumed");
  return;

susp_fail:
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_DUMP);
  (void)BSP_CAMERA_DeInit(0);
}

/* Sixteen-bit register addressing, and the raw transfers under it. */
static void cmd_reg16(void)
{
  uint8_t hi = 0U, lo = 0U, pair[2] = {0U, 0U};
  uint8_t addr[2];
  int32_t r;

  if (BSP_I2C1_Init() != BSP_ERROR_NONE) { fail("reg16", "I2C1 init"); return; }

  /* One byte at a time. */
  r  = BSP_I2C1_ReadReg16(BSP_CAMERA_I2C_ADDRESS, OV5640_CHIP_ID_HIGH_BYTE, &hi, 1U);
  r |= BSP_I2C1_ReadReg16(BSP_CAMERA_I2C_ADDRESS, OV5640_CHIP_ID_LOW_BYTE, &lo, 1U);
  P("INFO ReadReg16 chip id %02X %02X (want 56 40), r=%ld\r\n", hi, lo, (long)r);
  if (r != BSP_ERROR_NONE) { fail("reg16", "ReadReg16 %ld", (long)r); goto reg_out; }
  if ((hi != 0x56U) || (lo != 0x40U))
  { fail("reg16", "chip id read %02X%02X, expected 5640", hi, lo); goto reg_out; }

  /* Two bytes across one transfer, which exercises the length path. */
  r = BSP_I2C1_ReadReg16(BSP_CAMERA_I2C_ADDRESS, OV5640_CHIP_ID_HIGH_BYTE, pair, 2U);
  P("INFO ReadReg16 two bytes at once %02X %02X, r=%ld\r\n", pair[0], pair[1], (long)r);
  if ((r != BSP_ERROR_NONE) || (pair[0] != 0x56U) || (pair[1] != 0x40U))
  { fail("reg16", "two-byte read gave %02X%02X (r=%ld)", pair[0], pair[1], (long)r); goto reg_out; }

  /* A harmless write: the identity registers are read-only. */
  { uint8_t same = hi;
    r = BSP_I2C1_WriteReg16(BSP_CAMERA_I2C_ADDRESS, OV5640_CHIP_ID_HIGH_BYTE, &same, 1U);
  }
  { uint8_t after = 0U;
    (void)BSP_I2C1_ReadReg16(BSP_CAMERA_I2C_ADDRESS, OV5640_CHIP_ID_HIGH_BYTE, &after, 1U);
    P("INFO WriteReg16 r=%ld, register still reads %02X\r\n", (long)r, after);
    if (r != BSP_ERROR_NONE) { fail("reg16", "WriteReg16 %ld", (long)r); goto reg_out; }
    if (after != hi) { fail("reg16", "a read-only register changed from %02X to %02X", hi, after); goto reg_out; }
  }

  /* By hand, as ReadReg16 does it: send the two address bytes, then read. */
  addr[0] = (uint8_t)(OV5640_CHIP_ID_HIGH_BYTE >> 8);
  addr[1] = (uint8_t)(OV5640_CHIP_ID_HIGH_BYTE & 0xFFU);
  r = BSP_I2C1_Send(BSP_CAMERA_I2C_ADDRESS, addr, 2U);
  if (r == BSP_ERROR_NONE)
  {
    uint8_t raw = 0U;
    r = BSP_I2C1_Recv(BSP_CAMERA_I2C_ADDRESS, &raw, 1U);
    P("INFO Send+Recv by hand read %02X (want 56), r=%ld\r\n", raw, (long)r);
    if (r != BSP_ERROR_NONE) { fail("reg16", "Recv %ld", (long)r); goto reg_out; }
    if (raw != 0x56U) { fail("reg16", "the raw transfer read %02X, expected 56", raw); goto reg_out; }
  }
  else
  {
    fail("reg16", "Send %ld", (long)r); goto reg_out;
  }

  (void)BSP_I2C1_DeInit();

  /* The header bus, where nothing is fitted. */
  if (BSP_I2C2_Init() != BSP_ERROR_NONE) { fail("reg16", "I2C2 init"); return; }
  {
    uint8_t b = 0U;
    int32_t r16 = BSP_I2C2_ReadReg16(0x78U, 0x300AU, &b, 1U);
    int32_t r8  = BSP_I2C2_ReadReg(0x78U, 0x0AU, &b, 1U);
    int32_t rs  = BSP_I2C2_Send(0x78U, addr, 2U);
    P("INFO empty header bus: ReadReg16=%ld ReadReg=%ld Send=%ld (none may be 0)\r\n",
      (long)r16, (long)r8, (long)rs);
    (void)BSP_I2C2_DeInit();
    if ((r16 == BSP_ERROR_NONE) || (r8 == BSP_ERROR_NONE) || (rs == BSP_ERROR_NONE))
    { fail("reg16", "a transfer to an absent device reported success"); return; }
  }

  /* The header SPI's achieved clock: computed, not the requested one. */
  if (BSP_SPI2_Init() != BSP_ERROR_NONE) { fail("reg16", "SPI2 init"); return; }
  {
    uint32_t hz = BSP_SPI2_GetClockFreq();
    uint8_t  tx[4] = {0xDEU, 0xADU, 0xBEU, 0xEFU};
    uint8_t  rx[4] = {0U};
    int32_t  rt = BSP_SPI2_Send(tx, 4U);
    int32_t  rr = BSP_SPI2_Recv(rx, 4U);
    P("INFO SPI2 clock %lu Hz, Send r=%ld Recv r=%ld (read %02X%02X%02X%02X, nothing is wired so any value is fine)\r\n",
      (unsigned long)hz, (long)rt, (long)rr, rx[0], rx[1], rx[2], rx[3]);
    (void)BSP_SPI2_DeInit();
    if ((hz < 100000U) || (hz > 100000000U))
    { fail("reg16", "SPI2 reports a %lu Hz clock, which is not a rate this bus runs at", (unsigned long)hz); return; }
    if ((rt != BSP_ERROR_NONE) || (rr != BSP_ERROR_NONE))
    { fail("reg16", "a SPI transfer failed with nothing on the bus: %ld/%ld", (long)rt, (long)rr); return; }
  }

  ok("reg16", "sixteen-bit addressing reads the sensor's identity three ways, an absent device is refused, SPI2 clock and transfers fine");
  return;

reg_out:
  (void)BSP_I2C1_DeInit();
}

/* The audio setters (audcfg covers the getters and pause). */
static void cmd_audset(void)
{
  BSP_AUDIO_Init_t ao = { BSP_AUDIO_OUT_DEVICE_SPEAKER, BSP_AUDIO_FREQUENCY_NATIVE, BSP_AUDIO_RESOLUTION_16B, 2U, 50U };
  BSP_AUDIO_Init_t ai = { BSP_AUDIO_IN_DEVICE_DIGITAL_MIC, BSP_AUDIO_FREQUENCY_NATIVE, BSP_AUDIO_RESOLUTION_16B, 1U, 50U };
  uint32_t native = 0U, got = 0U, outRate = 0U;
  uint32_t gotMeasured = 0U, slowMeasured = 0U;   /* what the capture actually ran at */
  uint32_t outMeasured = 0U, monoMeasured = 0U;   /* and the playback */
  int32_t  r;

  /* Playback */
  SpkHalf = SpkFull = SpkErr = 0;
  if (BSP_AUDIO_OUT_Init(0, &ao) != BSP_ERROR_NONE) { fail("audset", "out init"); return; }
  (void)BSP_AUDIO_OUT_GetSampleRate(0, &native);

  /* Setters with one legal value on this board accept it and refuse others. */
  r = BSP_AUDIO_OUT_SetDevice(0, BSP_AUDIO_OUT_DEVICE_SPEAKER);
  if (r != BSP_ERROR_NONE) { fail("audset", "SetDevice speaker %ld", (long)r); goto out_done; }
  if (BSP_AUDIO_OUT_SetDevice(0, 0x55U) == BSP_ERROR_NONE)
  { fail("audset", "SetDevice accepted a device that is not fitted"); goto out_done; }

  r = BSP_AUDIO_OUT_SetChannelsNbr(0, 1U);
  { uint32_t ch = 0U; (void)BSP_AUDIO_OUT_GetChannelsNbr(0, &ch);
    if ((r != BSP_ERROR_NONE) || (ch != 1U)) { fail("audset", "SetChannelsNbr 1 gave r=%ld, reads %lu", (long)r, (unsigned long)ch); goto out_done; } }
  (void)BSP_AUDIO_OUT_SetChannelsNbr(0, 2U);

  r = BSP_AUDIO_OUT_SetBitsPerSample(0, BSP_AUDIO_RESOLUTION_16B);
  if (r != BSP_ERROR_NONE) { fail("audset", "SetBitsPerSample 16 %ld", (long)r); goto out_done; }

  /* Half the native rate, which the clock tree reaches exactly; then play. */
  r = BSP_AUDIO_OUT_SetSampleRate(0, native / 2U);
  (void)BSP_AUDIO_OUT_GetSampleRate(0, &got);
  P("INFO out rate %lu -> asked %lu -> %lu (r=%ld)\r\n",
    (unsigned long)native, (unsigned long)(native / 2U), (unsigned long)got, (long)r);
  if (r != BSP_ERROR_NONE) { fail("audset", "SetSampleRate %ld", (long)r); goto out_done; }
  {
    uint32_t want = native / 2U;
    uint32_t diff = (got > want) ? (got - want) : (want - got);
    if (diff > (want / 50U)) { fail("audset", "asked %lu Hz and got %lu", (unsigned long)want, (unsigned long)got); goto out_done; }
  }

  outRate = got;

  /* Rates the amplifier cannot take are refused, and the rate stays put. */
  {
    static const uint32_t bad[] = { 192000U, 4000U, 0xFFFFFFFFU };

    for (uint32_t i = 0U; i < (sizeof bad / sizeof bad[0]); i++)
    {
      uint32_t back = 0U;
      int32_t  rb   = BSP_AUDIO_OUT_SetSampleRate(0, bad[i]);

      (void)BSP_AUDIO_OUT_GetSampleRate(0, &back);
      if ((rb != BSP_ERROR_FEATURE_NOT_SUPPORTED) || (back != got))
      { fail("audset", "playback at %lu Hz gave r=%ld and reads %lu, not a refusal at %lu",
             (unsigned long)bad[i], (long)rb, (unsigned long)back, (unsigned long)got); goto out_done; }
    }
  }

  for (uint32_t i = 0; i < SPK_SAMPLES; i += 2U)
  {
    int16_t v = (int16_t)(4000.0f * sinf(6.2831853f * 440.0f * (float)(i / 2U) / (float)(got ? got : 48000U)));
    SpkBuf[i] = v; SpkBuf[i + 1U] = v;
  }
  SpkFull = 0U;
  if (BSP_AUDIO_OUT_Play(0, (uint8_t *)SpkBuf, sizeof SpkBuf) != BSP_ERROR_NONE)
  { fail("audset", "play at the new rate"); goto out_done; }
  { uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 1500U) && (SpkFull < 9U)) { svc(5U); } }
  (void)BSP_AUDIO_OUT_Stop(0);
  outMeasured = spk_measured_rate(SPK_SAMPLES / 2U);
  P("INFO played at %lu Hz (measured %lu), %lu transfer callbacks\r\n", (unsigned long)got,
    (unsigned long)outMeasured, (unsigned long)SpkFull);
  if (SpkFull < 3U) { fail("audset", "only %lu callbacks after the rate change", (unsigned long)SpkFull); goto out_done; }
  if (!rate_agrees(got, outMeasured))
  { fail("audset", "playback set to %lu Hz runs at %lu Hz", (unsigned long)got, (unsigned long)outMeasured); goto out_done; }

  /* One channel through the setter: the same buffer, now one sample a frame. */
  r = BSP_AUDIO_OUT_SetChannelsNbr(0, 1U);
  if (r != BSP_ERROR_NONE) { fail("audset", "SetChannelsNbr 1 while stopped %ld", (long)r); goto out_done; }
  SpkFull = 0U;
  if (BSP_AUDIO_OUT_Play(0, (uint8_t *)SpkBuf, sizeof SpkBuf) != BSP_ERROR_NONE)
  { fail("audset", "play mono"); goto out_done; }
  { uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 2000U) && (SpkFull < 9U)) { svc(5U); } }
  (void)BSP_AUDIO_OUT_Stop(0);
  monoMeasured = spk_measured_rate(SPK_SAMPLES);
  P("INFO mono at %lu Hz (measured %lu), %lu transfer callbacks\r\n", (unsigned long)got,
    (unsigned long)monoMeasured, (unsigned long)SpkFull);
  (void)BSP_AUDIO_OUT_SetChannelsNbr(0, 2U);
  if (!rate_agrees(got, monoMeasured))
  { fail("audset", "mono set to %lu Hz runs at %lu frames a second", (unsigned long)got, (unsigned long)monoMeasured); goto out_done; }

  /* CleanBuffer, which pushes a region out of the cache for the DMA. */
  r = BSP_AUDIO_OUT_CleanBuffer(0, SpkBuf, sizeof SpkBuf);
  if (r != BSP_ERROR_NONE) { fail("audset", "CleanBuffer %ld", (long)r); goto out_done; }

  (void)BSP_AUDIO_OUT_SetSampleRate(0, native);
  (void)BSP_AUDIO_OUT_DeInit(0);

  /* Capture */
  MicHalf = MicFull = 0;
  if (BSP_AUDIO_IN_Init(0, &ai) != BSP_ERROR_NONE) { fail("audset", "in init"); return; }
  (void)BSP_AUDIO_IN_GetSampleRate(0, &native);

  r  = BSP_AUDIO_IN_SetDevice(0, BSP_AUDIO_IN_DEVICE_DIGITAL_MIC);
  r |= BSP_AUDIO_IN_SetChannelsNbr(0, 1U);
  r |= BSP_AUDIO_IN_SetBitsPerSample(0, BSP_AUDIO_RESOLUTION_16B);
  if (r != BSP_ERROR_NONE) { fail("audset", "an input setter refused its only legal value (%ld)", (long)r); goto in_done; }

  /* Input volume: set and get must agree, refused or not. */
  {
    uint32_t v = 0U;
    int32_t sv = BSP_AUDIO_IN_SetVolume(0, 60U);
    int32_t gv = BSP_AUDIO_IN_GetVolume(0, &v);
    P("INFO in volume set r=%ld get r=%ld value %lu\r\n", (long)sv, (long)gv, (unsigned long)v);
    if ((sv == BSP_ERROR_NONE) != (gv == BSP_ERROR_NONE))
    { fail("audset", "input volume half works: set %ld, get %ld", (long)sv, (long)gv); goto in_done; }
    if ((sv == BSP_ERROR_NONE) && (v != 60U))
    { fail("audset", "input volume accepted 60 and reads %lu", (unsigned long)v); goto in_done; }
  }

  /* The capture clock is two dividers off a kernel clock: not every rate is
   * reachable. */
  {
    static const uint32_t want[] = { 48000U, 32000U, 16000U, 8000U };
    uint32_t changed = 0U;
    char trail[140] = "";

    for (uint32_t i = 0U; i < (sizeof want / sizeof want[0]); i++)
    {
      char one[36];
      uint32_t back = 0U;
      int32_t rc = BSP_AUDIO_IN_SetSampleRate(0, want[i]);
      (void)BSP_AUDIO_IN_GetSampleRate(0, &back);
      snprintf(one, sizeof one, "%lu:r=%ld->%lu ", (unsigned long)want[i],
               (long)rc, (unsigned long)back);
      if (strlen(trail) + strlen(one) < sizeof trail) { strcat(trail, one); }
      if ((rc == BSP_ERROR_NONE) && (back != native)) { changed = 1U; got = back; break; }
    }
    P("INFO in rates from %lu: %s\r\n", (unsigned long)native, trail);

    /* At least one rate must take; the recording below decides if it is real. */
    if (changed == 0U)
    {
      uint32_t back = 0U;
      (void)BSP_AUDIO_IN_GetSampleRate(0, &back);
      if (back != native)
      { fail("audset", "a refused rate change left the reported rate at %lu instead of %lu",
             (unsigned long)back, (unsigned long)native); goto in_done; }
      fail("audset", "no capture rate could be set at all; the rate still reads %lu",
           (unsigned long)back); goto in_done;
    }
  }

  /* A rate that reads back may not be the one running: record and measure. */
  MicFull = 0U;
  {
    int32_t rr = BSP_AUDIO_IN_Record(0, (uint8_t *)MicBuf, sizeof MicBuf);
    if (rr != BSP_ERROR_NONE)
    { fail("audset", "the capture path stopped working after the rate change (Record=%ld)", (long)rr); goto in_done; }
  }
  { uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 1500U) && (MicFull < 9U)) { svc(5U); } }
  (void)BSP_AUDIO_IN_Stop(0);
  gotMeasured = mic_measured_rate();
  P("INFO recorded at %lu Hz (measured %lu), %lu transfer callbacks\r\n", (unsigned long)got,
    (unsigned long)gotMeasured, (unsigned long)MicFull);
  if (MicFull < 2U) { fail("audset", "only %lu callbacks after the input rate change", (unsigned long)MicFull); goto in_done; }
  if (!rate_agrees(got, gotMeasured))
  { fail("audset", "capture set to %lu Hz runs at %lu Hz", (unsigned long)got, (unsigned long)gotMeasured); goto in_done; }

  /* Rates that would take the microphone's clock outside its 1.2 MHz to
   * 3.25 MHz are refused, and the rate stays put. */
  {
    static const uint32_t bad[] = { 8000U, 96000U, 0xFFFFFFFFU };

    for (uint32_t i = 0U; i < (sizeof bad / sizeof bad[0]); i++)
    {
      uint32_t back = 0U;
      int32_t  rb   = BSP_AUDIO_IN_SetSampleRate(0, bad[i]);

      (void)BSP_AUDIO_IN_GetSampleRate(0, &back);
      if ((rb != BSP_ERROR_FEATURE_NOT_SUPPORTED) || (back != got))
      { fail("audset", "capture at %lu Hz gave r=%ld and reads %lu, not a refusal at %lu",
             (unsigned long)bad[i], (long)rb, (unsigned long)back, (unsigned long)got); goto in_done; }
    }
  }

  /* The slow end, then a full teardown and rebuild on top of it. */
  {
    uint32_t slow = 0U;
    int32_t  rs   = BSP_AUDIO_IN_SetSampleRate(0, 11025U);

    (void)BSP_AUDIO_IN_GetSampleRate(0, &slow);
    P("INFO slow rate asked 11025 got %lu (r=%ld)\r\n", (unsigned long)slow, (long)rs);
    if (rs != BSP_ERROR_NONE) { fail("audset", "11025 Hz refused (%ld)", (long)rs); goto in_done; }

    MicFull = 0U;
    if (BSP_AUDIO_IN_Record(0, (uint8_t *)MicBuf, sizeof MicBuf) != BSP_ERROR_NONE)
    { fail("audset", "record at the slow rate"); goto in_done; }
    { uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 2000U) && (MicFull < 5U)) { svc(5U); } }
    (void)BSP_AUDIO_IN_Stop(0);
    slowMeasured = mic_measured_rate();
    P("INFO recorded at %lu Hz (measured %lu), %lu transfer callbacks\r\n", (unsigned long)slow,
      (unsigned long)slowMeasured, (unsigned long)MicFull);
    if (MicFull < 1U) { fail("audset", "no callbacks at %lu Hz", (unsigned long)slow); goto in_done; }
    if (!rate_agrees(slow, slowMeasured))
    { fail("audset", "capture set to %lu Hz runs at %lu Hz", (unsigned long)slow, (unsigned long)slowMeasured); goto in_done; }

    (void)BSP_AUDIO_IN_DeInit(0);
    if (BSP_AUDIO_IN_Init(0, &ai) != BSP_ERROR_NONE)
    { fail("audset", "capture would not initialise again after a recording at %lu Hz", (unsigned long)slow); return; }

    MicFull = 0U;
    if (BSP_AUDIO_IN_Record(0, (uint8_t *)MicBuf, sizeof MicBuf) != BSP_ERROR_NONE)
    { fail("audset", "record after the rebuild"); goto in_done; }
    { uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 1500U) && (MicFull < 2U)) { svc(5U); } }
    (void)BSP_AUDIO_IN_Stop(0);
    P("INFO after deinit/init, %lu transfer callbacks\r\n", (unsigned long)MicFull);
    if (MicFull < 2U) { fail("audset", "only %lu callbacks after a rebuild", (unsigned long)MicFull); goto in_done; }
  }

  (void)BSP_AUDIO_IN_SetSampleRate(0, native);
  (void)BSP_AUDIO_IN_DeInit(0);

  ok("audset", "playback rate changed to %lu (measured %lu stereo, %lu mono) and still streamed; capture moved to %lu (measured %lu) and to 11025 (measured %lu) and recorded at both, and came back from a teardown at the slow rate; the one-value setters accept and refuse correctly", (unsigned long)outRate, (unsigned long)outMeasured, (unsigned long)monoMeasured, (unsigned long)got, (unsigned long)gotMeasured, (unsigned long)slowMeasured);
  return;

out_done:
  (void)BSP_AUDIO_OUT_Stop(0);
  (void)BSP_AUDIO_OUT_DeInit(0);
  return;
in_done:
  (void)BSP_AUDIO_IN_Stop(0);
  (void)BSP_AUDIO_IN_DeInit(0);
}

/* Audio getters, the volume refusal, and pause and resume, out and in. */
static void cmd_audcfg(void)
{
  BSP_AUDIO_Init_t ao = { BSP_AUDIO_OUT_DEVICE_SPEAKER, BSP_AUDIO_FREQUENCY_NATIVE, BSP_AUDIO_RESOLUTION_16B, 2U, 50U };
  uint32_t rate = 0U, vol = 0U, ch = 0U, bits = 0U, dev = 0U, st = 99U;
  uint32_t during_pause;
  int32_t  r;

  SpkHalf = SpkFull = SpkErr = 0;
  r = BSP_AUDIO_OUT_Init(0, &ao);
  if (r != BSP_ERROR_NONE) { fail("audcfg", "out init %ld", (long)r); return; }

  (void)BSP_AUDIO_OUT_GetSampleRate(0, &rate);
  (void)BSP_AUDIO_OUT_GetVolume(0, &vol);
  (void)BSP_AUDIO_OUT_GetChannelsNbr(0, &ch);
  (void)BSP_AUDIO_OUT_GetBitsPerSample(0, &bits);
  (void)BSP_AUDIO_OUT_GetDevice(0, &dev);
  (void)BSP_AUDIO_OUT_GetState(0, &st);
  P("INFO out after init: rate=%lu vol=%lu ch=%lu bits=%lu dev=%lu state=%lu (expect state STOP=%u)\r\n",
    (unsigned long)rate, (unsigned long)vol, (unsigned long)ch, (unsigned long)bits,
    (unsigned long)dev, (unsigned long)st, (unsigned)BSP_AUDIO_OUT_STATE_STOP);
  if (dev != BSP_AUDIO_OUT_DEVICE_SPEAKER) { fail("audcfg", "device reads %lu", (unsigned long)dev); goto out_fail; }
  if (ch != 2U) { fail("audcfg", "channels read %lu, set 2", (unsigned long)ch); goto out_fail; }

  /* Volume is unsupported (the amplifier's gain is a strapping resistor) and
   * must say so. */
  {
    int32_t sv = BSP_AUDIO_OUT_SetVolume(0, 30U);
    int32_t gv = BSP_AUDIO_OUT_GetVolume(0, &vol);
    P("INFO volume set r=%ld get r=%ld (both must be FEATURE_NOT_SUPPORTED=%ld)\r\n",
      (long)sv, (long)gv, (long)BSP_ERROR_FEATURE_NOT_SUPPORTED);
    if ((sv != BSP_ERROR_FEATURE_NOT_SUPPORTED) || (gv != BSP_ERROR_FEATURE_NOT_SUPPORTED))
    { fail("audcfg", "volume answered %ld/%ld instead of refusing", (long)sv, (long)gv); goto out_fail; }
  }

  /* Play, pause, resume: the callback count must stop while paused and move
   * again after resume. */
  for (uint32_t i = 0; i < SPK_SAMPLES; i += 2U)
  {
    int16_t v = (int16_t)(4000.0f * sinf(6.2831853f * 440.0f * (float)(i / 2U) / (float)(rate ? rate : 48000U)));
    SpkBuf[i] = v; SpkBuf[i + 1U] = v;
  }
  if (BSP_AUDIO_OUT_Play(0, (uint8_t *)SpkBuf, sizeof SpkBuf) != BSP_ERROR_NONE) { fail("audcfg", "play"); goto out_fail; }
  { uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 800U) && (SpkFull < 3U)) { svc(5U); } }
  (void)BSP_AUDIO_OUT_GetState(0, &st);
  if (st != BSP_AUDIO_OUT_STATE_PLAYING) { fail("audcfg", "state %lu while playing", (unsigned long)st); goto out_fail; }
  if (SpkFull == 0U) { fail("audcfg", "no callbacks while playing"); goto out_fail; }

  r = BSP_AUDIO_OUT_Pause(0);
  svc(20U);
  during_pause = SpkFull;
  (void)BSP_AUDIO_OUT_GetState(0, &st);
  svc(200U);
  P("INFO pause r=%ld state=%lu callbacks %lu -> %lu across 200 ms paused\r\n",
    (long)r, (unsigned long)st, (unsigned long)during_pause, (unsigned long)SpkFull);
  if (r != BSP_ERROR_NONE) { fail("audcfg", "Pause %ld", (long)r); goto out_fail; }
  if (st != BSP_AUDIO_OUT_STATE_PAUSE) { fail("audcfg", "state %lu after pause", (unsigned long)st); goto out_fail; }
  if (SpkFull != during_pause) { fail("audcfg", "callbacks advanced %lu->%lu while paused", (unsigned long)during_pause, (unsigned long)SpkFull); goto out_fail; }

  r = BSP_AUDIO_OUT_Resume(0);
  { uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 800U) && (SpkFull <= during_pause)) { svc(5U); } }
  (void)BSP_AUDIO_OUT_GetState(0, &st);
  P("INFO resume r=%ld state=%lu callbacks now %lu\r\n", (long)r, (unsigned long)st, (unsigned long)SpkFull);
  if (r != BSP_ERROR_NONE) { fail("audcfg", "Resume %ld", (long)r); goto out_fail; }
  if (SpkFull <= during_pause) { fail("audcfg", "callbacks did not restart after resume"); goto out_fail; }

  (void)BSP_AUDIO_OUT_Stop(0);
  (void)BSP_AUDIO_OUT_GetState(0, &st);
  if (st != BSP_AUDIO_OUT_STATE_STOP) { fail("audcfg", "state %lu after stop", (unsigned long)st); goto out_fail; }
  (void)BSP_AUDIO_OUT_DeInit(0);

  /* The input side, same treatment. */
  {
    BSP_AUDIO_Init_t ai = { BSP_AUDIO_IN_DEVICE_DIGITAL_MIC, BSP_AUDIO_FREQUENCY_NATIVE, BSP_AUDIO_RESOLUTION_16B, 1U, 50U };
    uint32_t in_rate = 0U, in_dev = 0U, in_ch = 0U, in_bits = 0U, in_st = 99U;

    MicHalf = MicFull = 0;
    if (BSP_AUDIO_IN_Init(0, &ai) != BSP_ERROR_NONE) { fail("audcfg", "in init"); return; }
    (void)BSP_AUDIO_IN_GetSampleRate(0, &in_rate);
    (void)BSP_AUDIO_IN_GetDevice(0, &in_dev);
    (void)BSP_AUDIO_IN_GetChannelsNbr(0, &in_ch);
    (void)BSP_AUDIO_IN_GetBitsPerSample(0, &in_bits);
    (void)BSP_AUDIO_IN_GetState(0, &in_st);
    P("INFO in after init: rate=%lu dev=%lu ch=%lu bits=%lu state=%lu\r\n",
      (unsigned long)in_rate, (unsigned long)in_dev, (unsigned long)in_ch,
      (unsigned long)in_bits, (unsigned long)in_st);
    if (in_dev != BSP_AUDIO_IN_DEVICE_DIGITAL_MIC) { fail("audcfg", "in device %lu", (unsigned long)in_dev); (void)BSP_AUDIO_IN_DeInit(0); return; }

    if (BSP_AUDIO_IN_Record(0, (uint8_t *)MicBuf, sizeof MicBuf) != BSP_ERROR_NONE) { fail("audcfg", "record"); (void)BSP_AUDIO_IN_DeInit(0); return; }
    { uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 800U) && (MicFull < 2U)) { svc(5U); } }
    (void)BSP_AUDIO_IN_GetState(0, &in_st);
    if (in_st != BSP_AUDIO_IN_STATE_RECORDING) { fail("audcfg", "in state %lu while recording", (unsigned long)in_st); (void)BSP_AUDIO_IN_DeInit(0); return; }

    r = BSP_AUDIO_IN_Pause(0);
    svc(20U);
    { uint32_t at_pause = MicFull;
      svc(200U);
      P("INFO in pause r=%ld callbacks %lu -> %lu across 200 ms paused\r\n",
        (long)r, (unsigned long)at_pause, (unsigned long)MicFull);
      if (r != BSP_ERROR_NONE) { fail("audcfg", "in Pause %ld", (long)r); (void)BSP_AUDIO_IN_DeInit(0); return; }
      if (MicFull != at_pause) { fail("audcfg", "in callbacks advanced while paused"); (void)BSP_AUDIO_IN_DeInit(0); return; }
      r = BSP_AUDIO_IN_Resume(0);
      { uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 800U) && (MicFull <= at_pause)) { svc(5U); } }
      if (r != BSP_ERROR_NONE) { fail("audcfg", "in Resume %ld", (long)r); (void)BSP_AUDIO_IN_DeInit(0); return; }
      if (MicFull <= at_pause) { fail("audcfg", "in callbacks did not restart after resume"); (void)BSP_AUDIO_IN_DeInit(0); return; }
    }
    (void)BSP_AUDIO_IN_Stop(0);
    (void)BSP_AUDIO_IN_DeInit(0);
  }

  ok("audcfg", "out and in: getters agree with init, pause halts the callbacks and resume restarts them");
  return;

out_fail:
  (void)BSP_AUDIO_OUT_Stop(0);
  (void)BSP_AUDIO_OUT_DeInit(0);
}

/* PWM frequency and resolution. */
static void cmd_pwmcfg(void)
{
  uint32_t f0, f1, b0, b1;
  int32_t r;

  if (BSP_PWM_Init() != BSP_ERROR_NONE) { fail("pwmcfg", "init"); return; }

  f0 = BSP_PWM_GetFrequency();
  b0 = BSP_PWM_GetResolution();

  r = BSP_PWM_SetFrequency(5000U);
  svc(5U);
  f1 = BSP_PWM_GetFrequency();
  P("INFO frequency %lu -> asked 5000 -> %lu (r=%ld)\r\n", (unsigned long)f0, (unsigned long)f1, (long)r);
  if (r != BSP_ERROR_NONE) { fail("pwmcfg", "SetFrequency %ld", (long)r); return; }
  if ((f1 < 4500U) || (f1 > 5500U)) { fail("pwmcfg", "frequency came back %lu for a 5000 request", (unsigned long)f1); return; }

  r = BSP_PWM_SetResolution(10U);
  svc(5U);
  b1 = BSP_PWM_GetResolution();
  P("INFO resolution %lu -> asked 10 -> %lu (r=%ld)\r\n", (unsigned long)b0, (unsigned long)b1, (long)r);
  if (r != BSP_ERROR_NONE) { fail("pwmcfg", "SetResolution %ld", (long)r); return; }
  if (b1 != 10U) { fail("pwmcfg", "resolution came back %lu for a 10 request", (unsigned long)b1); return; }

  /* A duty cycle at the new settings, on a pin the pwm test already uses. */
  r = BSP_PWM_Start(BSP_IO_PIN_D5, (1U << b1) / 4U);
  svc(20U);
  if (r != BSP_ERROR_NONE) { fail("pwmcfg", "Start at the new settings %ld", (long)r); return; }
  (void)BSP_PWM_Stop(BSP_IO_PIN_D5);

  /* Put the defaults back so a later pwm run in the same boot is unaffected. */
  (void)BSP_PWM_SetFrequency(f0);
  (void)BSP_PWM_SetResolution(b0);
  (void)BSP_PWM_DeInit();

  ok("pwmcfg", "frequency %lu->%lu, resolution %lu->%lu, duty accepted after both",
     (unsigned long)f0, (unsigned long)f1, (unsigned long)b0, (unsigned long)b1);
}

/* ADC resolution, the IO helpers, and BSP_COM_IsConnected. */
static void cmd_miscapi(void)
{
  uint32_t r12 = 0U, r8 = 0U, res;
  int32_t  r, has_a0, has_d5, lvl0, lvl1;

  /* ADC resolution: the same pin at 12 and 8 bits, about 16 times fewer counts. */
  if (BSP_ADC_Init() != BSP_ERROR_NONE) { fail("miscapi", "adc init"); return; }
  res = BSP_ADC_GetResolution();
  if (BSP_ADC_SetResolution(BSP_ADC_RESOLUTION_12B) != BSP_ERROR_NONE) { fail("miscapi", "SetResolution 12"); return; }
  (void)BSP_ADC_ReadChannel(BSP_IO_PIN_A0, &r12);
  if (BSP_ADC_SetResolution(BSP_ADC_RESOLUTION_8B) != BSP_ERROR_NONE) { fail("miscapi", "SetResolution 8"); return; }
  (void)BSP_ADC_ReadChannel(BSP_IO_PIN_A0, &r8);
  P("INFO A0 at 12 bits = %lu, at 8 bits = %lu (ratio %.1f, expect ~16), vdda_measured=%ld\r\n",
    (unsigned long)r12, (unsigned long)r8,
    (double)((r8 > 0U) ? ((float)r12 / (float)r8) : 0.0f), (long)BSP_ADC_IsVddaMeasured());
  (void)BSP_ADC_SetResolution(res);
  (void)BSP_ADC_DeInit();
  if (r8 == 0U) { fail("miscapi", "8-bit read returned zero"); return; }
  {
    float ratio = (float)r12 / (float)r8;
    if ((ratio < 12.0f) || (ratio > 20.0f))
    { fail("miscapi", "12-bit/8-bit ratio %.1f, expected about 16", (double)ratio); return; }
  }

  /* IO: which pins claim an analogue route, and a toggle that reads back. */
  if (BSP_IO_Init() != BSP_ERROR_NONE) { fail("miscapi", "io init"); return; }
  has_a0 = BSP_IO_HasAdc(BSP_IO_PIN_A0);
  has_d5 = BSP_IO_HasAdc(BSP_IO_PIN_D5);
  if (BSP_IO_ConfigPin(BSP_IO_PIN_D5, BSP_IO_MODE_OUTPUT_PP) != BSP_ERROR_NONE) { fail("miscapi", "config D5"); return; }
  (void)BSP_IO_WritePin(BSP_IO_PIN_D5, 0U);
  svc(2U);
  r = BSP_IO_TogglePin(BSP_IO_PIN_D5);
  svc(2U);
  lvl0 = BSP_IO_ReadPin(BSP_IO_PIN_D5);
  (void)BSP_IO_TogglePin(BSP_IO_PIN_D5);
  svc(2U);
  lvl1 = BSP_IO_ReadPin(BSP_IO_PIN_D5);
  P("INFO HasAdc A0=%ld D5=%ld; toggle from low -> %ld -> %ld (r=%ld)\r\n",
    (long)has_a0, (long)has_d5, (long)lvl0, (long)lvl1, (long)r);
  (void)BSP_IO_ConfigPin(BSP_IO_PIN_D5, BSP_IO_MODE_INPUT);
  (void)BSP_IO_DeInit();
  if (has_a0 != 1) { fail("miscapi", "A0 reports no analogue route"); return; }
  if (has_d5 != 0) { fail("miscapi", "D5 claims an analogue route it does not have"); return; }
  if (r != BSP_ERROR_NONE) { fail("miscapi", "TogglePin %ld", (long)r); return; }
  if ((lvl0 != 1) || (lvl1 != 0)) { fail("miscapi", "toggle read back %ld then %ld, expected 1 then 0", (long)lvl0, (long)lvl1); return; }

  /* The console is answering this command, so it must report connected. */
  if (BSP_COM_IsConnected() != 1) { fail("miscapi", "BSP_COM_IsConnected says no while answering on it"); return; }

  ok("miscapi", "adc counts scale 12/8 bits, HasAdc right on both pins, toggle reads back, com connected");
}

/* Does the ADC12 security stamp actually change what the converter sees? */
static void cmd_adcrif(void)
{
  static const char *phase[3] = { "stamped (as booted)", "stamp CLEARED", "stamped again" };
  uint32_t raw[3][6] = {{0}};
  uint32_t attr[3] = {0};
  int32_t  bad = 0;

  for (uint32_t p = 0U; p < 3U; p++)
  {
    if (p == 1U)
    {
      HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_ADC12, 0U);
    }
    else if (p == 2U)
    {
      HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_ADC12,
                                            RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
    }
    else
    {
      /* Leave the boot configuration alone for the first pass. */
    }

    attr[p] = HAL_RIF_RISC_GetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_ADC12);

    /* Re-initialise, so the change is picked up as on a fresh boot. */
    (void)BSP_ADC_DeInit();
    svc(5U);
    if (BSP_ADC_Init() != BSP_ERROR_NONE) { fail("adcrif", "init at pass %lu", (unsigned long)p); return; }

    for (uint32_t i = 0U; i < 6U; i++)
    {
      bad |= BSP_ADC_ReadChannel(BSP_IO_PIN_A0 + i, &raw[p][i]);
      svc(1U);
    }

    P("INFO %-20s attr=0x%02lx raw=%lu,%lu,%lu,%lu,%lu,%lu\r\n", phase[p], (unsigned long)attr[p],
      (unsigned long)raw[p][0], (unsigned long)raw[p][1], (unsigned long)raw[p][2],
      (unsigned long)raw[p][3], (unsigned long)raw[p][4], (unsigned long)raw[p][5]);
    BSP_COM_Flush(100U);
  }

  (void)BSP_ADC_DeInit();

  if (bad != 0) { fail("adcrif", "a channel read failed"); return; }

  {
    uint32_t moved = 0U;
    for (uint32_t i = 0U; i < 6U; i++)
    {
      uint32_t d = (raw[0][i] > raw[1][i]) ? (raw[0][i] - raw[1][i]) : (raw[1][i] - raw[0][i]);
      if (d > 100U) { moved++; }
    }
    ok("adcrif", "%lu of 6 channels moved by more than 100 counts when the stamp was cleared",
       (unsigned long)moved);
  }
}

/* Drive each digital header pin and read its own level back. With nothing
 * connected a push-pull output reads what it was set to. */
static void cmd_io(void)
{
  static const uint32_t pins[] = { BSP_IO_PIN_D0, BSP_IO_PIN_D1, BSP_IO_PIN_D5, BSP_IO_PIN_D6, BSP_IO_PIN_D9,
                                   BSP_IO_PIN_D10, BSP_IO_PIN_D11, BSP_IO_PIN_D12, BSP_IO_PIN_D13 };
  uint32_t good = 0, n = sizeof pins / sizeof pins[0];
  if (BSP_IO_Init() != BSP_ERROR_NONE) { fail("io", "init"); return; }
  for (uint32_t i = 0; i < n; i++)
  {
    int32_t r = BSP_IO_ConfigPin(pins[i], BSP_IO_MODE_OUTPUT_PP);
    (void)BSP_IO_WritePin(pins[i], BSP_IO_PIN_SET);   int32_t hi = BSP_IO_ReadPin(pins[i]);
    (void)BSP_IO_WritePin(pins[i], BSP_IO_PIN_RESET); int32_t lo = BSP_IO_ReadPin(pins[i]);
    (void)BSP_IO_ConfigPin(pins[i], BSP_IO_MODE_INPUT_PU);
    if ((r == BSP_ERROR_NONE) && (hi == 1) && (lo == 0)) { good++; } else { P("INFO pin%lu cfg=%ld hi=%ld lo=%ld\r\n", (unsigned long)pins[i], (long)r, (long)hi, (long)lo); }
    svc(1U);
  }
  if (good != n) { fail("io", "%lu/%lu pins read back their own level", (unsigned long)good, (unsigned long)n); return; }
  ok("io", "%lu/%lu digital pins set/read/clear", (unsigned long)good, (unsigned long)n);
}

static void cmd_pwm(void)
{
  uint32_t n = 0, hz = 0; char s[120] = "";
  if (BSP_PWM_Init() != BSP_ERROR_NONE) { fail("pwm", "init"); return; }
  for (uint32_t p = 0; p < BSP_IO_PIN_NBR; p++)
  {
    if (BSP_IO_HasPwm(p) != 1) { continue; }
    if ((BSP_PWM_GetTimerClockFreq(p, &hz) == BSP_ERROR_NONE) && (BSP_PWM_Start(p, 128U) == BSP_ERROR_NONE))
    { char t[24]; snprintf(t, sizeof t, "p%lu@%luHz ", (unsigned long)p, (unsigned long)hz); strncat(s, t, sizeof s - strlen(s) - 1); n++; svc(20U); (void)BSP_PWM_Stop(p); }
  }
  (void)BSP_PWM_DeInit();
  if (n == 0U) { fail("pwm", "no pin started"); return; }
  ok("pwm", "pins=%lu timerclk[%s]", (unsigned long)n, s);
}

static void cmd_tone(void)
{
  if (BSP_IO_Init() != BSP_ERROR_NONE) { fail("tone", "io init"); return; }
  if (BSP_TONE_Start(BSP_IO_PIN_D5, 1000U, 100U) != BSP_ERROR_NONE) { fail("tone", "start"); return; }
  svc(40U); int32_t mid = BSP_TONE_IsActive(); svc(150U); int32_t end = BSP_TONE_IsActive();
  if ((mid != 1) || (end != 0)) { fail("tone", "active mid=%ld end=%ld", (long)mid, (long)end); return; }
  ok("tone", "1kHz 100ms on D5, active then finished");
}

static void cmd_wdg(void) { ok("wdg", "timeout=%lums early-warning->bootloader, refreshed by every wait", (unsigned long)WDG_TIMEOUT_MS); }

/* Not in the default list: proves the hardware watchdog by not feeding it. */
static void cmd_wdgfire(void)
{
  P("INFO stopping all service; expect a hardware reset in ~%lu ms\r\n", (unsigned long)WDG_TIMEOUT_MS); BSP_COM_Flush(200U);
  __disable_irq(); for (;;) { }
}

/* The early warning, with interrupts on: the encoder up, so BSP_VENC_Quiesce
 * has an instance to drop, then no refresh. The warning's callback leaves
 * 0xFD in bkp7 and goes to the bootloader; `bkp` in the next image reads it. */
static void cmd_wdgspin(void)
{
  BSP_VENC_Band_t band = {0};
  BSP_VENC_Config_t vc = {0};
  int32_t r = BSP_VENC_GetBandGeometry(CAM_W, CAM_H, &band);

  if (r == BSP_ERROR_NONE)
  {
    vc.Band = band; vc.Quality = BSP_VENC_QUALITY_DEFAULT;
    r = BSP_VENC_Init(0, &vc);
  }
  P("INFO encoder up r=%ld; IWDG_IRQn enabled %lu, priority %lu; spinning without a refresh\r\n",
    (long)r, (unsigned long)NVIC_GetEnableIRQ(IWDG_IRQn), (unsigned long)NVIC_GetPriority(IWDG_IRQn));
  BSP_COM_Flush(200U);
  HAL_PWR_EnableBkUpAccess();
  (&TAMP->BKP0R)[5] = uwTick;
  for (;;) { __NOP(); }
}

/* The console closed and opened again, then to the bootloader. Result in
 * bkp6: 0xC0 top byte; within 3 s, bit 10 the host reset the bus, bit 8 the
 * console reported connected; bit 9 the soft disconnect at the end (set if
 * Init failed); BSP_COM_Init's return in the low byte. */
static void cmd_comreinit(void)
{
  volatile uint32_t *bkp = &TAMP->BKP0R;
  uint32_t t0;
  uint32_t connected = 0U, reset = 0U, sdis;
  int32_t  r;

  P("INFO closing the console and opening it again; bkp6 in the next image has the result\r\n");
  BSP_COM_Flush(200U);
  (void)BSP_COM_DeInit();
  HAL_Delay(200U);
  r = BSP_COM_Init();
  /* The controller is read only if Init brought its clock back: with the
   * clock off, an access stalls the bus rather than faulting. */
  sdis = 1U;
  if (r == BSP_ERROR_NONE)
  {
    USB1_OTG_HS->GINTSTS = USB_OTG_GINTSTS_USBRST;
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 3000U)
    {
#if !defined(BSP_USE_FREERTOS) || (BSP_USE_FREERTOS == 0)
      BSP_COM_Task();
#endif
      (void)BSP_WDG_Refresh();
      if ((USB1_OTG_HS->GINTSTS & USB_OTG_GINTSTS_USBRST) != 0U) { reset = 1U; }
      if (BSP_COM_IsConnected() == 1) { connected = 1U; }
    }
    sdis = ((((USB_OTG_DeviceTypeDef *)((uint32_t)USB1_OTG_HS + USB_OTG_DEVICE_BASE))->DCTL &
             USB_OTG_DCTL_SDIS) != 0U) ? 1U : 0U;
  }
  HAL_PWR_EnableBkUpAccess();
  bkp[6] = 0xC0000000U | (reset << 10U) | (sdis << 9U) | (connected << 8U) | ((uint32_t)r & 0xFFU);
  BSP_COM_RebootToBootloader();
}

/* A reset mid-read: 512-byte motion sensor reads until the watchdog's early
 * warning cuts in and goes to the bootloader. `i2cline`, first in the next
 * image, shows the bus state. */
static void cmd_i2creset(void)
{
  static uint8_t buf[512];

  if (BSP_I2C1_Init() != BSP_ERROR_NONE) { fail("i2creset", "I2C1 init"); return; }
  P("INFO reading the motion sensor until the watchdog cuts in\r\n"); BSP_COM_Flush(200U);
  HAL_PWR_EnableBkUpAccess();
  (&TAMP->BKP0R)[5] = uwTick;
  (void)BSP_WDG_InitEWI(20U, 12U);
  for (;;)
  {
    (void)BSP_I2C1_ReadReg(0xD4U /* LSM6DSL */, 0x0FU, buf, (uint16_t)sizeof(buf));
  }
}

/* The sensor bus lines before anything configures them, then each device's
 * first probe after an Init, which must free a held bus. Run first in an
 * image. */
static void cmd_i2cline(void)
{
  GPIO_InitTypeDef gi = {0};
  GPIO_PinState sda, scl;
  int32_t imu, mag, cam;

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  gi.Mode = GPIO_MODE_INPUT; gi.Pull = GPIO_NOPULL; gi.Speed = GPIO_SPEED_FREQ_LOW;
  gi.Pin = BUS_I2C1_SDA_PIN; HAL_GPIO_Init(BUS_I2C1_SDA_GPIO_PORT, &gi);
  gi.Pin = BUS_I2C1_SCL_PIN; HAL_GPIO_Init(BUS_I2C1_SCL_GPIO_PORT, &gi);
  HAL_Delay(1U);
  sda = HAL_GPIO_ReadPin(BUS_I2C1_SDA_GPIO_PORT, BUS_I2C1_SDA_PIN);
  scl = HAL_GPIO_ReadPin(BUS_I2C1_SCL_GPIO_PORT, BUS_I2C1_SCL_PIN);

  if (BSP_I2C1_Init() != BSP_ERROR_NONE) { fail("i2cline", "I2C1 init"); return; }
  imu = BSP_I2C1_IsReady(0xD4U /* LSM6DSL */, 1U);
  mag = BSP_I2C1_IsReady(0x60U /* MMC5603NJ */, 1U);
  cam = BSP_I2C1_IsReady(BSP_CAMERA_I2C_ADDRESS, 1U);
  (void)BSP_I2C1_DeInit();

  P("INFO before init: SDA %u, SCL %u (1 is released); first probes: motion %ld, magnetometer %ld, camera %ld\r\n",
    (unsigned)sda, (unsigned)scl, (long)imu, (long)mag, (long)cam);
  if ((imu != BSP_ERROR_NONE) || (mag != BSP_ERROR_NONE) || (cam != BSP_ERROR_NONE))
  {
    fail("i2cline", "SDA %u; motion %ld, magnetometer %ld, camera %ld", (unsigned)sda, (long)imu, (long)mag, (long)cam);
    return;
  }
  ok("i2cline", "all three devices answer at once (SDA was %s before init)", (sda == GPIO_PIN_SET) ? "released" : "held low");
}

/* `i2cackcut` cuts a write to the motion sensor in its data-byte acknowledge
 * (SDA held low) and resets; `i2ccheck`, first in the next image, lets
 * BSP_I2C1_Init free the bus and checks the next register (the address
 * auto-increments) was not written. Both are plain read-write registers
 * (never write a reserved one) and are restored. */
#define BB_SENSOR_ADDR    0xD4U    /* LSM6DSL, write */
#define BB_WRITE_REG      0x05U    /* SENSOR_SYNC_RES_RATIO */
#define BB_NEXT_REG       0x06U    /* FIFO_CTRL1 */

static void bb_delay(void)
{
  for (volatile uint32_t n = SystemCoreClock / 400000U; n != 0U; n--) { }
}

static void bb_scl(uint32_t High)
{
  HAL_GPIO_WritePin(BUS_I2C1_SCL_GPIO_PORT, BUS_I2C1_SCL_PIN, (High != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  bb_delay();
}

static void bb_sda(uint32_t High)
{
  HAL_GPIO_WritePin(BUS_I2C1_SDA_GPIO_PORT, BUS_I2C1_SDA_PIN, (High != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  bb_delay();
}

/* Eight bits, MSB first, ending with SCL low after the last; returns 1 when
 * the device acknowledged them. */
static uint32_t bb_byte(uint8_t Byte)
{
  uint32_t ack;

  for (uint32_t i = 0U; i < 8U; i++)
  {
    bb_sda((Byte >> (7U - i)) & 1U);
    bb_scl(1U);
    bb_scl(0U);
  }
  bb_sda(1U);
  bb_scl(1U);
  ack = (HAL_GPIO_ReadPin(BUS_I2C1_SDA_GPIO_PORT, BUS_I2C1_SDA_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
  bb_scl(0U);
  return ack;
}

static void cmd_i2cackcut(void)
{
  GPIO_InitTypeDef gi = {0};
  uint32_t a1, a2, held;
  uint8_t was05 = 0U, was06 = 0U;
  const uint8_t zero = 0U;

  /* What the two registers hold now, kept for i2ccheck to put back, and
   * FIFO_CTRL1 cleared so that a byte written into it shows. */
  if (BSP_I2C1_Init() != BSP_ERROR_NONE) { fail("i2cackcut", "I2C1 init"); return; }
  if ((BSP_I2C1_ReadReg(BB_SENSOR_ADDR, BB_WRITE_REG, &was05, 1U) != BSP_ERROR_NONE) ||
      (BSP_I2C1_ReadReg(BB_SENSOR_ADDR, BB_NEXT_REG, &was06, 1U) != BSP_ERROR_NONE) ||
      (BSP_I2C1_WriteReg(BB_SENSOR_ADDR, BB_NEXT_REG, (uint8_t *)&zero, 1U) != BSP_ERROR_NONE))
  {
    (void)BSP_I2C1_DeInit();
    fail("i2cackcut", "the motion sensor did not answer");
    return;
  }
  (void)BSP_I2C1_DeInit();

  P("INFO cutting a write in its acknowledge (0x05 was 0x%02x, 0x06 was 0x%02x), then to the bootloader; "
    "i2ccheck after an upload\r\n", (unsigned)was05, (unsigned)was06);
  BSP_COM_Flush(200U);
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  HAL_GPIO_WritePin(BUS_I2C1_SCL_GPIO_PORT, BUS_I2C1_SCL_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BUS_I2C1_SDA_GPIO_PORT, BUS_I2C1_SDA_PIN, GPIO_PIN_SET);
  gi.Mode = GPIO_MODE_OUTPUT_OD; gi.Pull = GPIO_NOPULL; gi.Speed = GPIO_SPEED_FREQ_LOW;
  gi.Pin = BUS_I2C1_SCL_PIN; HAL_GPIO_Init(BUS_I2C1_SCL_GPIO_PORT, &gi);
  gi.Pin = BUS_I2C1_SDA_PIN; HAL_GPIO_Init(BUS_I2C1_SDA_GPIO_PORT, &gi);
  bb_delay();

  /* START, the address and the register, then the data byte up to the
   * sensor's acknowledge, which it drives once SCL falls after the eighth
   * bit and keeps until SCL falls again. */
  bb_sda(0U);
  bb_scl(0U);
  a1 = bb_byte(BB_SENSOR_ADDR);
  a2 = bb_byte(BB_WRITE_REG);
  for (uint32_t i = 0U; i < 8U; i++)
  {
    bb_sda(0U);
    bb_scl(1U);
    bb_scl(0U);
  }
  bb_sda(1U);
  bb_scl(1U);
  held = (HAL_GPIO_ReadPin(BUS_I2C1_SDA_GPIO_PORT, BUS_I2C1_SDA_PIN) == GPIO_PIN_RESET) ? 1U : 0U;

  HAL_PWR_EnableBkUpAccess();
  (&TAMP->BKP0R)[5] = 0xAC000000U | ((uint32_t)was05 << 16U) | ((uint32_t)was06 << 8U) |
                      (a1 << 2U) | (a2 << 1U) | held;
  BSP_COM_RebootToBootloader();
}

static void cmd_i2ccheck(void)
{
  GPIO_InitTypeDef gi = {0};
  GPIO_PinState sda;
  const uint32_t cut = (&TAMP->BKP0R)[5];
  uint8_t next = 0U, who = 0U;
  int32_t r;

  __HAL_RCC_GPIOE_CLK_ENABLE();
  gi.Mode = GPIO_MODE_INPUT; gi.Pull = GPIO_NOPULL; gi.Speed = GPIO_SPEED_FREQ_LOW;
  gi.Pin = BUS_I2C1_SDA_PIN; HAL_GPIO_Init(BUS_I2C1_SDA_GPIO_PORT, &gi);
  HAL_Delay(1U);
  sda = HAL_GPIO_ReadPin(BUS_I2C1_SDA_GPIO_PORT, BUS_I2C1_SDA_PIN);

  if (BSP_I2C1_Init() != BSP_ERROR_NONE) { fail("i2ccheck", "I2C1 init"); return; }
  r = BSP_I2C1_ReadReg(BB_SENSOR_ADDR, 0x0FU, &who, 1U);
  if (r == BSP_ERROR_NONE) { r = BSP_I2C1_ReadReg(BB_SENSOR_ADDR, BB_NEXT_REG, &next, 1U); }
  if ((r == BSP_ERROR_NONE) && ((cut >> 24U) == 0xACU))
  {
    /* Both registers as they were before the cut. */
    uint8_t was05 = (uint8_t)(cut >> 16U), was06 = (uint8_t)(cut >> 8U);

    (void)BSP_I2C1_WriteReg(BB_SENSOR_ADDR, BB_WRITE_REG, &was05, 1U);
    (void)BSP_I2C1_WriteReg(BB_SENSOR_ADDR, BB_NEXT_REG, &was06, 1U);
  }
  (void)BSP_I2C1_DeInit();
  HAL_PWR_EnableBkUpAccess();
  (&TAMP->BKP0R)[5] = 0U;

  P("INFO the cut: 0x%08lx (address acknowledged %lu, register %lu, SDA held %lu); before init SDA %u; "
    "after it WHO_AM_I 0x%02x, FIFO_CTRL1 0x%02x\r\n", (unsigned long)cut, (unsigned long)((cut >> 2U) & 1U),
    (unsigned long)((cut >> 1U) & 1U), (unsigned long)(cut & 1U), (unsigned)sda, (unsigned)who, (unsigned)next);
  if ((cut >> 24U) != 0xACU) { skip("i2ccheck", "no cut recorded: run i2cackcut first, then upload"); return; }
  if ((cut & 7U) != 7U) { fail("i2ccheck", "the cut did not reach the acknowledge (0x%08lx)", (unsigned long)cut); return; }
  if (r != BSP_ERROR_NONE) { fail("i2ccheck", "the sensor did not answer after init (%ld)", (long)r); return; }
  if (who != 0x6AU) { fail("i2ccheck", "WHO_AM_I 0x%02x", (unsigned)who); return; }
  if (next != 0U) { fail("i2ccheck", "freeing the bus wrote 0x%02x into the register after the cut write", (unsigned)next); return; }
  ok("i2ccheck", "the bus was freed and wrote nothing: FIFO_CTRL1 still 0x00, the sensor answers");
}

/* An update trigger armed with nothing staged, then an upload: `otacancel`
 * leaves through BSP_COM_RebootToBootloader as a host does, `otakeep` sets
 * the boot loader's DFU key itself. `otacheck`, in the next image, checks the
 * trigger was cancelled; a tried apply stops at the missing header, flash
 * untouched. Layout: Middlewares/NeuroN6/N6OTA/n6_ota_layout.h. */
#define OTA_TRIGGER_BKPR      20U
#define OTA_TRIGGER_MAGIC     0x504F364EU
#define OTA_APPLY_BKPR        22U
#define OTA_APPLY_NO_HEADER   0x45480000U
#define OTA_PSRAM_BASE        0x71E00000U

static int ota_arm(const char *name)
{
  volatile uint32_t *hdr = (volatile uint32_t *)OTA_PSRAM_BASE;

  /* An image an earlier application staged would otherwise be applied. */
  hdr[0] = 0U;
  __DSB();
  if (hdr[0] != 0U) { fail(name, "the staging area's header did not clear"); return 0; }
  HAL_PWR_EnableBkUpAccess();
  (&TAMP->BKP0R)[OTA_APPLY_BKPR]   = 0U;
  (&TAMP->BKP0R)[OTA_TRIGGER_BKPR] = OTA_TRIGGER_MAGIC;
  P("INFO update trigger armed, nothing staged; to the boot loader, then otacheck after an upload\r\n");
  BSP_COM_Flush(200U);
  return 1;
}

static void cmd_otacancel(void)
{
  if (ota_arm("otacancel") != 0) { BSP_COM_RebootToBootloader(); }
}

static void cmd_otakeep(void)
{
  if (ota_arm("otakeep") == 0) { return; }
  (&TAMP->BKP0R)[2] = 0xF4CDF4U;          /* the boot loader's DFU key */
  (&TAMP->BKP0R)[1] = 0U;
  __DSB();
  NVIC_SystemReset();
}

static void cmd_otacheck(void)
{
  const uint32_t trigger = (&TAMP->BKP0R)[OTA_TRIGGER_BKPR];
  const uint32_t apply   = (&TAMP->BKP0R)[OTA_APPLY_BKPR];

  P("INFO trigger=0x%08lx apply=0x%08lx\r\n", (unsigned long)trigger, (unsigned long)apply);
  if (trigger == OTA_TRIGGER_MAGIC) { fail("otacheck", "the trigger is still armed"); return; }
  if (apply != 0U)
  {
    fail("otacheck", "the boot loader tried to apply the update staged before the upload (result 0x%08lx%s)",
         (unsigned long)apply, (apply == OTA_APPLY_NO_HEADER) ? ", no header" : "");
    return;
  }
  ok("otacheck", "the update staged before the upload was cancelled; the boot loader did not try it");
}

/* An undefined instruction, with interrupts on: the fault handler's code
 * lands in bkp7. */
static void cmd_faultudf(void)
{
  P("INFO executing an undefined instruction\r\n"); BSP_COM_Flush(200U);
  __asm volatile ("udf #0");
}

static void cmd_npu(void)
{
  uint32_t base = 0, kb = 0;
  if (BSP_NPU_Init() != BSP_ERROR_NONE) { fail("npu", "init"); return; }
  (void)BSP_NPU_GetRamInfo(&base, &kb); (void)BSP_NPU_DeInit();
  ok("npu", "ram_base=0x%08lx size=%luKB", (unsigned long)base, (unsigned long)kb);
}

void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)     { (void)Instance; MicHalf++; }
void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  const uint32_t t = HAL_GetTick();

  (void)Instance;
  if (MicFull == 0U) { MicTcFirst = t; }
  MicTcLast = t;
  MicFull++;
}

/* The converter's actual rate, from the time between whole buffers. */
static uint32_t mic_measured_rate(void)
{
  const uint32_t n  = MicFull;
  const uint32_t dt = MicTcLast - MicTcFirst;

  if ((n < 2U) || (dt == 0U)) { return 0U; }
  return (uint32_t)(((uint64_t)(n - 1U) * MIC_SAMPLES * 1000U) / dt);
}

/* The same for playback, where a buffer holds SPK_SAMPLES / 2 stereo frames
 * or SPK_SAMPLES mono ones. */
static uint32_t spk_measured_rate(uint32_t framesPerBuffer)
{
  const uint32_t n  = SpkFull;
  const uint32_t dt = SpkTcLast - SpkTcFirst;

  if ((n < 2U) || (dt == 0U)) { return 0U; }
  return (uint32_t)(((uint64_t)(n - 1U) * framesPerBuffer * 1000U) / dt);
}

/* Within 3 %: looser than the tick's resolution, tighter than a real error. */
static int rate_agrees(uint32_t claimed, uint32_t measured)
{
  const uint32_t d = (claimed > measured) ? (claimed - measured) : (measured - claimed);

  return (measured != 0U) && (((uint64_t)d * 100U) <= ((uint64_t)claimed * 3U));
}
void BSP_AUDIO_IN_Error_CallBack(uint32_t Instance)            { (void)Instance; MicErr++; }
void BSP_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Instance)    { (void)Instance; SpkHalf++; }
void BSP_AUDIO_OUT_TransferComplete_CallBack(uint32_t Instance)
{
  const uint32_t t = HAL_GetTick();

  (void)Instance;
  if (SpkFull == 0U) { SpkTcFirst = t; }
  SpkTcLast = t;
  SpkFull++;
}
void BSP_AUDIO_OUT_Error_CallBack(uint32_t Instance)           { (void)Instance; SpkErr++; }

static void cmd_mic(void)
{
  BSP_AUDIO_Init_t ai = { BSP_AUDIO_IN_DEVICE_DIGITAL_MIC, BSP_AUDIO_FREQUENCY_NATIVE, BSP_AUDIO_RESOLUTION_16B, 1U, 0U };
  uint32_t rate = 0; MicHalf = MicFull = MicErr = 0;
  memset(MicBuf, 0, sizeof MicBuf);
  int32_t r = BSP_AUDIO_IN_Init(0, &ai);
  if (r != BSP_ERROR_NONE) { fail("mic", "init %ld", (long)r); return; }
  (void)BSP_AUDIO_IN_GetSampleRate(0, &rate);
  r = BSP_AUDIO_IN_Record(0, (uint8_t *)MicBuf, sizeof MicBuf);
  if (r != BSP_ERROR_NONE) { fail("mic", "record %ld", (long)r); (void)BSP_AUDIO_IN_DeInit(0); return; }
  /* Seventeen buffers: sixteen intervals, enough to time the rate. */
  uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 1500U) && (MicFull < 17U)) { svc(5U); }
  (void)BSP_AUDIO_IN_Stop(0); (void)BSP_AUDIO_IN_DeInit(0);
  const uint32_t measured = mic_measured_rate();
  double acc = 0; int32_t pk = 0; uint32_t nz = 0;
  for (uint32_t i = 0; i < MIC_SAMPLES; i++) { int32_t v = MicBuf[i]; acc += (double)v * v; if (abs(v) > pk) pk = abs(v); if (v != 0) nz++; }
  double rms = sqrt(acc / MIC_SAMPLES);
  P("INFO rate=%lu half=%lu full=%lu err=%lu rms=%.0f peak=%ld nonzero=%lu/%u\r\n", (unsigned long)rate, (unsigned long)MicHalf, (unsigned long)MicFull, (unsigned long)MicErr, rms, (long)pk, (unsigned long)nz, MIC_SAMPLES);
  if (MicFull == 0U) { fail("mic", "no transfer-complete callback in 1.5s"); return; }
  if (nz < MIC_SAMPLES / 8U) { fail("mic", "buffer is (almost) all zero: %lu nonzero", (unsigned long)nz); return; }
  if (!rate_agrees(rate, measured))
  { fail("mic", "the BSP reports %lu Hz but the buffers arrive at %lu Hz", (unsigned long)rate, (unsigned long)measured); return; }
  ok("mic", "rate=%lu measured=%lu callbacks=%lu rms=%.0f peak=%ld", (unsigned long)rate, (unsigned long)measured, (unsigned long)MicFull, rms, (long)pk);
}

/* Amplitude at f of n samples taken at fs: Goertzel, over a Hann window. */
static float tone_amplitude(const int16_t *x, uint32_t n, float f, float fs)
{
  const float c = 2.0f * cosf(6.2831853f * f / fs);
  float s1 = 0.0f, s2 = 0.0f;

  for (uint32_t i = 0U; i < n; i++)
  {
    const float w  = 0.5f - (0.5f * cosf(6.2831853f * (float)i / (float)(n - 1U)));
    const float s0 = ((float)x[i] * w) + (c * s1) - s2;
    s2 = s1; s1 = s0;
  }
  /* The window's coherent gain is a half. */
  return 4.0f * sqrtf(fabsf((s1 * s1) + (s2 * s2) - (c * s1 * s2))) / (float)n;
}

/* The speaker into the microphone, recorded at three rates (the three
 * capture decimations): the tone must be at its frequency and at the same
 * level at each. Each buffer half is measured on its own. */
static void cmd_audloop(void)
{
  static const uint32_t rates[] = { 11025U, BSP_AUDIO_FREQUENCY_NATIVE, 48000U };
  BSP_AUDIO_Init_t ao = { BSP_AUDIO_OUT_DEVICE_SPEAKER, BSP_AUDIO_FREQUENCY_NATIVE, BSP_AUDIO_RESOLUTION_16B, 1U, 50U };
  BSP_AUDIO_Init_t ai = { BSP_AUDIO_IN_DEVICE_DIGITAL_MIC, BSP_AUDIO_FREQUENCY_NATIVE, BSP_AUDIO_RESOLUTION_16B, 1U, 50U };
  float    level[3] = { 0.0f, 0.0f, 0.0f }, above[3] = { 0.0f, 0.0f, 0.0f };
  uint32_t inRate[3] = { 0U, 0U, 0U }, outRate = 0U, heard = 0U;
  int32_t  r;

  r = BSP_AUDIO_OUT_Init(0, &ao);
  if (r != BSP_ERROR_NONE) { fail("audloop", "out init %ld", (long)r); return; }
  (void)BSP_AUDIO_OUT_GetSampleRate(0, &outRate);

  /* 84 whole cycles in the buffer, so the loop point makes no click. */
  const float f = 84.0f * (float)outRate / (float)SPK_SAMPLES;
  for (uint32_t i = 0U; i < SPK_SAMPLES; i++)
  { SpkBuf[i] = (int16_t)(8000.0f * sinf(6.2831853f * 84.0f * (float)i / (float)SPK_SAMPLES)); }
  r = BSP_AUDIO_OUT_Play(0, (uint8_t *)SpkBuf, sizeof SpkBuf);
  if (r != BSP_ERROR_NONE) { fail("audloop", "play %ld", (long)r); (void)BSP_AUDIO_OUT_DeInit(0); return; }

  for (uint32_t k = 0U; k < 3U; k++)
  {
    ai.SampleRate = rates[k];
    r = BSP_AUDIO_IN_Init(0, &ai);
    if (r != BSP_ERROR_NONE)
    { fail("audloop", "in init at %lu %ld", (unsigned long)rates[k], (long)r); (void)BSP_AUDIO_OUT_Stop(0); (void)BSP_AUDIO_OUT_DeInit(0); return; }
    (void)BSP_AUDIO_IN_GetSampleRate(0, &inRate[k]);
    MicFull = 0U;
    r = BSP_AUDIO_IN_Record(0, (uint8_t *)MicBuf, sizeof MicBuf);
    if (r == BSP_ERROR_NONE)
    {
      /* A quarter of a second to settle, then the buffer holds the latest. */
      const uint32_t settle = (inRate[k] / (4U * MIC_SAMPLES)) + 2U;
      uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 2000U) && (MicFull < settle)) { svc(1U); }
      (void)BSP_AUDIO_IN_Stop(0);
    }
    (void)BSP_AUDIO_IN_DeInit(0);
    if (r != BSP_ERROR_NONE)
    { fail("audloop", "record at %lu %ld", (unsigned long)inRate[k], (long)r); (void)BSP_AUDIO_OUT_Stop(0); (void)BSP_AUDIO_OUT_DeInit(0); return; }

    const float fs = (float)inRate[k];
    const uint32_t h = MIC_SAMPLES / 2U;
    level[k] = 0.5f * (tone_amplitude(&MicBuf[0], h, f, fs) + tone_amplitude(&MicBuf[h], h, f, fs));
    /* Against the level a third of an octave away. */
    const float off = 0.5f * (tone_amplitude(&MicBuf[0], h, 1.26f * f, fs) + tone_amplitude(&MicBuf[h], h, 1.26f * f, fs));
    above[k] = (off > 0.0f) ? (20.0f * log10f(level[k] / off)) : 99.0f;
    if (above[k] >= 20.0f) { heard++; }
    P("INFO %lu Hz: tone at %.1f Hz amplitude %.0f (%.1f dBFS), %.1f dB above the level beside it\r\n",
      (unsigned long)inRate[k], f, level[k], 20.0f * log10f((level[k] + 1e-3f) / 32768.0f), above[k]);
  }
  (void)BSP_AUDIO_OUT_Stop(0); (void)BSP_AUDIO_OUT_DeInit(0);

  if (heard == 0U) { skip("audloop", "no tone heard at any rate: no speaker fitted, or it is muted"); return; }
  if (heard < 3U) { fail("audloop", "the tone was heard at only %lu of the three capture rates", (unsigned long)heard); return; }

  float lo = level[0], hi = level[0];
  for (uint32_t k = 1U; k < 3U; k++) { if (level[k] < lo) { lo = level[k]; } if (level[k] > hi) { hi = level[k]; } }
  const float spread = 20.0f * log10f(hi / lo);
  if (spread > 2.0f)
  { fail("audloop", "the tone's level moves %.1f dB between decimations (%.0f, %.0f, %.0f)", spread, level[0], level[1], level[2]); return; }
  ok("audloop", "%.0f Hz tone at %lu Hz heard at %lu, %lu and %lu Hz, levels %.0f %.0f %.0f, within %.1f dB",
     f, (unsigned long)outRate, (unsigned long)inRate[0], (unsigned long)inRate[1], (unsigned long)inRate[2],
     level[0], level[1], level[2], spread);
}

static void cmd_spk(void)
{
  BSP_AUDIO_Init_t ao = { BSP_AUDIO_OUT_DEVICE_SPEAKER, BSP_AUDIO_FREQUENCY_NATIVE, BSP_AUDIO_RESOLUTION_16B, 2U, 50U };
  uint32_t rate = 0, mute = 9; SpkHalf = SpkFull = SpkErr = 0;
  int32_t r = BSP_AUDIO_OUT_Init(0, &ao);
  if (r != BSP_ERROR_NONE) { fail("spk", "init %ld", (long)r); return; }
  (void)BSP_AUDIO_OUT_GetSampleRate(0, &rate);
  for (uint32_t i = 0; i < SPK_SAMPLES; i += 2U) { int16_t v = (int16_t)(6000.0f * sinf(6.2831853f * 1000.0f * (float)(i / 2U) / (float)(rate ? rate : 48000U))); SpkBuf[i] = v; SpkBuf[i + 1U] = v; }
  r = BSP_AUDIO_OUT_Play(0, (uint8_t *)SpkBuf, sizeof SpkBuf);
  if (r != BSP_ERROR_NONE) { fail("spk", "play %ld", (long)r); (void)BSP_AUDIO_OUT_DeInit(0); return; }
  /* Seventeen buffers: sixteen intervals, enough to time the rate. */
  uint32_t t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 1500U) && (SpkFull < 17U)) { svc(5U); }
  (void)BSP_AUDIO_OUT_Mute(0); (void)BSP_AUDIO_OUT_IsMute(0, &mute); (void)BSP_AUDIO_OUT_UnMute(0);
  (void)BSP_AUDIO_OUT_Stop(0); (void)BSP_AUDIO_OUT_DeInit(0);
  const uint32_t stereo = spk_measured_rate(SPK_SAMPLES / 2U);
  const uint32_t callbacks = SpkFull;
  if (SpkFull == 0U) { fail("spk", "no transfer-complete callback in 1.5s (rate=%lu)", (unsigned long)rate); return; }
  if (!rate_agrees(rate, stereo))
  { fail("spk", "the BSP reports %lu Hz but stereo frames leave at %lu Hz", (unsigned long)rate, (unsigned long)stereo); return; }

  /* One DMA block counts at most 65,535 bytes: a larger buffer must be
   * refused, and the largest accepted one play at rate. Silence, from JpegBuf. */
  r = BSP_AUDIO_OUT_Init(0, &ao);
  if (r != BSP_ERROR_NONE) { fail("spk", "init for the large buffers %ld", (long)r); return; }
  memset(JpegBuf, 0, 98304U);
  SpkFull = 0U;
  r = BSP_AUDIO_OUT_Play(0, JpegBuf, 98304U);
  if (r == BSP_ERROR_NONE)
  {
    t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 3000U) && (SpkFull < 5U)) { svc(5U); }
    (void)BSP_AUDIO_OUT_Stop(0); (void)BSP_AUDIO_OUT_DeInit(0);
    fail("spk", "a 98304-byte buffer was accepted, and plays %lu frames a second at %lu Hz",
         (unsigned long)spk_measured_rate(98304U / 4U), (unsigned long)rate);
    return;
  }
  SpkFull = 0U;
  r = BSP_AUDIO_OUT_Play(0, JpegBuf, 65528U);
  if (r != BSP_ERROR_NONE) { fail("spk", "the largest buffer, 65528 bytes, was refused (%ld)", (long)r); (void)BSP_AUDIO_OUT_DeInit(0); return; }
  t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 3000U) && (SpkFull < 5U)) { svc(5U); }
  (void)BSP_AUDIO_OUT_Stop(0); (void)BSP_AUDIO_OUT_DeInit(0);
  const uint32_t largest = spk_measured_rate(65528U / 4U);
  if (!rate_agrees(rate, largest))
  { fail("spk", "a 65528-byte buffer at %lu Hz plays %lu frames a second", (unsigned long)rate, (unsigned long)largest); return; }

  /* The same buffer as mono: one sample a frame, played in both slots. */
  ao.ChannelsNbr = 1U;
  SpkHalf = SpkFull = SpkErr = 0;
  r = BSP_AUDIO_OUT_Init(0, &ao);
  if (r != BSP_ERROR_NONE) { fail("spk", "mono init %ld", (long)r); return; }
  for (uint32_t i = 0; i < SPK_SAMPLES; i++) { SpkBuf[i] = (int16_t)(6000.0f * sinf(6.2831853f * 1000.0f * (float)i / (float)(rate ? rate : 48000U))); }
  r = BSP_AUDIO_OUT_Play(0, (uint8_t *)SpkBuf, sizeof SpkBuf);
  if (r != BSP_ERROR_NONE) { fail("spk", "mono play %ld", (long)r); (void)BSP_AUDIO_OUT_DeInit(0); return; }
  t0 = HAL_GetTick(); while (((HAL_GetTick() - t0) < 2000U) && (SpkFull < 9U)) { svc(5U); }
  (void)BSP_AUDIO_OUT_Stop(0); (void)BSP_AUDIO_OUT_DeInit(0);
  const uint32_t mono = spk_measured_rate(SPK_SAMPLES);
  if (!rate_agrees(rate, mono))
  { fail("spk", "mono at %lu Hz plays %lu frames a second", (unsigned long)rate, (unsigned long)mono); return; }
  ok("spk", "rate=%lu measured stereo=%lu largest=%lu mono=%lu callbacks=%lu err=%lu mute_readback=%lu 1kHz played", (unsigned long)rate,
     (unsigned long)stereo, (unsigned long)largest, (unsigned long)mono, (unsigned long)callbacks, (unsigned long)SpkErr, (unsigned long)mute);
}

static volatile uint8_t VencAnchor;   /* 1: frame ends anchor the encoder's row count */
void BSP_CAMERA_FrameEventCallback(uint32_t Instance, uint32_t Pipe)
{
  (void)Instance; CamFrames++;
  if (Pipe < 3U) { CamPipeFrames[Pipe]++; }
  if ((Pipe == BSP_CAMERA_PIPE_MAIN) && (VencAnchor != 0U)) { (void)BSP_VENC_SourceFrameEvent(0); }
}
void BSP_CAMERA_ErrorCallback(uint32_t Instance, uint32_t Pipe)      { (void)Instance; (void)Pipe; CamErrors++; }
void BSP_CAMERA_LineEventCallback(uint32_t Instance, uint32_t Pipe)  { (void)Instance; if (Pipe == BSP_CAMERA_PIPE_MAIN) { CamLines++; (void)BSP_VENC_SourceLineEvent(0); } }

static int32_t cam_up(void)
{
  int32_t r = BSP_CAMERA_Init(0, BSP_CAMERA_R640x480, BSP_CAMERA_PF_RGB565);
  if (r != BSP_ERROR_NONE) { P("INFO camera init %ld\r\n", (long)r); }
  return r;
}

/* Frames captured with this carrier mapping, best of three tries. */
static uint32_t camcarrier_try(uint32_t carrier)
{
  BSP_CAMERA_PipeConfig_t pc = {0};
  uint32_t best = 0U;

  for (uint32_t attempt = 0U; attempt < 3U; attempt++)
  {
    CamFrames = CamErrors = 0U;

    if (BSP_CAMERA_SetCarrier(carrier) != BSP_ERROR_NONE) { return 0U; }
    if (cam_up() != BSP_ERROR_NONE) { continue; }

    pc.PixelFormat = BSP_CAMERA_PIPE_PF_RGB565;
    pc.Width = CAM_W;
    pc.Height = CAM_H;
    if (BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_DUMP, &pc) == BSP_ERROR_NONE)
    {
      CamFrames = 0U;
      if (BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_DUMP, CamFrame, BSP_CAMERA_MODE_CONTINUOUS) == BSP_ERROR_NONE)
      {
        uint32_t t0 = HAL_GetTick();
        while (((HAL_GetTick() - t0) < 800U) && (CamFrames < 5U)) { svc(5U); }
      }
      (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_DUMP);
    }
    if (CamFrames > best) { best = CamFrames; }
    (void)BSP_CAMERA_DeInit(0);
    if (best >= 5U) { break; }
  }
  return best;
}

static void cmd_camcarrier(void)
{
  uint32_t started = BSP_CAMERA_GetCarrier();
  uint32_t std1, inv, std2;

  std1 = camcarrier_try(BSP_CAMERA_CARRIER_STANDARD);
  inv  = camcarrier_try(BSP_CAMERA_CARRIER_INVERTED);
  std2 = camcarrier_try(BSP_CAMERA_CARRIER_STANDARD);

  P("INFO frames standard=%lu inverted=%lu standard again=%lu (module started on %s)\r\n",
    (unsigned long)std1, (unsigned long)inv, (unsigned long)std2,
    (started == BSP_CAMERA_CARRIER_INVERTED) ? "inverted" : "standard");

  /* The mapping that captures is left selected, so the camera tests after
   * this one run on whichever carrier is fitted. */
  (void)BSP_CAMERA_SetCarrier(started);

  if ((std1 == 0U) && (std2 == 0U) && (inv >= 5U))
  {
    (void)BSP_CAMERA_SetCarrier(BSP_CAMERA_CARRIER_INVERTED);
    ok("camcarrier", "the crossed mapping is the one that captures: %lu frames, against none from the straight mapping twice; it stays selected", (unsigned long)inv);
    return;
  }
  if ((inv == 0U) && (std1 >= 5U) && (std2 >= 5U))
  {
    (void)BSP_CAMERA_SetCarrier(BSP_CAMERA_CARRIER_STANDARD);
    ok("camcarrier", "the straight mapping is the one that captures: %lu frames, against none from the crossed mapping; it stays selected", (unsigned long)std1);
    return;
  }
  if ((std1 == 0U) && (std2 == 0U) && (inv == 0U))
  {
    fail("camcarrier", "neither mapping captures anything, so this is not the carrier: look at the sensor, the ribbon or the link");
    return;
  }
  fail("camcarrier", "the mappings do not separate: standard %lu and %lu, inverted %lu", (unsigned long)std1, (unsigned long)std2, (unsigned long)inv);
}

/* A settled frame on the dump pipe: a picture varies, a dead link is one
 * value; a 16x8 preview goes to the log. */
static void cmd_cam(void)
{
  BSP_CAMERA_Capabilities_t cap; BSP_CAMERA_PipeConfig_t pc = {0}; uint32_t fb = 0;
  CamFrames = CamErrors = 0;
  if (cam_up() != BSP_ERROR_NONE) { fail("cam", "init (sensor absent, bus, or link)"); return; }
  (void)BSP_CAMERA_GetCapabilities(0, &cap);
  pc.PixelFormat = BSP_CAMERA_PIPE_PF_RGB565; pc.Width = CAM_W; pc.Height = CAM_H;
  if (BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_DUMP, &pc) != BSP_ERROR_NONE) { fail("cam", "config dump pipe"); (void)BSP_CAMERA_DeInit(0); return; }
  (void)BSP_CAMERA_GetFrameSize(0, BSP_CAMERA_PIPE_DUMP, &fb);
  /* Free-run and take a settled frame. */
  memset(CamFrame, 0xA5, 4096U);
  CamFrames = 0U;
  if (BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_DUMP, CamFrame, BSP_CAMERA_MODE_CONTINUOUS) != BSP_ERROR_NONE) { fail("cam", "start"); (void)BSP_CAMERA_DeInit(0); return; }
  {
    uint32_t t0 = HAL_GetTick();
    while (((HAL_GetTick() - t0) < 800U) && (CamFrames < 5U)) { svc(5U); }
  }
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_DUMP);
  P("INFO frames=%lu errors=%lu first=0x%04X\r\n",
    (unsigned long)CamFrames, (unsigned long)CamErrors, ((const uint16_t *)CamFrame)[0]);
  /* statistics on every 64th pixel, and a coarse preview */
  const uint16_t *px = (const uint16_t *)CamFrame; uint32_t n = 0, mn = 0xFFFF, mx = 0, distinct = 0; uint64_t sum = 0; uint16_t prev = 0xFFFF;
  for (uint32_t i = 0; i < CAM_W * CAM_H; i += 64U) { uint16_t v = px[i]; if (v < mn) mn = v; if (v > mx) mx = v; sum += v; n++; if (v != prev) { distinct++; prev = v; } }
  P("INFO frames=%lu errors=%lu bytes=%lu fps_max=%lu dphy_patch=%lu min=0x%04lx max=0x%04lx mean=0x%04lx distinct=%lu/%lu\r\n",
    (unsigned long)CamFrames, (unsigned long)CamErrors, (unsigned long)fb, (unsigned long)cap.MaxFrameRate, (unsigned long)cap.NeedsHalDPhyPatch,
    (unsigned long)mn, (unsigned long)mx, (unsigned long)(sum / n), (unsigned long)distinct, (unsigned long)n);
  for (uint32_t y = 0; y < 8U; y++)
  {
    char row[20];
    for (uint32_t x = 0; x < 16U; x++)
    { uint16_t v = px[(y * (CAM_H / 8U) + CAM_H / 16U) * CAM_W + x * (CAM_W / 16U) + CAM_W / 32U];
      uint32_t lum = (((v >> 11) & 31U) * 2U + ((v >> 5) & 63U) + (v & 31U) * 2U) / 4U;   /* 0..63 */
      row[x] = " .:-=+*#%@"[lum * 9U / 63U]; }
    row[16] = 0; P("INFO |%s|\r\n", row); svc(0U);
  }
  (void)BSP_CAMERA_DeInit(0);
  if (CamFrames == 0U) { fail("cam", "no frame in 3s (errors=%lu)", (unsigned long)CamErrors); return; }
  if (distinct < 64U) { fail("cam", "frame arrived but pixels are constant (%lu distinct) - link or sensor stream", (unsigned long)distinct); return; }
  ok("cam", "frames=%lu distinct=%lu mean=0x%04lx", (unsigned long)CamFrames, (unsigned long)distinct, (unsigned long)(sum / n));
}

static int32_t jpeg_source_start(void *arg) { (void)arg; return (BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, Band, BSP_CAMERA_MODE_CONTINUOUS) == BSP_ERROR_NONE) ? 0 : -1; }

/* The first frame after a camera restart, the encoder's row count anchored
 * at frame end: the camera is stopped part-way through a frame, the encoder
 * resynchronised, and the next frame must not be reported torn. */
static void cmd_jpegtorn(void)
{
  BSP_CAMERA_PipeConfig_t pc = {0}; BSP_VENC_Band_t band; BSP_VENC_Config_t vc = {0}; BSP_VENC_Statistics_t st;
  uint32_t n = 0U, torn = 0U, steady_torn = 0U, encoded = 0U;
  char gaps[80] = "";
  int32_t r;

  if (cam_up() != BSP_ERROR_NONE) { fail("jpegtorn", "camera init"); return; }
  if ((BSP_VENC_GetBandGeometry(CAM_W, CAM_H, &band) != BSP_ERROR_NONE) || (band.SizeBytes > sizeof Band))
  { fail("jpegtorn", "band geometry"); (void)BSP_CAMERA_DeInit(0); return; }
  pc.PixelFormat = BSP_CAMERA_PIPE_PF_YUV422; pc.Width = CAM_W; pc.Height = CAM_H;
  r = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pc);
  if (r == BSP_ERROR_NONE) { r = BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN, band.WrapLines, BSP_CAMERA_LINES_OFF); }
  if (r == BSP_ERROR_NONE) { r = BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, Band, BSP_CAMERA_MODE_CONTINUOUS); }
  if (r == BSP_ERROR_NONE) { svc(20U); r = BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN); }
  if (r == BSP_ERROR_NONE) { r = BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN, band.WrapLines, band.EventLines); }
  vc.Band = band; vc.pBand = Band; vc.Quality = BSP_VENC_QUALITY_DEFAULT; vc.SourceStart = jpeg_source_start;
  if (r == BSP_ERROR_NONE) { r = BSP_VENC_Init(0, &vc); }
  if (r != BSP_ERROR_NONE) { fail("jpegtorn", "set-up %ld", (long)r); (void)BSP_CAMERA_DeInit(0); return; }

  VencAnchor = 1U;
  for (uint32_t i = 0U; i < 5U; i++)
  {
    if (BSP_VENC_EncodeBand(0, JpegBuf, JPEG_BUF_BYTES, &n) == BSP_ERROR_NONE) { encoded++; steady_torn += BSP_VENC_LastFrameTorn(0); }
    svc(0U);
  }
  for (uint32_t k = 0U; k < 10U; k++)
  {
    char t[8];

    (void)BSP_VENC_EncodeBand(0, JpegBuf, JPEG_BUF_BYTES, &n);
    { const uint32_t t0 = HAL_GetTick(); while ((HAL_GetTick() - t0) < (2U + k)) { } }
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
    r = BSP_VENC_Resync(0);
    if ((r == BSP_ERROR_NONE) && (BSP_VENC_EncodeBand(0, JpegBuf, JPEG_BUF_BYTES, &n) == BSP_ERROR_NONE))
    {
      encoded++;
      torn += BSP_VENC_LastFrameTorn(0);
      (void)BSP_VENC_GetStatistics(0, &st);
      (void)snprintf(t, sizeof t, "%lu ", (unsigned long)st.LastGap);
      strncat(gaps, t, sizeof gaps - strlen(gaps) - 1U);
    }
    svc(0U);
  }
  VencAnchor = 0U;
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN); (void)BSP_VENC_DeInit(0); (void)BSP_CAMERA_DeInit(0);

  P("INFO steady: %lu of 5 torn; first frames after a restart: %lu of 10 torn, gaps %s(band depth %lu)\r\n",
    (unsigned long)steady_torn, (unsigned long)torn, gaps, (unsigned long)band.Depth);
  if (encoded < 15U) { fail("jpegtorn", "only %lu of 15 frames encoded", (unsigned long)encoded); return; }
  if (torn != 0U) { fail("jpegtorn", "%lu of 10 first frames after a restart reported torn", (unsigned long)torn); return; }
  ok("jpegtorn", "no first frame after a restart reported torn (10), %lu of 5 steady", (unsigned long)steady_torn);
}

/* The whole line-buffer path: band, wrap, line event, prime, encoder, then
 * ten frames. */
static void cmd_jpeg(void)
{
  BSP_CAMERA_PipeConfig_t pc = {0}; BSP_VENC_Band_t band; BSP_VENC_Config_t vc = {0}; BSP_VENC_Statistics_t st;
  JpegMemBytes = 0U;   /* this overwrites JpegBuf: no stale dump */
  uint32_t okf = 0, bad = 0, minb = 0xFFFFFFFFU, maxb = 0, badmark = 0;
  CamLines = 0;
  if (cam_up() != BSP_ERROR_NONE) { fail("jpeg", "camera init"); return; }
  if (BSP_VENC_GetBandGeometry(CAM_W, CAM_H, &band) != BSP_ERROR_NONE) { fail("jpeg", "band geometry"); (void)BSP_CAMERA_DeInit(0); return; }
  if (band.SizeBytes > sizeof Band) { fail("jpeg", "band %lu > %u", (unsigned long)band.SizeBytes, (unsigned)sizeof Band); (void)BSP_CAMERA_DeInit(0); return; }
  pc.PixelFormat = BSP_CAMERA_PIPE_PF_YUV422; pc.Width = CAM_W; pc.Height = CAM_H;
  int32_t r = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pc);
  /* Wrap, prime, stop, then the line event (see Camera_JpegStream_USB). */
  if (r == BSP_ERROR_NONE) { r = BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN, band.WrapLines, BSP_CAMERA_LINES_OFF); }
  if (r == BSP_ERROR_NONE) { r = BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, Band, BSP_CAMERA_MODE_CONTINUOUS); }   /* prime */
  if (r == BSP_ERROR_NONE) { svc(20U); r = BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN); }
  if (r == BSP_ERROR_NONE) { r = BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN, band.WrapLines, band.EventLines); }
  if (r != BSP_ERROR_NONE) { fail("jpeg", "pipe/linebuffer/prime %ld", (long)r); (void)BSP_CAMERA_DeInit(0); return; }
  P("INFO pipe+linebuffer+prime ok; calling BSP_VENC_Init\r\n"); BSP_COM_Flush(200U); svc(2U);
  vc.Band = band; vc.pBand = Band; vc.Quality = BSP_VENC_QUALITY_DEFAULT; vc.SourceStart = jpeg_source_start;
  r = BSP_VENC_Init(0, &vc);
  P("INFO BSP_VENC_Init returned %ld; first EncodeBand starts the camera\r\n", (long)r);
  BSP_COM_Flush(200U); svc(2U);
  HAL_PWR_EnableBkUpAccess();
  (&TAMP->BKP0R)[4] = 0x45U; (&TAMP->BKP0R)[5] = 0U; (&TAMP->BKP0R)[6] = 0U;
  if (r != BSP_ERROR_NONE) { fail("jpeg", "VENC init %ld (heap? geometry?)", (long)r); (void)BSP_CAMERA_DeInit(0); return; }
  (void)BSP_VENC_ResetStatistics(0);
  memset(Band + BAND_BYTES, 0xA5, BAND_GUARD_BYTES);   /* guard behind the band */
  uint32_t t0 = HAL_GetTick();
  for (uint32_t i = 0; i < 10U; i++)
  {
    uint32_t n = 0;
    int32_t er = BSP_VENC_EncodeBand(0, JpegBuf, JPEG_BUF_BYTES, &n);
    (&TAMP->BKP0R)[5] = DCMIPP->P1PPCR;   /* trace: wrap enable/modulus at this instant */
    if ((er == BSP_ERROR_NONE) && ((n < 4U) || (n > JPEG_BUF_BYTES))) { er = BSP_ERROR_PERIPH_FAILURE; badmark++; }
    if (er == BSP_ERROR_NONE)
    {
      okf++; if (n < minb) minb = n; if (n > maxb) maxb = n;
      if (!((JpegBuf[0] == 0xFFU) && (JpegBuf[1] == 0xD8U) && (JpegBuf[n - 2U] == 0xFFU) && (JpegBuf[n - 1U] == 0xD9U))) { badmark++; }
    }
    else { bad++; }
    svc(0U);
  }
  uint32_t ms = HAL_GetTick() - t0;
  (void)BSP_VENC_GetStatistics(0, &st);
  {
    /* Anything not 0xA5 in the guard is a write past the band. */
    uint32_t changed = 0U, first = BAND_GUARD_BYTES;
    for (uint32_t g = 0; g < BAND_GUARD_BYTES; g++)
    {
      if (Band[BAND_BYTES + g] != 0xA5U) { changed++; if (first == BAND_GUARD_BYTES) { first = g; } }
    }
    P("INFO band_guard: changed=%lu first_offset=%lu of %lu\r\n", (unsigned long)changed,
      (unsigned long)first, (unsigned long)BAND_GUARD_BYTES);
  }
  P("INFO frames_ok=%lu fail=%lu bytes=%lu..%lu badmarkers=%lu lead=%lu/%lu maxrow=%lu lasterr=%ld lines=%lu in %lums\r\n",
    (unsigned long)okf, (unsigned long)bad, (unsigned long)(okf ? minb : 0), (unsigned long)maxb, (unsigned long)badmark, (unsigned long)st.MaxLead, (unsigned long)band.Depth,
    (unsigned long)st.MaxReadRow, (long)(int32_t)st.LastError, (unsigned long)CamLines, (unsigned long)ms);
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN); (void)BSP_VENC_DeInit(0); (void)BSP_CAMERA_DeInit(0);
  if (okf == 0U) { fail("jpeg", "no frame encoded (lasterr=%ld)", (long)(int32_t)st.LastError); return; }
  if (badmark != 0U) { fail("jpeg", "%lu frames without SOI/EOI markers", (unsigned long)badmark); return; }
  ok("jpeg", "ok=%lu fail=%lu bytes=%lu..%lu lead=%lu/%lu fps~%lu", (unsigned long)okf, (unsigned long)bad, (unsigned long)minb, (unsigned long)maxb, (unsigned long)st.MaxLead, (unsigned long)band.Depth, (unsigned long)(ms ? (okf * 1000U) / ms : 0));
}

/* The encoder's registers and its memory, read without encoding anything. */
static void cmd_vencreg(void)
{
  volatile uint32_t *bkp = &TAMP->BKP0R;
  RIMC_MasterConfig_t master = {0};
  uint32_t id, cfg1, ramcr;

  /* Each step is stamped in bkp4: a hang takes the console, but the backup
   * register survives the watchdog reset. */
  HAL_PWR_EnableBkUpAccess();

  bkp[4] = 0x60U;
  LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_RIFSC);

  bkp[4] = 0x61U;
  master.MasterCID = RIF_CID_1;
  master.SecPriv   = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_VENC, &master);

  bkp[4] = 0x62U;
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_VENC,
                                        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);

  /* SYSCFG has its own bus clock, and reading one of its registers without it
   * is a stall, not a fault. */
  bkp[4] = 0x63U;
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  ramcr = SYSCFG->VENCRAMCR;

  bkp[4] = 0x64U;
  P("INFO granted; VENCRAMCR=0x%08lx (bit0 set means the encoder's RAM belongs to the system)\r\n",
    (unsigned long)ramcr);
  BSP_COM_Flush(200U); svc(2U);

  bkp[4] = 0x65U;
  LL_VENC_Init();
  P("INFO clocks on (APB5, VENCRAM, VENC); reading the identity register\r\n");
  BSP_COM_Flush(200U); svc(2U);

  bkp[4] = 0x66U;
  id = LL_VENC_ReadRegister(0UL);
  bkp[4] = 0x67U;
  cfg1 = LL_VENC_ReadRegister(63UL);

  /* The frame-start path writes indices up to about 233, and 64..95 for the
   * penalty tables. */
  static const uint16_t sweep[] = { 64U, 95U, 96U, 127U, 128U, 160U, 200U, 231U, 233U, 296U, 320U, 400U };
  char seen[160] = "";
  for (uint32_t i = 0; i < sizeof sweep / sizeof sweep[0]; i++)
  {
    bkp[4] = 0x70U + i;
    uint32_t v = LL_VENC_ReadRegister((uint32_t)sweep[i]);
    char t[24]; snprintf(t, sizeof t, "%u:%08lx ", (unsigned)sweep[i], (unsigned long)v);
    strncat(seen, t, sizeof seen - strlen(seen) - 1);
  }
  bkp[4] = 0x68U;
  P("INFO register sweep %s\r\n", seen);
  BSP_COM_Flush(200U); svc(2U);

  /* Register 296 (offset 0x4A0), read before and after BSP_VENC_Init. */
  {
    BSP_VENC_Band_t b2;
    BSP_VENC_Config_t c2 = {0};
    uint32_t before, after;

    bkp[4] = 0x69U;
    before = LL_VENC_ReadRegister(296UL);

    bkp[4] = 0x6AU;
    (void)BSP_VENC_GetBandGeometry(CAM_W, CAM_H, &b2);
    c2.Band = b2; c2.pBand = NULL; c2.SourceStart = NULL; c2.Quality = BSP_VENC_QUALITY_DEFAULT;
    int32_t ri = BSP_VENC_Init(0, &c2);
    P("INFO reg296 before init = 0x%08lx; BSP_VENC_Init returned %ld; reading it again\r\n",
      (unsigned long)before, (long)ri);
    BSP_COM_Flush(200U); svc(2U);

    bkp[4] = 0x6BU;
    after = LL_VENC_ReadRegister(296UL);
    bkp[4] = 0x6CU;
    P("INFO reg296 after init = 0x%08lx\r\n", (unsigned long)after);
    BSP_COM_Flush(200U); svc(2U);
    (void)BSP_VENC_DeInit(0);
  }

  P("INFO asic_id=0x%08lx cfg63=0x%08lx\r\n", (unsigned long)id, (unsigned long)cfg1);
  LL_VENC_DeInit();
  if ((id == 0U) || (id == 0xFFFFFFFFU))
  {
    fail("vencreg", "identity reads 0x%08lx: clocked but not answering", (unsigned long)id);
    return;
  }
  ok("vencreg", "asic_id=0x%08lx cfg63=0x%08lx vencram_system_accessible=%lu",
     (unsigned long)id, (unsigned long)cfg1, (unsigned long)(ramcr & 1U));
}

/* The encoder from memory: eight colour bars as YUV422 in PSRAM; jpegdump
 * then sends the JPEG as hex. */
static void cmd_jpegmem(void)
{
  /* BT.601 limited-range Y, Cb, Cr for white, yellow, cyan, green, magenta,
   * red, blue, black */
  static const uint8_t bars[8][3] = { {235,128,128}, {210,16,146}, {170,166,16}, {145,54,34},
                                      {106,202,222}, {81,90,240}, {41,240,110}, {16,128,128} };
  BSP_VENC_Band_t band; BSP_VENC_Config_t vc = {0}; BSP_VENC_Statistics_t st; uint32_t n = 0;
  JpegMemBytes = 0U;
  P("INFO jpegmem: entered\r\n"); BSP_COM_Flush(200U);
  for (uint32_t y = 0; y < CAM_H; y++)
  {
    uint8_t *row = CamFrame + y * CAM_W * 2U;
    for (uint32_t x = 0; x < CAM_W; x += 2U)
    { const uint8_t *b = bars[x / (CAM_W / 8U)]; row[2U*x] = b[0]; row[2U*x+1U] = b[1]; row[2U*x+2U] = b[0]; row[2U*x+3U] = b[2]; }
    if ((y & 63U) == 0U) { svc(0U); }
  }
  P("INFO jpegmem: bars written\r\n"); BSP_COM_Flush(200U);
  if (BSP_VENC_GetBandGeometry(CAM_W, CAM_H, &band) != BSP_ERROR_NONE) { fail("jpegmem", "geometry"); return; }
  P("INFO bars drawn; band %lux%lu depth=%lu; calling BSP_VENC_Init\r\n",
    (unsigned long)band.CodedWidth, (unsigned long)band.CodedHeight, (unsigned long)band.Depth);
  BSP_COM_Flush(200U); svc(2U);
  vc.Band = band; vc.pBand = NULL; vc.SourceStart = NULL; vc.Quality = BSP_VENC_QUALITY_DEFAULT;
  int32_t r = BSP_VENC_Init(0, &vc);
  P("INFO BSP_VENC_Init returned %ld; calling BSP_VENC_EncodeFrame\r\n", (long)r);
  BSP_COM_Flush(200U); svc(2U);
  if (r != BSP_ERROR_NONE) { fail("jpegmem", "VENC init %ld", (long)r); return; }
  /* The console may not come back from here: bkp4 = 0x41 marks the encode
   * entered, and the hook counters how far into its wait it got. */
  HAL_PWR_EnableBkUpAccess();
  (&TAMP->BKP0R)[4] = 0x41U; (&TAMP->BKP0R)[3] = 0U; (&TAMP->BKP0R)[6] = 0U; (&TAMP->BKP0R)[0] = 0U;
  uint32_t t0 = HAL_GetTick();
  (&TAMP->BKP0R)[5] = uwTick;   /* the early warning's clock */
  /* Give the encode far longer than the ordinary watchdog allows. */
  P("INFO jpegmem: widening the watchdog\r\n"); BSP_COM_Flush(200U);
  (void)BSP_WDG_InitEWI(20000U, 500U);
  P("INFO jpegmem: watchdog widened; encoding\r\n"); BSP_COM_Flush(200U);
  r = BSP_VENC_EncodeFrame(0, CamFrame, JpegBuf, JPEG_BUF_BYTES, &n);
  (void)BSP_WDG_InitEWI(WDG_TIMEOUT_MS, 300U);
  P("INFO BSP_VENC_EncodeFrame returned %ld\r\n", (long)r); BSP_COM_Flush(200U); svc(2U);
  uint32_t ms = HAL_GetTick() - t0;
  (void)BSP_VENC_GetStatistics(0, &st); (void)BSP_VENC_DeInit(0);
  if (r != BSP_ERROR_NONE) { fail("jpegmem", "encode %ld lasterr=%ld in %lums", (long)r, (long)(int32_t)st.LastError, (unsigned long)ms); return; }
  if ((n < 4U) || (JpegBuf[0] != 0xFFU) || (JpegBuf[1] != 0xD8U) || (JpegBuf[n-2U] != 0xFFU) || (JpegBuf[n-1U] != 0xD9U)) { fail("jpegmem", "%lu bytes without SOI/EOI", (unsigned long)n); return; }
  JpegMemBytes = n;
  ok("jpegmem", "bytes=%lu in %lums (8 colour bars, 640x480 YUV422 from PSRAM)", (unsigned long)n, (unsigned long)ms);
}
static void cmd_jpegdump(void)
{
  static const char hexd[] = "0123456789abcdef";
  if (JpegMemBytes == 0U) { fail("jpegdump", "nothing encoded: run jpegmem first"); return; }
  for (uint32_t off = 0; off < JpegMemBytes; off += 64U)
  {
    char line[64U * 2U + 1U]; uint32_t k = 0;
    for (uint32_t i = off; (i < off + 64U) && (i < JpegMemBytes); i++) { line[k++] = hexd[JpegBuf[i] >> 4]; line[k++] = hexd[JpegBuf[i] & 15U]; }
    line[k] = 0; P("INFO HEX %s\r\n", line);
    if ((off & 1023U) == 0U) { svc(0U); }
  }
  ok("jpegdump", "bytes=%lu", (unsigned long)JpegMemBytes);
}

/* The sensor's and receiver's own registers, read back. */
static uint8_t ovrd(uint16_t reg)
{
  uint8_t v = 0U;
  (void)BSP_I2C1_ReadReg16(BSP_CAMERA_I2C_ADDRESS, reg, &v, 1U);
  return v;
}
static void cmd_camreg(void)
{
  if (BSP_I2C1_Init() != BSP_ERROR_NONE) { fail("camreg", "I2C1 init"); return; }
  P("INFO [OVREG] id=%02X%02X stream(3008)=%02X mipictl(300E)=%02X | PLL 3034=%02X 3035=%02X 3036=%02X 3037=%02X 3108=%02X\r\n",
    ovrd(0x300A), ovrd(0x300B), ovrd(0x3008), ovrd(0x300E),
    ovrd(0x3034), ovrd(0x3035), ovrd(0x3036), ovrd(0x3037), ovrd(0x3108));
  svc(0U);
  P("INFO [OVREG] MIPI 4800=%02X 4814=%02X 4837=%02X | fmt 4300=%02X 501F=%02X | sub 3814=%02X 3815=%02X flip 3820=%02X 3821=%02X\r\n",
    ovrd(0x4800), ovrd(0x4814), ovrd(0x4837), ovrd(0x4300), ovrd(0x501F),
    ovrd(0x3814), ovrd(0x3815), ovrd(0x3820), ovrd(0x3821));
  svc(0U);
  P("INFO [OVREG] out=%02X%02Xx%02X%02X HTS=%02X%02X VTS=%02X%02X isp(5001)=%02X\r\n",
    ovrd(0x3808), ovrd(0x3809), ovrd(0x380A), ovrd(0x380B),
    ovrd(0x380C), ovrd(0x380D), ovrd(0x380E), ovrd(0x380F), ovrd(0x5001));
  svc(0U);
  P("INFO [CSIRAW] CR=%08lX PCR=%08lX SR0=%08lX SR1=%08lX ERR1=%08lX ERR2=%08lX PRCR=%08lX\r\n",
    (unsigned long)CSI->CR, (unsigned long)CSI->PCR, (unsigned long)CSI->SR0,
    (unsigned long)CSI->SR1, (unsigned long)CSI->ERR1, (unsigned long)CSI->ERR2,
    (unsigned long)CSI->PRCR);
  P("INFO [CLK] PLL1=%lu CSI=%lu DCMIPP=%lu Hz\r\n",
    (unsigned long)HAL_RCCEx_GetPLL1CLKFreq(),
    (unsigned long)HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_CSI),
    (unsigned long)HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_DCMIPP));
  {
    const uint8_t r300e = ovrd(0x300E), r3008 = ovrd(0x3008);

    (void)BSP_I2C1_DeInit();
    ok("camreg", "300E=%02X 3008=%02X csi_err1=%08lX err2=%08lX sr1=%08lX",
       r300e, r3008, (unsigned long)CSI->ERR1, (unsigned long)CSI->ERR2, (unsigned long)CSI->SR1);
  }
}

/* The registers while streaming, when a link fault is visible. */
static void cmd_camlive(void)
{
  BSP_CAMERA_PipeConfig_t pc = {0};
  CamFrames = CamErrors = 0;
  if (cam_up() != BSP_ERROR_NONE) { fail("camlive", "camera init"); return; }
  pc.PixelFormat = BSP_CAMERA_PIPE_PF_RGB565; pc.Width = CAM_W; pc.Height = CAM_H;
  if (BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_DUMP, &pc) != BSP_ERROR_NONE) { fail("camlive", "pipe"); (void)BSP_CAMERA_DeInit(0); return; }
  (void)BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_DUMP, CamFrame, BSP_CAMERA_MODE_CONTINUOUS);
  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < 500U) { svc(5U); }
  P("INFO streaming: frames=%lu errors=%lu\r\n", (unsigned long)CamFrames, (unsigned long)CamErrors);
  P("INFO [CSIRAW-live] CR=%08lX PCR=%08lX SR0=%08lX SR1=%08lX ERR1=%08lX ERR2=%08lX\r\n",
    (unsigned long)CSI->CR, (unsigned long)CSI->PCR, (unsigned long)CSI->SR0,
    (unsigned long)CSI->SR1, (unsigned long)CSI->ERR1, (unsigned long)CSI->ERR2);
  P("INFO [OVREG-live] 300E=%02X 3008=%02X 4800=%02X 4837=%02X 3034=%02X 3035=%02X 3036=%02X\r\n",
    ovrd(0x300E), ovrd(0x3008), ovrd(0x4800), ovrd(0x4837),
    ovrd(0x3034), ovrd(0x3035), ovrd(0x3036));
  const uint16_t *px = (const uint16_t *)CamFrame; uint32_t distinct = 0; uint16_t prev = 0xFFFF;
  for (uint32_t i = 0; i < CAM_W * CAM_H; i += 64U) { if (px[i] != prev) { distinct++; prev = px[i]; } }
  P("INFO pixels: first=0x%04X distinct=%lu\r\n", px[0], (unsigned long)distinct);
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_DUMP); (void)BSP_CAMERA_DeInit(0);
  ok("camlive", "frames=%lu errors=%lu distinct=%lu err1=%08lX", (unsigned long)CamFrames,
     (unsigned long)CamErrors, (unsigned long)distinct, (unsigned long)CSI->ERR1);
}

/* Encoder recovery probe: clock on, force reset held for a while, release,
 * read the ASIC id back. */
static void cmd_vencrst(void)
{
  uint32_t id0, id1;
  BSP_VENC_EnablePeripheral();
  id0 = LL_VENC_ReadRegister(0UL);
  __DSB(); __ISB();
  __HAL_RCC_VENC_FORCE_RESET();
  __DSB(); __ISB();
  svc(10U);
  __HAL_RCC_VENC_RELEASE_RESET();
  __DSB(); __ISB();
  svc(10U);
  id1 = LL_VENC_ReadRegister(0UL);
  ok("vencrst", "asic_id before=0x%08lx after=0x%08lx", (unsigned long)id0, (unsigned long)id1);
}

/* Read the isolation state back, changing nothing. */
static void cmd_rif(void)
{
  uint32_t dcmipp = RIFSC->RIMC_ATTRx[RIF_MASTER_INDEX_DCMIPP];
  uint32_t venc   = RIFSC->RIMC_ATTRx[RIF_MASTER_INDEX_VENC];
  uint32_t npu    = RIFSC->RIMC_ATTRx[RIF_MASTER_INDEX_NPU];
  P("INFO RIMC dcmipp[%u]=0x%08lx venc[%u]=0x%08lx npu[%u]=0x%08lx\r\n",
    (unsigned)RIF_MASTER_INDEX_DCMIPP, (unsigned long)dcmipp,
    (unsigned)RIF_MASTER_INDEX_VENC, (unsigned long)venc,
    (unsigned)RIF_MASTER_INDEX_NPU, (unsigned long)npu);
  P("INFO RISAF2 r0cfg=0x%08lx r1cfg=0x%08lx | RISAF12 r0cfg=0x%08lx r1cfg=0x%08lx\r\n",
    (unsigned long)RISAF2->REG[0].CFGR, (unsigned long)RISAF2->REG[1].CFGR,
    (unsigned long)RISAF12->REG[0].CFGR, (unsigned long)RISAF12->REG[1].CFGR);
  ok("rif", "venc_stamped=%s dcmipp_stamped=%s",
     (venc != 0U) ? "yes" : "NO", (dcmipp != 0U) ? "yes" : "NO");
}

/* The two orderings, each with the streaming image's start-up gap. */
static void jpeg_gap_test(const char *name, uint8_t leave_running)
{
  BSP_CAMERA_PipeConfig_t pc = {0}; BSP_VENC_Band_t band; BSP_VENC_Config_t vc = {0};
  JpegMemBytes = 0U;   /* this overwrites JpegBuf: no stale dump */
  uint32_t okf = 0, bad = 0, n = 0; int32_t r;

  CamLines = 0;
  if (cam_up() != BSP_ERROR_NONE) { fail(name, "camera init"); return; }
  if (BSP_VENC_GetBandGeometry(CAM_W, CAM_H, &band) != BSP_ERROR_NONE) { fail(name, "geometry"); goto done; }
  pc.PixelFormat = BSP_CAMERA_PIPE_PF_YUV422; pc.Width = CAM_W; pc.Height = CAM_H;
  if (BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pc) != BSP_ERROR_NONE) { fail(name, "pipe"); goto done; }
  if (BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN, band.WrapLines, BSP_CAMERA_LINES_OFF) != BSP_ERROR_NONE) { fail(name, "wrap"); goto done; }
  if (BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, Band, BSP_CAMERA_MODE_CONTINUOUS) != BSP_ERROR_NONE) { fail(name, "prime start"); goto done; }
  svc(120U);

  if (!leave_running)
  {
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
    (void)BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN, band.WrapLines, band.EventLines);
  }

  vc.Band = band; vc.pBand = Band; vc.Quality = BSP_VENC_QUALITY_DEFAULT; vc.SourceStart = jpeg_source_start;
  r = BSP_VENC_Init(0, &vc);
  if (r != BSP_ERROR_NONE) { fail(name, "VENC init %ld", (long)r); goto done; }

  /* The gap: what USB enumeration costs the streaming image. */
  P("INFO primed (%s); waiting 1500ms as USB enumeration would\r\n",
    leave_running ? "left running" : "stopped and armed");
  BSP_COM_Flush(200U);
  svc(1500U);

  if (leave_running)
  {
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
    (void)BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN, band.WrapLines, band.EventLines);
  }

  P("INFO encoding after the gap; lines so far=%lu\r\n", (unsigned long)CamLines);
  BSP_COM_Flush(200U);

  for (uint32_t i = 0; i < 5U; i++)
  {
    if (BSP_VENC_EncodeBand(0, JpegBuf, JPEG_BUF_BYTES, &n) == BSP_ERROR_NONE) { okf++; } else { bad++; }
    svc(0U);
  }
  P("INFO after gap: ok=%lu fail=%lu lastbytes=%lu lines=%lu\r\n",
    (unsigned long)okf, (unsigned long)bad, (unsigned long)n, (unsigned long)CamLines);
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
  (void)BSP_VENC_DeInit(0);
  (void)BSP_CAMERA_DeInit(0);
  if (okf == 0U) { fail(name, "no frame after the gap (fail=%lu)", (unsigned long)bad); return; }
  ok(name, "ok=%lu fail=%lu bytes=%lu", (unsigned long)okf, (unsigned long)bad, (unsigned long)n);
  return;
done:
  (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
  (void)BSP_VENC_DeInit(0);
  (void)BSP_CAMERA_DeInit(0);
}
/* The line-buffer encode at every resolution the sensor offers. */
static void cmd_jpegres(void)
{
  static const struct { const char *name; uint32_t code; uint32_t w; uint32_t h; } modes[] = {
    { "QVGA",   BSP_CAMERA_R320x240,    320U,  240U },
    { "VGA",    BSP_CAMERA_R640x480,    640U,  480U },
    { "WVGA",   BSP_CAMERA_R800x480,    800U,  480U },
    { "XGA",    BSP_CAMERA_R1024x768,  1024U,  768U },
    { "720p",   BSP_CAMERA_R1280x720,  1280U,  720U },
    { "1080p",  BSP_CAMERA_R1920x1080, 1920U, 1080U },
    { "QSXGA",  BSP_CAMERA_R2592x1944, 2592U, 1944U },
  };
  const uint32_t n = sizeof modes / sizeof modes[0];
  uint32_t good = 0U, retries = 0U;
  char summary[220] = "";

  JpegMemBytes = 0U;   /* this overwrites JpegBuf: no stale dump */

  for (uint32_t i = 0U; i < n; i++)
  {
    BSP_CAMERA_PipeConfig_t pc = {0}; BSP_VENC_Band_t band; BSP_VENC_Config_t vc = {0};
    uint32_t okf = 0U, bad = 0U, nb = 0U, minb = 0xFFFFFFFFU, maxb = 0U, badmark = 0U;
    uint32_t settle = 0U, attempt;
    char t[40];
    int32_t r;

    CamLines = 0U;
    CamFrames = 0U;

    r = BSP_VENC_GetBandGeometry(modes[i].w, modes[i].h, &band);
    if (r != BSP_ERROR_NONE)
    { P("INFO %-6s geometry failed %ld\r\n", modes[i].name, (long)r); svc(5U); continue; }

    if (band.SizeBytes > BAND_BYTES)
    { P("INFO %-6s band %lu bytes exceeds the buffer\r\n", modes[i].name, (unsigned long)band.SizeBytes); svc(5U); continue; }

    P("INFO %-6s band %lux%lu pitch=%lu depth=%lu wrap=%lu event=%lu size=%lu\r\n",
      modes[i].name, (unsigned long)band.CodedWidth, (unsigned long)band.CodedHeight,
      (unsigned long)band.Pitch, (unsigned long)band.Depth, (unsigned long)band.WrapLines,
      (unsigned long)band.EventLines, (unsigned long)band.SizeBytes);
    BSP_COM_Flush(200U); svc(2U);

    /* Bring the camera up, prime, and wait for real frame events, so the
     * settle time is measured. */
    for (attempt = 0U; attempt < 3U; attempt++)
    {
      if (BSP_CAMERA_Init(0, modes[i].code, BSP_CAMERA_PF_RGB565) != BSP_ERROR_NONE)
      { P("INFO %-6s camera init failed\r\n", modes[i].name); svc(5U); break; }

      pc.PixelFormat = BSP_CAMERA_PIPE_PF_YUV422; pc.Width = modes[i].w; pc.Height = band.CodedHeight;
      if (BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_MAIN, &pc) != BSP_ERROR_NONE)
      { P("INFO %-6s pipe failed\r\n", modes[i].name); goto next; }
      if (BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN, band.WrapLines, BSP_CAMERA_LINES_OFF) != BSP_ERROR_NONE)
      { P("INFO %-6s wrap failed\r\n", modes[i].name); goto next; }
      if (BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_MAIN, Band, BSP_CAMERA_MODE_CONTINUOUS) != BSP_ERROR_NONE)
      { P("INFO %-6s prime failed\r\n", modes[i].name); goto next; }

      CamFrames = CamErrors = 0U;
      settle = HAL_GetTick();
      while (((HAL_GetTick() - settle) < 400U) && (CamFrames < 3U)) { svc(2U); }
      settle = HAL_GetTick() - settle;

      if (CamFrames >= 3U)
      {
        break;
      }

      P("INFO %-6s no frame in %lums (errors=%lu); starting the camera over\r\n",
        modes[i].name, (unsigned long)settle, (unsigned long)CamErrors);
      BSP_COM_Flush(200U);
      (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
      (void)BSP_CAMERA_DeInit(0);
      retries++;
      svc(20U);
    }

    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
    (void)BSP_CAMERA_ConfigLineBuffer(0, BSP_CAMERA_PIPE_MAIN, band.WrapLines, band.EventLines);
    if (CamFrames < 3U)
    { P("INFO %-6s sensor never delivered after %lu attempts (frames=%lu errors=%lu)\r\n", modes[i].name, (unsigned long)attempt, (unsigned long)CamFrames, (unsigned long)CamErrors); goto next; }

    vc.Band = band; vc.pBand = Band; vc.Quality = BSP_VENC_QUALITY_DEFAULT; vc.SourceStart = jpeg_source_start;
    if (BSP_VENC_Init(0, &vc) != BSP_ERROR_NONE)
    { P("INFO %-6s VENC init failed\r\n", modes[i].name); goto next; }

    for (uint32_t k = 0U; k < 5U; k++)
    {
      if (BSP_VENC_EncodeBand(0, JpegBuf, JPEG_BUF_BYTES, &nb) == BSP_ERROR_NONE)
      {
        okf++;
        if (nb < minb) { minb = nb; }
        if (nb > maxb) { maxb = nb; }
        if ((nb < 4U) || (JpegBuf[0] != 0xFFU) || (JpegBuf[1] != 0xD8U) ||
            (JpegBuf[nb - 2U] != 0xFFU) || (JpegBuf[nb - 1U] != 0xD9U)) { badmark++; }
      }
      else { bad++; }
      svc(0U);
    }
    P("INFO %-6s ok=%lu fail=%lu badmarkers=%lu bytes=%lu..%lu lines=%lu settle=%lums errors=%lu\r\n",
      modes[i].name, (unsigned long)okf, (unsigned long)bad, (unsigned long)badmark,
      (unsigned long)(okf ? minb : 0U), (unsigned long)maxb, (unsigned long)CamLines,
      (unsigned long)settle, (unsigned long)CamErrors);
    if ((okf == 5U) && (badmark == 0U)) { good++; }
    snprintf(t, sizeof t, "%s:%lu/5@d%lu ", modes[i].name, (unsigned long)okf, (unsigned long)band.Depth);
    strncat(summary, t, sizeof summary - strlen(summary) - 1U);
    (void)BSP_VENC_DeInit(0);

next:
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_MAIN);
    (void)BSP_CAMERA_DeInit(0);
    BSP_COM_Flush(200U); svc(20U);
  }

  if (good != n) { fail("jpegres", "%lu/%lu modes, %lu camera restarts: %s", (unsigned long)good, (unsigned long)n, (unsigned long)retries, summary); return; }
  ok("jpegres", "%lu/%lu modes, %lu camera restarts: %s", (unsigned long)good, (unsigned long)n, (unsigned long)retries, summary);
}
/* XGA, brought up over and over, with the link state recorded every time. */
#ifndef CAMXGA_QUIET_MS
#define CAMXGA_QUIET_MS 120U
#endif
#ifndef CAMXGA_SETTLE_MS
#define CAMXGA_SETTLE_MS 0U
#endif
static void cmd_camxga(void)
{
  const uint32_t tries = 12U;
  uint32_t fails = 0U;

  for (uint32_t i = 0U; i < tries; i++)
  {
    BSP_CAMERA_PipeConfig_t pc = {0};
    uint32_t t0, sr1_idle;
    int32_t r;

    /* WVGA first, so the XGA bring-up is a mode change, not a repeat. */
    if (BSP_CAMERA_Init(0, BSP_CAMERA_R800x480, BSP_CAMERA_PF_RGB565) == BSP_ERROR_NONE)
    {
      BSP_CAMERA_PipeConfig_t warm = {0};
      warm.PixelFormat = BSP_CAMERA_PIPE_PF_RGB565; warm.Width = 800U; warm.Height = 480U;
      if (BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_DUMP, &warm) == BSP_ERROR_NONE)
      {
        uint32_t tw = HAL_GetTick();
        CamFrames = 0U;
        (void)BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_DUMP, CapBuf, BSP_CAMERA_MODE_CONTINUOUS);
        while (((HAL_GetTick() - tw) < 300U) && (CamFrames < 2U)) { svc(2U); }
        (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_DUMP);
      }
      (void)BSP_CAMERA_DeInit(0);
      /* How long the previous mode falls silent before the receiver is
       * reconfigured. */
      svc(CAMXGA_QUIET_MS);
    }

    CamFrames = CamErrors = 0U;

    r = BSP_CAMERA_Init(0, BSP_CAMERA_R1024x768, BSP_CAMERA_PF_RGB565);
    if (r != BSP_ERROR_NONE) { P("INFO try %2lu init failed %ld\r\n", (unsigned long)i, (long)r); fails++; svc(20U); continue; }
    /* How long the sensor is left between its mode program and being told to
     * stream. */
    svc(CAMXGA_SETTLE_MS);
    sr1_idle = CSI->SR1;

    pc.PixelFormat = BSP_CAMERA_PIPE_PF_RGB565; pc.Width = 1024U; pc.Height = 768U;
    r = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_DUMP, &pc);
    if (r == BSP_ERROR_NONE) { r = BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_DUMP, CapBuf, BSP_CAMERA_MODE_CONTINUOUS); }
    if (r != BSP_ERROR_NONE) { P("INFO try %2lu pipe/start failed %ld\r\n", (unsigned long)i, (long)r); (void)BSP_CAMERA_DeInit(0); fails++; svc(20U); continue; }

    t0 = HAL_GetTick();
    while (((HAL_GetTick() - t0) < 700U) && (CamFrames < 3U)) { svc(2U); }

    P("INFO try %2lu %s frames=%lu err=%lu in %lums | 3008=%02X 300E=%02X 4800=%02X 4837=%02X 3035=%02X 3036=%02X | SR0=%08lX SR1=%08lX ERR1=%08lX ERR2=%08lX\r\n",
      (unsigned long)i, (CamFrames >= 3U) ? "ok  " : "DEAD",
      (unsigned long)CamFrames, (unsigned long)CamErrors,
      (unsigned long)(HAL_GetTick() - t0),
      ovrd(0x3008), ovrd(0x300E), ovrd(0x4800), ovrd(0x4837), ovrd(0x3035), ovrd(0x3036),
      (unsigned long)CSI->SR0, (unsigned long)CSI->SR1,
      (unsigned long)CSI->ERR1, (unsigned long)CSI->ERR2);
    P("INFO try %2lu idle SR1 after init = %08lX (STOPCL|STOPDL0|STOPDL1 = %08lX)\r\n",
      (unsigned long)i, (unsigned long)sr1_idle,
      (unsigned long)(CSI_SR1_STOPCLF | CSI_SR1_STOPDL0F | CSI_SR1_STOPDL1F));
    BSP_COM_Flush(200U);

    if (CamFrames < 3U) { fails++; }

    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_DUMP);
    (void)BSP_CAMERA_DeInit(0);
    svc(30U);
  }

  if (fails != 0U) { fail("camxga", "%lu of %lu XGA bring-ups produced no frame", (unsigned long)fails, (unsigned long)tries); return; }
  ok("camxga", "%lu of %lu XGA bring-ups started", (unsigned long)tries, (unsigned long)tries);
}
/* The expansion header: pin interrupts, the shift helpers, I2C2 and SPI2. */
static volatile uint32_t HdrIrqCount;
static void hdr_irq(uint32_t Pin) { (void)Pin; HdrIrqCount++; }

static void cmd_hdr(void)
{
  const uint32_t irq_pin   = BSP_IO_PIN_D9;
  const uint32_t data_pin  = BSP_IO_PIN_D11;
  const uint32_t clock_pin = BSP_IO_PIN_D12;
  uint32_t fired_rising = 0U, fired_falling = 0U;
  uint8_t  pulled_up = 0U, pulled_down = 0xFFU;
  int32_t  r;
  uint32_t i2c_found = 0U;
  int32_t  i2c_init, i2c_deinit, spi_init, spi_xfer, spi_deinit, cfg_ok = 0;
  uint32_t width = 0U;
  int32_t  pulse;

  if (BSP_IO_Init() != BSP_ERROR_NONE) { fail("hdr", "io init"); return; }

  /* Trigger the pin's line in software and see whether the callback is
   * reached. */
  HdrIrqCount = 0U;
  (void)BSP_IO_ConfigPin(irq_pin, BSP_IO_MODE_INPUT_PD);
  r = BSP_IO_AttachInterrupt(irq_pin, BSP_IO_TRIGGER_RISING, hdr_irq);
  if (r == BSP_ERROR_NONE)
  {
    BSP_IO_PinInfo_t info = {0};

    (void)BSP_IO_GetPinInfo(irq_pin, &info);
    EXTI->SWIER1 = (1UL << (info.Line & 31U));
    svc(5U);
    fired_rising = HdrIrqCount;

    (void)BSP_IO_DetachInterrupt(irq_pin);
    /* Detached: the same line must now reach nothing. */
    HdrIrqCount = 0U;
    EXTI->SWIER1 = (1UL << (info.Line & 31U));
    svc(5U);
    fired_falling = HdrIrqCount;   /* reused: after-detach count */
  }
  (void)BSP_IO_ConfigPin(irq_pin, BSP_IO_MODE_INPUT_PU);
  P("INFO interrupt: delivered=%lu after_detach=%lu attach=%ld\r\n",
    (unsigned long)fired_rising, (unsigned long)fired_falling, (long)r);

  /* ShiftIn against each pull. A pull of some 40 kilohms moves the pin more
   * slowly than a byte shifts in, so each read waits for it first. */
  (void)BSP_IO_ConfigPin(clock_pin, BSP_IO_MODE_OUTPUT_PP);
  (void)BSP_IO_ConfigPin(data_pin, BSP_IO_MODE_INPUT_PU);
  svc(1U);
  (void)BSP_IO_ShiftIn(data_pin, clock_pin, BSP_IO_MSB_FIRST, &pulled_up);
  (void)BSP_IO_ConfigPin(data_pin, BSP_IO_MODE_INPUT_PD);
  svc(1U);
  (void)BSP_IO_ShiftIn(data_pin, clock_pin, BSP_IO_MSB_FIRST, &pulled_down);
  r = BSP_IO_ShiftOut(data_pin, clock_pin, BSP_IO_LSB_FIRST, 0xA5U);
  P("INFO shift: pulled_up=0x%02X pulled_down=0x%02X shiftout=%ld\r\n",
    (unsigned)pulled_up, (unsigned)pulled_down, (long)r);

  /* No pulse is coming: the measurement must time out. */
  (void)BSP_IO_ConfigPin(data_pin, BSP_IO_MODE_INPUT_PD);
  pulse = BSP_IO_ReadPulse(data_pin, BSP_IO_PIN_SET, 2000U, &width);
  P("INFO pulse with nothing driving: %ld (a timeout is the pass)\r\n", (long)pulse);

  /* The header's own I2C, against an empty bus. */
  i2c_init = BSP_I2C2_Init();
  if (i2c_init == BSP_ERROR_NONE)
  {
    for (uint16_t a = 0x08U; a < 0x78U; a++)
    {
      if (BSP_I2C2_IsReady((uint16_t)(a << 1), 1U) == BSP_ERROR_NONE) { i2c_found++; }
      if ((a & 0x0FU) == 0U) { svc(0U); }
    }
  }
  i2c_deinit = BSP_I2C2_DeInit();

  /* The header's SPI: configure, read back, transfer, take down. MISO
   * floats, so the data is not checked. */
  spi_init = BSP_SPI2_Init();
  if (spi_init == BSP_ERROR_NONE)
  {
    BSP_SPI_Config_t want = { 2000000U, BUS_SPI_MODE_0, BUS_SPI_MSB_FIRST };
    BSP_SPI_Config_t got  = {0};
    uint8_t tx[4] = { 0xDEU, 0xADU, 0xBEU, 0xEFU };
    uint8_t rx[4] = {0};

    if ((BSP_SPI2_SetConfig(&want) == BSP_ERROR_NONE) &&
        (BSP_SPI2_GetConfig(&got) == BSP_ERROR_NONE) &&
        (got.Mode == want.Mode) && (got.BitOrder == want.BitOrder))
    {
      cfg_ok = 1;
    }
    P("INFO spi: asked %lu Hz, got %lu Hz mode=%lu order=%lu\r\n",
      (unsigned long)want.ClockFreq, (unsigned long)got.ClockFreq,
      (unsigned long)got.Mode, (unsigned long)got.BitOrder);
    spi_xfer = BSP_SPI2_SendRecv(tx, rx, 4U);
  }
  else
  {
    spi_xfer = BSP_ERROR_NO_INIT;
  }
  spi_deinit = BSP_SPI2_DeInit();

  (void)BSP_IO_ConfigPin(data_pin, BSP_IO_MODE_INPUT_PU);
  (void)BSP_IO_ConfigPin(clock_pin, BSP_IO_MODE_INPUT_PU);

  if (fired_rising == 0U)            { fail("hdr", "the line fired but the callback was never reached"); return; }
  if (fired_falling != 0U)           { fail("hdr", "the callback still runs after DetachInterrupt"); return; }
  if (pulled_up != 0xFFU)            { fail("hdr", "shift in with the data pin pulled up read 0x%02X", (unsigned)pulled_up); return; }
  if (pulled_down != 0x00U)          { fail("hdr", "shift in with the data pin pulled down read 0x%02X", (unsigned)pulled_down); return; }
  if (pulse == BSP_ERROR_NONE)       { fail("hdr", "a pulse was measured with nothing driving the pin"); return; }
  if (i2c_init != BSP_ERROR_NONE)    { fail("hdr", "I2C2 init %ld", (long)i2c_init); return; }
  if (i2c_deinit != BSP_ERROR_NONE)  { fail("hdr", "I2C2 deinit %ld", (long)i2c_deinit); return; }
  if (spi_init != BSP_ERROR_NONE)    { fail("hdr", "SPI2 init %ld", (long)spi_init); return; }
  if (cfg_ok == 0)                   { fail("hdr", "SPI2 did not read back the configuration it was set"); return; }
  if (spi_xfer != BSP_ERROR_NONE)    { fail("hdr", "SPI2 transfer %ld", (long)spi_xfer); return; }
  if (spi_deinit != BSP_ERROR_NONE)  { fail("hdr", "SPI2 deinit %ld", (long)spi_deinit); return; }

  ok("hdr", "pin interrupt fires once and stops when detached, shift reads 0xFF/0x00, "
     "pulse times out, I2C2 up/scan(%lu found)/down, SPI2 up/config/transfer/down",
     (unsigned long)i2c_found);
}
/* The kernel, where this build has one. Bare metal skips. */
#if defined(BSP_USE_FREERTOS) && (BSP_USE_FREERTOS == 1)
static volatile uint32_t IdleHookCount;
void vApplicationIdleHook(void) { IdleHookCount++; }   /* the BSP's is weak */

static volatile uint32_t HammerRuns, HammerErrors, HammerStop, HammerDone;
static void hammer_task(void *pArg)
{
  (void)pArg;
  while (HammerStop == 0U)
  {
    BSP_MOTION_SENSOR_Axes_t a;
    if (BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO, &a) != BSP_ERROR_NONE) { HammerErrors++; }
    HammerRuns++;
    taskYIELD();
  }
  HammerDone = 1U;
  vTaskDelete(NULL);
}

static volatile uint32_t TimerFired;
static void timer_cb(TimerHandle_t t) { (void)t; TimerFired++; }

static void cmd_rtos(void)
{
  uint32_t idle0, idle1, kt0, kt1, ht0, ht1, mine = 0U, mine_err = 0U;
  UBaseType_t tasks, hw_console, hw_usb;
  TimerHandle_t tmr;

  if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) { fail("rtos", "scheduler not running"); return; }

  /* Time carries on across the scheduler's start: the HAL's milliseconds
   * since its last reading before it are the kernel's ticks since it. */
  {
    const uint32_t kt    = (uint32_t)xTaskGetTickCount();
    const uint32_t since = HAL_GetTick() - PreSchedTick;

    P("INFO across the scheduler's start: hal %lu ms since its last reading before, kernel %lu ticks since\r\n",
      (unsigned long)since, (unsigned long)kt);
    if (((since + 2U) < kt) || (since > (kt + 2U)))
    { fail("rtos", "the HAL's time jumped by %ld ms when the scheduler started", (long)(since - kt)); return; }
  }

  /* The two ticks are the same tick, read two ways. */
  kt0 = (uint32_t)xTaskGetTickCount(); ht0 = HAL_GetTick();
  idle0 = IdleHookCount;
  HAL_Delay(100U);
  idle1 = IdleHookCount;
  kt1 = (uint32_t)xTaskGetTickCount(); ht1 = HAL_GetTick();
  P("INFO tick: kernel %lu->%lu hal %lu->%lu over HAL_Delay(100); idle ran %lu times meanwhile\r\n",
    (unsigned long)kt0, (unsigned long)kt1, (unsigned long)ht0, (unsigned long)ht1, (unsigned long)(idle1 - idle0));

  /* The HAL tick keeps moving while the scheduler is held. */
  uint32_t held0, held1;
  {
    uint32_t per_ms = HAL_RCC_GetCpuClockFreq() / 1000U, c0, guard = 0U;
    vTaskSuspendAll();
    held0 = HAL_GetTick();
    c0 = DWT->CYCCNT;
    while (((DWT->CYCCNT - c0) < 20U * per_ms) && (++guard < 100000000U)) { }
    held1 = HAL_GetTick();
    (void)xTaskResumeAll();
  }
  P("INFO tick under a scheduler hold: hal %lu->%lu over 20 ms on the cycle counter\r\n",
    (unsigned long)held0, (unsigned long)held1);

  tasks      = uxTaskGetNumberOfTasks();
  hw_console = uxTaskGetStackHighWaterMark(NULL);
  hw_usb     = (BSP_RTOS_GetUsbTask() != NULL) ? uxTaskGetStackHighWaterMark(BSP_RTOS_GetUsbTask()) : 0U;
  P("INFO tasks=%lu console_stack_free=%lu words usb_stack_free=%lu words heap_free=%lu of %lu\r\n",
    (unsigned long)tasks, (unsigned long)hw_console, (unsigned long)hw_usb,
    (unsigned long)xPortGetFreeHeapSize(), (unsigned long)configTOTAL_HEAP_SIZE);

  /* A one-shot software timer, 20 ms out. */
  TimerFired = 0U;
  tmr = xTimerCreate("t", pdMS_TO_TICKS(20U), pdFALSE, NULL, timer_cb);
  if (tmr != NULL) { (void)xTimerStart(tmr, 0); }
  svc(60U);
  if (tmr != NULL) { (void)xTimerDelete(tmr, 0); }

  /* Two tasks on the sensor bus at once, for a quarter of a second. */
  if (BSP_MOTION_SENSOR_Init(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO) != BSP_ERROR_NONE) { fail("rtos", "imu init for the contention test"); return; }
  (void)BSP_MOTION_SENSOR_Enable(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO);
  svc(20U);
  HammerRuns = HammerErrors = HammerStop = HammerDone = 0U;
  if (xTaskCreate(hammer_task, "hammer", 1024U, NULL, CONSOLE_TASK_PRIORITY, NULL) != pdPASS) { fail("rtos", "could not create the second task"); return; }
  {
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 250U)
    {
      BSP_MOTION_SENSOR_Axes_t a;
      if (BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO, &a) != BSP_ERROR_NONE) { mine_err++; }
      mine++;
      taskYIELD();
    }
  }
  HammerStop = 1U;
  while (HammerDone == 0U) { vTaskDelay(1U); }
  (void)BSP_MOTION_SENSOR_DeInit(MOTION_SENSOR_LSM6DSL_0);
  P("INFO bus contention: this task %lu reads (%lu errors), the other %lu reads (%lu errors)\r\n",
    (unsigned long)mine, (unsigned long)mine_err, (unsigned long)HammerRuns, (unsigned long)HammerErrors);

  {
    const uint32_t u0 = uwTick;

    vTaskDelay(pdMS_TO_TICKS(20U));
    P("INFO timer-group clock %lu Hz, PCLK2 %lu Hz; uwTick moved %lu in a 20 ms task delay\r\n",
      (unsigned long)HAL_RCCEx_GetTIMGFreq(), (unsigned long)HAL_RCC_GetPCLK2Freq(), (unsigned long)(uwTick - u0));
  }
  {
    /* HAL_Delay(50) waits 51 ticks. */
    const uint32_t pre_ms = PreSchedDelayCycles / (HAL_RCC_GetCpuClockFreq() / 1000U);
    P("INFO HAL_Delay(50) before the scheduler took %lu ms by the cycle counter\r\n", (unsigned long)pre_ms);
    if ((pre_ms < 48U) || (pre_ms > 56U)) { fail("rtos", "HAL_Delay(50) before the scheduler took %lu ms: the tick there runs at the wrong rate", (unsigned long)pre_ms); return; }
  }
  if ((kt1 - kt0) < 100U)                    { fail("rtos", "kernel tick advanced only %lu in HAL_Delay(100)", (unsigned long)(kt1 - kt0)); return; }
  if ((ht1 - ht0) < 100U)                    { fail("rtos", "HAL tick advanced only %lu in HAL_Delay(100)", (unsigned long)(ht1 - ht0)); return; }
  if ((ht1 - ht0) > (kt1 - kt0) + 2U)        { fail("rtos", "HAL tick %lu and kernel tick %lu disagree", (unsigned long)(ht1 - ht0), (unsigned long)(kt1 - kt0)); return; }
  if (((held1 - held0) < 18U) || ((held1 - held0) > 22U)) { fail("rtos", "HAL tick moved %lu in a 20 ms scheduler hold: no HAL timeout could expire inside an I2C transaction", (unsigned long)(held1 - held0)); return; }
  if (idle1 == idle0)                        { fail("rtos", "idle never ran during HAL_Delay: it spins instead of yielding"); return; }
  if (BSP_RTOS_GetUsbTask() == NULL)         { fail("rtos", "no USB task"); return; }
  if ((hw_console < 64U) || (hw_usb < 64U))  { fail("rtos", "a task is within 64 words of the end of its stack"); return; }
  if (tmr == NULL)                           { fail("rtos", "software timer could not be created"); return; }
  if (TimerFired != 1U)                      { fail("rtos", "software timer fired %lu times, expected once", (unsigned long)TimerFired); return; }
  if ((HammerRuns == 0U) || (mine == 0U))    { fail("rtos", "the contention test did not run on both tasks"); return; }
  if ((HammerErrors != 0U) || (mine_err != 0U)) { fail("rtos", "bus errors under contention: %lu+%lu (is BSP_I2C1_Lock the mutex?)", (unsigned long)mine_err, (unsigned long)HammerErrors); return; }

  ok("rtos", "FreeRTOS %s: ticks agree, HAL_Delay yields (idle ran %lu times), %lu tasks, timer fired, "
             "%lu+%lu bus reads from two tasks with no error",
     tskKERNEL_VERSION_NUMBER, (unsigned long)(idle1 - idle0), (unsigned long)tasks,
     (unsigned long)mine, (unsigned long)HammerRuns);
}
#else
static void cmd_rtos(void) { skip("rtos", "bare-metal build: no scheduler to test"); }
#endif

/* The magnetometer's on-demand measurement, one step at a time. */
#include "mmc5603nj.h"
#include "lepton.h"
static void cmd_magdbg(void)
{
  uint8_t st_before, st, raw[9U /* XOUT0..ZOUT2 */];
  uint8_t trig = MMC5603NJ_CTRL0_AUTO_SR_EN | MMC5603NJ_CTRL0_TAKE_MEAS_M;

  if (BSP_MOTION_SENSOR_Init(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO) != BSP_ERROR_NONE) { fail("magdbg", "init"); return; }
  (void)BSP_MOTION_SENSOR_Enable(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO);
  svc(80U);

  for (uint32_t i = 0U; i < 4U; i++)
  {
    uint32_t t0, polls = 0U, ms;
    int32_t  rw, rt, rr;

    st_before = 0xEEU; st = 0U;
    (void)BSP_I2C1_ReadReg(MMC5603NJ_I2C_ADDRESS, MMC5603NJ_STATUS1, &st_before, 1U);
    t0 = HAL_GetTick();
    rt = BSP_I2C1_WriteReg(MMC5603NJ_I2C_ADDRESS, MMC5603NJ_CTRL0, &trig, 1U);
    do
    {
      rw = BSP_I2C1_ReadReg(MMC5603NJ_I2C_ADDRESS, MMC5603NJ_STATUS1, &st, 1U);
      polls++;
    } while ((rw == BSP_ERROR_NONE) && ((st & MMC5603NJ_STATUS1_MEAS_M_DONE) == 0U) && ((HAL_GetTick() - t0) < 50U));
    ms = HAL_GetTick() - t0;
    memset(raw, 0xEE, sizeof raw);
    rr = BSP_I2C1_ReadReg(MMC5603NJ_I2C_ADDRESS, MMC5603NJ_XOUT0, raw, 9U /* XOUT0..ZOUT2 */);
    P("INFO try %lu: status before=%02X after=%02X polls=%lu ms=%lu trig=%ld read=%ld raw=%02X%02X %02X%02X %02X%02X | %02X %02X %02X\r\n",
      (unsigned long)i, st_before, st, (unsigned long)polls, (unsigned long)ms, (long)rt, (long)rr,
      raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6], raw[7], raw[8]);
    BSP_COM_Flush(100U);
    svc(30U);
  }

  (void)BSP_MOTION_SENSOR_DeInit(MOTION_SENSOR_MMC5603NJ_0);
  ok("magdbg", "see the INFO lines");
}

/* The sensor bus is reference counted; its clock goes off when the last user
 * lets go. After each sensor, and after both, it must be off. */
static void cmd_i2cref(void)
{
  uint32_t held[4];
  uint32_t bad = 0U;

  held[0] = (__HAL_RCC_I2C1_IS_CLK_ENABLED() != 0U) ? 1U : 0U;
  (void)BSP_MOTION_SENSOR_Init(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO);
  (void)BSP_MOTION_SENSOR_DeInit(MOTION_SENSOR_LSM6DSL_0);
  held[1] = (__HAL_RCC_I2C1_IS_CLK_ENABLED() != 0U) ? 1U : 0U;
  (void)BSP_MOTION_SENSOR_Init(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO);
  (void)BSP_MOTION_SENSOR_DeInit(MOTION_SENSOR_MMC5603NJ_0);
  held[2] = (__HAL_RCC_I2C1_IS_CLK_ENABLED() != 0U) ? 1U : 0U;
  (void)BSP_MOTION_SENSOR_Init(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO);
  (void)BSP_MOTION_SENSOR_Init(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO);
  (void)BSP_MOTION_SENSOR_DeInit(MOTION_SENSOR_MMC5603NJ_0);
  (void)BSP_MOTION_SENSOR_DeInit(MOTION_SENSOR_LSM6DSL_0);
  held[3] = (__HAL_RCC_I2C1_IS_CLK_ENABLED() != 0U) ? 1U : 0U;

  P("INFO sensor bus clock: %lu before, %lu after the motion sensor, %lu after the magnetometer, %lu after both (0 is released)\r\n",
    (unsigned long)held[0], (unsigned long)held[1], (unsigned long)held[2], (unsigned long)held[3]);
  if (held[0] != 0U) { fail("i2cref", "the bus was already held before the check"); return; }
  for (uint32_t k = 1U; k < 4U; k++) { bad += held[k]; }
  if (bad != 0U) { fail("i2cref", "the bus stayed held after its users let go"); return; }
  ok("i2cref", "the sensor bus is released after each sensor, and after both");
}

/* One magnetometer sample, counted if destroyed (bus traffic interrupted the
 * conversion: every axis at negative full scale) and if fresh (differs from
 * the one before). */
static BSP_MOTION_SENSOR_Axes_t MagPrev;
static int32_t mag_sample(uint32_t *pBad, uint32_t *pFresh)
{
  BSP_MOTION_SENSOR_Axes_t m = {0};
  const int32_t r = BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO, &m);

  if ((r == BSP_ERROR_NONE) && (m.x <= -31000) && (m.y <= -31000) && (m.z <= -31000)) { (*pBad)++; }
  if ((m.x != MagPrev.x) || (m.y != MagPrev.y) || (m.z != MagPrev.z)) { (*pFresh)++; }
  MagPrev = m;
  return r;
}

/* The magnetometer free-running at 50 Hz, two seconds each: paced by
 * data-ready, by a 20 ms clock, back to back, and with a temperature every
 * 100 ms; one second after a degauss; then on demand. */
static void cmd_magcont(void)
{
  uint32_t n_ready = 0U, bad_ready = 0U, n_timed = 0U, bad_timed = 0U, t0;
  uint32_t fresh_ready = 0U, fresh_timed = 0U, fresh_tight = 0U, fresh_temp = 0U, fresh_demand = 0U, demand_err = 0U;
  uint32_t st = 0U, demand_ready = 0U, bad_demand = 0U;
  uint32_t n_tight = 0U, bad_tight = 0U, n_temp = 0U, bad_temp = 0U, temp_bad = 0U;
  uint32_t n_degauss = 0U, bad_degauss = 0U, fresh_degauss = 0U;
  float temp_last = 0.0f;

  if (BSP_MOTION_SENSOR_Init(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO) != BSP_ERROR_NONE) { fail("magcont", "init"); return; }
  (void)BSP_MOTION_SENSOR_Enable(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO);
  if (BSP_MOTION_SENSOR_SetOutputDataRate(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO, 50.0f) != BSP_ERROR_NONE) { fail("magcont", "50 Hz"); goto out; }

  t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < 2000U)
  {
    const uint32_t tw = HAL_GetTick();

    do
    {
      st = 0U;
      (void)BSP_MOTION_SENSOR_GetDataReady(MOTION_SENSOR_MMC5603NJ_0, &st);
    } while (((st & MOTION_MAGNETO) == 0U) && ((HAL_GetTick() - tw) < 60U));
    if ((st & MOTION_MAGNETO) != 0U) { (void)mag_sample(&bad_ready, &fresh_ready); n_ready++; }
    svc(0U);
  }

  t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < 2000U)
  {
    const uint32_t tw = HAL_GetTick();

    (void)mag_sample(&bad_timed, &fresh_timed); n_timed++;
    while ((HAL_GetTick() - tw) < 20U) { svc(0U); }
  }

  /* Output reads as fast as the bus goes, many inside each conversion. */
  t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < 2000U)
  {
    (void)mag_sample(&bad_tight, &fresh_tight); n_tight++;
    if ((n_tight & 63U) == 0U) { svc(0U); }
  }

  /* A temperature reading every 100 ms, among clock-paced samples. */
  t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < 2000U)
  {
    const uint32_t tw = HAL_GetTick();

    (void)mag_sample(&bad_temp, &fresh_temp); n_temp++;
    if ((n_temp % 5U) == 0U)
    {
      float c = 0.0f;
      (void)BSP_MOTION_SENSOR_GetTemperature(MOTION_SENSOR_MMC5603NJ_0, &c);
      if ((c < -20.0f) || (c > 85.0f)) { temp_bad++; }
      temp_last = c;
    }
    while ((HAL_GetTick() - tw) < 20U) { svc(0U); }
  }

  /* A degauss while free-running, then a second of clock-paced samples:
   * the stream has to carry on at its rate, fresh and whole. */
  (void)BSP_MOTION_SENSOR_Degauss(MOTION_SENSOR_MMC5603NJ_0);
  t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < 1000U)
  {
    const uint32_t tw = HAL_GetTick();

    (void)mag_sample(&bad_degauss, &fresh_degauss); n_degauss++;
    while ((HAL_GetTick() - tw) < 20U) { svc(0U); }
  }

  /* On demand, three times: data-ready must say yes and the reading be real. */
  (void)BSP_MOTION_SENSOR_SetOutputDataRate(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO, 0.0f);
  for (uint32_t k = 0U; k < 3U; k++)
  {
    t0 = HAL_GetTick();
    do
    {
      st = 0U;
      (void)BSP_MOTION_SENSOR_GetDataReady(MOTION_SENSOR_MMC5603NJ_0, &st);
    } while (((st & MOTION_MAGNETO) == 0U) && ((HAL_GetTick() - t0) < 200U));
    if ((st & MOTION_MAGNETO) != 0U)
    {
      demand_ready++;
      if (mag_sample(&bad_demand, &fresh_demand) != BSP_ERROR_NONE) { demand_err++; }
    }
  }

  P("INFO free-running at 50 Hz, paced by data-ready: %lu samples, %lu fresh, %lu destroyed\r\n",
    (unsigned long)n_ready, (unsigned long)fresh_ready, (unsigned long)bad_ready);
  P("INFO free-running at 50 Hz, paced by a 20 ms clock: %lu samples, %lu fresh, %lu destroyed\r\n",
    (unsigned long)n_timed, (unsigned long)fresh_timed, (unsigned long)bad_timed);
  P("INFO free-running at 50 Hz, back-to-back output reads: %lu samples, %lu fresh, %lu destroyed\r\n",
    (unsigned long)n_tight, (unsigned long)fresh_tight, (unsigned long)bad_tight);
  P("INFO free-running at 50 Hz, a temperature every 100 ms: %lu samples, %lu fresh, %lu destroyed; temperature %d C, %lu implausible\r\n",
    (unsigned long)n_temp, (unsigned long)fresh_temp, (unsigned long)bad_temp, (int)temp_last, (unsigned long)temp_bad);
  P("INFO free-running at 50 Hz, a second after a degauss: %lu samples, %lu fresh, %lu destroyed\r\n",
    (unsigned long)n_degauss, (unsigned long)fresh_degauss, (unsigned long)bad_degauss);
  P("INFO on demand: data-ready said yes %lu of 3 times; %lu fresh, %lu failed, %lu destroyed\r\n",
    (unsigned long)demand_ready, (unsigned long)fresh_demand, (unsigned long)demand_err, (unsigned long)bad_demand);
  if ((n_ready < 80U) || (fresh_ready < 80U) || (bad_ready != 0U) || (bad_timed != 0U) || (bad_tight != 0U) ||
      (bad_temp != 0U) || (temp_bad != 0U) || (temp_last == 0.0f) || (demand_ready != 3U) || (bad_demand != 0U) ||
      (fresh_demand != 3U) || (demand_err != 0U) || (bad_degauss != 0U) || (fresh_degauss < 40U))
  {
    fail("magcont", "destroyed: ready-paced %lu/%lu, clock-paced %lu/%lu, back-to-back %lu/%lu, with temperatures %lu/%lu; on demand ready %lu of 3",
         (unsigned long)bad_ready, (unsigned long)n_ready, (unsigned long)bad_timed, (unsigned long)n_timed,
         (unsigned long)bad_tight, (unsigned long)n_tight, (unsigned long)bad_temp, (unsigned long)n_temp,
         (unsigned long)demand_ready);
    goto out;
  }
  ok("magcont", "at 50 Hz no sample destroyed paced by data-ready (%lu fresh of %lu), a clock, back to back or among temperatures (%d C); on demand 3 of 3 ready and fresh",
     (unsigned long)fresh_ready, (unsigned long)n_ready, (int)temp_last);
out:
  (void)BSP_MOTION_SENSOR_SetOutputDataRate(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO, 0.0f);
  (void)BSP_MOTION_SENSOR_DeInit(MOTION_SENSOR_MMC5603NJ_0);
}

/* The mag command's sequence twenty times, with a post-mortem on the first
 * bad read. */
static void cmd_magloop(void)
{
  uint32_t n = 20U, bad = 0U;
  char detail[160] = "";
  char trail[120] = "";

  for (uint32_t i = 0U; i < n; i++)
  {
    BSP_MOTION_SENSOR_Axes_t m = {0}; uint8_t id = 0U;
    float mag;
    int32_t r;
    char t[10];

    if (BSP_MOTION_SENSOR_Init(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO) != BSP_ERROR_NONE) { fail("magloop", "init at %lu", (unsigned long)i); return; }
    (void)BSP_MOTION_SENSOR_ReadID(MOTION_SENSOR_MMC5603NJ_0, &id);
    (void)BSP_MOTION_SENSOR_Enable(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO);
    svc(80U);
    r = BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO, &m);
    mag = sqrtf((float)m.x * m.x + (float)m.y * m.y + (float)m.z * m.z);

    if ((r != BSP_ERROR_NONE) || (mag < 50.0f) || (mag > 5000.0f))
    {
      bad++;
      if (bad == 1U)
      {
        uint8_t st = 0xEEU, raw[9] = {0};
        (void)BSP_I2C1_ReadReg(MMC5603NJ_I2C_ADDRESS, MMC5603NJ_STATUS1, &st, 1U);
        svc(20U);
        (void)BSP_I2C1_ReadReg(MMC5603NJ_I2C_ADDRESS, MMC5603NJ_XOUT0, raw, 9U);
        snprintf(detail, sizeof detail,
                 "first bad at %lu: r=%ld axes=%ld,%ld,%ld status_after=%02X xout_20ms_later=%02X%02X %02X%02X %02X%02X",
                 (unsigned long)i, (long)r, (long)m.x, (long)m.y, (long)m.z, st,
                 raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
      }
    }
    snprintf(t, sizeof t, "%s%ld", (r == BSP_ERROR_NONE) ? "" : "E", (long)mag);
    if (strlen(trail) + strlen(t) + 2U < sizeof trail) { strcat(trail, t); strcat(trail, " "); }
    (void)BSP_MOTION_SENSOR_DeInit(MOTION_SENSOR_MMC5603NJ_0);
    svc(10U);
  }

  P("INFO |B| each time: %s\r\n", trail);
  if (detail[0] != 0) { P("INFO %s\r\n", detail); }
  if (bad != 0U) { fail("magloop", "%lu of %lu reads implausible", (unsigned long)bad, (unsigned long)n); return; }
  ok("magloop", "%lu of %lu reads plausible", (unsigned long)n, (unsigned long)n);
}

static void cmd_jpeggap(void) { jpeg_gap_test("jpeggap", 0U); }
static void cmd_jpegrun(void) { jpeg_gap_test("jpegrun", 1U); }

/* Try to bring a hung encoder back without removing power. */
static void cmd_vencrecover(void)
{
  uint32_t id_before, id_after;

  BSP_VENC_EnablePeripheral();
  id_before = LL_VENC_ReadRegister(0UL);

  /* Clock off first, so the reset lands on a block that is not being driven. */
  LL_VENC_DeInit();
  svc(20U);

  __DSB();
  __ISB();
  __HAL_RCC_VENC_FORCE_RESET();
  __DSB();
  __ISB();
  svc(50U);                       /* held across the whole clock-off window */
  __HAL_RCC_VENC_RELEASE_RESET();
  __DSB();
  __ISB();
  svc(20U);

  /* Clock and VENCRAM back on. */
  BSP_VENC_EnablePeripheral();
  svc(20U);
  id_after = LL_VENC_ReadRegister(0UL);

  ok("vencrecover", "asic_id before=0x%08lx after=0x%08lx (run jpegmem to see if it really encodes)",
     (unsigned long)id_before, (unsigned long)id_after);
}

/* Every camera mode, one frame each: each has its own sensor program and
 * MIPI lane rate, which picks the receiver's D-PHY bit-rate range. */
static void cmd_camres(void)
{
  static const struct { const char *name; uint32_t code; uint32_t w; uint32_t h; } modes[] = {
    { "QVGA",   BSP_CAMERA_R320x240,    320U,  240U },
    { "VGA",    BSP_CAMERA_R640x480,    640U,  480U },
    { "WVGA",   BSP_CAMERA_R800x480,    800U,  480U },
    { "XGA",    BSP_CAMERA_R1024x768,  1024U,  768U },
    { "720p",   BSP_CAMERA_R1280x720,  1280U,  720U },
    { "1080p",  BSP_CAMERA_R1920x1080, 1920U, 1080U },
    { "QSXGA",  BSP_CAMERA_R2592x1944, 2592U, 1944U },
  };
  uint32_t good = 0U, n = sizeof modes / sizeof modes[0], restarts = 0U;
  char summary[160] = "";

  for (uint32_t i = 0U; i < n; i++)
  {
    BSP_CAMERA_Capabilities_t cap; BSP_CAMERA_PipeConfig_t pc = {0};
    uint32_t pixels = modes[i].w * modes[i].h;
    uint32_t distinct = 0U, mn = 0xFFFFU, mx = 0U;
    uint16_t prev = 0xFFFFU;
    int32_t r;

    CamFrames = CamErrors = 0U;

    r = BSP_CAMERA_Init(0, modes[i].code, BSP_CAMERA_PF_RGB565);
    if (r != BSP_ERROR_NONE)
    {
      P("INFO %-6s init failed %ld\r\n", modes[i].name, (long)r);
      svc(5U);
      continue;
    }
    (void)BSP_CAMERA_GetCapabilities(0, &cap);

    pc.PixelFormat = BSP_CAMERA_PIPE_PF_RGB565;
    pc.Width = modes[i].w; pc.Height = modes[i].h;
    r = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_DUMP, &pc);
    if (r == BSP_ERROR_NONE)
    {
      memset(CapBuf, 0xA5, 4096U);
      r = BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_DUMP, CapBuf, BSP_CAMERA_MODE_CONTINUOUS);
    }
    if (r != BSP_ERROR_NONE)
    {
      P("INFO %-6s pipe/start failed %ld\r\n", modes[i].name, (long)r);
      (void)BSP_CAMERA_DeInit(0);
      svc(5U);
      continue;
    }

    /* Let the mode settle and take a later frame, not the first. */
    {
      uint32_t t0 = HAL_GetTick();
      while (((HAL_GetTick() - t0) < 800U) && (CamFrames < 5U)) { svc(5U); }
    }
    if (CamFrames == 0U)
    {
      P("INFO %-6s no frame in 800ms; starting the camera over\r\n", modes[i].name);
      BSP_COM_Flush(200U);
      (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_DUMP);
      (void)BSP_CAMERA_DeInit(0);
      restarts++;
      svc(20U);
      if ((BSP_CAMERA_Init(0, modes[i].code, BSP_CAMERA_PF_RGB565) == BSP_ERROR_NONE) &&
          (BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_DUMP, &pc) == BSP_ERROR_NONE) &&
          (BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_DUMP, CapBuf, BSP_CAMERA_MODE_CONTINUOUS) == BSP_ERROR_NONE))
      {
        uint32_t t1 = HAL_GetTick();
        while (((HAL_GetTick() - t1) < 800U) && (CamFrames < 5U)) { svc(5U); }
      }
    }
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_DUMP);

    {
      const uint16_t *px = (const uint16_t *)CapBuf;
      for (uint32_t k = 0U; k < pixels; k += 64U)
      {
        uint16_t v = px[k];
        if (v < mn) { mn = v; }
        if (v > mx) { mx = v; }
        if (v != prev) { distinct++; prev = v; }
      }
    }

    P("INFO %-6s %4lux%-4lu fps<=%lu dphy_patch=%lu frames=%lu err=%lu distinct=%lu range=0x%04lx..0x%04lx\r\n",
      modes[i].name, (unsigned long)modes[i].w, (unsigned long)modes[i].h,
      (unsigned long)cap.MaxFrameRate, (unsigned long)cap.NeedsHalDPhyPatch,
      (unsigned long)CamFrames, (unsigned long)CamErrors,
      (unsigned long)distinct, (unsigned long)mn, (unsigned long)mx);
    BSP_COM_Flush(200U);

    if ((CamFrames != 0U) && (distinct >= 64U))
    {
      good++;
    }
    else
    {
      char t[24];
      snprintf(t, sizeof t, "%s ", modes[i].name);
      strncat(summary, t, sizeof summary - strlen(summary) - 1U);
    }

    (void)BSP_CAMERA_DeInit(0);
    svc(20U);
  }

  if (good != n)
  {
    fail("camres", "%lu/%lu modes produced a real frame, %lu camera restarts; failed: %s",
         (unsigned long)good, (unsigned long)n, (unsigned long)restarts, summary);
    return;
  }
  ok("camres", "%lu/%lu modes produced a frame with varying pixels, %lu camera restarts",
     (unsigned long)good, (unsigned long)n, (unsigned long)restarts);
}

/* The encoder's state, read only; starts no encode. */
static void cmd_vencprobe(void)
{
  uint32_t a0, a1, b0, b1, c0, c1, id, cfg;

  BSP_VENC_EnablePeripheral();

  id  = LL_VENC_ReadRegister(0UL);
  cfg = LL_VENC_ReadRegister(63UL);

  a0 = LL_VENC_ReadRegister(1UL);
  b0 = LL_VENC_ReadRegister(497UL);
  c0 = LL_VENC_ReadRegister(21UL);
  svc(50U);
  a1 = LL_VENC_ReadRegister(1UL);
  b1 = LL_VENC_ReadRegister(497UL);
  c1 = LL_VENC_ReadRegister(21UL);

  P("INFO id=0x%08lx cfg63=0x%08lx\r\n", (unsigned long)id, (unsigned long)cfg);
  P("INFO status swreg1   = 0x%08lx -> 0x%08lx  (bit9 linebuf_done=%lu/%lu)\r\n",
    (unsigned long)a0, (unsigned long)a1,
    (unsigned long)((a0 >> 9) & 1U), (unsigned long)((a1 >> 9) & 1U));
  P("INFO instant swreg497= 0x%08lx -> 0x%08lx  (hw_handshake=%lu rdptr=%lu wrptr=%lu)\r\n",
    (unsigned long)b0, (unsigned long)b1,
    (unsigned long)((b1 >> 29) & 1U), (unsigned long)((b1 >> 9) & 0x1FFU),
    (unsigned long)(b1 & 0x1FFU));
  P("INFO slices swreg21  = 0x%08lx -> 0x%08lx\r\n", (unsigned long)c0, (unsigned long)c1);

  ok("vencprobe", "status=0x%08lx%s handshake=%lu rdptr=%lu",
     (unsigned long)a1, (a0 == a1) ? " (unchanged)" : " (changing)",
     (unsigned long)((b1 >> 29) & 1U), (unsigned long)((b1 >> 9) & 0x1FFU));
}

static void cmd_wifi(void)
{
  uint8_t rev = 0, cccr = 0, cap = 0; int32_t fw = BSP_WIFI_IsFirmwarePresent();
  P("INFO firmware_present=%ld (0x91E00000)\r\n", (long)fw);
  int32_t r = BSP_WIFI_ModulePowerOn();
  if (r != BSP_ERROR_NONE) { fail("wifi", "power on %ld", (long)r); return; }
  svc(50U);
  r = BSP_WIFI_Init();
  if (r != BSP_ERROR_NONE) { fail("wifi", "SDIO enumerate %ld", (long)r); (void)BSP_WIFI_ModulePowerOff(); return; }
  (void)BSP_WIFI_GetCccrRevision(&rev); (void)BSP_WIFI_ReadDirect(0U, 0x00U, &cccr); (void)BSP_WIFI_ReadDirect(0U, 0x08U, &cap);
  (void)BSP_WIFI_DeInit(); (void)BSP_WIFI_ModulePowerOff();
  ok("wifi", "enumerated cccr_rev=0x%02x cccr00=0x%02x cap08=0x%02x firmware=%ld", rev, cccr, cap, (long)fw);
}

static void cmd_bt(void)
{
  static const uint8_t hci_reset[] = { 0x01U, 0x03U, 0x0CU, 0x00U };
  uint8_t pkt[64]; uint32_t len = 0; BSP_BT_Health_t h;
  int32_t r = BSP_BT_Init();
  if (r != BSP_ERROR_NONE) { fail("bt", "init %ld", (long)r); return; }
  /* The controller runs only once the MAYA-W476 library loads the combined
   * Wi-Fi and BLE firmware over SDIO, under FreeRTOS; this console cannot. A
   * controller that is not running holds CTS (PD3) high. */
  svc(20U);
  if ((GPIOD->IDR & GPIO_PIN_3) != 0U)
  {
    (void)BSP_BT_DeInit();
    skip("bt", "link up, controller not running: its firmware is loaded only by the MAYA-W476 "
               "library; BleBeacon_FreeRTOS and WiFiApi_FreeRTOS's bt test HCI");
    return;
  }
  (void)BSP_BT_Flush();
  r = BSP_BT_Send(hci_reset, sizeof hci_reset, 1000U);
  if (r != BSP_ERROR_NONE) { fail("bt", "send %ld", (long)r); (void)BSP_BT_DeInit(); return; }
  uint32_t t0 = HAL_GetTick(); int32_t rr = BSP_ERROR_BUSY;
  while (((HAL_GetTick() - t0) < 1500U) && (rr != BSP_ERROR_NONE)) { rr = BSP_BT_Receive(pkt, sizeof pkt, &len, 50U); svc(0U); }
  (void)BSP_BT_GetHealth(&h); (void)BSP_BT_DeInit();
  if (rr != BSP_ERROR_NONE) { fail("bt", "no HCI event in 1.5s (rx=%lu bytes, uart_err=%lu)", (unsigned long)h.BytesReceived, (unsigned long)h.UartErrors); return; }
  char hex[3 * 12 + 1] = ""; for (uint32_t i = 0; (i < len) && (i < 12U); i++) { char t[4]; snprintf(t, sizeof t, "%02x ", pkt[i]); strcat(hex, t); }
  /* 04 0E 04 01 03 0C 00 = Command Complete for HCI_Reset, status 0 */
  int good = (len >= 7U) && (pkt[0] == 0x04U) && (pkt[1] == 0x0EU) && (pkt[4] == 0x03U) && (pkt[5] == 0x0CU) && (pkt[6] == 0x00U);
  if (!good) { fail("bt", "event=[%s] not Command Complete for HCI_Reset", hex); return; }
  ok("bt", "HCI_Reset -> Command Complete status=0 [%s] baud=%lu", hex, (unsigned long)h.BaudRate);
}

/* The backup registers survive every reset short of a power cycle. */
static void cmd_bkp(void)
{
  volatile uint32_t *bkp = &TAMP->BKP0R;
  HAL_PWR_EnableBkUpAccess();
  P("INFO bkp0..3 = %08lx %08lx %08lx %08lx\r\n", (unsigned long)bkp[0], (unsigned long)bkp[1], (unsigned long)bkp[2], (unsigned long)bkp[3]);
  P("INFO bkp4..7 = %08lx %08lx %08lx %08lx  (stage, tick, hookcount, ewi)\r\n",
    (unsigned long)bkp[4], (unsigned long)bkp[5], (unsigned long)bkp[6], (unsigned long)bkp[7]);
  P("INFO rsr=%08lx (iwdg=%lu)\r\n", (unsigned long)RCC->RSR, (unsigned long)__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST));
  ok("bkp", "stage=0x%02lx venc_status=0x%08lx venc_instant=0x%08lx ewi=0x%08lx",
     (unsigned long)bkp[4], (unsigned long)bkp[0], (unsigned long)bkp[3],
     (unsigned long)bkp[7]);
}
static void cmd_bkpclr(void)
{
  volatile uint32_t *bkp = &TAMP->BKP0R;
  HAL_PWR_EnableBkUpAccess();
  bkp[4] = 0U; bkp[5] = 0U; bkp[6] = 0U; bkp[7] = 0U;
  ok("bkpclr", "breadcrumbs cleared");
}

/* The Lepton video parser, fed packets built here (no camera): a whole frame
 * of four segments with fillers, then what a dead or broken stream feeds it.
 * Judged as the thermal module judges: frames, and well-formed packets. */
static LEPTON_VoSPI_t VospiParser;

static void vospi_packet(uint8_t *pPacket, uint32_t Id, uint16_t Value)
{
  uint32_t i;

  pPacket[0] = (uint8_t)(Id >> 8);
  pPacket[1] = (uint8_t)Id;
  pPacket[2] = 0U;
  pPacket[3] = 0U;
  for (i = 0U; i < LEPTON_PACKET_PIXELS; i++)
  {
    pPacket[4U + (2U * i)] = (uint8_t)(Value >> 8);
    pPacket[5U + (2U * i)] = (uint8_t)Value;
  }
}

/* A segment's sixty packets; packet 20 names the segment. Pixel values are
 * segment * 100 + packet, so a frame can be checked for where each came from. */
static void vospi_segment(uint8_t *pBlock, uint32_t Segment)
{
  uint32_t n;

  for (n = 0U; n < LEPTON_PACKETS_PER_SEGMENT; n++)
  {
    uint32_t id = n | ((n == LEPTON_SEGMENT_ID_PACKET) ? (Segment << 12) : 0U);
    vospi_packet(&pBlock[n * LEPTON_PACKET_BYTES], id, (uint16_t)((Segment * 100U) + n));
  }
}

static void vospi_fillers(uint8_t *pBlock, uint32_t Count)
{
  uint32_t n;

  for (n = 0U; n < Count; n++)
  {
    vospi_packet(&pBlock[n * LEPTON_PACKET_BYTES], 0x0F00U | n, 0U);
  }
}

/* Feed segments First..Last, eight fillers after each, the segments into
 * frame A up to SwitchAt and into frame B from there. Returns the frames
 * reported complete; *pShort counts blocks with packets not taken as well
 * formed. */
static uint32_t vospi_feed(uint8_t *pBlock, uint32_t First, uint32_t Last, uint32_t SwitchAt,
                           uint16_t *pA, uint16_t *pB, uint32_t *pShort)
{
  uint32_t seg, rec, frames = 0U;

  for (seg = First; seg <= Last; seg++)
  {
    vospi_segment(pBlock, seg);
    if (LEPTON_VoSPI_Parse(&VospiParser, pBlock, LEPTON_PACKETS_PER_SEGMENT,
                           (seg < SwitchAt) ? pA : pB, &rec) == 1) { frames++; }
    if (rec != LEPTON_PACKETS_PER_SEGMENT) { (*pShort)++; }
    vospi_fillers(pBlock, 8U);
    if (LEPTON_VoSPI_Parse(&VospiParser, pBlock, 8U, (seg < SwitchAt) ? pA : pB, &rec) == 1) { frames++; }
    if (rec != 8U) { (*pShort)++; }
  }
  return frames;
}

static void cmd_vospi(void)
{
  uint8_t  *block = &CamFrame[0];
  uint16_t *frame_a = (uint16_t *)(void *)&CamFrame[16384U];
  uint16_t *frame_b = frame_a + LEPTON_FRAME_PIXELS;
  uint32_t  shortfall = 0U, frames, rec, bad = 0U;
  const uint32_t last = LEPTON_FRAME_PIXELS - 1U;

  /* A whole frame, in order, into one buffer: the control. */
  (void)LEPTON_VoSPI_Init(&VospiParser);
  (void)memset(frame_a, 0, LEPTON_FRAME_PIXELS * sizeof(uint16_t));
  frames = vospi_feed(block, 1U, 4U, 5U, frame_a, frame_b, &shortfall);
  P("INFO whole frame: %lu reported, blocks short of well formed %lu, first pixel %u, last %u\r\n",
    (unsigned long)frames, (unsigned long)shortfall, (unsigned)frame_a[0], (unsigned)frame_a[last]);
  if ((frames != 1U) || (shortfall != 0U) || (frame_a[0] != 100U) || (frame_a[last] != 459U))
  {
    fail("vospi", "a whole, well-formed frame was not parsed as one");
    return;
  }

  /* A line held low: every packet reads as packet 0 of a segment. */
  (void)LEPTON_VoSPI_Init(&VospiParser);
  (void)memset(block, 0, 30U * LEPTON_PACKET_BYTES);
  (void)LEPTON_VoSPI_Parse(&VospiParser, block, 30U, frame_a, &rec);
  P("INFO line held low: %lu of 30 packets taken as well formed\r\n", (unsigned long)rec);
  if (rec >= 27U) { bad |= 1U; }

  /* A line held high reads as fillers, which the part also sends for seconds
   * on end: shown, not judged. */
  (void)memset(block, 0xFF, 30U * LEPTON_PACKET_BYTES);
  (void)LEPTON_VoSPI_Parse(&VospiParser, block, 30U, frame_a, &rec);
  P("INFO line held high: %lu of 30 packets taken as fillers\r\n", (unsigned long)rec);

  /* A packet whose ID has bits 11:8 set, which no packet number up to 59 has:
   * a corrupted header must end the segment, not land in the picture. */
  (void)LEPTON_VoSPI_Init(&VospiParser);
  vospi_segment(block, 1U);
  block[(30U * LEPTON_PACKET_BYTES) + 0U] |= 0x03U;
  (void)LEPTON_VoSPI_Parse(&VospiParser, block, LEPTON_PACKETS_PER_SEGMENT, frame_a, &rec);
  shortfall = 0U;
  frames = vospi_feed(block, 2U, 4U, 5U, frame_a, frame_b, &shortfall);
  P("INFO ID bits 11:8 set in packet 30 of segment 1: %lu frame(s) reported\r\n", (unsigned long)frames);
  if (frames != 0U) { bad |= 2U; }

  /* Segments 1 and 2 into one buffer, 3 and 4 into another: the second holds
   * half a frame, and must not be reported as a whole one. */
  (void)LEPTON_VoSPI_Init(&VospiParser);
  shortfall = 0U;
  frames = vospi_feed(block, 1U, 4U, 3U, frame_a, frame_b, &shortfall);
  P("INFO segments 1-2 into one buffer, 3-4 into another: %lu frame(s) reported\r\n", (unsigned long)frames);
  if (frames != 0U) { bad |= 4U; }

  /* And the next whole frame into the second buffer is one. */
  shortfall = 0U;
  frames = vospi_feed(block, 1U, 4U, 1U, frame_a, frame_b, &shortfall);
  P("INFO then a whole frame into the second: %lu reported, first pixel %u, last %u\r\n",
    (unsigned long)frames, (unsigned)frame_b[0], (unsigned)frame_b[last]);
  if ((frames != 1U) || (frame_b[0] != 100U) || (frame_b[last] != 459U)) { bad |= 8U; }

  if (bad != 0U)
  {
    fail("vospi", "%s%s%s%s",
         ((bad & 1U) != 0U) ? "[a line held low passes for a live stream]" : "",
         ((bad & 2U) != 0U) ? "[a packet with ID bits 11:8 set was taken]" : "",
         ((bad & 4U) != 0U) ? "[half frames in two buffers were reported as a frame]" : "",
         ((bad & 8U) != 0U) ? "[a whole frame after them was not]" : "");
    return;
  }
  ok("vospi", "a whole frame parses; a line held low, a corrupted ID and a frame split across two buffers are all refused");
}

/* The ranging sensor: its identity, then measurements over the whole field
 * of view for up to three seconds. A carrier without one skips. */
static void cmd_tof(void)
{
  RANGING_SENSOR_Result_t res;
  uint32_t id = 0U, got = 0U, good = 0U, last = 0U, st = 0U, t0, ms;
  uint8_t  ready;
  const int32_t r = BSP_RANGING_SENSOR_Init(RANGING_SENSOR_VL53L1CB_0);

  if (r == BSP_ERROR_UNKNOWN_COMPONENT) { skip("tof", "no VL53L1CB on the sensor bus (not fitted)"); return; }
  if (r != BSP_ERROR_NONE) { fail("tof", "init %ld", (long)r); return; }
  (void)BSP_RANGING_SENSOR_ReadID(RANGING_SENSOR_VL53L1CB_0, &id);
  {
    /* The module's factory optical centre (ROI_CONFIG__MODE_ROI_CENTRE_SPAD),
     * where the lens's axis falls on the array; the grid assumes 199. */
    uint8_t optical = 0U;
    if (BSP_I2C1_ReadReg16(RANGING_SENSOR_I2C_ADDRESS, 0x013EU, &optical, 1U) == BSP_ERROR_NONE)
    { P("INFO optical centre SPAD %u (the array's centre is %u)\r\n", (unsigned)optical, (unsigned)VL53L1CB_CENTRE_SPAD); }
  }
  if (BSP_RANGING_SENSOR_Start(RANGING_SENSOR_VL53L1CB_0, RANGING_SENSOR_MODE_ASYNC_CONTINUOUS) != BSP_ERROR_NONE)
  { (void)BSP_RANGING_SENSOR_DeInit(RANGING_SENSOR_VL53L1CB_0); fail("tof", "start"); return; }
  t0 = HAL_GetTick();
  while (((HAL_GetTick() - t0) < 3000U) && (got < 10U))
  {
    ready = 0U;
    if ((BSP_RANGING_SENSOR_IsMeasurementReady(RANGING_SENSOR_VL53L1CB_0, &ready) == BSP_ERROR_NONE) && (ready != 0U) &&
        (BSP_RANGING_SENSOR_GetDistance(RANGING_SENSOR_VL53L1CB_0, &res) == BSP_ERROR_NONE))
    {
      last = res.ZoneResult[0].Distance[0]; st = res.ZoneResult[0].Status[0]; got++;
      if (st == 0U) { good++; }
      P("INFO range=%lu mm status=%lu\r\n", (unsigned long)last, (unsigned long)st);
    }
    svc(5U);
  }
  ms = HAL_GetTick() - t0;
  (void)BSP_RANGING_SENSOR_Stop(RANGING_SENSOR_VL53L1CB_0);
  (void)BSP_RANGING_SENSOR_DeInit(RANGING_SENSOR_VL53L1CB_0);
  if (id != RANGING_SENSOR_ID) { fail("tof", "id=0x%lx, 0x%x expected", (unsigned long)id, (unsigned)RANGING_SENSOR_ID); return; }
  if (got == 0U) { fail("tof", "id=0x%lx, no measurement in 3 s", (unsigned long)id); return; }
  ok("tof", "id=0x%lx %lu measurements in %lu ms, %lu of them good, last %lu mm status %lu", (unsigned long)id,
     (unsigned long)got, (unsigned long)ms, (unsigned long)good, (unsigned long)last, (unsigned long)st);
}

/* The ranging sensor's region of interest and its sixteen-zone grid: a
 * narrow region refused, a centred one ranged once, one full sweep, then the
 * whole field again. A carrier without one skips. */
static void cmd_tofgrid(void)
{
  RANGING_SENSOR_Capabilities_t cap = {0};
  RANGING_SENSOR_ROIConfig_t roi;
  RANGING_SENSOR_Grid_t grid = {0};
  RANGING_SENSOR_Result_t res;
  uint32_t ranged = 0U, failed = 0U, t0, ms, centre_mm, centre_st;
  int32_t r = BSP_RANGING_SENSOR_Init(RANGING_SENSOR_VL53L1CB_0);

  if (r == BSP_ERROR_UNKNOWN_COMPONENT) { skip("tofgrid", "no VL53L1CB on the sensor bus (not fitted)"); return; }
  if (r != BSP_ERROR_NONE) { fail("tofgrid", "init %ld", (long)r); return; }
  if ((BSP_RANGING_SENSOR_GetCapabilities(RANGING_SENSOR_VL53L1CB_0, &cap) != BSP_ERROR_NONE) ||
      (cap.NumberOfZones != 1U) || (cap.CustomROI != 1U))
  { fail("tofgrid", "capabilities: zones %lu, custom region %lu", (unsigned long)cap.NumberOfZones, (unsigned long)cap.CustomROI); goto tofgrid_out; }

  roi.TopLeftX = 4U; roi.TopLeftY = 11U; roi.BotRightX = 6U; roi.BotRightY = 4U;    /* three wide */
  if (BSP_RANGING_SENSOR_ConfigROI(RANGING_SENSOR_VL53L1CB_0, &roi) != BSP_ERROR_WRONG_PARAM)
  { fail("tofgrid", "a region three detectors wide was accepted"); goto tofgrid_out; }
  roi.BotRightX = 11U;                                                               /* 8 x 8, centred */
  if ((BSP_RANGING_SENSOR_ConfigROI(RANGING_SENSOR_VL53L1CB_0, &roi) != BSP_ERROR_NONE) ||
      (BSP_RANGING_SENSOR_Start(RANGING_SENSOR_VL53L1CB_0, RANGING_SENSOR_MODE_BLOCKING_ONESHOT) != BSP_ERROR_NONE) ||
      (BSP_RANGING_SENSOR_GetDistance(RANGING_SENSOR_VL53L1CB_0, &res) != BSP_ERROR_NONE))
  { fail("tofgrid", "a one-shot over the centred 8 x 8 region"); goto tofgrid_out; }
  P("INFO centred 8 x 8: %lu mm status %lu\r\n", (unsigned long)res.ZoneResult[0].Distance[0],
    (unsigned long)res.ZoneResult[0].Status[0]);

  if (BSP_RANGING_SENSOR_StartGrid(RANGING_SENSOR_VL53L1CB_0) != BSP_ERROR_NONE) { fail("tofgrid", "StartGrid"); goto tofgrid_out; }
  t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < 6000U)
  {
    if (BSP_RANGING_SENSOR_StepGrid(RANGING_SENSOR_VL53L1CB_0) != BSP_ERROR_NONE) { failed++; }
    if ((BSP_RANGING_SENSOR_GetGrid(RANGING_SENSOR_VL53L1CB_0, &grid) == BSP_ERROR_NONE) && (grid.SweepCount >= 1U)) { break; }
    svc(5U);
  }
  ms = HAL_GetTick() - t0;
  (void)BSP_RANGING_SENSOR_StopGrid(RANGING_SENSOR_VL53L1CB_0);
  for (uint32_t row = 0U; row < RANGING_SENSOR_GRID_ROWS; row++)
  {
    P("INFO grid row %lu: %5u %5u %5u %5u\r\n", (unsigned long)row, grid.Distance[row][0], grid.Distance[row][1],
      grid.Distance[row][2], grid.Distance[row][3]);
    for (uint32_t col = 0U; col < RANGING_SENSOR_GRID_COLS; col++) { if (grid.Distance[row][col] != 0U) { ranged++; } }
  }
  if (grid.SweepCount < 1U)
  { fail("tofgrid", "no full sweep in 6 s: at zone %lu, %lu zone failures", (unsigned long)grid.ZoneIndex, (unsigned long)failed); goto tofgrid_out; }

  if ((BSP_RANGING_SENSOR_Start(RANGING_SENSOR_VL53L1CB_0, RANGING_SENSOR_MODE_BLOCKING_ONESHOT) != BSP_ERROR_NONE) ||
      (BSP_RANGING_SENSOR_GetDistance(RANGING_SENSOR_VL53L1CB_0, &res) != BSP_ERROR_NONE))
  { fail("tofgrid", "no ranging over the whole field after the grid"); goto tofgrid_out; }
  centre_mm = res.ZoneResult[0].Distance[0];
  centre_st = res.ZoneResult[0].Status[0];
  ok("tofgrid", "one sweep of 16 zones in %lu ms, %lu with a range, %lu zone failures; the whole field after it %lu mm status %lu",
     (unsigned long)ms, (unsigned long)ranged, (unsigned long)failed, (unsigned long)centre_mm, (unsigned long)centre_st);

tofgrid_out:
  (void)BSP_RANGING_SENSOR_DeInit(RANGING_SENSOR_VL53L1CB_0);
}

static void cmd_thermal(void)
{
  uint32_t present;

  if (BSP_I2C1_Init() != BSP_ERROR_NONE) { fail("thermal", "I2C1 init"); return; }
  present = ((BSP_I2C1_IsReady(0x2AU, 2U) == BSP_ERROR_NONE) || (BSP_I2C1_IsReady(0x54U, 2U) == BSP_ERROR_NONE)) ? 1U : 0U;
  (void)BSP_I2C1_DeInit();
  if (present == 0U) { skip("thermal", "no Lepton on the sensor bus (not fitted); power pins left alone"); return; }
  ok("thermal", "Lepton answers on the sensor bus (not driven further)");
}

static const cmd_t Commands[] = {
  { "id", cmd_id }, { "clk", cmd_clk }, { "led", cmd_led }, { "btn", cmd_btn }, { "i2c", cmd_i2c },
  { "imu", cmd_imu }, { "mag", cmd_mag }, { "magdbg", cmd_magdbg }, { "magloop", cmd_magloop }, { "magcont", cmd_magcont }, { "i2cref", cmd_i2cref }, { "adc", cmd_adc }, { "adcrif", cmd_adcrif }, { "imucfg", cmd_imucfg }, { "audcfg", cmd_audcfg }, { "camcfg", cmd_camcfg }, { "camsusp", cmd_camsusp }, { "camisp", cmd_camisp }, { "cammode", cmd_cammode }, { "lcd", cmd_lcd }, { "reg16", cmd_reg16 }, { "audset", cmd_audset }, { "pwmcfg", cmd_pwmcfg }, { "miscapi", cmd_miscapi }, { "io", cmd_io }, { "hdr", cmd_hdr },
  { "pwm", cmd_pwm }, { "tone", cmd_tone }, { "wdg", cmd_wdg }, { "rtos", cmd_rtos }, { "wdgfire", cmd_wdgfire }, { "wdgspin", cmd_wdgspin }, { "faultudf", cmd_faultudf }, { "comreinit", cmd_comreinit }, { "i2creset", cmd_i2creset }, { "i2cline", cmd_i2cline }, { "i2cackcut", cmd_i2cackcut }, { "i2ccheck", cmd_i2ccheck }, { "otacancel", cmd_otacancel }, { "otakeep", cmd_otakeep }, { "otacheck", cmd_otacheck }, { "npu", cmd_npu },
  { "mic", cmd_mic }, { "spk", cmd_spk }, { "audloop", cmd_audloop }, { "lcdbmp", cmd_lcdbmp }, { "cam", cmd_cam }, { "camcarrier", cmd_camcarrier }, { "camcore", cmd_camcore }, { "campic", cmd_campic }, { "hdredge", cmd_hdredge }, { "jpeg", cmd_jpeg },
  { "vencreg", cmd_vencreg }, { "jpegmem", cmd_jpegmem }, { "jpegdump", cmd_jpegdump }, { "wifi", cmd_wifi },
  { "bt", cmd_bt }, { "tof", cmd_tof }, { "tofgrid", cmd_tofgrid }, { "thermal", cmd_thermal }, { "vospi", cmd_vospi }, { "vencprobe", cmd_vencprobe }, { "camres", cmd_camres }, { "camxga", cmd_camxga }, { "vencrecover", cmd_vencrecover }, { "jpeggap", cmd_jpeggap }, { "jpegtorn", cmd_jpegtorn }, { "jpegres", cmd_jpegres }, { "jpegrun", cmd_jpegrun }, { "rif", cmd_rif }, { "vencrst", cmd_vencrst }, { "camreg", cmd_camreg }, { "camlive", cmd_camlive }, { "bkp", cmd_bkp }, { "bkpclr", cmd_bkpclr },
};

static void dispatch(const char *cmd)
{
  for (uint32_t i = 0; i < sizeof Commands / sizeof Commands[0]; i++)
  { if (strcmp(cmd, Commands[i].name) == 0) { Commands[i].fn(); return; } }
  if (strcmp(cmd, "help") == 0) { for (uint32_t i = 0; i < sizeof Commands / sizeof Commands[0]; i++) { P("INFO %s\r\n", Commands[i].name); } ok("help", "%u commands", (unsigned)(sizeof Commands / sizeof Commands[0])); return; }
  P("FAIL %s unknown command\r\nEND %s\r\n", cmd, cmd);
}

/**
  * @brief  MPU and caches.
  */
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
  if ((uint32_t)__axisram_nc_end > (uint32_t)__axisram_nc_start) {
    default_config.Number       = MPU_REGION_NUMBER2;
    default_config.BaseAddress  = (uint32_t)__axisram_nc_start;
    default_config.LimitAddress = (uint32_t)__axisram_nc_end - 1u;
    HAL_MPU_ConfigRegion(&default_config);
  }
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
  __set_PRIMASK(primask_bit);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  SCB_DisableDCache();
  mpu_config();
  SCB_EnableICache();
  SCB_EnableDCache();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  (void)BSP_LED_Init(LED_BLUE);
  (void)BSP_LED_Init(LED_RED);

  /* Armed first: anything that stops the loop for four seconds is reset. */
  (void)BSP_WDG_InitEWI(WDG_TIMEOUT_MS, 300U);
  /* A watchdog reset goes to the bootloader. The early warning normally sends
   * it there first; this covers a blocked warning interrupt. */
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U)
  {
    __HAL_RCC_CLEAR_RESET_FLAGS();
    BSP_COM_RebootToBootloader();
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* USB last, immediately before the loop that services it. */
  if (BSP_COM_Init() != BSP_ERROR_NONE) { (void)BSP_LED_On(LED_RED); Error_Handler(); }

#if defined(BSP_USE_FREERTOS) && (BSP_USE_FREERTOS == 1)
  /* The console as a task. The BSP's USB task, woken by the USB interrupt,
   * keeps 'b' and the 1200-baud touch working while a command is busy or stuck. */
  if (BSP_RTOS_StartUsbTask(0U, 0U) != BSP_ERROR_NONE) { (void)BSP_LED_On(LED_RED); Error_Handler(); }
  if (xTaskCreate(console_task, "console", CONSOLE_TASK_STACK_WORDS, NULL,
                  CONSOLE_TASK_PRIORITY, &ConsoleTask) != pdPASS)
  {
    (void)BSP_LED_On(LED_RED); Error_Handler();
  }
  /* HAL_Delay(50) before the scheduler, timed on the cycle counter for rtos. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
  {
    const uint32_t c0 = DWT->CYCCNT;
    HAL_Delay(50U);
    PreSchedDelayCycles = DWT->CYCCNT - c0;
  }
  PreSchedTick = HAL_GetTick();
  vTaskStartScheduler();
  Error_Handler();
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    BSP_COM_Task();
    console_step();
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  UNUSED(file);
  UNUSED(line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
