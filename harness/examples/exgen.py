"""Generate the standalone examples: one main.c and one README each.

Every example is a setup() and a loop() followed by the same board
scaffolding, generated from one string here so that no example drifts from it.

    python exgen.py            # writes every example
    python exgen.py Blink      # writes one
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: BSD-3-Clause
# Licensed under the terms in the LICENSE file at the root of this
# repository.

import io
import os
import sys

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

# The board scaffolding appended to every example's main.c.
SCAFFOLD = r'''
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
'''

HEAD = r'''/*
  {title}

{blurb}

  The circuit:
{circuit}

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
{includes}
/* Provided by the scaffolding at the end of this file. */
static inline void     delay(uint32_t ms);
static inline uint32_t millis(void);
static void            setup(void);
static void            loop(void);

/* Formatted output to the USB console. Dropped, not blocked, with no host. */
#define print(...)   (void)BSP_COM_Printf(__VA_ARGS__)

'''

README = '''# {title}

{blurb_md}

## Hardware

{circuit}

## Build and run

Import the projects under `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build
`Debug`. Upload only `Appli`: `FSBL` is the boot loader the board already
runs. Or:

```bash
bash harness/examples/cubebuild.sh {name}    # build both projects from clean, headless
python harness/examples/runex.py {name}      # upload, watch the console, send b, confirm the bootloader
```

## What you should see

{running}

## Notes

- Uses {uses}
- `b` on the console reboots into the bootloader. A `loop()` that blocks
  without calling `delay()` ends up there too, through the watchdog.
'''

EXAMPLES = {}


def example(name, title, blurb, circuit, includes, body, uses, running, blurb_md=None):
    EXAMPLES[name] = dict(title=title, blurb=blurb, circuit=circuit, includes=includes,
                          body=body, uses=uses, running=running,
                          blurb_md=blurb_md or blurb.replace("  ", "").strip())


example(
    "Blink",
    "Blink",
    """  Turns the on-board LED on for one second, then off for one second,
  repeatedly. The BSP handles the LED's polarity: On means lit.""",
    "  - Uses the on-board RGB LED, green channel. No wiring needed.",
    "",
    r'''static void setup(void)
{
  (void)BSP_LED_Init(LED_GREEN);
}

static void loop(void)
{
  (void)BSP_LED_On(LED_GREEN);
  delay(1000U);
  (void)BSP_LED_Off(LED_GREEN);
  delay(1000U);
}
''',
    "`BSP_LED_Init`, `BSP_LED_On` and `BSP_LED_Off` from `neuron6.h`.",
    "The green LED blinks at one second on, one second off. Nothing is printed.",
)

example(
    "BlinkWithoutDelay",
    "Blink without delay",
    """  Turns the on-board LED on and off without waiting inside loop(), so the
  rest of loop() keeps running. This is the pattern to use once a program
  has to do more than one thing at a time.""",
    "  - Uses the on-board RGB LED, green channel. No wiring needed.",
    "",
    r'''/* How long to wait between changes, in milliseconds. */
static const uint32_t interval = 1000U;

static uint32_t previousMillis = 0U;
static int32_t  ledState       = 0;   /* 0 off, 1 lit */

static void setup(void)
{
  (void)BSP_LED_Init(LED_GREEN);
  (void)BSP_LED_Off(LED_GREEN);
}

static void loop(void)
{
  uint32_t currentMillis = millis();

  if ((currentMillis - previousMillis) >= interval)
  {
    previousMillis = currentMillis;
    ledState = (ledState == 0) ? 1 : 0;
    if (ledState != 0) { (void)BSP_LED_On(LED_GREEN); }
    else               { (void)BSP_LED_Off(LED_GREEN); }
  }

  /* Other work goes here: loop() is called continuously, with USB and the
   * watchdog serviced between calls, so it never needs delay(). */
}
''',
    "`BSP_LED_Init`, `BSP_LED_On` and `BSP_LED_Off` from `neuron6.h`, and `HAL_GetTick` for the millisecond count.",
    "The green LED blinks at one second on, one second off, and `loop()` never blocks.",
)

example(
    "DigitalReadSerial",
    "Digital read, to the console",
    """  Reads the on-board USER button and prints its state to the USB console
  ten times a second. The button has an external pull-down, so it reads
  pressed while held and released otherwise.""",
    "  - Uses the on-board USER button. No wiring needed.",
    "",
    r'''static void setup(void)
{
  (void)BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
}

static void loop(void)
{
  int32_t buttonState = BSP_PB_GetState(BUTTON_USER);

  print("%s\r\n", (buttonState != 0) ? "pressed" : "released");
  delay(100U);
}
''',
    "`BSP_PB_Init` and `BSP_PB_GetState` from `neuron6.h`, and `BSP_COM_Printf` for the console.",
    "Open the board's USB serial port at any baud rate. It prints `released` ten times a second, and `pressed` while the USER button is held.",
)

example(
    "ButtonInterrupt",
    "Button interrupt",
    """  Counts presses of the on-board USER button with an interrupt, so the count
  is right even while loop() is busy. The button is on EXTI line 8; the BSP
  configures the line and its priority, the application only the callback.""",
    "  - Uses the on-board USER button. No wiring needed.",
    "",
    r'''/* Written by the interrupt, read by loop(), so it must be volatile. */
static volatile uint32_t pressCount = 0U;

/* The BSP calls this from the button's interrupt handler. */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    pressCount++;
  }
}

static void setup(void)
{
  (void)BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
}

static void loop(void)
{
  static uint32_t lastReported = 0U;
  uint32_t now = pressCount;

  if (now != lastReported)
  {
    lastReported = now;
    print("presses: %lu\r\n", (unsigned long)now);
  }
  delay(50U);
}
''',
    "`BSP_PB_Init` in `BUTTON_MODE_EXTI` and the `BSP_PB_Callback` hook from `neuron6.h`, and `BSP_COM_Printf` for the console.",
    "Open the board's USB serial port. Each press of the USER button prints the running count. The button bounces, so one press can count two or three times.",
)

example(
    "AnalogReadSerial",
    "Analog read, to the console",
    """  Reads the analogue input A0 five times a second and prints it to the USB
  console as raw counts and as millivolts.

  The analogue inputs are referenced to 1.8 V, not 3.3 V: A0 reads full scale
  at 1.8 V and is not rated above it, so a higher voltage needs a divider.""",
    "  - A potentiometer's wiper to A0, its outer pins to 1V8 and GND.",
    '#include "neuron6_adc.h"\n#include "neuron6_io.h"\n',
    r'''static int32_t adc;   /* how the ADC came up: BSP_ERROR_NONE if it did */

static void setup(void)
{
  adc = BSP_ADC_Init();
  if (adc == BSP_ERROR_NONE)
  {
    adc = BSP_ADC_SetResolution(BSP_ADC_RESOLUTION_12B);   /* 0 to 4095 */
  }
}

static void loop(void)
{
  uint32_t counts = 0U;
  uint32_t supply = 0U;
  int32_t  status = adc;

  /* Full scale, 4096 counts, is the analogue supply. */
  if (status == BSP_ERROR_NONE)
  {
    status = BSP_ADC_GetVdda(&supply);
  }
  if (status == BSP_ERROR_NONE)
  {
    status = BSP_ADC_ReadChannel(BSP_IO_PIN_A0, &counts);
  }
  if (status != BSP_ERROR_NONE)
  {
    print("ADC error %ld\r\n", (long)status);
    delay(1000U);
    return;
  }
  print("%lu  %lu mV\r\n", (unsigned long)counts, (unsigned long)((supply * counts) / 4096U));
  delay(200U);
}
''',
    "`BSP_ADC_Init`, `BSP_ADC_SetResolution`, `BSP_ADC_GetVdda` and `BSP_ADC_ReadChannel` from `neuron6_adc.h`, and `BSP_IO_PIN_A0` from `neuron6_io.h`.",
    "Open the board's USB serial port: the raw count and the millivolts from one conversion, five times a second. With nothing connected the pin floats and the reading wanders. If the ADC does not start or a conversion fails, the line gives the BSP's error code instead.",
)

example(
    "Fade",
    "Fade",
    """  Fades the red channel of the on-board RGB LED in and out with PWM. The LED
  is active low, so the program inverts the duty.""",
    "  - Uses the on-board RGB LED, red channel. No wiring needed.",
    '#include "neuron6_pwm.h"\n#include "neuron6_io.h"\n',
    r'''static int32_t brightness = 0;    /* how bright the LED is, 0 to 255 */
static int32_t fadeAmount = 5;    /* how many points to fade by       */

static void setup(void)
{
  (void)BSP_PWM_Init();             /* 1 kHz, 8-bit: duty is 0 to 255 */
}

static void loop(void)
{
  /* Active low, so the duty is inverted. It tops out at 255/256, which would
   * leave the LED glowing, so at 0 the PWM stops instead: pin high, LED off. */
  if (brightness == 0)
  {
    (void)BSP_PWM_Stop(BSP_IO_PIN_LED_RED);
  }
  else
  {
    (void)BSP_PWM_Start(BSP_IO_PIN_LED_RED, (uint32_t)(255 - brightness));
  }

  brightness += fadeAmount;
  if ((brightness <= 0) || (brightness >= 255))
  {
    fadeAmount = -fadeAmount;
  }
  delay(30U);
}
''',
    "`BSP_PWM_Init`, `BSP_PWM_Start` and `BSP_PWM_Stop` from `neuron6_pwm.h`, and `BSP_IO_PIN_LED_RED` from `neuron6_io.h`.",
    "The red LED breathes, brightening over about 1.5 seconds and dimming over the next 1.5 until it is off. Nothing is printed.",
)

example(
    "RgbLed",
    "RGB LED",
    """  Cycles the on-board RGB LED through its eight colours, one per second,
  and prints the name of each colour to the USB console.""",
    "  - Uses the on-board RGB LED. No wiring needed.",
    "",
    r'''/* A colour in the cycle: its name and its three channels. */
typedef struct
{
  const char *name;
  int32_t r, g, b;
} Step_t;

/* The eight colours three on-or-off channels can make. */
static const Step_t steps[] =
{
  { "red",     1, 0, 0 },
  { "yellow",  1, 1, 0 },
  { "green",   0, 1, 0 },
  { "cyan",    0, 1, 1 },
  { "blue",    0, 0, 1 },
  { "magenta", 1, 0, 1 },
  { "white",   1, 1, 1 },
  { "off",     0, 0, 0 },
};

static void set_led(Led_TypeDef led, int32_t on)
{
  if (on != 0) { (void)BSP_LED_On(led); }
  else         { (void)BSP_LED_Off(led); }
}

static void setup(void)
{
  (void)BSP_LED_Init(LED_RED);
  (void)BSP_LED_Init(LED_GREEN);
  (void)BSP_LED_Init(LED_BLUE);
}

static void loop(void)
{
  for (uint32_t i = 0U; i < (sizeof steps / sizeof steps[0]); i++)
  {
    print("RGB: %s\r\n", steps[i].name);
    set_led(LED_RED,   steps[i].r);
    set_led(LED_GREEN, steps[i].g);
    set_led(LED_BLUE,  steps[i].b);
    delay(1000U);
  }
}
''',
    "`BSP_LED_Init`, `BSP_LED_On` and `BSP_LED_Off` from `neuron6.h`, and `BSP_COM_Printf` for the console.",
    "The LED steps through red, yellow, green, cyan, blue, magenta, white and off, one second each, and the console names each colour as it changes.",
)


example(
    "WireScanner",
    "I2C scanner",
    """  Scans the sensor bus every two seconds and prints the addresses that
  acknowledge. The bus carries the motion sensor, the magnetometer and the
  camera carrier's parts, so several answer with nothing connected.""",
    "  - Nothing to wire: the on-board sensors are on I2C1 already.",
    '#include "neuron6_bus.h"\n',
    r'''static void setup(void)
{
  (void)BSP_I2C1_Init();
}

static void loop(void)
{
  uint32_t found = 0U;

  print("scanning...\r\n");
  for (uint16_t address = 0x08U; address <= 0x77U; address++)
  {
    /* The BSP takes the 7-bit address shifted left one bit. One try each. */
    if (BSP_I2C1_IsReady((uint16_t)(address << 1), 1U) == BSP_ERROR_NONE)
    {
      print("  device at 0x%02X\r\n", (unsigned)address);
      found++;
    }
    delay(1U);
  }
  print("%lu device%s\r\n", (unsigned long)found, (found == 1U) ? "" : "s");
  delay(2000U);
}
''',
    "`BSP_I2C1_Init` and `BSP_I2C1_IsReady` from `neuron6_bus.h`.",
    "Open the board's USB serial port. Every two seconds it lists the addresses that answered. With nothing connected expect 0x30 (MMC5603NJ) and 0x6A (LSM6DSL) on every board, plus 0x29 (VL53L1CB) and 0x3C (OV5640) on an OV5640 carrier; other carriers' cameras answer at their own addresses.",
)

example(
    "ReadAcceleration",
    "Read acceleration",
    """  Reads the LSM6DSL's accelerometer and gyroscope ten times a second and
  prints them to the USB console, in milli-g and milli-degrees per second.
  At rest one axis reads about 1000 mg, which is gravity.""",
    "  - Nothing to wire: the LSM6DSL is on the board.",
    '#include "neuron6_motion_sensor.h"\n',
    r'''static int32_t sensor;   /* how the sensor came up: BSP_ERROR_NONE if it did */

static void setup(void)
{
  sensor = BSP_MOTION_SENSOR_Init(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO);
  if (sensor == BSP_ERROR_NONE)
  {
    sensor = BSP_MOTION_SENSOR_Enable(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO);
  }
  if (sensor == BSP_ERROR_NONE)
  {
    sensor = BSP_MOTION_SENSOR_Enable(MOTION_SENSOR_LSM6DSL_0, MOTION_GYRO);
  }
}

static void loop(void)
{
  BSP_MOTION_SENSOR_Axes_t acc = {0};
  BSP_MOTION_SENSOR_Axes_t gyro = {0};
  int32_t status = sensor;

  if (status == BSP_ERROR_NONE)
  {
    status = BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_LSM6DSL_0, MOTION_ACCELERO, &acc);
  }
  if (status == BSP_ERROR_NONE)
  {
    status = BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_LSM6DSL_0, MOTION_GYRO, &gyro);
  }
  if (status != BSP_ERROR_NONE)
  {
    print("LSM6DSL error %ld\r\n", (long)status);
    delay(1000U);
    return;
  }
  print("acc mg %6ld %6ld %6ld   gyro mdps %7ld %7ld %7ld\r\n",
        (long)acc.x, (long)acc.y, (long)acc.z, (long)gyro.x, (long)gyro.y, (long)gyro.z);
  delay(100U);
}
''',
    "`BSP_MOTION_SENSOR_Init`, `BSP_MOTION_SENSOR_Enable` and `BSP_MOTION_SENSOR_GetAxes` from `neuron6_motion_sensor.h`.",
    "Open the board's USB serial port: ten lines a second, each with the three acceleration axes in milli-g and the three rates in milli-degrees per second. Tilt the board and gravity moves between axes. If the sensor does not answer, the line gives the BSP's error code instead.",
)

example(
    "ReadMagneticField",
    "Read magnetic field",
    """  Reads the MMC5603NJ magnetometer ten times a second and prints the field
  on each axis in milligauss, and its magnitude. The Earth's field is a few
  hundred milligauss; anything with a magnet in it is much more.""",
    "  - Nothing to wire: the MMC5603NJ is on the board.",
    '#include "neuron6_motion_sensor.h"\n#include <math.h>\n',
    r'''static int32_t sensor;   /* how the sensor came up: BSP_ERROR_NONE if it did */

static void setup(void)
{
  sensor = BSP_MOTION_SENSOR_Init(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO);
  if (sensor == BSP_ERROR_NONE)
  {
    sensor = BSP_MOTION_SENSOR_Enable(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO);
  }
}

static void loop(void)
{
  BSP_MOTION_SENSOR_Axes_t m = {0};
  float magnitude;
  int32_t status = sensor;

  if (status == BSP_ERROR_NONE)
  {
    status = BSP_MOTION_SENSOR_GetAxes(MOTION_SENSOR_MMC5603NJ_0, MOTION_MAGNETO, &m);
  }
  if (status != BSP_ERROR_NONE)
  {
    print("MMC5603NJ error %ld\r\n", (long)status);
    delay(1000U);
    return;
  }
  magnitude = sqrtf(((float)m.x * (float)m.x) + ((float)m.y * (float)m.y) + ((float)m.z * (float)m.z));
  /* Whole milligauss: the default C library has no floating-point printf. */
  print("mG %6ld %6ld %6ld   |B| %5ld\r\n", (long)m.x, (long)m.y, (long)m.z,
        (long)lroundf(magnitude));
  delay(100U);
}
''',
    "`BSP_MOTION_SENSOR_Init`, `BSP_MOTION_SENSOR_Enable` and `BSP_MOTION_SENSOR_GetAxes` from `neuron6_motion_sensor.h`.",
    "Open the board's USB serial port: ten lines a second with the field on each axis and its magnitude in milligauss. Rotate the board and the axes trade places while the magnitude stays roughly put; a magnet nearby changes it. If the sensor does not answer, the line gives the BSP's error code instead.",
)

example(
    "ReadDistance",
    "Read distance",
    """  Ranges with the VL53L1CB time-of-flight sensor over its whole field of
  view and prints each distance in millimetres, about ten a second, with its
  status. Status 0 is a good range; any other means no usable return, and
  the distance then means nothing.""",
    "  - Nothing to wire: the VL53L1CB is on the OV5640 carriers, the plain one\n    and the TFT-W. The thermal carrier has none, and the example says so.",
    '#include "neuron6_ranging_sensor.h"\n',
    r'''static uint8_t ranging;

static void setup(void)
{
  ranging = 0U;
}

static void loop(void)
{
  uint8_t ready = 0U;
  RANGING_SENSOR_Result_t result;

  if (ranging == 0U)
  {
    /* Started once the sensor answers; tried every two seconds until then. */
    int32_t status = BSP_RANGING_SENSOR_Init(RANGING_SENSOR_VL53L1CB_0);

    if (status != BSP_ERROR_NONE)
    {
      print("no VL53L1CB answered (%ld): this carrier may have none\r\n", (long)status);
      delay(2000U);
      return;
    }
    status = BSP_RANGING_SENSOR_Start(RANGING_SENSOR_VL53L1CB_0,
                                      RANGING_SENSOR_MODE_ASYNC_CONTINUOUS);
    if (status != BSP_ERROR_NONE)
    {
      print("the VL53L1CB would not start ranging (%ld)\r\n", (long)status);
      (void)BSP_RANGING_SENSOR_DeInit(RANGING_SENSOR_VL53L1CB_0);
      delay(2000U);
      return;
    }
    ranging = 1U;
  }

  if ((BSP_RANGING_SENSOR_IsMeasurementReady(RANGING_SENSOR_VL53L1CB_0, &ready) == BSP_ERROR_NONE)
      && (ready != 0U)
      && (BSP_RANGING_SENSOR_GetDistance(RANGING_SENSOR_VL53L1CB_0, &result) == BSP_ERROR_NONE))
  {
    print("%5lu mm  status %lu\r\n",
          (unsigned long)result.ZoneResult[0].Distance[0],
          (unsigned long)result.ZoneResult[0].Status[0]);
  }
  delay(10U);
}
''',
    "`BSP_RANGING_SENSOR_Init`, `BSP_RANGING_SENSOR_Start`, `BSP_RANGING_SENSOR_IsMeasurementReady` and `BSP_RANGING_SENSOR_GetDistance` from `neuron6_ranging_sensor.h`, which also sweeps a 4 x 4 grid of zones (`BSP_RANGING_SENSOR_StartGrid`, `StepGrid`, `GetGrid`).",
    "Open the board's USB serial port. About ten lines a second, each a distance and a status. Put a hand in front of the sensor and the number follows it.",
)

example(
    "ReadThermal",
    "Read thermal",
    """  Reads the FLIR Lepton thermal camera, about nine frames a second, and
  prints once a second the temperature at the centre of the picture and of
  its coldest and hottest pixels, in degrees Celsius. Each pixel of this
  camera is a temperature, in hundredths of a kelvin.""",
    """  - A Neuro N6 on the thermal carrier, with the Lepton 3.1R fitted. The
    carrier mounts the camera upside down, so every frame is rotated 180
    degrees.
  - PD3, the camera's power line, is the BLE flow control on other
    carriers. Nothing here uses BLE.""",
    '#include "neuron6_thermal.h"\n',
    r'''/* Video, written only by DMA: whole packet pairs and cache lines, as large
 * as one transfer can be. Read it within half its length (13 ms at the
 * camera's highest clock) or the stream skips to the next frame. */
#define VIDEO_BYTES  (49U * 8U * BSP_THERMAL_PACKET_BYTES)   /* 64288 */

static uint8_t  video[VIDEO_BYTES] __attribute__((aligned(32)));
static uint16_t frame[BSP_THERMAL_FRAME_PIXELS];

/* While booting, the camera is left alone and the loop keeps servicing USB. */
enum { CAMERA_OFF, CAMERA_BOOTING, CAMERA_RUNNING };
static int      camera     = CAMERA_OFF;
static uint32_t since      = 0U;   /* when it got there              */
static uint8_t  tried      = 0U;   /* 1 once a start has failed      */
static uint32_t lastFrame  = 0U;   /* when the last frame arrived    */
static uint32_t lastPrint  = 0U;
static uint32_t lastHealth = 0U;
static uint32_t frames     = 0U;   /* frames since the last print    */

/* The latest frame's readings, taken on arrival: the next frame is assembled
 * in the same buffer. */
static uint16_t centre, coldest, hottest;
static uint32_t hottestAt;

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

/* Init the camera, report what it is, and start the video. The command
 * interface is shut once the video starts. */
static int32_t startCamera(void)
{
  uint16_t serial[4] = {0};
  uint16_t sensor = 0U;
  uint16_t housing = 0U;
  uint8_t  gain = 0U;
  uint8_t  haveSensor;
  int32_t  status;

  status = BSP_THERMAL_Init(0);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  /* A camera that stops answering holds each read up to two seconds: a
   * delay(1) after each keeps the watchdog and the console serviced. */
  if (BSP_THERMAL_GetSerialNumber(0, serial) == BSP_ERROR_NONE)
  {
    print("Lepton serial %04X%04X%04X%04X\r\n",
          serial[3], serial[2], serial[1], serial[0]);
  }
  delay(1U);
  haveSensor = (BSP_THERMAL_GetFpaTemperature(0, &sensor) == BSP_ERROR_NONE);
  delay(1U);
  if (haveSensor && (BSP_THERMAL_GetAuxTemperature(0, &housing) == BSP_ERROR_NONE))
  {
    printCelsius("sensor", sensor);
    printCelsius(", housing", housing);
    print("\r\n");
  }
  delay(1U);
  if (BSP_THERMAL_GetAgcEnabled(0, &gain) == BSP_ERROR_NONE)
  {
    print("automatic gain %s\r\n", (gain != 0U) ? "on: pixels are not temperatures" : "off");
  }
  delay(1U);

  (void)BSP_THERMAL_SetFlip180(0, 1U);
  return BSP_THERMAL_VideoStart(0, video, VIDEO_BYTES);
}

static void setup(void)
{
}

static void loop(void)
{
  uint32_t now = millis();
  int32_t  status;

  if (camera == CAMERA_OFF)
  {
    /* Power up once a terminal opens the console, or ten seconds after the
     * start, so its report is seen; then retry every ten seconds. */
    if ((tried == 0U) && (BSP_COM_IsConnected() == 0) && (now < 10000U))
    {
      return;
    }
    if ((tried != 0U) && ((now - since) < 10000U))
    {
      return;
    }
    print("starting the thermal camera...\r\n");
    (void)BSP_THERMAL_PowerOn(0);
    camera = CAMERA_BOOTING;
    since = millis();
    return;
  }

  if (camera == CAMERA_BOOTING)
  {
    /* It boots only with the sensor bus quiet. After this wait,
     * BSP_THERMAL_Init asks at once instead of waiting with USB unserviced. */
    if ((now - since) < BSP_THERMAL_BOOT_QUIET_MS)
    {
      return;
    }
    status = startCamera();
    if (status != BSP_ERROR_NONE)
    {
      print("thermal camera not started: %ld\r\n", (long)status);
      (void)BSP_THERMAL_DeInit(0);
      camera = CAMERA_OFF;
      tried = 1U;
      since = millis();
      return;
    }
    print("video started\r\n");
    camera = CAMERA_RUNNING;
    lastFrame = millis();
    lastPrint = lastFrame;
    lastHealth = lastFrame;
    return;
  }

  /* A short wait, so USB and the watchdog are serviced between calls. */
  status = BSP_THERMAL_GetFrame(0, frame, 50U);
  if (status == BSP_ERROR_NONE)
  {
    uint32_t low = 0U;
    uint32_t high = 0U;

    for (uint32_t i = 1U; i < BSP_THERMAL_FRAME_PIXELS; i++)
    {
      if (frame[i] < frame[low])  { low = i; }
      if (frame[i] > frame[high]) { high = i; }
    }
    centre    = frame[(BSP_THERMAL_HEIGHT / 2U) * BSP_THERMAL_WIDTH + (BSP_THERMAL_WIDTH / 2U)];
    coldest   = frame[low];
    hottest   = frame[high];
    hottestAt = high;
    frames++;
    lastFrame = millis();
  }

  /* Once a second; a second with no frames (a recalibration) prints nothing. */
  now = millis();
  if ((now - lastPrint) >= 1000U)
  {
    if (frames != 0U)
    {
      print("%lu frames/s  ", (unsigned long)frames);
      printCelsius("centre", centre);
      printCelsius("  coldest", coldest);
      printCelsius("  hottest", hottest);
      print(" at %lu,%lu\r\n", (unsigned long)(hottestAt % BSP_THERMAL_WIDTH),
            (unsigned long)(hottestAt / BSP_THERMAL_WIDTH));
    }
    frames = 0U;
    lastPrint = now;
  }

  /* The stream's health, every ten seconds. */
  if ((now - lastHealth) >= 10000U)
  {
    BSP_THERMAL_Health_t h = {0};

    (void)BSP_THERMAL_GetHealth(0, &h);
    print("link %lu Hz: frames %lu, segments %lu, resyncs %lu, restarts %lu, overruns %lu, errors %lu\r\n",
          (unsigned long)h.SckHz, (unsigned long)h.FramesParsed, (unsigned long)h.SegmentsParsed,
          (unsigned long)h.Resyncs, (unsigned long)h.Restarts, (unsigned long)h.Overruns,
          (unsigned long)h.SpiErrors);
    lastHealth = now;
  }

  /* No frame for twenty seconds despite the module's own restarts: the camera
   * has stopped, so power-cycle it. The next read restarts the video while
   * the camera boots. */
  if ((now - lastFrame) >= 20000U)
  {
    print("no frame for twenty seconds: power-cycling the camera\r\n");
    (void)BSP_THERMAL_PowerOn(0);
    lastFrame = millis();
  }
}
''',
    "`neuron6_thermal.h`: `BSP_THERMAL_PowerOn`, `BSP_THERMAL_Init`, `BSP_THERMAL_GetSerialNumber`, `BSP_THERMAL_GetFpaTemperature`, `BSP_THERMAL_GetAuxTemperature`, `BSP_THERMAL_GetAgcEnabled`, `BSP_THERMAL_SetFlip180`, `BSP_THERMAL_VideoStart`, `BSP_THERMAL_GetFrame` and `BSP_THERMAL_GetHealth`, with `BSP_THERMAL_DMA_IRQHandler` and `BSP_THERMAL_SPI_IRQHandler` called from the example's two interrupt handlers.",
    "Open the board's USB serial port. The camera starts when a terminal opens the console (or ten seconds after the board starts), boots for about two seconds and prints its serial number, sensor and housing temperatures and gain setting. Then, once a second: the frames in that second, the centre, coldest and hottest temperatures, and where the hottest pixel is; a hand reads about 30 to 34 C. Every ten seconds a line describes the video link.\n\nReadings run high until the first flat-field correction, a few seconds in (a click): about 8 C on the camera tested. The camera pauses its video for about a second now and then to recalibrate, also with a click.",
)

example(
    "CheckThermal",
    "Check thermal",
    """  Checks the FLIR Lepton thermal camera a step at a time and says where it
  stops: power and reset lines, its answer on the sensor bus, boot, status
  register, a command, what it is, then its video. It times the first frame,
  draws a frame in characters every five seconds, and, if the camera does
  not answer, lists what the sensor bus does answer at.""",
    """  - A Neuro N6 on the thermal carrier, with the Lepton 3.1R fitted.
  - PD3, the camera's power line, is the BLE flow control on other
    carriers. Nothing here uses BLE.""",
    '#include "neuron6_thermal.h"\n#include "neuron6_bus.h"\n',
    r'''/* Video, written only by DMA, sized as in ReadThermal. */
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
''',
    "`neuron6_thermal.h` for the camera's power and reset lines (`BSP_THERMAL_PowerOn`, `BSP_THERMAL_GetControlPins`), its command interface (`BSP_THERMAL_Init`, `ReadRegister`, `Ping`, `GetLastResult`, `GetSerialNumber`, `GetUptime`, `GetFpaTemperature`, `GetAuxTemperature`, `GetAgcEnabled`) and its video (`SetFlip180`, `VideoStart`, `GetFrame`, `GetHealth`, and the two IRQ handlers), and `BSP_I2C1_IsReady` from `neuron6_bus.h`.",
    "Open the board's USB serial port. Seven numbered steps print OK or FAIL with what they found; the check stops at the first that matters and runs again ten seconds later. If step 2 fails, the addresses the sensor bus answers at tell a missing camera from a bus that is not working.\n\nOnce the video runs: the time to the first frame; one line a second with the frames and the coldest, mean, centre and hottest temperatures, which run high until the first flat-field correction, a few seconds in; and every five seconds the latest frame in characters (coldest a space, hottest `@`) with the link's counters. A minute in, step 8 power-cycles the camera once and times the first frame again.",
)

example(
    "SpeakerTone",
    "Speaker tone",
    """  Plays a tone of close to 1 kHz on the speaker for one second, then one
  second of silence, repeatedly. The looped buffer holds a whole number of
  sine cycles, so the tone is continuous and free of clicks.""",
    "  - Nothing to wire: the speaker is on the board.",
    '#include "neuron6_audio.h"\n#include <math.h>\n',
    r'''#define FRAMES   2048U                  /* stereo frames in the buffer   */
#define CYCLES   42U                    /* whole sine cycles it holds    */

static int16_t buffer[FRAMES * 2U] __attribute__((aligned(32)));

static int32_t  speaker;          /* how playback came up: BSP_ERROR_NONE if it did */
static uint32_t rate = 0U;        /* the sample rate it plays at                    */
static uint8_t  told = 0U;        /* 1 once the console has been told it            */

/* The BSP calls these as the DMA works through the buffer; nothing to do. */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Instance)     { (void)Instance; }
void BSP_AUDIO_OUT_TransferComplete_CallBack(uint32_t Instance) { (void)Instance; }
void BSP_AUDIO_OUT_Error_CallBack(uint32_t Instance)            { (void)Instance; }

static void setup(void)
{
  BSP_AUDIO_Init_t init = { BSP_AUDIO_OUT_DEVICE_SPEAKER, BSP_AUDIO_FREQUENCY_NATIVE,
                            BSP_AUDIO_RESOLUTION_16B, 2U, 50U };

  for (uint32_t i = 0U; i < FRAMES; i++)
  {
    int16_t v = (int16_t)(6000.0f * sinf(6.2831853f * (float)CYCLES * (float)i / (float)FRAMES));
    buffer[2U * i]      = v;   /* left  */
    buffer[2U * i + 1U] = v;   /* right */
  }

  speaker = BSP_AUDIO_OUT_Init(0, &init);
  if (speaker == BSP_ERROR_NONE)
  {
    speaker = BSP_AUDIO_OUT_GetSampleRate(0, &rate);
  }
  if (speaker == BSP_ERROR_NONE)
  {
    speaker = BSP_AUDIO_OUT_Play(0, (uint8_t *)buffer, sizeof buffer);
  }
}

static void loop(void)
{
  if (speaker != BSP_ERROR_NONE)
  {
    print("speaker error %ld\r\n", (long)speaker);
    delay(1000U);
    return;
  }

  /* Once a terminal has the console open: earlier output reaches nobody. */
  if ((told == 0U) && (BSP_COM_IsConnected() != 0))
  {
    print("sample rate %lu Hz, tone %lu Hz\r\n",
          (unsigned long)rate, (unsigned long)((rate * CYCLES) / FRAMES));
    told = 1U;
  }

  (void)BSP_AUDIO_OUT_UnMute(0);
  print("tone\r\n");
  delay(1000U);
  (void)BSP_AUDIO_OUT_Mute(0);
  print("silence\r\n");
  delay(1000U);
}
''',
    "`BSP_AUDIO_OUT_Init`, `BSP_AUDIO_OUT_GetSampleRate`, `BSP_AUDIO_OUT_Play`, `BSP_AUDIO_OUT_Mute` and `BSP_AUDIO_OUT_UnMute` from `neuron6_audio.h`, and the three `BSP_AUDIO_OUT_*_CallBack` hooks.",
    "A tone for a second, silence for a second, and the console says which, after the sample rate and the tone's exact pitch. If playback does not start, the console gives the BSP's error code instead. The volume is fixed by the amplifier's strapping resistor, so there is no volume call.",
)

example(
    "MicrophoneLevel",
    "Microphone level",
    """  Records from the on-board microphone continuously and prints the sound
  level ten times a second as a bar, with the RMS and the peak of the most
  recent buffer. Talk, clap or tap the board and watch the bar move.""",
    "  - Nothing to wire: the microphone is on the board.",
    '#include "neuron6_audio.h"\n#include <math.h>\n#include <stdlib.h>\n',
    r'''#define SAMPLES  1024U                  /* the most the staging buffer holds */

static int16_t buffer[SAMPLES] __attribute__((aligned(32)));

/* Set by the BSP from interrupt context each time the buffer is full. */
static volatile uint32_t buffersDone = 0U;

void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)     { (void)Instance; }
void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance) { (void)Instance; buffersDone++; }
void BSP_AUDIO_IN_Error_CallBack(uint32_t Instance)            { (void)Instance; }

static int32_t  microphone;       /* how recording came up: BSP_ERROR_NONE if it did */
static uint32_t rate = 0U;        /* the sample rate it records at                   */
static uint8_t  told = 0U;        /* 1 once the console has been told it             */

static void setup(void)
{
  BSP_AUDIO_Init_t init = { BSP_AUDIO_IN_DEVICE_DIGITAL_MIC, BSP_AUDIO_FREQUENCY_NATIVE,
                            BSP_AUDIO_RESOLUTION_16B, 1U, 0U };

  microphone = BSP_AUDIO_IN_Init(0, &init);
  if (microphone == BSP_ERROR_NONE)
  {
    microphone = BSP_AUDIO_IN_GetSampleRate(0, &rate);
  }
  if (microphone == BSP_ERROR_NONE)
  {
    microphone = BSP_AUDIO_IN_Record(0, (uint8_t *)buffer, sizeof buffer);
  }
}

static void loop(void)
{
  double  sum  = 0.0;
  int32_t peak = 0;
  uint32_t rms, bars;
  char bar[41];

  if (microphone != BSP_ERROR_NONE)
  {
    print("microphone error %ld\r\n", (long)microphone);
    delay(1000U);
    return;
  }

  /* Once a terminal has the console open: earlier output reaches nobody. */
  if ((told == 0U) && (BSP_COM_IsConnected() != 0))
  {
    print("sample rate %lu Hz\r\n", (unsigned long)rate);
    told = 1U;
  }

  for (uint32_t i = 0U; i < SAMPLES; i++)
  {
    int32_t v = buffer[i];
    sum += (double)v * (double)v;
    if (abs(v) > peak) { peak = abs(v); }
  }
  rms  = (uint32_t)sqrt(sum / (double)SAMPLES);
  bars = (rms > 8000U) ? 40U : (rms / 200U);
  for (uint32_t i = 0U; i < 40U; i++) { bar[i] = (i < bars) ? '#' : ' '; }
  bar[40] = '\0';
  print("|%s| rms %5lu peak %5ld buffers %lu\r\n",
        bar, (unsigned long)rms, (long)peak, (unsigned long)buffersDone);
  delay(100U);
}
''',
    "`BSP_AUDIO_IN_Init`, `BSP_AUDIO_IN_GetSampleRate` and `BSP_AUDIO_IN_Record` from `neuron6_audio.h`, and the three `BSP_AUDIO_IN_*_CallBack` hooks.",
    "Open the board's USB serial port: ten lines a second with a level bar, the RMS and peak of the latest buffer, and the number of buffers captured, which climbs while recording runs. If recording does not start, the console gives the BSP's error code instead.",
)

example(
    "LcdColourBars",
    "Display colour bars",
    """  Draws eight vertical colour bars on the 800 x 480 panel and shifts them
  one place to the left every second. The 768 KB frame buffer is in external
  PSRAM, and the display controller scans it out continuously.""",
    "  - Needs a display carrier fitted. Without one the program runs and the\n  drawing lands in the frame buffer, but nothing shows.",
    '#include "neuron6_lcd.h"\n',
    r'''/* 800 x 480 pixels at 16 bits, in external PSRAM. */
static uint8_t frameBuffer[BSP_LCD_WIDTH * BSP_LCD_HEIGHT * 2U]
    __attribute__((section(".ext_psram"), aligned(32)));

/* The eight colours, as RGB565: white, yellow, cyan, green, magenta, red, blue, black. */
static const uint16_t colours[8] = { 0xFFFFU, 0xFFE0U, 0x07FFU, 0x07E0U,
                                     0xF81FU, 0xF800U, 0x001FU, 0x0000U };

static uint32_t shift = 0U;
static int32_t  display;          /* how the display came up: BSP_ERROR_NONE if it did */

static void draw(void)
{
  const uint32_t barWidth = BSP_LCD_WIDTH / 8U;

  for (uint32_t i = 0U; i < 8U; i++)
  {
    (void)BSP_LCD_FillRect(0, i * barWidth, 0U, barWidth, BSP_LCD_HEIGHT,
                           colours[(i + shift) % 8U]);
  }
}

static void setup(void)
{
  BSP_LCD_LayerConfig_t layer = {0};

  layer.X0 = 0U;
  layer.X1 = BSP_LCD_WIDTH;
  layer.Y0 = 0U;
  layer.Y1 = BSP_LCD_HEIGHT;
  layer.PixelFormat = BSP_LCD_PIXEL_FORMAT_RGB565;
  layer.Address = (uint32_t)frameBuffer;
  layer.Alpha = 255U;                  /* opaque: 0 would show only the black behind it */

  display = BSP_LCD_Init(0, BSP_LCD_WIDTH, BSP_LCD_HEIGHT);
  if (display == BSP_ERROR_NONE)
  {
    display = BSP_LCD_ConfigLayer(0, BSP_LCD_LAYER_0, &layer);
  }
  if (display == BSP_ERROR_NONE)
  {
    display = BSP_LCD_SetActiveLayer(0, BSP_LCD_LAYER_0);
  }
  if (display == BSP_ERROR_NONE)
  {
    draw();
    display = BSP_LCD_DisplayOn(0);
  }
}

static void loop(void)
{
  delay(1000U);
  if (display != BSP_ERROR_NONE)
  {
    print("display error %ld\r\n", (long)display);
    return;
  }
  shift = (shift + 1U) % 8U;
  draw();
  print("bars shifted, first colour now index %lu\r\n", (unsigned long)shift);
}
''',
    "`BSP_LCD_Init`, `BSP_LCD_ConfigLayer`, `BSP_LCD_SetActiveLayer`, `BSP_LCD_FillRect` and `BSP_LCD_DisplayOn` from `neuron6_lcd.h`, and a frame buffer in the `.ext_psram` section, which the linker script places in external PSRAM.",
    "Eight colour bars fill the panel and step to the left once a second, and the console reports each step.",
)


example(
    "LcdCameraPreview",
    "Camera on the display",
    """  Shows the camera on the 800 x 480 panel. The capture pipe writes RGB565
  straight into the frame buffer the display controller scans out, so nothing
  copies or converts between them.""",
    "  - Needs the display carrier with its panel and camera module.",
    '#include "neuron6_camera.h"\n#include "neuron6_lcd.h"\n',
    r'''/* 800 x 480 pixels at 16 bits, in external PSRAM: the capture pipe writes
 * it and the display controller scans it out. */
static uint8_t frameBuffer[BSP_LCD_WIDTH * BSP_LCD_HEIGHT * 2U]
    __attribute__((section(".ext_psram"), aligned(32)));

static volatile uint32_t frames;
static uint32_t reported;

void BSP_CAMERA_FrameEventCallback(uint32_t Instance, uint32_t Pipe)
{
  if ((Instance == 0U) && (Pipe == BSP_CAMERA_PIPE_DUMP))
  {
    frames++;
  }
}

static int32_t startCamera(void)
{
  BSP_CAMERA_PipeConfig_t pipe = {0};
  int32_t status;

  /* The display carrier crosses the camera's data pair over. */
  status = BSP_CAMERA_SetCarrier(BSP_CAMERA_CARRIER_INVERTED);
  if (status == BSP_ERROR_NONE)
  {
    status = BSP_CAMERA_Init(0, BSP_CAMERA_R800x480, BSP_CAMERA_PF_RGB565);
  }
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  pipe.PixelFormat = BSP_CAMERA_PIPE_PF_RGB565;
  pipe.Width       = BSP_LCD_WIDTH;
  pipe.Height      = BSP_LCD_HEIGHT;
  status = BSP_CAMERA_ConfigPipe(0, BSP_CAMERA_PIPE_DUMP, &pipe);
  if (status != BSP_ERROR_NONE)
  {
    return status;
  }

  return BSP_CAMERA_Start(0, BSP_CAMERA_PIPE_DUMP, frameBuffer,
                          BSP_CAMERA_MODE_CONTINUOUS);
}

static void setup(void)
{
  BSP_LCD_LayerConfig_t layer = {0};

  (void)BSP_LCD_Init(0, BSP_LCD_WIDTH, BSP_LCD_HEIGHT);

  layer.X0 = 0U;
  layer.X1 = BSP_LCD_WIDTH;
  layer.Y0 = 0U;
  layer.Y1 = BSP_LCD_HEIGHT;
  layer.PixelFormat = BSP_LCD_PIXEL_FORMAT_RGB565;
  layer.Address = (uint32_t)frameBuffer;
  layer.Alpha = 255U;                  /* opaque: 0 would show only the black behind it */
  (void)BSP_LCD_ConfigLayer(0, BSP_LCD_LAYER_0, &layer);
  (void)BSP_LCD_SetActiveLayer(0, BSP_LCD_LAYER_0);
  (void)BSP_LCD_DisplayOn(0);

  print("camera %ld\r\n", (long)startCamera());
}

static void loop(void)
{
  uint32_t seen;

  delay(1000U);
  seen = frames;
  print("%lu frames a second\r\n", (unsigned long)(seen - reported));

  /* A bring-up can leave the link up with no frame arriving: restart it. */
  if (seen == reported)
  {
    (void)BSP_CAMERA_Stop(0, BSP_CAMERA_PIPE_DUMP);
    (void)BSP_CAMERA_DeInit(0);
    print("no frame, restarted: %ld\r\n", (long)startCamera());
  }
  reported = seen;
}
''',
    "`BSP_CAMERA_SetCarrier`, `BSP_CAMERA_Init`, `BSP_CAMERA_ConfigPipe` and `BSP_CAMERA_Start` from `neuron6_camera.h`, `BSP_LCD_Init`, `BSP_LCD_ConfigLayer`, `BSP_LCD_SetActiveLayer` and `BSP_LCD_DisplayOn` from `neuron6_lcd.h`, and one frame buffer in `.ext_psram`.",
    "The panel shows the camera at 800 x 480 and the console counts frames each second. One buffer serves both the camera and the display, so fast movement can tear.",
)


def render(name):
    e = EXAMPLES[name]
    main_c = HEAD.format(**e) + e["body"] + SCAFFOLD
    readme = README.format(name=name, **e)
    d = os.path.join(REPO, "Examples", name, "Core", "Src")
    os.makedirs(d, exist_ok=True)
    io.open(os.path.join(d, "main.c"), "w", encoding="utf-8", newline="\n").write(main_c)
    io.open(os.path.join(REPO, "Examples", name, "README.md"), "w",
            encoding="utf-8", newline="\n").write(readme)
    print("wrote Examples/%s" % name)


def main():
    names = sys.argv[1:] or list(EXAMPLES)
    for n in names:
        render(n)


if __name__ == "__main__":
    main()
