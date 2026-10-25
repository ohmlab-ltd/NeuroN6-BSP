# Fade

Fades the red channel of the on-board RGB LED in and out with PWM. The LED
is active low, so the program inverts the duty.

## Hardware

  - Uses the on-board RGB LED, red channel. No wiring needed.

## Build and run

Import the projects under `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build
`Debug`. Upload only `Appli`: `FSBL` is the boot loader the board already
runs. Or:

```bash
bash harness/examples/cubebuild.sh Fade    # build both projects from clean, headless
python harness/examples/runex.py Fade      # upload, watch the console, send b, confirm the bootloader
```

## What you should see

The red LED breathes, brightening over about 1.5 seconds and dimming over the next 1.5 until it is off. Nothing is printed.

## Notes

- Uses `BSP_PWM_Init`, `BSP_PWM_Start` and `BSP_PWM_Stop` from `neuron6_pwm.h`, and `BSP_IO_PIN_LED_RED` from `neuron6_io.h`.
- `b` on the console reboots into the bootloader. A `loop()` that blocks
  without calling `delay()` ends up there too, through the watchdog.
