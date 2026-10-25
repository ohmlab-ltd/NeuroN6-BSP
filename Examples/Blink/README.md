# Blink

Turns the on-board LED on for one second, then off for one second,
repeatedly. The BSP handles the LED's polarity: On means lit.

## Hardware

  - Uses the on-board RGB LED, green channel. No wiring needed.

## Build and run

Import the projects under `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build
`Debug`. Upload only `Appli`: `FSBL` is the boot loader the board already
runs. Or:

```bash
bash harness/examples/cubebuild.sh Blink    # build both projects from clean, headless
python harness/examples/runex.py Blink      # upload, watch the console, send b, confirm the bootloader
```

## What you should see

The green LED blinks at one second on, one second off. Nothing is printed.

## Notes

- Uses `BSP_LED_Init`, `BSP_LED_On` and `BSP_LED_Off` from `neuron6.h`.
- `b` on the console reboots into the bootloader. A `loop()` that blocks
  without calling `delay()` ends up there too, through the watchdog.
