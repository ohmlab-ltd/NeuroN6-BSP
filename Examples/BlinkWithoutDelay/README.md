# Blink without delay

Turns the on-board LED on and off without waiting inside loop(), so the
rest of loop() keeps running. This is the pattern to use once a program
has to do more than one thing at a time.

## Hardware

  - Uses the on-board RGB LED, green channel. No wiring needed.

## Build and run

Import the projects under `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build
`Debug`. Upload only `Appli`: `FSBL` is the boot loader the board already
runs. Or:

```bash
bash harness/examples/cubebuild.sh BlinkWithoutDelay    # build both projects from clean, headless
python harness/examples/runex.py BlinkWithoutDelay      # upload, watch the console, send b, confirm the bootloader
```

## What you should see

The green LED blinks at one second on, one second off, and `loop()` never blocks.

## Notes

- Uses `BSP_LED_Init`, `BSP_LED_On` and `BSP_LED_Off` from `neuron6.h`, and `HAL_GetTick` for the millisecond count.
- `b` on the console reboots into the bootloader. A `loop()` that blocks
  without calling `delay()` ends up there too, through the watchdog.
