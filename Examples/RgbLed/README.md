# RGB LED

Cycles the on-board RGB LED through its eight colours, one per second,
and prints the name of each colour to the USB console.

## Hardware

  - Uses the on-board RGB LED. No wiring needed.

## Build and run

Import the projects under `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build
`Debug`. Upload only `Appli`: `FSBL` is the boot loader the board already
runs. Or:

```bash
bash harness/examples/cubebuild.sh RgbLed    # build both projects from clean, headless
python harness/examples/runex.py RgbLed      # upload, watch the console, send b, confirm the bootloader
```

## What you should see

The LED steps through red, yellow, green, cyan, blue, magenta, white and off, one second each, and the console names each colour as it changes.

## Notes

- Uses `BSP_LED_Init`, `BSP_LED_On` and `BSP_LED_Off` from `neuron6.h`, and `BSP_COM_Printf` for the console.
- `b` on the console reboots into the bootloader. A `loop()` that blocks
  without calling `delay()` ends up there too, through the watchdog.
