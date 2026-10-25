# Display colour bars

Draws eight vertical colour bars on the 800 x 480 panel and shifts them
one place to the left every second. The 768 KB frame buffer is in external
PSRAM, and the display controller scans it out continuously.

## Hardware

  - Needs a display carrier fitted. Without one the program runs and the
  drawing lands in the frame buffer, but nothing shows.

## Build and run

Import the projects under `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build
`Debug`. Upload only `Appli`: `FSBL` is the boot loader the board already
runs. Or:

```bash
bash harness/examples/cubebuild.sh LcdColourBars    # build both projects from clean, headless
python harness/examples/runex.py LcdColourBars      # upload, watch the console, send b, confirm the bootloader
```

## What you should see

Eight colour bars fill the panel and step to the left once a second, and the console reports each step.

## Notes

- Uses `BSP_LCD_Init`, `BSP_LCD_ConfigLayer`, `BSP_LCD_SetActiveLayer`, `BSP_LCD_FillRect` and `BSP_LCD_DisplayOn` from `neuron6_lcd.h`, and a frame buffer in the `.ext_psram` section, which the linker script places in external PSRAM.
- `b` on the console reboots into the bootloader. A `loop()` that blocks
  without calling `delay()` ends up there too, through the watchdog.
