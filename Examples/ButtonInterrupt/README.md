# Button interrupt

Counts presses of the on-board USER button with an interrupt, so the count
is right even while loop() is busy. The button is on EXTI line 8; the BSP
configures the line and its priority, the application only the callback.

## Hardware

  - Uses the on-board USER button. No wiring needed.

## Build and run

Import the projects under `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build
`Debug`. Upload only `Appli`: `FSBL` is the boot loader the board already
runs. Or:

```bash
bash harness/examples/cubebuild.sh ButtonInterrupt    # build both projects from clean, headless
python harness/examples/runex.py ButtonInterrupt      # upload, watch the console, send b, confirm the bootloader
```

## What you should see

Open the board's USB serial port. Each press of the USER button prints the running count. The button bounces, so one press can count two or three times.

## Notes

- Uses `BSP_PB_Init` in `BUTTON_MODE_EXTI` and the `BSP_PB_Callback` hook from `neuron6.h`, and `BSP_COM_Printf` for the console.
- `b` on the console reboots into the bootloader. A `loop()` that blocks
  without calling `delay()` ends up there too, through the watchdog.
