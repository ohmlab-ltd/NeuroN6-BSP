# Digital read, to the console

Reads the on-board USER button and prints its state to the USB console
ten times a second. The button has an external pull-down, so it reads
pressed while held and released otherwise.

## Hardware

  - Uses the on-board USER button. No wiring needed.

## Build and run

Import the projects under `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build
`Debug`. Upload only `Appli`: `FSBL` is the boot loader the board already
runs. Or:

```bash
bash harness/examples/cubebuild.sh DigitalReadSerial    # build both projects from clean, headless
python harness/examples/runex.py DigitalReadSerial      # upload, watch the console, send b, confirm the bootloader
```

## What you should see

Open the board's USB serial port at any baud rate. It prints `released` ten times a second, and `pressed` while the USER button is held.

## Notes

- Uses `BSP_PB_Init` and `BSP_PB_GetState` from `neuron6.h`, and `BSP_COM_Printf` for the console.
- `b` on the console reboots into the bootloader. A `loop()` that blocks
  without calling `delay()` ends up there too, through the watchdog.
