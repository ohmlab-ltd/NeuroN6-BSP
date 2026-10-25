# I2C scanner

Scans the sensor bus every two seconds and prints the addresses that
acknowledge. The bus carries the motion sensor, the magnetometer and the
camera carrier's parts, so several answer with nothing connected.

## Hardware

  - Nothing to wire: the on-board sensors are on I2C1 already.

## Build and run

Import the projects under `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build
`Debug`. Upload only `Appli`: `FSBL` is the boot loader the board already
runs. Or:

```bash
bash harness/examples/cubebuild.sh WireScanner    # build both projects from clean, headless
python harness/examples/runex.py WireScanner      # upload, watch the console, send b, confirm the bootloader
```

## What you should see

Open the board's USB serial port. Every two seconds it lists the addresses that answered. With nothing connected expect 0x30 (MMC5603NJ) and 0x6A (LSM6DSL) on every board, plus 0x29 (VL53L1CB) and 0x3C (OV5640) on an OV5640 carrier; other carriers' cameras answer at their own addresses.

## Notes

- Uses `BSP_I2C1_Init` and `BSP_I2C1_IsReady` from `neuron6_bus.h`.
- `b` on the console reboots into the bootloader. A `loop()` that blocks
  without calling `delay()` ends up there too, through the watchdog.
