# Read magnetic field

Reads the MMC5603NJ magnetometer ten times a second and prints the field
on each axis in milligauss, and its magnitude. The Earth's field is a few
hundred milligauss; anything with a magnet in it is much more.

## Hardware

  - Nothing to wire: the MMC5603NJ is on the board.

## Build and run

Import the projects under `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build
`Debug`. Upload only `Appli`: `FSBL` is the boot loader the board already
runs. Or:

```bash
bash harness/examples/cubebuild.sh ReadMagneticField    # build both projects from clean, headless
python harness/examples/runex.py ReadMagneticField      # upload, watch the console, send b, confirm the bootloader
```

## What you should see

Open the board's USB serial port: ten lines a second with the field on each axis and its magnitude in milligauss. Rotate the board and the axes trade places while the magnitude stays roughly put; a magnet nearby changes it. If the sensor does not answer, the line gives the BSP's error code instead.

## Notes

- Uses `BSP_MOTION_SENSOR_Init`, `BSP_MOTION_SENSOR_Enable` and `BSP_MOTION_SENSOR_GetAxes` from `neuron6_motion_sensor.h`.
- `b` on the console reboots into the bootloader. A `loop()` that blocks
  without calling `delay()` ends up there too, through the watchdog.
