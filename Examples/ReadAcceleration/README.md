# Read acceleration

Reads the LSM6DSL's accelerometer and gyroscope ten times a second and
prints them to the USB console, in milli-g and milli-degrees per second.
At rest one axis reads about 1000 mg, which is gravity.

## Hardware

  - Nothing to wire: the LSM6DSL is on the board.

## Build and run

Import the projects under `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build
`Debug`. Upload only `Appli`: `FSBL` is the boot loader the board already
runs. Or:

```bash
bash harness/examples/cubebuild.sh ReadAcceleration    # build both projects from clean, headless
python harness/examples/runex.py ReadAcceleration      # upload, watch the console, send b, confirm the bootloader
```

## What you should see

Open the board's USB serial port: ten lines a second, each with the three acceleration axes in milli-g and the three rates in milli-degrees per second. Tilt the board and gravity moves between axes. If the sensor does not answer, the line gives the BSP's error code instead.

## Notes

- Uses `BSP_MOTION_SENSOR_Init`, `BSP_MOTION_SENSOR_Enable` and `BSP_MOTION_SENSOR_GetAxes` from `neuron6_motion_sensor.h`.
- `b` on the console reboots into the bootloader. A `loop()` that blocks
  without calling `delay()` ends up there too, through the watchdog.
