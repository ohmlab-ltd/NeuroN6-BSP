# Read distance

Ranges with the VL53L1CB time-of-flight sensor over its whole field of
view and prints each distance in millimetres, about ten a second, with its
status. Status 0 is a good range; any other means no usable return, and
the distance then means nothing.

## Hardware

  - Nothing to wire: the VL53L1CB is on the OV5640 carriers, the plain one
    and the TFT-W. The thermal carrier has none, and the example says so.

## Build and run

Import the projects under `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build
`Debug`. Upload only `Appli`: `FSBL` is the boot loader the board already
runs. Or:

```bash
bash harness/examples/cubebuild.sh ReadDistance    # build both projects from clean, headless
python harness/examples/runex.py ReadDistance      # upload, watch the console, send b, confirm the bootloader
```

## What you should see

Open the board's USB serial port. About ten lines a second, each a distance and a status. Put a hand in front of the sensor and the number follows it.

## Notes

- Uses `BSP_RANGING_SENSOR_Init`, `BSP_RANGING_SENSOR_Start`, `BSP_RANGING_SENSOR_IsMeasurementReady` and `BSP_RANGING_SENSOR_GetDistance` from `neuron6_ranging_sensor.h`, which also sweeps a 4 x 4 grid of zones (`BSP_RANGING_SENSOR_StartGrid`, `StepGrid`, `GetGrid`).
- `b` on the console reboots into the bootloader. A `loop()` that blocks
  without calling `delay()` ends up there too, through the watchdog.
