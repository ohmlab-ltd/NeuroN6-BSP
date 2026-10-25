# Read thermal

Reads the FLIR Lepton thermal camera, about nine frames a second, and
prints once a second the temperature at the centre of the picture and of
its coldest and hottest pixels, in degrees Celsius. Each pixel of this
camera is a temperature, in hundredths of a kelvin.

## Hardware

  - A Neuro N6 on the thermal carrier, with the Lepton 3.1R fitted. The
    carrier mounts the camera upside down, so every frame is rotated 180
    degrees.
  - PD3, the camera's power line, is the BLE flow control on other
    carriers. Nothing here uses BLE.

## Build and run

Import the projects under `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build
`Debug`. Upload only `Appli`: `FSBL` is the boot loader the board already
runs. Or:

```bash
bash harness/examples/cubebuild.sh ReadThermal    # build both projects from clean, headless
python harness/examples/runex.py ReadThermal      # upload, watch the console, send b, confirm the bootloader
```

## What you should see

Open the board's USB serial port. The camera starts when a terminal opens the console (or ten seconds after the board starts), boots for about two seconds and prints its serial number, sensor and housing temperatures and gain setting. Then, once a second: the frames in that second, the centre, coldest and hottest temperatures, and where the hottest pixel is; a hand reads about 30 to 34 C. Every ten seconds a line describes the video link.

Readings run high until the first flat-field correction, a few seconds in (a click): about 8 C on the camera tested. The camera pauses its video for about a second now and then to recalibrate, also with a click.

## Notes

- Uses `neuron6_thermal.h`: `BSP_THERMAL_PowerOn`, `BSP_THERMAL_Init`, `BSP_THERMAL_GetSerialNumber`, `BSP_THERMAL_GetFpaTemperature`, `BSP_THERMAL_GetAuxTemperature`, `BSP_THERMAL_GetAgcEnabled`, `BSP_THERMAL_SetFlip180`, `BSP_THERMAL_VideoStart`, `BSP_THERMAL_GetFrame` and `BSP_THERMAL_GetHealth`, with `BSP_THERMAL_DMA_IRQHandler` and `BSP_THERMAL_SPI_IRQHandler` called from the example's two interrupt handlers.
- `b` on the console reboots into the bootloader. A `loop()` that blocks
  without calling `delay()` ends up there too, through the watchdog.
