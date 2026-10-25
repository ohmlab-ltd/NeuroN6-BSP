# Check thermal

Checks the FLIR Lepton thermal camera a step at a time and says where it
stops: power and reset lines, its answer on the sensor bus, boot, status
register, a command, what it is, then its video. It times the first frame,
draws a frame in characters every five seconds, and, if the camera does
not answer, lists what the sensor bus does answer at.

## Hardware

  - A Neuro N6 on the thermal carrier, with the Lepton 3.1R fitted.
  - PD3, the camera's power line, is the BLE flow control on other
    carriers. Nothing here uses BLE.

## Build and run

Import the projects under `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build
`Debug`. Upload only `Appli`: `FSBL` is the boot loader the board already
runs. Or:

```bash
bash harness/examples/cubebuild.sh CheckThermal    # build both projects from clean, headless
python harness/examples/runex.py CheckThermal      # upload, watch the console, send b, confirm the bootloader
```

## What you should see

Open the board's USB serial port. Seven numbered steps print OK or FAIL with what they found; the check stops at the first that matters and runs again ten seconds later. If step 2 fails, the addresses the sensor bus answers at tell a missing camera from a bus that is not working.

Once the video runs: the time to the first frame; one line a second with the frames and the coldest, mean, centre and hottest temperatures, which run high until the first flat-field correction, a few seconds in; and every five seconds the latest frame in characters (coldest a space, hottest `@`) with the link's counters. A minute in, step 8 power-cycles the camera once and times the first frame again.

## Notes

- Uses `neuron6_thermal.h` for the camera's power and reset lines (`BSP_THERMAL_PowerOn`, `BSP_THERMAL_GetControlPins`), its command interface (`BSP_THERMAL_Init`, `ReadRegister`, `Ping`, `GetLastResult`, `GetSerialNumber`, `GetUptime`, `GetFpaTemperature`, `GetAuxTemperature`, `GetAgcEnabled`) and its video (`SetFlip180`, `VideoStart`, `GetFrame`, `GetHealth`, and the two IRQ handlers), and `BSP_I2C1_IsReady` from `neuron6_bus.h`.
- `b` on the console reboots into the bootloader. A `loop()` that blocks
  without calling `delay()` ends up there too, through the watchdog.
