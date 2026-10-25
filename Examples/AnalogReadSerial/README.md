# Analog read, to the console

Reads the analogue input A0 five times a second and prints it to the USB
console as raw counts and as millivolts.

The analogue inputs are referenced to 1.8 V, not 3.3 V: A0 reads full scale
at 1.8 V and is not rated above it, so a higher voltage needs a divider.

## Hardware

  - A potentiometer's wiper to A0, its outer pins to 1V8 and GND.

## Build and run

Import the projects under `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build
`Debug`. Upload only `Appli`: `FSBL` is the boot loader the board already
runs. Or:

```bash
bash harness/examples/cubebuild.sh AnalogReadSerial    # build both projects from clean, headless
python harness/examples/runex.py AnalogReadSerial      # upload, watch the console, send b, confirm the bootloader
```

## What you should see

Open the board's USB serial port: the raw count and the millivolts from one conversion, five times a second. With nothing connected the pin floats and the reading wanders. If the ADC does not start or a conversion fails, the line gives the BSP's error code instead.

## Notes

- Uses `BSP_ADC_Init`, `BSP_ADC_SetResolution`, `BSP_ADC_GetVdda` and `BSP_ADC_ReadChannel` from `neuron6_adc.h`, and `BSP_IO_PIN_A0` from `neuron6_io.h`.
- `b` on the console reboots into the bootloader. A `loop()` that blocks
  without calling `delay()` ends up there too, through the watchdog.
