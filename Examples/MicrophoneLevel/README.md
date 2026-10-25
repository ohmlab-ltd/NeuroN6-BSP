# Microphone level

Records from the on-board microphone continuously and prints the sound
level ten times a second as a bar, with the RMS and the peak of the most
recent buffer. Talk, clap or tap the board and watch the bar move.

## Hardware

  - Nothing to wire: the microphone is on the board.

## Build and run

Import the projects under `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build
`Debug`. Upload only `Appli`: `FSBL` is the boot loader the board already
runs. Or:

```bash
bash harness/examples/cubebuild.sh MicrophoneLevel    # build both projects from clean, headless
python harness/examples/runex.py MicrophoneLevel      # upload, watch the console, send b, confirm the bootloader
```

## What you should see

Open the board's USB serial port: ten lines a second with a level bar, the RMS and peak of the latest buffer, and the number of buffers captured, which climbs while recording runs. If recording does not start, the console gives the BSP's error code instead.

## Notes

- Uses `BSP_AUDIO_IN_Init`, `BSP_AUDIO_IN_GetSampleRate` and `BSP_AUDIO_IN_Record` from `neuron6_audio.h`, and the three `BSP_AUDIO_IN_*_CallBack` hooks.
- `b` on the console reboots into the bootloader. A `loop()` that blocks
  without calling `delay()` ends up there too, through the watchdog.
