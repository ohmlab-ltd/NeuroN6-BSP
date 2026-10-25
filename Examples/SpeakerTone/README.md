# Speaker tone

Plays a tone of close to 1 kHz on the speaker for one second, then one
second of silence, repeatedly. The looped buffer holds a whole number of
sine cycles, so the tone is continuous and free of clicks.

## Hardware

  - Nothing to wire: the speaker is on the board.

## Build and run

Import the projects under `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build
`Debug`. Upload only `Appli`: `FSBL` is the boot loader the board already
runs. Or:

```bash
bash harness/examples/cubebuild.sh SpeakerTone    # build both projects from clean, headless
python harness/examples/runex.py SpeakerTone      # upload, watch the console, send b, confirm the bootloader
```

## What you should see

A tone for a second, silence for a second, and the console says which, after the sample rate and the tone's exact pitch. If playback does not start, the console gives the BSP's error code instead. The volume is fixed by the amplifier's strapping resistor, so there is no volume call.

## Notes

- Uses `BSP_AUDIO_OUT_Init`, `BSP_AUDIO_OUT_GetSampleRate`, `BSP_AUDIO_OUT_Play`, `BSP_AUDIO_OUT_Mute` and `BSP_AUDIO_OUT_UnMute` from `neuron6_audio.h`, and the three `BSP_AUDIO_OUT_*_CallBack` hooks.
- `b` on the console reboots into the bootloader. A `loop()` that blocks
  without calling `delay()` ends up there too, through the watchdog.
