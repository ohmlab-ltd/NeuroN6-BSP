# Camera on the display

Shows the camera on the 800 x 480 panel. The capture pipe writes RGB565
straight into the frame buffer the display controller scans out, so nothing
copies or converts between them.

## Hardware

  - Needs the display carrier with its panel and camera module.

## Build and run

Import the projects under `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build
`Debug`. Upload only `Appli`: `FSBL` is the boot loader the board already
runs. Or:

```bash
bash harness/examples/cubebuild.sh LcdCameraPreview    # build both projects from clean, headless
python harness/examples/runex.py LcdCameraPreview      # upload, watch the console, send b, confirm the bootloader
```

## What you should see

The panel shows the camera at 800 x 480 and the console counts frames each second. One buffer serves both the camera and the display, so fast movement can tear.

## Notes

- Uses `BSP_CAMERA_SetCarrier`, `BSP_CAMERA_Init`, `BSP_CAMERA_ConfigPipe` and `BSP_CAMERA_Start` from `neuron6_camera.h`, `BSP_LCD_Init`, `BSP_LCD_ConfigLayer`, `BSP_LCD_SetActiveLayer` and `BSP_LCD_DisplayOn` from `neuron6_lcd.h`, and one frame buffer in `.ext_psram`.
- `b` on the console reboots into the bootloader. A `loop()` that blocks
  without calling `delay()` ends up there too, through the watchdog.
