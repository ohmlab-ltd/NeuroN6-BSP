# The VD66GY as a USB camera

Streams the VD66GY global-shutter camera on the thermal carrier as a standard
USB video device: Motion-JPEG, 640 x 480, 60 fps, on an isochronous endpoint,
with no driver needed on the host.

```
VD66GY --CSI-2, RAW8--> DCMIPP pipe 1 --band in AXISRAM--> VENC --JPEG--> UVC --> host
                        black level, colour,
                        white balance, colour
                        correction, gamma, YUV
```

The transport (descriptors, capture loop, encoder pacing) is
[Camera_UvcStream_USB](../Camera_UvcStream_USB/)'s.

## The sensor

`main.c` registers the sensor and the carrier before `BSP_CAMERA_Init`:

```c
BSP_CAMERA_VD66GY_Register();
BSP_CAMERA_SetCarrier(BSP_CAMERA_CARRIER_INVERTED);
BSP_CAMERA_VD66GY_SetFrameRate(UVC_FRAME_RATE);
BSP_CAMERA_Init(0, BSP_CAMERA_VD66GY_R640x480, BSP_CAMERA_PF_RAW8);
```

- **Carrier:** the thermal carrier crosses the two data lanes (`INVERTED`);
  the NeuroVision carrier wires them straight (`STANDARD`). A wrong setting
  gives no error: the sensor streams and no frame arrives. Change
  `UVC_CARRIER` at the top of `main.c`.
- **Colour is made on the chip.** The sensor sends raw readings under its
  colour filter; `isp_config()` sets pipe 1's stages from ST's daylight (D65)
  tuning for this module (STEVAL-66GYMAI1): right under daylight, warm under
  warmer light, cool under cooler. Nothing follows the light; that needs ST's
  ISP middleware, which this BSP does not carry.

| stage | setting |
|---|---|
| black level | 16 off each channel |
| colour reconstruction | the filter order the sensor reports, ST's strengths |
| white balance | red x1.71, green x1.00, blue x1.51 |
| colour correction | ST's daylight matrix; each row sums to one, so grey stays grey |
| gamma | on |

- **Exposure is the sensor's own loop**, aimed at 25 of 100
  (`UVC_EXPOSURE_TARGET`): it meters the raw readings, before white balance
  and gamma brighten the picture.
- **The rate is held to the one declared.** The sensor reads its whole
  1120 x 1364 array every frame, which caps every mode at about 88 fps; a
  faster request is accepted and never streams. VGA is a window from the
  middle of the array, so the view is narrower.
- **Start-up:** ST's driver uploads 17 KB of firmware to earlier silicon;
  a part that needs none starts in 38 ms. The watchdog is 4 s to leave room
  for the upload, and the console reports the time taken.
- **Arming:** every encode counts the frame from its start
  (`BSP_CAMERA_VsyncEventCallback`), not from its first line event. The first
  line event can come 0.95 ms after the frame starts (at 1024 x 1280) and
  arming takes about 0.43 ms, so an encode armed in between would run a band
  behind and be dropped as torn.

## Turning it on

`neuron6_conf.h` sets the video class and the picture, which the USB
descriptors and `main.c` both read; `main.c` checks the coded picture is that
size:

```c
#define BSP_USB_UVC                     1U
#define UVC_FRAME_WIDTH                 640
#define UVC_FRAME_HEIGHT                480
#define UVC_FRAME_RATE                  60
```

The `Appli` project adds to the OV5640 example's: `neuron6_camera_vd66gy.c`
and `.h` (linked from the repository root; ST's `Components/vd6g/vd6g.c` is
compiled already and includes its firmware images), and `NDEBUG`, because
ST's driver asserts on reachable error paths and a failed assertion is a
silent loop.

## The other modes

Change `UVC_RESOLUTION`, `UVC_BAND_BYTES` and `JPEG_BUF_BYTES` in `main.c`
and the three `UVC_FRAME_*` values in `neuron6_conf.h` together. The band
holds two bytes a pixel for 128 lines (64 at 640 x 480); the JPEG buffer
holds the largest frame.

| mode | top rate | band | JPEG buffer |
|---|---|---|---|
| 320 x 240 | 88 | 81920 | 512 KB |
| 640 x 480 | 88 | 81920 | 512 KB |
| 480 x 640 | 88 | 122880 | 512 KB |
| 1024 x 768 | 60 | 262144 | 512 KB |
| 768 x 1024 | 60 | 196608 | 512 KB |
| 720 x 1280 | 60 | 184320 | 512 KB |
| 1024 x 1280 | 45 | 262144 | 1 MB |
| 1120 x 720 | 60 | 286720 | 512 KB |
| 1120 x 1360, of 1364 | 45 | 286720 | 1 MB |

- The encoder codes whole 8-line blocks, so the full array's 1364 lines are
  coded and declared as 1360.
- The 1120-pixel modes read each line in 9.5 us rather than 7.7 us, so the
  encoder keeps up.
- With the sensor's random test pattern, the largest modes run below their
  top rate, limited by the isochronous endpoint (about 8 MB/s).

## Console

Every second the `[uvc]` line (frames encoded, sent, torn, failed); every five
seconds a `[cam]` line:

```
[cam] VD66GY 640x480 at 60 fps, crossed lanes: started in 38 ms from ready to start, now streaming, filter GRBG, 60 frames in the last second, 2 camera errors since start (codes 0x10)
```

- **Frames about 60, errors steady:** the picture reaches the chip. One pipe
  overrun (`0x10`) at start-up and one when a host opens the stream are
  normal.
- **Frames 0 with errors, sensor streaming:** the link never synchronised;
  suspect `UVC_CARRIER`. The console says so.
- **A failure** names the step, the BSP status (`neuron6_errno.h`: -5 driver
  failure, -7 no answer at the sensor's address) and the sensor's state:

```
[cam] VD66GY failed at camera init, status <status>, after <ms> ms; the sensor was <state> at start and is <state> now
[cam] the sensor was already running when this program started, and its driver starts it only from power-up: switch the board off and on
```

The second line is the one case the example cannot recover from: ST's driver
starts the sensor only from its power-up state, and neither carrier has a
reset line to it. On the Neuro N6 the board's reset reaches the sensor, so a
reset or re-flash with the power on starts it again.

## Staying reachable

The watchdog is armed first, with its early warning and its reset aimed at the
bootloader; `b` on the console works at any time; and if no host has brought
the device up within `UVC_HOST_TIMEOUT_MS` (30 s) the board returns to the
bootloader.

## Building

Import `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build `Debug`, or:

```bash
bash harness/examples/cubebuild.sh Camera_UvcStream_VD66GY_USB
```

`python harness/n6flash.py` with
`STM32CubeIDE/Appli/Debug/Camera_UvcStream_VD66GY_USB_Appli_Trusted.bin`
writes the application slot; `python harness/examples/runex.py
Camera_UvcStream_VD66GY_USB` uploads it, watches the console and sends `b`.

Tested on a thermal carrier: 1206 frames captured by ffmpeg in 20 s, every one
decoded; every mode above runs at its top rate on a room scene.
