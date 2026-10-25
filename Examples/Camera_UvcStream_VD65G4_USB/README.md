# The VD65G4 as a USB camera

Streams the VD65G4 global-shutter camera as a standard USB video device:
Motion-JPEG, 640 x 480, 60 fps, on an isochronous endpoint, with no driver
needed on the host.

```
VD65G4 --CSI-2, RAW8, one lane--> DCMIPP pipe 1 --band in AXISRAM--> VENC --JPEG--> UVC --> host
                                  black level, colour,
                                  white balance, colour
                                  correction, gamma, YUV
```

This is [Camera_UvcStream_VD66GY_USB](../Camera_UvcStream_VD66GY_USB/) with
the other global-shutter sensor; transport, capture loop, encoder pacing and
console are the same. What the VD65G4 changes:

- **One data lane**, at the same 804 Mbps.
- **The opposite carrier mapping:** `UVC_CARRIER` is
  `BSP_CAMERA_CARRIER_STANDARD` (the VD66GY needs `INVERTED` on the same
  carrier, which would put this sensor's single lane on the unrouted pair). A
  wrong setting is silent: no frames with the sensor streaming means change
  this first.
- **Configuration constants** in `neuron6_conf.h`:
  `BSP_CAMERA_VD65G4_CLOCK_LANE_SWAP`, `BSP_CAMERA_VD65G4_DATA_LANE_SWAP`,
  `BSP_CAMERA_VD65G4_I2C_ADDRESS`, `BSP_CAMERA_VD65G4_EXT_CLOCK_HZ` and
  `BSP_CAMERA_VD65G4_LANE_RATE_MBPS`. All five default to the VD66GY module's
  values (the two share a pinout); confirm them at a first bring-up.
- **The rate is the sensor's:** one rate per mode, 60 for VGA. `main.c`
  refuses to run if the module's rate differs from what the USB descriptors
  promise.
- **The colours are borrowed:** ST's daylight tuning for the VD66GY module,
  so the picture is close rather than right. Tune the white balance and colour
  matrix in `isp_config()`.
- **A monochrome part** (read from the sensor) gets the black level and gamma
  only, no white balance or colour matrix. The console says which part is
  fitted.
- **XSHUTDOWN needs a carrier pull-up below about 9 kOhm.** The part holds the
  pin low through roughly 20 kOhm of its own until it boots; a weaker pull-up
  leaves it in shutdown, answering on no bus.

## Turning it on

`neuron6_conf.h` sets the video class and the picture, which the USB
descriptors and `main.c` both read:

```c
#define BSP_USB_UVC                     1U
#define UVC_FRAME_WIDTH                 640
#define UVC_FRAME_HEIGHT                480
#define UVC_FRAME_RATE                  60
```

The `Appli` project adds to the OV5640 example's: `neuron6_camera_vd65g4.c`
and `.h` (linked from the repository root), `Components/vd55g1` on the include
path, and `NDEBUG` (ST's drivers assert on reachable error paths, and a failed
assertion is a silent loop).

## Console

Every second the `[uvc]` line, and every five seconds:

```
[cam] VD65G4 640x480 at 60 fps, straight lanes: started in <ms> ms from <state>, now <state>, <colour|monochrome> part, filter <order>, <n> frames in the last second, <n> camera errors since start (codes <hex>)
```

Frames near 60 with a steady error count: the picture reaches the chip. No
frames with the sensor streaming: the lane mapping. A failure names the step,
the `neuron6_errno.h` status and the sensor's state before and after; `-7`
means nothing answered at the sensor's address (address or wiring).

## Building

```bash
bash harness/examples/cubebuild.sh Camera_UvcStream_VD65G4_USB
```

or import `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build `Debug`. Write
`STM32CubeIDE/Appli/Debug/Camera_UvcStream_VD65G4_USB_Appli_Trusted.bin` to
the application slot with `python harness/n6flash.py`.

Tested: 640x480 at 60 fps sustained with the straight lane mapping; the
sensor identifies as `0x53354733` and streams 37 ms after reaching its ready
state.
