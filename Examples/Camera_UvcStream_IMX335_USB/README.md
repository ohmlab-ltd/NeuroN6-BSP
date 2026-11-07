# The IMX335 as a USB camera

Streams ST's B-CAMS-IMX camera (Sony IMX335, 5 MP) as a standard USB video
device: Motion-JPEG at quality 5, the full 2592 x 1944 at 21 fps (the most
the encoder sustains at that size), on an isochronous endpoint, with no
driver needed on the host. The module's VL53L5CX time-of-flight sensor
ranges alongside, and the console reports what it sees.

```
IMX335 --CSI-2, RAW10, 2 x 1600 Mbps--> DCMIPP pipe 1 --band in AXISRAM--> VENC --JPEG--> UVC --> host
                                        black level, colour, white balance,
                                        gamma, crop, reduce (smaller sizes),
                                        YUV; statistics for exposure and
                                        white balance
```

The transport (descriptors, capture loop, encoder pacing, recovery) is
[Camera_UvcStream_VD66GY_USB](../Camera_UvcStream_VD66GY_USB/)'s, with three JPEG buffers in place of two.

## What it needs

- A Neuro N6 with Ohm Lab's B-CAMS-IMX adapter, and ST's B-CAMS-IMX module
  on the adapter's flex connector.
- `neuron6_conf.h` names the adapter's EN_MODULE (PA7) and XCLR (PB11)
  pins, and the time-of-flight sensor's LPn (PA9) and INT (PA11). Another
  carrier names its own there (`BSP_CAMERA_IMX335_EN_*`, `_XCLR_*`,
  `BSP_RANGING_VL53L5CX_LPN_*`, `_INT_*`).
- `UVC_CARRIER` at the top of `main.c` matches the adapter's straight lanes.
  A wrong setting gives no error, only no frames; BSPTest's `bcamslink` says
  which mapping captures.

## The sensor

```c
BSP_CAMERA_IMX335_Register();               /* power up through EN_MODULE, XCLR */
BSP_CAMERA_SetCarrier(UVC_CARRIER);
BSP_CAMERA_IMX335_SetTiming(UVC_FRAME_RATE, UVC_LINE_PERCENT);
BSP_CAMERA_Init(0, BSP_CAMERA_IMX335_R2592x1944, BSP_CAMERA_PF_RAW10);
```

- **One mode:** 2592 x 1944 RAW10, RGGB. For smaller pictures pipe 1 crops
  it to the picture's shape (16:9 sizes lose the top and bottom) and reduces
  it.
- **Colour is made on the chip** (`isp_config()`), with automatic white
  balance (`neuron6_camera_awb`, `awb_step()`).

| stage | setting |
|---|---|
| black level | 12 of 255 (the sensor's 50 of 1023) |
| colour reconstruction | RGGB, ST's strengths |
| white balance | automatic, between ST's tungsten (2810 K), fluorescent (4015 K) and daylight (6650 K) gains, times 1.049 on all three |
| colour correction | off |
| gamma | on |

- **White balance:** the pipe averages red every frame, and green and blue in
  turn (after the gains); the loop makes the average grey, held to the line
  through ST's three lights. It steps every six frames.
- **Highlights clip to white:** the 1.049 gain (255 over 255 minus the black
  level) brings a clipped pixel to 255 on all three components, not tinted.
- **No colour matrix:** the pipe has no noise reduction, and any matrix
  amplifies the sensor's colour noise, which quality-5 JPEG breaks into purple
  and green speckle at its block edges. Without it colours are a little less
  saturated.

- **Exposure is an Ohm Lab loop** (`neuron6_camera_ae`): the pipe averages
  the picture's luminance every frame, and `exposure_step()` sets exposure
  time first, then gain up to 36 dB, aiming at 50 of 255 before gamma.
  Exposures over 10 ms are whole 10 ms steps (50 Hz lighting). A change shows
  three frames later, so the loop waits that long between steps.
- **Arming:** every encode counts the frame from its start
  (`BSP_CAMERA_VsyncEventCallback`), as in the VD66GY example.

## Other sizes

Set `UVC_FRAME_WIDTH`, `_HEIGHT`, `_RATE` and `UVC_LINE_PERCENT` in
`neuron6_conf.h`. Width must be a multiple of 16, height of 8. The band and
JPEG buffers in `main.c` fit every size.

| picture | rate | line percent | a coded row arrives every | encoder needs |
|---|---|---|---|---|
| 640 x 480 | 30 | 100 | 940 us | 78 us |
| 1280 x 960 | 30 | 100 | 470 us | 156 us |
| 1920 x 1080 (cropped) | 30 | 100 | 313 us | 234 us |
| 2592 x 1944 | 21 | 142 | 328 us (measured) | 316 us |

- The sensor sends a line every 14.5 us at 100% (measured by BSPTest's
  `bcamspipe`), proportionally longer at longer lines. Frame rate x line
  percent may not exceed 3000 (the frame length, VMAX, stays at 4500 or more).
- The encoder codes a 16-line row W pixels wide in about 0.122 x W us, about
  131 Mpixel/s (BSPTest's `vencspeed`), whatever the format, content or
  memory. A row that arrives faster overflows the band.
- 2592 x 1944: 22 fps would allow only 136% lines, a row every 315 us, too
  fast. 21 fps at 142% is the fastest full-size stream. 30 fps would need
  151 Mpixel/s; the encoder measures 131 at its 400 MHz clock, against the
  reference manual's 300.
- The isochronous endpoint carries about 7 to 8 MB/s: 21 fps of 2592 x 1944
  needs frames under about 350 KB. Larger frames lower the rate in step
  (470 KB: 16 to 17 fps). Three JPEG buffers keep the wire busy.

## Time of flight

```c
BSP_RANGING_VL53L5CX_Init();                                     /* after the camera, which powers the module */
BSP_RANGING_VL53L5CX_Start(BSP_RANGING_VL53L5CX_ZONES_8X8, 1);   /* UVC_TOF_ZONES, UVC_TOF_RATE_HZ */
```

- **Start-up:** `Init` loads the sensor's firmware, about 3 s;
  `BSP_RANGING_VL53L5CX_WaitHook` keeps USB and the watchdog served meanwhile.
- **Reading:** a result is read inside an encode, while at least half the
  frame's rows are still to come (`BSP_VENC_ReadPointerHook`), so the 11 ms
  read ends before the frame does. TOF_INT says when a result has landed, so
  until then the check costs no bus time.
- **Cost:** the read stops USB for its 11 ms: about 1% of the stream's time at
  one result a second.

## Console

Every second the `[uvc]` line (frames encoded, sent, torn, failed) and a
`[tof]` line; every five seconds two `[cam]` lines and the time-of-flight grid:

```
[cam] IMX335 2592x1944 at 21 fps, quality 5, straight lanes: started in <ms> ms, now streaming, 21 frames in the last second, <n> camera errors since start (codes 0x<code>)
[cam] exposure <us> us, gain <dB> dB, average <n> (target 50); white balance <K> K (red x<gain>, blue x<gain>)
[tof] 64 of 64 zones in range; nearest <mm> mm (row <r>, column <c>); centre <mm> mm
[tof] row 0:  1715  1707  1698  1660  1671  1687   340   283
...
```

- The grid is in millimetres, `-` where a zone has no valid range, in the
  order ST's driver gives the zones.

- **Frames 0, sensor streaming:** the link never synchronised; suspect
  `UVC_CARRIER`. The console says so.
- **"no answer" at start:** the module has no supply or clock, or is held in
  reset: check EN_MODULE and XCLR.

## Staying reachable

The watchdog is armed first, with its early warning and its reset aimed at the
bootloader; `b` on the console works at any time; and if no host has brought
the device up within `UVC_HOST_TIMEOUT_MS` (30 s) the board returns to the
bootloader.

## Building

Import `STM32CubeIDE/` into STM32CubeIDE 2.2.0 and build `Debug`, or:

```bash
bash harness/examples/cubebuild.sh Camera_UvcStream_IMX335_USB
```

Compared with the VD66GY example, the `Appli` project adds
`neuron6_camera_imx335.c`, `neuron6_camera_ae.c`, `neuron6_camera_awb.c`,
`neuron6_ranging_vl53l5cx.c` and their headers (linked from the repository
root), and `Components/imx335` and `Components/vl53l5cx` on the include path. `python harness/n6flash.py` with
`STM32CubeIDE/Appli/Debug/Camera_UvcStream_IMX335_USB_Appli_Trusted.bin`
writes the application slot.

## Limits

- White balance follows lights near ST's three; a scene filled by one colour
  pulls it as far as a real light would, no further.
- In dim light some colour noise remains (no noise reduction in the pipe);
  more light, or a longer exposure at a lower frame rate, reduces it.
- A start on a sensor no reset reached can show one or two CSI D-PHY errors
  (code 0x8000) before the first frame; frames then run clean.
- The module's ISM330DLC IMU answers at the board's LSM6DSL address (0xD4):
  with the module fitted, the board's motion readings are unreliable, and the
  IMU needs its address changed in hardware.
