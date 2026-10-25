# VGA 60 fps as a USB camera

Streams the camera as a standard USB video device: Motion-JPEG, 640 x 480,
60 fps, on an isochronous endpoint. A host opens it like any webcam, with no
driver and no host tool of ours.

```
OV5640 --CSI-2--> DCMIPP pipe 1 --band in AXISRAM--> VENC --JPEG--> UVC --> host
                       |                              ^
                       +--------- line event ---------+
                            (venc_rdy, in silicon)
```

The picture path is [Camera_JpegStream_USB](../Camera_JpegStream_USB/)'s;
only the last step differs.

## What the host sees

| interface | what it is |
|---|---|
| 0, 1 | the camera: VideoControl and VideoStreaming, one interface association |
| 2, 3 | the USB console, where `b` goes, so the board stays reachable while the camera is in use |

```
> ffmpeg -f dshow -list_options true -i video="NeuroN6 Camera"
  vcodec=mjpeg  min s=640x480 fps=60.0002 max s=640x480 fps=60.0002
```

## Isochronous transfer

- Alternate setting 0 has no endpoint (the host has stopped watching);
  alternate 1 has a 1023-byte isochronous endpoint at one packet per
  microframe, 8 MB/s at high speed. 1023 bytes is valid at full speed too,
  where the image streams QVGA at 60 fps.
- Isochronous packets are not retried. `CFG_TUD_VIDEO_STREAMING_BULK` moves
  the stream to a bulk endpoint for a host that prefers retries.
- The application hands the video class whole frames,
  `tud_video_n_frame_xfer(0, 0, JpegBuf[slot], bytes);`, and is told when one
  has gone, `tud_video_frame_xfer_complete_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx)`.
- One packet leaves per call into the USB stack, so USB is serviced from
  inside the encode, through `BSP_VENC_ReadPointerHook`, which the encoder
  calls while it waits on the camera.

## Keeping the encoder in step

The camera runs continuously and the encoder is re-armed for every frame, in
the gap between frames. Armed later, it codes a picture that starts part way
down one frame and ends in the next: rolled, with a seam. So:

- **Two buffers, taken in turn:** frame N goes into `JpegBuf[N & 1]` while the
  host takes the other; the encoder only holds back when the host is two
  frames behind.
- **The camera's row count is anchored to its frame** in
  `BSP_CAMERA_FrameEventCallback`, and the gap to the encoder's read row is
  sampled on every line event. A lapped band is reported by
  `BSP_VENC_LastFrameTorn`, and that frame is never sent.
- **Two torn frames in a row, or eight failed encodes, resynchronise:** the
  pipe stops, the encoder is reset, and the next capture starts the camera
  after the encoder is parked.
- **A held-back encode is armed only between frames.** It waits for the frame
  to end (at most five frame periods, never under 150 ms; a camera stopped
  mid-frame is restarted), and also while a frame has started
  (`BSP_CAMERA_VsyncEventCallback`) but no line has come, since the encoder
  takes about 0.43 ms to arm. An encode that follows another straight away
  does not wait.
- A picture bigger than the endpoint can carry at the frame rate costs whole
  skipped frames, not torn ones.

## Which way up

The OV5640 module reads out upside down on the standard carrier, and the BSP
flips it there by default; on the display carrier it is left as it is.
`BSP_CAMERA_DEFAULT_MIRRORFLIP` in `neuron6_conf.h` overrides that;
`BSP_CAMERA_SetMirrorFlip` changes it at run time.

## Turning it on

`neuron6_conf.h` in this example adds:

```c
#define BSP_USB_UVC                     1U
```

`Utilities/usb/tusb_config.h` turns that into the video class, and
`Utilities/usb/usb_descriptors.c` builds the descriptors. `UVC_FRAME_WIDTH`,
`UVC_FRAME_HEIGHT` and `UVC_FRAME_RATE` there and `UVC_RESOLUTION` in
`main.c` describe the same picture and change together. The vendor interface
is not built in this configuration.

Above VGA, `UVC_BAND_BYTES` and `JPEG_BUF_BYTES` in `main.c` change too: the
band must hold the mode's (`video_init` refuses a smaller one and the camera
stays off), and the JPEG buffer its largest frame.

| mode | fps | band | JPEG buffer |
|---|---|---|---|
| QVGA | 60 | 81920 | 256 KB |
| VGA | 60 | 81920 | 256 KB |
| WVGA | 60 | 102400 | 256 KB |
| XGA | 30 | 262144 | 512 KB |
| 720p | 60 | 327680 | 512 KB |
| 1080p | 30 | 491520 | 1.5 MB |
| QSXGA | 15 | 331776 | 1.5 MB |

The 1.5 MB buffers hold the sensor's random-data test pattern, the largest a
picture can code to.

## Staying reachable

The watchdog is armed first, with its early warning and its reset aimed at
the bootloader, and `b` on the console works at any time. An image no host
enumerates (a malformed descriptor looks like this) has no console, so if no
host has brought the device up within `UVC_HOST_TIMEOUT_MS` of boot (30 s by
default) the board returns to the bootloader. Once a host has been seen it no
longer applies; 0 disables it.

Tested: the host sees `NeuroN6 Camera` (Camera class) and captured 601 frames
in 10.03 s at 60 fps with no dropped or damaged frame; every mode in the table
ran at the sensor's rate.
