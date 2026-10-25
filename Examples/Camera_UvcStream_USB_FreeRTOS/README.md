# VGA 60 fps as a USB camera, under FreeRTOS

The camera of [Camera_UvcStream_USB](../Camera_UvcStream_USB/) (Motion-JPEG,
640 x 480, 60 fps on an isochronous endpoint, no driver needed) running under
FreeRTOS, with a USB task and a capture task. The picture path, the endpoint
and the encoder handshake are as that example's README describes; this one
covers what changes under the kernel.

## The USB stack

The BSP's own USB configuration is for a main loop (no DMA, the stack
serviced from inside the encode). This example uses the FreeRTOS
configuration instead, switched from the vendor interface to video:

| file | what it is |
|---|---|
| `Utilities/usb/tusb_config.h` | FreeRTOS, high speed, DMA with the D-cache cleaned and invalidated, 32-byte buffer alignment, the video class |
| `Utilities/usb/usb_descriptors.c` | device, console and strings, and the video function: one MJPEG format, 640 x 480, 60 fps, a 1023-byte isochronous endpoint. With no WinUSB interface, `bcdUSB` is 2.00 and there is no BOS descriptor |

TinyUSB is the repository's `Middlewares/tinyusb`. Its FreeRTOS and DWC2
changes take effect only in this DMA mode, and it carries one fix for the
video class, marked `NEURO N6 LOCAL PATCH (UVC)`: after an endpoint 0 data
stage, the cache is invalidated over the bytes received, not from the
driver's advanced buffer pointer. The wrong range would leave the first line
of a 48-byte UVC probe or commit stale and drop the unflushed cache line past
the 64-byte control buffer, where the driver's endpoint state lives.

## Building

The project under `STM32CubeIDE/` is the FreeRTOS flavour of the vendor
streaming example, set up as `Utilities/freertos/README.md` describes, with
the two files above. Import `STM32CubeIDE/Appli` into STM32CubeIDE 2.2.0 and
build `Debug`, or:

```bash
bash harness/examples/cubebuild.sh Camera_UvcStream_USB_FreeRTOS
```

`neuron6_conf.h` sets `BSP_USE_FREERTOS` and leaves `BSP_USB_UVC` alone: the
video function comes from the files above.

## Two tasks

| task | priority | what it does |
|---|---|---|
| `usb` | idle + 3 | drains the stack, hands the next frame to the video class, returns sent buffers, serves the console. While a frame is in flight it sleeps on a notification from the USB interrupt; otherwise it runs once a millisecond |
| `capture` | idle + 3, raised to + 4 | encodes into whichever buffer the host is not holding and queues the frame (two deep); sleeps while the hardware encodes |

Boot order: the watchdog, USB with a 500 ms enumeration pump before the
scheduler, the camera and encoder, then the queue, tasks and scheduler.

## Keeping the encoder in step

The capture loop is the bare-metal example's. Under the kernel, the capture
task also holds its raised priority from posting one frame through arming the
next, so no other task runs in the gap between frames, and drops back while it
waits for the host. It waits for the camera's frame end on a notification
from the frame event (at most five frame periods, never under 150 ms, then
the camera is restarted), and a held-back capture also waits for the first
line event of a frame that has started.

Two BSP behaviours keep repeated resynchronisations working:

- **The HAL tick is read under a lock.** `HAL_GetTick` extends the kernel tick
  from the cycle counter against a shared anchor, read and moved with
  interrupts off; otherwise a preempted task can read seconds ahead and a
  timeout expires at once.
- **A failed camera stop is retried:** if the HAL leaves the pipe busy,
  `BSP_CAMERA_Stop` stops through the HAL again next time instead of counting
  the pipe stopped.

## Staying reachable

As in the other examples: the watchdog first, its early warning and its reset
sent to the bootloader; `b` on the console at any time (it stops the pipe and
lets the encoder go idle first); and a return to the bootloader if no host has
brought the device up within `UVC_HOST_TIMEOUT_MS`.

Tested: the host sees `NeuroN6 Camera`; 1206 frames in 20 s, 60 encoded and
sent a second with none torn; twenty fresh opens; seven forced
resynchronisations a second for 18 s, recovered every time; every mode
decoded.
