# Global-shutter and thermal cameras to Neuro Studio, under FreeRTOS

Streams everything on the thermal carrier to Neuro Studio over the USB vendor
interface at once: the VD66GY global-shutter camera as 640 x 480 JPEG at
60 fps, the FLIR Lepton 3.1R at about 9 fps with a temperature for every
pixel, and the accelerometer, gyroscope and magnetometer, ten readings a
second, carried on the JPEG frames.

```
VD66GY --CSI-2--> DCMIPP: black level, demosaic, white balance,  --> band --> encoder --> JPEG --+
                          colour correction, gamma                                               |
Lepton --VoSPI, SPI1--> GPDMA1 --> 64 KB ring --> thermal task --> META_THERMAL_FRAME -----------+--> usb task --> vendor interface
                                                                                                 |
LSM6DSL, MMC5603NJ --I2C1--> sensors task --> META_IMU_FRAME + META_MAG_FRAME, on each JPEG -----+
```

It combines [Camera_JpegStream_USB](../Camera_JpegStream_USB/) (FreeRTOS
flavour: encoder, camera handshake, vendor link),
[Camera_UvcStream_VD66GY_USB](../Camera_UvcStream_VD66GY_USB/) (VD66GY
bring-up and tuning) and [Thermal_Stream_USB](../Thermal_Stream_USB/)
(thermal camera and record). Their READMEs describe each part.

## Tasks

| task | priority | what it does |
|---|---|---|
| `encode` | idle + 4 | camera and encoder; each frame coded into whichever of two buffers USB is not sending |
| `usb` | idle + 3 | USB, console and vendor link: sends a ready thermal frame, else the next JPEG with the latest motion readings, else a status line. Only this task prints |
| `thermal` | idle + 3 | woken by the Lepton's DMA at each half ring and every 100 ms; parses, and copies a complete frame into the thermal record if the link has sent the last |
| `sensors` | idle + 1 | the motion sensors every 100 ms, their temperatures every 10 s |

The encode task is highest so the re-arm at each frame end runs in one piece;
its wake comes through the kernel's timer task (idle + 2), which the sensors
task sits below. A late re-arm costs that frame (skipped, or dropped as torn).
Half the thermal ring takes at least 13 ms to fill, so the thermal task has
time; a thermal frame that completes while the last is still going out is
counted and skipped.

## The sensor bus

The Lepton's commands, the VD66GY's control and both motion sensors share
I2C1, and the Lepton does not finish booting while anything uses that bus.
Start-up therefore runs before the scheduler, in this order: motion sensors,
thermal camera, VD66GY and encoder, USB. Afterwards the sensors and thermal
tasks share a mutex; when the Lepton is power-cycled for recovery the motion
readings pause for about two seconds while it boots. Reading video never
touches the bus.

## On the wire

Every record is one DNV0 vendor frame:

| frame | flags | metadata | payload |
|---|---|---|---|
| picture | bit 0 | `META_IMU_FRAME` (0x1B): seven floats, acceleration in g, rotation in degrees a second, temperature in °C; `META_MAG_FRAME` (0x1A): four floats, field in microtesla, temperature | the JPEG |
| thermal | 0 | one `META_THERMAL_FRAME` (0x30), as Thermal_Stream_USB sends it: 160 x 120, format 0, flag bit 0 (pixels in hundredths of a kelvin), then the pixels | none |
| status line | 0 | `META_DEBUG_TEXT` (0x17) | none |

Until the first thermal frame, an empty thermal record goes out every two
seconds to announce the stream. A sensor that is not found is left out of the
metadata. If the VD66GY does not start, the sensor records go out on their
own, ten times a second.

## Settings

At the top of `main.c`:

| setting | as shipped | meaning |
|---|---|---|
| `CAM_RESOLUTION`, `CAM_FRAME_RATE` | VGA, 60 | the VD66GY's mode (up to 88 fps); the sensor is held to this rate |
| `CAM_CARRIER` | `BSP_CAMERA_CARRIER_INVERTED` | the thermal carrier crosses the camera's two data lanes. A wrong setting gives no error: the sensor streams and no frame arrives |
| `CAM_EXPOSURE_TARGET` | 25 | the sensor's exposure target, 0 to 100. It meters the raw picture, before white balance and gamma brighten it, so a middle target overexposes |
| `THERMAL_RESTART_MS` | 20 s | no thermal frame for this long, despite the thermal module's own restarts: the Lepton is power-cycled |
| `THERMAL_RETRY_MS` | 30 s | how often a Lepton that did not start is retried; the readings pause meanwhile |

The colours are ST's daylight tuning for this camera module.

## Console

Once a second, the frame, encode and send counters with the thermal centre
pixel's temperature, then acceleration (milli-g) and field (milligauss); every
ten seconds, the thermal link's health.

## Staying reachable

The watchdog (4 s, so the Lepton boot and VD66GY start-up before the scheduler
fit) runs from the first line of `main`, with its early warning and its reset
aimed at the bootloader; `b` on the console goes there at any time.

## Building

```bash
bash harness/examples/cubebuild.sh Camera_GlobalShutterThermal_USB_FreeRTOS
```

`Appli` is the JPEG stream's FreeRTOS project with the VD66GY, thermal and
motion-sensor modules linked in; `FSBL` is the boot loader. `neuron6_conf.h`
sets `BSP_USE_FREERTOS`.

## Status

Tested on a board with the thermal carrier for five minutes: 60.0 fps JPEG
and 8.7 thermal frames a second, with no gap in the frame ids. After a forced
thermal stall, thermal frames return 2.8 s after the power cycle while the
JPEGs continue. A reader that joins after another stopped part way through a
frame skips the rest of that frame first.
