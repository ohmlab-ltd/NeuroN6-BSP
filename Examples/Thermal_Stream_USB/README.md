# Thermal camera to Neuro Studio

Streams the FLIR Lepton 3.1R on the thermal carrier to Neuro Studio over the
USB vendor interface, about nine frames a second, with a temperature for
every pixel. Nothing is encoded or mapped to a palette: each pixel arrives as
measured, in hundredths of a kelvin.

```
Lepton --VoSPI, SPI1, up to 20 MHz--> GPDMA1 ch 1 --> 64 KB ring in AXISRAM
                                                         |
                    BSP_THERMAL_GetFrame <---------------+   (parsed as it lands)
                            |
                            +--> META_THERMAL_FRAME record --> DNV0 frame, flags 0 --> USB
```

## On the wire

Every frame is one DNV0 vendor frame with no payload and `flags` 0, whose
metadata is a single `META_THERMAL_FRAME` (0x30) record:

| bytes | field |
|---|---|
| 1 | type, 0x30 |
| 2 | length, little-endian: 8 + 38,400 |
| 2, 2 | width 160, height 120 |
| 1 | format 0: 16-bit pixels, little-endian |
| 1 | flags, bit 0 set: the pixels are temperatures |
| 2 | reserved |
| 38,400 | the pixels, row-major |

Until the first frame, the record goes out every two seconds with no pixels,
to announce the stream. Status lines go to the console and, as
`META_DEBUG_TEXT` (0x17) records, to Neuro Studio's terminal.

## The capture

- The video arrives on SPI1 into a 64,288-byte ring (the largest one
  transfer can cover). The loop parses whatever has landed on every pass,
  `BSP_THERMAL_GetFrame` with a 1 ms budget; half the ring takes at least
  13 ms to fill. A frame that arrives while the last is still going out is
  counted and skipped.
- The camera samples its chip select (high) as it leaves reset. Its command
  interface is used only before the video starts: it stops answering on the
  sensor bus while it streams, and it does not finish booting while anything
  uses that bus, so it is left alone for two seconds after its reset.
- The carrier mounts the camera upside down, so every frame is turned half a
  turn.
- The thermal module restarts a stalled or malformed stream (as happens after
  boot and after the first flat-field correction) by holding chip select high
  with no clock for 300 ms. No frame for twenty seconds: the camera is
  power-cycled. If it does not start, the example says why and retries every
  ten seconds while servicing USB.
- The camera's power line, PD3, is the BLE flow control on boards without
  this carrier. Nothing here uses BLE.

## Building

```bash
bash harness/examples/cubebuild.sh Thermal_Stream_USB     # Appli and FSBL, from clean, headless
```

`Appli` is the application, `FSBL` the boot loader. The project is the JPEG
stream's, with the thermal module in place of the camera and the encoder.

## Status

Tested on a board with a Lepton 3.1R on the thermal carrier: 8.8 frames a
second, none dropped with a reader attached. Until the camera's first
flat-field correction, a few seconds in, its readings run about 8 C high.

The console line once a second gives the frames arrived, sent, met the link
busy and dropped, and the centre pixel's temperature; every ten seconds, the
video link's clock, frames and segments parsed, resynchronisations, restarts,
overruns and any error.
