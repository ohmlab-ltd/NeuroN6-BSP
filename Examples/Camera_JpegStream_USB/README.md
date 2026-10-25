# VGA 60 fps JPEG to Neuro Studio

Streams the OV5640 to Neuro Studio over the USB vendor interface, encoded by
the hardware video encoder. The processor never touches a pixel.

```
OV5640 --CSI-2--> DCMIPP pipe 1 --band in AXISRAM--> VENC --JPEG--> USB
                       |                              ^
                       +--------- line event ---------+
                            (venc_rdy, in silicon)
```

Raw VGA at 60 fps is 36.9 MB/s, about 85 % of what USB 2.0 high speed
delivers in practice, shared with the CDC console; as JPEG at quality 2 it is
one to four MB/s. Neuro Studio only displays frames with the JPEG flag set.

## The handshake

- **The band:** DCMIPP address wrapping returns pipe 1's write address to the
  top every 64 lines, so the camera writes an 80 KB band, not a 600 KB frame.
- **The pacing wire:** the DCMIPP's line event (every 32 lines) is wired in
  silicon to the encoder's `venc_rdy`, telling it how far it may read.
- **The ordering:** the encoder must be armed before the camera's first line
  event, so `main.c` never calls `BSP_CAMERA_Start`. It passes a function
  that does as `BSP_VENC_Config_t::SourceStart`, which the encoder calls from
  inside the first `BSP_VENC_EncodeBand` once armed.
- The line-event granularity must equal the band depth;
  `BSP_VENC_GetBandGeometry` computes both. There is no back-pressure: a
  camera that laps the band gives a torn frame, not an error.

## Building

The CubeIDE projects under `STM32CubeIDE/` share `Core/`:

- `Appli`: bare metal. The BSP, `Components/{ov5640,venc}`, `Utilities/usb`,
  TinyUSB, and the HAL with `stm32n6xx_ll_venc.c` (not in CubeMX's default
  selection).
- `Appli_FreeRTOS`: `Appli` plus what `Utilities/freertos/README.md` lists;
  `BSP_USE_FREERTOS=1` also enables the HAL's TIM driver in
  `stm32n6xx_hal_conf.h`.
- `FSBL`: the boot loader, as in every example.

Import them into STM32CubeIDE 2.2.0 and build `Debug`, or build all three with
`bash harness/examples/cubebuild.sh Camera_JpegStream_USB`. Upload with
`harness/n6flash.py`, which writes the application slot with the header the
boot loader expects (`docs/BOOT-CONTRACT.md`).

- Encoder include order matters: `VideoEncoder/inc` before
  `VideoEncoder/source/*` (`Components/venc/README.md`).
- `Utilities/usb/tusb_config.h` keeps TinyUSB's DMA and D-cache options off;
  an image built with a FreeRTOS DMA configuration and no operating system
  does not enumerate.
- Each encode is armed the moment the last returns, and the finished frame
  goes out from the encoder's poll hook while the next is encoded. Under
  FreeRTOS, an encode task runs above the USB task so the per-frame re-arm
  runs in one piece.

## What a default project lacks

- **A heap.** The encoder takes its instance and working buffers from
  `malloc` once, at `BSP_VENC_Init` (`BSP_VENC_HEAP_BYTES` is 72 KB). The
  linker's `_Min_Heap_Size` is only the least the link checks for; the heap
  grows from the end of `.bss` towards the stack. Starved, `BSP_VENC_Init`
  returns `BSP_ERROR_NO_INIT`, as it does for a geometry the encoder refuses.
- **An MPU region for the band.** The boot loader maps nothing. The band is
  written by the camera and read by the encoder, never by the processor, so
  it must be non-cacheable; no maintenance call substitutes. `mpu_config` in
  `main.c` sets it from linker symbols. Get it wrong and nothing faults: the
  picture is built from stale bytes.

## Staying reachable

- **Watchdog.** `BSP_WDG_InitEWI(2000, 300)` from the first line of `main`;
  its early warning calls `BSP_COM_RebootToBootloader`, so the reset lands in
  the bootloader, and `main` goes there at boot after `RCC_FLAG_IWDGRST`.
- **Breadcrumbs.** Backup registers 4 to 7 survive any reset short of a power
  cycle: last stage reached (`CRUMB_*` in `main.c`), encodes, poll-hook calls,
  fault code (layout in `Common/Src/stm32n6xx_it.c`). `Examples/BSPTest`'s
  `bkp` reads them.
- **Fault handlers** record their code and go to the bootloader. The startup
  file's `Default_Handler` branches to `Default_Handler_C`
  (`b Default_Handler_C` in place of `b Infinite_Loop`), which records `0xFB`
  plus the vector number; that line is the only change outside `Core/Src`.
- **Console.** `b` bootloader, `s` and `x` stream on and off, `?` a one-line
  report. The stream starts once the host mounts the vendor interface.

## On the wire

20-byte little-endian header, then metadata, then payload:

| Offset | Field | This example |
|---|---|---|
| 0 | `magic` | `0x30564E44`, `DNV0` on the wire |
| 4 | `frame_id` | increments per frame; the host spots drops with it |
| 8 | `meta_len` | 0 |
| 12 | `payload_len` | encoded JPEG length |
| 16 | `flags` | 1, bit 0 says the payload is JPEG |

The device enumerates as VID `0x1209` / PID `0x8008`: the vendor interface (0)
that Neuro Studio binds, and the CDC console (1). Windows binds WinUSB
automatically through the MS OS 2.0 descriptors in
`Utilities/usb/usb_descriptors.c`; a board previously bound under another
VID/PID needs its driver reassigned once.

## Reading the console

Once a second:

```
[n6] enc=60 fail=0 torn=0 resync=0 sent=60 drop=0 last=5427B gap=2/4 err=0 lines=5491 init=0
```

- `gap`: the furthest the camera got ahead of the encoder, in macroblock rows,
  against the band's capacity (twice its depth).
- `torn`: an encode armed after the camera lapped the band. **A torn frame is
  never sent.** Two in a row and the loop restarts camera and encoder
  (`resync`).
- A host that pauses costs whole skipped frames: the next encode waits for
  the frame end and arms in vertical blanking (at most five frame periods,
  never under 150 ms).
- `err`: the encoder's last failure. `-15` is a hardware reset, nearly always
  the encoder and camera losing sync (erratum ES0620); after eight in a row
  the loop calls `BSP_VENC_Resync`, as re-arming alone does not recover it.
- `enc` should sit near the mode's frame rate; lower with `fail` at zero
  means an encode is longer than a frame period (a larger `last`).
- `drop` climbing or `enc` at zero: the host has the interface open but is not
  reading (normal with no capture tool). An untouched frame gives way after
  500 ms; a partly sent frame waits for the host.
- `n6stream.py` counts `joining` separately: a reader attaching mid-stream may
  skip the rest of a frame. That is not a fault within the first 1.5 s.

## Changing resolution

Set `STREAM_RESOLUTION` and `STREAM_BAND_BYTES` at the top of `main.c`. The
picture size is read back from the sensor, and `stream_init` refuses a band
too small for the mode.

| mode | | fps | band |
|---|---|---|---|
| `BSP_CAMERA_R320x240` | QVGA | 60 | 81920 |
| `BSP_CAMERA_R640x480` | VGA (default) | 60 | 81920 |
| `BSP_CAMERA_R800x480` | WVGA | 60 | 102400 |
| `BSP_CAMERA_R1024x768` | XGA | 30 | 262144 |
| `BSP_CAMERA_R1280x720` | 720p | 60 | 327680 |
| `BSP_CAMERA_R1920x1080` | 1080p | 30 | 491520 |
| `BSP_CAMERA_R2592x1944` | QSXGA | 15 | 331776 |

- Above WVGA the CSI link needs the D-PHY patch in `docs/HAL-PATCHES.md`.
  Read the warning on `BSP_CAMERA_R1024x768` before choosing XGA.
- Heights that are not a multiple of 8 lose up to seven rows: the encoder's
  macroblock is 16 by 8, and an incomplete final row stalls it.
  `BSP_VENC_GetBandGeometry` floors the height and reports the coded height.

Tested on a Neuro N6 with an OV5640: 54008 frames in fifteen minutes at
60.0 fps with no malformed frame and no gap in the frame counter; all seven
modes stream at the rates shown.
