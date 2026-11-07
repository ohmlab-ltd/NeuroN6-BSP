# IMX335 pictures to an SD card

Takes full-resolution pictures (2592 x 1944) with ST's B-CAMS-IMX camera
(Sony IMX335) and saves each as a numbered JPEG, quality 5, on the adapter's
SD card (FAT32 or exFAT). With each picture goes the module's VL53L5CX
time-of-flight grid: 8 x 8 distances in millimetres, as a CSV file.

```
IMX335 --CSI-2, RAW10--> DCMIPP pipe 1 --band in AXISRAM--> VENC --JPEG in AXISRAM--> FatFs --> SDMMC1 --> card
```

A picture is ready within one frame (0 to 48 ms) of the request and written in
about 30 ms.

The camera path is [Camera_UvcStream_IMX335_USB](../Camera_UvcStream_IMX335_USB/)'s
at full size: the same tuning, automatic white balance, exposure loop and
encoder pacing.

## What it needs

- A Neuro N6 with Ohm Lab's B-CAMS-IMX adapter: ST's B-CAMS-IMX module on
  its flex connector, and an SD card, formatted FAT32 or exFAT, in its slot.
- `neuron6_conf.h` names the adapter's EN_MODULE (PA7) and XCLR (PB11) pins,
  the time-of-flight sensor's LPn (PA9) and INT (PA11), and its card detect
  (PD9, low with a card in). Another carrier names its own there.
- `SD_CARRIER` in `main.c` matches the adapter's straight lanes (BSPTest's
  `bcamslink` says which mapping captures).

## Using it

- Press USER, or send `c` on the console, to take a picture.
  `SD_INTERVAL_MS` in `main.c` also takes one every so many milliseconds.
- Files are `IMG_00001.JPG` onwards in the card's root; numbering continues
  after the highest already there.
- Beside each, `IMG_nnnnn.CSV` holds the time-of-flight result: 8 lines of 8
  distances in millimetres, 0 where a zone has no valid range: the newest
  result when the picture is taken (the sensor ranges 10 times a second). The
  zones are in the order ST's driver gives them.
- Send `b` to return to the bootloader.

## What you should see

```
[sd] card mounted: FAT32, 32 KB clusters, 14911 MB, bus 33333333 Hz; next picture IMG_00001.JPG
[cam] 2592x1944, 21 frames and 21 encoded in the last second, 1 torn and 1 camera errors since start; exposure <us> us, gain <dB> dB, average <n>; white balance <K> K; card mounted
[sd] IMG_00001.JPG <bytes> bytes, quality 5, ready <ms> ms after the request, write <ms> ms (<KB/s> KB/s)
[sd] IMG_00001.CSV 8 x 8 distances, nearest <mm> mm
[tof] <n> results since start; nearest <mm> mm
```

- **Time of flight:** `Init` loads the sensor's firmware at start, about 3 s.
  A result is read inside an encode, early in the frame, so the 11 ms read ends
  before the frame does. `[tof] VL53L5CX not running (status n)` means it did
  not start; pictures are then saved without a CSV.

- **No card, or no FAT volume:** `[sd] no card (FatFs 3)` or `[sd] no FAT32 or exFAT volume on the card (FatFs 13)`, then `trying again in 2 s`, every 2 s until one is found.
- **Card full:** `[sd] card full: IMG_nnnnn.JPG not saved`; the partial file is deleted.
- **Write failed:** `[sd] write failed (FatFs n): ... remounting`; the next attempt remounts the card.
- **Torn or failed frame:** the request passes to the next frame, up to four, then `[cam] no picture`.
- **One torn frame at start** is normal: the first frame after the line
  events are armed.

## How it works

- **Sensor:** 21 fps with lines 142% of ST's length (`SD_FRAME_RATE`,
  `SD_LINE_PERCENT`): a 16-line row every 328 us, just slower than the
  316 us the encoder needs at full width. The fastest full-size rate.
- **Encoder:** runs on every frame while a card is mounted, so a picture is
  the frame being read out when it is asked for, not the next one.
- **JPEG buffer:** 832 KB of on-chip RAM, which the card's DMA reads directly.
  The on-chip RAM left after it is the heap the encoder allocates from at
  start: a larger buffer leaves the encoder too little.
  A picture too big for it (an encoder overflow) switches encoding to a 6 MB
  PSRAM buffer until a picture is 416 KB or less; writes from PSRAM go through
  `neuron6_sd`'s bounce buffer at about 18 MB/s of copying, about 45 ms a
  picture.
- **Card:** `neuron6_sd` holds the radio powered down (the card shares
  SDMMC1 with it), switches the card to high-speed mode and runs the bus at
  33.3 MHz, 4 bits wide. Every wait feeds the watchdog and serves the console.

## Limits

- FatFs writes at most one cluster per card command: a card formatted with
  larger clusters writes faster.
- The radio cannot run while the card is in use.
- Files are dated 2026-01-01 (no clock).
- The module's IMU answers at the board's own IMU address (0xD4), so with the
  module fitted the board's motion readings are unreliable. See
  [Camera_UvcStream_IMX335_USB's limits](../Camera_UvcStream_IMX335_USB/README.md#limits).

## Building

```bash
bash harness/examples/cubebuild.sh Camera_JpegToSd_IMX335
```

Compared with the UVC example, the `Appli` project adds `neuron6_sd.c`, FatFs
(`Middlewares/FatFs/source`) and its disk layer
(`Middlewares/NeuroN6/FatFsSd`), with `HAL_SD_MODULE_ENABLED` in
`stm32n6xx_hal_conf.h`. `python harness/n6flash.py` with
`STM32CubeIDE/Appli/Debug/Camera_JpegToSd_IMX335_Appli_Trusted.bin` writes the
application slot.
