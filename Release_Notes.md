# Release notes

## v0.2.0 (7 November 2026)

Adds Ohm Lab's B-CAMS-IMX adapter, with a flex connector for ST's B-CAMS-IMX
camera module and an SD card slot, and the VD65G4 on the NeuroVision ST cam
carrier. 37 examples.

### New

- **IMX335 (B-CAMS-IMX):** `neuron6_camera_imx335`, RAW10 at 1600 Mbps a lane, crop and resize on the main pipe, and auto-exposure and auto white balance from the pipe's statistics (`neuron6_camera_ae`, `neuron6_camera_awb`).
- **Time of flight (B-CAMS-IMX):** `neuron6_ranging_vl53l5cx`, the module's VL53L5CX through ST's ultra lite driver (BSD-3-Clause): 4 x 4 zones at up to 60 Hz, 8 x 8 at up to 15 Hz. TOF_INT (PA11) is held on its EXTI line, so waiting for a result costs no bus time.
- **SD card:** `neuron6_sd` on SDMMC1, sharing the radio's bus (one at a time), in high-speed mode at 33.3 MHz, about 10 MB/s. FatFs R0.16 (FAT and exFAT) runs on it.
- **Examples:** `Camera_UvcStream_IMX335_USB` (2592x1944 at 21 fps, quality 5, with the time-of-flight sensor's nearest target on the console) and `Camera_JpegToSd_IMX335` (2592x1944 JPEGs to the card, each with its 8 x 8 time-of-flight grid as a CSV file).
- **BSPTest:** `i2cscan`, `bcams`, `bcamstof`, `bcamslink`, `bcamspipe`, `bcamspins`, `sd`, `sdfat` and `vencspeed`.
- **Flash loader:** `Utilities/ExternalLoader/NeuroN6_MX25UM25645G.stldr` reads and writes the board's flash from STM32CubeProgrammer over an ST-LINK (development boot, access port 1).

### Changed

- **Camera clocks within RM0486's maximums:** the D-PHY configuration clock is 20 MHz from the crystal (was 26.7 MHz from the HSI) and the pixel pipes 300 MHz (was 400 MHz), with dividers computed from the bootloader's PLLs.
- **HAL fix 3:** one CSI-2 data lane on physical lane 1 ([docs/HAL-PATCHES.md](docs/HAL-PATCHES.md)). It lets the VD65G4 stream on the NeuroVision ST cam carrier.
- **One lane setting per carrier, for both ST cam sensors:** `BSP_CAMERA_CARRIER_INVERTED` on the NeuroVision ST cam carrier, which crosses the pair, and `STANDARD` on the thermal carrier. The VD66GY no longer swaps its lanes on the sensor, so code that sets its carrier must swap the setting it passes.
- `Camera_UvcStream_VD66GY_USB` and `Camera_UvcStream_VD65G4_USB` pick the carrier with one line, `UVC_CARRIER` (default ST cam). `Camera_GlobalShutterThermal_USB_FreeRTOS` takes `CAM_CARRIER` (default thermal).
- `BSP_GetVersion` reports 0.2.0.

### Verification

- The compile gate and host tests pass. All 76 projects build from a clean export: 72 by default, plus the 4 restricted ones.

### Known limitations

- **B-CAMS-IMX:** the module's IMU answers at the board's own IMU address, so with it fitted the board's motion readings are unreliable.
- **SD card:** the card and the radio cannot run together. Files are dated 2026-01-01 (no clock).

## v0.1.0 (25 October 2026)

First public release of the Neuro N6 board support package: drivers for every
peripheral on the board (STM32N657I0H3Q, Cortex-M55 with the Neural-ART NPU),
the first-stage bootloader's source, and 35 examples. Plain C,
`int32_t BSP_*` functions returning `neuron6_errno.h` codes, bare metal or
FreeRTOS.

### Core

- Board identity, RGB LED and user button.
- USB console with `printf`; the 1200-baud touch enters the bootloader.
- Watchdog with an early warning, which the examples route to the bootloader.
- I2C1, I2C2 and SPI2, with weak lock hooks for RTOS use.
- Header I/O: digital, EXTI, pulse measurement and shift register; unrouted pins are refused.
- ADC1 and ADC2, scaled against the measured supply.
- PWM and tone, with timer clocks measured rather than assumed.

### Sensors

- **Motion:** LSM6DSL and MMC5603NJ through ST's motion-sensor API.
- **Thermal:** FLIR Lepton 3.1R, 160x120 radiometric video on SPI1.
- **Ranging:** VL53L1CB through ST's ranging API, including a 16-zone grid.

### Vision

- **Camera:** three DCMIPP pipes (RGB565, YUV422, RGB888 with crop and scale), continuous and snapshot capture, and picture controls.
- **Sensors:** the OV5640, VD66GY and VD65G4, all through `BSP_CAMERA_RegisterSensor`.
- **JPEG encoder:** fed from the camera through an 80 KB line-buffer band, at the sensor's own rate in every mode. That is 60 fps up to WVGA and at 720p, 30 fps at XGA and 1080p, and 15 fps at QSXGA. The VD66GY runs every mode at its ceiling.
- **Display:** two LTDC layers with buffer flips and colour keying. The camera runs at 60 fps on the TFT-W panel.

### Audio

Speaker and PDM microphone with caller-owned buffers. A requested rate is
accepted only within 1%. Mute is the amplifier's shutdown pin.

### Radio and NPU

- **Wi-Fi (MAYA-W476):**
  - access point and station modes;
  - WPA2 and WPA3-Personal only, with open, WEP and WPA refused;
  - scanning, DNS, network time and power save;
  - settings stored on the board, and an optional link key;
  - the board's access point uses a random password; Neuro Studio is served on it only;
  - 2.4 GHz only, at 8 dBm.
- **BLE:** HCI transport and the Apache NimBLE host: advertising, scanning, both connection roles, GATT server and client, and LE Secure Connections pairing. Transmit power is capped at 8 dBm.
- **TLS 1.2 and 1.3** over Wi-Fi, with Mbed TLS and NXP's HTTPS server. The board makes its key and certificate at each start.
- **NPU:** the BSP prepares the accelerator; ST Edge AI's runtime runs the networks.
- **Security:** see `SECURITY.md`. Images are checked by CRC, not signed, and there is no network update path.

### Bootloader

`FSBL/` holds the v2.6.2 bootloader that boards ship with, and its image. It
owns the clock tree and the external memories, offers DFU over USB, and boots
one of two slots, rolling back an update that does not confirm itself. Its
contract with the BSP, and its known findings, are in `docs/BOOT-CONTRACT.md`.

### Examples

35 examples, each with its own CubeIDE projects. They include:

- 17 standalone examples, one thing each;
- the BSP test console;
- camera streams over USB and as a USB camera;
- thermal camera examples;
- five neural networks over USB and Wi-Fi;
- a Wi-Fi API console, HTTPS over Wi-Fi, and two BLE examples.

[Examples/README.md](Examples/README.md) lists them all.

- The examples' own code is CC0 1.0, except the two YOLOv8 examples, which are AGPL-3.0.
- `BSPTest` and `Camera_JpegStream_USB` also build under FreeRTOS.
- `Camera_YoloV8Pose_WiFi_FreeRTOS` restarts itself on failure, and backs off after five restarts in a row.

### Verification

- Every module builds clean under `-Wall -Wextra -Werror`, in both security states.
- All 72 projects build from a clean export: 68 by default, plus the 4 restricted ones.

### Known limitations

- No autofocus (fixed-focus modules), and no DMA2D in the LCD fills.
- **BLE:** bonds are kept in RAM, and the HCI link runs at 115200 baud.
- **TLS:** the certificate is new at each start, so clients pin it. There is no revocation check, and no TLS 1.3 resumption.
- **Wi-Fi throughput** is about 2.4 MB/s, limited by the 8.163 MHz SDIO clock. Larger frames are paced by the link.
- **WPA3:** no SAE hash-to-element.
- **Global-shutter sensors** start once per power cycle (no reset line).
- **Shared pin:** PD3 is both the thermal camera's power line and the BLE link's flow control (`docs/PART-VARIANT.md`).
- No XSPI or clock module; the bootloader owns both.

### Licensing

Ohm Lab's code is BSD 3-Clause. The examples are CC0 1.0 (the two YOLOv8
examples AGPL-3.0), and a few files noted in `LICENSE` are MIT. Third-party code
keeps its own licence, some of it not open source: see `LICENSE` and
`THIRD-PARTY-NOTICES.md`.
