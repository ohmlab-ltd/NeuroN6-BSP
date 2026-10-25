# Release notes

## v0.1.0 (unreleased)

First public release of the Neuro N6 board support package: drivers for every
peripheral on the board (STM32N657I0H3Q, Cortex-M55 with the Neural-ART NPU),
the first-stage boot loader's source, and 35 examples. Plain C,
`int32_t BSP_*` functions returning `neuron6_errno.h` codes, bare metal or
FreeRTOS.

### Core

- Board identity, RGB LED and user button.
- USB console with `printf`; the 1200-baud touch enters the boot loader.
- Watchdog with an early warning, which the examples route to the boot loader.
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

### Boot loader

`FSBL/` holds the v2.6.2 boot loader that boards ship with, and its image. It
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
- **On hardware:**
  - the board, watchdog, console, buses, sensors, analogue, header pins, PWM and audio;
  - all seven OV5640 modes, the JPEG encoder, USB streaming and USB video;
  - the NPU, and the TFT-W panel;
  - Wi-Fi in both modes, WPA3 on the access point, and TLS;
  - BLE in all four roles, with every pairing method;
  - FreeRTOS;
  - the boot loader across resets and staged updates.
- The VD66GY and VD65G4 each stream VGA at 60 fps.
- The YOLOv8 pose example runs at 15 inferences and 30 fps. Its day-long soak and restart paths are untested.

### Known limitations

- No autofocus (fixed-focus modules), and no DMA2D in the LCD fills.
- **BLE:** bonds are kept in RAM, and the HCI link runs at 115200 baud.
- **TLS:** the certificate is new at each start, so clients pin it. There is no revocation check, and no TLS 1.3 resumption.
- **Wi-Fi throughput** is about 2.4 MB/s, limited by the EMC-tested 8.163 MHz SDIO clock. Larger frames are paced by the link.
- **WPA3:** no SAE hash-to-element, and station mode on a WPA3 network is untested.
- **Global-shutter sensors** start once per power cycle (no reset line).
- **Ranging:** the grid's distances have not been checked against known targets.
- **Shared pin:** PD3 is both the thermal camera's power line and the BLE link's flow control (`docs/PART-VARIANT.md`).
- No XSPI or clock module; the boot loader owns both.

### Licensing

Ohm Lab's code is BSD 3-Clause. The examples are CC0 1.0 (the two YOLOv8
examples AGPL-3.0), and a few files noted in `LICENSE` are MIT. Third-party code
keeps its own licence, some of it not open source: see `LICENSE` and
`THIRD-PARTY-NOTICES.md`.
