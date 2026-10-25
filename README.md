# Neuro N6 BSP

Board support package for the Ohm Lab **Neuro N6** (STM32N657I0H3Q, Cortex-M55
with the Neural-ART NPU), in the style of an STMicroelectronics BSP: plain C,
`int32_t BSP_*` functions returning `neuron6_errno.h` codes, a configuration
template, and weak hooks to override. For bare-metal and FreeRTOS projects in
STM32CubeIDE.

The repository holds:

- **The BSP:** one module per part of the board ([Modules](#modules)).
- **35 examples**, each its own CubeIDE project: 17 that show one thing each,
  plus camera, thermal, neural-network, Wi-Fi, TLS and BLE examples
  ([Examples/README.md](Examples/README.md)).
- **The middleware they use**, each under its own terms: FreeRTOS, TinyUSB,
  lwIP and NXP's Wi-Fi driver, Apache NimBLE, Mbed TLS, ST's NPU runtime.
- **The first-stage boot loader's source** (v2.6.2) and the **host tools**
  that build, upload and test from the command line
  ([harness/README.md](harness/README.md)).

Hardware-validated on a Neuro N6: every module builds clean under
`-Wall -Wextra -Werror`, and `Examples/BSPTest` passes its sweep on the board,
bare metal and under FreeRTOS. [harness/board-tests/STATUS.md](harness/board-tests/STATUS.md)
has the detail.

## Requirements

- **STM32CubeIDE 2.2.0** (GNU Tools for STM32 14.3.rel1). Other versions are
  untested. Each build runs ST's signing tool with no key to make the
  `_Trusted.bin` the boot loader accepts: a header, not a signature.
- **A Neuro N6 with boot loader v2.5.0 or later.** Older ones (v2.0.2 to v2.4.0)
  read the external flash with errors, which breaks the radio and the NPU
  silently. `python harness/n6fsbl.py` reports the version. Boards ship with
  v2.6.2: `FSBL/Neuro-N6-FSBL-v2.6.2-Trusted.bin`, installed as
  [harness/uploader/README.md](harness/uploader/README.md) describes.
- **The radio firmware and network weights in external flash**, for the Wi-Fi
  and neural-network examples. `harness/n6weights.py` checks and writes them.
- **Python 3 with `pyserial`** for `harness/` (written for Windows with Git Bash).
- **A short path on Windows:** keep the repository folder's path under about
  100 characters, as builds pass Windows' 260-character limit otherwise.

[Examples/README.md](Examples/README.md) explains how to build and upload an example.

## The boot contract

The factory boot loader owns the clock tree and both external memories. **The
BSP never configures clocks, XSPI or PSRAM**, never calls `SystemClock_Config`,
and reads clock frequencies back at run time. [docs/BOOT-CONTRACT.md](docs/BOOT-CONTRACT.md)
defines the memory map, the image-size limit and the image header.

The part is the **STM32N657I0H3Q (VFBGA178, 106 I/O)**, not the Discovery kit's
VFBGA264. Some peripherals have no pin here, and PD3 is shared:
[docs/PART-VARIANT.md](docs/PART-VARIANT.md).

## Modules

| Module | Covers | Notes |
|---|---|---|
| `neuron6` | LEDs, USER button, board identity | Common-anode LEDs; button on PG8 |
| `neuron6_com` | USB CDC console | No debug UART. Retargets `printf`; the 1200-baud touch enters the boot loader |
| `neuron6_wdg` | Independent watchdog | Millisecond API, early-warning callback |
| `neuron6_npu` | NPU RAM, cache, interrupts, isolation | The runtime is ST Edge AI's (SLA0044), in `Middlewares/ST/AI_Runtime` |
| `neuron6_bus` | I2C1 (sensors), I2C2 and SPI2 (header) | Weak lock hooks for RTOS use |
| `neuron6_io` | Expansion header | Digital I/O, EXTI, pulse measurement, shift register |
| `neuron6_adc` | A0..A5, reference, battery, core rails | Scaled against a measured supply |
| `neuron6_pwm` | PWM and tone | Timer clocks measured, never assumed |
| `neuron6_motion_sensor` | LSM6DSL, MMC5603NJ | ST motion-sensor API |
| `neuron6_ranging_sensor` | VL53L1CB time of flight | ST ranging API; single ranging, ROI, 16-zone grid |
| `neuron6_thermal` | FLIR Lepton 3.1R | 160x120 radiometric video on SPI1 |
| `neuron6_camera` | OV5640 over CSI-2, three DCMIPP pipes | Continuous and snapshot; any sensor through `BSP_CAMERA_RegisterSensor` |
| `neuron6_camera_vd65g4`, `_vd66gy` | Global-shutter sensors | Registered through the camera module |
| `neuron6_venc` | Hardware JPEG encoder | Fed from the camera through the DCMIPP line buffer; every sensor mode at full rate |
| `neuron6_lcd` | 800x480 RGB565 panel on LTDC | Two layers, buffer flips, drawing primitives |
| `neuron6_audio` | MAX98357A speaker, PDM microphone | Rates accepted only within 1%; no volume control (no codec) |
| `neuron6_wifi` | MAYA-W476 power and SDIO | 8.163 MHz bus, the EMC-tested clock |
| `neuron6_bt` | BLE HCI over USART6 | No command can set BLE transmit power above 8 dBm |
| `neuron6_rtos` | FreeRTOS glue | Only with `BSP_USE_FREERTOS` |

Libraries and support:

| Path | What |
|---|---|
| `Components/` | Sensor drivers: OV5640, VD66GY and VD55G1 (ST, BSD-3-Clause), the VC8000NanoE JPEG encoder, and Ohm Lab drivers for the LSM6DSL, MMC5603NJ, Lepton and VL53L1CB |
| `Middlewares/NeuroN6/MayaW476/` | Wi-Fi library: access point or station, WPA2/WPA3 only, video over TCP on the board's own access point, 2.4 GHz only at 8 dBm |
| `Middlewares/NeuroN6/BleHost/` | Apache NimBLE on `neuron6_bt` (FreeRTOS only) |
| `Middlewares/NeuroN6/Tls/` | TLS 1.2/1.3 over Wi-Fi with Mbed TLS and NXP's HTTPS server (FreeRTOS only) |
| `Middlewares/NeuroN6/PostProcess/`, `Models/` | The five networks and their post-processing |
| `Middlewares/NeuroN6/N6OTA/` | Settings a host stores on the board, written by the boot loader |
| `Drivers/` | STM32N6 HAL and CMSIS, with two fixes ([docs/HAL-PATCHES.md](docs/HAL-PATCHES.md)) |
| `Utilities/` | USB, FreeRTOS and NPU memory configuration; the patched SDIO driver |
| `FSBL/` | The v2.6.2 boot loader's source and its published image |
| `harness/` | Compile gate, uploader and bench tools (not shipped API) |

## Using the BSP in a CubeIDE project

1. Add the repository root and each `Components/*` you use to the include paths, and their `.c` files to the build.
2. Copy `neuron6_conf_template.h` into your project as `neuron6_conf.h`.
3. Enable the HAL modules you need in `stm32n6xx_hal_conf.h`. `harness/stm32n6xx_hal_conf.h` enables them all.
4. Reserve a non-cacheable region and point `BSP_THERMAL_DMA_NODE_SECTION` at it.
5. Call the BSP's interrupt handlers from `stm32n6xx_it.c`. Each header names its vector.
6. Use this repository's `Drivers/`, or apply [docs/HAL-PATCHES.md](docs/HAL-PATCHES.md). The camera's larger modes and the radio need those fixes, and CubeMX regeneration reverts them.
7. `HAL_<PPP>_MspInit` hooks are yours, except `HAL_DCMIPP_MspInit` and `HAL_PCD_MspInit`. Override those through the `BSP_*_RegisterMspCallbacks` functions.
8. Memory isolation is handled by `BSP_ISOLATION_Config`. If your application owns it, set `BSP_ISOLATION_APPLICATION_OWNED` and admit every bus master the BSP drives.

## FreeRTOS

Bare metal by default. Set `BSP_USE_FREERTOS` to 1 in `neuron6_conf.h` to add:

- `Middlewares/FreeRTOS/`: FreeRTOS Kernel V10.6.2, Cortex-M55 port, `heap_4`;
- `Utilities/freertos/`: `FreeRTOSConfig.h`, with the HAL time base on TIM16;
- `neuron6_rtos`: bus and radio locks as mutexes, a yielding `HAL_Delay`, the USB task and a fatal hook.

BSP interrupts sit at priority 5 or below, so their callbacks can use `*FromISR`.
The watchdog's early warning, at priority 0, cannot.
[Utilities/freertos/README.md](Utilities/freertos/README.md) lists the steps.

## Cameras

The OV5640 is the default sensor. Register a global-shutter sensor before
`BSP_CAMERA_Init`:

```c
BSP_CAMERA_VD66GY_Register();
BSP_CAMERA_SetCarrier(BSP_CAMERA_CARRIER_INVERTED);   /* thermal carrier only */
BSP_CAMERA_Init(0, BSP_CAMERA_VD66GY_R1024x768, BSP_CAMERA_PF_RAW8);
```

```c
BSP_CAMERA_VD65G4_Register();
BSP_CAMERA_Init(0, BSP_CAMERA_VD65G4_R804x704, BSP_CAMERA_PF_RAW8);
```

- A raw sensor is reconstructed on the main pipe (YUV422); the auxiliary pipe
  follows it (RGB888); the dump pipe carries raw data.
- The VD65G4 needs a carrier pull-up on XSHUTDOWN below about 9 kOhm, or it
  stays in shutdown. Its constants are in `neuron6_conf.h`.
- The VD66GY needs the D-PHY fix in every mode, and leaves white balance and
  colour to `BSP_CAMERA_SetIsp*`. It starts only from power-up or a board reset.
- `BSP_CAMERA_SetCarrier` sets the receiver's lane mapping for the carrier;
  a wrong setting gives no frames.
- `neuron6_camera_vd66gy.c` shows how to add another sensor.

## Known limits

- The VD65G4 has run 640x480 at 60 fps; its other three modes have not run.
- A camera bring-up occasionally ends with no frame; the remedy is documented on `BSP_CAMERA_Init`.
- The radio firmware must be in external flash at `0x91E00000`
  (`harness/n6weights.py --write`). `BSP_WIFI_IsFirmwarePresent` checks it.
- PD3 is both the thermal camera's power line and the BLE link's flow control ([docs/PART-VARIANT.md](docs/PART-VARIANT.md)).
- No Ethernet, no H.264, no DMA2D in the LCD fills, and no clock or flash module (the boot loader owns those).
- Standby: nothing here enters it. An application that does must point
  `SYSCFG->INITSVTORCR` at a vector table that survives Standby.

## Licence

Not all of this repository is BSD 3-Clause. [LICENSE](LICENSE) covers Ohm Lab's
own code and lists the exceptions. [THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md)
lists every bundled part and its terms. A folder or file with its own licence is
governed by that licence.

- **Ohm Lab code:** BSD 3-Clause. Some files are MIT. The examples' code is CC0 1.0.
- **Vendored code:** mostly MIT, BSD-3-Clause or Apache-2.0. The video encoder
  is used under its BSD-3-Clause option.
- **ST's NPU stack** (`Middlewares/ST/`, the generated networks, the model-zoo
  wrappers): SLA0044. For use with ST devices only; it may not be placed under
  open-source terms.
- **YOLOv8 networks** (`Models/YoloV8Seg`, `YoloV8Pose`) **and their two
  examples' `main.c`:** AGPL-3.0. Firmware that combines them with the
  SLA0044 components may not be distributed in most cases. The default
  builds skip these examples unless `NEURON6_BUILD_RESTRICTED=1`. See
  THIRD-PARTY-NOTICES §1.3.
- **NIA Triad-416 model** (`Models/Nia416`): the
  [Ohm Lab Model Licence](Middlewares/NeuroN6/Models/Nia416/LICENSE-OhmLab-Model.txt).
  It may be used and redistributed, but runs on Ohm Lab hardware only
  (team@ohmlab.co.uk for other hardware).
- **IW610 radio firmware:** NXP proprietary (LA_OPT). It may be distributed as
  object code inside a product built on an NXP part, as NXP's software content
  register grants.
