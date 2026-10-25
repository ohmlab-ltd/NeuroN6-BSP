# BSP test console

One command per BSP module on the USB console, each answering `OK`, `FAIL`
or `SKIP`. `harness/n6test.py` drives it, so the BSP can be exercised on a
real board unattended.

```
host                                  board
 n6flash.py  --upload-->  bootloader (1209:8009) --boots--> this image (1209:8008)
 n6test.py   --"imu\n"-->  console
             <--"INFO id=0x6a acc_mg=12,-8,1004 |a|=1004 ..."
             <--"OK imu id=0x6a |a|=1004mg ..."
             <--"END imu"
             --"b\n"-->    back to the bootloader
```

- Send one command at a time and wait for its `END`. While a command runs the
  board reads the console only for `b` and discards the rest: a command sent
  early is lost, and a `b` in it (as in `bt`) sends the board to the
  bootloader.
- Each command prints any number of `INFO` lines, one verdict line with the
  numbers that matter, then `END <cmd>`, so the results in
  `harness/board-tests/` read on their own.

## Commands in the default run

| command | checks |
|---|---|
| `bkp` | breadcrumbs from the previous image: backup registers 4..7 and the reset flags. Register 7's top byte: 0xFD the watchdog's early warning; 0xF1 to 0xF8 a fault handler, the kernel or a non-maskable interrupt (layout in `Common/Src/stm32n6xx_it.c`); 0xFB an interrupt with no handler |
| `id`, `clk` | board name; the clock tree the boot loader left |
| `led`, `btn` | each LED lit and read back; button state |
| `i2c` | `BSP_I2C1_IsReady` at every possible address, 7- and 8-bit forms |
| `imu` | LSM6DSL: about 1 g at rest, gyro, temperature |
| `mag` | MMC5603NJ: a plausible field magnitude |
| `i2cref` | the reference-counted sensor bus is unclocked after each user releases it |
| `magcont` | magnetometer free-running at 50 Hz (paced four ways, then after a degauss) and on demand: no destroyed or stale samples, plausible temperature |
| `adc` | VDDA against the **1.8 V** reference, VBAT, VDDcore against its nominal 800 mV, A0..A5 in mV and counts |
| `io` | each digital header pin reads its own driven level |
| `hdr` | pin interrupts through attach and detach, the shift helper, a pulse timeout, I2C2 and SPI2 up, used and down |
| `pwm`, `tone` | every PWM-capable pin started; a 100 ms tone runs and ends |
| `hdredge` | header edge cases: PWM restart and shared-timer duty, LED off after PWM stop, sub-period tones, no stale edge after re-attach, SPI2 rate floor, idle levels and full duplex, pin 28 (USER button) interrupt |
| `wdg` | the watchdog is running (since the first line of `main`) |
| `imucfg` | IMU rate and full scale round-trip, sensitivity tracks full scale; magnetometer degauss |
| `audcfg` | audio getters match init; pause and resume in both directions; volume must be refused (the amplifier's gain is a strapping resistor) |
| `camcfg` | every sensor tuning control round-trips; brightness is checked on the picture |
| `camsusp` | pipe and sensor suspend stop frames; resume restarts them |
| `pwmcfg` | PWM frequency and resolution round-trip (the waveform itself is not seen) |
| `miscapi` | ADC resolution scaling, `HasAdc`, a toggled pin, the console reports connected |
| `camisp` | the five ISP stages: accepted on MAIN, NULL turns one off, refused on other pipes; the colour matrix is checked on the picture (full YUV frames in PSRAM) |
| `cammode` | the five capture starts other than `BSP_CAMERA_Start` (double buffering, both planar layouts): every buffer and plane written, within bounds; invalid starts refused |
| `rtos` | FreeRTOS build only (`SKIP` on bare metal): kernel and HAL ticks agree, the HAL tick moves while the scheduler is held, `HAL_Delay` yields, a software timer fires, a second task reads the IMU |
| `npu` | NPU init, RAM info, deinit |
| `mic` | MDF microphone: DMA callbacks fire, buffer not silent |
| `spk` | SAI speaker: 1 kHz tone, callbacks, mute readback |
| `audloop` | the speaker heard by the microphone at 11,025 Hz, the native rate and 48 kHz: right frequency, same level. Needs the TFT-W carrier's speaker |
| `camcarrier` | which CSI-2 lane mapping delivers frames (three attempts each). A wrong mapping raises no error anywhere else |
| `cam` | one OV5640 RGB565 snapshot, pixel statistics, a 16 x 8 ASCII preview |
| `camcore` | capture-core cases on the sensor's colour bars (no light needed): continuous after snapshot, independent pipes, chroma pitch, double-buffer halves, line events, AUX alone, AUX recovery from a HAL error state |
| `campic` | OV5640 picture controls keep each other's register bits; zoom limits at XGA and 720p |
| `camres` | all seven resolutions capture |
| `vencreg`, `rif` | encoder identity and capability registers; RIF master stamps |
| `jpeg` | the line-buffer path: ten frames, SOI/EOI checked |
| `jpegres` | five frames at every mode, QVGA to QSXGA, with the band geometry |
| `jpegmem`, `jpegdump` | colour bars encoded from memory (`BSP_VENC_EncodeFrame`) and sent as hex; `n6test.py` decodes and measures them |
| `jpeggap`, `jpegrun` | both primings around the 1500 ms gap USB enumeration costs: 5 of 5 encode |
| `jpegtorn` | after the camera is stopped mid-frame at ten points and the encoder resynchronised, the next frame is not reported torn |
| `lcd` | 800x480 panel: pixels, row pitch, clipping, both layers, scan line |
| `lcdbmp` | 16-, 24- and 32-bit bitmaps into RGB565, ARGB8888 and ARGB4444 layers, pixel for pixel |
| `reg16` | `BSP_I2C_ReadReg16` three ways, an absent device refused, the SPI2 clock |
| `audset` | playback at half the native rate; capture at 48 kHz and 8 kHz; re-init after the slow rate |
| `wifi` | MAYA-W476 over SDIO: power, CMD0/5/3/7, CCCR, firmware presence |
| `bt` | HCI over USART6. Always `SKIP` here: only the MAYA-W476 library, under FreeRTOS, loads the controller's firmware. BLE is tested by `Examples/BleBeacon_FreeRTOS` and `Examples/WiFiApi_FreeRTOS`'s `bt` |
| `tof` | ranging sensor identity, up to ten full-field measurements; skips without the sensor |
| `tofgrid` | ranging regions: three-wide refused, 8 x 8 ranged, a 16-zone sweep within 6 s; skips without the sensor |
| `thermal` | Lepton I2C probe only (power pins not driven); skips when no Lepton is fitted |
| `vospi` | the Lepton video parser on packets built on the board: a whole frame parses; a stuck line, bad packet IDs and split segments are rejected |

## Commands not in the default run

| command | does |
|---|---|
| `wdgfire`, `bkpclr` | stop feeding the watchdog; clear the breadcrumbs |
| `wdgspin`, `faultudf` | starve the watchdog with the encoder up (the early warning quiesces it and leaves 0xFD in register 7); run an undefined instruction (the fault handler leaves its code). `bkp` in the next image reads either |
| `comreinit` | close and reopen the console; the results go to backup register 6 for `bkp` in the next image |
| `i2creset`, `i2cline` | a watchdog reset mid-read; then the next image's `BSP_I2C1_Init` must free the bus and every device must answer its first probe |
| `i2cackcut`, `i2ccheck` | a reset while the sensor holds SDA low; then `BSP_I2C1_Init` must free the bus without completing a byte |
| `otacancel`, `otakeep`, `otacheck` | arm the update trigger with nothing staged and enter the bootloader through `BSP_COM_RebootToBootloader` (`otacancel`, must pass: an upload cancels a staged update) or directly (`otakeep`, fails by design with "no header"); `otacheck` after the upload. The flash is never written |
| `camxga` | XGA brought up twelve times, recording the D-PHY status; fails about half the time by design (see `BSP_CAMERA_R1024x768`) |
| `vencprobe`, `vencrecover`, `vencrst`, `camreg`, `camlive` | encoder probe and reset sequences; sensor and CSI registers, idle and streaming |
| `magdbg`, `magloop` | the magnetometer by hand with every register shown; `mag` twenty times with a post-mortem |
| `adcrif` | A0..A5 with the ADC12 security stamp on, off and on: the analogue pads boot secure and do not route to a non-secure converter. It briefly clears a security attribute |

## Not covered

- `neuron6_camera_vd66gy` and `neuron6_camera_vd65g4` have no command: this
  board carries the OV5640.
- `thermal` skips when no Lepton is fitted.

## Never off the bus

- Every wait in `main.c` goes through `svc()`, which runs USB, honours `b` and
  refreshes the watchdog. BSP calls that block (a probe timeout, the radio's
  power-up settle) are shorter than the watchdog.
- With `BSP_USE_FREERTOS` at 1 the same `main.c` runs as two tasks, the BSP's
  USB task (`BSP_RTOS_StartUsbTask`) and a console task (one `console_step`
  per tick); `svc()` yields instead of pumping.
- `BSP_COM_Read` returns what the host sent even after the host has dropped
  the port, so a tool that writes `b` and closes at once still reaches the
  bootloader.
- The watchdog runs at 4 s, with its early warning 300 ms before the reset.
  The warning writes the bootloader key, so a wedged test lands in the
  bootloader and `n6test.py` reports `WEDGED`. The fault handlers in
  `stm32n6xx_it.c` do the same and leave a code in backup register 7, as does
  `Default_Handler_C`: the startup file's `Default_Handler` branches to it
  (`b Default_Handler_C` in place of `b Infinite_Loop`), so an interrupt with
  no handler records its vector number.

## Building

`Core/Src/main.c`, `Core/Src/stm32n6xx_it.c` and `Core/Inc/neuron6_conf.h`
build from the CubeIDE projects under `STM32CubeIDE/`:

- `Appli`: bare metal; the whole BSP, `Components/` and every HAL module the
  BSP uses.
- `Appli_FreeRTOS`: `Appli` plus what `Utilities/freertos/README.md` lists
  (kernel, `neuron6_rtos.c`, the TIM16 time base, `BSP_USE_FREERTOS=1`).
- `FSBL`: the boot loader, as in every example.

Import them into STM32CubeIDE 2.2.0 and build `Debug`, or build all three
with `bash harness/examples/cubebuild.sh BSPTest`. `harness/compile.sh` also
compiles the console both ways. Then:

```
python harness/n6test.py Examples/BSPTest/STM32CubeIDE/Appli/Debug/BSPTest_Appli_Trusted.bin
python harness/n6test.py Examples/BSPTest/STM32CubeIDE/Appli_FreeRTOS/Debug/BSPTest_FreeRTOS_Appli_Trusted.bin
```

or, for the whole bench including the streaming image, `harness/n6bench.py`.
