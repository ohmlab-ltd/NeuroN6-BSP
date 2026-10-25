# Verification record

What has been verified on a Neuro N6, and what is open. Boards were fitted
with the OV5640-W carrier, the TFT-W display carrier, or a thermal carrier
(VD66GY or VD65G4, and a Lepton). Results are from `harness/n6test.py`
driving the test console in `Examples/BSPTest`, and from the bench tools in
`harness/`. `python harness/n6test.py` runs the sweep again and writes its
report into this folder.

| area | status | how verified |
|---|---|---|
| board: LEDs, button, identity, clocks | verified | BSPTest; clocks read back at 400 MHz system, 200 MHz bus |
| watchdog | verified | BSPTest: 4 s timeout, early warning routed to the bootloader |
| USB console, `b` to the bootloader, 1200-baud touch | verified | BSPTest and every example |
| I2C, motion sensor, magnetometer | verified | BSPTest, both builds; a bus held by a reset mid-read is freed at start-up |
| ranging (VL53L1CB) | partial | BSPTest `tof` and `tofgrid`; host test `test_ranging.c` |
| analogue, header pins, PWM, tone | verified | BSPTest, both builds |
| audio | verified; loopback open | rates timed against those reported; speaker heard by the microphone |
| camera (OV5640) | verified, all seven modes | BSPTest sweeps; QSXGA at 15 fps, 720p at 60 |
| camera (VD66GY) | verified, all nine modes | `Camera_UvcStream_VD66GY_USB`: 88 fps to 640x480, 60 to 1120x720, 45 for the whole array |
| camera (VD65G4) | 640x480 only | `Camera_UvcStream_VD65G4_USB`, 60 fps |
| JPEG encoder | verified | colour bars decoded on the host at every resolution |
| USB streaming, USB video | verified | `Camera_JpegStream_USB`, `Camera_UvcStream_USB`: 60 fps, both builds, every frame decoded |
| display controller | verified | TFT-W panel: colour bars, camera at 60 fps with detector boxes |
| NPU, NIA-416 detector | verified | 50 to 55 ms an inference, ten a second, over USB and Wi-Fi |
| palm, hand and segmentation networks | verified | 25 to 26, 19 to 26 and 36 to 39 ms an inference, streams at 30 fps |
| YOLOv8 pose over Wi-Fi | partial | 34 to 39 ms an inference, 29.9 fps; skeletons not yet checked |
| Wi-Fi | verified | `n6wifiapi.py` 120 of 120, `n6radiotest.py` 44 of 44, a two-hour stream |
| BLE | verified | NimBLE host 57 of 60 checks, `BlePeripheral_FreeRTOS` 18 of 18, `n6btpower.py` 14 of 14 at or below 8 dBm |
| TLS | verified, local only | 94 of 94 checks against Python and OpenSSL; `WiFiTls_FreeRTOS` 14 of 14 |
| FreeRTOS | verified | BSPTest `rtos` and the full sweep under FreeRTOS |
| Lepton | verified | `CheckThermal`, `ReadThermal`, `Thermal_Stream_USB` at 9 fps; host tests `test_vospi.c`, `test_thermal_*.c` |
| boot loader v2.6.1 | verified | resets and over-the-air updates, a board's first included |
| boot loader v2.6.2 (shipped) | identified | `harness/n6fsbl.py`; open findings tested on a simulated board (`harness/bench/hosttest/fsbl`) |
| examples | verified | all 24 application images uploaded, ran and returned on `b` |

Latest sweep, OV5640-W carrier on FSBL v2.6.2: bare metal 49 passed and 3
skipped; FreeRTOS 49 passed, 1 failed (an intermittent camera start, below)
and 2 skipped.

## Known limits

- **A camera bring-up can end with no frame**: 5 in 168 across every mode, 26
  in 288 at XGA after a mode change. Notice it and start the camera again, as
  `BSP_CAMERA_Init` documents. In full sweeps `camcore` check 8 and
  `cammode`'s double-buffered capture have failed this way; both pass alone.
- **The CSI lane mapping depends on the carrier, and a wrong one is silent**
  (no frame; the registers still read back). `camcarrier` reports which
  captures: straight on the plain carrier, crossed on the TFT-W
  (`BSP_CAMERA_DEFAULT_CARRIER` = `BSP_CAMERA_CARRIER_INVERTED` in
  `neuron6_conf.h`). The USB camera examples default to the standard carrier
  and get no frame on the TFT-W.
- **Picture tests need light**: `cam`, `camres`, `camisp` and the CAM0
  exposure check fail facing a dark scene, though frames still arrive.
- **No autofocus**: the OV5640 modules have a fixed lens and the BSP carries
  no autofocus firmware.
- **OV5640 orientation on the display carrier is unconfirmed**: it is left
  unflipped, not checked against a panel.
- **Audio loopback on the TFT-W carrier**: the microphone stopped hearing the
  speaker with no audio code changed (`audloop` skipped); the speaker's
  connection is to be checked.
- **Ranging zones**: 6 or 7 of the 4 x 4 grid's 16 zones range; their
  distances are unchecked against targets at known distances.
- **VD65G4**: only 640x480 recorded. One `BSP_CAMERA_Init` per power cycle on
  this carrier, which puts no GPIO on XSHUTDOWN. A pull-up on XSHUTDOWN above
  about 9 kOhm leaves the sensor in shutdown (it holds the pin low through
  roughly 20 kOhm until it boots).
- **Lepton** readings run about 8 C high until its first flat-field
  correction, a few seconds in.
- **Network weights share one flash region**: another model's deploy can
  replace them silently. The NN examples check the region's CRC at every
  start and refuse foreign weights; `harness/n6weights.py` writes them back.
- **Segmentation example**: one start enumerated but never opened its
  console, and fell back to the boot loader with no fault recorded; the 13
  starts after it were clean.
- **YOLOv8 pose example**: the skeletons, each restart path and a day's soak
  have not been checked.
- **Wi-Fi**:
  - large frames can tear: at a 33 KB median 0.14% tore and were dropped on
    the board, so the host received only intact frames; 16 to 24 KB frames
    showed none;
  - a client joining the access point can stall in the WPA2 handshake; the
    board sends it away after 3 s;
  - no station has joined a WPA3 network and a station's rejoin is untested;
    SAE hash-to-element is not offered (a Windows PC could not join with it);
  - the SDIO bus runs at 8.163 MHz, the clock it passed EMC tests at, which
    caps the stream at about 2.44 MB/s;
  - the driver's timers (the 802.11n receive-reorder flush among them) run on
    the FreeRTOS timer task, which needs a 2 KB stack;
  - the thermal carrier has no radio.
- **BLE**: the controller runs only from the combined image the Wi-Fi stack
  loads over SDIO, so the bare-metal console's `bt` test reports SKIP. Bonds
  are kept in RAM.
- **TLS**: the client has not reached the internet from the board. Two
  clients that stop reading a large reply fill lwIP's 96 KB heap, and a third
  handshake times out.
- **Boot loader**: cold power-up is outside this record. v2.6.2's open
  findings are in
  [docs/BOOT-CONTRACT.md, "Known issues in v2.6.2"](../../docs/BOOT-CONTRACT.md#known-issues-in-v262).
- **Constraints**: the encoder's output buffer must be uncached and its band
  on-chip (`neuron6_venc.h`); an idle hook that sleeps the processor stops
  every peripheral whose sleep enable is clear (`neuron6_rtos.c`); each
  chroma plane of a full-planar frame spans `Width x Height / 2` bytes (the
  DCMIPP has one pitch register), so size buffers with
  `BSP_CAMERA_GetFrameSize`.
