# XGA to Neuro Studio over Wi-Fi with the NIA Triad-416 detector, under FreeRTOS

Streams XGA (1024 x 768) Motion-JPEG to Neuro Studio over Wi-Fi, while Ohm
Lab's NIA Triad-416 detector runs on the Neural-ART NPU on a centred
768 x 768 square scaled to 416 x 416. Every frame carries the latest
detections (a box and class name for each of the 80 COCO classes found,
mapped onto the whole frame) and the inference time. The pipeline is
[Camera_Nia416Stream_USB_FreeRTOS](../Camera_Nia416Stream_USB_FreeRTOS)'s;
only the video transport differs.

- The board hosts its own access point, so no router is needed. The radio
  works on **2.4 GHz only, at 8 dBm**, the settings the Neuro N6's wireless
  build was EMC-tested with.
- The stream stays on USB until a Neuro host connects over Wi-Fi; that host
  then gets everything a USB host gets (video, metadata, panel announces, the
  Terminal with its boot log, live controls). When it leaves, the stream
  returns to USB. One host at a time.

The console reports the radio as it comes up, then a heartbeat and a rate
line once a second, and the network every five:

```text
[MayaW476] Region locked to WWSM (2.4 GHz world-safe, single SKU US/EU/UK).
[MayaW476] TX power cap: requested 8 dBm, firmware reports max 8 dBm over 14 ch x 7 mod-groups [OK]
[WiFi] SoftAP "NeuroN6-Cam"  password: xxxx-xxxx-xxxx-xxxx
[WiFi] status 7, address 192.168.10.1, port 5000
[WiFi] SDIO clock: kernel 114285714 Hz (IC4 /7), bus divider 7, bus 8163265 Hz
[MayaW476] WiFi up - Neuro host server on TCP :5000, discovery on UDP :5000, mDNS neuron6-xxxxxx.local
[MayaW476] Neuro host connected - stream and controls on WiFi.
[NET] host attached - stream moved to the network
[rate] enc=30 tx=30 torn=0 fail=0 resync=0 nn=10 npu=50ms jpg=26380B
[wifi] transport=1 status=8 ssid=NeuroN6-Cam ip=192.168.10.1:5000 heap=12840B min=12840B
```

## Before it runs

External flash holds three things besides the application:

- **The radio firmware**, 995,992 bytes at `0x91E00000`, CRC32 `0x8BEF4C0C`,
  downloaded to the radio at every start. Check it (read-only) or write it:

  ```bash
  python harness/n6weights.py Middlewares/NeuroN6/MayaW476/firmware/firmware.json
  python harness/n6weights.py Middlewares/NeuroN6/MayaW476/firmware/firmware.json --write
  ```

  A host writes the same file there when it uploads wireless firmware.
- **The detector's weights** at `0x90300000`, as in the USB example:
  `python harness/n6weights.py Middlewares/NeuroN6/Models/Nia416/model.json`.
  The example checks them before the network first runs and streams without
  detections if they are wrong.
- **A boot loader that hands over a correctly reading flash interface
  (XSPI1): FSBL v2.5.0 or later** (boards ship with v2.6.2). The firmware and
  the weights are read in place, and one wrong bit makes the radio reject its
  firmware ("FW Ready Registers not set"). `python harness/n6fsbl.py` reports
  the boot loader, read-only. See
  [the boot contract](../../docs/BOOT-CONTRACT.md#flash-speed-and-why-the-application-must-not-set-it).
  Inference takes 49 ms with the flash at 200 MHz, 66 ms at 100 MHz.

## Connecting

- **Access point** `NeuroN6-Cam`, WPA2. The password is the board's own:
  random (from the chip's RNG), made at first boot, when the board restarts
  once to store it, and kept through restarts and uploads. The console prints
  it at start-up and whenever a console connects; Neuro Studio reads it, with
  the network's name, over USB. The board is `192.168.10.1` and leases
  `192.168.10.10` to `.99`: point Neuro Studio at `192.168.10.1`, port 5000.
- **Finding the board** (on its own access point only): a UDP probe to port
  5000 gets its address, port and name; mDNS answers `neuron6-xxxxxx.local`
  (or a stored hostname); and the board announces its address once a second
  on the USB vendor interface.
- **In `main.c`:** `WIFI_AP_SSID`, `WIFI_AP_CHANNEL` (1 to 11, or 0 to
  choose) and `WIFI_AP_SECURITY` (`WIFI_SECURITY_WPA2`,
  `WIFI_SECURITY_WPA3_SAE` or `WIFI_SECURITY_WPA2_WPA3`; open networks are
  refused).
- **A stored access point** replaces the example's own: Neuro Studio's WiFi
  settings, or `harness/bench/n6bench.py wifiset --ap`, write it over USB and
  the board restarts into it, with its hostname and link key if given and the
  board's own password unless one was stored. The FSBL writes the settings to
  flash (the asset store). A stored network to join is not used here.
  `WIFI_USE_STORED 0` ignores them; `wifiset --clear` removes them.
- **A link key**, stored with the settings, makes a host answer an HMAC
  challenge within 10 s or be disconnected. It keeps other hosts off the
  board; the stream is only as private as the network.
- **One host at a time;** a second is refused. A host that takes nothing for
  5 s is dropped and its connection reset. The listen queue holds two, so a
  host abandoned mid-handshake cannot block the next one.
- A network host gets the live controls only. The asset commands and
  `REBOOT_DFU` need the USB cable, so the network cannot rewrite the stored
  settings or force the bootloader. USB stays up, and `b` on the console
  returns the bootloader.

## Radio settings

These are the settings the wireless build was EMC-tested with.

| setting | value | where |
|---|---|---|
| band | 2.4 GHz only; 5 GHz is compiled out of the driver | `CONFIG_5GHz_SUPPORT=0` on every file (`wifi_build.json`), and in `wifi_config.h` |
| region | World Wide Safe Mode, set before either mode starts | `maya_w4_connect.c` |
| transmit power | 8 dBm conducted on every 2.4 GHz channel and modulation group; the firmware's table is read back and printed at every start | `maya_w4_txpower.c` |
| access point channel | 1 to 11 (shared by FCC and ETSI; above falls back to 6), 20 MHz | `MayaW476.c`, `maya_w4_connect.c` |
| 802.11ac and ax | off | `wifi_config.h` |
| SDIO bus clock | 8.163 MHz: PLL1 / 7, then / 14. Both dividers are odd, so no harmonic lands on 200 MHz. Never gated; pins at high (not very high) slew; read back and printed at every start | `maya_w4_hw.c`, `sdio_port/stm32n6_sdio_host.c` |

The BSP's `neuron6_wifi` module uses the same clock.

## Licences

- The IW610 radio firmware (`Middlewares/NeuroN6/MayaW476/firmware`) is
  NXP's, under NXP's own licence; the ST software is under SLA0044. Neither
  is covered by this repository's BSD-3-Clause licence.
  `THIRD-PARTY-NOTICES.md` lists everything.
- The NIA Triad-416 model is Ohm Lab's, under the Ohm Lab Model Licence in
  `Middlewares/NeuroN6/Models/Nia416/LICENSE-OhmLab-Model.txt`: it may be run
  only on Ohm Lab hardware, and a firmware image built from this example
  contains it.

## Building it

Import `STM32CubeIDE/Appli` into STM32CubeIDE 2.2.0 and build `Debug`, or:

```bash
bash harness/examples/cubebuild.sh Camera_Nia416Stream_WiFi_FreeRTOS
```

The NPU half is as in the USB example. For the Wi-Fi half:

- `-fno-strict-aliasing` (strict aliasing breaks the firmware download) and
  `-fpermissive` (the NXP code calls functions it never declares, which
  CubeIDE's GCC 14 refuses).
- The switches `SDK_OS_FREE_RTOS OSA_USED FSL_OSA_TASK_ENABLE=1
  WIFI_IW610_BOARD_MURATA_2LL_M2 CONFIG_NXP_WIFI_SOFTAP_SUPPORT=1
  CONFIG_5GHz_SUPPORT=0` on every file; the last changes structure layouts in
  shared headers.
- The HAL's `sdio` driver (with the block-transfer fixes the radio needs) and
  `uart` driver (the pin map declares the BLE link's handle).

`wifi_build.json` lists the same stack for the compile gate and
`harness/examples/addstack.py`. The image is 852,032 bytes (867,328 with the
panel), under the boot loader's 1.5 MB limit.

## Tasks

The USB example's `app`, `capture` and `usb` tasks (`setup()` brings the
radio up first; `usb` sends each frame to the attached host, USB or Wi-Fi),
plus the Wi-Fi library's supervisor (`maya_w4_conn`: power saving off, region
lock, power cap) and host server (`maya_w4_vid`), lwIP's `tcp/ip`, and NXP's
driver tasks (priorities 51 to 55). The timer task (`Tmr Svc`) has 2 KB: the
driver's 802.11n reorder flush runs there and needs more than 512 bytes. The
kernel heap is 96 KB; 12.8 KB is left once the radio, camera and NPU are up.

## Behaviour and limits

- **The link carries about 2.4 MB/s,** set by the SDIO bus at the 8.163 MHz
  that passed radiated emissions. At JPEG quality 2 (the default) every mode
  keeps the camera's rate; at quality 4, 720p falls to 48.1 fps. When the
  link is full the encoder waits and skips camera frames; the detector keeps
  its pace.
- **A failed firmware download is not retried:** every later start reports
  no module and the video stays on USB.
- A client stuck in its WPA2 handshake is sent away after 3 s so that it
  joins again (the radio firmware sometimes never runs the handshake); see
  `Middlewares/NeuroN6/MayaW476/README.md`.
- `NIA_HOST_TIMEOUT_MS` (0, wait for ever, as shipped) can send the board to
  the bootloader when no host has connected within that time.
- An assertion in a task is reported and suspends only that task; one in an
  interrupt handler holds the board until the watchdog takes it to the
  bootloader.
- The network line is repeated whenever a console connects.
- No BLE here; `neuron6_bt` and `Examples/BleBeacon_FreeRTOS` show it beside
  Wi-Fi on the same module.
- Not yet tried with Neuro Studio itself: the stream was read by the bench
  scripts, which parse it as Neuro Studio does.

Tested on a board with the OV5640-W carrier: 30.0 fps over Wi-Fi with no gap
in the frame counter, ten inferences a second.

## On a display carrier's panel

With `NIA_LCD` set to 1 in `Core/Inc/neuron6_conf.h` and the TFT-W carrier,
the camera also shows on the 800 x 480 panel with the boxes drawn over it,
while the stream goes on as before. The camera then runs at WVGA, 60 fps, with
that carrier's crossed data lanes. 0 by default.

- The dump pipe writes RGB565 frames straight into layer 0's single 768 KB
  on-chip buffer, at no processor cost; a fast movement can show the join
  between two frames.
- The boxes are layer 1, ARGB4444, double-buffered in PSRAM and swapped at a
  vertical blank. `nia_lcd.c` implements `LCD_Overlay_Nia_From`: a box per
  detection in its class's colour, class and confidence, and the inference
  time, in ST's 16-pixel font.
- The band moves to AXISRAM6, `0x34350000`.
- An extra console line, `[lcd] camera=60 boxes=10 stalls=0 underrun=0 buserr=0`:
  camera frames shown, overlays drawn, draws that waited, display-controller
  errors.

Tested with the TFT-W carrier: 60.0 fps over Wi-Fi, inference 64 to 71 ms,
the panel at 60 camera frames and 10 overlays a second with no underrun.

## Other camera modes

The same four `#define`s as in
[Camera_Nia416Stream_USB_FreeRTOS](../Camera_Nia416Stream_USB_FreeRTOS), whose
README lists each mode's band and JPEG buffer. QVGA cannot feed the network.
