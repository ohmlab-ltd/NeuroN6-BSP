# Neuro N6 Wi-Fi

Wi-Fi for the Neuro N6's u-blox MAYA-W476 module (NXP IW610), in C. It brings
the radio up over SDIO, loads its firmware, joins a network or hosts an access
point, and serves Neuro Studio on the board's own access point: a Wi-Fi host
gets the same stream and live controls as over USB, and finds the board with a
UDP probe or over mDNS.

| file | what it is |
|---|---|
| `MayaW476.{h,c}` | the Wi-Fi API: join, host, stored settings, the board's own password, the host server (`MayaW476_Start()`), the firmware alone for BLE (`MayaW476_RadioStart()`), addressing, scanning, time, power save, link key |
| `maya_w4_board/maya_w4_hw.c` | control pins, SDMMC1, SDIO probe, firmware download; owns the SDMMC1 interrupt |
| `maya_w4_board/maya_w4_connect.c` | the supervisor task: every link change, region lock, power cap, station rejoin, handshake keeper |
| `maya_w4_board/maya_w4_video_wifi.c` | the Neuro host server: discovery, one host at a time, link-key proof, hand-over to the USB task |
| `maya_w4_board/maya_w4_mdns.c` | mDNS for `<hostname>.local` and `_neuron6._tcp` |
| `maya_w4_board/maya_w4_sha256.{h,c}` | SHA-256 and HMAC-SHA256, for the link key |
| `maya_w4_board/maya_w4_txpower.{h,c}` | the 8 dBm cap on every 2.4 GHz channel, read back at every start |
| `maya_w4_board/wifi_config.h` | the NXP driver's configuration: 2.4 GHz only, no 802.11ac or ax, no power saving |
| `maya_w4_board/wifi_bt_config.{h,c}`, `maya_w4_pinmap.h`, `maya_w4_creds.h` | power sequence, pin map, settings bridge |
| `maya_w4_board/wifi_bt_module_config.h`, `wlan_txpwrlimit_cfg_murata_NH.h` | NXP's module configuration and power tables |
| `sdio_port/stm32n6_sdio_host.{h,c}` | the SDIO host layer the NXP driver calls |
| `NeuroN6_FrameQueue.h` | the interface between the capture task, the USB task and this library |
| `firmware/sduartspi_iw610.bin.se` | the IW610 firmware (Wi-Fi and BLE); `firmware.json` gives its flash address and checksums |

The NXP driver (`Middlewares/NXP`) has three changes marked "Ohm Lab": in
`wlcmgr/wlan.c` and `wifidriver/wifi.c`, a disconnect during a connect scan
returns the scan lock, and a scan the firmware never finishes can be cleared;
in `dhcpd/dhcp-server.c`, the access point leases .10 to .99 beside its own
address.

## What an application provides

- **FreeRTOS** configured as in `Utilities/freertos`; the driver's timers need
  its 2 KB timer task.
- **`NeuroN6_DbgPrint()`**, and **`NeuroN6_NetAttach()` and
  `NeuroN6_IpAnnounceFrame()`** (`NeuroN6_FrameQueue.h`).
  `Examples/Camera_Nia416Stream_WiFi_FreeRTOS` streams to the host;
  `Examples/WiFiApi_FreeRTOS` only holds it.
- **The asset store** (`Middlewares/NeuroN6/N6OTA`) and
  `Middlewares/NeuroN6/PostProcess/n6_host_contract.h`, for stored settings.
- **The HAL's `sdio` and `uart` modules,** with the SDIO driver from
  `Utilities/hal`.
- **The firmware in external flash** at `0x91E00000`: `harness/n6weights.py
  Middlewares/NeuroN6/MayaW476/firmware/firmware.json` checks it, and writes
  it with `--write`.
- **FSBL v2.5.0 or later.** The firmware is read in place, and one wrong bit
  makes the radio reject it.

`wifi_build.json` in each Wi-Fi example lists the flags and include paths.

## Security

- **WPA2 and WPA3 only** (`WIFI_SECURITY_WPA2`, `WIFI_SECURITY_WPA3_SAE`,
  `WIFI_SECURITY_WPA2_WPA3`), with a passphrase of 8 to 63 characters, or 64
  hex digits under WPA2. Anything else fails with one console line. Stored
  settings with no key cannot join; hosting then uses the board's own
  password.
- **The board's own password** (`MayaW476_DevicePassword()`) is
  `xxxx-xxxx-xxxx-xxxx` from the hardware random number generator, never from
  the chip's ID. The first call stores it in asset record 3 and restarts the
  board once; if it cannot be stored, it lasts until the next restart.
- **The host server runs on the board's own access point only:** TCP 5000,
  discovery on UDP 5000 and mDNS start only from
  `MayaW476_Start(TRANSPORT_WIFI_SOFTAP)`. A joined network or
  `TRANSPORT_WIFI_STA` gets `-3`, and the stream stays on USB.
- **Link key:** up to three connections, one per address, may prove it at
  once, so a silent or reconnecting peer cannot lock out a host that knows the
  key.

## Limits and known issues

- **WPA3 uses hunting and pecking only.** With hash-to-element offered a
  Windows PC could not associate, so a network that insists on it is not
  joined. A station on a WPA3 access point is untested.
- **A client can stall in its handshake.** The firmware sometimes accepts an
  association and never runs the four-way handshake. The supervisor sends away
  a client still in its handshake 3 s after joining (`keep_softap_clients`),
  and it joins again; the host server's queue of two keeps a half-open
  connection from blocking the next host. Tested: a Windows PC reached the
  host server on 44 joins of 44.

## Regulatory settings

The wireless build was EMC-tested with these settings. None is a tuning knob:

- **2.4 GHz only.** `CONFIG_5GHz_SUPPORT=0` must reach every file that
  includes the driver's headers, because it changes structure layouts.
- **World Wide Safe Mode** as the region.
- **8 dBm conducted power,** capped in the firmware's per-channel table and
  read back.
- **Access point channels 1 to 11,** 20 MHz wide. Anything above 11 falls
  back to 6.
- **The SDIO bus clock at 8.163 MHz,** PLL1 / 7, then / 14: both dividers
  odd, so no harmonic lands on 200 MHz, never gated, with the pins at high
  slew.

## Terms

The library's own code is Ohm Lab Ltd's, under the MIT licence
(`LICENSE-MIT.txt`); `MayaW476.{h,c}` carry the same terms.
`wifi_bt_module_config.h` is NXP's, under BSD-3-Clause, and
`wlan_txpwrlimit_cfg_murata_NH.h` is NXP's, under the MIT licence its header
carries.

The firmware is NXP's proprietary software, under NXP's LA_OPT licence
(`firmware/LICENSE-LA_OPT.txt`). It is not covered by this repository's
licence. See `THIRD-PARTY-NOTICES.md` before redistributing it.
