# Host-link bench tools

Checks for the host link, talking to a board as Neuro Studio does: over the
USB vendor interface, over Wi-Fi and over the console.

| tool | what it does |
|---|---|
| `n6bench.py` | `stream` and `cmd` read the stream and send commands over USB or TCP; `probe` finds boards by broadcast; `wifitest` runs the two-way Wi-Fi checks (`--usb` adds the USB hand-over); `usbtest` the same stream checks over USB; `wifiset` stores Wi-Fi settings on a board over USB, reading any passphrase or link key from an environment variable so it is never on a command line. |
| `n6wifiapi.py` | Drives `Examples/WiFiApi_FreeRTOS` through the Wi-Fi API: the access point, station mode, stored settings and the link key. Windows. |
| `n6radiotest.py` | Checks the radio diagnostics and BLE scanning and advertising on `Examples/WiFiApi_FreeRTOS`. Windows; the BLE phases need `n6radio.exe`. |
| `n6btpower.py` | Has `Examples/WiFiApi_FreeRTOS` read back its BLE transmit power (advertising, extended advertising on each PHY, a power command asking 20 dBm, a connection from this PC) and checks every level in use against the 8 dBm cap. Windows; needs `n6radio.exe`. |
| `n6radio.cs`, `build_n6radio.py` | This PC's radios as fixtures, through the Windows Runtime: BLE scan, GATT checks and a Wi-Fi Direct group owner for station mode. Built with the .NET Framework's own compiler into the temp folder; nothing on the PC is changed beyond the run. |
| `hosttest/` | Host tests, against this repository's copies, of the asset store's record container and SHA-256/HMAC, the BLE transmit power policy, the Lepton's video parser, the thermal module's power-up, boot wait and video restarts, and the YOLOv8 segmentation metadata: `sh harness/bench/hosttest/run.sh` (on Windows, from WSL). |
| `hosttest/fsbl/` | The boot loader's own source on a simulated board, the uploader against it, and its USB descriptors: `sh harness/bench/hosttest/fsbl/run.sh` (Linux, or WSL). See its README. |

With `--any-app`, `wifitest` and `usbtest` check any Neuro N6 streaming
application; the checks that need the bench firmware are skipped.

A Wi-Fi check uses a temporary profile and a passphrase made up for the run,
deleted afterwards. Its servers run in `python.exe`, which the firewall must
let in on public networks.

## Terms

Ohm Lab Ltd's, under the MIT licence (`LICENSE-MIT.txt`).
