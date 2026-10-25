# A BLE device, under FreeRTOS

A device a phone connects to, pairs with and controls, on the Apache NimBLE
host (`Middlewares/NeuroN6/BleHost`). It advertises as `Neuro-N6` and serves:

| service | what it holds |
|---|---|
| Device Information (`180A`) | maker, model, a serial number from the chip's unique ID, the BSP version as firmware revision |
| Battery (`180F`) | a simulated level (the board has no battery): drops by one every ten seconds, restarts at 100, notified on change |
| Neuro N6 (`e7c70001-8aef-4711-80fa-af6b543319df`) | the LED, the button and the acceleration, below |
| Nordic UART (`6e400001-b5a3-f393-e0a9-e50e24dcca9e`) | what a phone writes to RX (`6e400002-…`) is printed on the USB console and echoed on TX (`6e400003-…`). Writing needs an encrypted link |

Neuro N6 service characteristics, each with a User Description:

| characteristic | UUID | what it does |
|---|---|---|
| LED | `e7c70002-…` | one byte, read and write: bit 0 red, bit 1 green, bit 2 blue. Writing needs an encrypted link, so the first write pairs the phone |
| Button | `e7c70003-…` | one byte, read or notified: 1 while USER is pressed |
| Acceleration | `e7c70004-…` | x, y, z in milli-g, three signed 16-bit values, little-endian, notified ten times a second |

With nRF Connect: scan, connect to `Neuro-N6` and write `07` to the LED; the
phone asks to pair and the LED goes white. Turn on the Button's notifications
and press USER. A UART terminal app (such as nRF Toolbox's UART) gets back
every line it sends once the phone has paired. The console:

```text
[BLE] 'Neuro-N6' advertising from AA:BB:CC:DD:EE:FF
[BLE] connected to 11:22:33:44:55:66, link 1
[BLE] link 1: MTU 247
[BLE] link 1 encrypted, bonded
[N6] LED RGB
[NUS] 6 bytes from link 1:
hello.
```

## Pairing

- LE Secure Connections; the bond is kept, so the phone reconnects encrypted
  without asking again.
- `PAIRING_WITH_PASSKEY` 0 (default): Just Works. At 1 the board prints a
  six-digit passkey on the console for the phone's user, and the LED needs a
  link paired that way.
- Bonds are kept in RAM. After a board reset the phone's bond no longer
  matches and the link fails to encrypt: remove `Neuro-N6` from the phone's
  paired devices and pair again.

## How it runs

`BleHost_Init()` loads the radio firmware (no Wi-Fi network) and starts
NimBLE; `setup()` registers the services; `on_sync()` advertises while there
is room for another connection (up to three); `loop()` samples the button
every 20 ms and the accelerometer every 100 ms. The GAP and GATT callbacks
run in NimBLE's host task.

## Before it runs

The radio's firmware must be in external flash at `0x91E00000`, as for the
Wi-Fi examples (`Camera_Nia416Stream_WiFi_FreeRTOS/README.md`).

## Building it

`ble_build.json` lists what the BLE host adds to the build (NimBLE, Mbed TLS,
the port, include paths, two switches); the CubeIDE project carries the same.
It does not define `DEBUG`: NimBLE names its log functions after their levels,
so a `DEBUG` macro breaks its build, and the port's configuration stops with
an `#error` saying so.

`Middlewares/NeuroN6/BleHost/README.md` describes the port, its tasks, its
limits and the test results.

Tested on a board with the TFT-W carrier against a Windows PC's BLE adapter:
18 of 18 checks passed.
