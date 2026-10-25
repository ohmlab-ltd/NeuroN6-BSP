# The Wi-Fi API, from the USB console, under FreeRTOS

A command shell for everything `MayaW476.h` offers: hosting and joining
networks, addressing, scanning, the radio off and IEEE power save, name
lookup and network time, the settings a host stored on the board, the link
key, and lwIP's BSD sockets for the application's own TCP and UDP. One
command a line; every answer line starts `[api]`, and each command ends with
`[api] ok <cmd>` or `[api] fail <cmd> <why>`. `stat` prints the whole link:

```text
[api] stat status=8 ip=192.168.10.1 mask=255.255.255.0 gw=192.168.10.1 dns=0.0.0.0 ssid=N6API-XXXX rssi=0 ch=6 bssid=AA:BB:CC:DD:EE:FF mac=AA:BB:CC:DD:EE:FF host=neuron6-xxxxxx ps=0
```

- The commands are listed at the top of `Core/Src/main.c`; each is one or two
  library calls, so the file doubles as a reference.
- `bt`, `btinfo`, `btscan` and `btadv` bring BLE up on the same module.
  `btpower` reads the controller's transmit power back, which
  `harness/bench/n6btpower.py` checks against the 8 dBm cap.
- Credentials arrive as commands; the example holds none, and the only key it
  prints is the board's own password, when asked (`pass`).

## Before it runs

The radio's firmware must be in external flash at `0x91E00000`, as for every
Wi-Fi example (`Camera_Nia416Stream_WiFi_FreeRTOS/README.md`).

## Checking it

`harness/bench/n6wifiapi.py` drives it:

```bash
python harness/bench/n6wifiapi.py --port PORT --phases pass,ap,sta,stored,link,wpa3
```

| phase | covers |
|---|---|
| `pass` | the board's own password: stored once, stable across restarts, new once removed, and an access point hosted with it (`ap SSID -`) |
| `ap` | refused security modes and short passphrases; then hosting, addressing, TCP and UDP both ways, broadcast and multicast, network time, name lookup, discovery, mDNS and DNS-SD, a scan while hosting, `MayaW476_Config()`, disconnect and radio off. The Neuro host server answers nothing until `wstart 1` |
| `sta` | joining the Wi-Fi Direct group owner `harness/bench/n6radio.cs` hosts on the PC (2.4 GHz): DHCP and fixed address, the socket checks, power save, a missing network, a lost link. `wstart` is refused there |
| `stored` | settings stored over USB, then `MayaW476_BeginStored()`, a user asset kept beside them, and removal. The board restarts at each write, as the FSBL writes the store |
| `link` | the link key, stored with the settings |
| `wpa3` | WPA3-Personal (`ap SSID KEY CH wpa3`, and `wpa2-wpa3`): SAE joins, WPA2 clients and wrong passwords refused, a 100-character password refused, WPA3 stored and hosted, and the board as a WPA2/WPA3 and WPA3 station |

`--no-usb-store` leaves out what writes stored settings over USB (the stored
phase, and the stored parts of link and wpa3). The test servers run in
`python.exe`, which the firewall must let in on public networks. Network time
needs UDP port 123 on the PC; while Windows' time service holds it, the check
is reported as not testable.

## Notes

- A client that resets the board's TCP server connection ends it, and the
  server takes the next. lwIP reports a reset only once, so the example
  detects one by the lost peer: a send that takes nothing or a receive that
  gets nothing.
- The Wi-Fi driver runs its timers, including the 802.11n receive-reorder
  flush, on the FreeRTOS timer task, which has a 2 KB stack for it.

Tested on a board with the OV5640-W carrier and FSBL v2.6.1:
`--phases ap,sta,stored,link,wpa3` passed 115 of 115. Not covered: a station
rejoining after its network returns, and a join with a 100-character WPA3
password (Windows will not store such a profile).
