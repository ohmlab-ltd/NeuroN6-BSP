# TLS over Wi-Fi, under FreeRTOS

An HTTPS server on the board and an HTTPS client from it, over a WPA3 or WPA2
network: NXP's HTTP server and Mbed TLS, brought up by
`Middlewares/NeuroN6/Tls`.

- **The network:** the one stored on the board (Neuro Studio's Wi-Fi
  settings, or `harness/bench/n6bench.py wifiset`), or else an access point,
  `NeuroN6-TLS`, WPA3 with WPA2 beside it, with the board's own random
  password, which the console shows. To join a network instead, set
  `WIFI_MODE` to `WIFI_MODE_STA` and `WIFI_STA_SSID` and `WIFI_STA_PSK` in
  `main.c`; `WIFI_SECURITY` allows WPA3 alone (`WIFI_SECURITY_WPA3_SAE`) or
  WPA2 alone.
- **The board's identity:** at every start a new ECDSA P-256 key, and a
  self-signed certificate naming its mDNS name (`neuron6-xxxxxx.local`) and
  address. The key never leaves RAM. The board answers to the mDNS name only
  while it streams to Neuro Studio, which this example does not, so reach it
  by address. The console shows the certificate and its SHA-256 fingerprint.
- **The server**, on port 443 (nothing listens on port 80):

  | path | what it does |
  |---|---|
  | `/` | a page with the board's state, updated every two seconds, the LED's eight colours, and a button that runs the client |
  | `/status.cgi` | the state as JSON: name, address, network, uptime, button, LED, memory, the certificate's fingerprint, the Mbed TLS version, the clock and the client's last result |
  | `/led.cgi` | POST `rgb=0` to `rgb=7`: bit 0 red, bit 1 green, bit 2 blue |
  | `/fetch.cgi` | POST: run the client again |

- **The client:** once the board has the time from the network, it fetches
  `https://valid-isrgrootx1.letsencrypt.org/` and accepts the server only if
  its certificate chains to ISRG Root X1 or X2, is in date and names that
  host. It reports the TLS version, cipher suite, server name, HTTP status and
  reply length. USER or the page's button runs it again. Without the time it
  does not try, and the board's own access point has no internet, so the
  client needs a station on a network with internet access.

The console, on the access point:

```text
=== WiFiTls: an HTTPS server and client over Wi-Fi, 'b' for the bootloader ===
[TLS] Mbed TLS 3.6.6
[WiFi] access point "NeuroN6-TLS" (WPA2/WPA3)  password: xxxx-xxxx-xxxx-xxxx
...
[WiFi] status 7, address 192.168.10.1
[TLS] new key, ECDSA P-256, for neuron6-xxxxxx.local and 192.168.10.1
[TLS] certificate SHA-256 AB:CD:...
-----BEGIN CERTIFICATE-----
...
-----END CERTIFICATE-----
[HTTPS] serving https://192.168.10.1/
[HTTPS] client: not run: the time is not known, so certificates cannot be checked
```

and as a station on a network with internet access:

```text
[TLS] time 2026-09-25 14:03:12 UTC, from the network
...
[HTTPS] GET https://valid-isrgrootx1.letsencrypt.org/
[HTTPS] TLSv1.3 TLS1-3-AES-256-GCM-SHA384, server CN=valid-isrgrootx1.letsencrypt.org, certificate verified
[HTTPS] client: https://valid-isrgrootx1.letsencrypt.org/: HTTP 200, ... bytes, TLSv1.3 ..., ... ms
```

The key lines are printed again whenever a console connects.

## Reaching the server

Only the board vouches for its certificate, so a browser warns that the
connection is not private. Compare the certificate's SHA-256 fingerprint with
the console's before going on: if they match, the connection is with this
board. A program can pin the certificate: save the console's PEM block as
`n6.pem`, then

```bash
curl --cacert n6.pem https://192.168.10.1/status.cgi
```

or in Python `ssl.create_default_context(cafile="n6.pem")`. The certificate
changes at every start.

## How it runs

`N6Tls_Init()` starts the RNG, Mbed TLS's mutexes and PSA crypto; a station
gets the time by NTP every hour (`MayaW476_GetTime()`, `N6Tls_SetTime()`);
`N6Tls_MakeIdentity()` makes the key and certificate, NXP's server starts with
them (`HTTPSRV_init()`, four sessions plus two waiting connections) and the
example wipes its copy of the key. Each session is a server task with a 10 KB
stack; the handshake runs in the listener (12 KB) and has 10 s to finish
(`HTTPSRV_CFG_TLS_HANDSHAKE_TIMEOUT` in `tls_build.json`). An idle connection
is closed after a second, so the page's two-second poll opens a new one.

## Before it runs

The radio's firmware must be in external flash at `0x91E00000`, as for every
Wi-Fi example (`Camera_Nia416Stream_WiFi_FreeRTOS/README.md`).

## Building it

`tls_build.json` lists what TLS adds to `wifi_build.json`'s Wi-Fi stack:
Mbed TLS, NXP's HTTP server, the glue, their include paths and switches (the
Mbed TLS configuration, the server settings, a 192 KB kernel heap for the
sessions' stacks) and the HAL's RNG driver, which
`Core/Inc/stm32n6xx_hal_conf.h` enables. The CubeIDE project carries the same.

The page is `web/index.html`; `Core/Src/httpsrv_fs_data.c` is generated from
it by NXP's `mkfs.pl`:

```bash
cd Examples/WiFiTls_FreeRTOS
perl ../../Middlewares/lwip/src/apps/httpsrv/mkfs/mkfs.pl web
mv httpsrv_fs_data.c Core/Src/
```

`Middlewares/NeuroN6/Tls/README.md` describes the port, the Mbed TLS
configuration and its limits.

Tested on the board's access point with a PC: 13 of 13 checks, with TLS 1.3
and 1.2; with WPA3 and WPA2 clients, 14 of 14. The client's internet fetch
has not yet run on the board.
