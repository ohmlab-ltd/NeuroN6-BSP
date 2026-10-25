# Neuro N6 BLE host

Apache NimBLE's host (`Middlewares/nimble`, unmodified), talking to the
MAYA-W476's BLE controller over the `neuron6_bt` HCI link. It adds
connections, GATT services and clients, pairing and bonding. Applications use
NimBLE's own API (`host/ble_hs.h`), and NimBLE's documentation applies.

| file | what it is |
|---|---|
| `ble_host.{h,c}` | `BleHost_Init()`, `BleHost_Start()`, `BleHost_GetStats()` |
| `ble_hci_transport.c` | NimBLE's transport over `BSP_BT_Send` and `BSP_BT_Receive` |
| `ble_host_int.h` | shared by the two files above |
| `include/syscfg/syscfg.h` | NimBLE's configuration for this board |
| `include/nimble/nimble_npl_os_log.h` | NimBLE's log: warnings and errors on the USB console |
| `include/mbedtls_config_ble.h` | Mbed TLS cut down to what pairing uses |

## Using it

```c
#include "ble_host.h"
#include "host/ble_hs.h"

BleHost_Init();                         /* from a task: firmware, link, NimBLE */
ble_hs_cfg.sync_cb = on_sync;           /* start advertising or scanning here  */
ble_svc_gap_init();                     /* the services, NimBLE's and yours    */
ble_gatts_count_cfg(my_services);
ble_gatts_add_svcs(my_services);
BleHost_Start();                        /* the host task; on_sync follows      */
```

`Examples/BlePeripheral_FreeRTOS` is a complete application. The build needs:

- the NimBLE, Mbed TLS and host-port sources and include paths listed in
  `Examples/BlePeripheral_FreeRTOS/ble_build.json`, and its two switches:
  `MBEDTLS_CONFIG_FILE="mbedtls_config_ble.h"` and
  `NIMBLE_NPL_OS_EXTRA_INCLUDE="stm32n6xx.h"`;
- the Wi-Fi stack of `wifi_build.json`: the Wi-Fi library loads the radio's
  combined Wi-Fi and BLE firmware (`MayaW476_RadioStart()`). No Wi-Fi network
  is started;
- FreeRTOS with timers on (NimBLE's timeouts are FreeRTOS timers);
- `USART6_IRQHandler` calling `BSP_BT_IRQHandler()`;
- no `DEBUG` macro, which CubeIDE's Debug configuration defines: it turns
  `BLE_EATT_LOG_DEBUG` into `BLE_EATT_LOG_1` and the link fails. `syscfg.h`
  stops the build with an `#error`.

## How it runs

| task | stack | priority | what it does |
|---|---|---|---|
| `ble_host` | 6 KB | idle + 2 | NimBLE's event queue, GAP and GATT callbacks, pairing's P-256 arithmetic |
| `ble_hci` | 2 KB | idle + 3 | moves each packet from the link to the host |
| FreeRTOS timer task | the kernel's | the kernel's | NimBLE's timeouts |

- Packets to the controller are sent in the calling task, one at a time under
  a lock. A data packet (up to 261 bytes, one more than
  `BSP_BT_MAX_PACKET_SIZE`) goes straight from NimBLE's buffers.
- When event buffers run out, advertising reports are dropped; other packets
  wait for a buffer. Drops are counted by `BleHost_GetStats()`.

## The configuration

`include/syscfg/syscfg.h` is NimBLE 1.10.0's generated default configuration;
each change is marked `Neuro N6`: this port's transport, up to three
connections, extended advertising, bonding and LE Secure Connections, no
isochronous channels, device name `Neuro-N6` (the application sets Device
Information's serial number and firmware revision), pairing's random numbers
from the controller (`LE Rand`), bonds in RAM, and warnings and errors only in
the log. Every value is guarded by `#ifndef`, so a `-D` switch can change it.

Extended advertising reaches BLE 5 advertisers; the legacy calls
(`ble_gap_adv_start()`, `ble_gap_disc()`, `ble_gap_connect()`) still work, and
scan reports arrive as `BLE_GAP_EVENT_EXT_DISC`. A connection also ends the
advertising set, so `BLE_GAP_EVENT_ADV_COMPLETE` follows
`BLE_GAP_EVENT_CONNECT`: restart advertising from the former, not the connect
event, or every later start is refused as Command Disallowed.

Pairing's cryptography is Mbed TLS's (`Middlewares/mbedtls`), cut down by
`include/mbedtls_config_ble.h` to AES, AES-CMAC and ECDH on P-256. It
allocates from the FreeRTOS heap.

Tested on a Neuro N6 with the TFT-W carrier against a Windows 11 PC: 57 of 60
checks passed; the other three record what Windows does.

## Limits

- **Bonds are kept in RAM.** A reset forgets them while the peer keeps its
  half: remove the board from the peer's paired devices and pair again.
  Until then Windows reconnects several times a second and never pairs
  again. NimBLE's persistent store needs Mynewt's configuration system, which
  this port does not carry.
- **The HCI link runs at 115200 baud**, `neuron6_bt`'s default: at most
  11.5 KB/s each way, headers included. Measured: 4.8 KB/s from the board in
  253-byte notifications, 1.8 to 3.9 KB/s to it in writes without response.
- **No periodic advertising and no isochronous channels.** One advertising
  set, the one the legacy calls use.
- FreeRTOS only.

## Licence

`ble_host.{h,c}`, `ble_host_int.h`, `ble_hci_transport.c`,
`include/nimble/nimble_npl_os_log.h` and `include/mbedtls_config_ble.h` are
Ohm Lab Ltd's, BSD 3-Clause, as the rest of the BSP.
`include/syscfg/syscfg.h` is derived from NimBLE's and stays under the Apache
License 2.0, with a notice of the changes in its header. NimBLE and Mbed TLS
keep their own licences; see `THIRD-PARTY-NOTICES.md`.
