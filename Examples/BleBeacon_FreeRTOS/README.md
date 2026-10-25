# A BLE beacon, under FreeRTOS

Advertises a non-connectable beacon named `Neuro-N6`, with a counter in its
manufacturer data that goes up once a second. A phone scanner such as nRF
Connect shows the name and the changing bytes. The console says where it is
advertising from:

```text
[BLE] beacon 'Neuro-N6' from AA:BB:CC:DD:EE:FF
```

## How it runs

The MAYA-W476 runs Wi-Fi and BLE from one firmware image, loaded over SDIO.

1. `MayaW476_RadioStart()` powers the module and downloads the firmware
   without starting a Wi-Fi network (it does nothing if Wi-Fi is running).
2. `BSP_BT_Init()` opens the controller's HCI link on USART6 with RTS/CTS. It
   shares the module's power with the Wi-Fi library and never power-cycles a
   running module.
3. `BSP_BT_Command()` sends one HCI command and waits for its Command
   Complete: HCI_Reset, HCI_Read_BD_ADDR, then the LE advertising parameters
   (non-connectable, 100 ms, all three channels), the advertising data (flags,
   name, manufacturer data) and advertising enable.

`loop()` updates the advertising data once a second. `USART6_IRQHandler` must
call `BSP_BT_IRQHandler()`, which fills the ring `BSP_BT_Receive()` reads.

The BSP carries the HCI transport only; a connectable device with GATT
services needs a host stack (see `Examples/BlePeripheral_FreeRTOS`).

## Before it runs

The radio's firmware must be in external flash at `0x91E00000`, as for the
Wi-Fi examples (`Camera_Nia416Stream_WiFi_FreeRTOS/README.md`).

Tested on a board with the OV5640-W carrier: a PC's BLE adapter
(`harness/bench/n6radio.cs scan --name Neuro-N6`) saw the beacon,
non-connectable, with manufacturer data `ffff:4e36nnnn` counting once a second.
