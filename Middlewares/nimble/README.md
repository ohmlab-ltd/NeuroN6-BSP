# Apache NimBLE

The host half of [Apache NimBLE](https://github.com/apache/mynewt-nimble)
**1.10.0**, tag `nimble_1_10_0_tag`, copied byte for byte from GitHub's source
archive of that tag (SHA-256
`ceda6114526dcdb8cf1681cd5b87648c8d45014c6e407e14c61f85313241dd07`). Its
port to the Neuro N6, the configuration included, is in
`Middlewares/NeuroN6/BleHost`.

| folder | what it is |
|---|---|
| `nimble/include/` | definitions the host and its transport share |
| `nimble/host/include/`, `nimble/host/src/` | the host: GAP, GATT client and server, ATT, L2CAP, the security manager |
| `nimble/host/util/` | the address helper applications call at start-up |
| `nimble/host/services/gap/`, `gatt/`, `dis/`, `bas/` | the GAP and GATT services every device carries, Device Information and Battery |
| `nimble/host/store/config/` | the bond store, kept in RAM in this port |
| `nimble/transport/` | the host side of the HCI transport, which the port's transport plugs into |
| `porting/nimble/` | the layer that stands in for Mynewt: buffers, memory pools, `nimble_port_init()` |
| `porting/npl/freertos/` | NimBLE's operating-system layer on FreeRTOS |
| `LICENSE`, `NOTICE` | the Apache License 2.0 and the project's notice |
| `nimble/syscfg.yml`, `nimble/host/syscfg.yml`, `nimble/transport/syscfg.yml` | what each setting in the port's `syscfg.h` means |

Left out: the controller (the MAYA-W476 has its own), the other services,
mesh, the other operating-system layers and the example ports. Five files of
the folders above are left out too:

- `porting/nimble/include/syscfg/syscfg.h`, the default configuration: the
  port supplies its own, derived from it;
- `porting/npl/freertos/include/nimble/nimble_npl_os_log.h`, the FreeRTOS
  layer's log output, which prints every level: the port supplies its own,
  which honours the configured level;
- `porting/nimble/src/hal_timer.c`, `os_cputime.c` and `os_cputime_pwr2.c`
  with their headers, which only the controller uses;
- `nimble/host/src/ble_gatts_lcl.c`, which prints the GATT table on Mynewt's
  console. NimBLE's own generic port (`porting/examples/dummy`) leaves it out
  for the same reason.

Nothing here is modified. `Examples/BlePeripheral_FreeRTOS` compiles every
file, and `harness/compile.sh` compiles them as that example does.

NimBLE is under the Apache License 2.0 (`LICENSE`); `NOTICE` must travel with
any redistribution. See `THIRD-PARTY-NOTICES.md`.
