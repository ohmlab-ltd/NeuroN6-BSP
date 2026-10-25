# Mbed TLS

[Mbed TLS](https://github.com/Mbed-TLS/mbedtls) **3.6.6** as ST delivers it
in STM32CubeN6 v1.4.1: `Middlewares/Third_Party/mbedtls` of that package,
which is ST's [stm32-mw-mbedtls](https://github.com/STMicroelectronics/stm32-mw-mbedtls)
at tag `v3.6.6_20260511` (commit `7ee7e20e`). ST's changes to Mbed TLS are
listed in `st_readme.txt`.

Copied byte for byte: `include/` and `library/`, with `LICENSE` and
`st_readme.txt`, and one test header, `tests/include/test/certs.h`, which
only declares Mbed TLS's test certificates: NXP's HTTP server includes it.
Nothing here is modified.

Two parts of the BSP use it, each with a configuration of its own selected
with `MBEDTLS_CONFIG_FILE`; with either, the files of `library/` that
configuration does not use compile to nothing:

- the Bluetooth host, for pairing: AES, AES-CMAC and elliptic-curve
  Diffie-Hellman on P-256, the arithmetic of LE Secure Connections
  (`Middlewares/NeuroN6/BleHost/include/mbedtls_config_ble.h`);
- TLS over Wi-Fi: TLS 1.2 and 1.3 as server and client, X.509, and the
  board's own key and certificate
  (`Middlewares/NeuroN6/Tls/include/mbedtls_config_tls.h`).

`Examples/BlePeripheral_FreeRTOS` and `Examples/WiFiTls_FreeRTOS` compile the
whole of `library/`, each with its configuration, and `harness/compile.sh`
compiles it as each of them does.

Mbed TLS is under the Apache License 2.0 (`LICENSE`). See
`THIRD-PARTY-NOTICES.md`.
