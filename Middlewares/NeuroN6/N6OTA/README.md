# Neuro N6 asset store

Small, persistent data a host writes to the board: a calibration, a table,
and typed records beside it (the Wi-Fi settings Neuro Studio stores, the
board's own access point password, `MayaW476_DevicePassword()`). It survives
a power cycle, and writing one record keeps every other.

- The application never writes flash: it stages the data in PSRAM, arms a
  backup register and resets. The FSBL verifies the stage, programs it to two
  4 KB copies at `0x90282000`, and reports what it did.
- A read takes the newest copy that verifies, so an interrupted write never
  looks like a good one; a failed check re-reads the flash, not the cache.
- Two stages before one reset are applied together. Every write is numbered
  above the store, up to `N6_ASSET_VERSION_MAX`; a store already there
  refuses the next stage.
- `n6_asset_apply()` clears the bootloader's double-press key before it
  resets, so the reset is not taken for a second press of the reset button.

| file | what it is |
|---|---|
| `n6_asset.{h,c}` | the store: `n6_asset_read()` and `n6_asset_stage()` for the asset, `n6_asset_record_read()` and `n6_asset_record_stage()` for a typed record, `n6_asset_apply()` to reset into the FSBL, `n6_asset_pending()` while a stage waits. It delivers no firmware update; those are installed over USB. |
| `n6_asset_host.{h,c}` | the AST0, ASR0, AST1 and AST2 commands (`n6_host_contract.h`), one at a time as the USB task takes them. Pass only commands received over the USB cable, never over the network. |
| `n6_ota_layout.h` | the flash layout and backup registers shared with the FSBL: `FSBL/Core/Inc/n6_ota_layout.h`, byte for byte, which `harness/compile.sh` checks. |

It needs FreeRTOS (a read or a stage holds the scheduler for under a
millisecond), the host contract from `Middlewares/NeuroN6/PostProcess`,
backup-domain access (every example's `main()` enables it), and a boot loader
that applies a staged asset, as the Neuro N6 FSBL does.
`harness/bench/hosttest` runs the store's record container on a PC.

## Terms

`n6_asset.{h,c}` are Ohm Lab Ltd's, under the MIT licence
(`LICENSE-MIT.txt`). `n6_asset_host.{h,c}`, the host's commands for the
store as the USB task takes them, carry the same terms.
`n6_ota_layout.h` is the boot loader's, under the Apache License 2.0
(`FSBL/LICENSE`).
