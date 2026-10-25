# Neuro N6 uploader

The command-line DFU uploader for the Neuro N6's bootloader
(`neuro-n6-uploader-cli.py` and `utils/`). It is Ohm Lab's, under the MIT
licence (`LICENSE.txt`).

It speaks the bootloader's DFU protocol over its USB serial port;
`utils/dfu.py` describes the wire format. A v2 bootloader also reports its
capabilities and the CRC32 of any region.

The harness uses it in two places, neither writing below `0x90100000`, where
the bootloader lives:

- `harness/n6flash.py` runs `neuro-n6-uploader-cli.py` to write an
  application image, always to `0x90100000`, the application slot;
- `harness/n6weights.py` uses `utils/dfu.py` to check a network's weights by
  CRC32, and to write them to their own region when asked.

Every session first brings the bootloader back to idle: zeros complete a
frame a broken-off session left half sent, then `DFU START` at `0x00300000`
ends an image left half written and erases nothing. With more than one board
connected, `--port` names the one to upload to (its application's or its
bootloader's port); the uploader does not choose.

## Writing the boot loader

The boot loader's own flash is written only when asked: the CLI refuses an
address below the application slot without `--fsbl`, and with it takes only a
boot loader image, which it puts at `0x90000000`:

```bash
python harness/uploader/neuro-n6-uploader-cli.py --bin bootloader_Trusted.bin --fsbl
```

`DfuSession.program()` in `utils/dfu.py` needs `fsbl=True` for the same.
Both refuse a boot loader's image (`<name>_FSBL_Trusted.bin`, which every
example builds) for an application slot. v2.6.2 itself neither locks its own
flash nor checks that what is written there is a boot loader: see
[docs/BOOT-CONTRACT.md, "Known issues in v2.6.2"](../../docs/BOOT-CONTRACT.md#known-issues-in-v262).
A wrong image there, or power lost while it is written, leaves a board that
no longer starts and that no tool here can reach.

`harness/bench/hosttest/fsbl` runs both against the boot loader's own source
on a simulated board.
