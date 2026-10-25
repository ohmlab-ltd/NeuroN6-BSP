# The Neuro N6 boot contract

The board's first-stage boot loader (FSBL) runs before the application. Its
source is in `FSBL/` (Apache-2.0; ST's External Memory Manager in it is
SLA0044, see `THIRD-PARTY-NOTICES.md` §5.5). This document says what the
application may rely on at entry, and what it must not touch.

- **The contract holds from FSBL v2.5.0.** v2.0.2 to v2.4.0 read the external flash at 200 MHz with bit errors, which breaks the radio and the NPU silently.
- **Boards ship with v2.6.2.** `FSBL/Neuro-N6-FSBL-v2.6.2-Trusted.bin` is its image, installed with `harness/uploader --fsbl`. `python harness/n6fsbl.py` identifies a board's loader, read-only.
- **`FSBL/` holds v2.6.2's source**, built by every example's `FSBL` project. It builds against this repository's newer HAL, so the result is not byte-identical: put the published image on boards.

## State at application entry

| Concern | State |
|---|---|
| Core clock | PLLs configured, CPU at the shipped frequency. Do not call `SystemClock_Config`. |
| XSPI1 (NOR, 32 MB) | Memory-mapped at `0x90000000`. Normally 200 MHz (prescaler 0) with the DQS delay that speed needs, or 100 MHz (prescaler 1) if a step of that switch failed: work at either. Weights and radio firmware are read from it in place. |
| XSPI2 (PSRAM) | Memory-mapped at `0x70000000`. |
| XSPIM | Both memories on one IO port (`HAL_XSPIM_IOPORT_2`, `MUXEN = 1`). Do not reconfigure either XSPI or the mux. |
| Application image | Copied whole, header included, from the boot slot (A at NOR offset `0x00100000`, or B at `0x01C80000` after an update) to AXISRAM `0x34000000`, and entered at `0x34000400`. |
| Vector table | `SCB->VTOR` points at the application's vectors; `SYSCFG->INITSVTORCR` mirrors it, in RAM that Standby does not keep. |

## Hard limits

- **Image size:** the application must end below `0x34180000`, a 1.5 MB limit with the header. Enforce it with a linker `ASSERT`.
- **Image header:** made by `STM32_SigningTool_CLI -nk -of 0x80000000 -t fsbl -hv 2.3`: a structural header, not a signature. The FSBL takes the vector table `0x400` bytes in, so an image without a header does not run. With an update control block present, a slot must also match the CRC32 recorded for it.
- **Upload path:** USB DFU in the FSBL (`1209:8009`), reached by the 1200-baud touch, a double press of reset, or the application's force-DFU request. There is no SWD route for end users.

## Flash speed, and why the application must not set it

- **Why the delay matters:** at 200 MHz with the DLL's own calibrated DQS delay, mapped reads drop 1-bits silently. The NPU then reads wrong weights, and the radio rejects its firmware.
- **What the FSBL does:** it sets a DQS input delay of fine 16, coarse 3 (`CALSIR`), mid-window. Before jumping it switches to prescaler 0 and reads once so the DLL calibrates, reads the image's first 64 KB, maps the flash, and writes the delay after the mapping's commands. If any step fails it returns to prescaler 1.
- **Unchecked at boot:** the window was measured on one board at room temperature, and v2.6.2 does not test it at boot.
- **Do not reconfigure XSPI1.** The delay holds only if written after the command that uses it, and a wrong value corrupts every later read with no error.

## Flash map (NOR window `0x90000000`)

| Region | Address | Size |
|---|---|---|
| FSBL | `0x90000000` | 1 MB, reserved |
| Application, slot A | `0x90100000` | up to 1.5 MB (to `0x90280000`) |
| Update control block, two copies | `0x90280000` | 2 x 4 KB |
| Asset store (stored settings), two copies | `0x90282000` | 2 x 4 KB, region reserved to `0x90300000` |
| Neural network weights pool | `0x90300000` | 25.5 MB (to `0x91C80000`) |
| Application, slot B (updates) | `0x91C80000` | 1.5 MB (to `0x91E00000`) |
| Radio (IW610) firmware | `0x91E00000` | about 1 MB |

- **Erase rule:** the loader refuses an upload whose erase crosses from one region into the next. The control block, the asset store and their reserve count as one region.
- **Host tools:** `harness/n6flash.py` writes only slot A; `harness/n6weights.py` refuses anything below the weights pool.
- **Slot uploads:** an upload to the start of slot A (or B, with a control block) makes it the running, confirmed slot.
- **Staged updates:** otherwise the loader writes slot B and the control block when it applies an update staged in PSRAM. This BSP stages settings, never image updates.
- **Memory pools:** `Utilities/npu/NeuroN6_MemPool.mpool` gives the weights pool as above. NIA-416's weights are 8.2 MB.

## RAM map

| Region | Address | Size |
|---|---|---|
| Application RAM (FLEXRAM + AXISRAM1/2) | `0x34000400` | 2047 KB |
| FSBL resident region | `0x34180000` | reserved |
| NPU RAM (AXISRAM3+) | `0x34200000` | 1792 KB |
| External PSRAM | `0x70000000` | 32 MB. Network pool `0x71000000` to `0x71DFFFFF`; the top 2 MB from `0x71E00000` is the loader's staging area |

## Backup registers

| Register | Owner | Use |
|---|---|---|
| 0, 3 to 10 | application | its own records |
| 1, 2 | both | time of the last start; the key that asks for the bootloader |
| 11 to 13 | none | |
| 14 | boot loader | reserved; applications must not write it |
| 15, 16 | both | result of a settings write; the request for one |
| 17 to 19 | none | |
| 20, 21, 22 | both | update request, confirmation, result |
| 23 to 31 | application | its crash record |

An upload that fills a slot from its start clears registers 20 and 16, so an
update or settings write the application staged is not applied over it.

## Clock policy

More than one FSBL build is in the field. BSP modules never hard-code a derived
clock: they read kernel clocks back at init (`HAL_RCC_GetPCLK1Freq`,
`HAL_RCCEx_GetPeriphCLKFreq`) and return `BSP_ERROR_FEATURE_NOT_SUPPORTED` for a
rate they cannot reach.

## Resource isolation (RIF)

The STM32N6 drops transactions from unconfigured bus masters silently. Every BSP
module that enables a DMA-capable peripheral makes its own RIF grants at init.
An application with its own isolation setup must run it before BSP init and keep
the BSP's grants.

## Known issues in v2.6.2

These are open findings in `harness/bench/hosttest/fsbl`, which runs the
loader's source on a simulated board.

- A slot is entered without checking its vectors: a blank or foreign slot faults instead of returning to the bootloader.
- If the update control block cannot be read, slot A is booted as if never updated.
- After a fallback, the active-slot record is not updated, so a later update can overwrite the slot the board falls back on.
- A staged image is not checked to be an application for its slot.
- A write cut short can leave a slot the loader still enters (the vector page is not written last).
- Power cycles count as tries: an update that never confirms is given up at the fourth start.
- An image on trial does not run under the watchdog, and a trial that ends in the bootloader stays there.
- A settings write cut short can damage the stored settings.
- A staged image longer than `N6_OTA_STAGE_CLEAR_OF_ASSET` (`0x17FFE0`) bytes overlaps the settings' stage. The BSP's `n6_asset_stage()` refuses while such an image is armed.
- Over USB:
  - the loader's own flash is not locked;
  - data is not held to an accepted erase;
  - a refusal does not stop later commands for the same image;
  - an erase does not first check the flash's mode;
  - an upload the control block could not record is not reported;
  - the other-speed configuration descriptor has the wrong type (USB 2.0 §9.6.4 requires 7).
