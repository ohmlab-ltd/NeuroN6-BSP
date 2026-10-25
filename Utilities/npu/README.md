# The board's memory, for ST Edge AI

`NeuroN6_MemPool.mpool` tells ST Edge AI Core which memories on the Neuro N6
a network may use, and where they are. It is Ohm Lab Ltd's, under the MIT
licence (`LICENSE-MIT.txt`).

| pool | where | size | used for |
|---|---|---|---|
| `npuRAM` (AXISRAM3 to 6) | `0x34200000` | 1792 KB | activations, which the NPU works on |
| `PSRAM` (xSPI2) | `0x71000000` | 14 MB | activations that do not fit on the chip |
| `OCTAFLASH` (xSPI1) | `0x90300000` | 26112 KB | weights, read in place from external flash |

- The CPU's AXISRAM pools have no size, so a network never takes memory the
  application uses.
- The weights sit above the application slot (from `0x90100000`) and end at
  `0x91C80000`, where the boot loader's slot B starts
  (`docs/BOOT-CONTRACT.md`).
- The PSRAM pool ends at `0x71E00000`, where the boot loader's staging area
  starts. A network that uses it needs the application's own PSRAM data to
  end below `0x71000000`. The generate report says how much of each pool a
  network uses.
- NIA-416 (`Middlewares/NeuroN6/Models/Nia416/`) was generated against an
  older pool file, whose SHA-256 its `model.json` records. It uses none of the
  PSRAM pool and 8.2 MB of the weights pool, so it runs the same with this one.

`harness/n6model.py` generates a network against this file.
