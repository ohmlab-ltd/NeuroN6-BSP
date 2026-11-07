# FatFs on the SD card

FatFs's disk layer on `neuron6_sd`: drive 0 is the card.

- Add `Middlewares/FatFs/source` (`ff.c`, `ffunicode.c`) and `sd_diskio.c` to the build, with `Middlewares/FatFs/source` on the include path.
- `f_mount` brings the card up; `f_mount(NULL, ...)` then `BSP_SD_DeInit` hands SDMMC1 back to the radio.
- Configuration (`Middlewares/FatFs/source/ffconf.h`): FAT and exFAT, long names, code page 437, no clock (files are dated 2026-01-01).
- Any buffer works: `neuron6_sd` bounces those its DMA cannot use.
- Writes return once the card has programmed them, so `f_sync` costs only FatFs's own updates.
- Licence: `sd_diskio.c` is Ohm Lab's, under BSD-3-Clause. FatFs keeps ChaN's licence (`Middlewares/FatFs/LICENSE.txt`); see `THIRD-PARTY-NOTICES.md`.
