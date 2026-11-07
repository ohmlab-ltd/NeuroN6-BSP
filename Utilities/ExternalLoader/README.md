# Flash loader

STM32CubeProgrammer loader for the Neuro N6's 32 MB MX25UM25645G octal NOR flash, on XSPI1.

- **File:** `NeuroN6_MX25UM25645G.stldr`; STM32CubeProgrammer lists it as `NeuroN6_MX25UM25645G`. Copy it into STM32CubeProgrammer's `bin/ExternalLoader` folder, or pass it with `-el`.
- **Flash:** `0x90000000`, 32 MB, 512 sectors of 64 KB, 256-byte pages.
- **Connect:** the board in development boot, an ST-LINK on SWD, access port 1 (the Cortex-M55). To write an application to slot A and verify it:

```bash
STM32_Programmer_CLI -c port=SWD mode=HOTPLUG ap=1 -el NeuroN6_MX25UM25645G.stldr -w app_Trusted.bin 0x90100000 -v
```

- **Flash map:** the bootloader is `0x90000000` to `0x900FFFFF` and slot A starts at `0x90100000`; stored settings, network weights and the radio firmware follow ([BOOT-CONTRACT.md](../../docs/BOOT-CONTRACT.md#flash-map-nor-window-0x90000000)). Writing below `0x90100000`, or a mass erase, replaces the bootloader.
- **Clocks:** the loader runs the core from the 64 MHz internal oscillator and XSPI1 from HCLK, 32 MHz.
- **Debugging a running board:** once the bootloader or an application is running, a hot-plug connection does not reach the core; reset the board, or connect under reset, to debug.
- **Licence:** built on ST's STM32_ExtMem_Loader (SLA0044, `LICENSE-SLA0044.md`) and the STM32N6 HAL (BSD-3-Clause); see `THIRD-PARTY-NOTICES.md` §5.6.
