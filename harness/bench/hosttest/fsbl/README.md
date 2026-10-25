# The boot loader on a simulated board

`FSBL/Core/Src/main.c`, the boot loader's own source, built for a PC and run
against a simulated Neuro N6. No board, no USB.

```bash
sh harness/bench/hosttest/fsbl/run.sh            # every test
sh harness/bench/hosttest/fsbl/run.sh update     # one, by its name
```

Needs Linux, or WSL on Windows (`wsl sh harness/bench/hosttest/fsbl/run.sh`),
and `gcc`: the flash, the PSRAM and the peripheral registers are mapped
(`mmap`) at the addresses the loader uses.

## What runs

The source is compiled unchanged against the real device, HAL and external
memory manager headers; `fake/` stands in for the core, TrustZone and TinyUSB
headers. `fsbl_sim.c` is the board: the NOR flash at the command level (octal
DTR and SPI, worst-case erase and program times), the PSRAM, the backup
registers and RTC, the watchdog, the USB serial stream and the jump to an
application. Each boot runs in its own process, so the loader's variables
start fresh while the flash, backup registers and clock carry over. A test
can cut the power after any erase or program, fail a memory mapping, give the
loader a flash that takes no writes, programs slowly or flips a bit, hang an
application, or leave the bootloader idle.

## Random runs

`fuzz_fsbl.c` generates upload sessions and board states from a seed and
checks invariants: the flash changes only where an accepted erase allows, the
loader jumps only into a whole application and never writes the slot it last
ran outside a trial, the settings keep a whole copy, the watchdog runs only
for an image on trial, and the board settles. `run.sh` makes 400 runs from a
fixed seed:

```bash
sh harness/bench/hosttest/fsbl/run.sh fuzz 5000 7          # 5000 runs from seed 7
sh harness/bench/hosttest/fsbl/run.sh fuzz 5000 7 1861     # run 1861 of them alone
sh harness/bench/hosttest/fsbl/run.sh fuzz 5000 7 0 boots  # every run a boot
```

A failure prints its seed and run, which reproduce it. v2.6.2's source in
`FSBL/` does not yet meet these rules, so the fuzzer reports the first finding
it meets and a run of everything leaves it out.

## The uploader against the loader

`test_uploader.py` runs the repository's uploader (`harness/uploader`, as
`harness/n6flash.py` and `harness/n6weights.py` drive it) against
`fsbl_serve`, the simulated board with its serial stream on a pseudo-terminal.
Needs Python 3.9 or later and pyserial; skipped, with a line saying so,
without them.

```bash
sh harness/bench/hosttest/fsbl/run.sh uploader          # these alone
sh harness/bench/hosttest/fsbl/run.sh uploader fsbl     # one of them
```

## The USB descriptors

`test_usb_descriptors.c` builds `FSBL/Core/Src/usb_descriptors.c` against the
loader's TinyUSB headers and checks each descriptor at both speeds, including
the other-speed configuration and the serial number from the chip's
identifier.

```bash
sh harness/bench/hosttest/fsbl/run.sh usb
```

## Another copy of the source

`FSBL_MAIN=<file>` and `FSBL_USB=<file>` run another copy of `main.c` or
`usb_descriptors.c`, for example to show a test failing without a fix:

```bash
git show <commit>:FSBL/Core/Src/main.c > /tmp/main_before.c
FSBL_MAIN=/tmp/main_before.c sh harness/bench/hosttest/fsbl/run.sh
```

## Limits

It tests the loader's logic, not real-hardware timing, the memories'
electrical behaviour, the USB stack or the jump code. A change that passes
here must still run on a board.

## Open findings

Tests marked open in the list at the end of `test_fsbl.c` state what must
hold once a finding is fixed. The run reports each as `OPEN`, and one that
passes while marked open as a failure; the exit code is 0 when every other
test passes. 53 tests in `test_fsbl.c` are open (most of `f1` to `f11`, and
`gap`, `gaphard` and `cut`), with the uploader's tests of the loader's lock
and of an upload cut short, and the other-speed descriptor's type. They are
v2.6.2's open findings:
[docs/BOOT-CONTRACT.md, "Known issues in v2.6.2"](../../../../docs/BOOT-CONTRACT.md#known-issues-in-v262).
