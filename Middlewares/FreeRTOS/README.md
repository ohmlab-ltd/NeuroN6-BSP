# FreeRTOS kernel

FreeRTOS Kernel **V10.6.2**, MIT licence, copied verbatim, with its
`LICENSE.md` and the `ohmlab_readme.txt` that describes the two patches to
`port.c`. This is the copy that has run on the board. Nothing here is edited
in this repository.

| | |
|---|---|
| port | `portable/GCC/ARM_CM55_NTZ/non_secure`, Cortex-M55 without TrustZone, run in the secure state (`configRUN_FREERTOS_SECURE_ONLY 1`), FPU on, MPU off |
| heap | `portable/MemMang/heap_4.c`, one static pool of `configTOTAL_HEAP_SIZE` (96 KB) in `.bss` |
| wrapper | `CMSIS_RTOS_V2/`, CMSIS-RTOS2 over the kernel, with `cmsis_os2.h` and `os_tick.h` beside it (from the core's `CMSIS/RTOS2/Include`, so the wrapper compiles without the rest of CMSIS-RTOS2). Shipped because the core ships it; nothing in this BSP uses it |
| licence, patch note | `LICENSE.md` and `ohmlab_readme.txt` are from the core's repository, which carries them beside `Source/`; the installed copy ships only `Source/` |

The kernel's configuration is `Utilities/freertos/FreeRTOSConfig.h`, and what
the BSP adds to it is `neuron6_rtos.c`. `Utilities/freertos/README.md` lists
what a project needs.

## Two local patches, both in the port

The core carries two edits to `port.c`, marked `NEURO N6 LOCAL PATCH` where
they sit, and they come along verbatim:

- `ulCriticalNesting` starts at 0 rather than the stock `0xaaaaaaaa`
  sentinel. With the sentinel, the first `taskENTER_CRITICAL` /
  `taskEXIT_CRITICAL` pair executed before the scheduler starts raises
  `BASEPRI` and never lowers it, which on this core masked the HAL tick and
  the USB interrupt for the rest of boot the moment TinyUSB's FreeRTOS
  abstraction took its first critical section inside `tusb_init`.
- `ulMaxPRIGROUPValue` starts at the permissive priority-group mask rather
  than 0, so a `*FromISR` call made before `xPortStartScheduler` has
  calibrated it does not fail the port's priority assert. The USB interrupt
  can fire between `tud_init` and the scheduler starting, and its event post
  is such a call.

Both exist because the core brings USB up before the scheduler runs. The
examples in this repository start the scheduler before anything pumps the
stack, so neither patch is load-bearing here, but the interrupt still fires
in that window, so the second one is still what makes it safe.
