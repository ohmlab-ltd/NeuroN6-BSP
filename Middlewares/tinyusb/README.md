# TinyUSB

TinyUSB **0.20.0** (`tusb_option.h`), the `src` tree of
[hathach/tinyusb](https://github.com/hathach/tinyusb), MIT licence, as each
file's header states. Every example's CubeIDE project builds with this tree.

Three files differ from TinyUSB 0.20.0:

- `osal/osal_freertos.h`: every wait is cut to zero until the scheduler
  starts, so the stack can be pumped before it does.
- `portable/synopsys/dwc2/dwc2_stm32.h`: the N6's tightly coupled memories
  are left out of cache maintenance.
- `portable/synopsys/dwc2/dcd_dwc2.c`: TinyUSB 0.21.0's DMA SETUP handling
  backported, and `NEURO N6 LOCAL PATCH (UVC)`: after an endpoint 0 data
  stage, the cache is invalidated from the start of the received bytes.

The changes only take effect under FreeRTOS with the controller in DMA mode
(`CFG_TUD_DWC2_DMA_ENABLE`), which the FreeRTOS examples' configurations
select. The bare-metal examples and the BSP's console run it in slave mode,
where these files behave as TinyUSB 0.20.0's.

The stack's configuration is not here: `Utilities/usb/tusb_config.h` is the
BSP's, and an example that brings its own keeps it under its own
`Utilities/usb`. `harness/compile.sh` compiles the BSP against this tree.
