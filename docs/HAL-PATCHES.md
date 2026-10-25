# Required HAL patches

Two stock STM32N6 HAL defects affect this board. Both are fixed in this
repository's `Drivers/`, which every example builds against. A project on an
unpatched Cube package inherits them, and CubeMX driver regeneration reverts
local patches, so check your HAL after regenerating.

## 1. DCMIPP CSI D-PHY oscillator target

- **File:** `stm32n6xx_hal_dcmipp.c`, in the CSI D-PHY configuration (marked `BUGFIX`).
- **Defect:** the stock HAL writes both bytes of `osc_freq_target` to test register `0xE3` and never the low byte to `0xE2`. At 768 Mbps and above the lane never synchronises, and no frame arrives, with no error.
- **Fix:** write the low byte to `0xE2` and the high byte to `0xE3`.
- **Affects:**
  - the OV5640 at XGA, 720p and 1080p, the modes `BSP_CAMERA_GetCapabilities` reports as needing the patch;
  - every VD66GY mode, as its link runs at 804 Mbps per lane.
- **Not sufficient alone:** a bring-up can still end with no frame, most often at XGA. `BSP_CAMERA_Init` documents the signature and the restart.

## 2. SDIO block transfers (the radio)

- **File:** `stm32n6xx_hal_sdio.c` (about ten sites).
- **Defects:**
  - the block-size field comes from a `__CLZ(__RBIT())` log2, invalid for sizes that are not powers of two;
  - the CMD53 count field is mis-programmed;
  - a FIFO pointer is declared `uint8_t *`, so only a quarter of each buffer reaches the FIFO.

  STM32CubeN6 1.4.1 fixes the pointer and the polled count field, but not the block size or the DMA count field the radio uses.
- **Affects:** the Wi-Fi stack (`Middlewares/NeuroN6/MayaW476/`), and any `neuron6_wifi` user with block sizes that are not powers of two.
- **Fix:** `Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_sdio.c`. The same code is in `Utilities/hal/`, for the compile harness and for projects on a stock HAL; `harness/examples/addstack.py` installs it when an example's `wifi_build.json` asks. It compiles against STM32CubeN6 1.4.1.

`Utilities/hal/ohmlab_readme.txt` lists both changes.
