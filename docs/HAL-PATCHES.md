# Required HAL patches

Three stock STM32N6 HAL defects affect this board. All are fixed in this
repository's `Drivers/`, which every example builds against. A project on an
unpatched Cube package inherits them, and CubeMX driver regeneration reverts
local patches, so check your HAL after regenerating.

## 1. DCMIPP CSI D-PHY oscillator target

- **File:** `stm32n6xx_hal_dcmipp.c`, in the CSI D-PHY configuration (marked `BUGFIX`).
- **Defect:** the stock HAL writes both bytes of `osc_freq_target` to test register `0xE3` and never the low byte to `0xE2`. At 768 Mbps and above the lane never synchronises, and no frame arrives, with no error.
- **Fix:** write the low byte to `0xE2` and the high byte to `0xE3`.
- **Affects:**
  - the OV5640 at XGA, 720p and 1080p, the modes `BSP_CAMERA_GetCapabilities` reports as needing the patch;
  - every VD66GY and VD65G4 mode (804 Mbps per lane), and the IMX335 (1600 Mbps).
- **Not sufficient alone:** a bring-up can still end with no frame, most often at XGA. `BSP_CAMERA_Init` documents the signature and the restart.
- **Upstream:** ST made the same fix in stm32n6xx-hal-driver commit `d3df886` (3 November 2026), after v1.4.0. It is not yet in a release; STM32CubeN6 1.4.1 does not have it.

## 2. SDIO block transfers (the radio)

- **File:** `stm32n6xx_hal_sdio.c` (about ten sites).
- **Defects:**
  - the block-size field comes from a `__CLZ(__RBIT())` log2, invalid for sizes that are not powers of two;
  - the CMD53 count field is mis-programmed;
  - a FIFO pointer is declared `uint8_t *`, so only a quarter of each buffer reaches the FIFO.

  STM32CubeN6 1.4.1 fixes the pointer and the polled count field, but not the block size or the DMA count field the radio uses; nor does ST's HAL repository as of `d3df886`.
- **Affects:** the Wi-Fi stack (`Middlewares/NeuroN6/MayaW476/`), and any `neuron6_wifi` user with block sizes that are not powers of two.
- **Fix:** `Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_sdio.c`. The same code is in `Utilities/hal/`, for the compile harness and for projects on a stock HAL; `harness/examples/addstack.py` installs it when an example's `wifi_build.json` asks. It compiles against STM32CubeN6 1.4.1.

## 3. DCMIPP CSI-2: one data lane on physical lane 1

- **File:** `stm32n6xx_hal_dcmipp.c`, in `HAL_DCMIPP_CSI_SetConfig` (marked `BUGFIX`).
- **Defect:** in one-lane mode the stock HAL enables only D-PHY data lane 0. A sensor whose single lane is wired to lane 1 (`DCMIPP_CSI_INVERTED_DATA_LANES`) gives no frames, with no error. Lane 1 also runs only with lane 0 enabled, and lane 0's unconnected inputs then break its synchronisation.
- **Fix:** for one lane on lane 1, enable both data lanes and set lane 0 to transmit (`CSI_PFCR_DLD` clear). `DLD` set means receive, the opposite of RM0486's description. Every other lane configuration is unchanged.
- **Affects:** the VD65G4 on the NeuroVision ST cam carrier, which wires the sensor's single lane to lane 1 (`Camera_UvcStream_VD65G4_USB` with `UVC_CARRIER_ST_CAM`, the default).
- **Upstream:** not fixed in ST's HAL as of `d3df886`.

`Utilities/hal/ohmlab_readme.txt` lists every change.
