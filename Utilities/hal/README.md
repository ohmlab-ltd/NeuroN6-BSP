# Patched HAL sources

STM32N6 HAL sources the board needs in patched form, from ST's STM32N6xx HAL
Driver 1.2.0.

| file | why |
|---|---|
| `stm32n6xx_hal_sdio.c` | Block-transfer fixes: the stock driver miscounts blocks whose size is not a power of two and writes the wrong count into the command in several places. The radio uses such transfers once its firmware runs. `ohmlab_readme.txt` describes each fix. |
| `stm32n6xx_ll_sdmmc.c` | The low-level layer under the SDIO driver, from the same HAL. It differs from STM32CubeN6 1.4.1's only in white space. |

Both compile against STM32CubeN6 1.4.1, whose SDIO and SDMMC headers are
identical to 1.2.0's. `harness/examples/addstack.py` puts them into a project
in place of the package's copies when an example's `wifi_build.json` asks for
them. Background: `docs/HAL-PATCHES.md`.

STMicroelectronics' software, under BSD-3-Clause (`LICENSE.txt`). Ohm Lab
Ltd's changes are under the same terms, as `ohmlab_readme.txt` states. That
file also records a DCMIPP fix, which this folder does not carry.
