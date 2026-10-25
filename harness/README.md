# Bench tooling

Scripts that compile the package, flash a board through its bootloader and
drive the test console. Not shipped API: this is how the package is verified.

## Environment

| Variable | Used by | Meaning |
|---|---|---|
| `NEURON6_HAL_ROOT` | `compile.sh` | A tree containing `STM32N6xx_HAL_Driver/` and `CMSIS/`, such as the STM32CubeN6 package's `Drivers` folder. |
| `NEURON6_CUBE_DIR` | `examples/mkexample.py`, `sync2.py`, `addstack.py`, `build.sh`; `runex.py` when the repository's project has no image | A CubeMX workspace of CubeIDE projects cloned from the examples, including `NeuroN6 - BSPTest`. The repository's own projects do not need it. |
| `STM32CUBE_REPOSITORY` | `examples/addstack.py` | The STM32Cube package repository, for HAL modules a project leaves out. Defaults to `~/STM32Cube/Repository`. |
| `STM32CUBEIDE_DIR` | `examples/cubebuild.sh`, `examples/build.sh` | The STM32CubeIDE installation (headless builder, GNU tool chain, `make`). Defaults to `C:/ST/STM32CubeIDE_2.2.0`. |
| `NEURON6_STEDGEAI` | `n6model.py` | The ST Edge AI Core executable, when it is not in its default place. |

`compile.sh` needs `arm-none-eabi-gcc` on the path; the Python tools need
Python 3 and `pyserial`. Written for Windows with Git Bash. Tools that talk
to a board want one Neuro N6 connected: with more they refuse rather than
choose, and the uploader takes `--port`.

## Tools

| Tool | What it does |
|---|---|
| `compile.sh` | Compiles every module in both security states with warnings as errors, vendored drivers with warnings shown, a C++ consumer of every public header, the FreeRTOS build with each example's stack code, and every module with the HAL's registered callbacks on. |
| `n6flash.py` | Uploads an application image (`*_Trusted.bin`) to the application slot, waits for it to enumerate, sends `b` and confirms the bootloader returns. |
| `n6weights.py` | Checks a flash region against its manifest (a network's `model.json`, or the radio's `Middlewares/NeuroN6/MayaW476/firmware/firmware.json`) by the bootloader's CRC32. `--write` programs it first, only inside the weights pool (`0x90300000` to `0x91C7FFFF`) or the radio firmware's region (from `0x91E00000`). |
| `n6fsbl.py` | Says which first-stage boot loader a board runs and whether this BSP can rely on it (v2.5.0 or later). Read-only. |
| `n6model.py` | Generates a network with ST Edge AI Core against `Utilities/npu/NeuroN6_MemPool.mpool`, recording its origin in `model.json`. |
| `n6test.py` | Flashes the test console and runs every command, writing a results table; reports `WEDGED` when the bootloader reappears mid-run. |
| `n6bench.py` | Runs the console sweep, the streaming image and a breadcrumb read, unattended. |
| `n6stream.py` | Reads the streaming image's vendor interface and checks every frame. |
| `n6probe.py` | Watches the streaming image's console while it starts. |
| `uploader/` | The DFU uploader. It writes the boot loader's own flash only with `--fsbl`, and only a boot loader image. |
| `bench/` | Host-link bench tools and host tests; see `bench/README.md`. |
| `examples/cubebuild.sh` | Builds an example's CubeIDE projects (`Appli`, `FSBL`, `Appli_FreeRTOS` where there is one) headless, from clean, in a throw-away workspace. `--all` builds every example. |
| `examples/exgen.py` | Regenerates the standalone examples from one template. |
| `examples/mkexample.py` | For a CubeMX workspace: clones a CubeIDE project for one example, fills it with `sync2.py` and `addstack.py`, and builds it. |
| `examples/sync2.py` | For a CubeMX workspace: copies the BSP and an example into a CubeIDE project, wherever the project already has the file. |
| `examples/addstack.py` | Adds the stacks an example's `*_build.json` files describe (`npu_build.json`, `wifi_build.json`, `ble_build.json`, `tls_build.json`) to a CubeMX workspace's project. `--flags` and `--sources` tell `compile.sh` how to compile the same code. |
| `examples/runex.py` | Uploads an example, captures its console, sends `b` and confirms the bootloader returns. Its exit code is the gate. |

## Records

`board-tests/STATUS.md` is the verification record. A sweep writes its report
into `board-tests/`.
