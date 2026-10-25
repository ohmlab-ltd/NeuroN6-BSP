# XGA to Neuro Studio with the NIA Triad-416 detector, under FreeRTOS

Streams XGA (1024 x 768) Motion-JPEG at 30 fps to Neuro Studio over the USB
vendor interface. Ohm Lab's NIA Triad-416 detector runs on the Neural-ART NPU
on a centred 768 x 768 square scaled to 416 x 416. Every frame carries the
latest detections as metadata: a box and class name for each of the 80 COCO
classes found, mapped onto the whole frame, and the inference time.

Everything needed to build, flash and check it is in this repository. The
console prints a heartbeat once a second, and a rate line:

```text
[hb] dets=2  top=tv 0.62 at x=0.69 y=0.11
[rate] enc=30 tx=30 torn=0 fail=0 resync=0 nn=10 npu=51ms jpg=28224B
```

## Licences

- The ST software (NPU runtime and libraries) is under SLA0044, outside this
  repository's BSD-3-Clause licence; `THIRD-PARTY-NOTICES.md` lists
  everything, and each middleware folder's README states its terms.
- The NIA Triad-416 model is Ohm Lab's, under the Ohm Lab Model Licence in
  `Middlewares/NeuroN6/Models/Nia416/LICENSE-OhmLab-Model.txt`: it may be run
  only on Ohm Lab hardware, and a firmware image built from this example
  contains it.

## The network

`Middlewares/NeuroN6/Models/Nia416/` (shared with the Wi-Fi example) holds the
network as ST Edge AI Core v4.0.1 generated it from
`nia_triad416_shipdist_int8_416_ASYM_CALIBRATED.onnx`, with the options in
`npu_build.json`. `model.json` records the model's SHA-256, the tool, the
options and the weights' address, size and checksums. It needs 1.19 MB of NPU
RAM; its 8,553,328 bytes of weights (CRC32 `0x221B72B9`) are read in place
from external flash at `0x90300000`. `nia416_model.c` is the one file a
project compiles.

To regenerate it (needs ST Edge AI Core 4.0 and the model):

```bash
python harness/n6model.py nia_triad416_shipdist_int8_416_ASYM_CALIBRATED.onnx --name nia416 --out Middlewares/NeuroN6/Models/Nia416
```

The weights have their own flash region, which flashing the application never
touches. Check them (read-only) or write them:

```bash
python harness/n6weights.py Middlewares/NeuroN6/Models/Nia416/model.json
python harness/n6weights.py Middlewares/NeuroN6/Models/Nia416/model.json --write
```

Before the network first runs, the example compares the region's CRC32 with
the network's and prints `[NN] weights at 0x90300000: CRC32 0x221B72B9 ... the
network's`. Wrong weights would run with no error and give nonsense, so on a
mismatch the network never runs, the stream carries on without detections,
and the console says how to write the weights. It also checks that the camera
frame for the network (`NN_WIDTH` x `NN_HEIGHT`, RGB888) fits the network's
input: a larger frame stops the network, a smaller one is reported.

## Building it

Import `STM32CubeIDE/Appli` into STM32CubeIDE 2.2.0 and build `Debug`, or:

```bash
bash harness/examples/cubebuild.sh Camera_Nia416Stream_USB_FreeRTOS
```

- The NPU code (runtime, post-processing, network) is built at `-O2`, the
  rest at `-O0`, with `-std=gnu17`, `-fno-strict-aliasing` and
  `LL_ATON_PLATFORM=9 LL_ATON_OSAL=5 LL_ATON_SW_FALLBACK=1 LL_ATON_RT_MODE=2
  LL_ATON_DBG_BUFFER_INFO_EXCLUDED=1 APP_HAS_PARALLEL_NETWORKS=0`.
- The project's `Model` folder links to `Middlewares/NeuroN6/Models/Nia416`.
  The link adds the runtime library and `-u _printf_float`.
- The USB product string is the descriptors' default, `NeuroN6 Vendor+CDC`
  (`BOARD_NAME=Neuro N6` cannot be one switch, for the space).
- `npu_build.json` lists the same stack for `harness/compile.sh` and
  `harness/examples/addstack.py`.
- `Middlewares/NeuroN6/PostProcess/NeuroN6.h` stands in for the application
  header the metadata builder expects; `neuron6.h` still finds the BSP's,
  which comes first on the include path. On Windows both names match either
  file.
- The epoch-controller blobs (217 KB) run from PSRAM; the startup file copies
  them there before clearing `.bss`, which overlaps their load copy. The image
  is 502,720 bytes, under the boot loader's 1.5 MB limit.

## Tasks

| task | priority | stack | what it does |
|---|---|---|---|
| `app` | idle + 2 | 16 KB | snapshots the NN pipe into the network's input, runs the network (blocking while the NPU computes), decodes and publishes the boxes |
| `capture` | idle + 3, + 4 across a re-arm | 8 KB | encodes into whichever buffer the host is not holding and queues the frame with its metadata; drops torn frames and resynchronises after two in a row |
| `usb` | idle + 3 | 8 KB | feeds the watchdog, runs the stack and the vendor transmitter, releases sent buffers, announces the capture controls, takes the host's commands |

USB comes up first, with a 500 ms enumeration pump before the scheduler.
`setup()` then brings up, in order: the sensor, the JPEG pipe and line buffer,
the NN pipe (cropped and scaled), the prime, the encoder, the NPU and runtime,
the post-processor, and last the capture task.

## Behaviour

- The console and the route to the bootloader work from boot. The watchdog is
  2 s with its early warning sent to the bootloader; `b` works in every build;
  a host that never mounts the device sends the board to the bootloader after
  30 s.
- The strobe LED is off (`CAM_STROBE` in `main.c` switches it on).
- There is no display: the overlay call is empty.
- Host commands for a manual ISP, tunables, evidence or the asset store are
  accepted and ignored. Capture controls are applied in the USB task.
- The NPU environment comes from `BSP_NPU_Init`.

## Other camera modes

Change four `#define`s at the top of `main.c` together: `CAM_RESOLUTION`,
`CAM_FPS`, `BAND_BYTES` (the band the mode needs) and `JPEG_BUF_BYTES` (the
largest frame). The network's crop follows the mode.

| mode | band | JPEG buffer | stream |
|---|---|---|---|
| QVGA | 81920 | 512 KB | none: refused |
| VGA | 81920 | 512 KB | 60 fps |
| WVGA | 102400 | 512 KB | 60 fps |
| XGA | 262144 | 512 KB | 30 fps |
| 720p | 327680 | 1.5 MB | 60 fps |
| 1080p | 491520 | 1.5 MB | 30 fps |
| QSXGA | 331776 | 1.5 MB | 15 fps |

QVGA cannot feed the network: the auxiliary pipe only reduces, and 240 lines
cannot make a 416-pixel square.

Tested on a board with the OV5640-W carrier: 30.0 fps over 60 s with no gap in
the frame counter, inference 49 to 52 ms, detections updated ten times a
second.
