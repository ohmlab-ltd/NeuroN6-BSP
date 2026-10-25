# YoloV8Pose: YOLOv8n pose estimation, for the Neural-ART NPU

> **AGPL-3.0.** This network is derived from Ultralytics YOLOv8. Distributing
> a product built on it carries obligations that reach the whole product, and
> ST's terms for the software that runs it on the NPU conflict with them. Read
> [Licence](#licence) before showing or shipping anything built from it. The
> other networks here are permissively licensed, apart from YoloV8Seg.

YOLOv8-nano pose estimation at 256 x 256: people, each with a box and 17
body points, converted for the STM32N6's NPU by ST Edge AI Core.
`Camera_YoloV8Pose_WiFi_FreeRTOS` runs it, and is kept out of a default build
for the reason above.

| | |
|---|---|
| Input | 256 x 256 x 3, uint8, channel last: packed RGB888, as the camera's NN pipe writes it |
| Output | one float32 tensor, 56 x 1344: for each of 1344 candidates a box, a person score and 17 points of x, y and confidence |
| Weights | 3,300,769 bytes at `0x90300000`, CRC32 `0xD40F7B10` |
| Activations | 588 KB of the NPU's RAM, `0x34200000` to `0x34293000` |
| Epochs | 131: 111 on the NPU, 18 hybrid, 2 in software |

ST's `mpe_pp_yolov8.c` (`Middlewares/ST/lib_vision_models_pp`) decodes the
output, through `app_postprocess_mpe_yolo_v8_ui.c` and the
`PostProcess_YOLOv8_MPE` post-processor in `Middlewares/NeuroN6/PostProcess`,
which also publish the skeletons as the stream's metadata.

## Where it comes from

`yolov8n_256_quant_pc_uf_pose_coco-st.tflite`, SHA-256
`e2666d331d5e8f55809f83dd3254996063ccfbb61b13fa68d609b13d391f2e84`,
unmodified from STMicroelectronics' distribution of the Ultralytics
software, <https://github.com/stm32-hotspot/ultralytics>, file
`examples/YOLOv8-STEdgeAI/stedgeai_models/pose_estimation/yolov8n_256_quant_pc_uf_pose_coco-st.tflite`.
Generated with `harness/n6model.py` and ST Edge AI Core v4.0.1 against the
board's memory pools.

## Contents

| file | what it is |
|---|---|
| `yolov8pose.c`, `yolov8pose_ecblobs.h` | the network as ST Edge AI Core generated it, and its epoch-controller blobs |
| `yolov8pose_model.c` | the one file a project compiles: it places the blobs in external RAM and includes `yolov8pose.c` |
| `yolov8pose_atonbuf.xSPI1.raw` | the weights, for external flash |
| `yolov8pose_generate_report.txt` | ST Edge AI Core's report for the conversion |
| `model.json` | what it was made from, with the source model's SHA-256, and the weights' address, size and checksum |

Write the weights to a board, then check them (read-only):

```bash
python harness/n6weights.py Middlewares/NeuroN6/Models/YoloV8Pose/model.json --write
python harness/n6weights.py Middlewares/NeuroN6/Models/YoloV8Pose/model.json
```

They share `0x90300000` with every other single-network example's weights,
so running another of those examples means writing these again.

Regenerate the network from the `.tflite` above:

```bash
python harness/n6model.py yolov8n_256_quant_pc_uf_pose_coco-st.tflite --name yolov8pose --out Middlewares/NeuroN6/Models/YoloV8Pose
```

## Licence

**The model is Ultralytics-derived, under AGPL-3.0 or Ultralytics' commercial
terms** (`LICENSE.AGPL-3.0.txt`).

Ultralytics release YOLOv8 under AGPL-3.0. A network derived from it, this one
included, carries that licence. Under AGPL-3.0, distributing a device or a
service built on the work places obligations on the whole distributed product,
not on the model alone, including making the corresponding source available to
recipients, and, where users interact with it over a network, offering it to
them. Ultralytics also sell an enterprise licence that removes those
obligations.

**The firmware that runs it also carries ST's software under SLA0044**: the
NPU runtime, the generated network code and the post-processing. SLA0044
forbids using or distributing that software in any way that would subject it
to open-source terms, and STMicroelectronics has stated that combining
software under AGPL-3.0 with software under SLA0044 gives a combination whose
distribution, or use other than internally, is in most cases forbidden by
ST's terms. Read that way, a firmware image containing the model with those
ST components may be built and used for personal or internal purposes, but
may not be distributed by anyone while AGPL-3.0 governs the model. The
Ultralytics enterprise licence takes the model out of AGPL-3.0, and with it
the conflict.

**Ohm Lab Ltd grants no Ultralytics licence, sub-licences none, and extends
none, and this is not legal advice.** Whether a use of a firmware image built
from this network is internal, a demonstration included, is yours to decide,
with advice if you need it. Buy the enterprise licence, keep the use internal,
or replace the model.

The corresponding source for the model is the distribution above, whose
directory also holds the scripts that produced the file, and the training
code and checkpoints at <https://github.com/ultralytics/ultralytics>. The
model is Copyright Ultralytics Inc.; Ultralytics and YOLO are its trademarks,
used here only to say where the model came from. The COCO dataset it was
trained on is separately licensed.

**The generated C is ST Edge AI Core's output**, under ST's SLA0044
(`LICENSE-SLA0044.txt`), as for every network in this repository.

Neither is covered by this repository's BSD-3-Clause licence.
`THIRD-PARTY-NOTICES.md` lists both. The example that embeds the model,
`Camera_YoloV8Pose_WiFi_FreeRTOS`, has its `main.c` licensed under AGPL-3.0.
