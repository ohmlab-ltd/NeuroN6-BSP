# YoloV8Seg: YOLOv8n instance segmentation, for the Neural-ART NPU

> **AGPL-3.0.** This network is derived from Ultralytics YOLOv8. Distributing
> a product built on it carries obligations that reach the whole product. Read
> [Licence](#licence) before shipping anything. The other networks here are
> permissively licensed; this one is not.

YOLOv8-nano instance segmentation at 256 x 256 over the 80 COCO classes,
converted for the STM32N6's NPU by ST Edge AI Core.
`Camera_YoloV8Seg_USB_FreeRTOS` runs it, and is kept out of a default build
for the reason above.

| | |
|---|---|
| Input | 256 x 256 x 3, uint8, channel last: packed RGB888, as the camera's NN pipe writes it |
| Output | boxes, class indices and mask coefficients, decoded to detections with masks |
| Weights | 3,474,865 bytes at `0x90300000`, CRC32 `0x53494735` |
| Epochs | 140: 122 on the NPU, 17 hybrid, 1 in software |

## Contents

| file | what it is |
|---|---|
| `yolov8seg.c`, `yolov8seg_ecblobs.h` | the network as ST Edge AI Core generated it, and its epoch-controller blobs |
| `yolov8seg_model.c` | the one file a project compiles: it places the blobs in external RAM and includes `yolov8seg.c` |
| `yolov8seg_atonbuf.xSPI1.raw` | the weights, for external flash |
| `yolov8seg_generate_report.txt` | ST Edge AI Core's report for the conversion |
| `model.json` | what it was made from, with the source model's SHA-256, and the weights' address, size and checksum |

To regenerate it:

```bash
python harness/n6model.py yolov8n_256_quant_pc_ii_seg_coco-st.tflite --name yolov8seg --out Middlewares/NeuroN6/Models/YoloV8Seg
```

## Licence

**The model is Ultralytics-derived, under AGPL-3.0 or Ultralytics' commercial
terms** (`LICENSE.AGPL-3.0.txt`).

Ultralytics release YOLOv8 under AGPL-3.0. A network derived from it, this one
included, carries that licence. Under AGPL-3.0, distributing a device or a
service built on the work places obligations on the whole distributed product,
not on the model alone, including making the corresponding source available to
recipients. Ultralytics also sell an enterprise licence that removes those
obligations.

The firmware that runs it also carries ST's software under SLA0044: the
NPU runtime, the generated network code and the post-processing. SLA0044
forbids using or distributing that software in any way that would subject it
to open-source terms, and STMicroelectronics has stated that combining
software under AGPL-3.0 with software under SLA0044 gives a combination whose
distribution, or use other than internally, is in most cases forbidden by
ST's terms. Read that way, a firmware image containing the model with those
ST components may be built and used for personal or internal purposes, but
may not be distributed while AGPL-3.0 governs the model. The enterprise
licence takes the model out of AGPL-3.0, and with it the conflict.

**Ohm Lab Ltd grants no Ultralytics licence, sub-licences none, and extends
none, and this is not legal advice.** Using or shipping this model is your
decision and your responsibility. Buy the enterprise licence, keep the use
internal, or replace the model.

The COCO dataset the model was trained on is separately licensed.

**The generated C is ST Edge AI Core's output**, under ST's SLA0044
(`LICENSE-SLA0044.txt`), as for every network in this repository.

Neither is covered by this repository's BSD-3-Clause licence.
`THIRD-PARTY-NOTICES.md` lists both. The example that embeds the model,
`Camera_YoloV8Seg_USB_FreeRTOS`, has its `main.c` licensed under AGPL-3.0.

The source file is `yolov8n_256_quant_pc_ii_seg_coco-st.tflite`, SHA-256
`3316733125a5fb85d66ed33993627b5d293b3baceb348b137132071ca365ebd6`, as
`model.json` records.
