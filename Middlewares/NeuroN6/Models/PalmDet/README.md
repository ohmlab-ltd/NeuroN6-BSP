# PalmDet: MediaPipe palm detection, for the Neural-ART NPU

Google's MediaPipe palm detector, converted for the STM32N6's NPU by ST Edge
AI Core. Stage one of the MediaPipe hand cascade: it finds each hand, its
extent, and the seven landmarks that say which way the hand is facing.
`Camera_PalmDetect_USB_FreeRTOS` runs it on its own;
`Camera_HandLandmarks_USB_FreeRTOS` runs it as the first of two networks.

| | |
|---|---|
| Input | 192 x 192 x 3, uint8, channel last: packed RGB888, as the camera's NN pipe writes it |
| Output | up to 2016 candidate boxes, decoded to at most 8 detections of 7 landmarks each |
| Weights | 1,122,641 bytes at `0x90300000`, CRC32 `0xEEE62DB9` |
| Epochs | 136: 127 on the NPU, 5 hybrid, 4 in software |
| Inference | 26 ms on the board |

## Contents

| file | what it is |
|---|---|
| `palmdet.c`, `palmdet_ecblobs.h` | the network as ST Edge AI Core generated it, and its epoch-controller blobs |
| `palmdet_model.c` | the one file a project compiles: it places the blobs in external RAM and includes `palmdet.c` |
| `palmdet_atonbuf.xSPI1.raw` | the weights, for external flash |
| `palmdet_generate_report.txt` | ST Edge AI Core's report for the conversion |
| `model.json` | what it was made from, with the source model's SHA-256, and the weights' address, size and checksum |

To regenerate it:

```bash
python harness/n6model.py 033_palm_detection_full_quant_pc_uf_od.tflite --name palmdet --out Middlewares/NeuroN6/Models/PalmDet
```

## Licence

**The model is Google's, under the Apache License 2.0** (`LICENSE-Apache-2.0.txt`).
It is permissive: keep the licence text, and it places no obligation on a
product built with it. The source file is
`033_palm_detection_full_quant_pc_uf_od.tflite`, SHA-256
`27d8512af80d612b81f8accacdc77db8b00f834f6fc9fd01e90485d5f5c282c7`, as
`model.json` records.

**The generated C is ST Edge AI Core's output**, under ST's SLA0044
(`LICENSE-SLA0044.txt`), as for every network in this repository.

Neither is covered by this repository's BSD-3-Clause licence.
`THIRD-PARTY-NOTICES.md` lists both.
