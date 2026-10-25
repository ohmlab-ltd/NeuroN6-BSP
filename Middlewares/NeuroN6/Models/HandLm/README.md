# HandLm: MediaPipe hand landmarks, for the Neural-ART NPU

Google's MediaPipe hand-landmark model, converted for the STM32N6's NPU by ST
Edge AI Core. Stage two of the MediaPipe hand cascade: given a crop of one
hand, it returns 21 three-dimensional landmarks, a hand-presence score and
which hand it is. `Camera_HandLandmarks_USB_FreeRTOS` runs it behind
[PalmDet](../PalmDet), which finds the hand to crop to.

| | |
|---|---|
| Input | 224 x 224 x 3, uint8, channel last: packed RGB888, as the camera's NN pipe writes it |
| Output | 21 landmarks with relative depth, a presence score, handedness |
| Weights | 3,182,977 bytes at `0x90420000`, CRC32 `0x3818DC1A` |
| Epochs | 55: 51 on the NPU, 4 in software |
| Inference | 26 ms on the board |

## Weights address

The cascade holds both networks at once, so this one is packed after the palm
detector, on the next 64 KB boundary (the erase block): writing one cannot
disturb the other. If the palm detector changes, regenerate this one at the
palm detector's end rounded up; `model.json` records the address:

```bash
python harness/n6model.py 033_hand_landmark_full_quant_pc_uf_handl.tflite --name handlm --out Middlewares/NeuroN6/Models/HandLm --address 0x90420000
```

## Contents

| file | what it is |
|---|---|
| `handlm.c`, `handlm_ecblobs.h` | the network as ST Edge AI Core generated it, and its epoch-controller blobs |
| `handlm_model.c` | the one file a project compiles: it places the blobs in external RAM and includes `handlm.c` |
| `handlm_atonbuf.xSPI1.raw` | the weights, for external flash |
| `handlm_generate_report.txt` | ST Edge AI Core's report for the conversion |
| `model.json` | what it was made from, with the source model's SHA-256, and the weights' address, size and checksum |

## Licence

**The model is Google's, under the Apache License 2.0**
(`LICENSE-Apache-2.0.txt`). It is permissive: keep the licence text, and it
places no obligation on a product built with it. The source file is
`033_hand_landmark_full_quant_pc_uf_handl.tflite`, SHA-256
`c5ebdcb9aa66848d483aad4d7cd44406c03cbf4fb7275633298a30e6251aa2d9`, as
`model.json` records.

**The generated C is ST Edge AI Core's output**, under ST's SLA0044
(`LICENSE-SLA0044.txt`), as for every network in this repository.

Neither is covered by this repository's BSD-3-Clause licence.
`THIRD-PARTY-NOTICES.md` lists both.
