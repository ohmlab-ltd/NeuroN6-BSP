# XGA to a host with MediaPipe's hand cascade, under FreeRTOS

Streams XGA (1024 x 768) Motion-JPEG at 30 fps to a host over the USB vendor
interface. Two networks run on the Neural-ART NPU, one after the other, and
every frame carries the 21 landmarks of the hand in view (one hand at a time)
as points in the picture.

```text
stage 1   palm detector    192 x 192   finds a hand and which way it points
stage 2   hand landmarks   224 x 224   runs on a crop of just that hand
```

[Camera_PalmDetect_USB_FreeRTOS](../Camera_PalmDetect_USB_FreeRTOS) is stage 1
on its own, and the simpler example to start from.

## How it works

- Once a hand is acquired the detector stops; the landmark model re-aims from
  its own landmarks every frame. The detector returns after
  `LM_MISSES_BEFORE_DETECT` frames with no hand.
- `BSP_CAMERA_ConfigPipe` on the auxiliary pipe crops and scales in hardware
  from the full sensor resolution, with no CPU cost or copy, so stage 2 sees
  a 224 x 224 image of the hand alone.
- `roi_build()` builds the region with MediaPipe's method and constants,
  grown to at least 224 pixels (the pipe can only reduce). When the landmarks
  re-aim it, only the twelve that stay put while the fingers move bound it.
- **The pixels are never rotated**: the cropper is axis-aligned, so a hand
  held roughly upright gives the best landmarks. Rotating would need the 2D
  GPU, which this BSP does not drive.

## Weights

Both networks are resident at once:

| network | address | size | CRC32 |
|---|---|---|---|
| palm detector | `0x90300000` | 1,122,641 | `0xEEE62DB9` |
| hand landmarks | `0x90420000` | 3,182,977 | `0x3818DC1A` |

The second starts on the next 64 KB (erase block) boundary after the first,
placed by `harness/n6model.py --address`. Both are checked at start-up; if
either is wrong, neither runs.

## Console

A heartbeat once a second, and a rate line:

```text
[hb] stage=detect hands=0
[hb] stage=landmarks score=0.97 hand=right roi=300x300@402,210 wrist=0.52,0.88 tip=0.44,0.18 miss=0
```

`wrist` and `tip` are landmarks 0 and 8, as fractions of stage 2's input;
`roi` is the region of the camera picture that input was cropped from.

## Building and flashing

```bash
bash harness/examples/cubebuild.sh Camera_HandLandmarks_USB_FreeRTOS Appli
```

```bash
python harness/n6weights.py Middlewares/NeuroN6/Models/PalmDet/model.json --write
python harness/n6weights.py Middlewares/NeuroN6/Models/HandLm/model.json --write
```

```bash
python harness/uploader/neuro-n6-uploader-cli.py --bin Examples/Camera_HandLandmarks_USB_FreeRTOS/STM32CubeIDE/Appli/Debug/Camera_HandLandmarks_USB_FreeRTOS_Appli_Trusted.bin
```

At start-up each stage's camera frame (RGB888, `NN_WIDTH` x `NN_HEIGHT` for
the palm detector, `LM_SIZE` square for the landmarks) is compared with its
network's input: a larger frame would overrun it, so that stage does not run;
a smaller one is reported. This matters if a network or frame size changes.

To regenerate the networks:

```bash
python harness/n6model.py 033_palm_detection_full_quant_pc_uf_od.tflite --name palmdet --out Middlewares/NeuroN6/Models/PalmDet
python harness/n6model.py 033_hand_landmark_full_quant_pc_uf_handl.tflite --name handlm --out Middlewares/NeuroN6/Models/HandLm --address 0x90420000
```

The second address is the first network's end rounded up to the next 64 KB
boundary; changing the first network means recomputing it.

## Other camera modes

Change four `#define`s at the top of `main.c` together: `CAM_RESOLUTION`,
`CAM_FPS`, `BAND_BYTES` (the band the mode needs) and `JPEG_BUF_BYTES` (the
largest frame). The network's crop follows the mode.

| mode | band | JPEG buffer | stream |
|---|---|---|---|
| QVGA | 81920 | 512 KB | 60 fps |
| VGA | 81920 | 512 KB | 60 fps |
| WVGA | 102400 | 512 KB | 60 fps |
| XGA | 262144 | 512 KB | 30 fps |
| 720p | 327680 | 1.5 MB | 60 fps |
| 1080p | 491520 | 1.5 MB | 30 fps |
| QSXGA | 331776 | 1.5 MB | 15 fps |

## Licence

Both networks are Google's MediaPipe models under the **Apache License 2.0**:
permissive, with no obligation beyond keeping the licence text. The YOLOv8
examples use an Ultralytics model under **AGPL-3.0**, which does place
obligations on a distributed product; for something you intend to ship, start
from this example or the palm detector.

Tested on a board with the OV5640 at XGA: 15 inferences a second, 26 ms each
in either stage, alongside the 30 fps stream.
