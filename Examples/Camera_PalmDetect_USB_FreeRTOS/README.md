# XGA to a host with MediaPipe's palm detector, under FreeRTOS

Streams XGA (1024 x 768) Motion-JPEG at 30 fps to a host over the USB vendor
interface. Google's MediaPipe palm detector runs on the Neural-ART NPU on a
centred 768 x 768 square scaled to 192 x 192. Every frame carries the latest
detections as metadata: for each hand a box and the seven landmarks that say
which way it faces, mapped onto the whole frame, and the inference time.

This is stage 1 of the MediaPipe hand cascade;
[Camera_HandLandmarks_USB_FreeRTOS](../Camera_HandLandmarks_USB_FreeRTOS)
adds stage 2. With one model and one post-processor it is the smallest
network example here, and the one to copy when starting something new.

The console prints a heartbeat once a second, and a rate line:

```text
[hb] hands=1  top=0.94 at x=0.39 y=0.59 w=0.14 h=0.14
[rate] enc=30 tx=30 torn=0 fail=0 resync=0 nn=15 npu=26ms jpg=21057B
```

`hands` is how many the detector found; the rest describe the strongest: its
score, centre and extent, as fractions of the network's input square.

## Licence

The network is Google's MediaPipe palm detector under the **Apache License
2.0**: keep the licence text; it places no obligation on an application. The
YOLOv8 examples use an Ultralytics model under **AGPL-3.0**, which does place
obligations on a distributed product, so start from this one for something
you intend to ship. `Middlewares/NeuroN6/Models/PalmDet/` holds the network,
its weights and provenance. The ST software is under SLA0044, outside this
repository's BSD-3-Clause licence; `THIRD-PARTY-NOTICES.md` lists everything.

## Building and flashing

The application goes to the boot loader's application slot, the weights to
the network pool in external flash at `0x90300000`.

```bash
bash harness/examples/cubebuild.sh Camera_PalmDetect_USB_FreeRTOS Appli
```

```bash
python harness/n6weights.py Middlewares/NeuroN6/Models/PalmDet/model.json --write
```

```bash
python harness/uploader/neuro-n6-uploader-cli.py --bin Examples/Camera_PalmDetect_USB_FreeRTOS/STM32CubeIDE/Appli/Debug/Camera_PalmDetect_USB_FreeRTOS_Appli_Trusted.bin
```

- The weights only need writing when the network changes. At start-up the
  example checks their CRC32; on a mismatch it says so once a second and runs
  no inference.
- It also checks that the camera frame for the network (`NN_WIDTH` x
  `NN_HEIGHT`, RGB888) fits the network's input: a larger frame stops the
  network, a smaller one is reported.

To regenerate the network from the original model:

```bash
python harness/n6model.py 033_palm_detection_full_quant_pc_uf_od.tflite --name palmdet --out Middlewares/NeuroN6/Models/PalmDet
```

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

Tested on a board with the OV5640 at XGA: 15 inferences a second, 26 ms each,
alongside the 30 fps stream.
