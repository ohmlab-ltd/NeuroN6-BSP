# XGA to a host with YOLOv8n instance segmentation, under FreeRTOS

> **This example is kept out of a default build.** Its network is derived from
> Ultralytics YOLOv8 and carries **AGPL-3.0**, which places obligations on any
> product distributed with it. Read [the licence section](#the-model-and-what-its-licence-means-for-a-product)
> before building anything you intend to ship. Every other network example in
> this repository uses a permissively licensed model. This example's `main.c`
> is licensed under AGPL-3.0 too, as it embeds the network.

Streams XGA (1024 x 768) Motion-JPEG at 30 fps to a host over the USB vendor
interface. YOLOv8n instance segmentation runs on the Neural-ART NPU on a
centred 768 x 768 square scaled to 256 x 256. Every frame carries the latest
detections as metadata: a box, a COCO class and a mask for each object found,
mapped onto the whole frame, and the inference time.

The console prints a heartbeat once a second, and a rate line:

```text
[hb] dets=2  top=tv 0.62 at x=0.69 y=0.11
[rate] enc=30 tx=30 torn=0 fail=0 resync=0 nn=13 npu=48ms jpg=21057B
```

## Building it

Because of the licence, `harness/compile.sh` skips this example unless asked:

```bash
NEURON6_BUILD_RESTRICTED=1 bash harness/compile.sh
```

The CubeIDE build is not gated, since asking for it by name is explicit:

```bash
bash harness/examples/cubebuild.sh Camera_YoloV8Seg_USB_FreeRTOS Appli
```

```bash
python harness/n6weights.py Middlewares/NeuroN6/Models/YoloV8Seg/model.json --write
```

```bash
python harness/uploader/neuro-n6-uploader-cli.py --bin Examples/Camera_YoloV8Seg_USB_FreeRTOS/STM32CubeIDE/Appli/Debug/Camera_YoloV8Seg_USB_FreeRTOS_Appli_Trusted.bin
```

At start-up the example checks that the camera frame for the network
(`NN_WIDTH` x `NN_HEIGHT`, RGB888) fits the network's input: a larger frame
stops the network, a smaller one is reported.

To regenerate the network from the original model:

```bash
python harness/n6model.py yolov8n_256_quant_pc_ii_seg_coco-st.tflite --name yolov8seg --out Middlewares/NeuroN6/Models/YoloV8Seg
```

## The model, and what its licence means for a product

The network is **YOLOv8n**, from Ultralytics, under **AGPL-3.0**.

Under AGPL-3.0, distributing a device or a service built on an AGPL work
carries obligations that reach the whole distributed product rather than the
model alone, including making corresponding source available to recipients.
Ultralytics also sell an **enterprise licence** that removes those
obligations.

The firmware that runs it also carries ST's software under SLA0044: the
NPU runtime, the generated network code and the post-processing. SLA0044
forbids using or distributing that software in any way that would subject it
to open-source terms, and STMicroelectronics has stated that combining
software under AGPL-3.0 with software under SLA0044 gives a combination whose
distribution, or use other than internally, is in most cases forbidden by
ST's terms. Read that way, a firmware image containing the model with those
ST components may be built and used for personal or internal purposes, but
may not be distributed while AGPL-3.0 governs the model. The enterprise licence takes the
model out of AGPL-3.0, and with it the conflict.

**Ohm Lab Ltd grants no Ultralytics licence, sub-licences none, and extends
none, and this is not legal advice.** Using or shipping this model is your
decision and your responsibility. Buy the enterprise licence, keep the use
internal, or replace the model.

The COCO dataset it was trained on is separately licensed.

This is why the example carries a `RESTRICTED-LICENCE` file and is skipped by
a default build: so that nothing derived from this network is produced by
accident. The network's generated C is additionally under ST's SLA0044, as
every network here is. `THIRD-PARTY-NOTICES.md` records all of it.

If you want a starting point without any of this,
[Camera_PalmDetect_USB_FreeRTOS](../Camera_PalmDetect_USB_FreeRTOS) and
[Camera_HandLandmarks_USB_FreeRTOS](../Camera_HandLandmarks_USB_FreeRTOS) use
Apache-2.0 models.

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
do not make a 256-pixel square.

Tested on a board with the TFT-W carrier: each inference takes 36 to 40 ms.
