# Neuro N6 post-processing

Turns the NPU outputs of the five networks the NN examples run (NIA
Triad-416, the palm detector, the hand landmark model, YOLOv8n segmentation
and pose) into results, and publishes them as metadata on the vendor stream.

| file | what it is | from | terms |
|---|---|---|---|
| `nia416_pp.{c,h}` | the decode: an int8 gate on the class logits (Helium when the compiler offers it), boxes from the distance outputs, class-aware NMS | core library | MIT |
| `nia_pp.{c,h}` | the detection types the decode fills, and the NIA decode the metadata builder links against | core library | MIT |
| `PostProcessor_NIA416.{c,h}` | the post-processor object an application binds: `PostProcess_NIA416(conf, iou)` | core library | MIT |
| `vnd_meta.{c,h}` | the stream's metadata slots, their TLV encoding, the debug-text and boot-log replay | core library | MIT |
| `vnd_meta_nia.{c,h}` | the detection builder: inference time, class names and boxes, mapped onto the camera frame | core library | MIT |
| `classes_table_default.c` | the 80 COCO class names, weak, so an application can replace them | core library | MIT |
| `LCD_Overlay_Nia.h`, `LCD_Overlay_PD.h`, `LCD_Overlay_Iseg.h`, `LCD_Overlay_MPE.h` | the overlay functions the post-processors call | core library | MIT |
| `PostProcessor_PD.{c,h}`, `app_postprocess_pd_ui.{c,h}`, `vnd_meta_pd.{c,h}` | the palm detector: anchors, the decode through ST's library with a correct post-NMS count, the post-processor object, the metadata builder | this repository | MIT |
| `PostProcessor_HandLM.{c,h}`, `app_postprocess_handlm_ui.{c,h}`, `vnd_meta_handlm.{c,h}` | the hand landmark model: 21 landmarks from a hand crop, the post-processor object, the metadata builder | this repository | MIT |
| `PostProcessor_YOLOv8.{c,h}`, `vnd_meta_yolov8_Iseg.{c,h}`, `vnd_meta_yolov8_MPE.{c,h}` | YOLOv8n segmentation and pose: the post-processor objects and the metadata builders for masks and keypoints | this repository | MIT |
| `app_postprocess_nia416_ui.{c,h}`, `app_postprocess_nia_ui.{c,h}`, `app_postprocess_iseg_yolo_v8_ui.{c,h}`, `app_postprocess_mpe_yolo_v8_ui.{c,h}` | the wrappers around the decodes, after ST's model zoo | core library | **SLA0044** |
| `NeuroN6_IPostProcessor.h` | the post-processor interface | this repository | MIT |
| `n6_host_contract.h` | the host protocol: traffic classes, command formats, CRC | this repository | MIT |
| `NeuroN6.h` | the two coordinate helpers `vnd_meta_nia.c` takes from the application header `NeuroN6.h` | this repository | BSD-3-Clause |

Ohm Lab's own code here is under the MIT licence. SLA0044 applies to
the model-zoo wrappers (`LICENSE-SLA0044.md`, the library's own licence
file), which are not covered by this repository's BSD-3-Clause licence and
may only be used on, or with, STMicroelectronics devices.

## What an application provides

- `uint32_t ts[2]`: the inference's start and end, in milliseconds; the
  metadata reports the difference.
- `uint16_t g_nn_crop_x, g_nn_crop_y, g_nn_crop_w, g_nn_crop_h,
  g_camera_full_w, g_camera_full_h`: the camera window the network sees, and
  the whole frame. Detections are mapped through them.
- The overlay function its post-processor draws through:
  `LCD_Overlay_Nia_From()` (NIA Triad-416), `LCD_Overlay_PD()` (palm
  detector), `LCD_Overlay_Iseg()` and `LCD_Overlay_MPE()` (YOLOv8
  segmentation and pose). Each draws on a display, or nothing without one.

On a case-insensitive file system `NeuroN6.h` matches the BSP's `neuron6.h`:
put this folder after the BSP's on the include path. `vnd_meta_nia.c`, the one
file that uses `NeuroN6.h`, finds it beside itself.

`Examples/Camera_Nia416Stream_USB_FreeRTOS` uses all of it, built with
`-O2 -fno-strict-aliasing -std=gnu17`.
