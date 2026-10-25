# ST vision post-processing headers

Two headers from STMicroelectronics' vision model post-processing library,
copied verbatim:

- `iseg_yolov8_pp_if.h`
- `iseg_pp_output_if.h`

They are here because the COCO class table in
`Middlewares/NeuroN6/PostProcess/classes_table_default.c` reaches them
through ST's instance-segmentation header. No code from the library is
compiled.

STMicroelectronics' software, under **SLA0044** (`LICENSE.md`). It is not
covered by this repository's BSD-3-Clause licence, and may only be used on,
or with, STMicroelectronics devices.
