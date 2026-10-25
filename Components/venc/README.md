# `Components/venc`: the hardware JPEG encoder

Two vendored components and nothing of ours. Both are BSD-3-Clause and both
are redistributed with their own licence files.

```
VideoEncoder/    Verisilicon VC8000NanoE software package 9.22.3.7
                 (07-Nov-2022), as redistributed by STMicroelectronics.
                 Verisilicon dual-licenses it GPL-2.0 or BSD-3-Clause at the
                 recipient's option; BSD-3-Clause is the option ST selected
                 and the option taken here. No GPL obligation flows from it.
                 See LICENSE.txt and st_readme.txt.

EWL/             STMicroelectronics Encoder Wrapper Layer, the shim that binds
                 the encoder core to the STM32N6. BSD-3-Clause.
                 See LICENSE.txt.
```

`neuron6_venc.{h,c}` in the repository root is the BSP module that drives
them. Nothing else in this BSP includes these headers.

## What is compiled

The encoder package covers JPEG and H.264. Only the JPEG half is carried:

```
VideoEncoder/inc/                  public headers, all of them
VideoEncoder/source/jpeg/          JpegEncApi.c EncJpeg.c EncJpegCodeFrame.c
                                   EncJpegInit.c EncJpegPutBits.c
VideoEncoder/source/common/        encasiccontroller.c encasiccontroller_v2.c
                                   encpreprocess.c encswhwregisters.c
                                   encInputLineBuffer.c
EWL/ewl_impl.c
```

No H.264 `.c` file is compiled. Its headers are all present, under
`VideoEncoder/source/h264/`, and that is not an oversight either: the shared
`encInputLineBuffer.c` includes `H264Instance.h` for a diagnostic helper whose
only call site is commented out upstream, and that include pulls in most of
the H.264 header set behind it. Headers cost nothing in the image; the
encoder itself is a much larger body of code this board has no use for.

## Deviations from the upstream tree

Two, both recorded here because vendored code in this repository is otherwise
byte-identical to what it came from.

**The duplicate public headers under `source/jpeg/` are dropped.** Upstream
ships a second copy of eleven headers there: `basetype.h`, `ewl.h`,
`enccommon.h`, `jpegencapi.h`, `encswhwregisters.h`, `encInputLineBuffer.h`,
`h264encapi.h`, `h264encapi_ext.h` and the three `reg_*_v7.h`. Ten are
byte-identical to their `inc/` counterparts and are simply redundant.

`basetype.h` is not, and that is the reason this is a deviation rather than
tidying. The `inc/` copy is ST's corrected one: it includes `<stddef.h>` and
leaves `size_t` alone. The `source/jpeg/` copy is the original, and it does
`typedef uint32_t size_t`. Which of the two a translation unit gets then
depends on include-path order, and getting the stale one redefines `size_t`
against the C library. Keeping both would make the build order-dependent in a
way that fails far from its cause.

**`ewl_conf.h` is ours**, not vendored. Upstream ships it as a template
(`EWL_conf_template.h`) meant to be copied into the application, which is what
`neuron6_venc_conf.h` does. It selects `EWL_USE_MALLOC_MM` and, unless
`BSP_USE_FREERTOS` is set in `neuron6_conf.h`, `EWL_USE_POLLING_SYNC`; under
FreeRTOS it selects `EWL_USE_FREERTOS_SYNC`, an event-group wait. All of them
the vendor sources implement in full.

**`ewl_impl.c` carries four hooks into this BSP**, marked `NEURON6` at each
site. The encoder must be armed and parked in its `venc_rdy` wait *before* the
camera starts, or it misses the first line events and the two deadlock; there
is no way to express that ordering from outside `EWLWaitHwRdy`, because that
is the only point at which the encoder is known to be armed. The hooks are
weak-linked callbacks with default no-op definitions, so the file still builds
against an application that does not supply them. See `neuron6_venc.c`.

**`ewl_impl.c` deletes the FreeRTOS event group on release**, marked `NEURON6`
in `EWLRelease`. Upstream only deleted it under the FreeRTOS *allocator*; with
malloc as the allocator and FreeRTOS as the synchronisation back end, every
release leaked one, and `BSP_VENC_Resync` releases and re-creates the
instance on every desync.

**`VENC_IRQHandler` ignores an interrupt while there is no event group**,
marked `NEURON6`. `BSP_VENC_Resync` releases the instance before it settles
and resets the encoder, and the encoder's interrupt stays enabled across that;
one arriving in between would hand FreeRTOS a deleted event group, whose
assertion resets the board.

## Warning policy

These sources are compiled at `-Wall -Wextra` without `-Werror`, like every
other vendored component here. Silencing a warning inside one would mean
editing it, which costs the property that makes it auditable against upstream.
See `harness/compile.sh`.
