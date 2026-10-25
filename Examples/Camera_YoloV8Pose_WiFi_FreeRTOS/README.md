# XGA to Neuro Studio over Wi-Fi with YOLOv8 pose estimation, under FreeRTOS, built to run unattended

Streams XGA (1024 x 768) Motion-JPEG at 30 fps to Neuro Studio over Wi-Fi.
YOLOv8n-pose runs on the Neural-ART NPU on a centred 768 x 768 square scaled
to 256 x 256, and every frame carries the people it found: for each, a box and
17 body points from the nose to the ankles, which Neuro Studio joins into a
skeleton over the video.

The board hosts its own access point and is built to run all day at a stand
with nobody watching: every part is watched, and anything that stops
restarts the board. See [Left running all day](#left-running-all-day).

> **The licence on the network is different. Read it before showing or
> shipping anything built from this example.** This example's `main.c` is
> licensed under AGPL-3.0, as it embeds the network. The network is derived from
> Ultralytics YOLOv8 and is under AGPL-3.0, and the firmware also carries
> STMicroelectronics software under SLA0044, whose terms in most cases forbid
> distributing, or using other than internally, software that combines the
> two. `RESTRICTED-LICENCE` beside this file keeps the example out of a
> default build of this repository; building it takes
> `NEURON6_BUILD_RESTRICTED=1` for the compile gate, while the CubeIDE
> project builds as it is. Whether showing it counts as internal use is a
> question for whoever shows it. See
> [the network's README](../../Middlewares/NeuroN6/Models/YoloV8Pose/README.md#licence)
> and `THIRD-PARTY-NOTICES.md`.

Everything else (camera, encoder, NPU set-up, console, watchdog, Wi-Fi
library, host protocol, USB/Wi-Fi hand-over) is
`Camera_Nia416Stream_WiFi_FreeRTOS`'s, and that example's README describes it.

## What it prints

| line | what it says |
|---|---|
| `[BOOT] reset=... (wdg=2s armed)` | why the chip last reset |
| `[BOOT] restart N in a row: <why> (<detail> <where>)` | only after a restart the example, the watchdog or a fault caused |
| `[BOOT] N restarts in a row: waiting S s before trying again` | past five restarts in a row, every ten seconds of the wait; then `[BOOT] trying again after N restarts in a row` |
| `[WiFi] SoftAP "NeuroN6-Pose"  password: xxxx-xxxx-xxxx-xxxx` | the network and its password, repeated whenever a console connects |
| `[NN] weights at 0x90300000: CRC32 0xD40F7B10 over 3300769 bytes, the network's (N ms)` | the weights check (see [Before it runs](#before-it-runs)) |
| `[hb] people=N  conf=C  nose=X,Y (P)` | people found; the first one's score and nose (0 to 1 in the network's square) with that point's confidence |
| `[rate] enc= tx= torn= fail= resync= nn= npu=ms jpg=B` | per second: frames encoded and sent, capture recoveries, inferences, last inference time, last frame size |
| `[wifi] transport= status= ssid= ip= heap= min=` | every 5 s: the network and the kernel heap the Wi-Fi stack uses |
| `[health] up= restarts= snapmiss= npuerr= probe=ok/failed retry=+ txfail=+ viewer=` | every 5 s: what the supervisor watches |
| `[stack] free: app= capture= usb= probe=` | every minute: each task's untouched stack |

Neuro Studio's terminal also gets the two `[BOOT]` lines, replayed after the
stream starts.

## Left running all day

Unlike the bench examples, which stop a failed task or park in the
bootloader, every failure here restarts the application.

**The supervisor.** The USB task feeds the watchdog and once a second checks:

| part | the check | limit |
|---|---|---|
| start-up | `setup()` finished: the radio, the camera, the pose network | 60 s |
| the application task | it went round `loop()` | 10 s |
| the pose network | an inference ran to the end | 10 s |
| the camera and the encoder | a frame was encoded, or held back because the host had not taken the last two | 10 s |
| the access point | it was hosting | 20 s |
| the radio | it answered the probe (every 10 s a task asks it for its counters) | 45 s |

The limits are several times the longest gap a working board shows. Each
part's own recovery runs first; the supervisor is for a part that has stopped
for good.

**Also restarting:** an assertion (including the Wi-Fi driver's after an
unanswered radio command for 20 s); the watchdog, if the USB task stops; a
processor fault, stack overflow or kernel assertion (the shared fault handlers
in `Common/Src/stm32n6xx_it_minimal.c` call `BSP_FaultCallback`, which this
example defines to restart); `Error_Handler`; a radio or access point that
does not start; a camera that does not start after three tries. A restart
costs a full start, including the radio firmware download.

**The restart record.** Before resetting, the example writes the cause, a
number and up to sixteen characters of where into backup registers 24 to 30.
The next start prints it and sends it to Neuro Studio's terminal:

```text
[BOOT] restart 1 in a row: the radio stopped answering (0x00000003 probe)
```

| cause | the number | where |
|---|---|---|
| setup did not finish | milliseconds since the start | `setup` |
| the application task stopped | milliseconds since it last went round | `app` |
| no inference finished | the times a snapshot was waited on longer than a second | `npu` |
| no frame was encoded | camera restarts so far | `capture` |
| the access point stopped | the library's status | `softap` |
| the radio stopped answering | probes unanswered | `probe` |
| Wi-Fi did not start | the library's status or result | `radio`, `transport` |
| the camera did not start | the BSP's error | `camera init`, `camera pipes` |
| an assertion failed | the line | the file |
| an unrecoverable error | the caller's address | |
| the watchdog fired | | the task that was running |
| a processor fault | the fault handlers' breadcrumb | |

A reset that left no record is reported as the watchdog or a fault.

**Back-off.** A restart counts when the board fails again within a minute of
running cleanly. After five in a row, each start first waits with only USB
up and the LED red: 30 s, then 1, 2, 4 and 8 minutes, then 10 minutes each
time. It never gives up. A clean minute, the reset button or a reflash clears
the count; a power cut clears it with the backup registers, unless the
backup domain stays powered.

**Not restarted:** weights that are not this network's, or a camera frame that
does not fit the network's input. The stream carries on without skeletons, the
console says why every second, and the LED turns red.

**The run LED** winks off for a tenth of a second each second, so a stopped
board is never mistaken for a running one:

| colour | meaning |
|---|---|
| purple | starting |
| blue | running, no viewer |
| green | a viewer has the stream over Wi-Fi |
| red | running, but it needs attention: see the console; or waiting between restarts |

**The bootloader is still reachable** with `b` on the console, `REBOOT_DFU`
from a USB host, the 1200-baud touch and a double press of the reset button,
during a wait too. There is no no-host timeout. The boot loader takes two
resets within the same second as a double press, so a failure in the first
second of a start (an `Error_Handler` before the scheduler) lands in the
bootloader; the start after the next reflash reports it.

## At a stand

1. **Before the day,** write the weights and the radio firmware once, and run
   the board for a day with a viewer on it, watching the health and stack
   lines. This example has not yet had that test (see [Status](#status)).
2. **On site, choose the channel** and store the network on the board over
   USB rather than rebuilding:

   ```bash
   N6_WIFI_KEY='a passphrase of 8 to 63 characters' python harness/bench/n6bench.py wifiset --ap NeuroN6-Pose --channel 1
   ```

   The board restarts into it. Channels 1, 6 and 11 do not overlap; pick the
   one the fewest strong networks use. `--hostname` names it for mDNS,
   `--link-key-env` keeps other laptops off it, and `--board-password` in
   place of the key keeps the board's own password.
3. **The viewer** joins that network and points Neuro Studio at
   `192.168.10.1`, port 5000, or finds it by discovery. Turn off the laptop's
   Wi-Fi power saving and its sleep.
4. **One viewer at a time;** a second is refused. The stream stays on USB
   while nobody has it over Wi-Fi.
5. **Power** from a supply that holds up: a brown-out is not something the
   board can restart its way out of, and the `[BOOT]` line names it.

## Connecting

As `Camera_Nia416Stream_WiFi_FreeRTOS`, with access point `NeuroN6-Pose`
(WPA2, channel 6, the board's own random password) at `192.168.10.1`. Change
it with `WIFI_AP_SSID`, `WIFI_AP_CHANNEL` or `WIFI_AP_SECURITY` in `main.c`,
or store one as above (`WIFI_USE_STORED 0` ignores it). A network host gets
the live controls only; the asset commands and `REBOOT_DFU` need the USB
cable.

The radio works on **2.4 GHz only, at 8 dBm, with the SDIO bus at
8.163 MHz**, the settings the wireless build was EMC-tested with
(`Camera_Nia416Stream_WiFi_FreeRTOS`'s README lists every setting).

## Before it runs

- **The network's weights**, 3,300,769 bytes at `0x90300000`, CRC32
  `0xD40F7B10` (without `--write` the command only checks them):

  ```bash
  python harness/n6weights.py Middlewares/NeuroN6/Models/YoloV8Pose/model.json --write
  ```

  Every single-network example keeps its weights at `0x90300000`, so running
  another one means writing these again. The example checks them, and that
  the camera's 256 x 256 RGB888 frame fits the network's input.
- **The radio firmware**, 995,992 bytes at `0x91E00000`, CRC32 `0x8BEF4C0C`:

  ```bash
  python harness/n6weights.py Middlewares/NeuroN6/MayaW476/firmware/firmware.json --write
  ```

  Without it the radio does not start and the board keeps restarting.
- **FSBL v2.5.0 or later**, which hands over a correctly reading flash
  interface; `python harness/n6fsbl.py` reports it, read-only.
- **The camera's lanes.** On a carrier that crosses them, such as the TFT-W,
  add `#define BSP_CAMERA_DEFAULT_CARRIER BSP_CAMERA_CARRIER_INVERTED` to
  `Core/Inc/neuron6_conf.h`. A wrong setting is silent and shows here as
  `no frame was encoded` restarts.

## Building it

Import `STM32CubeIDE/Appli` into STM32CubeIDE 2.2.0 and build `Debug`, or:

```bash
bash harness/examples/cubebuild.sh Camera_YoloV8Pose_WiFi_FreeRTOS
```

The project is the Wi-Fi NIA example's, with the model folder linked to
`Middlewares/NeuroN6/Models/YoloV8Pose`; `npu_build.json` and
`wifi_build.json` list the same stacks for the compile gate (only with
`NEURON6_BUILD_RESTRICTED=1`) and `harness/examples/addstack.py`. The image is
901,120 bytes, under the boot loader's 1.5 MB limit. The activations take
588 KB of NPU RAM from `0x34200000`; post-processing keeps 305 KB in PSRAM.
The tasks are the Wi-Fi NIA example's, with the supervisor in `usb`, plus
`probe` (idle + 1, a static 4 KB stack outside the 96 KB kernel heap), which
asks the radio for its counters every 10 s.

Provenance: the network is generated by `harness/n6model.py` from ST's
distribution of the Ultralytics model (the network's README names the file);
the pose post-processor and metadata builder in
`Middlewares/NeuroN6/PostProcess` are Ohm Lab's (the wrapper after ST's model
zoo); the pose decode is ST's `mpe_pp_yolov8.c` in
`Middlewares/ST/lib_vision_models_pp`.

## Choices

- XGA at 30 fps, JPEG quality 2 (0.61 MB/s over this link). Other modes: the
  four `#define`s at the top of `main.c`, as
  `Camera_Nia416Stream_USB_FreeRTOS`'s README describes.
- Thresholds 0.40 (how sure the network must be of a person) and 0.50 (how
  much two boxes may overlap before they are taken for one).
- A centred square, not a squashed frame, because a pose depends on the
  body's proportions; people at the sides of the picture are outside it.
- No panel (`LCD_Overlay_MPE` is an empty stub) and no BLE.

## Status

Tested on a board with the TFT-W carrier (crossed lanes set): 15 inferences a
second, 34 to 39 ms each, and `harness/bench/n6bench.py wifitest --any-app`
passed 17 of 17, with the stream at 29.9 fps.

Not yet tested: the skeletons in Neuro Studio (nobody was in front of the
camera), a day's soak, and each restart path provoked once. It needs those
before it goes to a stand.
