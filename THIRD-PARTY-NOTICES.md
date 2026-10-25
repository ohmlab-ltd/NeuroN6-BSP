# Third-Party Notices

**Neuro N6 board support package**
Copyright (c) 2026 Ohm Lab Ltd (team@ohmlab.co.uk)

The BSD 3-Clause licence in [`LICENSE`](LICENSE) covers **only** Ohm Lab Ltd's
own code. Everything below is licensed by its own copyright holders under its
own terms, some of which are **not open source**. A directory's own licence
file, or a file's own licence header or SPDX identifier, governs that component
and takes precedence over `LICENSE`.

Eleven ST licence files say "applicable license terms are in the
Package_license file". No such file is carried; each of those files gives the
terms that apply outside a package, and those are the terms used here.

---

## 1. Components that constrain redistribution

### 1.1 STMicroelectronics SLA0044: proprietary, ST devices only

| Path | Component |
| --- | --- |
| `Middlewares/ST/AI_Runtime/` | ST Edge AI ATON NPU runtime and network runtime library |
| `Middlewares/ST/lib_vision_models_pp/` | ST vision-model post-processing |
| `Middlewares/NeuroN6/PostProcess/app_postprocess_nia*`, `app_postprocess_iseg_yolo_v8_ui.*`, `app_postprocess_mpe_yolo_v8_ui.*` | post-processing wrappers after ST's model zoo |
| `Middlewares/NeuroN6/Models/*/`: `<name>.c`, `<name>_ecblobs.h`, `<name>_generate_report.txt` | the five networks as ST Edge AI Core generated them; each model's own licence also applies (§1.3, §1.5, §2.13, §2.14) |
| `FSBL/Middlewares/ST/STM32_ExtMem_Manager/` | ST's External Memory Manager, in the boot loader |

- **Not open source.** The `Middlewares/` paths carry SLA0044 Rev5 (February 2018); the External Memory Manager carries Rev6 (October 2025).
- **Clause 4:** the software and its derivatives may run only on, or with, a device made by or for ST.
- **Clause 5:** no use or redistribution may subject it to open-source terms. The clause names GPL, EPL, Apache, BSD and MIT.
- **Redistribution** must keep ST's copyright notice, the conditions and the disclaimer. A source copy keeps them in place; a binary reproduces them in its documentation, unless it is embedded in an ST device or is an update for one.
- **Who carries it:** the first four only matter on the neural-network path. Every Neuro N6 product carries the fifth, because it is in the boot loader.

### 1.2 NXP LA_OPT: proprietary, NXP parts only, object code only

| Path | Component |
| --- | --- |
| `Middlewares/NeuroN6/MayaW476/firmware/sduartspi_iw610.bin.se` | the IW610's Wi-Fi and BLE firmware |

- **Origin:** NXP `wifi_nb_fw`, `iw610x/sduartspi_iw610.bin.se`, commit `7d80b36c846d7948acc19fff17d9bfcc3e8acd53`, 995,992 bytes, unmodified (MD5 `99083a8fe886e59a648118b0f1606b07`); version 18.99.8.p10.
- **Licence:** NXP LA_OPT v63 (May 2025). Section 2.2: use only to develop a product that incorporates an NXP part. Section 2.3: distribution only as object code inside such a product, not standalone, where NXP's Software Content Register grants it.
- **Grant:** that commit's register (`scr.txt`) grants section 2.3 for the `iw610x` binaries at 18.99.8.p10. It lists the firmware's own third-party content: hostapd (BSD-3-Clause), easy-ecc (BSD-2-Clause) and OpenThread (BSD-3-Clause).
- **Text:** `Middlewares/NeuroN6/MayaW476/firmware/LICENSE-LA_OPT.txt`, also in `Middlewares/NXP/wifi_nxp/LA_OPT_NXP_Software_License.txt`.

### 1.3 Copyleft: two networks and their examples, AGPL-3.0, kept out of a default build

- **Networks:** `Middlewares/NeuroN6/Models/YoloV8Seg/` and `YoloV8Pose/` derive from Ultralytics YOLOv8 and are under **AGPL-3.0, or Ultralytics' commercial terms** (§2.15, §2.15a).
- **Example code:** the `main.c` of `Camera_YoloV8Seg_USB_FreeRTOS` and of `Camera_YoloV8Pose_WiFi_FreeRTOS` are AGPL-3.0, because each embeds one of those networks and is distributed with it. No other source file here is copyleft. The video encoder (§3.2) is taken under its BSD-3-Clause option.
- **What AGPL-3.0 means for a product:** distributing a device or service built on the work places obligations on the whole product, including making corresponding source available, also to users over a network. Ultralytics sell an enterprise licence that removes these obligations.
- **The SLA0044 conflict:** firmware running either network also contains SLA0044 software (§1.1). ST states that combining AGPL-3.0 and SLA0044 software gives a combination whose distribution, or non-internal use, is in most cases forbidden by its terms. Such firmware may therefore be built and used internally, but not distributed while AGPL-3.0 governs the model.
- **Guard rails:** both examples carry a `RESTRICTED-LICENCE` file, and `harness/compile.sh` and `cubebuild.sh --all` skip them unless `NEURON6_BUILD_RESTRICTED=1`.
- **Ohm Lab Ltd grants no Ultralytics licence, and this is not legal advice.** Using or shipping these models is the integrator's decision.
- **The other networks** are permissive: the MediaPipe models are Apache-2.0, and NIA Triad-416 is Ohm Lab's (§1.5).

### 1.4 Terms not confirmed

None. Every component names its licence, and every licence has been read.

### 1.5 NIA Triad-416: Ohm Lab Model Licence, Ohm Lab hardware only

`Middlewares/NeuroN6/Models/Nia416/` holds Ohm Lab Ltd's own model, under the
Ohm Lab Model Licence 1.0 (`LICENSE-OhmLab-Model.txt`), not BSD 3-Clause. Not open source.

- **Ohm Lab hardware only:** the model and its derivatives may be used, copied, modified and redistributed, but executed only on Ohm Lab hardware: the Neuro N6 and the Neuro Vision modules and carriers. Training and conversion may run on any computer.
- **Every form:** its weights (`nia416_atonbuf.xSPI1.raw`), the generated network (also SLA0044, §1.1), the weights on a board at `0x90300000`, and firmware built from `Camera_Nia416Stream_USB_FreeRTOS` or `Camera_Nia416Stream_WiFi_FreeRTOS`.
- **Other hardware:** needs a separate licence, from team@ohmlab.co.uk.
- **Attribution:** a redistributed derived model must state that it derives from NIA and name Ohm Lab as author. No warranty, and no patent rights beyond the permitted use.

---

## 2. Middleware (`Middlewares/`)

### 2.1 ST Edge AI ATON runtime: SLA0044

- **Path:** `Middlewares/ST/AI_Runtime/`. Runtime `atonn-v1.1.3-275-g6770925d`; network runtime library 10.2.0 (Cortex-M55, GCC). `Lib/` is a prebuilt static library.
- **Copyright:** STMicroelectronics. **Licence:** SLA0044 (§1.1). **Text:** `LICENSE.txt`.
- **Modified:** `Npu/ll_aton/ll_aton_runtime.c`, `ll_aton_NN_interface.h`, and `ll_aton_osal_rtos_template.c` renamed to `.h`. These changes are under SLA0044, not BSD; `Middlewares/ST/AI_Runtime/README.md` describes them.

### 2.2 ST vision-model post-processing: SLA0044

- **Path:** `Middlewares/ST/lib_vision_models_pp/`. Only the object-detection, palm, segmentation and pose parts are carried.
- **Copyright:** STMicroelectronics. **Licence:** SLA0044. **Text:** `LICENSE.md`. **Modified:** no.

### 2.3 CMSIS-DSP headers

- **Path:** `Middlewares/ARM/CMSIS-DSP/Include/` (CMSIS 5.9.0, CMSIS-DSP 1.10.0; headers only).
- **Copyright:** Arm Limited. **Licence:** Apache-2.0. **Text:** `LICENSE.txt`. **Modified:** no.

### 2.4 FreeRTOS Kernel

- **Path:** `Middlewares/FreeRTOS/`. Kernel V10.6.2, as ST redistributes it for the STM32N6.
- **Copyright:** Amazon.com, Inc. or its affiliates; Arm Limited for `Source/CMSIS_RTOS_V2/`.
- **Licence:** MIT; Apache-2.0 for `Source/CMSIS_RTOS_V2/`. **Text:** `LICENSE.md`.
- **Modified:** `Source/portable/GCC/ARM_CM55_NTZ/non_secure/port.c` (two initialisers), as `ohmlab_readme.txt` lists.

### 2.5 TinyUSB

- **Path:** `Middlewares/tinyusb/`. TinyUSB 0.20.0 (github.com/hathach/tinyusb), with only the Synopsys DWC2 port; the copy under `FSBL/src/` has the same subset.
- **Copyright:** Ha Thach (tinyusb.org) and contributors; Ohm Lab Ltd for the changes.
- **Licence:** MIT. **Text:** in every file header.
- **Modified:** `osal/osal_freertos.h`, `portable/synopsys/dwc2/dwc2_stm32.h` and `dcd_dwc2.c`, in the last of these marked `NEURO N6 LOCAL PATCH (UVC)`. The changes only take effect under FreeRTOS with DMA.

### 2.6 Neuro N6 post-processing

- **Path:** `Middlewares/NeuroN6/PostProcess/`.
- **Licence:** MIT (Ohm Lab Ltd), **except** `app_postprocess_nia416_ui.*`, `app_postprocess_nia_ui.*`, `app_postprocess_iseg_yolo_v8_ui.*` and `app_postprocess_mpe_yolo_v8_ui.*`, which are SLA0044 (§1.1). The YOLOv8 wrappers carry ST's own copyright. `NeuroN6.h` is BSD-3-Clause.
- **Text:** `LICENSE-MIT.txt`, `LICENSE-SLA0044.md`. The folder's README lists every file with its terms.

### 2.7 NXP Wi-Fi driver

- **Path:** `Middlewares/NXP/wifi_nxp/`. Driver `v1.3.r49.p2`; without `mlan_sdio.c` and without the `wmcrypto/` TLS sources (except `wm_mbedtls_net.c`, §2.19).
- **Copyright:** NXP; Swedish Institute of Computer Science (`port/net/wifi_netif.c`); Jouni Malinen and Cozybit (`wmcrypto/aescrypto/`); Michael Becker (`incl/port/osa/slist.h`, `stack_simple.h`); Amazon.com (`port/osa/osa_freertos.c`).
- **Licence:** per file. BSD-3-Clause for most. Apache-2.0 for `nw_utils/wifi_ping.c`, and alongside BSD-3-Clause in `incl/port/net/wm_net.h`. MIT for `osa_freertos.c`, `slist.h` and `stack_simple.h`. The authors' BSD licence for the `aescrypto` headers.
- **Text:** `LICENSE` and the file headers. `LA_OPT_NXP_Software_License.txt` governs the firmware (§1.2).
- **Modified (marked in the source):** `firmware_dnld/firmware_dnld.c`, `wifidriver/incl/mlan_wmm.h`, `wifi_bt_firmware/iw610x/iw610_cpu12_se.c` (firmware read from external flash), `wlcmgr/wlan.c`, `wifidriver/wifi.c`, `dhcpd/dhcp-server.c` and `incl/dhcp-server.h`, and `wifidriver/mlan_glue.c` (bounds checks in `process_rsn_ie()`, from NXP's later driver).

### 2.8 NXP OS abstraction

- **Path:** `Middlewares/NXP/osa/` (`fsl_os_abstraction*`, the list component, and `stubs/`, which are Ohm Lab's).
- **Copyright:** NXP. **Licence:** BSD-3-Clause, per file (SPDX).

### 2.9 lwIP

- **Path:** `Middlewares/lwip/`. lwIP 2.2.1 with NXP's FreeRTOS port and NXP's HTTP/HTTPS server (`src/apps/httpsrv/`, `mkfs/mkfs.pl`). lwIP's other apps and its 6LoWPAN are not carried.
- **Copyright:** Swedish Institute of Computer Science and lwIP contributors; NXP; Freescale Semiconductor; and, for the PPP headers, Carnegie Mellon University, Paul Mackerras, Tommi Komulainen and the NetBSD Foundation.
- **Licence:** BSD-3-Clause, except the PPP headers in `src/include/netif/ppp/`.
  - Twelve of those are 4-clause BSD, whose clause 4 asks for an acknowledgement in advertising material: Computing Services at Carnegie Mellon University (`fsm.h`, `ipcp.h`, `lcp.h`, `magic.h`, `upap.h`), Paul Mackerras (`ccp.h`, `chap-md5.h`, `chap-new.h`, `mppe.h`), Tommi Komulainen (`eui64.h`, `ipv6cp.h`) and the NetBSD Foundation (`pppoe.h`).
  - With `PPP_SUPPORT` at 0 they declare nothing, and no object code derives from them. The PPP implementation itself is not carried.
- **Text:** `COPYING` and the file headers.
- **Modified (marked `Ohm Lab:`):**
  - `port/lwipopts.h`;
  - `port/sys_arch/dynamic/sys_arch.c` (`lwip_rand()`);
  - the HTTP server: `httpsrv_config.h`, `httpsrv_supp.c`, `httpsrv_tls.c`, `httpsrv_script.c`.

### 2.10 Neuro N6 Wi-Fi library

- **Path:** `Middlewares/NeuroN6/MayaW476/` (Ohm Lab Ltd).
- **Licence:** MIT, except NXP's `maya_w4_board/wifi_bt_module_config.h` (BSD-3-Clause) and `wlan_txpwrlimit_cfg_murata_NH.h` (MIT, per its header). The firmware is §1.2.
- **Text:** `LICENSE-MIT.txt`; the NXP files' headers. The NXP files are unmodified.

### 2.11 Neuro N6 asset store

- **Path:** `Middlewares/NeuroN6/N6OTA/`.
- **Licence:** MIT for `n6_asset.{h,c}` and `n6_asset_host.{h,c}` (Ohm Lab Ltd); Apache-2.0 for `n6_ota_layout.h`, the boot loader's own header, byte for byte (§5.5).
- **Text:** `LICENSE-MIT.txt`, `FSBL/LICENSE`.

### 2.12 NIA Triad-416 network: Ohm Lab Model Licence and SLA0044

- **Path:** `Middlewares/NeuroN6/Models/Nia416/`, generated by ST Edge AI Core v4.0.1 through `harness/n6model.py`.
- **Licence:** the Ohm Lab Model Licence for the model in every form (§1.5); also SLA0044 for the generated `nia416.c`, `nia416_ecblobs.h` and `nia416_generate_report.txt`; BSD-3-Clause for `nia416_model.c` and `model.json`.
- **Text:** `LICENSE-OhmLab-Model.txt`, `LICENSE-SLA0044.txt`. **Modified:** generated files only have paths replaced with `<work>` and `<model>`.

### 2.13 MediaPipe palm detection network: Apache-2.0 and SLA0044

- **Path:** `Middlewares/NeuroN6/Models/PalmDet/`. Google's palm detector from the ST Edge AI Model Zoo, `033_palm_detection_full_quant_pc_uf_od.tflite` (SHA-256 `27d8512af80d612b81f8accacdc77db8b00f834f6fc9fd01e90485d5f5c282c7`), generated by ST Edge AI Core v4.0.1.
- **Copyright:** Google LLC; STMicroelectronics for the generated code.
- **Licence:** Apache-2.0 for the model; also SLA0044 for the generated `palmdet.c`, `palmdet_ecblobs.h` and `palmdet_generate_report.txt`; BSD-3-Clause for `palmdet_model.c` and `model.json`.
- **Text:** `LICENSE-Apache-2.0.txt`, `LICENSE-SLA0044.txt`. **Modified:** paths only.

### 2.14 MediaPipe hand-landmark network: Apache-2.0 and SLA0044

- **Path:** `Middlewares/NeuroN6/Models/HandLm/`. Google's hand-landmark model from the ST Edge AI Model Zoo, `033_hand_landmark_full_quant_pc_uf_handl.tflite` (SHA-256 `c5ebdcb9aa66848d483aad4d7cd44406c03cbf4fb7275633298a30e6251aa2d9`), generated by ST Edge AI Core v4.0.1.
- **Copyright:** Google LLC; STMicroelectronics for the generated code.
- **Licence:** Apache-2.0 for the model; also SLA0044 for the generated `handlm.c`, `handlm_ecblobs.h` and `handlm_generate_report.txt`; BSD-3-Clause for `handlm_model.c` and `model.json`.
- **Text:** `LICENSE-Apache-2.0.txt`, `LICENSE-SLA0044.txt`. **Modified:** paths only.

### 2.15 YOLOv8n segmentation network: AGPL-3.0 and SLA0044

- **Path:** `Middlewares/NeuroN6/Models/YoloV8Seg/`. Ultralytics YOLOv8-nano segmentation from the ST Edge AI Model Zoo, `yolov8n_256_quant_pc_ii_seg_coco-st.tflite` (SHA-256 `3316733125a5fb85d66ed33993627b5d293b3baceb348b137132071ca365ebd6`), generated by ST Edge AI Core v4.0.1.
- **Copyright:** Ultralytics; STMicroelectronics for the generated code.
- **Licence:** **AGPL-3.0, or Ultralytics' commercial terms**, for the model in every form (§1.3); also SLA0044 for the generated `yolov8seg.c`, `yolov8seg_ecblobs.h` and `yolov8seg_generate_report.txt`; BSD-3-Clause for `yolov8seg_model.c` and `model.json`.
- **Text:** `LICENSE.AGPL-3.0.txt`, `LICENSE-SLA0044.txt`. The COCO dataset is licensed separately. **Modified:** paths only.

### 2.15a YOLOv8n pose network: AGPL-3.0 and SLA0044

- **Path:** `Middlewares/NeuroN6/Models/YoloV8Pose/`. Ultralytics YOLOv8-nano pose, `yolov8n_256_quant_pc_uf_pose_coco-st.tflite` (SHA-256 `e2666d331d5e8f55809f83dd3254996063ccfbb61b13fa68d609b13d391f2e84`), unmodified from <https://github.com/stm32-hotspot/ultralytics>, `examples/YOLOv8-STEdgeAI/stedgeai_models/pose_estimation/`. Generated by ST Edge AI Core v4.0.1.
- **Corresponding source:** those scripts, plus Ultralytics' training code and checkpoints at <https://github.com/ultralytics/ultralytics>.
- **Copyright:** Ultralytics Inc.; STMicroelectronics for the generated code.
- **Licence:** **AGPL-3.0, or Ultralytics' commercial terms**, for the model in every form (§1.3); also SLA0044 for the generated `yolov8pose.c`, `yolov8pose_ecblobs.h` and `yolov8pose_generate_report.txt`; BSD-3-Clause for `yolov8pose_model.c` and `model.json`.
- **Text:** `LICENSE.AGPL-3.0.txt`, `LICENSE-SLA0044.txt`.
- **Notes:** the COCO dataset is licensed separately. Ultralytics and YOLO are trademarks of Ultralytics Inc. **Modified:** paths only.

### 2.16 Apache NimBLE

- **Path:** `Middlewares/nimble/`. NimBLE 1.10.0 (tag `nimble_1_10_0_tag`, GitHub source archive SHA-256 `ceda6114526dcdb8cf1681cd5b87648c8d45014c6e407e14c61f85313241dd07`). Host, services, bond store, host transport and FreeRTOS porting layer; `Middlewares/nimble/README.md` lists what is left out.
- **Copyright:** The Apache Software Foundation, with portions developed at Runtime Inc (`NOTICE`); Intel Corporation (`nimble/host/src/ble_sm_alg.c`); the Regents of the University of California (`porting/nimble/include/os/queue.h`, `porting/nimble/src/os_mbuf.c`).
- **Licence:** Apache-2.0, except `queue.h` and `os_mbuf.c` (BSD-3-Clause).
- **Text:** `LICENSE`. `NOTICE` must accompany any redistribution (Apache-2.0 §4(d)). **Modified:** no.

### 2.17 Mbed TLS

- **Path:** `Middlewares/mbedtls/`. Mbed TLS 3.6.6 as ST delivers it (stm32-mw-mbedtls `v3.6.6_20260511`, commit `7ee7e20e`): `include/`, `library/` and `tests/include/test/certs.h`.
- **Copyright:** The Mbed TLS Contributors; STMicroelectronics for its changes (`st_readme.txt`).
- **Licence:** Apache-2.0. **Text:** `LICENSE`. **Modified:** no.

### 2.18 Neuro N6 BLE host

- **Path:** `Middlewares/NeuroN6/BleHost/` (Ohm Lab Ltd).
- **Licence:** BSD-3-Clause, except `include/syscfg/syscfg.h`. That file derives from NimBLE 1.10.0's and stays Apache-2.0, with the ASF header and its changes marked (Apache-2.0 §4(b)).
- **Text:** `LICENSE`; for `syscfg.h`, `Middlewares/nimble/LICENSE`.

### 2.19 Neuro N6 TLS

- **Path:** `Middlewares/NeuroN6/Tls/`: Ohm Lab's glue and configuration, and NXP's `wm_mbedtls_net.c` from `wifi_nxp`, unmodified.
- **Licence:** BSD-3-Clause, the NXP file by its SPDX line. **Text:** `LICENSE`; for `wm_mbedtls_net.c`, `Middlewares/NXP/wifi_nxp/LICENSE`.

---

## 3. Components (`Components/`)

### 3.1 ST camera sensor drivers

- **Paths:** `Components/ov5640/`, `Components/vd6g/` (VD66GY), `Components/vd55g1/` (VD55G1, VD55G4, VD65G4), from <https://github.com/STMicroelectronics/stm32-mw-camera>. The SLA0044 camera middleware (`cmw_*`) is not carried.
- **Copyright:** STMicroelectronics. **Licence:** BSD-3-Clause. **Text:** the `LICENSE.md` in each directory.
- **Modified:**
  - `ov5640.c` and `ov5640.h`: mode tables and MIPI bring-up for the STM32N6 (high-resolution values ported from Linux's `drivers/media/i2c/ov5640.c`), bit-wise picture controls, grey-world white balance, and the added resolution constants. ST's header is kept, and the changes are under the same terms.
  - `vd6g.h`: `VD6G_MAX_FPS` raised from 88 to 240.

### 3.2 Verisilicon VC8000NanoE video encoder and ST's wrapper layer

- **Paths:** `Components/venc/VideoEncoder/`, `Components/venc/EWL/`. VC8000NanoE 9.22.3.7, and ST's Encoder Wrapper Layer, from ST's STM32N6 package.
- **Copyright:** Verisilicon Inc.; Google Inc.; STMicroelectronics for the wrapper.
- **Licence:** the encoder is **GPL-2.0 or BSD-3-Clause at the recipient's option**; BSD-3-Clause is taken here, as ST selected it (`st_readme.txt`). The wrapper layer is BSD-3-Clause. Every redistributed file states BSD-3-Clause. This rests on ST's delivery: Verisilicon publishes no licence page.
- **Modified:** duplicate headers dropped, `ewl_conf.h` replaced, and hooks and an interrupt guard added in `ewl_impl.c` (`Components/venc/README.md`).

### 3.3 The VL53L1CB's default configuration

- **Path:** `Components/vl53l1cb/vl53l1cb_st_tables.h`: the default values of registers 0x2D to 0x87 (91 bytes) from ST's VL53L1X ultra lite driver (STSW-IMG009).
- **Copyright:** STMicroelectronics. **Licence:** BSD-3-Clause, with ST's notice in the file. **Modified:** no.
- The rest of `Components/vl53l1cb/` is Ohm Lab's (§6). Nothing comes from ST's SLA0081 VL53L1CB driver.

The LSM6DSL, MMC5603NJ, Lepton and VL53L1CB drivers are Ohm Lab Ltd's (§6).

---

## 4. Examples (`Examples/`)

### 4.1 USB configuration

`Utilities/usb/tusb_config.h` follows TinyUSB's example configuration (MIT,
in its header). `usb_descriptors.c` is Ohm Lab Ltd's, MIT; the text is
`Utilities/npu/LICENSE-MIT.txt`.

### 4.2 Files generated from ST templates

- **CubeMX-template files:** every example's `Core/` holds files derived from ST CubeMX templates: `main.h`, `stm32n6xx_hal_conf.h`, `stm32n6xx_it.c`/`.h`, `stm32n6xx_hal_msp.c`, `partition_stm32n657xx.h`, `secure_nsc.c`, `syscalls.c`, `sysmem.c`, `system_stm32n6xx_s.c` and `Core/Startup/`. So does the `main.c` of `BSPTest`, `Camera_JpegStream_USB`, `Thermal_Stream_USB` and `Camera_GlobalShutterThermal_USB_FreeRTOS`.
- **Their terms:** ST's BSD-3-Clause, with Ohm Lab's changes under the same terms; ST's "attention" header governs where present. The same applies to the linker scripts and the CubeIDE project files.
- **The examples' own code** is Ohm Lab Ltd's, under CC0 1.0 (`SPDX-License-Identifier: CC0-1.0`). The exceptions:
  - the four ST-derived `main.c` above;
  - `nia_lcd.c`/`.h` in `Camera_Nia416Stream_WiFi_FreeRTOS`, which are MIT, with the text in `Middlewares/NeuroN6/PostProcess/LICENSE-MIT.txt`;
  - the two YOLOv8 examples' `main.c`, which are AGPL-3.0 (§1.3).

### 4.3 The NIA-416 examples

See §1.1 and §1.5. The Wi-Fi example's radio firmware is §1.2.

### 4.4 ST's LCD font

- **Paths:** `Examples/Camera_Nia416Stream_WiFi_FreeRTOS/Core/Inc/fonts.h`, `Core/Src/font16.c`, from STM32CubeN6 `Utilities/Fonts`.
- **Copyright:** 2014-2019 STMicroelectronics. **Licence:** BSD-3-Clause. **Text:** `Core/LICENSE-fonts.md`. **Modified:** no.

---

## 5. Tooling and utilities

### 5.1 Neuro N6 DFU uploader

`harness/uploader/`: Ohm Lab Ltd, MIT (`harness/uploader/LICENSE.txt`).

### 5.1a Bench tools

`harness/bench/` (`n6bench.py`, `n6wifiapi.py`, `n6radio.cs`, `build_n6radio.py`,
`hosttest/`): Ohm Lab Ltd, MIT (`harness/bench/LICENSE-MIT.txt`).

### 5.2 NPU memory pools

`Utilities/npu/NeuroN6_MemPool.mpool`: Ohm Lab Ltd, MIT (`Utilities/npu/LICENSE-MIT.txt`).

### 5.2a STM32N6 HAL, CMSIS and the secure gateway library

- **Path:** `Drivers/`, `Secure_nsclib/`. The HAL reports version 1.4.0. It is not ST's STM32Cube_FW_N6 V1.4.1 package: 22 C files differ from it, line endings aside.
  - 18 are the versions STM32CubeIDE 2.2.0 ships.
  - `stm32n6xx_hal_sdio.c` carries Ohm Lab's IW610 changes, marked `STM32N6 PATCH` (also in `Utilities/hal`).
  - `stm32n6xx_hal_dcmipp.c` carries Ohm Lab's CSI D-PHY calibration fix (`docs/HAL-PATCHES.md`).
  - `stm32n6xx_hal_xspi.c`/`.h` are another ST revision, not Ohm Lab's.
  - The RNG driver is V1.4.1's, unchanged.
- **Copyright:** STMicroelectronics; Arm Limited (`Drivers/CMSIS/Include`); Ohm Lab Ltd for the changes.
- **Licence:** BSD-3-Clause, with Ohm Lab's changes under the same terms; Apache-2.0 for Arm's CMSIS core headers.
- **Text:** `Drivers/STM32N6xx_HAL_Driver/LICENSE.txt`, `Drivers/CMSIS/LICENSE`, and the file headers.

### 5.3 HAL files for the compile harness and the FreeRTOS time base

- `harness/stm32n6xx_hal_conf.h` and `harness/stm32n6xx_hal_iwdg.h`: STMicroelectronics, BSD-3-Clause.
- `Utilities/freertos/stm32n6xx_hal_timebase_tim.c`: ST's CubeMX template, BSD-3-Clause, with Ohm Lab's changes under the same terms.
- `Utilities/freertos/FreeRTOSConfig.h`: the FreeRTOS template, with Amazon and STMicroelectronics portions, MIT (its header), set up for this board.
- `Utilities/usb/tusb_config.h`: MIT (its header). `Utilities/usb/usb_descriptors.c`: Ohm Lab Ltd, MIT.

### 5.4 Patched HAL SDIO driver

- **Path:** `Utilities/hal/` (`stm32n6xx_hal_sdio.c`, `stm32n6xx_ll_sdmmc.c`). The STM32N6 HAL 1.2.0 SDIO driver.
- **Copyright:** STMicroelectronics; Ohm Lab Ltd for the changes.
- **Licence:** BSD-3-Clause, with Ohm Lab's changes under the same terms. **Text:** `Utilities/hal/LICENSE.txt`.
- **Modified:** `stm32n6xx_hal_sdio.c`, as `ohmlab_readme.txt` lists.

### 5.5 The first-stage boot loader

- **Path:** `FSBL/`. The Neuro N6 FSBL v2.6.2 from `ohmlab-ltd/Neuro-N6-Bootloader`. `Core/Inc/n6_ota_layout.h` adds two values shared with the application. Every example's `STM32CubeIDE/FSBL` project links these sources.
- **Image:** `FSBL/Neuro-N6-FSBL-v2.6.2-Trusted.bin` is that repository's v2.6.2 image, unchanged: 83,744 bytes, CRC32 `0x3C092073`.
- **Terms:**
  - `FSBL/Core/`: Ohm Lab Ltd, **Apache License 2.0**, not BSD 3-Clause. Its CubeMX-derived files keep ST's header and remain BSD-3-Clause.
  - `FSBL/Middlewares/ST/STM32_ExtMem_Manager/`: **SLA0044** (§1.1). Ohm Lab's changes to `boot/stm32_boot_xip.c` are marked in the source.
  - `FSBL/src/`: TinyUSB 0.20.0, MIT.
  - `FSBL/STM32N657I0HXQ_AXISRAM2_fsbl.ld`: ST, BSD-3-Clause.
- **Text:** `FSBL/LICENSE`, `FSBL/Middlewares/ST/STM32_ExtMem_Manager/LICENSE.md`, and the file headers.

---

## 6. Ohm Lab Ltd original code

Everything not listed above is Ohm Lab Ltd's, under the BSD 3-Clause licence
in `LICENSE`, including:

- the `neuron6_*` modules;
- the LSM6DSL, MMC5603NJ, Lepton and VL53L1CB drivers;
- `Utilities/port/`, and the harness tools apart from §5;
- `Middlewares/NeuroN6/Models/*/<name>_model.c` and `model.json`;
- `Middlewares/NeuroN6/MayaW476/firmware/firmware.json`;
- the documentation.

The exceptions are marked in their files:

- `FSBL/` is Apache-2.0 (§5.5);
- the examples' own code is CC0-1.0 (§4.2), and AGPL-3.0 in the two YOLOv8 examples (§1.3).

To report a missing component or a wrongly recorded licence, contact team@ohmlab.co.uk.
