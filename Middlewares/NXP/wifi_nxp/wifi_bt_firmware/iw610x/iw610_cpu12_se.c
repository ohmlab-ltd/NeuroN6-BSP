/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdint.h>
#include "wifi_bt_module_config.h"

#if !CONFIG_UART_WIFI_BRIDGE

#if defined(IW610)

#if !(CONFIG_WIFI_IND_DNLD) && !defined(CONFIG_BT_IND_DNLD)

/* Phase 4A.5 (2026-05-21): the fw_cpu12[] array is no longer compiled into
 * the Appli binary. The firmware lives in XSPI2 memory-mapped flash at
 * 0x91E00000 (see new wlan_fw_bin definition below the historical comment
 * block). Pre-flash sduartspi_iw610.bin.se (995,992 bytes from the
 * MCUXpresso SDK) to that address via Cube Programmer before the first
 * boot. The CMOBO and non-CMOBO #if branches that used to define
 * fw_cpu12[] are removed; the comments documenting the 12-binary swap saga
 * are preserved below for historical record. */

#if 0  /* ===== HISTORICAL: fw_cpu12 array definitions removed in Phase 4A.5 ===== */

#if defined(CONFIG_WIFI_BT_CMOBO_DNLD)
const uint8_t fw_cpu12[] = {
#if CONFIG_UNCOMPRESSED_FIRMWARE
  #include <sduart_iw610.bin.se.inc>
#else
  #include <sduart_iw610_compressed.bin.se.inc>
#endif
};
#else
/* ------------------------------------------------------------------
 *  MAYA-W476 firmware swap experiment log
 *
 *  Each #include below is a different signed firmware blob we have
 *  tried to load into the IW610 inside MAYA-W476 (module lot 578C88,
 *  manufactured week 28 of 2025 ≈ Jul 14 2025). All but one are
 *  commented out -- the active blob is the single uncommented
 *  #include line. The full byte-perfect load is verified by the
 *  debugger telemetry, but every binary so far has been rejected by
 *  the chip's ROM bootloader at the chunk-1 (boot1 metadata)
 *  verification step. See app_connectivity.c and stm32n6_sdio_host.c
 *  for the diagnostics framework.
 *
 *  Each blob's SBV3 inner-header byte 28 ("image-type ID") is
 *  recorded in parentheses; differences confirm which blob we sent.
 *
 *      0x86 - sduartspi_iw610_compressed.bin.se.inc  (MCUXpresso SDK 26.03, May 2026 -- compressed)
 *      0xe0 - sduartspi_iw610_jul25.bin.se.inc       (imx-firmware lf-6.12.20-2.0.0, Q2 2025)
 *      0xb2 - sduartspi_iw610_mfg.bin.se.inc         (MCUXpresso SDK 26.03 MFG variant)
 *      0xe1 - sduartspi_iw610_25_06_p66.bin.se.inc   (wifi_nxp release/25.06.00, FW 18.99.5.p66 -- exact mfg era)
 *      0x84 - sduartspi_iw610_25_09_p76.bin.se.inc   (wifi_nxp release/25.09.00, FW 18.99.6.p76 -- first .6 era)
 *      0xe0 - sduartspi_iw610_25_03_p55.bin.se.inc   (wifi_nxp release/25.03.00, FW 18.99.5.p55 -- earliest IW610 SDIO release)
 *      0x85 - sd_iw610_stm32pack_2_0_0.bin.se.inc    (NXP STM32 CMSIS Pack 2.0.0 wifi_cli, May 29 2025, SHA 19ef69f1)
 *      0x85 - sd_iw610_stm32pack_bt_ble.bin.se.inc   (NXP STM32 CMSIS Pack 2.0.0 bt_ble_shell variant, SHA 043e61fe -- larger binary, BT-host support)
 *      0xe2 - sduartspi_iw610_imx_sep25.bin.se.inc   (imx-firmware lf-6.12.34-2.1.0, Sep 2025 -- likely p73 era, MAYA-W476 mfg+1 month)
 *      0xe4 - sduartspi_iw610_imx_dec25.bin.se.inc   (imx-firmware lf-6.12.49-2.2.0, Dec 2025)
 *      0xe4 - sduartspi_iw610_imx_mar26.bin.se.inc   (imx-firmware lf-6.18.2-1.0.0, Mar 2026)
 *      0xe1 - sduartspi_iw610_p65_jun25.bin.se.inc   (imx-firmware commit c4cb8c0e, Jun 20 2025, FW 18.99.5.p65)
 *      0xe1 - sduartspi_iw610_p68_jul13.bin.se.inc   (imx-firmware commit bded553d, Jul 13 2025, FW 18.99.5.p68 -- MODULE MFG DATE!)
 *      0xe2 - sduartspi_iw610_p70_jul23.bin.se.inc   (imx-firmware commit 1d9634fc, Jul 23 2025, FW 18.99.5.p70)
 *      0xe2 - sduartspi_iw610_p78.bin.se.inc         (imx-firmware commit b3655cb8, FW 18.99.5.p78 -- MAYA-W476 EVK OpenThread reference)
 *
 *  Active (uncommented) #include is the binary currently being tested.
 *
 *  NXP STM32 CMSIS Pack note: this is a Wi-Fi-ONLY firmware (sd_iw610),
 *  not combo (no sduartspi_iw610.bin.se.inc shipped in the Pack). NXP's
 *  STM32 reference architecture splits Wi-Fi (SDIO blob) from BT (UART
 *  blob loaded post-boot). For this test we only care whether the
 *  chip's secure-boot ROM accepts this signed binary -- if yes, this is
 *  the right key pipeline and we can later add BT via the Pack's
 *  uart_iw610_se.h header.
 *
 *  Source: NXP STM32 CMSIS Pack v2.0.0 (downloaded from
 *          nxp.com IW610 product page Design Resources -> Software,
 *          Jun 30 2025 release, 35.6 MB pack).
 *  SHA-1:  19ef69f19497c9cf9385b3af2fa13738661e4610
 *  Size:   3,317,095 bytes text (~ 553 KB binary)
 * ----------------------------------------------------------------*/
/* ============================================================================
 *  STATUS (2026-05-11): ROOT CAUSE FOUND -- it was a bug in ST's
 *  HAL_SDIO_WriteExtended (and ReadExtended), not a firmware-key problem.
 *
 *  Upstream HAL writes Size_byte (total bytes) into CMD53 bits [8:0]
 *  unconditionally. In BLOCK mode those 9 bits encode the BLOCK count,
 *  not the byte count. Our chunk 1 was 2048 bytes / 8 blocks of 256:
 *  2048 & 0x1FF = 0, and per SDIO Simplified Spec v3.00 §5.3, block-mode
 *  count=0 means "open-ended write, terminate via CMD52 ABORT". The HAL
 *  shipped 2048 bytes on the wire and stopped without sending ABORT, so
 *  the chip's ROM bootloader stayed in "waiting for more data" state
 *  forever. Manifested exactly as we saw: evt5c=0x04 (bits 0 + 3
 *  cleared), fw_status stuck at 0x0000, host_restart=0x00 (chip wasn't
 *  rejecting the firmware -- it was waiting for us to either send more
 *  blocks or issue ABORT).
 *
 *  Chunk 0 worked because it was 256 bytes in BYTE mode -- 256 & 0x1FF =
 *  256, which is the correct byte count for that single-block transfer.
 *  Our shim's heuristic (bcnt > 1 ? BLOCK : BYTE) sent chunk 0 in byte
 *  mode and chunk 1 in block mode, which is why the bug appeared only
 *  starting at chunk 1.
 *
 *  Fix: stm32n6xx_hal_sdio.c -- HAL_SDIO_ReadExtended (~line 1034) and
 *  HAL_SDIO_WriteExtended (~line 1265) now write (nbr_of_block & 0x1FFU)
 *  in block mode instead of (Size_byte & 0x1FFU). The DMA variants
 *  already had the conditional handled correctly.
 *
 *  The 12-firmware empirical sweep was diagnostically useful in that it
 *  ruled OUT firmware variation as the cause -- 12 different signed
 *  binaries all wedging identically meant the problem was on our side,
 *  not the firmware's. That ruled-out conclusion is what eventually
 *  pointed at the HAL bug.
 * ----------------------------------------------------------------------------
 *  Twelve distinct signed binaries tested, every public NXP signing-key family
 *  (image-type bytes 0x84, 0x85, 0x86, 0xb2, 0xe0, 0xe1, 0xe2, 0xe4). Every
 *  single one wedges with the identical signature:
 *
 *      chunk 0 (256 B size prefix)  ->  evt5c=0x0D  rb=0x0800  fws=0x0000
 *      chunk 1 (2048 B SBV3 boot1)  ->  evt5c=0x04  rb=0x0800  fws=0x0000
 *                                          ^^^^         ^^^^         ^^^^
 *                                       bits 0,3      no advance   no FW_READY
 *                                       CLEARED                    ever
 *      host_restart_58 = 0x00 throughout (chip doesn't even claim a CRC error)
 *      ts_evt5c locked at 0x04 for 10 s, no recovery
 *
 *  The SBV3 header byte 20..23 fingerprint (`91 9c ce 2c`) is IDENTICAL in
 *  p68 (image-type 0xe1) and p73 (image-type 0xe2). That field is the NXP
 *  signing-key identifier, not a content hash. Two different NXP-signed
 *  binaries with the same signer ID -> the module is rejecting the NXP key
 *  itself, not any individual build.
 *
 *  WHY THE OEM-KEY STORY DOESN'T HOLD (deep-search 2026-05-11):
 *    a. u-blox MAYA-W4 SIM page 38 says firmware comes from
 *       github.com/nxp-imx/imx-firmware -- the EXACT public repo we tested
 *       from. No mention of u-blox signing or custom ROTKH in any u-blox
 *       document.
 *    b. The EVK-MAYA-W4 User Guide instructs customers to use NXP's
 *       firmware-nxp-wifi Yocto recipe. If u-blox had fused a custom
 *       ROTKH, their own EVK couldn't boot the firmware they ship docs
 *       pointing at -- direct contradiction.
 *    c. NXP AN13813 sec 4.x: secure-boot on RW61x/IW61x requires BOTH
 *       SECURE_BOOT_EN fuse AND ROTKH fuse. ROTKH-only leaves the chip
 *       permissive; both-fused would mean even the EVK can't boot stock
 *       NXP firmware. Neither matches.
 *    d. Murata LBES0ZZ2LL (same IW610 silicon) ships in production using
 *       the identical nxp-imx/imx-firmware binaries (NXP UM11483). The
 *       chip silicon clearly accepts NXP-signed firmware in commercial use.
 *    e. Zero public reports of this wedge signature on any forum/issue
 *       tracker for IW610/IW612/RW610/RW612 -- if it were a secure-boot
 *       reject pattern, somebody would have hit it.
 *
 *  NEW WORKING HYPOTHESIS: host-side protocol-state bug. Something in our
 *  STM32 SDIO shim, board glue, or wifi_nxp middleware integration is in a
 *  different state than the NXP RT700-EVK / Linux i.MX reference paths.
 *  The chip's ROM bootloader is in an unexpected state after chunk 1, not
 *  rejecting the firmware content. The register signature (evt5c=0x04,
 *  fws=0x0000, host_restart=0x00, wedged) is mechanically indistinguishable
 *  at host level from a signature-reject lockup -- but the existence proof
 *  in points (a)-(e) above tells us the chip CAN boot this firmware. So
 *  the difference must be in our path, not the firmware.
 *
 *  Things that have been RULED OUT by this dataset:
 *    - Firmware version mismatch (12 versions spanning p55 -> mar26-1.0.0)
 *    - SDIO transport bug (chunk 0 always accepted; byte-perfect data
 *      delivery verified by debugger telemetry every time)
 *    - Power/reset timing (500 ms PDn LOW + 1000 ms HIGH does not change it)
 *    - Single-unit defect (two physical MAYA modules tested, both fail
 *      identically)
 *    - Chip dead/hardware fault (post-PDn re-enumeration always succeeds,
 *      enum_stage=4 enum_ok=1 -- chip is alive, just rejecting firmware)
 *    - Cache/DMA corruption (chunk_header capture matches source binary
 *      bytes 1:1 every run)
 *    - Wrong load address / opcode (CMD53 FIXED-mode write to MEM_PORT
 *      0x10000, NXP's standard download protocol; chip ACKs chunk 0)
 *
 *  Things only resolvable EXTERNALLY:
 *    1. u-blox support ticket -- request the *u-blox-signed* IW610 firmware
 *       for MAYA-W476. They produce a custom-signed variant for OEM
 *       customers; it isn't on the public download portal. Reference NXP FW
 *       version 18.99.5.p73 + module lot 578C88 + date code 25/28 in the
 *       ticket. See draft ticket text in chat / project notes.
 *    2. NXP customer Design Resources (NDA-walled) -- ask for the IW610
 *       signed boot image keyed to u-blox ROTKH. Alternative if u-blox is
 *       unresponsive.
 *    3. Switch hardware -- replace MAYA-W476 with the NXP-EVK Embedded
 *       Artists 2LL M.2 module (same IW610 silicon, NXP ROTKH so all the
 *       public firmware here would work).  ~2 weeks slip + a PCB change.
 *
 *  See also: app_connectivity.c and stm32n6_sdio_host.c for the diagnostic
 *  framework that produced the evidence in this comment. Keep the active
 *  #include below pointing at p73 (= the NXP-validated baseline) so when
 *  the u-blox-signed blob arrives we can A/B compare against a known-good
 *  reference of "right version, wrong key" rejection.
 *
 *  Previously-active blob (kept available in the iw610x folder for A/B):
 *      p73 / sduartspi_iw610_p73_aug22.bin.se.inc / image-type 0xe2
 *      imx-firmware commit 2be337a7 / Aug 22 2025 / FW 18.99.5.p73
 *
 *  STM32N6 PATCH (2026-05-18): switched to the canonical firmware blob
 *  shipped with the locally-downloaded MCUXpresso SDK at
 *  D:\OhmLab GitHub software\SDK\mcuxsdk\components\conn_fwloader\fw_bin\
 *  inc\iw610x\sduartspi_iw610.bin.se.inc (SHA256 68195A2E3F2B33B0...,
 *  5,851,453 bytes, copied 2026-05-18). This is the firmware version that
 *  the SDK's own wlan_fw_init_cfg / mlan code path was developed against,
 *  matching our locally-present wifi_nxp/wifidriver/wifi-sdio.c (which is
 *  byte-identical to the SDK's wifi-sdio.c -- verified by SHA256).
 *
 *  Rationale: even with the IRQ-spam and lost-wakeup fixes applied,
 *  wlan_init wedges on cmd-resp wait for the ~10th HostCmd while the chip
 *  asserts DAT[1] continuously without any HOST_INT_STATUS bit set. Newer
 *  mlan + older firmware is a likely culprit -- the SDK pairing should
 *  rule that out cleanly.
 * ============================================================================*/
const uint8_t fw_cpu12[] = {
  #include <sduartspi_iw610.bin.se.inc>
};
#endif

#endif  /* ===== END HISTORICAL fw_cpu12 array definitions ===== */

/* ----------------------------------------------------------------------------
 *  Phase 4A.5 active definitions (2026-05-21; auto-upload added 2026-07-26)
 *
 *  MAYA firmware lives at 0x91E00000 in XSPI memory-mapped flash.
 *
 *  harness/n6weights.py checks the region against the bootloader's CRC32
 *  from Middlewares/NeuroN6/MayaW476/firmware/firmware.json, and writes it
 *  when asked. The blob is sduartspi_iw610.bin.se from the MCUXpresso SDK's
 *  conn_fwloader, 995,992 bytes.
 *
 *  IW610_FW_BLOB_LEN below MUST match the shipped blob byte for byte: the
 *  prebuild hook cross-checks it against the file and fails the build on
 *  mismatch, so update the blob and this constant together.
 *
 *  If the firmware is missing (e.g., never flashed, or sector erased), the
 *  memory at 0x91E00000 reads as 0xFF. wlan_init() will then fail at the
 *  firmware-validation step inside wifi_nxp's download path -- the SDIO
 *  probe succeeds but FW_STATUS never reaches 0xFEDC. Inspect
 *  g_app_connectivity_diag.wlan_init_status (non-zero) to diagnose.
 * --------------------------------------------------------------------------*/
#define IW610_FW_BLOB_ADDR  (0x91E00000u)
#define IW610_FW_BLOB_LEN   (995992u)  /* sduartspi_iw610.bin.se SDK 2026-05 */

const unsigned char *wlan_fw_bin   = (const unsigned char *)IW610_FW_BLOB_ADDR;
const unsigned int wlan_fw_bin_len = IW610_FW_BLOB_LEN;

const unsigned char bt_fw_bin[]  = {0x00};
const unsigned int bt_fw_bin_len = 0;

#endif

#endif

#endif
