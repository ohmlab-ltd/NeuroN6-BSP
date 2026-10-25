
  @verbatim
  ******************************************************************************
  * @file    ohmlab_readme.txt
  * @author  Ohm Lab Ltd
  * @brief   This file lists the main modifications done by Ohm Lab Ltd on the
  *          STM32N6xx HAL Driver for the Neuro N6
  *          core.
  ******************************************************************************
  * Copyright (c) 2025-2026 Ohm Lab Ltd
  * Copyright (c) 2024-2025 STMicroelectronics (base component)
  *
  * The base component is provided by STMicroelectronics under the
  * BSD-3-Clause license; see LICENSE.txt in this directory. The
  * modifications described below are made available under the same
  * BSD-3-Clause terms.
  *
  * Contact: team@ohmlab.co.uk
  * Repository: https://github.com/ohmlab-ltd/Neuro-N6-BSP
  ******************************************************************************
  @endverbatim

  Base package
  ============
  STM32N6xx HAL Driver V1.2.0 (see __STM32N6xx_HAL_VERSION in Inc/stm32n6xx_hal.h),
  as delivered in the STM32Cube MCU Package for STM32N6.

  Only the two files listed below are modified. All other files in this tree
  are unmodified vendor sources.


### 31-May-2026 ###
=========================
  + Fix D-PHY DLL target-oscillation-frequency programming
     in \Src\stm32n6xx_hal_dcmipp.c  (function DCMIPP_CSI_SetDPHYConfig,
                                      around line 737)

    The stock sequence wrote BOTH bytes of osc_freq_target to CSI D-PHY test
    register 0xE3: the second write (low byte) clobbered the high byte, and
    register 0xE2 was never written at all. The DLL target therefore came up
    corrupt (~0xC00 instead of 0x1CC = 460), mis-calibrating the receiver
    delay line that sets the high-speed sampling point.

    Tolerable at WVGA's wide data eye; fatal at XGA (768 Mbit/s), where HS
    bursts arrive but SYNCDLxF never asserts and no frame is ever produced.

    Fix: write the high byte to 0xE3 and the low byte to 0xE2, per the
    Synopsys DWC convention (0xE3 = osc_freq_target[11:8], 0xE2 = [7:0]).
    The stock comment immediately above the code already said "@0xe3 & @0xe2".


### 21-May-2026 ###
=========================
  + Correct block-count computation for non-power-of-two SDIO block sizes
     in \Src\stm32n6xx_hal_sdio.c  (HAL_SDIO_ReadExtended ~line 990,
                                    HAL_SDIO_WriteExtended ~line 1236,
                                    HAL_SDIO_ReadExtended_DMA ~line 1487,
                                    HAL_SDIO_WriteExtended_DMA ~line 1655,
                                    SDIO_WriteExtended ~line 2838)

    The stock code derived the block count with the bit trick
      (Size_byte & ~(block_size & 1U)) >> __CLZ(__RBIT(block_size))
    which computes log2(block_size) by bit reversal and is therefore only
    correct when block_size is a power of two. The MAYA-W4 / NXP IW610
    Wi-Fi driver issues CMD53 transfers with non-power-of-two block sizes
    (e.g. 196) after firmware download; nbr_of_block then came out far too
    large and the subsequent remaining_data subtraction underflowed.

    Fix: compute nbr_of_block by integer division, guarded against a zero
    block_size. Behaviour for power-of-two block sizes is unchanged.

  + Correct the CMD53 count field for block mode
     in \Src\stm32n6xx_hal_sdio.c  (HAL_SDIO_ReadExtended_DMA ~line 1561,
                                    HAL_SDIO_WriteExtended_DMA ~line 1729)

    The stock DMA paths selected the CMD53 [8:0] count field using
    "nbr_of_block != 0" as a proxy for "block mode". That is wrong whenever
    the caller requests BYTE mode with a Size_byte that happens to be a
    multiple of block_size (e.g. bcnt=1, bsize=256, block_size=256), which
    made the command and the data-path state machine disagree about the
    transfer length and produced DCRCFAIL.

    Fix: select the count field from Argument->Block_Mode, matching the
    already-correct polling paths and SDIO Simplified Specification v3.00
    section 5.3 (cmd[8:0] is a byte count when cmd[27]=0 and a block count
    when cmd[27]=1). The now-redundant "Size_byte > 512" clamp in the write
    DMA path is dropped.


### 11-May-2026 ###
=========================
  + Write the block count, not the byte count, in block-mode CMD53
     in \Src\stm32n6xx_hal_sdio.c  (HAL_SDIO_ReadExtended ~line 1052,
                                    HAL_SDIO_WriteExtended ~line 1294)

    Per SDIO Simplified Specification v3.00 section 5.3, CMD53 bits [8:0]
    carry a byte count in byte mode but a BLOCK count in block mode, where
    0 means open-ended. The stock code wrote (Size_byte & 0x1FF)
    unconditionally. For the IW610 firmware download (block_size=256,
    Size_byte=2048) that yields 0, so the card entered open-ended mode,
    waited indefinitely for data, and hung its ROM bootloader state machine.

    Fix: in block mode write the already-computed nbr_of_block; byte mode is
    unchanged.

  + Fix 32-bit FIFO writes in the SDIO transmit path
     in \Src\stm32n6xx_hal_sdio.c  (HAL_SDIO_WriteExtended ~line 1328)

    The stock loop used a variable named u32tempbuff that was declared as
    uint8_t *, so each FIFO write pushed a single zero-extended byte while
    the pointer advanced by one byte and the remaining-length accounting
    still decremented by the full FIFO size. Only about a quarter of the
    source buffer reached the bus, the rest being zero padding: bus-level
    transfers succeeded (no DCRCFAIL, no TXUNDERR) but the IW610 never
    reported FIRMWARE_READY because the image it received was corrupt.

    Fix: pack four source bytes into one 32-bit word, write that word to the
    FIFO and advance the pointer by four, mirroring the receive path in
    HAL_SDIO_ReadExtended which was already correct.

