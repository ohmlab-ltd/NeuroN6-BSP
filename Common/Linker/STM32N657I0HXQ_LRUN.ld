/*
******************************************************************************
**
** @file        : STM32N657I0HXQ_LRUN.ld
**
** @author      : GPM Application Team
**
** @brief       : Linker script for STM32N6XX Device from STM32N6 series
**                      2048 KBytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
**
******************************************************************************
** @attention
**
** Copyright (c) 2025 STMicroelectronics.
** All rights reserved.
**
** This software is licensed under terms that can be found in the LICENSE file
** in the root directory of this software component.
** If no LICENSE file comes with this software, it is provided AS-IS.
**
******************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM); /* end of "RAM" Ram type memory */
_sstack = _estack - _Min_Stack_Size;

_Min_Heap_Size = 0x200; /* required amount of heap */
_Min_Stack_Size = 0x4000; /* required amount of stack */

/* Memories definition
 *
 * The RAM region spans THREE physical memories (RM0486 Table 24): FLEXRAM
 * 0x34000000-0x34063FFF (RISAF7), AXISRAM1 0x34064000-0x340FFFFF (RISAF2),
 * AXISRAM2 0x34100000-0x341FFFFF (RISAF3). BOARD_RisafConfig() (neuron6.c)
 * opens all three, and RISAF12, and RISAF7 MUST be among them: without it,
 * bus-master DMA (SDMMC/ETH/...) from FLEXRAM silently reads zeros while the
 * CPU (a secure master) works fine: a failure with no error to point at.
 *
 * The FSBL runs at 0x34180000 and copies the application image below it, so
 * the LOADED image must stay under 0x34180000 (1.5 MB), enforced by
 * `ASSERT(_sbss <= 0x34180000, ...)` at the end of SECTIONS, NOT by the
 * region length. The RUNTIME image (.bss + the NOLOAD .axisram_nc band +
 * heap + stack) may use the whole region: the FSBL space is dead after
 * hand-off (_estack at 0x34200000).
 *
 * The NOLOAD .axisram_nc section is not pinned to a fixed address: placed
 * after .bss, 32-byte aligned, marked NON-CACHEABLE by the application's MPU
 * set-up (each example's mpu_config) from its linker symbols
 * (__axisram_nc_start/_end) so the D-cache stays ON.
 */
MEMORY
{
  RAM    (xrw)    : ORIGIN = 0x34000400,   LENGTH = 2047K   /* 0x34000400 .. 0x34200000 (full 2 MB AXISRAM minus the 1 KB gap) */
  EXTMEM_PSRAM (rw)     : ORIGIN = 0x70000000,   LENGTH = 0x01C00000   /* 0x70000000 .. 0x71C00000 */
}

/* Sections */
SECTIONS
{
  /* The startup code into "RAM" Ram type memory */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >RAM

  /* The program code and other data into "RAM" Ram type memory */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >RAM

  /* Constant data into "RAM" Ram type memory */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >RAM

  .ARM.extab   (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
   {
     . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >RAM

  .ARM (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
   {
     . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >RAM

  .preinit_array     (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >RAM

  .init_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >RAM

  .fini_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >RAM

  /* Used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections into "RAM" Ram type memory */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */

  } >RAM

  .noncacheable :
  {
    . = ALIGN(8);
    __snoncacheable = .;/* create symbol for start of section */
    KEEP(*(noncacheable_buffer))
    . = ALIGN(8);
    __enoncacheable = .;  /* create symbol for end of section */
  } > RAM

  .gnu.sgstubs :
  {
    . = ALIGN(4);
    *(.gnu.sgstubs*)   /* Secure Gateway stubs */
    . = ALIGN(4);
  } >RAM

  /* Large const data (the ST Edge AI epoch-controller blobs, ~250 KB per
   * model) parked in EXTMEM_PSRAM at runtime: VMA in PSRAM, LMA here in
   * AXISRAM (the FSBL's flat image copy knows nothing of PSRAM, and the
   * FSBL has already brought the XSPI2 PSRAM up before hand-off).
   * Reset_Handler copies LMA -> VMA before the .bss zero-init; .bss's VMA
   * then OVERLAPS the dead LMA range, so the net AXISRAM cost is ZERO.
   *
   * CRITICAL: use declarative AT(addr), never
   * "AT> RAM": AT> advances RAM's allocator cursor by the section size, so
   * .bss lands after the LMA instead of over it (zero savings). And the
   * ALIGN sits OUTSIDE the section so __ext_psram_init_lma == LOADADDR
   * exactly; an inner leading ALIGN would offset the copy source. ALIGN(64)
   * matches the ec_blobs' own cache-line alignment.
   *
   * Content opts in via __attribute__((section(".ext_psram_init"))); the
   * ECBLOB_CONST_SECTION define in the neuron6_user_model_N.c slots routes
   * every model's ec_blobs here. (A .rodata._ec_blob_* input pattern here
   * would be dead: .rodata's *(.rodata*) matches first in script order.) */
  . = ALIGN(64);
  __ext_psram_init_lma = .;

  .ext_psram_init : AT(__ext_psram_init_lma)
  {
    _sext_psram_init = .;     /* VMA start in PSRAM (== ADDR(section)) */
    *(.ext_psram_init)
    *(.ext_psram_init*)
    . = ALIGN(16);
    _eext_psram_init = .;     /* VMA end in PSRAM */
  } > EXTMEM_PSRAM

  _sidata_ext_psram_init = LOADADDR(.ext_psram_init);

  /* Uninitialized data section into "RAM" Ram type memory. Because the
   * AT(addr) above did NOT advance the RAM allocator cursor, .bss starts at
   * __ext_psram_init_lma and overlaps (reclaims) the LMA staging range. */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM

  /* Non-cacheable AXISRAM DMA-buffer band (the zero-copy DCMIPP->VENC line buffer).
   * NOLOAD: the FSBL never copies it (so it doesn't count toward the loaded image) and
   * the startup never zeroes it (DMA fills it). NOT pinned to a fixed address; placed
   * here (after .bss) wherever there is room; it may sit anywhere up to 0x34200000.
   * The MPU set-up marks exactly [__axisram_nc_start, __axisram_nc_end) non-cacheable (from
   * these symbols, not a hardcoded address), so the band is DMA-coherent with the D-cache
   * ENABLED. ALIGN(32) at both ends = the M55 cache-line / MPU granule, so the band
   * occupies whole cache lines and never shares one with the cacheable .bss before it or
   * the heap after it. Buffers land here via __attribute__((section(".axisram_nc"))). */
  .axisram_nc (NOLOAD) :
  {
    . = ALIGN(32);            /* cache-line / MPU granule (DCMIPP DMA also needs >=16) */
    __axisram_nc_start = .;
    KEEP(*(.axisram_nc))
    *(.axisram_nc*)
    . = ALIGN(32);
    __axisram_nc_end = .;
  } >RAM

    .ext_psram (NOLOAD) :
  {
    . = ALIGN(16);            /* DCMIPP requires 16-byte alignment */
    *(.ext_psram)             /* any variable placed in this section */
    *(.ext_psram*)            /* allow wildcards */
    . = ALIGN(32);
    /* No allocator pool here: this BSP allocates nothing at run time.
     * Every buffer is a variable the application declares, placed with a
     * section attribute - .axisram_nc for the encoder's band, on chip;
     * .ext_psram for frames and JPEG output, out here - so the application
     * decides where each one lives and the map file shows it. */
  } >EXTMEM_PSRAM

  /* User_heap_stack section, used to check that there is enough "RAM" Ram  type memory left.
   * NOLOAD: keeps the linker from giving it an LMA slot in the binary (it would follow
   * .ext_psram_init's LMA and bloat/collide); heap and stack are runtime-only anyway. */
  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* Enforce the 1.5 MB LOADED-image cap. The FSBL's copy ends at the LMA tail of
   * .ext_psram_init (the last loaded bytes, PAST _sbss, whose VMA overlaps the LMA
   * staging range). It must stay below the FSBL line at 0x34180000, or the copy would
   * clobber the running FSBL. .bss + the NOLOAD .axisram_nc band + heap + stack are
   * runtime-only and MAY extend above the line, up to 0x34200000 (the linker still
   * errors if the TOTAL overflows 0x34200000). */
  ASSERT(__ext_psram_init_lma + SIZEOF(.ext_psram_init) <= 0x34180000, "Loaded image exceeds the 1.5 MB FSBL line (0x34000400..0x34180000) -- shrink code/rodata/data/ec_blobs")

  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
