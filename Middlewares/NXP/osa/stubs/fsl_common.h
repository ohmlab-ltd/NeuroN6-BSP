/**
  ******************************************************************************
  * @file    fsl_common.h  (STM32N6 stub)
  * @brief   Minimal stand-in for NXP MCUXpresso's fsl_common.h, provided so
  *          that wifi_nxp's OSA layer (fsl_os_abstraction.h /
  *          fsl_os_abstraction_free_rtos.c) compiles on STM32N6 without
  *          dragging the entire NXP HAL infrastructure
  *          (fsl_device_registers.h → MCU-specific CMSIS, fsl_clock.h, ...)
  *
  * This header intentionally provides ONLY the symbols that fsl_os_abstraction
  * and the linked-list utility actually use, namely:
  *   - status_t typedef
  *   - MAKE_STATUS macro
  *   - kStatus_Success, kStatus_Fail, kStatus_Timeout, ...
  *   - kStatusGroup_Generic, kStatusGroup_OSA, kStatusGroup_LIST
  *
  * Anything else from the upstream fsl_common.h (NVIC helpers, clock init,
  * interrupt enable wrappers, weak-attribute macros, etc.) is NOT here.
  * If a translation unit pulls fsl_common.h and needs more than the OSA
  * subset, extend this stub instead of importing the full SDK header.
  *
  * SPDX-License-Identifier: BSD-3-Clause
  ******************************************************************************
  */

#ifndef FSL_COMMON_H_STM32N6_STUB
#define FSL_COMMON_H_STM32N6_STUB

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------------------
 *  Status type / construction macro
 *  Identical to upstream fsl_common.h, so status_t values match NXP's
 *  prebuilt code.
 * --------------------------------------------------------------------------*/

typedef int32_t status_t;

#ifndef MAKE_STATUS
#define MAKE_STATUS(group, code) ((((group) * 100L) + (code)))
#endif

#ifndef MAKE_VERSION
#define MAKE_VERSION(major, minor, bugfix) (((major) * 65536L) + ((minor) * 256L) + (bugfix))
#endif


/* ----------------------------------------------------------------------------
 *  Status groups
 *  Values copied exactly from upstream fsl_common.h so any third-party code
 *  that compares against numeric status_t literals still works.
 * --------------------------------------------------------------------------*/

enum _status_groups
{
    kStatusGroup_Generic = 0,    /*!< Generic status codes */
    kStatusGroup_LIST    = 142,  /*!< Linked-list helper   */
    kStatusGroup_OSA     = 143,  /*!< OS abstraction       */
};


/* ----------------------------------------------------------------------------
 *  Generic status codes (what the OSA references)
 * --------------------------------------------------------------------------*/

enum _generic_status
{
    kStatus_Success         = MAKE_STATUS(kStatusGroup_Generic, 0),
    kStatus_Fail            = MAKE_STATUS(kStatusGroup_Generic, 1),
    kStatus_ReadOnly        = MAKE_STATUS(kStatusGroup_Generic, 2),
    kStatus_OutOfRange      = MAKE_STATUS(kStatusGroup_Generic, 3),
    kStatus_InvalidArgument = MAKE_STATUS(kStatusGroup_Generic, 4),
    kStatus_Timeout         = MAKE_STATUS(kStatusGroup_Generic, 5),
    kStatus_NoTransferInProgress = MAKE_STATUS(kStatusGroup_Generic, 6),
    kStatus_Busy            = MAKE_STATUS(kStatusGroup_Generic, 7),
    kStatus_NoData          = MAKE_STATUS(kStatusGroup_Generic, 8),
};


/* ----------------------------------------------------------------------------
 *  ARRAY_SIZE, used by wifi_nxp upper layers (mlan_11d.c, wlan.c, etc.)
 * --------------------------------------------------------------------------*/

#if !defined(ARRAY_SIZE)
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/* ----------------------------------------------------------------------------
 *  Cache-line alignment for SDIO DMA buffers (Cortex-M55 D-cache line is 32B).
 *  Used by wifi_nxp/wifidriver/sdio.c via SDK_ALIGN(...) for outbuf/inbuf.
 * --------------------------------------------------------------------------*/

#if !defined(BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE)
#define BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE  (32U)
#endif


/* ----------------------------------------------------------------------------
 *  Compiler attribute helpers used in OSA / generic_list source files
 * --------------------------------------------------------------------------*/

#if defined(__GNUC__) || defined(__clang__)
  #ifndef __WEAK_FUNC
    #define __WEAK_FUNC __attribute__((weak))
  #endif
  #ifndef __ALIGNED
    #define __ALIGNED(x) __attribute__((aligned(x)))
  #endif
  /* NXP SDK_ALIGN, used by wifi_nxp to align large MAC buffers (e.g.
   * BSSDescriptor_t BSS_List[]) for DMA / cache-line boundary. GCC syntax. */
  #ifndef SDK_ALIGN
    #define SDK_ALIGN(var, alignbytes) var __attribute__((aligned(alignbytes)))
  #endif
#elif defined(__ICCARM__)
  #ifndef __WEAK_FUNC
    #define __WEAK_FUNC __weak
  #endif
  #ifndef __ALIGNED
    #define __ALIGNED(x) _Pragma("data_alignment="#x)
  #endif
  #ifndef SDK_ALIGN
    #define SDK_PRAGMA(x) _Pragma(#x)
    #define SDK_ALIGN(var, alignbytes) SDK_PRAGMA(data_alignment = alignbytes) var
  #endif
#endif


/* ----------------------------------------------------------------------------
 *  Minimal interrupt-disable wrapper (used by some upstream OSA code paths)
 *  Maps to CMSIS primitives so it works identically on Cortex-M55.
 * --------------------------------------------------------------------------*/

#include "stm32n6xx_hal.h"   /* pulls cmsis_compiler.h → __disable_irq / __enable_irq */

static inline uint32_t DisableGlobalIRQ(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void EnableGlobalIRQ(uint32_t primask)
{
    __set_PRIMASK(primask);
}


#ifdef __cplusplus
}
#endif

#endif /* FSL_COMMON_H_STM32N6_STUB */
