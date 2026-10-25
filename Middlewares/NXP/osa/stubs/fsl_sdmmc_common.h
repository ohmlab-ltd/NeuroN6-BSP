/**
  ******************************************************************************
  * @file    fsl_sdmmc_common.h  (STM32N6 stub)
  * @brief   Minimal stub for NXP MCUXpresso's SDMMC stack header.
  *
  * The wifi_nxp middleware was originally written against NXP's uSDHC + SDMMC
  * stack. We replaced the SDIO transport entirely with a STM32 HAL_SDIO shim
  * (Drivers/Wireless/sdio_port/), so none of the NXP SDMMC types are actually
  * used at runtime: only the macros and forward declarations referenced in
  * wifi_nxp/wifidriver/sdio.h and friends to compile.
  *
  * SPDX-License-Identifier: BSD-3-Clause
  ******************************************************************************
  */

#ifndef FSL_SDMMC_COMMON_H_STM32N6_STUB
#define FSL_SDMMC_COMMON_H_STM32N6_STUB

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Standard SDIO block size, used by wifi_nxp/wifidriver/sdio.h to size
 * outbuf / inbuf for CMD53 transfers. */
#ifndef FSL_SDMMC_DEFAULT_BLOCK_SIZE
#define FSL_SDMMC_DEFAULT_BLOCK_SIZE  (512U)
#endif

/* Opaque forward declarations: wifi_nxp upper layers only ever take
 * pointers to these types; they never dereference them since our SDIO
 * shim handles all real I/O. */
typedef struct _sdio_card     sdio_card_t;
typedef struct _sdmmc_host    sdmmchost_t;

/* Function-number enum used as an integer everywhere it appears. */
typedef enum
{
    kSDIO_FunctionNum0 = 0U,
    kSDIO_FunctionNum1 = 1U,
    kSDIO_FunctionNum2 = 2U,
    kSDIO_FunctionNum3 = 3U,
    kSDIO_FunctionNum4 = 4U,
    kSDIO_FunctionNum5 = 5U,
    kSDIO_FunctionNum6 = 6U,
    kSDIO_FunctionNum7 = 7U,
} sdio_func_num_t;

/* Voltage enum, referenced by some board-glue paths. Only
 * operate at 1.8V on MAYA-W4 (set by wifi_bt_module_config.h). */
typedef enum
{
    kSDMMC_OperationVoltage_None = 0U,
    kSDMMC_OperationVoltage180V  = 1U,
    kSDMMC_OperationVoltage300V  = 2U,
    kSDMMC_OperationVoltage330V  = 3U,
} sdmmc_operation_voltage_t;

#ifdef __cplusplus
}
#endif

#endif /* FSL_SDMMC_COMMON_H_STM32N6_STUB */
