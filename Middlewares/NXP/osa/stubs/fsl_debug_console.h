/**
  ******************************************************************************
  * @file    fsl_debug_console.h  (STM32N6 stub)
  * @brief   Minimal stand-in for NXP MCUXpresso's fsl_debug_console.h.
  *
  * wifi_nxp's wmlog.h, wmerrno.h, and various other middleware files include
  * this header for the PRINTF macro (and a handful of debug-console helpers
  * that we don't care about on STM32). Rather than pull NXP's full
  * debug_console subsystem (UART/LPUART/FlexComm-specific), we provide a
  * thin layer that maps to the STM32 newlib `printf` family.
  *
  * If you have a debug UART hooked up via syscalls.c, output will appear
  * there. If you don't, the calls compile away to no-ops.
  *
  * SPDX-License-Identifier: BSD-3-Clause
  ******************************************************************************
  */

#ifndef FSL_DEBUG_CONSOLE_H_STM32N6_STUB
#define FSL_DEBUG_CONSOLE_H_STM32N6_STUB

#include <stdio.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The middleware uses these via wmlog.h. PRINTF is the workhorse. */
#ifndef PRINTF
#define PRINTF(...)        ((void)printf(__VA_ARGS__))
#endif

#ifndef DbgConsole_Printf
#define DbgConsole_Printf  printf
#endif

#ifndef DbgConsole_Scanf
#define DbgConsole_Scanf   scanf
#endif

#ifndef DbgConsole_Putchar
#define DbgConsole_Putchar putchar
#endif

#ifndef DbgConsole_Getchar
#define DbgConsole_Getchar getchar
#endif

/* Some upstream code uses SCANF and GETCHAR uppercase. */
#ifndef SCANF
#define SCANF(...)         ((void)scanf(__VA_ARGS__))
#endif

#ifndef PUTCHAR
#define PUTCHAR(c)         ((void)putchar(c))
#endif

#ifndef GETCHAR
#define GETCHAR()          getchar()
#endif

/* DbgConsole_Init / DbgConsole_Deinit are called by some bring-up paths.
 * No-ops on STM32: the existing UART/USB log channel is already up. */
static inline int DbgConsole_Init(void *uartBase, uint32_t baudRate, uint8_t device, uint32_t clkSrcFreq)
{
    (void)uartBase; (void)baudRate; (void)device; (void)clkSrcFreq;
    return 0;
}

static inline int DbgConsole_Deinit(void) { return 0; }

#ifdef __cplusplus
}
#endif

#endif /* FSL_DEBUG_CONSOLE_H_STM32N6_STUB */
