/* Stand-in for the Cortex-M55 core header, so the boot loader's own source
 * (FSBL/Core/Src/main.c) compiles and runs on a PC against the real device
 * and HAL headers. See ../README.md.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#ifndef FSBL_SIM_CORE_CM55_H
#define FSBL_SIM_CORE_CM55_H

#include <stdint.h>

#define __I   volatile const
#define __O   volatile
#define __IO  volatile
#define __IM  volatile const
#define __OM  volatile
#define __IOM volatile

#ifndef __STATIC_INLINE
#define __STATIC_INLINE      static inline
#endif
#ifndef __STATIC_FORCEINLINE
#define __STATIC_FORCEINLINE static inline
#endif
#ifndef __INLINE
#define __INLINE             inline
#endif
#ifndef __ASM
#define __ASM                __asm
#endif
#ifndef __NO_RETURN
#define __NO_RETURN          __attribute__((noreturn))
#endif
#ifndef __USED
#define __USED               __attribute__((used))
#endif
#ifndef __WEAK
#define __WEAK               __attribute__((weak))
#endif
#ifndef __PACKED
#define __PACKED             __attribute__((packed))
#endif
#ifndef __PACKED_STRUCT
#define __PACKED_STRUCT      struct __attribute__((packed))
#endif
#ifndef __ALIGNED
#define __ALIGNED(x)         __attribute__((aligned(x)))
#endif
#ifndef __RESTRICT
#define __RESTRICT           __restrict
#endif

/* The simulated board's hooks (fsbl_sim.c). */
void sim_irq_disabled(void);          /* the loader's Error_Handler stops here */
void sim_system_reset(void);          /* does not return */

#define __NOP() do { } while (0)
#define __DSB() do { } while (0)
#define __ISB() do { } while (0)
#define __WFI() do { } while (0)

static inline void     __disable_irq(void)        { sim_irq_disabled(); }
static inline void     __enable_irq(void)         { }
static inline uint32_t __get_PRIMASK(void)        { return 0u; }
static inline void     __set_PRIMASK(uint32_t v)  { (void)v; }
static inline uint32_t __CLZ(uint32_t v)          { return v ? (uint32_t)__builtin_clz(v) : 32u; }
static inline uint32_t __RBIT(uint32_t v)
{
  uint32_t r = 0u;
  for (int i = 0; i < 32; i++) { r = (r << 1) | (v & 1u); v >>= 1; }
  return r;
}

static inline void NVIC_SystemReset(void)                       { sim_system_reset(); }
static inline void NVIC_EnableIRQ(int irq)                      { (void)irq; }
static inline void NVIC_DisableIRQ(int irq)                     { (void)irq; }
static inline void NVIC_SetPriority(int irq, uint32_t prio)     { (void)irq; (void)prio; }
static inline void NVIC_ClearPendingIRQ(int irq)                { (void)irq; }

#endif /* FSBL_SIM_CORE_CM55_H */
