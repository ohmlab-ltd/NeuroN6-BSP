/* Stand-in for the device header, for test_asset.c and test_meta_box.c.
 * MIT, Ohm Lab Ltd. */
#pragma once
#include <stdint.h>
#include <stdlib.h>
typedef struct { volatile uint32_t BKP0R; volatile uint32_t rest[64]; } TAMP_TypeDef;
extern TAMP_TypeDef g_tamp;
#define TAMP (&g_tamp)
typedef struct { volatile uint32_t DBPCR; } PWR_TypeDef;
extern PWR_TypeDef g_pwr;
#define PWR (&g_pwr)
#define PWR_DBPCR_DBP (1U << 0)
#define __DSB() do { } while (0)
/* A reset ends the call that asked for it: the test jumps back. */
void fake_system_reset(void);
static inline void NVIC_SystemReset(void) { fake_system_reset(); }
/* The cycle counter and its enables, as vnd_meta.c uses them. */
typedef struct { volatile uint32_t CTRL; volatile uint32_t CYCCNT; } DWT_Type;
typedef struct { volatile uint32_t DEMCR; } DCB_Type;
extern DWT_Type g_dwt;
extern DCB_Type g_dcb;
#define DWT (&g_dwt)
#define DCB (&g_dcb)
#define DWT_CTRL_CYCCNTENA_Msk (1UL)
#define DCB_DEMCR_TRCENA_Msk   (1UL << 24)
/* One thread on the PC: there is no interrupt to mask. */
static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __set_PRIMASK(uint32_t primask) { (void)primask; }
static inline void __disable_irq(void) { }
/* The data cache over the mapped flash, as the test models it. */
#define __DCACHE_PRESENT 1U
void SCB_InvalidateDCache_by_Addr(volatile void *addr, int32_t dsize);
