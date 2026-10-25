/**
  ******************************************************************************
  * @file    ewl_conf.h
  * @brief   Encoder Wrapper Layer configuration for the Ohm Lab Neuro N6.
  *
  *          Upstream ships this as EWL_conf_template.h, to be copied into the
  *          application and edited. This is that copy. Everything it selects
  *          is a code path the vendor sources already implement in full; no
  *          vendor source is changed by any choice made here.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  *
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE.txt
  * file in this directory.
  *
  ******************************************************************************
  */

#ifndef EWL_CONF_H
#define EWL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#define EWL_USE_MALLOC_MM 0
#define EWL_USE_FREERTOS_MM 1
#define EWL_USE_THREADX_MM 2
#define EWL_USE_STM32MPM_MM 3
#define EWL_USER_MM 4

/* Memory manager. The encoder allocates its instance and its internal working
 * buffers through this, once at BSP_VENC_Init and never per frame, so the
 * malloc back end is enough and needs no scheduler. It does mean the encoder
 * wants a real heap: see BSP_VENC_HEAP_BYTES in neuron6_venc.h for how much,
 * and why an application that has never needed one until now will find this
 * out as an init failure rather than a fault. */
#define EWL_ALLOC_API EWL_USE_MALLOC_MM

#define EWL_USE_POLLING_SYNC 0
#define EWL_USE_FREERTOS_SYNC 1
#define EWL_USE_THREADX_SYNC 2
/* NEURON6 addition. Interrupt-driven, with no operating system: the same
 * interrupt handling as the RTOS back ends, signalling a flag instead of an
 * event group. See EWL_SYNC_API below. */
#define EWL_USE_BAREMETAL_SYNC 3
#define EWL_USER_SYNC 4

/* Hardware synchronisation. Polling, because it is the one that works from a
 * cold start with no operating system, and that was established the hard way.
 *
 * The vendor sources ship three back ends: a polling one, and two that wait on
 * an RTOS primitive. This BSP has no scheduler, so the RTOS ones are out. That
 * leaves polling, or the bare-metal interrupt back end below
 * (EWL_USE_BAREMETAL_SYNC), which reproduces the RTOS handlers with a
 * volatile flag instead of an event group. The interrupt back end does not
 * work from a cold boot: the VENC frame-ready interrupt never arrives from a
 * clean peripheral state, the wait flag is never set, and the encode spins
 * forever. It can appear to work on a board reflashed with the power left on,
 * because the encoder's interrupt path is then inherited already running. The
 * polling back end reads the status register in its own loop and depends on
 * none of that, so it works from any state. Verified: memory-source and
 * line-buffer encodes, ten of ten
 * frames, from a cold boot with the core never having run.
 *
 * The concern that drove the interrupt experiment - that a poll loop clears
 * the per-macroblock-row status too late and tears the band under load - did
 * not materialise: the polling EWLWaitHwRdy clears that status inside its loop
 * on the same iteration it sees it, and the streamed frames are clean. If a
 * future high-rate mode does tear, the answer is an RTOS back end
 * (EWL_USE_FREERTOS_SYNC / EWL_USE_THREADX_SYNC, with the include below and
 * BSP_VENC_WaitBeginHook implemented), not the bare-metal interrupt one, which
 * is kept below only so the cold-start finding is not lost.
 *
 * EWL_USE_BAREMETAL_SYNC remains defined and compiled, but is not selected.
 * Do not select it without proving an encode from a cold boot first.
 *
 * Under FreeRTOS (BSP_USE_FREERTOS in neuron6_conf.h) the RTOS back end is
 * selected instead: the wait blocks on an event group that VENC_IRQHandler
 * sets, and the interrupt it enables sits at priority 5, the highest at
 * which the port allows a
 * FromISR call. Polling would work there too, but would hold the encoding
 * task at full priority for the whole frame. */
#include "neuron6_conf.h"
#ifndef EWL_SYNC_API
#if defined(BSP_USE_FREERTOS) && (BSP_USE_FREERTOS == 1)
#define EWL_SYNC_API EWL_USE_FREERTOS_SYNC
#else
#define EWL_SYNC_API EWL_USE_POLLING_SYNC
#endif
#endif

/* #define TRACE_EWL */

/* How long the wait for the encoder lasts before it reports a timeout, in
 * milliseconds; ST's default is 100. A line-buffer encode's wait runs from
 * arming the encoder to the end of the camera's frame, so it has to cover the
 * slowest frame and the wait for it to start: at QSXGA, 15 fps, the first
 * frame after the camera starts can take a frame period, 67 ms, to begin and
 * then 66 ms of lines. At 100 ms every such encode timed out. */
#define EWL_TIMEOUT 300UL

#define ALIGNMENT_INCR 8UL

#define MEM_CHUNKS      32

#if (EWL_ALLOC_API == EWL_USE_MALLOC_MM)
#include <stdlib.h>
#endif

#if (EWL_ALLOC_API == EWL_USE_FREERTOS_MM) || (EWL_SYNC_API == EWL_USE_FREERTOS_SYNC)
#include "FreeRTOS.h"
#include "event_groups.h"
#endif

#if (EWL_ALLOC_API == EWL_USE_THREADX_MM) || (EWL_SYNC_API == EWL_USE_THREADX_SYNC)
#include "tx_api.h"
#endif

#ifdef  USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

#ifdef __cplusplus
}
#endif

#endif /* EWL_CONF_H */
