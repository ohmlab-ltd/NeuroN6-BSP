/**
 ******************************************************************************
 * @file    ll_aton_runtime.c
 * @author  SRA Artificial Intelligence & Embedded Architectures
 * @brief   ATON LL runtime.
 * @note    ATON LL runtime currently assumes a single network run
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>

#include "ll_aton_util.h" // Leave blank line after the include

#include "ll_aton.h"
#include "ll_aton_runtime.h"

#if defined(LL_ATON_RT_RELOC)
#include "ll_aton_reloc_network.h"
#endif

/* Helper macros for interrupt masks generation */
#if (ATON_INTCTRL_INTS(0) > 32)
#define ATON_INT_GET_MASK(_macro_, _unit_) (_macro_(_unit_, 0, 0) | (((uint64_t)_macro_(_unit_, 0, 1)) << 32))
#else
#define ATON_INT_GET_MASK(_macro_, _unit_) _macro_(_unit_, 0, 0)
#endif

/*** ATON RT Variables ***/

/* Check if current runtime is prepared for underlying ATON IP instance */
#if !defined(ATON_INTCTRL_INTS) || !defined(ATON_STRENG_NUM)
#error macros `ATON_INTCTRL_INTS` & `ATON_STRENG_NUM` MUST be defined but at least one is not!
#else // `ATON_INTCTRL_INTS` and `ATON_STRENG_NUM` are defined

#if (ATON_INTCTRL_INTS(0) > 64) || ((ATON_STRENG_NUM + ATON_STRENG_INT(0)) > 32)
#error current ATON runtime supports only up to 64 ATON interrupts and up to 32 streaming engines (with IRQ numbers lower than 32)!
#endif // (ATON_INTCTRL_INTS(0) > 64) || ((ATON_STRENG_NUM + ATON_STRENG_INT(0)) > 32)

#if defined(ATON_EPOCHCTRL_NUM) && (ATON_EPOCHCTRL_NUM > 32)
#error current ATON runtime supports only up to 32 epoch controllers!
#endif // (ATON_EPOCHCTRL_NUM > 32)

#endif // `ATON_INTCTRL_INTS` and `ATON_STRENG_NUM` are defined

LL_ATON_WEAK void dump_dma_state(void){};

/* Global variable for the current ATON IP owner */
const NN_Instance_TypeDef *volatile __ll_current_aton_ip_owner = NULL;
#ifndef NDEBUG
/* Current wait mask set */
uint32_t volatile __ll_current_wait_mask = 0;
#endif // !NDEBUG

/* Trace runtime callback */
static TraceRuntime_FuncPtr_t ll_aton_init_deinit_trace = NULL;

/* Forward declaration */
void ATON_STD_IRQHandler(void);

#define LL_ATON_DISABLE_ALL_IRQs()                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    LL_ATON_OSAL_DISABLE_IRQ(0);                                                                                       \
    LL_ATON_OSAL_DISABLE_IRQ(1);                                                                                       \
    LL_ATON_OSAL_DISABLE_IRQ(2);                                                                                       \
    LL_ATON_OSAL_DISABLE_IRQ(3);                                                                                       \
  } while (0)

/*** Helper Functions ***/

#ifndef NDEBUG
static uint32_t __LL_ATON_RT_CntEpochBlocks(const LL_ATON_RT_EpochBlockItem_t *list)
{
  int i = 0;

  if (list != NULL)
  {
    for (i = 1; !EpochBlock_IsLastEpochBlock(list); i++)
    { // Note: also terminating empty epoch block is counted
      list++;
    }
  }

  return i;
}
#endif

static inline void __LL_ATON_RT_ExecStartEpochBlock(const LL_ATON_RT_EpochBlockItem_t *eb,
                                                    NN_Instance_TypeDef *nn_instance)
{
  LL_ATON_ASSERT(nn_instance->exec_state.next_epoch_block == NULL);

  if (nn_instance->exec_state.epoch_callback_function != NULL)
    nn_instance->exec_state.epoch_callback_function(LL_ATON_RT_Callbacktype_PRE_START, nn_instance, eb);

  /* Is it the first epoch block in an AtoNN epoch? */
  if (EpochBlock_IsEpochStart(eb))
  {
    __LL_ATON_RT_Start_AtoNN_Epoch(nn_instance);
  }

  /* Grab ATON IP lock in case it's a HW epoch or an epoch blob */
  if (EpochBlock_IsEpochPureHW(eb)) // epoch blobs are flagged as pure HW, so checking for epoch blob is not necessary
  {
    __ll_set_aton_owner(nn_instance);
  }

  if (!EpochBlock_IsEpochBlob(eb))
  { // standard epoch block handling based on streaming engines
    /* set wait mask(s) in interrupt controller */
    if (EpochBlock_IsEpochPureHW(eb) || EpochBlock_IsEpochInternal(eb))
    {
      LL_ATON_ASSERT(__ll_current_aton_ip_owner == nn_instance);
      __LL_ATON_RT_SetWaitMask(eb->wait_mask);
    }
  }
  else
  { // epoch blob handling based on epoch controller
#if defined(ATON_EPOCHCTRL_NUM) &&                                                                                     \
    (LL_ATON_RT_MODE == LL_ATON_RT_ASYNC) // Polling mode is not allowed/supported when using the epoch controller
    /* reset wait mask(s) in interrupt controller, but ignore stream engine completion event interrupts */
    __LL_ATON_RT_SetWaitMask(ATON_STRENG_INT_MASK(ATON_STRENG_NUM, 0, 0));
#else // !ATON_EPOCHCTRL_NUM || LL_ATON_RT_POLLING
    LL_ATON_PRINTF("Trying to execute an epoch blob, but\n\t"
                   "- either ATON machine configuration does not contain an epoch controller unit or\n\t"
                   "- ATON runtime is configured for polling mode execution which does not support the usage of epoch "
                   "controllers\n");
#if (ATON_PLAT_HAS_FFLUSH)
    LL_ATON_FFLUSH(stdout);
#endif // ATON_PLAT_HAS_FFLUSH
    LL_ATON_ASSERT(false); // may never happen
#endif // !ATON_EPOCHCTRL_NUM || LL_ATON_RT_POLLING
  }

  if (eb->start_epoch_block != NULL)
  {
    LL_ATON_ASSERT(!EpochBlock_IsEpochPureSW(eb) && !EpochBlock_IsEpochHybrid(eb));

    /* start epoch block */
#if defined(LL_ATON_RT_RELOC)
    if (nn_instance->exec_state.inst_reloc != 0)
    {
      ai_rel_call_start_end_function(nn_instance->exec_state.inst_reloc, eb->start_epoch_block, eb, nn_instance);
    }
    else
    {
      eb->start_epoch_block(eb, nn_instance);
    }
#else
    eb->start_epoch_block(eb, nn_instance);
#endif
  }

  if (EpochBlock_IsEpochBlob(eb))
  {
#if defined(ATON_EPOCHCTRL_NUM)
    /* configure epoch controller */
    uint32_t ecId = EpochBlock_EpochControllerUnit(eb);
    LL_ATON_ASSERT(ecId < ATON_EPOCHCTRL_NUM); // may never happen

    LL_EpochCtrl_InitTypeDef conf;
    conf.stepmode = 0;
    conf.blobaddr = EpochBlock_EpochBlobAddr(eb);

    LL_EpochCtrl_Init(ecId, &conf);

    if (ATON_EPOCHCTRL_ENCRYPTION_EN(ecId) && EpochBlock_IsBlobEncrypted(eb))
    {
#if defined(LL_ATON_RT_RELOC)
      if (nn_instance->exec_state.inst_reloc != 0)
      {
        LL_EpochCtrl_EncryptionInit(ecId, ai_rel_network_blob_encryption_info(nn_instance->exec_state.inst_reloc));
      }
      else
      {
        LL_EpochCtrl_EncryptionInit(ecId, nn_instance->network->blob_encryption_info());
      }
#else
      // LL_ATON_ASSERT(nn_instance->network->blob_encryption_info()->enable == 1);
      LL_EpochCtrl_EncryptionInit(ecId, nn_instance->network->blob_encryption_info());
#endif
    }

    /* start/enable epoch controller */
    ATON_ENABLE(EPOCHCTRL, ecId);
#else  // !ATON_EPOCHCTRL_NUM
    LL_ATON_ASSERT(false); // may never happen
#endif // !ATON_EPOCHCTRL_NUM
  }

  if (nn_instance->exec_state.epoch_callback_function != NULL)
    nn_instance->exec_state.epoch_callback_function(LL_ATON_RT_Callbacktype_POST_START, nn_instance, eb);
}

static inline void __LL_ATON_RT_ExecEndEpochBlock(const LL_ATON_RT_EpochBlockItem_t *eb,
                                                  NN_Instance_TypeDef *nn_instance)
{
  if (nn_instance->exec_state.epoch_callback_function != NULL)
    nn_instance->exec_state.epoch_callback_function(LL_ATON_RT_Callbacktype_PRE_END, nn_instance, eb);

  if (EpochBlock_IsEpochBlob(eb))
  {
#if defined(ATON_EPOCHCTRL_NUM)
    /* stop/disable epoch controller */
    uint32_t ecId = EpochBlock_EpochControllerUnit(eb);
    LL_ATON_ASSERT(ecId < ATON_EPOCHCTRL_NUM); // may never happen
    uint32_t t;
    ATON_DISABLE_CLR_CONFCLR(EPOCHCTRL, ecId);

    /* disable epoch controller clock */
    LL_ATON_DisableClock(ATON_EPOCHCTRL_CLKB_CLK(ecId));
#else  // !ATON_EPOCHCTRL_NUM
    LL_ATON_ASSERT(false); // may never happen
#endif // !ATON_EPOCHCTRL_NUM
  }

  if (eb->end_epoch_block != NULL)
  {
#if defined(LL_ATON_RT_RELOC)
    if (nn_instance->exec_state.inst_reloc != 0)
    {
      ai_rel_call_start_end_function(nn_instance->exec_state.inst_reloc, eb->end_epoch_block, eb, nn_instance);
    }
    else
    {
      eb->end_epoch_block(eb, nn_instance);
    }
#else
    eb->end_epoch_block(eb, nn_instance);
#endif
  }

  if (EpochBlock_IsEpochPureHW(eb)) // epoch blobs are flagged as pure HW, so checking for epoch blob is not necessary
  {
    LL_ATON_ASSERT(nn_instance == __ll_current_aton_ip_owner);

    /* Reset wait mask */
    __LL_ATON_RT_SetWaitMask(0);

    /* Release ATON IP lock in case it's a pure HW epoch */
    __ll_clear_aton_owner(nn_instance, false);
  }

  LL_ATON_ASSERT(EpochBlock_IsEpochInternal(eb) || EpochBlock_IsEpochHybrid(eb) ||
                 (__ll_current_aton_ip_owner != nn_instance));

  if (nn_instance->exec_state.epoch_callback_function != NULL)
  {
    nn_instance->exec_state.epoch_callback_function(LL_ATON_RT_Callbacktype_POST_END, nn_instance, eb);
  }
}

static void __LL_ATON_RT_DetermineNextEpochBlock(NN_Instance_TypeDef *nn_instance)
{
  LL_ATON_ASSERT(nn_instance != NULL);
#if (LL_ATON_RT_MODE == LL_ATON_RT_ASYNC)
  LL_ATON_ASSERT(nn_instance->exec_state.triggered_events ==
                 0x0); // with the removal of parallel SW/HW epochs execution all triggered events must have been
                       // cleared at this point in time!
#endif                 // (LL_ATON_RT_MODE == LL_ATON_RT_ASYNC)

  /* Determine if there is a new inserted epoch block array */
  if ((nn_instance->exec_state.next_epoch_block != NULL))
  {
    LL_ATON_ASSERT(nn_instance->exec_state.saved_current_epoch_block == NULL);

    /* save current context */
    nn_instance->exec_state.saved_current_epoch_block = nn_instance->exec_state.current_epoch_block;
    nn_instance->exec_state.saved_first_epoch_block = nn_instance->exec_state.first_epoch_block;
#ifndef NDEBUG
    nn_instance->exec_state.saved_nr_of_epoch_blocks = nn_instance->exec_state.nr_of_epoch_blocks;
#endif

    /* set new context */
    nn_instance->exec_state.current_epoch_block = nn_instance->exec_state.next_epoch_block;
    nn_instance->exec_state.first_epoch_block = nn_instance->exec_state.next_epoch_block;
#ifndef NDEBUG
    nn_instance->exec_state.nr_of_epoch_blocks = __LL_ATON_RT_CntEpochBlocks(nn_instance->exec_state.first_epoch_block);
#endif

    /* reset next epoch block */
    nn_instance->exec_state.next_epoch_block = NULL;
  }
  else
  {
    nn_instance->exec_state.current_epoch_block++;
  }

#if (LL_ATON_RT_MODE == LL_ATON_RT_ASYNC)
  nn_instance->exec_state.current_epoch_block_started = false;
#endif
}

static inline uint32_t __LL_ATON_RT_GetWaitMask(const LL_ATON_RT_EpochBlockItem_t *eb)
{
  if (EpochBlock_IsEpochBlob(eb))
  {
    // in case of epoch blob `wait_mask` contains unit number of epoch controller to use
    return (1 << EpochBlock_EpochControllerUnit(eb));
  }
  else
  {
    return eb->wait_mask; // in case of "normal" epoch block `wait_mask` contains bitmask of (output) stream engines to
                          // wait for
  }
}

static inline void __LL_ATON_RT_Init_Network(NN_Instance_TypeDef *nn_instance)
{
  /** Exit if `nn_instance` is equal to NULL **/
  if (nn_instance == NULL)
  {
    return;
  }

  /** Exit if `nn_instance->network` is equal to NULL **/
  if (nn_instance->network == NULL)
  {
    return;
  }

  /** Initialize static variables **/
  /* set context */
#if defined(LL_ATON_RT_RELOC)
  const LL_ATON_RT_EpochBlockItem_t *eb_list;
  if (nn_instance->exec_state.inst_reloc != 0)
  {
    eb_list = ai_rel_network_get_epoch_items(nn_instance->exec_state.inst_reloc);
  }
  else
  {
    eb_list = nn_instance->network->epoch_block_items();
  }
#else
  const LL_ATON_RT_EpochBlockItem_t *eb_list = nn_instance->network->epoch_block_items();
#endif
  nn_instance->exec_state.current_epoch_block = eb_list;
  nn_instance->exec_state.first_epoch_block = eb_list;
  nn_instance->exec_state.next_epoch_block = NULL;

  /* set saved context */
  nn_instance->exec_state.saved_current_epoch_block = NULL;
  nn_instance->exec_state.saved_first_epoch_block = NULL;
#ifndef NDEBUG
  nn_instance->exec_state.nr_of_epoch_blocks = __LL_ATON_RT_CntEpochBlocks(nn_instance->exec_state.current_epoch_block);
  nn_instance->exec_state.saved_nr_of_epoch_blocks = 0;
#endif

  /* set information about running inference */
  nn_instance->exec_state.inference_started = false;

  /* set asynchronous status variables */
#if (LL_ATON_RT_MODE == LL_ATON_RT_ASYNC)
  nn_instance->exec_state.triggered_events = 0x0;
  nn_instance->exec_state.current_epoch_block_started = false;
#endif // (LL_ATON_RT_MODE == LL_ATON_RT_ASYNC)

  /** Call epoch callback with callback type `LL_ATON_RT_Callbacktype_NN_Init` and network instance **/
  if (nn_instance->exec_state.epoch_callback_function != NULL)
  {
    nn_instance->exec_state.epoch_callback_function(LL_ATON_RT_Callbacktype_NN_Init, nn_instance, NULL);
  }
}

/*** User API Functions ***/

/**
 * @brief Registers callbacks for ATON runtime related events (e.g. initialization/deinitialization, see
 * `LL_ATON_RT_Callbacktype_t`)
 * @param rt_callback Function pointer to callback function (set to `NULL` to disable epoch tracing)
 *
 * @note  This function must only be called when no network is currently executing!
 */
void LL_ATON_RT_SetRuntimeCallback(TraceRuntime_FuncPtr_t rt_callback)
{
  ll_aton_init_deinit_trace = rt_callback;
}

/**
 * @brief Register callback for tracing epoch/network related events (see `LL_ATON_RT_Callbacktype_t`)
 * @param epoch_block_callback Function pointer to callback function (set to `NULL` to disable epoch tracing)
 * @param nn_instance          Pointer to network instance for which to set the callback (may not be `NULL`)
 *
 * @deprecated This function is deprecated and will be removed in a future release.
 *             Use `LL_ATON_RT_SetNetworkCallback()` instead!
 */
void LL_ATON_RT_SetEpochCallback(TraceEpochBlock_FuncPtr_t epoch_block_callback, NN_Instance_TypeDef *nn_instance)
{
  LL_ATON_ASSERT(nn_instance != NULL);
  nn_instance->exec_state.epoch_callback_function = epoch_block_callback;
}

/**
 * @brief Register callback for tracing epoch/network related events (see `LL_ATON_RT_Callbacktype_t`)
 * @param nn_instance          Pointer to network instance for which to set the callback (may not be `NULL`)
 * @param epoch_block_callback Function pointer to callback function (set to `NULL` to disable epoch tracing)
 *
 * @note  This function must only be called while the passed network instance is not executing
 *        and should be called before `LL_ATON_RT_Init_Network()`!
 */
void LL_ATON_RT_SetNetworkCallback(NN_Instance_TypeDef *nn_instance, TraceEpochBlock_FuncPtr_t epoch_block_callback)
{
  LL_ATON_ASSERT(nn_instance != NULL);
  nn_instance->exec_state.epoch_callback_function = epoch_block_callback;
}

/**
 * @brief Initialize a network instance
 * @param nn_instance Pointer to network instance to initialize
 */
void LL_ATON_RT_Init_Network(NN_Instance_TypeDef *nn_instance)
{
  /** Exit if `nn_instance` is equal to NULL **/
  if (nn_instance == NULL)
  {
    return;
  }

  /** Exit if `nn_instance->network` is equal to NULL **/
  if (nn_instance->network == NULL)
  {
    return;
  }

  /* Care about epoch controller blobs relocation */
#if defined(LL_ATON_RT_RELOC)
  bool ret = false;
  if (nn_instance->exec_state.inst_reloc != 0)
  {
    ret = ai_rel_network_ec_network_init(nn_instance->exec_state.inst_reloc);
  }
  else
  {
    LL_ATON_ASSERT(nn_instance->network->ec_network_init != NULL);
    ret = nn_instance->network->ec_network_init();
  }
#else
  LL_ATON_ASSERT(nn_instance->network->ec_network_init != NULL);
  bool ret = nn_instance->network->ec_network_init();
#endif
  LL_ATON_ASSERT(ret == true);
  LL_ATON_LIB_UNUSED(ret);

  /* Call actual network instance initialization */
  __LL_ATON_RT_Init_Network(nn_instance);
}

/**
 * @brief De-initialize a network instance
 * @param nn_instance Pointer to network instance to de-initialize
 */
void LL_ATON_RT_DeInit_Network(NN_Instance_TypeDef *nn_instance)
{
  /** Exit if `nn_instance` is equal to NULL **/
  if (nn_instance == NULL)
  {
    return;
  }

  /** Call epoch callback with callback type `LL_ATON_RT_Callbacktype_NN_DeInit` and network instance **/
  if (nn_instance->exec_state.epoch_callback_function != NULL)
  {
    nn_instance->exec_state.epoch_callback_function(LL_ATON_RT_Callbacktype_NN_DeInit, nn_instance, NULL);
  }

  /** Re-set ATON IP owner */
  if (nn_instance == __ll_current_aton_ip_owner)
  { // In case this function gets called while an ATON lib internal EpochBlock (used to implement hybrid epochs) is
    // under execution we might still be owner of the ATON IP
    __ll_clear_aton_owner(nn_instance, true); // `true` means "allow for hard de-initialization of a network"
  }

  /** De-initialize static variables **/
  /* re-set context */
  const LL_ATON_RT_EpochBlockItem_t *eb_list = NULL;
  nn_instance->exec_state.current_epoch_block = eb_list;
  nn_instance->exec_state.first_epoch_block = eb_list;
  nn_instance->exec_state.next_epoch_block = NULL;

  /* re-set saved context */
  nn_instance->exec_state.saved_current_epoch_block = NULL;
  nn_instance->exec_state.saved_first_epoch_block = NULL;
#ifndef NDEBUG
  nn_instance->exec_state.nr_of_epoch_blocks = 0;
  nn_instance->exec_state.saved_nr_of_epoch_blocks = 0;
#endif

  /* intentional do not re-set information about running inference `nn_instance->exec_state.inference_started` */

  /* re-set asynchronous status variables */
#if (LL_ATON_RT_MODE == LL_ATON_RT_ASYNC)
  nn_instance->exec_state.triggered_events = 0x0;
  nn_instance->exec_state.current_epoch_block_started = false;
#endif // (LL_ATON_RT_MODE == LL_ATON_RT_ASYNC)
}

/**
 * @brief Reset network instance for getting ready for a new inference
 * @param nn_instance Pointer to network instance to initialize
 */
void LL_ATON_RT_Reset_Network(NN_Instance_TypeDef *nn_instance)
{
  LL_ATON_RT_DeInit_Network(nn_instance);
  __LL_ATON_RT_Init_Network(nn_instance);
}

/**
 * @brief Initialize the ATON runtime
 */
void LL_ATON_RT_RuntimeInit(void)
{
  /** Initialize ATON IPs **/
  LL_ATON_Init();

  /** Initialize IRQ Context **/
  {
    uint32_t t;

    /* Disable & Clear interrupt controller */
    ATON_DISABLE_CLR_CONFCLR(INTCTRL, 0);

    /* Preset Interrupt Controller masks */
    ATON_INTCTRL_STD_INTORMSK_SET(ATON_STRENG_INT_MASK(
        ATON_STRENG_NUM, 0, 0)); // OR-mask: disable all streaming engine events and enable all other events & errors
    ATON_INTCTRL_STD_INTANDMSK_SET(0xFFFFFFFF); // AND-mask: disable all events & errors

#if (ATON_INTCTRL_INTS(0) > 32)
    ATON_INTCTRL_STD_INTORMSK_H_SET(0);           // OR-mask: enable all events & errors
    ATON_INTCTRL_STD_INTANDMSK_H_SET(0xFFFFFFFF); // AND-mask: disable all events & errors
#endif

    /* Enable Interrupt Controller (again) */
    ATON_ENABLE(INTCTRL, 0);
  }

  /** Initialize OSAL layer **/
  LL_ATON_OSAL_INIT();

  /** Disable all four ATON interrupts **/
  LL_ATON_DISABLE_ALL_IRQs();

  /** Install IRQ handler **/
  LL_ATON_OSAL_INSTALL_IRQ(ATON_STD_IRQ_LINE, ATON_STD_IRQHandler);

  /** Enable ATON `ATON_STD_IRQ_LINE` interrupt **/
  LL_ATON_OSAL_ENABLE_IRQ(ATON_STD_IRQ_LINE);

  /** After having initialized ATON call callback (which among others might initialize further subsystems) */
  if (ll_aton_init_deinit_trace)
    ll_aton_init_deinit_trace(LL_ATON_RT_Callbacktype_RT_Init);
}

/**
 * @brief De-initialize the ATON runtime
 * @param nn_instance Pointer to network instance to de-initialize (optional - i.e. may be `NULL`, see
 * `LL_ATON_RT_DeInit_Network()`)
 */
void LL_ATON_RT_RuntimeDeInit(void)
{
  /* Call runtime de-init callback */
  if (ll_aton_init_deinit_trace)
    ll_aton_init_deinit_trace(LL_ATON_RT_Callbacktype_RT_Deinit);

  /* Disable all four ATON interrupts */
  LL_ATON_DISABLE_ALL_IRQs();

  /* Remove IRQ handler */
  LL_ATON_OSAL_REMOVE_IRQ(ATON_STD_IRQ_LINE);

  /* De-initialize OSAL layer */
  LL_ATON_OSAL_DEINIT();

  /* De-initialize ATON IPs */
  LL_ATON_DeInit();
}

/**
 * @brief  Checks status of previously started epoch block and
 *         starts execution of next epoch block of the NN provided the previous epoch block has terminated
 * @param nn_instance Pointer to network instance to run/continue (may not be `NULL`)
 * @retval LL_ATON_RT_NO_WFE  Next epoch block may be started, NN execution not finished, do not call
 *                            `LL_ATON_OSAL_WFE()`
 * @retval LL_ATON_RT_WFE     Epoch block is still running, NN execution not finished, you may call
 *                            `LL_ATON_OSAL_WFE()`.
 *                            NOTE, that in this case no other network instance may be run/continued from within the
 *                            same thread!
 *                            It is entirely the user's responsibility to comply with this restriction!
 * @retval LL_ATON_RT_DONE    NN execution finished
 */
LL_ATON_RT_RetValues_t LL_ATON_RT_RunEpochBlock(NN_Instance_TypeDef *nn_instance)
{
  LL_ATON_ASSERT(nn_instance != NULL);

  /* Test for wrong/missing initialization */
  LL_ATON_ASSERT(nn_instance->exec_state.current_epoch_block != NULL); // should never happen

  /* Check if network is starting a new inference */
  if (nn_instance->exec_state.inference_started == false)
  {
    /* Perform epoch controller blob relocation updates */
#if defined(LL_ATON_RT_RELOC)
    bool ret = false;
    if (nn_instance->exec_state.inst_reloc != 0)
    {
      ret = ai_rel_network_ec_inference_init(nn_instance->exec_state.inst_reloc);
    }
    else
    {
      LL_ATON_ASSERT((nn_instance->network != NULL) && (nn_instance->network->ec_inference_init != NULL));
      ret = nn_instance->network->ec_inference_init();
    }
#else
    LL_ATON_ASSERT((nn_instance->network != NULL) && (nn_instance->network->ec_inference_init != NULL));
    bool ret = nn_instance->network->ec_inference_init();
#endif

    LL_ATON_ASSERT(ret == true);
    LL_ATON_LIB_UNUSED(ret);

    /* Set inference started flag to `true` */
    nn_instance->exec_state.inference_started = true;

    /* Placeholder for things which need to be done before starting an inference */
    /* ==> here <== */
  }

#if (LL_ATON_RT_MODE == LL_ATON_RT_ASYNC)
  bool this_run_executed_end_epoch = false;
#endif // (LL_ATON_RT_MODE == LL_ATON_RT_ASYNC)

  while (true)
  {
#if (LL_ATON_RT_MODE == LL_ATON_RT_ASYNC)
    /* wait for current epoch block to finish */
    uint32_t _wait_mask = __LL_ATON_RT_GetWaitMask(nn_instance->exec_state.current_epoch_block);
    if (nn_instance->exec_state.current_epoch_block_started && (_wait_mask != 0))
    {
      if ((nn_instance->exec_state.triggered_events & _wait_mask) == _wait_mask)
      {
        /* Enter critical section */
        LL_ATON_ASSERT(__ll_current_aton_ip_owner ==
                       nn_instance); // when entering a critical section we MUST hold the ATON IP lock
        LL_ATON_OSAL_ENTER_CS();

        /* reset triggered events */
        nn_instance->exec_state.triggered_events &= ~_wait_mask;

        /* Exit critical section */
        LL_ATON_OSAL_EXIT_CS();

        /* end/clean-up epoch block */
        __LL_ATON_RT_ExecEndEpochBlock(nn_instance->exec_state.current_epoch_block, nn_instance);
        this_run_executed_end_epoch = true;

        /* advance epoch block */
        __LL_ATON_RT_DetermineNextEpochBlock(nn_instance);
      }
      else
      {
        /* Return to main loop */
        return LL_ATON_RT_WFE;
      }
    }
#endif // (LL_ATON_RT_MODE == LL_ATON_RT_ASYNC)

    /* test for last epoch block */
    if (EpochBlock_IsLastEpochBlock(nn_instance->exec_state.current_epoch_block))
    {
      if (nn_instance->exec_state.saved_current_epoch_block != NULL)
      {
        /* return from inserted epoch block */
        __LL_ATON_RT_RetFromLibEpochBlockArray(nn_instance);

        /* advance epoch block */
        nn_instance->exec_state.current_epoch_block++;

        /* Return to main loop (but do NOT call `LL_ATON_OSAL_WFE())`) */
        return LL_ATON_RT_NO_WFE;
      }
      else
      {
        /* Reached end of execution */
        return LL_ATON_RT_DONE;
      }
    }

    /* run/start current epoch block */
#if (LL_ATON_RT_MODE == LL_ATON_RT_ASYNC)
    if (this_run_executed_end_epoch)
    { // allow reset of network (see function `LL_ATON_RT_Reset_Network()`)
      /* Return to main loop (but do NOT call `LL_ATON_OSAL_WFE())`) */
      return LL_ATON_RT_NO_WFE;
    }

    if (!nn_instance->exec_state.current_epoch_block_started)
    {
      nn_instance->exec_state.current_epoch_block_started = true;

      __LL_ATON_RT_ExecStartEpochBlock(nn_instance->exec_state.current_epoch_block, nn_instance);
    }

    /* End epoch block and advance to next one */
    if (__LL_ATON_RT_GetWaitMask(nn_instance->exec_state.current_epoch_block) == 0x0)
    {
      /* end/clean-up epoch block */
      __LL_ATON_RT_ExecEndEpochBlock(nn_instance->exec_state.current_epoch_block, nn_instance);
      this_run_executed_end_epoch = true; // has no effect (just for cosmetics)

      /* advance epoch block */
      __LL_ATON_RT_DetermineNextEpochBlock(nn_instance);

      /* Return to main loop (but do NOT call `LL_ATON_OSAL_WFE())`) */
      return LL_ATON_RT_NO_WFE;
    }
    else
    {
      /* Return to main loop */
      return LL_ATON_RT_WFE;
    }

#else // (LL_ATON_RT_MODE == LL_ATON_RT_POLLING)

    __LL_ATON_RT_ExecStartEpochBlock(nn_instance->exec_state.current_epoch_block, nn_instance);

    /* wait for end of epoch block */
    uint32_t _wait_mask = __LL_ATON_RT_GetWaitMask(nn_instance->exec_state.current_epoch_block);
    if (_wait_mask != 0)
    {
      /* Perform polling wait */
      if (EpochBlock_IsEpochBlob(nn_instance->exec_state.current_epoch_block))
      {
#if defined(ATON_EPOCHCTRL_NUM)
        LL_EpochCtrl_Wait(_wait_mask);
#else  // !ATON_EPOCHCTRL_NUM
        LL_ATON_ASSERT(false); // may never happen
#endif // !ATON_EPOCHCTRL_NUM
      }
      else
      {
        LL_Streng_Wait(_wait_mask);
      }
    }

    /* End epoch block and advance to next one */
    __LL_ATON_RT_ExecEndEpochBlock(nn_instance->exec_state.current_epoch_block, nn_instance);

    /* advance epoch block */
    __LL_ATON_RT_DetermineNextEpochBlock(nn_instance);

    /* Return to main loop (but do NOT call `LL_ATON_OSAL_WFE())`) */
    return LL_ATON_RT_NO_WFE;

#endif // (LL_ATON_RT_MODE == LL_ATON_RT_POLLING)
  }
}

/*** ATON Irq Handler ***/

/* OhmLab fix (2026-07-16): ISR-SAFE ATON ERROR RECOVERY.
 *
 * The stock error path below did `LL_ATON_PRINTF(...)` (newlib printf) and
 * then `LL_ATON_ASSERT(false)` (asserts are ACTIVE in this core - no NDEBUG)
 * INSIDE the NPU interrupt handler. The first error/spurious ATON interrupt
 * therefore hung the CPU in ISR context at NVIC priority 5: every FreeRTOS
 * task - including n6_usb, which feeds the IWDG - stopped dead, and ~8 s
 * later the watchdog reset the board with NO crash record. Observed in the
 * field as an intermittent 20-100 s crash/recover loop while running
 * inference.
 *
 * This rewrite does what ST's own TODO here asked for: record the error,
 * acknowledge ALL unit-level interrupt sources (so the INTCTRL clear at the
 * end of `ATON_STD_IRQHandler` actually deasserts the line instead of
 * re-latching -> interrupt storm), and flag the inference task to abort.
 * the application's inference call sees `g_aton_err_abort`, drops the frame,
 * reports the recorded diagnostics from TASK context and recycles the ATON
 * runtime. Belt-and-braces: a burst of >32 unrecovered errors (a source
 * stuck at level despite the acks) disables the ATON NVIC lines outright;
 * the task-side `LL_ATON_RT_RuntimeInit()` re-enables them. */
volatile uint32_t g_aton_err_irq_count = 0; /* total error IRQs survived     */
volatile uint32_t g_aton_err_irqs_last = 0; /* INTCTRL INTREG at last error  */
volatile uint32_t g_aton_err_ec_irq = 0;    /* EC unit-0 IRQ reg at last err */
volatile uint32_t g_aton_err_ec_bc = 0;     /* EC unit-0 opcode counter      */
volatile uint32_t g_aton_err_no_owner = 0;  /* IRQs with no active network   */
volatile uint32_t g_aton_err_abort = 0;     /* -> Run_Inference: abort run   */
volatile uint32_t g_aton_err_burst = 0;     /* errors since last task clear  */

// REMEMBER: mask out all interrupt from parameter `irqs` you do NOT want to be handled in beyond function
#if (ATON_INTCTRL_INTS(0) > 32)
static void __LL_ATON_RT_IrqErr(uint64_t irqs)
#else  //(ATON_INTCTRL_INTS(0) <= 32)
static void __LL_ATON_RT_IrqErr(uint32_t irqs)
#endif //(ATON_INTCTRL_INTS(0) <= 32)
{
  int32_t i;

  if (!irqs)
    return;

  g_aton_err_irq_count++;
  g_aton_err_irqs_last = (uint32_t)irqs;

#ifdef ATON_STRENG_NUM
  /* Acknowledge ALL streaming-engine interrupt sources (error AND completion
   * views share the IRQ register). The run is being aborted anyway, and any
   * source left pending would re-latch into INTCTRL right after the
   * handler's INTCLR -> immediate re-entry storm. */
  for (i = 0; i < ATON_STRENG_NUM; i++)
  {
    uint32_t s = ATON_STRENG_IRQ_GET(i);
    if (s != 0u)
      ATON_STRENG_IRQ_SET(i, s);
  }
#endif // ATON_STRENG_NUM

#if defined(ATON_EPOCHCTRL_NUM)
  /* Acknowledge epoch-controller sources (ERROR / NOACK / completion),
   * capturing unit 0's state first for the task-context report. */
  for (i = 0; i < ATON_EPOCHCTRL_NUM; i++)
  {
    uint32_t e = ATON_EPOCHCTRL_IRQ_GET(i);
    if (i == 0)
    {
      g_aton_err_ec_irq = e;
      g_aton_err_ec_bc = ATON_EPOCHCTRL_BC_GET(0);
    }
    if (e != 0u)
      ATON_EPOCHCTRL_IRQ_SET(i, e);
  }
#endif // ATON_EPOCHCTRL_NUM

#ifdef ATON_BUSIF_NUM
  /* Observe (and, where read-clears, clear) latched bus-interface error
   * state. Harmless if not read-to-clear: the INTCTRL latch is cleared by
   * the caller and a stuck source trips the burst backstop below. */
  for (i = 0; i < ATON_BUSIF_NUM; i++)
    (void)ATON_BUSIF_ERR_GET(i);
#endif // ATON_BUSIF_NUM

  /* Tell the inference task to abort this run and recycle the runtime. It is
   * woken by the unconditional `LL_ATON_OSAL_SIGNAL_EVENT()` at the end of
   * `ATON_STD_IRQHandler`, so it cannot be left blocked in WFE. */
  g_aton_err_abort = 1u;

  /* Storm backstop: if errors keep arriving before the task-side recovery
   * runs (source stuck at level despite the acks above), silence the ATON
   * NVIC lines; `LL_ATON_RT_RuntimeInit()` in the recovery re-enables them. */
  if (++g_aton_err_burst > 32u)
    LL_ATON_DISABLE_ALL_IRQs();
}

#if (LL_ATON_RT_MODE == LL_ATON_RT_ASYNC)
#if (ATON_INTCTRL_INTS(0) > 32)
static inline void __LL_ATON_RT_IrqEpochBlock(uint64_t irqs)
#else  //(ATON_INTCTRL_INTS(0) <= 32)
static inline void __LL_ATON_RT_IrqEpochBlock(uint32_t irqs)
#endif //(ATON_INTCTRL_INTS(0) <= 32)
{
  int32_t i;

  /** AND-mask interrupts MUST be handled here **/
  /* Deal with Streaming Engine interrupts */
#if (ATON_INTCTRL_INTS(0) > 32)
  uint64_t wait_irqs;
#else  //(ATON_INTCTRL_INTS(0) <= 32)
  uint32_t wait_irqs;
#endif //(ATON_INTCTRL_INTS(0) <= 32)

  /* Beyond code assumes that stream engine interrupts are assigned in the order of their engine number and to
   * consecutive bits within the `INTREG` register (and within all other interrupt controller registers, like e.g.
   * status/mask/clear)! */
  irqs >>= ATON_STRENG_INT(0);
  wait_irqs =
      irqs &
      __ll_current_aton_ip_owner->exec_state.current_epoch_block
          ->wait_mask; /* treat only IRQs we are currently waiting for
                          (Note: we might be running in a hybrid function which uses DMAs in parallel with a "normal"
                          ATON execution and we must not clear the IRQs of this "normal" ATON execution here) */
  if (wait_irqs)
  {
    uint32_t _tmp_triggered_events = __ll_current_aton_ip_owner->exec_state.triggered_events;
    for (i = 0; i < ATON_STRENG_NUM; i++)
    {
      /* Handle event interrupts */
      if ((wait_irqs >> i) & 1)
      { /* more future-proofed but less efficient alternative:
           `if (wait_irqs & ATON_STRENG_INT_MASK(i, 0, 0))`
         */
        uint32_t strengIrqs = ATON_STRENG_IRQ_GET(i);
        ATON_STRENG_IRQ_SET(
            i, strengIrqs); /* Acknowledge ATON interrupt source (i.e. stream engine #i) - could be more fine grain */

        /* Handle RT integration */
        _tmp_triggered_events |= (1 << i);
      }
    }
    ((NN_Instance_TypeDef *)__ll_current_aton_ip_owner)->exec_state.triggered_events = _tmp_triggered_events;
  }
}

#if defined(ATON_EPOCHCTRL_NUM)
#if (ATON_INTCTRL_INTS(0) > 32)
static inline void __LL_ATON_RT_IrqEpochBlob(uint64_t irqs)
#else  //(ATON_INTCTRL_INTS(0) <= 32)
static inline void __LL_ATON_RT_IrqEpochBlob(uint32_t irqs)
#endif //(ATON_INTCTRL_INTS(0) <= 32)
{
  uint32_t ecId = EpochBlock_EpochControllerUnit(__ll_current_aton_ip_owner->exec_state.current_epoch_block);
  LL_ATON_ASSERT(ecId < ATON_EPOCHCTRL_NUM); // may never happen
  if (irqs & ATON_INT_GET_MASK(ATON_EPOCHCTRL_INT_MASK, ecId))
  {
    /* Acknowledge interrupts in active epoch controller unit - could be more fine grain */
    uint32_t ecIrqs = ATON_EPOCHCTRL_IRQ_GET(ecId);
    ATON_EPOCHCTRL_IRQ_SET(ecId, ecIrqs);

    /* Handle RT integration */
    uint32_t _tmp_triggered_events = __ll_current_aton_ip_owner->exec_state.triggered_events;
    _tmp_triggered_events |= (1 << ecId);
    ((NN_Instance_TypeDef *)__ll_current_aton_ip_owner)->exec_state.triggered_events = _tmp_triggered_events;
  }
}
#endif // ATON_EPOCHCTRL_NUM
#endif // (LL_ATON_RT_MODE == LL_ATON_RT_ASYNC)

/* ATON ISR
 * ll_aton routes all interrupts to `ATON_STD_IRQ_LINE` interrupt line */
void ATON_STD_IRQHandler(void)
{
  /** Figure out which interrupt(s) fired **/
#if (ATON_INTCTRL_INTS(0) > 32)
  uint32_t irqs_l = ATON_INTCTRL_INTREG_GET(0);
  uint32_t irqs_h = ATON_INTCTRL_INTREG_H_GET(0);
  uint64_t irqs = (uint64_t)irqs_l | ((uint64_t)irqs_h << 32);
#else  //(ATON_INTCTRL_INTS(0) <= 32)
  uint32_t irqs = ATON_INTCTRL_INTREG_GET(0);
#endif //(ATON_INTCTRL_INTS(0) <= 32)

#if (LL_ATON_RT_MODE == LL_ATON_RT_ASYNC)
  /* OhmLab fix (2026-07-16): guard the NULL-owner / NULL-epoch-block cases
   * (reachable when a straggler interrupt lands in the per-frame window
   * between `LL_ATON_RT_DeInit_Network()` and the next `..._Init_Network()`).
   * The stock code asserted here (active assert -> abort() in ISR -> IWDG
   * reset) and then dereferenced the NULL owner. Now: record + acknowledge
   * everything via `__LL_ATON_RT_IrqErr()` and fall through to the INTCLR. */
  if ((__ll_current_aton_ip_owner == NULL) ||
      (__ll_current_aton_ip_owner->exec_state.current_epoch_block == NULL))
  {
    g_aton_err_no_owner++;
    __LL_ATON_RT_IrqErr(irqs); /* treat all interrupts as errors */
  }
  else if (!EpochBlock_IsEpochBlob(__ll_current_aton_ip_owner->exec_state
                                       .current_epoch_block)) // standard epoch block handling based on streaming
                                                              // engines
  {
    /** OR-mask (error) interrupts MUST be handled first **/
    __LL_ATON_RT_IrqErr(
        irqs & ~ATON_STRENG_INT_MASK(ATON_STRENG_NUM, 0, 0)); /* exclude all streaming engine completion interrupts */
    __LL_ATON_RT_IrqEpochBlock(irqs);
  }
  else // epoch blob handling based on epoch controller
  {
#if defined(ATON_EPOCHCTRL_NUM)
    uint32_t ecId = EpochBlock_EpochControllerUnit(__ll_current_aton_ip_owner->exec_state.current_epoch_block);
    if (ecId < ATON_EPOCHCTRL_NUM)
    {
      /** OR-mask (error) interrupts MUST be handled first **/
      __LL_ATON_RT_IrqErr(
          irqs & ~ATON_INT_GET_MASK(ATON_EPOCHCTRL_INT_MASK,
                                    ecId)); /* exclude epoch controller interrupt for active epoch controller unit */
      __LL_ATON_RT_IrqEpochBlob(irqs);
    }
    else // corrupt epoch block: treat everything as an error (was assert)
    {
      g_aton_err_no_owner++;
      __LL_ATON_RT_IrqErr(irqs);
    }
#else  // !ATON_EPOCHCTRL_NUM
    g_aton_err_no_owner++;
    __LL_ATON_RT_IrqErr(irqs); /* was `LL_ATON_ASSERT(false)` */
#endif // !ATON_EPOCHCTRL_NUM
  }
#else  // (LL_ATON_RT_MODE == LL_ATON_RT_POLLING)
  __LL_ATON_RT_IrqErr(irqs); /* treat all interrupts as errors */
#endif // (LL_ATON_RT_MODE == LL_ATON_RT_POLLING)

#if (LL_ATON_RT_MODE == LL_ATON_RT_ASYNC)

  /* Data Synchronization Barrier */
  LL_ATON_DSB();

  /* Clear all interrupts in interrupt controller.
   * Note: this must be done *after* having cleared ATON interrupt sources otherwise
   * interrupts will be re-latched.
   */
#if (ATON_INTCTRL_INTS(0) > 32)
  ATON_INTCTRL_INTCLR_SET(0, irqs_l);
  ATON_INTCTRL_INTCLR_H_SET(0, irqs_h);
#else  //(ATON_INTCTRL_INTS(0) <= 32)
  ATON_INTCTRL_INTCLR_SET(0, irqs);
#endif //(ATON_INTCTRL_INTS(0) <= 32)

  /* Data Synchronization Barrier */
  LL_ATON_DSB();

  /* Signal event */
  LL_ATON_OSAL_SIGNAL_EVENT();

#endif // (LL_ATON_RT_MODE == LL_ATON_RT_ASYNC)

  return;
}
