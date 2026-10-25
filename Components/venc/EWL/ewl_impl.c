/**
  ******************************************************************************
  * @file    ewl_impl.c
  * @author  GPM Application Team
  * @brief   Encoder Wrapper Layer for Video Encoder (VENC) IP
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ewl.h"
#include "ewl_impl.h"
#include "stm32n6xx_hal.h"
#include "stm32n6xx_ll_venc.h"
#include "reg_offset_v7.h"



/* Global variables ----------------------------------------------------------*/

#ifdef TRACE_EWL
/* static strings used to display hardware configuration in trace */
static const char *busTypeName[7] = {"UNKNOWN", "AHB", "OCP", "AXI", "PCI", "AXIAHB", "AXIAPB"};
static const char *synthLangName[3] = {"UNKNOWN", "VHDL", "VERILOG"};
u32 mem_counter = 0;
#endif /* TRACE_EWL */

/* Private typedef -----------------------------------------------------------*/


/* Private defines -----------------------------------------------------------*/

#ifndef EWL_TIMEOUT
#define EWL_TIMEOUT 100UL
#endif /* EWL_TIMEOUT */

#ifndef EWL_DEFAULT_POOL_SIZE
#define EWL_DEFAULT_POOL_SIZE 0x190000
#endif /* EWL_DEFAULT_POOL_SIZE */

/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* NEURON6 bench instrumentation. A hang inside the vendor library cannot
 * report itself - the console is blocked and the watchdog is being fed by the
 * poll hook - so the last register access is recorded where it survives the
 * reflash that follows. Off unless BSP_VENC_TRACE_REGS is defined. */
#if defined(BSP_VENC_TRACE_REGS)
#include "stm32n6xx.h"
#define VENC_TRACE_REG(offset, is_write)                                       \
  do {                                                                         \
    volatile uint32_t *bkp_ = &TAMP->BKP0R;                                    \
    bkp_[0] = (uint32_t)(offset) | ((is_write) ? 0x80000000U : 0U);            \
    bkp_[3]++;                                                                 \
  } while (0)
#else
#define VENC_TRACE_REG(offset, is_write) do { } while (0)
#endif

static VENC_EWL_TypeDef ewl_instance;
/* Private function prototypes -----------------------------------------------*/

/* Functions Definition ------------------------------------------------------*/

/**
  * @brief  Read ASIC ID register, static implementation
  * @retval u32 returns the ASIC ID and version (VENC register 0)
  */
u32 EWLReadAsicID(void)
{
  return LL_VENC_ReadRegister(0UL);
}

/**
  * @brief  Reads ASIC capability registers, static implementation
  * @retval EWLHwConfig_t structure containing all hardware ASIC configurations
  */
EWLHwConfig_t EWLReadAsicConfig(void)
{

  /* read first part of the configuration stored in register 63 */
  u32 cfgval = LL_VENC_ReadRegister(63UL);
  EWLHwConfig_t cfg_info;
  cfg_info.maxEncodedWidth = cfgval & ((1U << 12U) - 1U);
  cfg_info.h264Enabled = (cfgval >> 27U) & 1U;
  cfg_info.vp8Enabled = (cfgval >> 26U) & 1U;
  cfg_info.jpegEnabled = (cfgval >> 25U) & 1U;
  cfg_info.vsEnabled = (cfgval >> 24U) & 1U;
  cfg_info.rgbEnabled = (cfgval >> 28U) & 1U;
  cfg_info.searchAreaSmall = (cfgval >> 29U) & 1U;
  cfg_info.scalingEnabled = (cfgval >> 30U) & 1U;

  cfg_info.busType = (cfgval >> 20U) & 15U;
  cfg_info.synthesisLanguage = (cfgval >> 16U) & 15U;
  cfg_info.busWidth = (cfgval >> 12U) & 15U;

  /* read second part of the configuration stored in register 296 */
  cfgval = LL_VENC_ReadRegister(296UL);

  cfg_info.addr64Support = (cfgval >> 31U) & 1U;
  cfg_info.dnfSupport = (cfgval >> 30U) & 1U;
  cfg_info.rfcSupport = (cfgval >> 28U) & 3U;
  cfg_info.enhanceSupport = (cfgval >> 27U) & 1U;
  cfg_info.instantSupport = (cfgval >> 26U) & 1U;
  cfg_info.svctSupport = (cfgval >> 25U) & 1U;
  cfg_info.inAxiIdSupport = (cfgval >> 24U) & 1U;
  cfg_info.inLoopbackSupport = (cfgval >> 23U) & 1U;
  cfg_info.irqEnhanceSupport = (cfgval >> 22U) & 1U;

  PTRACE("EWLReadAsicConfig:\n"
         "    maxEncodedWidth   = %d\n"
         "    h264Enabled       = %s\n"
         "    jpegEnabled       = %s\n"
         "    vp8Enabled        = %s\n"
         "    vsEnabled         = %s\n"
         "    rgbEnabled        = %s\n"
         "    searchAreaSmall   = %s\n"
         "    scalingEnabled    = %s\n"
         "    address64bits     = %s\n"
         "    denoiseEnabled    = %s\n"
         "    rfcEnabled        = %s\n"
         "    instanctEnabled   = %s\n"
         "    busType           = %s\n"
         "    synthesisLanguage = %s\n"
         "    busWidth          = %d\n",
         cfg_info.maxEncodedWidth,
         cfg_info.h264Enabled == 1 ? "YES" : "NO",
         cfg_info.jpegEnabled == 1 ? "YES" : "NO",
         cfg_info.vp8Enabled == 1 ? "YES" : "NO",
         cfg_info.vsEnabled == 1 ? "YES" : "NO",
         cfg_info.rgbEnabled == 1 ? "YES" : "NO",
         cfg_info.searchAreaSmall == 1 ? "YES" : "NO",
         cfg_info.scalingEnabled == 1 ? "YES" : "NO",
         cfg_info.addr64Support == 1 ? "YES" : "NO",
         cfg_info.dnfSupport == 1 ? "YES" : "NO",
         cfg_info.rfcSupport == 0 ? "NO" :
         (cfg_info.rfcSupport == 2 ? "LUMA" : (cfg_info.rfcSupport == 1 ? "CHROMA" : "FULL")),
         cfg_info.instantSupport == 1 ? "YES" : "NO",
         cfg_info.busType < 7 ? busTypeName[cfg_info.busType] : "UNKNOWN",
         cfg_info.synthesisLanguage < 3
         ? synthLangName[cfg_info.synthesisLanguage]
         : "ERROR",
         cfg_info.busWidth * 32);

  return cfg_info;
}

/**
  * @brief Allocate resources and setup the wrapper module
  * @note   implementation depends on the OS compatibility setup in ewl_conf.h
  * @param  param EWLInitParam_t type of encoding to be done
  * @note   0 : H264
  *         1 : JPEG
  * @retval void* pointer to the EWL instance
  * @note Only one instance is supported. No apparent need for multi-instance support.
  */
const void *EWLInit(EWLInitParam_t *param)
{

  PTRACE("EWLInit: Start\n");

  /* Check for NULL pointer */
  assert_param(param != NULL);
  /* only H264 (0) and JPEG (1) are supported */
  assert_param(param->clientType <= 1U);

  u8 *mem_pool = NULL;
  size_t mem_pool_size = EWL_DEFAULT_POOL_SIZE;

#if (EWL_ALLOC_API == EWL_USE_THREADX_MM) || (EWL_ALLOC_API == EWL_USE_STM32MPM_MM)
  /* for ThreadX and STM32MPM, a user defined pool is required */
  EWLPoolChoiceCb(&mem_pool, &mem_pool_size);
  if (mem_pool == NULL)
  {
    PTRACE("error : memory pool pointer NULL");
    return NULL;
  }
#endif /* EWL_ALLOC_API */
  /* set pool parameters in instance */
  ewl_instance.pool = mem_pool;
  ewl_instance.pool_size = mem_pool_size;

#if (EWL_ALLOC_API == EWL_USE_MALLOC_MM)
  /* nothing to do if malloc is used for memory management */
#elif (EWL_ALLOC_API == EWL_USE_FREERTOS_MM)
  /* no initialization required for freertos MM */
#elif (EWL_ALLOC_API == EWL_USE_THREADX_MM)
  uint32_t err;
  /* if ThreadX is chosen as memory management, initialize a byte pool */
  err = tx_byte_pool_create(&ewl_instance.byte_pool, "ewl byte pool", mem_pool, mem_pool_size);
  if (err != TX_SUCCESS)
  {
    return NULL;
  }
#elif (EWL_ALLOC_API == EWL_USE_STM32MPM_MM)
#error "no STM32MPM yet"
#elif (EWL_ALLOC_API == EWL_USER_MM)
#else
#error "incorrect EWL memory management configuration"
#endif /* EWL_ALLOC_API */

#if (EWL_SYNC_API == EWL_USE_POLLING_SYNC)
  /* no need for specific tools for polling synchronization */
#elif (EWL_SYNC_API == EWL_USE_BAREMETAL_SYNC)
  /* NEURON6. No primitive to create; the flag is in the instance. Priority 5
   * rather than the lowest: an event-paced USB interrupt must not delay the
   * encoder's frame-ready handling, because the
   * per-frame re-arm has to land inside the vertical blanking. */
  ewl_instance.hw_ready = 0U;
  HAL_NVIC_SetPriority(VENC_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(VENC_IRQn);
#elif (EWL_SYNC_API == EWL_USE_THREADX_SYNC)
  /* if threadx is chosen as synchronization, initialize an event flags tool */
  err = tx_event_flags_create(&ewl_instance.flag_events, "EWL_event_group");
  if (err != TX_SUCCESS)
  {
    return NULL;
  }
  HAL_NVIC_SetPriority(VENC_IRQn, 0x07, 0);
  HAL_NVIC_EnableIRQ(VENC_IRQn);
#elif (EWL_SYNC_API == EWL_USE_FREERTOS_SYNC)
  /* FREERTOS synchronization requires a semaphore. Initialize it and put ID in instance */
  ewl_instance.ewl_event_group = xEventGroupCreate();
  if(ewl_instance.ewl_event_group == NULL){
    return NULL;
  }
  /* 5 (=USB IRQ prio), not 0x07: keeps the event-paced USB ISR from preempting the VENC
   * FRAME_READY IRQ, so the per-frame re-arm is not pushed past the vertical blanking. 5 is
   * configMAX_SYSCALL_INTERRUPT_PRIORITY, the lowest legal prio for an ISR using FromISR APIs. */
  HAL_NVIC_SetPriority(VENC_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(VENC_IRQn);
#elif (EWL_SYNC_API == EWL_USER_SYNC)
#else
#error "incorrect EWL synchronization configuration"
#endif /* EWL_SYNC_API */

  /* set client type and pool attributes */
  ewl_instance.clientType = param->clientType;

  return (void *)&ewl_instance;
}

/**
  * @brief  Release the wrapper module by freeing all the resources
  * @param  inst     EWL instance to release
  * @retval i32 error code : EWL_OK, EWL_ERROR
  * @note no error possible in this implementation, only returns EWL_OK
  */
i32 EWLRelease(const void *inst)
{
  assert_param(inst != NULL);

  const VENC_EWL_TypeDef *enc = (const VENC_EWL_TypeDef *)inst;
#if (EWL_ALLOC_API == EWL_USE_THREADX_MM)
  i32 err = tx_byte_pool_delete(&ewl_instance.byte_pool);
  if (err)
  {
    PTRACE("error deleting encoder byte pool");
  }
  err = tx_event_flags_delete(&ewl_instance.flag_events);
  if (err)
  {
    PTRACE("error deleting encoder flag events");
  }
#elif (EWL_ALLOC_API == EWL_USE_STM32MPM_MM)
#elif (EWL_ALLOC_API == EWL_USER_MM)
#elif (EWL_ALLOC_API == EWL_USE_FREERTOS_MM)
#else
#endif /* EWL_ALLOC_API */
#if (EWL_SYNC_API == EWL_USE_FREERTOS_SYNC)
  /* NEURON6. The event group belongs to the synchronisation back end, not the
   * allocator, and upstream only deleted it under the FreeRTOS allocator: with
   * malloc as the allocator every release leaked one, and BSP_VENC_Resync
   * releases and re-creates the instance on every desync. */
  if (ewl_instance.ewl_event_group != NULL)
  {
    vEventGroupDelete(ewl_instance.ewl_event_group);
    ewl_instance.ewl_event_group = NULL;
  }
#endif /* EWL_SYNC_API */
  /* save memory pool address */
  u8 *pool = enc->pool;
  /* free the associated pool */
  EWLPoolReleaseCb(&pool);

  PTRACE("EWLRelease: instance freed\n");
  return EWL_OK;
}

/** Note that EWLWriteReg, EWLEnableHW and EWLDisableHW have the same functionality.
  * This behaviour and API is defined in the IP SW integration guide and needs to be
  * implemented this way for compatibility with the higher level encoding software stack.
  */

/**
  * @brief  Set the content of a hadware register
  * @param  inst       EWL instance (unused)
  * @param  offset     offset of the register to change (in bytes)
  * @param  val        value the register shall be set to
  * @retval None
  */
void EWLWriteReg(const void *inst, u32 offset, u32 val)
{
  (void)inst;
  /* offset is defined in the EWL API in bytes. convert to number of registers by shifting */
  VENC_TRACE_REG(offset, 1);
  LL_VENC_WriteRegister((offset >> 2), val);
}

/**
  * @brief  Set the content of a hadware register
  * @param  inst       EWL instance (unused)
  * @param  offset     offset of the register to change (in bytes)
  * @param  val        value the register shall be set to
  * @retval None
  */
void EWLEnableHW(const void *inst, u32 offset, u32 val)
{
  (void)inst;
  /* offset is defined in the EWL API in bytes. convert to number of registers by shifting */
  VENC_TRACE_REG(offset, 1);
  LL_VENC_WriteRegister((offset >> 2), val);
}

/**
  * @brief  Set the content of a hadware register
  * @param  inst       EWL instance (unused)
  * @param  offset     offset of the register to change (in bytes)
  * @param  val        value the register shall be set to
  * @retval None
  */
void EWLDisableHW(const void *inst, u32 offset, u32 val)
{
  (void)inst;
  /* offset is defined in the EWL API in bytes. convert to number of registers by shifting */
  VENC_TRACE_REG(offset, 1);
  LL_VENC_WriteRegister((offset >> 2), val);
}

/**
  * @brief  Retrieve the content of a hadware register
  * @note   The status register will be read after every MB
so it may be needed to buffer it's content if reading
the HW register is slow.
  * @param  inst      EWL instance affected
  * @param  offset    offset of the register to read (in bytes)
  * @retval u32       desired register's value
  */
u32 EWLReadReg(const void *inst, u32 offset)
{
  (void)inst;
  /* offset is defined in the EWL API in bytes. convert to number of registers by shifting */
  VENC_TRACE_REG(offset, 0);
  return LL_VENC_ReadRegister((offset >> 2));
}

/**
  * @brief  Allocate a frame buffer (contiguous linear RAM memory)
  * @param  instance     EWL instance associated with the buffer to allocate
  * @param  size         size in bytes of the requested memory
  * @param  info         structure containing the allocated memory buffer parameters
  * @retval i32          0 for success or a negative error code
  */
__weak i32 EWLMallocRefFrm(const void *instance, u32 size, EWLLinearMem_t *info)
{
  return EWLMallocLinear(instance, size, info);
}

/**
  * @brief   Release a frame buffer previously allocated with EWLMallocRefFrm
  * @param  instance     EWL instance associated with the buffer to free
  * @param  info         structure containing the memory buffer parameters
  * @retval None
  */
__weak void EWLFreeRefFrm(const void *instance, EWLLinearMem_t *info)
{
  EWLFreeLinear(instance, info);
}

/**
  * @brief  Allocate a contiguous, linear RAM  memory buffer
  * @note   implementation depends on the OS compatibility setup in ewl_conf.h
  * @param  instance     EWL instance associated with the buffer to allocate
  * @param  size         size in bytes of the requested memory
  * @param  info         structure containing the allocated memory buffer parameters
  * @retval i32          0 for success or a negative error code
  */
__weak i32 EWLMallocLinear(const void *instance, u32 size, EWLLinearMem_t *info)
{
  if (instance == NULL)
  {
    return EWL_ERROR;
  }
  VENC_EWL_TypeDef *inst = (VENC_EWL_TypeDef *) instance;
  /* make size 8-byte aligned */
  u32 size_aligned = ALIGNED_SIZE(size);
  info->size = size_aligned;

#if (EWL_ALLOC_API == EWL_USE_MALLOC_MM)
  /* allocate using malloc and check return */
  inst->chunks[inst->totalChunks] = (u32 *)malloc(size_aligned);
  if (inst->chunks[inst->totalChunks] == NULL)
  {
    return EWL_ERROR;
  }

  /* align given malloc buffer */
  inst->alignedChunks[inst->totalChunks] = (u32 *) ALIGNED_SIZE((u32)inst->chunks[inst->totalChunks]);
  /* put the aligned pointer in the return structure */
  info->virtualAddress = inst->alignedChunks[inst->totalChunks++];
#elif (EWL_ALLOC_API == EWL_USE_FREERTOS_MM)
  info->virtualAddress = pvPortMalloc(size_aligned);
#elif (EWL_ALLOC_API == EWL_USE_THREADX_MM)
  i32 err = tx_byte_allocate(&inst->byte_pool, (void **) &inst->chunks[inst->totalChunks], size_aligned, TX_NO_WAIT);
  if (err != TX_SUCCESS)
  {
    return EWL_ERROR;
  }
  if (inst->chunks[inst->totalChunks] == NULL)
  {
    return EWL_ERROR;
  }

  /* align given malloc buffer */
  inst->alignedChunks[inst->totalChunks] = (u32 *) ALIGNED_SIZE((u32)inst->chunks[inst->totalChunks]);
  /* put the aligned pointer in the return structure */
  info->virtualAddress = inst->alignedChunks[inst->totalChunks++];
#elif (EWL_ALLOC_API == EWL_USE_STM32MPM_MM)
#error "no STM32MPM yet"
#elif (EWL_ALLOC_API == EWL_USER_MM)
  return EWL_ERROR;
#else
#error "incorrect EWL memory management configuration"
#endif /* EWL_ALLOC_API */

  if (info->virtualAddress == NULL)
  {
    PTRACE("EWLMallocLinear unable to allocate %8d bytes\n", size_aligned);
    return EWL_ERROR;
  }

  /* bus address is the same as virtual address because no MMU */
  info->busAddress = (ptr_t)info->virtualAddress;

  /* check buffer alignment */
  assert_param((info->busAddress & 0b111UL) == 0U);

#ifdef TRACE_EWL
  mem_counter += size;
  PTRACE("EWLMallocLinear allocated %8d bytes --> %p / %d. Total : %d\n", size_aligned, info->virtualAddress,
         info->busAddress, mem_counter);
#endif /* TRACE_EWL */
  return EWL_OK;
}

/**
  * @brief   Release a frame buffer previously allocated with EWLMallocLinear
  * @note   implementation depends on the OS compatibility setup in ewl_conf.h
  * @param  instance     EWL instance associated with the buffer to free
  * @param  info         structure containing the memory buffer parameters
  * @retval None
  */
__weak void EWLFreeLinear(const void *instance, EWLLinearMem_t *info)
{
  VENC_EWL_TypeDef *inst = (VENC_EWL_TypeDef *) instance;
#if (EWL_ALLOC_API == EWL_USE_MALLOC_MM)
  /* find the pointer corresponding to the aligned buffer */
  for (u32 i = 0; i < inst->totalChunks; i++)
  {
    if (inst->alignedChunks[i] == info->virtualAddress)
    {
      free(inst->chunks[i]);
      break;
    }
  }
#elif (EWL_ALLOC_API == EWL_USE_FREERTOS_MM)
  vPortFree(info->virtualAddress);
#elif (EWL_ALLOC_API == EWL_USE_THREADX_MM)
  for (u32 i = 0; i < inst->totalChunks; i++)
  {
    if (inst->alignedChunks[i] == info->virtualAddress)
    {
      tx_byte_release(inst->chunks[i]);
      break;
    }
  }

#elif (EWL_ALLOC_API == EWL_USE_STM32MPM_MM)
#error "no STM32MPM yet"
#elif (EWL_ALLOC_API == EWL_USER_MM)
  return;
#else
#error "incorrect EWL memory management configuration"
#endif /* EWL_ALLOC_API */
  info->virtualAddress = NULL;
  info->busAddress = 0;
  info->size = 0;
}

/**
  * @brief  Reserve HW resource for currently running codec
  * @note  Currently unused due to only one instance being supported
  * @param  inst EWL instance (unused)
  * @retval i32  0 for success or a negative error code
  */
i32 EWLReserveHw(const void *inst)
{
  assert_param(inst != NULL);
  return EWL_OK;
}

/**
  * @brief  Release HW resource for currently running codec
  * @note  Currently unused due to only one instance being supported
  * @param  inst  EWL instance (unused)
  * @retval i32   0 for success or a negative error code
  */
void EWLReleaseHw(const void *inst)
{
  assert_param(inst != NULL);
}

/**
  * @brief            Allocate a memory block. Same functionality as the ANSI C malloc()
  * @note             Implementation depends on the OS compatibility setup in ewl_conf.h
  * @param  n         Number of bytes to allocate
  * @retval  void     pointer to the allocated space, or NULL if there is insufficient memory available
  */
__weak void *EWLmalloc(u32 n)
{
  void * p = NULL;
#if (EWL_ALLOC_API == EWL_USE_MALLOC_MM)
  p = malloc((size_t)n);
#elif (EWL_ALLOC_API == EWL_USE_FREERTOS_MM)
  p = pvPortMalloc(n);
#elif (EWL_ALLOC_API == EWL_USE_THREADX_MM)
  p = NULL;
  i32 err = tx_byte_allocate(&ewl_instance.byte_pool, &p, n, TX_NO_WAIT);
  if (err != TX_SUCCESS)
  {
    return NULL;
  }
#elif (EWL_ALLOC_API == EWL_USE_STM32MPM_MM)
#error "no STM32MPM yet"
#elif (EWL_ALLOC_API == EWL_USER_MM)

#else
#error "incorrect EWL memory management configuration"
#endif /* EWL_ALLOC_API */
#ifdef TRACE_EWL
  mem_counter += n;
#endif /* TRACE_EWL */
  PTRACE("EWLmalloc %8d bytes --> %p, total : %d\n", n, p, mem_counter);
  return p;
}

/**
  * @brief    Deallocates or frees a memory block. Same functionality as the ANSI C free()
  * @note     Implementation depends on the OS compatibility setup in ewl_conf.h
  * @param p  Previously allocated memory block to be freed
  * @retval   None
  */
__weak void EWLfree(void *p)
{
  PTRACE("EWLfree %p\n", p);

#if (EWL_ALLOC_API == EWL_USE_MALLOC_MM)
  free(p);
#elif (EWL_ALLOC_API == EWL_USE_FREERTOS_MM)
  vPortFree(p);
#elif (EWL_ALLOC_API == EWL_USE_THREADX_MM)
  tx_byte_release(p);
#elif (EWL_ALLOC_API == EWL_USE_STM32MPM_MM)
#error "no STM32MPM yet"
#elif (EWL_ALLOC_API == EWL_USER_MM)
  return;
#else
#error "incorrect EWL memory management configuration"
#endif /* EWL_ALLOC_API */
}

/**
  * @brief            Allocate an array in memory with elements initialized
  *       to 0. Same functionality as the ANSI C calloc()
  * @param  n         Number of elements
  * @param  s         Size of each element (in bytes)
  * @retval  void     pointer to the allocated space, or NULL if there is insufficient memory available
  */
__weak void *EWLcalloc(u32 n, u32 s)
{
  void *p = EWLmalloc(n * s);
  EWLmemset(p, 0, n * s);
  return p;
}

/**
  * @brief  Copies characters between buffers. Same functionality as the ANSI C memcpy()
  * @param  d          Destination buffer
  * @param  s          Buffer to copy from
  * @param  n          Number of bytes to copy
  * @retval void* The value of destination d
  */
 void *EWLmemcpy(void *d, const void *s, u32 n)
{
  return memcpy(d, s, (size_t)n);
}

/**
  * @brief  Sets buffers to a specified character. Same functionality as the ANSI C memset()
  * @param  d     Pointer to destination
  * @param  c     Character to set
  * @param  n     Number of characters
  * @retval void* The value of destination d
  */
void *EWLmemset(void *d, i32 c, u32 n)
{
  return memset(d, c, (size_t)n);
}

/**
  * @brief  Compares two buffers. Same functionality is the ANSI C memcmp()
  * @param  s1     Buffer to compare
  * @param  s2     Buffer to compare
  * @param  n      Number of bytes to compare
  * @retval int Zero if the first n bytes of s1 match s2
  */
int EWLmemcmp(const void *s1, const void *s2, u32 n)
{
  return memcmp((const uint8_t *) s1, (const uint8_t *) s2, (size_t)n);
}

/* ---- Phase D: VENC input line-buffer (low-latency) SRAM base --------------
 * The Hantro core calls EWLGetInputLineBufferBase from HEncInitInputLineBuffer
 * (ONLY when JpegEncCfg.inputLineBufEn=1) to learn where the on-chip line-buffer
 * "band" lives: the small buffer the encoder reads instead of a full frame.
 * In our zero-copy design that band is an AXISRAM buffer the DCMIPP writes and
 * the VENC reads. The caller registers it via EWLSetInputLineBufferBase before
 * a line-buffer encode. Undefined in the stock ST EWL port; defined here.
 * Inert for the existing full-frame path (inputLineBufEn=0 → never called). */
static EWLLinearMem_t s_venc_input_linebuf = {0};

void EWLSetInputLineBufferBase(u32 *virt, ptr_t bus, u32 size)
{
  s_venc_input_linebuf.virtualAddress = virt;
  s_venc_input_linebuf.busAddress     = bus;
  s_venc_input_linebuf.size           = size;
}

i32 EWLGetInputLineBufferBase(const void *instance, EWLLinearMem_t *info)
{
  (void)instance;
  if ((info == NULL) || (s_venc_input_linebuf.virtualAddress == NULL))
    return EWL_ERROR;
  *info = s_venc_input_linebuf;
  return EWL_OK;
}

/**
  * @brief  Wait for the encoder hardware
  * @note   Implementation depends on the OS compatibility setup in ewl_conf.h
  * @param  inst     EWL instance (used to get the instance sync tools)
  * @param  slicesReady Number of encoded slices. (ignored if NULL)
  * @retval i32 EWL_OK on success, EWL_HW_WAIT_TIMEOUT on timeout error, EWL_ERROR on other error
  */
__weak i32 EWLWaitHwRdy(const void *inst, u32 *slicesReady)
{
  /* Check invalid parameters */
  if (inst == NULL)
  {
    return EWL_HW_WAIT_ERROR;
  }

  /* NEURON6 hook 1 of 3. DCMIPP->VENC ordering (RM0486 45.4.7): the encoder has
   * just been armed by EncAsicFrameStart and is parked waiting for venc_rdy. The
   * camera source must be started NOW, not before the encode, so that the encoder
   * is already waiting when the first line event arrives; started earlier, those
   * early edges are missed, the read pointer never leaves zero and the two
   * deadlock. There is no way to express that ordering from outside this
   * function, because this is the only point at which the encoder is known to be
   * armed. neuron6_venc.c makes this a one-shot per stream. */
  { extern void BSP_VENC_SourceArmHook(void); BSP_VENC_SourceArmHook(); }

  /* NEURON6 hook 2 of 3. The encoder is armed and the long source-paced wait
   * begins here. Nothing to do without a scheduler; an RTOS integrator whose
   * encode runs at a raised priority drops it here so another task can drain the
   * previous frame while this one waits. Weak, default empty. */
  { extern void BSP_VENC_WaitBeginHook(void); BSP_VENC_WaitBeginHook(); }

  /* Copied and adapted from polling reference implementation*/

  u32 ret = EWL_HW_WAIT_TIMEOUT;
#if (EWL_SYNC_API == EWL_USE_POLLING_SYNC)
  PTRACE("EWLWaitHw: Start\n");
  volatile u32 irq_stats;
  u32 tickstart = HAL_GetTick();
  u32 prevSlicesReady = 0;
  /* check how to clear IRQ flags for VENC */
  u32 clrByWrite1 = (EWLReadReg(inst, BASE_HWFuse2) & HWCFGIrqClearSupport);
  do
  {
    irq_stats = VENC_REG(1UL);
    /* Get the number of completed slices from ASIC registers. */
    if (slicesReady != NULL)
    {
      *slicesReady = (VENC_REG(21UL) >> 16) & 0xFFUL;
    }
    PTRACE("EWLWaitHw: IRQ stat = %08x\n", irq_stats);
    u32 hw_handshake_status = READ_BIT(VENC_REG(BASE_HEncInstantInput >> 2U), (1U << 29U));
    /* NEURON6 hook 3 of 3. Back-pressure probe. The encoder's read pointer sits in
     * 0x7C4[17:9], in macroblock rows, and it is the one number that says whether
     * the consumer is keeping up with the band. Reported here because this poll
     * loop is the only place inside a frame where it can be sampled. Weak,
     * default empty; neuron6_venc.c tracks the maximum for
     * BSP_VENC_GetStatistics. */
    { extern void BSP_VENC_ReadPointerHook(uint32_t MbRow);
      BSP_VENC_ReadPointerHook((VENC_REG(BASE_HEncInstantInput >> 2U) >> 9) & 0x1FFU); }
    /* ignore the irq status of input line buffer in hw handshake mode */
    if ((irq_stats == ASIC_STATUS_LINE_BUFFER_DONE) && (hw_handshake_status != 0UL))
    {
      VENC_REG(1U) = (1<<9);
      continue;
    }
    if ((irq_stats & ASIC_STATUS_ALL) != 0UL)
    {
      /* clear IRQ and slice ready status */
      u32 clr_stats;
      irq_stats &= (~(ASIC_STATUS_SLICE_READY | ASIC_IRQ_LINE));

      if (clrByWrite1 != 0UL)
      {
        clr_stats = ASIC_STATUS_SLICE_READY | ASIC_IRQ_LINE;
      }
      else
      {
        clr_stats = irq_stats;
      }
      EWLWriteReg(inst, 0x04, clr_stats);
      ret = EWL_OK;
      break;
    }

    if (slicesReady != NULL)
    {
      if (*slicesReady > prevSlicesReady)
      {
        ret = EWL_OK;
        break;
      }
    }
  } while ((HAL_GetTick() - tickstart) < EWL_TIMEOUT);
  if (slicesReady != NULL)
  {
    PTRACE("EWLWaitHw: slicesReady = %d\n", *slicesReady);
  }
#elif (EWL_SYNC_API == EWL_USE_BAREMETAL_SYNC)
  /* NEURON6. The RTOS back ends block here; with no scheduler this waits on
   * the flag VENC_IRQHandler sets, which is the same event at the same moment
   * and leaves the per-macroblock-row status to be cleared in the handler
   * rather than whenever a poll next arrives.
   *
   * The read-pointer hook is called while waiting, on the same terms as the
   * poll loop above: it is how an application keeps USB serviced and the
   * watchdog fed through an encode, and this back end must not take that
   * away. */
  {
    u32 tickstart = HAL_GetTick();

    ewl_instance.hw_ready = 0U;

    while (ewl_instance.hw_ready == 0U)
    {
      { extern void BSP_VENC_ReadPointerHook(uint32_t MbRow);
        BSP_VENC_ReadPointerHook((VENC_REG(BASE_HEncInstantInput >> 2U) >> 9) & 0x1FFU); }

      if ((HAL_GetTick() - tickstart) >= EWL_TIMEOUT)
      {
        return EWL_HW_WAIT_TIMEOUT;
      }
    }

    ewl_instance.hw_ready = 0U;
    ret = EWL_OK;

    if (slicesReady != NULL)
    {
      *slicesReady = (VENC_REG(21UL) >> 16) & 0xFFUL;
      PTRACE("EWLWaitHw: slicesReady = %d\n", *slicesReady);
    }
  }
#elif (EWL_SYNC_API == EWL_USE_THREADX_SYNC)
  VENC_EWL_TypeDef *enc = (VENC_EWL_TypeDef *) inst;
  /* wait for event flags set in IRQ */
  u32 events = 0;
  u32 status = tx_event_flags_get(&enc->flag_events, 1, TX_OR_CLEAR, (ULONG *) &events, EWL_TIMEOUT);
  if (status != TX_SUCCESS)
  {
    PTRACE("sync error\n");
    return EWL_HW_WAIT_TIMEOUT;
  }
  ret = EWL_OK;
  /* Get the number of completed slices from ASIC registers. */
  if (slicesReady != NULL)
  {
    *slicesReady = (VENC_REG(21UL) >> 16) & 0xFFUL;
    PTRACE("EWLWaitHw: slicesReady = %d\n", *slicesReady);
  }
#elif (EWL_SYNC_API == EWL_USE_FREERTOS_SYNC)
  if (xEventGroupWaitBits(ewl_instance.ewl_event_group, 1, pdTRUE, pdTRUE, EWL_TIMEOUT) == 0)
  {
    return EWL_HW_WAIT_TIMEOUT;
  }
  ret = EWL_OK;
  /* Get the number of completed slices from ASIC registers. */
  if (slicesReady != NULL)
  {
    *slicesReady = (VENC_REG(21UL) >> 16) & 0xFFUL;
    PTRACE("EWLWaitHw: slicesReady = %d\n", *slicesReady);
  }
#elif (EWL_SYNC_API == EWL_USER_SYNC)
  return EWL_ERROR;
#else
#endif /* EWL_SYNC_API */
  return ret;
}


/* Used for memory pool selection */
/* If pool_ptr is not NULL and size is bigger than 0, pool pointer is used to initialize the EWL instance memory pool */
/* if pool_ptr is NULL or size is 0 after this function call, the pool is allocation on the heap using malloc */
/* the default pool size is set in ewl_impl.C but can be overwritten by the user. */
__weak void EWLPoolChoiceCb(u8 **pool_ptr, size_t *size)
{
  *size = EWL_DEFAULT_POOL_SIZE;
  *pool_ptr = (u8 *) malloc(EWL_DEFAULT_POOL_SIZE);
}

__weak void EWLPoolReleaseCb(u8 **pool_ptr)
{
  free(*pool_ptr);
}


void VENC_IRQHandler(void)
{
  u32 hw_handshake_status = READ_BIT(VENC_REG(BASE_HEncInstantInput >> 2U), (1U << 29U));
  uint32_t irq_status = VENC_REG(1U);
  if(!hw_handshake_status && (irq_status & ASIC_STATUS_FUSE))
  {
    VENC_REG(1U) = ASIC_STATUS_FUSE | ASIC_IRQ_LINE;
    /* read back the IRQ status to update its value */
    irq_status = VENC_REG(1U);
  }
  /* NEURON6. Line-buffer (instant) mode: the per-macroblock-row LINE_BUFFER_DONE
   * event fires continuously as the source feeds the band - that is the venc_rdy
   * handshake working. Clear it WITHOUT signalling, so a waiter wakes only on the
   * real terminal status (FRAME_READY) and not on every row, which would end the
   * encode after one row and break the line buffer. Mirrors the poll loop's own
   * LINE_BUFFER_DONE handling above. Reached only under an interrupt-driven sync
   * back end; the polling back end this BSP selects never enables VENC_IRQn. */
  if ((hw_handshake_status != 0UL) && (irq_status == ASIC_STATUS_LINE_BUFFER_DONE))
  {
    /* Per-MB-row event: clear WITHOUT signaling so the wait wakes only on FRAME_READY.
     * (Defensive: the H1 raises the IRQ only on FRAME_READY here; the back-pressure gap probe
     * moved to the DCMIPP line IRQ, which fires reliably; see HAL_DCMIPP_PIPE_LineEventCallback.) */
    VENC_REG(1U) = (1U << 9);
    return;
  }
  /* See if there are other flags than the FUSE status raised */
  if(irq_status != 0U)
  {
    /* status flag is raised, clear the ones that the IRQ needs to clear and signal to EWLWaitHwReady */
    VENC_REG(1U) = ASIC_STATUS_SLICE_READY | ASIC_IRQ_LINE;
#if (EWL_SYNC_API == EWL_USE_BAREMETAL_SYNC)
    /* NEURON6. What the event group set is on the RTOS back ends. */
    ewl_instance.hw_ready = 1U;
#elif (EWL_SYNC_API == EWL_USE_THREADX_SYNC)
    /* set event flags for synchronization */
    u32 status = tx_event_flags_set(&ewl_instance.flag_events, 1, TX_OR);
    if (status != TX_SUCCESS)
    {

      return ;
    }
#elif (EWL_SYNC_API == EWL_USE_FREERTOS_SYNC)
    BaseType_t higher_prio_task_woken = pdFALSE;
    /* NEURON6. Between EWLRelease and the next EWLInit there is no event group,
     * and an interrupt there has nothing to wake. */
    if (ewl_instance.ewl_event_group == NULL)
    {
      return;
    }
    if(xEventGroupSetBitsFromISR(ewl_instance.ewl_event_group, 1, &higher_prio_task_woken) == pdFAIL){
      PTRACE("set flags error\n");
    }
    portYIELD_FROM_ISR(higher_prio_task_woken);
#endif /* EWL_SYNC_API */
  }
}
