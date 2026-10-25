/**
  * @file    neuron6_npu.c
  * @brief   Execution environment for the Neural-ART NPU on the Neuro N6.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6_conf.h"
#include "neuron6_npu.h"
#include "neuron6.h"

/* CACHEAXI enable/disable retries: the HAL can answer HAL_BUSY while an
 * invalidation is still in flight. */
#define NPU_CACHE_RETRY_COUNT       16U

/* Static (zeroed) so HAL_CACHEAXI_Init() sees STATE_RESET and runs its MSP init. */
static CACHEAXI_HandleTypeDef NpuCacheHandle;

static uint8_t NpuIsInitialised;

/* Grant the NPU its RIF rights: the N6 silently drops transactions from an
 * unconfigured bus master. */
static void NPU_RifConfig(void)
{
  RIMC_MasterConfig_t RIMC_master = {0};

  /* Board-wide isolation first. */
  (void)BSP_ISOLATION_Config();

#if defined(CPU_IN_SECURE_STATE)
  __HAL_RCC_RIFSC_CLK_ENABLE();
#endif /* CPU_IN_SECURE_STATE */

  RIMC_master.MasterCID = RIF_CID_1;
  RIMC_master.SecPriv = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_NPU, &RIMC_master);

  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_NPU,
                                        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
}

/* Clock the NPU and power up AXISRAM3-6 (the RAM window at NEURON6_NPU_RAM_BASE). */
static void NPU_RamEnable(void)
{
  RAMCFG_HandleTypeDef hramcfg = {0};

  __HAL_RCC_NPU_CLK_ENABLE();
  __HAL_RCC_NPU_FORCE_RESET();
  __HAL_RCC_NPU_RELEASE_RESET();

  __HAL_RCC_AXISRAM3_MEM_CLK_ENABLE();
  __HAL_RCC_AXISRAM4_MEM_CLK_ENABLE();
  __HAL_RCC_AXISRAM5_MEM_CLK_ENABLE();
  __HAL_RCC_AXISRAM6_MEM_CLK_ENABLE();
  __HAL_RCC_RAMCFG_CLK_ENABLE();

  hramcfg.Instance = RAMCFG_SRAM3_AXI;
  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
  hramcfg.Instance = RAMCFG_SRAM4_AXI;
  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
  hramcfg.Instance = RAMCFG_SRAM5_AXI;
  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
  hramcfg.Instance = RAMCFG_SRAM6_AXI;
  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
}

/* The RAM banks stay on: other buffers may live in them, and nothing would
 * power them up again. */
static void NPU_ClockDisable(void)
{
  __HAL_RCC_NPU_CLK_DISABLE();
}

/* CACHEAXI is separate from the M55 D-cache; SCB_EnableDCache() does not touch it. */
static int32_t NPU_CacheEnable(void)
{
  HAL_StatusTypeDef status;
  uint32_t retry;

  __HAL_RCC_CACHEAXIRAM_MEM_CLK_ENABLE();
  __HAL_RCC_CACHEAXI_CLK_ENABLE();
  __HAL_RCC_CACHEAXI_FORCE_RESET();
  __HAL_RCC_CACHEAXI_RELEASE_RESET();

  NpuCacheHandle.Instance = CACHEAXI;

  retry = NPU_CACHE_RETRY_COUNT;
  do
  {
    status = HAL_CACHEAXI_Init(&NpuCacheHandle);
    retry--;
  } while ((status == HAL_BUSY) && (retry != 0U));

  if (status == HAL_BUSY)
  {
    return BSP_ERROR_BUSY;
  }
  if (status != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  retry = NPU_CACHE_RETRY_COUNT;
  do
  {
    status = HAL_CACHEAXI_Enable(&NpuCacheHandle);
    retry--;
  } while ((status == HAL_BUSY) && (retry != 0U));

  if (status == HAL_BUSY)
  {
    return BSP_ERROR_BUSY;
  }
  if (status != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return BSP_ERROR_NONE;
}

static int32_t NPU_CacheDisable(void)
{
  HAL_StatusTypeDef status;
  uint32_t retry;

  retry = NPU_CACHE_RETRY_COUNT;
  do
  {
    status = HAL_CACHEAXI_Disable(&NpuCacheHandle);
    retry--;
  } while ((status == HAL_BUSY) && (retry != 0U));

  if (status == HAL_BUSY)
  {
    return BSP_ERROR_BUSY;
  }
  if (status != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  if (HAL_CACHEAXI_DeInit(&NpuCacheHandle) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  __HAL_RCC_CACHEAXI_CLK_DISABLE();
  __HAL_RCC_CACHEAXIRAM_MEM_CLK_DISABLE();

  return BSP_ERROR_NONE;
}

/* NPU0-3: the runtime attaches its epoch-completion and error handlers. */
static void NPU_NvicConfig(void)
{
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  HAL_NVIC_SetPriority(NPU0_IRQn, BSP_NPU_IT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(NPU0_IRQn);

  HAL_NVIC_SetPriority(NPU1_IRQn, BSP_NPU_IT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(NPU1_IRQn);

  HAL_NVIC_SetPriority(NPU2_IRQn, BSP_NPU_IT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(NPU2_IRQn);

  HAL_NVIC_SetPriority(NPU3_IRQn, BSP_NPU_IT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(NPU3_IRQn);
}

/** @brief Bring up the RIF grants, NPU clock and RAM banks, cache and interrupts. */
int32_t BSP_NPU_Init(void)
{
  int32_t ret;

  if (NpuIsInitialised != 0U)
  {
    return BSP_ERROR_NONE;
  }

  NPU_RifConfig();
  NPU_RamEnable();

  ret = NPU_CacheEnable();
  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }

  NPU_NvicConfig();

  NpuIsInitialised = 1U;

  return BSP_ERROR_NONE;
}

/** @brief Undo BSP_NPU_Init(); the RAM banks stay on. */
int32_t BSP_NPU_DeInit(void)
{
  int32_t ret;

  if (NpuIsInitialised == 0U)
  {
    return BSP_ERROR_NONE;
  }

  HAL_NVIC_DisableIRQ(NPU0_IRQn);
  HAL_NVIC_DisableIRQ(NPU1_IRQn);
  HAL_NVIC_DisableIRQ(NPU2_IRQn);
  HAL_NVIC_DisableIRQ(NPU3_IRQn);

  ret = NPU_CacheDisable();
  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }

  NPU_ClockDisable();

  NpuIsInitialised = 0U;

  return BSP_ERROR_NONE;
}

/** @brief Report the NPU RAM window: base address and size in KB. */
int32_t BSP_NPU_GetRamInfo(uint32_t *pBase, uint32_t *pSizeKB)
{
  if ((pBase == NULL) || (pSizeKB == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  *pBase = NEURON6_NPU_RAM_BASE;
  *pSizeKB = NEURON6_NPU_RAM_SIZE_KB;

  return BSP_ERROR_NONE;
}
