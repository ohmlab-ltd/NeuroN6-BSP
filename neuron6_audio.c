/**
  * @file    neuron6_audio.c
  * @brief   Neuro N6 audio: SAI1 Block B playback to a MAX98357A, PDM microphone on MDF1.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6_conf.h"
#include "neuron6_audio.h"

#if (BSP_AUDIO_OUT_DMA_CHANNEL) == (BSP_AUDIO_IN_DMA_CHANNEL)
#error "BSP_AUDIO_OUT_DMA_CHANNEL and BSP_AUDIO_IN_DMA_CHANNEL must differ"
#endif

#if (BSP_AUDIO_IN_STAGING_SAMPLES) < 16U
#error "BSP_AUDIO_IN_STAGING_SAMPLES must be at least 16"
#endif

/* The staging buffer is one GPDMA block: at most 65,535 bytes. */
#if (BSP_AUDIO_IN_STAGING_SAMPLES) > 16383U
#error "BSP_AUDIO_IN_STAGING_SAMPLES must be at most 16383"
#endif

#define AUDIO_DMA_CHANNEL_(n)           GPDMA1_Channel##n
#define AUDIO_DMA_CHANNEL(n)            AUDIO_DMA_CHANNEL_(n)

#define AUDIO_OUT_DMA_INSTANCE          AUDIO_DMA_CHANNEL(BSP_AUDIO_OUT_DMA_CHANNEL)
#define AUDIO_IN_DMA_INSTANCE           AUDIO_DMA_CHANNEL(BSP_AUDIO_IN_DMA_CHANNEL)

/* Board wiring */

#define AUDIO_OUT_SAI_BLOCK             SAI1_Block_B
#define AUDIO_OUT_DMA_REQUEST           GPDMA1_REQUEST_SAI1_B

#define AUDIO_OUT_SCK_FS_PORT           GPIOG
#define AUDIO_OUT_SCK_PIN               GPIO_PIN_1
#define AUDIO_OUT_FS_PIN                GPIO_PIN_2
#define AUDIO_OUT_SD_PORT               GPIOE
#define AUDIO_OUT_SD_PIN                GPIO_PIN_3
#define AUDIO_OUT_SAI_AF                GPIO_AF6_SAI1

/* PB6 = amplifier SD_MODE: low is shutdown, high is on in left-channel mode. */
#define AUDIO_OUT_AMP_PORT              GPIOB
#define AUDIO_OUT_AMP_PIN               GPIO_PIN_6

#define AUDIO_IN_MDF_FILTER             MDF1_Filter1
#define AUDIO_IN_DMA_REQUEST            GPDMA1_REQUEST_MDF1_FLT1

/* PDM microphone: PB2 is MDF1_SDI1 (data), PE2 is MDF1_CCK0 (clock out). */
#define AUDIO_IN_SDI_PORT               GPIOB
#define AUDIO_IN_SDI_PIN                GPIO_PIN_2
#define AUDIO_IN_CCK_PORT               GPIOE
#define AUDIO_IN_CCK_PIN                GPIO_PIN_2
#define AUDIO_IN_MDF_AF                 GPIO_AF4_MDF1

/* Clock chain */

#define AUDIO_OUT_IC_INDEX              RCC_IC7
#define AUDIO_IN_IC_INDEX               RCC_IC8

#define AUDIO_IC_DIV_MAX                256U

/* PLL4 400 MHz / 16 = 25 MHz SAI kernel clock; with MCKDIV 16 and a 32-bit frame,
 * 48,828 Hz. Also the divider used to measure PLL4. */
#define AUDIO_OUT_IC_DIV_NATIVE         16U
#define AUDIO_OUT_MCKDIV_NATIVE         16U

#define AUDIO_OUT_MCKDIV_MAX            63U

/* MAX98357A range; a request outside it is refused before any clock is touched. */
#define AUDIO_OUT_RATE_MIN_HZ           8000U
#define AUDIO_OUT_RATE_MAX_HZ           96000U

/* HAL_SAI_InitProtocol makes two 16-bit I2S slots a 32-bit frame. */
#define AUDIO_OUT_FRAME_BITS            32U

/* IC8 divider for the native capture rate; also used to measure PLL4. */
#define AUDIO_IN_IC_DIV_NATIVE          4U

/* 100 MHz kernel clock / 2 / 16 = 3.125 MHz bitstream clock; / 96 = 32,552 Hz. */
#define AUDIO_IN_CCKDIV_NATIVE          16U

#define AUDIO_IN_CCKDIV_MAX             16U

/* The bitstream clock is divided from the processing clock, so
 * rate = kernel / PROC_DIV / CCKDIV / decimation. */
#define AUDIO_IN_PROC_DIV               2U

/* The bitstream is sampled with the processing clock: at CCKDIV 1 the two are equal,
 * clock absence is flagged and nothing is captured. 3 is the smallest value validated. */
#define AUDIO_IN_CCKDIV_MIN             3U

/* Idle-wait bound in ms; it only stops a dead peripheral hanging the caller. */
#define AUDIO_IN_IDLE_TIMEOUT_MS        10U

/* Reshape filter decimation, after the CIC. */
#define AUDIO_IN_RSF_DECIMATION         4U

/* MP34DT05-A clock range in normal mode (datasheet table 5). */
#define AUDIO_IN_MIC_CLK_MIN_HZ         1200000U
#define AUDIO_IN_MIC_CLK_MAX_HZ         3250000U

/* Clock-absence detection threshold. */
#define AUDIO_IN_SITF_THRESHOLD         31U

/* Native rate: s_in_decimations[1], 96 in all. */
#define AUDIO_IN_DECIMATION_NATIVE      1U

#define AUDIO_PCM16_MAX                 32767
#define AUDIO_PCM16_MIN                 (-32768)

/* The achieved rate must be within this many percent of the request. */
#define AUDIO_RATE_TOLERANCE_PCT        1U

/* D-cache line size; invalidation is per line, so a shared line loses its other data. */
#define AUDIO_CACHE_LINE_BYTES          32U

/* One GPDMA block (16-bit byte count): 65,535 rounded down to a multiple of 8. */
#define AUDIO_OUT_MAX_BYTES             65528U

/* State */

typedef struct
{
  uint32_t SampleRate;    /*!< Achieved rate in Hz, computed from readback */
  uint32_t ChannelsNbr;   /*!< Channels per frame in the caller's buffer */
  uint32_t IcDiv;         /*!< IC7 divider currently programmed */
  uint32_t MckDiv;        /*!< SAI master divider currently programmed */
  uint32_t IsMuted;       /*!< Logical mute request, not the pin state */
  uint32_t State;         /*!< One of BSP_AUDIO_OUT_STATE_* */
} audio_out_ctx_t;

typedef struct
{
  uint32_t SampleRate;    /*!< Achieved rate in Hz, computed from readback */
  uint32_t IcDiv;         /*!< IC8 divider currently programmed */
  uint32_t CckDiv;        /*!< MDF output clock divider currently programmed */
  uint32_t Decimation;    /*!< Index into s_in_decimations */
  int16_t *pBuff;         /*!< Caller's PCM buffer, or NULL when idle */
  uint32_t Samples;       /*!< Length of that buffer in 16-bit samples */
  uint32_t State;         /*!< One of BSP_AUDIO_IN_STATE_* */
} audio_in_ctx_t;

typedef struct
{
  uint32_t Cic;           /*!< CIC decimation; the reshape filter adds 4 */
  int32_t  Divisor;       /*!< Raw MDF word to 16-bit PCM at this decimation */
} audio_in_decimation_t;

/* Divisor 2048 at CIC 32: the raw MDF word is the sample x256, with 8x headroom for this
 * microphone. SINC4 gain is Cic^4, so each divisor is 2048 * (Cic / 32)^4: same level.
 * Overall decimations 128, 96 and 64 reach 9.4-25.4, 12.5-33.9 and 18.8-50.8 kHz. */
static const audio_in_decimation_t s_in_decimations[] =
{
  { 32U, 2048 },
  { 24U,  648 },
  { 16U,  128 },
};

#define AUDIO_IN_DECIMATIONS            (sizeof(s_in_decimations) / sizeof(s_in_decimations[0]))

static audio_out_ctx_t s_out_ctx;
static audio_in_ctx_t  s_in_ctx;

static SAI_HandleTypeDef s_out_sai;
static DMA_HandleTypeDef s_out_dma;
static DMA_QListTypeDef  s_out_queue;

static MDF_HandleTypeDef       s_in_mdf;
static MDF_FilterConfigTypeDef s_in_filter;
static DMA_HandleTypeDef       s_in_dma;
static DMA_QListTypeDef        s_in_queue;

/* Cache-line aligned; at 36 bytes a node spills into a second line, which is safe
 * because nodes are only ever cleaned, never invalidated. */
static DMA_NodeTypeDef s_out_node __attribute__((aligned(AUDIO_CACHE_LINE_BYTES)));

static DMA_NodeTypeDef s_in_node __attribute__((aligned(AUDIO_CACHE_LINE_BYTES)));

/* DMA target for the MDF's 32-bit words; the caller's buffer gets 16-bit PCM. */
static int32_t s_in_staging[BSP_AUDIO_IN_STAGING_SAMPLES] __attribute__((aligned(AUDIO_CACHE_LINE_BYTES)));

/* Cache maintenance */

static void audio_cache_clean(const void *pAddr, uint32_t Bytes)
{
#if defined(USE_BSP_CPU_CACHE_MAINTENANCE) && (USE_BSP_CPU_CACHE_MAINTENANCE == 1U)
  SCB_CleanDCache_by_Addr((void *)pAddr, (int32_t)Bytes);
#else
  (void)pAddr;
  (void)Bytes;
#endif
}

/* pAddr and Bytes must be whole cache lines. */
static void audio_cache_invalidate(const void *pAddr, uint32_t Bytes)
{
#if defined(USE_BSP_CPU_CACHE_MAINTENANCE) && (USE_BSP_CPU_CACHE_MAINTENANCE == 1U)
  SCB_InvalidateDCache_by_Addr((void *)pAddr, (int32_t)Bytes);
#else
  (void)pAddr;
  (void)Bytes;
#endif
}

/* Rate arithmetic */

static uint32_t audio_abs_diff(uint32_t A, uint32_t B)
{
  return (A > B) ? (A - B) : (B - A);
}

static uint32_t audio_rate_acceptable(uint32_t Achieved, uint32_t Requested)
{
  uint64_t slack = (uint64_t)audio_abs_diff(Achieved, Requested) * 100U;

  return (slack <= ((uint64_t)Requested * AUDIO_RATE_TOLERANCE_PCT)) ? 1U : 0U;
}

/* Playback: clocking */

/* Program IC7 (from PLL4) for SAI1 and read back the kernel clock. */
static int32_t audio_out_clock_config(uint32_t IcDiv, uint32_t *pKerHz)
{
  RCC_PeriphCLKInitTypeDef periph = {0};

  periph.PeriphClockSelection                       = RCC_PERIPHCLK_SAI1;
  periph.Sai1ClockSelection                         = RCC_SAI1CLKSOURCE_IC7;
  periph.ICSelection[AUDIO_OUT_IC_INDEX].ClockSelection = RCC_ICCLKSOURCE_PLL4;
  periph.ICSelection[AUDIO_OUT_IC_INDEX].ClockDivider   = IcDiv;

  if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK)
  {
    return BSP_ERROR_CLOCK_FAILURE;
  }

  *pKerHz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SAI1);

  return (*pKerHz == 0U) ? BSP_ERROR_CLOCK_FAILURE : BSP_ERROR_NONE;
}

/* Searches both dividers between PLL4 and the bit clock: that is what reaches standard
 * rates from a PLL with no factor of three. */
static int32_t audio_out_setup_clock(uint32_t SampleRate, uint32_t *pIcDiv,
                                     uint32_t *pMckDiv, uint32_t *pAchieved)
{
  uint32_t kerHz;
  uint64_t srcHz;
  uint32_t bestIc   = 0U;
  uint32_t bestMck  = 0U;
  uint32_t bestRate = 0U;
  uint32_t bestErr  = 0xFFFFFFFFU;
  uint32_t mckDiv;
  int32_t  ret;

  if ((SampleRate != BSP_AUDIO_FREQUENCY_NATIVE) &&
      ((SampleRate < AUDIO_OUT_RATE_MIN_HZ) || (SampleRate > AUDIO_OUT_RATE_MAX_HZ)))
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  /* Measure PLL4: read the kernel clock at the native divider and scale back up. */
  ret = audio_out_clock_config(AUDIO_OUT_IC_DIV_NATIVE, &kerHz);
  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }
  srcHz = (uint64_t)kerHz * (uint64_t)AUDIO_OUT_IC_DIV_NATIVE;

  if (SampleRate == BSP_AUDIO_FREQUENCY_NATIVE)
  {
    *pIcDiv    = AUDIO_OUT_IC_DIV_NATIVE;
    *pMckDiv   = AUDIO_OUT_MCKDIV_NATIVE;
    *pAchieved = kerHz / (AUDIO_OUT_MCKDIV_NATIVE * AUDIO_OUT_FRAME_BITS);
    return (*pAchieved == 0U) ? BSP_ERROR_CLOCK_FAILURE : BSP_ERROR_NONE;
  }

  for (mckDiv = 1U; mckDiv <= AUDIO_OUT_MCKDIV_MAX; mckDiv++)
  {
    uint64_t denom = (uint64_t)SampleRate * (uint64_t)AUDIO_OUT_FRAME_BITS * (uint64_t)mckDiv;
    uint64_t icDiv = (srcHz + (denom / 2U)) / denom;
    uint64_t total;
    uint32_t candidate;
    uint32_t err;

    if (icDiv == 0U)
    {
      icDiv = 1U;
    }
    if (icDiv > (uint64_t)AUDIO_IC_DIV_MAX)
    {
      continue;
    }

    total     = icDiv * (uint64_t)mckDiv * (uint64_t)AUDIO_OUT_FRAME_BITS;
    candidate = (uint32_t)(srcHz / total);
    err       = audio_abs_diff(candidate, SampleRate);

    if (err < bestErr)
    {
      bestErr  = err;
      bestIc   = (uint32_t)icDiv;
      bestMck  = mckDiv;
      bestRate = candidate;
    }
  }

  /* Nothing is programmed for a rate already known to be out of reach. */
  if ((bestIc != 0U) && (audio_rate_acceptable(bestRate, SampleRate) == 1U))
  {
    ret = audio_out_clock_config(bestIc, &kerHz);
    if (ret == BSP_ERROR_NONE)
    {
      uint32_t achieved = kerHz / (bestMck * AUDIO_OUT_FRAME_BITS);

      if (audio_rate_acceptable(achieved, SampleRate) == 1U)
      {
        *pIcDiv    = bestIc;
        *pMckDiv   = bestMck;
        *pAchieved = achieved;
        return BSP_ERROR_NONE;
      }
      ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
    }
  }
  else
  {
    ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  /* Nothing usable: put the tree back where it was found. */
  (void)audio_out_clock_config(AUDIO_OUT_IC_DIV_NATIVE, &kerHz);

  return ret;
}

/* Playback: amplifier control */

/* Drive PB6 low before making it an output: SD_MODE has no pull and floats until then. */
static void audio_out_amp_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(AUDIO_OUT_AMP_PORT, AUDIO_OUT_AMP_PIN, GPIO_PIN_RESET);

  gpio.Pin   = AUDIO_OUT_AMP_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AUDIO_OUT_AMP_PORT, &gpio);
}

/* SD_MODE high is on, in left-channel mode (so mono content goes in both slots). */
static void audio_out_amp_enable(uint32_t Enable)
{
  HAL_GPIO_WritePin(AUDIO_OUT_AMP_PORT, AUDIO_OUT_AMP_PIN,
                    (Enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Playback: pins, DMA, isolation */

static void audio_out_gpio_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = AUDIO_OUT_SAI_AF;

  gpio.Pin = AUDIO_OUT_SCK_PIN | AUDIO_OUT_FS_PIN;
  HAL_GPIO_Init(AUDIO_OUT_SCK_FS_PORT, &gpio);

  gpio.Pin = AUDIO_OUT_SD_PIN;
  HAL_GPIO_Init(AUDIO_OUT_SD_PORT, &gpio);
}

/* The circular queue is built once and reused: nothing in it depends on the caller. */
static int32_t audio_out_dma_init(void)
{
  DMA_NodeConfTypeDef node = {0};

  __HAL_RCC_GPDMA1_CLK_ENABLE();

  s_out_dma.Instance = AUDIO_OUT_DMA_INSTANCE;

  if (s_out_queue.Head == NULL)
  {
    node.NodeType                         = DMA_GPDMA_LINEAR_NODE;
    node.Init.Request                     = AUDIO_OUT_DMA_REQUEST;
    node.Init.BlkHWRequest                = DMA_BREQ_SINGLE_BURST;
    node.Init.Direction                   = DMA_MEMORY_TO_PERIPH;
    node.Init.SrcInc                      = DMA_SINC_INCREMENTED;
    node.Init.DestInc                     = DMA_DINC_FIXED;
    node.Init.SrcDataWidth                = DMA_SRC_DATAWIDTH_HALFWORD;
    node.Init.DestDataWidth               = DMA_DEST_DATAWIDTH_HALFWORD;
    node.Init.SrcBurstLength              = 1U;
    node.Init.DestBurstLength             = 1U;
    node.Init.Priority                    = DMA_HIGH_PRIORITY;
    node.Init.TransferEventMode           = DMA_TCEM_BLOCK_TRANSFER;
    node.Init.TransferAllocatedPort       = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
    node.DataHandlingConfig.DataExchange  = DMA_EXCHANGE_NONE;
    node.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
    node.TriggerConfig.TriggerPolarity    = DMA_TRIG_POLARITY_MASKED;
#if defined(CPU_IN_SECURE_STATE)
    node.SrcSecure                        = DMA_CHANNEL_SRC_SEC;
    node.DestSecure                       = DMA_CHANNEL_DEST_SEC;
#endif

    if (HAL_DMAEx_List_BuildNode(&node, &s_out_node) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
    if (HAL_DMAEx_List_InsertNode_Tail(&s_out_queue, &s_out_node) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
    if (HAL_DMAEx_List_SetCircularMode(&s_out_queue) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
  }

  s_out_dma.InitLinkedList.Priority          = DMA_HIGH_PRIORITY;
  s_out_dma.InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
  s_out_dma.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT1;
  s_out_dma.InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
  s_out_dma.InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;

  if (HAL_DMAEx_List_Init(&s_out_dma) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* RIF grants apply only in the secure state, which the shipped firmware runs in. */
#if defined(CPU_IN_SECURE_STATE)
  __HAL_RCC_RIFSC_CLK_ENABLE();

  if (HAL_DMA_ConfigChannelAttributes(&s_out_dma,
                                      DMA_CHANNEL_SEC | DMA_CHANNEL_PRIV |
                                      DMA_CHANNEL_SRC_SEC | DMA_CHANNEL_DEST_SEC) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* The secure channel writes the SAI1 slave. */
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_SAI1,
                                        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
#endif

  if (HAL_DMAEx_List_LinkQ(&s_out_dma, &s_out_queue) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  __HAL_LINKDMA(&s_out_sai, hdmatx, s_out_dma);

  HAL_NVIC_SetPriority(BSP_AUDIO_OUT_DMA_IRQn, BSP_AUDIO_OUT_IT_PRIORITY, 0U);
  HAL_NVIC_EnableIRQ(BSP_AUDIO_OUT_DMA_IRQn);

  return BSP_ERROR_NONE;
}

/* Mono mode sends each sample in both slots, so mono plays at the frame rate. */
static int32_t audio_out_sai_init(void)
{
  s_out_sai.Instance             = AUDIO_OUT_SAI_BLOCK;
  s_out_sai.Init.AudioMode       = SAI_MODEMASTER_TX;
  s_out_sai.Init.Synchro         = SAI_ASYNCHRONOUS;
  s_out_sai.Init.OutputDrive     = SAI_OUTPUTDRIVE_ENABLE;
  s_out_sai.Init.NoDivider       = SAI_MASTERDIVIDER_DISABLE;
  s_out_sai.Init.FIFOThreshold   = SAI_FIFOTHRESHOLD_1QF;
  s_out_sai.Init.AudioFrequency  = SAI_AUDIO_FREQUENCY_MCKDIV;
  s_out_sai.Init.Mckdiv          = s_out_ctx.MckDiv;
  s_out_sai.Init.MckOutput       = SAI_MCK_OUTPUT_DISABLE;
  s_out_sai.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  s_out_sai.Init.MonoStereoMode  = (s_out_ctx.ChannelsNbr == 1U) ? SAI_MONOMODE : SAI_STEREOMODE;
  s_out_sai.Init.CompandingMode  = SAI_NOCOMPANDING;
  s_out_sai.Init.TriState        = SAI_OUTPUT_NOTRELEASED;

  if (HAL_SAI_InitProtocol(&s_out_sai, SAI_I2S_STANDARD,
                           SAI_PROTOCOL_DATASIZE_16BIT, 2U) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return BSP_ERROR_NONE;
}

/* Pre-write the three words HAL_SAI_Transmit_DMA writes and clean the node, so the
 * descriptor is in memory before the HAL enables the channel. */
static void audio_out_prime_node(uint32_t SrcAddr, uint32_t Bytes)
{
  s_out_node.LinkRegisters[NODE_CBR1_DEFAULT_OFFSET] = Bytes;
  s_out_node.LinkRegisters[NODE_CSAR_DEFAULT_OFFSET] = SrcAddr;
  s_out_node.LinkRegisters[NODE_CDAR_DEFAULT_OFFSET] = (uint32_t)&AUDIO_OUT_SAI_BLOCK->DR;

  audio_cache_clean(&s_out_node, (uint32_t)sizeof(s_out_node));
}

/* Playback: public interface */

/* Release the SAI, DMA, IRQ, pins and clock. Also undoes a part-way Init, whose state is
 * still RESET, so DeInit would skip it. */
static int32_t audio_out_release(void)
{
  int32_t ret = BSP_ERROR_NONE;

  HAL_NVIC_DisableIRQ(BSP_AUDIO_OUT_DMA_IRQn);

  if (HAL_SAI_DeInit(&s_out_sai) != HAL_OK)
  {
    ret = BSP_ERROR_PERIPH_FAILURE;
  }
  if (s_out_dma.Instance != NULL)
  {
    if (HAL_DMAEx_List_UnLinkQ(&s_out_dma) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    if (HAL_DMAEx_List_DeInit(&s_out_dma) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
  }

  HAL_GPIO_DeInit(AUDIO_OUT_SCK_FS_PORT, AUDIO_OUT_SCK_PIN | AUDIO_OUT_FS_PIN);
  HAL_GPIO_DeInit(AUDIO_OUT_SD_PORT, AUDIO_OUT_SD_PIN);
  /* Read back before gating the clock, so the last write has landed. */
  (void)READ_REG(s_out_sai.Instance->CR1);
  __DSB();
  __HAL_RCC_SAI1_CLK_DISABLE();

  return ret;
}

int32_t BSP_AUDIO_OUT_Init(uint32_t Instance, BSP_AUDIO_Init_t *AudioInit)
{
  uint32_t icDiv;
  uint32_t mckDiv;
  uint32_t achieved;
  int32_t  ret;

  if ((Instance >= BSP_AUDIO_OUT_INSTANCES_NBR) || (AudioInit == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (AudioInit->Device != BSP_AUDIO_OUT_DEVICE_SPEAKER)
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  if (AudioInit->BitsPerSample != BSP_AUDIO_RESOLUTION_16B)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if ((AudioInit->ChannelsNbr != 1U) && (AudioInit->ChannelsNbr != 2U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_out_ctx.State != BSP_AUDIO_OUT_STATE_RESET)
  {
    return BSP_ERROR_BUSY;
  }

  /* Amplifier down before anything starts clocking. */
  audio_out_amp_init();

  ret = audio_out_setup_clock(AudioInit->SampleRate, &icDiv, &mckDiv, &achieved);
  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }

  s_out_ctx.IcDiv       = icDiv;
  s_out_ctx.MckDiv      = mckDiv;
  s_out_ctx.SampleRate  = achieved;
  s_out_ctx.ChannelsNbr = AudioInit->ChannelsNbr;
  s_out_ctx.IsMuted     = BSP_AUDIO_MUTE_DISABLED;

  __HAL_RCC_SAI1_CLK_ENABLE();
  /* From reset, whatever an earlier run left in the SAI. */
  __HAL_RCC_SAI1_FORCE_RESET();
  __HAL_RCC_SAI1_RELEASE_RESET();
  s_out_sai.Instance = AUDIO_OUT_SAI_BLOCK;
  audio_out_gpio_init();

  ret = audio_out_dma_init();
  if (ret == BSP_ERROR_NONE)
  {
    ret = audio_out_sai_init();
  }
  if (ret != BSP_ERROR_NONE)
  {
    (void)audio_out_release();
    return ret;
  }

  s_out_ctx.State = BSP_AUDIO_OUT_STATE_STOP;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_OUT_DeInit(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  if (Instance >= BSP_AUDIO_OUT_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_out_ctx.State == BSP_AUDIO_OUT_STATE_RESET)
  {
    return BSP_ERROR_NONE;
  }

  /* Silence first, teardown second: the reverse order pops. */
  audio_out_amp_enable(0U);

  if (s_out_ctx.State != BSP_AUDIO_OUT_STATE_STOP)
  {
    if (HAL_SAI_DMAStop(&s_out_sai) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
  }

  if (audio_out_release() != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_PERIPH_FAILURE;
  }

  /* PB6 stays driven low, so the amplifier stays shut down. */

  s_out_ctx.State      = BSP_AUDIO_OUT_STATE_RESET;
  s_out_ctx.SampleRate = 0U;

  return ret;
}

int32_t BSP_AUDIO_OUT_Play(uint32_t Instance, uint8_t *pData, uint32_t NbrOfBytes)
{
  if ((Instance >= BSP_AUDIO_OUT_INSTANCES_NBR) || (pData == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if ((NbrOfBytes == 0U) || ((NbrOfBytes % 8U) != 0U) || (NbrOfBytes > AUDIO_OUT_MAX_BYTES))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if ((((uint32_t)pData) % 4U) != 0U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_out_ctx.State != BSP_AUDIO_OUT_STATE_STOP)
  {
    return BSP_ERROR_BUSY;
  }

  /* Whatever the caller prepared is still sitting in the D-cache. */
  audio_cache_clean(pData, NbrOfBytes);

  audio_out_prime_node((uint32_t)pData, NbrOfBytes);

  if (HAL_SAI_Transmit_DMA(&s_out_sai, pData, (uint16_t)(NbrOfBytes / 2U)) != HAL_OK)
  {
    /* A FIFO timeout leaves the DMA running and the SAI busy: stop both. */
    (void)HAL_SAI_DMAStop(&s_out_sai);
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* Clocks are live and the FIFO holds real samples: safe to unshut. */
  if (s_out_ctx.IsMuted == BSP_AUDIO_MUTE_DISABLED)
  {
    audio_out_amp_enable(1U);
  }

  s_out_ctx.State = BSP_AUDIO_OUT_STATE_PLAYING;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_OUT_Pause(uint32_t Instance)
{
  if (Instance >= BSP_AUDIO_OUT_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_out_ctx.State != BSP_AUDIO_OUT_STATE_PLAYING)
  {
    return BSP_ERROR_BUSY;
  }

  /* The bit clock keeps running with no data: shut the amplifier before the underrun. */
  audio_out_amp_enable(0U);

  if (HAL_SAI_DMAPause(&s_out_sai) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  s_out_ctx.State = BSP_AUDIO_OUT_STATE_PAUSE;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_OUT_Resume(uint32_t Instance)
{
  if (Instance >= BSP_AUDIO_OUT_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_out_ctx.State != BSP_AUDIO_OUT_STATE_PAUSE)
  {
    return BSP_ERROR_BUSY;
  }

  /* A DMA error since the pause ended the transfer; Stop then Play restarts it. */
  if (s_out_sai.State != HAL_SAI_STATE_BUSY_TX)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  if (HAL_SAI_DMAResume(&s_out_sai) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  if (s_out_ctx.IsMuted == BSP_AUDIO_MUTE_DISABLED)
  {
    audio_out_amp_enable(1U);
  }

  s_out_ctx.State = BSP_AUDIO_OUT_STATE_PLAYING;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_OUT_Stop(uint32_t Instance)
{
  if (Instance >= BSP_AUDIO_OUT_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if ((s_out_ctx.State != BSP_AUDIO_OUT_STATE_PLAYING) &&
      (s_out_ctx.State != BSP_AUDIO_OUT_STATE_PAUSE))
  {
    return BSP_ERROR_BUSY;
  }

  audio_out_amp_enable(0U);

  if (HAL_SAI_DMAStop(&s_out_sai) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  s_out_ctx.State = BSP_AUDIO_OUT_STATE_STOP;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_OUT_Mute(uint32_t Instance)
{
  if (Instance >= BSP_AUDIO_OUT_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_out_ctx.State == BSP_AUDIO_OUT_STATE_RESET)
  {
    return BSP_ERROR_NO_INIT;
  }

  audio_out_amp_enable(0U);
  s_out_ctx.IsMuted = BSP_AUDIO_MUTE_ENABLED;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_OUT_UnMute(uint32_t Instance)
{
  if (Instance >= BSP_AUDIO_OUT_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_out_ctx.State == BSP_AUDIO_OUT_STATE_RESET)
  {
    return BSP_ERROR_NO_INIT;
  }

  s_out_ctx.IsMuted = BSP_AUDIO_MUTE_DISABLED;

  /* Drive the pin only while playing; otherwise the next Play or Resume does. */
  if (s_out_ctx.State == BSP_AUDIO_OUT_STATE_PLAYING)
  {
    audio_out_amp_enable(1U);
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_OUT_IsMute(uint32_t Instance, uint32_t *IsMute)
{
  if ((Instance >= BSP_AUDIO_OUT_INSTANCES_NBR) || (IsMute == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_out_ctx.State == BSP_AUDIO_OUT_STATE_RESET)
  {
    return BSP_ERROR_NO_INIT;
  }

  *IsMute = s_out_ctx.IsMuted;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_OUT_SetDevice(uint32_t Instance, uint32_t Device)
{
  if (Instance >= BSP_AUDIO_OUT_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_out_ctx.State == BSP_AUDIO_OUT_STATE_RESET)
  {
    return BSP_ERROR_NO_INIT;
  }
  if (Device != BSP_AUDIO_OUT_DEVICE_SPEAKER)
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  /* One amplifier: nothing to program. */
  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_OUT_GetDevice(uint32_t Instance, uint32_t *Device)
{
  if ((Instance >= BSP_AUDIO_OUT_INSTANCES_NBR) || (Device == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_out_ctx.State == BSP_AUDIO_OUT_STATE_RESET)
  {
    return BSP_ERROR_NO_INIT;
  }

  *Device = BSP_AUDIO_OUT_DEVICE_SPEAKER;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_OUT_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample)
{
  if (Instance >= BSP_AUDIO_OUT_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_out_ctx.State == BSP_AUDIO_OUT_STATE_RESET)
  {
    return BSP_ERROR_NO_INIT;
  }
  if (s_out_ctx.State != BSP_AUDIO_OUT_STATE_STOP)
  {
    return BSP_ERROR_BUSY;
  }
  if (BitsPerSample != BSP_AUDIO_RESOLUTION_16B)
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_OUT_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample)
{
  if ((Instance >= BSP_AUDIO_OUT_INSTANCES_NBR) || (BitsPerSample == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_out_ctx.State == BSP_AUDIO_OUT_STATE_RESET)
  {
    return BSP_ERROR_NO_INIT;
  }

  *BitsPerSample = BSP_AUDIO_RESOLUTION_16B;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_OUT_SetChannelsNbr(uint32_t Instance, uint32_t ChannelsNbr)
{
  uint32_t prevChannels;
  int32_t  ret;

  if (Instance >= BSP_AUDIO_OUT_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_out_ctx.State == BSP_AUDIO_OUT_STATE_RESET)
  {
    return BSP_ERROR_NO_INIT;
  }
  if (s_out_ctx.State != BSP_AUDIO_OUT_STATE_STOP)
  {
    return BSP_ERROR_BUSY;
  }
  if ((ChannelsNbr != 1U) && (ChannelsNbr != 2U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (ChannelsNbr == s_out_ctx.ChannelsNbr)
  {
    return BSP_ERROR_NONE;
  }

  /* Mono and stereo are different SAI configurations. */
  prevChannels          = s_out_ctx.ChannelsNbr;
  s_out_ctx.ChannelsNbr = ChannelsNbr;

  __HAL_SAI_DISABLE(&s_out_sai);

  ret = audio_out_sai_init();
  if (ret != BSP_ERROR_NONE)
  {
    s_out_ctx.ChannelsNbr = prevChannels;
    (void)audio_out_sai_init();
  }

  return ret;
}

int32_t BSP_AUDIO_OUT_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelsNbr)
{
  if ((Instance >= BSP_AUDIO_OUT_INSTANCES_NBR) || (ChannelsNbr == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_out_ctx.State == BSP_AUDIO_OUT_STATE_RESET)
  {
    return BSP_ERROR_NO_INIT;
  }

  *ChannelsNbr = s_out_ctx.ChannelsNbr;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_OUT_SetVolume(uint32_t Instance, uint32_t Volume)
{
  (void)Instance;
  (void)Volume;

  /* No codec: the MAX98357A gain is set by its GAIN pin. */
  return BSP_ERROR_FEATURE_NOT_SUPPORTED;
}

int32_t BSP_AUDIO_OUT_GetVolume(uint32_t Instance, uint32_t *Volume)
{
  (void)Instance;
  (void)Volume;

  return BSP_ERROR_FEATURE_NOT_SUPPORTED;
}

int32_t BSP_AUDIO_OUT_SetSampleRate(uint32_t Instance, uint32_t SampleRate)
{
  uint32_t icDiv;
  uint32_t mckDiv;
  uint32_t achieved;
  uint32_t prevIcDiv;
  uint32_t prevMckDiv;
  uint32_t kerHz = 0U;
  int32_t  ret;

  if (Instance >= BSP_AUDIO_OUT_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_out_ctx.State != BSP_AUDIO_OUT_STATE_STOP)
  {
    return BSP_ERROR_BUSY;
  }

  prevIcDiv  = s_out_ctx.IcDiv;
  prevMckDiv = s_out_ctx.MckDiv;

  ret = audio_out_setup_clock(SampleRate, &icDiv, &mckDiv, &achieved);
  if (ret != BSP_ERROR_NONE)
  {
    (void)audio_out_clock_config(prevIcDiv, &kerHz);
    return ret;
  }

  s_out_ctx.IcDiv      = icDiv;
  s_out_ctx.MckDiv     = mckDiv;
  s_out_ctx.SampleRate = achieved;

  __HAL_SAI_DISABLE(&s_out_sai);

  ret = audio_out_sai_init();
  if (ret != BSP_ERROR_NONE)
  {
    s_out_ctx.IcDiv  = prevIcDiv;
    s_out_ctx.MckDiv = prevMckDiv;
    (void)audio_out_clock_config(prevIcDiv, &kerHz);
    s_out_ctx.SampleRate = kerHz / (prevMckDiv * AUDIO_OUT_FRAME_BITS);
    (void)audio_out_sai_init();
  }

  return ret;
}

int32_t BSP_AUDIO_OUT_GetSampleRate(uint32_t Instance, uint32_t *SampleRate)
{
  if ((Instance >= BSP_AUDIO_OUT_INSTANCES_NBR) || (SampleRate == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_out_ctx.State == BSP_AUDIO_OUT_STATE_RESET)
  {
    return BSP_ERROR_NO_INIT;
  }

  *SampleRate = s_out_ctx.SampleRate;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_OUT_GetState(uint32_t Instance, uint32_t *State)
{
  if ((Instance >= BSP_AUDIO_OUT_INSTANCES_NBR) || (State == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  *State = s_out_ctx.State;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_OUT_CleanBuffer(uint32_t Instance, const void *pData, uint32_t NbrOfBytes)
{
  if ((Instance >= BSP_AUDIO_OUT_INSTANCES_NBR) || (pData == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  audio_cache_clean(pData, NbrOfBytes);

  return BSP_ERROR_NONE;
}

__weak void BSP_AUDIO_OUT_TransferComplete_CallBack(uint32_t Instance)
{
  (void)Instance;
}

__weak void BSP_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Instance)
{
  (void)Instance;
}

__weak void BSP_AUDIO_OUT_Error_CallBack(uint32_t Instance)
{
  (void)Instance;
}

void BSP_AUDIO_OUT_IRQHandler(uint32_t Instance)
{
  if (Instance < BSP_AUDIO_OUT_INSTANCES_NBR)
  {
    HAL_DMA_IRQHandler(&s_out_dma);
  }
}

/* Capture: clocking */

/* Program IC8 (from PLL4) for MDF1 and read back the kernel clock. */
static int32_t audio_in_clock_config(uint32_t IcDiv, uint32_t *pKerHz)
{
  RCC_PeriphCLKInitTypeDef periph = {0};

  periph.PeriphClockSelection                          = RCC_PERIPHCLK_MDF1;
  periph.Mdf1ClockSelection                            = RCC_MDF1CLKSOURCE_IC8;
  periph.ICSelection[AUDIO_IN_IC_INDEX].ClockSelection = RCC_ICCLKSOURCE_PLL4;
  periph.ICSelection[AUDIO_IN_IC_INDEX].ClockDivider   = IcDiv;

  if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK)
  {
    return BSP_ERROR_CLOCK_FAILURE;
  }

  *pKerHz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_MDF1);

  return (*pKerHz == 0U) ? BSP_ERROR_CLOCK_FAILURE : BSP_ERROR_NONE;
}

static uint32_t audio_in_rate(uint32_t KerHz, uint32_t CckDiv, uint32_t Decimation)
{
  return KerHz / (AUDIO_IN_PROC_DIV * CckDiv * s_in_decimations[Decimation].Cic *
                  AUDIO_IN_RSF_DECIMATION);
}

/* Searches IC8, CCKDIV and the decimation with the microphone clock in range; the PCM
 * divisor follows the decimation, so the level does not change. */
static int32_t audio_in_setup_clock(uint32_t SampleRate, uint32_t *pIcDiv,
                                    uint32_t *pCckDiv, uint32_t *pDecimation,
                                    uint32_t *pAchieved)
{
  uint32_t kerHz;
  uint64_t srcHz;
  uint32_t bestIc   = 0U;
  uint32_t bestCck  = 0U;
  uint32_t bestDec  = 0U;
  uint32_t bestRate = 0U;
  uint32_t bestErr  = 0xFFFFFFFFU;
  uint32_t cckDiv;
  uint32_t dec;
  int32_t  ret;

  ret = audio_in_clock_config(AUDIO_IN_IC_DIV_NATIVE, &kerHz);
  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }
  srcHz = (uint64_t)kerHz * (uint64_t)AUDIO_IN_IC_DIV_NATIVE;

  if (SampleRate == BSP_AUDIO_FREQUENCY_NATIVE)
  {
    *pIcDiv      = AUDIO_IN_IC_DIV_NATIVE;
    *pCckDiv     = AUDIO_IN_CCKDIV_NATIVE;
    *pDecimation = AUDIO_IN_DECIMATION_NATIVE;
    *pAchieved   = audio_in_rate(kerHz, AUDIO_IN_CCKDIV_NATIVE, AUDIO_IN_DECIMATION_NATIVE);
    return (*pAchieved == 0U) ? BSP_ERROR_CLOCK_FAILURE : BSP_ERROR_NONE;
  }

  for (dec = 0U; dec < AUDIO_IN_DECIMATIONS; dec++)
  {
    for (cckDiv = AUDIO_IN_CCKDIV_MIN; cckDiv <= AUDIO_IN_CCKDIV_MAX; cckDiv++)
    {
      uint64_t perIc = (uint64_t)AUDIO_IN_PROC_DIV * cckDiv * s_in_decimations[dec].Cic *
                       AUDIO_IN_RSF_DECIMATION;
      uint64_t denom = (uint64_t)SampleRate * perIc;
      uint64_t icDiv = (srcHz + (denom / 2U)) / denom;
      uint64_t bitHz;
      uint32_t candidate;
      uint32_t err;

      if (icDiv == 0U)
      {
        icDiv = 1U;
      }
      if (icDiv > (uint64_t)AUDIO_IC_DIV_MAX)
      {
        continue;
      }

      /* The bitstream clock this gives, which the microphone has to accept. */
      bitHz = srcHz / (icDiv * AUDIO_IN_PROC_DIV * cckDiv);
      if ((bitHz < AUDIO_IN_MIC_CLK_MIN_HZ) || (bitHz > AUDIO_IN_MIC_CLK_MAX_HZ))
      {
        continue;
      }

      candidate = (uint32_t)(srcHz / (icDiv * perIc));
      err       = audio_abs_diff(candidate, SampleRate);

      if (err < bestErr)
      {
        bestErr  = err;
        bestIc   = (uint32_t)icDiv;
        bestCck  = cckDiv;
        bestDec  = dec;
        bestRate = candidate;
      }
    }
  }

  /* Nothing is programmed for a rate already known to be out of reach. */
  if ((bestIc != 0U) && (audio_rate_acceptable(bestRate, SampleRate) == 1U))
  {
    ret = audio_in_clock_config(bestIc, &kerHz);
    if (ret == BSP_ERROR_NONE)
    {
      uint32_t achieved = audio_in_rate(kerHz, bestCck, bestDec);

      if (audio_rate_acceptable(achieved, SampleRate) == 1U)
      {
        *pIcDiv      = bestIc;
        *pCckDiv     = bestCck;
        *pDecimation = bestDec;
        *pAchieved   = achieved;
        return BSP_ERROR_NONE;
      }
      ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
    }
  }
  else
  {
    ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  (void)audio_in_clock_config(AUDIO_IN_IC_DIV_NATIVE, &kerHz);

  return ret;
}

/* Capture: pins, DMA, isolation */

static void audio_in_gpio_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = AUDIO_IN_MDF_AF;

  gpio.Pin = AUDIO_IN_SDI_PIN;
  HAL_GPIO_Init(AUDIO_IN_SDI_PORT, &gpio);

  gpio.Pin = AUDIO_IN_CCK_PIN;
  HAL_GPIO_Init(AUDIO_IN_CCK_PORT, &gpio);
}

static int32_t audio_in_dma_init(void)
{
  DMA_NodeConfTypeDef node = {0};

  __HAL_RCC_GPDMA1_CLK_ENABLE();

  s_in_dma.Instance = AUDIO_IN_DMA_INSTANCE;

  if (s_in_queue.Head == NULL)
  {
    node.NodeType                         = DMA_GPDMA_LINEAR_NODE;
    node.Init.Request                     = AUDIO_IN_DMA_REQUEST;
    node.Init.BlkHWRequest                = DMA_BREQ_SINGLE_BURST;
    node.Init.Direction                   = DMA_PERIPH_TO_MEMORY;
    node.Init.SrcInc                      = DMA_SINC_FIXED;
    node.Init.DestInc                     = DMA_DINC_INCREMENTED;
    node.Init.SrcDataWidth                = DMA_SRC_DATAWIDTH_WORD;
    node.Init.DestDataWidth               = DMA_DEST_DATAWIDTH_WORD;
    node.Init.SrcBurstLength              = 1U;
    node.Init.DestBurstLength             = 1U;
    node.Init.Priority                    = DMA_HIGH_PRIORITY;
    node.Init.TransferEventMode           = DMA_TCEM_BLOCK_TRANSFER;
    node.Init.TransferAllocatedPort       = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
    node.DataHandlingConfig.DataExchange  = DMA_EXCHANGE_NONE;
    node.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
    node.TriggerConfig.TriggerPolarity    = DMA_TRIG_POLARITY_MASKED;
#if defined(CPU_IN_SECURE_STATE)
    node.SrcSecure                        = DMA_CHANNEL_SRC_SEC;
    node.DestSecure                       = DMA_CHANNEL_DEST_SEC;
#endif

    if (HAL_DMAEx_List_BuildNode(&node, &s_in_node) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
    if (HAL_DMAEx_List_InsertNode_Tail(&s_in_queue, &s_in_node) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
    if (HAL_DMAEx_List_SetCircularMode(&s_in_queue) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
  }

  s_in_dma.InitLinkedList.Priority          = DMA_HIGH_PRIORITY;
  s_in_dma.InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
  s_in_dma.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT1;
  s_in_dma.InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
  s_in_dma.InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;

  if (HAL_DMAEx_List_Init(&s_in_dma) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* RIF grants apply only in the secure state. */
#if defined(CPU_IN_SECURE_STATE)
  __HAL_RCC_RIFSC_CLK_ENABLE();

  if (HAL_DMA_ConfigChannelAttributes(&s_in_dma,
                                      DMA_CHANNEL_SEC | DMA_CHANNEL_PRIV |
                                      DMA_CHANNEL_SRC_SEC | DMA_CHANNEL_DEST_SEC) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* The secure channel reads the MDF1 slave. */
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_MDF1,
                                        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
#endif

  if (HAL_DMAEx_List_LinkQ(&s_in_dma, &s_in_queue) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  __HAL_LINKDMA(&s_in_mdf, hdma, s_in_dma);

  HAL_NVIC_SetPriority(BSP_AUDIO_IN_DMA_IRQn, BSP_AUDIO_IN_IT_PRIORITY, 0U);
  HAL_NVIC_EnableIRQ(BSP_AUDIO_IN_DMA_IRQn);

  return BSP_ERROR_NONE;
}

/* Stop the interface while it still has a clock; call before every HAL_MDF_DeInit. */
static void audio_in_quiesce(void)
{
  uint32_t start;

  AUDIO_IN_MDF_FILTER->SITFCR &= ~MDF_SITFCR_SITFEN;

  start = HAL_GetTick();
  while ((AUDIO_IN_MDF_FILTER->SITFCR & MDF_SITFCR_SITFACTIVE) != 0U)
  {
    if ((HAL_GetTick() - start) > AUDIO_IN_IDLE_TIMEOUT_MS)
    {
      break;
    }
  }
}

/* SPI mode from CCK0, sampled on the falling edge: how this microphone is wired. */
static int32_t audio_in_mdf_init(void)
{
  uint32_t start;

  s_in_mdf.Instance = AUDIO_IN_MDF_FILTER;

  /* Wait for the peripheral to finish shutting down before setting it up. */
  start = HAL_GetTick();
  while (((MDF1->CKGCR & MDF_CKGCR_CCKACTIVE) != 0U) ||
         ((AUDIO_IN_MDF_FILTER->SITFCR & MDF_SITFCR_SITFACTIVE) != 0U))
  {
    if ((HAL_GetTick() - start) > AUDIO_IN_IDLE_TIMEOUT_MS)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
  }

  s_in_mdf.Init.CommonParam.InterleavedFilters            = 0U;
  s_in_mdf.Init.CommonParam.ProcClockDivider              = AUDIO_IN_PROC_DIV;
  s_in_mdf.Init.CommonParam.OutputClock.Activation        = ENABLE;
  s_in_mdf.Init.CommonParam.OutputClock.Pins              = MDF_OUTPUT_CLOCK_0;
  s_in_mdf.Init.CommonParam.OutputClock.Divider           = s_in_ctx.CckDiv;
  s_in_mdf.Init.CommonParam.OutputClock.Trigger.Activation = DISABLE;
  s_in_mdf.Init.SerialInterface.Activation                = ENABLE;
  s_in_mdf.Init.SerialInterface.Mode                      = MDF_SITF_NORMAL_SPI_MODE;
  s_in_mdf.Init.SerialInterface.ClockSource               = MDF_SITF_CCK0_SOURCE;
  s_in_mdf.Init.SerialInterface.Threshold                 = AUDIO_IN_SITF_THRESHOLD;
  s_in_mdf.Init.FilterBistream                            = MDF_BITSTREAM1_FALLING;

  if (HAL_MDF_Init(&s_in_mdf) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  s_in_filter.DataSource                     = MDF_DATA_SOURCE_BSMX;
  s_in_filter.Delay                          = 0U;
  s_in_filter.CicMode                        = MDF_ONE_FILTER_SINC4;
  s_in_filter.DecimationRatio                = s_in_decimations[s_in_ctx.Decimation].Cic;
  s_in_filter.Offset                         = 0;
  s_in_filter.Gain                           = 1;
  s_in_filter.ReshapeFilter.Activation       = ENABLE;
  s_in_filter.ReshapeFilter.DecimationRatio  = MDF_RSF_DECIMATION_RATIO_4;
  s_in_filter.HighPassFilter.Activation      = ENABLE;
  s_in_filter.HighPassFilter.CutOffFrequency = MDF_HPF_CUTOFF_0_000625FPCM;
  s_in_filter.Integrator.Activation          = DISABLE;
  s_in_filter.AcquisitionMode                = MDF_MODE_ASYNC_CONT;
  s_in_filter.FifoThreshold                  = MDF_FIFO_THRESHOLD_NOT_EMPTY;
  s_in_filter.DiscardSamples                 = 0U;

  return BSP_ERROR_NONE;
}

/* As audio_out_prime_node, for the capture descriptor. */
static void audio_in_prime_node(uint32_t Bytes)
{
  s_in_node.LinkRegisters[NODE_CBR1_DEFAULT_OFFSET] = Bytes;
  s_in_node.LinkRegisters[NODE_CSAR_DEFAULT_OFFSET] = (uint32_t)&AUDIO_IN_MDF_FILTER->DFLTDR;
  s_in_node.LinkRegisters[NODE_CDAR_DEFAULT_OFFSET] = (uint32_t)s_in_staging;

  audio_cache_clean(&s_in_node, (uint32_t)sizeof(s_in_node));
}

static int32_t audio_in_start_acquisition(void)
{
  MDF_DmaConfigTypeDef dmaConfig;
  uint32_t bytes = s_in_ctx.Samples * (uint32_t)sizeof(int32_t);

  audio_in_prime_node(bytes);

  dmaConfig.Address    = (uint32_t)s_in_staging;
  dmaConfig.DataLength = bytes;
  dmaConfig.MsbOnly    = DISABLE;

  if (HAL_MDF_AcqStart_DMA(&s_in_mdf, &s_in_filter, &dmaConfig) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return BSP_ERROR_NONE;
}

/* After a DMA error HAL_MDF_AcqStop_DMA fails, leaving the filter running, and refuses
 * every later stop: so take the filter down and rebuild it. */
static int32_t audio_in_stop_acquisition(void)
{
  if (HAL_MDF_AcqStop_DMA(&s_in_mdf) == HAL_OK)
  {
    return BSP_ERROR_NONE;
  }

  audio_in_quiesce();
  (void)HAL_MDF_DeInit(&s_in_mdf);

  if (audio_in_dma_init() != BSP_ERROR_NONE)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return audio_in_mdf_init();
}

/* ISR context. The half is invalidated first; the CPU never writes the staging buffer,
 * so no dirty line can be lost. */
static void audio_in_convert_half(uint32_t Offset)
{
  uint32_t count   = s_in_ctx.Samples / 2U;
  int16_t *pcm     = s_in_ctx.pBuff;
  int32_t  divisor = s_in_decimations[s_in_ctx.Decimation].Divisor;
  uint32_t i;

  if (pcm == NULL)
  {
    return;
  }

  audio_cache_invalidate(&s_in_staging[Offset], count * (uint32_t)sizeof(int32_t));

  for (i = 0U; i < count; i++)
  {
    int32_t sample = s_in_staging[Offset + i] / divisor;

    if (sample > AUDIO_PCM16_MAX)
    {
      sample = AUDIO_PCM16_MAX;
    }
    if (sample < AUDIO_PCM16_MIN)
    {
      sample = AUDIO_PCM16_MIN;
    }
    pcm[Offset + i] = (int16_t)sample;
  }
}

/* Capture: public interface */

/* Release the filter, DMA, IRQ, pins and clock; also undoes a part-way Init. */
static int32_t audio_in_release(void)
{
  int32_t ret = BSP_ERROR_NONE;

  HAL_NVIC_DisableIRQ(BSP_AUDIO_IN_DMA_IRQn);

  audio_in_quiesce();

  if (HAL_MDF_DeInit(&s_in_mdf) != HAL_OK)
  {
    ret = BSP_ERROR_PERIPH_FAILURE;
  }
  if (s_in_dma.Instance != NULL)
  {
    if (HAL_DMAEx_List_UnLinkQ(&s_in_dma) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    if (HAL_DMAEx_List_DeInit(&s_in_dma) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
  }

  HAL_GPIO_DeInit(AUDIO_IN_SDI_PORT, AUDIO_IN_SDI_PIN);
  HAL_GPIO_DeInit(AUDIO_IN_CCK_PORT, AUDIO_IN_CCK_PIN);
  /* Read back before gating the clock, so the last write has landed. */
  (void)READ_REG(s_in_mdf.Instance->DFLTCR);
  __DSB();
  __HAL_RCC_MDF1_CLK_DISABLE();

  return ret;
}

int32_t BSP_AUDIO_IN_Init(uint32_t Instance, BSP_AUDIO_Init_t *AudioInit)
{
  uint32_t icDiv;
  uint32_t cckDiv;
  uint32_t decimation;
  uint32_t achieved;
  int32_t  ret;

  if ((Instance >= BSP_AUDIO_IN_INSTANCES_NBR) || (AudioInit == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (AudioInit->BitsPerSample != BSP_AUDIO_RESOLUTION_16B)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (AudioInit->Device != BSP_AUDIO_IN_DEVICE_DIGITAL_MIC)
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  if (AudioInit->ChannelsNbr != 1U)
  {
    /* One microphone, one bitstream, one filter. */
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_in_ctx.State != BSP_AUDIO_IN_STATE_RESET)
  {
    return BSP_ERROR_BUSY;
  }

  ret = audio_in_setup_clock(AudioInit->SampleRate, &icDiv, &cckDiv, &decimation, &achieved);
  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }

  s_in_ctx.IcDiv      = icDiv;
  s_in_ctx.CckDiv     = cckDiv;
  s_in_ctx.Decimation = decimation;
  s_in_ctx.SampleRate = achieved;
  s_in_ctx.pBuff      = NULL;
  s_in_ctx.Samples    = 0U;

  __HAL_RCC_MDF1_CLK_ENABLE();
  /* Reset: a clock removed mid-shutdown leaves the active bits set for good. */
  __HAL_RCC_MDF1_FORCE_RESET();
  __HAL_RCC_MDF1_RELEASE_RESET();
  s_in_mdf.Instance = AUDIO_IN_MDF_FILTER;
  audio_in_gpio_init();

  ret = audio_in_dma_init();
  if (ret == BSP_ERROR_NONE)
  {
    ret = audio_in_mdf_init();
  }
  if (ret != BSP_ERROR_NONE)
  {
    (void)audio_in_release();
    return ret;
  }

  s_in_ctx.State = BSP_AUDIO_IN_STATE_STOP;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_IN_DeInit(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  if (Instance >= BSP_AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_in_ctx.State == BSP_AUDIO_IN_STATE_RESET)
  {
    return BSP_ERROR_NONE;
  }

  if (s_in_ctx.State == BSP_AUDIO_IN_STATE_RECORDING)
  {
    if (HAL_MDF_AcqStop_DMA(&s_in_mdf) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
  }

  if (audio_in_release() != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_PERIPH_FAILURE;
  }

  s_in_ctx.State      = BSP_AUDIO_IN_STATE_RESET;
  s_in_ctx.SampleRate = 0U;
  s_in_ctx.pBuff      = NULL;
  s_in_ctx.Samples    = 0U;

  return ret;
}

int32_t BSP_AUDIO_IN_Record(uint32_t Instance, uint8_t *pData, uint32_t NbrOfBytes)
{
  int32_t ret;

  if ((Instance >= BSP_AUDIO_IN_INSTANCES_NBR) || (pData == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  /* Whole cache lines per staging half, so invalidating one cannot discard the other. */
  if ((NbrOfBytes == 0U) || ((NbrOfBytes % AUDIO_CACHE_LINE_BYTES) != 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if ((NbrOfBytes / 2U) > BSP_AUDIO_IN_STAGING_SAMPLES)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if ((((uint32_t)pData) % 2U) != 0U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_in_ctx.State != BSP_AUDIO_IN_STATE_STOP)
  {
    return BSP_ERROR_BUSY;
  }

  s_in_ctx.pBuff   = (int16_t *)(void *)pData;
  s_in_ctx.Samples = NbrOfBytes / 2U;

  ret = audio_in_start_acquisition();
  if (ret != BSP_ERROR_NONE)
  {
    s_in_ctx.pBuff   = NULL;
    s_in_ctx.Samples = 0U;
    return ret;
  }

  s_in_ctx.State = BSP_AUDIO_IN_STATE_RECORDING;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_IN_Pause(uint32_t Instance)
{
  if (Instance >= BSP_AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_in_ctx.State != BSP_AUDIO_IN_STATE_RECORDING)
  {
    return BSP_ERROR_BUSY;
  }

  if (audio_in_stop_acquisition() != BSP_ERROR_NONE)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  s_in_ctx.State = BSP_AUDIO_IN_STATE_PAUSE;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_IN_Resume(uint32_t Instance)
{
  int32_t ret;

  if (Instance >= BSP_AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_in_ctx.State != BSP_AUDIO_IN_STATE_PAUSE)
  {
    return BSP_ERROR_BUSY;
  }

  ret = audio_in_start_acquisition();
  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }

  s_in_ctx.State = BSP_AUDIO_IN_STATE_RECORDING;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_IN_Stop(uint32_t Instance)
{
  if (Instance >= BSP_AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if ((s_in_ctx.State != BSP_AUDIO_IN_STATE_RECORDING) &&
      (s_in_ctx.State != BSP_AUDIO_IN_STATE_PAUSE))
  {
    return BSP_ERROR_BUSY;
  }

  if (s_in_ctx.State == BSP_AUDIO_IN_STATE_RECORDING)
  {
    if (audio_in_stop_acquisition() != BSP_ERROR_NONE)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
  }

  s_in_ctx.pBuff   = NULL;
  s_in_ctx.Samples = 0U;
  s_in_ctx.State   = BSP_AUDIO_IN_STATE_STOP;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_IN_SetVolume(uint32_t Instance, uint32_t Volume)
{
  (void)Instance;
  (void)Volume;

  /* No microphone gain; the MDF gain is fixed to match the PCM scaling. */
  return BSP_ERROR_FEATURE_NOT_SUPPORTED;
}

int32_t BSP_AUDIO_IN_GetVolume(uint32_t Instance, uint32_t *Volume)
{
  (void)Instance;
  (void)Volume;

  return BSP_ERROR_FEATURE_NOT_SUPPORTED;
}

int32_t BSP_AUDIO_IN_SetSampleRate(uint32_t Instance, uint32_t SampleRate)
{
  uint32_t icDiv;
  uint32_t cckDiv;
  uint32_t decimation;
  uint32_t achieved;
  uint32_t prevIcDiv;
  uint32_t prevCckDiv;
  uint32_t prevDecimation;
  uint32_t kerHz = 0U;
  int32_t  ret;

  if (Instance >= BSP_AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_in_ctx.State != BSP_AUDIO_IN_STATE_STOP)
  {
    return BSP_ERROR_BUSY;
  }

  prevIcDiv      = s_in_ctx.IcDiv;
  prevCckDiv     = s_in_ctx.CckDiv;
  prevDecimation = s_in_ctx.Decimation;

  /* Take the peripheral down before changing anything. */
  audio_in_quiesce();

  if (HAL_MDF_DeInit(&s_in_mdf) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  ret = audio_in_setup_clock(SampleRate, &icDiv, &cckDiv, &decimation, &achieved);
  if (ret != BSP_ERROR_NONE)
  {
    /* The peripheral is down: bring it back up even though the rate is refused. */
    (void)audio_in_clock_config(prevIcDiv, &kerHz);
    (void)audio_in_dma_init();
    (void)audio_in_mdf_init();
    return ret;
  }

  s_in_ctx.IcDiv      = icDiv;
  s_in_ctx.CckDiv     = cckDiv;
  s_in_ctx.Decimation = decimation;
  s_in_ctx.SampleRate = achieved;

  /* The DMA has to be brought back too, not just the filter. */
  ret = audio_in_dma_init();
  if (ret == BSP_ERROR_NONE)
  {
    ret = audio_in_mdf_init();
  }
  if (ret != BSP_ERROR_NONE)
  {
    s_in_ctx.IcDiv      = prevIcDiv;
    s_in_ctx.CckDiv     = prevCckDiv;
    s_in_ctx.Decimation = prevDecimation;
    (void)audio_in_clock_config(prevIcDiv, &kerHz);
    s_in_ctx.SampleRate = audio_in_rate(kerHz, prevCckDiv, prevDecimation);
    (void)audio_in_dma_init();
    (void)audio_in_mdf_init();
  }

  return ret;
}

int32_t BSP_AUDIO_IN_GetSampleRate(uint32_t Instance, uint32_t *SampleRate)
{
  if ((Instance >= BSP_AUDIO_IN_INSTANCES_NBR) || (SampleRate == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_in_ctx.State == BSP_AUDIO_IN_STATE_RESET)
  {
    return BSP_ERROR_NO_INIT;
  }

  *SampleRate = s_in_ctx.SampleRate;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_IN_GetState(uint32_t Instance, uint32_t *State)
{
  if ((Instance >= BSP_AUDIO_IN_INSTANCES_NBR) || (State == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  *State = s_in_ctx.State;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_IN_SetDevice(uint32_t Instance, uint32_t Device)
{
  if (Instance >= BSP_AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_in_ctx.State == BSP_AUDIO_IN_STATE_RESET)
  {
    return BSP_ERROR_NO_INIT;
  }
  if (Device != BSP_AUDIO_IN_DEVICE_DIGITAL_MIC)
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  /* One microphone: nothing to program. */
  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_IN_GetDevice(uint32_t Instance, uint32_t *Device)
{
  if ((Instance >= BSP_AUDIO_IN_INSTANCES_NBR) || (Device == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_in_ctx.State == BSP_AUDIO_IN_STATE_RESET)
  {
    return BSP_ERROR_NO_INIT;
  }

  *Device = BSP_AUDIO_IN_DEVICE_DIGITAL_MIC;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_IN_SetChannelsNbr(uint32_t Instance, uint32_t ChannelsNbr)
{
  if (Instance >= BSP_AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_in_ctx.State == BSP_AUDIO_IN_STATE_RESET)
  {
    return BSP_ERROR_NO_INIT;
  }
  if (s_in_ctx.State != BSP_AUDIO_IN_STATE_STOP)
  {
    return BSP_ERROR_BUSY;
  }
  /* One microphone, one filter: mono only. */
  if (ChannelsNbr != 1U)
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_IN_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelsNbr)
{
  if ((Instance >= BSP_AUDIO_IN_INSTANCES_NBR) || (ChannelsNbr == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_in_ctx.State == BSP_AUDIO_IN_STATE_RESET)
  {
    return BSP_ERROR_NO_INIT;
  }

  *ChannelsNbr = 1U;

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_IN_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample)
{
  if (Instance >= BSP_AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_in_ctx.State == BSP_AUDIO_IN_STATE_RESET)
  {
    return BSP_ERROR_NO_INIT;
  }
  if (s_in_ctx.State != BSP_AUDIO_IN_STATE_STOP)
  {
    return BSP_ERROR_BUSY;
  }
  /* The filter's 24-bit output is narrowed to 16-bit PCM. */
  if (BitsPerSample != BSP_AUDIO_RESOLUTION_16B)
  {
    return BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_AUDIO_IN_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample)
{
  if ((Instance >= BSP_AUDIO_IN_INSTANCES_NBR) || (BitsPerSample == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if (s_in_ctx.State == BSP_AUDIO_IN_STATE_RESET)
  {
    return BSP_ERROR_NO_INIT;
  }

  *BitsPerSample = BSP_AUDIO_RESOLUTION_16B;

  return BSP_ERROR_NONE;
}

__weak void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  (void)Instance;
}

__weak void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
  (void)Instance;
}

__weak void BSP_AUDIO_IN_Error_CallBack(uint32_t Instance)
{
  (void)Instance;
}

void BSP_AUDIO_IN_IRQHandler(uint32_t Instance)
{
  if (Instance < BSP_AUDIO_IN_INSTANCES_NBR)
  {
    HAL_DMA_IRQHandler(&s_in_dma);
  }
}

/* Strong HAL callback definitions: this module owns them for the whole application. */

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == AUDIO_OUT_SAI_BLOCK)
  {
    BSP_AUDIO_OUT_TransferComplete_CallBack(0U);
  }
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == AUDIO_OUT_SAI_BLOCK)
  {
    BSP_AUDIO_OUT_HalfTransfer_CallBack(0U);
  }
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == AUDIO_OUT_SAI_BLOCK)
  {
    /* READY means a DMA error stopped the SAI (an underrun is only reported): shut the
     * amplifier, as Stop does, rather than leave it on over a stopped bit clock. */
    if (hsai->State == HAL_SAI_STATE_READY)
    {
      audio_out_amp_enable(0U);
    }
    BSP_AUDIO_OUT_Error_CallBack(0U);
  }
}

void HAL_MDF_AcqCpltCallback(MDF_HandleTypeDef *hmdf)
{
  if (hmdf->Instance == AUDIO_IN_MDF_FILTER)
  {
    audio_in_convert_half(s_in_ctx.Samples / 2U);
    BSP_AUDIO_IN_TransferComplete_CallBack(0U);
  }
}

void HAL_MDF_AcqHalfCpltCallback(MDF_HandleTypeDef *hmdf)
{
  if (hmdf->Instance == AUDIO_IN_MDF_FILTER)
  {
    audio_in_convert_half(0U);
    BSP_AUDIO_IN_HalfTransfer_CallBack(0U);
  }
}

void HAL_MDF_ErrorCallback(MDF_HandleTypeDef *hmdf)
{
  if (hmdf->Instance == AUDIO_IN_MDF_FILTER)
  {
    BSP_AUDIO_IN_Error_CallBack(0U);
  }
}
