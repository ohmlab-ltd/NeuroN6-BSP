/**
  * @file    neuron6_audio.h
  * @brief   Neuro N6 audio: SAI1 Block B playback to a MAX98357A, PDM microphone on MDF1.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_AUDIO_H
#define NEURON6_AUDIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "neuron6_conf.h"
#include "neuron6_errno.h"

/** @brief Playback GPDMA1 channel, 0..15. A plain decimal literal with no suffix: it is
  * token-pasted into the GPDMA1_ChannelN names. */
#ifndef BSP_AUDIO_OUT_DMA_CHANNEL
#define BSP_AUDIO_OUT_DMA_CHANNEL       2
#endif

/** @brief Capture GPDMA1 channel; same rules. */
#ifndef BSP_AUDIO_IN_DMA_CHANNEL
#define BSP_AUDIO_IN_DMA_CHANNEL        0
#endif

/** @brief Capture staging buffer in 32-bit words (16 to 16383, 4 bytes of RAM each);
  * a Record of NbrOfBytes needs NbrOfBytes / 2. */
#ifndef BSP_AUDIO_IN_STAGING_SAMPLES
#define BSP_AUDIO_IN_STAGING_SAMPLES    1024U
#endif

#define BSP_AUDIO_OUT_INSTANCES_NBR     1U
#define BSP_AUDIO_IN_INSTANCES_NBR      1U

/** @brief SampleRate for the native rate: 48,828 Hz playback, 32,552 Hz capture. */
#define BSP_AUDIO_FREQUENCY_NATIVE      0U

/** @brief The only sample resolution either path supports. */
#define BSP_AUDIO_RESOLUTION_16B        16U

/** @brief The only output: the class-D amplifier. */
#define BSP_AUDIO_OUT_DEVICE_SPEAKER    1U

/** @brief The only input: the PDM microphone. */
#define BSP_AUDIO_IN_DEVICE_DIGITAL_MIC 1U

/** @brief BSP_AUDIO_OUT_IsMute results. */
#define BSP_AUDIO_MUTE_DISABLED         0U
#define BSP_AUDIO_MUTE_ENABLED          1U

/** @brief Playback states (BSP_AUDIO_OUT_GetState). */
#define BSP_AUDIO_OUT_STATE_RESET       0U
#define BSP_AUDIO_OUT_STATE_PLAYING     1U
#define BSP_AUDIO_OUT_STATE_STOP        2U
#define BSP_AUDIO_OUT_STATE_PAUSE       3U

/** @brief Capture states (BSP_AUDIO_IN_GetState). */
#define BSP_AUDIO_IN_STATE_RESET        0U
#define BSP_AUDIO_IN_STATE_RECORDING    1U
#define BSP_AUDIO_IN_STATE_STOP         2U
#define BSP_AUDIO_IN_STATE_PAUSE        3U

/** @brief IRQn of GPDMA1 channel n, a plain decimal literal. */
#define BSP_AUDIO_DMA_IRQN_(n)          GPDMA1_Channel##n##_IRQn
#define BSP_AUDIO_DMA_IRQN(n)           BSP_AUDIO_DMA_IRQN_(n)

/** @brief Playback DMA IRQ, enabled at init; the application provides its handler. */
#define BSP_AUDIO_OUT_DMA_IRQn          BSP_AUDIO_DMA_IRQN(BSP_AUDIO_OUT_DMA_CHANNEL)

/** @brief Capture DMA IRQ, enabled at init; the application provides its handler. */
#define BSP_AUDIO_IN_DMA_IRQn           BSP_AUDIO_DMA_IRQN(BSP_AUDIO_IN_DMA_CHANNEL)

/** @brief Init parameters, shared by both directions. */
typedef struct
{
  /** BSP_AUDIO_OUT_DEVICE_SPEAKER or BSP_AUDIO_IN_DEVICE_DIGITAL_MIC. */
  uint32_t Device;

  /** Hz, or BSP_AUDIO_FREQUENCY_NATIVE. It must be reachable within 1 %: 8-96 kHz out,
    * about 9.4-50.8 kHz in. */
  uint32_t SampleRate;

  /** BSP_AUDIO_RESOLUTION_16B only. */
  uint32_t BitsPerSample;

  /** 1 or 2 out (a mono sample plays in both I2S slots), 1 in. */
  uint32_t ChannelsNbr;

  /** Ignored; present for ST BSP compatibility. */
  uint32_t Volume;
} BSP_AUDIO_Init_t;

/**
  * @brief  Bring up SAI1 Block B, its DMA and the amplifier pin.
  * @retval BSP_ERROR_FEATURE_NOT_SUPPORTED for another device or an unreachable rate.
  * @note   The amplifier stays shut down until BSP_AUDIO_OUT_Play: no turn-on pop.
  */
int32_t BSP_AUDIO_OUT_Init(uint32_t Instance, BSP_AUDIO_Init_t *AudioInit);

/** @brief Mute, then release SAI1 Block B and its DMA. Safe from any state. */
int32_t BSP_AUDIO_OUT_DeInit(uint32_t Instance);

/**
  * @brief  Play pData in a loop until BSP_AUDIO_OUT_Stop.
  * @param  pData      16-bit PCM, L/R interleaved for stereo, 4-byte aligned.
  * @param  NbrOfBytes Non-zero multiple of 8, at most 65528 (one GPDMA block).
  */
int32_t BSP_AUDIO_OUT_Play(uint32_t Instance, uint8_t *pData, uint32_t NbrOfBytes);

/** @brief Hold off DMA and shut the amplifier down; the bit clock keeps running. */
int32_t BSP_AUDIO_OUT_Pause(uint32_t Instance);

/** @brief Resume a paused stream where the DMA left off. */
int32_t BSP_AUDIO_OUT_Resume(uint32_t Instance);

/** @brief Stop; the amplifier stays shut down until the next Play, unless muted. */
int32_t BSP_AUDIO_OUT_Stop(uint32_t Instance);

/** @brief Hardware mute: SD_MODE low shuts the output stage down, so no hiss and no
  * idle current. */
int32_t BSP_AUDIO_OUT_Mute(uint32_t Instance);

/** @brief Unmute: SD_MODE goes high now if playing, else at the next Play or Resume. */
int32_t BSP_AUDIO_OUT_UnMute(uint32_t Instance);

/** @brief The requested mute state, not the pin: Pause and Stop do not change it. */
int32_t BSP_AUDIO_OUT_IsMute(uint32_t Instance, uint32_t *IsMute);

/** @brief Returns BSP_ERROR_FEATURE_NOT_SUPPORTED: the gain is set by the GAIN pin. */
int32_t BSP_AUDIO_OUT_SetVolume(uint32_t Instance, uint32_t Volume);

/** @brief Returns BSP_ERROR_FEATURE_NOT_SUPPORTED. */
int32_t BSP_AUDIO_OUT_GetVolume(uint32_t Instance, uint32_t *Volume);

/**
  * @brief  Reprogram the playback rate (Hz or BSP_AUDIO_FREQUENCY_NATIVE) while stopped.
  * @retval BSP_ERROR_FEATURE_NOT_SUPPORTED outside 8-96 kHz or not within 1 %; the
  *         previous rate is kept. Read the result with BSP_AUDIO_OUT_GetSampleRate.
  */
int32_t BSP_AUDIO_OUT_SetSampleRate(uint32_t Instance, uint32_t SampleRate);

/** @brief The achieved playback rate in Hz, from the kernel clock read back. */
int32_t BSP_AUDIO_OUT_GetSampleRate(uint32_t Instance, uint32_t *SampleRate);

/** @brief One of BSP_AUDIO_OUT_STATE_*. */
int32_t BSP_AUDIO_OUT_GetState(uint32_t Instance, uint32_t *State);

/** @brief Speaker only; any other device gives BSP_ERROR_FEATURE_NOT_SUPPORTED. */
int32_t BSP_AUDIO_OUT_SetDevice(uint32_t Instance, uint32_t Device);

/** @brief Always BSP_AUDIO_OUT_DEVICE_SPEAKER. */
int32_t BSP_AUDIO_OUT_GetDevice(uint32_t Instance, uint32_t *Device);

/** @brief 16 bits only: the slot width sets the bit clock. */
int32_t BSP_AUDIO_OUT_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample);

/** @brief Always BSP_AUDIO_RESOLUTION_16B. */
int32_t BSP_AUDIO_OUT_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample);

/**
  * @brief  Set the buffer's channels, 1 or 2, while stopped; the wire always carries two
  *         slots, with a mono sample in both.
  * @retval BSP_ERROR_PERIPH_FAILURE if the SAI refuses; the previous setting is kept.
  */
int32_t BSP_AUDIO_OUT_SetChannelsNbr(uint32_t Instance, uint32_t ChannelsNbr);

/** @brief Channels in the caller's buffer: 1 or 2. */
int32_t BSP_AUDIO_OUT_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelsNbr);

/** @brief Clean the region just refilled from the D-cache; call after each refill. */
int32_t BSP_AUDIO_OUT_CleanBuffer(uint32_t Instance, const void *pData, uint32_t NbrOfBytes);

/** @brief Weak: the second half has been consumed; refill and clean it. */
void BSP_AUDIO_OUT_TransferComplete_CallBack(uint32_t Instance);

/** @brief Weak: the first half has been consumed; refill and clean it. */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Instance);

/** @brief Weak: playback DMA transfer error. SAI underruns are not reported. */
void BSP_AUDIO_OUT_Error_CallBack(uint32_t Instance);

/** @brief Call from the GPDMA1 handler for BSP_AUDIO_OUT_DMA_IRQn. */
void BSP_AUDIO_OUT_IRQHandler(uint32_t Instance);

/**
  * @brief  Bring up MDF1 filter 1, its DMA and the PDM clock (on PE2 from here on).
  * @retval BSP_ERROR_FEATURE_NOT_SUPPORTED for another device or a rate not reachable
  *         within 1 % with the microphone clock in its rating.
  */
int32_t BSP_AUDIO_IN_Init(uint32_t Instance, BSP_AUDIO_Init_t *AudioInit);

/** @brief Stop the PDM clock and release MDF1 and its DMA. */
int32_t BSP_AUDIO_IN_DeInit(uint32_t Instance);

/**
  * @brief  Capture into pData in a loop until BSP_AUDIO_IN_Stop.
  * @param  pData      16-bit mono PCM, 2-byte aligned. The CPU writes it, not the DMA.
  * @param  NbrOfBytes Non-zero multiple of 32, with NbrOfBytes / 2 at most
  *                    BSP_AUDIO_IN_STAGING_SAMPLES.
  */
int32_t BSP_AUDIO_IN_Record(uint32_t Instance, uint8_t *pData, uint32_t NbrOfBytes);

/** @brief Halt acquisition (the filter is stopped), keeping the buffer. */
int32_t BSP_AUDIO_IN_Pause(uint32_t Instance);

/** @brief Restart acquisition at the start of the buffer, not where Pause left off. */
int32_t BSP_AUDIO_IN_Resume(uint32_t Instance);

/** @brief Stop and forget the buffer. Recovers from a DMA error without a DeInit. */
int32_t BSP_AUDIO_IN_Stop(uint32_t Instance);

/** @brief Returns BSP_ERROR_FEATURE_NOT_SUPPORTED: the MDF gain is fixed. */
int32_t BSP_AUDIO_IN_SetVolume(uint32_t Instance, uint32_t Volume);

/** @brief Returns BSP_ERROR_FEATURE_NOT_SUPPORTED. */
int32_t BSP_AUDIO_IN_GetVolume(uint32_t Instance, uint32_t *Volume);

/**
  * @brief  Reprogram the capture rate (Hz or BSP_AUDIO_FREQUENCY_NATIVE) while stopped.
  *         The MDF is re-initialised and its DMA relinked; the level does not change.
  * @retval BSP_ERROR_FEATURE_NOT_SUPPORTED outside about 9.4-50.8 kHz or not within 1 %;
  *         the previous rate is restored.
  */
int32_t BSP_AUDIO_IN_SetSampleRate(uint32_t Instance, uint32_t SampleRate);

/** @brief The achieved capture rate in Hz, from the kernel clock read back. */
int32_t BSP_AUDIO_IN_GetSampleRate(uint32_t Instance, uint32_t *SampleRate);

/** @brief One of BSP_AUDIO_IN_STATE_*. */
int32_t BSP_AUDIO_IN_GetState(uint32_t Instance, uint32_t *State);

/** @brief Microphone only; any other device gives BSP_ERROR_FEATURE_NOT_SUPPORTED. */
int32_t BSP_AUDIO_IN_SetDevice(uint32_t Instance, uint32_t Device);

/** @brief Always BSP_AUDIO_IN_DEVICE_DIGITAL_MIC. */
int32_t BSP_AUDIO_IN_GetDevice(uint32_t Instance, uint32_t *Device);

/** @brief 16 bits only: the width is built into the PCM conversion. */
int32_t BSP_AUDIO_IN_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample);

/** @brief Always BSP_AUDIO_RESOLUTION_16B. */
int32_t BSP_AUDIO_IN_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample);

/** @brief Only 1; others give BSP_ERROR_FEATURE_NOT_SUPPORTED, where BSP_AUDIO_IN_Init
  * gives BSP_ERROR_WRONG_PARAM. */
int32_t BSP_AUDIO_IN_SetChannelsNbr(uint32_t Instance, uint32_t ChannelsNbr);

/** @brief Always 1. */
int32_t BSP_AUDIO_IN_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelsNbr);

/** @brief Weak: the second half of the buffer holds new samples. Runs in the DMA
  * interrupt, at BSP_AUDIO_IN_IT_PRIORITY. */
void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance);

/** @brief Weak: the first half holds new samples (same context as the second). */
void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance);

/** @brief Weak: capture DMA transfer error (MDF overruns are not reported). Recover with
  * BSP_AUDIO_IN_Stop. */
void BSP_AUDIO_IN_Error_CallBack(uint32_t Instance);

/** @brief Call from the GPDMA1 handler for BSP_AUDIO_IN_DMA_IRQn. */
void BSP_AUDIO_IN_IRQHandler(uint32_t Instance);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_AUDIO_H */
