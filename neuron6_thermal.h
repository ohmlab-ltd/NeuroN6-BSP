/**
  * @file    neuron6_thermal.h
  * @brief   FLIR Lepton 3.1R thermal camera: command interface on I2C1, video on SPI1.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_THERMAL_H
#define NEURON6_THERMAL_H

#include "neuron6_conf.h"
#include "neuron6_errno.h"
#include "lepton.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The Lepton 3.1R on the thermal carrier: 160x120 radiometric pixels (temperatures
 * with AGC off). This module owns both its interfaces and its three control pins. */

#define BSP_THERMAL_INSTANCES_NBR       1U
#define BSP_THERMAL_WIDTH               LEPTON_WIDTH
#define BSP_THERMAL_HEIGHT              LEPTON_HEIGHT
#define BSP_THERMAL_FRAME_PIXELS        LEPTON_FRAME_PIXELS
#define BSP_THERMAL_FRAME_BYTES         (LEPTON_FRAME_PIXELS * 2U)
#define BSP_THERMAL_PACKET_BYTES        LEPTON_PACKET_BYTES

/** @brief Command-bus address, 8-bit form. */
#define BSP_THERMAL_I2C_ADDRESS         LEPTON_I2C_ADDRESS

/* Control pins */

/** @brief Power-down line, active low: PD3. */
#define BSP_THERMAL_PWRDWN_PIN          GPIO_PIN_3
#define BSP_THERMAL_PWRDWN_GPIO_PORT    GPIOD
#define BSP_THERMAL_PWRDWN_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()

/** @brief Reset line, active low: PD9. */
#define BSP_THERMAL_RESET_PIN           GPIO_PIN_9
#define BSP_THERMAL_RESET_GPIO_PORT     GPIOD
#define BSP_THERMAL_RESET_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()

/** @brief Video chip select, active low: PF4. Driven as a plain output. */
#define BSP_THERMAL_CS_PIN              GPIO_PIN_4
#define BSP_THERMAL_CS_GPIO_PORT        GPIOF
#define BSP_THERMAL_CS_CLK_ENABLE()     __HAL_RCC_GPIOF_CLK_ENABLE()

/* Video link */

#define BSP_THERMAL_SPI                 SPI1
#define BSP_THERMAL_SPI_CLK_ENABLE()    __HAL_RCC_SPI1_CLK_ENABLE()
#define BSP_THERMAL_SPI_CLK_DISABLE()   __HAL_RCC_SPI1_CLK_DISABLE()
#define BSP_THERMAL_SPI_IRQn            SPI1_IRQn

#define BSP_THERMAL_SCK_PIN             GPIO_PIN_5
#define BSP_THERMAL_SCK_GPIO_PORT       GPIOA
#define BSP_THERMAL_SCK_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()

#define BSP_THERMAL_MISO_PIN            GPIO_PIN_4
#define BSP_THERMAL_MISO_GPIO_PORT      GPIOB
#define BSP_THERMAL_MISO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()

/** @brief Not read by the camera; configured because the carrier wires it. */
#define BSP_THERMAL_MOSI_PIN            GPIO_PIN_7
#define BSP_THERMAL_MOSI_GPIO_PORT      GPIOA
#define BSP_THERMAL_MOSI_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()

#define BSP_THERMAL_SPI_AF              GPIO_AF5_SPI1

/** @brief Video clock limit in Hz; the link uses the fastest division not above it. */
#define BSP_THERMAL_SCK_MAX_HZ          20000000U

/** @brief Video GPDMA1 channel: a plain decimal literal, pasted into symbol names. */
#ifndef BSP_THERMAL_DMA_CHANNEL
#define BSP_THERMAL_DMA_CHANNEL         1
#endif

/** @cond */
#define BSP_THERMAL_DMA_CH_(n)          GPDMA1_Channel##n
#define BSP_THERMAL_DMA_IRQN_(n)        GPDMA1_Channel##n##_IRQn
/** @endcond */

#define BSP_THERMAL_DMA_CH(n)           BSP_THERMAL_DMA_CH_(n)
#define BSP_THERMAL_DMA_IRQN(n)         BSP_THERMAL_DMA_IRQN_(n)

#define BSP_THERMAL_DMA_IRQn            BSP_THERMAL_DMA_IRQN(BSP_THERMAL_DMA_CHANNEL)

/** @brief Preemption priority of the video interrupts, as for the camera and audio. */
#ifndef BSP_THERMAL_IT_PRIORITY
#define BSP_THERMAL_IT_PRIORITY         5U
#endif

/** @brief Section for the video DMA descriptor, which must be mapped non-cacheable.
  * Most examples' MPU maps .axisram_nc; the four FreeRTOS radio examples map the
  * .noncacheable output instead, whose input section is noncacheable_buffer. */
#ifndef BSP_THERMAL_DMA_NODE_SECTION
#define BSP_THERMAL_DMA_NODE_SECTION    ".axisram_nc"
#endif

/** @brief Idle time (select high, no clock) that makes the camera restart its framer,
  * ms. FLIR gives more than 185 ms, which is not always enough. */
#ifndef BSP_THERMAL_RESYNC_IDLE_MS
#define BSP_THERMAL_RESYNC_IDLE_MS      300U
#endif

/** @brief Consecutive halves with under half their packets well formed before capture
  * restarts: after boot and the first FFC the stream can fall out of step. */
#ifndef BSP_THERMAL_RESYNC_BAD_HALVES
#define BSP_THERMAL_RESYNC_BAD_HALVES   4U
#endif

/** @brief Stall timeout in ms; longer than the ~1 s an FFC pauses the video. */
#ifndef BSP_THERMAL_STALL_TIMEOUT_MS
#define BSP_THERMAL_STALL_TIMEOUT_MS    1500U
#endif

/** @brief 1: BSP_THERMAL_GetFrame() resynchronises a stalled stream and carries on;
  * 0: it returns BSP_ERROR_PERIPH_FAILURE. */
#ifndef BSP_THERMAL_AUTO_RESYNC
#define BSP_THERMAL_AUTO_RESYNC         1U
#endif

/** @brief Power-down time in a power cycle, ms. */
#ifndef BSP_THERMAL_POWER_CYCLE_MS
#define BSP_THERMAL_POWER_CYCLE_MS      100U
#endif

/** @brief Reset held low after power returns, ms. */
#ifndef BSP_THERMAL_RESET_PULSE_MS
#define BSP_THERMAL_RESET_PULSE_MS      30U
#endif

/** @brief Quiet time after the reset release before the camera is first asked, ms:
  * repeated sensor-bus traffic (for any device) during the boot stalls it until a
  * power cycle. */
#ifndef BSP_THERMAL_BOOT_QUIET_MS
#define BSP_THERMAL_BOOT_QUIET_MS       2000U
#endif

/** @brief How long BSP_THERMAL_Init() waits for the boot after the reset release, ms,
  * BSP_THERMAL_BOOT_QUIET_MS included. The boot takes about 950 ms. */
#ifndef BSP_THERMAL_BOOT_TIMEOUT_MS
#define BSP_THERMAL_BOOT_TIMEOUT_MS     3000U
#endif

/** @brief Video stream health (BSP_THERMAL_GetHealth()). Where the counts stop rising
  * names the fault: no new halves means the link or its DMA is dead. */
typedef struct
{
  uint32_t HalvesProduced; /*!< Buffer halves the DMA has filled.            */
  uint32_t HalvesConsumed; /*!< Buffer halves the reader has parsed.         */
  uint32_t PacketsSeen;    /*!< Well-formed packets carrying pixels.         */
  uint32_t FillersSeen;    /*!< Well-formed packets carrying nothing.        */
  uint32_t SegmentsParsed; /*!< Complete segments assembled.                 */
  uint32_t FramesParsed;   /*!< Complete frames assembled.                   */
  uint32_t Resyncs;        /*!< Times the packet sequence was lost.          */
  uint32_t Restarts;       /*!< Times capture was restarted from scratch.    */
  uint32_t Overruns;       /*!< Times the DMA lapped the reader, each a lost frame:
                             *   the buffer is too small or reads too rare. */
  uint32_t SpiErrors;      /*!< Errors the video peripheral reported.        */
  uint32_t LastSpiError;   /*!< Most recent peripheral error code.           */
  uint32_t LastHeader;     /*!< First four bytes of the last block parsed. */
  uint32_t SckHz;          /*!< Clock frequency the video link is running at. */
  uint32_t KernelHz;       /*!< Kernel clock the divider was derived from.   */
} BSP_THERMAL_Health_t;

/**
  * @brief  Power the camera up (unless BSP_THERMAL_PowerOn() already did), bring up
  *         the sensor bus and wait for the boot.
  * @note   Asks nothing until BSP_THERMAL_BOOT_QUIET_MS after the reset release, then
  *         asks until BSP_THERMAL_BOOT_TIMEOUT_MS (so, called late, it asks once).
  *         Nothing else may use the sensor bus from the power-up until it returns.
  * @retval BSP_ERROR_BUS_FAILURE if the bus could not be brought up,
  *         BSP_ERROR_COMPONENT_FAILURE if the camera never reported a boot.
  */
int32_t BSP_THERMAL_Init(uint32_t Instance);

/** @brief Stop video, power the camera down and release its share of the sensor bus. */
int32_t BSP_THERMAL_DeInit(uint32_t Instance);

/**
  * @brief  Power-cycle the camera and release its reset; may precede BSP_THERMAL_Init().
  * @note   Returns once the pins are driven; the boot takes about a second more, which
  *         BSP_THERMAL_Init() waits out. In a video session the next
  *         BSP_THERMAL_GetFrame() restarts capture.
  * @note   Keep the sensor bus quiet (all devices) for BSP_THERMAL_BOOT_QUIET_MS
  *         afterwards, or the camera does not finish booting.
  */
int32_t BSP_THERMAL_PowerOn(uint32_t Instance);

/** @brief Hold the camera powered down. After BSP_THERMAL_Init() this is
  * BSP_THERMAL_DeInit(); before it, only the pins are driven. */
int32_t BSP_THERMAL_PowerOff(uint32_t Instance);

/** @brief Diagnostic: read back the power-down and reset pins, 1 = high (released);
  * an isolation policy or an external pull can silently override a write. */
int32_t BSP_THERMAL_GetControlPins(uint32_t Instance, uint8_t *pPowerHigh, uint8_t *pResetHigh);

/* Command interface: BSP_ERROR_BUSY during a video session, BSP_ERROR_COMPONENT_FAILURE
 * if the camera fails or refuses (BSP_THERMAL_GetLastResult() gives its code). */

/** @brief Probe the camera's address; BSP_ERROR_UNKNOWN_COMPONENT if nothing answers. */
int32_t BSP_THERMAL_IsPresent(uint32_t Instance);

/** @brief Ask the camera whether it is alive. */
int32_t BSP_THERMAL_Ping(uint32_t Instance);

/** @brief Serial number, four words. */
int32_t BSP_THERMAL_GetSerialNumber(uint32_t Instance, uint16_t *pSerial);

/** @brief Camera uptime in ms. */
int32_t BSP_THERMAL_GetUptime(uint32_t Instance, uint32_t *pUptime_ms);

/** @brief Detector (FPA) temperature in 0.01 K. */
int32_t BSP_THERMAL_GetFpaTemperature(uint32_t Instance, uint16_t *pKelvin100);

/** @brief Housing temperature in 0.01 K. */
int32_t BSP_THERMAL_GetAuxTemperature(uint32_t Instance, uint16_t *pKelvin100);

/** @brief AGC state, 1 or 0; with AGC on, pixels are display values, not temperatures. */
int32_t BSP_THERMAL_GetAgcEnabled(uint32_t Instance, uint8_t *pEnabled);

int32_t BSP_THERMAL_SetAgcEnabled(uint32_t Instance, uint8_t Enabled);

/** @brief Flat-field correction: the camera closes its shutter and calibrates on it. */
int32_t BSP_THERMAL_RunFfc(uint32_t Instance);

/** @brief Read or write one command-interface register. */
int32_t BSP_THERMAL_ReadRegister(uint32_t Instance, uint16_t Reg, uint16_t *pValue);
int32_t BSP_THERMAL_WriteRegister(uint32_t Instance, uint16_t Reg, uint16_t Value);

/** @brief Get any attribute by command word (lepton.h or FLIR's), up to 16 words. */
int32_t BSP_THERMAL_GetAttribute(uint32_t Instance, uint16_t Command,
                                 uint16_t *pWords, uint16_t LengthWords);

/** @brief Set any attribute by command word, up to 16 words. */
int32_t BSP_THERMAL_SetAttribute(uint32_t Instance, uint16_t Command,
                                 const uint16_t *pWords, uint16_t LengthWords);

/** @brief Run any procedure by command word. */
int32_t BSP_THERMAL_RunCommand(uint32_t Instance, uint16_t Command);

/** @brief The camera's own code for the last command: 0, or a small negative refusal. */
int32_t BSP_THERMAL_GetLastResult(uint32_t Instance);

/**
  * @brief  Open a video session into a caller-owned circular buffer.
  * @param  pBuffer 32-byte aligned; filled only by the DMA and invalidated before reads.
  * @param  Size    A multiple of 1312 (eight packets), at most 64288 (16-bit length).
  * @note   Commands are refused until BSP_THERMAL_VideoStop(). Capture starts in the
  *         first BSP_THERMAL_GetFrame(), after a BSP_THERMAL_RESYNC_IDLE_MS resync.
  * @retval BSP_ERROR_CLOCK_FAILURE if no usable clock, BSP_ERROR_PERIPH_FAILURE if
  *         the SPI or its DMA refused.
  */
int32_t BSP_THERMAL_VideoStart(uint32_t Instance, uint8_t *pBuffer, uint32_t Size);

/** @brief End the video session and release the link; commands work again. */
int32_t BSP_THERMAL_VideoStop(uint32_t Instance);

/** @brief Make the camera restart its framer: the link idles for
  * BSP_THERMAL_RESYNC_IDLE_MS, then capture restarts. */
int32_t BSP_THERMAL_VideoResync(uint32_t Instance);

/**
  * @brief  Wait up to Timeout_ms for the next complete frame (BSP_THERMAL_FRAME_PIXELS,
  *         row major), parsing what the DMA has delivered.
  * @param  pFrame Pass the same buffer on every call: frames are assembled in it, so
  *                use it only after BSP_ERROR_NONE.
  * @note   Pixels are 0.01 K with AGC off, display values with it on. What arrives
  *         between calls is overwritten, not counted as a fault.
  * @note   With BSP_THERMAL_AUTO_RESYNC, a stalled or out-of-step stream is restarted
  *         inside the call.
  * @retval BSP_ERROR_BUSY if the time ran out without a complete frame.
  */
int32_t BSP_THERMAL_GetFrame(uint32_t Instance, uint16_t *pFrame, uint32_t Timeout_ms);

/** @brief Rotate every frame 180 degrees (Enable 1), for a camera mounted upside down. */
int32_t BSP_THERMAL_SetFlip180(uint32_t Instance, uint8_t Enable);

int32_t BSP_THERMAL_GetHealth(uint32_t Instance, BSP_THERMAL_Health_t *pHealth);

/** @brief Call from the application's GPDMA1_ChannelN_IRQHandler (BSP_THERMAL_DMA_IRQn);
  * the BSP installs no vector. */
void BSP_THERMAL_DMA_IRQHandler(uint32_t Instance);

/** @brief Call from the application's SPI1_IRQHandler. */
void BSP_THERMAL_SPI_IRQHandler(uint32_t Instance);

/** @brief A buffer half was filled. Weak, empty; runs in interrupt context. */
void BSP_THERMAL_HalfCpltCallback(uint32_t Instance);

/** @brief The video link reported an error. Weak, empty; runs in interrupt context. */
void BSP_THERMAL_ErrorCallback(uint32_t Instance);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_THERMAL_H */
