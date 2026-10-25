/**
  * @file    neuron6_bus.h
  * @brief   I2C and SPI bus services for the Neuro N6 board.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#ifndef NEURON6_BUS_H
#define NEURON6_BUS_H

#include "neuron6_conf.h"
#include "neuron6_errno.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Two I2C buses and one SPI bus; this module owns their pins, clocks and timing. */

/* Bus 1: on-board sensor bus */

#define BUS_I2C1                          I2C1
#define BUS_I2C1_CLK_ENABLE()             __HAL_RCC_I2C1_CLK_ENABLE()
#define BUS_I2C1_CLK_DISABLE()            __HAL_RCC_I2C1_CLK_DISABLE()
#define BUS_I2C1_FORCE_RESET()            __HAL_RCC_I2C1_FORCE_RESET()
#define BUS_I2C1_RELEASE_RESET()          __HAL_RCC_I2C1_RELEASE_RESET()

#define BUS_I2C1_SCL_PIN                  GPIO_PIN_9
#define BUS_I2C1_SCL_GPIO_PORT            GPIOH
#define BUS_I2C1_SCL_AF                   GPIO_AF4_I2C1
#define BUS_I2C1_SCL_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOH_CLK_ENABLE()

#define BUS_I2C1_SDA_PIN                  GPIO_PIN_6
#define BUS_I2C1_SDA_GPIO_PORT            GPIOE
#define BUS_I2C1_SDA_AF                   GPIO_AF4_I2C1
#define BUS_I2C1_SDA_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOE_CLK_ENABLE()

/** @brief Fixed bus 1 TIMINGR word, for its PCLK1 kernel clock. */
#if defined(BUS_I2C1_TIMING_OVERRIDE)
#define BUS_I2C1_TIMING                   BUS_I2C1_TIMING_OVERRIDE
#else
#define BUS_I2C1_TIMING                   0x10707CBBU
#endif

/* Bus 2: expansion-header I2C bus */

#define BUS_I2C2                          I2C2
#define BUS_I2C2_CLK_ENABLE()             __HAL_RCC_I2C2_CLK_ENABLE()
#define BUS_I2C2_CLK_DISABLE()            __HAL_RCC_I2C2_CLK_DISABLE()
#define BUS_I2C2_FORCE_RESET()            __HAL_RCC_I2C2_FORCE_RESET()
#define BUS_I2C2_RELEASE_RESET()          __HAL_RCC_I2C2_RELEASE_RESET()

/* Header pins SCL (PD14) and SDA (PD15). */
#define BUS_I2C2_SCL_PIN                  GPIO_PIN_14
#define BUS_I2C2_SCL_GPIO_PORT            GPIOD
#define BUS_I2C2_SCL_AF                   GPIO_AF4_I2C2
#define BUS_I2C2_SCL_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOD_CLK_ENABLE()

#define BUS_I2C2_SDA_PIN                  GPIO_PIN_15
#define BUS_I2C2_SDA_GPIO_PORT            GPIOD
#define BUS_I2C2_SDA_AF                   GPIO_AF4_I2C2
#define BUS_I2C2_SDA_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOD_CLK_ENABLE()

/** @brief Bus 2 SCL rate in Hz; Init computes the timing from the live PCLK1. */
#ifndef BUS_I2C2_FREQUENCY
#define BUS_I2C2_FREQUENCY                100000U
#endif

/* Bus 3: expansion-header SPI bus */

#define BUS_SPI2                          SPI2
#define BUS_SPI2_CLK_ENABLE()             __HAL_RCC_SPI2_CLK_ENABLE()
#define BUS_SPI2_CLK_DISABLE()            __HAL_RCC_SPI2_CLK_DISABLE()
#define BUS_SPI2_FORCE_RESET()            __HAL_RCC_SPI2_FORCE_RESET()
#define BUS_SPI2_RELEASE_RESET()          __HAL_RCC_SPI2_RELEASE_RESET()

/* Header pins SCK (PF2), MOSI (PD7) and MISO (PD11). */
#define BUS_SPI2_SCK_PIN                  GPIO_PIN_2
#define BUS_SPI2_SCK_GPIO_PORT            GPIOF
#define BUS_SPI2_SCK_AF                   GPIO_AF5_SPI2
#define BUS_SPI2_SCK_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOF_CLK_ENABLE()

#define BUS_SPI2_MOSI_PIN                 GPIO_PIN_7
#define BUS_SPI2_MOSI_GPIO_PORT           GPIOD
#define BUS_SPI2_MOSI_AF                  GPIO_AF5_SPI2
#define BUS_SPI2_MOSI_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOD_CLK_ENABLE()

#define BUS_SPI2_MISO_PIN                 GPIO_PIN_11
#define BUS_SPI2_MISO_GPIO_PORT           GPIOD
#define BUS_SPI2_MISO_AF                  GPIO_AF5_SPI2
#define BUS_SPI2_MISO_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOD_CLK_ENABLE()

/** @brief Default bus 3 SCK in Hz; it runs at the fastest PCLK1 / 2^n at or below it. */
#ifndef BUS_SPI2_FREQUENCY
#define BUS_SPI2_FREQUENCY                4000000U
#endif

/** @brief SPI modes: bit 1 is the clock polarity, bit 0 the phase. */
#define BUS_SPI_MODE_0                    0U /*!< Idle low,  sample leading.  */
#define BUS_SPI_MODE_1                    1U /*!< Idle low,  sample trailing. */
#define BUS_SPI_MODE_2                    2U /*!< Idle high, sample leading.  */
#define BUS_SPI_MODE_3                    3U /*!< Idle high, sample trailing. */

#define BUS_SPI_MSB_FIRST                 0U
#define BUS_SPI_LSB_FIRST                 1U

/** @brief Bus 3 settings, for BSP_SPI2_SetConfig. */
typedef struct
{
  uint32_t ClockFreq; /*!< Requested SCK frequency in Hz.        */
  uint32_t Mode;      /*!< One of the BUS_SPI_MODE_* values.     */
  uint32_t BitOrder;  /*!< One of the BUS_SPI_*_FIRST values.    */
} BSP_SPI_Config_t;

#if defined(USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)
/** @brief Replacement MSP routines; the HAL calls them inside HAL_I2C_Init/DeInit. */
typedef struct
{
  void (* pMspInitCb)(I2C_HandleTypeDef *);   /*!< Pins and clocks up.   */
  void (* pMspDeInitCb)(I2C_HandleTypeDef *); /*!< Pins and clocks down. */
} BSP_I2C_Cb_t;
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
/** @brief As BSP_I2C_Cb_t, for bus 3. */
typedef struct
{
  void (* pMspInitCb)(SPI_HandleTypeDef *);   /*!< Pins and clocks up.   */
  void (* pMspDeInitCb)(SPI_HandleTypeDef *); /*!< Pins and clocks down. */
} BSP_SPI_Cb_t;
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */

/** @brief Bus 1 HAL handle, valid from the first BSP_I2C1_Init() to the last DeInit. */
extern I2C_HandleTypeDef hbus_i2c1;

/** @brief HAL handle for bus 2, on the same terms as hbus_i2c1. */
extern I2C_HandleTypeDef hbus_i2c2;

/** @brief HAL handle for bus 3, on the same terms as hbus_i2c1. */
extern SPI_HandleTypeDef hbus_spi2;

/**
  * @brief  Bring up bus 1, or count one more user (reference counted).
  * @note   A device holding SDA low (e.g. after a reset mid-read) is clocked free first;
  *         a transfer that times out or finds the bus busy restarts the bus the same way.
  * @retval BSP_ERROR_BUS_FAILURE if the peripheral refused its configuration,
  *         BSP_ERROR_MSP_FAILURE if the pins or clocks could not be set up.
  */
int32_t BSP_I2C1_Init(void);

/** @brief Release one user of bus 1; the last one tears it down (BSP_ERROR_NO_INIT if
  * there are none). */
int32_t BSP_I2C1_DeInit(void);

/* Bus 1 transfers. DevAddr is left aligned in 8 bits. Errors: BSP_ERROR_NO_INIT before
 * Init, BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE on a NACK, BSP_ERROR_BUSY if the peripheral was
 * busy, or BSP_ERROR_PERIPH_FAILURE; a held bus is restarted, so a retry can succeed. */

/** @brief Write a register, 8-bit register address. */
int32_t BSP_I2C1_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);

/** @brief Read a register, 8-bit register address. */
int32_t BSP_I2C1_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);

/** @brief Write a register, 16-bit register address. */
int32_t BSP_I2C1_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);

/** @brief Read a register, 16-bit register address. */
int32_t BSP_I2C1_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);

/** @brief Send raw bytes, no register address. */
int32_t BSP_I2C1_Send(uint16_t DevAddr, uint8_t *pData, uint16_t Length);

/** @brief Receive raw bytes, no register address. */
int32_t BSP_I2C1_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t Length);

/** @brief Probe DevAddr (Trials attempts); BSP_ERROR_BUSY if there is no answer. */
int32_t BSP_I2C1_IsReady(uint16_t DevAddr, uint32_t Trials);

/** @brief Weak no-op lock for bus 1, shared by the image sensor, thermal camera CCI and
  * ranger: override this pair when more than one task drives the bus. */
void BSP_I2C1_Lock(void);
void BSP_I2C1_Unlock(void);

/** @brief Weak no-op: override to keep task switches out of one transaction. A switch
  * stretches SCL, and the board's magnetometer answers with a corrupted register. */
void BSP_I2C_TransactionBegin(void);
void BSP_I2C_TransactionEnd(void);

#if defined(USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)
/** @brief Restore this module's MSP routines on bus 1. */
int32_t BSP_I2C1_RegisterDefaultMspCallbacks(void);

/** @brief Replace the bus 1 MSP routines (neither may be NULL); register before the first
  * BSP_I2C1_Init(), which runs them. */
int32_t BSP_I2C1_RegisterMspCallbacks(BSP_I2C_Cb_t *Callbacks);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

/**
  * @brief  Bring up bus 2, or count one more user, on the same terms as BSP_I2C1_Init().
  * @retval BSP_ERROR_WRONG_PARAM if BUS_I2C2_FREQUENCY cannot be reached from PCLK1.
  */
int32_t BSP_I2C2_Init(void);

/** @brief As BSP_I2C1_DeInit(), for bus 2. */
int32_t BSP_I2C2_DeInit(void);

/* Bus 2 transfers and probe: as bus 1. */
int32_t BSP_I2C2_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C2_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C2_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C2_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C2_Send(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C2_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C2_IsReady(uint16_t DevAddr, uint32_t Trials);

/** @brief Weak no-op lock for bus 2; override it as BSP_I2C1_Lock(). */
void BSP_I2C2_Lock(void);
void BSP_I2C2_Unlock(void);

#if defined(USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)
/** @brief Restore this module's MSP routines on bus 2. */
int32_t BSP_I2C2_RegisterDefaultMspCallbacks(void);

/** @brief As BSP_I2C1_RegisterMspCallbacks(), for bus 2. */
int32_t BSP_I2C2_RegisterMspCallbacks(BSP_I2C_Cb_t *Callbacks);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

/**
  * @brief  Bring up bus 3, or count one more user, on the same terms as BSP_I2C1_Init().
  * @note   No chip select: drive a header pin from neuron6_io around each transaction.
  */
int32_t BSP_SPI2_Init(void);

/** @brief As BSP_I2C1_DeInit(), for bus 3. */
int32_t BSP_SPI2_DeInit(void);

/**
  * @brief  Set bus 3's rate, mode and bit order; reprograms only if a field changed.
  * @note   Rate: the fastest PCLK1 / 2..256 (powers of two) at or below the request;
  *         the slowest is 781,250 Hz at the default 200 MHz PCLK1.
  * @retval BSP_ERROR_WRONG_PARAM for NULL, an unknown mode or order, or a slower rate.
  */
int32_t BSP_SPI2_SetConfig(const BSP_SPI_Config_t *pConfig);

/** @brief Settings in force; ClockFreq is the request, not the achieved rate. */
int32_t BSP_SPI2_GetConfig(BSP_SPI_Config_t *pConfig);

/** @brief Achieved bus 3 clock in Hz, or 0 if the bus is not initialised. */
uint32_t BSP_SPI2_GetClockFreq(void);

/**
  * @brief  Send bytes on bus 3, discarding what arrives.
  * @retval BSP_ERROR_BUSY on a timeout; BSP_ERROR_BUS_TRANSACTION_FAILURE if it failed.
  */
int32_t BSP_SPI2_Send(uint8_t *pData, uint16_t Length);

/** @brief Receive bytes on bus 3, sending zeroes; errors as BSP_SPI2_Send. */
int32_t BSP_SPI2_Recv(uint8_t *pData, uint16_t Length);

/** @brief Full-duplex exchange; pTxData and pRxData may be the same buffer. */
int32_t BSP_SPI2_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t Length);

/** @brief Weak no-op lock for bus 3; override it as BSP_I2C1_Lock(). */
void BSP_SPI2_Lock(void);
void BSP_SPI2_Unlock(void);

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
/** @brief Restore this module's MSP routines on bus 3. */
int32_t BSP_SPI2_RegisterDefaultMspCallbacks(void);

/** @brief As BSP_I2C1_RegisterMspCallbacks(), for bus 3. */
int32_t BSP_SPI2_RegisterMspCallbacks(BSP_SPI_Cb_t *Callbacks);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_BUS_H */
