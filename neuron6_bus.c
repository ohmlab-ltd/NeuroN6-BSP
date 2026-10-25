/**
  * @file    neuron6_bus.c
  * @brief   I2C and SPI bus services for the Neuro N6 board.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  *
  * The I2C timing computation is adapted from STMicroelectronics'
  * stm32n6570_discovery_bus.c (BSD-3-Clause).
  */

#include "neuron6_conf.h"
#include "neuron6_bus.h"

/* Every transfer this module issues is blocking and polled. */
#define BUS_I2C_TIMEOUT_MS                     1000U

/* Per transfer; generous enough that only a stalled peripheral reaches it. */
#define BUS_SPI_TIMEOUT_MS                     1000U

/* SPI baud-rate dividers: 2 to 256. */
#define BUS_SPI_PRESCALER_NBR                  8U

/* Speed bands recognised by the timing search, indexing I2C_Charac. */
#define I2C_SPEED_FREQ_STANDARD                0U    /* 100 kHz */
#define I2C_SPEED_FREQ_FAST                    1U    /* 400 kHz */
#define I2C_SPEED_FREQ_FAST_PLUS               2U    /* 1 MHz   */

/* Analog filter propagation delay, in ns, from the device datasheet. */
#define I2C_ANALOG_FILTER_DELAY_MIN            50U
#define I2C_ANALOG_FILTER_DELAY_MAX            260U

/* Digital filter is off: the analog filter alone suits both buses, and a
 * digital filter would eat into the SCL low period the search budgets for. */
#define I2C_DIGITAL_FILTER_COEF                0U

/* Field widths of the TIMINGR register. */
#define I2C_PRESC_MAX                          16U
#define I2C_SCLDEL_MAX                         16U
#define I2C_SDADEL_MAX                         16U
#define I2C_SCLH_MAX                           256U
#define I2C_SCLL_MAX                           256U

/* At most one candidate per prescaler value, so this bounds the candidate table. */
#define I2C_VALID_TIMING_NBR                   I2C_PRESC_MAX

#define SEC2NSEC                               1000000000UL

/* Characteristics of one I2C speed band; times in ns. */
typedef struct
{
  uint32_t freq;       /* Nominal frequency in Hz                  */
  uint32_t freq_min;   /* Lowest frequency accepted for this band  */
  uint32_t freq_max;   /* Highest frequency accepted for this band */
  uint32_t hddat_min;  /* Minimum data hold time                   */
  uint32_t vddat_max;  /* Maximum data valid time                  */
  uint32_t sudat_min;  /* Minimum data setup time                  */
  uint32_t lscl_min;   /* Minimum SCL low period                   */
  uint32_t hscl_min;   /* Minimum SCL high period                  */
  uint32_t trise;      /* Rise time                                */
  uint32_t tfall;      /* Fall time                                */
  uint32_t dnf;        /* Digital noise filter coefficient         */
} I2C_Charac_t;

/* One candidate set of TIMINGR fields. */
typedef struct
{
  uint32_t presc;      /* Timing prescaler */
  uint32_t tscldel;    /* SCL delay        */
  uint32_t tsdadel;    /* SDA delay        */
  uint32_t sclh;       /* SCL high period  */
  uint32_t scll;       /* SCL low period   */
} I2C_Timings_t;

static const I2C_Charac_t I2C_Charac[] =
{
  [I2C_SPEED_FREQ_STANDARD] =
  {
    .freq      = 100000,
    .freq_min  = 80000,
    .freq_max  = 120000,
    .hddat_min = 0,
    .vddat_max = 3450,
    .sudat_min = 250,
    .lscl_min  = 4700,
    .hscl_min  = 4000,
    .trise     = 640,
    .tfall     = 20,
    .dnf       = I2C_DIGITAL_FILTER_COEF,
  },
  [I2C_SPEED_FREQ_FAST] =
  {
    .freq      = 400000,
    .freq_min  = 320000,
    .freq_max  = 480000,
    .hddat_min = 0,
    .vddat_max = 900,
    .sudat_min = 100,
    .lscl_min  = 1300,
    .hscl_min  = 600,
    .trise     = 250,
    .tfall     = 100,
    .dnf       = I2C_DIGITAL_FILTER_COEF,
  },
  [I2C_SPEED_FREQ_FAST_PLUS] =
  {
    .freq      = 1000000,
    .freq_min  = 800000,
    .freq_max  = 1200000,
    .hddat_min = 0,
    .vddat_max = 450,
    .sudat_min = 50,
    .lscl_min  = 500,
    .hscl_min  = 260,
    .trise     = 60,
    .tfall     = 100,
    .dnf       = I2C_DIGITAL_FILTER_COEF,
  },
};

I2C_HandleTypeDef hbus_i2c1;
I2C_HandleTypeDef hbus_i2c2;
SPI_HandleTypeDef hbus_spi2;

/* Users of each bus; changed only under the bus lock hooks. */
static uint32_t s_i2c1_users;
static uint32_t s_i2c2_users;
static uint32_t s_spi2_users;

/* Candidate table for the timing search, refilled on every computation. */
static I2C_Timings_t s_valid_timing[I2C_VALID_TIMING_NBR];
static uint32_t      s_valid_timing_nbr;

/* Bus 3 settings in force, and the clock the power-of-two divider achieved. */
static BSP_SPI_Config_t s_spi2_config =
{
  .ClockFreq = BUS_SPI2_FREQUENCY,
  .Mode      = BUS_SPI_MODE_0,
  .BitOrder  = BUS_SPI_MSB_FIRST,
};
static uint32_t s_spi2_actual_hz;

static HAL_StatusTypeDef I2C1_MspInit(void);
static void              I2C1_MspDeInit(void);
static HAL_StatusTypeDef I2C2_MspInit(void);
static void              I2C2_MspDeInit(void);
static HAL_StatusTypeDef SPI2_MspInit(void);
static void              SPI2_MspDeInit(void);

/* Pins and clocks up and down, directly or through the HAL's registered callbacks. */
static int32_t           I2C1_MspUp(void);
static int32_t           I2C1_MspResult(void);
static void              I2C1_MspDown(void);
static int32_t           I2C2_MspUp(void);
static int32_t           I2C2_MspResult(void);
static void              I2C2_MspDown(void);
static int32_t           SPI2_MspUp(void);
static int32_t           SPI2_MspResult(void);
static void              SPI2_MspDown(void);

static HAL_StatusTypeDef SPI_ConfigurePeripheral(void);
static int32_t           SPI_MapError(HAL_StatusTypeDef status);

static void              I2C_RecoverBus(GPIO_TypeDef *pSclPort, uint16_t SclPin,
                                        GPIO_TypeDef *pSdaPort, uint16_t SdaPin);
static void              I2C_RecoveryDelay(void);
static int32_t           I2C1_After(int32_t Ret);
static int32_t           I2C2_After(int32_t Ret);
static HAL_StatusTypeDef I2C_ConfigurePeripheral(I2C_HandleTypeDef *hI2c, uint32_t timing);
static int32_t           I2C_MapError(const I2C_HandleTypeDef *hI2c, HAL_StatusTypeDef status);
static int32_t           I2C_MemWrite(I2C_HandleTypeDef *hI2c, uint16_t DevAddr, uint16_t Reg,
                                      uint16_t MemAddSize, uint8_t *pData, uint16_t Length);
static int32_t           I2C_MemRead(I2C_HandleTypeDef *hI2c, uint16_t DevAddr, uint16_t Reg,
                                     uint16_t MemAddSize, uint8_t *pData, uint16_t Length);
static int32_t           I2C_Transmit(I2C_HandleTypeDef *hI2c, uint16_t DevAddr,
                                      uint8_t *pData, uint16_t Length);
static int32_t           I2C_Receive(I2C_HandleTypeDef *hI2c, uint16_t DevAddr,
                                     uint8_t *pData, uint16_t Length);
static int32_t           I2C_Probe(I2C_HandleTypeDef *hI2c, uint16_t DevAddr, uint32_t Trials);

static uint32_t I2C_ComputeTiming(uint32_t clock_src_freq, uint32_t i2c_freq);
static void     I2C_ComputePrescDelays(uint32_t clock_src_freq, uint32_t I2C_speed);
static uint32_t I2C_ComputeSclPhases(uint32_t clock_src_freq, uint32_t I2C_speed);

/* Lock hooks */

/* Weak no-ops, right for a single-threaded application or before the scheduler starts. */

__weak void BSP_I2C1_Lock(void)
{
}

__weak void BSP_I2C1_Unlock(void)
{
}

__weak void BSP_I2C2_Lock(void)
{
}

__weak void BSP_I2C2_Unlock(void)
{
}

__weak void BSP_SPI2_Lock(void)
{
}

__weak void BSP_SPI2_Unlock(void)
{
}

__weak void BSP_I2C_TransactionBegin(void)
{
}

__weak void BSP_I2C_TransactionEnd(void)
{
}

/* Bus 1: on-board sensor bus, fixed TIMINGR word */

int32_t BSP_I2C1_Init(void)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_I2C1_Lock();

  if (s_i2c1_users == 0U)
  {
    hbus_i2c1.Instance = BUS_I2C1;

    ret = I2C1_MspUp();

    if (ret == BSP_ERROR_NONE)
    {
      if (I2C_ConfigurePeripheral(&hbus_i2c1, BUS_I2C1_TIMING) != HAL_OK)
      {
        ret = BSP_ERROR_BUS_FAILURE;
      }
      else
      {
        /* The MSP may have run inside HAL_I2C_Init, which cannot report its failure. */
        ret = I2C1_MspResult();
      }

      if (ret != BSP_ERROR_NONE)
      {
        /* Release the pins and clock, and reset the handle so a retry starts clean. */
        (void)HAL_I2C_DeInit(&hbus_i2c1);
        I2C1_MspDown();
      }
    }
  }

  /* Count every successful call, not only the one that configured the hardware. */
  if (ret == BSP_ERROR_NONE)
  {
    s_i2c1_users++;
  }

  BSP_I2C1_Unlock();

  return ret;
}

int32_t BSP_I2C1_DeInit(void)
{
  int32_t ret;

  BSP_I2C1_Lock();

  if (s_i2c1_users == 0U)
  {
    /* Unbalanced call: report it rather than wrap the counter. */
    ret = BSP_ERROR_NO_INIT;
  }
  else
  {
    ret = BSP_ERROR_NONE;
    s_i2c1_users--;

    if (s_i2c1_users == 0U)
    {
      /* Stop the peripheral before releasing the pins it drives. */
      if (HAL_I2C_DeInit(&hbus_i2c1) != HAL_OK)
      {
        ret = BSP_ERROR_BUS_FAILURE;
      }

      I2C1_MspDown();
    }
  }

  BSP_I2C1_Unlock();

  return ret;
}

int32_t BSP_I2C1_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret;

  BSP_I2C1_Lock();
  ret = (s_i2c1_users == 0U) ? BSP_ERROR_NO_INIT :
        I2C1_After(I2C_MemWrite(&hbus_i2c1, DevAddr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length));
  BSP_I2C1_Unlock();

  return ret;
}

int32_t BSP_I2C1_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret;

  BSP_I2C1_Lock();
  ret = (s_i2c1_users == 0U) ? BSP_ERROR_NO_INIT :
        I2C1_After(I2C_MemRead(&hbus_i2c1, DevAddr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length));
  BSP_I2C1_Unlock();

  return ret;
}

int32_t BSP_I2C1_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret;

  BSP_I2C1_Lock();
  ret = (s_i2c1_users == 0U) ? BSP_ERROR_NO_INIT :
        I2C1_After(I2C_MemWrite(&hbus_i2c1, DevAddr, Reg, I2C_MEMADD_SIZE_16BIT, pData, Length));
  BSP_I2C1_Unlock();

  return ret;
}

int32_t BSP_I2C1_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret;

  BSP_I2C1_Lock();
  ret = (s_i2c1_users == 0U) ? BSP_ERROR_NO_INIT :
        I2C1_After(I2C_MemRead(&hbus_i2c1, DevAddr, Reg, I2C_MEMADD_SIZE_16BIT, pData, Length));
  BSP_I2C1_Unlock();

  return ret;
}

int32_t BSP_I2C1_Send(uint16_t DevAddr, uint8_t *pData, uint16_t Length)
{
  int32_t ret;

  BSP_I2C1_Lock();
  ret = (s_i2c1_users == 0U) ? BSP_ERROR_NO_INIT :
        I2C1_After(I2C_Transmit(&hbus_i2c1, DevAddr, pData, Length));
  BSP_I2C1_Unlock();

  return ret;
}

int32_t BSP_I2C1_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t Length)
{
  int32_t ret;

  BSP_I2C1_Lock();
  ret = (s_i2c1_users == 0U) ? BSP_ERROR_NO_INIT :
        I2C1_After(I2C_Receive(&hbus_i2c1, DevAddr, pData, Length));
  BSP_I2C1_Unlock();

  return ret;
}

int32_t BSP_I2C1_IsReady(uint16_t DevAddr, uint32_t Trials)
{
  int32_t ret;

  BSP_I2C1_Lock();
  ret = (s_i2c1_users == 0U) ? BSP_ERROR_NO_INIT :
        I2C1_After(I2C_Probe(&hbus_i2c1, DevAddr, Trials));
  BSP_I2C1_Unlock();

  return ret;
}

/* Bus 2: expansion-header bus */

int32_t BSP_I2C2_Init(void)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_I2C2_Lock();

  if (s_i2c2_users == 0U)
  {
    /* Timing from the live PCLK1, not frozen at build time. */
    uint32_t timing = I2C_ComputeTiming(HAL_RCC_GetPCLK1Freq(), BUS_I2C2_FREQUENCY);

    hbus_i2c2.Instance = BUS_I2C2;

    if (timing == 0U)
    {
      /* BUS_I2C2_FREQUENCY is outside every band, or unreachable from this PCLK1. */
      ret = BSP_ERROR_WRONG_PARAM;
    }
    else
    {
      ret = I2C2_MspUp();

      if (ret == BSP_ERROR_NONE)
      {
        if (I2C_ConfigurePeripheral(&hbus_i2c2, timing) != HAL_OK)
        {
          ret = BSP_ERROR_BUS_FAILURE;
        }
        else
        {
          ret = I2C2_MspResult();
        }

        if (ret != BSP_ERROR_NONE)
        {
          (void)HAL_I2C_DeInit(&hbus_i2c2);
          I2C2_MspDown();
        }
      }
    }
  }

  if (ret == BSP_ERROR_NONE)
  {
    s_i2c2_users++;
  }

  BSP_I2C2_Unlock();

  return ret;
}

int32_t BSP_I2C2_DeInit(void)
{
  int32_t ret;

  BSP_I2C2_Lock();

  if (s_i2c2_users == 0U)
  {
    ret = BSP_ERROR_NO_INIT;
  }
  else
  {
    ret = BSP_ERROR_NONE;
    s_i2c2_users--;

    if (s_i2c2_users == 0U)
    {
      if (HAL_I2C_DeInit(&hbus_i2c2) != HAL_OK)
      {
        ret = BSP_ERROR_BUS_FAILURE;
      }

      I2C2_MspDown();
    }
  }

  BSP_I2C2_Unlock();

  return ret;
}

int32_t BSP_I2C2_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret;

  BSP_I2C2_Lock();
  ret = (s_i2c2_users == 0U) ? BSP_ERROR_NO_INIT :
        I2C2_After(I2C_MemWrite(&hbus_i2c2, DevAddr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length));
  BSP_I2C2_Unlock();

  return ret;
}

int32_t BSP_I2C2_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret;

  BSP_I2C2_Lock();
  ret = (s_i2c2_users == 0U) ? BSP_ERROR_NO_INIT :
        I2C2_After(I2C_MemRead(&hbus_i2c2, DevAddr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length));
  BSP_I2C2_Unlock();

  return ret;
}

int32_t BSP_I2C2_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret;

  BSP_I2C2_Lock();
  ret = (s_i2c2_users == 0U) ? BSP_ERROR_NO_INIT :
        I2C2_After(I2C_MemWrite(&hbus_i2c2, DevAddr, Reg, I2C_MEMADD_SIZE_16BIT, pData, Length));
  BSP_I2C2_Unlock();

  return ret;
}

int32_t BSP_I2C2_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret;

  BSP_I2C2_Lock();
  ret = (s_i2c2_users == 0U) ? BSP_ERROR_NO_INIT :
        I2C2_After(I2C_MemRead(&hbus_i2c2, DevAddr, Reg, I2C_MEMADD_SIZE_16BIT, pData, Length));
  BSP_I2C2_Unlock();

  return ret;
}

int32_t BSP_I2C2_Send(uint16_t DevAddr, uint8_t *pData, uint16_t Length)
{
  int32_t ret;

  BSP_I2C2_Lock();
  ret = (s_i2c2_users == 0U) ? BSP_ERROR_NO_INIT :
        I2C2_After(I2C_Transmit(&hbus_i2c2, DevAddr, pData, Length));
  BSP_I2C2_Unlock();

  return ret;
}

int32_t BSP_I2C2_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t Length)
{
  int32_t ret;

  BSP_I2C2_Lock();
  ret = (s_i2c2_users == 0U) ? BSP_ERROR_NO_INIT :
        I2C2_After(I2C_Receive(&hbus_i2c2, DevAddr, pData, Length));
  BSP_I2C2_Unlock();

  return ret;
}

int32_t BSP_I2C2_IsReady(uint16_t DevAddr, uint32_t Trials)
{
  int32_t ret;

  BSP_I2C2_Lock();
  ret = (s_i2c2_users == 0U) ? BSP_ERROR_NO_INIT :
        I2C2_After(I2C_Probe(&hbus_i2c2, DevAddr, Trials));
  BSP_I2C2_Unlock();

  return ret;
}

/* Bus 3: expansion-header SPI bus */

int32_t BSP_SPI2_Init(void)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_SPI2_Lock();

  if (s_spi2_users == 0U)
  {
    hbus_spi2.Instance = BUS_SPI2;

    ret = SPI2_MspUp();

    if (ret == BSP_ERROR_NONE)
    {
      if (SPI_ConfigurePeripheral() != HAL_OK)
      {
        ret = BSP_ERROR_BUS_FAILURE;
      }
      else
      {
        ret = SPI2_MspResult();
      }

      if (ret != BSP_ERROR_NONE)
      {
        /* Only a handle the HAL brought up: see SPI_ConfigurePeripheral. */
        if (hbus_spi2.State != HAL_SPI_STATE_RESET)
        {
          (void)HAL_SPI_DeInit(&hbus_spi2);
        }
        SPI2_MspDown();
      }
    }
  }

  if (ret == BSP_ERROR_NONE)
  {
    s_spi2_users++;
  }

  BSP_SPI2_Unlock();

  return ret;
}

int32_t BSP_SPI2_DeInit(void)
{
  int32_t ret;

  BSP_SPI2_Lock();

  if (s_spi2_users == 0U)
  {
    ret = BSP_ERROR_NO_INIT;
  }
  else
  {
    ret = BSP_ERROR_NONE;
    s_spi2_users--;

    if (s_spi2_users == 0U)
    {
      if (HAL_SPI_DeInit(&hbus_spi2) != HAL_OK)
      {
        ret = BSP_ERROR_BUS_FAILURE;
      }

      SPI2_MspDown();
      s_spi2_actual_hz = 0U;
    }
  }

  BSP_SPI2_Unlock();

  return ret;
}

int32_t BSP_SPI2_SetConfig(const BSP_SPI_Config_t *pConfig)
{
  int32_t ret = BSP_ERROR_NONE;

  if (pConfig == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if ((pConfig->Mode > BUS_SPI_MODE_3) ||
      (pConfig->BitOrder > BUS_SPI_LSB_FIRST) ||
      (pConfig->ClockFreq == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Slowest rate is PCLK1 / 256; a slower request is refused rather than run faster. */
  if (pConfig->ClockFreq < (HAL_RCC_GetPCLK1Freq() >> BUS_SPI_PRESCALER_NBR))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  BSP_SPI2_Lock();

  if ((pConfig->ClockFreq != s_spi2_config.ClockFreq) ||
      (pConfig->Mode      != s_spi2_config.Mode) ||
      (pConfig->BitOrder  != s_spi2_config.BitOrder))
  {
    s_spi2_config = *pConfig;

    /* Applied now only if the bus is up; otherwise the first init applies it. */
    if (s_spi2_users != 0U)
    {
      if (SPI_ConfigurePeripheral() != HAL_OK)
      {
        ret = BSP_ERROR_BUS_FAILURE;
      }
    }
  }

  BSP_SPI2_Unlock();

  return ret;
}

int32_t BSP_SPI2_GetConfig(BSP_SPI_Config_t *pConfig)
{
  if (pConfig == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  BSP_SPI2_Lock();
  *pConfig = s_spi2_config;
  BSP_SPI2_Unlock();

  return BSP_ERROR_NONE;
}

uint32_t BSP_SPI2_GetClockFreq(void)
{
  return s_spi2_actual_hz;
}

int32_t BSP_SPI2_Send(uint8_t *pData, uint16_t Length)
{
  int32_t ret;

  if ((pData == NULL) || (Length == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  BSP_SPI2_Lock();

  if (s_spi2_users == 0U)
  {
    ret = BSP_ERROR_NO_INIT;
  }
  else
  {
    ret = SPI_MapError(HAL_SPI_Transmit(&hbus_spi2, pData, Length, BUS_SPI_TIMEOUT_MS));
  }

  BSP_SPI2_Unlock();

  return ret;
}

int32_t BSP_SPI2_Recv(uint8_t *pData, uint16_t Length)
{
  int32_t ret;

  if ((pData == NULL) || (Length == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  BSP_SPI2_Lock();

  if (s_spi2_users == 0U)
  {
    ret = BSP_ERROR_NO_INIT;
  }
  else
  {
    uint16_t index;

    /* Full duplex from a zeroed buffer, received in place: the receive-only
     * mode leaves MOSI undriven. Each byte is sent before its reply lands. */
    for (index = 0U; index < Length; index++)
    {
      pData[index] = 0U;
    }

    ret = SPI_MapError(HAL_SPI_TransmitReceive(&hbus_spi2, pData, pData, Length,
                                               BUS_SPI_TIMEOUT_MS));
  }

  BSP_SPI2_Unlock();

  return ret;
}

int32_t BSP_SPI2_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t Length)
{
  int32_t ret;

  if ((pTxData == NULL) || (pRxData == NULL) || (Length == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  BSP_SPI2_Lock();

  if (s_spi2_users == 0U)
  {
    ret = BSP_ERROR_NO_INIT;
  }
  else
  {
    ret = SPI_MapError(HAL_SPI_TransmitReceive(&hbus_spi2, pTxData, pRxData, Length,
                                               BUS_SPI_TIMEOUT_MS));
  }

  BSP_SPI2_Unlock();

  return ret;
}

/* Pins and clocks (MSP) */

/* This module sets up its own pins and clocks, not through the application's
 * HAL_I2C_MspInit(), so a project can adopt one bus without a whole board init. */

static HAL_StatusTypeDef I2C1_MspInit(void)
{
  GPIO_InitTypeDef gpio_init = {0};
  RCC_PeriphCLKInitTypeDef periph_clk_init = {0};

  /* Kernel clock PCLK1: the fixed timing word assumes it. */
  periph_clk_init.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  periph_clk_init.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_PCLK1;

  if (HAL_RCCEx_PeriphCLKConfig(&periph_clk_init) != HAL_OK)
  {
    return HAL_ERROR;
  }

  BUS_I2C1_SCL_GPIO_CLK_ENABLE();
  BUS_I2C1_SDA_GPIO_CLK_ENABLE();

  I2C_RecoverBus(BUS_I2C1_SCL_GPIO_PORT, BUS_I2C1_SCL_PIN,
                 BUS_I2C1_SDA_GPIO_PORT, BUS_I2C1_SDA_PIN);

  /* Open drain, no internal pull: both lines have board pull-ups. */
  gpio_init.Mode  = GPIO_MODE_AF_OD;
  gpio_init.Pull  = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  gpio_init.Pin       = BUS_I2C1_SCL_PIN;
  gpio_init.Alternate = BUS_I2C1_SCL_AF;
  HAL_GPIO_Init(BUS_I2C1_SCL_GPIO_PORT, &gpio_init);

  gpio_init.Pin       = BUS_I2C1_SDA_PIN;
  gpio_init.Alternate = BUS_I2C1_SDA_AF;
  HAL_GPIO_Init(BUS_I2C1_SDA_GPIO_PORT, &gpio_init);

  BUS_I2C1_CLK_ENABLE();

  /* Reset the peripheral, in case something configured I2C1 earlier in the boot. */
  BUS_I2C1_FORCE_RESET();
  BUS_I2C1_RELEASE_RESET();

  return HAL_OK;
}

/* Free a bus held after a reset mid-transfer, before the peripheral takes the pins.
 * SDA high: drive nothing (an SCL edge could move a device stopped on a 1 onto a 0).
 * SDA low: up to 9 pulses, each ending in a STOP, so nothing is written to a device. */
static void I2C_RecoverBus(GPIO_TypeDef *pSclPort, uint16_t SclPin,
                           GPIO_TypeDef *pSdaPort, uint16_t SdaPin)
{
  GPIO_InitTypeDef gpio_init = {0};
  uint32_t clock;

  /* Both lines released: open drain, written high. */
  HAL_GPIO_WritePin(pSclPort, SclPin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(pSdaPort, SdaPin, GPIO_PIN_SET);

  gpio_init.Mode  = GPIO_MODE_OUTPUT_OD;
  gpio_init.Pull  = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;

  gpio_init.Pin = SclPin;
  HAL_GPIO_Init(pSclPort, &gpio_init);
  gpio_init.Pin = SdaPin;
  HAL_GPIO_Init(pSdaPort, &gpio_init);
  I2C_RecoveryDelay();

  if (HAL_GPIO_ReadPin(pSdaPort, SdaPin) == GPIO_PIN_SET)
  {
    return;
  }

  /* Nine pulses cover the rest of a byte and its acknowledge. */
  for (clock = 0U; clock < 9U; clock++)
  {
    HAL_GPIO_WritePin(pSclPort, SclPin, GPIO_PIN_RESET);
    I2C_RecoveryDelay();
    HAL_GPIO_WritePin(pSdaPort, SdaPin, GPIO_PIN_RESET);
    I2C_RecoveryDelay();
    HAL_GPIO_WritePin(pSclPort, SclPin, GPIO_PIN_SET);
    I2C_RecoveryDelay();
    HAL_GPIO_WritePin(pSdaPort, SdaPin, GPIO_PIN_SET);
    I2C_RecoveryDelay();
    if (HAL_GPIO_ReadPin(pSdaPort, SdaPin) == GPIO_PIN_SET)
    {
      break;
    }
  }
}

/* At least 5 us, half a 100 kHz period: each pass takes at least four cycles, and
 * nothing else can be assumed to run this early. */
static void I2C_RecoveryDelay(void)
{
  volatile uint32_t spin = SystemCoreClock / 800000U;

  while (spin > 0U)
  {
    spin--;
  }
}

/* GPIO port clocks stay on: other peripherals share the ports. */
static void I2C1_MspDeInit(void)
{
  HAL_GPIO_DeInit(BUS_I2C1_SCL_GPIO_PORT, BUS_I2C1_SCL_PIN);
  HAL_GPIO_DeInit(BUS_I2C1_SDA_GPIO_PORT, BUS_I2C1_SDA_PIN);

  /* Read back before gating the clock, so the last write has landed. */
  (void)READ_REG(hbus_i2c1.Instance->CR1);
  __DSB();
  BUS_I2C1_CLK_DISABLE();
}

static HAL_StatusTypeDef I2C2_MspInit(void)
{
  GPIO_InitTypeDef gpio_init = {0};
  RCC_PeriphCLKInitTypeDef periph_clk_init = {0};

  /* Kernel clock PCLK1: BSP_I2C2_Init() computes the timing word from it. */
  periph_clk_init.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
  periph_clk_init.I2c2ClockSelection   = RCC_I2C2CLKSOURCE_PCLK1;

  if (HAL_RCCEx_PeriphCLKConfig(&periph_clk_init) != HAL_OK)
  {
    return HAL_ERROR;
  }

  BUS_I2C2_SCL_GPIO_CLK_ENABLE();
  BUS_I2C2_SDA_GPIO_CLK_ENABLE();

  I2C_RecoverBus(BUS_I2C2_SCL_GPIO_PORT, BUS_I2C2_SCL_PIN,
                 BUS_I2C2_SDA_GPIO_PORT, BUS_I2C2_SDA_PIN);

  gpio_init.Mode  = GPIO_MODE_AF_OD;
  gpio_init.Pull  = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  gpio_init.Pin       = BUS_I2C2_SCL_PIN;
  gpio_init.Alternate = BUS_I2C2_SCL_AF;
  HAL_GPIO_Init(BUS_I2C2_SCL_GPIO_PORT, &gpio_init);

  gpio_init.Pin       = BUS_I2C2_SDA_PIN;
  gpio_init.Alternate = BUS_I2C2_SDA_AF;
  HAL_GPIO_Init(BUS_I2C2_SDA_GPIO_PORT, &gpio_init);

  BUS_I2C2_CLK_ENABLE();

  BUS_I2C2_FORCE_RESET();
  BUS_I2C2_RELEASE_RESET();

  return HAL_OK;
}

/* The header pins return to high impedance; the board pull-ups hold the bus idle. */
static void I2C2_MspDeInit(void)
{
  HAL_GPIO_DeInit(BUS_I2C2_SCL_GPIO_PORT, BUS_I2C2_SCL_PIN);
  HAL_GPIO_DeInit(BUS_I2C2_SDA_GPIO_PORT, BUS_I2C2_SDA_PIN);

  /* Read back before gating the clock, so the last write has landed. */
  (void)READ_REG(hbus_i2c2.Instance->CR1);
  __DSB();
  BUS_I2C2_CLK_DISABLE();
}

static HAL_StatusTypeDef SPI2_MspInit(void)
{
  GPIO_InitTypeDef gpio_init = {0};
  RCC_PeriphCLKInitTypeDef periph_clk_init = {0};

  /* Kernel clock PCLK1: SPI_ConfigurePeripheral() picks the divider from it. */
  periph_clk_init.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
  periph_clk_init.Spi2ClockSelection   = RCC_SPI2CLKSOURCE_PCLK1;

  if (HAL_RCCEx_PeriphCLKConfig(&periph_clk_init) != HAL_OK)
  {
    return HAL_ERROR;
  }

  BUS_SPI2_SCK_GPIO_CLK_ENABLE();
  BUS_SPI2_MOSI_GPIO_CLK_ENABLE();
  BUS_SPI2_MISO_GPIO_CLK_ENABLE();

  /* Push-pull, no pulls: MISO is driven by a device when one is present. */
  gpio_init.Mode  = GPIO_MODE_AF_PP;
  gpio_init.Pull  = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;

  gpio_init.Pin       = BUS_SPI2_SCK_PIN;
  gpio_init.Alternate = BUS_SPI2_SCK_AF;
  HAL_GPIO_Init(BUS_SPI2_SCK_GPIO_PORT, &gpio_init);

  gpio_init.Pin       = BUS_SPI2_MOSI_PIN;
  gpio_init.Alternate = BUS_SPI2_MOSI_AF;
  HAL_GPIO_Init(BUS_SPI2_MOSI_GPIO_PORT, &gpio_init);

  gpio_init.Pin       = BUS_SPI2_MISO_PIN;
  gpio_init.Alternate = BUS_SPI2_MISO_AF;
  HAL_GPIO_Init(BUS_SPI2_MISO_GPIO_PORT, &gpio_init);

  BUS_SPI2_CLK_ENABLE();

  BUS_SPI2_FORCE_RESET();
  BUS_SPI2_RELEASE_RESET();

  return HAL_OK;
}

/* The header pins return to high impedance: no driven clock after teardown. */
static void SPI2_MspDeInit(void)
{
  HAL_GPIO_DeInit(BUS_SPI2_SCK_GPIO_PORT, BUS_SPI2_SCK_PIN);
  HAL_GPIO_DeInit(BUS_SPI2_MOSI_GPIO_PORT, BUS_SPI2_MOSI_PIN);
  HAL_GPIO_DeInit(BUS_SPI2_MISO_GPIO_PORT, BUS_SPI2_MISO_PIN);

  /* Read back before gating the clock, so the last write has landed. */
  (void)READ_REG(hbus_spi2.Instance->CR1);
  __DSB();
  BUS_SPI2_CLK_DISABLE();
}

/* MSP registration */

/* Two ways to reach the routines above, chosen by the HAL's own build option. */

#if defined(USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)

static uint8_t           s_i2c1_msp_cb_valid;
static HAL_StatusTypeDef s_i2c1_msp_status = HAL_OK;
static uint8_t           s_i2c2_msp_cb_valid;
static HAL_StatusTypeDef s_i2c2_msp_status = HAL_OK;

static void I2C1_MspInitCb(I2C_HandleTypeDef *hI2c)
{
  UNUSED(hI2c);
  s_i2c1_msp_status = I2C1_MspInit();
}

static void I2C1_MspDeInitCb(I2C_HandleTypeDef *hI2c)
{
  UNUSED(hI2c);
  I2C1_MspDeInit();
}

static void I2C2_MspInitCb(I2C_HandleTypeDef *hI2c)
{
  UNUSED(hI2c);
  s_i2c2_msp_status = I2C2_MspInit();
}

static void I2C2_MspDeInitCb(I2C_HandleTypeDef *hI2c)
{
  UNUSED(hI2c);
  I2C2_MspDeInit();
}

static int32_t I2C_RegisterMsp(I2C_HandleTypeDef *hI2c,
                               pI2C_CallbackTypeDef pInit,
                               pI2C_CallbackTypeDef pDeInit)
{
  if ((pInit == NULL) || (pDeInit == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (HAL_I2C_RegisterCallback(hI2c, HAL_I2C_MSPINIT_CB_ID, pInit) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  if (HAL_I2C_RegisterCallback(hI2c, HAL_I2C_MSPDEINIT_CB_ID, pDeInit) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_I2C1_RegisterDefaultMspCallbacks(void)
{
  int32_t ret;

  BSP_I2C1_Lock();
  ret = I2C_RegisterMsp(&hbus_i2c1, I2C1_MspInitCb, I2C1_MspDeInitCb);
  if (ret == BSP_ERROR_NONE)
  {
    s_i2c1_msp_cb_valid = 1U;
  }
  BSP_I2C1_Unlock();

  return ret;
}

int32_t BSP_I2C1_RegisterMspCallbacks(BSP_I2C_Cb_t *Callbacks)
{
  int32_t ret;

  if (Callbacks == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  BSP_I2C1_Lock();
  ret = I2C_RegisterMsp(&hbus_i2c1, Callbacks->pMspInitCb, Callbacks->pMspDeInitCb);
  if (ret == BSP_ERROR_NONE)
  {
    /* So the next Init keeps the application's routines. */
    s_i2c1_msp_cb_valid = 1U;
  }
  BSP_I2C1_Unlock();

  return ret;
}

int32_t BSP_I2C2_RegisterDefaultMspCallbacks(void)
{
  int32_t ret;

  BSP_I2C2_Lock();
  ret = I2C_RegisterMsp(&hbus_i2c2, I2C2_MspInitCb, I2C2_MspDeInitCb);
  if (ret == BSP_ERROR_NONE)
  {
    s_i2c2_msp_cb_valid = 1U;
  }
  BSP_I2C2_Unlock();

  return ret;
}

int32_t BSP_I2C2_RegisterMspCallbacks(BSP_I2C_Cb_t *Callbacks)
{
  int32_t ret;

  if (Callbacks == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  BSP_I2C2_Lock();
  ret = I2C_RegisterMsp(&hbus_i2c2, Callbacks->pMspInitCb, Callbacks->pMspDeInitCb);
  if (ret == BSP_ERROR_NONE)
  {
    s_i2c2_msp_cb_valid = 1U;
  }
  BSP_I2C2_Unlock();

  return ret;
}

#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)

static uint8_t           s_spi2_msp_cb_valid;
static HAL_StatusTypeDef s_spi2_msp_status = HAL_OK;

static void SPI2_MspInitCb(SPI_HandleTypeDef *hSpi)
{
  UNUSED(hSpi);
  s_spi2_msp_status = SPI2_MspInit();
}

static void SPI2_MspDeInitCb(SPI_HandleTypeDef *hSpi)
{
  UNUSED(hSpi);
  SPI2_MspDeInit();
}

static int32_t SPI_RegisterMsp(SPI_HandleTypeDef *hSpi,
                               pSPI_CallbackTypeDef pInit,
                               pSPI_CallbackTypeDef pDeInit)
{
  if ((pInit == NULL) || (pDeInit == NULL))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  if (HAL_SPI_RegisterCallback(hSpi, HAL_SPI_MSPINIT_CB_ID, pInit) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  if (HAL_SPI_RegisterCallback(hSpi, HAL_SPI_MSPDEINIT_CB_ID, pDeInit) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return BSP_ERROR_NONE;
}

int32_t BSP_SPI2_RegisterDefaultMspCallbacks(void)
{
  int32_t ret;

  BSP_SPI2_Lock();
  ret = SPI_RegisterMsp(&hbus_spi2, SPI2_MspInitCb, SPI2_MspDeInitCb);
  if (ret == BSP_ERROR_NONE)
  {
    s_spi2_msp_cb_valid = 1U;
  }
  BSP_SPI2_Unlock();

  return ret;
}

int32_t BSP_SPI2_RegisterMspCallbacks(BSP_SPI_Cb_t *Callbacks)
{
  int32_t ret;

  if (Callbacks == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  BSP_SPI2_Lock();
  ret = SPI_RegisterMsp(&hbus_spi2, Callbacks->pMspInitCb, Callbacks->pMspDeInitCb);
  if (ret == BSP_ERROR_NONE)
  {
    s_spi2_msp_cb_valid = 1U;
  }
  BSP_SPI2_Unlock();

  return ret;
}

#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */

static int32_t I2C1_MspUp(void)
{
#if defined(USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)
  s_i2c1_msp_status = HAL_OK;

  if (s_i2c1_msp_cb_valid == 0U)
  {
    return (BSP_I2C1_RegisterDefaultMspCallbacks() == BSP_ERROR_NONE)
           ? BSP_ERROR_NONE : BSP_ERROR_MSP_FAILURE;
  }

  return BSP_ERROR_NONE;
#else
  return (I2C1_MspInit() == HAL_OK) ? BSP_ERROR_NONE : BSP_ERROR_MSP_FAILURE;
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
}

static int32_t I2C1_MspResult(void)
{
#if defined(USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)
  return (s_i2c1_msp_status == HAL_OK) ? BSP_ERROR_NONE : BSP_ERROR_MSP_FAILURE;
#else
  return BSP_ERROR_NONE;
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
}

static void I2C1_MspDown(void)
{
#if defined(USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)
  /* HAL_I2C_DeInit has already run the registered routine. */
#else
  I2C1_MspDeInit();
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
}

static int32_t I2C2_MspUp(void)
{
#if defined(USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)
  s_i2c2_msp_status = HAL_OK;

  if (s_i2c2_msp_cb_valid == 0U)
  {
    return (BSP_I2C2_RegisterDefaultMspCallbacks() == BSP_ERROR_NONE)
           ? BSP_ERROR_NONE : BSP_ERROR_MSP_FAILURE;
  }

  return BSP_ERROR_NONE;
#else
  return (I2C2_MspInit() == HAL_OK) ? BSP_ERROR_NONE : BSP_ERROR_MSP_FAILURE;
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
}

static int32_t I2C2_MspResult(void)
{
#if defined(USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)
  return (s_i2c2_msp_status == HAL_OK) ? BSP_ERROR_NONE : BSP_ERROR_MSP_FAILURE;
#else
  return BSP_ERROR_NONE;
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
}

static void I2C2_MspDown(void)
{
#if defined(USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)
  /* HAL_I2C_DeInit has already run the registered routine. */
#else
  I2C2_MspDeInit();
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
}

static int32_t SPI2_MspUp(void)
{
#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
  s_spi2_msp_status = HAL_OK;

  if (s_spi2_msp_cb_valid == 0U)
  {
    return (BSP_SPI2_RegisterDefaultMspCallbacks() == BSP_ERROR_NONE)
           ? BSP_ERROR_NONE : BSP_ERROR_MSP_FAILURE;
  }

  return BSP_ERROR_NONE;
#else
  return (SPI2_MspInit() == HAL_OK) ? BSP_ERROR_NONE : BSP_ERROR_MSP_FAILURE;
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
}

static int32_t SPI2_MspResult(void)
{
#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
  return (s_spi2_msp_status == HAL_OK) ? BSP_ERROR_NONE : BSP_ERROR_MSP_FAILURE;
#else
  return BSP_ERROR_NONE;
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
}

static void SPI2_MspDown(void)
{
#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
  /* HAL_SPI_DeInit has already run the registered routine. */
#else
  SPI2_MspDeInit();
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
}

/* Bus 3 configuration */

/* PCLK1 is read on every call, not cached: the BSP does not own the clock tree. */
static HAL_StatusTypeDef SPI_ConfigurePeripheral(void)
{
  static const uint32_t prescaler_value[BUS_SPI_PRESCALER_NBR] =
  {
    SPI_BAUDRATEPRESCALER_2,   SPI_BAUDRATEPRESCALER_4,
    SPI_BAUDRATEPRESCALER_8,   SPI_BAUDRATEPRESCALER_16,
    SPI_BAUDRATEPRESCALER_32,  SPI_BAUDRATEPRESCALER_64,
    SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_256,
  };

  uint32_t kernel_hz = HAL_RCC_GetPCLK1Freq();
  uint32_t index = 0U;

  if (kernel_hz == 0U)
  {
    return HAL_ERROR;
  }

  /* Largest division at or below the request. */
  while ((index < (BUS_SPI_PRESCALER_NBR - 1U)) &&
         ((kernel_hz >> (index + 1U)) > s_spi2_config.ClockFreq))
  {
    index++;
  }

  /* Even the slowest division is faster than asked. BSP_SPI2_SetConfig
   * refuses such a rate; this catches BUS_SPI2_FREQUENCY. */
  if ((kernel_hz >> (index + 1U)) > s_spi2_config.ClockFreq)
  {
    return HAL_ERROR;
  }

  /* Re-initialise a running bus. Skip DeInit on a handle the HAL never brought up: it
   * writes the peripheral, whose clock may only come on inside HAL_SPI_Init. */
  if (hbus_spi2.State != HAL_SPI_STATE_RESET)
  {
    (void)HAL_SPI_DeInit(&hbus_spi2);
  }

  hbus_spi2.Instance               = BUS_SPI2;
  hbus_spi2.Init.Mode              = SPI_MODE_MASTER;
  hbus_spi2.Init.Direction         = SPI_DIRECTION_2LINES;
  hbus_spi2.Init.DataSize          = SPI_DATASIZE_8BIT;
  hbus_spi2.Init.CLKPolarity       = ((s_spi2_config.Mode & 0x2U) != 0U)
                                     ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
  hbus_spi2.Init.CLKPhase          = ((s_spi2_config.Mode & 0x1U) != 0U)
                                     ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
  hbus_spi2.Init.NSS               = SPI_NSS_SOFT;
  hbus_spi2.Init.BaudRatePrescaler = prescaler_value[index];
  hbus_spi2.Init.FirstBit          = (s_spi2_config.BitOrder == BUS_SPI_LSB_FIRST)
                                     ? SPI_FIRSTBIT_LSB : SPI_FIRSTBIT_MSB;
  hbus_spi2.Init.TIMode            = SPI_TIMODE_DISABLE;
  hbus_spi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hbus_spi2.Init.CRCPolynomial     = 7U;
  hbus_spi2.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
  /* The HAL disables the SPI after every transfer; this keeps SCK and MOSI driven at
   * their idle levels in between. */
  hbus_spi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;

  if (HAL_SPI_Init(&hbus_spi2) != HAL_OK)
  {
    s_spi2_actual_hz = 0U;
    return HAL_ERROR;
  }

  s_spi2_actual_hz = kernel_hz >> (index + 1U);

  return HAL_OK;
}

static int32_t SPI_MapError(HAL_StatusTypeDef status)
{
  switch (status)
  {
    case HAL_OK:
      return BSP_ERROR_NONE;

    case HAL_TIMEOUT:
      return BSP_ERROR_BUSY;

    default:
      return BSP_ERROR_BUS_TRANSACTION_FAILURE;
  }
}

/* I2C transactions */

/* hI2c->Instance must already be set. */
static HAL_StatusTypeDef I2C_ConfigurePeripheral(I2C_HandleTypeDef *hI2c, uint32_t timing)
{
  HAL_StatusTypeDef status;

  hI2c->Init.Timing           = timing;
  hI2c->Init.OwnAddress1      = 0;
  hI2c->Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  hI2c->Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  hI2c->Init.OwnAddress2      = 0;
  hI2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hI2c->Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;

  /* Clock stretching on: the thermal camera stretches during CCI accesses. */
  hI2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  status = HAL_I2C_Init(hI2c);

  if (status == HAL_OK)
  {
    status = HAL_I2CEx_ConfigAnalogFilter(hI2c, I2C_ANALOGFILTER_ENABLE);
  }

  if (status == HAL_OK)
  {
    status = HAL_I2CEx_ConfigDigitalFilter(hI2c, I2C_DIGITAL_FILTER_COEF);
  }

  return status;
}

static int32_t I2C_MapError(const I2C_HandleTypeDef *hI2c, HAL_StatusTypeDef status)
{
  int32_t ret;

  if (status == HAL_BUSY)
  {
    /* Still busy, or another master holds the bus: nothing was sent; a retry is fine. */
    ret = BSP_ERROR_BUSY;
  }
  else if ((HAL_I2C_GetError(hI2c) & HAL_I2C_ERROR_AF) == HAL_I2C_ERROR_AF)
  {
    /* Test the bit rather than the whole word. */
    ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_PERIPH_FAILURE;
  }

  return ret;
}

/* 1 if a failed transfer left the bus stuck: handle not ready, a timeout, or still BUSY,
 * whatever the error. The HAL leaves no STOP then, and every later transfer fails. */
static uint8_t I2C_LeftStuck(const I2C_HandleTypeDef *hI2c, int32_t Ret)
{
  if (Ret == BSP_ERROR_NONE)
  {
    return 0U;
  }

  return ((hI2c->State != HAL_I2C_STATE_READY) ||
          ((HAL_I2C_GetError(hI2c) & HAL_I2C_ERROR_TIMEOUT) != 0U) ||
          (READ_BIT(hI2c->Instance->ISR, I2C_ISR_BUSY) != 0U)) ? 1U : 0U;
}

/* Under the bus lock, after a transfer: restart a stuck bus as Init does (clocking a held
 * bus free, peripheral from reset). Returns Ret unchanged; the caller may retry. */
static int32_t I2C1_After(int32_t Ret)
{
  if (I2C_LeftStuck(&hbus_i2c1, Ret) != 0U)
  {
    const uint32_t timing = hbus_i2c1.Init.Timing;

    (void)HAL_I2C_DeInit(&hbus_i2c1);
    I2C1_MspDown();
    if (I2C1_MspUp() == BSP_ERROR_NONE)
    {
      (void)I2C_ConfigurePeripheral(&hbus_i2c1, timing);
    }
  }
  return Ret;
}

/* As I2C1_After, for bus 2. */
static int32_t I2C2_After(int32_t Ret)
{
  if (I2C_LeftStuck(&hbus_i2c2, Ret) != 0U)
  {
    const uint32_t timing = hbus_i2c2.Init.Timing;

    (void)HAL_I2C_DeInit(&hbus_i2c2);
    I2C2_MspDown();
    if (I2C2_MspUp() == BSP_ERROR_NONE)
    {
      (void)I2C_ConfigurePeripheral(&hbus_i2c2, timing);
    }
  }
  return Ret;
}

static int32_t I2C_MemWrite(I2C_HandleTypeDef *hI2c, uint16_t DevAddr, uint16_t Reg,
                            uint16_t MemAddSize, uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;
  HAL_StatusTypeDef status;

  /* The HAL clears ErrorCode only once the bus is free: clear a stale one first. */
  hI2c->ErrorCode = HAL_I2C_ERROR_NONE;
  BSP_I2C_TransactionBegin();
  status = HAL_I2C_Mem_Write(hI2c, DevAddr, Reg, MemAddSize, pData, Length,
                             BUS_I2C_TIMEOUT_MS);
  BSP_I2C_TransactionEnd();

  if (status != HAL_OK)
  {
    ret = I2C_MapError(hI2c, status);
  }

  return ret;
}

static int32_t I2C_MemRead(I2C_HandleTypeDef *hI2c, uint16_t DevAddr, uint16_t Reg,
                           uint16_t MemAddSize, uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;
  HAL_StatusTypeDef status;

  /* The HAL clears ErrorCode only once the bus is free: clear a stale one first. */
  hI2c->ErrorCode = HAL_I2C_ERROR_NONE;
  BSP_I2C_TransactionBegin();
  status = HAL_I2C_Mem_Read(hI2c, DevAddr, Reg, MemAddSize, pData, Length,
                            BUS_I2C_TIMEOUT_MS);
  BSP_I2C_TransactionEnd();

  if (status != HAL_OK)
  {
    ret = I2C_MapError(hI2c, status);
  }

  return ret;
}

static int32_t I2C_Transmit(I2C_HandleTypeDef *hI2c, uint16_t DevAddr,
                            uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;
  HAL_StatusTypeDef status;

  /* The HAL clears ErrorCode only once the bus is free: clear a stale one first. */
  hI2c->ErrorCode = HAL_I2C_ERROR_NONE;
  BSP_I2C_TransactionBegin();
  status = HAL_I2C_Master_Transmit(hI2c, DevAddr, pData, Length, BUS_I2C_TIMEOUT_MS);
  BSP_I2C_TransactionEnd();

  if (status != HAL_OK)
  {
    ret = I2C_MapError(hI2c, status);
  }

  return ret;
}

static int32_t I2C_Receive(I2C_HandleTypeDef *hI2c, uint16_t DevAddr,
                           uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;
  HAL_StatusTypeDef status;

  /* The HAL clears ErrorCode only once the bus is free: clear a stale one first. */
  hI2c->ErrorCode = HAL_I2C_ERROR_NONE;
  BSP_I2C_TransactionBegin();
  status = HAL_I2C_Master_Receive(hI2c, DevAddr, pData, Length, BUS_I2C_TIMEOUT_MS);
  BSP_I2C_TransactionEnd();

  if (status != HAL_OK)
  {
    ret = I2C_MapError(hI2c, status);
  }

  return ret;
}

static int32_t I2C_Probe(I2C_HandleTypeDef *hI2c, uint16_t DevAddr, uint32_t Trials)
{
  int32_t           ret = BSP_ERROR_NONE;
  HAL_StatusTypeDef status;

  /* An absent device is silent: a probe that finds nothing gives BUSY, not a fault. */
  BSP_I2C_TransactionBegin();
  status = HAL_I2C_IsDeviceReady(hI2c, DevAddr, Trials, BUS_I2C_TIMEOUT_MS);
  BSP_I2C_TransactionEnd();

  if (status != HAL_OK)
  {
    /* The HAL flags an unanswered probe as a timeout; with the bus idle that is only an
     * absent device, which needs no restart. */
    if ((status == HAL_ERROR) && (READ_BIT(hI2c->Instance->ISR, I2C_ISR_BUSY) == 0U))
    {
      hI2c->ErrorCode &= ~HAL_I2C_ERROR_TIMEOUT;
    }
    ret = BSP_ERROR_BUSY;
  }

  return ret;
}

/* I2C timing computation, adapted from ST's stm32n6570_discovery_bus.c */

/* TIMINGR word for i2c_freq (Hz) from the kernel clock, or 0 if it cannot be reached. */
static uint32_t I2C_ComputeTiming(uint32_t clock_src_freq, uint32_t i2c_freq)
{
  uint32_t ret = 0U;
  uint32_t speed;
  uint32_t idx;

  if ((clock_src_freq != 0U) && (i2c_freq != 0U))
  {
    for (speed = 0U; speed <= I2C_SPEED_FREQ_FAST_PLUS; speed++)
    {
      if ((i2c_freq >= I2C_Charac[speed].freq_min) &&
          (i2c_freq <= I2C_Charac[speed].freq_max))
      {
        /* Start empty: stale entries from an earlier bring-up would be searched. */
        s_valid_timing_nbr = 0U;

        I2C_ComputePrescDelays(clock_src_freq, speed);
        idx = I2C_ComputeSclPhases(clock_src_freq, speed);

        if (idx < I2C_VALID_TIMING_NBR)
        {
          ret = ((s_valid_timing[idx].presc   & 0x0FU) << 28) |
                ((s_valid_timing[idx].tscldel & 0x0FU) << 20) |
                ((s_valid_timing[idx].tsdadel & 0x0FU) << 16) |
                ((s_valid_timing[idx].sclh    & 0xFFU) << 8)  |
                ((s_valid_timing[idx].scll    & 0xFFU) << 0);
        }

        break;
      }
    }
  }

  return ret;
}

/* Fill the candidate table with PRESC, SCLDEL and SDADEL sets. */
static void I2C_ComputePrescDelays(uint32_t clock_src_freq, uint32_t I2C_speed)
{
  uint32_t prev_presc = I2C_PRESC_MAX;
  uint32_t ti2cclk;
  int32_t  tsdadel_min;
  int32_t  tsdadel_max;
  int32_t  tscldel_min;
  uint32_t presc;
  uint32_t scldel;
  uint32_t sdadel;
  uint32_t tafdel_min;
  uint32_t tafdel_max;

  ti2cclk = (SEC2NSEC + (clock_src_freq / 2U)) / clock_src_freq;

  tafdel_min = I2C_ANALOG_FILTER_DELAY_MIN;
  tafdel_max = I2C_ANALOG_FILTER_DELAY_MAX;

  /* tDNF = DNF x tI2CCLK, tPRESC = (PRESC+1) x tI2CCLK
   * SDADEL >= {tf + tHD;DAT(min) - tAF(min) - tDNF - [3 x tI2CCLK]} / tPRESC
   * SDADEL <= {tVD;DAT(max) - tr - tAF(max) - tDNF - [4 x tI2CCLK]} / tPRESC */
  tsdadel_min = (int32_t)I2C_Charac[I2C_speed].tfall +
                (int32_t)I2C_Charac[I2C_speed].hddat_min -
                (int32_t)tafdel_min -
                (((int32_t)I2C_Charac[I2C_speed].dnf + 3) * (int32_t)ti2cclk);

  tsdadel_max = (int32_t)I2C_Charac[I2C_speed].vddat_max -
                (int32_t)I2C_Charac[I2C_speed].trise -
                (int32_t)tafdel_max -
                (((int32_t)I2C_Charac[I2C_speed].dnf + 4) * (int32_t)ti2cclk);

  /* {[tr + tSU;DAT(min)] / [tPRESC]} - 1 <= SCLDEL */
  tscldel_min = (int32_t)I2C_Charac[I2C_speed].trise +
                (int32_t)I2C_Charac[I2C_speed].sudat_min;

  if (tsdadel_min <= 0)
  {
    tsdadel_min = 0;
  }

  if (tsdadel_max <= 0)
  {
    tsdadel_max = 0;
  }

  for (presc = 0U; presc < I2C_PRESC_MAX; presc++)
  {
    for (scldel = 0U; scldel < I2C_SCLDEL_MAX; scldel++)
    {
      /* TSCLDEL = (SCLDEL+1) x (PRESC+1) x TI2CCLK */
      uint32_t tscldel = (scldel + 1U) * (presc + 1U) * ti2cclk;

      if (tscldel >= (uint32_t)tscldel_min)
      {
        for (sdadel = 0U; sdadel < I2C_SDADEL_MAX; sdadel++)
        {
          /* TSDADEL = SDADEL x (PRESC+1) x TI2CCLK */
          uint32_t tsdadel = (sdadel * (presc + 1U)) * ti2cclk;

          if ((tsdadel >= (uint32_t)tsdadel_min) && (tsdadel <= (uint32_t)tsdadel_max))
          {
            if (presc != prev_presc)
            {
              s_valid_timing[s_valid_timing_nbr].presc   = presc;
              s_valid_timing[s_valid_timing_nbr].tscldel = scldel;
              s_valid_timing[s_valid_timing_nbr].tsdadel = sdadel;
              prev_presc = presc;
              s_valid_timing_nbr++;

              if (s_valid_timing_nbr >= I2C_VALID_TIMING_NBR)
              {
                return;
              }
            }
          }
        }
      }
    }
  }
}

/* Index of the candidate whose SCL period is closest to the target, or 0xFFFFFFFF. */
static uint32_t I2C_ComputeSclPhases(uint32_t clock_src_freq, uint32_t I2C_speed)
{
  uint32_t ret = 0xFFFFFFFFU;
  uint32_t ti2cclk;
  uint32_t ti2cspeed;
  uint32_t prev_error;
  uint32_t dnf_delay;
  uint32_t clk_min;
  uint32_t clk_max;
  uint32_t count;
  uint32_t scll;
  uint32_t sclh;
  uint32_t tafdel_min;

  ti2cclk   = (SEC2NSEC + (clock_src_freq / 2U)) / clock_src_freq;
  ti2cspeed = (SEC2NSEC + (I2C_Charac[I2C_speed].freq / 2U)) / I2C_Charac[I2C_speed].freq;

  tafdel_min = I2C_ANALOG_FILTER_DELAY_MIN;

  /* tDNF = DNF x tI2CCLK */
  dnf_delay = I2C_Charac[I2C_speed].dnf * ti2cclk;

  clk_max = SEC2NSEC / I2C_Charac[I2C_speed].freq_min;
  clk_min = SEC2NSEC / I2C_Charac[I2C_speed].freq_max;

  prev_error = ti2cspeed;

  for (count = 0U; count < s_valid_timing_nbr; count++)
  {
    /* tPRESC = (PRESC+1) x tI2CCLK */
    uint32_t tpresc = (s_valid_timing[count].presc + 1U) * ti2cclk;

    for (scll = 0U; scll < I2C_SCLL_MAX; scll++)
    {
      /* tLOW(min) <= tAF(min) + tDNF + 2 x tI2CCLK + [(SCLL+1) x tPRESC] */
      uint32_t tscl_l = tafdel_min + dnf_delay + (2U * ti2cclk) + ((scll + 1U) * tpresc);

      /* tI2CCLK must satisfy tI2CCLK < (tLOW - tfilters) / 4 and tI2CCLK < tHIGH */
      if ((tscl_l > I2C_Charac[I2C_speed].lscl_min) &&
          (ti2cclk < ((tscl_l - tafdel_min - dnf_delay) / 4U)))
      {
        for (sclh = 0U; sclh < I2C_SCLH_MAX; sclh++)
        {
          /* tHIGH(min) <= tAF(min) + tDNF + 2 x tI2CCLK + [(SCLH+1) x tPRESC] */
          uint32_t tscl_h = tafdel_min + dnf_delay + (2U * ti2cclk) + ((sclh + 1U) * tpresc);

          /* tSCL = tf + tLOW + tr + tHIGH */
          uint32_t tscl = tscl_l + tscl_h +
                          I2C_Charac[I2C_speed].trise + I2C_Charac[I2C_speed].tfall;

          if ((tscl >= clk_min) && (tscl <= clk_max) &&
              (tscl_h >= I2C_Charac[I2C_speed].hscl_min) && (ti2cclk < tscl_h))
          {
            int32_t error = (int32_t)tscl - (int32_t)ti2cspeed;

            if (error < 0)
            {
              error = -error;
            }

            if ((uint32_t)error < prev_error)
            {
              prev_error = (uint32_t)error;
              s_valid_timing[count].scll = scll;
              s_valid_timing[count].sclh = sclh;
              ret = count;
            }
          }
        }
      }
    }
  }

  return ret;
}
