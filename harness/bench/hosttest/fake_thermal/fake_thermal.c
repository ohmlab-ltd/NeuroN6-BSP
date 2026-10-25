/* Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 *
 * The fake board the thermal host tests build neuron6_thermal.c against.
 * See fake_thermal.h. */
#include "fake_thermal.h"

#include "neuron6_thermal.h"
#include "neuron6.h"
#include "neuron6_bus.h"

/* The clock: milliseconds, advanced by waits, and by each reading when a
 * test asks (the first tick_slow readings by 1 ms, the rest by tick_step). */
uint32_t now;
uint32_t tick_slow;
uint32_t tick_step;

void HAL_Delay(uint32_t Delay) { now += Delay; }
int32_t BSP_GetTick(void)
{
  if (tick_slow > 0U)
  {
    tick_slow--;
    now += 1U;
  }
  else
  {
    now += tick_step;
  }
  return (int32_t)now;
}

/* The pins. */
GPIO_TypeDef fake_gpio[8];
DMA_Channel_TypeDef fake_dma_channel;
SPI_TypeDef fake_spi1;

int      power_high, reset_high;
int      power_ups;
uint32_t released_at;
int      camera_on;

static int      select_high;
static uint32_t select_high_since;
static uint32_t select_last_high_ms;

void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init) { (void)GPIOx; (void)GPIO_Init; }

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
  const int high = (PinState == GPIO_PIN_SET);

  if ((GPIOx == BSP_THERMAL_PWRDWN_GPIO_PORT) && (GPIO_Pin == BSP_THERMAL_PWRDWN_PIN))
  {
    power_high = high;
  }
  else if ((GPIOx == BSP_THERMAL_RESET_GPIO_PORT) && (GPIO_Pin == BSP_THERMAL_RESET_PIN))
  {
    if (high && !reset_high && power_high)
    {
      power_ups++;
      released_at = now;
    }
    reset_high = high;
  }
  else if ((GPIOx == BSP_THERMAL_CS_GPIO_PORT) && (GPIO_Pin == BSP_THERMAL_CS_PIN))
  {
    if (high && !select_high)
    {
      select_high_since = now;
    }
    if (!high && select_high)
    {
      select_last_high_ms = now - select_high_since;
    }
    select_high = high;
    return;
  }
  else
  {
    return;
  }
  camera_on = power_high && reset_high;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  (void)GPIOx; (void)GPIO_Pin;
  return GPIO_PIN_RESET;
}

/* The camera on the bus. */
int      present = 1;
int      disturbed;
int      transactions;
uint32_t first_transaction_at;

static void bus_transaction(void)
{
  if (transactions == 0)
  {
    first_transaction_at = now;
  }
  transactions++;
  if (camera_on && ((now - released_at) < BOOT_DISTURBED_MS))
  {
    disturbed = 1;
  }
}

int32_t BSP_I2C1_Init(void)   { return 0; }
int32_t BSP_I2C1_DeInit(void) { return 0; }
void    BSP_I2C1_Lock(void)   { }
void    BSP_I2C1_Unlock(void) { }

int32_t BSP_I2C1_IsReady(uint16_t DevAddr, uint32_t Trials)
{
  (void)DevAddr; (void)Trials;
  bus_transaction();
  return (present && camera_on) ? 0 : -1;
}

int32_t BSP_I2C1_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  (void)DevAddr; (void)Reg; (void)pData; (void)Length;
  bus_transaction();
  return (present && camera_on) ? 0 : -1;
}

int32_t BSP_I2C1_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  uint16_t value = 0U;

  (void)DevAddr;
  bus_transaction();
  if (!present || !camera_on)
  {
    return -1;                       /* no acknowledge */
  }
  if ((Reg == LEPTON_REG_STATUS) && !disturbed && ((now - released_at) >= BOOT_TAKES_MS))
  {
    value = LEPTON_STATUS_BOOT_MODE | LEPTON_STATUS_BOOT_STATUS;
  }
  if (Length >= 2U)
  {
    pData[0] = (uint8_t)(value >> 8);
    pData[1] = (uint8_t)value;
  }
  return 0;
}

/* The video link. */
int      captures;
uint32_t idle_before_capture;
uint8_t *capture_buffer;
uint16_t capture_bytes;

HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *h, uint8_t *p, uint16_t n)
{
  (void)h;
  captures++;
  idle_before_capture = select_last_high_ms;
  capture_buffer = p;
  capture_bytes = n;
  return HAL_OK;
}

/* The rest of the HAL the module links against. */
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *p) { (void)p; return HAL_OK; }
uint32_t HAL_RCC_GetPCLK2Freq(void) { return 200000000U; }
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t a, uint32_t b) { (void)IRQn; (void)a; (void)b; }
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn) { (void)IRQn; }
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn) { (void)IRQn; }
HAL_StatusTypeDef HAL_DMAEx_List_BuildNode(DMA_NodeConfTypeDef *c, DMA_NodeTypeDef *n) { (void)c; (void)n; return HAL_OK; }
HAL_StatusTypeDef HAL_DMAEx_List_InsertNode_Tail(DMA_QListTypeDef *q, DMA_NodeTypeDef *n) { (void)q; (void)n; return HAL_OK; }
HAL_StatusTypeDef HAL_DMAEx_List_SetCircularMode(DMA_QListTypeDef *q) { (void)q; return HAL_OK; }
HAL_StatusTypeDef HAL_DMAEx_List_Init(DMA_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_DMAEx_List_LinkQ(DMA_HandleTypeDef *h, DMA_QListTypeDef *q) { (void)h; (void)q; return HAL_OK; }
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *h) { (void)h; }
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *h) { (void)h; return HAL_OK; }
void HAL_SPI_IRQHandler(SPI_HandleTypeDef *h) { (void)h; }

void fake_reset(uint32_t Start)
{
  now = Start;
  tick_slow = 0U;
  tick_step = 0U;
  power_high = reset_high = camera_on = 0;
  power_ups = 0;
  released_at = 0U;
  present = 1;
  disturbed = 0;
  transactions = 0;
  first_transaction_at = 0U;
  select_high = 0;
  select_high_since = Start;
  select_last_high_ms = 0U;
  captures = 0;
  idle_before_capture = 0U;
  capture_buffer = NULL;
  capture_bytes = 0U;
}
