/**
  * @file    neuron6.c
  * @brief   Neuro N6 board core: BSP identity, isolation, LEDs and the user button.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * All rights reserved.
  * SPDX-License-Identifier: BSD-3-Clause
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  */

#include "neuron6.h"

static void LED_Write(Led_TypeDef Led, uint32_t Lit);
static void BUTTON_USER_EXTI_Callback(void);

static EXTI_HandleTypeDef hpb_exti[BUTTONn];

static uint8_t PB_ExtiArmed[BUTTONn];

static GPIO_TypeDef *const LED_PORT[LEDn] =
{
  LED1_GPIO_PORT,
  LED2_GPIO_PORT,
  LED3_GPIO_PORT,
};

static const uint16_t LED_PIN[LEDn] =
{
  LED1_PIN,
  LED2_PIN,
  LED3_PIN,
};

static GPIO_TypeDef *const BUTTON_PORT[BUTTONn] =
{
  BUTTON_USER_GPIO_PORT,
};

static const uint16_t BUTTON_PIN[BUTTONn] =
{
  BUTTON_USER_PIN,
};

static const IRQn_Type BUTTON_IRQn[BUTTONn] =
{
  BUTTON_USER_EXTI_IRQn,
};

static const uint32_t BUTTON_EXTI_LINE[BUTTONn] =
{
  BUTTON_USER_EXTI_LINE,
};

/* Common anode: a low pin lights the LED. Led is range-checked by the caller. */
static void LED_Write(Led_TypeDef Led, uint32_t Lit)
{
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led],
                    (Lit != LED_DARK) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void BUTTON_USER_EXTI_Callback(void)
{
  BSP_PB_Callback(BUTTON_USER);
}

/* Open the memory firewalls (RISAF) to every compartment; values as the board ships. */
static void BOARD_RisafConfig(void)
{
  RISAF_TypeDef *const firewall[] = { RISAF2, RISAF3, RISAF7, RISAF12 };
  uint32_t i;

  for (i = 0U; i < (sizeof(firewall) / sizeof(firewall[0])); i++)
  {
    firewall[i]->REG[0].CFGR    = 0x00000000U;
    firewall[i]->REG[1].CFGR    = 0x00000000U;

    firewall[i]->REG[0].CIDCFGR = 0x000F000FU;
    firewall[i]->REG[0].ENDR    = 0xFFFFFFFFU;
    firewall[i]->REG[0].CFGR    = 0x00000101U;

    firewall[i]->REG[1].CIDCFGR = 0x00FF00FFU;
    firewall[i]->REG[1].ENDR    = 0xFFFFFFFFU;
    firewall[i]->REG[1].CFGR    = 0x00000001U;
  }
}

/* Put every bus master (RIMC) and shared peripheral (RISC) in one compartment. */
static void BOARD_IsolationConfig(void)
{
  RIMC_MasterConfig_t master = {0};

#if defined(CPU_IN_SECURE_STATE)
  __HAL_RCC_RIFSC_CLK_ENABLE();
#endif /* CPU_IN_SECURE_STATE */

  master.MasterCID = RIF_CID_1;
  master.SecPriv   = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;

  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_NPU,    &master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_DMA2D,  &master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_DCMIPP, &master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_LTDC1,  &master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_LTDC2,  &master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_VENC,   &master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_OTG1,   &master);

  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_NPU,
                                        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_DMA2D,
                                        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_CSI,
                                        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_DCMIPP,
                                        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDC,
                                        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDCL1,
                                        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDCL2,
                                        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_VENC,
                                        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);

  /* ADC12 too: without it the (secure) analogue pads are not routed to the converter. */
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_ADC12,
                                        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);

  BOARD_RisafConfig();

#if defined(CPU_IN_SECURE_STATE)
  /* The PWR supply configuration and the five pins the boot loader leaves owned: same
   * attributes, so a later write from this compartment is not refused. */
  HAL_PWR_ConfigAttributes(PWR_ITEM_0, PWR_SEC_NPRIV);

  HAL_GPIO_ConfigPinAttributes(GPIOC, GPIO_PIN_1, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOD, GPIO_PIN_2, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOD, GPIO_PIN_6, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOE, GPIO_PIN_5, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOE, GPIO_PIN_6, GPIO_PIN_SEC | GPIO_PIN_NPRIV);

  /* The IAC latches a refused access instead of letting it vanish. */
  __HAL_RCC_IAC_CLK_ENABLE();
  __HAL_RCC_IAC_FORCE_RESET();
  __HAL_RCC_IAC_RELEASE_RESET();
#endif /* CPU_IN_SECURE_STATE */
}

/* Power up AXISRAM3-6, which the boot loader leaves shut down. Skipped without the
 * RAMCFG HAL module, for projects that use only AXISRAM1-2. */
static void BOARD_AxisramEnable(void)
{
#if defined(HAL_RAMCFG_MODULE_ENABLED)
  RAMCFG_HandleTypeDef ramcfg = {0};

  __HAL_RCC_AXISRAM3_MEM_CLK_ENABLE();
  __HAL_RCC_AXISRAM4_MEM_CLK_ENABLE();
  __HAL_RCC_AXISRAM5_MEM_CLK_ENABLE();
  __HAL_RCC_AXISRAM6_MEM_CLK_ENABLE();
  __HAL_RCC_RAMCFG_CLK_ENABLE();

  ramcfg.Instance = RAMCFG_SRAM3_AXI;
  (void)HAL_RAMCFG_EnableAXISRAM(&ramcfg);
  ramcfg.Instance = RAMCFG_SRAM4_AXI;
  (void)HAL_RAMCFG_EnableAXISRAM(&ramcfg);
  ramcfg.Instance = RAMCFG_SRAM5_AXI;
  (void)HAL_RAMCFG_EnableAXISRAM(&ramcfg);
  ramcfg.Instance = RAMCFG_SRAM6_AXI;
  (void)HAL_RAMCFG_EnableAXISRAM(&ramcfg);
#endif /* HAL_RAMCFG_MODULE_ENABLED */
}

/**
  * @brief  Board-wide isolation: memory firewalls, bus-master compartments, AXI RAM banks.
  * @note   BSP modules call it too; an application with its own policy sets
  *         BSP_ISOLATION_APPLICATION_OWNED in neuron6_conf.h to make it a no-op.
  */
int32_t BSP_ISOLATION_Config(void)
{
#if (BSP_ISOLATION_APPLICATION_OWNED == 0U)
  static uint8_t configured = 0U;

  if (configured != 0U)
  {
    return BSP_ERROR_NONE;
  }

  BOARD_IsolationConfig();
  BOARD_AxisramEnable();

  configured = 1U;
#endif /* BSP_ISOLATION_APPLICATION_OWNED */

  return BSP_ERROR_NONE;
}

/** @brief BSP version: main, sub1, sub2 and RC in bytes 3 to 0. */
int32_t BSP_GetVersion(void)
{
  return (int32_t)NEURON6_BSP_VERSION;
}

const uint8_t *BSP_GetBoardName(void)
{
  return (const uint8_t *)NEURON6_BSP_BOARD_NAME;
}

const uint8_t *BSP_GetBoardID(void)
{
  return (const uint8_t *)NEURON6_BSP_BOARD_ID;
}

/** @brief HAL tick in ms, returned as int32_t: the time base component drivers take. */
int32_t BSP_GetTick(void)
{
  return (int32_t)HAL_GetTick();
}

/** @brief Configure the LED's pin as an output, LED dark. */
int32_t BSP_LED_Init(Led_TypeDef Led)
{
  int32_t ret = BSP_ERROR_NONE;
  GPIO_InitTypeDef gpio_init_structure = {0};

  switch (Led)
  {
    case LED1:
      LED1_GPIO_CLK_ENABLE();
      break;

    case LED2:
      LED2_GPIO_CLK_ENABLE();
      break;

    case LED3:
      LED3_GPIO_CLK_ENABLE();
      break;

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  if (ret == BSP_ERROR_NONE)
  {
    /* Dark before the pin becomes an output, so the LED never flashes on init. */
    LED_Write(Led, LED_DARK);

    gpio_init_structure.Pin   = LED_PIN[Led];
    gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT[Led], &gpio_init_structure);

    LED_Write(Led, LED_DARK);
  }

  return ret;
}

/** @brief Release the LED's pin; the port clock stays on (it may be shared). */
int32_t BSP_LED_DeInit(Led_TypeDef Led)
{
  if ((uint32_t)Led >= (uint32_t)LEDn)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  LED_Write(Led, LED_DARK);
  HAL_GPIO_DeInit(LED_PORT[Led], LED_PIN[Led]);

  return BSP_ERROR_NONE;
}

int32_t BSP_LED_On(Led_TypeDef Led)
{
  if ((uint32_t)Led >= (uint32_t)LEDn)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  LED_Write(Led, LED_LIT);

  return BSP_ERROR_NONE;
}

int32_t BSP_LED_Off(Led_TypeDef Led)
{
  if ((uint32_t)Led >= (uint32_t)LEDn)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  LED_Write(Led, LED_DARK);

  return BSP_ERROR_NONE;
}

int32_t BSP_LED_Toggle(Led_TypeDef Led)
{
  if ((uint32_t)Led >= (uint32_t)LEDn)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  HAL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);

  return BSP_ERROR_NONE;
}

/** @brief LED_LIT or LED_DARK (the light, not the pin level), or a negative error. */
int32_t BSP_LED_GetState(Led_TypeDef Led)
{
  GPIO_PinState pin_state;

  if ((uint32_t)Led >= (uint32_t)LEDn)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  pin_state = HAL_GPIO_ReadPin(LED_PORT[Led], LED_PIN[Led]);

  return (pin_state == GPIO_PIN_RESET) ? (int32_t)LED_LIT : (int32_t)LED_DARK;
}

/** @brief BUTTON_MODE_GPIO: polled. BUTTON_MODE_EXTI: BSP_PB_Callback on each press. */
int32_t BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  GPIO_InitTypeDef gpio_init_structure = {0};

  if ((uint32_t)Button >= (uint32_t)BUTTONn)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  BUTTON_USER_GPIO_CLK_ENABLE();

  gpio_init_structure.Pin   = BUTTON_PIN[Button];
  gpio_init_structure.Pull  = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;

  if (ButtonMode == BUTTON_MODE_GPIO)
  {
    /* Reconfiguring as an input leaves an earlier EXTI arming in place: take it down
     * and drop any latched edge. */
    if (PB_ExtiArmed[Button] != 0U)
    {
      PB_ExtiArmed[Button] = 0U;
      HAL_NVIC_DisableIRQ(BUTTON_IRQn[Button]);
      HAL_GPIO_DeInit(BUTTON_PORT[Button], BUTTON_PIN[Button]);
      __HAL_GPIO_EXTI_CLEAR_IT(BUTTON_PIN[Button]);
      HAL_NVIC_ClearPendingIRQ(BUTTON_IRQn[Button]);
    }
    gpio_init_structure.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(BUTTON_PORT[Button], &gpio_init_structure);
  }
  else if (ButtonMode == BUTTON_MODE_EXTI)
  {
    /* Pressed reads HIGH, so the press edge is the rising one. */
    gpio_init_structure.Mode = GPIO_MODE_IT_RISING;
    HAL_GPIO_Init(BUTTON_PORT[Button], &gpio_init_structure);

    if (HAL_EXTI_GetHandle(&hpb_exti[Button], BUTTON_EXTI_LINE[Button]) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }

    if (HAL_EXTI_RegisterCallback(&hpb_exti[Button], HAL_EXTI_COMMON_CB_ID,
                                  BUTTON_USER_EXTI_Callback) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }

    PB_ExtiArmed[Button] = 1U;
    HAL_NVIC_SetPriority(BUTTON_IRQn[Button], BSP_BUTTON_IT_PRIORITY, 0x00U);
    HAL_NVIC_EnableIRQ(BUTTON_IRQn[Button]);
  }
  else
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  return BSP_ERROR_NONE;
}

/** @brief Release the button's pin and interrupt; the port clock stays on. */
int32_t BSP_PB_DeInit(Button_TypeDef Button)
{
  if ((uint32_t)Button >= (uint32_t)BUTTONn)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  PB_ExtiArmed[Button] = 0U;
  HAL_NVIC_DisableIRQ(BUTTON_IRQn[Button]);
  HAL_GPIO_DeInit(BUTTON_PORT[Button], BUTTON_PIN[Button]);

  return BSP_ERROR_NONE;
}

/** @brief BUTTON_PRESSED or BUTTON_RELEASED, or a negative error. */
int32_t BSP_PB_GetState(Button_TypeDef Button)
{
  GPIO_PinState pin_state;

  if ((uint32_t)Button >= (uint32_t)BUTTONn)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  pin_state = HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);

  return (pin_state == GPIO_PIN_SET) ? (int32_t)BUTTON_PRESSED
                                     : (int32_t)BUTTON_RELEASED;
}

/**
  * @brief  Call from the application's EXTI8_IRQHandler; acts only in BUTTON_MODE_EXTI.
  * @note   Line 8 is shared with header pin 28: call BSP_IO_IRQHandler(8) after this;
  *         whichever owns the line clears it.
  */
void BSP_PB_IRQHandler(Button_TypeDef Button)
{
  if (((uint32_t)Button < (uint32_t)BUTTONn) && (PB_ExtiArmed[Button] != 0U))
  {
    HAL_EXTI_IRQHandler(&hpb_exti[Button]);
  }
}

/** @brief Weak; override to react to presses. Runs in interrupt context. */
__weak void BSP_PB_Callback(Button_TypeDef Button)
{
  UNUSED(Button);
}
