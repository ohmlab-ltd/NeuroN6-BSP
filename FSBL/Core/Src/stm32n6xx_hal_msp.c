/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32n6xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  /* System interrupt init*/

  HAL_PWREx_EnableVddIO3();
  HAL_PWREx_ConfigVddIORange(PWR_VDDIO3,PWR_VDDIO_RANGE_1V8);

  HAL_PWREx_EnableVddIO4();
  HAL_PWREx_ConfigVddIORange(PWR_VDDIO4,PWR_VDDIO_RANGE_3V3);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

static uint32_t HAL_RCC_XSPIM_CLK_ENABLED=0;

/**
  * @brief XSPI MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hxspi: XSPI handle pointer
  * @retval None
  */
void HAL_XSPI_MspInit(XSPI_HandleTypeDef* hxspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hxspi->Instance==XSPI1)
  {
    /* USER CODE BEGIN XSPI1_MspInit 0 */

    /* USER CODE END XSPI1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_XSPI1;
    PeriphClkInitStruct.Xspi1ClockSelection = RCC_XSPI1CLKSOURCE_HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    HAL_RCC_XSPIM_CLK_ENABLED++;
    if(HAL_RCC_XSPIM_CLK_ENABLED==1){
      __HAL_RCC_XSPIM_CLK_ENABLE();
    }
    __HAL_RCC_XSPI1_CLK_ENABLE();

    __HAL_RCC_GPION_CLK_ENABLE();
    /**XSPI1 GPIO Configuration
    PN4     ------> XSPIM_P2_IO2
    PN8     ------> XSPIM_P2_IO4
    PN6     ------> XSPIM_P2_CLK
    PN5     ------> XSPIM_P2_IO3
    PN2     ------> XSPIM_P2_IO0
    PN9     ------> XSPIM_P2_IO5
    PN0     ------> XSPIM_P2_DQS0
    PN10     ------> XSPIM_P2_IO6
    PN3     ------> XSPIM_P2_IO1
    PN11     ------> XSPIM_P2_IO7
    PN1     ------> XSPIM_P2_NCS1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_6|GPIO_PIN_5
                          |GPIO_PIN_2|GPIO_PIN_9|GPIO_PIN_0|GPIO_PIN_10
                          |GPIO_PIN_3|GPIO_PIN_11|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_XSPIM_P2;
    HAL_GPIO_Init(GPION, &GPIO_InitStruct);

    /* USER CODE BEGIN XSPI1_MspInit 1 */

    /* USER CODE END XSPI1_MspInit 1 */
  }
  else if(hxspi->Instance==XSPI2)
  {
    /* USER CODE BEGIN XSPI2_MspInit 0 */

    /* USER CODE END XSPI2_MspInit 0 */

  /** Initializes the peripherals clock
  */

	    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_XSPI2;
	    PeriphClkInitStruct.Xspi2ClockSelection = RCC_XSPI2CLKSOURCE_IC3;
	    PeriphClkInitStruct.ICSelection[RCC_IC3].ClockSelection = RCC_ICCLKSOURCE_PLL4;
	    PeriphClkInitStruct.ICSelection[RCC_IC3].ClockDivider = 2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    HAL_RCC_XSPIM_CLK_ENABLED++;
    if(HAL_RCC_XSPIM_CLK_ENABLED==1){
      __HAL_RCC_XSPIM_CLK_ENABLE();
    }
    __HAL_RCC_XSPI2_CLK_ENABLE();

    __HAL_RCC_GPION_CLK_ENABLE();
    /**XSPI2 GPIO Configuration
    PN12     ------> XSPIM_P2_NCS2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_XSPIM_P2;
    HAL_GPIO_Init(GPION, &GPIO_InitStruct);

    /* USER CODE BEGIN XSPI2_MspInit 1 */

    /* USER CODE END XSPI2_MspInit 1 */
  }

}

/**
  * @brief XSPI MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hxspi: XSPI handle pointer
  * @retval None
  */
void HAL_XSPI_MspDeInit(XSPI_HandleTypeDef* hxspi)
{
  if(hxspi->Instance==XSPI1)
  {
    /* USER CODE BEGIN XSPI1_MspDeInit 0 */

    /* USER CODE END XSPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_XSPIM_CLK_ENABLED--;
    if(HAL_RCC_XSPIM_CLK_ENABLED==0){
      __HAL_RCC_XSPIM_CLK_DISABLE();
    }
    __HAL_RCC_XSPI1_CLK_DISABLE();

    /**XSPI1 GPIO Configuration
    PN4     ------> XSPIM_P2_IO2
    PN8     ------> XSPIM_P2_IO4
    PN6     ------> XSPIM_P2_CLK
    PN5     ------> XSPIM_P2_IO3
    PN2     ------> XSPIM_P2_IO0
    PN9     ------> XSPIM_P2_IO5
    PN0     ------> XSPIM_P2_DQS0
    PN10     ------> XSPIM_P2_IO6
    PN3     ------> XSPIM_P2_IO1
    PN11     ------> XSPIM_P2_IO7
    PN1     ------> XSPIM_P2_NCS1
    */
    HAL_GPIO_DeInit(GPION, GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_6|GPIO_PIN_5
                          |GPIO_PIN_2|GPIO_PIN_9|GPIO_PIN_0|GPIO_PIN_10
                          |GPIO_PIN_3|GPIO_PIN_11|GPIO_PIN_1);

    /* USER CODE BEGIN XSPI1_MspDeInit 1 */

    /* USER CODE END XSPI1_MspDeInit 1 */
  }
  else if(hxspi->Instance==XSPI2)
  {
    /* USER CODE BEGIN XSPI2_MspDeInit 0 */

    /* USER CODE END XSPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_XSPIM_CLK_ENABLED--;
    if(HAL_RCC_XSPIM_CLK_ENABLED==0){
      __HAL_RCC_XSPIM_CLK_DISABLE();
    }
    __HAL_RCC_XSPI2_CLK_DISABLE();

    /**XSPI2 GPIO Configuration
    PN12     ------> XSPIM_P2_NCS2
    */
    HAL_GPIO_DeInit(GPION, GPIO_PIN_12);

    /* USER CODE BEGIN XSPI2_MspDeInit 1 */

    /* USER CODE END XSPI2_MspDeInit 1 */
  }

}


/**
* @brief RTC MSP Initialization
* This function configures the hardware resources used in this example
* @param hrtc: RTC handle pointer
* @retval None
*/
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hrtc->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */
    /* Enables access to the backup domain */
    /* To enable access on RTC registers */
    HAL_PWR_EnableBkUpAccess();

  /* USER CODE END RTC_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_RTCAPB_CLK_ENABLE();
    __HAL_RCC_RTC_CLK_ENABLE();
    /* RTC interrupt Init */
    HAL_NVIC_SetPriority(TAMP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TAMP_IRQn);
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */

  }

}

/**
* @brief RTC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hrtc: RTC handle pointer
* @retval None
*/
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTCAPB_CLK_DISABLE();
    __HAL_RCC_RTC_CLK_DISABLE();

    /* RTC interrupt DeInit */
    HAL_NVIC_DisableIRQ(TAMP_IRQn);
  /* USER CODE BEGIN RTC_MspDeInit 1 */
    HAL_NVIC_DisableIRQ(TAMP_IRQn);
  /* USER CODE END RTC_MspDeInit 1 */
  }

}
/* USER CODE END 1 */


/* USER CODE BEGIN 1 */

/**
  * @brief PCD MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hpcd: PCD handle pointer
  * @retval None
  */
void HAL_PCD_MspInit(PCD_HandleTypeDef* hpcd)
{
  if (hpcd->Instance == USB1_OTG_HS)
  {
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /* USER CODE BEGIN USB1_OTG_HS_MspInit 0 */

    /* 1. Configure OTGHS1 and PHY1 clocks */

    /* OTGHS1 clock source: from PHY1 reference clock */
    PeriphClkInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_USBOTGHS1;
    PeriphClkInitStruct.UsbOtgHs1ClockSelection = RCC_USBPHY1CLKSOURCE_HSE_DIRECT;  // or _OTGPHY1 depending on your HSE config
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* PHY1 reference clock source: from HSE (or HSE/2) */
    PeriphClkInitStruct.PeriphClockSelection   = RCC_PERIPHCLK_USBPHY1;
    PeriphClkInitStruct.UsbPhy1ClockSelection  = RCC_USBPHY1CLKSOURCE_HSE_DIV2;      // typical for 48 MHz HSE
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* 2. Reset controller + PHY in proper order */

    __HAL_RCC_GPIOA_CLK_ENABLE();  // if your USB pins are on GPIOA; adjust if needed

    // ST's N6 example uses this AHB5 reset mask
    LL_AHB5_GRP1_ForceReset(0x00800000);

    __HAL_RCC_USB1_OTG_HS_FORCE_RESET();
    __HAL_RCC_USB1_OTG_HS_PHY_FORCE_RESET();

    LL_RCC_HSE_SelectHSEDiv2AsDiv2Clock();

    LL_AHB5_GRP1_ReleaseReset(0x00800000);

    /* 3. Enable OTGHS1 clock */
    __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();

    /* Required few clock cycles before accessing USB PHY Controller Registers */
    for (volatile uint32_t i = 0; i < 10; i++) {
        __NOP(); // No Operation instruction to create a delay
    }  // or a small NOP loop

    /* 4. Configure HS PHY registers (24 MHz ref in this example) */

    USB1_HS_PHYC->USBPHYC_CR &= ~(0x7U << 4);      // clear FSEL bits
    USB1_HS_PHYC->USBPHYC_CR |=
          (0x1U << 16)  // ENUTMILEVEL2
        | (0x2U << 4)   // FSEL = 24 MHz
        | (0x1U << 2)   // ENHSE
        | 0x1U;         // PHYEN

    __HAL_RCC_USB1_OTG_HS_PHY_RELEASE_RESET();

    /* Required few clock cycles before releasing reset */
    for (volatile uint32_t i = 0; i < 10; i++) {
        __NOP(); // No Operation instruction to create a delay
    }

    __HAL_RCC_USB1_OTG_HS_RELEASE_RESET();

    /* 5. Peripheral PHY clock enable */
    __HAL_RCC_USB1_OTG_HS_PHY_CLK_ENABLE();

    /* 6. Power & NVIC */

    HAL_PWREx_EnableVddUSB();

    HAL_NVIC_SetPriority(USB1_OTG_HS_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB1_OTG_HS_IRQn);

    /* USER CODE END USB1_OTG_HS_MspInit 0 */

    /* USER CODE BEGIN USB1_OTG_HS_MspInit 1 */
    /* (no extra Cube-generated code needed here for N6; USB pins are auto-configured) */
    /* USER CODE END USB1_OTG_HS_MspInit 1 */
  }
}

/**
  * @brief PCD MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hpcd: PCD handle pointer
  * @retval None
  */
void HAL_PCD_MspDeInit(PCD_HandleTypeDef* hpcd)
{
  if(hpcd->Instance==USB1_OTG_HS)
  {
    /* USER CODE BEGIN USB1_OTG_HS_MspDeInit 0 */

    /* USER CODE END USB1_OTG_HS_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USB1_OTG_HS_CLK_DISABLE();
    __HAL_RCC_USB1_OTG_HS_PHY_CLK_DISABLE();

    /* Disable VDDUSB */
      HAL_PWREx_DisableVddUSB();

    /* USB1_OTG_HS interrupt DeInit */

    /* USER CODE BEGIN USB1_OTG_HS_MspDeInit 1 */

    /* USER CODE END USB1_OTG_HS_MspDeInit 1 */
  }

}

/* USER CODE END 1 */
