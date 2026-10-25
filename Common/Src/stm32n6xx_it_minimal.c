/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32n6xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32n6xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "neuron6.h"
#include "neuron6_com.h"
#include "neuron6_camera.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "neuron6_com.h"

/* Where the board goes once a fault's breadcrumb is written: the bootloader,
 * so it comes back on the bus flashable and says what it was. An application
 * that has to recover by itself, one left running unattended, defines this to
 * restart instead. It runs in the fault handler, is given the breadcrumb, and
 * must not return. */
__weak void BSP_FaultCallback(uint32_t Breadcrumb)
{
  (void)Breadcrumb;
  BSP_COM_RebootToBootloader();
}

/* A fault leaves a code in backup register 7 and goes to BSP_FaultCallback,
 * the bootloader unless the application says otherwise. The top
 * byte is 0xF0 plus the cause: 1 HardFault, 2 MemManage, 3 BusFault,
 * 4 UsageFault, 5 a task's stack overflow, 6 a failed configASSERT,
 * 7 SecureFault, 8 a non-maskable interrupt. Bit 23 is
 * HFSR.FORCED, set when a fault escalated to HardFault; bits 22..16 are the
 * UsageFault status (UFSR bits 0..4, then 8 and 9), and bits 15..0 the
 * MemManage and BusFault status. An interrupt with no handler leaves 0xFB. */
static void fault_leave(uint32_t which)
{
  volatile uint32_t *bkp = &TAMP->BKP0R;
  const uint32_t cfsr = SCB->CFSR;
  const uint32_t ufsr = cfsr >> 16U;

  HAL_PWR_EnableBkUpAccess();
  bkp[7] = ((0xF0U | which) << 24U) |
           (((SCB->HFSR & SCB_HFSR_FORCED_Msk) != 0U) ? (1UL << 23U) : 0U) |
           ((ufsr & 0x1FU) << 16U) | (((ufsr >> 8U) & 0x3U) << 21U) |
           (cfsr & 0xFFFFU);
  BSP_FaultCallback(bkp[7]);
}
/* An interrupt with no handler: save which one and leave as a fault does,
 * rather than spin in the startup file with USB dead. */
void Default_Handler_C(void)
{
  volatile uint32_t *bkp = &TAMP->BKP0R;
  HAL_PWR_EnableBkUpAccess();
  bkp[7] = 0xFB000000U | (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk);
  BSP_FaultCallback(bkp[7]);
  /* A callback that returns leaves the interrupt pending, which would
   * bring the processor straight back here: it waits for the watchdog. */
  for (;;)
  {
  }
}

#if defined(BSP_USE_FREERTOS) && (BSP_USE_FREERTOS == 1)
#include "neuron6_rtos.h"
/* The kernel found something it cannot continue from: a task ran off its
 * stack, or a configASSERT failed. Same treatment as a fault - a class byte
 * in the breadcrumb, then BSP_FaultCallback - with 0xF5 for an overflow and
 * 0xF6 for an assert, so the console's bkp afterwards says which. */
void BSP_RTOS_FatalHook(const char *pWhat, const char *pDetail)
{
  (void)pDetail;
  fault_leave(((pWhat != NULL) && (pWhat[0] == 's')) ? 5U : 6U);
}
#endif
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  fault_leave(8U);
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  fault_leave(1U);
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  fault_leave(2U);
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  fault_leave(3U);
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  fault_leave(4U);
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Secure fault.
  */
void SecureFault_Handler(void)
{
  /* USER CODE BEGIN SecureFault_IRQn 0 */
  fault_leave(7U);
  /* USER CODE END SecureFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_SecureFault_IRQn 0 */
    /* USER CODE END W1_SecureFault_IRQn 0 */
  }
}

/* Under FreeRTOS the port defines SVC_Handler, PendSV_Handler and
 * SysTick_Handler itself, and the HAL tick runs on TIM16 instead - see
 * Utilities/freertos/stm32n6xx_hal_timebase_tim.c. The three below exist
 * only in the bare-metal build, and TIM16's vector only in the other. */
#if !defined(BSP_USE_FREERTOS) || (BSP_USE_FREERTOS == 0)
/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}
#endif

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

#if !defined(BSP_USE_FREERTOS) || (BSP_USE_FREERTOS == 0)
/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}
#else
/* The HAL tick, on TIM16 because the kernel has SysTick. The handle is the
 * time base file's. */
extern TIM_HandleTypeDef htim16;
void TIM16_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim16);
}
#endif

/******************************************************************************/
/* STM32N6xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32n6xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
/* The BSP enables these lines and sets their priorities, but it cannot install
 * the vectors: the vector table belongs to the application.
 *
 * There is no VENC_IRQHandler here because there does not need to be: the
 * encoder's synchronisation back end defines it in
 * Components/venc/EWL/ewl_impl.c, and the startup file's weak vector picks
 * that definition up. Adding one here would take the vector away from the
 * encoder. */

/**
  * @brief  Camera pipe events: frame complete, line event, errors.
  *
  *         The line event is the one that matters here. It paces the encoder,
  *         and does that in silicon whether or not this handler exists - the
  *         vector is installed so the BSP can also deliver it to
  *         BSP_CAMERA_LineEventCallback, where main.c counts the camera's
  *         progress for its tear diagnostics.
  */
void DCMIPP_IRQHandler(void)
{
  BSP_CAMERA_DCMIPP_IRQHandler(0);
}

/**
  * @brief  CSI receiver errors.
  *
  *         Without this vector a link error latches the peripheral into an
  *         error state that silently swallows later configuration writes, and
  *         the camera stops producing frames for no visible reason.
  */
void CSI_IRQHandler(void)
{
  BSP_CAMERA_CSI_IRQHandler(0);
}

/**
  * @brief  USB device.
  */
void USB1_OTG_HS_IRQHandler(void)
{
  BSP_COM_IRQHandler();
}

#include "neuron6_wdg.h"

/* Watchdog early warning: 300 ms before the reset, whatever context is
 * stuck gets interrupted and the reset is pointed at the bootloader. */
void IWDG_IRQHandler(void)
{
  BSP_WDG_IRQHandler();
}

/* USER CODE END 1 */
