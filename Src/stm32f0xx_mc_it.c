
/**
  ******************************************************************************
  * @file    stm32f0xx_mc_it.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt
  *          service routine related to Motor Control for the STM32F0 Family.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup STM32F0xx_IRQ_Handlers
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_config.h"
#include "mc_type.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32F0xx_IRQ_Handlers STM32F0xx IRQ Handlers
  * @{
  */

/* USER CODE BEGIN PRIVATE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */

/* Public prototypes of IRQ handlers called from assembly code ---------------*/
void PERIOD_COMM_IRQHandler(void);
void BEMF_READING_IRQHandler(void);
void TIMx_BRK_M1_IRQHandler(void);
void SPD_TIM_M1_IRQHandler(void);

/**
  * @brief  This function handles TIMx Break-in interrupt request.
  * @param  None
  */
void TIMx_BRK_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_UP_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_UP_BRK_M1_IRQn 0 */

  if(LL_TIM_IsActiveFlag_BRK(TIM1) && LL_TIM_IsEnabledIT_BRK(TIM1))
  {
    LL_TIM_ClearFlag_BRK(TIM1);
    PWMC_BRK_IRQHandler(&PWM_Handle_M1);

    /* USER CODE BEGIN Break */

    /* USER CODE END Break */
  }
  else
  {
    /* No other interrupts are routed to this handler */
  }
  /* USER CODE BEGIN TIMx_UP_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_UP_BRK_M1_IRQn 1 */
}

/**
  * @brief  This function handles current sensing interrupt request.
  * @param[in] None
  */
void CURRENT_SENSE_IRQHandler(void)
{
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 0 */

  /* USER CODE END CURRENT_REGULATION_IRQn 0 */

  if(LL_ADC_IsActiveFlag_EOC(CurrMonitor_M1.regADC) && LL_ADC_IsEnabledIT_EOC(CurrMonitor_M1.regADC))
  {
    /* Clear Flags */
    LL_ADC_ClearFlag_EOC(CurrMonitor_M1.regADC);
    RCM_ReadCurrentMonitor(&CurrMonitor_M1);
  }
  else
  {
    /* Nothing to do */
  }

  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 2 */

  /* USER CODE END CURRENT_REGULATION_IRQn 2 */
}

/**
  * @brief  This function handles TIMx global interrupt request for M1 Speed Sensor.
  * @param  None
  */
void SPD_TIM_M1_IRQHandler(void)
{
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 0 */

  /* USER CODE END SPD_TIM_M1_IRQn 0 */

  /* HALL Timer Update IT always enabled, no need to check enable UPDATE state */
  if (LL_TIM_IsActiveFlag_UPDATE(HALL_M1.TIMx) != 0)
  {
    LL_TIM_ClearFlag_UPDATE(HALL_M1.TIMx);
    HALL_TIMx_UP_IRQHandler(&HALL_M1);

    /* USER CODE BEGIN HALL_Update */

    /* USER CODE END HALL_Update   */
  }
  else
  {
    /* Nothing to do */
  }

  /* HALL Timer CC1 IT always enabled, no need to check enable CC1 state */
  if (LL_TIM_IsActiveFlag_CC1 (HALL_M1.TIMx) != 0)
  {
    LL_TIM_ClearFlag_CC1(HALL_M1.TIMx);
    TSK_SpeedTIM_task();

    /* USER CODE BEGIN M1 HALL_CC1 */

    /* USER CODE END M1 HALL_CC1 */
  }
  else
  {
    /* Nothing to do */
  }

  if ((LL_TIM_IsActiveFlag_CC2 (HALL_M1.TIMx) != 0) && LL_TIM_IsEnabledIT_CC2(HALL_M1.TIMx))
  {
    LL_TIM_ClearFlag_CC2(HALL_M1.TIMx);

    if (RUN == Mci[M1].State)
    {
     (void)SixStep_StepCommution();
    }
    else
    {
      /* Nothing to do. */
    }
    LL_TIM_DisableIT_CC2(HALL_M1.TIMx); /* To avoid consecutive event when prescaler change. */

    /* USER CODE BEGIN M1 HALL_CC1 */

    /* USER CODE END M1 HALL_CC1 */
  }
  else
  {
    /* Nothing to do */
  }

  /* USER CODE BEGIN SPD_TIM_M1_IRQn 1 */

  /* USER CODE END SPD_TIM_M1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
