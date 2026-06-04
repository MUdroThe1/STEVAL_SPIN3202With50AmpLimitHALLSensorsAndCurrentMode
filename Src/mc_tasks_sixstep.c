
/**
  ******************************************************************************
  * @file    mc_tasks_sixstep.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition
  *
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
  * @ingroup MCTasksSixStep
  */

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h"
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "digital_output.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "mcp_config.h"
#include "mc_app_hooks.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SixStep
  * @{
  */

/** @addtogroup	MCCockpitSixStep
  * @{
  */

  /** @defgroup MCCockpitSixStep MC Cockpit
  *
  * @brief
  *
  * @{
  */

/** @addtogroup	MCTasksSixStep
  * @{
  */

    /** @defgroup MCTasksSixStep Motor Control Tasks
  *
  * @brief  	Motor Control subsystem configuration and operation routines for SixStep applications.
  *
  * @{
  */

/** @defgroup MCTasksSixStep Motor Control Tasks for Six Step algorithm
  *
  * @brief SixStep Motor Control subsystem configuration and operation routines.
  *
  * @{
  */

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/

/* USER CODE END Private define */

/* Private variables----------------------------------------------------------*/
static volatile uint16_t hBootCapDelayCounterM1   = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM1 = ((uint16_t)0);

#define M1_CHARGE_BOOT_CAP_TICKS       (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M1_CHARGE_BOOT_CAP_DUTY_CYCLES ((uint32_t)0.000\
                                     * ((uint32_t)PWM_PERIOD_CYCLES / 2U))
#define M2_CHARGE_BOOT_CAP_TICKS       (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M2_CHARGE_BOOT_CAP_DUTY_CYCLES ((uint32_t)0\
                                      *((uint32_t)PWM_PERIOD_CYCLES2 / 2U))

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
void TSK_MF_StopProcessing(uint8_t motor);
MCI_Handle_t *GetMCI(uint8_t bMotor);

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  * @param  None
  */
__weak void SIX_STEP_Init(void)
{
  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */

    /**********************************************************/
    /*    PWM and current sensing component initialization    */
    /**********************************************************/
    pwmcHandle[M1] = &PWM_Handle_M1;
    PWMC_Init(&PWM_Handle_M1);
    LL_TIM_OC_SetCompareCH1(TIM3,PWM_PERIOD_CYCLES_REF); /* Set initial compare value for current control. */
    LL_TIM_EnableCounter(TIM3); /* Enable the timer counter. */
    LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH1); /* Select the channel. */
    LL_TIM_EnableAllOutputs(TIM3); /* Enable the PWM output. */

    /* USER CODE BEGIN MCboot 1 */

    /* USER CODE END MCboot 1 */

    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_HandleInit(&PIDSpeedHandle_M1);

    /******************************************************/
    /*   Main speed sensor component initialization       */
    /******************************************************/
    HALL_Init(&HALL_M1);

    /******************************************************/
    /*   Speed & duty cycle component initialization          */
    /******************************************************/
    SIX_STEP_Clear(M1);
    MCI_ExecSpeedRamp(&Mci[M1],
    SDC_GetMecSpeedRefUnitDefault(pSDC[M1]),0); /* First command to SDC */

    /* USER CODE BEGIN MCboot 2 */

    /* USER CODE END MCboot 2 */
}

/**
 * @brief Performs stop process and update the state machine.This function
 *        shall be called only during medium frequency task.
 */
void TSK_MF_StopProcessing(uint8_t motor)
{
  SIX_STEP_Clear(motor);
  TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
  Mci[motor].State = STOP;
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the
  * present state of its state machine. In particular, duties requiring a periodic
  * execution at a medium frequency rate (such as the speed controller for instance)
  * are executed here.
  */
__weak void TSK_MediumFrequencyTaskM1(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */

  bool IsSpeedReliable = HALL_CalcAvrgMecSpeedUnit(&HALL_M1);

  if (MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS)
  {
    if (MCI_GetOccurredFaults(&Mci[M1]) == MC_NO_FAULTS)
    {
      switch (Mci[M1].State)
      {

        case IDLE:
        {
          if (MCI_START == Mci[M1].DirectCommand)
          {
            PWMC_TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
            TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
            Mci[M1].State = CHARGE_BOOT_CAP;
          }
          else
          {
            /* Nothing to be done, FW stays in IDLE state. */
          }
          break;
        }

        case CHARGE_BOOT_CAP:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (TSK_ChargeBootCapDelayHasElapsedM1())
            {
              PWMC_SwitchOffPWM(pwmcHandle[M1]);
              HALL_SetDirection(&HALL_M1, (int8_t)MCI_GetImposedMotorDirection(&Mci[M1]));
              SIX_STEP_Clear(M1);

#if (PID_SPEED_INTEGRAL_INIT_DIV == 0)
              PID_SetIntegralTerm(&PIDSpeedHandle_M1, 0);
#else
              PID_SetIntegralTerm(&PIDSpeedHandle_M1,
                                 (((int32_t)SixStepVars[M1].DutyCycleRef * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M1))
                                 / PID_SPEED_INTEGRAL_INIT_DIV));
#endif
              MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
              SixStepVars[M1].DutyCycleRef = SDC_CalcSpeedReference(pSDC[M1]);
              Mci[M1].State = RUN;
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
              (void)SixStep_StepCommution();
            }
            else
            {
              /* Nothing to be done, FW waits for bootstrap capacitor to charge */
            }
          }
          break;
        }

        case RUN:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            /* USER CODE BEGIN MediumFrequencyTask M1 2 */

            /* USER CODE END MediumFrequencyTask M1 2 */

            if (SDC_GetOpenLoopFlag(pOLS[M1]))
            {
              SDC_SetControlMode(pSDC[M1], MCM_DUTY_MODE);
            }
            else
            {
              MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
            }
            SixStepVars[M1].DutyCycleRef = SDC_CalcSpeedReference(pSDC[M1]);
            /* Update PWM at this Medium frequency rate after compution to allow a better start at low speed. */
            PWMC_SetPhaseVoltage(pwmcHandle[M1], PWM_Handle_M1.StartCntPh);
            LL_TIM_OC_SetCompareCH1(TIM3,SixStepVars[M1].DutyCycleRef); /* Set counter compare for current control. */

            if(!IsSpeedReliable)
            {
              MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);

            }
            else
            {
              /* Nothing to do */
            }
          }
          break;
        }

        case STOP:
        {
          if (TSK_StopPermanencyTimeHasElapsedM1())
          {
            SDC_ClearDutyCycleMean(pOLS[M1]);
            /* USER CODE BEGIN MediumFrequencyTask M1 5 */

            /* USER CODE END MediumFrequencyTask M1 5 */

            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW waits for to stop */
          }
          break;
        }

        case FAULT_OVER:
        {
          if (MCI_ACK_FAULTS == Mci[M1].DirectCommand)
          {
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW stays in FAULT_OVER state until acknowledgement */
          }
          break;
        }

        case FAULT_NOW:
        {
          Mci[M1].State = FAULT_OVER;
          break;
        }

        default:
          break;
       }
    }
    else
    {
      Mci[M1].State = FAULT_OVER;
    }
  }
  else
  {
    Mci[M1].State = FAULT_NOW;
  }

  /* Perform the Regular conversion. */
  RCM_ExecNextConv();
  RCM_WaitForConv();
  RCM_ReadOngoingConv();

  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */

}

/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void SIX_STEP_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN SixStep_Clear 0 */

  /* USER CODE END SixStep_Clear 0 */
  SDC_Clear(pSDC[bMotor]);
  SixStepVars[bMotor].DutyCycleRef = SDC_GetDutyCycleRef(pSDC[bMotor]);
#if (PID_SPEED_INTEGRAL_INIT_DIV == 0)
  PID_SetIntegralTerm(&PIDSpeedHandle_M1, 0);
#else
  PID_SetIntegralTerm(&PIDSpeedHandle_M1,
                      (((int32_t)SixStepVars[M1].DutyCycleRef * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M1))
                      / PID_SPEED_INTEGRAL_INIT_DIV));
#endif
  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
  HALL_Clear(&HALL_M1);
  LL_TIM_SetCounter(TIM3, 0u); /* Clear the counter. */
  LL_TIM_OC_SetCompareCH1(TIM3,PWM_PERIOD_CYCLES_REF); /* Init compare couter value. */

  /* USER CODE BEGIN SixStep_Clear 1 */

  /* USER CODE END SixStep_Clear 1 */
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

/**
  * @brief  This is the Hall sensors commutation task. It configures
  * the speed timer for an immediate or delayed step commutation.
  */
void TSK_SpeedTIM_task(void)
{
  (void)HALL_TIMx_CC_IRQHandler(&HALL_M1);

  if ((RUN == Mci[M1].State) && ((0U == HALL_M1.PhaseShift) || (HALL_M1.PhaseShift >= 30U)))
  {
    (void)SixStep_StepCommution();
  }
  else
  {
    /* Nothing to do. */
  }
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
inline void SixStep_StepCommution(void)
{
  PWMC_SetPhaseVoltage(pwmcHandle[M1], PWM_Handle_M1.StartCntPh);
  LL_TIM_OC_SetCompareCH1(TIM3,SixStepVars[M1].DutyCycleRef); /* Set counter compare for current control. */
  PWMC_ForceNextStep(&PWM_Handle_M1, 0, HALL_GetStep(&HALL_M1));
  LL_TIM_SetPrescaler(HALL_M1.TIMx, HALL_M1.Prescaler); /* To avoid update at CCH2 event instead of Hall event. */
  PWMC_LoadNextStep(&PWM_Handle_M1);
}

/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
