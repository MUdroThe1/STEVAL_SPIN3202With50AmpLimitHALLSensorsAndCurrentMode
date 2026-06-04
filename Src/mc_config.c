
/**
  ******************************************************************************
  * @file    mc_config.c
  * @author  Motor Control SDK Team,ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044,the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
//cstat -MISRAC2012-Rule-21.1
#include "main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "mc_config.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

/* USER CODE BEGIN Additional define */

/* USER CODE END Additional define */

/**
  * @brief  PI / PID Speed loop parameters Motor 1.
  */
PID_Handle_t PIDSpeedHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)(PERIODMAX_REF * SP_KIDIV),
  .wLowerIntegralLimit = 0,
  .hUpperOutputLimit   = (int16_t)PERIODMAX_REF,
  .hLowerOutputLimit   = 0,
  .hKpDivisor          = (uint16_t)SP_KPDIV,
  .hKiDivisor          = (uint16_t)SP_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

/**
  * @brief  Openloop sixstep Controller parameters Motor 1.
  */
OpenLoopSixstepCtrl_Handle_t OpenLoopSixstepCtrllM1 =
{
  .DutyCycleRefMean           = 0U,                               /* Computed DutyCycleRef mean value. */
  .CurrentFactor              = 10U,                              /* Curent factor for openloop speed control % of max DutyCycle. */
  .VoltageFactor              = 85U,                              /* Voltage factor for openloop speed control % of max DutyCycle. */
  .DutyCycleRef               = 20U,                              /* DutyCycleRef  duty cycle for PWM timer. */
  .DutyCycleRefFilter         = M1_OPENLOOP_DIGITAL_FILTER_SHIFT, /* Shif value of the digital filter. */
  .Openloop                   = false,                             /* Openloop flag. */
  .RevUp                      = 1,                                /* RevUp enabling flag. */
  .OnSensing                  = 0,                                /* OnSensing enabling flag. */
};

static SpeednDutyCtrl_Handle_t SpeednDutyCtrlM1 =
{
  .Mode                       = DEFAULT_CONTROL_MODE,                         /* Changed during RevUp phase */
  .TargetFinal                = 0,                                            /* Will be updated with SDC_ExecRamp and SDC_CalcSpeedReference. */
  .SpeedRefUnitExt            = (int32_t)(DEFAULT_TARGET_SPEED_UNIT) * 65536, /* Mechanical rotor speed reference. */
  .DutyCycleRef               = 0U * 65536U,                                  /* Will be updated during RevUp phase. */
  .RampRemainingStep          = 0U,
  .PISpeed                    = &PIDSpeedHandle_M1,                            /* Speed PID pointeur. */
  .SPD                        = &HALL_M1._Super,                               /* Speed sensor pointer. */
  .IncDecAmount               = 0U,
  .SDCFrequencyHz             = MEDIUM_FREQUENCY_TASK_RATE,
  .MaxAppPositiveMecSpeedUnit = (uint16_t)(MAX_APPLICATION_SPEED_UNIT),
  .MinAppPositiveMecSpeedUnit = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
  .MaxAppNegativeMecSpeedUnit = (int16_t)(-MIN_APPLICATION_SPEED_UNIT),
  .MinAppNegativeMecSpeedUnit = (int16_t)(-MAX_APPLICATION_SPEED_UNIT),
  .MaxPositiveDutyCycle       = (uint16_t)PERIODMAX_REF,
  .ModeDefault                = DEFAULT_CONTROL_MODE,
  .MecSpeedRefUnitDefault     = (int16_t)(DEFAULT_TARGET_SPEED_UNIT),
  .DutyCycleRefDefault        = (uint16_t)PERIODMAX_REF,
};

PWMC_Handle_t PWM_Handle_M1 =
{
  .StartCntPh           = PWM_PERIOD_CYCLES,
  .PWMperiod            = PWM_PERIOD_CYCLES,
  .OverCurrentFlag      = false,
  .OverVoltageFlag      = false,
  .driverProtectionFlag = false,
  .TimerCfg             = &SixPwm_TimerCfgM1,
  .QuasiSynchDecay      = false,
  .LSModArray           = {0,0,0,0,0,0},
  .pParams_str          = &PWMC_ParamsM1,
  .LowSideOutputs       = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,
  .TurnOnLowSidesAction = false,
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - HALL.
  */
HALL_6S_Handle_t HALL_M1 =
{
  ._Super =
  {
    .bElToMecRatio             = POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit  = (uint16_t)(1.15 * MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit  = 0,
    .bMaximumSpeedErrorsNumber = M1_SS_MEAS_ERRORS_BEFORE_FAULTS,
    .speedConvFactor           = SPEED_TIMER_CONV_FACTOR,
  },

  .SensorPlacement             = HALL_SENSORS_PLACEMENT,
  .PhaseShift                  = PHASE_SHIFT_DEG,
  .StepShift                   = STEP_SHIFT,
  .SpeedBufferSize             = HALL_AVERAGING_FIFO_DEPTH,
  .TIMClockFreq                = HALL_TIM_CLK,
  .TIMx                        = TIM2,
  .ICx_Filter                  = M1_HALL_IC_FILTER_LL,
  .H1Port                      = M1_HALL_H1_GPIO_Port,
  .H1Pin                       = M1_HALL_H1_Pin,
  .H2Port                      = M1_HALL_H2_GPIO_Port,
  .H2Pin                       = M1_HALL_H2_Pin,
  .H3Port                      = M1_HALL_H3_GPIO_Port,
  .H3Pin                       = M1_HALL_H3_Pin,
  .Direction                   = 1,
};

SixStepVars_t SixStepVars[NBR_OF_MOTORS];
SpeednDutyCtrl_Handle_t *pSDC[NBR_OF_MOTORS]    = {&SpeednDutyCtrlM1};
NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS] = {&TempSensor_M1};
OpenLoopSixstepCtrl_Handle_t *pOLS[NBR_OF_MOTORS] = {&OpenLoopSixstepCtrllM1};

MCI_Handle_t Mci[NBR_OF_MOTORS] =
{
  {
    .pSDC = &SpeednDutyCtrlM1,
    .pSixStepVars = &SixStepVars[0],
    .pPWM = &PWM_Handle_M1,
    .lastCommand = MCI_NOCOMMANDSYET,
    .hFinalSpeed = 0,
    .hFinalTorque = 0,
    .pScale = &scaleParams_M1,
    .hDurationms = 0,
    .DirectCommand = MCI_NO_COMMAND,
    .State = IDLE,
    .CurrentFaults = MC_NO_FAULTS,
    .PastFaults = MC_NO_FAULTS,
    .CommandState = MCI_BUFFER_EMPTY,
  },

};

/**
  * Current monitor parameters Motor 1.
  */
CurrMonitor_t CurrMonitor_M1 =
{
  .regADC                   = ADC1,
  .channel                  = MC_ADC_CHANNEL_4,
  .samplingTime             = M1_CUR_MON_ADC_SAMPLING_TIME,
  .currentConvFactor        = CURRENT_MONITOR_CONV_FACTOR,
  .samplingPointConvFact    = M1_CUR_MON_SAMPLING_CONV,
  .samplingDistance2Edge    = M1_CUR_MON_ADC_SAMPLING_TIME_NS,
  .hLowPassFilterBW         = 8,
};

/* USER CODE BEGIN Additional configuration */

/* USER CODE END Additional configuration */

/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/

