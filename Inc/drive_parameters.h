
/**
  ******************************************************************************
  * @file    drive_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a motor drive.
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
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVE_PARAMETERS_H
#define DRIVE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED_RPM          4800 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM          (0 + (U_RPM / SPEED_UNIT)) /*!< rpm, mechanical, absolute value */
#define M1_SS_MEAS_ERRORS_BEFORE_FAULTS    3 /*!< Number of speed measurement errors before main sensor goes in fault */

/****** Hall sensors ************/
#define HALL_AVERAGING_FIFO_DEPTH          6 /*!< depth of the FIFO used to average  mechanical speed in 0.1Hz resolution */
#define STEP_SHIFT                         (HALL_PHASE_SHIFT / 60)            /*!< Number of steps (60 degrees) into the Placement electrical angle. */
#define PHASE_SHIFT_DEG                    (HALL_PHASE_SHIFT - (STEP_SHIFT * 60)) /*!< Number of degrees remaining after STEP_SHIFT. */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */
#define PWM_FREQUENCY                      15000
#define PWM_FREQ_SCALING                   1
#define PWM_FREQUENCY_REF                  60000
#define LOW_SIDE_SIGNALS_ENABLING          LS_PWM_TIMER

#define REP_COUNTER                        0

#define SW_DEADTIME_NS                     850 /*!< Dead-time to be inserted by FW, only if low side signals are enabled */

/* Speed control loop */
#define SPEED_LOOP_FREQUENCY_HZ            (uint16_t)1000 /*!<Execution rate of speed regulation loop (Hz) */
#define PID_SPEED_KP_DEFAULT              -2182/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT              -11/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT               0/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/

/* Speed PID parameter dividers */
#define SP_KPDIV                           16384
#define SP_KIDIV                           16384
#define SP_KDDIV                           16
#define SP_KPDIV_LOG                       LOG2((16384))
#define SP_KIDIV_LOG                       LOG2((16384))
#define SP_KDDIV_LOG                       LOG2((16))

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV 1 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING     DISABLE
#define PERIODMAX_REF                      (uint16_t)((PWM_PERIOD_CYCLES_REF * 100 / 100) + 1)
#define LF_TIMER_ARR                       65535
#define LF_TIMER_PSC                       100U

/* Default settings */
#define DEFAULT_CONTROL_MODE               MCM_SPEED_MODE
#define DEFAULT_DRIVE_MODE                 CM /*!< VOLTAGE_MODE (VM) or CURRENT_MODE (CM) */
#define DEFAULT_TARGET_SPEED_RPM           1728
#define DEFAULT_TARGET_SPEED_UNIT          (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/U_RPM)

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
#define OV_VOLTAGE_THRESHOLD_V             28.8 /*!< Over-voltage threshold */
#define UD_VOLTAGE_THRESHOLD_V             6.7 /*!< Under-voltage threshold */
#ifdef NOT_IMPLEMENTED
#define ON_OVER_VOLTAGE                    TURN_OFF_PWM /*!< TURN_OFF_PWM */
#endif /* NOT_IMPLEMENTED */
#define OV_TEMPERATURE_THRESHOLD_C         70 /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C        10 /*!< Celsius degrees */

/******************************   START-UP PARAMETERS   **********************/

/******************************   Initial Position Detection   **********************/

/******************************   BUS VOLTAGE Motor 1  **********************/
#define  M1_VBUS_SAMPLING_TIME             LL_ADC_SAMPLING_CYCLE(28)

/******************************   Current sensing Motor 1   **********************/
#define  M1_CUR_MON_ADC_SAMPLING_TIME      LL_ADC_SAMPLING_CYCLE(28)
#define  M1_CUR_MON_ADC_SAMPLING_TIME_NS   2036

/******************************   ADDITIONAL FEATURES   **********************/

/*** On the fly start-up ***/

/**************************
 *** Control Parameters ***
 **************************/

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
