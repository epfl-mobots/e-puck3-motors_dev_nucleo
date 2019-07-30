/**
 * @file	motors_conf.h
 * @brief  	Configuration file for the motors library
 * 
 * @written by  	Eliot Ferragni
 * @creation date	29.07.2019
 */
 
 #include "motors.h"


	ioline_t 	p_control_pin;
	ioline_t 	n_control_pin;
	PWMDriver	*pwmp;
	uint8_t		PWM_p_channel;
	uint8_t		PWM_n_channel;
	uint8_t		ADCVoltageMeasureChannel;
	uint8_t		ADCCurrentMeasureChannel;


/********************              BOARD SPECIFIC CONFIG            ********************/

//Voltage measurement must be on ADC3 and Current measurement on ADC1 without confilct

#define P_CONTROL_PIN_1				LINE_OUT_MOT1_PH1_P
#define N_CONTROL_PIN_1				LINE_OUT_MOT1_PH1_N
#define PWM_DRIVER_1				&PWMD1
#define PWM_P_CHANNEL_1				TIM_CHANNEL_1
#define PWM_N_CHANNEL_1				TIM_CHANNEL_1
#define ADC_VOLTAGE_CHANNEL_1		ADC_CHANNEL_IN0
#define ADC_CURRENT_CHANNEL_1		ADC_CHANNEL_IN4

#define P_CONTROL_PIN_2				LINE_OUT_MOT1_PH2_P
#define N_CONTROL_PIN_2				LINE_OUT_MOT1_PH2_N
#define PWM_DRIVER_2				&PWMD1
#define PWM_P_CHANNEL_2				TIM_CHANNEL_2
#define PWM_N_CHANNEL_2				TIM_CHANNEL_2
#define ADC_VOLTAGE_CHANNEL_2		ADC_CHANNEL_IN1
#define ADC_CURRENT_CHANNEL_2		ADC_CHANNEL_IN5

#define P_CONTROL_PIN_3				LINE_OUT_MOT1_PH3_P
#define N_CONTROL_PIN_3				LINE_OUT_MOT1_PH3_N
#define PWM_DRIVER_3				&PWMD1
#define PWM_P_CHANNEL_3				TIM_CHANNEL_3
#define PWM_N_CHANNEL_3				TIM_CHANNEL_3
#define ADC_VOLTAGE_CHANNEL_3		ADC_CHANNEL_IN2
#define ADC_CURRENT_CHANNEL_3		ADC_CHANNEL_IN6

#define P_CONTROL_PIN_4				LINE_OUT_MOT2_PH1_P
#define N_CONTROL_PIN_4				LINE_OUT_MOT2_PH1_N
#define PWM_DRIVER_4				&PWMD2
#define PWM_P_CHANNEL_4				TIM_CHANNEL_1
#define PWM_N_CHANNEL_4				TIM_CHANNEL_2
#define ADC_VOLTAGE_CHANNEL_4		ADC_CHANNEL_IN4
#define ADC_CURRENT_CHANNEL_4		ADC_CHANNEL_IN4

#define P_CONTROL_PIN_5				LINE_OUT_MOT2_PH2_P
#define N_CONTROL_PIN_5				LINE_OUT_MOT2_PH2_N
#define PWM_DRIVER_5				&PWMD2
#define PWM_P_CHANNEL_5				TIM_CHANNEL_3
#define PWM_N_CHANNEL_5				TIM_CHANNEL_4
#define ADC_VOLTAGE_CHANNEL_5		ADC_CHANNEL_IN5
#define ADC_CURRENT_CHANNEL_5		ADC_CHANNEL_IN5






