/**
 * @file	motors.c
 * @brief  	Library to control motors
 * 
 * @written by  	Eliot Ferragni
 * @creation date	29.07.2019
 */
 

 #include "motors.h"

typedef struct {
	ioline_t 	p_control_pin;
	ioline_t 	n_control_pin;
	PWMDriver	*pwmp;
	uint16_t	PWM_period;
	uint8_t		PWM_p_channel;
	uint8_t		PWM_n_channel;
	uint8_t		ADCVoltageMeasureChannel;
	uint8_t		ADCCurrentMeasureChannel;
} half_bridge_t;

static half_bridge_t half_bridges[NB_OF_HALF_BRIDGES] = {
	{
		.p_control_pin				= P_CONTROL_PIN_1,
		.n_control_pin				= N_CONTROL_PIN_1,
		.pwmp 						= PWM_DRIVER_1,
		.PWM_p_channel				= PWM_P_CHANNEL_1,
		.PWM_n_channel				= PWM_N_CHANNEL_1,
		.ADCVoltageMeasureChannel	= ADC_VOLTAGE_CHANNEL_1,
		.ADCCurrentMeasureChannel	= ADC_CURRENT_CHANNEL_1
	},
	{
		.p_control_pin				= P_CONTROL_PIN_2,
		.n_control_pin				= N_CONTROL_PIN_2,
		.pwmp 						= PWM_DRIVER_2,
		.PWM_p_channel				= PWM_P_CHANNEL_2,
		.PWM_n_channel				= PWM_N_CHANNEL_2,
		.ADCVoltageMeasureChannel	= ADC_VOLTAGE_CHANNEL_2,
		.ADCCurrentMeasureChannel	= ADC_CURRENT_CHANNEL_2
	},
	{
		.p_control_pin				= P_CONTROL_PIN_3,
		.n_control_pin				= N_CONTROL_PIN_3,
		.pwmp 						= PWM_DRIVER_3,
		.PWM_p_channel				= PWM_P_CHANNEL_3,
		.PWM_n_channel				= PWM_N_CHANNEL_3,
		.ADCVoltageMeasureChannel	= ADC_VOLTAGE_CHANNEL_3,
		.ADCCurrentMeasureChannel	= ADC_CURRENT_CHANNEL_3
	},
	{
		.p_control_pin				= P_CONTROL_PIN_4,
		.n_control_pin				= N_CONTROL_PIN_4,
		.pwmp 						= PWM_DRIVER_4,
		.PWM_p_channel				= PWM_P_CHANNEL_4,
		.PWM_n_channel				= PWM_N_CHANNEL_4,
		.ADCVoltageMeasureChannel	= ADC_VOLTAGE_CHANNEL_4,
		.ADCCurrentMeasureChannel	= ADC_CURRENT_CHANNEL_4
	},
	{
		.p_control_pin				= P_CONTROL_PIN_5,
		.n_control_pin				= N_CONTROL_PIN_5,
		.pwmp 						= PWM_DRIVER_5,
		.PWM_p_channel				= PWM_P_CHANNEL_5,
		.PWM_n_channel				= PWM_N_CHANNEL_5,
		.ADCVoltageMeasureChannel	= ADC_VOLTAGE_CHANNEL_5,
		.ADCCurrentMeasureChannel	= ADC_CURRENT_CHANNEL_5
	},
	{
		.p_control_pin				= P_CONTROL_PIN_6,
		.n_control_pin				= N_CONTROL_PIN_6,
		.pwmp 						= PWM_DRIVER_6,
		.PWM_p_channel				= PWM_P_CHANNEL_6,
		.PWM_n_channel				= PWM_N_CHANNEL_6,
		.ADCVoltageMeasureChannel	= ADC_VOLTAGE_CHANNEL_6,
		.ADCCurrentMeasureChannel	= ADC_CURRENT_CHANNEL_6
	},
	{
		.p_control_pin				= P_CONTROL_PIN_7,
		.n_control_pin				= N_CONTROL_PIN_7,
		.pwmp 						= PWM_DRIVER_7,
		.PWM_p_channel				= PWM_P_CHANNEL_7,
		.PWM_n_channel				= PWM_N_CHANNEL_7,
		.ADCVoltageMeasureChannel	= ADC_VOLTAGE_CHANNEL_7,
		.ADCCurrentMeasureChannel	= ADC_CURRENT_CHANNEL_7
	},
	{
		.p_control_pin				= P_CONTROL_PIN_8,
		.n_control_pin				= N_CONTROL_PIN_8,
		.pwmp 						= PWM_DRIVER_8,
		.PWM_p_channel				= PWM_P_CHANNEL_8,
		.PWM_n_channel				= PWM_N_CHANNEL_8,
		.ADCVoltageMeasureChannel	= ADC_VOLTAGE_CHANNEL_8,
		.ADCCurrentMeasureChannel	= ADC_CURRENT_CHANNEL_8
	},
	{
		.p_control_pin				= P_CONTROL_PIN_9,
		.n_control_pin				= N_CONTROL_PIN_9,
		.pwmp 						= PWM_DRIVER_9,
		.PWM_p_channel				= PWM_P_CHANNEL_9,
		.PWM_n_channel				= PWM_N_CHANNEL_9,
		.ADCVoltageMeasureChannel	= ADC_VOLTAGE_CHANNEL_9,
		.ADCCurrentMeasureChannel	= ADC_CURRENT_CHANNEL_9
	},
	{
		.p_control_pin				= P_CONTROL_PIN_10,
		.n_control_pin				= N_CONTROL_PIN_10,
		.pwmp 						= PWM_DRIVER_10,
		.PWM_p_channel				= PWM_P_CHANNEL_10,
		.PWM_n_channel				= PWM_N_CHANNEL_10,
		.ADCVoltageMeasureChannel	= ADC_VOLTAGE_CHANNEL_10,
		.ADCCurrentMeasureChannel	= ADC_CURRENT_CHANNEL_10
	},
	{
		.p_control_pin				= P_CONTROL_PIN_11,
		.n_control_pin				= N_CONTROL_PIN_11,
		.pwmp 						= PWM_DRIVER_11,
		.PWM_p_channel				= PWM_P_CHANNEL_11,
		.PWM_n_channel				= PWM_N_CHANNEL_11,
		.ADCVoltageMeasureChannel	= ADC_VOLTAGE_CHANNEL_11,
		.ADCCurrentMeasureChannel	= ADC_CURRENT_CHANNEL_11
	},
	{
		.p_control_pin				= P_CONTROL_PIN_12,
		.n_control_pin				= N_CONTROL_PIN_12,
		.pwmp 						= PWM_DRIVER_12,
		.PWM_p_channel				= PWM_P_CHANNEL_12,
		.PWM_n_channel				= PWM_N_CHANNEL_12,
		.ADCVoltageMeasureChannel	= ADC_VOLTAGE_CHANNEL_12,
		.ADCCurrentMeasureChannel	= ADC_CURRENT_CHANNEL_12
	}
}
