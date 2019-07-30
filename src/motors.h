/**
 * @file	motors.h
 * @brief  	Library to control motors
 * 
 * @written by  	Eliot Ferragni
 * @creation date	29.07.2019
 */

#ifndef MOTORS_H
#define MOTORS_H

#define NB_OF_HALF_BRIDGES	12


#define PERIOD_PWM_52_KHZ_APB2_216MHZ	4096
#define PERIOD_PWM_52_KHZ_APB1_108MHZ	2048

typedef enum
{
    TIM_CHANNEL_1 = 0,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4,
    TIM_CHANNEL_5,
    TIM_CHANNEL_6
}tim_channel_t;

/**
 * @brief   Structure representing a physical half_bridge.
 */
typedef struct {
	ioline_t 		p_control_pin;
	ioline_t 		n_control_pin;
	PWMDriver		*pwmp;
	tim_channel_t	PWM_p_channel;
	tim_channel_t	PWM_n_channel;
	uint8_t			ADCVoltageMeasureChannel;
	uint8_t			ADCCurrentMeasureChannel;
} half_bridge_t;


#include "motors_conf.h"

#endif /* MOTORS_H */