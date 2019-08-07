/**
 * @file	motors.h
 * @brief  	Library to control motors
 * 
 * @written by  	Eliot Ferragni
 * @creation date	29.07.2019
 */

#ifndef MOTORS_H
#define MOTORS_H


/**
 * Timer's channels list
 */
typedef enum{
    TIM_CHANNEL_1 = 0,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4,
    TIM_CHANNEL_5,
    TIM_CHANNEL_6
}tim_channel_t;

/**
 * Brushless motors list
 */
typedef enum
{
    BRUSHLESS_MOTOR_1 = 0,
    BRUSHLESS_MOTOR_2,
    BRUSHLESS_MOTOR_3,
    BRUSHLESS_MOTOR_4,
}brushless_motors_names_t;

#define MAX_NB_OF_BRUSHLESS_MOTOR   4

/**
 * Possible rotation directions
 */
typedef enum{
    CCW = -1,
    CW  = 1,
}rotation_dir_t;

/**
 * Commutation's scheme list
 */
typedef enum{
    DOUBLE_PWM = 0,
    SIMPLE_PWM,
    GROUND_PWM,
    NB_OF_COMMUTATION_SCHEME
}commutation_schemes_t;

void motorsStart(void);

void motorSetDutyCycle(brushless_motors_names_t motor_name, uint8_t duty_cycle);

#include "motors_conf.h"

#endif /* MOTORS_H */