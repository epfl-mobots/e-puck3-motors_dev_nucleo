/**
 * @file	main.h
 * @brief  	Main file of the e-puck2_programmer firmware used by the onboard programmer of the
 * 			e-puck2 educational robot.
 * 
 * @written by  	Eliot Ferragni
 * @creation date	18.06.2018
 */

#ifndef MAIN_H
#define MAIN_H

#include <ch.h>
#include <hal.h>

#include "utility.h"
#include <usbcfg.h>
#include <shell.h>
#include <chprintf.h>

#define CONFIGURED		1
#define NOT_CONFIGURED	0

#define I2C_PD_CONTROLER	I2CD2
#define SPI_DRV8323			SPID3

#define NB_MOTORS		4
#define NB_PHASES		3
#define NB_TRANSISTORS	2

extern const ioline_t motor_pins[NB_MOTORS][NB_PHASES][NB_TRANSISTORS];
void zcs_ext_reset(void);

#endif  /* MAIN_H */
