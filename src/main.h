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

#include <usbcfg.h>
#include <shell.h>
#include <chprintf.h>

#define BOARD_IDENT "Motors_dev_nucleo"

#define CONFIGURED		1
#define NOT_CONFIGURED	0

#define I2C_PD_CONTROLER	I2CD2


#endif  /* MAIN_H */