/**
 * @file	gate_driver.h
 * @brief  	Library to manage the gate drivers
 * 
 * @written by  	Eliot Ferragni
 * @creation date	17.03.2019
 */
 

#ifndef GATE_DRIVERS_H
#define GATE_DRIVERS_H

#include "drv8323.h"

/**
 * do not put more than 32 gate drivers, otherwise the event handling 
 * made inside gate_drivers.c will be broken
 */
typedef enum {
	DRV8323_1 = 0,
	DRV8323_2,
	DRV8323_3,
	DRV8323_4,
	NB_OF_DRV8323,
} gateDriver_id;

void gateDriversWriteReg(gateDriver_id id, uint16_t reg);
uint16_t gateDriversReadReg(gateDriver_id id, uint16_t reg);
void gateDriversInit(void);
#endif /* GATE_DRIVERS_H */