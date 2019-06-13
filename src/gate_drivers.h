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
 * Define here the names of the DRR8223 to use
 * Do not put more than 32 gate drivers, otherwise the event handling 
 * made inside gate_drivers.c will be broken.
 */
typedef enum {
	GATE_DRIVER_1 = 0,
	GATE_DRIVER_2,
	GATE_DRIVER_3,
	GATE_DRIVER_4,
	NB_OF_GATE_DRIVERS,
} gateDriver_id;

/**
 * @brief 		Writes the given register to the given DRV8323
 * 
 * @param id 	Device number. See gateDriver_id for choice.
 * @param reg 	Register to write to the device. It should contain the
 * 				address and the configuration bits of the whole register
 * 				since it will be replaced by this one on the device.
 */
void gateDriversWriteReg(gateDriver_id id, uint16_t reg);

/**
 * @brief 		Reads the given register from the given DRV8323
 * 
 * @param id 	Device number. See gateDriver_id for choice.
 * @param reg 	Register to read from the device. It should at least 
 * 				contain the address of the register.
 * 				
 * @return 		The register read.
 */
uint16_t gateDriversReadReg(gateDriver_id id, uint16_t reg);

/**
 * @brief 	Enables the given gate driver, configures it and creates the thread
 * 			which handles the faults if not already created.
 */
void gateDriversEnable(gateDriver_id id);

/**
 * @brief 	Enables all the gate drivers, configures them and create the thread
 * 			which handles the faults if not already created.
 */
void gateDriversEnableAll(void);

/**
 * @brief 		Disables the given gate driver and suspends the thread if no
 * 				other device is enabled.
 * 
 * @param id 	Device number. See gateDriver_id for choice.
 */
void gateDriversDisable(gateDriver_id id);

/**
 * @brief 		Disables the given gate driver and suspends the thread
 */
void gateDriversDisableAll(void);

#endif /* GATE_DRIVERS_H */