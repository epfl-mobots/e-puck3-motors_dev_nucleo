/**
 * @file	utility.c
 * @brief  	File containing some utility functions
 * 
 * @written by  	Eliot Ferragni
 * @creation date	18.03.2019
 */

#include "ch.h"
#include "hal.h"
#include "utility.h"

uint16_t utilityChangePUPDRGpio(ioline_t line, uint16_t mode){

	//lock to be sure to modify atomically the pupdr register
	chSysLock();
	//prepares the pupdr to write
	uint32_t pupdr = ( (mode & PAL_STM32_PUPDR_MASK) >> 5) << ((PAL_PAD(line) * 2));
	//reads the current state
	uint16_t current_mode = ( ( PAL_PORT(line)->PUPDR >> (PAL_PAD(line) * 2) ) & 0x3 ) << 5;
	//writes the new state
	PAL_PORT(line)->PUPDR = ( PAL_PORT(line)->PUPDR & ~(0x3 << ((PAL_PAD(line) * 2)) ) ) | pupdr;

	chSysUnlock();
	return current_mode;
}