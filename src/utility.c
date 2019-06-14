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

void utilityReconfigureSPI(SPIDriver *spip, const SPIConfig *config){

	if(spip->config == config){
		return;
	}else{
		spiStart(spip, config);
	}
}

bool utilityComputeParity(parity_type_t parity_type, uint16_t value){
	
	// recursively divide the (16-bit) integer into two equal 
	// halves and take their XOR until only 1 bit is left

	value = (value & 0x00FF)^(value >> 8);
	value = (value & 0x000F)^(value >> 4);
	value = (value & 0x0003)^(value >> 2);
	value = (value & 0x0001)^(value >> 1);
	
	// return the last bit
	return value ^ parity_type;
}