/**
 * @file	utility.h
 * @brief  	File containing some utility functions
 * 
 * @written by  	Eliot Ferragni
 * @creation date	18.03.2019
 */

#ifndef UTILITY_H
#define UTILITY_H

/**
 * @brief 		Writes the given mode in the PUPDR register of the line given
 * 
 * @param line 	Line to change
 * @param mode 	PUPDR mode to set. PAL_STM32_PUPDR_FLOATING, PAL_STM32_PUPDR_PULLUP or PAL_STM32_PUPDR_PULLDOWN
 * 
 * @return 		Returns the mode that was enabled before (usefull to restore its state later)
 */	
uint16_t utilityChangePUPDRGpio(ioline_t line, uint16_t mode);

#endif /* UTILITY_H */