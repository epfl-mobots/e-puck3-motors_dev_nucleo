/**
 * @file	utility.h
 * @brief  	File containing some utility functions
 * 
 * @written by  	Eliot Ferragni
 * @creation date	18.03.2019
 */

#ifndef UTILITY_H
#define UTILITY_H

typedef enum{
	EVEN_PARITY = 0,
	ODD_PARITY,
}parity_type_t;

/**
 * @brief 		Writes the given mode in the PUPDR register of the line given
 * 
 * @param line 	Line to change
 * @param mode 	PUPDR mode to set. PAL_STM32_PUPDR_FLOATING, PAL_STM32_PUPDR_PULLUP or PAL_STM32_PUPDR_PULLDOWN
 * 
 * @return 		Returns the mode that was enabled before (usefull to restore its state later)
 */	
uint16_t utilityChangePUPDRGpio(ioline_t line, uint16_t mode);

/**
 * @brief 			Reconfigures or configures the given SPIDriver with the given 
 * 					SPIConfig only if the config is different from the current one 
 * 					(we test the pointer, not the content).
 * 					Note 1 : The SPI bus should have been aquired before the call to this function
 * 					and should be released after. See spiAcquireBus() and spiReleaseBus() for info.
 * 					Note 2 : It starts the SPI bus if it's not already the case.
 * 
 * @param spip 		SPI bus to reconfigure
 * @param config 	New config
 */
void utilityReconfigureSPI(SPIDriver *spip, const SPIConfig *config);

/**
 * @brief 				Computes the parity of a given 16bits value.
 * 
 * @param parity_type 	Tell if we want the odd or even parity. See parity_type_t
 * @param value 		16 bits value to compute the parity
 * 
 * @return 				Returns true or false depending on the parity type chosen
 */
bool utilityComputeParity(parity_type_t parity_type, uint16_t value);

#endif /* UTILITY_H */