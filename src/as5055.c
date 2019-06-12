/**
 * @file	drv8323.c
 * @brief  	File containing the driver to communicate by SPI with a DRV8323 gate driver
 * 
 * @written by  	Eliot Ferragni
 * @creation date	14.03.2019
 */
 
#include "ch.h"
#include "hal.h"
#include "utility.h"
#include "as5055.h"

#define DEGREES_360		360
#define RES_12BITS		4095


void as5055DoAMasterReset(AS5550Config *as){
	uint16_t rcv = 0;

	uint16_t order = AS5055_WRITE | AS5055_MASTER_RESET_REG;
	order |= utilityComputeParity(EVEN_PARITY, order);

	uint16_t send[NB_OF_AS5055];
	for(uint8_t i = 0 ; i < NB_OF_AS5055 ; i++){
		send[i] = order;
	}
	/* Bus acquisition and SPI reprogramming.*/
    spiAcquireBus(as->spip);
    utilityReconfigureSPI(as->spip, as->spicfg);

    //we need a pulldown for the MISO when communicating with AS5055
    uint16_t pupdr = utilityChangePUPDRGpio(LINE_SPI3_MISO, PAL_STM32_PUPDR_PULLDOWN);

	spiSelect(as->spip);
	spiExchange(as->spip, NB_OF_AS5055, send, &rcv);
	spiUnselect(as->spip);

	//we restore the original pupdr state
	utilityChangePUPDRGpio(LINE_SPI3_MISO, pupdr);

	/* Releasing the bus.*/
    spiReleaseBus(as->spip);

}

void as5055ReadAngleBlocking(AS5550Config *as){

	uint16_t order = AS5055_READ | AS5055_ANGULAR_DATA_REG;
	order |= utilityComputeParity(EVEN_PARITY, order);

	uint16_t send[NB_OF_AS5055];
	for(uint8_t i = 0 ; i < NB_OF_AS5055 ; i++){
		send[i] = order;
	}

	uint16_t rcv[NB_OF_AS5055] = {0};

	/* Bus acquisition and SPI reprogramming.*/
    spiAcquireBus(as->spip);
    utilityReconfigureSPI(as->spip, as->spicfg);

    //we need a pulldown for the MISO when communicating with AS5055
    uint16_t pupdr = utilityChangePUPDRGpio(LINE_SPI3_MISO, PAL_STM32_PUPDR_PULLDOWN);

	spiSelect(as->spip);
	spiExchange(as->spip, NB_OF_AS5055, send, rcv);
	spiUnselect(as->spip);

	//we restore the original pupdr state
	utilityChangePUPDRGpio(LINE_SPI3_MISO, pupdr);

	/* Releasing the bus.*/
    spiReleaseBus(as->spip);

    for(uint8_t i = 0 ; i < NB_OF_AS5055 ; i++){
    	as->data[i].alarm_lo = (rcv[i] & ALARM_LO) 	>> ALARM_LO_Pos;
    	as->data[i].alarm_hi = (rcv[i] & ALARM_HI) 	>> ALARM_HI_Pos;
    	as->data[i].angle 	 = (rcv[i] & ANGLE_VAL)	>> ANGLE_VAL_Pos;
    	as->data[i].angle 	 = ((float)as->data[i].angle * DEGREES_360)/RES_12BITS;
	}
}	
















