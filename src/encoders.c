/**
 * @file	encoders.c
 * @brief  	File containing the driver to communicate by SPI with a AS5055 encoder
 * 
 * @written by  	Eliot Ferragni
 * @creation date	07.06.2019
 */

#include "main.h"
#include "encoders.h"
#include "as5055.h"

static SPIConfig spicfg = {
	.circular = false,
	.end_cb = NULL,
	.ssline = LINE_CS_ENCODERS_n,
	.cr1 = SPI_CR1_BR_1 | SPI_CR1_CPHA,	//PRESCALER = 010, POL = 0, PHASE = 1
	.cr2 = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0	//16bits
};

static encoders_data_t encoders_data[NB_OF_ENCODERS];

static AS5550Config encoders = {
	.spip = &SPI_ENCODERS,
	.spicfg = &spicfg,
	.ssline = LINE_CS_ENCODERS_n,
	.interruptline = LINE_INT_ENCODERS_n,
	.nb_of_as5055 = NB_OF_ENCODERS,
	.data = encoders_data,
};

static THD_WORKING_AREA(encoders_thd_wa, 128);
static THD_FUNCTION(encoders_thd, arg)
{

	(void) arg;
	chRegSetThreadName("Encoders reader");

	as5055DoAMasterReset(&encoders);
	
	palEnableLineEvent(encoders.interruptline, PAL_EVENT_MODE_FALLING_EDGE);

	while(1){
		/*	
		 *	We read as fast as possible the encoders waiting for their interrupt pins to 
		 *	to signal a ready measurement.
		 */
		palWaitLineTimeout(encoders.interruptline, TIME_MS2I(5));
		if(!palReadLine(encoders.interruptline)){
			as5055ReadAngle(&encoders);
		}
	}
}

void encodersStartReading(void){
	chThdCreateStatic(encoders_thd_wa, sizeof(encoders_thd_wa), NORMALPRIO, encoders_thd, NULL);
}

encoders_data_t* encodersGetData(void){
	return encoders_data;
}
