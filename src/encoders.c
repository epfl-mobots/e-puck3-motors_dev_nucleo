/**
 * @file	encoders.c
 * @brief  	File containing the driver to communicate by SPI with a AS5055 encoder
 * 
 * @written by  	Eliot Ferragni
 * @creation date	07.06.2019
 */

#include "main.h"
#include "encoders.h"

static SPIConfig spicfg = {
	.circular = false,
	.end_cb = NULL,
	.ssline = LINE_CS_ENCODERS_n,
	.cr1 = SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_CPHA,	//PRESCALER = 010, POL = 0, PHASE = 1
	.cr2 = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0	//16bits
};

static AS5550Config encoders = {
	.spip = &SPI_AS5055,
	.spicfg = &spicfg,
	.ssline = LINE_CS_ENCODERS_n,
	.interruptline = LINE_INT_ENCODERS_n,
};



static THD_WORKING_AREA(encoders_thd_wa, 128);
static THD_FUNCTION(encoders_thd, arg)
{

	(void) arg;
	chRegSetThreadName("Encoders reader");

	as5055DoAMasterReset(&encoders);
	
	palEnableLineEvent(encoders.interruptline, PAL_EVENT_MODE_FALLING_EDGE);

	while(1){
		palWaitLineTimeout(encoders.interruptline, TIME_MS2I(5));
		if(!palReadLine(encoders.interruptline)){
			as5055ReadAngleBlocking(&encoders);
		}

	}
	
}

void encodersStartReading(void){
	chThdCreateStatic(encoders_thd_wa, sizeof(encoders_thd_wa), NORMALPRIO, encoders_thd, NULL);
}

AS5055_data_t* encodersGetData(void){
	return encoders.data;
}
