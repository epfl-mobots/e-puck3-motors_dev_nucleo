/**
 * @file	drv8323.c
 * @brief  	File containing the driver to communicate by SPI with a DRV8323 gate driver
 * 
 * @written by  	Eliot Ferragni
 * @creation date	14.03.2019
 */
 
#include "ch.h"
#include "hal.h"
#include "drv8323.h"
/**
 * @brief   Structure representing a DRV8323 driver.
 */
typedef struct {
	/**
	 * SPI driver used
	 */
	SPIDriver*			spip;
	/**
	 * SPI config used. 
	 * Note 1 :	The ssline field of spicfg is managed by
	 * 			the drv8323 driver so you can let it uninitialized.
	 * Note 2 :	The spicfg should not be declared as static const.
	 * 			Use static instead.
	 * Note 3 :	The SPI com should be configured to use 16bits words
	 */
	SPIConfig 			spicfg;
	/**
	 * Enables lines of the DRV8323 conected
	 */
	ioline_t			enline[NB_OF_DRV8323];
	/**
	 * chip select lines of the DRV8323 conected
	 */
	ioline_t			ssline[NB_OF_DRV8323];
	/**
	 * Fault lines of the DRV8323 conected
	 */
	ioline_t			faultline[NB_OF_DRV8323];

} DRV8323Driver;

/**
 * @brief   Structure representing a DRV8323 Config.
 */
typedef struct {
	uint16_t 	fault_status_1;
	uint16_t	fault_status_2;
	uint16_t	driver_control;
	uint16_t	gate_drive_hs;
	uint16_t	gate_drive_ls;
	uint16_t	ocp_control;
	uint16_t	csa_control;
} DRV8323Config;

static DRV8323Driver drv8323 = {
	.spip = &SPI_DRV8323,
	.spicfg = {
		.circular = false,
 		.end_cb = NULL,
		.ssline = 0,
		.cr1 = SPI_CR1_BR_1 | SPI_CR1_CPHA,	//PRESCALER = 010, POL = 0, PHASE = 1
		.cr2 = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0	//16bits
	},
	.enline = {
		LINE_EN_DRIVER_1,
		LINE_EN_DRIVER_2,
		LINE_EN_DRIVER_3,
		LINE_EN_DRIVER_4
	},
	.ssline = {
		LINE_CS_DRIVER_1_n,
		LINE_CS_DRIVER_2_n,
		LINE_CS_DRIVER_3_n,
		LINE_CS_DRIVER_4_n
	},
	.faultline = {
		LINE_FAULT_DRIVER_1_n,
		LINE_FAULT_DRIVER_2_n,
		LINE_FAULT_DRIVER_3_n,
		LINE_FAULT_DRIVER_4_n
	},
};

static DRV8323Config default_config {
	.fault_status_1	= 0,
	.fault_status_2	= 0,
	.driver_control	= 0,
	.gate_drive_hs	= 0,
	.gate_drive_ls 	= 0,
	.ocp_control 	= 0,
	.csa_control 	= 0
};

void drv8323WriteConf(DRV8323Driver *drv, DRV8323Config *config){
	/* Bus acquisition and SPI reprogramming.*/
    spiAcquireBus(drv->spip);
    spiStart(drv->spip, &drv->spicfg);

    uint16_t rcv = 0xFFFF;

	for(uint8_t i = 0 ; i < NB_OF_DRV8323 ; i++){

		drv->spicfg.ssline = drv->ssline[i];

		spiSelect(drv->spip);
		spiExchange(drv->spip, 1, &default_reg1, &rcv);
		spiUnselect(drv->spip);
	}

	/* Releasing the bus.*/
	spiReleaseBus(drv->spip);
}

uint16_t drv8323WriteReg(DRV8323Driver *drv, uint16_t reg, drv8232_id_t target){
	uint16_t rcv = 0;

	if(target > NB_OF_DRV8323){
		return 0xB0B0;
	}

	reg |= (1 << 16);

	/* Bus acquisition and SPI reprogramming.*/
    spiAcquireBus(drv->spip);
    spiStart(drv->spip, &drv->spicfg);

	drv->spicfg.ssline = drv->ssline[target];

	spiSelect(drv->spip);
	spiExchange(drv->spip, 1, &reg, &rcv);
	spiUnselect(drv->spip);

	/* Releasing the bus.*/
    spiReleaseBus(drv->spip);

	return rcv;
}

uint16_t drv8323ReadReg(DRV8323Driver *drv, uint16_t reg, drv8232_id_t target){
	uint16_t rcv = 0;

	if(target > NB_OF_DRV8323){
		return 0xB0B0;
	}

	reg &= ~(1 << 16);

	/* Bus acquisition and SPI reprogramming.*/
    spiAcquireBus(drv->spip);
    spiStart(drv->spip, &drv->spicfg);

	drv->spicfg.ssline = drv->ssline[target];

	spiSelect(drv->spip);
	spiExchange(drv->spip, 1, &reg, &rcv);
	spiUnselect(drv->spip);

	/* Releasing the bus.*/
    spiReleaseBus(drv->spip);

	return rcv;
}















