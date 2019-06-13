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
#include "drv8323.h"

void drv8323WriteConf(DRV8323Config *drv){

	uint16_t rx = 0xFFFF;
    uint16_t tx = 0;

	/* Bus acquisition and SPI reprogramming.*/
    spiAcquireBus(drv->spip);

    utilityReconfigureSPI(drv->spip, drv->spicfg);

    //we need a pullup for the MISO when communicating with DRV8323
    uint16_t pupdr = utilityChangePUPDRGpio(LINE_SPI3_MISO, PAL_STM32_PUPDR_PULLUP);
    //sets the chip select line to the good one
	drv->spicfg->ssline = drv->ssline;

	//we need to release the CS pin between each word

	spiSelect(drv->spip);
	tx = DRV8323_WRITE | DRV8323_FAULT_STATUS_1_REG | drv->registers->fault_status_1;
	spiExchange(drv->spip, 1, &tx, &rx);
	spiUnselect(drv->spip);

	spiSelect(drv->spip);
	tx = DRV8323_WRITE | DRV8323_FAULT_STATUS_2_REG | drv->registers->fault_status_2;
	spiExchange(drv->spip, 1, &tx, &rx);
	spiUnselect(drv->spip);

	spiSelect(drv->spip);
	tx = DRV8323_WRITE | DRV8323_DRIVER_CONTROL_REG | drv->registers->driver_control;
	spiExchange(drv->spip, 1, &tx, &rx);
	spiUnselect(drv->spip);

	spiSelect(drv->spip);
	tx = DRV8323_WRITE | DRV8323_GATE_DRIVE_HS_REG | drv->registers->gate_drive_hs;
	spiExchange(drv->spip, 1, &tx, &rx);
	spiUnselect(drv->spip);

	spiSelect(drv->spip);
	tx = DRV8323_WRITE | DRV8323_GATE_DRIVE_LS_REG | drv->registers->gate_drive_ls;
	spiExchange(drv->spip, 1, &tx, &rx);
	spiUnselect(drv->spip);
	
	spiSelect(drv->spip);
	tx = DRV8323_WRITE | DRV8323_OCP_CONTROL_REG | drv->registers->ocp_control;
	spiExchange(drv->spip, 1, &tx, &rx);
	spiUnselect(drv->spip);

	spiSelect(drv->spip);
	tx = DRV8323_WRITE | DRV8323_CSA_CONTROL_REG | drv->registers->csa_control;
	spiExchange(drv->spip, 1, &tx, &rx);
	spiUnselect(drv->spip);

	//we restore the original pupdr state
	utilityChangePUPDRGpio(LINE_SPI3_MISO, pupdr);

	/* Releasing the bus.*/
	spiReleaseBus(drv->spip);
}

uint16_t drv8323WriteReg(DRV8323Config *drv, uint16_t reg){
	uint16_t rcv = 0;

	reg |= DRV8323_WRITE;

	/* Bus acquisition and SPI reprogramming.*/
    spiAcquireBus(drv->spip);
    utilityReconfigureSPI(drv->spip, drv->spicfg);

    //we need a pullup for the MISO when communicating with DRV8323
    uint16_t pupdr = utilityChangePUPDRGpio(LINE_SPI3_MISO, PAL_STM32_PUPDR_PULLUP);
    //sets the chip select line to the good one
	drv->spicfg->ssline = drv->ssline;

	spiSelect(drv->spip);
	spiExchange(drv->spip, 1, &reg, &rcv);
	spiUnselect(drv->spip);

	//we restore the original pupdr state
	utilityChangePUPDRGpio(LINE_SPI3_MISO, pupdr);

	/* Releasing the bus.*/
    spiReleaseBus(drv->spip);

	return rcv;
}

uint16_t drv8323ReadReg(DRV8323Config *drv, uint16_t reg){
	uint16_t rcv = 0;

	reg |= DRV8323_READ;

	/* Bus acquisition and SPI reprogramming.*/
    spiAcquireBus(drv->spip);
    utilityReconfigureSPI(drv->spip, drv->spicfg);

    //we need a pullup for the MISO when communicating with DRV8323
    uint16_t pupdr = utilityChangePUPDRGpio(LINE_SPI3_MISO, PAL_STM32_PUPDR_PULLUP);
    //sets the chip select line to the good one
	drv->spicfg->ssline = drv->ssline;

	spiSelect(drv->spip);
	spiExchange(drv->spip, 1, &reg, &rcv);
	spiUnselect(drv->spip);

	//we restore the original pupdr state
	utilityChangePUPDRGpio(LINE_SPI3_MISO, pupdr);

	/* Releasing the bus.*/
    spiReleaseBus(drv->spip);

	return rcv;
}















