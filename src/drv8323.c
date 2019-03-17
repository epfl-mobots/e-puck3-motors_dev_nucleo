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

void drv8323WriteConf(DRV8323Config *drv){
	/* Bus acquisition and SPI reprogramming.*/
    spiAcquireBus(drv->spip);
    spiStart(drv->spip, drv->spicfg);

    uint16_t rcv = 0xFFFF;

	drv->spicfg->ssline = drv->ssline;

	spiSelect(drv->spip);
	spiExchange(drv->spip, 1, &drv->registers->fault_status_1, &rcv);
	spiExchange(drv->spip, 1, &drv->registers->fault_status_2, &rcv);
	spiExchange(drv->spip, 1, &drv->registers->driver_control, &rcv);
	spiExchange(drv->spip, 1, &drv->registers->gate_drive_hs, &rcv);
	spiExchange(drv->spip, 1, &drv->registers->gate_drive_ls, &rcv);
	spiExchange(drv->spip, 1, &drv->registers->ocp_control, &rcv);
	spiExchange(drv->spip, 1, &drv->registers->csa_control, &rcv);
	spiUnselect(drv->spip);
	

	/* Releasing the bus.*/
	spiReleaseBus(drv->spip);
}

uint16_t drv8323WriteReg(DRV8323Config *drv, uint16_t reg){
	uint16_t rcv = 0;

	reg |= DRV8322_WRITE;

	/* Bus acquisition and SPI reprogramming.*/
    spiAcquireBus(drv->spip);
    spiStart(drv->spip, drv->spicfg);

	drv->spicfg->ssline = drv->ssline;

	spiSelect(drv->spip);
	spiExchange(drv->spip, 1, &reg, &rcv);
	spiUnselect(drv->spip);

	/* Releasing the bus.*/
    spiReleaseBus(drv->spip);

	return rcv;
}

uint16_t drv8323ReadReg(DRV8323Config *drv, uint16_t reg){
	uint16_t rcv = 0;

	reg |= DRV8322_READ;

	/* Bus acquisition and SPI reprogramming.*/
    spiAcquireBus(drv->spip);
    spiStart(drv->spip, drv->spicfg);

	drv->spicfg->ssline = drv->ssline;

	spiSelect(drv->spip);
	spiExchange(drv->spip, 1, &reg, &rcv);
	spiUnselect(drv->spip);

	/* Releasing the bus.*/
    spiReleaseBus(drv->spip);

	return rcv;
}















