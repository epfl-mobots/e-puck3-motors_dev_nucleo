/**
 * @file	gate_driver.c
 * @brief  	Library to manage the gate drivers
 * 
 * @written by  	Eliot Ferragni
 * @creation date	17.03.2019
 */
 
#include "main.h"
#include "gate_drivers.h"
#include "drv8323.h"

static SPIConfig spicfg = {
	.circular = false,
	.end_cb = NULL,
	.ssline = 0,
	.cr1 = SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0| SPI_CR1_CPHA,	//PRESCALER = 010, POL = 0, PHASE = 1
	.cr2 = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0	//16bits
};

static DRV8323ConfRegisters drv8323cfg = {
	.fault_status_1	= 0,
	.fault_status_2	= 0,
	.driver_control	= 0,
	.gate_drive_hs	= IDRIVEN_HS | IDRIVEP_HS,
	.gate_drive_ls 	= IDRIVEN_LS | IDRIVEP_LS | TDRIVE | CBC,
	.ocp_control 	= VDS_LVL_3 | VDS_LVL_0 | OCP_DEG_0 | OCP_MODE_0 | DEAD_TIME_0,
	.csa_control 	= SEN_LVL | CSA_GAIN_1 | VREF_DIV | CSA_FET
};

DRV8323Config drv8323_1 = {
	.spip = &SPI_DRV8323,
	.spicfg = &spicfg,
	.enline = LINE_EN_DRIVER_1,
	.ssline = LINE_CS_DRIVER_1_n,
	.faultline = LINE_FAULT_DRIVER_1_n,
	.registers = &drv8323cfg,
};

DRV8323Config drv8323_2 = {
	.spip = &SPI_DRV8323,
	.spicfg = &spicfg,
	.enline = LINE_EN_DRIVER_2,
	.ssline = LINE_CS_DRIVER_2_n,
	.faultline = LINE_FAULT_DRIVER_2_n,
	.registers = &drv8323cfg,
};

DRV8323Config drv8323_3 = {
	.spip = &SPI_DRV8323,
	.spicfg = &spicfg,
	.enline = LINE_EN_DRIVER_3,
	.ssline = LINE_CS_DRIVER_3_n,
	.faultline = LINE_FAULT_DRIVER_3_n,
	.registers = &drv8323cfg,
};

DRV8323Config drv8323_4 = {
	.spip = &SPI_DRV8323,
	.spicfg = &spicfg,
	.enline = LINE_EN_DRIVER_4,
	.ssline = LINE_CS_DRIVER_4_n,
	.faultline = LINE_FAULT_DRIVER_4_n,
	.registers = &drv8323cfg,
};

void gateDriversEnable(DRV8323Config* drv){
	palSetLine(drv->enline);
}

void gateDriversDisable(DRV8323Config* drv){
	palClearLine(drv->enline);
}

void gateDriversInit(void){
	gateDriversEnable(&drv8323_1);
	gateDriversEnable(&drv8323_2);
	gateDriversEnable(&drv8323_3);
	gateDriversEnable(&drv8323_4);
	chThdSleepMilliseconds(1);
	drv8323WriteConf(&drv8323_1);
	drv8323WriteConf(&drv8323_2);	
	drv8323WriteConf(&drv8323_3);
	drv8323WriteConf(&drv8323_4);
}







