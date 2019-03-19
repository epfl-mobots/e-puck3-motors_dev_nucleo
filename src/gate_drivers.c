/**
 * @file	gate_driver.c
 * @brief  	Library to manage the gate drivers
 * 
 * @written by  	Eliot Ferragni
 * @creation date	17.03.2019
 */
 
#include "main.h"
#include "gate_drivers.h"

static thread_t* fault_handler = NULL;

static SPIConfig spicfg = {
	.circular = false,
	.end_cb = NULL,
	.ssline = 0,
	.cr1 = SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_CPHA,	//PRESCALER = 010, POL = 0, PHASE = 1
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

static DRV8323Config gateDrivers[NB_OF_DRV8323] = {
	{
		.spip = &SPI_DRV8323,
		.spicfg = &spicfg,
		.enline = LINE_EN_DRIVER_1,
		.ssline = LINE_CS_DRIVER_1_n,
		.faultline = LINE_FAULT_DRIVER_1_n,
		.registers = &drv8323cfg,
	},
	{
		.spip = &SPI_DRV8323,
		.spicfg = &spicfg,
		.enline = LINE_EN_DRIVER_2,
		.ssline = LINE_CS_DRIVER_2_n,
		.faultline = LINE_FAULT_DRIVER_2_n,
		.registers = &drv8323cfg,
	},
	{
		.spip = &SPI_DRV8323,
		.spicfg = &spicfg,
		.enline = LINE_EN_DRIVER_3,
		.ssline = LINE_CS_DRIVER_3_n,
		.faultline = LINE_FAULT_DRIVER_3_n,
		.registers = &drv8323cfg,
	},
	{
		.spip = &SPI_DRV8323,
		.spicfg = &spicfg,
		.enline = LINE_EN_DRIVER_4,
		.ssline = LINE_CS_DRIVER_4_n,
		.faultline = LINE_FAULT_DRIVER_4_n,
		.registers = &drv8323cfg,
	}
};

void gateDriverCb(void* arg){

	chSysLockFromISR();
	//we set the bit at the position corresponding to the device number
	//example : 0b00000000000000000000000000001000 corresponds to the device number 3
	chEvtSignalI(fault_handler, (1 << (uint32_t)arg) );
	chSysUnlockFromISR();
}

static THD_WORKING_AREA(gate_drivers_thd_wa, 128);
static THD_FUNCTION(gate_drivers_thd, arg)
{

	(void) arg;
	chRegSetThreadName("Gate drivers fault handler");

	uint8_t device_id = 0;
	uint16_t fault_1_reg = 0;
	uint16_t fault_2_reg = 0;

	while(1){
		//we wait an infinite time for one fault event
		eventmask_t evt = chEvtWaitOne(ALL_EVENTS);
		

		device_id = 0;

		//this loop finds on many times the bit has been shifted, which gives the device number
		while(evt > 0){
			if(evt == 1){
				break;
			}else{
				evt = evt >> 1;
				device_id +=1;
			}
		}

		fault_1_reg = drv8323ReadReg(&gateDrivers[device_id], FAULT_STATUS_1_REG);
		fault_2_reg = drv8323ReadReg(&gateDrivers[device_id], FAULT_STATUS_2_REG);

		/*
		*	To do : do something with the fault detected
		*/

		//to remove the "variable unused" warning
		(void)fault_1_reg;
		(void)fault_2_reg;
	}
}

void gateDriversEnable(DRV8323Config* drv){
	palSetLine(drv->enline);
}

void gateDriversDisable(DRV8323Config* drv){
	palClearLine(drv->enline);
}

void gateDriversWriteReg(gateDriver_id id, uint16_t reg){
	drv8323ReadReg(&gateDrivers[id], reg);
}

uint16_t gateDriversReadReg(gateDriver_id id, uint16_t reg){
	return drv8323ReadReg(&gateDrivers[id], reg);
}

void gateDriversInit(void){

	fault_handler = chThdCreateStatic(gate_drivers_thd_wa, sizeof(gate_drivers_thd_wa), NORMALPRIO, gate_drivers_thd, NULL);

	for(uint32_t i = 0 ; i < NB_OF_DRV8323 ; i++){
		palSetLineCallback(gateDrivers[i].faultline, gateDriverCb, (void*) i);
		gateDriversEnable(&gateDrivers[i]);
	}

	//we wait that the drivers are completely enabled and ready
	chThdSleepMilliseconds(1);

	for(uint8_t i = 0 ; i < NB_OF_DRV8323 ; i++){
		drv8323WriteConf(&gateDrivers[i]);
		/* Enabling events on falling edge of the fault line.
		 * We do it after the power on of the chips to not catch the fault pulse on init from the chip.
		 */
		palEnableLineEvent(gateDrivers[i].faultline, PAL_EVENT_MODE_FALLING_EDGE);
	}
}







