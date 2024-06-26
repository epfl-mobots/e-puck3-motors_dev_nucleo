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

/********************              INTERNAL VARIABLES              ********************/

static thread_t* fault_handler_thd = NULL;
static bool thread_must_pause = FALSE;
static BSEMAPHORE_DECL(fault_handler_bsem, TRUE);
static bool enable_states[NB_OF_GATE_DRIVERS] = {FALSE};

/********************            CONFIGURATION VARIABLES           ********************/

/**
 * SPI config to communicate with a DRV8323 device
 */
static SPIConfig spicfg = {
	.circular = false,
	.end_cb = NULL,
	.ssline = 0,
	.cr1 = SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_CPHA,	//PRESCALER = 010, POL = 0, PHASE = 1
	.cr2 = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0	//16bits
};

/**
 * Config to use to configure a DRV8323
 */
static DRV8323ConfRegisters drv8323cfg = {
	.fault_status_1	= 0,
	.fault_status_2	= 0,
	.driver_control	= 0,
	.gate_drive_hs	= IDRIVEN_HS | IDRIVEP_HS,
	.gate_drive_ls 	= IDRIVEN_LS | IDRIVEP_LS | TDRIVE | CBC,
	.ocp_control 	= VDS_LVL_1 | VDS_LVL_0 | OCP_DEG_0 | OCP_MODE_0 | DEAD_TIME_0,
	.csa_control 	= SEN_LVL | CSA_GAIN_1 /*| VREF_DIV*/ | CSA_FET
};

/**
 * Top config of the DRV8232 to use
 */
static DRV8323Config gateDrivers[NB_OF_GATE_DRIVERS] = {
	{
		.spip = &SPI_GATE_DRIVERS,
		.spicfg = &spicfg,
		.enline = LINE_EN_DRIVER_1,
		.ssline = LINE_CS_DRIVER_1_n,
		.faultline = LINE_FAULT_DRIVER_1_n,
		.registers = &drv8323cfg,
	},
	{
		.spip = &SPI_GATE_DRIVERS,
		.spicfg = &spicfg,
		.enline = LINE_EN_DRIVER_2,
		.ssline = LINE_CS_DRIVER_2_n,
		.faultline = LINE_FAULT_DRIVER_2_n,
		.registers = &drv8323cfg,
	},
	{
		.spip = &SPI_GATE_DRIVERS,
		.spicfg = &spicfg,
		.enline = LINE_EN_DRIVER_3,
		.ssline = LINE_CS_DRIVER_3_n,
		.faultline = LINE_FAULT_DRIVER_3_n,
		.registers = &drv8323cfg,
	},
	{
		.spip = &SPI_GATE_DRIVERS,
		.spicfg = &spicfg,
		.enline = LINE_EN_DRIVER_4,
		.ssline = LINE_CS_DRIVER_4_n,
		.faultline = LINE_FAULT_DRIVER_4_n,
		.registers = &drv8323cfg,
	}
};

/********************               PRIVATE FUNCTIONS              ********************/

/**
 * @brief Callback called when a fault is detected. It sends an event to the fault thread
 * 
 * @param arg 	Device id. Configured during the call of palSetLineCallback()
 */
void gateDriverCb(void* arg){

	chSysLockFromISR();
	//we set the event bit at the position corresponding to the device number.
	//example : 0b00000000000000000000000000001000 corresponds to the device number 3
	chEvtSignalI(fault_handler_thd, (1 << (uint32_t)arg) );
	chSysUnlockFromISR();
}

/**
 * @brief Thread handling the fault detected from the devices. Triggered by the gateDriverCb callback
 * 
 * @param arg 	Unused parameter
 */
static THD_WORKING_AREA(fault_thd_wa, 128);
static THD_FUNCTION(fault_thd, arg)
{

	(void) arg;
	chRegSetThreadName("Gate drivers fault handler");

	uint8_t device_id = 0;
	uint16_t fault_1_reg = 0;
	uint16_t fault_2_reg = 0;

	while(1){
		if(!thread_must_pause){
			//we wait 500ms for one fault event
			eventmask_t evt = chEvtWaitOneTimeout(ALL_EVENTS, TIME_MS2I(500));
			
			if(evt != 0){
				device_id = 0;

				//this loop finds on many times the bit has been shifted, which gives the device number
				while(evt > 0){
					if(evt == 1){
						break;
					}else{
						evt = evt >> 1;
						device_id += 1;
					}
				}

				fault_1_reg = drv8323ReadReg(&gateDrivers[device_id], DRV8323_FAULT_STATUS_1_REG);
				fault_2_reg = drv8323ReadReg(&gateDrivers[device_id], DRV8323_FAULT_STATUS_2_REG);

				/*
				*	To do : do something with the fault detected
				*/

				if(fault_1_reg & VDS_LC){
					chprintf((BaseSequentialStream *) &USB_GDB, "VDS overcurrent fault on the C low-side %d\n", device_id);
				}
				if(fault_1_reg & VDS_HC){
					chprintf((BaseSequentialStream *) &USB_GDB, "VDS overcurrent fault on the C high-side %d\n", device_id);
				}
				if(fault_1_reg & VDS_LB){
					chprintf((BaseSequentialStream *) &USB_GDB, "VDS overcurrent fault on the B low-side %d\n", device_id);
				}
				if(fault_1_reg & VDS_HB){
					chprintf((BaseSequentialStream *) &USB_GDB, "VDS overcurrent fault on the B high-side %d\n", device_id);
				}
				if(fault_1_reg & VDS_LA){
					chprintf((BaseSequentialStream *) &USB_GDB, "VDS overcurrent fault on the A low-side %d\n", device_id);
				}
				if(fault_1_reg & VDS_HA){
					chprintf((BaseSequentialStream *) &USB_GDB, "VDS overcurrent fault on the A high-side %d\n", device_id);
				}
				if(fault_1_reg & OTSD){
					chprintf((BaseSequentialStream *) &USB_GDB, "Overtemperature shutdown %d\n", device_id);
				}
				if(fault_1_reg & UVLO){
					chprintf((BaseSequentialStream *) &USB_GDB, "Undervoltage lockout fault condition %d\n", device_id);
				}
				if(fault_1_reg & GDF){
					chprintf((BaseSequentialStream *) &USB_GDB, "Gate drive fault condition %d\n", device_id);
				}
				if(fault_1_reg & VDS_OCP){
					chprintf((BaseSequentialStream *) &USB_GDB, "VDS monitor overcurrent fault condition %d\n", device_id);
				}
				if(fault_2_reg & VGS_LC){
					chprintf((BaseSequentialStream *) &USB_GDB, "Gate drive fault on the C low-side %d\n", device_id);
				}
				if(fault_2_reg & VGS_HC){
					chprintf((BaseSequentialStream *) &USB_GDB, "Gate drive fault on the C high-side %d\n", device_id);
				}
				if(fault_2_reg & VGS_LB){
					chprintf((BaseSequentialStream *) &USB_GDB, "Gate drive fault on the B low-side %d\n", device_id);
				}
				if(fault_2_reg & VGS_HB){
					chprintf((BaseSequentialStream *) &USB_GDB, "Gate drive fault on the B high-side %d\n", device_id);
				}
				if(fault_2_reg & VGS_LA){
					chprintf((BaseSequentialStream *) &USB_GDB, "Gate drive fault on the A low-side %d\n", device_id);
				}
				if(fault_2_reg & VGS_HA){
					chprintf((BaseSequentialStream *) &USB_GDB, "Gate drive fault on the A high-side %d\n", device_id);
				}
				if(fault_2_reg & CPUV){
					chprintf((BaseSequentialStream *) &USB_GDB, "Charge pump undervoltage fault condition %d\n", device_id);
				}
				if(fault_2_reg & OTW){
					chprintf((BaseSequentialStream *) &USB_GDB, "Overtemperature warning %d\n", device_id);
				}
				if(fault_2_reg & SC_OC){
					chprintf((BaseSequentialStream *) &USB_GDB, "Overcurrent on phase C sense amplifier %d\n", device_id);
				}
				if(fault_2_reg & SB_OC){
					chprintf((BaseSequentialStream *) &USB_GDB, "Overcurrent on phase B sense amplifier %d\n", device_id);
				}
				if(fault_2_reg & SA_OC){
					chprintf((BaseSequentialStream *) &USB_GDB, "Overcurrent on phase A sense amplifier %d\n", device_id);
				}


				//to remove the "variable unused" warning
				(void)fault_1_reg;
				(void)fault_2_reg;

				//chprintf((BaseSequentialStream *) &USB_GDB, "fault detected on device %d\n", device_id);

			}
		}else{
			chBSemWait(&fault_handler_bsem);
		}
	}
}

/**
 * @brief Resumes the thread if paused or creates it if not existing
 */
void _gateDriversResumeThread(void){
	//if the thread doesn't exist, creates it
	if(fault_handler_thd == NULL){
		fault_handler_thd = chThdCreateStatic(fault_thd_wa, sizeof(fault_thd_wa), NORMALPRIO, fault_thd, NULL);
	}else{
		//resumes the thread if already created and if paused
		if(thread_must_pause){
			thread_must_pause = FALSE;
			chBSemSignal(&fault_handler_bsem);
		}
	}
}

/**
 * @brief Suspends the thread only if all devices are off, otherwise does nothing
 */
void _gateDriverSuspendThread(void){

	//searches for at least one enabled device. If none, then pauses the thread
	for(uint8_t i = 0 ; i < NB_OF_GATE_DRIVERS ; i++){
		if(enable_states[i]){
			return;
		}
	}

	thread_must_pause = TRUE;
}

/**
 * @brief Enables the given device and updates the enable_states state.
 * 
 * @param id  Device to enable. See gateDriver_id for choice.
 */
void _gateDriversSetEnable(gateDriver_id id){
	palSetLine(gateDrivers[id].enline);
	enable_states[id] = TRUE;
}

/**
 * @brief Disables the given device and updates the enable_states state.
 * 
 * @param id  Device to disable. See gateDriver_id for choice.
 */
void _gateDriversClearEnable(gateDriver_id id){
	palClearLine(gateDrivers[id].enline);
	enable_states[id] = FALSE;
}

/********************                PUBLIC FUNCTIONS              ********************/

void gateDriversCalibrateCurrentProbe(gateDriver_id id){
	drv8323CalibrateCurrentProbe(&gateDrivers[id]);
}

void gateDriversWriteReg(gateDriver_id id, uint16_t reg){
	drv8323ReadReg(&gateDrivers[id], reg);
}

uint16_t gateDriversReadReg(gateDriver_id id, uint16_t reg){
	return drv8323ReadReg(&gateDrivers[id], reg);
}

void gateDriversEnable(gateDriver_id id){
	if(!enable_states[id]){
		_gateDriversResumeThread();

		palSetLineCallback(gateDrivers[id].faultline, gateDriverCb, (void*) id);
		_gateDriversSetEnable(id);
		//we wait that the driver is completely enabled and ready
		chThdSleepMilliseconds(2);

		drv8323WriteConf(&gateDrivers[id]);
		gateDriversCalibrateCurrentProbe(id);
		/* Enabling events on falling edge of the fault line.
		 * We do it after the power on of the chip to not catch the fault pulse on init from the chip.
		 */
		palEnableLineEvent(gateDrivers[id].faultline, PAL_EVENT_MODE_FALLING_EDGE);
	}
}

void gateDriversEnableAll(void){

	_gateDriversResumeThread();

	for(uint32_t i = 0 ; i < NB_OF_GATE_DRIVERS ; i++){
		if(!enable_states[i]){
			_gateDriversSetEnable(i);
			palSetLineCallback(gateDrivers[i].faultline, gateDriverCb, (void*) i);
		}
	}

	//we wait that the drivers are completely enabled and ready
	chThdSleepMilliseconds(2);

	for(uint8_t i = 0 ; i < NB_OF_GATE_DRIVERS ; i++){
		drv8323WriteConf(&gateDrivers[i]);
		gateDriversCalibrateCurrentProbe(i);
		/* Enabling events on falling edge of the fault line.
		 * We do it after the power on of the chips to not catch the fault pulse on init from the chip.
		 */
		palEnableLineEvent(gateDrivers[i].faultline, PAL_EVENT_MODE_FALLING_EDGE);
	}
}

void gateDriversDisable(gateDriver_id id){
	if(enable_states[id]){
		palDisableLineEvent(gateDrivers[id].faultline);

		_gateDriversClearEnable(id);

		_gateDriverSuspendThread();
	}
}

void gateDriversDisableAll(void){
	for(uint8_t i = 0 ; i < NB_OF_GATE_DRIVERS ; i++){
		gateDriversDisable(i);
	}
}







