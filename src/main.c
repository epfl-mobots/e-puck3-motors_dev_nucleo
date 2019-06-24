/**
 * @file	main.c
 * @brief  	Main file of the e-puck2_programmer firmware used by the onboard programmer of the
 * 			e-puck2 educational robot.
 * 
 * @written by  	Eliot Ferragni
 * @creation date	18.06.2018
 */

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "user_shell.h"
#include "uc_usage.h"
#include "gdb.h"
#include "gate_drivers.h"
#include "power_button.h"
#include "usb_pd_controller.h"
#include "encoders.h"

static THD_WORKING_AREA(waThread1,128);
static THD_FUNCTION(Thread1,arg) {
  (void)arg;
  chRegSetThreadName("blinker");
  while(true){
		palClearLine(LINE_STATUS_LED1_RED);
		chThdSleepMilliseconds(50);
		palClearLine(LINE_STATUS_LED2_RED);
		chThdSleepMilliseconds(50);
		palClearLine(LINE_STATUS_LED3_RED);
		chThdSleepMilliseconds(50);
		palClearLine(LINE_STATUS_LED1_GREEN);
		chThdSleepMilliseconds(50);
		palClearLine(LINE_STATUS_LED2_GREEN);
		chThdSleepMilliseconds(50);
		palClearLine(LINE_STATUS_LED3_GREEN);
		chThdSleepMilliseconds(50);
		palClearLine(LINE_STATUS_LED1_BLUE);
		chThdSleepMilliseconds(50);
		palClearLine(LINE_STATUS_LED2_BLUE);
		chThdSleepMilliseconds(50);
		palClearLine(LINE_STATUS_LED3_BLUE);
		chThdSleepMilliseconds(50);
		palSetLine(LINE_STATUS_LED1_RED);
		chThdSleepMilliseconds(50);
		palSetLine(LINE_STATUS_LED2_RED);
		chThdSleepMilliseconds(50);
		palSetLine(LINE_STATUS_LED3_RED);
		chThdSleepMilliseconds(50);
		palSetLine(LINE_STATUS_LED1_GREEN);
		chThdSleepMilliseconds(50);
		palSetLine(LINE_STATUS_LED2_GREEN);
		chThdSleepMilliseconds(50);
		palSetLine(LINE_STATUS_LED3_GREEN);
		chThdSleepMilliseconds(50);
		palSetLine(LINE_STATUS_LED1_BLUE);
		chThdSleepMilliseconds(50);
		palSetLine(LINE_STATUS_LED2_BLUE);
		chThdSleepMilliseconds(50);
		palSetLine(LINE_STATUS_LED3_BLUE);
		chThdSleepMilliseconds(50);
  }
}


const ioline_t motor_pins[NB_MOTORS][NB_PHASES][NB_TRANSISTORS] = {
	{
		{LINE_OUT_MOT1_PH1_P, LINE_OUT_MOT1_PH1_N},
		{LINE_OUT_MOT1_PH2_P, LINE_OUT_MOT1_PH2_N},
		{LINE_OUT_MOT1_PH3_P, LINE_OUT_MOT1_PH3_N}
	},
	{
		{LINE_OUT_MOT2_PH1_P, LINE_OUT_MOT2_PH1_N},
		{LINE_OUT_MOT2_PH2_P, LINE_OUT_MOT2_PH2_N},
		{LINE_OUT_MOT2_PH3_P, LINE_OUT_MOT2_PH3_N}
	},
	{
		{LINE_OUT_MOT3_PH1_P, LINE_OUT_MOT3_PH1_N},
		{LINE_OUT_MOT3_PH2_P, LINE_OUT_MOT3_PH2_N},
		{LINE_OUT_MOT3_PH3_P, LINE_OUT_MOT3_PH3_N}
	},
	{
		{LINE_OUT_MOT4_PH1_P, LINE_OUT_MOT4_PH1_N},
		{LINE_OUT_MOT4_PH2_P, LINE_OUT_MOT4_PH2_N},
		{LINE_OUT_MOT4_PH3_P, LINE_OUT_MOT4_PH3_N}
	}
};

/*===========================================================================*/
/* Main.                                                       */
/*===========================================================================*/

int main(void) {

	powerButtonStartSequence();

	/*
	* System initializations.
	* - HAL initialization, this also initializes the configured device drivers
	*   and performs the board-specific initializations.
	* - Kernel initialization, the main() function becomes a thread and the
	*   RTOS is active.
	*/
	halInit();
	chSysInit();

	powerButtonStart();

	initGDBEvents();
	gdbStart();

	usbPDControllerStart();

	/*
	* Initializes two serial-over-USB CDC drivers and starts and connects the USB.
	*/
	usbSerialStart();

	shellInit();
	gateDriversEnableAll();

	for(uint8_t i = 0 ; i < NB_MOTORS ; i++){
		for(uint8_t j = 0 ; j < NB_PHASES ; j++){
			for(uint8_t k = 0 ; k < NB_TRANSISTORS ; k++){
				palSetLineMode(motor_pins[i][j][k], PAL_MODE_OUTPUT_PUSHPULL);
				palClearLine(motor_pins[i][j][k]);
				//chprintf((BaseSequentialStream *) &USB_GDB, "GPIO: 0x%x pin: %d\n",PAL_PORT(motor_pins[i][j][k]), PAL_PAD(motor_pins[i][j][k]));
			}
		}
	}

	// Configure the Thread that will blink the leds on the boards
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);
	encodersStartReading();
	encoders_data_t *data;
	while (true) {
		if(isUSBConfigured()){
			//spawns the shell if the usb is connected
			spawn_shell();
		}
		chThdSleepMilliseconds(1);

		data = encodersGetData();
		chprintf((BaseSequentialStream *) &USB_SERIAL, "Hi = %d, Lo = %d, Angle 1 = %.2f, Hi = %d, Lo = %d, Angle 2 = %.2f\n",data[0].alarm_hi, data[0].alarm_lo, data[0].angle, data[1].alarm_hi, data[1].alarm_lo, data[1].angle);
		
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "fault status 1 	= 0x%x\n", gateDriversReadReg(GATE_DRIVER_1, DRV8323_FAULT_STATUS_1_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "fault status 2 	= 0x%x\n", gateDriversReadReg(GATE_DRIVER_1, DRV8323_FAULT_STATUS_2_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "driver control  = 0x%x\n", gateDriversReadReg(GATE_DRIVER_1, DRV8323_DRIVER_CONTROL_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "gate driver hs 	= 0x%x\n", gateDriversReadReg(GATE_DRIVER_1, DRV8323_GATE_DRIVE_HS_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "gate driver hs 	= 0x%x\n", gateDriversReadReg(GATE_DRIVER_1, DRV8323_GATE_DRIVE_LS_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "ocp control 	= 0x%x\n", gateDriversReadReg(GATE_DRIVER_1, DRV8323_OCP_CONTROL_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "csa control 	= 0x%x\n\n", gateDriversReadReg(GATE_DRIVER_1, DRV8323_CSA_CONTROL_REG));
		
	}
}
