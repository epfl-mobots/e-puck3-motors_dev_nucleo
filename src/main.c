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
#include "uc_usage.h"
#include "gdb.h"

int main(void) {

	/*
	* System initializations.
	* - HAL initialization, this also initializes the configured device drivers
	*   and performs the board-specific initializations.
	* - Kernel initialization, the main() function becomes a thread and the
	*   RTOS is active.
	*/
	halInit();
	chSysInit();

	initGDBEvents();
	gdbStart();

	/*
	* Initializes two serial-over-USB CDC drivers and starts and connects the USB.
	*/
	usbSerialStart();

	palSetLineMode(LINE_OUT_MOT3_PH1_P, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(PAL_LINE(GPIOB, 7U), PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_OUT_MOT4_PH2_N, PAL_MODE_OUTPUT_PUSHPULL);


	while (true) {
		chThdSleepMilliseconds(500);
		palToggleLine(LINE_OUT_MOT3_PH1_P);
		palToggleLine(PAL_LINE(GPIOB, 7U));
		palToggleLine(LINE_OUT_MOT4_PH2_N);
		// chprintf((BaseSequentialStream *) &USB_GDB, "GDB\n");
		// printUcUsage((BaseSequentialStream *) &USB_GDB);
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "SERIAL\n");
		// printUcUsage((BaseSequentialStream *) &USB_SERIAL);
	}
}
