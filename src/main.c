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


static THD_WORKING_AREA(waThread1,128);
static THD_FUNCTION(Thread1,arg) {
  (void)arg;
  chRegSetThreadName("blinker");
  while(true){
		palSetLine(LD1_LINE);
		chThdSleepMilliseconds(50);
		palSetLine(LD2_LINE);
		chThdSleepMilliseconds(100);
		palSetLine(LD3_LINE);
		chThdSleepMilliseconds(150);
		palClearLine(LD1_LINE);
		chThdSleepMilliseconds(50);
		palClearLine(LD2_LINE);
		chThdSleepMilliseconds(100);
		palClearLine(LD3_LINE);
		chThdSleepMilliseconds(150);
  }
}



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

	/*Configure the IO mode*/
	palSetLineMode(LD1_LINE,PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LD2_LINE,PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LD3_LINE,PAL_MODE_OUTPUT_PUSHPULL);

	/*
	palSetLineMode(LINE_OUT_MOT3_PH1_P, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(PAL_LINE(GPIOB, 7U), PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_OUT_MOT4_PH2_N, PAL_MODE_OUTPUT_PUSHPULL);
	 */

	// Configure the Thread that will blink the leds on the boards
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);


	while (true) {
		chThdSleepMilliseconds(500);
		//palToggleLine(LINE_OUT_MOT3_PH1_P);
		//palToggleLine(PAL_LINE(GPIOB, 7U));
		//palToggleLine(LINE_OUT_MOT4_PH2_N);
		// chprintf((BaseSequentialStream *) &USB_GDB, "GDB\n");
		// printUcUsage((BaseSequentialStream *) &USB_GDB);
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "SERIAL\n");
		// printUcUsage((BaseSequentialStream *) &USB_SERIAL);
	}
}
