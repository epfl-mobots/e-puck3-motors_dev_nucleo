/**
 * @file	main.c
 * @brief  	Main file of the demo brushless command WIP
 * 
 * @written by  	Mohammed-Ismail Ben Salah
 * @creation date   14.02.2019
 */

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "user_shell.h"
#include "uc_usage.h"
#include "gdb.h"
#include "ch.h"
#include "hal.h"
#include "gate_drivers.h"
#include "power_button.h"
#include "usb_pd_controller.h"
#include "custom_io.h"
#include "tim1_motor.h"
#include "adc_motor.h"
#include "adc_tx.h"
#include "encoders.h"


/*===========================================================================*/
/* Define				                                                 */
/*===========================================================================*/


/*===========================================================================*/
/* Macro                                                                     */
/*===========================================================================*/
#define ABS(x)  ( (x<0) ? -x : x )




/*===========================================================================*/
/* Structure                                                                 */
/*===========================================================================*/



/*===========================================================================*/
/* Variables				                                                 */
/*===========================================================================*/


extern binary_semaphore_t dtx_ready;




/*===========================================================================*/
/* Prototypes				                                                 */
/*===========================================================================*/


// UART
void Send_ADT_Uart(AdcDataTx* adt);

/*===========================================================================*/
/* Threads				                                                 */
/*===========================================================================*/
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

static THD_WORKING_AREA(waThread2,128);
static THD_FUNCTION(Thread2,arg) {
  (void)arg;
  chRegSetThreadName("sender");
  while(true){

    chBSemWait(&dtx_ready);
    gADT.data_lock = 1;
    Send_ADT_Uart(&gADT);
    Adt_Reset_Struct(&gADT);
  }
}


/*===========================================================================*/
/* UART Communication                                                        */
/*===========================================================================*/

void Send_ADT_Uart(AdcDataTx* adt)
{

  streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)"START", 5);

  streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)"CH0", 3);
  streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)&(adt->nb_points), sizeof(uint16_t));
  streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)&(adt->data[0]), sizeof(uint16_t) * adt->nb_points);

  streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)"CH1", 3);
  streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)&adt->nb_points, sizeof(uint16_t));
  streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)&adt->data[1], sizeof(uint16_t) * adt->nb_points);

  streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)"CH2", 3);
  streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)&adt->nb_points, sizeof(uint16_t));
  streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)&adt->data[2], sizeof(uint16_t) * adt->nb_points);

  streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)"CH3", 3);
  streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)&adt->nb_points, sizeof(uint16_t));
  streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)&adt->data[3], sizeof(uint16_t) * adt->nb_points);

}


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

	// Debug MCU Config
	DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP; // Clock and outputs of TIM 1 are disabled when the core is halted
	chBSemObjectInit(&dtx_ready,true);

	Adt_Reset_Struct(&gADT);
	Zcs_Reset_Struct(&gZCS);

	/* Debug IO for interrupt timings */
	palSetLineMode(DEBUG_INT_LINE,PAL_MODE_OUTPUT_PUSHPULL);
	palClearLine(DEBUG_INT_LINE);
    palSetLineMode(DEBUG_INT_LINE2,PAL_MODE_OUTPUT_PUSHPULL);
    palClearLine(DEBUG_INT_LINE2);
    palSetLineMode(DEBUG_INT_LINE3,PAL_MODE_OUTPUT_PUSHPULL);
    palClearLine(DEBUG_INT_LINE3);
    palSetLineMode(DEBUG_INT_LINE4,PAL_MODE_OUTPUT_PUSHPULL);
    palClearLine(DEBUG_INT_LINE4);

    /* Motor 1 IO configuration when High-Impedance due to TIM 1
     * Phase 1 P : PA8  AF : 1
     * Phase 1 N : PB13 AF : 1
     * Phase 2 P : PE11 AF : 1
     * Phase 2 N : PE10 AF : 1
     * Phase 3 P : PA10 AF : 1
     * Phase 3 N : PE12 AF : 1
     *
     * */
    initTIM1MotorIo();

	/*
	* Initializes two serial-over-USB CDC drivers and starts and connects the USB.
	*/
	usbSerialStart();

	shellInit();
	gateDriversEnableAll();

	adc3Start();

    timer1Start();



	// Configure the Thread that will blink the leds on the boards
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);

    // Configure the Thread that will blink the leds on the boards
    chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO + 2, Thread2, NULL);

	while (true)
	{
		/* Send ADC values */

		if(isUSBConfigured()){
			//spawns the shell if the usb is connected
			spawn_shell();
		}
		chThdSleepMilliseconds(500);

    static float percent = 90;

    if(palReadLine(LINE_NUCLEO_USER_BUTTON)){
      percent -= 5;
      if(percent < 0){
        percent = 90;
      }
      (&PWMD1)->tim->CCR[kTimChannel1]  =  (percent/100) * PERIOD_PWM_20_KHZ - 1;  // Select the quarter-Period to overflow
      (&PWMD1)->tim->CCR[kTimChannel2]  =  (percent/100) * PERIOD_PWM_20_KHZ - 1;  // Select the quarter-Period to overflow
      (&PWMD1)->tim->CCR[kTimChannel3]  =  (percent/100) * PERIOD_PWM_20_KHZ - 1;  // Select the quarter-Period to overflow
      chThdSleepMilliseconds(500);
      chprintf((BaseSequentialStream *)&USB_GDB, "duty cycle = %f\r\n",100-percent);
    }

   
		
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "fault status 1 	= 0x%x\n", gateDriversReadReg(DRV8323_1, DRV8323_FAULT_STATUS_1_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "fault status 2 	= 0x%x\n", gateDriversReadReg(DRV8323_1, DRV8323_FAULT_STATUS_2_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "driver control  = 0x%x\n", gateDriversReadReg(DRV8323_1, DRV8323_DRIVER_CONTROL_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "gate driver hs 	= 0x%x\n", gateDriversReadReg(DRV8323_1, DRV8323_GATE_DRIVE_HS_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "gate driver hs 	= 0x%x\n", gateDriversReadReg(DRV8323_1, DRV8323_GATE_DRIVE_LS_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "ocp control 	= 0x%x\n", gateDriversReadReg(DRV8323_1, DRV8323_OCP_CONTROL_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "csa control 	= 0x%x\n\n", gateDriversReadReg(DRV8323_1, DRV8323_CSA_CONTROL_REG));
		
	}
}
