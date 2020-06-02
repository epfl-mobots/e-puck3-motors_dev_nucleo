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
#include "gdb.h"
#include "ch.h"
#include "hal.h"
#include "gate_drivers.h"
#include "power_button.h"
#include "usb_pd_controller.h"
#include "custom_io.h"
#include "encoders.h"
#include "motors.h"
#include "threads_utilities.h"


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



/*===========================================================================*/
/* Prototypes				                                                 */
/*===========================================================================*/


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

#define THD15_WA_SIZE   THD_WORKING_AREA_SIZE(128)
static THD_FUNCTION(Thd15, arg) {

    chRegSetThreadName(__FUNCTION__);
    //logThisThreadTimestamps();
    (void)arg;
	systime_t time;
	time = chVTGetSystemTime() + TIME_MS2I(1000);
    while(1){
        for(uint32_t i = 0 ; i < 1350000 ; i++){
            __asm__ volatile ("nop");
        }
        chThdSleepMilliseconds(80);
        if(chVTGetSystemTime() > time){
        	break;
        }
    }
}

static THD_WORKING_AREA(waThd16, 128);
static THD_FUNCTION(Thd16, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
	systime_t time;
	time = chVTGetSystemTime() + TIME_MS2I(1000);
    while(1){
        for(uint32_t i = 0 ; i < 1350000 ; i++){
            __asm__ volatile ("nop");
        }
        chThdSleepMilliseconds(80);
        if(chVTGetSystemTime() > time){
        	break;
        }
    }
}

static THD_WORKING_AREA(waThd17, 128);
#define THD17_WA_SIZE   THD_WORKING_AREA_SIZE(128)
static THD_FUNCTION(Thd17, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
	systime_t time;
	time = chVTGetSystemTime() + TIME_MS2I(1000);
    while(1){
        for(uint32_t i = 0 ; i < 1350000 ; i++){
            __asm__ volatile ("nop");
        }
        chThdSleepMilliseconds(80);
        if(chVTGetSystemTime() > time){
        	break;
        }
    }
}

static THD_WORKING_AREA(waThd18, 128);
static THD_FUNCTION(Thd18, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
	systime_t time;
	time = chVTGetSystemTime() + TIME_MS2I(600);
    while(1){
        for(uint32_t i = 0 ; i < 1350000 ; i++){
            __asm__ volatile ("nop");
        }
        chThdSleepMilliseconds(80);
        if(chVTGetSystemTime() > time){
        	break;
        }
    }
}

static THD_WORKING_AREA(waThd19, 128);
#define THD19_WA_SIZE   THD_WORKING_AREA_SIZE(128)
static THD_FUNCTION(Thd19, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
	systime_t time;
	time = chVTGetSystemTime() + TIME_MS2I(100);
    while(1){
        for(uint32_t i = 0 ; i < 1350000 ; i++){
            __asm__ volatile ("nop");
        }
        chThdSleepMilliseconds(80);
        if(chVTGetSystemTime() > time){
        	break;
        }
    }
}

static THD_WORKING_AREA(waThd20, 128);
static THD_FUNCTION(Thd20, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
	systime_t time;
	time = chVTGetSystemTime() + TIME_MS2I(500);
    while(1){
        for(uint32_t i = 0 ; i < 1350000 ; i++){
            __asm__ volatile ("nop");
        }
        chThdSleepMilliseconds(80);
        if(chVTGetSystemTime() > time){
        	break;
        }
    }
}

static THD_WORKING_AREA(waThd21, 128);
#define THD21_WA_SIZE   THD_WORKING_AREA_SIZE(128)
static THD_FUNCTION(Thd21, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
	systime_t time;
	time = chVTGetSystemTime() + TIME_MS2I(100);
    while(1){
        for(uint32_t i = 0 ; i < 1350000 ; i++){
            __asm__ volatile ("nop");
        }
        chThdSleepMilliseconds(80);
        if(chVTGetSystemTime() > time){
        	break;
        }
    }
}

/*===========================================================================*/
/* Main.                                                       */
/*===========================================================================*/

int main(void) {
	logNextCreatedThreadsTimestamps();
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
	logThisThreadTimestamps();

	powerButtonStart();

	// initGDBEvents();
	// gdbStart();

	// usbPDControllerStart();

	// Debug MCU Config
	DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP | DBGMCU_APB2_FZ_DBG_TIM8_STOP; // Clock and outputs of TIM 1 are disabled when the core is halted
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP | DBGMCU_APB1_FZ_DBG_TIM3_STOP | DBGMCU_APB1_FZ_DBG_TIM4_STOP;

	/* Debug IO for interrupt timings */
	palSetLineMode(DEBUG_INT_LINE,PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palClearLine(DEBUG_INT_LINE);
  palSetLineMode(DEBUG_INT_LINE2,PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
  palClearLine(DEBUG_INT_LINE2);
  palSetLineMode(DEBUG_INT_LINE3,PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
  palClearLine(DEBUG_INT_LINE3);
  palSetLineMode(DEBUG_INT_LINE4,PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
  palClearLine(DEBUG_INT_LINE4);

	/*
	* Initializes two serial-over-USB CDC drivers and starts and connects the USB.
	*/
	usbSerialStart();

	// shellInit();
	// gateDriversEnableAll();
 //  gateDriversDisable(GATE_DRIVER_3);

 //  motorsStart();

 //  encodersStartReading();
	//dontLogNextCreatedThreadsTimestamps();
	// Configure the Thread that will blink the leds on the boards
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);

	chThdCreateFromHeap(NULL, THD15_WA_SIZE,"", NORMALPRIO, Thd15, NULL);
    chThdSleepMilliseconds(100);
    chThdCreateStatic(waThd16, sizeof(waThd16),NORMALPRIO, Thd16, NULL);
    chThdSleepMilliseconds(100);
    chThdCreateStatic(waThd17, sizeof(waThd17), NORMALPRIO, Thd17, NULL);
    // chThdCreateFromHeap(NULL, THD17_WA_SIZE, NORMALPRIO, Thd17, NULL);
    chThdSleepMilliseconds(100);
    chThdCreateStatic(waThd18, sizeof(waThd18), NORMALPRIO, Thd18, NULL);
    chThdSleepMilliseconds(100);
    chThdCreateStatic(waThd19, sizeof(waThd19), NORMALPRIO, Thd19, NULL);
    // chThdCreateFromHeap(NULL, THD19_WA_SIZE, NORMALPRIO, Thd19, NULL);
    chThdSleepMilliseconds(100);
    chThdCreateStatic(waThd20, sizeof(waThd20), NORMALPRIO, Thd20, NULL);

    chThdSleepMilliseconds(2000);
    setTriggerTimestamps(__FUNCTION__);
    chThdCreateStatic(waThd21, sizeof(waThd21), NORMALPRIO, Thd21, NULL);
    // chThdCreateFromHeap(NULL, THD21_WA_SIZE, NORMALPRIO, Thd21, NULL);
    chThdCreateFromHeap(NULL, THD15_WA_SIZE,"", NORMALPRIO, Thd15, NULL);

    //chThdCreateStatic(waThd16, sizeof(waThd16), NORMALPRIO, Thd16, NULL);

  // motorSetDutyCycle(BRUSHLESS_MOTOR_1, 10);
  // motorSetDutyCycle(BRUSHLESS_MOTOR_2, 10);
  // motorSetDutyCycle(BRUSHLESS_MOTOR_3, 10);
  // motorSetDutyCycle(BRUSHLESS_MOTOR_4, 10);
  
	while (true)
	{
		/* Send ADC values */

		if(isUSBConfigured()){
			//spawns the shell if the usb is connected
			spawn_shell();
		}
		chThdSleepMilliseconds(100);

    // chprintf((BaseSequentialStream *)&USB_GDB, "rpm = %f, current = %f \r\n",motorsGetRPM(BRUSHLESS_MOTOR_2), motorsGetCurrent(BRUSHLESS_MOTOR_2));

    static float percent = 90;

    // if(palReadLine(LINE_NUCLEO_USER_BUTTON)){
    //   percent -= 5;
    //   if(percent < 0){
    //     percent = 90;
    //   }

    //   chprintf((BaseSequentialStream *)&USB_GDB, "duty cycle = %f\r\n",100-percent);
    //   chThdSleepMilliseconds(500);
    // }
    //chprintf((BaseSequentialStream *)&USB_GDB,"step = %d\n",gBrushCfg.ZCPeriod);
    //chprintf((BaseSequentialStream *) &USB_GDB, "Hi = %d, Lo = %d, Angle 1 = %.2f, Hi = %d, Lo = %d, Angle 2 = %.2f\n",data[0].alarm_hi, data[0].alarm_lo, data[0].angle, data[1].alarm_hi, data[1].alarm_lo, data[1].angle);
    //chprintf((BaseSequentialStream *)&USB_GDB, "averages : %d %d %d %d %d %d\r\n",gBrushCfg.kChannelNeutralPoint[1], gBrushCfg.kChannelNeutralPoint[2],gBrushCfg.kChannelNeutralPoint[3],gBrushCfg.kChannelNeutralPoint[4],gBrushCfg.kChannelNeutralPoint[5],gBrushCfg.kChannelNeutralPoint[6]);
   
		
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "fault status 1 	= 0x%x\n", gateDriversReadReg(DRV8323_1, DRV8323_FAULT_STATUS_1_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "fault status 2 	= 0x%x\n", gateDriversReadReg(DRV8323_1, DRV8323_FAULT_STATUS_2_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "driver control  = 0x%x\n", gateDriversReadReg(DRV8323_1, DRV8323_DRIVER_CONTROL_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "gate driver hs 	= 0x%x\n", gateDriversReadReg(DRV8323_1, DRV8323_GATE_DRIVE_HS_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "gate driver hs 	= 0x%x\n", gateDriversReadReg(DRV8323_1, DRV8323_GATE_DRIVE_LS_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "ocp control 	= 0x%x\n", gateDriversReadReg(DRV8323_1, DRV8323_OCP_CONTROL_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "csa control 	= 0x%x\n\n", gateDriversReadReg(DRV8323_1, DRV8323_CSA_CONTROL_REG));
		
	}
}
