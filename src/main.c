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
#include "encoders.h"
#include "motors.h"


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
		// palClearLine(LINE_STATUS_LED1_RED);
		// chThdSleepMilliseconds(50);
		// palClearLine(LINE_STATUS_LED2_RED);
		// chThdSleepMilliseconds(50);
		// palClearLine(LINE_STATUS_LED3_RED);
		// chThdSleepMilliseconds(50);
		// palClearLine(LINE_STATUS_LED1_GREEN);
		// chThdSleepMilliseconds(50);
		// palClearLine(LINE_STATUS_LED2_GREEN);
		// chThdSleepMilliseconds(50);
		// palClearLine(LINE_STATUS_LED3_GREEN);
		// chThdSleepMilliseconds(50);
		// palClearLine(LINE_STATUS_LED1_BLUE);
		// chThdSleepMilliseconds(50);
		// palClearLine(LINE_STATUS_LED2_BLUE);
		// chThdSleepMilliseconds(50);
		// palClearLine(LINE_STATUS_LED3_BLUE);
		// chThdSleepMilliseconds(50);
		// palSetLine(LINE_STATUS_LED1_RED);
		// chThdSleepMilliseconds(50);
		// palSetLine(LINE_STATUS_LED2_RED);
		// chThdSleepMilliseconds(50);
		// palSetLine(LINE_STATUS_LED3_RED);
		// chThdSleepMilliseconds(50);
		// palSetLine(LINE_STATUS_LED1_GREEN);
		// chThdSleepMilliseconds(50);
		// palSetLine(LINE_STATUS_LED2_GREEN);
		// chThdSleepMilliseconds(50);
		// palSetLine(LINE_STATUS_LED3_GREEN);
		// chThdSleepMilliseconds(50);
		// palSetLine(LINE_STATUS_LED1_BLUE);
		// chThdSleepMilliseconds(50);
		// palSetLine(LINE_STATUS_LED2_BLUE);
		// chThdSleepMilliseconds(50);
		// palSetLine(LINE_STATUS_LED3_BLUE);
		// chThdSleepMilliseconds(50);
		palSetPad(GPIOB,0);
		chThdSleepMilliseconds(150);
		palSetPad(GPIOB,7);
		chThdSleepMilliseconds(150);
		palSetPad(GPIOB,14);
		chThdSleepMilliseconds(150);
		palClearPad(GPIOB,0);
		chThdSleepMilliseconds(150);
		palClearPad(GPIOB,7);
		chThdSleepMilliseconds(150);
		palClearPad(GPIOB,14);
		chThdSleepMilliseconds(150);
  }
}

#include "py/compile.h"
#include "py/runtime.h"
#include "py/repl.h"
#include "py/gc.h"
#include "py/mperrno.h"
#include "lib/utils/pyexec.h"
#include "mpconfigport.h"

static char *stack_top;
#if MICROPY_ENABLE_GC
static char heap[120000];
#endif

void gc_collect(void) {
    // WARNING: This gc_collect implementation doesn't try to get root
    // pointers from CPU registers, and thus may function incorrectly.
    void *dummy;
    gc_collect_start();
    gc_collect_root(&dummy, ((mp_uint_t)stack_top - (mp_uint_t)&dummy) / sizeof(mp_uint_t));
    gc_collect_end();
    gc_dump_info();
}

// Receive single character
int mp_hal_stdin_rx_chr(void) {
//     unsigned char c = 0;
// #if MICROPY_MIN_USE_STDOUT
//     int r = read(0, &c, 1);
//     (void)r;
// #elif MICROPY_MIN_USE_STM32_MCU
//     // wait for RXNE
//     while ((USART3->SR & (1 << 5)) == 0) {
//     }
//     c = USART3->DR;
// #endif
//     return c;

    static uint8_t c[1] = {0};

	chnRead((BaseChannel*)&USB_SERIAL, c, 1);

	return (unsigned char)c[0];

}

// Send string of given length
void mp_hal_stdout_tx_strn(const char *str, mp_uint_t len) {
// #if MICROPY_MIN_USE_STDOUT
//     int r = write(1, str, len);
//     (void)r;
// #elif MICROPY_MIN_USE_STM32_MCU
//     while (len--) {
//         // wait for TXE
//         while ((USART3->SR & (1 << 7)) == 0) {
//         }
//         USART3->DR = *str++;
//     }
// #endif

	if(len > 0){
		chnWrite((BaseChannel*)&USB_SERIAL, (uint8_t*)str, len);
	}
}

static THD_WORKING_AREA(waMicropythonThd,1024);
static THD_FUNCTION(MicropythonThd,arg) {
  	(void)arg;
  	chRegSetThreadName("Micropython");


  	int stack_dummy;
	stack_top = (char*)&stack_dummy;

#if MICROPY_ENABLE_GC
	gc_init(heap, heap + sizeof(heap));
#endif
	mp_init();
#if MICROPY_ENABLE_COMPILER
	// Main script is finished, so now go into REPL mode.
	// The REPL mode can change, or it can request a soft reset.

soft_reset:
	for (;;) {
	    if (pyexec_mode_kind == PYEXEC_MODE_RAW_REPL) {
	        if (pyexec_raw_repl() != 0) {
	            break;
	        }
	    } else {
	        if (pyexec_friendly_repl() != 0) {
	            break;
	        }
	    }
	}
	printf("MPY: soft reboot\n");
	goto soft_reset;
#else
	pyexec_frozen_module("frozentest.py");
#endif
	mp_deinit();
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

	//usbPDControllerStart();

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

	/* leds Nucleo */
	palSetPadMode(GPIOB, 0, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOB, 7, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOB, 14, PAL_MODE_OUTPUT_PUSHPULL);

	/*
	* Initializes two serial-over-USB CDC drivers and starts and connects the USB.
	*/
	usbSerialStart();

	//shellInit();
	// gateDriversEnableAll();
 //  	gateDriversDisable(GATE_DRIVER_3);

 //  	motorsStart();

 //  	encodersStartReading();

	// Configure the Thread that will blink the leds on the boards
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);

	// motorSetDutyCycle(BRUSHLESS_MOTOR_1, 10);
	// motorSetDutyCycle(BRUSHLESS_MOTOR_2, 10);
	// motorSetDutyCycle(BRUSHLESS_MOTOR_3, 10);
	// motorSetDutyCycle(BRUSHLESS_MOTOR_4, 10);
  	uint8_t configured = 0;
	while (true)
	{
		/* Send ADC values */

		if(isUSBConfigured() && !configured){
			//spawns the shell if the usb is connected
			//spawn_shell();
			chThdCreateStatic(waMicropythonThd, sizeof(waMicropythonThd), NORMALPRIO, MicropythonThd, NULL);
			configured = 1;
		}
		// chThdSleepMilliseconds(100);
		chThdSleepMilliseconds(1000);
  //   chprintf((BaseSequentialStream *)&USB_GDB, "rpm = %f, current = %f \r\n",motorsGetRPM(BRUSHLESS_MOTOR_2), motorsGetCurrent(BRUSHLESS_MOTOR_2));

  //   static float percent = 90;

  //   if(palReadLine(LINE_NUCLEO_USER_BUTTON)){
  //     percent -= 5;
  //     if(percent < 0){
  //       percent = 90;
  //     }

  //     chprintf((BaseSequentialStream *)&USB_GDB, "duty cycle = %f\r\n",100-percent);
  //     chThdSleepMilliseconds(500);
  //   }
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
