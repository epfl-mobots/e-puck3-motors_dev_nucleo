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
#include "drv8323.h"

extern DRV8323Config drv8323_1;
extern DRV8323Config drv8323_2;
extern DRV8323Config drv8323_3;
extern DRV8323Config drv8323_4;

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



/*===========================================================================*/
/* ADC Configuration with DMA.                                               */
/*===========================================================================*/


#define ADC_GRP1_NUM_CHANNELS   1
#define ADC_GRP1_BUF_DEPTH      32

static adcsample_t adc_samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
static uint32_t adc_value;

/* Prototypes */
static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n);
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err);

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

/* ADC 1 Configuration */
static const ADCConversionGroup ADC1group = {
    .circular = false,
    .num_channels = 1,
    .end_cb = adccallback,
    .error_cb = adcerrorcallback,
    .cr1 = 0,	/*No OVR int,12 bit resolution,no AWDG/JAWDG,*/
    .cr2 = ADC_CR2_SWSTART, /* manual start of regular channels,EOC is set at end of each sequence^,no OVR detect */
    .htr = 0,
	.ltr = 0,
	.smpr1 = 0,
    .smpr2 = ADC_SMPR2_SMP_AN9(ADC_SAMPLE_3),
    .sqr1 = ADC_SQR1_NUM_CH(1),
    .sqr2 = 0,
    .sqr3 = ADC_SQR3_SQ1_N(9),
};


/*
 * ADC streaming callback.
 */
static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

	(void)adcp;

	adc_value = 0;
    for (size_t i = 0; i < n; i++) {
        adc_value += buffer[i];
    }
    adc_value /= n;

    /*
	 * reLaunch the conversion
	 */
    chSysLockFromISR();
    adcStartConversionI(&ADCD1, &ADC1group, adc_samples, ADC_GRP1_BUF_DEPTH);
    chSysUnlockFromISR();
}

/*
 * ADC errors callback, should never happen.
 */
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}






/*===========================================================================*/
/* Main.                                                       */
/*===========================================================================*/

int main(void) {

	//keeps the power on
	palSetLine(LINE_PWR_ON);

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

	shellInit();
	gateDriversInit();

	/*
	 * Activates the ADC1 driver
	 */
	for(int i = 0;i<ADC_GRP1_BUF_DEPTH;i++)
	{
		adc_samples[i] = 0;
	}
	adcStart(&ADCD1, NULL);


	/*
	 * Launch the conversion
	 */
	adcStartConversion(&ADCD1, &ADC1group, adc_samples, ADC_GRP1_BUF_DEPTH);

	/* Configure the IO mode */
	palSetLineMode(LD1_LINE,PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LD2_LINE,PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LD3_LINE,PAL_MODE_OUTPUT_PUSHPULL);
	palClearLine(LD1_LINE);
	palClearLine(LD2_LINE);
	palClearLine(LD3_LINE);

	for(uint8_t i = 0 ; i < NB_MOTORS ; i++){
		for(uint8_t j = 0 ; j < NB_PHASES ; j++){
			for(uint8_t k = 0 ; k < NB_TRANSISTORS ; k++){
				palSetLineMode(motor_pins[i][j][k], PAL_MODE_OUTPUT_PUSHPULL);
				palClearLine(motor_pins[i][j][k]);
				//chprintf((BaseSequentialStream *) &USB_GDB, "GPIO: 0x%x pin: %d\n",PAL_PORT(motor_pins[i][j][k]), PAL_PAD(motor_pins[i][j][k]));
			}
		}
	}

	/*
	palSetLineMode(LINE_OUT_MOT3_PH1_P, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(PAL_LINE(GPIOB, 7U), PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_OUT_MOT4_PH2_N, PAL_MODE_OUTPUT_PUSHPULL);
	 */

	// Configure the Thread that will blink the leds on the boards
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);


	while (true) {
		if(isUSBConfigured()){
			//spawns the shell if the usb is connected
			spawn_shell();
		}
		chThdSleepMilliseconds(500);

		chprintf((BaseSequentialStream *) &USB_SERIAL, "fault status 1 	= 0x%x\n", drv8323ReadReg(&drv8323_1, FAULT_STATUS_1_REG));
		chprintf((BaseSequentialStream *) &USB_SERIAL, "fault status 2 	= 0x%x\n", drv8323ReadReg(&drv8323_1, FAULT_STATUS_2_REG));
		chprintf((BaseSequentialStream *) &USB_SERIAL, "driver control  = 0x%x\n", drv8323ReadReg(&drv8323_1, DRIVER_CONTROL_REG));
		chprintf((BaseSequentialStream *) &USB_SERIAL, "gate driver hs 	= 0x%x\n", drv8323ReadReg(&drv8323_1, GATE_DRIVE_HS_REG));
		chprintf((BaseSequentialStream *) &USB_SERIAL, "gate driver hs 	= 0x%x\n", drv8323ReadReg(&drv8323_1, GATE_DRIVE_LS_REG));
		chprintf((BaseSequentialStream *) &USB_SERIAL, "ocp control 	= 0x%x\n", drv8323ReadReg(&drv8323_1, OCP_CONTROL_REG));
		chprintf((BaseSequentialStream *) &USB_SERIAL, "csa control 	= 0x%x\n\n", drv8323ReadReg(&drv8323_1, CSA_CONTROL_REG));
		
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "buffer SPI 1 = 0x%x\n", drv8323ReadReg(&drv8323_1, CSA_CONTROL_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "buffer SPI 2 = 0x%x\n", drv8323ReadReg(&drv8323_2, CSA_CONTROL_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "buffer SPI 3 = 0x%x\n", drv8323ReadReg(&drv8323_3, CSA_CONTROL_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "buffer SPI 4 = 0x%x\n", drv8323ReadReg(&drv8323_4, CSA_CONTROL_REG));
		//palToggleLine(LINE_OUT_MOT3_PH1_P);
		//palToggleLine(PAL_LINE(GPIOB, 7U));
		//palToggleLine(LINE_OUT_MOT4_PH2_N);
		// chprintf((BaseSequentialStream *) &USB_GDB, "GDB\n");
		// printUcUsage((BaseSequentialStream *) &USB_GDB);
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "SERIAL\n");
		// printUcUsage((BaseSequentialStream *) &USB_SERIAL);
	}
}
