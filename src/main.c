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
#include "gate_drivers.h"
#include "power_button.h"
#include "custom_io.h"
#include "tim1_motor.h"

/*===========================================================================*/
/* Define				                                                 */
/*===========================================================================*/
#define ADC_GRP1_NUM_CHANNELS   1
#define ADC_GRP1_BUF_DEPTH      32

#define ADC_GRP3_NUM_CHANNELS 	4
#define ADC_GRP3_BUF_DEPTH		6

#define DELAY_DEMO				1000

/*===========================================================================*/
/* Variables				                                                 */
/*===========================================================================*/
// ADC1
static adcsample_t adc_samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
static uint32_t adc_value;
// ADC 3
static adcsample_t adc_sample_3[ADC_GRP3_NUM_CHANNELS * ADC_GRP3_BUF_DEPTH];
static uint32_t adc_grp_3[ADC_GRP3_NUM_CHANNELS] = {0};



/*===========================================================================*/
/* Prototypes				                                                 */
/*===========================================================================*/
// ADC 1
static void adc_1_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n);
static void adc_1_err(ADCDriver *adcp, adcerror_t err);
// ADC 3
static void adc_3_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n);
static void adc_3_err_cb(ADCDriver *adcp, adcerror_t err);

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



/*===========================================================================*/
/* ADC Configuration with DMA.                                               */
/*===========================================================================*/


/* ADC 3 Configuration */
static const ADCConversionGroup ADC3group = {
    .circular = false,
    .num_channels = ADC_GRP3_NUM_CHANNELS,
    .end_cb = adc_3_cb,
    .error_cb = adc_3_err_cb,
    .cr1 = 0,	/*No OVR int,12 bit resolution,no AWDG/JAWDG,*/
    .cr2 = ADC_CR2_SWSTART, /* manual start of regular channels,EOC is set at end of each sequence^,no OVR detect */
    .htr = 0,
	.ltr = 0,
	.smpr1 = 0,
    .smpr2 = ADC_SMPR2_SMP_AN0(ADC_SAMPLE_3)|
			 ADC_SMPR2_SMP_AN1(ADC_SAMPLE_3)|
			 ADC_SMPR2_SMP_AN2(ADC_SAMPLE_3)|
			 ADC_SMPR2_SMP_AN3(ADC_SAMPLE_3),
    .sqr1 = ADC_SQR1_NUM_CH(ADC_GRP3_NUM_CHANNELS),
    .sqr2 = 0,
    .sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0)|
			ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1)|
			ADC_SQR3_SQ3_N(ADC_CHANNEL_IN2)|
			ADC_SQR3_SQ4_N(ADC_CHANNEL_IN3),
};


/*
 * ADC streaming callback.
 */
static void adc_3_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n)
{
  palSetLine(DEBUG_INT_LINE2);
	size_t i = 0;
	//Reset the array
	for(i = 0;i < ADC_GRP3_NUM_CHANNELS;i++)
	{
		adc_grp_3[i] = 0;
	}
	// Getting the values from the differents channels
	for(i = 0; i < n ;i++)
	{
	  adc_grp_3[0] += buffer[ADC_GRP3_NUM_CHANNELS * i];     // CH3_IN0
	  adc_grp_3[1] += buffer[ADC_GRP3_NUM_CHANNELS * i + 1]; // CH3_IN1
	  adc_grp_3[2] += buffer[ADC_GRP3_NUM_CHANNELS * i + 2]; // CH3_IN2
	  adc_grp_3[3] += buffer[ADC_GRP3_NUM_CHANNELS * i + 3]; // CH3_IN3
	}

	// Averaging
    for (i = 0; i < ADC_GRP3_NUM_CHANNELS; i++) {
    	adc_grp_3[i] /= ADC_GRP3_BUF_DEPTH;
    }

    /*
	 * reLaunch the conversion
	 */

    chSysLockFromISR();
    adcStartConversionI(&ADCD3, &ADC3group, adc_sample_3,ADC_GRP3_BUF_DEPTH);
    chSysUnlockFromISR();
    palClearLine(DEBUG_INT_LINE2);
}

static void adc_3_err_cb(ADCDriver *adcp, adcerror_t err)
{
	(void)adcp;
	(void)err;
}



// Periodic callback called when Update event happens
static void pwm_p_cb(PWMDriver *pwmp)
{
  
}

uint8_t state = 0;
uint32_t count = 0;
static void debug_cb(PWMDriver *pwmp)
{
  /* Configure the mode of each channel */
  (&PWMD1)->tim->CCMR1|=  STM32_TIM_CCMR1_OC1M(kPWMMode1);  // OC1 Mode : PWM Mode 1
  (&PWMD1)->tim->CCMR1|=  STM32_TIM_CCMR1_OC2M(kPWMMode1);  // OC2 Mode : PWM Mode 1
  (&PWMD1)->tim->CCMR2|=  STM32_TIM_CCMR2_OC3M(kPWMMode1);  // OC3 Mode : PWM Mode 1



  // STATE MACHINE
  if(0 == state)
  {
	  palSetLine(LD2_LINE);
	  palSetLine(DEBUG_INT_LINE);
	  palClearLine(DEBUG_INT_LINE2);

	  count++;
	  if(count > 1000)
	  {
		 state = 1;
		 count = 0;
	  }
  }

  if(1 == state)
  {
	  palClearLine(LD2_LINE);
	  palClearLine(DEBUG_INT_LINE);
	  palSetLine(DEBUG_INT_LINE2);
	  count++;
	  if(count > 1000)
	  {
		 state = 0;
		 count = 0;
	  }
  }

}






/*
 * Use PWM_Config only to configure the callback definitions
 * */
static PWMConfig tim_1_cfg = {
  .frequency = 10000,                        /* PWM clock frequency.   */
  .period    = 4096,                         /* PWM period in ticks  (here 0.4096 second)  */
  pwm_p_cb,									 /* Callback called when UIF is set*/
  	  	  	  	  	  	  	  	  	  	  	 /* PWM Channels configuration */
  // Complete configuration is done after the call of PWmStart
  // Channels Callback are called when the counter matches the compare value (CCxIF)
  {
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, commutation_cb}
  },
  .cr2  = 0,
  .bdtr = 0,
  .dier = 0
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


	// Debug MCU Config
	DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP; // Clock and outputs of TIM 1 are disabled when the core is halted


	/* Configure the IO mode */
	palSetLineMode(LD1_LINE,PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LD2_LINE,PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LD3_LINE,PAL_MODE_OUTPUT_PUSHPULL);
	palClearLine(LD1_LINE);
	palClearLine(LD2_LINE);
	palClearLine(LD3_LINE);

	/* Debug IO for interrupt timings */
	palSetLineMode(DEBUG_INT_LINE,PAL_MODE_OUTPUT_PUSHPULL);
	palClearLine(DEBUG_INT_LINE);
    palSetLineMode(DEBUG_INT_LINE2,PAL_MODE_OUTPUT_PUSHPULL);
    palClearLine(DEBUG_INT_LINE2);


    /* Motor 1 IO configuration when High-Impedance due to TIM 1
     * Phase 1 P : PA8  AF : 1
     * Phase 1 N : PB13 AF : 1
     * Phase 2 P : PE11 AF : 1
     * Phase 2 N : PE10 AF : 1
     * Phase 3 P : PA10 AF : 1
     * Phase 3 N : PE12 AF : 1
     *
     * */

    /* Phase 1 P */
    palSetLineMode(LINE_OUT_MOT1_PH1_P,PAL_MODE_OUTPUT_PUSHPULL); // Set to IO Output mode
    palClearLine(LINE_OUT_MOT1_PH1_P);                            // Set to 0
    // Set back to alternate function
    palSetLineMode(LINE_OUT_MOT1_PH1_P,PAL_STM32_MODE_ALTERNATE | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLDOWN | PAL_STM32_ALTERNATE(1));
    /* Phase 1 N */
    palSetLineMode(LINE_OUT_MOT1_PH1_N,PAL_MODE_OUTPUT_PUSHPULL); // Set to IO Output mode
    palClearLine(LINE_OUT_MOT1_PH1_N);                            // Set to 0
    // Set back to alternate function
    palSetLineMode(LINE_OUT_MOT1_PH1_N,PAL_STM32_MODE_ALTERNATE | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLDOWN | PAL_STM32_ALTERNATE(1));

    /* Phase 2 P */
    palSetLineMode(LINE_OUT_MOT1_PH2_P,PAL_MODE_OUTPUT_PUSHPULL); // Set to IO Output mode
    palClearLine(LINE_OUT_MOT1_PH2_P);                            // Set to 0
    // Set back to alternate function
    palSetLineMode(LINE_OUT_MOT1_PH2_P,PAL_STM32_MODE_ALTERNATE | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLDOWN | PAL_STM32_ALTERNATE(1));
    /* Phase 2 N */
    palSetLineMode(LINE_OUT_MOT1_PH2_N,PAL_MODE_OUTPUT_PUSHPULL); // Set to IO Output mode
    palClearLine(LINE_OUT_MOT1_PH2_N);                            // Set to 0
    // Set back to alternate function
    palSetLineMode(LINE_OUT_MOT1_PH2_N,PAL_STM32_MODE_ALTERNATE | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLDOWN | PAL_STM32_ALTERNATE(1));

    /* Phase 3 P */
    palSetLineMode(LINE_OUT_MOT1_PH3_P,PAL_MODE_OUTPUT_PUSHPULL); // Set to IO Output mode
    palClearLine(LINE_OUT_MOT1_PH3_P);                            // Set to 0
    // Set back to alternate function
    palSetLineMode(LINE_OUT_MOT1_PH3_P,PAL_STM32_MODE_ALTERNATE | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLDOWN | PAL_STM32_ALTERNATE(1));
    /* Phase 3 N */
    palSetLineMode(LINE_OUT_MOT1_PH3_N,PAL_MODE_OUTPUT_PUSHPULL); // Set to IO Output mode
    palClearLine(LINE_OUT_MOT1_PH3_N);                            // Set to 0
    // Set back to alternate function
    palSetLineMode(LINE_OUT_MOT1_PH3_N,PAL_STM32_MODE_ALTERNATE | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLDOWN | PAL_STM32_ALTERNATE(1));


	/*
	* Initializes two serial-over-USB CDC drivers and starts and connects the USB.
	*/
	usbSerialStart();

	shellInit();
	gateDriversEnableAll();

	/*
	 * Activates the ADC3 driver
	 */
	for(int i = 0;i< (ADC_GRP3_BUF_DEPTH * ADC_GRP3_NUM_CHANNELS);i++)
	{
		adc_sample_3[i] = 0;
	}
	adcStart(&ADCD3, NULL);


	/*
	 * Starting PWM driver 1 and enabling the notifications.
	 */


	// TIMER 1 Config
	pwmStart(&PWMD1, &tim_1_cfg); // WARNING : PWM MODE 1 BY DEFAULT AND MOE SET TO 1 !!
	uint32_t brush_6stpes_cfg = 1;

	if(1 == brush_6stpes_cfg)
	{
	  timer_1_pwm_config();
	}
	else
	{
		// Break stage configuration
		(&PWMD1)->tim->CR1 |= STM32_TIM_CR1_CKD(2);  // Modification of the CR1 CKD in order to have a bigger period for the dead times
	}

	pwmEnablePeriodicNotification(&PWMD1); // Enable the Update Event interruption

	/*
	 * Launch the conversion
	 */
	adcStartConversion(&ADCD3, &ADC3group, adc_sample_3,ADC_GRP3_BUF_DEPTH);



	// Configure the Thread that will blink the leds on the boards
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);

	while (true)
	{
		/* Send ADC values */
		/* chThdSleepMilliseconds(500);
		chprintf((BaseSequentialStream *) &USB_GDB, "IN1 : %d,IN2 : %d,IN3 : %d,IN4 : %d\n\r",adc_grp_3[0],adc_grp_3[1],adc_grp_3[2],adc_grp_3[3]);*/

/*		 Enable simple PWM
		pwmEnableChannel(&PWMD1, PWM_TIM_1_CH2 , PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 7500)); // Set CH1 and CH1N to 75% duty cycle
		pwmEnableChannelNotification(&PWMD1, PWM_TIM_1_CH2);
		chThdSleepMilliseconds(DELAY_DEMO);

		pwmEnableChannel(&PWMD1, PWM_TIM_1_CH2 , PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 5000)); // Set CH1 and CH1N to 50% duty cycle
		chThdSleepMilliseconds(DELAY_DEMO);

		pwmEnableChannel(&PWMD1, PWM_TIM_1_CH2 , PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 2500)); // Set CH1 and CH1N to 25% duty cycle
		chThdSleepMilliseconds(DELAY_DEMO);

		pwmEnableChannel(&PWMD1, PWM_TIM_1_CH2 , PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 1000)); // Set CH1 and CH1N to 10% duty cycle
		chThdSleepMilliseconds(DELAY_DEMO);

		 Disable the CH 1 and
	    pwmDisableChannel(&PWMD1, DELAY_DEMO);*/
	    
		if(isUSBConfigured()){
			//spawns the shell if the usb is connected
			spawn_shell();
		}
		chThdSleepMilliseconds(500);
		
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "fault status 1 	= 0x%x\n", gateDriversReadReg(DRV8323_1, FAULT_STATUS_1_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "fault status 2 	= 0x%x\n", gateDriversReadReg(DRV8323_1, FAULT_STATUS_2_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "driver control  = 0x%x\n", gateDriversReadReg(DRV8323_1, DRIVER_CONTROL_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "gate driver hs 	= 0x%x\n", gateDriversReadReg(DRV8323_1, GATE_DRIVE_HS_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "gate driver hs 	= 0x%x\n", gateDriversReadReg(DRV8323_1, GATE_DRIVE_LS_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "ocp control 	= 0x%x\n", gateDriversReadReg(DRV8323_1, OCP_CONTROL_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "csa control 	= 0x%x\n\n", gateDriversReadReg(DRV8323_1, CSA_CONTROL_REG));
		
	}
}
