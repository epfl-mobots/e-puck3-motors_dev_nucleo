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
#include "custom_io.h"
#include "tim1_motor.h"
#include <chprintf.h>


/*===========================================================================*/
/* Define				                                                 */
/*===========================================================================*/
#define ADC_GRP1_NUM_CHANNELS   1
#define ADC_GRP1_BUF_DEPTH      32

#define ADC_GRP3_NUM_CHANNELS 	4
#define ADC_GRP3_BUF_DEPTH		6

#define DELAY_DEMO				1000

/*  DATA TX  */
#define DTX_SIZE_1K                 1024
#define DTX_NB_POINTS               6 * DTX_SIZE_1K

static BSEMAPHORE_DECL(dtx_ready, true);


/*===========================================================================*/
/* Enumeration                                                               */
/*===========================================================================*/
typedef enum
{
  kTimer1_CH1   = 0,
  kTimer1_CH2   = 1,
  kTimer1_CH3   = 2,
  kTimer2_CH2   = 3,
  kTimer5_TRGO  = 4,
  kTimer4_CH4   = 5,
  kTimer3_CH4   = 6,
  kTimer8_TRGO  = 7,
  kTimer8_TRGO2 = 8,
  kTimer1_TRGO  = 9,
  kTimer1_TRGO2 = 10,
  kTimer2_TRGO  = 11,
  kTimer4_TRGO  = 12,
  kTimer6_TRGO  = 13,
  kReserved     = 14,
  kExti_Line11  = 15
}AdcExtTrigSrc;




/*===========================================================================*/
/* Structure                                                                 */
/*===========================================================================*/
typedef struct
{
  // DATA
  uint16_t data [ADC_GRP3_NUM_CHANNELS][DTX_NB_POINTS];
  // MGT
  const uint16_t nb_channels;
  const uint16_t nb_points;
  uint16_t data_left;
  uint16_t data_idx;
  uint8_t data_full;
}AdcDataTx;

/*===========================================================================*/
/* Variables				                                                 */
/*===========================================================================*/
static AdcDataTx gADT= {
   // VAR
   .data_full = 0,
   .data_idx  = 0,
   .data_left = DTX_NB_POINTS,
   // CST
   .nb_channels = ADC_GRP3_NUM_CHANNELS,
   .nb_points   = DTX_NB_POINTS,
};

static uint16_t test_array [12] = {
                                    2,2,2,
                                    2,2,2,
                                    2,2,2,
                                    2,2,2
                                  };

// ADC 3
static adcsample_t adc_sample_3[ADC_GRP3_NUM_CHANNELS * ADC_GRP3_BUF_DEPTH * 2];
static uint32_t adc_grp_3[ADC_GRP3_NUM_CHANNELS] = {0};
static uint8_t acq_done = 0;

/*===========================================================================*/
/* Prototypes				                                                 */
/*===========================================================================*/
void Adt_Reset_Struct(AdcDataTx* adt);
void Adt_Insert_Data(AdcDataTx* adt,uint16_t* input_data,size_t size);

// ADC 3
static void adc_3_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n);
static void adc_3_err_cb(ADCDriver *adcp, adcerror_t err);

// UART
static void Send_UINT16_Uart(uint16_t* data, uint16_t size);
static void Send_UINT32_Uart(uint32_t* data, uint16_t size);
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
    Send_ADT_Uart(&gADT);
    Adt_Reset_Struct(&gADT);
    chprintf((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)"CHC", 3);
  }
}


/*===========================================================================*/
/* UART Communication                                                        */
/*===========================================================================*/

void Send_UINT16_Uart(uint16_t* data, uint16_t size)
{

    chprintf((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)"START", 5);
    chprintf((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)&size, sizeof(uint16_t));
    chprintf((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)data, sizeof(uint16_t) * size);
}

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



void Send_UINT32_Uart(uint32_t* data, uint16_t size)
{
    chprintf((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)"START", 5);
    chprintf((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)&size, sizeof(uint16_t));
    chprintf((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)data, sizeof(uint32_t) * size);
}



/*===========================================================================*/
/* Data management                                                           */
/*===========================================================================*/
void Adt_Reset_Struct(AdcDataTx* adt)
{
  adt->data_full = 0;
  adt->data_idx  = 0;
  adt->data_left = adt->nb_points;
}

void Adt_Insert_Data(AdcDataTx* adt,uint16_t* input_data,size_t size)
{
  size_t i,j;

  //
  if(0==adt->data_full)
  {

    // Enough space for all the data
    if(adt->data_left >= size)
    {
        for(i = 0;i<size;i++)
        {
          adt->data[0][adt->data_idx] = input_data[ adt->nb_channels * i];
          adt->data[1][adt->data_idx] = input_data[ adt->nb_channels * i +1];
          adt->data[2][adt->data_idx] = input_data[ adt->nb_channels * i +2];
          adt->data[3][adt->data_idx] = input_data[ adt->nb_channels * i +3];
          adt->data_idx += 1;
        }
        adt->data_left -= size;
    }
    // Fill it until maximum capacity
    else
    {
      for(i = 0;i<adt->data_left;i++)
      {
        adt->data[0][adt->data_idx] = input_data[ adt->nb_channels * i];
        adt->data[1][adt->data_idx] = input_data[ adt->nb_channels * i +1];
        adt->data[2][adt->data_idx] = input_data[ adt->nb_channels * i +2];
        adt->data[3][adt->data_idx] = input_data[ adt->nb_channels * i +3];
        adt->data_idx += 1;
      }
      adt->data_left -= adt->data_left;
    }

  }

  // Check if full
  if(0 == adt->data_left || 1 == adt->data_full)
  {
    adt->data_full = 1;
    chSysLockFromISR();
    chBSemSignalI(&dtx_ready);
    chSysUnlockFromISR();
  }

}


/*===========================================================================*/
/* ADC Configuration with DMA.                                               */
/*===========================================================================*/


/* ADC 3 Configuration */
static const ADCConversionGroup ADC3group = {
    .circular = true,
    .num_channels = ADC_GRP3_NUM_CHANNELS,
    .end_cb = adc_3_cb,
    .error_cb = adc_3_err_cb,
    .cr1 = ADC_CR1_EOCIE,	/*End of Conversion interruption ,No OVR int,12 bit resolution,no AWDG/JAWDG,*/
    .cr2 = //ADC_CR2_SWSTART      | /* manual start of regular channels,EOC is set at end of each sequence^,no OVR detect */
           ADC_CR2_EXTEN_BOTH             |  /* We need both as OCxREF don't behave as expected */
           ADC_CR2_EXTSEL_SRC(kTimer1_TRGO2)|  /* External trigger is from Timer 1 TRGO 2*/
           0,                       /**/
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
    palSetLine(DEBUG_INT_LINE);

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
    	adc_grp_3[i] /= n;
    }

    Adt_Insert_Data(&gADT,test_array,n);


    acq_done = 1;

    palClearLine(DEBUG_INT_LINE);

}

static void adc_3_err_cb(ADCDriver *adcp, adcerror_t err)
{
	(void)adcp;
	(void)err;
}



/*===========================================================================*/
/* PWM Configuration                                                         */
/*===========================================================================*/


// Periodic callback called when Update event happens
static void pwm_p_cb(PWMDriver *pwmp)
{
  
}

/*
 * Use PWM_Config only to configure the callback definitions
 * */
static PWMConfig tim_1_cfg = {
  .frequency = 10000,                        /* PWM clock frequency.   */
  .period    = 4096,                         /* PWM period in ticks  (here 0.4096 second)  */
  commutation_cb,						     /* Callback called when UIF is set*/
  	  	  	  	  	  	  	  	  	  	  	 /* PWM Channels configuration */
  // Complete configuration is done after the call of PWmStart
  // Channels Callback are called when the counter matches the compare value (CCxIF)
  {
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, pwm_cb_ch4}
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
     * Launch the conversion (to get the buffer configured)
     */
    adcStartConversion(&ADCD3, &ADC3group, adc_sample_3,ADC_GRP3_BUF_DEPTH);


	/*
	 * Starting PWM driver 1 and enabling the notifications.
	 */
	// TIMER 1 Config
	pwmStart(&PWMD1, &tim_1_cfg); // WARNING : PWM MODE 1 BY DEFAULT AND MOE SET TO 1 !!
	timer_1_pwm_config();
	pwmEnablePeriodicNotification(&PWMD1); // Enable the Update Event interruption

	// Configure the Thread that will blink the leds on the boards
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);

    // Configure the Thread that will blink the leds on the boards
    chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO + 2, Thread2, NULL);

	while (true)
	{
		/* Send ADC values */
		/* chThdSleepMilliseconds(500);
		chprintf((BaseSequentialStream *) &USB_GDB, "IN1 : %d,IN2 : %d,IN3 : %d,IN4 : %d\n\r",adc_grp_3[0],adc_grp_3[1],adc_grp_3[2],adc_grp_3[3]);*/


	    
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
