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
#include "uc_usage.h"
#include "gdb.h"
#include "custom_io.h"

/*===========================================================================*/
/* Define				                                                 */
/*===========================================================================*/
#define ADC_GRP1_NUM_CHANNELS   1
#define ADC_GRP1_BUF_DEPTH      32

#define ADC_GRP3_NUM_CHANNELS 	4
#define ADC_GRP3_BUF_DEPTH		6

#define PWM_TIM_1_CH1			0
#define PWM_TIM_1_CH2			1
#define PWM_TIM_1_CH3 			2
#define PWM_TIM_1_CH4			3

#define DELAY_DEMO				1000
#define PERIOD_PWM_32_KHZ		6750
#define PERIOD_PWM_52_KHZ		4096
#define PERIOD_PWM_20_KHZ       10800
#define PERIOD_100_MS_INT		5273

#define NB_PHASE                3


/*===========================================================================*/
/* Typedefs				                                                     */
/*===========================================================================*/

typedef enum
{
  kCW   =0,
  kCC1  =1
}Rotation;

typedef enum
{
  kStop    = 0,
  kPhaseUV = 1,
  kPhaseUW = 2,
  kPhaseVW = 3,
  kPhaseVU = 4,
  kPhaseWU = 5,
  kPhaseWV = 6
}CommutationStateMachine;

typedef enum
{
	kTimChannel1=0,
	kTimChannel2=1,
	kTimChannel3=2,
	kTimChannel4=3,
	kTimChannel5=4,
	kTimChannel6=5
}TimChannel;


typedef enum
{
 kFrozen          = 0,
 kForcedMatchHigh = 1,
 kForcedMatchLow  = 2,
 kToggle          = 3,
 kForceRefHigh    = 4,
 kForceRefLow     = 5,
 kPWMMode1        = 6,
 kPWMMode2        = 7,
 kRetrOPM1        = 8,
 kRetrOPM2        = 9,
 kReserved1       = 10,
 kReserved2       = 11,
 kCombPWM1        = 12,
 kCombPWM2        = 13,
 kAsymPWM1        = 14,
 kAsymPWM2        = 15
}TimOCMode;

typedef enum
{
	kTimCh_Disable = 0,
	kTimCh_Enable  = 1
}TimChannelState;


/*===========================================================================*/
/* Structures                                                                */
/*===========================================================================*/
typedef struct
{
  uint32_t InStepCount;             // Count the number of iteration has done in a given step,determine the frequency of the 6 steps
  const uint32_t kMaxStepCount;     // Maximum number of iteration inside a step
  const iomode_t kDefaultIOConfig;
  ioline_t P_Channels[NB_PHASE];
  ioline_t N_Channels[NB_PHASE];

}BrushlessConfig;



/*===========================================================================*/
/* Variables				                                                 */
/*===========================================================================*/
// ADC1
static adcsample_t adc_samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
static uint32_t adc_value;
// ADC 3
static adcsample_t adc_sample_3[ADC_GRP3_NUM_CHANNELS * ADC_GRP3_BUF_DEPTH];
static uint32_t adc_grp_3[ADC_GRP3_NUM_CHANNELS] = {0};

// PWM
static CommutationStateMachine gCommutation=kStop;
static BrushlessConfig gBrushCfg = {
    .InStepCount = 0,
    .kMaxStepCount = 666,
    .kDefaultIOConfig = PAL_STM32_MODE_ALTERNATE | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLDOWN | PAL_STM32_ALTERNATE(1),
    .P_Channels = {LINE_OUT_MOT1_PH1_P,LINE_OUT_MOT1_PH2_P,LINE_OUT_MOT1_PH3_P},
    .N_Channels = {LINE_OUT_MOT1_PH1_N,LINE_OUT_MOT1_PH2_N,LINE_OUT_MOT1_PH3_N}
};

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
		palSetLine(LD1_LINE);
		chThdSleepMilliseconds(50);
		palClearLine(LD1_LINE);
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


/*===========================================================================*/
/* Timer 1 Configuration                                                     */
/*===========================================================================*/
static void tim_1_oc_cmd(TimChannel aChannel,TimChannel aState)
{
  uint32_t lCCEnable;
  switch(aChannel)
  {
    case kTimChannel1:
    {
      lCCEnable = STM32_TIM_CCER_CC1E;
	  break;
	}

    case kTimChannel2:
    {
      lCCEnable = STM32_TIM_CCER_CC2E;
	  break;
	}

    case kTimChannel3:
    {
      lCCEnable = STM32_TIM_CCER_CC3E;
      break;
	}

    case kTimChannel4:
    {
     lCCEnable = STM32_TIM_CCER_CC4E;
	  break;
	}

    case kTimChannel5:
    {
      lCCEnable = STM32_TIM_CCER_CC5E;
	  break;
	}

    case kTimChannel6:
    {
      lCCEnable = STM32_TIM_CCER_CC6E;
	  break;
	}

    default:
      break;
  }

  switch(aState)
  {
    case kTimCh_Disable:
    {
      palSetLineMode(gBrushCfg.P_Channels[aChannel],PAL_MODE_OUTPUT_PUSHPULL);
      palClearLine(gBrushCfg.P_Channels[aChannel]);
      (&PWMD1)->tim->CCER &= (~lCCEnable);
      break;
    }
    case kTimCh_Enable:
    {
      palSetLineMode(gBrushCfg.P_Channels[aChannel],gBrushCfg.kDefaultIOConfig);
      (&PWMD1)->tim->CCER |= lCCEnable;
      break;
    }
  }
}

static void tim_1_ocn_cmd(TimChannel aChannel,TimChannel aState)
{
  uint32_t lCCNEnable;
  switch(aChannel)
  {
    case kTimChannel1:
    {
      lCCNEnable = STM32_TIM_CCER_CC1NE;
      break;
    }

    case kTimChannel2:
    {
      lCCNEnable = STM32_TIM_CCER_CC2NE;
      break;
    }

    case kTimChannel3:
    {
      lCCNEnable = STM32_TIM_CCER_CC3NE;
      break;
    }

    case kTimChannel4:
    case kTimChannel5:
    case kTimChannel6:
    default:
    {

    }
  }

  switch(aState)
  {
    case kTimCh_Disable:
    {
      palSetLineMode(gBrushCfg.N_Channels[aChannel],PAL_MODE_OUTPUT_PUSHPULL);
      palClearLine(gBrushCfg.N_Channels[aChannel]);
      (&PWMD1)->tim->CCER &= (~lCCNEnable);
      break;
    }
    case kTimCh_Enable:
    {
      palSetLineMode(gBrushCfg.N_Channels[aChannel],gBrushCfg.kDefaultIOConfig);
      (&PWMD1)->tim->CCER |= lCCNEnable;
      break;
    }
  }
}

static void tim_1_oc_start(TimChannel aChannel)
{
  tim_1_oc_cmd(aChannel,kTimCh_Enable);
}

static void tim_1_oc_stop(TimChannel aChannel)
{
  tim_1_oc_cmd(aChannel,kTimCh_Disable);
}

static void tim_1_ocn_start(TimChannel aChannel)
{
  tim_1_ocn_cmd(aChannel,kTimCh_Enable);
}

static void tim_1_ocn_stop(TimChannel aChannel)
{
  tim_1_ocn_cmd(aChannel,kTimCh_Disable);
}



static void tim_1_enable_channel_out(TimChannel aChannel)
{

}


static void tim_1_disable_channel_out(TimChannel aChannel)
{

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


static void commutation_cb(PWMDriver *pwmp)
{

  palSetLine(DEBUG_INT_LINE);

  switch (gCommutation)
  {
    case kStop:
    {

      //TODO : Not optimal but will do the job.
      palSetLine(LD2_LINE);
/*       palSetLine(DEBUG_INT_LINE);
      palSetLine(DEBUG_INT_LINE2);
 */
      /* Stop all the channels */
      tim_1_oc_stop(kTimChannel1);
      tim_1_oc_stop(kTimChannel2);
      tim_1_oc_stop(kTimChannel3);
      tim_1_ocn_stop(kTimChannel1);
      tim_1_ocn_stop(kTimChannel2);
      tim_1_ocn_stop(kTimChannel3);

      /* Set all the OC output to same PWM */
      (&PWMD1)->tim->CCR[kTimChannel1]  =  PERIOD_PWM_20_KHZ/2 - 1;  // Select the Half-Period to overflow
      (&PWMD1)->tim->CCR[kTimChannel2]  =  PERIOD_PWM_20_KHZ/2 - 1;  // Select the Half-Period to overflow
      (&PWMD1)->tim->CCR[kTimChannel3]  =  PERIOD_PWM_20_KHZ/2 - 1;  // Select the Half-Period to overflow

      // Force update event (if preload enabled)
      (&PWMD1)->tim->EGR |= STM32_TIM_EGR_UG;

      /* Configure the mode of each channel */
      (&PWMD1)->tim->CCMR1|=  STM32_TIM_CCMR1_OC1M(kPWMMode1);  // OC1 Mode : PWM Mode 1
      (&PWMD1)->tim->CCMR1|=  STM32_TIM_CCMR1_OC2M(kPWMMode1);  // OC2 Mode : PWM Mode 1
      (&PWMD1)->tim->CCMR2|=  STM32_TIM_CCMR2_OC3M(kPWMMode1);  // OC3 Mode : PWM Mode 1

      // Force update event (if preload enabled)
      (&PWMD1)->tim->EGR |= STM32_TIM_EGR_COMG;

      gBrushCfg.InStepCount = 0;
      gCommutation = kPhaseUV;

      break;
    }

    /*
     * To see which step we are we will count from 0 up to 3 and down to 1
     */
    case kPhaseUV:
    {
      /* Debug IO in order to see in which step we are */
      palSetLine(LD2_LINE);

      /* Step 0 */
/*       palClearLine(DEBUG_INT_LINE);
      palClearLine(DEBUG_INT_LINE2);
 */
      /* Channel 1 High transistor connected */
      tim_1_oc_start(kTimChannel1);
      tim_1_ocn_stop(kTimChannel1);
      /* Channel 2 Low  transistor connected */
      tim_1_oc_stop(kTimChannel2);
      tim_1_ocn_start(kTimChannel2);
      /* Channel 3 not connected */
      tim_1_oc_stop(kTimChannel3);
      tim_1_ocn_stop(kTimChannel3);

      gBrushCfg.InStepCount++;
      if (gBrushCfg.kMaxStepCount == gBrushCfg.InStepCount)
      {
        gCommutation = kPhaseUW;
        gBrushCfg.InStepCount = 0;
      }

      break;
    }

    case kPhaseUW:
    {
      /* Step 1 */
/*       palSetLine(DEBUG_INT_LINE);      
      palClearLine(DEBUG_INT_LINE2);
 */
      /* Channel 1 High transistor connected */
      tim_1_oc_start(kTimChannel1);
      tim_1_ocn_stop(kTimChannel1);
      /* Channel 2 not connected */
      tim_1_oc_stop(kTimChannel2);
      tim_1_ocn_stop(kTimChannel2);
      /* Channel 3 Low transistor connected */
      tim_1_oc_stop(kTimChannel3);
      tim_1_ocn_start(kTimChannel3);

      gBrushCfg.InStepCount++;
      if (gBrushCfg.kMaxStepCount == gBrushCfg.InStepCount)
      {
        gCommutation = kPhaseVW;
        gBrushCfg.InStepCount = 0;
      }

      break;
    }

    case kPhaseVW:
    {

      /* Step 2 */
/*       palClearLine(DEBUG_INT_LINE);
      palSetLine(DEBUG_INT_LINE2);
 */
      /* Channel 1 not connected */
      tim_1_oc_stop(kTimChannel1);
      tim_1_ocn_stop(kTimChannel1);
      /* Channel 2 High connected*/
      tim_1_oc_start(kTimChannel2);
      tim_1_ocn_stop(kTimChannel2);
      /* Channel 3 Low transistor connected */
      tim_1_oc_stop(kTimChannel3);
      tim_1_ocn_start(kTimChannel3);

      gBrushCfg.InStepCount++;
      if (gBrushCfg.kMaxStepCount == gBrushCfg.InStepCount)
      {
        gCommutation = kPhaseVU;
        gBrushCfg.InStepCount = 0;
      }

      break;
    }

    case kPhaseVU:
    {

      /* Step 3 */
/*       palSetLine(DEBUG_INT_LINE);
      palSetLine(DEBUG_INT_LINE2);
 */
      /* Channel 1 Low connected */
      tim_1_oc_stop(kTimChannel1);
      tim_1_ocn_start(kTimChannel1);
      /* Channel 2 High connected*/
      tim_1_oc_start(kTimChannel2);
      tim_1_ocn_stop(kTimChannel2);
      /* Channel 3 not connected */
      tim_1_oc_stop(kTimChannel3);
      tim_1_ocn_stop(kTimChannel3);

      gBrushCfg.InStepCount++;
      if (gBrushCfg.kMaxStepCount == gBrushCfg.InStepCount)
      {
        gCommutation = kPhaseWU;
        gBrushCfg.InStepCount = 0;
      }
      break;
    }

    case kPhaseWU:
    {
      /* Step 2 */
/*       palClearLine(DEBUG_INT_LINE);
      palSetLine(DEBUG_INT_LINE2); */

      /* Channel 1 Low connected */
      tim_1_oc_stop(kTimChannel1);
      tim_1_ocn_start(kTimChannel1);
      /* Channel 2 not connected */
      tim_1_oc_stop(kTimChannel2);
      tim_1_ocn_stop(kTimChannel2);
      /* Channel 3 High connected */
      tim_1_oc_start(kTimChannel3);
      tim_1_ocn_stop(kTimChannel3);

      gBrushCfg.InStepCount++;
      if (gBrushCfg.kMaxStepCount == gBrushCfg.InStepCount)
      {
        gCommutation = kPhaseWV;
        gBrushCfg.InStepCount = 0;
      }
      break;
    }

    case kPhaseWV:
    {

      palClearLine(LD2_LINE);

      /* Channel 1 not connected */
      tim_1_oc_stop(kTimChannel1);
      tim_1_ocn_stop(kTimChannel1);
      /* Channel 2 Low connected */
      tim_1_oc_stop(kTimChannel2);
      tim_1_ocn_start(kTimChannel2);
      /* Channel 3 High connected */
      tim_1_oc_start(kTimChannel3);
      tim_1_ocn_stop(kTimChannel3);


      gBrushCfg.InStepCount++;
      if (gBrushCfg.kMaxStepCount == gBrushCfg.InStepCount)
      {
        gCommutation = kPhaseUV;
        gBrushCfg.InStepCount = 0;
      }
      break;
    }

    default:
        break;
  }

  palClearLine(DEBUG_INT_LINE);

  // Force update event (if preload enabled) for CxE,CxNE and OCxM
  (&PWMD1)->tim->EGR |= STM32_TIM_EGR_COMG;
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
		/* !!!! WIP NOT VALIDATED CONFIG : Based on TIM_6STEPS  !!!!*/

		(&PWMD1)->tim->CR1 &= (~STM32_TIM_CR1_CEN);     // Disable the counter until correct configuration
		(&PWMD1)->tim->CR1 &= (~STM32_TIM_CR1_CMS(0));  // Edge-aligned mode
		(&PWMD1)->tim->CR1 &= (~STM32_TIM_CR1_DIR);     // Direction : upcounter
		(&PWMD1)->tim->PSC =  0; 					    // Set the prescaler to 0 TIM_FREQ = 216 MHz
		(&PWMD1)->tim->ARR =  PERIOD_PWM_20_KHZ - 1;    // Set the period of our PWM

		(&PWMD1)->tim->CR1 &= (~STM32_TIM_CR1_ARPE);    // Remove the ARPE
		(&PWMD1)->tim->CR2 |= STM32_TIM_CR2_CCPC;       // Enable the Preload of the CxE,CxNE bits

		(&PWMD1)->tim->CCER = 0; // Reset Capture/Compare Register
		//

		// 1 : Output channels configuration
		// General Config
		(&PWMD1)->tim->CCMR1 = 0;                         // Reset OC 1 and OC2 configuration
        (&PWMD1)->tim->CCMR2 = 0;                         // Reset OC 3 and OC4 configuration

		// Channel 1 Config
		(&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC1P);    // OC1 Polarity  : Active High
		(&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC1NP);   // OC1N Polarity : Active High
		(&PWMD1)->tim->CR2  &=  (~STM32_TIM_CR2_OIS1);    // OC1 Idle State (when MOE=0) : 0
		(&PWMD1)->tim->CR2  &=  (~STM32_TIM_CR2_OIS1N);   // OC1N Idle State (when MOE=0): 0
		(&PWMD1)->tim->CCMR1|=  STM32_TIM_CCMR1_OC1M(0);  // OC1 Mode : Frozen
		(&PWMD1)->tim->CCMR1 &= (~STM32_TIM_CCMR1_OC1FE); // Disable the Fast Mode
		(&PWMD1)->tim->CCMR1 |= STM32_TIM_CCMR1_OC1PE;    // Enable the Preload -> CCR is loaded in the active register at each update event
		(&PWMD1)->tim->CCR[kTimChannel1] =  PERIOD_PWM_20_KHZ/2 - 1;  // Select the Half-period to overflow

		// Channel 2 Config
		(&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC2P);    // OC2 Polarity  : Active High
		(&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC2NP);   // OC2N Polarity : Active High
		(&PWMD1)->tim->CR2  &=  (~STM32_TIM_CR2_OIS2);    // OC2 Idle State (when MOE=0) : 0
		(&PWMD1)->tim->CR2  &=  (~STM32_TIM_CR2_OIS2N);   // OC2N Idle State (when MOE=0): 0
		(&PWMD1)->tim->CCMR1|=  STM32_TIM_CCMR1_OC2M(0);  // OC2 Mode : Frozen
		(&PWMD1)->tim->CCMR1 &= (~STM32_TIM_CCMR1_OC2FE); // Disable the Fast Mode
		(&PWMD1)->tim->CCMR1 |= STM32_TIM_CCMR1_OC2PE;    // Enable the Preload
		(&PWMD1)->tim->CCR[kTimChannel2] =  PERIOD_PWM_20_KHZ/4 - 1;  // Select the Quarter-period to overflow

		// Channel 3 Config
		(&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC3P);    // OC3 Polarity  : Active High
		(&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC3NP);   // OC3N Polarity : Active High
		(&PWMD1)->tim->CR2  &=  (~STM32_TIM_CR2_OIS3);    // OC3 Idle State (when MOE=0) : 0
		(&PWMD1)->tim->CR2  &=  (~STM32_TIM_CR2_OIS3N);   // OC3N Idle State (when MOE=0): 0
		(&PWMD1)->tim->CCMR2|=  STM32_TIM_CCMR2_OC3M(0);  // OC3 Mode : Frozen
		(&PWMD1)->tim->CCMR2 &= (~STM32_TIM_CCMR2_OC3FE); // Disable the Fast Mode
		(&PWMD1)->tim->CCMR2 |= STM32_TIM_CCMR2_OC3PE;    // Enable the Preload
		(&PWMD1)->tim->CCR[kTimChannel3]  =  PERIOD_PWM_20_KHZ/8 - 1;  // Select the 1/8 period to overflow

		// Channel 4 Config (for interruption each ms)
		(&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC4P);    // OC4  Polarity : Active High
		(&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC4NP);   // OC4N Polarity : Active High
		(&PWMD1)->tim->CR2  &= (~STM32_TIM_CR2_OIS4);     // OC4 Idle State (when MOE=0): 0
		(&PWMD1)->tim->CCMR2|=  STM32_TIM_CCMR2_OC4M(3);  // OC4 Mode : Toggle
		(&PWMD1)->tim->CCMR2 &= (~STM32_TIM_CCMR2_OC4FE); // Disable the Fast Mode

		// Configure the time and the interruption enable

		// BSP - Style
		// (&PWMD1)->tim->CCR4   =  PERIOD_100_MS_INT;    // Each 100 ms an interruption
		// (&PWMD1)->tim->DIER |= STM32_TIM_DIER_CC4IE;	  // Enable Capture/Compare 4 Interrupt

		// ChibiOS - Style
		pwmEnableChannel(&PWMD1, kTimChannel4 , 1);  // Interruption when CNT == CCR (same period)
		pwmEnableChannelNotification(&PWMD1, PWM_TIM_1_CH4); 		     // Enable the callback to be called for the specific channel

		// Break stage configuration

		(&PWMD1)->tim->CR1 |= STM32_TIM_CR1_CKD(0);  	// Modification of the CR1 CKD in order to have a bigger period for the dead times
														// OSSR and OSSI not needed
		(&PWMD1)->tim->BDTR = 0; 						// Reset BDTR to everything disabled (BK,BK2 not enabled)
		(&PWMD1)->tim->BDTR |= STM32_TIM_BDTR_DTG(0);   // Dead-time generator Setup
		(&PWMD1)->tim->BDTR |= STM32_TIM_BDTR_OSSR;     // Off-state selection for Run Mode
		(&PWMD1)->tim->BDTR |= STM32_TIM_BDTR_MOE;		// Main Output Enable

		// Commutation event configuration (Not needed at the moment.)




		// Start signal generation
		tim_1_oc_start(kTimChannel1);  // Channel 1 OC
		tim_1_ocn_start(kTimChannel1); // Channel 1 OC Complementary
		tim_1_oc_start(kTimChannel2);  // Channel 2 OC
		tim_1_ocn_start(kTimChannel2); // Channel 2 OC Complementary
		tim_1_oc_start(kTimChannel3);  // Channel 3 OC
		tim_1_ocn_start(kTimChannel3); // Channel 3 OC Complementary
		tim_1_oc_start(kTimChannel4);  // Channel 4 OC


		// Force update event (if preload enabled)
		(&PWMD1)->tim->EGR |= STM32_TIM_EGR_UG;

        // Force update event (if preload enabled) for CxE,CxNE and OCxM
        (&PWMD1)->tim->EGR |= STM32_TIM_EGR_COMG;

		// Enable the Timer
		(&PWMD1)->tim->CR1 |= STM32_TIM_CR1_CEN;     // Enable the counter in correct configuration

		/* !!!! WIP NOT VALIDATED CONFIG : Based on TIM_6STEPS  !!!!*/
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

	}
}
