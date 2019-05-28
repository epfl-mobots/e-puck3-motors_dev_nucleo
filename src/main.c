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
#define ADC_GRP3_BUF_DEPTH		1

#define BACK_EMF_THRES          250
#define DELAY_DEMO				1000

/*  ZERO-CROSSING DETECTION */
#define ZC_NB_POINTS            1000
#define ZC_THRESHOLD            925
#define ZC_DELTA                100
#define ZC_LOW_BOUND            ZC_THRESHOLD - ZC_DELTA
#define ZC_HIGH_BOUND           ZC_THRESHOLD + ZC_DELTA
#define ZC_SLOPE_PTS            4
/* NEW */
#define ZC_HALF_BUS             (int32_t) 925
#define CORR_FACTOR_HALF_BUS    1.04f
#define LOW_PASS_COEFF_A        0.99f
#define LOW_PASS_COEFF_B        0.01f

/*  DATA TX  */
#define DTX_SIZE_1K             1024
#define DTX_NB_POINTS           6 * DTX_SIZE_1K

#define TWO_ELEM_IDX            1
#define SIX_ELEM_IDX            7
#define LATEST_DATA(x)          (x) - 1
#define PREVIOUS_DATA(x)        (x) - 2

/*===========================================================================*/
/* Macro                                                                     */
/*===========================================================================*/
#define ABS(x)  ( (x<0) ? -x : x )

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
  uint16_t data         [ADC_GRP3_NUM_CHANNELS][ZC_NB_POINTS];
  uint8_t  slope        [ADC_GRP3_NUM_CHANNELS][ZC_NB_POINTS];
  uint8_t  zero_crossing[ADC_GRP3_NUM_CHANNELS];

  // DATA MGT
  const uint16_t nb_channels;
  const uint16_t nb_points;
  uint16_t data_left;
  uint16_t data_idx;
  uint8_t  data_full;

}ZCSDetect;


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
  uint8_t data_lock;
}AdcDataTx;

/*===========================================================================*/
/* Variables				                                                 */
/*===========================================================================*/
extern BrushlessConfig gBrushCfg;

static BSEMAPHORE_DECL(dtx_ready, true);

static AdcDataTx gADT= {
   // VAR
   .data_full = 0,
   .data_idx  = 0,
   .data_lock = 0,
   .data_left = DTX_NB_POINTS,
   // CST
   .nb_channels = ADC_GRP3_NUM_CHANNELS,
   .nb_points   = DTX_NB_POINTS,
};

static ZCSDetect gZCS = {
    .data_full = 0,
    .data_idx  = 0,
    .data_left = ZC_NB_POINTS,
    // CST
    .nb_channels = ADC_GRP3_NUM_CHANNELS,
    .nb_points = ZC_NB_POINTS
};

static uint16_t test_array [12] = {
                                    2,3,4,5,
                                    2,3,4,5,
                                    2,3,4,5
                                  };

// ADC 3
static adcsample_t adc_sample_3[ADC_GRP3_NUM_CHANNELS * ADC_GRP3_BUF_DEPTH * 2];
static uint32_t adc_grp_3[ADC_GRP3_NUM_CHANNELS] = {0};
static uint8_t acq_done = 0;
static uint16_t gValue=0;

/*===========================================================================*/
/* Prototypes				                                                 */
/*===========================================================================*/
/* Data Transmission */
void Adt_Reset_Struct(AdcDataTx* adt);
void Adt_Insert_Data(AdcDataTx* adt,uint16_t* input_data,size_t size,uint8_t zc);

/* Zero crossing detection */
void Zcs_Reset_Struct(ZCSDetect* zcs);
void Zcs_Insert_Data (ZCSDetect* zcs,uint16_t* input_data,size_t size);
uint8_t Zcs_Detect(ZCSDetect* zcs);
void zcs_ext_reset(void);


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
    gADT.data_lock = 1;
    Send_ADT_Uart(&gADT);
    Adt_Reset_Struct(&gADT);
  }
}


/*===========================================================================*/
/* UART Communication                                                        */
/*===========================================================================*/

void Send_UINT16_Uart(uint16_t* data, uint16_t size)
{
  streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)"START", 5);
  streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)&size, sizeof(uint16_t));
  streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)data, sizeof(uint16_t) * size);
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

  // B: TO REMOVE
  // streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)"EMF", 3);
  // streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)&adt->nb_points, sizeof(uint16_t));
  // streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)&adt->data[4], sizeof(uint16_t) * adt->nb_points);
  // E: TO REMOVE

}

void Send_UINT32_Uart(uint32_t* data, uint16_t size)
{
  streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)"START", 5);
  streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)&size, sizeof(uint16_t));
  streamWrite((BaseSequentialStream *) &USB_SERIAL, (uint8_t*)data, sizeof(uint32_t) * size);
}



/*===========================================================================*/
/* Data management                                                           */
/*===========================================================================*/
void Adt_Reset_Struct(AdcDataTx* adt)
{
  size_t i,j;
  adt->data_full = 0;
  adt->data_idx  = 0;
  adt->data_left = adt->nb_points;
  adt->data_lock = 0;
}

void Adt_Insert_Data(AdcDataTx* adt,uint16_t* input_data,size_t size,uint8_t zc)
{
  size_t i;
  size_t nb_insertion=0;

  //
  if(0==adt->data_full)
  {

    // Enough space for all the data
    if(adt->data_left >= size)
    {
      nb_insertion = size;
    }
    else
    {
      // Fill it until maximum capacity
      nb_insertion = adt->data_left;
    }

    for(i = 0;i<nb_insertion;i++)
    {
      adt->data[0][adt->data_idx] = input_data[ adt->nb_channels * i];
      adt->data[1][adt->data_idx] = input_data[ adt->nb_channels * i +1];
/*    adt->data[2][adt->data_idx] = input_data[ adt->nb_channels * i +2];
      adt->data[3][adt->data_idx] = input_data[ adt->nb_channels * i +3]; */
      adt->data[2][adt->data_idx] = zc * 1000;
      adt->data[3][adt->data_idx] = gBrushCfg.StateIterator * 250;

      adt->data_idx += 1;
    }
    adt->data_left -= nb_insertion;

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
/* Zero-crossing detection                                                   */
/*===========================================================================*/
void Zcs_Reset_Struct(ZCSDetect* zcs)
{
  zcs->data_idx  = 0;
  zcs->data_full = 0;
  zcs->data_left = zcs->nb_points;
}

void Zcs_Insert_Data (ZCSDetect* zcs,uint16_t* input_data,size_t size)
{
  size_t i;

  // Check if full
  if(0 == zcs->data_left || 1 == zcs->data_full)
  {
    zcs->data_full = 1;
    Zcs_Reset_Struct(zcs);
  }

  //
  if(0==zcs->data_full)
  {

    // Enough space for all the data
    if(zcs->data_left >= size)
    {
        for(i = 0;i<size;i++)
        {
          zcs->data[0][zcs->data_idx] = input_data[ zcs->nb_channels * i];
          zcs->data[1][zcs->data_idx] = input_data[ zcs->nb_channels * i +1];
          zcs->data[2][zcs->data_idx] = input_data[ zcs->nb_channels * i +2];
          zcs->data[3][zcs->data_idx] = input_data[ zcs->nb_channels * i +3];
          zcs->data_idx += 1;
        }
        zcs->data_left -= size;
    }
    // Fill it until maximum capacity
    else
    {
      for(i = 0;i<zcs->data_left;i++)
      {
        zcs->data[0][zcs->data_idx] = input_data[ zcs->nb_channels * i];
        zcs->data[1][zcs->data_idx] = input_data[ zcs->nb_channels * i +1];
        zcs->data[2][zcs->data_idx] = input_data[ zcs->nb_channels * i +2];
        zcs->data[3][zcs->data_idx] = input_data[ zcs->nb_channels * i +3];
        zcs->data_idx += 1;
      }
      zcs->data_left -= zcs->data_left;
    }

  }


}


uint8_t Zcs_Detect(ZCSDetect* zcs)
{

  static volatile uint8_t  MeasurementArray[NB_STATE] = {0,1,2,0,1,2,0};
  static volatile uint8_t  MeasurementArrayHigh[NB_STATE] = {0,0,0,2,2,1,1};
  static volatile uint8_t  MeasurementArrayLow[NB_STATE] = {0,2,1,1,0,0,2};
  static volatile uint8_t  MeasureChannel = 0;
  int32_t lOldMeasure = 0;
  int32_t lCurMeasure = 0;
  uint8_t lChangeSign = 0; // 0 is FALSE, 1 is TRUE
  static  int32_t lOldStateIterator = 36;

  static uint16_t lhighest_voltage = 0;
  static uint16_t llowest_voltage = 0;
  static uint16_t lhalf_bus = ZC_HALF_BUS;

  uint16_t ret_val = 0;

  MeasureChannel = MeasurementArray[gBrushCfg.StateIterator];

  lhighest_voltage = zcs->data[MeasurementArrayHigh[gBrushCfg.StateIterator]][LATEST_DATA(zcs->data_idx)];
  llowest_voltage = zcs->data[MeasurementArrayLow[gBrushCfg.StateIterator]][LATEST_DATA(zcs->data_idx)];
  lhalf_bus = LOW_PASS_COEFF_A * lhalf_bus + LOW_PASS_COEFF_B * (lhighest_voltage + llowest_voltage)/2;

      // Check if the sign has changed between old measurement and actual
      if(zcs->data_idx >  TWO_ELEM_IDX)
      {
        lCurMeasure = (int32_t) zcs->data[MeasureChannel][LATEST_DATA(zcs->data_idx)]   - (int32_t)(CORR_FACTOR_HALF_BUS * lhalf_bus);
        lOldMeasure = (int32_t) zcs->data[MeasureChannel][PREVIOUS_DATA(zcs->data_idx)] - (int32_t)(CORR_FACTOR_HALF_BUS * lhalf_bus);
        lChangeSign = ((lOldMeasure ^ lCurMeasure) < 0); // TRUE if sign has changed
        gBrushCfg.ZCFlag |= lChangeSign;
        ret_val = (MeasureChannel + 1)*lChangeSign;
      }


      if(ret_val > 0)
      {

        gBrushCfg.ZCDetectOld = gBrushCfg.ZCDetect;
        gBrushCfg.ZCDetect    = gBrushCfg.TimeBLDCCommut;
        gBrushCfg.ZCPeriodOld = gBrushCfg.ZCPeriod;
        gBrushCfg.ZCPeriod    = gBrushCfg.ZCDetect - gBrushCfg.ZCDetectOld;
        gBrushCfg.ZCPeriodMean = ((gBrushCfg.ZCPeriodOld + gBrushCfg.ZCPeriod) >> 1);
        gBrushCfg.ZCNextCommut = gBrushCfg.TimeBLDCCommut + (gBrushCfg.ZCPeriodMean >> 1);

      }

    return ret_val;

}

void zcs_ext_reset(void)
{
  Zcs_Reset_Struct(&gZCS);
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
    uint8_t zc_detect;

    palSetLine(DEBUG_INT_LINE);

    // Zero-crossing and slope detection
	Zcs_Insert_Data(&gZCS,buffer,n);
	zc_detect = Zcs_Detect(&gZCS);

    // Data transmission
    if(0 == gADT.data_lock)
    {
      Adt_Insert_Data(&gADT,buffer,n,zc_detect);
    }

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

	Adt_Reset_Struct(&gADT);
	Zcs_Reset_Struct(&gZCS);

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

		if(isUSBConfigured()){
			//spawns the shell if the usb is connected
			spawn_shell();
		}
		chThdSleepMilliseconds(500);

    static float percent = 90;

    if(palReadLine(LINE_NUCLEO_USER_BUTTON)){
      percent -= 2;
      if(percent < 50){
        percent = 90;
      }
      (&PWMD1)->tim->CCR[kTimChannel1]  =  (percent/100) * PERIOD_PWM_20_KHZ - 1;  // Select the quarter-Period to overflow
      (&PWMD1)->tim->CCR[kTimChannel2]  =  (percent/100) * PERIOD_PWM_20_KHZ - 1;  // Select the quarter-Period to overflow
      (&PWMD1)->tim->CCR[kTimChannel3]  =  (percent/100) * PERIOD_PWM_20_KHZ - 1;  // Select the quarter-Period to overflow
      chThdSleepMilliseconds(500);
    }

   
		
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "fault status 1 	= 0x%x\n", gateDriversReadReg(DRV8323_1, FAULT_STATUS_1_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "fault status 2 	= 0x%x\n", gateDriversReadReg(DRV8323_1, FAULT_STATUS_2_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "driver control  = 0x%x\n", gateDriversReadReg(DRV8323_1, DRIVER_CONTROL_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "gate driver hs 	= 0x%x\n", gateDriversReadReg(DRV8323_1, GATE_DRIVE_HS_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "gate driver hs 	= 0x%x\n", gateDriversReadReg(DRV8323_1, GATE_DRIVE_LS_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "ocp control 	= 0x%x\n", gateDriversReadReg(DRV8323_1, OCP_CONTROL_REG));
		// chprintf((BaseSequentialStream *) &USB_SERIAL, "csa control 	= 0x%x\n\n", gateDriversReadReg(DRV8323_1, CSA_CONTROL_REG));
		
	}
}
