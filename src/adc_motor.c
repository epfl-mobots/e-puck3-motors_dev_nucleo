/*
 * adc_motor.c
 *
 *  Created on: 4 juin 2019
 *      Author: Ismail-P51
 */



#include "adc_motor.h"

extern BrushlessConfig gBrushCfg;

// ADC 3
static adcsample_t adc_sample_3[ADC_GRP3_NUM_CHANNELS * ADC_GRP3_BUF_DEPTH * 2];
static adcsample_t adc_sample_1[3 * 2];

static adcsample_t adc_sample_1_copy[3];


/* ADC 3 Configuration */
static const ADCConversionGroup ADC3group = {
    .circular = true,
    .num_channels = ADC_GRP3_NUM_CHANNELS,
    .end_cb = adc_3_cb,
    .error_cb = adc_3_err_cb,
    .cr1 = ADC_CR1_EOCIE,   /*End of Conversion interruption ,No OVR int,12 bit resolution,no AWDG/JAWDG,*/
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

/* ADC 1 Configuration */
static const ADCConversionGroup ADC1group = {
    .circular = true,
    .num_channels = 3,
    .end_cb = adc_1_cb,
    .error_cb = NULL,
    .cr1 = ADC_CR1_EOCIE,   /*End of Conversion interruption ,No OVR int,12 bit resolution,no AWDG/JAWDG,*/
    .cr2 = //ADC_CR2_SWSTART      | /* manual start of regular channels,EOC is set at end of each sequence^,no OVR detect */
           ADC_CR2_EXTEN_BOTH             |  /* We need both as OCxREF don't behave as expected */
           ADC_CR2_EXTSEL_SRC(kTimer1_TRGO2)|  /* External trigger is from Timer 1 TRGO 2*/
           0,                       /**/
    .htr = 0,
    .ltr = 0,
    .smpr1 = 0,
    .smpr2 = ADC_SMPR2_SMP_AN0(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN1(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN2(ADC_SAMPLE_3),
    .sqr1 = ADC_SQR1_NUM_CH(3),
    .sqr2 = 0,
    .sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN4)|
            ADC_SQR3_SQ2_N(ADC_CHANNEL_IN5)|
            ADC_SQR3_SQ3_N(ADC_CHANNEL_IN6),
};


/*===========================================================================*/
/* ADC Configuration with DMA.                                               */
/*===========================================================================*/

/*
 * ADC streaming callback.
 */
void adc_1_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n)
{
    (void) adcp;


    adc_sample_1_copy[0] = buffer[0];
    adc_sample_1_copy[1] = buffer[1];
    adc_sample_1_copy[2] = buffer[2];

}

/*
 * ADC streaming callback.
 */
void adc_3_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n)
{
    (void) adcp;

    uint8_t zc_detect = 0;

    palSetLine(DEBUG_INT_LINE);

    if(gBrushCfg.Mode == kCalibrate){

        Zcs_Average(buffer,n);
    }else{
        // Zero-crossing and slope detection
        Zcs_Insert_Data(&gZCS,buffer,n);
        zc_detect = Zcs_Detect(&gZCS);
    }

    // Data transmission
    if(0 == gADT.data_lock)
    {
      //replaces the data by the ones of ADC1 
      buffer[1] = adc_sample_1_copy[0];
      buffer[2] = adc_sample_1_copy[1];
      buffer[3] = adc_sample_1_copy[2];
      Adt_Insert_Data(&gADT,buffer,n,zc_detect);
    }

    palClearLine(DEBUG_INT_LINE);

}

void adc_3_err_cb(ADCDriver *adcp, adcerror_t err)
{
    (void)adcp;
    (void)err;
}



void adc3Start()
{
  /*
     * Activates the ADC3 driver
     */
    for(int i = 0;i< (ADC_GRP3_BUF_DEPTH * ADC_GRP3_NUM_CHANNELS);i++)
    {
        adc_sample_3[i] = 0;
    }
    adcStart(&ADCD3, NULL);
    adcStart(&ADCD1, NULL);

    /*
     * Launch the conversion (to get the buffer configured)
     */
    adcStartConversion(&ADCD3, &ADC3group, adc_sample_3,ADC_GRP3_BUF_DEPTH);
    adcStartConversion(&ADCD1, &ADC1group, adc_sample_1,1);
}
