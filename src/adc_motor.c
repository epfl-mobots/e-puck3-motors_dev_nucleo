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
static adcsample_t adc_sample_1[12 * 2];


/* ADC 3 Configuration */
static const ADCConversionGroup ADC3group = {
    .circular = true,
    .num_channels = ADC_GRP3_NUM_CHANNELS,
    .end_cb = adc_3_cb,
    .error_cb = adc_3_err_cb,
    .cr1 = 0,//ADC_CR1_DISCEN | ((ADC_GRP3_NUM_CHANNELS-1) << ADC_CR1_DISCNUM_Pos),   /*No OVR int,12 bit resolution,no AWDG/JAWDG, discontinuous 4 samples at each trigger*/
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
    .sqr1 =  ADC_SQR1_NUM_CH(ADC_GRP3_NUM_CHANNELS),
    .sqr3 =  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0)|
             ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1)|
             ADC_SQR3_SQ3_N(ADC_CHANNEL_IN2)|
             ADC_SQR3_SQ4_N(ADC_CHANNEL_IN3),
    .sqr2 = 0,
};

/* ADC 1 Configuration */
static const ADCConversionGroup ADC1group = {
    .circular = true,
    .num_channels = 12,
    .end_cb = adc_1_cb,
    .error_cb = NULL,
    .cr1 = 0,   /*No OVR int,12 bit resolution,no AWDG/JAWDG*/
    .cr2 = //ADC_CR2_SWSTART      | /* manual start of regular channels,EOC is set at end of each sequence^,no OVR detect */
           //ADC_CR2_EXTEN_BOTH             |  /* We need both as OCxREF don't behave as expected */
           //ADC_CR2_EXTSEL_SRC(kTimer1_TRGO)|  /* External trigger is from Timer 1 TRGO 2*/
           0,                       /**/
    .htr = 0,
    .ltr = 0,
    .smpr2 = ADC_SMPR2_SMP_AN0(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN1(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN2(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN3(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN4(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN5(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN6(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN7(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN8(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN9(ADC_SAMPLE_3),
    .smpr1 = ADC_SMPR1_SMP_AN10(ADC_SAMPLE_3)|
             ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3)|
             ADC_SMPR1_SMP_AN12(ADC_SAMPLE_3)|
             ADC_SMPR1_SMP_AN13(ADC_SAMPLE_3)|
             ADC_SMPR1_SMP_AN14(ADC_SAMPLE_3)|
             ADC_SMPR1_SMP_AN15(ADC_SAMPLE_3),
    .sqr3 =  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN4)|
             ADC_SQR3_SQ2_N(ADC_CHANNEL_IN5)|
             ADC_SQR3_SQ3_N(ADC_CHANNEL_IN6)|
             ADC_SQR3_SQ4_N(ADC_CHANNEL_IN9)| //dummy to simulate 4 channels sampling
             ADC_SQR3_SQ5_N(ADC_CHANNEL_IN4)|
             ADC_SQR3_SQ6_N(ADC_CHANNEL_IN5),
    .sqr2 =  ADC_SQR2_SQ7_N(ADC_CHANNEL_IN6)|
             ADC_SQR2_SQ8_N(ADC_CHANNEL_IN9)| //dummy to simulate 4 channels sampling
             ADC_SQR2_SQ9_N(ADC_CHANNEL_IN4)|
             ADC_SQR2_SQ10_N(ADC_CHANNEL_IN5)|
             ADC_SQR2_SQ11_N(ADC_CHANNEL_IN6)|
             ADC_SQR2_SQ12_N(ADC_CHANNEL_IN9), //dummy to simulate 4 channels sampling
    .sqr1 =  ADC_SQR1_SQ13_N(ADC_CHANNEL_IN4)|
             ADC_SQR1_SQ14_N(ADC_CHANNEL_IN5)|
             ADC_SQR1_SQ15_N(ADC_CHANNEL_IN6)|
             ADC_SQR1_SQ16_N(ADC_CHANNEL_IN9)| //dummy to simulate 4 channels sampling
             ADC_SQR1_NUM_CH(12),
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
    palSetLine(DEBUG_INT_LINE);
    static float average = 0;
    static float average2 = 0;
    ADCD1.adc->SQR3 =   ADC_SQR3_SQ1_N(motor1.kchannelCurrentSense[brushcfg_GetStateIterator(&motor1)]) |
                        ADC_SQR3_SQ2_N(ADC_CHANNEL_IN9)|
                        ADC_SQR3_SQ3_N(ADC_CHANNEL_IN9)|
                        ADC_SQR3_SQ4_N(motor4.kchannelCurrentSense[brushcfg_GetStateIterator(&motor4)])| //dummy to simulate 4 channels sampling
                        ADC_SQR3_SQ5_N(motor1.kchannelCurrentSense[brushcfg_GetStateIterator(&motor1)])|
                        ADC_SQR3_SQ6_N(ADC_CHANNEL_IN9);
    ADCD1.adc->SQR2 =   ADC_SQR2_SQ7_N(ADC_CHANNEL_IN9)|
                        ADC_SQR2_SQ8_N(motor4.kchannelCurrentSense[brushcfg_GetStateIterator(&motor4)])| //dummy to simulate 4 channels sampling
                        ADC_SQR2_SQ9_N(motor1.kchannelCurrentSense[brushcfg_GetStateIterator(&motor1)])|
                        ADC_SQR2_SQ10_N(ADC_CHANNEL_IN9)|
                        ADC_SQR2_SQ11_N(ADC_CHANNEL_IN9)|
                        ADC_SQR2_SQ12_N(motor4.kchannelCurrentSense[brushcfg_GetStateIterator(&motor4)]);
    
    ADCD1.adc->CR2 |= ADC_CR2_SWSTART;
    average = 0.99 * (float)average + 0.01 * (float)buffer[0];
    average = 0.99 * (float)average + 0.01 * (float)buffer[4];
    average = 0.99 * (float)average + 0.01 * (float)buffer[8];
    average2 = 0.99 * average2 + 0.01 * average;
    buffer[1] = buffer[5] = buffer[9] = (uint16_t)average2;
    // Data transmission
    if(0 == gADT.data_lock)
    { 
      //Adt_Insert_Data(&gADT,buffer,3,0);
    }

    //Zcs_Detect(&gZCS);
    palClearLine(DEBUG_INT_LINE);
}

/*
 * ADC streaming callback.
 */
void adc_3_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n)
{
    (void) adcp;
    uint8_t zc_detect = 0;

    palSetLine(DEBUG_INT_LINE2);

    ADCD3.adc->SQR3 =   ADC_SQR3_SQ1_N(motor1.kChannelMeasureArray[brushcfg_GetStateIterator(&motor1)])|
                        ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1)|
                        ADC_SQR3_SQ3_N(ADC_CHANNEL_IN2)|
                        ADC_SQR3_SQ4_N(motor4.kChannelMeasureArray[brushcfg_GetStateIterator(&motor4)]);

    static uint8_t i = 1;
    if(i){
      //we sampled OFF PWM
      PWMD1.tim->CCR[3] = 0.75 * PERIOD_PWM_52_KHZ - 1;
      Zcs_Insert_Data(&gZCS,buffer,n, 0);

      // Data transmission
      if(0 == gADT.data_lock)
      {
        //replaces the data by the ones of ADC1 
        // if((motor1.StateIterator == 4) || (motor1.StateIterator == 5)){
        //   current = (uint16_t)(0.99 * (float)current + 0.01 * (float)adc_sample_1_copy[0]);
        //   current2 = (uint16_t)(0.99 * (float)current2 + 0.01 * (float)current);
        // }
        // buffer[1] = current;
        // buffer[2] = adc_sample_1_copy[0];
        // buffer[3] = current2;
        //buffer[3] = adc_sample_1_copy[2];
        // buffer[0] = adc_sample_1_copy[0];
        // buffer[1] = adc_sample_1_copy[1];
        // buffer[2] = adc_sample_1_copy[2];
        Adt_Insert_Data(&gADT,buffer,n,zc_detect);
      }
    }else{
      //we sampled ON PWM
      PWMD1.tim->CCR[3] = 0.20 * PERIOD_PWM_52_KHZ - 1;
      // if(motor1.Mode == kCalibrate){

      //     Zcs_Average(buffer,n);
      // }else{
          // Zero-crossing and slope detection
          Zcs_Insert_Data(&gZCS,buffer,n, 1);
          zc_detect = Zcs_Detect(&gZCS, &motor4);
          zc_detect = Zcs_Detect(&gZCS, &motor1);
      // }
    }
    i = !i;
    palClearLine(DEBUG_INT_LINE2);
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
    adcStartConversion(&ADCD3, &ADC3group, adc_sample_3,2);
    adcStartConversion(&ADCD1, &ADC1group, adc_sample_1,2);
    ADCD1.adc->CR2 |= ADC_CR2_SWSTART;
}
