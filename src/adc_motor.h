/*
 * adc_motor.h
 *
 *  Created on: 4 juin 2019
 *      Author: Ismail-P51
 */

#ifndef ADC_MOTOR_H_
#define ADC_MOTOR_H_

#include <ch.h>
#include <hal.h>

#include "custom_io.h"
#include "zero_cross.h"


#define ADC_GRP3_NUM_CHANNELS   ZC_NUMBER_CHANNELS
#define ADC_GRP3_BUF_DEPTH      2

#include "adc_tx.h"


// ADC 3
void adc_1_cb		(ADCDriver *adcp, adcsample_t *buffer, size_t n);
void adc_3_cb       (ADCDriver *adcp, adcsample_t *buffer, size_t n);
void adc_3_err_cb   (ADCDriver *adcp, adcerror_t err);
void adc3Start(void);

#endif /* ADC_MOTOR_H_ */
