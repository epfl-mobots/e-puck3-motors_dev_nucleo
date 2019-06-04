/*
 * adt_tx.h
 *
 *  Created on: 4 juin 2019
 *      Author: Ismail-P51
 */

#ifndef ADC_TX_H_
#define ADC_TX_H_

#include <ch.h>
#include <hal.h>

#include "adc_motor.h"

/*  DATA TX  */
#define DTX_SIZE_1K             1024
#define DTX_NB_POINTS           6 * DTX_SIZE_1K

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


/* Data Transmission */
void Adt_Reset_Struct(AdcDataTx* adt);
void Adt_Insert_Data(AdcDataTx* adt,uint16_t* input_data,size_t size,uint8_t zc);

/*===========================================================================*/
/* Export global variable                                                    */
/*===========================================================================*/
extern AdcDataTx gADT;
extern binary_semaphore_t dtx_ready;


#endif /* ADC_TX_H_ */
