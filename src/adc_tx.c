/*
 * adc_tx.c
 *
 *  Created on: 4 juin 2019
 *      Author: Ismail-P51
 */

#include "adc_tx.h"
#include "encoders.h"

#include "tim1_motor.h"

extern uint16_t average[NB_STATE];
extern uint32_t count[NB_STATE];

AdcDataTx gADT= {
   // VAR
   .data_full = 0,
   .data_idx  = 0,
   .data_lock = 0,
   .data_left = DTX_NB_POINTS,
   // CST
   .nb_channels = ADC_GRP3_NUM_CHANNELS,
   .nb_points   = DTX_NB_POINTS,
};

binary_semaphore_t dtx_ready;

/*===========================================================================*/
/* Data management                                                           */
/*===========================================================================*/
void Adt_Reset_Struct(AdcDataTx* adt)
{
  adt->data_full = 0;
  adt->data_idx  = 0;
  adt->data_left = adt->nb_points;
  adt->data_lock = 0;
}

void Adt_Insert_Data(AdcDataTx* adt,uint16_t* input_data,size_t size,uint8_t zc)
{
  (void) zc; // COULD BE USED FOR DEBUGGING PURPOSE
  size_t i;
  size_t nb_insertion=0;
  encoders_data_t* encoders;
  encoders = encodersGetData();

  static uint8_t MeasurementArray[7] = {0,1,2,0,1,2,0};

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
      adt->data[2][adt->data_idx] = input_data[ adt->nb_channels * i +2];

      if(zc){
        adt->data[MeasurementArray[gBrushCfg.StateIterator]][adt->data_idx] = 0;
      }

      //adt->data[3][adt->data_idx] = (uint16_t)(encoders[0].angle*10);
      adt->data[3][adt->data_idx] = (uint16_t)(brushcfg_GetStateIterator(&gBrushCfg) * 100);
      //adt->data[3][adt->data_idx] = (uint16_t)(count[6]);
      //adt->data[3][adt->data_idx] = (uint16_t)(gBrushCfg.ZCPeriod * gBrushCfg.ZCTiming);
      //adt->data[3][adt->data_idx] = (uint16_t)(gBrushCfg.ZCPeriodMean);

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
