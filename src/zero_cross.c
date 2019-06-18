/*
 * zero_cross.c
 *
 *  Created on: 4 juin 2019
 *      Author: Ismail-P51
 */

#include "zero_cross.h"

ZCSDetect gZCS = {
    .data_full = 0,
    .data_idx  = 0,
    .data_left = ZC_NB_POINTS,
    // CST
    .nb_channels = ZC_NUMBER_CHANNELS,
    .nb_points = ZC_NB_POINTS
};

static uint32_t average = 0;
static uint32_t count = 0;

/*===========================================================================*/
/* Zero-crossing detection                                                   */
/*===========================================================================*/

uint16_t Zcs_Get_average(void){
  return average/count;
}

void Zcs_Reset_Average(void){
  average = 0;
  count = 0;
}

void Zcs_Average(uint16_t* input_data, size_t size){
  for(uint16_t i = 0 ; i < size ; i++){
    average += input_data[gBrushCfg.kChannelMeasureArray[gBrushCfg.StateIterator]];
    count++;
  }
}

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

  static volatile uint8_t  MeasurementArrayHigh[NB_STATE] = {0,0,0,2,2,1,1};
  static volatile uint8_t  MeasurementArrayLow[NB_STATE] = {0,2,1,1,0,0,2};
  static volatile uint8_t  MeasureChannel = 0;
  int32_t lOldMeasure = 0;
  int32_t lCurMeasure = 0;
  uint8_t lChangeSign = 0; // 0 is FALSE, 1 is TRUE

  static uint16_t lhighest_voltage = 0;
  static uint16_t llowest_voltage = 0;
  static uint16_t lhalf_bus = ZC_HALF_BUS;
  static int32_t  lStateIterator = 0;

  uint16_t ret_val = 0;

  lStateIterator = brushcfg_GetStateIterator(&gBrushCfg);
  MeasureChannel = gBrushCfg.kChannelMeasureArray[lStateIterator];;

  lhighest_voltage = zcs->data[MeasurementArrayHigh[lStateIterator]][LATEST_DATA(zcs->data_idx)];
  llowest_voltage = zcs->data[MeasurementArrayLow[lStateIterator]][LATEST_DATA(zcs->data_idx)];
  lhalf_bus = LOW_PASS_COEFF_A * lhalf_bus + LOW_PASS_COEFF_B * (lhighest_voltage + llowest_voltage)/2;


  // Check if the sign has changed between old measurement and actual
  if(zcs->data_idx > TWO_ELEM_IDX && !gBrushCfg.ZCFlag)
  {
    lCurMeasure = (int32_t) zcs->data[MeasureChannel][LATEST_DATA(zcs->data_idx)]   - (int32_t)(CORR_FACTOR_HALF_BUS * lhalf_bus);
    lOldMeasure = (int32_t) zcs->data[MeasureChannel][PREVIOUS_DATA(zcs->data_idx)] - (int32_t)(CORR_FACTOR_HALF_BUS * lhalf_bus);
    // lCurMeasure = (int32_t) zcs->data[MeasureChannel][LATEST_DATA(zcs->data_idx)]   - gBrushCfg.kChannelNeutralPoint[lStateIterator];
    // lOldMeasure = (int32_t) zcs->data[MeasureChannel][PREVIOUS_DATA(zcs->data_idx)] - gBrushCfg.kChannelNeutralPoint[lStateIterator];
    
    lChangeSign = ((lOldMeasure ^ lCurMeasure) < 0); // TRUE if sign has changed
     //gBrushCfg.ZCFlag |= lChangeSign;
    ret_val = (MeasureChannel + 1)*lChangeSign;
  }


  if(ret_val > 0)
  {
    brushcfg_ComputeZCPeriod(&gBrushCfg);
    brushcfg_SetZCFlag(&gBrushCfg);
  }

  return ret_val;

}

void zcs_ext_reset(void)
{
  Zcs_Reset_Struct(&gZCS);
}
