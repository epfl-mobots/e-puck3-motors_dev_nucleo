/*
 * zero_cross.c
 *
 *  Created on: 4 juin 2019
 *      Author: Ismail-P51
 */

#include "zero_cross.h"

ZCSDetect gZCS = {
    .data_left = {ZC_NB_POINTS, ZC_NB_POINTS, ZC_NB_POINTS, ZC_NB_POINTS},
    .data_idx  = {0}, 
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
    average += input_data[motor1.kChannelMeasureArray[motor1.StateIterator]];
    count++;
  }
}

void Zcs_Reset_Struct(ZCSDetect* zcs, uint8_t motorNb)
{
  zcs->data_idx[motorNb]  = 0;
  zcs->data_left[motorNb] = ZC_NB_POINTS;
}

void Zcs_Insert_Data (ZCSDetect* zcs,uint16_t* input_data,size_t size, uint8_t onOff)
{

  // Check if full
  if(0 == zcs->data_left[0])
  {
    Zcs_Reset_Struct(zcs, 0);
  }
  if(0 == zcs->data_left[1])
  {
    Zcs_Reset_Struct(zcs, 1);
  }
  if(0 == zcs->data_left[2])
  {
    Zcs_Reset_Struct(zcs, 2);
  }
  if(0 == zcs->data_left[3])
  {
    Zcs_Reset_Struct(zcs, 3);
  }

  zcs->data[0][onOff][zcs->data_idx[0]] = input_data[0];
  zcs->data[1][onOff][zcs->data_idx[1]] = input_data[1];
  zcs->data[2][onOff][zcs->data_idx[2]] = input_data[2];
  zcs->data[3][onOff][zcs->data_idx[3]] = input_data[3];

  if(onOff){
    zcs->data_idx[0] += 1;
    zcs->data_left[0] -= 1;

    zcs->data_idx[1] += 1;
    zcs->data_left[1] -= 1;

    zcs->data_idx[2] += 1;
    zcs->data_left[2] -= 1;

    zcs->data_idx[3] += 1;
    zcs->data_left[3] -= 1;
  }

}

uint8_t Zcs_Detect(ZCSDetect* zcs, BrushlessConfig* motor)
{

  static uint8_t MeasureChannel = 0;
  static int32_t lOldMeasure = 0;
  static int32_t lCurMeasure = 0;
  static uint8_t lChangeSign = 0; // 0 is FALSE, 1 is TRUE

  static uint16_t lhalf_bus = ZC_HALF_BUS;
  static int32_t  lStateIterator = 0;

  static uint8_t count2[4] = {0};

  static uint16_t ret_val = 0;
  static uint16_t motorNb = 0;

  motorNb = motor->motorNb;

  lStateIterator = brushcfg_GetStateIterator(motor);
  MeasureChannel = motor->kChannelMeasureArray[lStateIterator];


  if(((motor->pwmp->tim->CCR[kTimChannel1]+1)*100/PERIOD_PWM_52_KHZ) < 60){
    // Check if the sign has changed between old measurement and actual
    ret_val = 0;
    if(zcs->data_idx[motorNb] > TWO_ELEM_IDX && !motor->ZCFlag)
    {
      lCurMeasure = (int32_t) zcs->data[motorNb][1][LATEST_DATA(zcs->data_idx[motorNb])]   - (int32_t)(CORR_FACTOR_HALF_BUS * lhalf_bus);
      lOldMeasure = (int32_t) zcs->data[motorNb][1][PREVIOUS_DATA(zcs->data_idx[motorNb])] - (int32_t)(CORR_FACTOR_HALF_BUS * lhalf_bus);
      // lCurMeasure = (int32_t) zcs->data[motorNb][LATEST_DATA(zcs->data_idx)]   - motor1.kChannelNeutralPoint[lStateIterator];
      // lOldMeasure = (int32_t) zcs->data[motorNb][PREVIOUS_DATA(zcs->data_idx)] - motor1.kChannelNeutralPoint[lStateIterator];
      
      lChangeSign = ((lOldMeasure ^ lCurMeasure) < 0); // TRUE if sign has changed
       //motor1.ZCFlag |= lChangeSign;
      ret_val = (MeasureChannel + 1)*lChangeSign;

      if(ret_val > 0)
      {
        brushcfg_ComputeZCPeriod(motor);
        brushcfg_SetZCFlag(motor);
      }
    }
  }else{
    ret_val = 0;
    if(!motor->ZCFlag)
    {
      if(count2[motorNb] > 1){
        if(motor->kchannelSlope[lStateIterator] == 0){
          if(zcs->data[motorNb][0][LATEST_DATA(zcs->data_idx[motorNb])] > motor->kchannelOffset[motor->kChannelMeasureArray[lStateIterator]]){
            ret_val = (MeasureChannel + 1);
            brushcfg_ComputeZCPeriod(motor);
            brushcfg_SetZCFlag(motor);
            count2[motorNb] = 0;
          }
        }else{
          if(zcs->data[motorNb][0][LATEST_DATA(zcs->data_idx[motorNb])] <= motor->kchannelOffset[motor->kChannelMeasureArray[lStateIterator]]){
            ret_val = (MeasureChannel + 1);
            brushcfg_ComputeZCPeriod(motor);
            brushcfg_SetZCFlag(motor);
            count2[motorNb] = 0;
          }
        }
      }else{
        count2[motorNb]++;
      }
    }
  }
    motor->TimeBLDCCommut++;
    commutation_nextstep(motor);

    motor->pwmp->tim->EGR |= STM32_TIM_EGR_COMG;
  


  // if(ret_val > 0)
  // {
  //   brushcfg_ComputeZCPeriod(&motor1);
  //   brushcfg_SetZCFlag(&motor1);
  // }

  return ret_val;

}

void zcs_ext_reset(uint16_t motorNb)
{
  Zcs_Reset_Struct(&gZCS, motorNb);
}
