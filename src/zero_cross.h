/*
 * zero_cross.h
 *
 *  Created on: 4 juin 2019
 *      Author: Ismail-P51
 */

#ifndef ZERO_CROSS_H_
#define ZERO_CROSS_H_

#include <ch.h>
#include <hal.h>

#include "tim1_motor.h"

/*  ZERO-CROSSING DETECTION */
#define ZC_NUMBER_CHANNELS      4


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

#define TWO_ELEM_IDX            1
#define SIX_ELEM_IDX            7
#define LATEST_DATA(x)          (x) - 1
#define PREVIOUS_DATA(x)        (x) - 2

#include "adc_motor.h"

/*===========================================================================*/
/* Typedefs                                                                  */
/*===========================================================================*/
typedef struct
{
  uint16_t data         [ZC_NUMBER_CHANNELS * 2][ZC_NB_POINTS];
  uint8_t  slope        [ZC_NUMBER_CHANNELS * 2][ZC_NB_POINTS];
  uint8_t  zero_crossing[ZC_NUMBER_CHANNELS * 2];

  // DATA MGT
  const uint16_t nb_channels;
  const uint16_t nb_points;
  uint16_t data_left;
  uint16_t data_idx;
  uint8_t  data_full;

}ZCSDetect;

/*===========================================================================*/
/* Prototypes                                                                */
/*===========================================================================*/
/* Zero crossing detection */
uint16_t Zcs_Get_average(void);
void Zcs_Reset_Average(void);
void Zcs_Average(uint16_t* input_data, size_t size);
void Zcs_Reset_Struct(ZCSDetect* zcs);
void Zcs_Insert_Data (ZCSDetect* zcs,uint16_t* input_data,size_t size, uint8_t onOff);
uint8_t Zcs_Detect(ZCSDetect* zcs, uint8_t motorNb);
void zcs_ext_reset(void);

/*===========================================================================*/
/* Export global variable                                                    */
/*===========================================================================*/
extern ZCSDetect gZCS;


#endif /* ZERO_CROSS_H_ */
