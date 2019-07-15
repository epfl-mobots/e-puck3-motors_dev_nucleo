/*
 * tim1_motor.h
 *
 *  Created on: 10 avr. 2019
 *      Author: Ismail-P51
 */

#ifndef TIM1_MOTOR_H_
#define TIM1_MOTOR_H_


/*===========================================================================*/
/* Define                                                                    */
/*===========================================================================*/

#define PERIOD_PWM_32_KHZ       6750
#define PERIOD_PWM_52_KHZ       4096
#define PERIOD_PWM_20_KHZ       10800
#define PERIOD_100_MS_INT       5273

#define NB_PHASE                3
#define NB_CHANNELS             2 * NB_PHASE
#define NB_STATE                7
#define NB_RAMP_STEPS           5

#define COEF_ADV                (float)0.375
#define COEF_MARGIN             10


/*===========================================================================*/
/* Typedefs                                                                  */
/*===========================================================================*/


/*===========================================================================*/
/* ADC                                                                       */
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
/* Brushless Cfg                                                             */
/*===========================================================================*/

typedef enum
{
  kCW   =0,
  kCCW  =1
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
  kInitCfg  = 0,
  kCalibrate    = 1,
  kInitRamp = 2,
  kValidSensorless = 3,
  kEndless  = 255
}CmdMode;


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
  kReset_TRGO2  = 0,
  kEnable_TRGO2 = 1,
  kUpdate_TRGO2 = 2,
  kCompPulse_CC1IF = 3
}TimMMS2;


typedef enum
{
    kTimCh_Low = 0,
    kTimCh_High  = 1,
    kTimCh_PWM = 2
}TimChannelState;


/*===========================================================================*/
/* Structures                                                                */
/*===========================================================================*/
typedef struct
{

  Rotation RotationDir;
  uint32_t InStepCount;             // Count the number of iteration has done in a given step,determine the frequency of the 6 steps
  uint32_t kMaxStepCount;           // Maximum number of iteration inside a step
  const iomode_t kDefaultIOConfig;
  ioline_t P_Channels[NB_PHASE];
  ioline_t N_Channels[NB_PHASE];


  /* Actual command mode inside the brushless config */
  CmdMode Mode;

  /* Commutation tables and iterator */
  int32_t StateIterator;
  const TimChannelState kChannelStateArray[NB_STATE][NB_CHANNELS]; // Each phase has a P and N channel
  const uint8_t kChannelMeasureArray[NB_STATE]; // To know which channel to measure
  const uint8_t kchannelSlope[NB_STATE];
  uint16_t kchannelOffset[NB_PHASE];
  uint16_t kChannelNeutralPoint[NB_STATE];  // To store the calibration values of the neutral points

  /* Ramp speed */

  uint32_t RampTime; // Number of cycle in a given speed
  const uint32_t kMaxRampTime;

  virtual_timer_t ramp_vt;
  sysinterval_t RampInterval;

  virtual_timer_t calibration_vt;
  sysinterval_t CalibrationInterval;
  uint8_t       CalibrationTimeout;
  uint8_t       CalibrationState;
  uint8_t       DoAverage;
  uint8_t RampTimeout;

  uint32_t RampMinSpeed;
  uint32_t RampMaxSpeed;
  uint32_t RampCurSpeed;
  uint32_t RampStep;

  uint16_t ZCVCount;

  /* Zero-crossing */
  uint32_t TimeBLDCCommut;


  uint32_t ZCDetect;
  uint32_t ZCDetectOld;

  uint32_t ZCTimeout;

  uint32_t ZCPeriod;
  uint32_t ZCPeriodOld;
  uint32_t ZCPeriodMean;
  uint32_t ZCNextCommut;
  uint8_t  ZCFlag;
  float  ZCTiming;



  /* Speed control */
  uint16_t Speed;

}BrushlessConfig;

/*===========================================================================*/
/* Prototypes                                                                */
/*===========================================================================*/


/*===========================================================================*/
/* IO                                                                        */
/*===========================================================================*/
void initTIM1MotorIo(void);

/*===========================================================================*/
/* ADC                                                                       */
/*===========================================================================*/


/*===========================================================================*/
/* Brushless cfg                                                             */
/*===========================================================================*/
int32_t brushcfg_GetStateIterator   (BrushlessConfig* bcfg);
void    brushcfg_SetZCFlag          (BrushlessConfig* bcfg);
void    brushcfg_ClearZCFlag        (BrushlessConfig* bcfg);
void    brushcfg_ComputeZCPeriod    (BrushlessConfig* bcfg);

/*===========================================================================*/
/* Misc.                                                                     */
/*===========================================================================*/

void commutation_cb (PWMDriver *pwmp);
void pwm_cb_ch4(PWMDriver *pwmp);
void timer_1_pwm_config (void);
void timer1Start(void);

/*===========================================================================*/
/* Export global variable                                                    */
/*===========================================================================*/
extern BrushlessConfig gBrushCfg;


#endif /* TIM1_MOTOR_H_ */
