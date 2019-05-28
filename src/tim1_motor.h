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

/*===========================================================================*/
/* Typedefs                                                                  */
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
  kAlign    = 1,
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

  /* Ramp speed */

  uint32_t RampTime; // Number of cycle in a given speed
  const uint32_t kMaxRampTime;

  virtual_timer_t ramp_vt;
  sysinterval_t RampInterval;
  uint8_t RampTimeout;

  uint32_t RampMinSpeed;
  uint32_t RampMaxSpeed;
  uint32_t RampCurSpeed;
  uint32_t RampStep;

  /* Startup alignement */
  virtual_timer_t align_vt;
  sysinterval_t   AlignInterval;
  uint8_t         AlignInProgress;
  uint8_t         AlignTimeout;

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



  /* Speed control */
  uint16_t Speed;

}BrushlessConfig;

/*===========================================================================*/
/* Prototypes                                                                */
/*===========================================================================*/
void commutation_cb (PWMDriver *pwmp);
void pwm_cb_ch4(PWMDriver *pwmp);
void timer_1_pwm_config (void);


#endif /* TIM1_MOTOR_H_ */
