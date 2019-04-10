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
#define NB_STATE                7

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
  kInitRamp = 0,
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
    kTimCh_Disable = 0,
    kTimCh_Enable  = 1
}TimChannelState;


/*===========================================================================*/
/* Structures                                                                */
/*===========================================================================*/
typedef struct
{
  CommutationStateMachine StateCommutation;
  Rotation RotationDir;
  uint32_t InStepCount;             // Count the number of iteration has done in a given step,determine the frequency of the 6 steps
  uint32_t kMaxStepCount;           // Maximum number of iteration inside a step
  const iomode_t kDefaultIOConfig;
  ioline_t P_Channels[NB_PHASE];
  ioline_t N_Channels[NB_PHASE];

  int32_t StateIterator;
  const CommutationStateMachine StateArray[NB_STATE];

}BrushlessConfig;

/*===========================================================================*/
/* Prototypes                                                                */
/*===========================================================================*/
void commutation_cb (PWMDriver *pwmp);
void timer_1_pwm_config (void);


#endif /* TIM1_MOTOR_H_ */
