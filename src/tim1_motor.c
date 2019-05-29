/*
 * tim1_motor.c
 *
 *  Created on: 10 avr. 2019
 *      Author: Ismail-P51
 */


#include <ch.h>
#include <hal.h>

#include "custom_io.h"
#include "tim1_motor.h"
#include "main.h"

/*===========================================================================*/
/* Variables                                                                 */
/*===========================================================================*/
// PWM

BrushlessConfig gBrushCfg = {

    .RotationDir=kCCW,
    .StateIterator=0,
    .InStepCount = 0,
    .kMaxStepCount = 55,
    .kDefaultIOConfig = PAL_STM32_MODE_ALTERNATE | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLDOWN | PAL_STM32_ALTERNATE(1),
    .Mode = kInitCfg,

    /** Configuration of the commutation steps of the brushless motor **/

    /* Channel 1 - Channel 1 Comp - Channel 2 - Channel 2 Comp - Channel 3 - Channel 3 Comp */

    /* ALL CHANNELS OFF */
    .kChannelStateArray[kStop] =    {kTimCh_Low,kTimCh_Low,kTimCh_Low,kTimCh_Low,kTimCh_Low,kTimCh_Low},

    /* PWM Simple from DRV 8323 */
    /* C1 : ENABLE - C1C - C2 - C2C : ENABLE - C3 - C3C */
    .kChannelStateArray[kPhaseUV] = {kTimCh_PWM,kTimCh_PWM,kTimCh_Low,kTimCh_Low,kTimCh_Low,kTimCh_High},
    /* C1 : ENABLE - C1C - C2 - C2C - C3 - C3C : ENABLE */
    .kChannelStateArray[kPhaseUW] = {kTimCh_PWM,kTimCh_PWM,kTimCh_Low,kTimCh_High,kTimCh_Low,kTimCh_Low},
    /* C1 - C1C - C2 : ENABLE - C2C - C3 - C3C : ENABLE */
    .kChannelStateArray[kPhaseVW] = {kTimCh_Low,kTimCh_Low,kTimCh_Low,kTimCh_High,kTimCh_PWM,kTimCh_PWM},
    /* C1 - C1C : ENABLE - C2 : ENABLE - C2C - C3 - C3C */
    .kChannelStateArray[kPhaseVU] = {kTimCh_Low,kTimCh_High,kTimCh_Low,kTimCh_Low,kTimCh_PWM,kTimCh_PWM},
    /* C1 - C1C : ENABLE - C2 - C2C - C3 : ENABLE - C3C */
    .kChannelStateArray[kPhaseWU] = {kTimCh_Low,kTimCh_High,kTimCh_PWM,kTimCh_PWM,kTimCh_Low,kTimCh_Low},
    /* C1 - C1C - C2 - C2C : ENABLE - C3 : ENABLE - C3C */
    .kChannelStateArray[kPhaseWV] = {kTimCh_Low,kTimCh_Low,kTimCh_PWM,kTimCh_PWM,kTimCh_Low,kTimCh_High},

    /* PWM Double from scratch */
/*    .kChannelStateArray[kPhaseUV] = {kTimCh_PWM,kTimCh_Low,kTimCh_Low,kTimCh_PWM,kTimCh_Low,kTimCh_Low},
    .kChannelStateArray[kPhaseUW] = {kTimCh_PWM,kTimCh_Low,kTimCh_Low,kTimCh_Low,kTimCh_Low,kTimCh_PWM},
    .kChannelStateArray[kPhaseVW] = {kTimCh_Low,kTimCh_Low,kTimCh_PWM,kTimCh_Low,kTimCh_Low,kTimCh_PWM},
    .kChannelStateArray[kPhaseVU] = {kTimCh_Low,kTimCh_PWM,kTimCh_PWM,kTimCh_Low,kTimCh_Low,kTimCh_Low},
    .kChannelStateArray[kPhaseWU] = {kTimCh_Low,kTimCh_PWM,kTimCh_Low,kTimCh_Low,kTimCh_PWM,kTimCh_Low},
    .kChannelStateArray[kPhaseWV] = {kTimCh_Low,kTimCh_Low,kTimCh_Low,kTimCh_PWM,kTimCh_PWM,kTimCh_Low},*/

    /** Ramp speed **/

    .RampInterval = TIME_MS2I(10),
    .RampMaxSpeed = 26,
    .RampTimeout  = 0,
    .RampMinSpeed = 100,
    .RampCurSpeed = 0,
    .RampStep = 1,
    .RampTime = 0,
    .kMaxRampTime = 2,

    /** Alignement **/
    .AlignInterval   = TIME_MS2I(10),
    .AlignTimeout    = 0,
    .AlignInProgress = 0,

    /* Zero-crossing valid */
    .ZCVCount       =0,

    /* Zero-crossing conf */
    .ZCFlag         = 0,
    .ZCDetect       = 0,
    .ZCDetectOld    = 0,
    .ZCPeriod       = 0,
    .ZCPeriodOld    = 0,
    .ZCNextCommut   = 0,
    .ZCTimeout      = 0,
    .ZCTiming       = 0.5,

    /** IO Configuration **/
    .P_Channels = {LINE_OUT_MOT1_PH1_P,LINE_OUT_MOT1_PH2_P,LINE_OUT_MOT1_PH3_P},
    .N_Channels = {LINE_OUT_MOT1_PH1_N,LINE_OUT_MOT1_PH2_N,LINE_OUT_MOT1_PH3_N}
};

/*===========================================================================*/
/* Timer 1 Configuration                                                     */
/*===========================================================================*/

static void tim_1_oc_cmd(TimChannel aChannel,TimChannelState aState)
{
  uint32_t lCCEnable;
  switch(aChannel)
  {
    case kTimChannel1:
    {
      lCCEnable = STM32_TIM_CCER_CC1E;
      break;
    }

    case kTimChannel2:
    {
      lCCEnable = STM32_TIM_CCER_CC2E;
      break;
    }

    case kTimChannel3:
    {
      lCCEnable = STM32_TIM_CCER_CC3E;
      break;
    }

    case kTimChannel4:
    {
     lCCEnable = STM32_TIM_CCER_CC4E;
      break;
    }

    case kTimChannel5:
    {
      lCCEnable = STM32_TIM_CCER_CC5E;
      break;
    }

    case kTimChannel6:
    {
      lCCEnable = STM32_TIM_CCER_CC6E;
      break;
    }

    default:
      break;
  }

  switch(aState)
  {
    case kTimCh_Low:
    {
      palSetLineMode(gBrushCfg.P_Channels[aChannel],PAL_MODE_OUTPUT_PUSHPULL);
      palClearLine(gBrushCfg.P_Channels[aChannel]);
      (&PWMD1)->tim->CCER &= (~lCCEnable);
      break;
    }
    case kTimCh_High:
    {
      palSetLineMode(gBrushCfg.P_Channels[aChannel],PAL_MODE_OUTPUT_PUSHPULL);
      palSetLine(gBrushCfg.P_Channels[aChannel]);
      (&PWMD1)->tim->CCER &= (~lCCEnable);
      break;
    }
    case kTimCh_PWM:
    {
      palSetLineMode(gBrushCfg.P_Channels[aChannel],gBrushCfg.kDefaultIOConfig);
      (&PWMD1)->tim->CCER |= lCCEnable;
      break;
    }
  }
}

static void tim_1_ocn_cmd(TimChannel aChannel,TimChannelState aState)
{
  uint32_t lCCNEnable;
  switch(aChannel)
  {
    case kTimChannel1:
    {
      lCCNEnable = STM32_TIM_CCER_CC1NE;
      break;
    }

    case kTimChannel2:
    {
      lCCNEnable = STM32_TIM_CCER_CC2NE;
      break;
    }

    case kTimChannel3:
    {
      lCCNEnable = STM32_TIM_CCER_CC3NE;
      break;
    }

    case kTimChannel4:
    case kTimChannel5:
    case kTimChannel6:
    default:
    {

    }
  }

  switch(aState)
  {
    case kTimCh_Low:
    {
      palSetLineMode(gBrushCfg.N_Channels[aChannel],PAL_MODE_OUTPUT_PUSHPULL);
      palClearLine(gBrushCfg.N_Channels[aChannel]);
      (&PWMD1)->tim->CCER &= (~lCCNEnable);
      break;
    }
    case kTimCh_High:
    {
      palSetLineMode(gBrushCfg.N_Channels[aChannel],PAL_MODE_OUTPUT_PUSHPULL);
      palSetLine(gBrushCfg.N_Channels[aChannel]);
      (&PWMD1)->tim->CCER &= (~lCCNEnable);
      break;
    }
    case kTimCh_PWM:
    {
      palSetLineMode(gBrushCfg.N_Channels[aChannel],gBrushCfg.kDefaultIOConfig);
      (&PWMD1)->tim->CCER |= lCCNEnable;
      break;
    }
  }
}

static void tim_1_oc_start(TimChannel aChannel)
{
  tim_1_oc_cmd(aChannel,kTimCh_PWM);
}

static void tim_1_oc_stop(TimChannel aChannel)
{
  tim_1_oc_cmd(aChannel,kTimCh_Low);
}

static void tim_1_ocn_start(TimChannel aChannel)
{
  tim_1_ocn_cmd(aChannel,kTimCh_PWM);
}

static void tim_1_ocn_stop(TimChannel aChannel)
{
  tim_1_ocn_cmd(aChannel,kTimCh_Low);
}



void timer_1_pwm_config (void)
{
    (&PWMD1)->tim->CR1 &= (~STM32_TIM_CR1_CEN);     // Disable the counter until correct configuration
    (&PWMD1)->tim->CR1 &= (~STM32_TIM_CR1_CMS(0));  // Edge-aligned mode
    (&PWMD1)->tim->CR1 &= (~STM32_TIM_CR1_DIR);     // Direction : upcounter
    (&PWMD1)->tim->PSC =  0;                        // Set the prescaler to 0 TIM_FREQ = 216 MHz
    (&PWMD1)->tim->ARR =  PERIOD_PWM_20_KHZ - 1;    // Set the period of our PWM

    (&PWMD1)->tim->CR1 &= (~STM32_TIM_CR1_ARPE);    // Remove the ARPE
    (&PWMD1)->tim->CR2 |= STM32_TIM_CR2_CCPC;       // Enable the Preload of the CxE,CxNE bits

    (&PWMD1)->tim->CCER = 0; // Reset Capture/Compare Register
    //

    // 1 : Output channels configuration
    // General Config
    (&PWMD1)->tim->CCMR1 = 0;                         // Reset OC 1 and OC2 configuration
    (&PWMD1)->tim->CCMR2 = 0;                         // Reset OC 3 and OC4 configuration

    // Channel 1 Config
    (&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC1P);    // OC1 Polarity  : Active High
    (&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC1NP);   // OC1N Polarity : Active High
    (&PWMD1)->tim->CR2  &=  (~STM32_TIM_CR2_OIS1);    // OC1 Idle State (when MOE=0) : 0
    (&PWMD1)->tim->CR2  &=  (~STM32_TIM_CR2_OIS1N);   // OC1N Idle State (when MOE=0): 0
    (&PWMD1)->tim->CCMR1|=  STM32_TIM_CCMR1_OC1M(kFrozen);  // OC1 Mode : Frozen
    (&PWMD1)->tim->CCMR1 &= (~STM32_TIM_CCMR1_OC1FE); // Disable the Fast Mode
    (&PWMD1)->tim->CCMR1 |= STM32_TIM_CCMR1_OC1PE;    // Enable the Preload -> CCR is loaded in the active register at each update event
    // (&PWMD1)->tim->CCR[kTimChannel1] =  PERIOD_PWM_20_KHZ/2 - 1;  // Select the Half-period to overflow

    // Channel 2 Config
    (&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC2P);    // OC2 Polarity  : Active High
    (&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC2NP);   // OC2N Polarity : Active High
    (&PWMD1)->tim->CR2  &=  (~STM32_TIM_CR2_OIS2);    // OC2 Idle State (when MOE=0) : 0
    (&PWMD1)->tim->CR2  &=  (~STM32_TIM_CR2_OIS2N);   // OC2N Idle State (when MOE=0): 0
    (&PWMD1)->tim->CCMR1|=  STM32_TIM_CCMR1_OC2M(kFrozen);  // OC2 Mode : Frozen
    (&PWMD1)->tim->CCMR1 &= (~STM32_TIM_CCMR1_OC2FE); // Disable the Fast Mode
    (&PWMD1)->tim->CCMR1 |= STM32_TIM_CCMR1_OC2PE;    // Enable the Preload
    // (&PWMD1)->tim->CCR[kTimChannel2] =  PERIOD_PWM_20_KHZ/4 - 1;  // Select the Quarter-period to overflow

    // Channel 3 Config
    (&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC3P);    // OC3 Polarity  : Active High
    (&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC3NP);   // OC3N Polarity : Active High
    (&PWMD1)->tim->CR2  &=  (~STM32_TIM_CR2_OIS3);    // OC3 Idle State (when MOE=0) : 0
    (&PWMD1)->tim->CR2  &=  (~STM32_TIM_CR2_OIS3N);   // OC3N Idle State (when MOE=0): 0
    (&PWMD1)->tim->CCMR2|=  STM32_TIM_CCMR2_OC3M(kFrozen);  // OC3 Mode : Frozen
    (&PWMD1)->tim->CCMR2 &= (~STM32_TIM_CCMR2_OC3FE); // Disable the Fast Mode
    (&PWMD1)->tim->CCMR2 |= STM32_TIM_CCMR2_OC3PE;    // Enable the Preload
    // (&PWMD1)->tim->CCR[kTimChannel3]  =  PERIOD_PWM_20_KHZ/8 - 1;  // Select the 1/8 period to overflow

    // Channel 4 Config (for ADC trigger)
    (&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC4P);    // OC4  Polarity : Active High
    (&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC4NP);   // OC4N Polarity : Active High
    (&PWMD1)->tim->CR2  &= (~STM32_TIM_CR2_OIS4);     // OC4 Idle State (when MOE=0): 0
    (&PWMD1)->tim->CCMR2|=  STM32_TIM_CCMR2_OC4M(kPWMMode2);  // OC4 Mode : PWM Mode 2
    (&PWMD1)->tim->CCMR2 |= STM32_TIM_CCMR2_OC4PE;    // Enable the Preload -> CCR is loaded in the active register at each update event
    (&PWMD1)->tim->CCMR2 &= (~STM32_TIM_CCMR2_OC4FE); // Disable the Fast Mode

    (&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC6P);    // OC6  Polarity : Active High
    (&PWMD1)->tim->CR2  &= (~STM32_TIM_CR2_OIS6);     // OC6 Idle State (when MOE=0): 0
    (&PWMD1)->tim->CCMR3|=  STM32_TIM_CCMR3_OC6M(kPWMMode1);  // OC6 Mode : PWM Mode 1
    (&PWMD1)->tim->CCMR3 |= STM32_TIM_CCMR3_OC6PE;    // Enable the Preload -> CCR is loaded in the active register at each update event
    (&PWMD1)->tim->CCMR3 &= (~STM32_TIM_CCMR3_OC6FE); // Disable the Fast Mode

    // Configure the time and the interruption enable

    // BSP - Style
    // (&PWMD1)->tim->CCR4   =  PERIOD_100_MS_INT;    // Each 100 ms an interruption
    // (&PWMD1)->tim->DIER |= STM32_TIM_DIER_CC4IE;   // Enable Capture/Compare 4 Interrupt



    // ChibiOS - Style
    pwmEnableChannel(&PWMD1, kTimChannel4 , 0.95 * PERIOD_PWM_20_KHZ - 1);                 // Enabled to allow HAL support
    pwmEnableChannelNotification(&PWMD1, kTimChannel4);         // Enable the callback to be called for the specific channel
  
    pwmEnableChannel(&PWMD1, kTimChannel6 , 0.50 * PERIOD_PWM_20_KHZ - 1);                 // Enabled to allow HAL support
    
    // Break stage configuration

    (&PWMD1)->tim->CR1 |= STM32_TIM_CR1_CKD(0);     // Modification of the CR1 CKD in order to have a bigger period for the dead times
                                                    // OSSR and OSSI not needed
    (&PWMD1)->tim->BDTR = 0;                        // Reset BDTR to everything disabled (BK,BK2 not enabled)
    (&PWMD1)->tim->BDTR |= STM32_TIM_BDTR_DTG(0);   // Dead-time generator Setup
    (&PWMD1)->tim->BDTR |= STM32_TIM_BDTR_OSSR;     // Off-state selection for Run Mode
    (&PWMD1)->tim->BDTR |= STM32_TIM_BDTR_MOE;      // Main Output Enable

    // Commutation event configuration (Not needed at the moment.)

    // Select OC4REF as TRGO2
    (&PWMD1)->tim->CR2 |= STM32_TIM_CR2_MMS2(7); // Master Mode Selection 2 : OC4REF
    //(&PWMD1)->tim->CR2 |= STM32_TIM_CR2_MMS2(13); // Master Mode Selection 2 : OC44REF Rising or OC6REF falling

    // Force update event (if preload enabled)
    (&PWMD1)->tim->EGR |= STM32_TIM_EGR_UG;

    // Force update event (if preload enabled) for CxE,CxNE and OCxM
    (&PWMD1)->tim->EGR |= STM32_TIM_EGR_COMG;

    // Enable the Timer
    (&PWMD1)->tim->CR1 |= STM32_TIM_CR1_CEN;     // Enable the counter in correct configuration

    chVTObjectInit(&gBrushCfg.ramp_vt);  // Init the virtual timer
    chVTObjectInit(&gBrushCfg.align_vt); // Init the virtual timer
}


void commutation_step(BrushlessConfig *pBrushCfg)
{
  if(pBrushCfg->RotationDir==kCCW)
  {
    pBrushCfg->StateIterator++;
    if(NB_STATE <= pBrushCfg->StateIterator)
    {
      pBrushCfg->StateIterator=kPhaseUV;
    }

  }
  else
  {
    pBrushCfg->StateIterator--;
    if(((int32_t) kStop) >= pBrushCfg->StateIterator)
    {
      pBrushCfg->StateIterator=kPhaseWV;
    }
  }
  pBrushCfg->InStepCount = 0;
}


void commutation_nextstep(BrushlessConfig *pBrushCfg)
{
  static volatile uint32_t else_cnt = 0;

  switch(pBrushCfg->Mode)
  {
    case kInitRamp:
      pBrushCfg->InStepCount++;
      if (pBrushCfg->kMaxStepCount <= pBrushCfg->InStepCount)
      {
        commutation_step(pBrushCfg);
        if(1==gBrushCfg.ZCFlag){
          gBrushCfg.ZCFlag = 0;
          gBrushCfg.ZCVCount++;
        }
      }
      break;

    case kEndless:
      if(pBrushCfg->TimeBLDCCommut >= pBrushCfg->ZCNextCommut)
      {
        if(1== pBrushCfg->ZCFlag)
        {
          pBrushCfg->ZCFlag=0; // Reset zero-crossing flag
          commutation_step(pBrushCfg);
          zcs_ext_reset();
          gBrushCfg.ZCNextCommut = gBrushCfg.TimeBLDCCommut + (36 * gBrushCfg.ZCPeriodMean);
        }
        else
        {
          else_cnt++;
        }
      }
      break;
    default:
      break;
  }
}


void commutation_zc_reset(BrushlessConfig *pBrushCfg)
{
  pBrushCfg->ZCFlag        = 0;
  pBrushCfg->ZCDetect      = 0;
  pBrushCfg->ZCDetectOld   = 0;
  pBrushCfg->ZCPeriod      = 20;
  pBrushCfg->ZCPeriodOld   = 0;
  pBrushCfg->ZCPeriodMean  = 0;
  pBrushCfg->ZCNextCommut  = 40;
  pBrushCfg->ZCTimeout     = 100;
}

static void vt_cb(void* arg)
{
  if(kInitRamp == gBrushCfg.Mode)
  {
    gBrushCfg.RampTimeout = 1;
  }
  else if(kAlign == gBrushCfg.Mode)
  {
    gBrushCfg.AlignTimeout = 1;
  }

}



void pwm_cb_ch4(PWMDriver *pwmp)
{
  palSetLine(DEBUG_INT_LINE2);
}


void commutation_cb(PWMDriver *pwmp)
{
  palClearLine(DEBUG_INT_LINE2);
  palSetLine(LD2_LINE);

  switch (gBrushCfg.Mode)
  {


    case kInitCfg:
    {

      /* Timer 1 config */
      /* Stop all the channels */
      tim_1_oc_stop(kTimChannel1);
      tim_1_oc_stop(kTimChannel2);
      tim_1_oc_stop(kTimChannel3);
      tim_1_ocn_stop(kTimChannel1);
      tim_1_ocn_stop(kTimChannel2);
      tim_1_ocn_stop(kTimChannel3);

      /* Set all the OC output to same PWM */
      (&PWMD1)->tim->CCR[kTimChannel1]  =  0.9 * PERIOD_PWM_20_KHZ - 1;  // Select the quarter-Period to overflow
      (&PWMD1)->tim->CCR[kTimChannel2]  =  0.9 * PERIOD_PWM_20_KHZ - 1;  // Select the quarter-Period to overflow
      (&PWMD1)->tim->CCR[kTimChannel3]  =  0.9 * PERIOD_PWM_20_KHZ - 1;  // Select the quarter-Period to overflow

      // Force update event (if preload enabled)
      (&PWMD1)->tim->EGR |= STM32_TIM_EGR_UG;

      /* Configure the mode of each channel */
      (&PWMD1)->tim->CCMR1|=  STM32_TIM_CCMR1_OC1M(kPWMMode2);  // OC1 Mode : PWM Mode 2
      (&PWMD1)->tim->CCMR1|=  STM32_TIM_CCMR1_OC2M(kPWMMode2);  // OC2 Mode : PWM Mode 2
      (&PWMD1)->tim->CCMR2|=  STM32_TIM_CCMR2_OC3M(kPWMMode2);  // OC3 Mode : PWM Mode 2

      // Force update event (if preload enabled)
      (&PWMD1)->tim->EGR |= STM32_TIM_EGR_COMG;

      gBrushCfg.Mode = kAlign;
      break;
    }

    case kAlign:
    {

      if(0 == gBrushCfg.AlignInProgress)
      {
        gBrushCfg.AlignInProgress = 1;

        // Set to UV phase
        tim_1_oc_cmd(kTimChannel1,kTimCh_High);
        tim_1_ocn_cmd(kTimChannel2,kTimCh_High);
        //tim_1_ocn_cmd(kTimChannel3,kTimCh_High);

        chSysLockFromISR();
        chVTSetI(&gBrushCfg.align_vt, gBrushCfg.AlignInterval, vt_cb, NULL); // Start the virtual timer
        chSysUnlockFromISR();
      }

      if(1==gBrushCfg.AlignTimeout)
      {
        // Disconnect the selected path
        tim_1_oc_cmd(kTimChannel1,kTimCh_Low);
        tim_1_ocn_cmd(kTimChannel2,kTimCh_Low);
        //tim_1_ocn_cmd(kTimChannel3,kTimCh_Low);

        chSysLockFromISR();
        chVTSetI(&gBrushCfg.ramp_vt, gBrushCfg.RampInterval, vt_cb, NULL); // Start the virtual timer
        chSysUnlockFromISR();

        gBrushCfg.Mode = kInitRamp;
        gBrushCfg.RampCurSpeed = gBrushCfg.RampMinSpeed;
      }

     break;
    }

    case kInitRamp:
    {

      // Set the speed to the ramp speed
      gBrushCfg.kMaxStepCount = gBrushCfg.RampCurSpeed;

      tim_1_oc_cmd(kTimChannel1 ,gBrushCfg.kChannelStateArray[gBrushCfg.StateIterator][0]);
      tim_1_ocn_cmd(kTimChannel1,gBrushCfg.kChannelStateArray[gBrushCfg.StateIterator][1]);
      tim_1_oc_cmd(kTimChannel2 ,gBrushCfg.kChannelStateArray[gBrushCfg.StateIterator][2]);
      tim_1_ocn_cmd(kTimChannel2,gBrushCfg.kChannelStateArray[gBrushCfg.StateIterator][3]);
      tim_1_oc_cmd(kTimChannel3 ,gBrushCfg.kChannelStateArray[gBrushCfg.StateIterator][4]);
      tim_1_ocn_cmd(kTimChannel3,gBrushCfg.kChannelStateArray[gBrushCfg.StateIterator][5]);
      commutation_nextstep(&gBrushCfg);

      // Detect when we got a timeout
      if(1 == gBrushCfg.RampTimeout)
      {

        gBrushCfg.RampTimeout = 0;
        gBrushCfg.RampTime++;
        // Check if we can change the speed
        if(gBrushCfg.RampTime >= gBrushCfg.kMaxRampTime)
        {
          gBrushCfg.RampTime = 0;
          gBrushCfg.RampCurSpeed -= gBrushCfg.RampStep;

        }

        chSysLockFromISR();
        chVTSetI(&gBrushCfg.ramp_vt, gBrushCfg.RampInterval, vt_cb, NULL); // Restart the virtual timer
        chSysUnlockFromISR();

        // Check if we have done all the ramp speeds
        if(gBrushCfg.RampMaxSpeed >= gBrushCfg.RampCurSpeed || 6==gBrushCfg.ZCVCount)
        {
          chSysLockFromISR();
          chVTResetI(&gBrushCfg.ramp_vt);
          chSysUnlockFromISR();
          gBrushCfg.Mode = kEndless;
          /* Reset all the zero-crossing variables */
          commutation_zc_reset(&gBrushCfg);
          gBrushCfg.TimeBLDCCommut = 0;
        }

      }

      break;
    }


    case kEndless:
    {
      gBrushCfg.TimeBLDCCommut++;

      tim_1_oc_cmd(kTimChannel1 ,gBrushCfg.kChannelStateArray[gBrushCfg.StateIterator][0]);
      tim_1_ocn_cmd(kTimChannel1,gBrushCfg.kChannelStateArray[gBrushCfg.StateIterator][1]);
      tim_1_oc_cmd(kTimChannel2 ,gBrushCfg.kChannelStateArray[gBrushCfg.StateIterator][2]);
      tim_1_ocn_cmd(kTimChannel2,gBrushCfg.kChannelStateArray[gBrushCfg.StateIterator][3]);
      tim_1_oc_cmd(kTimChannel3 ,gBrushCfg.kChannelStateArray[gBrushCfg.StateIterator][4]);
      tim_1_ocn_cmd(kTimChannel3,gBrushCfg.kChannelStateArray[gBrushCfg.StateIterator][5]);

      commutation_nextstep(&gBrushCfg);
      break;
    }

    default:
      break;
  }
  palClearLine(LD2_LINE);
  // Force update event (if preload enabled) for CxE,CxNE and OCxM
  (&PWMD1)->tim->EGR |= STM32_TIM_EGR_COMG;
}
