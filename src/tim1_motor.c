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

/*===========================================================================*/
/* Variables                                                                 */
/*===========================================================================*/
// PWM

BrushlessConfig gBrushCfg = {
    .StateCommutation=kStop,
    .RotationDir=kCCW,
    .StateIterator=0,
    .StateArray = {kStop,kPhaseUV,kPhaseUW,kPhaseVW,kPhaseVU,kPhaseWU,kPhaseWV},
    .InStepCount = 0,
    .kMaxStepCount = 666,
    .kDefaultIOConfig = PAL_STM32_MODE_ALTERNATE | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLDOWN | PAL_STM32_ALTERNATE(1),
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
    case kTimCh_Disable:
    {
      palSetLineMode(gBrushCfg.P_Channels[aChannel],PAL_MODE_OUTPUT_PUSHPULL);
      palClearLine(gBrushCfg.P_Channels[aChannel]);
      (&PWMD1)->tim->CCER &= (~lCCEnable);
      break;
    }
    case kTimCh_Enable:
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
    case kTimCh_Disable:
    {
      palSetLineMode(gBrushCfg.N_Channels[aChannel],PAL_MODE_OUTPUT_PUSHPULL);
      palClearLine(gBrushCfg.N_Channels[aChannel]);
      (&PWMD1)->tim->CCER &= (~lCCNEnable);
      break;
    }
    case kTimCh_Enable:
    {
      palSetLineMode(gBrushCfg.N_Channels[aChannel],gBrushCfg.kDefaultIOConfig);
      (&PWMD1)->tim->CCER |= lCCNEnable;
      break;
    }
  }
}

static void tim_1_oc_start(TimChannel aChannel)
{
  tim_1_oc_cmd(aChannel,kTimCh_Enable);
}

static void tim_1_oc_stop(TimChannel aChannel)
{
  tim_1_oc_cmd(aChannel,kTimCh_Disable);
}

static void tim_1_ocn_start(TimChannel aChannel)
{
  tim_1_ocn_cmd(aChannel,kTimCh_Enable);
}

static void tim_1_ocn_stop(TimChannel aChannel)
{
  tim_1_ocn_cmd(aChannel,kTimCh_Disable);
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
    (&PWMD1)->tim->CCMR1|=  STM32_TIM_CCMR1_OC1M(0);  // OC1 Mode : Frozen
    (&PWMD1)->tim->CCMR1 &= (~STM32_TIM_CCMR1_OC1FE); // Disable the Fast Mode
    (&PWMD1)->tim->CCMR1 |= STM32_TIM_CCMR1_OC1PE;    // Enable the Preload -> CCR is loaded in the active register at each update event
    (&PWMD1)->tim->CCR[kTimChannel1] =  PERIOD_PWM_20_KHZ/2 - 1;  // Select the Half-period to overflow

    // Channel 2 Config
    (&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC2P);    // OC2 Polarity  : Active High
    (&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC2NP);   // OC2N Polarity : Active High
    (&PWMD1)->tim->CR2  &=  (~STM32_TIM_CR2_OIS2);    // OC2 Idle State (when MOE=0) : 0
    (&PWMD1)->tim->CR2  &=  (~STM32_TIM_CR2_OIS2N);   // OC2N Idle State (when MOE=0): 0
    (&PWMD1)->tim->CCMR1|=  STM32_TIM_CCMR1_OC2M(0);  // OC2 Mode : Frozen
    (&PWMD1)->tim->CCMR1 &= (~STM32_TIM_CCMR1_OC2FE); // Disable the Fast Mode
    (&PWMD1)->tim->CCMR1 |= STM32_TIM_CCMR1_OC2PE;    // Enable the Preload
    (&PWMD1)->tim->CCR[kTimChannel2] =  PERIOD_PWM_20_KHZ/4 - 1;  // Select the Quarter-period to overflow

    // Channel 3 Config
    (&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC3P);    // OC3 Polarity  : Active High
    (&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC3NP);   // OC3N Polarity : Active High
    (&PWMD1)->tim->CR2  &=  (~STM32_TIM_CR2_OIS3);    // OC3 Idle State (when MOE=0) : 0
    (&PWMD1)->tim->CR2  &=  (~STM32_TIM_CR2_OIS3N);   // OC3N Idle State (when MOE=0): 0
    (&PWMD1)->tim->CCMR2|=  STM32_TIM_CCMR2_OC3M(0);  // OC3 Mode : Frozen
    (&PWMD1)->tim->CCMR2 &= (~STM32_TIM_CCMR2_OC3FE); // Disable the Fast Mode
    (&PWMD1)->tim->CCMR2 |= STM32_TIM_CCMR2_OC3PE;    // Enable the Preload
    (&PWMD1)->tim->CCR[kTimChannel3]  =  PERIOD_PWM_20_KHZ/8 - 1;  // Select the 1/8 period to overflow

    // Channel 4 Config (for interruption each ms)
    (&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC4P);    // OC4  Polarity : Active High
    (&PWMD1)->tim->CCER &= (~STM32_TIM_CCER_CC4NP);   // OC4N Polarity : Active High
    (&PWMD1)->tim->CR2  &= (~STM32_TIM_CR2_OIS4);     // OC4 Idle State (when MOE=0): 0
    (&PWMD1)->tim->CCMR2|=  STM32_TIM_CCMR2_OC4M(3);  // OC4 Mode : Toggle
    (&PWMD1)->tim->CCMR2 &= (~STM32_TIM_CCMR2_OC4FE); // Disable the Fast Mode

    // Configure the time and the interruption enable

    // BSP - Style
    // (&PWMD1)->tim->CCR4   =  PERIOD_100_MS_INT;    // Each 100 ms an interruption
    // (&PWMD1)->tim->DIER |= STM32_TIM_DIER_CC4IE;   // Enable Capture/Compare 4 Interrupt

    // ChibiOS - Style
    pwmEnableChannel(&PWMD1, kTimChannel4 , 1);  // Interruption when CNT == CCR (same period)
    pwmEnableChannelNotification(&PWMD1, kTimChannel4);             // Enable the callback to be called for the specific channel

    // Break stage configuration

    (&PWMD1)->tim->CR1 |= STM32_TIM_CR1_CKD(0);     // Modification of the CR1 CKD in order to have a bigger period for the dead times
                                                    // OSSR and OSSI not needed
    (&PWMD1)->tim->BDTR = 0;                        // Reset BDTR to everything disabled (BK,BK2 not enabled)
    (&PWMD1)->tim->BDTR |= STM32_TIM_BDTR_DTG(0);   // Dead-time generator Setup
    (&PWMD1)->tim->BDTR |= STM32_TIM_BDTR_OSSR;     // Off-state selection for Run Mode
    (&PWMD1)->tim->BDTR |= STM32_TIM_BDTR_MOE;      // Main Output Enable

    // Commutation event configuration (Not needed at the moment.)

    // Start signal generation
    tim_1_oc_start(kTimChannel1);  // Channel 1 OC
    tim_1_ocn_start(kTimChannel1); // Channel 1 OC Complementary
    tim_1_oc_start(kTimChannel2);  // Channel 2 OC
    tim_1_ocn_start(kTimChannel2); // Channel 2 OC Complementary
    tim_1_oc_start(kTimChannel3);  // Channel 3 OC
    tim_1_ocn_start(kTimChannel3); // Channel 3 OC Complementary
    tim_1_oc_start(kTimChannel4);  // Channel 4 OC


    // Force update event (if preload enabled)
    (&PWMD1)->tim->EGR |= STM32_TIM_EGR_UG;

    // Force update event (if preload enabled) for CxE,CxNE and OCxM
    (&PWMD1)->tim->EGR |= STM32_TIM_EGR_COMG;

    // Enable the Timer
    (&PWMD1)->tim->CR1 |= STM32_TIM_CR1_CEN;     // Enable the counter in correct configuration

}

void commutation_nextstep(BrushlessConfig *pBrushCfg)
{
  pBrushCfg->InStepCount++;
  if (pBrushCfg->kMaxStepCount <= pBrushCfg->InStepCount)
  {
    if(pBrushCfg->RotationDir==kCCW)
    {
      pBrushCfg->StateIterator++;
      if(NB_STATE == pBrushCfg->StateIterator)
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

    pBrushCfg->StateCommutation = pBrushCfg->StateArray[pBrushCfg->StateIterator];
    pBrushCfg->InStepCount = 0;
  }
}



void commutation_cb(PWMDriver *pwmp)
{

  //palSetLine(DEBUG_INT_LINE);

  switch (gBrushCfg.StateCommutation)
  {
    case kStop:
    {

      //TODO : Not optimal but will do the job.
      palSetLine(LD2_LINE);
/*       palSetLine(DEBUG_INT_LINE);
      palSetLine(DEBUG_INT_LINE2);
 */
      /* Stop all the channels */
      tim_1_oc_stop(kTimChannel1);
      tim_1_oc_stop(kTimChannel2);
      tim_1_oc_stop(kTimChannel3);
      tim_1_ocn_stop(kTimChannel1);
      tim_1_ocn_stop(kTimChannel2);
      tim_1_ocn_stop(kTimChannel3);

      /* Set all the OC output to same PWM */
      (&PWMD1)->tim->CCR[kTimChannel1]  =  PERIOD_PWM_20_KHZ/2 - 1;  // Select the Half-Period to overflow
      (&PWMD1)->tim->CCR[kTimChannel2]  =  PERIOD_PWM_20_KHZ/2 - 1;  // Select the Half-Period to overflow
      (&PWMD1)->tim->CCR[kTimChannel3]  =  PERIOD_PWM_20_KHZ/2 - 1;  // Select the Half-Period to overflow

      // Force update event (if preload enabled)
      (&PWMD1)->tim->EGR |= STM32_TIM_EGR_UG;

      /* Configure the mode of each channel */
      (&PWMD1)->tim->CCMR1|=  STM32_TIM_CCMR1_OC1M(kPWMMode1);  // OC1 Mode : PWM Mode 1
      (&PWMD1)->tim->CCMR1|=  STM32_TIM_CCMR1_OC2M(kPWMMode1);  // OC2 Mode : PWM Mode 1
      (&PWMD1)->tim->CCMR2|=  STM32_TIM_CCMR2_OC3M(kPWMMode1);  // OC3 Mode : PWM Mode 1

      // Force update event (if preload enabled)
      (&PWMD1)->tim->EGR |= STM32_TIM_EGR_COMG;

      commutation_nextstep(&gBrushCfg);
      //gBrushCfg.StateCommutation = kPhaseUV;
      //gBrushCfg.InStepCount = 0;

      break;
    }

    /*
     * To see which step we are we will count from 0 up to 3 and down to 1
     */
    case kPhaseUV:
    {
      /* Debug IO in order to see in which step we are */
      palSetLine(LD2_LINE);

      /* Step 0 */
      palClearLine(DEBUG_INT_LINE);
      palClearLine(DEBUG_INT_LINE2);

      /* Channel 1 High transistor connected */
      tim_1_oc_start(kTimChannel1);
      tim_1_ocn_stop(kTimChannel1);
      /* Channel 2 Low  transistor connected */
      tim_1_oc_stop(kTimChannel2);
      tim_1_ocn_start(kTimChannel2);
      /* Channel 3 not connected */
      tim_1_oc_stop(kTimChannel3);
      tim_1_ocn_stop(kTimChannel3);

      commutation_nextstep(&gBrushCfg);

      /*gBrushCfg.InStepCount++;
      if (gBrushCfg.kMaxStepCount <= gBrushCfg.InStepCount)
      {
        gBrushCfg.StateCommutation = kPhaseUW;
        gBrushCfg.InStepCount = 0;
      }*/

      break;
    }

    case kPhaseUW:
    {
      /* Step 1 */
      palSetLine(DEBUG_INT_LINE);
      palClearLine(DEBUG_INT_LINE2);
      /* Channel 1 High transistor connected */
      tim_1_oc_start(kTimChannel1);
      tim_1_ocn_stop(kTimChannel1);
      /* Channel 2 not connected */
      tim_1_oc_stop(kTimChannel2);
      tim_1_ocn_stop(kTimChannel2);
      /* Channel 3 Low transistor connected */
      tim_1_oc_stop(kTimChannel3);
      tim_1_ocn_start(kTimChannel3);

      commutation_nextstep(&gBrushCfg);
      /*gBrushCfg.InStepCount++;
      if (gBrushCfg.kMaxStepCount <= gBrushCfg.InStepCount)
      {
        gBrushCfg.StateCommutation = kPhaseVW;
        gBrushCfg.InStepCount = 0;
      }*/

      break;
    }

    case kPhaseVW:
    {

      /* Step 2 */
      palClearLine(DEBUG_INT_LINE);
      palSetLine(DEBUG_INT_LINE2);

      /* Channel 1 not connected */
      tim_1_oc_stop(kTimChannel1);
      tim_1_ocn_stop(kTimChannel1);
      /* Channel 2 High connected*/
      tim_1_oc_start(kTimChannel2);
      tim_1_ocn_stop(kTimChannel2);
      /* Channel 3 Low transistor connected */
      tim_1_oc_stop(kTimChannel3);
      tim_1_ocn_start(kTimChannel3);

      commutation_nextstep(&gBrushCfg);
      /*gBrushCfg.InStepCount++;
      if (gBrushCfg.kMaxStepCount <= gBrushCfg.InStepCount)
      {
        gBrushCfg.StateCommutation = kPhaseVU;
        gBrushCfg.InStepCount = 0;
      }*/


      break;
    }

    case kPhaseVU:
    {

      /* Step 3 */
      palSetLine(DEBUG_INT_LINE);
      palSetLine(DEBUG_INT_LINE2);

      /* Channel 1 Low connected */
      tim_1_oc_stop(kTimChannel1);
      tim_1_ocn_start(kTimChannel1);
      /* Channel 2 High connected*/
      tim_1_oc_start(kTimChannel2);
      tim_1_ocn_stop(kTimChannel2);
      /* Channel 3 not connected */
      tim_1_oc_stop(kTimChannel3);
      tim_1_ocn_stop(kTimChannel3);

      commutation_nextstep(&gBrushCfg);
      /*gBrushCfg.InStepCount++;
      if (gBrushCfg.kMaxStepCount <= gBrushCfg.InStepCount)
      {
        gBrushCfg.StateCommutation = kPhaseWU;
        gBrushCfg.InStepCount = 0;
      }*/

      break;
    }

    case kPhaseWU:
    {
      /* Step 2 */
      palClearLine(DEBUG_INT_LINE);
      palSetLine(DEBUG_INT_LINE2);

      /* Channel 1 Low connected */
      tim_1_oc_stop(kTimChannel1);
      tim_1_ocn_start(kTimChannel1);
      /* Channel 2 not connected */
      tim_1_oc_stop(kTimChannel2);
      tim_1_ocn_stop(kTimChannel2);
      /* Channel 3 High connected */
      tim_1_oc_start(kTimChannel3);
      tim_1_ocn_stop(kTimChannel3);

      commutation_nextstep(&gBrushCfg);
     /* gBrushCfg.InStepCount++;
      if (gBrushCfg.kMaxStepCount <= gBrushCfg.InStepCount)
      {
        gBrushCfg.StateCommutation = kPhaseWV;
        gBrushCfg.InStepCount = 0;
      }*/

      break;
    }

    case kPhaseWV:
    {

      /* Step 1 */
      palSetLine(DEBUG_INT_LINE);
      palClearLine(DEBUG_INT_LINE2);

      /* Channel 1 not connected */
      tim_1_oc_stop(kTimChannel1);
      tim_1_ocn_stop(kTimChannel1);
      /* Channel 2 Low connected */
      tim_1_oc_stop(kTimChannel2);
      tim_1_ocn_start(kTimChannel2);
      /* Channel 3 High connected */
      tim_1_oc_start(kTimChannel3);
      tim_1_ocn_stop(kTimChannel3);


      commutation_nextstep(&gBrushCfg);
      /*gBrushCfg.InStepCount++;
      if (gBrushCfg.kMaxStepCount <= gBrushCfg.InStepCount)
      {
        gBrushCfg.StateCommutation = kPhaseUV;
        gBrushCfg.InStepCount = 0;
      }*/

      break;
    }

    default:
        break;
  }

  //palClearLine(DEBUG_INT_LINE);

  // Force update event (if preload enabled) for CxE,CxNE and OCxM
  (&PWMD1)->tim->EGR |= STM32_TIM_EGR_COMG;
}
