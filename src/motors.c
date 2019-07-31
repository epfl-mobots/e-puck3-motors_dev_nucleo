/**
 * @file	motors.c
 * @brief  	Library to control motors
 * 
 * @written by  	Eliot Ferragni
 * @creation date	29.07.2019
 */
 
#include "ch.h"
#include "hal.h"
#include "motors.h"

#define ADC3_BUFFER_DEPTH		2		//2 sequences of MAX_NB_OF_BRUSHLESS_MOTOR samples
#define ADC1_BUFFER_DEPTH		2		//2 sequences of MAX_NB_OF_BRUSHLESS_MOTOR samples
#define ADC1_NB_ELEMENT_SEQ		3		/* with 52KHz, we can do approx 24 measurements by PWM cycle 
											so we need to do 2 sequences of 12 elements (3 * 4 motors)*/
#define ADC3_OFF_SAMPLE_TIME	0.20	
#define ADC3_ON_SAMPLE_TIME		0.75


#define PERIOD_PWM_52_KHZ_APB2  (STM32_TIMCLK2/52000)
#define PERIOD_PWM_52_KHZ_APB1 	(STM32_TIMCLK1/52000)

/**
 * Possible phases for a brushless motor
 */
typedef enum{
    PHASE1 = 0,
    PHASE2,
    PHASE3,
    NB_BRUSHLESS_PHASES
}brushless_phases_t;

typedef enum{
	OUT_PWM = 0,
	OUT_LOW,
	OUT_HIGH,
}timer_output_state_t;

/**
 * Half Bridges list
 */
typedef enum
{
    HALF_BRIDGE_1 = 0,
    HALF_BRIDGE_2,
    HALF_BRIDGE_3,
    HALF_BRIDGE_4,
    HALF_BRIDGE_5,
    HALF_BRIDGE_6,
    HALF_BRIDGE_7,
    HALF_BRIDGE_8,
    HALF_BRIDGE_9,
    HALF_BRIDGE_10,
    HALF_BRIDGE_11,
    HALF_BRIDGE_12,
}half_bridges_names_t;

/**
 * Brushless motors list
 */
typedef enum
{
    BRUSHLESS_MOTOR_1 = 0,
    BRUSHLESS_MOTOR_2,
    BRUSHLESS_MOTOR_3,
    BRUSHLESS_MOTOR_4,
}brushless_motors_names_t;

typedef enum{
	PHASE1_P = 0,
	PHASE1_N,
	PHASE2_P,
	PHASE2_N,
	PHASE3_P,
	PHASE3_N,
	VOLT_MEASURE_CHANNEL,
	CURR_MEASURE_CHANNEL
}commutation_schemes_fields_t;

/**
 *  ADC external triggers list for STM32F746
 */
typedef enum{
	TRG_TIMER1_CH1 = 0,
	TRG_TIMER1_CH2,
	TRG_TIMER1_CH3,
	TRG_TIMER2_CH2,
	TRG_TIMER5_TRGO,
	TRG_TIMER4_CH4,
	TRG_TIMER3_CH4,
	TRG_TIMER8_TRGO,
	TRG_TIMER8_TRGO2,
	TRG_TIMER1_TRGO,
	TRG_TIMER1_TRGO2,
	TRG_TIMER2_TRGO,
	TRG_TIMER4_TRGO,
	TRG_TIMER6_TRGO,
	TRG_RESERVED,
	TRG_EXTI_LINE11,
}adc_triggers_list_t;

/**
 *  Timer triggers list for STM32F746
 */
typedef enum{
	TS_ITR0 = 0,
	TS_ITR1,
	TS_ITR2,
	TS_ITR3,
	TS_T1F_ED,
	TS_TI1FP1,
	TS_TI2FP2,
	TS_ETRF
}tim_trigger_selection_list_t;

/**
 *  Timer salve mode list for STM32F746
 */
typedef enum{
	SMS_DISABLED = 0,
	SMS_ENC_MODE_1,
	SMS_ENC_MODE_2,
	SMS_ENC_MODE_3,
	SMS_RESET_MODE,
	SMS_GATED_MODE,
	SMS_TRIGGER_MODE,
	SMS_EXTERNAL_CLOCK_MODE,
	SMS_COMBINED_RESET_TRIGGER

}tim_slave_mode_selection_list_t;

/**
 *  Timer master mode 1 list for STM32F746
 */
typedef enum{
	MMS_RESET = 0,
	MMS_ENABLE,
	MMS_UPDATE,
	MMS_COMPARE_PULSE,
	MMS_COMPARE_OC1REF,
	MMS_COMPARE_OC2REF,
	MMS_COMPARE_OC3REF,
	MMS_COMPARE_OC4REF
}tim_mms_modes_t;

/**
 *  Timer master mode 2 list for STM32F746
 */
typedef enum{
	MMS2_RESET = 0,
	MMS2_ENABLE,
	MMS2_UPDATE,
	MMS2_COMPARE_PULSE,
	MMS2_COMPARE_OC1REF,
	MMS2_COMPARE_OC2REF,
	MMS2_COMPARE_OC3REF,
	MMS2_COMPARE_OC4REF,
	MMS2_COMPARE_OC5REF,
	MMS2_COMPARE_OC6REF,
	MMS2_COMPARE_PULSE_OC4REF_RISING_FALLING,
	MMS2_COMPARE_PULSE_OC6REF_RISING_FALLING,
	MMS2_COMPARE_PULSE_OC4REF_RISING_OC6REF_RISING,
	MMS2_COMPARE_PULSE_OC4REF_RISING_OC6REF_FALLING,
	MMS2_COMPARE_PULSE_OC5REF_RISING_OC6REF_RISING,
	MMS2_COMPARE_PULSE_OC5REF_RISING_OC6REF_FALLING
}tim_mms2_modes_t;

/**
 *  Timer mode list for STM32F746
 */
typedef enum{
	OC_FROZEN = 0,
	OC_FORCED_MATCH_HIGH,
	OC_FORCED_MATCH_LOW,
	OC_TOGGLE,
	OC_FORCE_REF_HIGH,
	OC_FORCE_REF_LOW,
	OC_PWM_MODE_1,
	OC_PWM_MODE_2,
	OC_RETR_OPM_1,
	OC_RETR_OPM_2,
	OC_RESERVED_1,
	OC_RESERVED_2,
	OC_COMB_PWM_1,
	OC_COMB_PWM_2,
	OC_ASYM_PWM_1,
	OC_ASYM_PWM_2
}tim_oc_modes_t;

/**
 * NB of steps for a 6-steps commutation scheme 
 */
#define NB_STEPS_BRUSHLESS			6

/**
 * Zero Crossing variables
 */
typedef struct{
    bool 			flag;
    uint16_t 		detection_time;
    uint16_t		previous_detection_time;
    uint16_t		period;
    uint16_t		period_filtered;
    uint16_t		advance_timing;
    uint16_t		next_commutation_time;
}zero_crossing_t;

/**
 * Structure representing a physical half_bridge.
 */
typedef struct {
	ioline_t 		p_control_line;
	ioline_t 		n_control_line;
	PWMDriver*		pwmp;
	tim_channel_t	PWM_p_channel;
	tim_channel_t	PWM_n_channel;
	uint8_t			ADC3VoltageMeasureChannel;
	uint8_t			ADC1CurrentMeasureChannel;
} half_bridge_t;

/**
 * Structure representing a brushless motor
 */
typedef struct {
	const half_bridge_t*	phases[NB_BRUSHLESS_PHASES];
	commutation_schemes_t	commutation_scheme;
	uint8_t 				step_iterator;
	rotation_dir_t			direction;
	zero_crossing_t			zero_crossing;
} brushless_motor_t;

/********************         PRIVATE FUCNTION DECLARATIONS         ********************/

void _adc1_current_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n);
void _adc3_voltage_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n);

/********************               INTERNAL VARIABLES              ********************/

/**
 * Commutation table for Double PWM
 * (comes from the DRV8323 Datasheet)
 */

static const uint8_t pwm_commutation_schemes[NB_OF_COMMUTATION_SCHEME] 			// 2 Schemes
											[NB_STEPS_BRUSHLESS]				// 6 steps
											[2 * NB_BRUSHLESS_PHASES + 2] = {	// 2 * 3 Phases + 2 measures infos
	/*	Phase1 P	Phase1 N	Phase2 P	Phase2 N	Phase3 P	Phase3 N	Voltage 	Current 
																				measure 	measure
																				phase		phase		*/
	/**
	 * Double PWM
	 */
	{
		{OUT_PWM,	OUT_PWM,	OUT_LOW,	OUT_LOW,	OUT_LOW,	OUT_HIGH,	PHASE2,		PHASE3},
		{OUT_PWM, 	OUT_PWM,	OUT_LOW,	OUT_HIGH,	OUT_LOW,	OUT_LOW,	PHASE3, 	PHASE2},
		{OUT_LOW, 	OUT_LOW,	OUT_LOW,	OUT_HIGH,	OUT_PWM,	OUT_PWM, 	PHASE1, 	PHASE2},
		{OUT_LOW, 	OUT_HIGH,	OUT_LOW,	OUT_LOW,	OUT_PWM,	OUT_PWM,	PHASE2, 	PHASE1},
		{OUT_LOW, 	OUT_HIGH,	OUT_PWM,	OUT_PWM,	OUT_LOW,	OUT_LOW,	PHASE3,		PHASE1},
		{OUT_LOW, 	OUT_LOW,	OUT_PWM,	OUT_PWM,	OUT_LOW,	OUT_HIGH,	PHASE1,		PHASE3}
	},
	/**
	 * Simple PWM
	 */
	{
		{OUT_PWM,	OUT_LOW,	OUT_LOW,	OUT_LOW,	OUT_LOW,	OUT_HIGH,	PHASE2,		PHASE3},
		{OUT_PWM, 	OUT_LOW,	OUT_LOW,	OUT_HIGH,	OUT_LOW,	OUT_LOW,	PHASE3, 	PHASE2},
		{OUT_LOW, 	OUT_LOW,	OUT_LOW,	OUT_HIGH,	OUT_PWM,	OUT_LOW, 	PHASE1, 	PHASE2},
		{OUT_LOW, 	OUT_HIGH,	OUT_LOW,	OUT_LOW,	OUT_PWM,	OUT_LOW,	PHASE2, 	PHASE1},
		{OUT_LOW, 	OUT_HIGH,	OUT_PWM,	OUT_LOW,	OUT_LOW,	OUT_LOW,	PHASE3,		PHASE1},
		{OUT_LOW, 	OUT_LOW,	OUT_PWM,	OUT_LOW,	OUT_LOW,	OUT_HIGH,	PHASE1,		PHASE3}
	}
};

/**
 *	Half bridges structure
 */
#if (NB_OF_HALF_BRIDGES > 0)
const half_bridge_t half_bridges[NB_OF_HALF_BRIDGES] = {
	{
		.p_control_line				= P_CONTROL_LINE_1,
		.n_control_line				= N_CONTROL_LINE_1,
		.pwmp 						= PWM_DRIVER_1,
		.PWM_p_channel				= PWM_P_CHANNEL_1,
		.PWM_n_channel				= PWM_N_CHANNEL_1,
		.ADC3VoltageMeasureChannel	= ADC3_VOLTAGE_CHANNEL_1,
		.ADC1CurrentMeasureChannel	= ADC1_CURRENT_CHANNEL_1
	},
#if (NB_OF_HALF_BRIDGES > 1)
	{
		.p_control_line				= P_CONTROL_LINE_2,
		.n_control_line				= N_CONTROL_LINE_2,
		.pwmp 						= PWM_DRIVER_2,
		.PWM_p_channel				= PWM_P_CHANNEL_2,
		.PWM_n_channel				= PWM_N_CHANNEL_2,
		.ADC3VoltageMeasureChannel	= ADC3_VOLTAGE_CHANNEL_2,
		.ADC1CurrentMeasureChannel	= ADC1_CURRENT_CHANNEL_2
	},
#endif /* (NB_OF_HALF_BRIDGES > 1) */
#if (NB_OF_HALF_BRIDGES > 2)
	{
		.p_control_line				= P_CONTROL_LINE_3,
		.n_control_line				= N_CONTROL_LINE_3,
		.pwmp 						= PWM_DRIVER_3,
		.PWM_p_channel				= PWM_P_CHANNEL_3,
		.PWM_n_channel				= PWM_N_CHANNEL_3,
		.ADC3VoltageMeasureChannel	= ADC3_VOLTAGE_CHANNEL_3,
		.ADC1CurrentMeasureChannel	= ADC1_CURRENT_CHANNEL_3
	},
#endif /* (NB_OF_HALF_BRIDGES > 2) */
#if (NB_OF_HALF_BRIDGES > 3)
	{
		.p_control_line				= P_CONTROL_LINE_4,
		.n_control_line				= N_CONTROL_LINE_4,
		.pwmp 						= PWM_DRIVER_4,
		.PWM_p_channel				= PWM_P_CHANNEL_4,
		.PWM_n_channel				= PWM_N_CHANNEL_4,
		.ADC3VoltageMeasureChannel	= ADC3_VOLTAGE_CHANNEL_4,
		.ADC1CurrentMeasureChannel	= ADC1_CURRENT_CHANNEL_4
	},
#endif /* (NB_OF_HALF_BRIDGES > 3) */
#if (NB_OF_HALF_BRIDGES > 4)
	{
		.p_control_line				= P_CONTROL_LINE_5,
		.n_control_line				= N_CONTROL_LINE_5,
		.pwmp 						= PWM_DRIVER_5,
		.PWM_p_channel				= PWM_P_CHANNEL_5,
		.PWM_n_channel				= PWM_N_CHANNEL_5,
		.ADC3VoltageMeasureChannel	= ADC3_VOLTAGE_CHANNEL_5,
		.ADC1CurrentMeasureChannel	= ADC1_CURRENT_CHANNEL_5
	},
#endif /* (NB_OF_HALF_BRIDGES > 4) */
#if (NB_OF_HALF_BRIDGES > 5)
	{
		.p_control_line				= P_CONTROL_LINE_6,
		.n_control_line				= N_CONTROL_LINE_6,
		.pwmp 						= PWM_DRIVER_6,
		.PWM_p_channel				= PWM_P_CHANNEL_6,
		.PWM_n_channel				= PWM_N_CHANNEL_6,
		.ADC3VoltageMeasureChannel	= ADC3_VOLTAGE_CHANNEL_6,
		.ADC1CurrentMeasureChannel	= ADC1_CURRENT_CHANNEL_6
	},
#endif /* (NB_OF_HALF_BRIDGES > 5) */
#if (NB_OF_HALF_BRIDGES > 6)
	{
		.p_control_line				= P_CONTROL_LINE_7,
		.n_control_line				= N_CONTROL_LINE_7,
		.pwmp 						= PWM_DRIVER_7,
		.PWM_p_channel				= PWM_P_CHANNEL_7,
		.PWM_n_channel				= PWM_N_CHANNEL_7,
		.ADC3VoltageMeasureChannel	= ADC3_VOLTAGE_CHANNEL_7,
		.ADC1CurrentMeasureChannel	= ADC1_CURRENT_CHANNEL_7
	},
#endif /* (NB_OF_HALF_BRIDGES > 6) */
#if (NB_OF_HALF_BRIDGES > 7)
	{
		.p_control_line				= P_CONTROL_LINE_8,
		.n_control_line				= N_CONTROL_LINE_8,
		.pwmp 						= PWM_DRIVER_8,
		.PWM_p_channel				= PWM_P_CHANNEL_8,
		.PWM_n_channel				= PWM_N_CHANNEL_8,
		.ADC3VoltageMeasureChannel	= ADC3_VOLTAGE_CHANNEL_8,
		.ADC1CurrentMeasureChannel	= ADC1_CURRENT_CHANNEL_8
	},
#endif /* (NB_OF_HALF_BRIDGES > 7) */
#if (NB_OF_HALF_BRIDGES > 8)
	{
		.p_control_line				= P_CONTROL_LINE_9,
		.n_control_line				= N_CONTROL_LINE_9,
		.pwmp 						= PWM_DRIVER_9,
		.PWM_p_channel				= PWM_P_CHANNEL_9,
		.PWM_n_channel				= PWM_N_CHANNEL_9,
		.ADC3VoltageMeasureChannel	= ADC3_VOLTAGE_CHANNEL_9,
		.ADC1CurrentMeasureChannel	= ADC1_CURRENT_CHANNEL_9
	},
#endif /* (NB_OF_HALF_BRIDGES > 8) */
#if (NB_OF_HALF_BRIDGES > 9)
	{
		.p_control_line				= P_CONTROL_LINE_10,
		.n_control_line				= N_CONTROL_LINE_10,
		.pwmp 						= PWM_DRIVER_10,
		.PWM_p_channel				= PWM_P_CHANNEL_10,
		.PWM_n_channel				= PWM_N_CHANNEL_10,
		.ADC3VoltageMeasureChannel	= ADC3_VOLTAGE_CHANNEL_10,
		.ADC1CurrentMeasureChannel	= ADC1_CURRENT_CHANNEL_10
	},
#endif /* (NB_OF_HALF_BRIDGES > 9) */
#if (NB_OF_HALF_BRIDGES > 10)
	{
		.p_control_line				= P_CONTROL_LINE_11,
		.n_control_line				= N_CONTROL_LINE_11,
		.pwmp 						= PWM_DRIVER_11,
		.PWM_p_channel				= PWM_P_CHANNEL_11,
		.PWM_n_channel				= PWM_N_CHANNEL_11,
		.ADC3VoltageMeasureChannel	= ADC3_VOLTAGE_CHANNEL_11,
		.ADC1CurrentMeasureChannel	= ADC1_CURRENT_CHANNEL_11
	},
#endif /* (NB_OF_HALF_BRIDGES > 10) */
#if (NB_OF_HALF_BRIDGES > 11)
	{
		.p_control_line				= P_CONTROL_LINE_12,
		.n_control_line				= N_CONTROL_LINE_12,
		.pwmp 						= PWM_DRIVER_12,
		.PWM_p_channel				= PWM_P_CHANNEL_12,
		.PWM_n_channel				= PWM_N_CHANNEL_12,
		.ADC3VoltageMeasureChannel	= ADC3_VOLTAGE_CHANNEL_12,
		.ADC1CurrentMeasureChannel	= ADC1_CURRENT_CHANNEL_12
	}
#endif /* (NB_OF_HALF_BRIDGES > 11) */
};
#endif /* (NB_OF_HALF_BRIDGES > 0) */


/**
 *	Brushless motors structure
 */
#if (NB_OF_BRUSHLESS_MOTOR > 0)
#if (NB_OF_HALF_BRIDGES < 3)
#error "we need at least 3 half bridges to drive one brushless motors !"
#endif /* (NB_OF_HALF_BRIDGES < 3) */
static brushless_motor_t brushless_motors[NB_OF_BRUSHLESS_MOTOR] = {
	{
		.phases[PHASE1] 	= &half_bridges[BRUSHLESS_MOTOR_1_PHASE1],
		.phases[PHASE2] 	= &half_bridges[BRUSHLESS_MOTOR_1_PHASE2],
		.phases[PHASE3] 	= &half_bridges[BRUSHLESS_MOTOR_1_PHASE3],
		.commutation_scheme = BRUSHLESS_MOTOR_1_COMMUTATION,
		.direction 			= BRUSHLESS_MOTOR_1_DIRECTION
	},
#if (NB_OF_BRUSHLESS_MOTOR > 1)
#if (NB_OF_HALF_BRIDGES < 6)
#error "we need at least 6 half bridges to drive two brushless motors !"
#endif /* (NB_OF_HALF_BRIDGES < 6) */
	{
		.phases[PHASE1] 	= &half_bridges[BRUSHLESS_MOTOR_2_PHASE1],
		.phases[PHASE2] 	= &half_bridges[BRUSHLESS_MOTOR_2_PHASE2],
		.phases[PHASE3] 	= &half_bridges[BRUSHLESS_MOTOR_2_PHASE3],
		.commutation_scheme = BRUSHLESS_MOTOR_2_COMMUTATION,
		.direction 			= BRUSHLESS_MOTOR_2_DIRECTION
	},
#endif /* (NB_OF_BRUSHLESS_MOTOR > 1) */
#if (NB_OF_BRUSHLESS_MOTOR > 2)
#if (NB_OF_HALF_BRIDGES < 9)
#error "we need at least 9 half bridges to drive three brushless motors !"
#endif /* (NB_OF_HALF_BRIDGES < 9) */
	{
		.phases[PHASE1] 	= &half_bridges[BRUSHLESS_MOTOR_3_PHASE1],
		.phases[PHASE2] 	= &half_bridges[BRUSHLESS_MOTOR_3_PHASE2],
		.phases[PHASE3] 	= &half_bridges[BRUSHLESS_MOTOR_3_PHASE3],
		.commutation_scheme = BRUSHLESS_MOTOR_3_COMMUTATION,
		.direction 			= BRUSHLESS_MOTOR_3_DIRECTION
	},
#endif /* (NB_OF_BRUSHLESS_MOTOR > 2) */
#if (NB_OF_BRUSHLESS_MOTOR > 3)
#if (NB_OF_HALF_BRIDGES < 12)
#error "we need at least 12 half bridges to drive four brushless motors !"
#endif /* (NB_OF_HALF_BRIDGES < 12) */
	{
		.phases[PHASE1] 	= &half_bridges[BRUSHLESS_MOTOR_4_PHASE1],
		.phases[PHASE2] 	= &half_bridges[BRUSHLESS_MOTOR_4_PHASE2],
		.phases[PHASE3] 	= &half_bridges[BRUSHLESS_MOTOR_4_PHASE3],
		.commutation_scheme = BRUSHLESS_MOTOR_4_COMMUTATION,
		.direction 			= BRUSHLESS_MOTOR_4_DIRECTION
	},
#endif /* (NB_OF_BRUSHLESS_MOTOR > 3) */
};
#endif /* (NB_OF_BRUSHLESS_MOTOR > 0) */


/**
 *	ADCs config
 */

/* circular buffer */
static adcsample_t adc1_buffer[MAX_NB_OF_BRUSHLESS_MOTOR * ADC3_BUFFER_DEPTH] = {0};

/* ADC 1 Configuration */
static const ADCConversionGroup ADC1Config = {
    .circular = true,
    .num_channels = ADC1_NB_ELEMENT_SEQ * MAX_NB_OF_BRUSHLESS_MOTOR,
    .end_cb = _adc1_current_cb,
    .error_cb = NULL,
    .cr1 = 0,   /* No OVR int, 12 bit resolution, no AWDG/JAWDG*/
    .cr2 = 0,  	/* Manual start of regular channels, no OVR detect */                     
    .htr = 0,
    .ltr = 0,
    .smpr2 = ADC_SMPR2_SMP_AN0(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN1(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN2(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN3(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN4(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN5(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN6(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN7(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN8(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN9(ADC_SAMPLE_3),
    .smpr1 = ADC_SMPR1_SMP_AN10(ADC_SAMPLE_3)|
             ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3),
    .sqr3 =  0,
    .sqr2 =  0,
    .sqr1 =  ADC_SQR1_NUM_CH(ADC1_NB_ELEMENT_SEQ * MAX_NB_OF_BRUSHLESS_MOTOR),
};

/* circular buffer */
static adcsample_t adc3_buffer[MAX_NB_OF_BRUSHLESS_MOTOR * ADC3_BUFFER_DEPTH] = {0};
/* ADC 3 Configuration */
static ADCConversionGroup ADC3Config = {
	.circular = true,
    .num_channels = MAX_NB_OF_BRUSHLESS_MOTOR,
    .end_cb = _adc3_voltage_cb,
    .error_cb = NULL,
    .cr1 = 0, 	/* No OVR int, 12 bit resolution, no AWDG/JAWDG */
    .cr2 = 		/* Manual start of regular channels, no OVR detect */
           ADC_CR2_EXTEN_BOTH    |	/* We need both as OCxREF don't behave as expected. See Errata STM32F7 */
           ADC_CR2_EXTSEL_SRC(TRG_TIMER1_TRGO2),  /* External trigger is from Timer 1 TRGO 2*/                
    .htr = 0,
    .ltr = 0,
    .smpr2 = ADC_SMPR2_SMP_AN0(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN1(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN2(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN3(ADC_SAMPLE_3),
    .smpr1 = 0,
    .sqr3 =  0,
    .sqr2 = 0,
    .sqr1 =  ADC_SQR1_NUM_CH(MAX_NB_OF_BRUSHLESS_MOTOR),
};

/**
 *	Timers config
 */

static PWMConfig tim_1_cfg = {
  .frequency = STM32_TIMCLK2,                	/* PWM clock frequency.   */
  .period    = PERIOD_PWM_52_KHZ_APB2,			/* PWM period in ticks    */
  .callback  = NULL,                            /* Callback called when UIF is set*/
  /* PWM Channels configuration */
  {
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  /* Master Mode Selection 1 : Enable, Master Mode Selection 2 : OC4REF */
  .cr2  = STM32_TIM_CR2_MMS(MMS_ENABLE) | STM32_TIM_CR2_MMS2(MMS2_COMPARE_OC4REF), 
  .bdtr = STM32_TIM_BDTR_OSSR, /* OSSR = 1 */
  .dier = 0
};

static PWMConfig tim_8_cfg = {
  .frequency = STM32_TIMCLK2,                	/* PWM clock frequency.   */
  .period    = PERIOD_PWM_52_KHZ_APB2,			/* PWM period in ticks    */
  .callback  = NULL,                            /* Callback called when UIF is set*/
  /* PWM Channels configuration */
  {
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  .cr2  = 0,
  .bdtr = STM32_TIM_BDTR_OSSR, /* OSSR = 1 */
  .dier = 0
};

static PWMConfig tim_234_cfg = {
  .frequency = STM32_TIMCLK1,                	/* PWM clock frequency.   */
  .period    = PERIOD_PWM_52_KHZ_APB1,			/* PWM period in ticks    */
  .callback  = NULL,                            /* Callback called when UIF is set*/
  /* PWM Channels configuration */
  {
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  .cr2  = 0,
  .bdtr = 0, /* OSSR = 1 */
  .dier = 0
};

/********************               PRIVATE FUNCTIONS               ********************/

/**
 * Changes the value of the timer channel linked to the ADC3 trigger
 */
#define UPDATE_ADC3_TRIGGER(x) (PWMD1.tim->CCR[TIM_CHANNEL_4] = x * PWMD1.tim->ARR)
/**
 * Changes the sequence of the ADC3
 */
#define UPDATE_ADC3_SEQUENCE(x) (ADCD3.adc->SQR3 = x)
/**
 * Changes the sequence of the ADC1
 */
#define UPDATE_ADC1_SEQUENCE(x, y) {\
									ADCD1.adc->SQR3 = x; \
									ADCD1.adc->SQR2 = y; \
									}
/**
 * Starts the ADC1 for one sequence
 */
#define DO_ONE_SEQUENCE_ADC1()	(ADCD1.adc->CR2 |= ADC_CR2_SWSTART)

#if (NB_OF_BRUSHLESS_MOTOR > 0)
/**
 * Gives the ADC3 channel to measure given the motor number
 */
#define GET_VOLTAGE_MEASUREMENT_CHANNEL(x) (brushless_motors[x].phases[pwm_commutation_schemes[brushless_motors[x].commutation_scheme][brushless_motors[x].step_iterator][VOLT_MEASURE_CHANNEL]]->ADC3VoltageMeasureChannel)
/**
 * Gives the ADC1 channel to measure given the motor number
 */
#define GET_CURRENT_MEASUREMENT_CHANNEL(x) (brushless_motors[x].phases[pwm_commutation_schemes[brushless_motors[x].commutation_scheme][brushless_motors[x].step_iterator][CURR_MEASURE_CHANNEL]]->ADC1CurrentMeasureChannel)
#endif /* (NB_OF_BRUSHLESS_MOTOR > 0) */

void _adc1_current_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n){
	(void) adcp;
	(void) n;

	UPDATE_ADC1_SEQUENCE(
		ADC_SQR3_SQ1_N (GET_CURRENT_MEASUREMENT_CHANNEL(BRUSHLESS_MOTOR_1))|
		ADC_SQR3_SQ2_N (GET_CURRENT_MEASUREMENT_CHANNEL(BRUSHLESS_MOTOR_2))|
		ADC_SQR3_SQ3_N (GET_CURRENT_MEASUREMENT_CHANNEL(BRUSHLESS_MOTOR_3))|
		ADC_SQR3_SQ4_N (GET_CURRENT_MEASUREMENT_CHANNEL(BRUSHLESS_MOTOR_4))|
		ADC_SQR3_SQ5_N (GET_CURRENT_MEASUREMENT_CHANNEL(BRUSHLESS_MOTOR_1))|
		ADC_SQR3_SQ6_N (GET_CURRENT_MEASUREMENT_CHANNEL(BRUSHLESS_MOTOR_2)),
		ADC_SQR2_SQ7_N (GET_CURRENT_MEASUREMENT_CHANNEL(BRUSHLESS_MOTOR_3))|
		ADC_SQR2_SQ8_N (GET_CURRENT_MEASUREMENT_CHANNEL(BRUSHLESS_MOTOR_4))|
		ADC_SQR2_SQ9_N (GET_CURRENT_MEASUREMENT_CHANNEL(BRUSHLESS_MOTOR_1))|
		ADC_SQR2_SQ10_N(GET_CURRENT_MEASUREMENT_CHANNEL(BRUSHLESS_MOTOR_2))|
		ADC_SQR2_SQ11_N(GET_CURRENT_MEASUREMENT_CHANNEL(BRUSHLESS_MOTOR_3))|
		ADC_SQR2_SQ12_N(GET_CURRENT_MEASUREMENT_CHANNEL(BRUSHLESS_MOTOR_4))
	);
	
	DO_ONE_SEQUENCE_ADC1();
}

void _adc3_voltage_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n){
	(void) adcp;
	(void) n;

	static bool state = true;
	if(state){
		//we sampled OFF PWM
		UPDATE_ADC3_TRIGGER(ADC3_ON_SAMPLE_TIME);
	}else{
		//we sampled ON PWM
		UPDATE_ADC3_TRIGGER(ADC3_OFF_SAMPLE_TIME);
#if (NB_OF_BRUSHLESS_MOTOR > 0)
		UPDATE_ADC3_SEQUENCE(
		ADC_SQR3_SQ1_N(GET_VOLTAGE_MEASUREMENT_CHANNEL(BRUSHLESS_MOTOR_1))|
#if (NB_OF_BRUSHLESS_MOTOR > 1)
    	ADC_SQR3_SQ2_N(GET_VOLTAGE_MEASUREMENT_CHANNEL(BRUSHLESS_MOTOR_2))|
#endif /* (NB_OF_BRUSHLESS_MOTOR > 1) */
#if (NB_OF_BRUSHLESS_MOTOR > 2)
    	ADC_SQR3_SQ3_N(GET_VOLTAGE_MEASUREMENT_CHANNEL(BRUSHLESS_MOTOR_3))|
#endif /* (NB_OF_BRUSHLESS_MOTOR > 2) */
#if (NB_OF_BRUSHLESS_MOTOR > 3)
    	ADC_SQR3_SQ4_N(GET_VOLTAGE_MEASUREMENT_CHANNEL(BRUSHLESS_MOTOR_4))|
#endif /* (NB_OF_BRUSHLESS_MOTOR > 3) */
    	0);
#endif /* (NB_OF_BRUSHLESS_MOTOR > 0) */
	}
	//switches the state
	state = !state;

}

void _adcStart(void){

	adcStart(&ADCD1, NULL);
	adcStart(&ADCD3, NULL);

	/* We have one interrupt per sequence and we use a double buffer */
	adcStartConversion(&ADCD1, &ADC1Config, adc1_buffer, ADC1_BUFFER_DEPTH);
	adcStartConversion(&ADCD3, &ADC3Config, adc3_buffer, ADC3_BUFFER_DEPTH);
	DO_ONE_SEQUENCE_ADC1();

}

void _timersStart(void){

	pwmStart(&PWMD1, &tim_1_cfg);
	/* additionnal config */
	PWMD1.tim->CR1 		&= ~STM32_TIM_CR1_CEN;     	// Disables the counter until correct configuration
	PWMD1.tim->CCMR1 	|= STM32_TIM_CCMR1_OC1M(OC_PWM_MODE_2) | STM32_TIM_CCMR1_OC2M(OC_PWM_MODE_2);	// Sets channels 1 and 2 to PWM mode 2
	PWMD1.tim->CCMR2 	|= STM32_TIM_CCMR2_OC3M(OC_PWM_MODE_2) | STM32_TIM_CCMR2_OC4M(OC_PWM_MODE_2);  // Sets channels 3 and 4 to PWM mode 2
	PWMD1.tim->CCMR2 	&= ~STM32_TIM_CCMR2_OC4PE;	// Disables the preload for channel 4
	PWMD1.tim->CNT 		 = 0;							// Resets the counter to zero

	pwmStart(&PWMD8, &tim_8_cfg);
	/* additionnal config */
	PWMD8.tim->CR1 		&= ~STM32_TIM_CR1_CEN;     	// Disables the counter until correct configuration
	PWMD8.tim->SMCR   	 = STM32_TIM_SMCR_SMS(SMS_TRIGGER_MODE) | STM32_TIM_SMCR_TS(TS_ITR0); //external trigger mode and TIM1 master
	PWMD8.tim->CCMR1 	|= STM32_TIM_CCMR1_OC1M(OC_PWM_MODE_2) | STM32_TIM_CCMR1_OC2M(OC_PWM_MODE_2);	// Sets channels 1 and 2 to PWM mode 2
	PWMD8.tim->CCMR2 	|= STM32_TIM_CCMR2_OC3M(OC_PWM_MODE_2);  // Sets channels 3 to PWM mode 2
	PWMD8.tim->CNT 		 = 0;							// Resets the counter to zero

	pwmStart(&PWMD2, &tim_234_cfg);
	PWMD2.tim->CR1 		&= ~STM32_TIM_CR1_CEN;     	// Disables the counter until correct configuration
	PWMD2.tim->SMCR   	 = STM32_TIM_SMCR_SMS(SMS_TRIGGER_MODE) | STM32_TIM_SMCR_TS(TS_ITR0); //external trigger mode and TIM1 master
	PWMD2.tim->CCER 	|= STM32_TIM_CCER_CC2P | STM32_TIM_CCER_CC4P;	// Active Low Polarity for channels 2 and 4 (to act as complementary outputs)
	PWMD2.tim->CCMR1 	|= STM32_TIM_CCMR1_OC1M(OC_PWM_MODE_2) | STM32_TIM_CCMR1_OC2M(OC_PWM_MODE_2);	// Sets channels 1 and 2 to PWM mode 2
	PWMD2.tim->CCMR2 	|= STM32_TIM_CCMR2_OC3M(OC_PWM_MODE_2) | STM32_TIM_CCMR2_OC4M(OC_PWM_MODE_2);  // Sets channels 3 and 4 to PWM mode 2
	PWMD2.tim->CNT 		 = 0;

	pwmStart(&PWMD3, &tim_234_cfg);
	PWMD3.tim->CR1 		&= ~STM32_TIM_CR1_CEN;     	// Disables the counter until correct configuration
	PWMD3.tim->SMCR   	 = STM32_TIM_SMCR_SMS(SMS_TRIGGER_MODE) | STM32_TIM_SMCR_TS(TS_ITR0); //external trigger mode and TIM1 master
	PWMD3.tim->CCER 	|= STM32_TIM_CCER_CC2P | STM32_TIM_CCER_CC4P;	// Active Low Polarity for channels 2 and 4 (to act as complementary outputs)
	PWMD3.tim->CCMR1 	|= STM32_TIM_CCMR1_OC1M(OC_PWM_MODE_2) | STM32_TIM_CCMR1_OC2M(OC_PWM_MODE_2);	// Sets channels 1 and 2 to PWM mode 2
	PWMD3.tim->CCMR2 	|= STM32_TIM_CCMR2_OC3M(OC_PWM_MODE_2) | STM32_TIM_CCMR2_OC4M(OC_PWM_MODE_2);  // Sets channels 3 and 4 to PWM mode 2
	PWMD3.tim->CNT 		 = 0;

	pwmStart(&PWMD4, &tim_234_cfg);
	PWMD4.tim->CR1 		&= ~STM32_TIM_CR1_CEN;     	// Disables the counter until correct configuration
	PWMD4.tim->SMCR   	 = STM32_TIM_SMCR_SMS(SMS_TRIGGER_MODE) | STM32_TIM_SMCR_TS(TS_ITR0); //external trigger mode and TIM1 master
	PWMD4.tim->CCER 	|= STM32_TIM_CCER_CC2P | STM32_TIM_CCER_CC4P;	// Active Low Polarity for channels 2 and 4 (to act as complementary outputs)
	PWMD4.tim->CCMR1 	|= STM32_TIM_CCMR1_OC1M(OC_PWM_MODE_2) | STM32_TIM_CCMR1_OC2M(OC_PWM_MODE_2);	// Sets channels 1 and 2 to PWM mode 2
	PWMD4.tim->CCMR2 	|= STM32_TIM_CCMR2_OC3M(OC_PWM_MODE_2) | STM32_TIM_CCMR2_OC4M(OC_PWM_MODE_2);  // Sets channels 3 and 4 to PWM mode 2
	PWMD4.tim->CNT 		 = 0;

	/* Enables the timers */ 
	PWMD1.tim->CR1 |= STM32_TIM_CR1_CEN;
}

/********************               PUBLIC FUNCTIONS                ********************/
void motorsStart(void){
	_adcStart();
	_timersStart();
}


