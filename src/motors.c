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

#define NB_ADC_MEASURES_VOLTAGE		4

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
static brushless_motor_t brushless_motors[NB_OF_BRUSHLESS_MOTOR] = {
	{
		.phases[PHASE1] 	= &half_bridges[BRUSHLESS_MOTOR_1_PHASE1],
		.phases[PHASE2] 	= &half_bridges[BRUSHLESS_MOTOR_1_PHASE2],
		.phases[PHASE3] 	= &half_bridges[BRUSHLESS_MOTOR_1_PHASE3],
		.commutation_scheme = BRUSHLESS_MOTOR_1_COMMUTATION,
		.direction 			= BRUSHLESS_MOTOR_1_DIRECTION
	},
#if (NB_OF_BRUSHLESS_MOTOR > 1)
	{
		.phases[PHASE1] 	= &half_bridges[BRUSHLESS_MOTOR_2_PHASE1],
		.phases[PHASE2] 	= &half_bridges[BRUSHLESS_MOTOR_2_PHASE2],
		.phases[PHASE3] 	= &half_bridges[BRUSHLESS_MOTOR_2_PHASE3],
		.commutation_scheme = BRUSHLESS_MOTOR_2_COMMUTATION,
		.direction 			= BRUSHLESS_MOTOR_2_DIRECTION
	},
#endif /* (NB_OF_BRUSHLESS_MOTOR > 1) */
#if (NB_OF_BRUSHLESS_MOTOR > 2)
	{
		.phases[PHASE1] 	= &half_bridges[BRUSHLESS_MOTOR_3_PHASE1],
		.phases[PHASE2] 	= &half_bridges[BRUSHLESS_MOTOR_3_PHASE2],
		.phases[PHASE3] 	= &half_bridges[BRUSHLESS_MOTOR_3_PHASE3],
		.commutation_scheme = BRUSHLESS_MOTOR_3_COMMUTATION,
		.direction 			= BRUSHLESS_MOTOR_3_DIRECTION
	},
#endif /* (NB_OF_BRUSHLESS_MOTOR > 2) */
#if (NB_OF_BRUSHLESS_MOTOR > 3)
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

/**
 *  ADC external triggers list for STM32F746
 *  kTimer1_CH1   0
	kTimer1_CH2   1
	kTimer1_CH3   2
	kTimer2_CH2   3
	kTimer5_TRGO  4
	kTimer4_CH4   5
	kTimer3_CH4   6
	kTimer8_TRGO  7
	kTimer8_TRGO2 8
	kTimer1_TRGO  9
	kTimer1_TRGO2 10
	kTimer2_TRGO  11
	kTimer4_TRGO  12
	kTimer6_TRGO  13
	kReserved     14
	kExti_Line11  15
 */

static ADCConversionGroup ADC3group = {
	.circular = true,
    .num_channels = NB_ADC_MEASURES_VOLTAGE,
    .end_cb = _adc_voltage_cb,
    .error_cb = NULL,
    .cr1 = 0, 	/* No OVR int, 12 bit resolution, no AWDG/JAWDG */
    .cr2 = 		/* Manual start of regular channels, no OVR detect */
           ADC_CR2_EXTEN_BOTH    |	/* We need both as OCxREF don't behave as expected. See Errata STM32F7 */
           ADC_CR2_EXTSEL_SRC(10),  /* External trigger is from Timer 1 TRGO 2*/                
    .htr = 0,
    .ltr = 0,
    .smpr1 = 0,
    .smpr2 = ADC_SMPR2_SMP_AN0(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN1(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN2(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN3(ADC_SAMPLE_3),
    .sqr1 =  ADC_SQR1_NUM_CH(NB_ADC_MEASURES_VOLTAGE),
    .sqr3 =  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0)|
             ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1)|
             ADC_SQR3_SQ3_N(ADC_CHANNEL_IN2)|
             ADC_SQR3_SQ4_N(ADC_CHANNEL_IN3),
    .sqr2 = 0,
};

/********************               PRIVATE FUNCTIONS               ********************/
void _adcInit(void){

}

void _timersInit(void){

}

/********************               PUBLIC FUNCTIONS                ********************/
void motorsInit(void){

}


